#include "lib/perception_and_ld_map_fusion/data_fusion/multi_source_data_mixer.h"

#include <bits/stdc++.h>

#include <array>
#include <cmath>

#include "lib/perception_and_ld_map_fusion/data_fusion/pose_offset_solver_residual.h"

namespace cem {
namespace fusion {
MultiSourceDataMixer::MultiSourceDataMixer() {}
MultiSourceDataMixer::~MultiSourceDataMixer() {}

#include <ceres/ceres.h>

#include <iostream>
#include <vector>

void MultiSourceDataMixer::Init(BevMapProcessor* bev_map_processor,
                                LdMapProcessor* ld_map_processor,
                                BevFilletingMachine* slicer) {
  LoadBevMap(bev_map_processor);
  LoadLdMap(ld_map_processor);
  LoadBevSlicer(slicer);
  pose_filter_ = std::make_unique<PoseFilter>(7, 5, 1);
  process_noise_ = Eigen::MatrixXd::Identity(7, 7);
  process_noise_(0, 0) = 0.274;  // heading_std = 30°
  process_noise_(1, 1) = 2 * 2;  // x_std = 1m
  process_noise_(2, 2) = 2 * 2;  // y_std = 1m
  process_noise_(3, 3) = 3;      // vx_std = 3m/s
  process_noise_(4, 4) = 3;      // vy_std = 3m/s
  measurement_noise_by_matching_info_ = Eigen::MatrixXd::Identity(5, 5);
  measurement_noise_by_matching_info_(0, 0) = 0.1218;  // heading_std = 20°
  measurement_noise_by_matching_info_(1, 1) = 1 * 1;   // x_std = 0.2m
  measurement_noise_by_matching_info_(2, 2) = 1 * 1;   // y_std = 0.2m
  measurement_noise_by_matching_info_(3, 3) = 1;       // vx_std = 1m/s
  measurement_noise_by_matching_info_(4, 4) = 1;       // vy_std = 1m/s
  measurement_noise_by_last_state_ = Eigen::MatrixXd::Identity(5, 5);
  measurement_noise_by_last_state_(0, 0) = 0.03;   // heading_std = 10°
  measurement_noise_by_last_state_(1, 1) = 2 * 2;  // x_std = 0.4m
  measurement_noise_by_last_state_(2, 2) = 2 * 2;  // y_std = 0.4m
  measurement_noise_by_last_state_(3, 3) = 3;      // vx_std = 1m/s
  measurement_noise_by_last_state_(4, 4) = 3;      // vy_std = 1m/s
  noise_factor_of_matching_info_ = 15;

  predicted_pose_offset_.resize(3, 0);
}

void MultiSourceDataMixer::LoadBevMap(BevMapProcessor* bev_map_processor) {
  assert(bev_map_preprocessor_ != nullptr);
  bev_map_preprocessor_ = bev_map_processor;
}

void MultiSourceDataMixer::LoadLdMap(LdMapProcessor* ld_map_processor) {
  assert(ld_map_preprocessor_ != nullptr);
  ld_map_preprocessor_ = ld_map_processor;
}

void MultiSourceDataMixer::LoadBevSlicer(BevFilletingMachine* bev_slicer) {
  assert(ld_map_preprocessor_ != nullptr);
  bev_slicer_ = bev_slicer;
}

void MultiSourceDataMixer::Process(
    const std::unordered_map<
        uint64_t,
        std::vector<std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>>>&
        match_details,
    const cem::fusion::LocalizationConstPtr pose) {
  ExtractHdMapInfoIntoPerceptionMap(match_details);
  // AlignHdMapToPerceptionMap(match_details, pose);
}

void MultiSourceDataMixer::ExtractHdMapInfoIntoPerceptionMap(
    const std::unordered_map<
        uint64_t,
        std::vector<std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>>>&
        match_details) {
  for (auto& [bev_id, match_detail_seq] : match_details) {
    if (bev_map_preprocessor_->GetLane(bev_id) == nullptr) {
      continue;
    }
    for (auto& match_info : match_detail_seq) {
      auto ld_lane = ld_map_preprocessor_->GetLane(match_info->ldmap_id);
      if (ld_lane == nullptr) {
        continue;
      }
      if (ld_lane->turn_type != cem::message::env_model::TurnType::RIGHT_TURN) {
        continue;
      }
      bev_map_preprocessor_->SetRightTurnToLane(bev_id);
      break;
    }
  }
}

void MultiSourceDataMixer::AlignHdMapToPerceptionMap(
    const std::unordered_map<
        uint64_t,
        std::vector<std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>>>&
        match_details,
    const cem::fusion::LocalizationConstPtr pose) {
  AINFO << "pose timstamp:" << std::setprecision(16) << pose->header.timestamp
        << ", measure timestamp: " << bev_map_preprocessor_->GetTimestamp();
  double heading = pose->attitude_dr * M_PI / 180;
  double x = pose->posne_dr.at(0);
  double y = pose->posne_dr.at(1);
  double vx = 0.0, vy = 0.0, ax = 0.0, ay = 0.0;
  std::vector<double> pose_offset;  // heading, tx, ty

  if (pose->velocity.size() == 3) {
    vx = pose->velocity.at(0);
    vy = pose->velocity.at(1);
  }
  if (!pose_filter_->IsInit()) {
    Eigen::VectorXd X(7);
    X << heading, x, y, vx, vy, ax, ay;
    pose_filter_->Init(X);
  } else {
    double delta_t = bev_map_preprocessor_->GetTimestamp() - timestamp_;
    auto hist_state = pose_filter_->GetState();
    pose_filter_->Predict(delta_t, process_noise_);
    auto cur_state = pose_filter_->GetState();
    predicted_pose_offset_.at(0) = cur_state(0) - heading;
    predicted_pose_offset_.at(1) = cur_state(1) - x;
    predicted_pose_offset_.at(2) = cur_state(2) - y;
    auto T =
        CalcRotateTranslateMatrix(hist_state(0), hist_state(1), hist_state(2),
                                  cur_state(0), cur_state(1), cur_state(2));
    AINFO << T.matrix();
    std::vector<double> pose_offset_by_last_state =
        EstimatePoseOffsetByLastState(T);
    // todo: 是否要通过predict量对历史帧纠偏的当前位姿做检验？
    // todo: set R
    auto R = measurement_noise_by_last_state_;
    if (!match_details.empty()) {
      auto pose_offset_by_vision =
          EstimatePoseOffsetByMatchInfoRANSAC(match_details);
      if (IsPoseEstimateBelievable(pose_offset_by_last_state,
                                   pose_offset_by_vision)) {
        pose_offset = std::move(pose_offset_by_vision);
        // todo: set R
        R = noise_factor_of_matching_info_ *
            measurement_noise_by_matching_info_;
      } else {
        pose_offset = std::move(pose_offset_by_last_state);
      }
    } else {
      noise_factor_of_matching_info_ = 15;
      pose_offset = std::move(pose_offset_by_last_state);
    }
    // AINFO << pose_offset.at(0) << " " << pose_offset.at(1) << " "
    //       << pose_offset.at(2);
    auto tmp_x = std::cos(pose_offset.at(0)) * pose_offset.at(1) -
                 std::sin(pose_offset.at(0)) * pose_offset.at(2);
    auto tmp_y = std::sin(pose_offset.at(0)) * pose_offset.at(1) +
                 std::cos(pose_offset.at(0)) * pose_offset.at(2);
    pose_offset.at(1) = tmp_x;
    pose_offset.at(2) = tmp_y;
    Eigen::VectorXd Z(5);
    Z(0) = heading + pose_offset.at(0);
    Z(1) = x + pose_offset.at(1);
    Z(2) = y + pose_offset.at(2);
    Z(3) = vx;
    Z(4) = vy;
    // {
    //   const auto& state = pose_filter_->GetState();
    //   AINFO << "predict: " << state(0) << " " << state(1) << " " << state(2)
    //         << " " << state(3) << " " << state(4) << " " << state(5) << " "
    //         << state(6);
    // }
    // AINFO << "measurement: " << Z(0) << " " << Z(1) << " " << Z(2) << " "
    //       << Z(3) << " " << Z(4);
    pose_filter_->Update(Z, R);
    // const auto& state = pose_filter_->GetState();
    // AINFO << "update:" << state(0) << " " << state(1) << " " << state(2) << "
    // "
    //       << state(3) << " " << state(4) << " " << state(5) << " " <<
    //       state(6);
    // DataFusion();
    noise_factor_of_matching_info_ = noise_factor_of_matching_info_ > 1
                                         ? noise_factor_of_matching_info_ - 1
                                         : 1;
  }

  const auto& state = pose_filter_->GetState();
  // AINFO << "state:" << state(0) << " " << state(1) << " " << state(2);

  pose_offset.resize(3);
  pose_offset.at(0) = state(0) - heading;
  pose_offset.at(1) = state(1) - x;
  pose_offset.at(2) = state(2) - y;
  // AINFO << pose_offset.at(0) << " " << pose_offset.at(1) << " "
  //       << pose_offset.at(2);

  auto T_map_correct = ConstructLdMapCorrectT(pose_offset);
  Eigen::Isometry3d T_map_correct_iso(T_map_correct);
  ld_map_preprocessor_->RotateAndTranslate(T_map_correct_iso);
  corrected_map_ = ld_map_preprocessor_->GetData();
  timestamp_ = bev_map_preprocessor_->GetTimestamp();
}

// 拟合三次曲线 y = a*x^3 + b*x^2 + c*x + d
void MultiSourceDataMixer::FitCubicCurve(
    const std::vector<const std::vector<cem::message::env_model::Point>*>&
        points,
    const int point_size, Eigen::Vector4d& coeffs, double& start_x,
    double& end_x) {
  Eigen::MatrixXd A(point_size, 4);
  Eigen::VectorXd Y(point_size);
  start_x = std::numeric_limits<double>::max();
  end_x = std::numeric_limits<double>::lowest();

  int i = 0;
  for (auto& pts : points) {
    for (auto& pt : *pts) {
      if (pt.x < start_x) {
        start_x = pt.x;
      }
      if (pt.x > end_x) {
        end_x = pt.x;
      }
      A(i, 0) = 1.0;                 // 常数项
      A(i, 1) = pt.x;                // x
      A(i, 2) = pt.x * pt.x;         // x²
      A(i, 3) = pt.x * pt.x * pt.x;  // x³
      Y(i) = pt.y;
      i++;
    }
  }

  // 最小二乘拟合:  A * coeffs = Y
  coeffs = A.colPivHouseholderQr().solve(Y);
}

std::vector<double> MultiSourceDataMixer::EstimatePoseOffsetByMatchInfo(
    const std::unordered_map<
        uint64_t,
        std::vector<std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>>>&
        match_details) {
  auto start_ts = GetMwTimeNowSec();
  std::vector<double> params(3, 0);  // theta, tx, ty，初始值
  params.at(0) = predicted_pose_offset_.at(0);
  params.at(1) = predicted_pose_offset_.at(1);
  params.at(2) = predicted_pose_offset_.at(2);
  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

  int min_x = std::numeric_limits<double>::max();
  int max_x = std::numeric_limits<double>::lowest();
  for (auto& matcher : match_details) {
    std::vector<const std::vector<cem::message::env_model::Point>*> points_all;
    // std::vector<int> bev_slice_id;
    std::vector<Eigen::Vector2d> bev_sampling_points;
    std::vector<double> point_match_prob;
    int point_size = 0;
    for (auto& seg : matcher.second) {
      auto ld_lane = ld_map_preprocessor_->GetLane(seg->ldmap_id);
      points_all.push_back(&(ld_lane->points));
      point_size += ld_lane->points.size();
      for (auto i = seg->bev_start_point_idx; i <= seg->bev_end_point_idx;
           ++i) {
        Eigen::Vector2d pt;
        if (bev_slicer_->GetSamplingPoint(matcher.first, i, pt)) {
          bev_sampling_points.push_back(std::move(pt));
          if (seg->point_probability.count(i) > 0) {
            point_match_prob.push_back(seg->point_probability.at(i));
          } else {
            point_match_prob.push_back(0.0);
          }
        }
      }
    }
    if (point_size >= 4 && bev_sampling_points.size() >= 4) {
      Eigen::Vector4d coeffs;
      double start_x, end_x;
      coeffs << 0, 0, 0, 0;
      FitCubicCurve(points_all, point_size, coeffs, start_x, end_x);
      if (start_x < end_x) {
        double start_y = coeffs(0) + coeffs(1) * start_x +
                         coeffs(2) * start_x * start_x +
                         coeffs(3) * start_x * start_x * start_x;
        double end_y = coeffs(0) + coeffs(1) * end_x +
                       coeffs(2) * end_x * end_x +
                       coeffs(3) * end_x * end_x * end_x;
        for (size_t i = 0; i < bev_sampling_points.size(); ++i) {
          problem.AddResidualBlock(
              new ceres::AutoDiffCostFunction<Point2CurveResidual, 1, 3>(
                  new Point2CurveResidual(bev_sampling_points.at(i).x(),
                                          bev_sampling_points.at(i).y(),
                                          coeffs(0), coeffs(1), coeffs(2),
                                          coeffs(3), start_x, start_y, end_x,
                                          end_y, point_match_prob.at(i))),
              loss_function, params.data());
          if (bev_sampling_points.at(i).x() > max_x) {
            max_x = bev_sampling_points.at(i).x();
          }
          if (bev_sampling_points.at(i).x() < min_x) {
            min_x = bev_sampling_points.at(i).x();
          }
        }
      }
    };
  }
  if (max_x - min_x < 30) {
    is_match_info_believable_ = false;
    return params;
  } else {
    is_match_info_believable_ = true;
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.is_constrained) {
    params = std::vector<double>(3, 0);
  }
  auto end_ts = GetMwTimeNowSec();
  AINFO << "optimize by matching info: theta=" << std::setprecision(16)
        << params[0] << ", tx=" << params[1] << ", ty=" << params[2]
        << ", time spent: " << end_ts - start_ts << std::endl;
  delete loss_function;
  return params;
}

std::vector<double> MultiSourceDataMixer::EstimatePoseOffsetByMatchInfoRANSAC(
    const std::unordered_map<
        uint64_t,
        std::vector<std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>>>&
        match_details,
    int max_iteration_num, int minimum_sample_size,
    double inliner_distance_threshold,
    double inliner_min_sample_prob_threshold) {
  struct LdFittedLine {
    double c0;
    double c1;
    double c2;
    double c3;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
  };
  struct MatchInfoInternal {
    Eigen::Vector2d bev_point;
    std::shared_ptr<LdFittedLine> ld_line;
    double match_probility;
  };
  auto start_ts = GetMwTimeNowSec();

  // std::array<std::shared_ptr<LdFittedLine>, 300> ld_fitted_line_buf;
  std::vector<double> best_estimate(3, 0);  // theta, tx, ty，初始值
  std::vector<MatchInfoInternal> match_samples;

  int min_x = std::numeric_limits<double>::max();
  int max_x = std::numeric_limits<double>::lowest();
  size_t ld_line_count = 0;
  for (auto& matcher : match_details) {
    std::vector<const std::vector<cem::message::env_model::Point>*> ld_points;
    int ld_point_size = 0;
    for (auto& seg : matcher.second) {
      auto ld_lane = ld_map_preprocessor_->GetLane(seg->ldmap_id);
      ld_points.push_back(&(ld_lane->points));
      ld_point_size += ld_lane->points.size();
    }

    if (ld_point_size < 4) {
      continue;
    }

    Eigen::Vector4d coeffs;
    coeffs << 0, 0, 0, 0;
    std::shared_ptr<LdFittedLine> ld_line_ptr =
        std::make_shared<LdFittedLine>();
    FitCubicCurve(ld_points, ld_point_size, coeffs, ld_line_ptr->start_x,
                  ld_line_ptr->end_x);

    if (ld_line_ptr->start_x >= ld_line_ptr->end_x) {
      continue;
    }

    ld_line_ptr->c0 = coeffs(0);
    ld_line_ptr->c1 = coeffs(1);
    ld_line_ptr->c2 = coeffs(2);
    ld_line_ptr->c3 = coeffs(3);
    ld_line_ptr->start_y =
        coeffs(0) + coeffs(1) * ld_line_ptr->start_x +
        coeffs(2) * ld_line_ptr->start_x * ld_line_ptr->start_x +
        coeffs(3) * ld_line_ptr->start_x * ld_line_ptr->start_x *
            ld_line_ptr->start_x;
    ld_line_ptr->end_y = coeffs(0) + coeffs(1) * ld_line_ptr->end_x +
                         coeffs(2) * ld_line_ptr->end_x * ld_line_ptr->end_x +
                         coeffs(3) * ld_line_ptr->end_x * ld_line_ptr->end_x *
                             ld_line_ptr->end_x;

    for (auto& seg : matcher.second) {
      for (auto i = seg->bev_start_point_idx; i <= seg->bev_end_point_idx;
           ++i) {
        Eigen::Vector2d pt;
        if (!bev_slicer_->GetSamplingPoint(matcher.first, i, pt)) {
          continue;
        }
        max_x = max_x < pt.x() ? pt.x() : max_x;
        min_x = min_x > pt.x() ? pt.x() : min_x;
        MatchInfoInternal match_sample;
        match_sample.bev_point = std::move(pt);
        match_sample.ld_line = ld_line_ptr;
        match_sample.match_probility = 0.0;
        if (seg->point_probability.count(i) > 0) {
          match_sample.match_probility = seg->point_probability.at(i);
        }
        match_samples.push_back(std::move(match_sample));
      }
    }
  }

  if (max_x - min_x < 30) {
    is_match_info_believable_ = false;
    return best_estimate;
  } else {
    is_match_info_believable_ = true;
  }

  if (match_samples.size() <= 2 * minimum_sample_size) {
    // todo:params
    is_small_size_of_sampling_point = true;
    return EstimatePoseOffsetByMatchInfo(match_details);
  } else {
    is_small_size_of_sampling_point = false;
  }

  auto calc_sample_point_error =
      [](const MatchInfoInternal& sample,
         const std::vector<double>& params) -> double {
    double bev_trans_x = sample.bev_point.x() * std::cos(-params.at(0)) -
                         sample.bev_point.y() * std::sin(-params.at(0));
    double bev_trans_y = sample.bev_point.x() * std::sin(-params.at(0)) +
                         sample.bev_point.y() * std::cos(-params.at(0)) -
                         params.at(2);
    if (bev_trans_x > sample.ld_line->end_x) {
      auto delta_x = bev_trans_x - sample.ld_line->end_x;
      auto delta_y = bev_trans_y - sample.ld_line->end_y;
      return std::sqrt(delta_x * delta_x + delta_y * delta_y);
    } else if (bev_trans_x < sample.ld_line->start_x) {
      auto delta_x = bev_trans_x - sample.ld_line->start_x;
      auto delta_y = bev_trans_y - sample.ld_line->start_y;
      return std::sqrt(delta_x * delta_x + delta_y * delta_y);
    } else {
      double delta_y =
          sample.ld_line->c0 + sample.ld_line->c1 * bev_trans_x +
          sample.ld_line->c2 * bev_trans_x * bev_trans_x +
          sample.ld_line->c3 * bev_trans_x * bev_trans_x * bev_trans_x -
          bev_trans_y;
      return std::abs(delta_y);
    }
  };

  auto solve_pose_offset = [](ceres::Problem& problem,
                              ceres::Solver::Options& options,
                              ceres::Solver::Summary& summary,
                              std::vector<MatchInfoInternal*>& inliners,
                              std::vector<double>& params_estimate) -> bool {
    std::vector<ceres::ResidualBlockId> residual_blocks;
    problem.GetResidualBlocks(&residual_blocks);
    for (auto& block_id : residual_blocks) {
      problem.RemoveResidualBlock(block_id);
    }
    for (auto sample : inliners) {
      problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction<Point2CurveResidual, 1, 3>(
              new Point2CurveResidual(
                  sample->bev_point.x(), sample->bev_point.y(),
                  sample->ld_line->c0, sample->ld_line->c1, sample->ld_line->c2,
                  sample->ld_line->c3, sample->ld_line->start_x,
                  sample->ld_line->start_y, sample->ld_line->end_x,
                  sample->ld_line->end_y, sample->match_probility)),
          nullptr, params_estimate.data());
    };

    ceres::Solve(options, &problem, &summary);

    return true;
  };

  auto get_random_initial_samples =
      [this, minimum_sample_size](std::vector<MatchInfoInternal>& match_samples,
                                  std::vector<MatchInfoInternal*> inliners) {
        inliners.clear();
        auto random_idxs =
            this->GetRandomSamplesIndex(match_samples.size(), false);
        for (size_t i = 0; i < minimum_sample_size; ++i) {
          auto idx = random_idxs.at(i);
          inliners.emplace_back(&match_samples.at(idx));
        }
      };

  auto random_idxs = GetRandomSamplesIndex(match_samples.size(), true);
  std::vector<std::vector<int>> initial_samples_groups;
  for (size_t i = 0; i + minimum_sample_size <= random_idxs.size();
       i += minimum_sample_size) {
    if (initial_samples_groups.size() > 3) {
      break;
    }
    std::vector<int> sample_tmp(random_idxs.begin() + i,
                                random_idxs.begin() + i + minimum_sample_size);
    initial_samples_groups.push_back(std::move(sample_tmp));
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;

  int max_count_inliner = 0;
  double sigma = 0.3;  // 随机选取一个点是内点的概率
  std::vector<MatchInfoInternal*> inliners;

  ceres::Problem problem;
  for (auto& group : initial_samples_groups) {
    std::vector<double> params_estimate(3, 0);  // theta, tx, ty，初始值

    inliners.clear();
    for (auto idx : group) {
      inliners.emplace_back(&match_samples.at(idx));
    }

    for (auto iter_num = 1; iter_num <= max_iteration_num; ++iter_num) {
      if (!solve_pose_offset(problem, options, summary, inliners,
                             params_estimate)) {
        // todo: 重新选初始点
        get_random_initial_samples(match_samples, inliners);
        continue;
      }

      inliners.clear();
      for (auto& sample : match_samples) {
        auto error = calc_sample_point_error(sample, params_estimate);
        if (error < inliner_distance_threshold) {
          inliners.push_back(&sample);
        }
      }
      if (inliners.size() <= minimum_sample_size) {
        get_random_initial_samples(match_samples, inliners);
        continue;
      }

      if (!solve_pose_offset(problem, options, summary, inliners,
                             params_estimate)) {
        get_random_initial_samples(match_samples, inliners);
        continue;
      }

      int count_inlier = 0;
      for (auto& sample : match_samples) {
        auto error = calc_sample_point_error(sample, params_estimate);
        if (error < inliner_distance_threshold) {
          count_inlier++;
        }
      }
      if (count_inlier > max_count_inliner) {
        max_count_inliner = count_inlier;
        best_estimate = params_estimate;
      }

      if (std::pow((1 - std::pow(sigma, minimum_sample_size)), iter_num) <
          1 - inliner_min_sample_prob_threshold) {
        break;
      }
      sigma = double(max_count_inliner) / match_samples.size();
    }

    get_random_initial_samples(match_samples, inliners);
  }
  if (max_count_inliner > 0.7 * match_samples.size()) {
    is_match_info_believable_ = true;
  } else {
    is_match_info_believable_ = false;
  }

  auto end_ts = GetMwTimeNowSec();
  AINFO << "optimize by matching info: theta=" << std::setprecision(16)
        << best_estimate[0] << ", tx=" << best_estimate[1]
        << ", ty=" << best_estimate[2] << ", time spent: " << end_ts - start_ts
        << std::endl;
  return best_estimate;
}

std::vector<double> MultiSourceDataMixer::EstimatePoseOffsetByLastState(
    const Eigen::Isometry3d& T) {
  std::vector<double> params(3, 0);  // theta, tx, ty，初始值
  params.at(0) = predicted_pose_offset_.at(0);
  params.at(1) = predicted_pose_offset_.at(1);
  params.at(2) = predicted_pose_offset_.at(2);
  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

  std::unordered_map<uint64_t, const cem::message::env_model::LaneInfo*>
      input_lanes;
  std::unordered_map<uint64_t, const cem::message::env_model::LaneInfo*>
      hist_lanes;
  for (auto& lane : ld_map_preprocessor_->GetData()->lanes) {
    if (lane.points.empty()) {
      continue;
    }
    input_lanes.emplace(lane.id, &lane);
  }
  for (auto& lane : corrected_map_->lanes) {
    if (lane.points.empty()) {
      continue;
    }
    hist_lanes.emplace(lane.id, &lane);
  }
  for (auto [id, input_lane] : input_lanes) {
    if (hist_lanes.count(id) == 0) {
      continue;
    }
    auto hist_lane = hist_lanes.at(id);
    if (input_lane->points.size() != hist_lane->points.size()) {
      continue;
    }

    Eigen::Vector3d hist_point(hist_lane->points.front().x,
                               hist_lane->points.front().y,
                               hist_lane->points.front().z);
    auto& cur_point_front = input_lane->points.front();
    hist_point = T * hist_point;
    // AINFO << "hist_pt1: " << hist_point.x() << ", " << hist_point.y();
    // AINFO << "cur__pt1: " << cur_point_front.x << ", " << cur_point_front.y;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Point2PointResidual, 2, 3>(
            new Point2PointResidual(cur_point_front.x, cur_point_front.y,
                                    hist_point.x(), hist_point.y())),
        loss_function, params.data());

    hist_point =
        Eigen::Vector3d(hist_lane->points.back().x, hist_lane->points.back().y,
                        hist_lane->points.back().z);
    auto& cur_point_back = input_lane->points.back();
    hist_point = T * hist_point;
    // AINFO << "hist_pt2: " << hist_point.x() << ", " << hist_point.y();
    // AINFO << "cur__pt2: " << cur_point_back.x << ", " << cur_point_back.y;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Point2PointResidual, 1, 3>(
            new Point2PointResidual(cur_point_back.x, cur_point_back.y,
                                    hist_point.x(), hist_point.y())),
        loss_function, params.data());
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  AINFO << "optimize by hist state: theta=" << params[0] << ", tx=" << params[1]
        << ", ty=" << params[2] << std::endl;
  delete loss_function;
  return params;
}

std::vector<double> MultiSourceDataMixer::EstimatePoseOffsetByLastStateRANSAC(
    const Eigen::Isometry3d& T, int max_iteration_num, int minimum_sample_size,
    double inliner_distance_threshold,
    double inliner_min_sample_prob_threshold) {
  std::vector<double> params(3, 0);  // theta, tx, ty，初始值
  params.at(0) = predicted_pose_offset_.at(0);
  params.at(1) = predicted_pose_offset_.at(1);
  params.at(2) = predicted_pose_offset_.at(2);
  ceres::Problem problem;
  ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);

  std::unordered_map<uint64_t, const cem::message::env_model::LaneInfo*>
      input_lanes;
  std::unordered_map<uint64_t, const cem::message::env_model::LaneInfo*>
      hist_lanes;
  for (auto& lane : ld_map_preprocessor_->GetData()->lanes) {
    if (lane.points.empty()) {
      continue;
    }
    input_lanes.emplace(lane.id, &lane);
  }
  for (auto& lane : corrected_map_->lanes) {
    if (lane.points.empty()) {
      continue;
    }
    hist_lanes.emplace(lane.id, &lane);
  }
  for (auto [id, input_lane] : input_lanes) {
    if (hist_lanes.count(id) == 0) {
      continue;
    }
    auto hist_lane = hist_lanes.at(id);
    if (input_lane->points.size() != hist_lane->points.size()) {
      continue;
    }

    Eigen::Vector3d hist_point(hist_lane->points.front().x,
                               hist_lane->points.front().y,
                               hist_lane->points.front().z);
    auto& cur_point_front = input_lane->points.front();
    hist_point = T * hist_point;
    // AINFO << "hist_pt1: " << hist_point.x() << ", " << hist_point.y();
    // AINFO << "cur__pt1: " << cur_point_front.x << ", " << cur_point_front.y;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Point2PointResidual, 2, 3>(
            new Point2PointResidual(cur_point_front.x, cur_point_front.y,
                                    hist_point.x(), hist_point.y())),
        loss_function, params.data());

    hist_point =
        Eigen::Vector3d(hist_lane->points.back().x, hist_lane->points.back().y,
                        hist_lane->points.back().z);
    auto& cur_point_back = input_lane->points.back();
    hist_point = T * hist_point;
    // AINFO << "hist_pt2: " << hist_point.x() << ", " << hist_point.y();
    // AINFO << "cur__pt2: " << cur_point_back.x << ", " << cur_point_back.y;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Point2PointResidual, 1, 3>(
            new Point2PointResidual(cur_point_back.x, cur_point_back.y,
                                    hist_point.x(), hist_point.y())),
        loss_function, params.data());
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  AINFO << "optimize by hist state: theta=" << params[0] << ", tx=" << params[1]
        << ", ty=" << params[2] << std::endl;
  delete loss_function;
  return params;
}

bool MultiSourceDataMixer::IsPoseEstimateBelievable(
    const std::vector<double>& pose_offset_by_last_state,
    const std::vector<double>& pose_offset_by_vision) {
  // todo:
  if (!is_match_info_believable_) {
    return false;
  }
  if (is_small_size_of_sampling_point) {
    if (fabs(pose_offset_by_last_state.at(2) - pose_offset_by_vision.at(2)) >
        0.5) {
      return false;
    }
    if (fabs(pose_offset_by_last_state.at(1) - pose_offset_by_vision.at(1)) >
        0.5) {
      return false;
    }
  }
  return true;
}

Eigen::Matrix4d MultiSourceDataMixer::ConstructLdMapCorrectT(
    const std::vector<double>& params) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(params[0]);
  T(0, 1) = std::sin(params[0]);
  T(0, 2) = 0.0;
  T(0, 3) = params[1];
  T(1, 0) = std::sin(params[0]);
  T(1, 1) = std::cos(params[0]);
  T(1, 2) = 0.0;
  T(1, 3) = params[2];
  T(2, 0) = 0.0;
  T(2, 1) = 0.0;
  T(2, 2) = 1.0;
  T(2, 3) = 0.0;

  return T;
}

std::vector<int> MultiSourceDataMixer::GetRandomSamplesIndex(int n,
                                                             bool fix_seed) {
  std::vector<int> v(n);
  std::iota(v.begin(), v.end(), 0);
  if (fix_seed) {
    std::mt19937 rng(1568771);
    shuffle(v.begin(), v.end(), rng);
  } else {
    std::mt19937 rng(std::random_device{}());
    shuffle(v.begin(), v.end(), rng);
  }
  return v;
}

void MultiSourceDataMixer::DataFusion() {
  // todo:
}

}  // namespace fusion
}  // namespace cem
