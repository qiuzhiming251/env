#include "kde_regression.h"

#include <cmath>
#include <numeric>
#include <algorithm>




namespace geometry {

// bool KdeRegression::SmoothingWithSpecifiedStart(
//     const std::vector<WeightedCurve> &raw_curves,
//     const double &max_error, const double &resolution,
//     const Eigen::Vector3f &start_pt,
//     cem::message::env_model::GeometryLine &fitted_curve) {
//   const double half_resolution = resolution / 2.0;
//   // adapt inputs
//   std::vector<WeightedPoint> points;
//   std::vector<DirectedPoint> seeds;
//   AdaptInputs(raw_curves, points, seeds);

//   auto end_pt = FindIterationEndPoint(raw_curves, half_resolution);

//   // smoothing
//   std::vector<Point> smoothed_points;
//   PerformKernelSmoothing(
//       start_pt, end_pt, points, static_cast<float>(max_error), smoothed_points);

//   // adapt outputs
//   fitted_curve.sampling_points.clear();
//   fitted_curve.sampling_points.reserve(smoothed_points.size());
//   for (const auto &pt : smoothed_points) {
//     fitted_curve.sampling_points.emplace_back(pt.x(), pt.y());
//   }

//   // post-process, resample
//   curve_operations::Resample(resolution, fitted_curve);

//   bool are_all_start_pts_visited = std::all_of(
//       raw_curves.begin(), raw_curves.end(),
//       [&](const WeightedCurve &weighted_curve) {
//     return spatial_relationships::IsPointOnCurve(
//         weighted_curve.curve.sampling_points.front(), fitted_curve, half_resolution);
//   });
//   return are_all_start_pts_visited;
// }
//计算点index为I的点（和他后面一个点的）曲线方向  反正切
double KdeRegression::CalcCurveDirection(const  cem::message::env_model::GeometryLine &curve, int i) {
  const auto &points = curve.sampling_points;
  int size = static_cast<int>(points.size());
  if (size < 2) { return 0.0; }
  int tar_idx = i;
  // 处理负数idx（逆向数）
  if (i < 0) {
    tar_idx = size + i;
  }
  // 有效性制约
  if(tar_idx + 1 > size - 1) {return  0.0;};
  tar_idx = std::max(0, tar_idx);//小于0 则等于0 大于0 则等于tar_idx
  tar_idx = std::min(size - 2, tar_idx);//大于size-2 则等于size-2  两头截断

  return std::atan2(points[tar_idx + 1].y - points[tar_idx].y,
                    points[tar_idx + 1].x - points[tar_idx].x);
}


bool KdeRegression::Smoothing(
    const std::vector<WeightedCurve> &raw_curves,
    const double &max_error, const double &resolution,
    cem::message::env_model::GeometryLine &fitted_curve) {
  // adapt inputs
  std::vector<WeightedPoint> points;
  std::vector<DirectedPoint> seeds;
  AdaptInputs(raw_curves, points, seeds);
  // 寻找迭代起始点
  const double tolerance{resolution / 1.3};
  auto start = FindIterationStartPoint(raw_curves, tolerance);//obb
  auto end = FindIterationEndPoint(raw_curves, tolerance);

  // smoothing
  std::vector<Point> smoothed_points;
  PerformKernelSmoothing(
      start, end, points, static_cast<float>(max_error), smoothed_points);//AABB

  // adapt outputs
  fitted_curve.sampling_points.clear();
  fitted_curve.sampling_points.reserve(smoothed_points.size());
  for (const auto &pt : smoothed_points) {
    fitted_curve.sampling_points.emplace_back(pt.x(), pt.y());
  }
  if (fitted_curve.sampling_points.size() < 2U) { return false; }

  // post-process, resample
  // curve_operations::Resample(resolution, fitted_curve);

  // bool are_all_start_pts_visited = std::all_of(
  //     raw_curves.begin(), raw_curves.end(),
  //     [&](const WeightedCurve &w_curve) {
  //       if (w_curve.weight < 1e-5) { return true; }
  //       if (!w_curve.start_end_candidate) { return true; }
  //       return spatial_relationships::IsPointOnCurve(
  //           w_curve.curve.sampling_points.front(), fitted_curve, resolution) ||
  //           spatial_relationships::IsPointOnCurve(
  //           w_curve.curve.sampling_points.back(), fitted_curve, resolution);
  //     });
  return true;
}

void KdeRegression::AdaptInputs(
    const std::vector<WeightedCurve> &raw_curves,
    std::vector<WeightedPoint> &points, std::vector<DirectedPoint> &seeds) {
  points.clear();
  seeds.clear();

  seeds.reserve(raw_curves.size());
  for (const auto &ele : raw_curves) {
    const auto &curve_points = ele.curve.sampling_points;
    const auto &curve_weight = ele.weight;
    // 这里的点要大于等于两个  取每条线的两个点
    if (curve_points.size() < 2U) { continue; }
    const auto &st_pt = curve_points[0];
    const auto &next_pt = curve_points[1];
    float angle = atan2f(static_cast<float>(next_pt.y - st_pt.y),
                         static_cast<float>(next_pt.x - st_pt.x));

    seeds.emplace_back(st_pt.x, st_pt.y, angle);

    for (const auto &pt : curve_points) {
      points.emplace_back(Point(pt.x, pt.y), curve_weight);
    }
  }
}

bool KdeRegression::CurveFindSuccessor(const std::vector<WeightedCurve> &curves,
                                       double max_error, WeightedDPoint &seed,
                                       std::vector<bool> &visit_status) {
  constexpr double kAngleDiffThresh = 2.2 / 180.0 * M_PI;
  constexpr double kDistDiffThresh = 0.24;

  const cem::message::env_model::SamplingPoint &seed_pt = seed.pt;
  const double seed_theta = seed.theta;
  const cem::message::env_model::SamplingPoint seed_heading_vct{std::cos(seed_theta),
                                              std::sin(seed_theta)};
  const cem::message::env_model::SamplingPoint seed_perpendicular_vct{-seed_heading_vct.y,
                                                    seed_heading_vct.x};
  const double seed_weight = seed.weight;
  // 寻找末端点落在曲线上的可能候选
  double max_weight{0.0};
  int max_weight_index{-1};
  for (size_t i = 0UL; i < curves.size(); ++i) {
    if (visit_status[i]) { continue; }
    if (!curves[i].start_end_candidate) { continue; }
    // 权重为0的观测最低优选为迭代起/止点
    if (curves[i].weight < 1e-5 && seed_weight > 1e-5) { continue; }
    // 挑选的是满足条件的权重最高点
    if (curves[i].weight <= max_weight) { continue; }
    // 检查点不落在曲线上的情况
    auto index_pt_on_curve = spatial_relationships::IndexOfPointOnCurve(
        seed_pt, curves[i].curve, max_error);
    if (index_pt_on_curve < 0) { continue; }
    // 首点
    const auto &st_pt = curves[i].curve.sampling_points.front();
    // 尾点
    const auto &ed_pt = curves[i].curve.sampling_points.back();
    // 投影点
    const cem::message::env_model::SamplingPoint proj =
        spatial_relationships::ProjectionPointPointToCurve(seed_pt,
                                                           curves[i].curve);
    const cem::message::env_model::SamplingPoint proj_pt(proj);
    const double proj_theta = proj.heading;//角度
    // 连线
    cem::message::env_model::SamplingPoint seed_to_st = st_pt + (-1.0 * seed_pt);
    cem::message::env_model::SamplingPoint seed_to_ed = ed_pt + (-1.0 * seed_pt);
    cem::message::env_model::SamplingPoint seed_to_proj = proj_pt + (-1.0 * seed_pt);

    if (geometry::spatial_relationships::PointDotProduct(seed_heading_vct,
                                                         seed_to_ed) > 1.2) {
      double lat_dist = std::abs(spatial_relationships::PointDotProduct(
          seed_perpendicular_vct, seed_to_proj));

      double candidate_theta =
          CalcCurveDirection(curves[i].curve, -1);
      if ((curves[i].weight < seed_weight) && (lat_dist > kDistDiffThresh) &&
          (CalcAngelAbs(proj_theta, seed_theta) > kAngleDiffThresh)) {//算角度差值
        // std::cout << "[debug] proj_theta is " << proj_theta / M_PI * 180
        //           << ", seed heading is " << seed_theta / M_PI * 180
        //           << ", diff is "
        //           << CalcAngelAbs(proj_theta, seed_theta) / M_PI * 180
        //           << std::endl;
        continue;
      }
      seed = WeightedDPoint{curves[i].curve.sampling_points.back(), candidate_theta,
                            curves[i].weight};
      // 选取为新的seed
      // 更新权重记录
      max_weight = curves[i].weight;
      max_weight_index = static_cast<int>(i);
    } else if (geometry::spatial_relationships::PointDotProduct(
                   seed_heading_vct, seed_to_st) > 1.2) {
      double lat_dist = std::abs(spatial_relationships::PointDotProduct(
          seed_perpendicular_vct, seed_to_proj));

      double candidate_theta =
          CalcCurveDirection(curves[i].curve, 0) + M_PI;
      if ((curves[i].weight < seed_weight) && (lat_dist > kDistDiffThresh) &&
          (CalcAngelAbs(proj_theta + M_PI, seed_theta) >
              kAngleDiffThresh)) {
        // std::cout << "[debug] proj_theta is " << proj_theta / M_PI * 180
        //           << ", seed heading is " << seed_theta / M_PI * 180
        //           << ", diff is "
        //           << CalcAngelAbs(proj_theta, seed_theta) / M_PI * 180
        //           << std::endl;
        continue;
      }
      seed = WeightedDPoint{curves[i].curve.sampling_points.front(), candidate_theta,
                            curves[i].weight};
      // 选取为新的seed
      // 更新权重记录
      max_weight = curves[i].weight;
      max_weight_index = static_cast<int>(i);
    }
  }
  // 给出搜索结论，更新访问状态
  if (max_weight_index == -1) {
    return false;
  }
  visit_status[max_weight_index] = true;
  return true;
}

bool KdeRegression::CurveFindPredecessor(
    const std::vector<WeightedCurve> &curves, double max_error,
    WeightedDPoint &seed, std::vector<bool> &visit_status) {
  const cem::message::env_model::SamplingPoint &seed_pt = seed.pt;
  const double seed_theta = seed.theta;
  const double seed_weight = seed.weight;
  // 寻找末端点落在曲线上的可能候选
  double max_weight{0.0};
  int max_weight_index{-1};
  for (size_t i = 0U; i < curves.size(); ++i) {//遍历多个曲线
    if (visit_status[i]) { continue; }
    if (!curves[i].start_end_candidate) { continue; }
    // 权重为0的观测最低优选为迭代起/止点
    if (curves[i].weight < 1e-5 && seed_weight > 1e-5) { continue; }
    // 挑选的是满足条件的权重最高点
    if (curves[i].weight <= max_weight) { continue; }

    auto index_pt_on_curve = spatial_relationships::IndexOfPointOnCurve(
        seed_pt, curves[i].curve, max_error);
    // 点不落在曲线上
    if (index_pt_on_curve < 0) { continue; }

    cem::message::env_model::SamplingPoint norm_seed{std::cos(seed_theta), std::sin(seed_theta)};
    const auto &st_pt = curves[i].curve.sampling_points.front();//曲线起点
    const auto &ed_pt = curves[i].curve.sampling_points.back();//曲线终点
    cem::message::env_model::SamplingPoint st_line = st_pt + (-1.0 * seed_pt);//起点减去seed_pt 点
    cem::message::env_model::SamplingPoint ed_line = ed_pt + (-1.0 * seed_pt);

    if (geometry::spatial_relationships::PointDotProduct(
        norm_seed, st_line) < -0.6) {//点集
      double candidate_theta =
          CalcCurveDirection(curves[i].curve, 0);
      seed = WeightedDPoint{curves[i].curve.sampling_points.front(),
                            candidate_theta, curves[i].weight};
      // 选取为新的seed
      // 更新权重记录
      max_weight = curves[i].weight;
      max_weight_index = static_cast<int>(i);
    } else if (geometry::spatial_relationships::PointDotProduct(
        norm_seed, ed_line) < -0.6) {
      double candidate_theta =
          CalcCurveDirection(curves[i].curve, -1);
      seed = WeightedDPoint{curves[i].curve.sampling_points.back(),
                            candidate_theta + M_PI, curves[i].weight};
      // 选取为新的seed
      // 更新权重记录
      max_weight = curves[i].weight;
      max_weight_index = static_cast<int>(i);
    }
  }
  // 给出搜索结论，更新访问状态
  if (max_weight_index == -1) { return false; }
  visit_status[max_weight_index] = true;
  return true;
}
// 返回具有方向和坐标的点
DirectedPoint KdeRegression::FindIterationStartPoint(
    const std::vector<WeightedCurve> &curves, double max_error) {
  // 找到最高权重的观测，从它出发寻找迭代起始点
  //curves 为空直接返回
  if (curves.empty()) {
    return {0.0f, 0.0f, 0.0f};
  }
  auto target_curve = std::max_element(
      curves.begin(), curves.end(), CompareWeight);//找到最大元素和begin的index的距离 
  int target_index =
      static_cast<int>(std::distance(curves.begin(), target_curve));
  // 搜索状态维护
  std::vector<bool> visit_status(curves.size(), false);
  visit_status[target_index] = true;
  // 确定搜索seed
  double candidate_theta =
      CalcCurveDirection(curves[target_index].curve, 0);
  WeightedDPoint seed{curves[target_index].curve.sampling_points.front(),
                      candidate_theta, curves[target_index].weight};
  // 迭代搜索
  while (true) {
    bool has_found =
        CurveFindPredecessor(curves, max_error, seed, visit_status);
    if (!has_found) { break; }
  }
  // 返回搜索结果
  return {static_cast<float>(seed.pt.x),
          static_cast<float>(seed.pt.y),
          static_cast<float>(seed.theta)};
}

Point KdeRegression::FindIterationEndPoint(
    const std::vector<WeightedCurve> &curves, double max_error) {
  // 找到最高权重的观测，从它出发寻找迭代起始点
  //curves 为空直接返回
  if (curves.empty()) {
    return {0.0f, 0.0f};
  }
  auto target_curve = std::max_element(
      curves.begin(), curves.end(), CompareWeight);
  int target_index =
      static_cast<int>(std::distance(curves.begin(), target_curve));
  // 搜索状态维护
  std::vector<bool> visit_status(curves.size(), false);
  visit_status[target_index] = true;
  // 确定搜索seed
  double candidate_theta =
      CalcCurveDirection(curves[target_index].curve, -1);
  WeightedDPoint seed{curves[target_index].curve.sampling_points.back(),
                      candidate_theta, curves[target_index].weight};
  // 迭代搜索
  while (true) {
    bool has_found =
        CurveFindSuccessor(curves, max_error, seed, visit_status);
    if (!has_found) { break; }
  }
  // 返回搜索结果
  return {seed.pt.x, seed.pt.y};
}

bool KdeRegression::FindIterationStartPoint(
    const std::vector<WeightedPoint> &points,
    const std::vector<DirectedPoint> &seeds,
    const double &max_error,
    DirectedPoint &start) {
  //----------------------------------------------------------------------------
  // 寻找key_seed，即满足“身后有向矩形OBB内无其他seeds”条件的seed
  //----------------------------------------------------------------------------
  // 对每个seed构建有向矩形OBB
  double rect_side_len = 1.5F * max_error;
  std::vector<OBB> candidate_rectangles;
  candidate_rectangles.reserve(seeds.size());
  for (const auto &seed : seeds) {
    DirectedPoint obb_st{seed[0], seed[1], static_cast<float>(seed[2] + M_PI)};
    candidate_rectangles.emplace_back(obb_st, rect_side_len,
                                      0.5F * rect_side_len);
  }
  // 寻找满足条件的向矩形OBB
  auto key_rect = std::find_if(
      candidate_rectangles.begin(), candidate_rectangles.end(),
      [&](const OBB &rect) {
        return IsRectangleClean(rect, points);
      });
  if (key_rect == candidate_rectangles.end()) { return false; }
  DirectedPoint key_seed = key_rect->GetStartPoint();

  //----------------------------------------------------------------------------
  // 从key_seed出发，计算迭代起始点
  //----------------------------------------------------------------------------
  // 构建以key_seed为中心的OBB
  auto side_len = static_cast<float>(max_error);
  DirectedPoint seg_center(
      key_seed[0] - 0.5F * side_len * std::cos(key_seed[2]),
      key_seed[1] - 0.5F * side_len * std::sin(key_seed[2]),
      key_seed[2]);
  OBB obb(seg_center, side_len, 0.5F * side_len);
  // 找出obb内所有seeds
  auto filter_func =
      [obb](const DirectedPoint &seed) {
    return obb.IsContainPoint({seed.x(), seed.y()});
  };
  auto inner_seeds = VectorFilter(seeds, filter_func);
  // 求重心点作为迭代起始点
  start = CalcCenterPoint(inner_seeds);
  return true;
}

DirectedPoint KdeRegression::CalcCenterPoint(
    const std::vector<DirectedPoint> &pts) {
  // average position
  DirectedPoint center = std::accumulate(
      pts.begin(), pts.end(),
      DirectedPoint(0.0F, 0.0F, 0.0F)) / pts.size();
  // average angle
  std::vector<double> angles;
  angles.reserve(pts.size());
  for (const auto &pt : pts) {
    angles.push_back(pt[2]);
  }
  center[2] = static_cast<float >(AngleAverage(angles));
  return center;
}

bool KdeRegression::PerformKernelSmoothing(
    const DirectedPoint &start, const Point &end,
    const std::vector<WeightedPoint> &raw_points,
    float max_error, std::vector<Point> &smoothed_points) {

  std::vector<int> visit_status(raw_points.size(), 0);

  smoothed_points.clear();
  smoothed_points.emplace_back(start.x(), start.y());

  DirectedPoint iter_pt(start);
  bool circle_flag = false;
  int visited_point_num{0};
  float side_len = 0.59634F * max_error;

  bool is_end_visited{false};
  while (true) {
    // construct directed_rectangle
    OBB ref_rect(iter_pt, max_error, side_len);
    // find rectangle inner points
    Point sum_inner_points(0.0F, 0.0F);
    float sum_weights{0.0F};
    int cnt_inner_points{0};
    int cnt_inner_new_points{0};
    for (size_t i = 0U; i < raw_points.size(); ++i) {
      const auto &pt = raw_points[i].first;
      float pt_weight = std::max(raw_points[i].second, 0.1F);
      if (!ref_rect.IsContainPoint(pt)) { continue; }
      // 确定是否是初次访问的点
      bool is_pt_first_time_visited = (visit_status[i] == 0);
      // 计数维护与更新
      ++cnt_inner_points;
      cnt_inner_new_points += static_cast<int>(is_pt_first_time_visited);
      visited_point_num += static_cast<int>(is_pt_first_time_visited);
      // 为访问过的点降权以促进收敛
      int visit_times = visit_status[i];
      switch (visit_times) {
        case 0: break;
        case 1: pt_weight /= 2.0F; break;
        case 2: pt_weight /= 4.0F; break;
        default: pt_weight = 0.0F; break;
      }
      // 加权求和
      sum_inner_points.noalias() += pt * pt_weight;
      sum_weights += pt_weight;
      // 访问次数更新
      ++visit_status[i];
      if (visit_status[i] > 5)  {
        circle_flag = true;
        break;
      }
    }

    // 迭代收敛条件：重复访问
    if (sum_weights < 1e-5F) { break; }
    // 迭代收敛条件：遇到环
    if (circle_flag) { break; }
    // box内没有其他点了，终止迭代
    if (cnt_inner_new_points == 0 && cnt_inner_points <= 1) { break; }

    // calc weight center
    const Point center_pt = sum_inner_points / sum_weights;
    // calc angle from iter_pt to weight center
    float direction =
        std::atan2(center_pt.y() - iter_pt.y(), center_pt.x() - iter_pt.x());
    // determine next iteration point
    DirectedPoint new_iter_pt{
        center_pt.x() + -0.1F * static_cast<float>(std::cos(direction)),
        center_pt.y() + -0.1F * static_cast<float>(std::sin(direction)),
        direction};

    // 步长过小，迭代终止
    if (std::abs(new_iter_pt.x() - iter_pt.x()) < 1e-5 &&
        std::abs(new_iter_pt.y() - iter_pt.y()) < 1e-5) {
      break;
    }

    // 检查是否到达迭代终止点, 到达之后再进行最后一次迭代
    if (is_end_visited) {
      smoothed_points.emplace_back(center_pt);
      smoothed_points.emplace_back(end);
      break;
    }
    if (IsTwoPointsEqual(center_pt, end, 2.5)) {
      is_end_visited = true;
    }

    // 角度变化过大，迭代终止
    // float delta_angle = utils::AngleDiff(iter_pt.z(), new_iter_pt.z());
    // if (std::abs(delta_angle) > M_PI_4) { break; }

    // 弯度过大，迭代终止
    // Point new_pt{new_iter_pt.x(), new_iter_pt.y()};
    // map_interface::GEO_LINE2D last_line{
    //   {iter_pt.x(), iter_pt.y()},
    //   {iter_pt.x() + std::cos(iter_pt.z()),
    //    iter_pt.y() + std::sin(iter_pt.z())}};
    // float dist =
    // map_interface::GeometryXY::DistanceXY2Line(new_pt, last_line);
    // if (dist > 1.6f) { break; }

    // 迭代
    smoothed_points.emplace_back(center_pt);
    iter_pt = new_iter_pt;
  }

  if (is_end_visited) { return true; }
  // check visit status
  bool is_most_sampling_pointsvisited =
      std::abs(visited_point_num - static_cast<int>(raw_points.size())) <= 5;
  XLOG << "is_most_sampling_pointsvisited " << is_most_sampling_pointsvisited;
  return is_most_sampling_pointsvisited;
}

bool KdeRegression::IsRectangleClean(
    const OBB &rect,
    const std::vector<WeightedPoint> &raw_points) {
  return std::all_of(
      raw_points.begin(), raw_points.end(),
      [rect](const WeightedPoint &pt) {
        return !rect.IsContainPoint(pt.first);
      });
}

double KdeRegression::CalcAngelAbs(const double &angle1, const double &angle2){
  float diff = std::abs(angle1 - angle2);
  if(diff > M_PI){
    diff = 2 * M_PI - diff;
  }
  return diff;
}
double KdeRegression::AngleAverage(const std::vector<double> &angles){
  std::vector<double> vec = angles;
  std::transform(vec.begin(), vec.end(), vec.begin(), [](double &x) { return std::fmod(x, 2*M_PI); });
  double avg = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
  return avg;
}


OBB::OBB(const DirectedPoint &st, float length, float width)
    : st_(st), length_(length), width_(width), half_length_(length * 0.5F),
      half_width_(width * 0.5F),
      max_axis_diff_(), x_max_(), x_min_(), y_max_(), y_min_(),
      rotation_matrix_(Eigen::Matrix2f::Identity()) {
  assert(length > 0);
  assert(width > 0);

  float angle = st[2];
  float cos{std::cos(angle)};
  float sin{std::sin(angle)};
  // 计算并储存中心点和旋转矩阵
  rotation_matrix_ << cos, sin, -sin, cos;
  center_ = Point(st.x(), st.y()) + 0.5 * length_ * Point(cos, sin);
  // 判断内点时允许的最大坐标差，用于加速计算 (0.707 == 根号2 * 0.5)
  max_axis_diff_ = 0.707F * std::max<float>(length_, width_);
  x_max_ = center_.x() + max_axis_diff_;
  x_min_ = center_.x() - max_axis_diff_;
  y_max_ = center_.y() + max_axis_diff_;
  y_min_ = center_.y() - max_axis_diff_;
}

inline bool OBB::IsContainPoint(
    const Point &pt) const {
  // 横/纵坐标差异大的提前return
  if (pt.y() > y_max_ || pt.y() < y_min_
    || pt.x() > x_max_ || pt.x() < x_min_) {
    return false;
  }

  constexpr float ESP{0.1F};
  Point delta = pt - center_;
  // 在新坐标系下的坐标
  Point new_pt;
  new_pt.noalias() =  rotation_matrix_ * delta;
  // ESP保证数值稳定性，排除临界点
  return (std::abs(new_pt.x()) < half_length_ - ESP) &&
      (std::abs(new_pt.y()) < half_width_);
}

const DirectedPoint &OBB::GetStartPoint() const {
  return st_;
}

AABB::AABB(const DirectedPoint &st, float forward_len, float side_len) {
  float angle = st[2];
  float cos{std::cos(angle)};
  float sin{std::sin(angle)};

  Point center{st.x() + 0.5F * forward_len * cos,
               st.y() + 0.5F * forward_len * sin};

  x_max_ = center.x() + side_len;
  x_min_ = center.x() - side_len;
  y_max_ = center.y() + side_len;
  y_min_ = center.y() - side_len;
}

inline bool AABB::IsContainPoint(const Point &pt) const {
  // 横/纵坐标差异大的提前return
  return pt.y() <= y_max_ && pt.y() >= y_min_
      && pt.x() <= x_max_ && pt.x() >= x_min_;
}

const DirectedPoint &AABB::GetStartPoint() const {
  return st_;
}



}  // namespace geometry
