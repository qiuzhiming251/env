#include "virtual_egoroad_processor.h"
#include "base/params_manager/params_manager.h"
#include "base/sensor_data_manager.h"
#include "lib/common/fitter/bezier_points.h"
#include "lib/common/fitter/two_segment_bezier_interpolator.h"
#include "lib/common/log_custom.h"
#include "lib/common/math/math.h"
#include "lib/common/utility.h"
#include "lib/message/internal_message.h"
#include "utility.h"
#define SAVE_U_EDAGE_PT 0

///
namespace cem {
namespace fusion {

using byd::common::math::LineSegment2d;
VirtualEgoRoadProcessor::VirtualEgoRoadProcessor() {}
void VirtualEgoRoadProcessor::Init(const std::shared_ptr<CrossDataManager> &data_manager) {
  data_manager_ = data_manager;
  cross_road_selector_ = std::make_shared<CrossRoadSelector>();
  cross_road_selector_->Init(data_manager);
}

void VirtualEgoRoadProcessor::ReSet() {
  is_first_generate_   = true;
  is_generate_success_ = false;
  ego_lane_            = {};
  dr_ego_lane_         = {};
  his_turn_type_ =  BevAction::STRAIGHT;
  // ego_left_lanemarker_ = {};
  // ego_right_lanemarker_ = {};
  virtual_lane_                  = {};
  virtual_left_lanemarker_       = {};
  virtual_right_lanemarker_      = {};
  has_virtual_lane_              = false;
  is_straight_exceed_heading_    = false;
  straight_exceed_heading_count_ = 0;
  left_right_move_dis_ = 0.0;
}

bool VirtualEgoRoadProcessor::GetVirtualLanes() {
  bool is_success{false};
  bool is_ego_lane_change = false;
  if (data_manager_->has_ego_lane() && data_manager_->EgoLane() != nullptr && ego_lane_.id != 0 && !data_manager_->GetIsInCrossroad()) {
    is_ego_lane_change = data_manager_->EgoLane()->id != ego_lane_.id;
  }
  if ((his_turn_type_ != data_manager_->turn_type() || is_ego_lane_change) && is_generate_success_) {
    is_generate_success_ = false;
  }
  switch (data_manager_->turn_type()) {
    case BevAction::STRAIGHT:
      is_success = GenerateStraightLaneNew();
      break;
    case BevAction::LEFT_TURN:
      is_success = GenerateLeftTurnLaneNew();
      break;
    case BevAction::RIGHT_TURN:
      is_success = GenerateRightTurnLaneNew();
      break;
    case BevAction::U_TURN:
      is_success = GenerateAroundTurnLane();
      break;
    default:
      FLOG_CROSS << "error ";
      break;
  }
  his_turn_type_ = data_manager_->turn_type();
  is_first_generate_ = !is_success;
  return is_success;
}

void VirtualEgoRoadProcessor::UpdateVirtualLanes() {
  XLOG << " PredictionAction UpdateVirtualLanes -> GetVirtualLanes" ;
  GetVirtualLanes();
  is_first_generate_ = false;
}

bool VirtualEgoRoadProcessor::GenerateStraightLaneNew() {
  XLOG << " GenerateStraightLane data_manager_->has_ego_lane(): " <<data_manager_->has_ego_lane();
  // 1.没成功前没有ego_lane返回false 2.成功后，在路口前没有了egolane，返回false使用历史值
  if (!data_manager_->has_ego_lane() && !is_generate_success_) {
    return false;
  }

  // 2. 拟合ego_lane
  Curve curve;
  fitter_egolane_success_ = FitterEgelane(curve);
  XLOG << " fitter_egolane_success_: " << fitter_egolane_success_;
  // 3. 生成mainlane
  GenerateMainLane(curve);

  // 4.路口前拟合失败返回,使用历史值
  if (!fitter_egolane_success_ && !data_manager_->GetIsInCrossroad()) {
    return false;
  }

  // 5. 生成虚拟lane，并转成dr坐标系，后续可以一直使用，中间只需要更新MainLane用于连接使用
  std::vector<Point2DF> virtual_line_points;
  if (!GenerateVirtualStraightLinePoints(virtual_line_points, curve)) {
    XLOG << " can not generate  virtual_line_points!!!";
    return false;
  }
  is_generate_success_ = true;

  // 6. 路口前生成了virtual_line_points就更新ego_lane_和dr_ego_lane_
  if (!data_manager_->GetIsInCrossroad()) {
    ego_lane_ = *data_manager_->EgoLane();
    // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
    dr_ego_lane_ = ego_lane_;
    for (auto &point : dr_ego_lane_.line_points) {
      TransformPoint(&point, data_manager_->Twb());
      // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
    }
  }

  // 7. virtual_line_points中心线型点填充到成员变量   
  GenerateVirtualLaneAndLaneMarker(virtual_line_points);
  virtual_lane_.bev_turn_type = BevAction::STRAIGHT;
  return true;
}

void VirtualEgoRoadProcessor::GenerateMainLane(Curve &ego_curve) {
  const auto *ego_lane = data_manager_->EgoLane();
  if (fitter_egolane_success_) {
    NewMainLane(ego_curve.c0, ego_curve.c1, ego_lane->line_points, ego_lane->width / 2, ego_lane->id);
  } else {
    UpdateVirtualMainLane();
  }
}

bool VirtualEgoRoadProcessor::FitterEgelane(Curve &ego_curve) {
  FLOG_CROSS << " FitterEgelane";
  // 车辆过了停止线或者未过之前没有了egolane都不进行拟合
  if (data_manager_->GetIsInCrossroad() || (!data_manager_->GetIsInCrossroad() && !data_manager_->has_ego_lane())) {
    return false;
  }
  const auto &points = data_manager_->EgoLane()->line_points;
  auto        size   = points.size();
  if (size < 2) {
    return false;
  }
  // 选取egolane的最后20m进行直线拟合求heading
  double                end_x = data_manager_->EgoLane()->line_points.back().x;
  std::vector<Point2DF> fit_points;
  // std::vector<Point2DF> fit_points(
  //     ((size > 25) ? data_manager_->EgoLane()->line_points.begin() + (size - 25) : data_manager_->EgoLane()->line_points.begin()),
  //     data_manager_->EgoLane()->line_points.end());
  for (int i = 1; i < size; ++i) {
    if (end_x - points[i].x < 20.0) {
      // FLOG_CROSS << " fit x: " << points[i].x << " ,y: " << points[i].y;
      fit_points.emplace_back(points[i]);
    }
  }
  return FitterLeastSquareMethod(fit_points, 1, ego_curve);
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve) {
  bool is_success{false};
  auto  virtual_line_source = data_manager_->GetVirtualLineSource();
  // 判断是否使用经验轨迹中心线
  if (kStraightUsingExpTrajPoints) {
    is_success = GenerateVirtualStraightLinePointsByExpTraj(virtual_line_points, curve);
    XLOG << "is_success " << is_success << "virtual_line_source " << int (virtual_line_source);
    // 已经成功且进入了路口模式，路口前失败了也不再继续使用sd
    if (!is_success && data_manager_->GetCrossState() == CrossState::PREDICTION && virtual_line_source ==  VirtualLineSource::EXP) {
      XLOG << " data_manager_->GetCrossState() " << int (data_manager_->GetCrossState());
      return false;
    }
    if (is_success) {
      virtual_line_source = VirtualLineSource::EXP;
      XLOG << " virtual_line_source: " << EnumToString(virtual_line_source);
      data_manager_->SetVirtualLineSource(virtual_line_source);
      return true;
    }
  }
  // 判断是否使用LD中心线
  if (kStraightUsingLdPoints) {
    is_success = GenerateVirtualStraightLinePointsByLd(virtual_line_points, curve);
    // 已经成功且进入了路口模式，路口前失败了也不再继续使用经验轨迹或sd
    if (!is_success && data_manager_->GetCrossState() == CrossState::PREDICTION && virtual_line_source ==  VirtualLineSource::LD) {
      return false;
    }
    if (is_success) {
      virtual_line_source = VirtualLineSource::LD;
      XLOG << " virtual_line_source: " << EnumToString(virtual_line_source);
      data_manager_->SetVirtualLineSource(virtual_line_source);
      return true;
    }
  }
  // 新增只使用ld和经验轨迹
  if (virtual_using_only_ld_traj) {
    XLOG << "Straight virtual_using_only_ld_traj";
    return false;
  }

  // 最后选择heading
  is_success = GenerateVirtualStraightLinePointsByHeading(virtual_line_points, curve);
  if (is_success) {
    virtual_line_source = VirtualLineSource::PRE;
    XLOG << " virtual_line_source: " << EnumToString(virtual_line_source);
    data_manager_->SetVirtualLineSource(virtual_line_source);
    return true;
  }
  return false;

  // switch (DecideStraightVirtualLineSource()) {
  //   case 0:
  //     is_success = GenerateVirtualStraightLinePointsByHeading(virtual_line_points, curve);
  //     break;
  //   case 1:
  //     is_success = GenerateVirtualStraightLinePointsByObs(virtual_line_points, curve);
  //     break;
  //   case 2:
  //     is_success = GenerateVirtualStraightLinePointsBySd(virtual_line_points, curve);
  //     break;
  //   case 3:
  //     is_success = GenerateVirtualStraightLinePointsByLd(virtual_line_points, curve);
  //     break;
  //   case 4:
  //     is_success = GenerateVirtualStraightLinePointsByExpTraj(virtual_line_points, curve);
  //     break;
  //   default:
  //     FLOG_CROSS << "error ";
  //     break;
  // }
  // return is_success;

}

// int VirtualEgoRoadProcessor::DecideStraightVirtualLineSource() {
//   auto virtual_line_source = data_manager_->GetVirtualLineSource();
//   if (kStraightUsingExpTrajPoints) {
//     virtual_line_source = 4;
//   } else if (kStraightUsingLdPoints) {
//     virtual_line_source = 3;
//   } else {
//     virtual_line_source = 0;
//   }
//   data_manager_->SetVirtualLineSource(virtual_line_source);
//   FLOG_CROSS << " virtual_line_source: " << virtual_line_source;
//   return virtual_line_source;
// }

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return GenerateVirtualLinePointsByLd(virtual_line_points, ego_curve);
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return GenerateVirtualLinePointsByExpTraj(virtual_line_points, ego_curve);
}

bool VirtualEgoRoadProcessor::GenerateVirtualLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  // 路口后不再更新
  if (data_manager_->GetIsInCrossroad()) {
    return false;
  }
  auto ld_ego_lane_id = data_manager_->GetLDEgoLaneId();
  if (data_manager_->routing_map() == nullptr) {
    return false;
  }
  std::vector<Point2DF> ld_lane_points;
  const auto *          lane = data_manager_->FindRoutingMapLaneById(ld_ego_lane_id);
  if (lane == nullptr) {
    FLOG_CROSS << " has no ld_ego_lane_id";
    return false;
  }
  // 前继是虚的返回
  if (lane->previous_lane_ids.size() > 0) {
    for (auto pre_id : lane->previous_lane_ids) {
      const auto *pre_lane = data_manager_->FindRoutingMapLaneById(pre_id);
      if (pre_lane == nullptr) {
        continue;
      }
      if (pre_lane->is_virtual) {
        FLOG_CROSS << " pre_lane->is_virtual";
        return false;
      }
    }
  }
  if (!cross_road_selector_->Process(lane, ld_lane_points)) {
    FLOG_CROSS << " cross_road_selector faild!!!";
    return false;
  }

  {
  // double   lane_length{0.0};
  // int      index   = 0;
  // uint64_t lane_id = ld_ego_lane_id;
  // bool has_is_virtual = false;
  // while (lane_length < 100 && index < 100) {
  //   index++;
  //   const auto *lane = FindLaneById(lane_id);
  //   if (lane == nullptr) {
  //     break;
  //   }
  //   // FLOG_CROSS << " ld lane id: " << lane_id << " ,turn_type: " << int(lane->turn_type)
  //   //            << " ,ego turn_type: " << int(data_manager_->turn_type());
  //   for (const auto &point : lane->points) {
  //     auto ego_point = TransformPoint(point, data_manager_->Tbw());
  //     ld_lane_points.emplace_back(ego_point.x, ego_point.y);
  //     if (ld_lane_points.size() > 1) {
  //       lane_length += std::hypot(ld_lane_points.back().x - ld_lane_points[ld_lane_points.size() - 2].x,
  //                                 ld_lane_points.back().y - ld_lane_points[ld_lane_points.size() - 2].y);
  //     }
  //   }
  //   if (lane->next_lane_ids.empty()) {
  //     break;
  //   }
  //   // 再虚拟线后再增加30m中心线
  //   constexpr double max_length = 30.0;
  //   if (lane->is_virtual) {
  //     has_is_virtual = true;
  //     if (lane->next_lane_ids.empty()) {
  //       break;
  //     }
  //     const auto *next_lane = FindLaneById(lane->next_lane_ids.front());
  //     if (next_lane == nullptr || next_lane->points.empty()) {
  //       break;
  //     }
  //     double length = 0;
  //     for (const auto &point : next_lane->points) {
  //       auto ego_point = TransformPoint(point, data_manager_->Tbw());
  //       ld_lane_points.emplace_back(ego_point.x, ego_point.y);
  //       if (ld_lane_points.size() > 1) {
  //         length += std::hypot(ld_lane_points.back().x - ld_lane_points[ld_lane_points.size() - 2].x,
  //                              ld_lane_points.back().y - ld_lane_points[ld_lane_points.size() - 2].y);
  //       }
  //       if (length > max_length) {
  //         break;
  //       }
  //     }
  //     break;
  //   }
  //   // 
  //   auto next_lane_id_size = lane->next_lane_ids.size();
  //   std::vector<uint64_t> fit_next_lane_ids;
  //   double min_dis = DBL_MAX;
  //   uint64_t min_dis_lane_id = 0;
  //   for (auto id : lane->next_lane_ids) {
  //     const auto *next_lane = FindLaneById(id);
  //     if (next_lane == nullptr) {
  //       continue;
  //     }
  //     if(!next_lane->is_virtual && next_lane_id_size > 1){
  //       FLOG_CROSS << "can not find crossroad lane!";
  //       return false;
  //     }
  //     // FLOG_CROSS << " ld next lane id: " << id << " ,turn_type: " << int(next_lane->turn_type);
  //     // 路口内的车道只选择对应转向的车道中心线
  //     if (next_lane->is_virtual && !HasSameTurnType(next_lane->turn_type, data_manager_->turn_type())) {
  //       continue;
  //     }
  //     if(next_lane->points.empty()){
  //       continue;
  //     }
  //     const auto &point = TransformPoint(next_lane->points.back(), data_manager_->Tbw());
  //     double      dis   = data_manager_->turn_type() == BevAction::STRAIGHT ? std::abs(point.y) : point.x * point.x + point.y * point.y;
  //     if(dis < min_dis){
  //       min_dis = dis;
  //       min_dis_lane_id = id;
  //     }
  //     fit_next_lane_ids.emplace_back(id);
  //   }
  //   if (fit_next_lane_ids.empty()) {
  //     return false;
  //     // lane_id = lane->next_lane_ids.front();
  //   } else if (fit_next_lane_ids.size() == 1) {
  //     lane_id = fit_next_lane_ids.front();
  //   } else {
  //     lane_id = min_dis_lane_id;
  //   }
  //   // lane_id = fit_next_lane_ids.empty() ? lane->next_lane_ids.front() : fit_next_lane_ids.front();
  // }
  // if (!has_is_virtual) {
  //   FLOG_CROSS << " next lane has no cross!";
  //   return false;
  // }
}

  const auto &start_point = data_manager_->EgoLane()->line_points.back();
  FLOG_CROSS << "start_point x: " << start_point.x << " ,y: " << start_point.y;

  double y_err{0.0};
  int index = 0;
  double ld_heading{0.0};
  for (int i = 1; i < ld_lane_points.size(); i++) {
    // FLOG_CROSS << " ld_lane_points[" << i << "]x: " << ld_lane_points[i].x;
    if (ld_lane_points[i].x > start_point.x) {
      double y = ((ld_lane_points[i].y - ld_lane_points[i - 1].y) / (ld_lane_points[i].x - ld_lane_points[i - 1].x)) *
                     (start_point.x - ld_lane_points[i - 1].x) +
                 ld_lane_points[i - 1].y;
      ld_heading = Vec2d(ld_lane_points[i].x - ld_lane_points[i - 1].x, ld_lane_points[i].y - ld_lane_points[i - 1].y).Angle();
      y_err      = start_point.y - y;
      index      = i;
      break;
    }
  }
  if(index == 0){
    return false;
  }
  // 如果index位置的点距离start超过10m就在中间6m处再插值补一个点
  double dis = std::hypot(ld_lane_points[index].x - start_point.x, ld_lane_points[index].y - start_point.y);
  if (dis > 10.0) {
    ld_lane_points[index - 1].x = start_point.x + (6.0 / dis) * (ld_lane_points[index].x - start_point.x);
    ld_lane_points[index - 1].y = start_point.y + (6.0 / dis) * (ld_lane_points[index].y - start_point.y);
    index--;
  }
  FLOG_CROSS << " ld heading: " << ld_heading << " ,bev heading: " << std::atan(ego_curve.c1) << " ,index: " << index << " ,y_err: " << y_err;
  // 使用旋转和平移
  // double delta_x{0.0};
  // double delta_y{0.0};
  // double delta_heading = ld_heading - std::atan(ego_curve.c1);
  // for (int i = index; i < ld_lane_points.size(); i++) {
  //   delta_x  = ld_lane_points[i].x - start_point.x;
  //   delta_y  = ld_lane_points[i].y - start_point.y;
  //   double x = start_point.x + delta_x * std::cos(delta_heading) - delta_y * std::sin(delta_heading);
  //   double y = start_point.y + delta_x * std::sin(delta_heading) + delta_y * std::cos(delta_heading);
  //   virtual_line_points.emplace_back(x, y);
  //   FLOG_CROSS << "virtual_line_points x: " << x << " ,y: " << y;
  // }

  // 只使用横向平移偏差
  for (int i = index; i < ld_lane_points.size(); i++) {
    virtual_line_points.emplace_back(ld_lane_points[i].x, ld_lane_points[i].y + y_err);
    // FLOG_CROSS << "virtual_line_points x: " << ld_lane_points[i].x << " ,y: " << ld_lane_points[i].y + y_err;
  }

  return true;
}

bool VirtualEgoRoadProcessor::GenerateVirtualLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  // 路口后不再更新
  if (data_manager_->GetIsInCrossroad() || !data_manager_->has_ego_lane()) {
    FLOG_CROSS << " IsInCrossroad!!! ";
    return false;
  }
  auto ld_ego_lane_id = data_manager_->GetLDEgoLaneId();
  if (data_manager_->routing_map() == nullptr) {
    return false;
  }
  // 寻找120m内的lane有没有经验轨迹
  constexpr double min_length = 120.0;

  double   length{0.0};
  int      index{0};
  uint64_t ld_lane_id = ld_ego_lane_id;
  bool     has_exp_traj{false};
  while (length < min_length && index < 100) {
    const auto *lane = data_manager_->FindRoutingMapLaneById(ld_lane_id);
    if (lane == nullptr || lane->is_virtual || lane->points.size() < 2) {
      break;
    }
    if (!lane->exp_trajectory_ids.empty()) {
      has_exp_traj = true;
      break;
    }
    if (lane->next_lane_ids.size() != 1) {
      break;
    }
    ld_lane_id         = lane->next_lane_ids.front();
    double step_length = (TransformPoint(lane->points.front(), data_manager_->Tbw())).x < 0.0
                             ? (TransformPoint(lane->points.back(), data_manager_->Tbw())).x
                             : lane->length;
    length += step_length;
    index++;
  }

  if (!has_exp_traj) {
    FLOG_CROSS << " has not find exp_traj!!! ";
    return false;
  }

  std::vector<cem::message::env_model::Trajectory> exp_trajs;
  const auto *lane = data_manager_->FindRoutingMapLaneById(ld_lane_id);
  if (lane == nullptr) {
    FLOG_CROSS << " lane == nullptr!!! ";
    return false;
  }
  std::vector<uint64_t> end_lane_ids;
  for (auto exp_traj_id : lane->exp_trajectory_ids) {
    for (const auto &exp_traj : data_manager_->routing_map()->exp_trajectories) {
      if (exp_traj.id == exp_traj_id && data_manager_->SdHasEepTrajEndLaneid(exp_traj.end_lane_id) && exp_traj.points.size() > 1) {
        exp_trajs.emplace_back(exp_traj);
        end_lane_ids.emplace_back(exp_traj.end_lane_id);
        break;
      }
    }
  }
  if (exp_trajs.empty()) {
    FLOG_CROSS << " exp_traj_ids.empty!!! ";
    return false;
  }
  // for (const auto exp_traj : exp_trajs) {
  //   FLOG_CROSS << " exp_traj_ids_id: " << exp_traj.id << " ,end_x: " << exp_traj.points.back().x << " ,y: " << exp_traj.points.back().y;
  // }

  index          = 0;
  double min_dis = DBL_MAX;
  for (int i = 0; i < exp_trajs.size(); i++) {
    const auto &point = TransformPoint(exp_trajs[i].points.back(), data_manager_->Tbw());
    double      dis   = std::pow(point.x, 2) + std::pow(point.y, 2);
    if (dis < min_dis) {
      min_dis = dis;
      index   = i;
    }
  }

  if (exp_trajs[index].points.size() < 2) {
    FLOG_CROSS << " exp_line_points.size() < 2";
    return false;
  }
  FLOG_CROSS << "exp_traj_id_id: " << exp_trajs[index].id << " ,end_x: " << exp_trajs[index].points.back().x
             << " ,y: " << exp_trajs[index].points.back().y;
  std::vector<Point> exp_line_points;
  exp_line_points.reserve(exp_trajs[index].points.size());
  for(const auto& point:exp_trajs[index].points){
    exp_line_points.emplace_back(TransformPoint(point, data_manager_->Tbw()));
  }
  // 将经验轨迹后的ld 点也拼接进去
  const auto *end_lane = data_manager_->FindRoutingMapLaneById(end_lane_ids[index]);
  if (end_lane != nullptr && end_lane->points.size() > 1) {
    FLOG_CROSS << " ld_end_lane_id: " << end_lane->id << " ,start_x: " << end_lane->points.front().x
               << " ,y: " << end_lane->points.front().y;
    exp_line_points.emplace_back(TransformPoint(end_lane->points.front(), data_manager_->Tbw()));
    double total_length = 0.0;
    for (int i = 1; i < end_lane->points.size(); i++) {
      exp_line_points.emplace_back(TransformPoint(end_lane->points[i], data_manager_->Tbw()));
      total_length += std::hypot((end_lane->points[i].x - end_lane->points[i - 1].x), (end_lane->points[i].y - end_lane->points[i - 1].y));
      if (total_length > 20.0) {
        break;
      }
    }
  }

  const auto &start_point = data_manager_->EgoLane()->line_points.back();
  FLOG_CROSS << "start_point x: " << start_point.x << " ,y: " << start_point.y;
  double y_err{0.0};
  index = 0;
  double ld_heading{0.0};
  for (int i = 1; i < exp_line_points.size(); i++) {
    auto &last_point = exp_line_points[i - 1];
    auto &now_point  = exp_line_points[i];
    if (now_point.x > start_point.x) {
      double y   = ((now_point.y - last_point.y) / (now_point.x - last_point.x)) * (start_point.x - last_point.x) + last_point.y;
      ld_heading = Vec2d(now_point.x - last_point.x, now_point.y - last_point.y).Angle();
      y_err      = start_point.y - y;
      index      = i;
      break;
    }
  }
  FLOG_CROSS << " exp heading: " << ld_heading << " ,bev heading: " << std::atan(ego_curve.c1) << " ,end_x: " << exp_line_points.back().x
             << " ,y: " << exp_line_points.back().y;
  // 经验轨迹最后面延长20m
  // double dx           = exp_line_points[exp_line_points.size() - 1].x - exp_line_points[exp_line_points.size() - 2].x;
  // double dy           = exp_line_points[exp_line_points.size() - 1].y - exp_line_points[exp_line_points.size() - 2].y;
  // double delta_length = std::hypot(dx, dy);
  // if (delta_length > 0.1 && delta_length < 0.5) {
  //   double k     = std::abs(1 / delta_length);
  //   dx           = dx * k;
  //   dy           = dy * k;
  //   delta_length = std::hypot(dx, dy);
  // }
  // length = 0.0;
  // int break_index{0};
  // while (length < 20.0 && break_index < 20) {
  //   Point point;
  //   point.x = exp_line_points.back().x + dx;
  //   point.y = exp_line_points.back().y + dy;
  //   exp_line_points.emplace_back(point);
  //   length += delta_length;
  //   break_index++;
  // }
  // FLOG_CROSS << " exp_line_points dx: " << dx << " ,dy: " << dy << " ,end_x: " << exp_line_points.back().x
  //            << " ,y: " << exp_line_points.back().y;
  // 只使用横向平移偏差
  for (int i = index; i < exp_line_points.size(); i++) {
    virtual_line_points.emplace_back(exp_line_points[i].x, exp_line_points[i].y + y_err);
    // FLOG_CROSS << "virtual_line_points x: " << exp_line_points[i].x << " ,y: " << exp_line_points[i].y + y_err;
  }

  return true;
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePointsBySd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  FLOG_CROSS << "GenerateVirtualStraightLinePointsBySd";
  auto &main_lane  = data_manager_->GetMainLane();
  auto &sd_section = data_manager_->GetSdSections();
  auto  top_lane   = GenerateTopLanePoints(sd_section.end_point, sd_section.top_angle(), ego_lane_.width / 2);
  // 连接中心线
  bool is_lane_valid{false};
  int  bot_lane_index = -1;
  int  top_index      = -1;

  std::vector<Vec2d> control_points = {};

  BevLaneInfo virtual_lane;
  for (const auto &point : main_lane.points) {
    FLOG_CROSS << " input main_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  for (const auto &point : top_lane.points) {
    FLOG_CROSS << " input top_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  if (InsertBezierPoints(main_lane.points, top_lane.points, control_points, bot_lane_index, top_index)) {
    FLOG_CROSS << " bot_lane_index: " << bot_lane_index << " ,top_index: " << top_index
               << ", control_points.size: " << control_points.size();
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
      }
      control_points.emplace_back(top_lane.points.back());
      virtual_line_points.reserve(control_points.size());
      for (const auto &point : control_points) {
        virtual_line_points.emplace_back(point.x(), point.y());
      }
      return true;
    }
  }
  return false;
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePointsByObs(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return false;
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightLinePointsByHeading(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  XLOG << "GenerateVirtualStraightLinePointsByHeading!!! " << "fitter_egolane_success_ " << fitter_egolane_success_ << " is_generate_success_ " << is_generate_success_;
  // 第一次拟合失败返回false
  if (!is_generate_success_ && !fitter_egolane_success_) {
    return false;
  }
  const auto &opening = data_manager_->GetOpening();
  XLOG << "opening.has_heading: " << opening.has_heading << " ,opening.dis: " << opening.dis
             << " ,is_straight_exceed_heading: " << is_straight_exceed_heading_;
  if (is_generate_success_ && (opening.dis < 20.0 || !opening.has_heading)) {//生成成功过 且距离小 或者没有出口方向
    XLOG << "opening.dis return! ";
    return false;
  }
  std::vector<Point2DF> line_points;
  Point2DF              start_point;
  Curve                 curve;
  // 1. 路口前拟合成功使用拟合heading 2. 路口后需要更新时，使用已生成的轨迹生成heading
  if (!data_manager_->GetIsInCrossroad() && fitter_egolane_success_) {
    XLOG << " IN CORSS fitter_egolane_success_ " << fitter_egolane_success_;
    start_point = data_manager_->EgoLane()->line_points.back();
    curve       = ego_curve;
  } else if (data_manager_->GetIsInCrossroad() && KUpdateStraightVirtualIncrossroad) {//KUpdateStraightVirtualIncrossroad 关闭
    XLOG << "is_in_cross_road update virtual lane! ";
    line_points.reserve(virtual_lane_.line_points.size());
    int count{0};
    for (const auto &point : virtual_lane_.line_points) {
      line_points.emplace_back(TransformPoint(point, data_manager_->Tbw()));
      if (line_points.back().x > 0 && count > 0) {
        break;
      }
      count++;
    }
    if (line_points.size() < 2) {
      XLOG << "line_points.empty!!!";
      return false;
    }
    start_point = line_points.back();
    curve.c1    = std::tan(Vec2d(line_points[line_points.size() - 1].x - line_points[line_points.size() - 2].x,
                              line_points[line_points.size() - 1].y - line_points[line_points.size() - 2].y)
                            .Angle());
    curve.c0    = start_point.y - start_point.x * curve.c1;
  } else {
    return false;
  }
  // 路口前拟合成功
  virtual_line_points = line_points;//成员变量
  XLOG << "line_points.curve.c0: " << curve.c0 << " ,c1: " << curve.c1 << "heading abs" << std::abs(opening.heading - std::atan(curve.c1)) << " line_points size " << line_points.size();

  // 使用曲率和heading生成line
  double kPredictMaxDis = 90.0;

  if (opening.has_heading && opening.dis > start_point.x) {
    kPredictMaxDis = std::max(opening.dis - start_point.x + 26.0, 50.0);
  } else if (data_manager_->GetSdSections().has_intersection_zone) {
    kPredictMaxDis = std::max(data_manager_->GetSdSections().end_point.x() - start_point.x + 26.0, 50.0);
  }

  double x{0.0};
  double y{0.0};
  if (std::abs(opening.heading - std::atan(curve.c1)) > 0.05) {
    straight_exceed_heading_count_++;
  }
  if (!is_straight_exceed_heading_ && straight_exceed_heading_count_ > 5) {
    is_straight_exceed_heading_ = true;
  }
  XLOG << "opening.has_heading " << opening.has_heading << "  opening.dis " << opening.dis << " is_straight_exceed_heading_ " << is_straight_exceed_heading_; 
  if (opening.has_heading && opening.dis > 10.0 && is_straight_exceed_heading_) {
    double top_lane_dis = opening.dis - start_point.x;
    double ego_heading  = std::atan(curve.c1);
    double heading_diff = opening.heading - ego_heading;
    double radius       = top_lane_dis / std::sin(std::abs(heading_diff));
    double delta_length = 1.0;
    double delta_angle  = delta_length / radius;
    int    phi_num      = std::floor(std::abs(heading_diff) / delta_angle);
    double dx{0.0};
    double dy{0.0};
    double total_length{0.0};
    double phi{0.0};
    XLOG << " radius: " << radius << " ,phi_num: " << phi_num << " ,opening.dis: " << opening.dis
               << " ,start_point.x: " << start_point.x;
    if (radius <= 20.0 && is_generate_success_) {
      return false;
    }

    if (radius <= 20.0 || radius >= 3000) {
      virtual_line_points.reserve(91);
      for (int i = 1; i < 91; i++) {
        x = start_point.x + i;
        y = curve.c0 + curve.c1 * x;
        XLOG << "virtual_line_points x: " << x << " ,y: " << y;
        virtual_line_points.emplace_back(Point2DF(x, y));
      }
      if(virtual_line_points.size() < 2){
        return false;
      }
      return true;
    }

    if (radius < 3000) {
      for (int i = 0; i < phi_num + 1; i++) {
        total_length += delta_length;
        phi = i * delta_angle;
        dx  = radius * std::sin(phi);
        dy  = std::copysign(radius * (1 - std::cos(phi)), heading_diff);
        x   = start_point.x + dx * std::cos(ego_heading) - dy * std::sin(ego_heading);
        y   = start_point.y + dx * std::sin(ego_heading) + dy * std::cos(ego_heading);
        virtual_line_points.emplace_back(x, y);
      }
      if (virtual_line_points.size() < 2) {
        FLOG_CROSS << "virtual_line_points.empty!!!";
        return false;
      }
      int      index{0};
      Point2DF pre_point = virtual_line_points.at(virtual_line_points.size() - 2);
      Point2DF now_point = virtual_line_points.at(virtual_line_points.size() - 1);
      double   delta_x   = now_point.x - pre_point.x;
      double   delta_y   = now_point.y - pre_point.y;
      while (total_length < kPredictMaxDis && index < 200) {
        index++;
        total_length += delta_length;
        x = virtual_line_points.back().x + delta_x;
        y = virtual_line_points.back().y + delta_y;
        virtual_line_points.emplace_back(x, y);
      }
    }
    // else {
    //   int index{0};
    //   virtual_line_points.emplace_back(start_point);
    //   while (total_length < kPredictMaxDis && index < 200) {
    //     index++;
    //     total_length += delta_length;
    //     x = virtual_line_points.back().x + delta_length;
    //     y = virtual_line_points.back().y + delta_length * std::tan(heading_diff);
    //     virtual_line_points.emplace_back(x, y);
    //   }
    // }
  } else {
    virtual_line_points.reserve(91);
    for (int i = 1; i < kPredictMaxDis; i++) {
      x = start_point.x + i;
      y = curve.c0 + curve.c1 * x;
      virtual_line_points.emplace_back(Point2DF(x, y));
    }
    if(virtual_line_points.size() < 2){
      return false;
    }
  }
  return true;
}

bool VirtualEgoRoadProcessor::GenerateStraightLane() {
  FLOG_CROSS << " GenerateStraightLane is_first_generate: " << is_first_generate_;
  if (!data_manager_->has_ego_lane() && is_first_generate_) {
    return false;
  }
  // 非第一次,没有ego_lane或者ego_lane在车后的情况下，直接使用历史值
  if ((!data_manager_->has_ego_lane() || (data_manager_->has_ego_lane() && data_manager_->EgoLane()->line_points.size() > 2 &&
                                          data_manager_->EgoLane()->line_points.back().x < 1.0)) &&
      !is_first_generate_) {
    FLOG_CROSS << " UpdateStraightMainLane no ego lane or x < -10!";
    UpdateStraightMainLane();
    return true;
  }

  // 在ego_lane后方生成40m的VirtualLane
  Curve                 curve;
  auto                  size = data_manager_->EgoLane()->line_points.size();
  std::vector<Point2DF> fit_points(
      ((size > 25) ? data_manager_->EgoLane()->line_points.begin() + (size - 25) : data_manager_->EgoLane()->line_points.begin()),
      data_manager_->EgoLane()->line_points.end());
  bool valid = FitterLeastSquareMethod(fit_points, 1, curve);

  // 第一次拟合失败返回false，后面拟合失败使用历史的ego_lane_
  if (!valid && is_first_generate_) {
    FLOG_CROSS << " is_first_generate_ and valid";
    return false;
  } else if (!valid) {
    FLOG_CROSS << " UpdateStraightMainLane";
    UpdateStraightMainLane();
    return true;
  }

  // 生成40m的虚拟lane，并转成dr坐标系，后续可以一直使用，中间只需要更新MainLane用于连接使用

  std::vector<Point2DF> virtual_line_points;
  if (!GenerateVirtualStraightPoints(virtual_line_points, fit_points.back(), curve)) {
    return false;
  }
  // 拟合成功后才更新ego_lane_
  ego_lane_ = *data_manager_->EgoLane();
  for (const auto &point : fit_points) {
    FLOG_CROSS << "ego fit point x: " << point.x << " ,y: " << point.y;
  }
  // virtual_line_points.reserve(61);
  // for (int i = 1; i < 41; i++) {
  //   double x = fit_points.back().x + i;
  //   double y = curve.c0 + curve.c1 * x;
  //   // FLOG_CROSS << "virtual_line_points x: " << x << " ,y: " << y;
  //   virtual_line_points.emplace_back(Point2DF(x, y));
  // }
  GenerateVirtualLaneAndLaneMarker(virtual_line_points);
  // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
  dr_ego_lane_ = ego_lane_;
  for (auto &point : dr_ego_lane_.line_points) {
    TransformPoint(&point, data_manager_->Twb());
    // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
  }
  auto &main_lane = data_manager_->GetMainLane();
  main_lane.Reset();
  double half_width = ego_lane_.width / 2;
  // 第一次生成main_lane
  main_lane.lane_id     = dr_ego_lane_.id;
  main_lane.navi_action = data_manager_->turn_type();
  main_lane.angle       = std::atan(curve.c1);
  main_lane.top_points.set_x(ego_lane_.line_points.back().x);
  main_lane.top_points.set_y(ego_lane_.line_points.back().y);
  main_lane.valid      = true;
  main_lane.is_virtual = false;
  main_lane.points.emplace_back(main_lane.top_points);
  // 使用系数求取为了防止原始车道中心线末端两个点是斜的
  double x = ego_lane_.line_points.back().x + 10.0;
  double y = curve.c0 + curve.c1 * x;
  Vec2d  next_point(x, y);
  main_lane.points.emplace_back(next_point);
  // 求左侧main lane
  Vec2d axis = next_point - main_lane.top_points;
  axis.Normalize();
  axis.SelfRotate(pi / 2);
  main_lane.left_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, half_width);
  main_lane.left_top_points = main_lane.left_points.front();
  // 求右侧main lane
  main_lane.right_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, -half_width);
  main_lane.right_top_points = main_lane.right_points.front();
  is_first_generate_         = false;
  FLOG_CROSS << " first/update generate main lane angle: " << main_lane.angle << " ,1x: " << main_lane.top_points.x()
             << " ,y: " << main_lane.top_points.y() << " ,2x: " << next_point.x() << " ,y: " << next_point.y();
  FLOG_CROSS << " UpdateVirtualLanes end";
  return true;
}

void VirtualEgoRoadProcessor::UpdateStraightMainLane() {
  FLOG_CROSS << " UpdateStraightMainLane";
  // 对历史dr ego lane进行坐标转换
  auto                  dr_points_size = dr_ego_lane_.line_points.size();
  std::vector<Point2DF> fit_dr_ego_points;
  fit_dr_ego_points.reserve(dr_points_size);
  for (auto it = dr_ego_lane_.line_points.begin(); it != dr_ego_lane_.line_points.end(); it++) {
    fit_dr_ego_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }
  FLOG_CROSS << " dr_points_size: " << dr_points_size;
  // 使用历史值egolane拟合更新, 或者使用虚拟点更新
  if (fit_dr_ego_points.back().x > 1.0) {
    auto start_it =
        dr_points_size > kFitPointsSize ? fit_dr_ego_points.begin() + (dr_points_size - kFitPointsSize) : fit_dr_ego_points.begin();
    auto curve = FitterLeastSquareMethod(start_it, fit_dr_ego_points.end(), 1);
    FLOG_CROSS << " FitterLeastSquareMethod";
    NewMainLane(curve(0), curve(1), fit_dr_ego_points, dr_ego_lane_.width / 2, dr_ego_lane_.id);
  } else {
    FLOG_CROSS << " UpdateVirtualMainLane";
    UpdateVirtualMainLane();
  }
}

void VirtualEgoRoadProcessor::GenerateVirtualLaneAndLaneMarker(const std::vector<Point2DF> &points) {
  FLOG_CROSS << "GeneratetVirtualLaneAndLaneMarker";
  // 对virtual_lane进行赋值
  virtual_lane_             = {};
  virtual_left_lanemarker_  = {};
  virtual_right_lanemarker_ = {};
  virtual_lane_.id = kPreLaneId;
  // virtual_lane_.id = data_manager_->GetNewLandId();

  virtual_left_lanemarker_.id  = kPreLeftLanemarkerId;
  virtual_right_lanemarker_.id = kPreRightLanemarkerId;
  FLOG_CROSS << "points.size: " << points.size() << ",data_manager_->GetMainLane().lane_id: " << data_manager_->GetMainLane().lane_id;
  virtual_lane_.number_of_points = points.size();
  virtual_lane_.previous_lane_ids.emplace_back(data_manager_->GetMainLane().lane_id);
  virtual_lane_.line_points.reserve(points.size());
  // virtual_lane_.left_lane_marker_id = virtual_left_lanemarker_.id;
  // virtual_lane_.right_lane_marker_id = virtual_right_lanemarker_.id;
  virtual_lane_.width       = ego_lane_.width;
  virtual_lane_.is_virtual  = true;
  virtual_lane_.navi_action = data_manager_->GetMainLane().navi_action;
  bool has_ego_pos          = false;
  for (const auto &lane : data_manager_->bev_map()->lane_infos) {
    if (lane.position == 0) {
      has_ego_pos = true;
      break;
    }
  }
  if (has_ego_pos) {
    virtual_lane_.position = 7;
  }
  // 型点转换到dr坐标系
  for (const auto &point : points) {
    virtual_lane_.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
  }
  double lane_length{0.0};
  for (int i = 1; i < points.size(); i++) {
    lane_length += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  virtual_lane_.length = lane_length;

  double half_lane_width = ego_lane_.width / 2;

  // // 生成左右lanemarker
  // virtual_right_lanemarker_.number_of_points = points.size();
  // virtual_right_lanemarker_.line_points.reserve(points.size());
  // for (const auto &point : points) {
  //   virtual_right_lanemarker_.line_points.emplace_back(
  //       TransformPoint(Point2DF(point.x, point.y - half_lane_width), data_manager_->Twb()));
  // }

  // virtual_left_lanemarker_.number_of_points = points.size();
  // virtual_left_lanemarker_.line_points.reserve(points.size());
  // for (const auto &point : points) {
  //   virtual_left_lanemarker_.line_points.emplace_back(
  //       TransformPoint(Point2DF(point.x, point.y + half_lane_width), data_manager_->Twb()));
  // }
  FLOG_CROSS << " virtual_lane_points_size: " << virtual_lane_.number_of_points;
}

void VirtualEgoRoadProcessor::NewMainLane(double c0, double c1, const std::vector<Point2DF> &ego_points, double half_width,
                                          uint64_t lane_id, double control_dis) {
  auto &main_lane = data_manager_->GetMainLane();
  main_lane.Reset();
  // 第一次生成main_lane
  main_lane.lane_id     = lane_id;
  main_lane.navi_action = data_manager_->turn_type();
  main_lane.angle       = std::atan(c1);
  main_lane.top_points.set_x(ego_points.back().x);
  main_lane.top_points.set_y(ego_points.back().y);
  main_lane.valid      = true;
  main_lane.is_virtual = false;
  main_lane.points.emplace_back(main_lane.top_points);
  // 使用系数求取为了防止原始车道中心线末端两个点是斜的
  double x = ego_points.back().x + control_dis;
  double y = c0 + c1 * x;
  Vec2d  next_point(x, y);
  main_lane.points.emplace_back(next_point);
  // 求左侧main lane
  Vec2d axis = next_point - main_lane.top_points;
  axis.Normalize();
  axis.SelfRotate(pi / 2);
  main_lane.left_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, half_width);
  main_lane.left_top_points = main_lane.left_points.front();
  // 求右侧main lane
  main_lane.right_points     = TranslateSegmentAlongAxisStart(axis, main_lane.points, -half_width);
  main_lane.right_top_points = main_lane.right_points.front();
  FLOG_CROSS << " generate NewMainLane end!";
}

void VirtualEgoRoadProcessor::UpdateVirtualMainLane() {
  auto &main_lane = data_manager_->GetMainLane();
  main_lane.Reset();
  main_lane.lane_id     = dr_ego_lane_.id;
  main_lane.navi_action = data_manager_->turn_type();
  main_lane.valid       = true;
  main_lane.is_virtual  = true;
  // 中心线
  auto dr_points_size = virtual_lane_.line_points.size();
  FLOG_CROSS << "dr_points_size:" << dr_points_size;
  std::vector<Point2DF> fit_dr_ego_points;
  fit_dr_ego_points.reserve(dr_points_size);
  for (auto it = virtual_lane_.line_points.begin(); it != virtual_lane_.line_points.end(); it++) {
    fit_dr_ego_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }

  // 将车后的点放入到main_lane中，最少放两个
  if (dr_points_size > 1) {
    int index = 0;
    for (int i = fit_dr_ego_points.size() - 1; i >= 0; i--) {
      if (fit_dr_ego_points[i].x < 5.0) {
        index = i;
        break;
      }
    }
    index = std::max(index, 1);
    for (int i = 0; i <= index; i++) {
      main_lane.points.emplace_back(fit_dr_ego_points[i].x, fit_dr_ego_points[i].y);
      if (i == index) {
        main_lane.angle =
            Vec2d(fit_dr_ego_points[i].x - fit_dr_ego_points[i - 1].x, fit_dr_ego_points[i].y - fit_dr_ego_points[i - 1].y).Angle();
      }
    }
  }

  if (main_lane.points.size() > 1) {
    main_lane.top_points = main_lane.points.back();
    FLOG_CROSS << "UpdateVirtualMainLane heading:" << main_lane.angle << " ,main_lane backpoint_x: " << main_lane.points.back().x()
               << " ,y: " << main_lane.points.back().y();
  }

  // 左侧
  auto                  dr_left_points_size = virtual_left_lanemarker_.line_points.size();
  std::vector<Point2DF> fit_dr_left_points;
  fit_dr_left_points.reserve(dr_left_points_size);
  for (auto it = virtual_left_lanemarker_.line_points.begin(); it != virtual_left_lanemarker_.line_points.end(); it++) {
    fit_dr_left_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }
  for (int i = 0; i < fit_dr_left_points.size(); i++) {
    main_lane.left_points.emplace_back(fit_dr_left_points[i].x, fit_dr_left_points[i].y);
    if (fit_dr_left_points[i].x > 0.0 && main_lane.left_points.size() > 1) {
      main_lane.angle =
          Vec2d(fit_dr_left_points[i].x - fit_dr_left_points[i - 1].x, fit_dr_left_points[i].y - fit_dr_left_points[i - 1].y).Angle();
      break;
    }
  }
  if (!main_lane.left_points.empty()) {
    main_lane.left_top_points = main_lane.left_points.back();
    FLOG_CROSS << " main_left point_1_x: " << main_lane.left_points.front().x() << " ,y: " << main_lane.left_points.front().y();
  }

  FLOG_CROSS << "UpdateVirtualMainLane :" << virtual_right_lanemarker_.line_points.empty();
  // 右侧
  auto dr_right_points_size = virtual_right_lanemarker_.line_points.size();

  std::vector<Point2DF> fit_dr_right_points;
  fit_dr_right_points.reserve(dr_right_points_size);
  for (auto it = virtual_right_lanemarker_.line_points.begin(); it != virtual_right_lanemarker_.line_points.end(); it++) {
    fit_dr_right_points.emplace_back(TransformPoint(*it, data_manager_->Tbw()));
  }

  for (int i = 0; i < fit_dr_right_points.size(); i++) {
    main_lane.right_points.emplace_back(fit_dr_right_points[i].x, fit_dr_right_points[i].y);
    if (fit_dr_right_points[i].x > 0.0 && main_lane.right_points.size() > 1) {
      main_lane.angle =
          Vec2d(fit_dr_right_points[i].x - fit_dr_right_points[i - 1].x, fit_dr_right_points[i].y - fit_dr_right_points[i - 1].y).Angle();
      break;
    }
  }
  if (!main_lane.right_points.empty()) {
    main_lane.right_top_points = main_lane.right_points.back();
    FLOG_CROSS << "mian_right point_1_x: " << main_lane.right_points.front().x() << " ,y: " << main_lane.right_points.front().y();
  }
  FLOG_CROSS << "UpdateVirtualMainLane end";
}

bool VirtualEgoRoadProcessor::GenerateLeftTurnLaneNew() {
  bool success = GenerateLRTurnLaneNew();
  if (success) {
    virtual_lane_.plan_turn_type = BevTurnType::LEFT_TURN;
    virtual_lane_.navi_action    = BevAction::LEFT_TURN;
    virtual_lane_.bev_turn_type = BevAction::LEFT_TURN;
  }
  return success;
}

bool VirtualEgoRoadProcessor::GenerateRightTurnLaneNew() {
  bool success = GenerateLRTurnLaneNew();
  if (success) {
    virtual_lane_.plan_turn_type = BevTurnType::RIGHT_TURN;
    virtual_lane_.navi_action    = BevAction::RIGHT_TURN;
    virtual_lane_.bev_turn_type = BevAction::RIGHT_TURN;
  }
  return success;
}

bool VirtualEgoRoadProcessor::GenerateLRTurnLaneNew() {
  FLOG_CROSS << " GenerateLeftTurnLane is_generate_success_: " << is_generate_success_;
  // 1.没成功前没有ego_lane返回false
  if (!data_manager_->has_ego_lane() && !is_generate_success_) {
    return false;
  }

  // 2.拟合ego_lane
  Curve curve;
  fitter_egolane_success_ = FitterEgelane(curve);
  FLOG_CROSS << " fitter_egolane_success_: " << fitter_egolane_success_;
  // 3.生成/更新mainlane
  GenerateMainLane(curve);

  // 4.路口前拟合失败返回,使用历史值
  if (!fitter_egolane_success_ && !data_manager_->GetIsInCrossroad()) {
    return false;
  }

  // 5.生成虚拟lane，并转成dr坐标系，后续可以一直使用，中间只需要更新MainLane用于连接使用
  std::vector<Point2DF> virtual_line_points;
  if (!GenerateVirtualLRTurnLinePoints(virtual_line_points, curve)) {
    FLOG_CROSS << " can not generate  virtual_line_points!!!";
    return false;
  }
  is_generate_success_ = true;

  // 6.路口前生成了virtual_line_points就更新ego_lane_和dr_ego_lane_
  if (!data_manager_->GetIsInCrossroad()) {
    ego_lane_ = *data_manager_->EgoLane();
    // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
    dr_ego_lane_ = ego_lane_;
    for (auto &point : dr_ego_lane_.line_points) {
      TransformPoint(&point, data_manager_->Twb());
      // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
    }
  }

  // 7.生成lanemarker
  GenerateVirtualLaneAndLaneMarker(virtual_line_points);
  return true;
}

bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve) {
  bool is_success{false}; 
  auto virtual_line_source = data_manager_->GetVirtualLineSource();
  // 判断是否使用经验轨迹中心线
  if (kTurnUsingExpTrajPoints) {
    is_success = GenerateVirtualLRTurnLinePointsByExpTraj(virtual_line_points, curve);
    // 已经成功且进入了路口模式，路口前失败了也不再继续使用sd
    if (!is_success && data_manager_->GetCrossState() == CrossState::PREDICTION && virtual_line_source ==  VirtualLineSource::EXP) {
      return false;
    }
    if (is_success) {
      virtual_line_source = VirtualLineSource::EXP;
      FLOG_CROSS << " virtual_line_source: " << EnumToString(virtual_line_source);
      data_manager_->SetVirtualLineSource(virtual_line_source);
      return true;
    }
  }
  // 判断是否使用LD中心线
  if (kTurnUsingLdPoints) {
    is_success = GenerateVirtualLRTurnLinePointsByLd(virtual_line_points, curve);
    // 已经成功且进入了路口模式，路口前失败了也不再继续使用经验轨迹或sd
    if (!is_success && data_manager_->GetCrossState() == CrossState::PREDICTION && virtual_line_source ==  VirtualLineSource::LD) {
      return false;
    }
    if (is_success) {
      virtual_line_source = VirtualLineSource::LD;
      FLOG_CROSS << " virtual_line_source: " << EnumToString(virtual_line_source);
      data_manager_->SetVirtualLineSource(virtual_line_source);
      return true;
    }
  }

  // 新增只使用ld和经验轨迹
  if (virtual_using_only_ld_traj) {
    FLOG_CROSS << "LRTurn virtual_using_only_ld_traj";
    return false;
  }

  // 最后选择sd
  is_success = GenerateVirtualLRTurnLinePointsBySd(virtual_line_points, curve);
  if (is_success) {
    virtual_line_source = VirtualLineSource::SD;
    FLOG_CROSS << " virtual_line_source: " << EnumToString(virtual_line_source);
    data_manager_->SetVirtualLineSource(virtual_line_source);
    return true;
  }
  return false;

  // switch (DecideLRTurnVirtualLineSource()) {
  //   case 0:
  //     is_success = GenerateVirtualLRTurnLinePointsByHeading(virtual_line_points, curve);
  //     break;
  //   case 1:
  //     is_success = GenerateVirtualLRTurnLinePointsByObs(virtual_line_points, curve);
  //     break;
  //   case 2:
  //     is_success = GenerateVirtualLRTurnLinePointsBySd(virtual_line_points, curve);
  //     break;
  //   case 3:
  //     is_success = GenerateVirtualLRTurnLinePointsByLd(virtual_line_points, curve);
  //     break;
  //   case 4:
  //     is_success = GenerateVirtualLRTurnLinePointsByExpTraj(virtual_line_points, curve);
  //     break;
  //   default:
  //     FLOG_CROSS << "error ";
  //     break;
  // }
  // return is_success;
}

bool VirtualEgoRoadProcessor::GenerateVirtualAroundTurnLinePoints(std::vector<Point2DF> &virtual_line_points, Curve curve) {
  bool is_success{false};
  // 屏蔽掉头
  if (virtual_using_only_ld_traj) {
    FLOG_CROSS << "not enable GenerateVirtualAroundTurnLinePoints!";
    return false;
  }
  switch (DecideAroundTurnVirtualLineSource()) {
    case VirtualLineSource::PRE:
      is_success = GenerateVirtualAroundTurnLinePointsByPer(virtual_line_points, curve);
      break;
    case VirtualLineSource::OBS:
      is_success = GenerateVirtualLRTurnLinePointsByObs(virtual_line_points, curve);
      break;
    case VirtualLineSource::SD:
      is_success = GenerateVirtualLRTurnLinePointsBySd(virtual_line_points, curve);
      break;
    case VirtualLineSource::LD:
      is_success = GenerateVirtualLRTurnLinePointsByLd(virtual_line_points, curve);
      break;
    case VirtualLineSource::EXP:
      is_success = GenerateVirtualLRTurnLinePointsByExpTraj(virtual_line_points, curve);
      break;
    default:
      FLOG_CROSS << "error ";
      break;
  }
  return is_success;
}

VirtualLineSource VirtualEgoRoadProcessor::DecideAroundTurnVirtualLineSource() {
  auto virtual_line_source = data_manager_->GetVirtualLineSource();
  if (kAroundUsingExpTrajPoints) {
    virtual_line_source = VirtualLineSource::EXP;
  }else if(kAroundUsingLdPoints){
    virtual_line_source = VirtualLineSource::LD;
  }
  FLOG_CROSS << " virtual_line_source: " << EnumToString(virtual_line_source);
  data_manager_->SetVirtualLineSource(virtual_line_source);
  return virtual_line_source;
}
bool VirtualEgoRoadProcessor::GenerateVirtualAroundTurnLinePointsByPer(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  AINFO << " GenerateVirtualAroundTurnLinePointsByPer ";
  // auto &main_lane  = data_manager_->GetMainLane();
  // auto &sd_section = data_manager_->GetSdSections();
  // auto  top_lane   = GenerateTopLanePoints(sd_section.end_point, sd_section.top_angle(), ego_lane_.width / 2);//
  // bool is_lane_valid{false};
  // int  bot_lane_index = -1;
  // int  top_index      = -1;

  // std::vector<Vec2d> control_points = {};

  // BevLaneInfo virtual_lane;
  // for (const auto &point : main_lane.points) {
  //   AINFO << " input main_lane Points X: " << point.x() << " ,y: " << point.y();
  // }
  // for (const auto &point : top_lane.points) {
  //   AINFO << " input top_lane Points X: " << point.x() << " ,y: " << point.y();
  // }
  // if (InsertBezierPoints(main_lane.points, top_lane.points, control_points, bot_lane_index, top_index)) {
  //   AINFO << " bot_lane_index: " << bot_lane_index << " ,top_index: " << top_index
  //              << ", control_points.size: " << control_points.size();
  //   if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
  //     // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
  //     if (main_lane.is_virtual) {
  //       control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
  //     }
  //     control_points.emplace_back(top_lane.points.back());
  //     virtual_line_points.reserve(control_points.size());
  //     for (const auto &point : control_points) {
  //       virtual_line_points.emplace_back(point.x(), point.y());
  //     }
  //     return true;
  //   }
  // }

  if (!data_manager_) {
    AINFO << "data_manager_ is null";
    return false;
  }
  BevMapInfoPtr bev_map_info;
  SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_info);  //形点用跟踪输出
  auto ego_lanes = data_manager_->GetEgoLanes();
  for (auto &lane : ego_lanes) {
    AINFO << " ego_lanes ID: " << lane.lane_id << " is_ego: " << lane.is_ego;
  }
  if (ego_lanes.empty() || !bev_map_info) {  //top_lanes实时更新 到距离30才有值
    AINFO << "ego_lanes is null";
    return false;
  }
  std::vector<byd::common::math::Vec2d> newCurve;
  auto itEgoLane = std::find_if(ego_lanes.begin(), ego_lanes.end(), [](const auto &lane) { return lane.is_ego; });
  if (itEgoLane == ego_lanes.end()) {
    AINFO << "itEgoLane is null";
    return false;
  }
  std::vector<byd::common::math::Vec2d> tarGetPoints;
  auto                                  sdAngle          = data_manager_->GetSdSections().top_angle();  //使用sd方向
  auto                                  sdDir            = byd::common::math::Vec2d(std::cos(sdAngle), std::sin(sdAngle));
  auto                                  sd_box_corss_pts = data_manager_->GetSdSections().end_point;
  if (std::abs(sd_box_corss_pts.x()) < 1e-3 && std::abs(sd_box_corss_pts.y()) < 1e-3) {
    AINFO << "sd_box_corss_pts is null";
    return false;
  }
  AINFO << "sd_box_corss_pts: " << sd_box_corss_pts.x() << " , " << sd_box_corss_pts.y();
  AINFO << "sdDir: " << sdDir.x() << " , " << sdDir.y();
  double dis = 2.0;  // 2m
  auto   pts = sd_box_corss_pts + dis * sdDir;
  tarGetPoints.emplace_back(pts);               //第2个点
  tarGetPoints.emplace_back(sd_box_corss_pts);  //第2个点
  // newCurve = cem::fusion::interpolator::GenerateStaticSmoothCurve(tarGetPoints, itEgoLane->points);
  //itEgoLane 延长和路沿对齐  延长方向 egolane方向  也可能路沿是靠后的
  std::vector<byd::common::math::Vec2d> obs_points;
  auto                                  Tbw    = data_manager_->Twb().inverse();
  auto &                                edages = bev_map_info->edges;
#if SAVE_U_EDAGE_PT
  auto & ppmImage  = data_manager_->ppmImage;
  auto   top_lanes = data_manager_->GetTopLanes();
  double y_offset  = 1.6f;
  for (const auto &toplane : top_lanes) {
    auto color_index = 0;
    color_index      = toplane.is_target ? 1 : 0;
    ppmImage->DrawNumber(static_cast<int>(toplane.lane_id), 4.0f, 48.0f - y_offset, color_index);
    y_offset += 8.0f;
  }
  y_offset = 1.6f;
  for (const auto &toplane : ego_lanes) {
    auto color_index = 1;
    color_index      = toplane.is_ego ? 0 : 1;
    ppmImage->DrawNumber(static_cast<int>(toplane.lane_id), 2.0f, 48.0f - y_offset, color_index);
    y_offset += 8.0f;
  }
#endif
  for (auto &edage : edages) {
    for (auto &point : edage.line_points) {
      if ((point.x * point.x + point.y * point.y) < 400.f) {  //
#if SAVE_U_EDAGE_PT
        ppmImage->DrawPoint(point.x, point.y, 1);
#endif
        obs_points.emplace_back(point.x, point.y);
      }
    }
  }

  newCurve = cem::fusion::interpolator::GenerateUturnCircle(tarGetPoints, itEgoLane->points, obs_points);
  // newCurve = cem::fusion::interpolator::TrajectoryPlanner::planUTurn(itEgoLane->points[1],0.f, tarGetPoints[1],M_PI,obs_points);
  if (!newCurve.empty()) {
    for (const auto &point : newCurve) {
      cem::message::common::Point2DF vcsPt(point.x(), point.y());
#if SAVE_U_EDAGE_PT
      ppmImage->DrawPoint(point.x(), point.y(), 0);
#endif
      // drPoint = TransformPoint(drPoint, data_manager_->Twb());
      virtual_line_points.emplace_back(vcsPt);
    }

    auto last_point = newCurve.back();
    auto direction  = tarGetPoints[0] - tarGetPoints[1];
    direction.Normalize();

    for (int i = 1; i < 10; i++) {
      last_point += direction;  // 延长10个点
#if SAVE_U_EDAGE_PT
      ppmImage->DrawPoint(last_point.x(), last_point.y(), 0);
      AINFO << "last_point x: " << last_point.x() << " ,y: " << last_point.y();
#endif
      virtual_line_points.emplace_back(last_point.x(), last_point.y());
    }
#if SAVE_U_EDAGE_PT
    ppmImage->Save();
#endif
    return true;
  } else {
    AINFO << "newCurve is empty, return";
    return false;
  }

  // bev_map_->lane_infos.emplace_back();
  // auto &connect_lane = bev_map_->lane_infos.back();
  // for (const auto &point : newCurve) {
  //   cem::message::common::Point2DF drPoint(point.x(), point.y());
  //   drPoint = TransformPoint(drPoint, data_manager_->Twb());
  //   connect_lane.line_points.emplace_back(drPoint);
  // }
  // connect_lane.id = data_manager_->GetNewLandId();
  // connect_lane.is_junction_lane = true;
  // connect_lane.is_virtual = true;
  // connect_lane.previous_lane_ids.emplace_back(itEgoLane->lane_id);
  // if(best_lane.is_connect_lane){
  //   connect_lane.next_lane_ids.emplace_back(best_lane.lane_id);
  // }
  // road_connector_->SetConnectLane(connect_lane);//存连接线
  return false;
}
// int VirtualEgoRoadProcessor::DecideLRTurnVirtualLineSource() {
//   int virtual_line_source = data_manager_->GetVirtualLineSource();
//   if (kTurnUsingExpTrajPoints) {
//     virtual_line_source = 4;
//   } else if (kTurnUsingLdPoints) {
//     virtual_line_source = 3;
//   } else {
//     virtual_line_source = 2;
//   }
//   FLOG_CROSS << " virtual_line_source: " << virtual_line_source;
//   data_manager_->SetVirtualLineSource(virtual_line_source);
//   return virtual_line_source;
// }

bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePointsByLd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return GenerateVirtualLinePointsByLd(virtual_line_points, ego_curve);
}

bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePointsByExpTraj(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return GenerateVirtualLinePointsByExpTraj(virtual_line_points, ego_curve);
}

bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePointsBySd(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  // 暂时路口后不再更新
  if (!data_manager_->GetIsInCrossroad() && !data_manager_->has_ego_lane()) {
    FLOG_CROSS << " IsInCrossroad!!! ";
    return false;
  }

  FLOG_CROSS << "GenerateTurnLaneAndLaneMarker left_right_move_dis: " << left_right_move_dis_;
  auto &main_lane  = data_manager_->GetMainLane();
  auto &sd_section = data_manager_->GetSdSections();
  // 更新策略,第一次成功后再更新
  if (!CaculateMoveDisOcc(left_right_move_dis_)) {
    FLOG_CROSS << "CaculateMoveDisOcc faild!";
    return false;
  }
  //
  auto top_lane = GenerateTopLanePoints(sd_section.end_point, sd_section.top_angle(), ego_lane_.width / 2);
  if (main_lane.points.size() < 2) {
    FLOG_CROSS << "GenerateTurnLaneAndLaneMarker";
    return false;
  }
  // 根据需要的偏移距离左右平移sd点
  std::vector<Vec2d> sd_input_points;
  std::vector<Vec2d> sd_points;
  sd_points.emplace_back(sd_section.end_point);
  sd_points.emplace_back(sd_section.top_vector);
  FLOG_CROSS << " begin move sd_section.end_point_x: " << sd_section.end_point.x() << " ,y: " << sd_section.end_point.y()
             << " ,top_vector_x: " << sd_section.top_vector.x() << " ,y: " << sd_section.top_vector.y()
             << " ,angle: " << (sd_section.top_vector - sd_section.end_point).Angle();
  if (std::abs(left_right_move_dis_) > 0.2) {
    Vec2d axis = sd_section.top_vector - sd_section.end_point;
    axis.Normalize();
    axis.SelfRotate(pi / 2);
    sd_input_points = TranslateSegmentAlongAxisStart(axis, sd_points, left_right_move_dis_);
  } else {
    sd_input_points = sd_points;
  }
  FLOG_CROSS << " after move sd_section.end_point_x: " << sd_input_points.front().x() << " ,y: " << sd_input_points.front().y()
             << " ,top_vector_x: " << sd_input_points.back().x() << " ,y: " << sd_input_points.back().y()
             << " ,angle: " << (sd_input_points.back() - sd_input_points.front()).Angle();

  auto intersection = LineIntersection(sd_input_points.front(), sd_input_points.back(), main_lane.points[main_lane.points.size() - 2],
                                       main_lane.points.back());
  // for(const auto& point:main_lane.points){
  //   FLOG_CROSS << "main_lane.point_x: "<< point.x() << " ,y: " << point.y();
  // }
  FLOG_CROSS << "intersection.first: " << intersection.first << " ,second_x: " << intersection.second.x() << " ,y:" << intersection.second.y();
  if (!intersection.first || intersection.second.x() < main_lane.top_points.x() ||
      (intersection.second.x() > sd_input_points.front().x() &&
       std::abs(intersection.second.y()) > std::abs(sd_input_points.front().y()))) {
    return false;
  }
  if (!data_manager_->GetIsInCrossroad() || data_manager_->GetOccSdEdgeInfo().max_main_left_x > 0.0) {
    if (data_manager_->GetOccSdEdgeInfo().max_main_left_x - main_lane.top_points.x() > 5.0 &&
        data_manager_->GetOccSdEdgeInfo().max_main_left_x < intersection.second.x()) {
      double k = (intersection.second.y() - main_lane.top_points.y()) / (intersection.second.x() - main_lane.top_points.x());
      std::vector<byd::common::math::Vec2d> points;
      points.emplace_back(main_lane.top_points);
      int end_index = static_cast<int>(std::floor(data_manager_->GetOccSdEdgeInfo().max_main_left_x - main_lane.top_points.x()));
      FLOG_CROSS << " main_lane_move: " << end_index;
      for (int i = 2; i < end_index; i += 2) {
        points.emplace_back(points.back().x() + 2.0, points.back().y() + 2.0 * k);
      }
      main_lane.top_points = {data_manager_->GetOccSdEdgeInfo().max_main_left_x, main_lane.top_points.y() + k * end_index};
      main_lane.points = points;
      main_lane.is_virtual = true;
    }
  }
  // 得到起点控制点范围
  double start_max_control_dis = main_lane.top_points.DistanceTo(intersection.second);
  // 得到终点控制点范围
  double end_max_control_dis = sd_input_points.front().DistanceTo(intersection.second);

  FLOG_CROSS << " start_max_control_dis: " << start_max_control_dis << " , end_max_control_dis: " << end_max_control_dis;
  // 生成main_lane_points
  std::vector<Vec2d> main_lane_points;
  main_lane_points.emplace_back(main_lane.top_points);
  auto & next_point = main_lane_points.emplace_back();
  double x          = main_lane.top_points.x() + (intersection.second.x() - main_lane.top_points.x()) * 0.88;
  double y          = main_lane.top_points.y() + (intersection.second.y() - main_lane.top_points.y()) * 0.88;
  next_point.set_x(x);
  next_point.set_y(y);

  // 生成top_lane_points
  std::vector<Vec2d> top_lane_points;
  x = intersection.second.x() + (sd_input_points.front().x() - intersection.second.x()) * 0.66;
  y = intersection.second.y() + (sd_input_points.front().y() - intersection.second.y()) * 0.66;
  top_lane_points.emplace_back(x, y);
  top_lane_points.emplace_back(sd_input_points.front());
  for (const auto &point : main_lane_points) {
    FLOG_CROSS << " input main_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  for (const auto &point : top_lane_points) {
    FLOG_CROSS << " input top_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  if (sd_input_points.front().x() < 10.0) {
    FLOG_CROSS << " sd_input_points NOT MATCH main_lane_points ";
    return false;
  }
  // 连接中心线
  std::vector<Vec2d> control_points = {};
  int                bot_lane_index = -1;
  int                top_index      = -1;
  if (InsertBezierPoints(main_lane_points, top_lane_points, control_points, bot_lane_index, top_index)) {
    FLOG_CROSS << " bot_lane_index: " << bot_lane_index << " ,top_index: " << top_index
               << ", control_points.size: " << control_points.size();
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 1) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
      }
      control_points.emplace_back(top_lane_points.back());
      auto size = control_points.size();
      virtual_line_points.reserve(size);
      // FLOG_CROSS << " control_points_size: " << control_points.size();
      // double left_nearst_dis  = data_manager_->FindNearstLeftBoundaryDis(control_points.front());
      // // double right_nearst_dis = data_manager_->FindNearstRightBoundaryDis(control_points.front());
      // FLOG_CROSS << " point_x: " << control_points.front().x() << " ,y: " << control_points.front().y()
      //            << " ,left_nearst_dis: " << left_nearst_dis;
      for (const auto &point : control_points) {
        virtual_line_points.emplace_back(point.x(), point.y());
      }
      // 最后延长20m
      double dx           = virtual_line_points[size - 1].x - virtual_line_points[size - 2].x;
      double dy           = virtual_line_points[size - 1].y - virtual_line_points[size - 2].y;
      double delta_length = std::hypot(dx, dy);
      if (delta_length > 0.1 && delta_length < 0.5) {
        double k     = std::abs(1 / delta_length);
        dx           = dx * k;
        dy           = dy * k;
        delta_length = std::hypot(dx, dy);
      }
      FLOG_CROSS << " dx: " << dx << " ,dy: " << dy;
      double length = 0.0;
      int    break_index{0};
      while (length < 20.0 && break_index < 20) {
        double x = virtual_line_points.back().x + dx;
        double y = virtual_line_points.back().y + dy;
        virtual_line_points.emplace_back(x, y);
        length += delta_length;
        break_index++;
      }
      // for(int i = 0; i < virtual_line_points.size();i++){
      //   FLOG_CROSS << " virtual_line_points_x: " << virtual_line_points[i].x << " ,y: " << virtual_line_points[i].y;
      // }
      return true;
    }
  }
  return false;
}

bool VirtualEgoRoadProcessor::CaculateMoveDisOcc(double& move_dis){
  FLOG_CROSS << " CaculateMoveDisOcc!!!";
  if (!is_generate_success_) {
    return true;
  }

  auto &sd_section = data_manager_->GetSdSections();
  // 历史预推线转换
  std::vector<Vec2d> virtual_line_geos;
  virtual_line_geos.reserve(virtual_lane_.line_points.size());
  int pre_index = 0;
  for (const auto &point : virtual_lane_.line_points) {
    const auto &pt = TransformPoint(point, data_manager_->Tbw());
    if (pt.x < 0.0) {
      continue;
    }
    // FLOG_CROSS << "prediction_line_geos_pre_index: " << pre_index << " ,x: " << pt.x << " ,pt.y: " << pt.y;
    virtual_line_geos.emplace_back(pt.x, pt.y);
    pre_index++;
  }
  if (virtual_line_geos.size() < 2) {
    FLOG_CROSS << "connect_line_geos";
    return !data_manager_->GetIsInCrossroad();
  }
  // data_manager_->DealBoundary(virtual_line_geos);
  const auto& occ_sd_edge_info = data_manager_->GetOccSdEdgeInfo();
  const auto& sd_edge_points = occ_sd_edge_info.sd_edge_points;
  if (!occ_sd_edge_info.is_valid || sd_edge_points.empty() || sd_edge_points.size() <= occ_sd_edge_info.last_is_diff_heading_index ||
      sd_edge_points.size() <= occ_sd_edge_info.end_point_sd_index + 1) {
    FLOG_CROSS << "has no sd_edge_points!";
    return !data_manager_->GetIsInCrossroad();
  }

  // 求历史线在endpoint处到sd的距离
  int end_point_sd_index = occ_sd_edge_info.end_point_sd_index;
  double virtual_line_dis = 0.0;
  for (int i = 0; i < virtual_line_geos.size(); i++) {
    double lon_dis = PointToVectorDist(sd_edge_points[end_point_sd_index].sd_point, sd_edge_points[end_point_sd_index].right_point,
                                       virtual_line_geos[i]);
    if (lon_dis > 2.0) {
      continue;
    }
    double lat_dis   = PointToVectorDist(sd_edge_points[end_point_sd_index].sd_point, sd_edge_points[end_point_sd_index + 1].sd_point,
                                       virtual_line_geos[i]);
    bool   is_right  = PointInVectorSide(sd_edge_points[end_point_sd_index].sd_point, sd_edge_points[end_point_sd_index + 1].sd_point,
                                      virtual_line_geos[i]) >= 0.0;
    virtual_line_dis = is_right ? -lat_dis : lat_dis;
    break;
  }

  // 求目标distance
  constexpr double default_lane_width    = 3.5;
  double           target_left_edge_dis  = std::numeric_limits<double>::max();
  double           target_right_edge_dis = std::numeric_limits<double>::max();
  int              left_lane_num         = 0;
  int              right_lane_num        = std::max(sd_section.current_lane_num, 1);
  // 车道选择--index从左往右0~n
  int target_top_lane_index = 0;
  if (data_manager_->turn_type() == BevAction::LEFT_TURN) {
    // if (data_manager_->GetEgoRoadLanes().left_same_turn_size == 0 || right_lane_num == 1) {
    //   target_top_lane_index = 0;
    // } else {
    //   target_top_lane_index = std::min(right_lane_num - 1, data_manager_->GetEgoRoadLanes().left_same_turn_size);
    // }
    // 暂定逻辑优先走左二车道
    if (right_lane_num == 1) {
      target_top_lane_index = 0;
    } else if (data_manager_->GetEgoRoadLanes().is_one_lane) {
      target_top_lane_index = std::min(right_lane_num - 2, 1);
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size == 0 && data_manager_->GetEgoRoadLanes().left_same_turn_size != 0) {
      target_top_lane_index = std::min(right_lane_num - 1, data_manager_->GetEgoRoadLanes().left_same_turn_size);
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size != 0 && data_manager_->GetEgoRoadLanes().left_same_turn_size == 0) {
      target_top_lane_index = 0;
    }
  } else if (data_manager_->turn_type() == BevAction::RIGHT_TURN) {
    if (right_lane_num == 1) {
      target_top_lane_index = 0;
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size == 0) {
      target_top_lane_index = right_lane_num - 1;
    } else {
      target_top_lane_index = std::max(0, right_lane_num - data_manager_->GetEgoRoadLanes().right_same_turn_size - 1);
    }
  }
  FLOG_CROSS << " left_top_lane_num: " << left_lane_num << " ,right_top_lane_num: " << right_lane_num
             << " ,target_top_lane_index: " << target_top_lane_index;
  // 跟据选择的车道计算左右目标距离，距离都会偏大设置
  target_right_edge_dis = (right_lane_num - target_top_lane_index) * default_lane_width - 1.5;
  target_left_edge_dis  = (left_lane_num + target_top_lane_index + 1) * default_lane_width - 1.5;

  bool   has_top_right_min_dis = false;

    // 左右侧距离合理性检测
  double target_right_distance = occ_sd_edge_info.top_right_min_distance + std::max(target_right_edge_dis, 2.0);
  double target_left_distance  = occ_sd_edge_info.top_left_min_distance - std::max(target_left_edge_dis, 2.0);

  // 位置合理性判断
  constexpr double move_step = 0.3;

  FLOG_CROSS << "top_right_min_dis fit virtual_line_dis: " << virtual_line_dis << " ,target_right_distance: " << target_right_distance;
  if ((virtual_line_dis > target_right_distance - 1.0 && virtual_line_dis < target_right_distance + 1.0) ||
      virtual_line_dis - occ_sd_edge_info.top_right_min_distance < 2.5 || occ_sd_edge_info.top_left_min_distance - virtual_line_dis < 2.5) {
    if (virtual_line_dis - occ_sd_edge_info.top_right_min_distance < 2.5) {
      move_dis = occ_sd_edge_info.top_right_min_distance + 2.8;
      FLOG_CROSS << "move to right boundary: " << move_dis;
      return true;
    } else if (occ_sd_edge_info.top_left_min_distance - virtual_line_dis < 2.5) {
      move_dis = occ_sd_edge_info.top_left_min_distance - 2.8;
      FLOG_CROSS << "move to left boundary: " << move_dis;
      return true;
    } else {
      FLOG_CROSS << "fet not move";
      move_dis = virtual_line_dis;
    }
    FLOG_CROSS << "move_dis: " << move_dis;
    return !data_manager_->GetIsInCrossroad();
  } else if (virtual_line_dis > target_right_distance + 0.8 && virtual_line_dis - occ_sd_edge_info.top_right_min_distance > 2.2) {
    if (data_manager_->GetIsInCrossroad()) {
      FLOG_CROSS << "right move 1";
      move_dis = -move_step;
    } else {
      FLOG_CROSS << "right move 2";
      // move_dis = target_right_distance - virtual_line_dis;
      move_dis = -move_step * 2;
    }
  } else if (virtual_line_dis < target_right_distance - 0.8 && occ_sd_edge_info.top_left_min_distance - virtual_line_dis > 2.2) {
    if (data_manager_->GetIsInCrossroad()) {
      FLOG_CROSS << "left move 1";
      move_dis = move_step;
    } else {
      FLOG_CROSS << "left move 2";
      // move_dis = target_right_edge_dis - top_right_min_dis;
      move_dis = 2 * move_step;
    }
  } else {
    move_dis = 0.0;
    FLOG_CROSS << "not move!";
  }

  FLOG_CROSS << "before move_dis: " << move_dis;
  // 下发的是sd的点应该左移或者右移的距离
  move_dis = move_dis + virtual_line_dis;
  FLOG_CROSS << " target_right_distance: " << target_right_distance << " ,target_left_edge_dis: " << target_left_edge_dis
             << " ,move_dis: " << move_dis;
  return true;
}

bool VirtualEgoRoadProcessor::CaculateMoveDis(double &move_dis) {
  FLOG_CROSS << " CaculateMoveDis!!!";
  if (!is_generate_success_) {
    return true;
  }
  auto &main_lane  = data_manager_->GetMainLane();
  auto &sd_section = data_manager_->GetSdSections();
  // 历史预推线转换
  std::vector<Vec2d> virtual_line_geos;
  virtual_line_geos.reserve(virtual_lane_.line_points.size());
  int pre_index = 0;
  for (const auto &point : virtual_lane_.line_points) {
    const auto &pt = TransformPoint(point, data_manager_->Tbw());
    if (pt.x < 0.0) {
      continue;
    }
    // FLOG_CROSS << "prediction_line_geos_pre_index: " << pre_index << " ,x: " << pt.x << " ,pt.y: " << pt.y;
    virtual_line_geos.emplace_back(pt.x, pt.y);
    pre_index++;
  }
  if (virtual_line_geos.size() < 2) {
    FLOG_CROSS << "connect_line_geos";
    return !data_manager_->GetIsInCrossroad();
  }
  // 求历史预推线距离左右边界的距离--5米求一次
  std::vector<VirtualLaneDegeDid> left_dis;
  std::vector<VirtualLaneDegeDid> right_dis;
  const auto &                    left_edges  = data_manager_->GetLeftCrossEdges();
  const auto &                    right_edges = data_manager_->GetRightCrossEdges();

  // 左右都没边界点直接返回
  if (left_edges.empty() && right_edges.empty()) {
    FLOG_CROSS << "CrossEdges empty!!!";
    return !data_manager_->GetIsInCrossroad();
  }
  double           step        = 1;
  constexpr double step_length = 4.0;
  for (int i = 0; i < virtual_line_geos.size() - 1; i = i + step) {
    // 计算下一个点的index
    int    index  = 0;
    double length = 0;
    while (length < step_length && i < virtual_line_geos.size() - 1 && index < 100) {
      length += virtual_line_geos[i].DistanceTo(virtual_line_geos[i + 1]);
      index++;
    }
    step = index;
    // 计算左侧最近距离
    if (!left_edges.empty()) {
      auto & left_edge_dis = left_dis.emplace_back();
      int    edge_index{-1};
      int    point_index{-1};
      double min_dis = std::numeric_limits<double>::max();
      for (int j = 0; j < left_edges.size(); j++) {
        const auto &points = left_edges[j].points;
        for (int k = 0; k < points.size(); k++) {
          double dis = virtual_line_geos[i].DistanceSquareTo(points[k]);
          if (dis < min_dis) {
            min_dis     = dis;
            edge_index  = j;
            point_index = k;
          }
        }
      }
      if (edge_index < 0) {
        continue;
      }
      // 判断最小值是否有效
      auto  ab   = virtual_line_geos[i + 1] - virtual_line_geos[i];
      Vec2d perp = {-ab.y(), ab.x()};
      perp.Normalize();
      Vec2d  projrct      = {virtual_line_geos[i].x() + perp.x(), virtual_line_geos[i].y() + perp.y()};
      auto   edge_point   = left_edges[edge_index].points[point_index];
      double pro_dis      = PointToVectorDist(virtual_line_geos[i], projrct, edge_point);
      left_edge_dis.index = i;
      if (pro_dis < 2.0) {
        left_edge_dis.val = true;
        left_edge_dis.dis = std::sqrt(min_dis);
        FLOG_CROSS << " left_edge_dis index: " << i << " ,dis: " << left_edge_dis.dis << " ,edge_index: " << edge_index
                   << " ,point_index: " << point_index << " ,edge_point_x: " << edge_point.x() << " ,y:" << edge_point.y();
      }
    }

    // 计算右侧最近距离
    if (!right_edges.empty()) {
      auto & right_edge_dis = right_dis.emplace_back();
      int    edge_index{-1};
      int    point_index{-1};
      double min_dis = std::numeric_limits<double>::max();
      for (int j = 0; j < right_edges.size(); j++) {
        const auto &points = right_edges[j].points;
        for (int k = 0; k < points.size(); k++) {
          double dis = virtual_line_geos[i].DistanceSquareTo(points[k]);
          if (dis < min_dis) {
            min_dis     = dis;
            edge_index  = j;
            point_index = k;
          }
        }
      }
      if (edge_index < 0) {
        continue;
      }
      // 判断最小值是否有效
      auto  ab   = virtual_line_geos[i + 1] - virtual_line_geos[i];
      Vec2d perp = {-ab.y(), ab.x()};
      perp.Normalize();
      Vec2d  projrct       = {virtual_line_geos[i].x() + perp.x(), virtual_line_geos[i].y() + perp.y()};
      auto   edge_point    = right_edges[edge_index].points[point_index];
      double pro_dis       = PointToVectorDist(virtual_line_geos[i], projrct, right_edges[edge_index].points[point_index]);
      right_edge_dis.index = i;
      if (pro_dis < 2.0) {
        right_edge_dis.val = true;
        right_edge_dis.dis = std::sqrt(min_dis);
        FLOG_CROSS << " right_edge_dis index: " << index << " ,dis: " << right_edge_dis.dis << " ,edge_index: " << edge_index
                   << " ,point_index: " << point_index << " ,edge_point_x: " << edge_point.x() << " ,y:" << edge_point.y();
      }
    }
  }

  // for (const auto &dis : left_dis) {
  //   FLOG_CROSS << " left_dis_index: " << dis.index << " ,val: " << dis.val << " ,dis: " << dis.dis;
  // }
  // for (const auto &dis : right_dis) {
  //   FLOG_CROSS << " right_dis_index: " << dis.index << " ,val: " << dis.val << " ,dis: " << dis.dis;
  // }

  // 求左侧最近点
  double min_left_edge_dis       = std::numeric_limits<double>::max();
  int    min_left_edge_dis_index = -1;
  for (const auto &dis : left_dis) {
    if (dis.val && dis.dis < min_left_edge_dis) {
      min_left_edge_dis       = dis.dis;
      min_left_edge_dis_index = dis.index;
    }
  }
  // 求右侧最近点
  double min_right_edge_dis       = std::numeric_limits<double>::max();
  int    min_right_edge_dis_index = -1;
  for (const auto &dis : right_dis) {
    if (dis.val && dis.dis < min_right_edge_dis) {
      min_right_edge_dis       = dis.dis;
      min_right_edge_dis_index = dis.index;
    }
  }
  FLOG_CROSS << " min_left_edge_dis_index: " << min_left_edge_dis_index << " ,min_left_edge_dis: " << min_left_edge_dis
             << " ,min_right_edge_dis_index: " << min_right_edge_dis_index << " ,min_right_edge_dis: " << min_right_edge_dis;
  // 左右都没有效的最近距离点直接返回
  if (min_left_edge_dis_index < 0 && min_right_edge_dis_index < 0) {
    FLOG_CROSS << " has no left and right min dis";
    return !data_manager_->GetIsInCrossroad();
  }
  // 碰撞判断
  bool has_left_edge_collision  = min_left_edge_dis < 1.5;
  bool has_right_edge_collision = min_right_edge_dis < 1.5;

  // 寻找路口后距离左右路沿的距离，先寻找对应endpoint的点，然后找附近4个点中的最小左右距离
  double sd_top_2_lane_dis = std::numeric_limits<double>::max();
  int    start_index       = -1;
  if (!left_dis.empty()) {
    for (int i = 0; i < left_dis.size() - 1; i++) {
      float distance = 0;
      if (GetDistPointLane(sd_section.end_point, virtual_line_geos[left_dis[i].index], virtual_line_geos[left_dis[i + 1].index],
                           distance)) {
        bool is_left      = PointInVectorSide(virtual_line_geos[left_dis[i].index], virtual_line_geos[left_dis[i + 1].index],
                                         Point2DF(sd_section.end_point.x(), sd_section.end_point.y())) < 0.0;
        sd_top_2_lane_dis = is_left ? distance : -distance;
        start_index       = i;
        break;
      }
    }
  } else if (!right_dis.empty()) {
    for (int i = 0; i < right_dis.size() - 1; i++) {
      float distance = 0;
      if (GetDistPointLane(sd_section.end_point, virtual_line_geos[right_dis[i].index], virtual_line_geos[right_dis[i + 1].index],
                           distance)) {
        bool is_left      = PointInVectorSide(virtual_line_geos[right_dis[i].index], virtual_line_geos[right_dis[i + 1].index],
                                         Point2DF(sd_section.end_point.x(), sd_section.end_point.y())) < 0.0;
        sd_top_2_lane_dis = is_left ? distance : -distance;
        start_index       = i;
        break;
      }
    }
  }
  FLOG_CROSS << " sd_top_2_lane_dis: " << sd_top_2_lane_dis << " , start_index: " << start_index;
  // 求路口后历史预推线到左边路沿的距离
  bool   has_top_left_min_dis = false;
  double top_left_min_dis     = std::numeric_limits<double>::max();
  if (!left_dis.empty() && start_index >= 0) {
    int index     = std::max(0, start_index - 1);
    int end_index = std::min(static_cast<int>(left_dis.size()), index + 4);
    for (int i = index; i < end_index; i++) {
      if (left_dis[i].val && left_dis[i].dis < top_left_min_dis) {
        top_left_min_dis     = left_dis[i].dis;
        has_top_left_min_dis = true;
      }
    }
  }
  // 求路口后历史预推线到右边路沿的距离
  bool   has_top_right_min_dis = false;
  double top_right_min_dis     = std::numeric_limits<double>::max();
  if (!right_dis.empty() && start_index >= 0) {
    int index     = std::max(0, start_index - 1);
    int end_index = std::min(static_cast<int>(right_dis.size()), index + 4);
    for (int i = index; i < end_index; i++) {
      if (right_dis[i].val && right_dis[i].dis < top_right_min_dis) {
        top_right_min_dis     = right_dis[i].dis;
        has_top_right_min_dis = true;
      }
    }
  }
  FLOG_CROSS << "has_top_right_min_dis: " << has_top_right_min_dis << " , top_right_min_dis: " << top_right_min_dis
             << " ,has_top_left_min_dis: " << has_top_left_min_dis << " ,top_left_min_dis: " << top_left_min_dis;
  // 求距离左边界的目标距离
  // 求距离右边界的目标距离
  constexpr double default_lane_width    = 3.5;
  double           target_left_edge_dis  = std::numeric_limits<double>::max();
  double           target_right_edge_dis = std::numeric_limits<double>::max();
  int              left_lane_num         = 0;
  int              right_lane_num        = std::max(sd_section.current_lane_num, 1);
  // if (sd_section.current_sd_dirction_ == cem::message::env_model::SDDirectionType::BIDIRECTIONAL_PASSABLE) {
  //   left_lane_num  = std::max(static_cast<int>(sd_section.current_lane_num > 1), 1);
  //   right_lane_num = std::max(left_lane_num, 1);
  // } else {
  //   right_lane_num = sd_section.current_lane_num;
  // }

  // double total_road_width = (left_lane_num + right_lane_num) * default_lane_width;
  // if (top_right_min_dis > total_road_width) {
  //   has_top_right_min_dis = false;
  // }
  // if (top_left_min_dis > total_road_width) {
  //   has_top_left_min_dis = false;
  // }
  bool   has_top_boundary_width{false};
  double top_boundary_width = 0;
  if (has_top_right_min_dis && has_top_left_min_dis) {
    top_boundary_width = top_right_min_dis + top_left_min_dis;
  }

  // 车道选择--index从左往右0~n
  int target_top_lane_index = 0;
  if (data_manager_->turn_type() == BevAction::LEFT_TURN) {
    // if (data_manager_->GetEgoRoadLanes().left_same_turn_size == 0 || right_lane_num == 1) {
    //   target_top_lane_index = 0;
    // } else {
    //   target_top_lane_index = std::min(right_lane_num - 1, data_manager_->GetEgoRoadLanes().left_same_turn_size);
    // }
    // 暂定逻辑优先走左二车道
    if (right_lane_num == 1) {
      target_top_lane_index = 0;
    } else if (data_manager_->GetEgoRoadLanes().is_one_lane) {
      target_top_lane_index = std::min(right_lane_num - 2, 1);
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size == 0 && data_manager_->GetEgoRoadLanes().left_same_turn_size != 0) {
      target_top_lane_index = std::min(right_lane_num - 1, data_manager_->GetEgoRoadLanes().left_same_turn_size);
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size != 0 && data_manager_->GetEgoRoadLanes().left_same_turn_size == 0) {
      target_top_lane_index = 0;
    }
  } else if (data_manager_->turn_type() == BevAction::RIGHT_TURN) {
    if (right_lane_num == 1) {
      target_top_lane_index = 0;
    } else if (data_manager_->GetEgoRoadLanes().right_same_turn_size == 0) {
      target_top_lane_index = right_lane_num - 1;
    } else {
      target_top_lane_index = std::max(0, right_lane_num - data_manager_->GetEgoRoadLanes().right_same_turn_size - 1);
    }
  }
  FLOG_CROSS << " left_top_lane_num: " << left_lane_num << " ,right_top_lane_num: " << right_lane_num
             << " ,target_top_lane_index: " << target_top_lane_index;
  // 跟据选择的车道计算左右目标距离，距离都会偏大设置
  target_right_edge_dis = (right_lane_num - target_top_lane_index) * default_lane_width - 1.5;
  target_left_edge_dis  = (left_lane_num + target_top_lane_index + 1) * default_lane_width - 1.5;

  // 左右侧距离合理性检测
  target_right_edge_dis = std::max(target_right_edge_dis, 3.0);
  target_left_edge_dis  = std::max(target_left_edge_dis, 3.0);
  if (has_top_boundary_width) {
    target_right_edge_dis = std::min(target_right_edge_dis, top_boundary_width - 1.5);
    target_left_edge_dis  = std::min(target_left_edge_dis, top_boundary_width - 1.5);
  }

  // 位置合理性判断
  constexpr double move_step = 0.5;
  if (!has_top_right_min_dis) {
    if (has_left_edge_collision && has_right_edge_collision) {
      FLOG_CROSS << "has double edge collision ";
      return !data_manager_->GetIsInCrossroad();
    } else if (has_left_edge_collision) {
      move_dis = -move_step;
    } else if (has_right_edge_collision) {
      move_dis = move_step;
    } else {
      FLOG_CROSS << "has no top_min_dis ";
      return !data_manager_->GetIsInCrossroad();
    }
  } else if (has_top_right_min_dis) {
    if (top_right_min_dis > target_right_edge_dis - 0.8 && top_right_min_dis < target_right_edge_dis + 0.8) {
      FLOG_CROSS << "top_right_min_dis fit";
      return !data_manager_->GetIsInCrossroad();
    } else if (top_right_min_dis > target_right_edge_dis + 0.8) {
      if (data_manager_->GetIsInCrossroad()) {
        move_dis = -move_step;
      } else {
        // move_dis = target_right_edge_dis - top_right_min_dis;
        move_dis = -move_step * 2;
      }
    } else if (top_right_min_dis < target_right_edge_dis - 0.8) {
      if (data_manager_->GetIsInCrossroad()) {
        move_dis = move_step;
      } else {
        // move_dis = target_right_edge_dis - top_right_min_dis;
        move_dis = 2 * move_step;
      }
    }
  } 
  // else if (has_top_left_min_dis) {
  //   if (top_left_min_dis > target_left_edge_dis - 0.8 && top_left_min_dis < target_left_edge_dis + 0.8) {
  //     return false;
  //   } else if (top_left_min_dis > target_left_edge_dis + 0.8) {
  //     if (data_manager_->GetIsInCrossroad()) {
  //       move_dis = move_step;
  //     } else {
  //       // move_dis = top_left_min_dis - target_left_edge_dis;
  //       move_dis = move_step * 2;
  //     }
  //   } else if (top_left_min_dis < target_left_edge_dis - 0.8) {
  //     if (data_manager_->GetIsInCrossroad()) {
  //       move_dis = -move_step;
  //     } else {
  //       // move_dis = top_left_min_dis - target_left_edge_dis;
  //       move_dis = -move_step * 2;
  //     }
  //   }
  // }
  FLOG_CROSS << "before move_dis: " << move_dis;
  move_dis = std::abs(move_dis) > 0.2 ? move_dis - sd_top_2_lane_dis : 0.0;
  FLOG_CROSS << " target_right_edge_dis: " << target_right_edge_dis << " ,target_left_edge_dis: " << target_left_edge_dis
             << " ,move_dis: " << move_dis;
  return true;
}

bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePointsByObs(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return false;
}
bool VirtualEgoRoadProcessor::GenerateVirtualLRTurnLinePointsByHeading(std::vector<Point2DF> &virtual_line_points, Curve ego_curve) {
  return false;
}
bool VirtualEgoRoadProcessor::GenerateLeftTurnLane() {
  FLOG_CROSS << " GenerateLeftTurnLane is_first_generate: " << is_first_generate_;
  if (is_first_generate_) {
    if (!data_manager_->has_ego_lane() || data_manager_->EgoLane()->line_points.size() < 3) {
      return false;
    }
    Curve                 curve;
    auto                  size = data_manager_->EgoLane()->line_points.size();
    std::vector<Point2DF> fit_points(
        ((size > 25) ? data_manager_->EgoLane()->line_points.begin() + (size - 25) : data_manager_->EgoLane()->line_points.begin()),
        data_manager_->EgoLane()->line_points.end());

    bool valid = FitterLeastSquareMethod(fit_points, 1, curve);
    if (!valid) {
      return false;
    }
    // 拟合成功后才更新ego_lane_
    ego_lane_ = *data_manager_->EgoLane();
    // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
    dr_ego_lane_ = ego_lane_;
    for (auto &point : dr_ego_lane_.line_points) {
      TransformPoint(&point, data_manager_->Twb());
      // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
    }
    NewMainLane(curve.c0, curve.c1, fit_points, ego_lane_.width / 2, ego_lane_.id);
    if (!GenerateTurnLaneAndLaneMarker()) {
      return false;
    }
    is_first_generate_ = false;
    return true;
  }

  // 没有ego_lane或者ego_lane在车后的情况下，直接使用历史值
  if (!data_manager_->has_ego_lane() || (data_manager_->has_ego_lane() && data_manager_->EgoLane()->line_points.size() > 2 &&
                                         data_manager_->EgoLane()->line_points.back().x < 1.0)) {
    UpdateTurnMainLane();
    return false;
  }
  // 在ego_lane后方生成40m的VirtualLane
  Curve                 curve;
  auto                  size = data_manager_->EgoLane()->line_points.size();
  std::vector<Point2DF> fit_points(
      ((size > 25) ? data_manager_->EgoLane()->line_points.begin() + (size - 25) : data_manager_->EgoLane()->line_points.begin()),
      data_manager_->EgoLane()->line_points.end());
  bool valid = FitterLeastSquareMethod(fit_points, 1, curve);

  // 第一次拟合失败返回false，后面拟合失败使用历史的ego_lane_
  if (!valid) {
    UpdateTurnMainLane();
    return false;
  }

  // 拟合成功后才更新ego_lane_
  ego_lane_ = *data_manager_->EgoLane();
  // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
  dr_ego_lane_ = ego_lane_;
  for (auto &point : dr_ego_lane_.line_points) {
    TransformPoint(&point, data_manager_->Twb());
    // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
  }
  NewMainLane(curve.c0, curve.c1, fit_points, ego_lane_.width / 2, ego_lane_.id);
  if (!GenerateTurnLaneAndLaneMarker()) {
    return false;
  }
  return true;
}

bool VirtualEgoRoadProcessor::GenerateAroundTurnLaneAndLaneMarker() {
  FLOG_CROSS << "GenerateTurnLaneAndLaneMarker";
  auto &main_lane  = data_manager_->GetMainLane();
  auto &sd_section = data_manager_->GetSdSections();

  auto top_lane = GenerateAroundTurnTopLanePoints(sd_section.end_point, sd_section.top_angle(), ego_lane_.width / 2);
  // 连接中心线
  bool               is_lane_valid{false};
  int                bot_lane_index = -1;
  std::vector<Vec2d> control_points = {};
  int                top_index      = -1;
  BevLaneInfo        virtual_lane;
  BevLaneMarker      virtual_left_lanemarker;
  BevLaneMarker      virtual_right_lanemarker;
  for (const auto &point : main_lane.points) {
    FLOG_CROSS << " input main_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  for (const auto &point : top_lane.points) {
    FLOG_CROSS << " input top_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  if (InsertBezierPoints(main_lane.points, top_lane.points, control_points, bot_lane_index, top_index)) {
    FLOG_CROSS << " bot_lane_index: " << bot_lane_index << " ,top_index: " << top_index
               << ", control_points.size: " << control_points.size();
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
      }
      control_points.emplace_back(top_lane.points.back());
      virtual_lane_.id = kPreLaneId;
      // virtual_lane.id                   = data_manager_->GetNewLandId();
      virtual_lane.left_lane_marker_id  = kPreLeftLanemarkerId;
      virtual_lane.right_lane_marker_id = kPreRightLanemarkerId;
      virtual_lane.previous_lane_ids.emplace_back(main_lane.lane_id);
      virtual_lane.line_points.reserve(control_points.size());
      virtual_lane.is_virtual  = true;
      virtual_lane.navi_action = main_lane.navi_action;
      for (auto &pt : control_points) {
        Point2DF point;
        point.x = pt.x();
        point.y = pt.y();
        FLOG_CROSS << " BezierPoints X: " << point.x << " ,y: " << point.y;
        point.mse          = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
        virtual_lane.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
      }
      //
      is_lane_valid = true;
    }
  }
  if (is_lane_valid) {
    virtual_lane_ = virtual_lane;
    FLOG_CROSS << "GenerateTurnLaneAndLaneMarker success lane size: " << virtual_lane_.line_points.size();
    return true;
  }
  FLOG_CROSS << "GenerateTurnLaneAndLaneMarker is_lane_valid: " << is_lane_valid;
  return false;
}

TopLane VirtualEgoRoadProcessor::GenerateAroundTurnTopLanePoints(const Vec2d &point, double heading, double half_width) {
  TopLane top_lane;
  top_lane.valid = true;

  double x = point.x() + 35.0;
  double y = point.y() - std::tan(heading) * 35.0;
  top_lane.points.emplace_back(Vec2d(x, y));
  top_lane.points.emplace_back(point);
  // for (const auto &point : top_lane.points) {
  //   FLOG_CROSS << "TopLanePoint x: " << point.x() << " ,y: " << point.y();
  // }
  // 生成左边点
  Vec2d axis = Vec2d::CreateUnitVec2d(heading);
  axis.SelfRotate(pi / 2);
  top_lane.left_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, half_width);
  // 生成右边点
  top_lane.right_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, -half_width);
  FLOG_CROSS << "toplane half_width: " << half_width << " ,heading:" << heading << " ,top_lane point_1_x: " << top_lane.points.front().x()
             << " ,y: " << top_lane.points.front().y() << " top_left point_1_x: " << top_lane.left_points.front().x()
             << " ,y: " << top_lane.left_points.front().y() << " top_right point_1_x: " << top_lane.right_points.front().x()
             << " ,y: " << top_lane.right_points.front().y();
  return top_lane;
}

bool VirtualEgoRoadProcessor::GenerateTurnLaneAndLaneMarker() {
  FLOG_CROSS << "GenerateTurnLaneAndLaneMarker";
  auto &  main_lane  = data_manager_->GetMainLane();
  auto &  sd_section = data_manager_->GetSdSections();
  TopLane top_lane;
  if (sd_section.top_points.size() > 1) {
    top_lane.points.emplace_back(sd_section.top_points[0].x(), sd_section.top_points[0].y());
    top_lane.points.emplace_back(sd_section.top_points[1].x(), sd_section.top_points[1].y());
    AINFO << "USE SD";
  } else {
    top_lane = GenerateTopLanePoints(sd_section.end_point, sd_section.top_angle(), ego_lane_.width / 2);
    AINFO << "USE CORSS PLANE";
  }
  // 连接中心线
  bool               is_lane_valid{false};
  int                bot_lane_index = -1;
  std::vector<Vec2d> control_points = {};
  int                top_index      = -1;
  BevLaneInfo        virtual_lane;
  BevLaneMarker      virtual_left_lanemarker;
  BevLaneMarker      virtual_right_lanemarker;
  for (const auto &point : main_lane.points) {
    FLOG_CROSS << " input main_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  for (const auto &point : top_lane.points) {
    FLOG_CROSS << " input top_lane Points X: " << point.x() << " ,y: " << point.y();
  }
  if (InsertBezierPoints(main_lane.points, top_lane.points, control_points, bot_lane_index, top_index)) {
    FLOG_CROSS << " bot_lane_index: " << bot_lane_index << " ,top_index: " << top_index
               << ", control_points.size: " << control_points.size();
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.points.begin(), main_lane.points.end() - 1);
      }
      control_points.emplace_back(top_lane.points.back());
      virtual_lane_.id = kPreLaneId;
      // virtual_lane.id                   = data_manager_->GetNewLandId();
      virtual_lane.left_lane_marker_id  = kPreLeftLanemarkerId;
      virtual_lane.right_lane_marker_id = kPreRightLanemarkerId;
      virtual_lane.previous_lane_ids.emplace_back(main_lane.lane_id);
      virtual_lane.line_points.reserve(control_points.size());
      virtual_lane.is_virtual  = true;
      virtual_lane.navi_action = main_lane.navi_action;
      for (auto &pt : control_points) {
        Point2DF point;
        point.x            = pt.x();
        point.y            = pt.y();
        point.mse          = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
        virtual_lane.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
      }
      //
      is_lane_valid = true;
    }
  }

  // 连接左边车道线
  bool is_left_valid{false};
  bot_lane_index = -1;
  top_index      = -1;
  control_points.clear();
  if (InsertBezierPoints(main_lane.left_points, top_lane.left_points, control_points, bot_lane_index, top_index)) {
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.left_points.begin(), main_lane.left_points.end());
      }
      control_points.emplace_back(top_lane.left_points.back());
      virtual_left_lanemarker.id         = kConnectLeftLanemarkerId;
      virtual_left_lanemarker.is_virtual = true;
      virtual_left_lanemarker.line_points.reserve(control_points.size());
      for (auto &pt : control_points) {
        Point2DF point;
        point.x            = pt.x();
        point.y            = pt.y();
        point.mse          = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
        virtual_left_lanemarker.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
      }
      //
      is_left_valid = true;
    }
  }
  //连接右边车道线
  bool is_right_valid{false};
  bot_lane_index = -1;
  top_index      = -1;
  control_points.clear();
  if (InsertBezierPoints(main_lane.right_points, top_lane.right_points, control_points, bot_lane_index, top_index)) {
    if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
      // 需要判断main_lane用于连接的线是原始车道线还是virtual线，判断是否需要使用
      if (main_lane.is_virtual) {
        control_points.insert(control_points.begin(), main_lane.right_points.begin(), main_lane.right_points.end());
      }
      control_points.emplace_back(top_lane.right_points.back());
      virtual_right_lanemarker.id         = kConnectRightLanemarkerId;
      virtual_right_lanemarker.is_virtual = true;
      virtual_right_lanemarker.line_points.reserve(control_points.size());
      for (auto &pt : control_points) {
        Point2DF point;
        point.x            = pt.x();
        point.y            = pt.y();
        point.mse          = NAN;
        point.point_source = cem::message::common::PointSource::TOPO_INSETRT;

        virtual_right_lanemarker.line_points.emplace_back(TransformPoint(point, data_manager_->Twb()));
      }
      //
      is_right_valid = true;
    }
  }

  // if (is_lane_valid && is_left_valid && is_right_valid) {
  if (is_lane_valid) {
    virtual_lane_             = virtual_lane;
    virtual_left_lanemarker_  = virtual_left_lanemarker;
    virtual_right_lanemarker_ = virtual_right_lanemarker;
    AINFO << "virtual_lane_ SET";
    return true;
  }
  return false;
}

TopLane VirtualEgoRoadProcessor::GenerateTopLanePoints(const Vec2d &point, double heading, double half_width) {
  TopLane top_lane;
  top_lane.valid = true;

  double x = point.x() - 15.0 * std::tan(heading - pi / 2) * (-1);
  double y = point.y() - 15.0;
  top_lane.points.emplace_back(Vec2d(x, y));
  top_lane.points.emplace_back(point);
  // for (const auto &point : top_lane.points) {
  //   FLOG_CROSS << "TopLanePoint x: " << point.x() << " ,y: " << point.y();
  // }
  // 生成左边点
  Vec2d axis = Vec2d::CreateUnitVec2d(heading);
  axis.SelfRotate(pi / 2);
  top_lane.left_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, half_width);
  // 生成右边点
  top_lane.right_points = TranslateSegmentAlongAxisStart(axis, top_lane.points, -half_width);
  FLOG_CROSS << "toplane half_width: " << half_width << " ,heading:" << heading << " ,top_lane point_1_x: " << top_lane.points.front().x()
             << " ,y: " << top_lane.points.front().y() << " top_left point_1_x: " << top_lane.left_points.front().x()
             << " ,y: " << top_lane.left_points.front().y() << " top_right point_1_x: " << top_lane.right_points.front().x()
             << " ,y: " << top_lane.right_points.front().y();
  return top_lane;
}

void VirtualEgoRoadProcessor::UpdateTurnMainLane() {
  UpdateVirtualMainLane();
}

bool VirtualEgoRoadProcessor::GenerateRightTurnLane() {
  return false;
}

bool VirtualEgoRoadProcessor::GenerateAroundTurnLane() {
  FLOG_CROSS << " GenerateLeftTurnLane is_generate_success_: " << is_generate_success_;
  // 1.没成功前没有ego_lane返回false
  if (!data_manager_->has_ego_lane() && !is_generate_success_) {
    return false;
  }

  // 2.拟合ego_lane
  Curve curve;
  fitter_egolane_success_ = FitterEgelane(curve);

  // 3.生成/更新mainlane
  if (fitter_egolane_success_) {
    NewMainLane(curve.c0, curve.c1, data_manager_->EgoLane()->line_points, data_manager_->EgoLane()->width / 2,
                data_manager_->EgoLane()->id, 35.0);
  } else {
    UpdateVirtualMainLane();
  }

  // 4.路口前拟合失败返回,使用历史值
  if (!fitter_egolane_success_ && !data_manager_->GetIsInCrossroad()) {
    return false;
  }

  // 5.生成虚拟lane，并转成dr坐标系，后续可以一直使用，中间只需要更新MainLane用于连接使用
  std::vector<Point2DF> virtual_line_points;
  if (!GenerateVirtualAroundTurnLinePoints(virtual_line_points, curve)) {
    FLOG_CROSS << " can not generate  virtual_line_points!!!";
    return false;
  }
  is_generate_success_ = true;
  // 6.路口前生成了virtual_line_points就更新ego_lane_和dr_ego_lane_
  if (!data_manager_->GetIsInCrossroad()) {
    ego_lane_ = *data_manager_->EgoLane();
    // 将ego_lane_转换到dr下存储起来,当预推后没ego lane或者拟合失败的时候使用
    dr_ego_lane_ = ego_lane_;
    for (auto &point : dr_ego_lane_.line_points) {
      TransformPoint(&point, data_manager_->Twb());
      // FLOG_CROSS << " dr point x: " << point.x << " ,y: " << point.y;
    }
  }

  // 7.生成lanemarker
  GenerateVirtualLaneAndLaneMarker(virtual_line_points);
  virtual_lane_.plan_turn_type = BevTurnType::U_TURN;
  virtual_lane_.navi_action    = BevAction::U_TURN;
  virtual_lane_.bev_turn_type    = BevAction::U_TURN;
  return true;
}

bool VirtualEgoRoadProcessor::GenerateVirtualStraightPoints(std::vector<Point2DF> &virtual_line_points, Point2DF start_point,
                                                            Curve ego_curve) {

  constexpr double kPredictMaxDis = 80.0;
  const auto &     opening        = data_manager_->GetOpening();
  if (opening.has_heading && opening.dis > 20.0 && std::abs(opening.heading) > 0.1) {
    double           top_lane_dis = opening.dis - start_point.x;
    double           ego_heading  = std::atan(ego_curve.c1);
    double           heading_diff = opening.heading - ego_heading;
    double           radius       = top_lane_dis / std::sin(std::abs(heading_diff));
    constexpr double delta_length = 1.0;
    double           delta_angle  = delta_length / radius;
    int              phi_num      = std::floor(std::abs(heading_diff) / delta_angle);
    double           x{0.0};
    double           y{0.0};
    double           total_length{0.0};
    double           phi{0.0};
    FLOG_CROSS << " radius: " << radius << " ,phi_num: " << phi_num << " ,opening.dis: " << opening.dis
               << " ,start_point.x: " << start_point.x;
    if (radius <= 50.0 || radius >= 3000) {
      virtual_line_points.reserve(61);
      for (int i = 1; i < 61; i++) {
        double x = start_point.x + i;
        double y = ego_curve.c0 + ego_curve.c1 * x;
        // FLOG_CROSS << "virtual_line_points x: " << x << " ,y: " << y;
        virtual_line_points.emplace_back(Point2DF(x, y));
      }
      return true;
    }

    if (radius < 3000) {
      for (int i = 0; i < phi_num + 1; i++) {
        total_length += delta_length;
        phi       = i * delta_angle;
        double dx = radius * std::sin(phi);
        double dy = std::copysign(radius * (1 - std::cos(phi)), heading_diff);
        x         = start_point.x + dx * std::cos(ego_heading) - dy * std::sin(ego_heading);
        y         = start_point.y + dx * std::sin(ego_heading) + dy * std::cos(ego_heading);
        virtual_line_points.emplace_back(x, y);
      }
      int      index{0};
      Point2DF pre_point = virtual_line_points.at(virtual_line_points.size() - 2);
      Point2DF now_point = virtual_line_points.at(virtual_line_points.size() - 1);
      double   delta_x   = now_point.x - pre_point.x;
      double   delta_y   = now_point.y - pre_point.y;
      while (total_length < kPredictMaxDis && index < 200) {
        index++;
        total_length += delta_length;
        x = virtual_line_points.back().x + delta_x;
        y = virtual_line_points.back().y + delta_y;
        virtual_line_points.emplace_back(x, y);
      }
    }
    // else {
    //   int index{0};
    //   virtual_line_points.emplace_back(start_point);
    //   while (total_length < kPredictMaxDis && index < 200) {
    //     index++;
    //     total_length += delta_length;
    //     x = virtual_line_points.back().x + delta_length;
    //     y = virtual_line_points.back().y + delta_length * std::tan(heading_diff);
    //     virtual_line_points.emplace_back(x, y);
    //   }
    // }
  } else {
    virtual_line_points.reserve(61);
    for (int i = 1; i < 61; i++) {
      double x = start_point.x + i;
      double y = ego_curve.c0 + ego_curve.c1 * x;
      // FLOG_CROSS << "virtual_line_points x: " << x << " ,y: " << y;
      virtual_line_points.emplace_back(Point2DF(x, y));
    }
  }
  return true;
}

}  // namespace fusion
}  // namespace cem