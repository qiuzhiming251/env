#include "crossroad_lane_selector.h"

namespace cem {
namespace fusion {

CrossRoadSelector::CrossRoadSelector() {}
bool CrossRoadSelector::Process(const cem::message::env_model::LaneInfo *lane, std::vector<Point2DF> &ld_lane_points) {
  // lane已经检查不会为空指针，先寻找路口虚拟线及虚拟线前的egolane
  FLOG_CROSS << " CrossRoadSelector before egolane_id: " << lane->id;
  uint64_t egolane_id{0};
  if (!FindRoutingMapVirtualLane(lane, egolane_id)) {
    FLOG_CROSS << " find no egolane!!!";
    return false;
  }
  FLOG_CROSS << " CrossRoadSelector after egolane_id: " << egolane_id;
  auto *ego_lane = data_manager_->FindRoutingMapLaneById(egolane_id);
  if (ego_lane == nullptr || ego_lane->points.size() < 2 || ego_lane->next_lane_ids.empty()) {
    FLOG_CROSS << " ego_lane->points.size() < 2 || ego_lane->next_lane_ids.empty()!!";
    return false;
  }
  // 求取egolane的heading，index及左右车道数
  Vec2d                          egolane_point{};
  const auto &                   ego_end_p = TransformPoint(ego_lane->points.back(), data_manager_->Tbw());
  cem::message::env_model::Point ego_start_p;
  for (int i = ego_lane->points.size() - 2; i >= 0; i--) {
    const auto &i_p = TransformPoint(ego_lane->points[i], data_manager_->Tbw());
    double      dis = (i_p.x - ego_end_p.x) * (i_p.x - ego_end_p.x) + (i_p.y - ego_end_p.y) * (i_p.y - ego_end_p.y);
    if (dis > 100.0 || i == 0) {
      ego_start_p   = i_p;
      egolane_point = Vec2d(ego_end_p.x - i_p.x, ego_end_p.y - i_p.y);
      break;
    }
  }
  FLOG_CROSS << " ego_lane->next_lane_ids: " << ego_lane->next_lane_ids.size();
  // 候选中心线
  std::vector<CrossRoadCenterBackupLane> ld_backup_lanes;
  double after_heading{0.0};
  for (auto virtual_id : ego_lane->next_lane_ids) {
    auto *virtual_lane = data_manager_->FindRoutingMapLaneById(virtual_id);
    if (virtual_lane == nullptr || virtual_lane->points.size() < 2 || !virtual_lane->is_virtual ||
        !HasSameTurnType(virtual_lane->turn_type, data_manager_->turn_type()) || virtual_lane->next_lane_ids.empty()) {
      continue;
    }
    FLOG_CROSS << " virtual_lane: " << virtual_lane->id;
    auto *next_lane = data_manager_->FindRoutingMapLaneById(virtual_lane->next_lane_ids.front());
    if (next_lane == nullptr || next_lane->points.size() < 2) {
      continue;
    }
    auto &ld_backup_lane = ld_backup_lanes.emplace_back();
    ld_backup_lane.id    = virtual_lane->id;
    for (const auto &p : virtual_lane->points) {
      auto ego_point = TransformPoint(p, data_manager_->Tbw());
      ld_backup_lane.points.emplace_back(ego_point.x, ego_point.y);
    }
    ld_backup_lane.next_id = virtual_lane->next_lane_ids.front();
    ld_backup_lane.heading = Vec2d(ld_backup_lane.points.back().x - ld_backup_lane.points.front().x,
                                   ld_backup_lane.points.back().y - ld_backup_lane.points.front().y)
                                 .Angle();
    const auto &next_begin_p = TransformPoint(next_lane->points.front(), data_manager_->Tbw());
    for (int i = next_lane->points.size() - 2; i >= 0; i--) {
      const auto &i_p = TransformPoint(next_lane->points[i], data_manager_->Tbw());
      double      dis = (i_p.x - next_begin_p.x) * (i_p.x - next_begin_p.x) + (i_p.y - next_begin_p.y) * (i_p.y - next_begin_p.y);
      if (dis > 10.0 || i == 0) {
        ld_backup_lane.next_lane_heading = Vec2d(i_p.x - next_begin_p.x, i_p.y - next_begin_p.y).Angle();
        break;
      }
    }
    // 求取横向偏差
    ld_backup_lane.lat_err =
        PointToVectorDist(Vec2d(ego_start_p.x, ego_start_p.y), Vec2d(ego_end_p.x, ego_end_p.y), Point2DF(next_begin_p.x, next_begin_p.y));
    ld_backup_lane.distance = Vec2d(ld_backup_lane.points.back().x - ld_backup_lane.points.front().x,
                                    ld_backup_lane.points.back().y - ld_backup_lane.points.front().y)
                                  .Length();
  }
  // 如果后继虚拟线只有一个直接选择
  if(ld_backup_lanes.empty()){
    FLOG_CROSS << " ld_backup_lanes.empty!!";
    return false;
  }

  // 求平均路口后的heading
  for (int i = 0; i < ld_backup_lanes.size(); i++) {
    after_heading += ld_backup_lanes[i].next_lane_heading;
  }
  after_heading /= ld_backup_lanes.size();
  // double heading_err = std::abs(egolane_point.Angle() - after_heading);
  // FLOG_CROSS << " heading_err: " << heading_err;
  double fit_index{0};
  if (data_manager_->turn_type() == BevAction::U_TURN) {
    // 掉头寻找最近的dis
    double min_lat_err = DBL_MAX;
    for (int i = 0; i < ld_backup_lanes.size(); i++) {
      if (Cross_Compare(ld_backup_lanes[i].distance, min_lat_err) < 0) {
        min_lat_err = ld_backup_lanes[i].distance;
        fit_index   = i;
      }
    }
  }
  // else if (heading_err < 0.05) {
  //   // 小角度遍历寻找最小的lat err
  //   double min_lat_err = DBL_MAX;
  //   for (int i = 0; i < ld_backup_lanes.size(); i++) {
  //     if (Cross_Compare(ld_backup_lanes[i].lat_err, min_lat_err) < 0) {
  //       min_lat_err = ld_backup_lanes[i].lat_err;
  //       fit_index   = i;
  //     }
  //   }
  // }
  else {
    // 大角度遍历寻找最小的heading err
    double min_heading_err = DBL_MAX;
    for (int i = 0; i < ld_backup_lanes.size(); i++) {
      FLOG_CROSS << "back_up_lane_id: " << ld_backup_lanes[i].id << " ,egolane_point_angle: " << egolane_point.Angle()
                 << " ,heading: " << ld_backup_lanes[i].heading << " ,opending_heading: " << ld_backup_lanes[i].next_lane_heading;
      double heading_err = std::abs(ld_backup_lanes[i].heading - egolane_point.Angle()) +
                           std::abs(ld_backup_lanes[i].next_lane_heading - ld_backup_lanes[i].heading);
      if (Cross_Compare(heading_err, min_heading_err) < 0) {
        min_heading_err = heading_err;
        fit_index       = i;
      }
    }
  }
  // 根据选择的线在加延长的30m
  constexpr double max_add_length = 30.0;
  ld_lane_points                  = ld_backup_lanes[fit_index].points;
  AddLaneLength(ld_backup_lanes[fit_index].next_id, max_add_length, ld_lane_points);

  return true;
  // 如果后继虚拟线有多个需要选择
  // 求路口前egolane的index，路口数以及
}

void CrossRoadSelector::AddLaneLength(uint64_t next_id,double max_add_length, std::vector<Point2DF> &ld_lane_points) {
  auto *next_lane = data_manager_->FindRoutingMapLaneById(next_id);
  if (next_lane == nullptr || next_lane->points.empty()) {
    return;
  }
  double           length         = 0;
  int              index{0};
  while (length < max_add_length && index < 100) {
    if (next_lane == nullptr || next_lane->points.empty()) {
      break;
    }
    for (const auto &point : next_lane->points) {
      auto   ego_point = TransformPoint(point, data_manager_->Tbw());
      double delta_dis = std::hypot(ld_lane_points.back().x - ego_point.x, ld_lane_points.back().y - ego_point.y);
      if(delta_dis < 0.1){
        continue;
      }
      ld_lane_points.emplace_back(ego_point.x, ego_point.y);
      if (ld_lane_points.size() > 1) {
        length += delta_dis;
      }
      if (length > max_add_length) {
        break;
      }
    }
    if (length > max_add_length || next_lane->next_lane_ids.size() != 1) {
      break;
    }
    next_lane = data_manager_->FindRoutingMapLaneById(next_lane->next_lane_ids.front());
  }
  return;
}

bool CrossRoadSelector::FindRoutingMapVirtualLane(const cem::message::env_model::LaneInfo* lane,uint64_t& egolane_id) {
  if (data_manager_->routing_map() == nullptr) {
    FLOG_CROSS << " data_manager_->routing_map() == nullptr";
    return false;
  }

  if (lane == nullptr || lane->points.empty()) {
    FLOG_CROSS << " lane == nullptr or lane->points.empty";
    return false;
  }
  auto back_point = TransformPoint(lane->points.back(), data_manager_->Tbw());
  if (back_point.x < 0.0) {
    FLOG_CROSS << " lane->points.back().x < 0 ; x: " << back_point.x;
    return false;
  }
  FLOG_CROSS << " lane->points.back().x: " << back_point.x << " ,id: " << lane->id;
  double lane_length = back_point.x;
  if (lane_length > 80.0) {
    return false;
  }
  auto next_lane_id_size = lane->next_lane_ids.size();
  if (next_lane_id_size == 0) {
    FLOG_CROSS << " next_lane_id_size = 0";
    return false;
  }
  bool next_lane_is_virtual = false;
  bool next_lane_has_virtual = false;
  for (const auto id : lane->next_lane_ids) {
    const auto *next_lane = data_manager_->FindRoutingMapLaneById(id);
    if (next_lane == nullptr) {
      continue;
    }
    if(next_lane->is_virtual){
      next_lane_has_virtual = true;
    }
    FLOG_CROSS << " next_laneid: " << next_lane->id << " ,is_virtual: " << next_lane->is_virtual;
    if (next_lane->is_virtual && HasSameTurnType(next_lane->turn_type, data_manager_->turn_type())) {
      next_lane_is_virtual = true;
      egolane_id           = lane->id;
      break;
    }
  }
  FLOG_CROSS << " next_lane_id_size: " << next_lane_id_size << " ,next_lane_is_virtual: " << next_lane_is_virtual;
  if (next_lane_id_size > 1 && !next_lane_has_virtual) {
    FLOG_CROSS << " next_lane_id_size > 1 && !next_lane_is_virtual";
    return false;
  }
  if (next_lane_is_virtual) {
    return true;
  }

  int      index          = 0;
  uint64_t lane_id = lane->next_lane_ids.front();;
  if (next_lane_has_virtual) {
    for (const auto id : lane->next_lane_ids) {
      const auto *next_lane = data_manager_->FindRoutingMapLaneById(id);
      if (next_lane == nullptr) {
        continue;
      }
      if (!next_lane->is_virtual) {
        lane_id = id;
        break;
      }
    }
  }
  bool     has_is_virtual = false;
  while (lane_length < 80.0 && index < 100) {
    index++;
    const auto *lane = data_manager_->FindRoutingMapLaneById(lane_id);
    if (lane == nullptr) {
      break;
    }
    lane_length += lane->length;
    if (lane->next_lane_ids.empty() || lane_length > 80.0) {
      break;
    }

    next_lane_is_virtual = false;
    next_lane_has_virtual = false;
    for (const auto id : lane->next_lane_ids) {
      const auto *next_lane = data_manager_->FindRoutingMapLaneById(id);
      if (next_lane == nullptr) {
        continue;
      }
      if (next_lane->is_virtual) {
        next_lane_has_virtual = true;
      }
      if (next_lane->is_virtual && HasSameTurnType(next_lane->turn_type, data_manager_->turn_type())) {
        egolane_id = lane->id;
        next_lane_is_virtual = true;
      }
    }
    if (next_lane_is_virtual) {
      has_is_virtual = true;
      break;
    }
    if (lane->next_lane_ids.size() > 1 && !next_lane_has_virtual) {
      break;
    }
    lane_id = lane->next_lane_ids.front();
    if (next_lane_has_virtual) {
      for (const auto id : lane->next_lane_ids) {
        const auto *next_lane = data_manager_->FindRoutingMapLaneById(id);
        if (next_lane == nullptr) {
          continue;
        }
        if (!next_lane->is_virtual) {
          lane_id = id;
          break;
        }
      }
    }
  }
  FLOG_CROSS << " has_is_virtual: " << has_is_virtual;
  return has_is_virtual;
}
}  // namespace fusion
}  // namespace cem