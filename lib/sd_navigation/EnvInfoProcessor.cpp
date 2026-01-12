#include "EnvInfoProcessor.h"
#include <log_custom.h>
#include <fmt/format.h>

#include "lib/sd_navigation/routing_map_debug.h"

namespace cem {
namespace fusion {
namespace navigation {

EnvInfoProcessor::EnvInfoProcessor()
    : env_info_(std::make_unique<EnvInfo>()),
      is_switched_to_LD_(false),
      last_remain_dis_(-1.0),
      accumulated_distance_(0.0),
      last_in_ramp_(false),
      last_on_highway_(false),
      is_accumulated_valid_(false) {
  // is_entered_ramp_ = CheckIfInRamp(*INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr());
  // if (is_entered_ramp_) {
  //   is_switched_to_LD_ = true;  // 如果初始化时已在匝道内，设置标志位为 true
  // }
}

void EnvInfoProcessor::Process(const std::vector<cem::message::env_model::StopLine> &stop_line_obj) {
  auto *sensor_manager = SensorDataManager::Instance();
  if (!sensor_manager) {
    AWARN << "SensorDataManager instance is null";
    SD_ENV_INFO_LOG << fmt::format("SensorDataManager instance is null, exiting");
    return;
  }

  MapEventPtr map_event_ptr;
  sensor_manager->GetLatestSensorFrame(map_event_ptr);
  if (!map_event_ptr) {
    AWARN << "Failed to get latest sensor frame";
    SD_ENV_INFO_LOG << fmt::format("Failed to get latest MapEvent, exiting");
    return;
  }
  const auto &map_event = *map_event_ptr;

  // 获取 SDRouteInfo
  // if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
  //   AWARN << "[EnvInfoProcessor] SDRouteInfo is null";
  //   SD_ENV_INFO_LOG << fmt::format("SDRouteInfo is null, exiting");
  //   return;
  // }
  // const auto &sd_route = *INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  // SD_ENV_INFO_LOG << fmt::format("{}", sd_route);

  // 清空并重新初始化 env_info_
  // env_info_ = std::make_unique<EnvInfo>();
  env_info_->v2_road_classes.clear();
  env_info_->v2_turn_info.clear();
  env_info_->traffic_flows.clear();
  env_info_->v2_curvatures.clear();
  env_info_->v2_non_odd_info.clear();
  env_info_->switch_ld_reason.clear();

  // 从 MapEvent 提取字段
  env_info_->v2_valid          = (map_event.navi_stat != NaviStatus::NO_NAVAGATION);
  env_info_->v2_has_navigation = (map_event.navi_stat != NaviStatus::NO_NAVAGATION);
  SD_ENV_INFO_LOG << fmt::format("Set v2_valid: {}, v2_has_navigation: {}", env_info_->v2_valid, env_info_->v2_has_navigation);

  // 处理 v2_dist_to_ramp
  if (CheckIfInRamp()) {
    env_info_->v2_dist_to_ramp = 0.0;
    SD_ENV_INFO_LOG << fmt::format("Vehicle is in ramp, set v2_dist_to_ramp: 0.0");
  } else if (map_event.control_waypoint_type == ControlWayPoint::COMING_LEFT_ONTO_THE_RAMP ||
             map_event.control_waypoint_type == ControlWayPoint::COMING_RIGHT_ONTO_THE_RAMP) {
    env_info_->v2_dist_to_ramp = map_event.control_waypoint_dis;
    SD_ENV_INFO_LOG << fmt::format("Approaching ramp, set v2_dist_to_ramp: {}", env_info_->v2_dist_to_ramp);
  } else if (map_event.control_waypoint_type == ControlWayPoint::MERGE_LEFT_ONTO_THE_MAIN_ROAD ||
             map_event.control_waypoint_type == ControlWayPoint::MERGE_RIGHT_ONTO_THE_MAIN_ROAD) {
    env_info_->v2_dist_to_ramp = 0.0;
    SD_ENV_INFO_LOG << fmt::format("Approaching mainroad, set v2_dist_to_ramp: {}", env_info_->v2_dist_to_ramp);
  } else {
    env_info_->v2_dist_to_ramp = DEFAULT_LARGE_DISTANCE;
    SD_ENV_INFO_LOG << fmt::format("No ramp detected, set v2_dist_to_ramp: {}", DEFAULT_LARGE_DISTANCE);
  }

  // 处理 v2_dist_to_toll 和 v2_dist_to_tunnel
  env_info_->v2_dist_to_toll   = DEFAULT_LARGE_DISTANCE;
  env_info_->v2_dist_to_tunnel = DEFAULT_LARGE_DISTANCE;
  for (const auto &reminder_info : map_event.reminder_way_info) {
    if (reminder_info.reminder_waypoint_type == ReminderWayPoint::TOLL_BOOTH) {
      env_info_->v2_dist_to_toll = reminder_info.reminder_waypoint_dis;
      SD_ENV_INFO_LOG << fmt::format("Toll booth detected, set v2_dist_to_toll: {}", env_info_->v2_dist_to_toll);
    } else if (reminder_info.reminder_waypoint_type == ReminderWayPoint::TUNNEL) {
      env_info_->v2_dist_to_tunnel = reminder_info.reminder_waypoint_dis;
      SD_ENV_INFO_LOG << fmt::format("Tunnel detected, set v2_dist_to_tunnel: {}", env_info_->v2_dist_to_tunnel);
    }
  }

  // 处理 v2_curvatures
  env_info_->v2_curvatures.clear();
  for (const auto &curvature_info : map_event.curvature_info) {
    V2Curvature v2_curvature;
    v2_curvature.distance  = curvature_info.remain_dis;
    v2_curvature.curvature = curvature_info.curvature;
    env_info_->v2_curvatures.emplace_back(v2_curvature);
  }
  SD_ENV_INFO_LOG << fmt::format("Set v2_curvatures with size: {}", env_info_->v2_curvatures.size());

  // 处理 traffic_flows
  env_info_->traffic_flows.clear();
  V2TrafficFlow traffic_flow;
  traffic_flow.start_s = map_event.traffic_info.dist_to_start_traffic_jam;
  traffic_flow.end_s   = traffic_flow.start_s + map_event.traffic_info.traffic_jam_dist;
  switch (map_event.traffic_info.traffic_jam_status) {
    case 1:
      traffic_flow.type = V2TrafficFlow::V2TrafficFlowType::SMOOTH_FLOW;
      break;
    case 2:
      traffic_flow.type = V2TrafficFlow::V2TrafficFlowType::SLOW_FLOW;
      break;
    case 3:
      traffic_flow.type = V2TrafficFlow::V2TrafficFlowType::JAMMED_FLOW;
      break;
    case 4:
      traffic_flow.type = V2TrafficFlow::V2TrafficFlowType::SEVERE_JAMMED_FLOW;
      break;
    default:
      traffic_flow.type = V2TrafficFlow::V2TrafficFlowType::UNKNOWN_FLOW;
      break;
  }
  env_info_->traffic_flows.emplace_back(traffic_flow);
  SD_ENV_INFO_LOG << fmt::format("Set traffic_flows with size: {}", env_info_->traffic_flows.size());

  // 从 sd_route 计算 v2_dist_to_subpath 和 v2_dist_to_split_routelanenum_dec
  env_info_->v2_dist_to_subpath                = CalculateDistanceToSubpath();
  env_info_->v2_dist_to_split_routelanenum_dec = CalculateDistanceToSplitRouteLaneNumDec();
  SD_ENV_INFO_LOG << fmt::format("Set v2_dist_to_subpath: {}, v2_dist_to_split_routelanenum_dec: {}", env_info_->v2_dist_to_subpath,
                                 env_info_->v2_dist_to_split_routelanenum_dec);

  env_info_->v2_road_classes = CalculateV2RoadClasses();

  env_info_->v2_turn_info = CalculateV2TurnInfo(stop_line_obj);

  env_info_->is_switched_to_LD_ = UpdateIsSwitchedToLD(map_event);
  // SD_ENV_INFO_LOG << fmt::format(
  //     "Exiting ProcessEnvInfo with env_info: v2_valid={}, v2_has_navigation={}, v2_dist_to_ramp={}, v2_dist_to_toll={}, "
  //     "v2_dist_to_tunnel={}, v2_dist_to_subpath={}, v2_dist_to_split_routelanenum_dec={}, v2_curvatures_size={}, "
  //     "traffic_flows_size={}, is_switched_to_LD_={}",
  //     env_info_->v2_valid, env_info_->v2_has_navigation, env_info_->v2_dist_to_ramp, env_info_->v2_dist_to_toll,
  //     env_info_->v2_dist_to_tunnel, env_info_->v2_dist_to_subpath, env_info_->v2_dist_to_split_routelanenum_dec,
  //     env_info_->v2_curvatures.size(), env_info_->traffic_flows.size(), env_info_->is_switched_to_LD_);

  PrintEnvInfo(*env_info_);
}

bool EnvInfoProcessor::UpdateIsSwitchedToLD(const MapEvent &map_event) {
  //城区不更新is_switched_to_LD_高速切图标志位
  const bool on_highway = (env_info_ && env_info_->is_on_highway_);
  // if (!on_highway) {
  //   is_switched_to_LD_    = false;
  //   accumulated_distance_ = 0.0;
  //   return is_switched_to_LD_;
  // }

  ControlWayPoint control_type = map_event.control_waypoint_type;
  double          control_dis  = map_event.control_waypoint_dis;

  double remain_dis = -1.0;
  for (const auto &remain_info : map_event.remain_dis_info) {
    if (remain_info.remain_dis_type == RemainType::NAVI_MAP_MATCHED_END) {
      remain_dis = remain_info.remain_dis;
      break;
    }
  }

  SD_ENV_INFO_LOG << fmt::format("[UpdateIsSwitchedToLD:GetEgoGlobalS], last_ego_s_on_route_: {}", last_ego_s_on_route_);
  double ego_s_now = std::numeric_limits<double>::quiet_NaN();
  if (on_highway) {
    auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
    if (ld_route) {
      ego_s_now = GetEgoGlobalS(*ld_route);
    }
  } else {
    auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
    if (sd_route) {
      ego_s_now = GetEgoGlobalS(*sd_route);
    }
  }
  // // 场景切换时重置累计，避免跨场景的 ds 异常
  // if (on_highway != last_on_highway_) {
  //   last_ego_s_on_route_ = std::numeric_limits<double>::quiet_NaN();
  //   SD_ENV_INFO_LOG << fmt::format("Mode switched (HW:{} -> {}), reset last_ego_s_on_route_", last_on_highway_, on_highway);
  // }
  double ds = 0.0;
  if (std::isfinite(ego_s_now) && std::isfinite(last_ego_s_on_route_)) {
    ds = std::max(0.0, ego_s_now - last_ego_s_on_route_);
  }
  last_ego_s_on_route_ = ego_s_now;
  SD_ENV_INFO_LOG << fmt::format("[UpdateIsSwitchedToLD:GetEgoGlobalS], ego_s_now: {}", ego_s_now);

  bool approaching_mainroad =
      (control_type == ControlWayPoint::MERGE_LEFT_ONTO_THE_MAIN_ROAD || control_type == ControlWayPoint::MERGE_RIGHT_ONTO_THE_MAIN_ROAD);
  double v2_dist_to_mainroad = map_event.control_waypoint_dis;

  // 判断当前是否在闸道内
  const bool in_ramp           = CheckIfInRamp() || approaching_mainroad;
  const bool ramp_exit_edge    = (!in_ramp && last_in_ramp_);
  const bool highway_fall_edge = (!on_highway && last_on_highway_);
  SD_ENV_INFO_LOG << fmt::format("last_in_ramp_:{}, in_ramp:{}, is_accumulated_valid_:{}, accumulated_distance_: {:.2f}m, ds: {:.2f}m",
                                 last_in_ramp_, in_ramp, is_accumulated_valid_, accumulated_distance_, ds);

  // 判断是否即将进入闸道（控制点为进入闸道类型且距离小于 800 米）
  bool approaching_ramp =
      (control_type == ControlWayPoint::COMING_LEFT_ONTO_THE_RAMP || control_type == ControlWayPoint::COMING_RIGHT_ONTO_THE_RAMP) &&
      control_dis <= LD_MAP_ACTIVATION_DIST_THRESH;

  // 状态1：在闸道内，切换到 LD 模式
  if (in_ramp) {
    is_switched_to_LD_    = true;
    is_accumulated_valid_ = false;
    accumulated_distance_ = 0.0;
    last_in_ramp_         = true;
    last_remain_dis_      = remain_dis;
    last_on_highway_      = on_highway;
    env_info_->switch_ld_reason.push_back("In ramp → LD=ON");
    SD_ENV_INFO_LOG << fmt::format("Vehicle is in ramp, set is_switched_to_LD_: true");
    return is_switched_to_LD_;
  }
  // 状态2：不在闸道内，即将进入闸道（控制点距离 < 800 米）
  if (approaching_ramp) {
    is_switched_to_LD_    = true;
    is_accumulated_valid_ = false;
    accumulated_distance_ = 0.0;
    last_in_ramp_         = in_ramp;
    last_remain_dis_      = remain_dis;
    last_on_highway_      = on_highway;
    env_info_->switch_ld_reason.push_back(fmt::format("Approaching ramp (<= {:.0f} m) → LD=ON", LD_MAP_ACTIVATION_DIST_THRESH));
    SD_ENV_INFO_LOG << fmt::format("Approaching ramp (<= {} m), set is_switched_to_LD_: true", LD_MAP_ACTIVATION_DIST_THRESH);
    return is_switched_to_LD_;
  }

  if (approaching_mainroad) {
    is_switched_to_LD_    = true;
    is_accumulated_valid_ = false;
    accumulated_distance_ = 0.0;
    last_in_ramp_         = true;
    last_remain_dis_      = remain_dis;
    last_on_highway_      = on_highway;
    env_info_->switch_ld_reason.push_back(fmt::format("Approaching mainroad → LD=ON"));
    SD_ENV_INFO_LOG << fmt::format("Approaching mainroad (= {} m), set is_switched_to_LD_: true", v2_dist_to_mainroad);
    return is_switched_to_LD_;
  }
  // 状态3：不在闸道内，控制点距离 > 800 米
  // else if (control_type == ControlWayPoint::COMING_LEFT_ONTO_THE_RAMP || control_type == ControlWayPoint::COMING_RIGHT_ONTO_THE_RAMP) {
  //   is_switched_to_LD_ = false;
  //   SD_ENV_INFO_LOG << fmt::format("Control point distance > 800m, set is_switched_to_LD_: false");
  // }
  // 状态4 和 5：不在闸道内，根据离开闸道后的累积距离判断

  if (ramp_exit_edge || (highway_fall_edge && is_switched_to_LD_ && !is_accumulated_valid_)) {
    is_accumulated_valid_ = true;
    accumulated_distance_ = 0.0;
    SD_ENV_INFO_LOG << fmt::format("Just exited ramp, reset accumulated_distance_ (edge={}, onHW_fall={})", ramp_exit_edge,
                                   highway_fall_edge);
  }

  if (is_accumulated_valid_) {
    // 计算累积行驶距离，过滤异常跳变
    // double delta = std::abs(last_remain_dis_ - remain_dis);
    // if (delta <= MAX_REMAIN_DIS_JUMP) {
    //   accumulated_distance_ += delta;
    //   SD_ENV_INFO_LOG << fmt::format("Normal remain_dis change: {} meters, accumulated_distance_: {}", delta, accumulated_distance_);
    // } else {
    //   SD_ENV_INFO_LOG << fmt::format("Detected abnormal remain_dis jump: {} meters, skip accumulation", delta);
    // }
    accumulated_distance_ += ds;
    SD_ENV_INFO_LOG << fmt::format("Accumulating: +{:.2f}m → {:.2f}m", ds, accumulated_distance_);

    // 根据累积距离判断 LD 模式
    if (accumulated_distance_ >= LD_MAP_DEACTIVATION_DIST_THRESH) {
      is_switched_to_LD_    = false;
      is_accumulated_valid_ = false;
      env_info_->switch_ld_reason.push_back(
          fmt::format("accumulated >= {:.0f} m since ramp exit → LD=OFF", LD_MAP_DEACTIVATION_DIST_THRESH));
      SD_ENV_INFO_LOG << fmt::format("Accumulated distance reached {}m , exit LD mode (switch back to perception)",
                                     LD_MAP_DEACTIVATION_DIST_THRESH);
    } else {
      is_switched_to_LD_ = true;  // 状态4：累积距离 < 300 米
      env_info_->switch_ld_reason.push_back(
          fmt::format("accumulated_distance {:.0f} exit ramp (<= {:.0f} m) → LD=ON", accumulated_distance_, LD_MAP_ACTIVATION_DIST_THRESH));
      SD_ENV_INFO_LOG << fmt::format("Accumulated distance < 300m, maintain LD mode");
    }
    last_in_ramp_    = in_ramp;
    last_on_highway_ = on_highway;
    last_remain_dis_ = remain_dis;
    return is_switched_to_LD_;
  }

  // 不在闸道、无冷却、mapevent没有发临近闸道的控制点，也没发进入主路的控制点, 默认使用之前的切图标志
  // is_switched_to_LD_ = false;
  last_in_ramp_    = in_ramp;
  last_remain_dis_ = remain_dis;
  last_on_highway_ = on_highway;

  // 高速复杂场景切图逻辑
  if (on_highway) {
    auto highway_junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoPtr();
    if (highway_junctions_ptr) {
      const std::vector<navigation::JunctionInfo> &highway_junctions = *highway_junctions_ptr;

      auto inCutWindowAround = [](double offset) -> bool {
        return (offset < HIGHWAY_CUT_WINDOW_BEFORE && offset > -HIGHWAY_CUT_WINDOW_AFTER);
      };

      std::vector<int> ord(highway_junctions.size());
      std::iota(ord.begin(), ord.end(), 0);
      std::sort(ord.begin(), ord.end(), [&](int a, int b) { return highway_junctions[a].offset < highway_junctions[b].offset; });

      // 场景1：高速路口1分3的case
      for (int idx : ord) {
        const auto &j = highway_junctions[idx];
        if (j.split_num >= 3 && inCutWindowAround(j.offset)) {
          is_switched_to_LD_               = true;
          is_set_by_highway_complex_scene_ = true;
          env_info_->switch_ld_reason.push_back(fmt::format("Highway 1->3 split, id:{} off:{:.1f}", j.junction_id, j.offset));
          SD_ENV_INFO_LOG << fmt::format("Highway 1->3 split, id:{} off:{:.1f}, set is_switched_to_LD_: true", j.junction_id, j.offset);
          return is_switched_to_LD_;
        }
      }

      // 场景2：存在两个相邻路口，距离150m内
      static std::unordered_set<uint64_t> remembered_scene2_junctions;
      for (size_t i = 0; i + 1 < ord.size(); ++i) {
        const auto &s = highway_junctions[ord[i]];      // 第一个路口
        const auto &t = highway_junctions[ord[i + 1]];  // 下一个路口

        // 如果两个连续的路口都是ApproachRampMerge类型，则不认为是复杂场景
        if (s.junction_type == JunctionType::ApproachRampMerge && t.junction_type == JunctionType::ApproachRampMerge) {
          SD_ENV_INFO_LOG << fmt::format("Two consecutive ApproachRampMerge junctions, not considered as complex scene");
          continue;
        }

        const double gap = t.offset - s.offset;
        if (gap >= 0.0 && gap < HIGHWAY_CONSECUTIVE_JUNCTION_MAX_GAP) {
          const bool active_bridge = (s.offset < HIGHWAY_CUT_WINDOW_BEFORE) && (t.offset > -HIGHWAY_CUT_WINDOW_AFTER);
          if (active_bridge) {
            is_switched_to_LD_               = true;
            is_set_by_highway_complex_scene_ = true;
            env_info_->switch_ld_reason.push_back(
                fmt::format("Two adjacent highway junctions within {:.0f}m, gap:{:.1f}", HIGHWAY_CONSECUTIVE_JUNCTION_MAX_GAP, gap));
            SD_ENV_INFO_LOG << fmt::format("Two adjacent highway junctions within {:.0f}m, gap:{:.1f}, set is_switched_to_LD_: true",
                                           HIGHWAY_CONSECUTIVE_JUNCTION_MAX_GAP, gap);

            // 记忆该复杂场景的第二个路口
            remembered_scene2_junctions.insert(t.junction_id);
            return is_switched_to_LD_;
          }
        }
      }

      std::vector<uint64_t> to_erase;
      for (const auto &junction_id : remembered_scene2_junctions) {
        bool found = false;
        for (const auto &j : highway_junctions) {
          if (j.junction_id == junction_id) {
            found = true;
            if (j.offset < -HIGHWAY_CUT_WINDOW_AFTER) {
              to_erase.push_back(junction_id);
            }
            break;
          }
        }
        if (!found) {
          to_erase.push_back(junction_id);
        }
      }
      for (const auto &junction_id : to_erase) {
        remembered_scene2_junctions.erase(junction_id);
      }

      if (!remembered_scene2_junctions.empty()) {
        is_switched_to_LD_               = true;
        is_set_by_highway_complex_scene_ = true;
        env_info_->switch_ld_reason.push_back(fmt::format("In remembered highway complex scene, keep is_switched_to_LD_: true"));
        SD_ENV_INFO_LOG << fmt::format("In remembered highway complex scene, keep is_switched_to_LD_: true");
        return is_switched_to_LD_;
      }
    }

    // 如果之前是由高速复杂场景切图逻辑设置的，但现在条件不再满足，则恢复到默认逻辑
    if (is_set_by_highway_complex_scene_) {
      is_set_by_highway_complex_scene_ = false;
      is_switched_to_LD_               = false;
      env_info_->switch_ld_reason.push_back(fmt::format("Highway complex scene condition no longer met, reset to default behavior"));
      SD_ENV_INFO_LOG << fmt::format("Highway complex scene condition no longer met, reset to default behavior");
      last_in_ramp_    = in_ramp;
      last_remain_dis_ = remain_dis;
      last_on_highway_ = on_highway;
      return is_switched_to_LD_;
    }
  }

  env_info_->switch_ld_reason.push_back(fmt::format("maintain last is_switched_to_LD_ mode"));
  SD_ENV_INFO_LOG << fmt::format("maintain last is_switched_to_LD_ mode");
  return is_switched_to_LD_;
}

const EnvInfo *EnvInfoProcessor::GetEnvInfo() const {
  return env_info_.get();
}

bool EnvInfoProcessor::CheckIfInRamp() {
  // 高速场景：走 LD
  if (IsHighwayMode()) {
    auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
    if (!ld_route)
      return false;
    const uint64_t ego_sec = ld_route->navi_start.section_id;
    for (const auto &sec : ld_route->sections) {
      if (sec.id == ego_sec) {
        const bool is_ramp = IsRampSection(sec);
        SD_ENV_INFO_LOG << fmt::format("[LD] Section {} is_ramp: {}", ego_sec, is_ramp);
        return is_ramp;
      }
    }
    return false;
  }

  // 城区场景：沿用 SD
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route)
    return false;
  uint64_t ego_section_id = sd_route->navi_start.section_id;
  for (const auto &sec : sd_route->mpp_sections) {
    if (sec.id == ego_section_id) {
      bool is_ramp = IsRampSection(sec);
      SD_ENV_INFO_LOG << fmt::format("[SD] Section {} is_ramp: {}", ego_section_id, is_ramp);
      return is_ramp;
    }
  }
  return false;
}

bool EnvInfoProcessor::IsRampSection(const SDSectionInfo &section_info) {
  SD_ENV_INFO_LOG << fmt::format("Entering IsRampSection for section_id: {}", section_info.id);
  // if ((section_info.link_type & static_cast<uint32_t>(SDLinkTypeMask::SDLT_RAMP)) == static_cast<uint32_t>(SDLinkTypeMask::SDLT_RAMP)) {
  //   SD_ENV_INFO_LOG << "Section is ramp due to link_type";
  //   return true;
  // }
  if (section_info.direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
    return false;
  }
  uint32_t section_type = section_info.link_type;
  if ((section_type & (uint32_t)SDLinkTypeMask::SDLT_JCT) == (uint32_t)SDLinkTypeMask::SDLT_JCT ||
      (section_type & (uint32_t)SDLinkTypeMask::SDLT_IC) == (uint32_t)SDLinkTypeMask::SDLT_IC ||
      (section_type & (uint32_t)SDLinkTypeMask::SDLT_SERVICE) == (uint32_t)SDLinkTypeMask::SDLT_SERVICE ||
      (section_type & (uint32_t)SDLinkTypeMask::SDLT_PARK) == (uint32_t)SDLinkTypeMask::SDLT_PARK ||
      (section_type & (uint32_t)SDLinkTypeMask::SDLT_RAMP) == (uint32_t)SDLinkTypeMask::SDLT_RAMP ||
      (section_info.road_class == SDRoadClass::SD_OTHER_ROAD &&
       (section_type & (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION) == (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION)) {
    return true;
  } else {
    return false;
  }

  //测试发现ld地图存在非闸道section里面存在lane是RAMP问题，先注销该逻辑
  // for (const auto &lg_idx : section_info.lane_group_idx) {
  //   const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
  //   if (lane_group) {
  //     for (const auto &lane : lane_group->lane_info) {
  //       if (lane.type == LaneType::LANE_RAMP) {
  //         SD_ENV_INFO_LOG << "Section is ramp due to lane type";
  //         return true;
  //       }
  //     }
  //   }
  // }
  // SD_ENV_INFO_LOG << "Section is not ramp";
  return false;
}

bool EnvInfoProcessor::IsRampSection(const SectionInfo &ld_section) {
  // 任一车道为 LANE_RAMP 则认定为匝道
  for (uint64_t lane_id : ld_section.lane_ids) {
    const auto *lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
    if (lane && lane->type == LaneType::LANE_RAMP) {
      return true;
    }
  }

  const uint32_t section_type = ld_section.link_type;
  auto           has_flag     = [section_type](LDLinkTypeMask m) -> bool {
    return (section_type & static_cast<uint32_t>(m)) == static_cast<uint32_t>(m);
  };

  if (has_flag(LDLinkTypeMask::LT_JCT))
    return true;
  if (has_flag(LDLinkTypeMask::LT_IC))
    return true;
  if (has_flag(LDLinkTypeMask::LT_SAPA))
    return true;
  if (ld_section.road_class == cem::message::env_model::RoadClass::OTHERS && has_flag(LDLinkTypeMask::LT_MAINROAD_CONNECTION)) {
    return true;
  }

  return false;
}

double EnvInfoProcessor::CalculateDistanceToSubpath() {
  if (IsHighwayMode()) {
    SD_ENV_INFO_LOG << fmt::format("[LD] CalculateDistanceToSubpath");
    auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
    if (!ld_route)
      return DEFAULT_LARGE_DISTANCE;
    const double ego_s = GetEgoGlobalS(*ld_route);

    std::unordered_set<uint64_t> route_sec_ids;
    route_sec_ids.reserve(ld_route->sections.size());
    for (const auto &sec : ld_route->sections)
      route_sec_ids.insert(sec.id);

    double min_dist = DEFAULT_LARGE_DISTANCE;
    for (const auto &sp : ld_route->subpaths) {
      if (!route_sec_ids.count(sp.enter_section_id))
        continue;
      auto enter_sec = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(sp.enter_section_id);
      if (!enter_sec)
        continue;

      const bool is_split = enter_sec->successor_section_id_list.size() > 1;
      const bool is_merge = enter_sec->predecessor_section_id_list.size() > 1;

      double s0 = GetSectionStartS(*enter_sec, *ld_route);
      double target_s;
      if (is_split) {
        target_s = s0 + enter_sec->length;  // split: 终点
      } else if (is_merge) {
        target_s = s0;  // merge: 起点
      } else {
        target_s = s0;
      }
      const double dist = target_s - ego_s;
      if (dist >= 0.0 && dist < min_dist)
        min_dist = dist;
    }
    return min_dist;
  }

  // SD 原逻辑
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    AWARN << "[EnvInfoProcessor] SDRouteInfo is null in CalculateDistanceToSubpath";
    SD_ENV_INFO_LOG << fmt::format("SDRouteInfo is null, returning default large distance");
    return DEFAULT_LARGE_DISTANCE;
  }
  SD_ENV_INFO_LOG << fmt::format("[SD] CalculateDistanceToSubpath");
  const auto &sd_route     = *INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  double      ego_global_s = GetEgoGlobalS(sd_route);
  double      min_dist     = DEFAULT_LARGE_DISTANCE;

  std::unordered_set<uint64_t> route_section_ids;
  for (const auto &sec : sd_route.mpp_sections) {
    route_section_ids.insert(sec.id);
  }

  for (const auto &subpath : sd_route.subpaths) {
    if (subpath.path_type == SDPathType::MERGE || subpath.path_type == SDPathType::SPLIT) {
      uint64_t enter_section_id = subpath.enter_section_id;

      if (route_section_ids.find(enter_section_id) == route_section_ids.end()) {
        SD_ENV_INFO_LOG << fmt::format("Subpath section_id {} not in route, skipping", enter_section_id);
        continue;
      }

      auto enter_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(enter_section_id);
      if (!enter_section) {
        SD_ENV_INFO_LOG << fmt::format("Enter section {} not found, skipping", enter_section_id);
        continue;
      }

      double section_start_s = GetSectionStartS(*enter_section, sd_route);
      double section_end_s   = section_start_s + enter_section->length;

      double target_s = (subpath.path_type == SDPathType::MERGE) ? section_start_s : section_end_s;
      double dist     = target_s - ego_global_s;

      if (dist >= 0 && dist < min_dist) {
        min_dist = dist;
        // SD_ENV_INFO_LOG << "Updated min_dist to " << min_dist << " for subpath section_id: " << enter_section_id;
      }
    }
  }
  //   SD_ENV_INFO_LOG << "Returning min_dist: " << min_dist;
  return min_dist;
}

double EnvInfoProcessor::CalculateDistanceToSplitRouteLaneNumDec() {

  if (IsHighwayMode()) {
    auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
    if (!ld_route)
      return DEFAULT_LARGE_DISTANCE;

    const double ego_s    = GetEgoGlobalS(*ld_route);
    double       min_dist = DEFAULT_LARGE_DISTANCE;

    // 为了方便定位 section -> start_s
    std::unordered_map<uint64_t, double> sec_start;
    sec_start.reserve(ld_route->sections.size());
    double acc_s = 0.0;
    for (const auto &sec : ld_route->sections) {
      sec_start[sec.id] = acc_s;
      acc_s += sec.length;
    }

    // 逐 section 检查“后继车道数减少”
    for (const auto &sec : ld_route->sections) {
      int enter_lane_num = CountNonEmergencyLanesLD(sec.id);
      if (enter_lane_num <= 0)
        continue;

      // 简化：若没有后继或后继不在 route.sections 列表，则跳过
      for (uint64_t succ_id : sec.successor_section_id_list) {
        if (!sec_start.count(succ_id))
          continue;  // 仅考虑主线后继

        int next_lane_num = CountNonEmergencyLanesLD(succ_id);
        if (next_lane_num <= 0)
          continue;

        if (enter_lane_num > next_lane_num) {
          double split_s = sec_start.at(sec.id) + sec.length;  // 取 enter_section 的末端作为分叉近似
          double dist    = split_s - ego_s;
          if (dist >= 0.0 && dist < min_dist)
            min_dist = dist;
        }
      }
    }
    return min_dist;
  }

  // 城区（SD）维持原逻辑（含 lanegroup / successor 校验）
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    AWARN << "[EnvInfoProcessor] SDRouteInfo is null in CalculateDistanceToSplitRouteLaneNumDec";
    SD_ENV_INFO_LOG << fmt::format("SDRouteInfo is null, returning default large distance");
    return DEFAULT_LARGE_DISTANCE;
  }
  const auto &sd_route     = *INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  double      ego_global_s = GetEgoGlobalS(sd_route);
  double      min_dist     = DEFAULT_LARGE_DISTANCE;

  for (const auto &subpath : sd_route.subpaths) {
    if (subpath.path_type != SDPathType::SPLIT) {
      // SD_ENV_INFO_LOG << fmt::format("Subpath is not SPLIT, skipping");
      continue;
    }

    uint64_t enter_section_id = subpath.enter_section_id;
    auto     enter_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(enter_section_id);
    if (!enter_section) {
      SD_ENV_INFO_LOG << fmt::format("Enter section {} not found, skipping", enter_section_id);
      continue;
    }

    // 获取 enter_section 的最后一个 lanegroup
    if (enter_section->lane_group_idx.empty()) {
      // SD_ENV_INFO_LOG << fmt::format("No lane groups in section {}, skipping", enter_section_id);
      continue;
    }

    // 获取 enter_section 的车道数
    int         enter_lane_num = -1;
    uint64_t    last_lg_id     = enter_section->lane_group_idx.back().id;
    const auto *last_lg        = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(last_lg_id);
    if (!last_lg) {
      enter_lane_num = GetLaneNumWithoutEmergency(last_lg);
    } else {
      // SD_ENV_INFO_LOG << fmt::format("Last lane group {} not found, skipping", last_lg_id);
      enter_lane_num = static_cast<int>(enter_section->lane_num);
    }

    // 遍历后继 section
    for (uint64_t successor_id : enter_section->successor_section_id_list) {
      auto successor_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(successor_id);
      if (!successor_section || !successor_section->is_mpp_section) {
        // SD_ENV_INFO_LOG << fmt::format("Successor section {} not found or not MPP, skipping", successor_id);
        continue;
      }

      // 获取 successor_section 的第一个 lanegroup
      if (successor_section->lane_group_idx.empty()) {
        // SD_ENV_INFO_LOG << fmt::format("No lane groups in successor section {}, skipping", successor_id);
        continue;
      }
      uint64_t    first_successor_lg_id = successor_section->lane_group_idx.front().id;
      const auto *first_successor_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(first_successor_lg_id);
      if (!first_successor_lg) {
        SD_ENV_INFO_LOG << fmt::format("First successor lane group {} not found, skipping", first_successor_lg_id);
        continue;
      }

      // 检查车道后继
      bool all_successors_in_first_lg = true;
      for (const auto &lane : last_lg->lane_info) {
        for (uint64_t successor_lane_id : lane.next_lane_ids) {
          bool found = false;
          for (const auto &successor_lane : first_successor_lg->lane_info) {
            if (successor_lane.id == successor_lane_id) {
              found = true;
              break;
            }
          }
          if (!found) {
            all_successors_in_first_lg = false;
            break;
          }
        }
        if (!all_successors_in_first_lg) {
          break;
        }
      }

      // 获取 successor_section 的车道数
      int next_lane_num = GetSDSectionMinLaneNumNOTEmergency(*successor_section);

      // 判断条件：车道数减少 或 后继车道不全部在下一个 lanegroup 中
      if (enter_lane_num > next_lane_num || !all_successors_in_first_lg) {
        double split_s = GetSectionStartS(*enter_section, sd_route) + enter_section->length;
        double dist    = split_s - ego_global_s;
        if (dist >= 0 && dist < min_dist) {
          min_dist = dist;
          // SD_ENV_INFO_LOG << fmt::format("Condition met at section {} (lane decrease: {}, successor check: {}), updated min_dist: {}",
          //                                enter_section_id, (enter_lane_num > next_lane_num), !all_successors_in_first_lg, min_dist);
        }
      }
    }
  }
  // SD_ENV_INFO_LOG << fmt::format("Returning min_dist: {}", min_dist);
  return min_dist;
}

double EnvInfoProcessor::GetEgoGlobalS(const SDRouteInfo &sd_route) const {
  // SD_ENV_INFO_LOG << fmt::format("Entering GetEgoGlobalS");
  double   ego_global_s   = 0.0;
  uint64_t ego_section_id = sd_route.navi_start.section_id;
  double   ego_s_offset   = sd_route.navi_start.s_offset;
  for (const auto &sec : sd_route.mpp_sections) {
    if (sec.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      break;
    }
    ego_global_s += sec.length;
  }
  // SD_ENV_INFO_LOG << fmt::format("Returning ego_global_s: {}", ego_global_s);
  return ego_global_s;
}

double EnvInfoProcessor::GetSectionStartS(const SDSectionInfo &section, const SDRouteInfo &sd_route) const {
  // SD_ENV_INFO_LOG << "Entering GetSectionStartS for section_id: " << section.id;
  double current_s = 0.0;
  for (const auto &sec : sd_route.mpp_sections) {
    if (sec.id == section.id) {
      // SD_ENV_INFO_LOG << fmt::format("Returning current_s: {}", current_s);
      return current_s;
    }
    current_s += sec.length;
  }
  // SD_ENV_INFO_LOG << fmt::format("Section not found, returning current_s: {}", current_s);
  return current_s;
}

double EnvInfoProcessor::GetEgoGlobalS(const RouteInfo &ld_route) const {
  double         s       = 0.0;
  const uint64_t ego_sec = ld_route.navi_start.section_id;
  const double   ego_off = ld_route.navi_start.s_offset;
  for (const auto &sec : ld_route.sections) {
    if (sec.id == ego_sec) {
      s += ego_off;
      break;
    }
    s += sec.length;
  }
  return s;
}

double EnvInfoProcessor::GetSectionStartS(const SectionInfo &sec_in, const RouteInfo &ld_route) const {
  double s = 0.0;
  for (const auto &sec : ld_route.sections) {
    if (sec.id == sec_in.id)
      return s;
    s += sec.length;
  }
  return s;
}

int EnvInfoProcessor::GetLaneNumWithoutEmergency(const SDLaneGroupInfo *lanegroup) {
  SD_ENV_INFO_LOG << fmt::format("Entering GetLaneNumWithoutEmergency");
  if (!lanegroup) {
    SD_ENV_INFO_LOG << fmt::format("Lane group is null, returning 0");
    return 0;
  }
  int count = 0;
  for (const auto &lane : lanegroup->lane_info) {
    if (lane.type != LaneType::LANE_EMERGENCY) {
      count++;
    }
  }
  SD_ENV_INFO_LOG << fmt::format("Returning non-emergency lane count: {}", count);
  return count;
}

int EnvInfoProcessor::CountNonEmergencyLanesLD(uint64_t section_id) {
  const auto *sec = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(section_id);
  if (!sec)
    return 0;
  int cnt = 0;
  for (auto lane_id : sec->lane_ids) {
    const auto *lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
    if (lane && lane->type != LaneType::LANE_EMERGENCY)
      ++cnt;
  }
  return cnt;
}

int EnvInfoProcessor::GetSDSectionMinLaneNumNOTEmergency(const SDSectionInfo &section_info) {
  // SD_ENV_INFO_LOG << fmt::format("Entering GetSDSectionMinLaneNumNOTEmergency for section_id: {}", section_info.id);
  if (section_info.lane_group_idx.empty()) {
    SD_ENV_INFO_LOG << fmt::format("No lane groups, returning section lane_num: {}", section_info.lane_num);
    return static_cast<int>(section_info.lane_num);
  }
  int min_lane_num = std::numeric_limits<int>::max();
  for (const auto &lane_group_id_tmp : section_info.lane_group_idx) {
    auto lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lane_group_id_tmp.id);
    if (lane_group) {
      int emergency_lane_count = 0;
      for (const auto &lane_info_tmp : lane_group->lane_info) {
        if (lane_info_tmp.type == LaneType::LANE_EMERGENCY) {
          emergency_lane_count++;
        }
      }
      int current_lane_num = lane_group->lane_info.size() - emergency_lane_count;
      if (current_lane_num < min_lane_num) {
        min_lane_num = current_lane_num;
      }
    }
  }
  if (min_lane_num == std::numeric_limits<int>::max()) {
    SD_ENV_INFO_LOG << fmt::format("No valid lane groups, returning section lane_num: {}", section_info.lane_num);
    return section_info.lane_num != 0 ? static_cast<int>(section_info.lane_num) : 0;
  }
  int result = min_lane_num == 0 ? static_cast<int>(section_info.lane_num) : min_lane_num;
  // SD_ENV_INFO_LOG << fmt::format("Returning min non-emergency lane count: {}", result);
  return result;
}

V2RoadClassType EnvInfoProcessor::ConvertSDRoadClassToV2RoadClassType(SDRoadClass sd_road_class) const {
  switch (sd_road_class) {
    case SDRoadClass::SD_HIGHWAY:
      return V2RoadClassType::HIGH_WAY_ROAD;
    case SDRoadClass::SD_CITY_FAST_WAY:
      return V2RoadClassType::EXPRESS_WAY_ROAD;
    case SDRoadClass::SD_NATIONAL_ROAD:
      return V2RoadClassType::NATIOANL_ROAD;
    case SDRoadClass::SD_PROVINCIAL_ROAD:
      return V2RoadClassType::PROVINCIAL_ROAD;
    case SDRoadClass::SD_COUNTY_ROAD:
    case SDRoadClass::SD_TOWNSHIP_ROAD:
      return V2RoadClassType::MAIN_ROAD;
    case SDRoadClass::SD_OTHER_ROAD:
    case SDRoadClass::SD_LEVEL_9_ROAD:
    case SDRoadClass::SD_FERRY:
    case SDRoadClass::SD_WALY_WAY:
    case SDRoadClass::SD_INVALID:
      return V2RoadClassType::UNKNOWN_ROAD;
    default:
      return V2RoadClassType::UNKNOWN_ROAD;
  }
}

V2RoadClassType EnvInfoProcessor::ConvertLDRoadClassToV2RoadClassType(RoadClass rc) const {
  switch (rc) {
    case RoadClass::EXPRESSWAY:
      return V2RoadClassType::HIGH_WAY_ROAD;
    case RoadClass::URBAN_EXPRESSWAY:
      return V2RoadClassType::EXPRESS_WAY_ROAD;
    case RoadClass::NATION_ROAD:
      return V2RoadClassType::NATIOANL_ROAD;
    case RoadClass::PROVINCE_ROAD:
      return V2RoadClassType::PROVINCIAL_ROAD;
    case RoadClass::COUNTRY_ROAD:
    case RoadClass::TOWN_ROAD:
      return V2RoadClassType::MAIN_ROAD;
    default:
      return V2RoadClassType::UNKNOWN_ROAD;
  }
}

std::vector<V2RoadClass> EnvInfoProcessor::CalculateV2RoadClasses() const {
  std::vector<V2RoadClass> v2_road_classes;
  v2_road_classes.clear();

  if (IsHighwayMode()) {
    auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
    if (!ld_route || ld_route->sections.empty())
      return v2_road_classes;

    const double ego_s   = GetEgoGlobalS(*ld_route);
    double       start_s = GetSectionStartS(ld_route->sections[0], *ld_route);
    double       end_s   = start_s + ld_route->sections[0].length;
    V2RoadClass  current{start_s - ego_s, end_s - ego_s, ConvertLDRoadClassToV2RoadClassType(ld_route->sections[0].road_class)};

    for (size_t i = 1; i < ld_route->sections.size(); ++i) {
      const auto &sec  = ld_route->sections[i];
      double      s0   = GetSectionStartS(sec, *ld_route);
      double      s1   = s0 + sec.length;
      auto        type = ConvertLDRoadClassToV2RoadClassType(sec.road_class);
      if (type == current.road_class) {
        current.end = s1 - ego_s;
      } else {
        v2_road_classes.push_back(current);
        current = {s0 - ego_s, s1 - ego_s, type};
      }
    }
    v2_road_classes.push_back(current);
    return v2_road_classes;
  }

  // 城区（SD）维持原逻辑
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    AWARN << "[EnvInfoProcessor] SDRouteInfo is null";
    SD_ENV_INFO_LOG << fmt::format("SDRouteInfo is null, exiting");
    return {};
  }
  const auto &sd_route = *INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (sd_route.mpp_sections.empty()) {
    return {};
  }

  double      ego_global_s  = GetEgoGlobalS(sd_route);
  double      first_start_s = GetSectionStartS(sd_route.mpp_sections[0], sd_route);
  double      first_end_s   = first_start_s + sd_route.mpp_sections[0].length;
  V2RoadClass current{first_start_s - ego_global_s, first_end_s - ego_global_s,
                      ConvertSDRoadClassToV2RoadClassType(sd_route.mpp_sections[0].road_class)};

  // Process remaining sections
  for (size_t i = 1; i < sd_route.mpp_sections.size(); ++i) {
    const auto &section = sd_route.mpp_sections[i];
    double      start_s = GetSectionStartS(section, sd_route);
    double      end_s   = start_s + section.length;
    auto        type    = ConvertSDRoadClassToV2RoadClassType(section.road_class);
    // SD_ENV_INFO_LOG << "Section_id: " << section.id <<" start_s: " << start_s << " end_s: " << end_s << " type: " << static_cast<int>(section.road_class);

    if (type == current.road_class) {
      // Same road class type, extend the current segment
      current.end = end_s - ego_global_s;
    } else {
      // Different road class type, save current and start a new segment
      v2_road_classes.push_back(current);
      current = {start_s - ego_global_s,  // Start relative to ego
                 end_s - ego_global_s,    // End relative to ego
                 type};
    }
  }

  // Add the last segment
  v2_road_classes.push_back(current);
  return v2_road_classes;
}

UTurnPosition EnvInfoProcessor::FindUTurnLanePosition(const std::vector<TurnType> &lane_arrows) {
  if (lane_arrows.empty())
    return UTurnPosition::UNKNOWN;

  // 查找最左侧和最右侧的 U-turn 车道
  bool leftmost_has_uturn  = false;
  bool rightmost_has_uturn = false;

  for (size_t i = 0; i < lane_arrows.size(); ++i) {
    if (lane_arrows[i] == TurnType::STRAIGHT_AND_U_TURN || lane_arrows[i] == TurnType::LEFT_TURN_AND_U_TURN ||
        lane_arrows[i] == TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN) {
      if (i == 0)
        leftmost_has_uturn = true;
    }
    if (lane_arrows[i] == TurnType::STRAIGHT_AND_U_TURN || lane_arrows[i] == TurnType::RIGHT_TURN_AND_U_TURN ||
        lane_arrows[i] == TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN) {
      if (i == lane_arrows.size() - 1)
        rightmost_has_uturn = true;
    }
  }

  if (leftmost_has_uturn)
    return UTurnPosition::LEFTMOST;
  if (rightmost_has_uturn)
    return UTurnPosition::RIGHTMOST;
  return UTurnPosition::UNKNOWN;
}

std::pair<V2TurnInfo::V2TurnType, V2TurnInfo::V2DetailTurnType> EnvInfoProcessor::GetTurnInfoForRoadSplit(JunctionAction action) {
  static const std::map<JunctionAction, std::pair<V2TurnInfo::V2TurnType, V2TurnInfo::V2DetailTurnType>> turn_map = {
      {JunctionAction::TurnLeft, {V2TurnInfo::V2TurnType::LEFT, V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT}},
      {JunctionAction::TurnRight, {V2TurnInfo::V2TurnType::RIGHT, V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT}},
      {JunctionAction::GoStraight, {V2TurnInfo::V2TurnType::STRAIGHT, V2TurnInfo::V2DetailTurnType::CONTINUE}}};
  auto it = turn_map.find(action);
  if (it != turn_map.end()) {
    return it->second;
  }
  return {V2TurnInfo::V2TurnType::UNKNOWN, V2TurnInfo::V2DetailTurnType::NONE};
}

std::vector<V2TurnInfo> EnvInfoProcessor::CalculateV2TurnInfo(const std::vector<cem::message::env_model::StopLine> &stop_line_obj) {
  if (!env_info_) {
    SD_ENV_INFO_LOG << fmt::format("EnvInfo is null, returning empty turn info");
    return {};
  }

  // 根据Master层计算的is_on_highway_ 区分处理高速还是城区逻辑
  if (env_info_->is_on_highway_) {
    SD_ENV_INFO_LOG << fmt::format("is_on_highway_ is: {}, Processing highway logic.", env_info_->is_on_highway_);
    auto highway_junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoPtr();
    if (highway_junctions_ptr) {
      return ProcessHighwayJunctions(stop_line_obj, *highway_junctions_ptr);
    } else {
      SD_ENV_INFO_LOG << fmt::format("Highway junctions pointer is null, returning empty turn info");
      return {};
    }
  } else {
    SD_ENV_INFO_LOG << fmt::format("is_on_highway_ is: {}, Processing city logic.", env_info_->is_on_highway_);
    auto city_junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoCityPtr();
    if (city_junctions_ptr) {
      return ProcessCityJunctions(stop_line_obj, *city_junctions_ptr);
    } else {
      SD_ENV_INFO_LOG << fmt::format("City junctions pointer is null, returning empty turn info");
      return {};
    }
  }
}

std::vector<V2TurnInfo> EnvInfoProcessor::ProcessCityJunctions(const std::vector<cem::message::env_model::StopLine> &stop_line_obj,
                                                               const std::vector<navigation::JunctionInfoCity>      &city_junctions) {
  if (city_junctions.empty()) {
    return {};
  }
  std::vector<V2TurnInfo> turn_infos;
  turn_infos.clear();
  std::vector<uint64_t> v2_junction_ids = INTERNAL_PARAMS.navigation_info_data.get_v2_junction_ids();
  for (const auto &junction : city_junctions) {
    auto find_v2 = std::find(v2_junction_ids.begin(), v2_junction_ids.end(), junction.junction_id);
    if (find_v2 == v2_junction_ids.end()) {
      continue;
    }
    SD_ENV_INFO_LOG << fmt::format("{}", junction);

    V2TurnInfo turn_info;
    turn_info.id       = junction.junction_id;
    turn_info.is_valid = true;

    // 处理 RoadSplit 路口的特殊逻辑
    if (junction.junction_type_city == JunctionTypeCity::RoadSplit) {
      // 检查是否为右转专用道
      if (junction.is_dedicated_right_turn_lane && junction.split_merge_direction == DirectionSplitMerge::Right) {
        turn_info.turn_type        = V2TurnInfo::V2TurnType::RIGHT;
        turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY;
      } else {
        // 非右转专用道的RoadSplit路口，使用原默认逻辑
        auto [turn_type, detail_turn_type] = GetTurnInfoForRoadSplit(junction.junction_action);
        turn_info.turn_type                = turn_type;
        turn_info.detail_turn_type         = detail_turn_type;
      }
    } else {
      // 其他路口类型的原有逻辑
      switch (junction.junction_action) {
        case JunctionAction::TurnLeft:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::LEFT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::TURN_LEFT;
          break;
        case JunctionAction::TurnRight:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::RIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::TURN_RIGHT;
          break;
        case JunctionAction::GoStraight:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::STRAIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::CONTINUE;
          break;
        case JunctionAction::UTurn: {
          UTurnPosition uturn_pos = FindUTurnLanePosition(junction.map_lane_arrows_plan);
          if (uturn_pos == UTurnPosition::LEFTMOST) {
            turn_info.turn_type = V2TurnInfo::V2TurnType::U_TURN_LEFT;
          } else if (uturn_pos == UTurnPosition::RIGHTMOST) {
            turn_info.turn_type = V2TurnInfo::V2TurnType::U_TURN_RIGHT;
          }
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::UTURN;
          break;
        }
        default:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::UNKNOWN;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::NONE;
          break;
      }
    }

    turn_info.v2_dist = junction.offset;
    if (!stop_line_obj.empty() && (std::fabs(stop_line_obj.at(0).distance_to_stopline - turn_info.v2_dist) < 30.0)) {
      SD_ENV_INFO_LOG << fmt::format("Using stopline dist to calib dist_v2");
      turn_info.dist = stop_line_obj.at(0).distance_to_stopline;
    } else {
      turn_info.dist = turn_info.v2_dist;
    }
    turn_info.dist = turn_info.v2_dist;  // TMP: 使用 v2_dist 作为 dist, 避免因 stopline 距离不准或者绑定错误导致的问题
    turn_info.before_turn.lane_num = junction.main_road_lane_nums;
    turn_info.after_turn.lane_num  = junction.target_road_lane_nums;

    if (!junction.junction_ids.empty()) {
      uint64_t before_section_id = junction.junction_ids.front();
      auto     before_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(before_section_id);
      if (before_section) {
        turn_info.before_turn.road_class = ConvertSDRoadClassToV2RoadClassType(before_section->road_class);
      } else {
        turn_info.before_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
        SD_ENV_INFO_LOG << fmt::format("Before section {} not found for junction {}", before_section_id, junction.junction_id);
      }

      uint64_t after_section_id = junction.junction_ids.back();
      auto     after_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(after_section_id);
      if (after_section) {
        turn_info.after_turn.road_class = ConvertSDRoadClassToV2RoadClassType(after_section->road_class);
      } else {
        turn_info.after_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
        SD_ENV_INFO_LOG << fmt::format("After section {} not found for junction {}", after_section_id, junction.junction_id);
      }
    } else {
      turn_info.before_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
      turn_info.after_turn.road_class  = V2RoadClassType::UNKNOWN_ROAD;
      SD_ENV_INFO_LOG << fmt::format("Junction {} has empty junction_ids", junction.junction_id);
    }

    turn_infos.push_back(turn_info);
  }
  return turn_infos;
}

std::vector<V2TurnInfo> EnvInfoProcessor::ProcessHighwayJunctions(const std::vector<cem::message::env_model::StopLine> &stop_line_obj,
                                                                  const std::vector<navigation::JunctionInfo>          &highway_junctions) {
  std::vector<V2TurnInfo> turn_infos;
  std::vector<uint64_t>   v2_junction_ids = INTERNAL_PARAMS.navigation_info_data.get_v2_junction_ids();

  // 定义闸道里分叉路口类型集合
  std::unordered_set<JunctionType> split_types = {JunctionType::RampSplitLeft,      JunctionType::RampSplitRight,
                                                  JunctionType::RampSplitMiddle,    JunctionType::MainRoadSplitLeft,
                                                  JunctionType::MainRoadSplitRight, JunctionType::MainRoadSplitMiddle};

  for (const auto &junction : highway_junctions) {
    auto find_v2 = std::find(v2_junction_ids.begin(), v2_junction_ids.end(), junction.junction_id);
    if (find_v2 == v2_junction_ids.end()) {
      continue;
    }
    SD_ENV_INFO_LOG << fmt::format("Evaluating highway junction ID: {}, type: {}, SplitDirection: {}, offset: {}", junction.junction_id,
                                   static_cast<int>(junction.junction_type), static_cast<int>(junction.split_direction), junction.offset);

    V2TurnInfo turn_info;
    turn_info.id       = junction.junction_id;
    turn_info.is_valid = true;

    // 检查是否为闸道里分叉路口类型
    if (split_types.count(junction.junction_type)) {
      // 对于分叉路口，使用 split_direction 设置转向信息
      switch (junction.split_direction) {
        case SplitDirection::Straight:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::STRAIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::CONTINUE;
          break;
        case SplitDirection::SplitLeft:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::LEFT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT;
          break;
        case SplitDirection::SplitRight:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::RIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT;
          break;
        default:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::UNKNOWN;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::NONE;
          break;
      }
    } else {
      // 对于非分叉路口，保留原有 junction_type 逻辑
      switch (junction.junction_type) {
        case JunctionType::RampInto:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::RIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT;
          break;
        case JunctionType::ApproachRampInto:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::STRAIGHT;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::CONTINUE;
          break;
        default:
          turn_info.turn_type        = V2TurnInfo::V2TurnType::UNKNOWN;
          turn_info.detail_turn_type = V2TurnInfo::V2DetailTurnType::NONE;
          break;
      }
    }

    turn_info.v2_dist              = junction.offset;
    turn_info.dist                 = turn_info.v2_dist;
    turn_info.before_turn.lane_num = junction.main_road_lane_nums;
    turn_info.after_turn.lane_num  = junction.target_road_lane_nums;

    if (!junction.junction_ids.empty()) {
      uint64_t before_section_id = junction.junction_ids.front();
      uint64_t after_section_id  = junction.junction_ids.back();
      auto     before_section    = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(before_section_id);
      if (before_section) {
        turn_info.before_turn.road_class = ConvertLDRoadClassToV2RoadClassType(before_section->road_class);
      } else {
        turn_info.before_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
        SD_ENV_INFO_LOG << fmt::format("Before section {} not found for junction {}", before_section_id, junction.junction_id);
      }

      auto after_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(after_section_id);
      if (after_section) {
        turn_info.after_turn.road_class = ConvertLDRoadClassToV2RoadClassType(after_section->road_class);
      } else {
        turn_info.after_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
        SD_ENV_INFO_LOG << fmt::format("After section {} not found for junction {}", after_section_id, junction.junction_id);
      }
    } else {
      turn_info.before_turn.road_class = V2RoadClassType::UNKNOWN_ROAD;
      turn_info.after_turn.road_class  = V2RoadClassType::UNKNOWN_ROAD;
      SD_ENV_INFO_LOG << fmt::format("Junction {} has empty junction_ids", junction.junction_id);
    }

    turn_infos.push_back(turn_info);
  }
  return turn_infos;
}

void EnvInfoProcessor::SetIsOnHighway(bool is_on_highway) {
  if (env_info_) {
    env_info_->is_on_highway_ = is_on_highway;
    // SD_ENV_INFO_LOG << fmt::format("Set value is_on_highway:{}", env_info_->is_on_highway_);
  }
}

void EnvInfoProcessor::PrintEnvInfo(const EnvInfo &env_info) const {
  SD_ENV_INFO_LOG << fmt::format("{}", env_info);
}

bool EnvInfoProcessor::EgoIsInTunnel() {
  SD_ENV_INFO_LOG << "Entering EgoIsInTunnel with LD";
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[EgoIsInTunnel LD] ld_route_info is null";
    SD_ENV_INFO_LOG << "ld_route_info is null";
    return false;
  }

  const auto &mpp_sections = ld_route_info->sections;
  if (mpp_sections.empty()) {
    AWARN << "[EgoIsInTunnel LD] mpp_sections is empty";
    SD_ENV_INFO_LOG << "mpp_sections is empty";
    return false;
  }

  uint64_t current_section_id = ld_route_info->navi_start.section_id;
  SD_ENV_INFO_LOG << "current_section_id: " << current_section_id;

  const SectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[EgoIsInTunnel LD] current section not found";
    SD_ENV_INFO_LOG << "current section not found";
    return false;
  }
  SD_ENV_INFO_LOG << "current link type: " << current_section->link_type;
  bool is_tunnel = (current_section->link_type & static_cast<uint32_t>(LDLinkTypeMask::LT_TUNNEL)) != 0;
  SD_ENV_INFO_LOG << "[EgoIsInTunnel LD] link_type: " << StrLinkType(current_section->link_type) << "result: " << is_tunnel;
  return is_tunnel;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem