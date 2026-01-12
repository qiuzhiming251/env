#include "lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.h"

using namespace byd::msg::orin::routing_map;

namespace cem {
namespace fusion {

MapSourceSwitch::MapSourceSwitch() {}

MapSourceSwitch::~MapSourceSwitch() {}

void MapSourceSwitch::Init(const FusionManager* fusion_mgr) {
  fusion_mgr_ = fusion_mgr;
  env_status_ = std::make_unique<EnvStatus>();
}

void MapSourceSwitch::Process() {
  // env_status_->Clear();
  env_status_->clear_debug_info();

  SetEgoLaneChangingStatus();
  JudgeDrivingScenario();
  SetMapMatchStatus();
  LoadMapEvent();
  if (IsConditionSatisfiedToUseHdMap()) {
    if (is_on_freeway_) {
      HNOAMapSourceSwitch();
    } else {
      CNOAMapSourceSwitch();
    }
    ForceChangeSourceToPerceptionMap();
  } else {
    use_perception_ = true;
  }
  IsEgoCloseToUnreliableRoad();
  if (use_perception_) {
    env_status_->set_map_source(
        EnvStatus::EnvSource::EnvStatus_EnvSource_PERCEPTION_MAP);
    env_status_->clear_perception_map_suppression_reason();
  } else {
    env_status_->set_map_source(
        EnvStatus::EnvSource::EnvStatus_EnvSource_HD_MAP);
    env_status_->clear_hd_map_suppression_reason();
  }
}

void MapSourceSwitch::SetRampFlag(bool flag) { is_in_ramp_range_ = flag; }

void MapSourceSwitch::SetMultipleConsecutiveRoadIntersectionsFlag(bool flag) {
  is_ego_in_multi_consecutive_road_intersections_ = flag;
}

void MapSourceSwitch::SetJunctionInfo(
    double distance_to_previous_junction, double distance_to_next_junction,
    double distance_to_section_over_first_junction,
    bool is_junction_from_hd_map) {
  distance_to_previous_junction_ = distance_to_previous_junction;
  distance_to_next_junction_ = distance_to_next_junction;
  distance_to_section_over_first_junction_ =
      distance_to_section_over_first_junction;
  is_junction_from_hd_map_ = is_junction_from_hd_map;
  ProcessSpecialJunctionInfo();

  env_status_->set_distance_to_next_junction(distance_to_next_junction_);
  env_status_->set_distance_to_passed_junction(distance_to_previous_junction_);
}

void MapSourceSwitch::SetEgoLaneMatchingInfo(
    const GeometryMatchResult& ego_lane_match_info) {
  EvaluateEgoLaneMatchStatus(ego_lane_match_info);
}

void MapSourceSwitch::SetEgoLaneChangingStatus() {
  ego_is_changing_lane_ = false;

  PLanningResultPtr planning_result{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(planning_result);
  if (planning_result != nullptr) {
    if (planning_result->lc_state != PLanningResultData::Keeping) {
      ego_is_changing_lane_ = true;
    }
  }
}

void MapSourceSwitch::SetMapMatchStatus() {
  map_match_status_.valid = false;
  auto time_now = GetMwTimeNowSec();

  if (function_mode_ == LdMapProcessor::FunctionMode::kHNOA) {
    MapMatchResultBaiduPtr map_match_result = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(map_match_result);
    if (map_match_result == nullptr) {
      return;
    }
    if (fabs(time_now - map_match_result->header.timestamp) > 3) {
      return;
    }
    map_match_status_.valid = true;
    map_match_status_.header.recv_timestamp = time_now;
    map_match_status_.header.sequence_num =
        map_match_result->header.sequence_num;
    map_match_status_.header.timestamp = map_match_result->header.timestamp;
    map_match_status_.location_level = map_match_result->location_level;
  } else {
    MapMatchResultPtr map_match_result = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(map_match_result);
    if (map_match_result == nullptr) {
      return;
    }
    if (!map_match_result->has_header() ||
        !map_match_result->has_location_level()) {
      return;
    }
    if (fabs(time_now - map_match_result->header().measurement_timestamp()) >
        3) {
      return;
    }
    map_match_status_.valid = true;
    map_match_status_.header.recv_timestamp = time_now;
    map_match_status_.header.sequence_num =
        map_match_result->header().sequence_num();
    map_match_status_.header.timestamp =
        map_match_result->header().measurement_timestamp();
    map_match_status_.location_level =
        static_cast<cem::message::sensor::LocationLevel>(
            map_match_result->location_level());
  }
}

void MapSourceSwitch::LoadMapEvent() {
  map_event_ptr_ = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr_);
}

void MapSourceSwitch::JudgeDrivingScenario() {
  function_mode_ = fusion_mgr_->GetFunctionMode();
  if (function_mode_ == LdMapProcessor::FunctionMode::kHNOA) {
    is_on_freeway_ = fusion_mgr_->IsOnFreeway();
  } else {
    // 当上一帧和这一帧地图中给定城区高速切换时,考虑切换逻辑
    if (fusion_mgr_->IsOnFreeway() != is_on_freeway_) {
      // 城区/高速切换逻辑的情况
      // 1. 已经在用感知的话就直接切换
      // 2.
      // 城区用图，但是过了最后一个路口超过90米，直接切高速，不等到100m防止跳变
      // 3. 高速用图，但是即将靠近路口<200米，切城区过路口方案
      if (use_perception_ ||
          (!use_perception_ && (!is_on_freeway_) &&
           (distance_to_previous_junction_ > 90 &&
            distance_to_next_junction_ <= 100)) ||
          (!use_perception_ && is_on_freeway_ &&
           (distance_to_next_junction_ < 200))) {
        is_on_freeway_ = fusion_mgr_->IsOnFreeway();
      } else {
        // noting to do
      }
    }
  }
  env_status_->set_is_on_freeway(is_on_freeway_);
}

bool MapSourceSwitch::IsConditionSatisfiedToUseHdMap() {
  if (IsMapDataValid()) {
    if (!IsDowngradeNoaToICC()) {
      if (!IsNoaActivated()) {
        return false;
      }
    } else {
      return false;
    }
  } else {
    return false;
  }
  return true;
}

void MapSourceSwitch::EvaluateEgoLaneMatchStatus(
    const GeometryMatchResult& ego_lane_match_info) {
  // is_ego_lane_matched_ = ego_lane_match_info.is_matched;

  // env_status_->mutable_left_laneline_match_info()->set_average_offset(
  //     ego_lane_match_info.left_offset_average);
  // env_status_->mutable_left_laneline_match_info()->set_max_offset(
  //     ego_lane_match_info.left_offset_max);
  // env_status_->mutable_right_laneline_match_info()->set_average_offset(
  //     ego_lane_match_info.right_offset_average);
  // env_status_->mutable_right_laneline_match_info()->set_max_offset(
  //     ego_lane_match_info.right_offset_max);

  env_status_->clear_left_laneline_match_info();
  GeometryMatchResult result;
  fusion_mgr_->GetEgoLaneMatchResult(result);
  is_ego_lane_match_offset_in_range_ = result.is_matched;
  is_ego_lane_matched_ = is_ego_lane_match_offset_in_range_;
  if (is_ego_lane_match_offset_in_range_) {
    if (use_perception_) {
      if (std::abs(result.ld_line_c0) > 1) {
        is_ego_lane_matched_ = false;
      }
    } else {
      if (std::abs(result.bev_line_c0) > 1) {
        is_ego_lane_matched_ = false;
      }
    }
  }

  env_status_->mutable_left_laneline_match_info()->set_average_offset(
      result.left_offset_average);
  env_status_->mutable_left_laneline_match_info()->set_max_offset(
      result.left_offset_max);
}

void MapSourceSwitch::HNOAMapSourceSwitch() {
  if (is_in_area_should_use_hdmap_) {
    // 在匝道范围内
    if (!is_ego_lane_matched_ && use_perception_) {
      // 如果是切图帧没匹配上, 暂时不切图
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
    } else {
      // 此前也是用图, 或者切图帧且匹配上了, 就用图
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_IN_THE_RAMP);
      use_perception_ = false;
    }
  } else {
    // 非匝道范围内
    if (!use_perception_ && (ego_is_changing_lane_ || !is_ego_lane_matched_)) {
      // 切感知帧, 自车正在换道, 换道结束再切图
      if (ego_is_changing_lane_) {
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE);
      } else if (!is_ego_lane_matched_) {
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
      }
    } else {
      use_perception_ = true;
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_NOT_IN_THE_RAMP);
    }
  }
}

void MapSourceSwitch::CNOAMapSourceSwitch() {
  if (!use_perception_) {
    // 上一帧使用地图
    if (is_in_area_should_use_hdmap_) {
      // 1. 如果当前帧还在路口范围内, 维持用图。
      if (is_ego_in_junction_range_) {
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_IN_JUNCTION);
      } else {
        // 2. 如果当前帧在路口多岔口场景, 维持用图。
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_IN_MULTIPLE_CONSECUTIVE_ROAD_INTERSECTIONS);
      }
    } else {
      // 3. 如果当前帧不在路口范围内,也不在多分岔口场景下，考虑切图。
      if ((distance_to_next_junction_ - BEV_TO_MAP_DISTANCE_THRESHOLD <
           NEXT_USE_BEV_AREA_DISTANCE_LIMIT) &&
          (next_low_precision_start_offset_ >
           distance_to_section_over_first_junction_)) {
        // 3.1 本该切图了,
        // 但是距离下一个该切图的位置不足100米的话就别切感知了，怪麻烦
        // 的。当然，如果下一个路口之结束前有低精区域就不要因为怕小麻烦而引来了大麻烦
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_BETWEEN_TWO_VERY_CLOSED_JUNCTIONS);
      } else {
        // 3.2
        // 距离下个切图点还挺远的话，此时感知和地图能匹配上而且自车没有在换道，那就
        //     切感知, 否则仍然维持用图。
        if (!is_ego_lane_matched_) {
          env_status_->set_perception_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
        } else if (ego_is_changing_lane_) {
          env_status_->set_perception_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE);
        } else {
          use_perception_ = true;
          env_status_->set_hd_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_EGO_PASSED_JUNCTION);
        }
      }
    }
  } else {
    // 上一帧使用感知
    // 如果当前帧在路口中，或者在多岔口场景下
    if (is_in_area_should_use_hdmap_) {
      // 1. 如果当前帧在路口范围内, 且成功匹配, 则切图。
      if (is_ego_lane_matched_) {
        use_perception_ = false;
        if (is_ego_in_junction_range_) {
          env_status_->set_perception_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_EGO_IN_JUNCTION);
        } else {
          env_status_->set_perception_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_EGO_IN_MULTIPLE_CONSECUTIVE_ROAD_INTERSECTIONS);
        }
      } else {
        if (distance_to_next_junction_ <
                LD_MAP_TO_PERCEPTION_FAILURE_WARNING_LIMIT &&
            is_junction_from_hd_map_) {
          sensor_status_info_.warning_type =
              SensorWarningType::SWITCH_TO_HD_MAP_FAILURE;
        }
        env_status_->set_hd_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
      }
    } else {
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_NOT_CLOSE_ENOUGH_TO_JUNCTION);
    }
  }
}

bool MapSourceSwitch::IsMapDataValid() {
  auto routing_map = fusion_mgr_->GetLdMapInfo();
  // 检查地图数据有没有获取到
  if (routing_map == nullptr) {
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_NO_HD_MAP_FRAME);
    return false;
  }

  auto front_section_id = routing_map->route.navi_start.section_id;
  if (fusion_mgr_->GetLdSection(front_section_id) == nullptr) {
    // 检查地图route起始点是否正常。
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_NO_NAVIGATION_START_SECTION);
    return false;
  } else {
    // 检查地图剩余长度够不够，是否道路偏航
    auto front_section = fusion_mgr_->GetLdSection(front_section_id);

    // if (front_section->points.empty()) {
    //     AERROR << "navigation start section has no point.";
    //     return;
    // }

    bool ego_in_map_range = false;
    double distance_to_back =
        front_section->length - routing_map->route.navi_start.s_offset;
    auto cur_section = front_section;
    while (true) {
      if (distance_to_back > 50.0) {
        break;
      }
      if (cur_section->successor_section_id_list.empty()) {
        break;
      }
      bool found_successor_section = false;
      for (auto id : cur_section->successor_section_id_list) {
        if (fusion_mgr_->GetLdSection(id) == nullptr) {
          continue;
        }
        cur_section = fusion_mgr_->GetLdSection(id);
        distance_to_back += cur_section->length;
        found_successor_section = true;
        break;
      }
      if (!found_successor_section) {
        break;
      }
    }
    if (distance_to_back > 50) {
      ego_in_map_range = true;
    }
    if (ego_in_map_range) {
      /*
      double max_y = -1000.0;
      double min_y = 1000.0;
      for (auto& lane : routing_map->exp_trajectories) {
        bool is_lane_relative = false;
        if (lane.relative_lane_id.size() >= 3) {
          for (size_t i = 1; i < lane.relative_lane_id.size() - 1; ++i) {
            auto it = std::find(front_section->lane_ids.begin(),
                                front_section->lane_ids.end(),
                                lane.relative_lane_id.at(i));
            if (it != front_section->lane_ids.end()) {
              is_lane_relative = true;
              break;
            }
          }
        }
        if (is_lane_relative) {
          if (lane.points.empty()) {
            continue;
          }
          double tmp = 0.0;
          double min_dist = numeric_limits<double>::max();
          for (auto& p : lane.points) {
            Point pts;
            pts.x = p.x;
            pts.y = p.y;
            TransformPoint(&pts, T_dr2body);
            double dist = pts.x * pts.x + pts.y * pts.y;
            if (dist < min_dist) {
              tmp = pts.y;
              min_dist = dist;
            }
          }
          if (max_y < tmp) {
            max_y = tmp;
          }
          if (min_y > tmp) {
            min_y = tmp;
          }
        }
      }

      std::vector<cem::message::env_model::LaneInfo*> lanes;
      bool has_virtual_lane = false;
      for (auto id : front_section->lane_ids) {
        auto lane = std::find_if(
            routing_map->lanes.begin(), routing_map->lanes.end(),
            [id](const LaneInfo& lane) { return id == lane.id; });
        if (lane->junction_id != 0) {
          has_virtual_lane = true;
          break;
        }
        if (lane != routing_map->lanes.end()) {
          lanes.push_back(&(*lane));
        }
      }
      if (!has_virtual_lane) {
        for (auto lane : lanes) {
          if (lane->points.empty()) {
            continue;
          }
          double tmp = 0.0;
          double min_dist = numeric_limits<double>::max();
          for (auto& p : lane->points) {
            Point pts;
            pts.x = p.x;
            pts.y = p.y;
            TransformPoint(&pts, T_dr2body);
            double dist = pts.x * pts.x + pts.y * pts.y;
            if (dist < min_dist) {
              tmp = pts.y;
              min_dist = dist;
            }
          }

          if (max_y < tmp) {
            max_y = tmp;
          }
          if (min_y > tmp) {
            min_y = tmp;
          }
        }

        if (max_y < min_y) {
          ego_outside_map = true;
          ret = DowngradeReason::kOutsideMap;
        }
        if (((min_y > 2.5) || (max_y < -2.5)) && (max_y >= min_y)) {
          ego_outside_map = true;
          ret = DowngradeReason::kOutsideMap;
        }
      }
      // AINFO << "min_Y: " << min_y << ", max_Y: " << max_y;
    */
    } else {
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_APPROACH_MAP_END);
      return false;
    }
  }

  return true;
}

bool MapSourceSwitch::IsDowngradeNoaToICC() {
  sensor_status_info_.sensor_status = 0;
  sensor_status_info_.special_type = 0;
  sensor_status_info_.warning_type = SensorWarningType::NO_WARNING;

  if (IsDowngradeCausedByLowPrecisionZone()) {
    return true;
  } else {
    if (is_on_freeway_) {
      return IsDowngradeCausedByEgoLaneNotMatched();
    } else {
      return false;
      return IsDowngradeCausedByPerceptionMapStillInUseNearJunction();
    }
  }
}

bool MapSourceSwitch::IsDowngradeCausedByLowPrecisionZone() {
  if (is_on_freeway_) {
    is_in_area_should_use_hdmap_ = is_in_ramp_range_;
  } else {
    if (use_perception_ &&
        (env_status_->hd_map_suppression_reason() !=
         EnvStatus::MapSuppressionReason::
             EnvStatus_MapSuppressionReason_APPROACH_LOW_PRECISION_AREA_IN_HD_MAP)) {
      is_ego_in_junction_range_ =
          (distance_to_next_junction_ < BEV_TO_MAP_DISTANCE_THRESHOLD);
    } else {
      is_ego_in_junction_range_ =
          ((distance_to_previous_junction_ < MAP_TO_BEV_DISTANCE_THRESHOLD) ||
           (distance_to_next_junction_ < BEV_TO_MAP_DISTANCE_THRESHOLD));
    }
    is_in_area_should_use_hdmap_ =
        (is_ego_in_junction_range_ ||
         is_ego_in_multi_consecutive_road_intersections_);
  }

  next_low_precision_start_offset_ = std::numeric_limits<double>::max();
  if (nullptr == map_event_ptr_) {
    return false;
  }
  // 获取在本该用图的区域内的最近的一个低精odd信息
  double min_start_offset = std::numeric_limits<double>::max();
  double min_end_offset = std::numeric_limits<double>::lowest();
  for (const auto& odd : map_event_ptr_->odd_info) {
    if (cem::message::env_model::ValueType::LOW_PRECISION == odd.value) {
      // Get the min distance of the low precision.
      next_low_precision_start_offset_ =
          next_low_precision_start_offset_ < odd.start_offset
              ? next_low_precision_start_offset_
              : odd.start_offset;
      if (is_on_freeway_) {
        if (is_in_area_should_use_hdmap_) {
          if (min_start_offset > odd.start_offset) {
            min_start_offset = odd.start_offset;
            min_end_offset = odd.end_offset;
          }
        }
      } else {
        if (((distance_to_next_junction_ - BEV_TO_MAP_DISTANCE_THRESHOLD <
              odd.end_offset) &&
             (distance_to_section_over_first_junction_ +
                  MAP_TO_BEV_DISTANCE_THRESHOLD >
              odd.start_offset)) ||
            is_ego_in_multi_consecutive_road_intersections_) {
          if (min_start_offset > odd.start_offset) {
            min_start_offset = odd.start_offset;
            min_end_offset = odd.end_offset;
          }
        }
        if (distance_to_previous_junction_ < MAP_TO_BEV_DISTANCE_THRESHOLD) {
          if (odd.start_offset <
              MAP_TO_BEV_DISTANCE_THRESHOLD - distance_to_previous_junction_) {
            if (min_start_offset > odd.start_offset) {
              min_start_offset = odd.start_offset;
              min_end_offset = odd.end_offset;
            }
          }
        }
      }
    }
  }

  // 处理低精告警信息
  if ((min_start_offset < LD_MAP_LOW_PRECISION_DISTANCE_WARNING_LIMIT &&
       min_start_offset >= LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT)) {
    sensor_status_info_.warning_type =
        cem::message::env_model::SensorWarningType::CLOSE_TO_UNRELIABLE_ROAD;
    return false;
  }

  // 处理低精降级信息
  if (!is_in_area_should_use_hdmap_) {
    return false;
  }

  if (min_start_offset < LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT) {
    sensor_status_info_.sensor_status = 1;
    sensor_status_info_.special_type |=
        static_cast<uint64_t>(SpecialType::ST_LD_MAP_LOW_PRECISION);
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_APPROACH_LOW_PRECISION_AREA_IN_HD_MAP);
    return true;
  } else {
    return false;
  }
}

bool MapSourceSwitch::IsDowngradeCausedByPerceptionMapStillInUseNearJunction() {
  if (is_on_freeway_ || !use_perception_) {
    return false;
  }

  if (distance_to_next_junction_ < BEV_TO_MAP_DOWNGRADE_DISTANCE_THRESHOLD) {
    sensor_status_info_.sensor_status = 1;
    sensor_status_info_.special_type = 2;
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_UNABLE_TO_USE_HD_MAP_NEAR_JUNCTION);
    return true;
  }
  return false;
}

bool MapSourceSwitch::IsDowngradeCausedByEgoLaneNotMatched() {
  if (!is_on_freeway_ ||
      (use_perception_ && localization_not_reliable_frame_cnt_ <= 3) ||
      !map_match_status_.valid) {
    localization_not_reliable_frame_cnt_ = 0;
    return false;
  }

  if (localization_not_reliable_frame_cnt_ <= 3) {
    if ((map_match_status_.location_level !=
         cem::message::sensor::LocationLevel::Roadlevel) ||
        is_ego_lane_matched_) {
      localization_not_reliable_frame_cnt_ = 0;
      return false;
    }
  } else {
    if ((map_match_status_.location_level !=
         cem::message::sensor::LocationLevel::Roadlevel) &&
        is_ego_lane_matched_) {
      localization_not_reliable_frame_cnt_ = 0;
      return false;
    }
  }

  localization_not_reliable_frame_cnt_++;
  if (localization_not_reliable_frame_cnt_ > 3) {
    sensor_status_info_.sensor_status = 1;
    sensor_status_info_.special_type = 4;
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_LOCALIZATION_IS_NOT_RELIABLE);
    return true;
  } else {
    return false;
  }
}

bool MapSourceSwitch::IsNoaActivated() {
  CAN1Ptr canoutptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(canoutptr);
  if (canoutptr != nullptr) {
    if (canoutptr->DNP_Stats_S != 0x6) {
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_NOT_NOA_MODE);
      return false;
    }
  }
  return true;
}

bool MapSourceSwitch::IsEgoCloseToUnreliableRoad() {
  // 预留函数，
  auto routing_map = fusion_mgr_->GetLdMapInfo();
  if (routing_map == nullptr || is_on_freeway_) {
    env_status_->set_distance_to_unreliable_road(2000.0);
  } else {
    env_status_->set_distance_to_unreliable_road(
        fusion_mgr_->GetDistanceToUnreliableRoad());
  }
  return false;
}

void MapSourceSwitch::CopySensorStatusInfoTo(
    cem::fusion::RoutingMapPtr& routing_map) {
  if (routing_map == nullptr) {
    return;
  }
  routing_map->sensor_status_info.sensor_status =
      sensor_status_info_.sensor_status;
  routing_map->sensor_status_info.special_type =
      sensor_status_info_.special_type;
  routing_map->sensor_status_info.warning_type =
      sensor_status_info_.warning_type;
}

void MapSourceSwitch::ProcessSpecialJunctionInfo() {
  if (std::abs(distance_to_next_junction_) < 1e-5) {
    return;
  }

  auto routing_map = fusion_mgr_->GetLdMapInfo();
  if (routing_map == nullptr) {
    return;
  }

  auto front_section_id = routing_map->route.navi_start.section_id;
  if (fusion_mgr_->GetLdSection(front_section_id) == nullptr) {
    return;
  }

  const SectionInfo* front_section =
      fusion_mgr_->GetLdSection(front_section_id);
  double distance_to_back = -routing_map->route.navi_start.s_offset;

  bool found_special_junction = false;
  bool found_entire_special_junction = false;
  const SectionInfo* cur_section = front_section;
  while (true) {
    if (distance_to_back > 200.0) {
      break;
    }
    bool is_cur_section_are_junction =
        ((cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_ROUNDABOUT)) ||
         (cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_IC)) ||
         (cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_JCT)));
    if (!found_special_junction) {
      if (distance_to_back >= distance_to_next_junction_) {
        break;
      }
      if (is_cur_section_are_junction) {
        distance_to_next_junction_ = distance_to_back;
        found_special_junction = true;
      }
    } else {
      if (!is_cur_section_are_junction) {
        distance_to_section_over_first_junction_ = distance_to_back;
        found_entire_special_junction = true;
        break;
      }
    }

    distance_to_back += cur_section->length;
    bool found_successor_section = false;
    for (auto id : cur_section->successor_section_id_list) {
      if (fusion_mgr_->GetLdSection(id) == nullptr) {
        continue;
      }
      cur_section = fusion_mgr_->GetLdSection(id);
      found_successor_section = true;
      break;
    }
    if (!found_successor_section) {
      break;
    }
  }

  if (found_special_junction && (!found_entire_special_junction)) {
    distance_to_section_over_first_junction_ = distance_to_back;
  }
  distance_to_next_junction_ =
      distance_to_next_junction_ < 0.0 ? 0.0 : distance_to_next_junction_;

  double distance_to_front = routing_map->route.navi_start.s_offset;
  bool is_junction_continuous =
      ((front_section->link_type &
        static_cast<uint32_t>(
            cem::message::env_model::LDLinkTypeMask::LT_ROUNDABOUT)) ||
       (front_section->link_type &
        static_cast<uint32_t>(
            cem::message::env_model::LDLinkTypeMask::LT_IC)) ||
       (front_section->link_type &
        static_cast<uint32_t>(
            cem::message::env_model::LDLinkTypeMask::LT_JCT)));
  cur_section = front_section;
  while (true) {
    if (distance_to_front >= distance_to_previous_junction_ ||
        distance_to_front > 100.0) {
      break;
    }
    bool found_predecessor_section = false;
    for (auto id : cur_section->predecessor_section_id_list) {
      if (fusion_mgr_->GetLdSection(id) == nullptr) {
        continue;
      }
      cur_section = fusion_mgr_->GetLdSection(id);
      found_predecessor_section = true;
      break;
    }
    if (!found_predecessor_section) {
      break;
    }

    bool is_cur_section_are_junction =
        ((cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_ROUNDABOUT)) ||
         (cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_IC)) ||
         (cur_section->link_type &
          static_cast<uint32_t>(
              cem::message::env_model::LDLinkTypeMask::LT_JCT)));

    if (is_cur_section_are_junction) {
      if (is_junction_continuous) {
        distance_to_front += cur_section->length;
      } else {
        distance_to_previous_junction_ = distance_to_front;
        break;
      }
    } else {
      distance_to_front += cur_section->length;
      is_junction_continuous = false;
    }
  }

  is_junction_from_hd_map_ = true;
}

void MapSourceSwitch::ForceChangeSourceToPerceptionMap() {
  if (use_perception_) {
    route_snapshot_for_perception_map_switch_failure_.clear();
    return;
  }
  auto routing_map = fusion_mgr_->GetLdMapInfo();
  if (routing_map == nullptr) {
    route_snapshot_for_perception_map_switch_failure_.clear();
    use_perception_ = true;
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_NO_HD_MAP_FRAME);
    return;
  }
  if (env_status_->perception_map_suppression_reason() ==
      EnvStatus::MapSuppressionReason::
          EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP) {
    if (next_low_precision_start_offset_ <
        LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT) {
      route_snapshot_for_perception_map_switch_failure_.clear();
      use_perception_ = true;
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_LOW_PRECISION_ZONE);
      return;
    }

    auto front_section_id = routing_map->route.navi_start.section_id;
    if (fusion_mgr_->GetLdSection(front_section_id) == nullptr) {
      route_snapshot_for_perception_map_switch_failure_.clear();
      use_perception_ = true;
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_NO_NAVIGATION_START_SECTION);

      return;
    }
    auto front_section = fusion_mgr_->GetLdSection(front_section_id);

    if (route_snapshot_for_perception_map_switch_failure_.empty()) {
      route_snapshot_for_perception_map_switch_failure_.push_back(
          std::move(std::pair<uint64_t, double>(
              front_section_id,
              front_section->length - routing_map->route.navi_start.s_offset)));
      double total_length =
          route_snapshot_for_perception_map_switch_failure_.front().second;

      auto cur_section = front_section;
      while (total_length < 200) {
        bool found_successor_section = false;
        for (auto id : cur_section->successor_section_id_list) {
          if (fusion_mgr_->GetLdSection(id) == nullptr) {
            continue;
          }
          cur_section = fusion_mgr_->GetLdSection(id);
          found_successor_section = true;
          break;
        }
        if (!found_successor_section) {
          break;
        } else {
          route_snapshot_for_perception_map_switch_failure_.push_back(
              std::move(std::pair<uint64_t, double>(cur_section->id,
                                                    cur_section->length)));
          total_length += cur_section->length;
        }
      }
      return;
    } else {
      bool is_snapshot_valid = false;
      double switch_failure_distance =
          -(front_section->length - routing_map->route.navi_start.s_offset);
      for (auto& section : route_snapshot_for_perception_map_switch_failure_) {
        switch_failure_distance += section.second;
        if (section.first == front_section->id) {
          is_snapshot_valid = true;
          break;
        }
      }
      if (!is_snapshot_valid) {
        route_snapshot_for_perception_map_switch_failure_.clear();
        use_perception_ = true;
        env_status_->set_hd_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_HISTORY_ROUTE_INVALID);
        return;
      } else {
        if (switch_failure_distance >
            LD_MAP_TO_PERCEPTION_FORCE_SWITCH_THRESHOLD) {
          route_snapshot_for_perception_map_switch_failure_.clear();
          use_perception_ = true;
          env_status_->set_hd_map_suppression_reason(
              EnvStatus::MapSuppressionReason::
                  EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_NOT_MATCHED_WITH_LONG_DISTANCE);
          return;
        }
      }
    }
  } else if (env_status_->perception_map_suppression_reason() ==
             EnvStatus::MapSuppressionReason::
                 EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE) {
    if (next_low_precision_start_offset_ <
        LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT) {
      route_snapshot_for_perception_map_switch_failure_.clear();
      use_perception_ = true;
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_LOW_PRECISION_ZONE);
      return;
    }
  } else {
    if (is_ego_lane_match_offset_in_range_) {
      route_snapshot_for_perception_map_switch_failure_.clear();
    }
  }
}

}  // namespace fusion
}  // namespace cem
