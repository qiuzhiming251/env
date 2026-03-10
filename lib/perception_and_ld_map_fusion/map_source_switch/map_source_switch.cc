#include "lib/perception_and_ld_map_fusion/map_source_switch/map_source_switch.h"

#include <cmath>

using namespace byd::msg::orin::routing_map;

namespace cem {
namespace fusion {

namespace {

constexpr double kMapMatchTimestampToleranceSec = 3.0;
constexpr double kMapDataMinAvailableLength = 50.0;
constexpr double kSpecialJunctionSearchDistance = 200.0;
constexpr double kPreviousJunctionSearchDistance = 100.0;
constexpr double kSwitchFailureSnapshotLength = 200.0;
constexpr int kLocalizationNotReliableFrameThreshold = 3;

bool IsJunctionSection(const SectionInfo* section) {
  return (section->link_type & static_cast<uint32_t>(
                                   cem::message::env_model::LDLinkTypeMask::
                                       LT_ROUNDABOUT)) ||
         (section->link_type & static_cast<uint32_t>(
                                   cem::message::env_model::LDLinkTypeMask::
                                       LT_IC)) ||
         (section->link_type & static_cast<uint32_t>(
                                   cem::message::env_model::LDLinkTypeMask::
                                       LT_JCT));
}

const SectionInfo* GetFirstValidLinkedSection(
    const std::vector<uint64_t>& linked_section_ids,
    const FusionManager* fusion_mgr) {
  for (const auto id : linked_section_ids) {
    const auto* section = fusion_mgr->GetLdSection(id);
    if (section != nullptr) {
      return section;
    }
  }
  return nullptr;
}

void ForceUsePerceptionMap(
    std::vector<std::pair<uint64_t, double>>& route_snapshot,
    bool& use_perception, const std::shared_ptr<EnvStatus>& env_status,
    const EnvStatus::MapSuppressionReason reason) {
  route_snapshot.clear();
  use_perception = true;
  env_status->set_hd_map_suppression_reason(reason);
}

void BuildSwitchFailureRouteSnapshot(
    const FusionManager* fusion_mgr, const SectionInfo* front_section,
    const double front_section_remaining_length,
    std::vector<std::pair<uint64_t, double>>& route_snapshot) {
  route_snapshot.clear();
  route_snapshot.emplace_back(front_section->id, front_section_remaining_length);
  double total_length = route_snapshot.front().second;
  const SectionInfo* cur_section = front_section;
  while (total_length < kSwitchFailureSnapshotLength) {
    const auto* successor_section = GetFirstValidLinkedSection(
        cur_section->successor_section_id_list, fusion_mgr);
    if (successor_section == nullptr) {
      break;
    }
    cur_section = successor_section;
    route_snapshot.emplace_back(cur_section->id, cur_section->length);
    total_length += cur_section->length;
  }
}

bool ComputeSwitchFailureDistance(
    const std::vector<std::pair<uint64_t, double>>& route_snapshot,
    const uint64_t front_section_id, const double front_section_remaining_length,
    double* switch_failure_distance) {
  if (switch_failure_distance == nullptr) {
    return false;
  }
  *switch_failure_distance = -front_section_remaining_length;
  for (const auto& section : route_snapshot) {
    *switch_failure_distance += section.second;
    if (section.first == front_section_id) {
      return true;
    }
  }
  return false;
}

}  // namespace

MapSourceSwitch::MapSourceSwitch() {}

MapSourceSwitch::~MapSourceSwitch() {}

void MapSourceSwitch::Init(const FusionManager* fusion_mgr) {
  fusion_mgr_ = fusion_mgr;
  env_status_ = std::make_shared<EnvStatus>();
}

void MapSourceSwitch::Process() {
  // env_status_->Clear();
  env_status_->clear_debug_info();

  SetEgoLaneChangingStatus();
  JudgeDrivingScenario();
  SetMapMatchStatus();
  LoadMapEvent();

  GeometryMatchResult result;
  SetEgoLaneMatchingInfo(result);

  if (!IsConditionSatisfiedToUseHdMap()) {
    use_perception_ = true;
    IsEgoCloseToUnreliableRoad();
    FinalizeMapSourceStatus();
    return;
  }

  if (is_on_freeway_) {
    HNOAMapSourceSwitch();
  } else {
    CNOAMapSourceSwitch();
  }
  ForceChangeSourceToPerceptionMap();

  IsEgoCloseToUnreliableRoad();
  FinalizeMapSourceStatus();
}

void MapSourceSwitch::FinalizeMapSourceStatus() {
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
  const auto time_now = GetMwTimeNowSec();

  auto update_map_match_status =
      [this, time_now](const uint64_t sequence_num, const double timestamp,
                       const cem::message::sensor::LocationLevel level) {
        map_match_status_.valid = true;
        map_match_status_.header.recv_timestamp = time_now;
        map_match_status_.header.sequence_num = sequence_num;
        map_match_status_.header.timestamp = timestamp;
        map_match_status_.location_level = level;
      };

  if (function_mode_ == LdMapProcessor::FunctionMode::kHNOA) {
    MapMatchResultBaiduPtr map_match_result = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(map_match_result);
    if (map_match_result == nullptr) {
      return;
    }
    if (std::fabs(time_now - map_match_result->header.timestamp) >
        kMapMatchTimestampToleranceSec) {
      return;
    }
    update_map_match_status(map_match_result->header.sequence_num,
                            map_match_result->header.timestamp,
                            map_match_result->location_level);
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
    if (std::fabs(time_now -
                  map_match_result->header().measurement_timestamp()) >
        kMapMatchTimestampToleranceSec) {
      return;
    }
    update_map_match_status(
        map_match_result->header().sequence_num(),
        map_match_result->header().measurement_timestamp(),
        static_cast<cem::message::sensor::LocationLevel>(
            map_match_result->location_level()));
  }
}

void MapSourceSwitch::LoadMapEvent() {
  map_event_ptr_ = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr_);
}

void MapSourceSwitch::JudgeDrivingScenario() {
  function_mode_ = fusion_mgr_->GetFunctionMode();
  const bool latest_is_on_freeway = fusion_mgr_->IsOnFreeway();
  if (function_mode_ == LdMapProcessor::FunctionMode::kHNOA) {
    is_on_freeway_ = latest_is_on_freeway;
  } else {
    // 当上一帧和这一帧地图中给定城区高速切换时,考虑切换逻辑
    if (latest_is_on_freeway != is_on_freeway_) {
      // 城区/高速切换逻辑的情况
      // 1. 已经在用感知的话就直接切换
      // 2.
      // 城区用图，但是过了最后一个路口超过90米，直接切高速，不等到100m防止跳变
      // 3. 高速用图，但是即将靠近路口<200米，切城区过路口方案
      const bool should_switch_to_latest_scenario =
          use_perception_ ||
          (!use_perception_ && (!is_on_freeway_) &&
           (distance_to_previous_junction_ > 90 &&
            distance_to_next_junction_ <= 100)) ||
          (!use_perception_ && is_on_freeway_ &&
           (distance_to_next_junction_ < 200));
      if (should_switch_to_latest_scenario) {
        is_on_freeway_ = latest_is_on_freeway;
      } else {
        // noting to do
      }
    }
  }
  env_status_->set_is_on_freeway(is_on_freeway_);
}

bool MapSourceSwitch::IsConditionSatisfiedToUseHdMap() {
  if (!IsMapDataValid()) {
    return false;
  }
  if (IsDowngradeNoaToICC()) {
    return false;
  }
  if (!IsNoaActivated()) {
    return false;
  }
  return true;
}

void MapSourceSwitch::EvaluateEgoLaneMatchStatus(
    const GeometryMatchResult& ego_lane_match_info) {
  (void)ego_lane_match_info;
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
    SD_COARSE_MATCH_LOG << "[EvaluateEgoLaneMatchStatus] GeometryMatchResult: "
                      << "fail_reason=" << static_cast<int>(result.fail_reason) << ", is_matched=" << result.is_matched
                                            << ", left_offset_average=" << result.left_offset_average << ", left_offset_max=" << result.left_offset_max
                      << ", right_offset_average=" << result.right_offset_max
                      << ", ld_line_c0=" << result.ld_line_c0 << ", bev_line_c0=" << result.bev_line_c0;

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
    const bool cannot_switch_to_hd_this_frame =
        !is_ego_lane_matched_ && use_perception_;
    if (cannot_switch_to_hd_this_frame) {
      env_status_->set_hd_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
      return;
    }

    env_status_->set_perception_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_EGO_IN_THE_RAMP);
    use_perception_ = false;
    return;
  }

  const bool keep_hd_map_temporarily =
      !use_perception_ && (ego_is_changing_lane_ || !is_ego_lane_matched_);
  if (keep_hd_map_temporarily) {
    if (ego_is_changing_lane_) {
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE);
    } else if (!is_ego_lane_matched_) {
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
    }
    return;
  }

  use_perception_ = true;
  env_status_->set_hd_map_suppression_reason(
      EnvStatus::MapSuppressionReason::
          EnvStatus_MapSuppressionReason_EGO_NOT_IN_THE_RAMP);
}
void MapSourceSwitch::CNOAMapSourceSwitch() {
  if (!use_perception_) {
    if (is_in_area_should_use_hdmap_) {
      if (is_ego_in_junction_range_) {
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_IN_JUNCTION);
      } else {
        env_status_->set_perception_map_suppression_reason(
            EnvStatus::MapSuppressionReason::
                EnvStatus_MapSuppressionReason_EGO_IN_MULTIPLE_CONSECUTIVE_ROAD_INTERSECTIONS);
      }
      return;
    }

    const bool close_to_next_switch_point =
        (distance_to_next_junction_ - BEV_TO_MAP_DISTANCE_THRESHOLD <
         NEXT_USE_BEV_AREA_DISTANCE_LIMIT) &&
        (next_low_precision_start_offset_ >
         distance_to_section_over_first_junction_);
    if (close_to_next_switch_point) {
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_BETWEEN_TWO_VERY_CLOSED_JUNCTIONS);
      return;
    }

    if (!is_ego_lane_matched_) {
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
      return;
    }
    if (ego_is_changing_lane_) {
      env_status_->set_perception_map_suppression_reason(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE);
      return;
    }

    use_perception_ = true;
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_EGO_PASSED_JUNCTION);
    return;
  }

  if (!is_in_area_should_use_hdmap_) {
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_EGO_NOT_CLOSE_ENOUGH_TO_JUNCTION);
    return;
  }

  if (!is_ego_lane_matched_) {
    if (distance_to_next_junction_ <
            LD_MAP_TO_PERCEPTION_FAILURE_WARNING_LIMIT &&
        is_junction_from_hd_map_) {
      sensor_status_info_.warning_type =
          SensorWarningType::SWITCH_TO_HD_MAP_FAILURE;
    }
    env_status_->set_hd_map_suppression_reason(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP);
    return;
  }

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
    while (distance_to_back <= kMapDataMinAvailableLength) {
      const auto* next_section = GetFirstValidLinkedSection(
          cur_section->successor_section_id_list, fusion_mgr_);
      if (next_section == nullptr) {
        break;
      }
      cur_section = next_section;
      distance_to_back += cur_section->length;
    }
    ego_in_map_range = (distance_to_back > kMapDataMinAvailableLength);
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
  }

  if (is_on_freeway_) {
    return IsDowngradeCausedByEgoLaneNotMatched();
  }

  // Keep legacy CNOA behavior unchanged.
  return false;
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
      (use_perception_ &&
       localization_not_reliable_frame_cnt_ <=
           kLocalizationNotReliableFrameThreshold) ||
      !map_match_status_.valid) {
    localization_not_reliable_frame_cnt_ = 0;
    return false;
  }

  if (localization_not_reliable_frame_cnt_ <=
      kLocalizationNotReliableFrameThreshold) {
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
  if (localization_not_reliable_frame_cnt_ >
      kLocalizationNotReliableFrameThreshold) {
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
    if (distance_to_back > kSpecialJunctionSearchDistance) {
      break;
    }
    const bool is_cur_section_junction = IsJunctionSection(cur_section);
    if (!found_special_junction) {
      if (distance_to_back >= distance_to_next_junction_) {
        break;
      }
      if (is_cur_section_junction) {
        distance_to_next_junction_ = distance_to_back;
        found_special_junction = true;
      }
    } else {
      if (!is_cur_section_junction) {
        distance_to_section_over_first_junction_ = distance_to_back;
        found_entire_special_junction = true;
        break;
      }
    }

    distance_to_back += cur_section->length;
    const auto* successor_section = GetFirstValidLinkedSection(
        cur_section->successor_section_id_list, fusion_mgr_);
    if (successor_section == nullptr) {
      break;
    }
    cur_section = successor_section;
  }

  if (found_special_junction && (!found_entire_special_junction)) {
    distance_to_section_over_first_junction_ = distance_to_back;
  }
  distance_to_next_junction_ =
      distance_to_next_junction_ < 0.0 ? 0.0 : distance_to_next_junction_;

  double distance_to_front = routing_map->route.navi_start.s_offset;
  bool is_junction_continuous = IsJunctionSection(front_section);
  cur_section = front_section;
  while (true) {
    if (distance_to_front >= distance_to_previous_junction_ ||
        distance_to_front > kPreviousJunctionSearchDistance) {
      break;
    }
    const auto* predecessor_section = GetFirstValidLinkedSection(
        cur_section->predecessor_section_id_list, fusion_mgr_);
    if (predecessor_section == nullptr) {
      break;
    }
    cur_section = predecessor_section;

    const bool is_cur_section_junction = IsJunctionSection(cur_section);

    if (is_cur_section_junction) {
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
  auto force_use_perception =
      [this](const EnvStatus::MapSuppressionReason reason) {
        ForceUsePerceptionMap(route_snapshot_for_perception_map_switch_failure_,
                              use_perception_, env_status_, reason);
      };

  if (use_perception_) {
    route_snapshot_for_perception_map_switch_failure_.clear();
    return;
  }

  auto routing_map = fusion_mgr_->GetLdMapInfo();
  if (routing_map == nullptr) {
    force_use_perception(
        EnvStatus::MapSuppressionReason::
            EnvStatus_MapSuppressionReason_NO_HD_MAP_FRAME);
    return;
  }

  const auto suppression_reason =
      env_status_->perception_map_suppression_reason();
  if (suppression_reason ==
      EnvStatus::MapSuppressionReason::
          EnvStatus_MapSuppressionReason_UNABLE_TO_MATCH_PERCEPTION_WITH_HD_MAP) {
    if (next_low_precision_start_offset_ <
        LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT) {
      force_use_perception(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_LOW_PRECISION_ZONE);
      return;
    }

    const auto front_section_id = routing_map->route.navi_start.section_id;
    const auto* front_section = fusion_mgr_->GetLdSection(front_section_id);
    if (front_section == nullptr) {
      force_use_perception(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_NO_NAVIGATION_START_SECTION);
      return;
    }

    const double front_section_remaining_length =
        front_section->length - routing_map->route.navi_start.s_offset;

    if (route_snapshot_for_perception_map_switch_failure_.empty()) {
      BuildSwitchFailureRouteSnapshot(
          fusion_mgr_, front_section, front_section_remaining_length,
          route_snapshot_for_perception_map_switch_failure_);
      return;
    }

    double switch_failure_distance = 0.0;
    const bool is_snapshot_valid = ComputeSwitchFailureDistance(
        route_snapshot_for_perception_map_switch_failure_, front_section->id,
        front_section_remaining_length, &switch_failure_distance);
    if (!is_snapshot_valid) {
      force_use_perception(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_HISTORY_ROUTE_INVALID);
      return;
    }
    if (switch_failure_distance >
        LD_MAP_TO_PERCEPTION_FORCE_SWITCH_THRESHOLD) {
      force_use_perception(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_NOT_MATCHED_WITH_LONG_DISTANCE);
      return;
    }
    return;
  }

  if (suppression_reason ==
      EnvStatus::MapSuppressionReason::
          EnvStatus_MapSuppressionReason_EGO_IS_CHANGING_LANE) {
    if (next_low_precision_start_offset_ <
        LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT) {
      force_use_perception(
          EnvStatus::MapSuppressionReason::
              EnvStatus_MapSuppressionReason_FORCE_USE_PERCEPTION_MAP_SINCE_LOW_PRECISION_ZONE);
    }
    return;
  }

  if (is_ego_lane_match_offset_in_range_) {
    route_snapshot_for_perception_map_switch_failure_.clear();
  }
}

}  // namespace fusion
}  // namespace cem
