#pragma once

#include <limits>

#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/sensor/localization/map_match.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"
#include "lib/perception_and_ld_map_fusion/fusion_manager.h"
#include "modules/msg/localization_msgs/map_match.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"

namespace cem {
namespace fusion {

class MapSourceSwitch {
 public:
  MapSourceSwitch();

  ~MapSourceSwitch();

  void Init(const FusionManager* fusion_mgr);


  void Process();

  void SetRampFlag(bool flag);

  void SetMultipleConsecutiveRoadIntersectionsFlag(bool flag);

  void SetJunctionInfo(double distance_to_previous_junction,
                       double distance_to_next_junction,
                       double distance_to_section_over_first_junction,
                       bool is_junction_from_hd_map);

  void SetEgoLaneMatchingInfo(const GeometryMatchResult& ego_lane_match_info);


  inline bool IsUsePerceptionMap() const { return use_perception_; };

  void CopySensorStatusInfoTo(cem::fusion::RoutingMapPtr& routing_map);

  inline std::shared_ptr<byd::msg::orin::routing_map::EnvStatus>
  GetEnvStatus() {
    return env_status_;
  };

 private:
  void SetEgoLaneChangingStatus();

  void SetMapMatchStatus();

  void LoadMapEvent();

  void JudgeDrivingScenario();

  bool IsConditionSatisfiedToUseHdMap();

  void EvaluateEgoLaneMatchStatus(
      const GeometryMatchResult& ego_lane_match_info);

  void HNOAMapSourceSwitch();

  void CNOAMapSourceSwitch();

  bool IsDowngradeNoaToICC();

  bool IsDowngradeCausedByLowPrecisionZone();

  bool IsDowngradeCausedByPerceptionMapStillInUseNearJunction();

  bool IsDowngradeCausedByEgoLaneNotMatched();

  bool IsMapDataValid();

  bool IsNoaActivated();

  bool IsEgoCloseToUnreliableRoad();

  void ProcessSpecialJunctionInfo();

  void ForceChangeSourceToPerceptionMap();

  void CorrectJunctionInfoByHistory(double distance_to_next_junction);

 private:
  bool is_ego_lane_match_offset_in_range_ = false;
  bool is_ego_lane_matched_ = false;
  bool is_in_ramp_range_ = false;

  bool is_ego_in_junction_range_ = false;
  bool is_ego_in_multi_consecutive_road_intersections_ = false;

  bool ego_is_changing_lane_ = false;
  bool use_perception_ = true;
  bool is_on_freeway_ = false;
  bool is_in_area_should_use_hdmap_ = false;
  double distance_to_previous_junction_ =
      std::numeric_limits<double>::infinity();
  double distance_to_next_junction_ = std::numeric_limits<double>::infinity();
  double distance_to_section_over_first_junction_ =
      std::numeric_limits<double>::infinity();
  double next_low_precision_start_offset_ = std::numeric_limits<double>::max();

  const double MAP_TO_BEV_DISTANCE_THRESHOLD = 100.0;
  const double BEV_TO_MAP_DISTANCE_THRESHOLD = 200.0;
  const double NEXT_USE_BEV_AREA_DISTANCE_LIMIT = 100.0;
  const double BEV_TO_MAP_DOWNGRADE_DISTANCE_THRESHOLD = 50.0;

  const double LD_MAP_LOW_PRECISION_DISTANCE_DOWNGRADE_LIMIT = 100.0;
  const double LD_MAP_LOW_PRECISION_DISTANCE_WARNING_LIMIT = 300.0;
  const double LD_MAP_NO_MAP_DATA_LIMIT = 50.0;
  const double LD_MAP_TO_PERCEPTION_FORCE_SWITCH_THRESHOLD = 40.0;
  const double LD_MAP_TO_PERCEPTION_FAILURE_WARNING_LIMIT = 50.0;
  // const double APPROCH_NO_LD_MAP_DISTANCE_LIMIT = 150.0f;

  const FusionManager* fusion_mgr_ = nullptr;
  std::shared_ptr<byd::msg::orin::routing_map::EnvStatus> env_status_ = nullptr;
  cem::message::env_model::SensorStatusInfo sensor_status_info_;

  LdMapProcessor::FunctionMode function_mode_ =
      LdMapProcessor::FunctionMode::kMNOA;

  cem::message::sensor::MapMatchResult map_match_status_;

  int localization_not_reliable_frame_cnt_ = 0;

  MapEventPtr map_event_ptr_ = nullptr;

  std::vector<std::pair<uint64_t, double>>
      route_snapshot_for_perception_map_switch_failure_;

  bool is_junction_from_hd_map_;
};

} // namespace fusion
} // namespace cem