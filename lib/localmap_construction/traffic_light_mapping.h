/**
 * @file traffic_light_mapping.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-11
 * 
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 * 
 */
#ifndef TRAFFIC_LIGHT_MAPPING_H_
#define TRAFFIC_LIGHT_MAPPING_H_

#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>

#include <cyber/common/log.h>
#include <localmap_construction/cloud_config_traffic_light.h>

#include "Eigen/Dense"
#include "lib/common/log_custom.h"
#include "lib/localmap_construction/traffic_light_tracker.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/internal_message.h"
#include "lib/message/sensor/vision/tsrmobject.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/perception/env/src/lib/localmap_construction/traffic_light_common.h"
#include "modules/perception/env/src/lib/localmap_construction/traffic_light_map_topo.h"
#include "modules/perception/env/src/lib/sd_navigation/SdNavigationCity.h"

namespace cem::fusion {
using byd::common::math::LineSegment2d;
using byd::common::math::Polygon2d;
using cem::message::sensor::TrfObjectInfo;
using SequencenumObjId = std::pair<uint64_t, uint64_t>;

#if defined(TRAFFIC_LIGHT_LOG) && (TRAFFIC_LIGHT_LOG)
#define TRAFFIC_LOG LOG_LP
#else
#define TRAFFIC_LOG NULL_LOG
#endif

#if defined(TRAFFIC_LIGHT_LOG) && (TRAFFIC_LIGHT_LOG)
#define LD_TRAFFIC_LOG LOG_LP
#else
#define LD_TRAFFIC_LOG NULL_LOG
#endif

#if defined(TRAFFIC_REC) && (TRAFFIC_REC)
#define TRAFFIC_REC_LOG LOG_LP
#else
#define TRAFFIC_REC_LOG NULL_LOG
#endif

constexpr double lane_belong_traffic_light_threshold = 48.0;
constexpr double dis_crosswalk_right_dedicated       = 14.0;
const uint32_t   max_vec_size                        = 10;
constexpr double lane_virtual_dis_threshold          = 50.0;
constexpr double lane_uturn_dis_threshold            = 30.0;
constexpr double dis_novirtual_light                 = 25.0;
constexpr double dis_stopline_pan                    = 0.3;

class TrafficLightMapping {
 public:
  TrafficLightMapping()  = default;
  ~TrafficLightMapping() = default;

  void Process(const RoutingMapPtr &routing_map_input, const RoutingMapPtr &routing_map_output, const BevMapInfoPtr &bev_map,
               const std::shared_ptr<navigation::SdNavigationCity> &sd_navigation_city, BevMapInfoConstPtr &bev_raw,
               const Eigen::Isometry3d &Tbw);

  double timestamp() const { return timestamp_; }

  uint32_t GetPerceptionCyberCounter() const { return cycle_counter_perception_obj_; }

  double DistanceToJunction() const { return distance_to_junction_; }

  double DistanceToSectionOverFirstJunction() const { return distance_to_section_over_first_junction_; };

  const std::string &TrafficLightLaneDebug() const { return info_traffic_lights_; }

  bool IsUnreliableRoadCrossJunction();

  const TrafficLightsE2EInfoPtr &GetTrafficLightsE2E() { return traffic_light_e2e_; }

  double GetPreviousJunctionEgoPassed();

  std::optional<TrfObjectInfo> GetTrafficLightBaseTurnType(TurnType turn_type) {
    const auto &light_t = traffic_lights_.GetLightState(turn_type);
    if (light_t.is_valid && light_t.traffic_obj_info) {
      return *light_t.traffic_obj_info;
    }
    if (!deal_per_traffic_light_objects_.empty()) {
      return deal_per_traffic_light_objects_.begin()->second;
    }
    return std::nullopt;
  };

  static std::tuple<uint64_t, bool, bool> LaneStopLineInfo(const std::vector<LaneInfo>     &lanes_output,
                                                           const std::vector<StopLineInfo> &stop_lines, uint64_t id_temp);

  ::byd::msg::orin::routing_map::TrafficLightStatus GetLeftStatus() const { return light_left_; };
  ::byd::msg::orin::routing_map::TrafficLightStatus GetUturnStatus() const { return light_uturn_; };
  ::byd::msg::orin::routing_map::TrafficLightStatus GetStraightStatus() const { return light_straight_; };
  ::byd::msg::orin::routing_map::TrafficLightStatus GetRightStatus() const { return light_right_; };

  void SetEgoLdLaneId(uint64_t id) { ego_ld_lane_id_ = id; }

  bool IsNextSectionHasJunction() const { return is_section_junction_; }

 private:
  void IsEgoDedicatedRight();

  void Reset() {
    deal_per_traffic_light_objects_.clear();
    perception_traffic_lights_.clear();
    distance_to_junction_prev_           = distance_to_junction_;
    distance_to_ego_light_junction_prev_ = dis_ego_light_junction_;
    traffic_lights_                      = TrafficLights();
    distance_to_junction_                = std::numeric_limits<double>::infinity();
    dis_ego_light_junction_              = std::numeric_limits<double>::infinity();
    junction_orin_vertical_              = std::nullopt;
    junction_orin_connect_               = std::nullopt;
    section_orin_vertical_               = std::nullopt;

    is_section_junction_ = false;

    perception_traffic_light_info_.reset();
    routing_map_ptr_.reset();
    sd_navigation_city_.reset();
    ldmap_junction_ids_.clear();
    maybe_junctions_.clear();
    if (junction_ids_vec_.size() > max_vec_size) {
      junction_ids_vec_.pop_front();
    }
    light_uturn_.Clear();
    light_left_.Clear();
    light_right_.Clear();
    light_straight_.Clear();
    junction_light_ids_.clear();
    section_light_ids_.clear();
    for (auto &[turn_type, seq_deque] : history_ids_) {
      if (seq_deque.size() > 30) {
        seq_deque.erase(seq_deque.begin(), seq_deque.begin() + (seq_deque.size() - 30));
      }
    }
  }

  void MatchLightAndJunction(const RoutingMapPtr &routing_map);

  void FindBestLight(const RoutingMapPtr &routing_map);

  void GetTrafficLightsLd(const std::optional<byd::common::math::Polygon2d> &junction_polygon,
                          const std::vector<TrfObjectInfo>                  &traffic_lights_vec);
  // TrafficLights GetTrafficLightsInJunction(const std::vector<TrfObjectInfo> &traffic_lights_vec);
  // TrafficLights GetTrafficLightsInSection(const std::vector<TrfObjectInfo> &traffic_lights_vec);

  void SetAllLightStatus();

  static void LightStatusConverter(const TrafficLight &light, byd::msg::orin::routing_map::TrafficLightStatus &rhs);

  static void StayPreviousState(const TrafficLight &previous_light, TrafficLight &traffic_light);

  void FillPerceptionTrafficLights();

  void FilterLightsByPosition();

  void SetLightsTurn(const std::vector<TrafficLightShapeType> &shapes, LightTurn turn);

  void SetTrafficLights(LightTurn turn, const std::vector<TrafficLightShapeType> &shapes, const TrafficLight &previous_light,
                        TrafficLight &traffic_light);

  void SetLightT(const TrfObjectInfo &obj, TrafficLight &traffic_light_t);

  void SetTrafficLight(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output);

  double LaneNearDisToJunction(const LaneInfo &ld_lane);

  void SetE2EInfo(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &lanes_traffic_all);

  static bool IsPrevLaneHasStopLine(const RoutingMapPtr &routing_map_output, TrafficLightStatus &traffic_light_status);

  void ProcessNotRoutingLanes(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &traffic_light_lanes);

  std::optional<LaneInfo> SetLaneTrafficInfo(const LaneInfo &lane_traffic, std::vector<LaneInfo> &lanes_output,
                                             const std::vector<StopLineInfo> &stop_lines);

  void GenerateStopLinePoints(const RoutingMapPtr &routing_map_output);

  void SetSrTrafficLight(const RoutingMapPtr &routing_map_output);

  static void TransSrTrafficLight(const TrfObjectInfo &ori_light, cem::message::env_model::TrafficLightMap &out_light, uint64_t id,
                                  uint32_t shape);

  void SetJunctionPoints(const RoutingMapPtr &routing_map, const std::vector<LaneInfo> &lanes_traffic_all);

  std::vector<std::vector<TrfObjectInfo>::iterator> SatisfyTrafficLight(const std::vector<TrafficLightShapeType> &shapes,
                                                                        const std::vector<TrafficLightColorType> &colors, LightTurn turn,
                                                                        const TrafficLight &previous_light);

  void SetTrafficLightColor();

  void BindingTrafficLightToBev(const BevMapInfoPtr &bev_map);

  std::optional<Eigen::Isometry3d> GetTransformMatrix(double timestamp);

  bool TransformPostionOfObjs(double timestamp);

  static void ValidateComparePrevLight(const TrafficLight &light_previous, std::vector<std::vector<TrfObjectInfo>::iterator> &valid_objs);

  static bool TrafficLightPositionTrack(std::vector<std::vector<TrfObjectInfo>::iterator> &objs, const TrafficLight &previous_light);

  static bool IsValid(const TrafficLight &light);

  static TrfObjectInfo FindOptObj(const TrafficLight &previous_light, std::vector<std::vector<TrfObjectInfo>::iterator> &valid_objs);

  void GetJunctionIdsPool(const RoutingMapPtr &routing_map);

  static double DistanceToJunction(const byd::common::math::Polygon2d &polygon, const TrafficLight &light);

  void StayPrevTrafficLightSet();

  void CalculateLightIsBlocked();

  bool IsLightBlockFailed(const TrafficLight &light_prev, TrafficLight &light) const;

  void SetTrafficLightInfo();

  void IsRightExit();

  bool LaneCrossHasTrafficLight(const LaneInfo &lane_rhs);

  bool LaneStoplineTrafficLight(uint64_t section_id, const LaneInfo &lane_rhs, double min_x_threshold = -3.0,
                                std::optional<TrfObjectInfo> tra_obj = std::nullopt);

  void CollectAllLightLanes(const RoutingMapPtr &routing_map_output, const BevMapInfoPtr &bev_map);

  static TrafficLights ChooseBestTrafficLights(const TrafficLights &cloud_lights, const TrafficLights &default_lights);

  std::optional<std::tuple<uint64_t, double, bool>> MatchTrafficLightAndJunctionStopline(const RoutingMapPtr &routing_map);

  static bool IsSameColor(const std::vector<TrfObjectInfo> &qual_obj_temp);

  void GetTrafficArrow(const std::vector<TrfObjectInfo> &traffic_lights_vec, const std::vector<TrafficLightShapeType> &shapes,
                       const TrafficLight &traffic_light_prev, TrafficLight &traffic_light_tmp);

  void GetPolygonFromJunction(const RoutingMapPtr &routing_map, uint64_t junction_id,
                              std::vector<uint64_t> *const prev_lanes_ptr = nullptr);

  static std::vector<TrfObjectInfo> FilterTrfObjByPolygon(const TransformParams            &transform_para,
                                                          const std::vector<TrfObjectInfo> &traffic_lights_vec, double width_offset_left,
                                                          double width_offset_right, double upper_offset, double lower_offset = -10);

  void GetPolygonFromSection(const RoutingMapPtr &routing_map, uint64_t section_id, std::vector<uint64_t> *const prev_lanes_ptr);

  void UpdateHistoryObjId();

  void PrevStopWhenLeftWait(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output, uint64_t junction_id,
                            double distance_to_stopline);

  uint64_t GetHisTimes(TurnType turn_type, uint64_t id);

  std::vector<TrfObjectInfo> FilterTrafObjsUturnPoly(const RoutingMapPtr &routing_map, const TransformParams &transform_para,
                                                     const std::vector<TrfObjectInfo> &traffic_lights_vec,
                                                     const std::vector<uint64_t>      &prev_lane_ids);

  bool IsUturnSection(const RoutingMapPtr &routing_map, const std::vector<uint64_t> &section_lanes_id, const std::set<uint64_t> &light_ids);

  static bool IsSameLight(const TrfObjectInfo &lhs, const TrfObjectInfo &rhs);

  bool IsSectionEnteringRoundabout(uint64_t section_id);

  static bool IsLinkType(uint32_t link_type, byd::msg::orin::routing_map::SectionInfo::LDLinkTypeMask linktype_mask) {
    return (link_type & static_cast<uint32_t>(linktype_mask)) == static_cast<uint32_t>(linktype_mask);
    //     (link_type & static_cast<uint32_t>(linktype_mask)) == static_cast<uint32_t>(linktype_mask);
  }

  void ProcessArrowCircleRightLight();

  cem::message::sensor::TrafficLightColorType IsFlashLight(const TrfObjectInfo &lhs);

  std::vector<TrafficLightShapeType> GetTurnType(TurnType turn_type);

  double timestamp_{0.0};  // sec

  bool is_right_exist{false};

  uint32_t cycle_counter_perception_obj_ = 0;

  TrafficLights traffic_lights_;

  TrafficLights traffic_lights_previous_;

  PerTrfObjectInfoMaps deal_per_traffic_light_objects_;

  std::vector<TrfObjectInfo> perception_traffic_lights_;

  double distance_to_junction_{std::numeric_limits<double>::infinity()};
  double dis_ego_light_junction_{std::numeric_limits<double>::infinity()};
  double distance_to_section_over_first_junction_{std::numeric_limits<double>::infinity()};

  double distance_to_junction_prev_{std::numeric_limits<double>::infinity()};
  double distance_to_ego_light_junction_prev_{std::numeric_limits<double>::infinity()};

  TrafficLightsE2EInfoPtr traffic_light_e2e_{nullptr};

  std::string info_traffic_lights_;

  std::vector<Point2D> previous_junction_points_;

  PercepTrfInfoPtr perception_traffic_light_info_{nullptr};

  std::deque<uint64_t> junction_ids_vec_{};

  RoutingMapPtr routing_map_ptr_{nullptr};

  std::shared_ptr<navigation::SdNavigationCity> sd_navigation_city_{nullptr};

  std::set<uint64_t> prev_find_small_sections_;

  std::map<uint64_t, Polygon2d> ldmap_junction_ids_;

  std::multimap<uint64_t, Polygon2d> maybe_junctions_;  // 1. only_has_stopline

  cem::message::env_model::SectionInfo *first_section_over_juntion_ = nullptr;

  double time_previous_{0.0};

  TrafficLightDiscern traffic_light_descern_;

  ::byd::msg::orin::routing_map::TrafficLightStatus light_uturn_;
  ::byd::msg::orin::routing_map::TrafficLightStatus light_left_;
  ::byd::msg::orin::routing_map::TrafficLightStatus light_right_;
  ::byd::msg::orin::routing_map::TrafficLightStatus light_straight_;

  std::set<uint64_t> ld_map_lanes_light_;
  std::set<uint64_t> bev_map_lanes_light_;

  std::map<uint64_t, std::set<uint64_t>> junction_light_ids_;
  std::map<uint64_t, std::set<uint64_t>> section_light_ids_;

  TrafficLights traffic_lights_ld_;
  TrafficLights traffic_lights_prev_ld_;
  uint64_t      ego_ld_lane_id_ = 0;

  std::optional<TrafficLights> cloud_traffic_light_;

  std::optional<TransformParams> junction_orin_vertical_;
  std::optional<TransformParams> junction_orin_connect_;
  std::optional<TransformParams> section_orin_vertical_;

  bool is_section_junction_{false};

  std::map<TurnType, std::deque<SequencenumObjId>> history_ids_;

  std::map<uint64_t, double> left_distance_stopline_map_;

  const std::vector<TrafficLightShapeType> straight_shapes_ = {TLS_UP_ARROW, TLS_UP_LEFT_ARROW, TLS_UP_RIGHT_ARROW};
  const std::vector<TrafficLightShapeType> left_shapes_     = {TLS_UP_LEFT_ARROW, TLS_LEFT_ARROW, TLS_LEFT_TURN_ARROUND_ARROW};
  const std::vector<TrafficLightShapeType> uturn_shapes_    = {TLS_LEFT_TURN_ARROUND_ARROW, TLS_TURN_ARROUND_ARROW, TLS_UP_LEFT_ARROW};
  const std::vector<TrafficLightShapeType> right_shapes_    = {TLS_UP_RIGHT_ARROW, TLS_RIGHT_ARROW};
};

}  // namespace cem::fusion

#endif  // TRAFFIC_LIGHT_MAPPING_H_
