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
#ifndef TRAFFIC_LIGHT_MAPPING_E2E_H_
#define TRAFFIC_LIGHT_MAPPING_E2E_H_

#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>

#include <cyber/common/log.h>

#include "Eigen/Dense"
#include "lib/common/log_custom.h"
#include "lib/localmap_construction/traffic_light_tracker.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "lib/message/internal_message.h"
#include "lib/message/sensor/vision/tsrmobject.h"
#include "magic_enum/magic_enum.hpp"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/perception_msgs/e2e_traffic_info.pb.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/localmap_construction/cloud_config_traffic_light.h"
#include "modules/perception/env/src/lib/localmap_construction/traffic_light_common.h"
// #include "modules/perception/env/src/lib/sd_navigation/SdNavigationCity.h"

// namespace fmt {
// template <>
// struct formatter<cem::message::env_model::Point> {
//   constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

//   template <typename FormatContext>
//   auto format(const cem::message::env_model::Point &vec, FormatContext &ctx) const {
//     return format_to(ctx.out(), "({:.2f}, {:.2f})", vec.x, vec.y);
//   }
// };

// template <>
// struct formatter<cem::message::env_model::Point2D> {
//   constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

//   template <typename FormatContext>
//   auto format(const cem::message::env_model::Point2D &rhs, FormatContext &ctx) const {
//     return format_to(ctx.out(), "({:.2f}, {:.2f})", rhs.x, rhs.y);
//   }
// };

// template <>
// struct formatter<cem::message::env_model::TrafficLightStatus> {
//   constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

//   template <typename FormatContext>
//   auto format(const cem::message::env_model::TrafficLightStatus &rhs, FormatContext &ctx) const {
//     return format_to(ctx.out(), "(light_status:{}, turn:{} num:{} lanes:{} is_navi:{})", rhs.light_status, rhs.turn_type,
//                      rhs.traffic_light_num, rhs.lane_ids, rhs.is_navi_light);
//   }
// };
// }  // namespace fmt
namespace cem::fusion::e2e {
using byd::common::math::LineSegment2d;
using byd::common::math::Polygon2d;
using byd::common::math::Vec2d;

extern LightStatus LightColorConvert(const TrafficLightColorType &color_type, bool is_blinking);
using PerTrfObjectInfoMaps = std::unordered_map<uint32_t, cem::message::sensor::TrfObjectInfo>;

#if defined(TRAFFIC_LIGHT_LOG) && (TRAFFIC_LIGHT_LOG)
#define TRAFFIC_LOG LOG_LP
#else
#define TRAFFIC_LOG NULL_LOG
#endif

#if defined(TRAFFIC_REC) && (TRAFFIC_REC)
#define TRAFFIC_REC_LOG LOG_LP
#else
#define TRAFFIC_REC_LOG NULL_LOG
#endif

#if defined(E2E_TRAFFIC_LIGHT_LOG) && (E2E_TRAFFIC_LIGHT_LOG)
#define E2E_TRAFFIC_LIGHT LOG_LP
#else
#define E2E_TRAFFIC_LIGHT NULL_LOG
#endif

/// 判断pt在向量p0p1的左侧还是右侧.
/// 返回值：
///  < 0：pt在p0p1左侧；
///  > 0：pt在p0p1右侧；
///  = 0：pt在p0p1所在直线上.
template <typename T, typename M>
double PointInVectorSide(const T &p0, const T &p1, const M &pt) {
  Eigen::Matrix<double, 2, 1> p0_p1(p1.x - p0.x, p1.y - p0.y);
  Eigen::Matrix<double, 2, 1> p0_pt(pt.x - p0.x, pt.y - p0.y);
  double                      cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

// track_index, detect_index, match_score
using MatchScoreTuple = std::tuple<uint32_t, uint32_t, double>;

struct LightAssociationResult {
  std::vector<MatchScoreTuple> assignments;
  std::vector<uint32_t>        unassigned_tracks;
  std::vector<uint32_t>        unsigned_objects;
};

struct TinLight {
  explicit TinLight(TurnType turn_type_t) : turn_type(turn_type_t) {}
  TurnType turn_type{TurnType::OTHER_UNKNOWN};
  bool     is_valid{false};

  std::vector<cem::message::sensor::TrfObjectInfo> match_objs;
  cem::message::sensor::TrafficLightColorType      color{cem::message::sensor::TrafficLightColorType::TLC_NONE_LIGHT};

  std::map<int, std::vector<cem::message::sensor::TrfObjectInfo>> match_obj_records;          // 匹配列表
  std::map<int, std::vector<int>>                                 match_light_records;        // 带时序的目标所有状态记录
  std::map<int, std::vector<int>>                                 match_light_valid_records;  // 带时序的目标有效状态记录
  std::map<int, std::pair<float, int>>                            final_matched_scores;       // 记录目标有效分数以及数量
  std::vector<cem::message::sensor::TrfObjectInfo>                final_matched_objs;
  cem::message::sensor::TrfObjectInfo                             final_matched_obj;
  void addFrameData(int key, const cem::message::sensor::TrfObjectInfo &obj, int value) {
    if (match_light_records[key].size() > 100) {
      match_light_records[key].erase(match_light_records[key].begin());
    }
    if (match_obj_records[key].size() > 100) {
      match_obj_records[key].erase(match_obj_records[key].begin());
    }
    if (match_light_valid_records[key].size() > 100) {
      match_light_valid_records[key].erase(match_light_valid_records[key].begin());
    }

    match_light_records[key].push_back(value);
    if (value != 4) {
      match_obj_records[key].push_back(obj);
    }
    if (value == 0 || value == 1) {
      match_light_valid_records[key].push_back(value);
    }
  }
};
struct TinLightsObj {
  TinLight left{TurnType::LEFT_TURN};
  TinLight u_turn{TurnType::U_TURN};
  TinLight straight{TurnType::NO_TURN};
  TinLight right{TurnType::RIGHT_TURN};
};

constexpr double lane_belong_traffic_light_threshold = 40.0;
constexpr double dis_crosswalk_right_dedicated       = 8.0;

struct RecommendInfo {
  double dis_start{0.0};
  double dis_end{0.0};
  bool   is_virtual{false};

  const ::byd::msg::orin::e2e_map::LaneGroupInfo          *lane_group_ptr{nullptr};
  const ::byd::msg::orin::e2e_map::RecommendLaneInfo      *recommend_lanes_info_ptr{nullptr};
  std::vector<const ::byd::msg::orin::e2e_map::LaneInfo *> recommend_lane_vec;

  std::vector<const ::byd::msg::orin::e2e_map::ReminderSceneInfo *> junctions_vec;
  std::vector<byd::msg::orin::e2e_map::NaviActionInfo>              actions_vec;

  [[nodiscard]] std::string DebugString() const;
};

class TrafficLightMapping {
  class IDPool;

 public:
 private:
 public:
  TrafficLightMapping();

  ~TrafficLightMapping();

  void Process(double target_timestamp = 0.0);

  void Process(const RoutingMapPtr &routing_map_input, const RoutingMapPtr &routing_map_output, const BevMapInfoPtr &bev_map,
               const std::vector<cem::message::env_model::StopLine> &stopline);

  const PerTrfObjectInfoMaps &GetTrfficLights() const { return perception_traffic_light_objects_; }

  const double timestamp() const { return timestamp_; }

  double DistanceToJunction() const { return distance_to_junction_; }

  const std::string &TrafficLightLaneDebug() const { return info_traffic_lights_; }

  const TrafficLightsE2EInfoPtr &GetTrafficLightsE2E() { return traffic_light_e2e_; }

  uint32_t GetPerceptionCyberCounter() const { return cycle_counter_perception_obj_; }

  double GetPreviousJunctionEgoPassed();

  std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> GetActionEvent(const E2EMapRawPtr &e2e_map);

  template <typename T>
  Polygon2d GetTheElementOfMap(const std::vector<T> &ele_pool, uint64_t ele_id) {
    auto it = std::find_if(ele_pool.begin(), ele_pool.end(), [ele_id](const auto &rhs) { return rhs.id == ele_id; });
    if (it == ele_pool.end() || it->points.size() < 4) {
      return Polygon2d{};
    }
    std::vector<Vec2d> points;
    for (auto point : it->points) {
      points.emplace_back(point.x, point.y);
    }
    return Polygon2d{points};
  }

  static std::tuple<uint64_t, bool, bool> LaneStopLineInfo(const std::vector<LaneInfo>     &lanes_output,
                                                           const std::vector<StopLineInfo> &stop_lines, uint64_t id_temp);

 private:
  void FeedLatestFrame();

  void IsEgoDedicatedRight();

  void SetLightStatus_E2E(const std::vector<cem::message::env_model::StopLine> &stopline, const BevMapInfoPtr &bev_map);

  void SetLightStatus_E2E_TIN(const std::vector<cem::message::env_model::StopLine> &stopline, const BevMapInfoPtr &bev_map);

  void SetE2EResultLightStatus(const std::vector<cem::message::env_model::StopLine> &stopline, const BevMapInfoPtr &bev_map);

  static TurnType ConvertTINTurnType(byd::msg::perception::DriveDirection direction);

  bool LaneCrossHasTrafficLight(const LaneInfo &lane_rhs);
  bool LaneStoplineTrafficLight(uint64_t section_id, const LaneInfo &lane_rhs);

  std::optional<cem::message::sensor::TrfObjectInfo> GetTrafficLightBaseTurnType(TurnType turn_type) {
    const auto &light_t = traffic_lights_.GetLightState(turn_type);
    if (light_t.is_valid && light_t.traffic_obj_info) {
      return *light_t.traffic_obj_info;
    }
    if (!deal_per_traffic_light_objects_.empty()) {
      return deal_per_traffic_light_objects_.begin()->second;
    }
    return std::nullopt;
  };

  void Reset() {
    distance_to_junction_prev_ = distance_to_junction_;
    deal_per_traffic_light_objects_.clear();
    perception_traffic_light_objects_.clear();
    perception_traffic_lights_.clear();
    traffic_lights_               = TrafficLights();
    cycle_counter_perception_obj_ = 0;
    distance_to_junction_         = std::numeric_limits<double>::infinity();
    perception_traffic_light_info_.reset();
    tin_e2e_result_.reset();
    routing_map_ptr_.reset();
    // sd_navigation_city_.reset();
    ldmap_junction_ids_.clear();
    maybe_junctions_.clear();
    traffic_light_e2e_   = std::make_shared<cem::message::env_model::TrafficLightsE2EInfo>();
    next_event_          = std::nullopt;
    is_virtual_junction_ = true;
    stopline_ego_e2e_    = std::nullopt;
    recommend_infos_.clear();
    if (junction_ids_vec_.size() > 10) {
      junction_ids_vec_.pop_front();
    }

    stop_line_msg_.Reset();
    lights_info_this_junction_.reset();
  }
  static void StayPreviousState(const TrafficLight &previous_light, TrafficLight &traffic_light);

  void FillPerceptionTrafficLights();

  void FilterLightsByPosition();

  void SetLightsTurn(const std::vector<TrafficLightShapeType> &shapes, LightTurn turn);

  void SetTrafficLights(LightTurn turn, const std::vector<TrafficLightShapeType> &shapes, const TrafficLight &previous_light,
                        TrafficLight &traffic_light);

  double GetJunctionDis() const {
    // 有绑定的路口面时，设置小阈值，否则使用大小路口判断
    if (bind_intersection_zone_dist_.size() > 0) {
      return 50.0;
    } else {
      return is_virtual_junction_ ? 120.0 : 70.0;
    }
  }

  void TimeSync();

  void LocalMapping(double timestamp);

  void RemoveNearTrafficLight();

  void RemoveTrackTrafficLight();

  void Tracking();

  void Association(LightAssociationResult &association_result);

  void UpdateAssignedTracks(const LightAssociationResult &association_result);

  void UpdateUnassignedTracks(const LightAssociationResult &association_result);

  void CreateNewTracker(const LightAssociationResult &association_result);

  void LaneAssociation();

  void DealPerTrafficLight(const BevMapInfoPtr &bev_map, const RoutingMapPtr &routing_map);

  void SetTrafficLight(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output);

  static bool FilterTrafficLightShape(const cem::message::sensor::TrafficLightShapeType &shape);

  void GetRoutingTrafficLight(const RoutingMapPtr &routing_map);

  void ExtractTrafficLight(const cem::message::sensor::TrfObjectInfo &trf_obj, TrafficLights &traffic_lights);

  void TransTrafficLight(const cem::message::sensor::TrfObjectInfo &trf_obj, TrafficLight &traffic_light, bool is_arrow);

  void UpdataTracks(double timestamp);

  void SetTrafficLightInfo();

  void LogTrafficLightObjects(const PerTrfObjectInfoMaps &traffic_light_objects);
  void LogTrafficLightObjects(const std::unordered_map<uint32_t, TrafficLightTrackerPtr> &traffic_light_objects);

  double LaneNearDisToJunction(const LaneInfo &ld_lane);

  void SetE2EInfo(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &lanes_traffic_all);

  static bool IsPrevLaneHasStopLine(const RoutingMapPtr &routing_map_output, TrafficLightStatus &traffic_light_status);

  void ProcessNotRoutingLanes(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &traffic_light_lanes);

  std::optional<LaneInfo> SetLaneTrafficInfo(const LaneInfo &lane_traffic, std::vector<LaneInfo> &lanes_output,
                                             const std::vector<StopLineInfo> &stop_lines);

  void GenerateStopLinePoints(const RoutingMapPtr &routing_map_output);

  void SetSrTrafficLight(std::vector<TrafficLightMap> &traffic_lights);

  static void TransSrTrafficLight(const cem::message::sensor::TrfObjectInfo &ori_light, cem::message::env_model::TrafficLightMap &out_light,
                                  uint64_t id, uint32_t shape);

  void SetJunctionPoints(const RoutingMapPtr &routing_map, const std::vector<LaneInfo> &lanes_traffic_all);

  std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> SatisfyTrafficLight(
      const std::vector<TrafficLightShapeType> &shapes, const std::vector<TrafficLightColorType> &colors, LightTurn turn);

  void SetTrafficLightColor();

  void BindingTrafficLightToBev(const BevMapInfoPtr &bev_map) const;

  std::optional<Eigen::Isometry3d> GetTransformMatrix(double timestamp);

  std::optional<Eigen::Isometry3d> GetTransformMatrix(double timestamp_prev, double timestamp_now);

  bool TransformPostionOfObjs(double timestamp);

  void TransformIntersection(double timestamp);

  static TrafficLights ChooseBestTrafficLights(const TrafficLights &cloud_lights, const TrafficLights &default_lights);

  static void ValidateComparePrevLight(const TrafficLight                                                      &light_previous,
                                       std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &valid_objs);

  static bool TrafficLightPositionTrack(std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &objs,
                                        const TrafficLight                                                      &previous_light);

  static bool IsValid(const TrafficLight &light);

  static cem::message::sensor::TrfObjectInfo FindOptObj(
      const TrafficLight &previous_light, std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &valid_objs);

  void GetJunctionIdsPool(const RoutingMapPtr &routing_map);

  static double DistanceToJunction(const byd::common::math::Polygon2d &polygon, const TrafficLight &light);

  void StayPrevTrafficLightSet();

  void CalculateLightIsBlocked();

  void TrafficLightNumCorrection();

  bool IsLightBlockFailed(const TrafficLight &light_prev, TrafficLight &light) const;

  static void ProcessFlashing(const TrafficLight &traffic_light, TrafficLight &rhs);

  void IsLightBlockFailed_TIN(const TrafficLight &light_prev, TrafficLight &light);

  void FindFinalTINResult(E2EResultRawPtr tin_e2e_result, PercepTrfInfoPtr lights_info_this_junction, bool is_virtual_stopline = false);

  bool IsTrafficLightValid(const TrfObjectInfo &obj);

  bool IsTrafficLightBindZone(const TrfObjectInfo &obj);

  void GetMatchLights(const PercepTrfInfoPtr &lights_info_this_junction, const std::vector<TrafficLightShapeType> &light_shape,
                      TinLight &tin_light, bool is_virtual_stopline = false);

  void FilterByChronology(TinLight &tin_light, std::string &binding_info);

  bool BindingTrafficlightToTIN(TinLight &tin_light);

  void LightColorToTin(TinLight &tin_light);

  void ClearInvalidObj(TinLight &cur_tin_light);

  TinLight &GetTinLight(TurnType direction);

  void GetBestLight(const TinLight &tin_light, std::string &info_tmp);

  void GetFinalObj(const TinLightsObj &raw_tin_obj);

  void LightColorFilter(const std::deque<TrafficLights> &traffic_lights_raw, TrafficLight &curr_lights);

  void GetActionBaseRecommendLanes(const E2EMapRawPtr &e2e_map);

  std::vector<RecommendInfo> GetActionSpecial(double event_dis, double offset);

  void SetActionBlur(::byd::msg::orin::e2e_map::NaviActionInfo &next_event);

  // lml edit 绑定路口面和红绿灯
  void BindLightsByGroundMarkings(const std::vector<cem::message::env_model::StopLine> &stopline, const BevMapInfoPtr &bev_map);

  void BindGroundMarkingsAndStopLine(const BevMapInfoPtr &bev_map);

  void FilterLightsNextJunction();

  void FilterLightsNextJunction(std::vector<cem::message::sensor::TrfObjectInfo> &lights, std::string str);

  void GetIntersectionZone(const BevMapInfoPtr &bev_map);

  int JudgeJunctionSize(const BevMapInfoPtr &bev_map);
  // 路口面结构体
  struct IntersectionZoneMsg {
    int                             id_         = 0;
    double                          timestamp_  = 0.0;
    double                          max_length_ = 0.0;
    double                          max_area_   = 0.0;
    std::unordered_map<int, double> bind_lights_map_;
    std::unordered_map<int, double> lights_dis_map_;
    std::unordered_map<int, double> keep_bind_lights_map_;
    byd::common::math::Polygon2d    polygon_;
    bool                            is_bind_stop_line_;
    // 丢失路口面时，转到当前时间戳下
    double                       timestamp_trans_ = 0.0;
    byd::common::math::Polygon2d polygon_trans_;
  };
  // 自车前方路口面的id以及具体的信息
  std::unordered_map<int, IntersectionZoneMsg> intersection_zone_map_;
  // 停止线绑定的路口面以及它们的距离
  std::unordered_map<int, double> bind_intersection_zone_dist_;
  std::unordered_map<int, double> unbind_intersection_zone_dist_;

  struct StopLineEnv {
    std::vector<Point2D> line_t_;
    std::vector<Point2D> line_ego_;
    double               line_max_x_ = std::numeric_limits<double>::lowest();
    // clear
    void Reset() {
      line_t_.clear();
      line_ego_.clear();
      line_max_x_ = std::numeric_limits<double>::lowest();
    }
  };
  StopLineEnv stop_line_msg_;
  void        GetStopLineMsg(const std::vector<cem::message::env_model::StopLine> &stopline, StopLineEnv &stop_line_msg);
  // bool        is_stopline_bind_zone_;
  bool is_stopline_zone_valid_;

  int lost_count_;

  double timestamp_;  // sec

  uint32_t cycle_counter_perception_obj_ = 0;

  std::vector<TrafficLightMap> routing_traffic_light_maps_;

  TrafficLights traffic_lights_;

  TrafficLights traffic_lights_previous_;

  TinLightsObj final_tin;

  PerTrfObjectInfoMaps perception_traffic_light_objects_;

  PerTrfObjectInfoMaps deal_per_traffic_light_objects_;

  std::vector<cem::message::sensor::TrfObjectInfo> perception_traffic_lights_;

  std::unordered_map<uint32_t, TrafficLightTrackerPtr> traffic_light_trackers_;

  // 下发到routing_map给srshiyong
  std::vector<uint32_t> perception_traffic_light_ids_;

  IDPool *id_pool_ = nullptr;

  double last_tracker_time_{0.0};  // sec

  double distance_to_junction_{std::numeric_limits<double>::infinity()};

  double distance_to_junction_prev_{std::numeric_limits<double>::infinity()};

  std::unordered_map<uint64_t, bool> previous_associate_lane_ids_;

  TrafficLightsE2EInfoPtr traffic_light_e2e_{nullptr};

  std::string info_traffic_lights_;

  std::vector<Point2D> previous_junction_points_;

  PercepTrfInfoPtr perception_traffic_light_info_{nullptr};
  // 原始红绿灯过滤掉最近路口以外的灯的结果
  PercepTrfInfoPtr lights_info_this_junction_{nullptr};

  std::deque<uint64_t> junction_ids_vec_{};

  RoutingMapPtr routing_map_ptr_{nullptr};

  // std::shared_ptr<navigation::SdNavigationCity> sd_navigation_city_{nullptr};

  std::set<uint64_t> prev_find_small_sections_;

  std::map<uint64_t, Polygon2d> ldmap_junction_ids_;

  std::map<uint64_t, Polygon2d> maybe_junctions_;  // 1. only_has_stopline

  ::byd::msg::orin::e2e_map::NaviActionInfo e2e_navi_action_previous_;

  E2EResultRawPtr tin_e2e_result_{nullptr};

  std::deque<E2EResultRawPtr>  tin_e2e_deque_;
  std::deque<PercepTrfInfoPtr> perception_traffic_light_info_deque_;

  std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> next_event_;

  double prev_action_dis_{std::numeric_limits<double>::infinity()};

  const std::vector<TrafficLightShapeType> straight_shapes_ = {TLS_UP_ARROW, TLS_UP_LEFT_ARROW, TLS_UP_RIGHT_ARROW};
  const std::vector<TrafficLightShapeType> left_shapes_     = {TLS_UP_LEFT_ARROW, TLS_LEFT_ARROW, TLS_LEFT_TURN_ARROUND_ARROW};
  const std::vector<TrafficLightShapeType> uturn_shapes_    = {TLS_LEFT_TURN_ARROUND_ARROW, TLS_TURN_ARROUND_ARROW, TLS_UP_LEFT_ARROW,
                                                               TLS_LEFT_ARROW};
  const std::vector<TrafficLightShapeType> right_shapes_    = {TLS_UP_RIGHT_ARROW, TLS_RIGHT_ARROW};

  bool is_virtual_junction_ = true;

  std::optional<LineSegment2d> stopline_ego_e2e_{std::nullopt};
  std::vector<RecommendInfo>   recommend_infos_;

  google::protobuf::RepeatedPtrField<byd::msg::orin::e2e_map::NaviActionInfo> actions_prev_;
};

}  // namespace cem::fusion::e2e

#endif  // TRAFFIC_LIGHT_MAPPING_H_
