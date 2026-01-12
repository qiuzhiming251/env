/**
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 * 
 * 
 * @file traffic_light_map_topo.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-25
 */

#pragma once

#include <sys/types.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include <cyber/common/log.h>
#include <message/common/geometry.h>

#include "Eigen/Dense"
#include "lib/common/log_custom.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/internal_message.h"
#include "lib/message/sensor/vision/tsrmobject.h"
#include "modules/msg/drivers_msgs/navi_traffic_info.pb.h"
#include "modules/msg/drivers_msgs/sdmap_inform_service.pb.h"
#include "modules/msg/orin_msgs/e2e_map.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#include "modules/msg/perception_msgs/dynamic_object.pb.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/perception/env/src/lib/base/sensor_data_manager.h"
#include "modules/perception/env/src/lib/common/math/math.h"
#include "modules/perception/env/src/lib/common/utility.h"
#include "modules/perception/env/src/lib/localmap_construction/traffic_light_common.h"
#include "modules/perception/env/src/lib/sd_navigation/SDMapElementExtract.h"
#include "modules/perception/env/src/lib/sd_navigation/SdNavigationCity.h"

namespace cem::fusion {

#if defined(TRAFFIC_NEW) && (TRAFFIC_NEW)
#define NEW_LOG LOG_LP
#else
#define NEW_LOG NULL_LOG
#endif

constexpr double dis_max_longitudinal   = 180.0;
constexpr double dis_max_lateral        = 30.0;
constexpr double dis_min_longitudinal   = -10.0;
constexpr double dis_max_section_length = 350.0;
constexpr double junction_search_dis    = 150.0;
constexpr double default_ego_left       = 100.0;
constexpr double default_ego_right      = -100.0;
constexpr double k_epsilon              = 1e-6;
constexpr double default_jun_lower      = -50.0;
constexpr double default_jun_upper      = 120.0;
constexpr double default_jun_left       = 20;
constexpr double default_jun_right      = -20;
constexpr double width_offset           = 5.0;
constexpr double angle_fov_max          = 13.0;
constexpr double RAD_TO_DEG             = 180.0 / M_PI;
constexpr double lateral_distance_max   = 25.0;

const std::size_t max_deque_size = 30;

using RoutingMapLaneInfo = byd::msg::orin::routing_map::LaneInfo;

struct NaviActionInfoSection {
  NaviActionInfo navi_action;
  double         s_offset{0.0};

  [[nodiscard]] std::string DebugString() const;
};

struct TrfObjectInfoSection {
  std::vector<TrfObjectInfo> traffic_obj_info;

  double                     s_offset{0.0};
  double                     l_offset{0.0};
  uint64_t                   vision_counter = 0;
  std::optional<Point3DD>    junction_pos{std::nullopt};
  std::vector<TrfObjectInfo> traffic_obj_info_filtered{};
  uint64_t                   track_age_max      = 0;
  double                     direction_conf_max = 0.0;

  void SetMaxTrackAge() {
    for (const auto &obj : traffic_obj_info) {
      if (track_age_max < obj.track_age) {
        track_age_max = obj.track_age;
      }
      if (direction_conf_max < obj.direction_conf) {
        direction_conf_max = obj.direction_conf;
      }
    }
  }

  bool operator==(const TrfObjectInfoSection &rhs) const {
    if (rhs.vision_counter == 0 || this->vision_counter == 0) {
      return false;
    }
    if (this->traffic_obj_info.size() != rhs.traffic_obj_info.size() || this->traffic_obj_info.empty() || rhs.traffic_obj_info.empty()) {
      return false;
    }
    for (std::size_t idx = 0; idx < this->traffic_obj_info.size(); idx++) {
      if (this->traffic_obj_info[idx].id != rhs.traffic_obj_info[idx].id) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] std::string DebugString() const;

  void ValidTrafficLight();
};

struct PolygonInfo {
  byd::common::math::Polygon2d polygon;

  uint64_t       bev_id{0};
  Vec2d          inter_start;
  Vec2d          inter_end;
  double         dis_start_in_section{0.0};
  double         dis_end_in_section{0.0};
  SDSectionInfo *section_ptr_start{nullptr};
  SDSectionInfo *section_ptr_end{nullptr};

  [[nodiscard]] std::string DebugString() const;
};

struct SectionAppInfo {
  std::vector<NaviActionInfoSection> navi_actions;
  std::optional<double>              traffic_light_raw;
  std::vector<TrfObjectInfoSection>  traffic_light_perception;
  const SDSectionInfo               *section;
  std::size_t                        idx{0};
  uint64_t                           junction_id{0};

  [[nodiscard]] std::string DebugString() const;
};

struct StopLineAuxInfo {
  uint64_t id{0};
  Vec2d    section_point{};
};

enum class JunctionTypeSource {
  CrossLink = 0b01,
  Event     = 0b10,
};

struct JunctionInfoRem {
  double theta{0.0};
  bool   has_traffic_light{false};
};

struct JunctionInfo {
  uint64_t junction_id;
  uint64_t junction_type{0};  // JunctionTypeSource |=
  double   dis_2_ego{std::numeric_limits<double>::infinity()};
  double   lowest_bound{default_jun_lower};
  double   upper_bound{default_jun_upper};
  double   left_bound{default_jun_left};
  double   right_bound{default_jun_right};
  bool     come_from_polygon{false};

  std::vector<uint64_t> section_ids;
  TransformParams       orin_coordinate;

  std::vector<TrfObjectInfoSection> traffic_light_perception;
  std::optional<Point3DD>           best_pos;
  std::optional<std::size_t>        best_idx;

  [[nodiscard]] std::string DebugString() const;
};

struct LineSegInfo {
  LineSegment2d  line_seg;
  SDSectionInfo *section_ptr{nullptr};
};

struct SectionRange {
  std::size_t idx_start{0};
  std::size_t idx_end{0};
};

class TrafficLightDiscern {
 public:
  TrafficLightDiscern()  = default;
  ~TrafficLightDiscern() = default;

  void Proc(const RoutingMapPtr &routing_map_input, BevMapInfoConstPtr &bev_raw, const BevMapInfoPtr &bev_map,
            const Eigen::Isometry3d &Tbw);

  TrafficLights GetTrafficLights() const { return traffic_lights_; };

  PerTrfObjectInfoMaps GetDealObjMap() const { return deal_per_traffic_light_objects_; }

 private:
  void Reset();

  bool ValidCheck();

  void FilterVisionFusionTrafficLights();

  void ClustorTrafficLights();

  void ProjectTrafficLightsToSD();

  void TransformEgoPosBaseSD();

  void SectionInfoPtrIdx();

  void IsInJunction();

  void EventJunctions();

  void CrossLinkJunctions();

  void GetSDJunctions();

  void JunctionSquare();

  void JunctionLeftRightBasePedestrian();

  void JunctionLeftRightBaseEdge();

  void JunctionBaseStopline();

  void JunctionBasePolygon();

  static std::vector<NaviActionInfo> GetActionEvent(const MapEventPtr &map_event_ptr);

  static void SetTrafficLightColor(std::vector<TrfObjectInfo> &perception_traffic_lights);

  bool SetArrowLight(const std::vector<TrfObjectInfo> &objs, const std::vector<TrafficLightShapeType> &shapes, TrafficLight &res);

  std::optional<std::pair<double, double>> GetLaneGroupWidth(uint64_t section_id, double offset);

  bool CollectSDSections();

  void E2eMapSdRoutingMap();

  void GetLeastTopics();

  bool IsBlockByObjects();

  void ProjectRawTrafficLightToSD();

  void ProjectEventLightToSD();

  void JunctionPolygonInfer(JunctionInfo &rhs);

  void ProjectMapEventToSD();

  void GetEgoPara(const BevMapInfoPtr &bev_map, const Eigen::Isometry3d &Tbw);

  std::pair<int, double> JunctionNextIds(const std::vector<SDSectionInfo> &route_sections, std::size_t start_idx);

  std::pair<int, double> JunctionPreviousIds(const std::vector<SDSectionInfo> &route_sections, std::size_t start_idx);

  bool IsSectionJunction(const std::vector<SDSectionInfo> &route_sections, std::size_t idx);

  void ValidSectionRange();

  std::optional<std::size_t> GetEgoIndexInSDSections();

  void TrafficLightJunctionPos();

  void FindJunctionBestTrafficLight();

  void CalculateEgoDis();

  static bool IsPointInBound(double left_bound, double right_bound, double lowest_bound, double upper_bound, const Point3DD &rhs);

  void FilterJunction();

  void TransformSectionPos();

  void FindNextJunction();

  void GetEgoEdgeWidthBevRoadEdge();

  void GetEgoEdgeWidthLaneGroup();

  void DebugTrafficLightResult();

  void FilterJunctionSectionTrf();

  void SetTrafficLight(const TrfObjectInfo &obj, TrafficLight &res, const std::vector<TrafficLightShapeType> &shape);

  std::optional<TrafficLights> GetTrafficLightInJunction(const TrfObjectInfoSection &trf_obj);

  std::optional<TrafficLights> GetTrafficLightInSection(const TrfObjectInfoSection &trf_obj);

  std::optional<TrafficLights> FindTrafficLightsInJunction(uint64_t id);

  std::optional<TrafficLights> FindTrafficLightsInSection(uint64_t id);

  std::optional<TrafficLights> ChooseTrafficLightInJunction(std::vector<TrfObjectInfoSection> &obj_res, const JunctionInfo &jun_info);

  bool SetJunctionBaseOnPolygon(const PolygonInfo &polygon_info);

  double GetWidthOffset();

  RoutingMapPtr              routing_map_ptr_;
  BevMapInfoConstPtr         bev_map_ptr_;
  PercepTrfInfoPtr           vision_traffic_light_ptr_;
  FusObjInfoPtr              objects_fusion_ptr_;
  std::vector<TrfObjectInfo> perception_traffic_lights_;
  TrafficLights              traffic_lights_;
  ClusterTrafficLights       cluster_traffics_;

  std::map<uint64_t, const SDSectionInfo *>   sd_sections_map_;
  std::map<uint64_t, const SDLaneGroupInfo *> sd_lane_groups_map_;
  std::map<uint64_t, const LaneInfo *>        sd_lanes_map_;

  double          ego_dis_{std::numeric_limits<double>::infinity()};
  NaviPosition    ego_state_;
  TransformParams ego_coordinate_;

  navigation::SDMapElementExtract sd_extract_;

  std::shared_ptr<byd::msg::drivers::sdTrafficLight> sd_traffic_light_raw_ptr_;

  std::vector<NaviActionInfo> navi_actions_;

  std::shared_ptr<byd::msg::orin::e2e_map::E2EMap> e2e_map_ptr_;

  std::shared_ptr<byd::msg::perception::ObjInfoDetects> dynamic_objects_;

  SectionRange section_ranges_{};

  std::map<uint64_t, SectionAppInfo> section_aux_map_infos_;

  std::map<uint64_t, JunctionInfo> sd_junctions_;

  std::vector<SDSectionInfo> mpp_sections_;

  std::optional<JunctionInfo *> next_junction_;

  std::deque<SectionAppInfo> section_app_info_previous_;
  std::deque<JunctionInfo>   junction_previous_;

  PerTrfObjectInfoMaps deal_per_traffic_light_objects_;

  std::vector<StopLineAuxInfo> stopline_info_;

  double ego_width_left_{default_ego_left};
  double ego_width_right_{default_ego_right};

  std::optional<TransformParams> ego_back_;

  std::map<uint64_t, JunctionInfoRem> junction_corrects_;

  std::vector<uint32_t> ego_left_ids_;
  std::vector<uint32_t> ego_right_ids_;
  bool                  ego_is_last_wait_{false};
  bool                  lane_match_bev_sd_{false};

  std::vector<PolygonInfo> polygon_info_;
};
}  // namespace cem::fusion