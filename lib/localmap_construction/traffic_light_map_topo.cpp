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
 * @file traffic_light_map_topo.cpp
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-25
 */

#include "modules/perception/env/src/lib/localmap_construction/traffic_light_map_topo.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <log_custom.h>
#include <sys/types.h>
#include <Eigen/Geometry>
#include <cyber/common/log.h>
#include <cyber/time/clock.h>
#include <localmap_construction/cloud_config_traffic_light.h>
#include <localmap_construction/traffic_light_common.h>
#include <math/math.h>
#include <message/common/geometry.h>
#include <message/env_model/routing_map/map_event.h>
#include <message/internal_message.h>
#include <message/sensor/vision/tsrmobject.h>
#include <nlohmann/json.hpp>
#include <sd_navigation/SDMapElementExtract.h>
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "magic_enum/magic_enum.hpp"

#include <perception_and_ld_map_fusion/data_fusion/geometry_match_info.h>
#include "gtest/gtest.h"
#include "Eigen/src/Core/ArrayWrapper.h"
#include "Eigen/src/Core/Matrix.h"
#include "base/sensor_data_manager.h"
#include "fmt/format.h"
#include "lib/common/utility.h"
#include "magic_enum/magic_enum.hpp"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/base/params_manager/params_manager.h"

namespace cem::fusion {

std::vector<TrafficLightShapeType> GetStraightShapes() {
  return {TLS_UP_ARROW, TLS_UP_LEFT_ARROW, TLS_UP_RIGHT_ARROW};
}
std::vector<TrafficLightShapeType> GetLeftShapes() {
  return {TLS_UP_LEFT_ARROW, TLS_LEFT_ARROW, TLS_LEFT_TURN_ARROUND_ARROW};
}
std::vector<TrafficLightShapeType> GetRightShapes() {
  return {TLS_UP_RIGHT_ARROW, TLS_RIGHT_ARROW};
}
std::vector<TrafficLightShapeType> GetUturnShapes() {
  return {TLS_LEFT_TURN_ARROUND_ARROW, TLS_TURN_ARROUND_ARROW, TLS_UP_LEFT_ARROW, TLS_LEFT_ARROW};
}

std::string TrfObjectInfoSection::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "trf_ids:[");
  for (const auto &light : traffic_obj_info) {
    fmt::format_to(buf, "{},", light.id);
  }
  fmt::format_to(buf, "]  filted:[");
  for (const auto &light : traffic_obj_info_filtered) {
    fmt::format_to(buf, "{},", light.id);
  }
  fmt::format_to(buf, "]");
  fmt::format_to(buf, " counter:{}", vision_counter);
  fmt::format_to(buf, " sl:[{:.2f},{:.2f}]", s_offset, l_offset);
  if (junction_pos) {
    fmt::format_to(buf, " jun_pos:[{:.2f},{:.2f}]", junction_pos->x, junction_pos->y);
  }
  fmt::format_to(buf, "  max_track_age:{}  direction_conf:{:.2f}", track_age_max, direction_conf_max);
  fmt::format_to(buf, ";");
  return {buf.data(), buf.size()};
}

void TrfObjectInfoSection::ValidTrafficLight() {
  traffic_obj_info_filtered = traffic_obj_info;
  std::vector<std::size_t> removed_index;
  for (std::size_t idx = 0; idx < traffic_obj_info.size(); idx++) {
    if (traffic_obj_info[idx].attributes.traffic_light_shape == TrafficLightShapeType::TLS_PEDESTRIAN) {
      removed_index.emplace_back(idx);
      continue;
    }
    if (traffic_obj_info[idx].attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK) {
      removed_index.emplace_back(idx);
      continue;
    }
  }
  for (auto it = removed_index.rbegin(); it != removed_index.rend(); it++) {
    traffic_obj_info_filtered.erase(traffic_obj_info_filtered.begin() + *it);
  }
}

std::string NaviActionInfoSection::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "{{main:{} ass:{}  dis:{:.2f}}}", navi_action.main_action, navi_action.assistant_action, navi_action.action_dis);
  return {buf.data(), buf.size()};
}

std::string JunctionInfo::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "JunctionInfo junction_id:{}  dis_ego:{:.2f} polygon:{:d}", junction_id, dis_2_ego, come_from_polygon);
  fmt::format_to(buf, " lon:[{:.2f},{:.2f}]  lat:[{:.2f},{:.2f}]", lowest_bound, upper_bound, left_bound, right_bound);
  if (traffic_light_perception.empty()) {
    fmt::format_to(buf, " No_Light.");
  }
  for (const auto &obj_info : traffic_light_perception) {
    fmt::format_to(buf, "  {} ", obj_info.DebugString());
  }
  fmt::format_to(buf, " ids:{}", section_ids);
  fmt::format_to(buf, " cor:[{:.2f},{:.2f},{:.2f}]", orin_coordinate.tx, orin_coordinate.ty, orin_coordinate.theta);
  return {buf.data(), buf.size()};
}

std::string PolygonInfo::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "PolyInfo bev_id:{}  dis:[{:.2f},{:.2f}]", bev_id, dis_start_in_section, dis_end_in_section);
  if (section_ptr_start != nullptr) {
    fmt::format_to(buf, " section_start:{} point:[{:.2f},{:.2f}]", section_ptr_start->id, inter_start.x(), inter_start.y());
  }
  if (section_ptr_end != nullptr) {
    fmt::format_to(buf, " section_end:{} point:[{:.2f},{:.2f}]", section_ptr_end->id, inter_end.x(), inter_end.y());
  }

  return {buf.data(), buf.size()};
}

std::string SectionAppInfo::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "SectionAppInfo");
  fmt::format_to(buf, " id:{}  idx:{}", junction_id, idx);
  if (navi_actions.empty()) {
    fmt::format_to(buf, " Action_empty.");
  } else {
    fmt::format_to(buf, " Action_size:{} ", navi_actions.size());
  }
  for (const auto &act : navi_actions) {
    fmt::format_to(buf, "{},", act.DebugString());
  }
  if (traffic_light_raw) {
    fmt::format_to(buf, " light_raw:{:.2f}", *traffic_light_raw);
  }
  if (traffic_light_perception.empty()) {
    fmt::format_to(buf, " Light_empty.");
  } else {
    fmt::format_to(buf, " Light_size:{} ", navi_actions.size());
  }
  for (const auto &ojb : traffic_light_perception) {
    fmt::format_to(buf, " {}", ojb.DebugString());
  }

  return {buf.data(), buf.size()};
}

void TrafficLightDiscern::Proc(const RoutingMapPtr &routing_map_input, BevMapInfoConstPtr &bev_raw, const BevMapInfoPtr &bev_map,
                               const Eigen::Isometry3d &Tbw) {
  NEW_LOG << "\n\n";
  NEW_LOG << "------------------------------start-----------";
  Reset();
  routing_map_ptr_ = routing_map_input;
  bev_map_ptr_     = bev_raw;
  GetLeastTopics();
  E2eMapSdRoutingMap();

  if (!ValidCheck()) {
    return;
  }
  if (!CollectSDSections()) {
    return;
  }
  GetEgoPara(bev_map, Tbw);
  TransformEgoPosBaseSD();
  TransformSectionPos();

  FilterVisionFusionTrafficLights();

  SetTrafficLightColor(perception_traffic_lights_);

  uint64_t routing_counter = routing_map_ptr_ ? routing_map_ptr_->header.cycle_counter : 0;
  uint64_t bev_counter     = bev_map_ptr_ ? bev_map_ptr_->header.cycle_counter : 0;
  uint64_t vision_counter  = vision_traffic_light_ptr_ ? vision_traffic_light_ptr_->header.cycle_counter : 0;
  NEW_LOG << fmt::format("CCounter~~  routing_map:{}  bev_map:{} vision_fusion:{}", routing_counter, bev_counter, vision_counter);

  CalculateEgoDis();
  GetEgoEdgeWidthBevRoadEdge();
  GetEgoEdgeWidthLaneGroup();
  ClustorTrafficLights();

  // sd_extract_.Proc(routing_map_ptr_);

  ProjectRawTrafficLightToSD();

  ProjectEventLightToSD();

  GetSDJunctions();

  ValidSectionRange();

  ProjectTrafficLightsToSD();

  SectionInfoPtrIdx();

  JunctionSquare();

  FilterJunction();

  FindNextJunction();

  TrafficLightJunctionPos();

  FilterJunctionSectionTrf();

  FindJunctionBestTrafficLight();

  DebugTrafficLightResult();
}

void TrafficLightDiscern::FilterJunctionSectionTrf() {
  for (auto &[id, jun] : sd_junctions_) {
    std::for_each(jun.traffic_light_perception.begin(), jun.traffic_light_perception.end(), [](auto &rhs) {
      rhs.SetMaxTrackAge();
      rhs.ValidTrafficLight();
    });
    std::sort(jun.traffic_light_perception.begin(), jun.traffic_light_perception.end(),
              [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) {
                if (lhs.traffic_obj_info_filtered.empty() && !rhs.traffic_obj_info_filtered.empty()) {
                  return false;
                };
                if (!lhs.traffic_obj_info_filtered.empty() && rhs.traffic_obj_info_filtered.empty()) {
                  return true;
                };
                if (lhs.traffic_obj_info_filtered.empty() && rhs.traffic_obj_info_filtered.empty()) {
                  return false;
                };
                // return std::fabs(lhs.junction_pos->y) < std::fabs(rhs.junction_pos->y);
                return lhs.junction_pos->x < rhs.junction_pos->x;
              });

    NEW_LOG << jun.DebugString();
  }
  for (auto &[id, aux_info] : section_aux_map_infos_) {
    std::for_each(aux_info.traffic_light_perception.begin(), aux_info.traffic_light_perception.end(), [](auto &rhs) {
      rhs.SetMaxTrackAge();
      rhs.ValidTrafficLight();
    });
    aux_info.junction_id = id;
    NEW_LOG << aux_info.DebugString();
  }
}

void TrafficLightDiscern::GetEgoPara(const BevMapInfoPtr &bev_map, const Eigen::Isometry3d &Tbw) {
  if (!bev_map) {
    NEW_LOG << "bev_map_ptr_nullptr.";
    return;
  }
  uint64_t                        ego_sec_id = bev_map->route.navi_start.section_id;
  const BevLaneInfo              *it_ego_lane_info{nullptr};
  const std::vector<BevLaneInfo> *ego_sec_lanes{nullptr};
  for (const auto &sec : bev_map->route.sections) {
    if (sec.id != ego_sec_id) {
      continue;
    }
    for (const auto &lane : sec.lane_infos) {
      ego_sec_lanes = &sec.lane_infos;
      if (lane.position == 0) {
        it_ego_lane_info = &lane;
        break;
      }
    }
    break;
  }
  if (ego_sec_lanes == nullptr || it_ego_lane_info == nullptr) {
    NEW_LOG << fmt::format("ego_sec_lane:{}  it_ego_lane:{}", fmt::ptr(ego_sec_lanes), fmt::ptr(it_ego_lane_info));
    return;
  }
  if (ego_is_last_wait_) {
    NEW_LOG << "ego_is_in_left_wait.";
    return;
  }

  const auto *left_ego = it_ego_lane_info;
  std::size_t counter  = 0;

  const std::size_t max_counter = 100;
  while ((left_ego != nullptr) && left_ego->left_lane_id != 0 && counter++ < max_counter) {
    ego_left_ids_.emplace_back(left_ego->left_lane_id);
    auto val = std::find_if(ego_sec_lanes->begin(), ego_sec_lanes->end(),
                            [&](const BevLaneInfo &rhs) { return rhs.id == left_ego->left_lane_id; });
    if (val == ego_sec_lanes->end()) {
      break;
    }
    left_ego = &(*val);
  }

  counter = 0;

  const auto *right_ego = &(*it_ego_lane_info);
  while ((right_ego != nullptr) && right_ego->right_lane_id != 0 && counter++ < max_counter) {
    ego_right_ids_.emplace_back(right_ego->right_lane_id);
    auto val = std::find_if(ego_sec_lanes->begin(), ego_sec_lanes->end(),
                            [&](const BevLaneInfo &rhs) { return rhs.id == right_ego->right_lane_id; });
    if (val == ego_sec_lanes->end()) {
      break;
    }
    right_ego = &(*val);
  }
  std::reverse(ego_left_ids_.begin(), ego_left_ids_.end());
  NEW_LOG << fmt::format("bev_ego_lane_id:{}  left_id:{}  right_id:{}", it_ego_lane_info->id, ego_left_ids_, ego_right_ids_);

  std::vector<cem::message::common::Point2DF> ego_lane_points;
  if (it_ego_lane_info->line_points.empty()) {
    NEW_LOG << "ego_lane_point_empty.";
    return;
  }
  Eigen::Vector3d                line_point_st;
  cem::message::common::Point2DF point_2df;
  for (const auto &point : it_ego_lane_info->line_points) {
    line_point_st.x() = point.x;
    line_point_st.y() = point.y;
    line_point_st     = Tbw * line_point_st;
    point_2df.x       = static_cast<float>(line_point_st.x());
    point_2df.y       = static_cast<float>(line_point_st.y());
    ego_lane_points.push_back(point_2df);
  }
  auto it_p =
      std::find_if(ego_lane_points.begin(), ego_lane_points.end(), [](const cem::message::common::Point2DF &rhs) { return rhs.x > -1.0; });
  if (it_p == ego_lane_points.end() || ego_lane_points.empty()) {
    NEW_LOG << "no_find_bigger_1. size:" << ego_sec_lanes->size();
    return;
  }
  constexpr double default_theta_dis = 100.0;

  double navi_dis      = navi_actions_.empty() ? std::numeric_limits<double>::infinity() : navi_actions_.front().action_dis;
  double min_theta_dis = std::min(navi_dis, default_theta_dis);

  std::size_t idx_start = std::distance(ego_lane_points.begin(), it_p);
  std::size_t idx_end   = ego_lane_points.size() - 1;
  if (ego_lane_points[idx_end].x > min_theta_dis) {
    for (int idx = static_cast<int>(idx_end);
         idx >= static_cast<int>(idx_start) && idx >= 0 && idx < static_cast<int>(ego_lane_points.size()); idx--) {
      if (ego_lane_points[idx].x < min_theta_dis) {
        idx_end = idx;
        break;
      }
    }
  }
  if (idx_start == idx_end || idx_start > idx_end || idx_start >= ego_lane_points.size() || idx_end >= ego_lane_points.size()) {
    NEW_LOG << "idx_equal. idx_start:" << idx_start << "  idx_end:" << idx_end << "  ego_size:" << ego_lane_points.size();
    return;
  }
  for (int i = static_cast<int>(idx_start); i >= 0; i--) {
    if (ego_lane_points[idx_end].x - ego_lane_points[i].x > min_theta_dis) {
      idx_start = i;
      break;
    }
  }
  double dis_temp = ego_lane_points[idx_end].x - ego_lane_points[idx_start].x;
  NEW_LOG << fmt::format("x:[{:.2f},{:.2f}] idx:[{},{}]  min_theta_dis:{:.2f} navi_dis:{:.2f} dis_temp:{:.2f}",
                         ego_lane_points[idx_start].x, ego_lane_points[idx_end].x, idx_start, idx_end, min_theta_dis, navi_dis, dis_temp);
  if (dis_temp < min_theta_dis) {
    return;
  }
  std::vector<Vec2d> points;
  for (std::size_t idx = idx_start; idx <= idx_end && idx < ego_lane_points.size(); idx++) {
    points.emplace_back(ego_lane_points[idx].x, ego_lane_points[idx].y);
  }
  if (points.size() <= 2) {
    NEW_LOG << "point_is_less_2.";
    return;
  }
  byd::common::math::LineSegment2d seg_end{points.front(), points.back()};

  /*
  constexpr double max_lat = 0.2;

  double max_l = 0;
  for (const auto &point : points) {
    double proj_l = seg_end.ProductOntoUnit(point);
    if (std::fabs(max_l) < std::fabs(proj_l)) {
      max_l = proj_l;
    }
  }
  if (std::fabs(max_l) > max_lat) {
    NEW_LOG << fmt::format("ego_max_l:{:.2f}", max_l);
    return;
  }
  */
  TransformParams res{points.back().x(), points.back().y(), seg_end.heading(), 1.0, {}};
  res.ego_line.Init(points);

  NEW_LOG << fmt::format("ego_para  front_x:{:.2f}  back_x:{:.2f}", points.front().x(), points.back().x());
  ego_back_ = res;
}

std::vector<NaviActionInfo> TrafficLightDiscern::GetActionEvent(const MapEventPtr &map_event_ptr) {
  auto ActionIsValid = [](const NaviActionInfo &event) {
    if (event.main_action == NaviMainAction::NMA_NONE) {
      return false;
    }
    if (event.assistant_action == NaviAssistantAction::NAA_VIA1 || event.assistant_action == NaviAssistantAction::NAA_VIA2 ||
        event.assistant_action == NaviAssistantAction::NAA_VIA3 || event.assistant_action == NaviAssistantAction::NAA_VIA4) {
      return false;
    }

    return true;
  };

  static std::vector<NaviActionInfo> actions_prev;
  std::vector<NaviActionInfo>        actions_all;

  std::optional<NaviActionInfo> next_event = std::nullopt;

  if (map_event_ptr == nullptr || map_event_ptr->navi_action.empty()) {
    actions_prev.clear();
  } else {
    auto actions_current = map_event_ptr->navi_action;
    auto action_0        = *actions_current.begin();
    actions_prev.erase(
        std::remove_if(actions_prev.begin(), actions_prev.end(), [&](const NaviActionInfo &rhs) { return !ActionIsValid(rhs); }),
        actions_prev.end());
    actions_current.erase(
        std::remove_if(actions_current.begin(), actions_current.end(), [&](const NaviActionInfo &rhs) { return !ActionIsValid(rhs); }),
        actions_current.end());
    if (!actions_current.empty()) {
      next_event = actions_current[0];
    }

    double dis_offset{0.0};
    int    idx_target = -1;
    for (int idx = 0; idx < static_cast<int>(actions_prev.size()); idx++) {
      if (actions_prev[idx].main_action != action_0.main_action || actions_prev[idx].assistant_action != action_0.assistant_action) {
        continue;
      }
      double dis_off_t = action_0.action_dis - actions_prev[idx].action_dis;
      NEW_LOG << fmt::format("dis_off_t:{:.2f}  ", dis_off_t);
      if (dis_off_t > -40.0 && dis_off_t < 10) {
        dis_offset = dis_off_t;
        idx_target = idx;
        break;
      }
    }

    NEW_LOG << fmt::format("dis_offset:{:.2f}  idx_target:{}", dis_offset, idx_target);
    for (int idx = std::max(0, idx_target - 1); idx < static_cast<int>(actions_prev.size()) && idx < idx_target; idx++) {
      double dis_set = actions_prev[idx].action_dis + dis_offset;
      if (dis_set < -40.0) {
        continue;
      }
      auto &act_t      = actions_all.emplace_back();
      act_t            = actions_prev[idx];
      act_t.action_dis = dis_set;
      NEW_LOG << fmt::format("insert_main_action:{}  ass_action:{}  dis:{:.2f}", act_t.main_action, act_t.assistant_action,
                             act_t.action_dis);
    }
    actions_all.insert(actions_all.end(), actions_current.begin(), actions_current.end());
    for (const NaviActionInfo &act : actions_current) {
      if (act.action_dis > 500) {
        continue;
      }
      NEW_LOG << fmt::format("current_main_action:{}  ass_action:{}  dis:{:.2f}", act.main_action, act.assistant_action, act.action_dis);
    }
    for (const NaviActionInfo &act : actions_all) {
      if (act.action_dis > 500) {
        continue;
      }
      NEW_LOG << fmt::format("all_current_main_action:{}  ass_action:{}  dis:{:.2f}", act.main_action, act.assistant_action,
                             act.action_dis);
    }
  }
  actions_prev = actions_all;
  return actions_prev;
}

void TrafficLightDiscern::DebugTrafficLightResult() {
  uint64_t vision_counter = 0;
  if (vision_traffic_light_ptr_) {
    vision_counter = vision_traffic_light_ptr_->header.cycle_counter;
  }
  NEW_LOG << fmt::format("traffic_counter:{} U {}", vision_counter, traffic_lights_.u_turn);
  NEW_LOG << fmt::format("traffic_counter:{} L {}", vision_counter, traffic_lights_.left);
  NEW_LOG << fmt::format("traffic_counter:{} S {}", vision_counter, traffic_lights_.straight);
  NEW_LOG << fmt::format("traffic_counter:{} R {}\n\n", vision_counter, traffic_lights_.right);
}

void TrafficLightDiscern::TransformSectionPos() {
  if (!routing_map_ptr_) {
    return;
  }
  mpp_sections_ = routing_map_ptr_->sd_route.mpp_sections;
  Point3DD point_t{};
  for (auto &sec : mpp_sections_) {
    auto raw_points = sec.points;
    sec.points      = std::make_shared<std::vector<Eigen::Vector2f>>();
    *sec.points     = *raw_points;
    for (auto &point : *sec.points) {
      point.x() -= static_cast<float>(ego_coordinate_.tx);
      point.y() -= static_cast<float>(ego_coordinate_.ty);
    }
  }
}

void TrafficLightDiscern::SetTrafficLight(const TrfObjectInfo &obj, TrafficLight &res, const std::vector<TrafficLightShapeType> &shape) {
  if (res.is_valid) {
    return;
  }
  if (res.turn_type == TurnType::OTHER_UNKNOWN) {
    return;
  }
  bool is_valid = shape.empty();
  if (!is_valid) {
    auto shapes = shape;
    shapes.emplace_back(TLS_CIRCULAR);
    shapes.emplace_back(TLS_OTHER_SHAPE);
    shapes.emplace_back(TLS_UNKNOWN);

    std::vector<int> shape_ints;
    for (const auto &shape : shapes) {
      shape_ints.emplace_back(static_cast<int>(shape));
    }

    is_valid = std::any_of(shapes.begin(), shapes.end(),
                           [&obj](const TrafficLightShapeType &rhs) { return rhs == obj.attributes.traffic_light_shape; });
  }
  if (!is_valid) {
    return;
  }
  NEW_LOG << fmt::format("turn:{}  obj:{}", magic_enum::enum_name(res.turn_type), obj.id);
  res.perception_seq_num     = vision_traffic_light_ptr_ ? vision_traffic_light_ptr_->header.cycle_counter : 0;
  res.is_valid               = true;
  res.color                  = obj.attributes.traffic_light_color;
  res.traffic_light_num      = obj.attributes.traffic_light_num;
  res.traffic_light_flashing = obj.attributes.traffic_light_flashing;
  res.traffic_obj_info       = std::make_shared<TrfObjectInfo>(obj);
}

bool TrafficLightDiscern::SetArrowLight(const std::vector<TrfObjectInfo> &objs, const std::vector<TrafficLightShapeType> &shapes,
                                        TrafficLight &res) {
  std::vector<TrfObjectInfo> shape_objs;
  for (const auto &shape : shapes) {
    for (const auto &obj : objs) {
      if (obj.attributes.traffic_light_shape == shape) {
        NEW_LOG << fmt::format("turn:{} arrow_obj_id:{}  shape:{}", magic_enum::enum_name(res.turn_type), obj.id,
                               magic_enum::enum_name(shape));
        shape_objs.emplace_back(obj);
      }
    }
  };
  if (shape_objs.empty()) {
    return false;
  }
  SetTrafficLight(shape_objs.front(), res, {});

  return true;
}

void TrafficLightDiscern::SetTrafficLightColor(std::vector<TrfObjectInfo> &perception_traffic_lights) {
  for (auto &obj : perception_traffic_lights) {
    if (obj.attributes.traffic_light_flashing) {
      switch (obj.attributes.traffic_light_color) {
        case TrafficLightColorType::TLC_RED:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_RED_FLASHING;
          break;
        case TrafficLightColorType::TLC_GREEN:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_GREEN_FLASHING;
          break;
        case TrafficLightColorType::TLC_YELLOW:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_YELLOW_FLASHING;
          break;
        default:
          break;
      };
    }
  }
}

std::optional<TrafficLights> TrafficLightDiscern::GetTrafficLightInJunction(const TrfObjectInfoSection &trf_obj) {
  TrafficLights res;
  return res;
}

std::optional<TrafficLights> TrafficLightDiscern::GetTrafficLightInSection(const TrfObjectInfoSection &trf_obj) {
  TrafficLights res;
  std::size_t   filtered_size = trf_obj.traffic_obj_info_filtered.size();
  if (filtered_size == 0) {
    return std::nullopt;
  }
  auto objs_temp = trf_obj.traffic_obj_info_filtered;

  std::vector<uint64_t> obj_temp_ids;
  for (const auto &obj : objs_temp) {
    obj_temp_ids.emplace_back(obj.id);
    deal_per_traffic_light_objects_.insert({obj.id, obj});
  }

  std::vector<TrafficLightShapeType> straight_shapes = GetStraightShapes();
  std::vector<TrafficLightShapeType> left_shapes     = GetLeftShapes();
  std::vector<TrafficLightShapeType> uturn_shapes    = GetUturnShapes();
  std::vector<TrafficLightShapeType> right_shapes    = GetRightShapes();

  SetArrowLight(objs_temp, straight_shapes, res.straight);
  SetArrowLight(objs_temp, left_shapes, res.left);
  SetArrowLight(objs_temp, uturn_shapes, res.u_turn);
  SetArrowLight(objs_temp, right_shapes, res.right);
  if (!res.right.is_valid) {
    res.right.color          = TrafficLightColorType::TLC_GREEN;
    res.right.traffic_reason = RoutingMapLaneInfo::TrafficSetReason::LaneInfo_TrafficSetReason_SET_DEFAULT_OBJ;
    res.right.is_valid       = true;
  }
  if (res.left.is_valid && res.straight.is_valid && res.right.is_valid && res.u_turn.is_valid) {
    return res;
  }
  auto first_color    = objs_temp[0].attributes.traffic_light_color;
  bool all_same_color = std::all_of(objs_temp.begin(), objs_temp.end(), [first_color](const TrfObjectInfo &rhs) {
    if (first_color == message::sensor::TLC_RED || first_color == message::sensor::TLC_RED_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_RED ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_RED_FLASHING;
    }
    if (first_color == message::sensor::TLC_YELLOW || first_color == message::sensor::TLC_YELLOW_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_YELLOW ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_YELLOW_FLASHING;
    }
    if (first_color == message::sensor::TLC_GREEN || first_color == message::sensor::TLC_GREEN_FLASHING) {
      return rhs.attributes.traffic_light_color == message::sensor::TLC_GREEN ||
             rhs.attributes.traffic_light_color == message::sensor::TLC_GREEN_FLASHING;
    }
    return false;
  });

  if (objs_temp.size() == 2 && objs_temp[0].attributes.traffic_light_shape == message::sensor::TLS_CIRCULAR &&
      objs_temp[1].attributes.traffic_light_shape == message::sensor::TLS_RIGHT_ARROW) {
    NEW_LOG << fmt::format("circle_right_arrow.");
    SetTrafficLight(objs_temp[0], res.left, left_shapes);
    SetTrafficLight(objs_temp[0], res.u_turn, uturn_shapes);
    SetTrafficLight(objs_temp[0], res.straight, straight_shapes);
    SetTrafficLight(objs_temp[1], res.right, right_shapes);
    return res;
  }
  if (objs_temp.size() == 2 && objs_temp[0].attributes.traffic_light_shape == message::sensor::TLS_CIRCULAR &&
      objs_temp[1].attributes.traffic_light_shape == message::sensor::TLS_LEFT_ARROW) {
    NEW_LOG << fmt::format("circle_left_arrow.");
    SetTrafficLight(objs_temp[1], res.left, left_shapes);
    SetTrafficLight(objs_temp[1], res.u_turn, uturn_shapes);
    SetTrafficLight(objs_temp[0], res.straight, straight_shapes);
    // SetTrafficLight(objs_temp[0], res.right, right_shapes);
    return res;
  }

  uint64_t    circle_counter   = 0;
  std::size_t circle_idx       = 0;
  uint64_t    straight_counter = 0;

  std::vector<TrafficLightShapeType> other_straight_shapes{straight_shapes};
  other_straight_shapes.emplace_back(TrafficLightShapeType::TLS_UNKNOWN);
  other_straight_shapes.emplace_back(TrafficLightShapeType::TLS_OTHER_SHAPE);
  for (std::size_t idx = 0; idx < objs_temp.size(); idx++) {
    const auto &obj = objs_temp[idx];
    if (obj.attributes.traffic_light_shape == TLS_CIRCULAR) {
      circle_counter++;
      circle_idx = idx;
    }
    if (std::find(other_straight_shapes.begin(), other_straight_shapes.end(), obj.attributes.traffic_light_shape) !=
        other_straight_shapes.end()) {
      straight_counter++;
    }
  }

  if (circle_counter == 1 && straight_counter == 0) {
    // http://jira-irc.byd.com:8080/browse/CNOAC2-9744
    NEW_LOG << fmt::format("no_arrow_other_straight_type_light, only_one_circle_light.");
    SetTrafficLight(objs_temp[circle_idx], res.straight, straight_shapes);
  }

  if (objs_temp.size() >= 2 && objs_temp[0].attributes.traffic_light_shape == message::sensor::TLS_CIRCULAR &&
      objs_temp[1].attributes.traffic_light_shape == message::sensor::TLS_CIRCULAR) {
    NEW_LOG << fmt::format("left_2_circle.");
    SetTrafficLight(objs_temp[0], res.left, left_shapes);
    SetTrafficLight(objs_temp[0], res.u_turn, uturn_shapes);
    SetTrafficLight(objs_temp[1], res.straight, straight_shapes);
    return res;
  }
  NEW_LOG << fmt::format("same_color:{:d} ids:{}", all_same_color, obj_temp_ids);
  if (all_same_color) {
    for (const auto &obj : objs_temp) {
      SetTrafficLight(obj, res.straight, straight_shapes);
      SetTrafficLight(obj, res.left, left_shapes);
      SetTrafficLight(obj, res.u_turn, uturn_shapes);
    }
    return res;
  }
  if (objs_temp.size() >= 2) {
    SetTrafficLight(objs_temp[0], res.left, left_shapes);
    SetTrafficLight(objs_temp[0], res.u_turn, uturn_shapes);
    for (std::size_t idx = 1; idx < objs_temp.size(); idx++) {
      SetTrafficLight(objs_temp[idx], res.straight, straight_shapes);
    }
  }

  return res;
}

void TrafficLightDiscern::FilterJunction() {
  if (!routing_map_ptr_ || routing_map_ptr_->sd_route.navi_start.section_id == 0) {
    return;
  }
  double dis_jun = 0.0;
  for (const auto &jun : mpp_sections_) {
    if (sd_junctions_.count(jun.id) > 0) {
      std::vector<Vec2d> points;
      for (const auto &poi : *jun.points) {
        points.emplace_back(poi.x(), poi.y());
      }
      bool        is_point_valid = false;
      const auto &sd_jun         = sd_junctions_[jun.id];
      double      proj_s         = 0.0;
      double      proj_l         = 0.0;
      double      min_dis        = 0.0;
      if (points.size() > 2) {
        MultiLineSegment seg;
        seg.Init(points);
        if (seg.GetProjection(Vec2d{sd_jun.orin_coordinate.tx, sd_jun.orin_coordinate.ty}, &proj_s, &proj_l, &min_dis)) {
          sd_junctions_[jun.id].dis_2_ego = dis_jun + proj_s - ego_dis_;
          is_point_valid                  = true;
        }
      }

      if (!is_point_valid) {
        if (sd_junctions_[jun.id].section_ids.size() >= 2) {
          sd_junctions_[jun.id].dis_2_ego = dis_jun + jun.length - ego_dis_;
        } else {
          sd_junctions_[jun.id].dis_2_ego = dis_jun - ego_dis_;
        }
      }
      NEW_LOG << fmt::format("junction_id:{} is_point_valid:{:d}  dis_2_ego:{:.2f} proj_s:{:.2f}", sd_junctions_[jun.id].junction_id,
                             is_point_valid, sd_junctions_[jun.id].dis_2_ego, proj_s);
    }
    dis_jun += jun.length;
    if (dis_jun - ego_dis_ > 400) {
      break;
    }
  }

  const auto        &route_sections = mpp_sections_;
  std::set<uint64_t> sections_set;
  for (std::size_t idx = section_ranges_.idx_start; idx <= section_ranges_.idx_end && idx < route_sections.size(); idx++) {
    sections_set.insert(route_sections[idx].id);
  }

  auto GetEgoPoints = [&]() {
    if (!bev_map_ptr_) {
      NEW_LOG << "bev_map_ptr_nullptr.";
      return false;
    }
    auto it_ego_lane_info = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
                                         [](const BevLaneInfo &rhs) { return rhs.position == 0; });
    if (it_ego_lane_info == bev_map_ptr_->lane_infos.end()) {
      NEW_LOG << "no_find_ego_bev_lane_id.";
      return false;
    }
    NEW_LOG << "bev_ego_lane_id:" << it_ego_lane_info->id;
    const auto &ego_lane_points = it_ego_lane_info->line_points;
    if (ego_lane_points.empty()) {
      NEW_LOG << "ego_lane_point_empty.";
      return false;
    }
    return ego_lane_points.back().x > 6.0;
  };

  auto it = sd_junctions_.begin();
  while (it != sd_junctions_.end()) {
    bool has_valid_section = it->second.dis_2_ego > -200.0 && it->second.dis_2_ego < 400.0;
    bool ego_pass_junction = it->second.dis_2_ego < -30.0 && GetEgoPoints();

    // if (has_valid_section) {
    //   for (const auto &section_id : it->second.section_ids) {
    //     if (sections_set.find(section_id) != sections_set.end()) {
    //       has_valid_section = true;
    //       break;
    //     }
    //   }
    // }

    NEW_LOG << fmt::format("valid_junction_id:{}  has_valid_section:{:d} ego_pass_junction:{:d}", it->first, has_valid_section,
                           ego_pass_junction);
    if (!has_valid_section || ego_pass_junction) {
      it = sd_junctions_.erase(it);
    } else {
      ++it;
    }
  }
}

void TrafficLightDiscern::FindNextJunction() {
  if (ego_back_ && !ego_back_->ego_line.GetPathPoints().empty()) {
    for (auto &[id, jun] : sd_junctions_) {
      double proj_s  = 0.0;
      double proj_l  = 0.0;
      double min_dis = 0.0;

      if (jun.dis_2_ego <= ego_back_->ego_line.GetPathPoints().back().x() + 20.0 &&
          jun.dis_2_ego >= ego_back_->ego_line.GetPathPoints().front().x() - 5.0 &&
          ego_back_->ego_line.GetProjection({jun.orin_coordinate.tx, jun.orin_coordinate.ty}, &proj_s, &proj_l, &min_dis)) {
        auto spec_point = ego_back_->ego_line.GetPointAtDistance(proj_s);
        NEW_LOG << fmt::format("junction_id:{} point:{:.2f},{:.2f} heading:{:.2f}", jun.junction_id, spec_point.first.x(),
                               spec_point.first.y(), spec_point.second);
        JunctionInfoRem rem_temp;
        rem_temp.theta = spec_point.second - jun.orin_coordinate.theta;
        junction_corrects_.insert({id, rem_temp});
        junction_corrects_[id].theta = spec_point.second - jun.orin_coordinate.theta;
      } else {
        junction_corrects_.erase(id);
      }
    }
  }
  for (const auto &[id, val] : junction_corrects_) {
    NEW_LOG << fmt::format("jun_id:{}  junction_corrects_theta:{:.2f} prev_has_light:{:d}", id, val.theta, val.has_traffic_light);
  }
  for (auto &[id, jun] : sd_junctions_) {
    auto it = junction_corrects_.find(id);
    if (it != junction_corrects_.end()) {
      NEW_LOG << fmt::format("jun:{}  theta:{:.4f}  cor:{:.4f}  val:{:.4f}", jun.junction_id, jun.orin_coordinate.theta, it->second.theta,
                             jun.orin_coordinate.theta + it->second.theta);
      jun.orin_coordinate.theta += it->second.theta;
    }
  }
}

void TrafficLightDiscern::CalculateEgoDis() {
  if (!routing_map_ptr_ || routing_map_ptr_->sd_route.navi_start.section_id == 0) {
    return;
  }
  const auto &route_sections = mpp_sections_;

  double dis = 0.0;
  for (const auto &sec : route_sections) {
    if (sec.id == routing_map_ptr_->sd_route.navi_start.section_id) {
      dis += routing_map_ptr_->sd_route.navi_start.s_offset;
      ego_dis_ = dis;
      return;
    }
    dis += sec.length;
  }
}

void TrafficLightDiscern::TrafficLightJunctionPos() {
  Point3DD              pos_jun{};
  TrfObjectInfoSection  tra_t;
  std::size_t           vision_counter = vision_traffic_light_ptr_ ? vision_traffic_light_ptr_->header.cycle_counter : 0;
  std::vector<uint64_t> light_ids;
  bool                  valid_obj = true;
  constexpr double      dis_val   = 3.0;

  for (auto &[id, jun] : sd_junctions_) {
    for (const auto &traffic_lights : cluster_traffics_.clusters) {
      light_ids.clear();
      valid_obj = true;

      for (const auto &obj : traffic_lights) {
        light_ids.emplace_back(obj.id);
      }
      const auto &point = ClusterTrafficLights::GetClusterPosition(traffic_lights);
      pos_jun           = TransformCoordinate(point, jun.orin_coordinate);
      double angle      = std::atan2(pos_jun.y, pos_jun.x) * RAD_TO_DEG;

      if (pos_jun.x > jun.upper_bound || pos_jun.x < jun.lowest_bound) {
        valid_obj = false;
      }
      if (pos_jun.y > jun.left_bound + dis_val + GetWidthOffset() || pos_jun.y < jun.right_bound - dis_val - GetWidthOffset()) {
        valid_obj = false;
      }
      if (std::fabs(angle) < angle_fov_max + 5.0 && std::fabs(pos_jun.y) < lateral_distance_max) {
        valid_obj = true;
      }
      NEW_LOG << fmt::format(
          "jun_id:{}  counter:{}  obj_ids:{} tan:{:.2f} pos:({:.2f},{:.2f}) valid_obj:{:d} jun_low_upper:[{:.2f},{:.2f}] "
          "left_right:[{:.2f},{:.2f}]  offset:{:.2f}",
          id, vision_counter, light_ids, angle, pos_jun.x, pos_jun.y, valid_obj, jun.lowest_bound, jun.upper_bound, jun.left_bound,
          jun.right_bound, GetWidthOffset());
      if (!valid_obj) {
        continue;
      }

      tra_t.traffic_obj_info = traffic_lights;
      tra_t.vision_counter   = vision_counter;
      tra_t.junction_pos     = pos_jun;
      jun.traffic_light_perception.push_back(tra_t);
    }
  }
}

bool TrafficLightDiscern::IsPointInBound(double left_bound, double right_bound, double lowest_bound, double upper_bound,
                                         const Point3DD &rhs) {
  if (rhs.y > left_bound || rhs.y < right_bound) {
    return false;
  }
  if (rhs.x < lowest_bound || rhs.x > upper_bound) {
    return false;
  }
  return true;
};

std::optional<TrafficLights> TrafficLightDiscern::FindTrafficLightsInJunction(uint64_t id) {
  TrafficLights res;

  auto it = sd_junctions_.find(id);
  if (it == sd_junctions_.end()) {
    return res;
  }
  auto &jun_info = it->second;
  // std::sort(jun_info.traffic_light_perception.begin(), jun_info.traffic_light_perception.end(),
  //           [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) { return lhs.junction_pos->x > rhs.junction_pos->x; });

  // constexpr double delta_dis = 2.0;

  std::set<std::size_t> obj_res_set;
  for (std::size_t idx_ratio = 0; idx_ratio <= 0; idx_ratio++) {
    for (std::size_t idx = 0; idx < jun_info.traffic_light_perception.size(); idx++) {
      const auto &cluster_obj = jun_info.traffic_light_perception[idx];
      if (cluster_obj.traffic_obj_info_filtered.empty() || obj_res_set.count(idx) > 0) {
        continue;
      }
      double dis_t          = GetWidthOffset();
      double left_bound_t   = jun_info.left_bound + dis_t;
      double right_bound_t  = jun_info.right_bound - dis_t;
      double lowest_bound_t = jun_info.lowest_bound;
      double upper_bound_t  = jun_info.upper_bound;
      bool   is_cluster_in  = cluster_obj.junction_pos
                                  ? IsPointInBound(left_bound_t, right_bound_t, lowest_bound_t, upper_bound_t, *cluster_obj.junction_pos)
                                  : false;
      bool   is_angle_valid = false;
      double angle_all      = 100.0;
      if (cluster_obj.junction_pos) {
        angle_all      = std::atan2(cluster_obj.junction_pos->y, cluster_obj.junction_pos->x) * RAD_TO_DEG;
        is_angle_valid = std::fabs(angle_all) < angle_fov_max && std::fabs(cluster_obj.junction_pos->y) < lateral_distance_max;
      }
      if (!is_cluster_in && !is_angle_valid) {
        bool is_obj_in_jun = false;
        for (const auto &rhs : cluster_obj.traffic_obj_info_filtered) {
          cem::message::common::Point3DD pos_rev = TransformCoordinate(rhs.position, jun_info.orin_coordinate);
          is_obj_in_jun                          = IsPointInBound(left_bound_t, right_bound_t, lowest_bound_t, upper_bound_t, pos_rev);
          double angle_tmp                       = std::atan2(pos_rev.y, pos_rev.x) * RAD_TO_DEG;
          NEW_LOG << fmt::format("jun_id:{}  dis_t:{:.2f} obj_id:{}  pos:[{:.2f},{:.2f}]  is_in:{:d} angle_valid:{:.2f}",
                                 jun_info.junction_id, dis_t, rhs.id, pos_rev.x, pos_rev.y, is_obj_in_jun, angle_tmp);
          if (is_obj_in_jun || (std::fabs(angle_tmp) < angle_fov_max && std::fabs(pos_rev.y) < lateral_distance_max)) {
            is_cluster_in = true;
            break;
          }
        }
        if (!is_obj_in_jun) {
          continue;
        }
      }
      if (is_cluster_in || is_angle_valid) {
        NEW_LOG << fmt::format("FindJunction:{} dis_t:{} is_cluster_in:{:d} angle_valid:{:d} angle_all:{:.2f}  {}", jun_info.junction_id,
                               dis_t, is_cluster_in, is_angle_valid, angle_all, cluster_obj.DebugString());
        obj_res_set.insert(idx);
      }
    }

    if (obj_res_set.empty()) {
      continue;
    }
    std::vector<TrfObjectInfoSection> obj_res;
    for (auto idx : obj_res_set) {
      obj_res.emplace_back(jun_info.traffic_light_perception[idx]);
    }

    return ChooseTrafficLightInJunction(obj_res, jun_info);
  }

  return res;
}

std::optional<TrafficLights> TrafficLightDiscern::ChooseTrafficLightInJunction(std::vector<TrfObjectInfoSection> &obj_res,
                                                                               const JunctionInfo                &jun_info) {
  if (obj_res.empty()) {
    NEW_LOG << "Error: Empty traffic light input.";
    return std::nullopt;
  }
  std::sort(obj_res.begin(), obj_res.end(), [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) {
    return lhs.junction_pos && rhs.junction_pos && lhs.junction_pos->x < rhs.junction_pos->x;
  });
  constexpr double min_off = 90.0;
  double           start_x = obj_res.front().junction_pos->x;
  obj_res.erase(std::remove_if(obj_res.begin(), obj_res.end(),
                               [start_x](const TrfObjectInfoSection &rhs) { return rhs.junction_pos->x - start_x > min_off; }),
                obj_res.end());
  std::size_t res_size = obj_res.size();
  if (res_size == 0) {
    NEW_LOG << fmt::format("Error: No traffic lights after filtering.");
    return {};
  }
  if (res_size == 1) {
    std::sort(obj_res[0].traffic_obj_info_filtered.begin(), obj_res[0].traffic_obj_info_filtered.end(),
              [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.position.y > rhs.position.y; });
    NEW_LOG << fmt::format("only_1_traffic_light.");
    return GetTrafficLightInSection(obj_res[0]);
  }

  // TODO(lingpeng): How to choose the best traffic_light?
  std::vector<std::size_t>   remove_id;
  std::vector<TrfObjectInfo> arrow_light;

  obj_res.erase(
      std::remove_if(obj_res.begin(), obj_res.end(),
                     [&](const TrfObjectInfoSection &rhs) {
                       if (rhs.traffic_obj_info_filtered.size() != 1) {
                         return false;
                       }
                       if (rhs.traffic_obj_info_filtered[0].attributes.traffic_light_shape == message::sensor::TLS_RIGHT_ARROW ||
                           rhs.traffic_obj_info_filtered[0].attributes.traffic_light_shape == message::sensor::TLS_LEFT_ARROW ||
                           //  rhs.traffic_obj_info_filtered[0].attributes.traffic_light_shape == message::sensor::TLS_UP_ARROW ||
                           rhs.traffic_obj_info_filtered[0].attributes.traffic_light_shape == message::sensor::TLS_TURN_ARROUND_ARROW) {
                         arrow_light.push_back(rhs.traffic_obj_info_filtered[0]);
                         NEW_LOG << fmt::format("AlwaysConsiderObj:{}", rhs.traffic_obj_info_filtered[0].id);
                         return true;
                       }
                       return false;
                     }),
      obj_res.end());

  constexpr double offset_width_thre = 1.0;

  std::vector<std::pair<double, std::size_t>> width_vec(obj_res.size());
  for (std::size_t idx = 0; idx < obj_res.size(); idx++) {
    width_vec[idx].first  = 0.0;
    width_vec[idx].second = idx;
    if (obj_res[idx].traffic_obj_info_filtered.empty()) {
      continue;
    }

    double left_pos  = std::numeric_limits<double>::lowest();
    double right_pos = std::numeric_limits<double>::infinity();
    for (const auto &obj : obj_res[idx].traffic_obj_info_filtered) {
      cem::message::common::Point3DD pos_rev = TransformCoordinate(obj.position, jun_info.orin_coordinate);

      double left_width  = pos_rev.y + obj.width / 2;
      double right_width = pos_rev.y - obj.width / 2;

      left_pos  = left_pos < left_width ? left_width : left_pos;
      right_pos = right_pos > right_width ? right_width : right_pos;
      NEW_LOG << fmt::format("obj_id:{} orin_pos:{:.2f},{:.2f} cor_pos:{:.2f},{:.2f} width:{:.2f}  pos:{:.2f},{:.2f}", obj.id,
                             obj.position.x, obj.position.y, pos_rev.x, pos_rev.y, obj.width, left_pos, right_pos);
    }
    width_vec[idx].first = left_pos - right_pos;
    NEW_LOG << fmt::format("idx:{}  pos:{:.2f},{:.2f}  width:{:.2f}", idx, left_pos, right_pos, width_vec[idx].first);
  }
  std::sort(width_vec.begin(), width_vec.end(),
            [](const std::pair<double, std::size_t> lhs, const std::pair<double, std::size_t> rhs) { return lhs.first > rhs.first; });
  if (!width_vec.empty()) {
    NEW_LOG << fmt::format("0_width:{:.2f},{}  end_width:{:.2f},{}", width_vec[0].first, width_vec[0].second, width_vec.rbegin()->first,
                           width_vec.rbegin()->second);
  }
  if (!width_vec.empty() && width_vec[0].first > offset_width_thre) {
    width_vec.erase(std::remove_if(width_vec.begin(), width_vec.end(),
                                   [](const std::pair<double, std::size_t> rhs) { return rhs.first < offset_width_thre; }),
                    width_vec.end());
    auto obj_tmp = obj_res;
    obj_res.clear();
    std::vector<std::size_t> tra_ids_t;
    for (auto val : width_vec) {
      tra_ids_t.emplace_back(val.second);
      obj_res.emplace_back(obj_tmp[val.second]);
      obj_res.back().SetMaxTrackAge();
      NEW_LOG << "obj_res_fileted:" << obj_res.back().DebugString();
    }
  }
  bool track_angle_sort = false;
  if (obj_res.size() > 1) {
    std::sort(obj_res.begin(), obj_res.end(),
              [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) { return lhs.track_age_max > rhs.track_age_max; });
    if (obj_res[0].track_age_max - obj_res[1].track_age_max > 40 && obj_res[1].track_age_max < 300) {
      NEW_LOG << fmt::format("obj[0] age:{}  obj[1]  age:{}", obj_res[0].track_age_max, obj_res[1].track_age_max);
      track_angle_sort = true;
    } else {
      std::sort(obj_res.begin(), obj_res.end(), [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) {
        return lhs.direction_conf_max > rhs.direction_conf_max;
      });
      if (obj_res[0].direction_conf_max - obj_res[1].direction_conf_max > 0.25) {
        NEW_LOG << fmt::format("obj[0] age:{}  obj[1]  age:{}", obj_res[0].direction_conf_max, obj_res[1].direction_conf_max);
        track_angle_sort = true;
      }
    }
  }
  if (!track_angle_sort) {
    std::sort(obj_res.begin(), obj_res.end(),
              [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) { return lhs.junction_pos->x < rhs.junction_pos->x; });
    if (obj_res.size() >= 2 && std::fabs(obj_res.at(0).junction_pos->x - obj_res.at(1).junction_pos->x) < 3 &&
        std::fabs(obj_res.at(0).junction_pos->y - obj_res.at(1).junction_pos->y) > 3) {
      NEW_LOG << "the_first_2_obj_pos_x_is_close.";
      std::sort(obj_res.begin(), obj_res.begin() + 2, [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) {
        return std::fabs(lhs.junction_pos->y) < std::fabs(rhs.junction_pos->y);
      });
    }
  }

  if (!obj_res.empty()) {
    std::sort(obj_res[0].traffic_obj_info_filtered.begin(), obj_res[0].traffic_obj_info_filtered.end(),
              [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.position.y > rhs.position.y; });
  }

  if (!arrow_light.empty()) {
    if (obj_res.empty()) {
      obj_res.emplace_back();
    }
    for (const auto &obj_t : arrow_light) {
      auto it = std::find_if(
          obj_res[0].traffic_obj_info_filtered.begin(), obj_res[0].traffic_obj_info_filtered.end(),
          [obj_t](const TrfObjectInfo &rhs) { return rhs.attributes.traffic_light_shape == obj_t.attributes.traffic_light_shape; });
      if (it == obj_res[0].traffic_obj_info_filtered.end()) {
        NEW_LOG << "arrow_obj INSERT:" << obj_t.id;
        obj_res[0].traffic_obj_info_filtered.emplace_back(obj_t);
      } else {
        NEW_LOG << "arrow_obj SKIP:" << obj_t.id;
      }
    }
    // obj_res[0].traffic_obj_info_filtered.insert(obj_res[0].traffic_obj_info_filtered.end(), arrow_light.begin(), arrow_light.end());
  }

  if (!navi_actions_.empty()) {
    TurnType turn_type = E2EActionConvertTurnType(navi_actions_.front().main_action);
    if (turn_type == TurnType::LEFT_TURN || turn_type == TurnType::U_TURN) {
      NEW_LOG << fmt::format("final_obj left-uturn {}", obj_res[0].DebugString());
      return GetTrafficLightInSection(obj_res[0]);
    }
    if (turn_type == TurnType::RIGHT_TURN) {
      NEW_LOG << fmt::format("final_obj  right_turn {}", obj_res.rbegin()->DebugString());
      return GetTrafficLightInSection(obj_res[0]);
    }
    if (turn_type == TurnType::NO_TURN) {
      NEW_LOG << fmt::format("final_obj straight {}", obj_res[0].DebugString());
      return GetTrafficLightInSection(obj_res[0]);
    }
  }
  NEW_LOG << fmt::format("final_obj  default {}", obj_res[0].DebugString());
  return GetTrafficLightInSection(obj_res[0]);
}

std::optional<TrafficLights> TrafficLightDiscern::FindTrafficLightsInSection(uint64_t id) {
  TrafficLights    res;
  constexpr double width_offset = 4.0;

  auto it = section_aux_map_infos_.find(id);
  if (it == section_aux_map_infos_.end()) {
    return std::nullopt;
  }
  auto &app_info = it->second;
  std::sort(app_info.traffic_light_perception.begin(), app_info.traffic_light_perception.end(),
            [](const TrfObjectInfoSection &lhs, const TrfObjectInfoSection &rhs) { return lhs.s_offset < rhs.s_offset; });

  for (auto &obj : app_info.traffic_light_perception) {
    if (!obj.traffic_obj_info_filtered.empty() && obj.l_offset < ego_width_left_ + width_offset &&
        obj.l_offset > ego_width_right_ - width_offset) {
      NEW_LOG << fmt::format("FindSection:{}  {}", id, obj.DebugString());
      std::sort(obj.traffic_obj_info_filtered.begin(), obj.traffic_obj_info_filtered.end(),
                [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.position.y > rhs.position.y; });
      return GetTrafficLightInSection(obj);
    };
  }

  return std::nullopt;
}

double TrafficLightDiscern::GetWidthOffset() {
  // if (lane_match_bev_sd_) {
  //   return 3.5;
  // }
  return width_offset;
}

void TrafficLightDiscern::FindJunctionBestTrafficLight() {
  for (std::size_t idx = section_ranges_.idx_start; idx <= section_ranges_.idx_end && idx < mpp_sections_.size(); idx++) {
    const auto &sec = mpp_sections_[idx];
    for (const auto &[id, jun] : sd_junctions_) {
      if (!junction_corrects_[id].has_traffic_light &&
          (jun.traffic_light_perception.empty() || std::all_of(jun.traffic_light_perception.begin(), jun.traffic_light_perception.end(),
                                                               [](const auto &rhs) { return rhs.traffic_obj_info_filtered.empty(); }))) {
        continue;
      }
      auto it_t = std::find(jun.section_ids.begin(), jun.section_ids.end(), sec.id);
      if (it_t == jun.section_ids.end()) {
        continue;
      }
      NEW_LOG << fmt::format("light_junction_id:{}  prev_has_light:{}", id, junction_corrects_[id].has_traffic_light);
      auto val = FindTrafficLightsInJunction(id);
      if (val) {
        junction_corrects_[id].has_traffic_light = true;
        traffic_lights_                          = *val;
        junction_previous_.push_back(jun);
      }
      return;
    }
    auto it_sec = section_aux_map_infos_.find(sec.id);
    if (it_sec == section_aux_map_infos_.end() ||
        std::all_of(it_sec->second.traffic_light_perception.begin(), it_sec->second.traffic_light_perception.end(),
                    [](const auto &rhs) { return rhs.traffic_obj_info_filtered.empty(); })) {
      continue;
    }
    NEW_LOG << "light_section_id:" << it_sec->first;
    auto val = FindTrafficLightsInSection(it_sec->first);
    if (val) {
      traffic_lights_ = *val;
      section_app_info_previous_.push_back(it_sec->second);
      return;
    }
  }
}

void TrafficLightDiscern::SectionInfoPtrIdx() {
  if (!routing_map_ptr_) {
    return;
  }
  const auto &route_sections = mpp_sections_;
  for (std::size_t idx = 0; idx < route_sections.size(); idx++) {
    const auto &sec = route_sections[idx];
    if (auto it = section_aux_map_infos_.find(sec.id); it != section_aux_map_infos_.end()) {
      it->second.section = &sec;
      it->second.idx     = idx;
    }
  }
}

void TrafficLightDiscern::GetSDJunctions() {
  const auto &route_sections = mpp_sections_;
  auto        ego_index_opt  = GetEgoIndexInSDSections();
  if (!ego_index_opt || *ego_index_opt >= route_sections.size()) {
    NEW_LOG << "no_find_ego_index_opt.";
    return;
  }
  std::size_t idx           = *ego_index_opt;
  section_ranges_.idx_start = idx > 0 ? idx - 1 : idx;
  section_ranges_.idx_end   = idx;

  JunctionBasePolygon();
  CrossLinkJunctions();
  EventJunctions();
  constexpr double min_dis = 25.0;

  for (auto &[id, jun] : sd_junctions_) {
    double dis = 0;

    auto it_section = FindTheElement(mpp_sections_, id);
    if (!it_section) {
      NEW_LOG << fmt::format("not_find_section:{}", id);
      continue;
    }

    bool        find_target_section_idx = false;
    std::size_t target_section_idx      = 0;
    if (it_section.value()->length < min_dis) {
      auto idx = std::distance(mpp_sections_.cbegin(), it_section.value());
      for (; idx >= 0; idx--) {
        if (dis + mpp_sections_[idx].length >= min_dis) {
          target_section_idx      = idx;
          find_target_section_idx = true;
          break;
        }
        dis += mpp_sections_[idx].length;
      }
    } else {
      find_target_section_idx = true;
      target_section_idx      = std::distance(mpp_sections_.cbegin(), it_section.value());
    }
    if (!find_target_section_idx) {
      NEW_LOG << fmt::format("not_find_section_idx:{} id:{}", find_target_section_idx, jun.junction_id);
      continue;
    }

    const auto &points_t = *mpp_sections_[target_section_idx].points;

    if (!jun.come_from_polygon) {
      if (jun.section_ids.size() >= 2) {
        std::size_t target_idx = points_t.size() - 1;
        for (std::size_t idx = target_idx; idx >= 1; idx--) {
          dis += (points_t[idx] - points_t[idx - 1]).norm();
          if (dis > min_dis) {
            target_idx = idx;
            break;
          }
        }
        Eigen::Vector2f target_point = CalculatePointAlongLine(points_t[target_idx - 1], points_t[target_idx], dis - min_dis);

        jun.orin_coordinate.tx    = target_point.x();
        jun.orin_coordinate.ty    = target_point.y();
        auto delta_p              = points_t[target_idx] - points_t[target_idx - 1];
        jun.orin_coordinate.theta = std::atan2(delta_p.y(), delta_p.x());
      } else {
        jun.orin_coordinate.tx    = points_t[0].x();
        jun.orin_coordinate.ty    = points_t[0].y();
        auto delta_p              = points_t[1] - points_t[0];
        jun.orin_coordinate.theta = std::atan2(delta_p.y(), delta_p.x());
      }
    }
    NEW_LOG << fmt::format("orin_corrdinate juncion_id:{} xy:{:.2f},{:.2f} heading:{:.2f} come_from_polygon:{:d}", jun.junction_id,
                           jun.orin_coordinate.tx, jun.orin_coordinate.ty, jun.orin_coordinate.theta, jun.come_from_polygon);
  }
}
void TrafficLightDiscern::JunctionLeftRightBasePedestrian() {
  constexpr double later_dis = 0.0;
  for (auto &[id, junc] : sd_junctions_) {
    for (const auto &obj_info : junc.traffic_light_perception) {
      if (!obj_info.junction_pos) {
        continue;
      }
      for (const auto &tra_o : obj_info.traffic_obj_info) {
        if (tra_o.attributes.traffic_light_shape == TrafficLightShapeType::TLS_PEDESTRIAN) {
          NEW_LOG << fmt::format("pedes junction_id:{}  left:{:.2f}  right:{:.2f} obj_id:{} y:{:.2f}", junc.junction_id, junc.left_bound,
                                 junc.right_bound, tra_o.id, tra_o.position.y);
          if (tra_o.position.y < 0) {
            junc.right_bound = std::max(obj_info.junction_pos->y - later_dis, junc.right_bound);
          }
          if (tra_o.position.y > 0) {
            junc.left_bound = std::min(obj_info.junction_pos->y + later_dis, junc.left_bound);
          }
        }
      }
    }
  }
}

void TrafficLightDiscern::JunctionBaseStopline() {
  if (!bev_map_ptr_) {
    return;
  }
  std::vector<LineSegment2d> section_segs;

  Vec2d point_0{};
  Vec2d point_1{};
  bool  is_first_point{true};

  constexpr double min_dis = 0.01;
  for (std::size_t idx_i = section_ranges_.idx_start; idx_i <= section_ranges_.idx_end && idx_i < mpp_sections_.size(); idx_i++) {
    const auto &section_points = *mpp_sections_[idx_i].points;
    if (section_points.empty()) {
      continue;
    }
    for (const auto &section_point : section_points) {
      if (!is_first_point) {
        point_0 = point_1;
        point_1.set_x(section_point.x());
        point_1.set_y(section_point.y());
        if (point_0.DistanceTo(point_1) < min_dis) {
          continue;
        }
        section_segs.emplace_back(point_0, point_1);
      } else {
        point_1.set_x(section_point.x());
        point_1.set_y(section_point.y());
        is_first_point = false;
      }
    }
  }

  Vec2d stoplin_0{};
  Vec2d stoplin_1{};
  Vec2d intersect{};

  constexpr double extend_len = 3.0;

  for (const auto &line_stop : bev_map_ptr_->stop_lines) {
    if (line_stop.line_points.size() < 2) {
      continue;
    }
    stoplin_0.set_x(line_stop.line_points.front().x);
    stoplin_0.set_y(line_stop.line_points.front().y);
    stoplin_1.set_x(line_stop.line_points.back().x);
    stoplin_1.set_y(line_stop.line_points.back().y);
    LineSegment2d stopline_seg{stoplin_0, stoplin_1};
    stoplin_0 -= stopline_seg.unit_direction() * extend_len;
    stoplin_1 += stopline_seg.unit_direction() * extend_len;
    LineSegment2d stopline_seg_extend{stoplin_0, stoplin_1};

    for (const auto &seg2 : section_segs) {
      if (stopline_seg_extend.GetIntersect(seg2, &intersect)) {
        // NEW_LOG << fmt::format(
        //     "stopline_id:{} stopline_seg:{:.2f},{:.2f}=>{:.2f},{:.2f}  seg2:{:.2f},{:.2f}=>{:.2f},{:.2f} sec_x:{:.2f},{:.2f}", line_stop.id,
        //     stopline_seg.start().x(), stopline_seg.start().x(), stopline_seg.end().x(), stopline_seg.end().y(), seg2.start().x(),
        //     seg2.start().y(), seg2.end().x(), seg2.end().x(), intersect.x(), intersect.y());
        stopline_info_.emplace_back(StopLineAuxInfo{line_stop.id, intersect});
        break;
      }
    }
  }

  Point3DD pos_jun{};
  Point3DD pos_inter{};

  constexpr double max_x    = 20.0;
  constexpr double min_x    = -80.0;
  constexpr double max_len  = 100.0;
  constexpr double dis_stop = 5.0;
  for (const auto &stop_line_info : stopline_info_) {
    for (auto &[id, junc] : sd_junctions_) {
      pos_inter.x = stop_line_info.section_point.x();
      pos_inter.y = stop_line_info.section_point.y();
      pos_jun     = TransformCoordinate(pos_inter, junc.orin_coordinate);
      NEW_LOG << fmt::format("junc_stopline_id:{}  jun_id:{}  cor_x:{:.2f}  cor_y:{:.2f} inter:{:.2f},{:.2f}", stop_line_info.id,
                             junc.junction_id, pos_jun.x, pos_jun.y, pos_inter.x, pos_inter.y);
      if (pos_jun.x < max_x && pos_jun.x > min_x) {
        junc.lowest_bound = pos_jun.x - dis_stop;
        junc.upper_bound  = junc.lowest_bound + max_len + dis_stop;
      }
    }
  }
}

void TrafficLightDiscern::GetEgoEdgeWidthBevRoadEdge() {
  if (!bev_map_ptr_) {
    return;
  }
  for (const auto &edge : bev_map_ptr_->edges) {
    if (edge.line_points.empty()) {
      continue;
    }
    const auto &points_t = edge.line_points;
    if (points_t.front().x > 1.0) {
      continue;
    }
    if (points_t.back().x < -1.0) {
      continue;
    }
    auto it_t = std::find_if(points_t.begin(), points_t.end(), [](const cem::message::common::Point2DF &rhs) { return rhs.x > 0.0; });
    if (it_t == points_t.end()) {
      continue;
    }
    if (it_t->y < 0) {
      ego_width_right_ = std::max(ego_width_right_, static_cast<double>(it_t->y));
    } else {
      ego_width_left_ = std::min(ego_width_left_, static_cast<double>(it_t->y));
    }
  }
  if (ego_width_left_ < 100.0 || ego_width_right_ > -100.0) {
    NEW_LOG << fmt::format("bev_ego_left_width:{:.2f}  right_width:{:.2f}", ego_width_left_, ego_width_right_);
  }
}

std::optional<std::pair<double, double>> TrafficLightDiscern::GetLaneGroupWidth(uint64_t section_id, double offset) {
  if (section_id == 0) {
    return std::nullopt;
  }
  auto it_section = sd_sections_map_.find(section_id);
  if (it_section == sd_sections_map_.end()) {
    NEW_LOG << fmt::format("no_section_id:{}", section_id);
    return std::nullopt;
  }
  auto lane_groups = it_section->second->lane_group_idx;
  lane_groups.erase(std::remove_if(lane_groups.begin(), lane_groups.end(),
                                   [&](const SDLaneGroupIndex &rhs) {
                                     auto it_lane_group = sd_lane_groups_map_.find(rhs.id);
                                     if (it_lane_group == sd_lane_groups_map_.end()) {
                                       return true;
                                     }
                                     return std::all_of(it_lane_group->second->lane_info.begin(), it_lane_group->second->lane_info.end(),
                                                        [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_LEFT_WAIT; });
                                   }),
                    lane_groups.end());
  if (lane_groups.empty()) {
    NEW_LOG << fmt::format("section_id:{} lane_group_is_empty.", section_id);
    return std::nullopt;
  }
  std::sort(lane_groups.begin(), lane_groups.end(),
            [](const SDLaneGroupIndex &lhs, const SDLaneGroupIndex &rhs) { return lhs.end_range_offset < rhs.end_range_offset; });

  auto it_group_idx = std::find_if(lane_groups.begin(), lane_groups.end(), [&](const SDLaneGroupIndex &rhs) {
    return offset <= rhs.end_range_offset && offset >= rhs.start_range_offset;
  });

  if (it_group_idx == lane_groups.end()) {
    if (offset >= lane_groups.back().end_range_offset) {
      it_group_idx = --lane_groups.end();
    } else if (offset < lane_groups.back().start_range_offset) {
      it_group_idx = lane_groups.begin();
    } else {
      return std::nullopt;
    }
  }
  auto it_lane_group = sd_lane_groups_map_.find(it_group_idx->id);
  if (it_lane_group == sd_lane_groups_map_.end() || ((it_lane_group->second) == nullptr) || it_lane_group->second->lane_info.empty()) {
    return std::nullopt;
  }
  constexpr double standard_lane_width_max = 3.75;

  auto   lane_size = static_cast<double>(it_lane_group->second->lane_info.size());
  double max_width = standard_lane_width_max * lane_size;
  NEW_LOG << fmt::format("section_id:{} lane_group:{}  lane_size:{} offset:{}  width:{:.2f}", section_id, it_lane_group->second->id,
                         lane_size, offset, max_width);

  return std::make_pair(max_width, -max_width);
}

void TrafficLightDiscern::GetEgoEdgeWidthLaneGroup() {
  if (ego_state_.section_id == 0) {
    return;
  }
  auto it_section = sd_sections_map_.find(ego_state_.section_id);
  if (it_section == sd_sections_map_.end()) {
    NEW_LOG << fmt::format("no_find_ego_state section_id:{}", ego_state_.section_id);
    return;
  }
  auto lane_groups = it_section->second->lane_group_idx;
  if (lane_groups.empty()) {
    NEW_LOG << fmt::format("section_id:{} lane_group_is_empty.", ego_state_.section_id);
    return;
  }
  std::sort(lane_groups.begin(), lane_groups.end(),
            [](const SDLaneGroupIndex &lhs, const SDLaneGroupIndex &rhs) { return lhs.end_range_offset < rhs.end_range_offset; });

  auto it_group_idx = std::find_if(lane_groups.begin(), lane_groups.end(), [&](const SDLaneGroupIndex &rhs) {
    return ego_state_.s_offset <= rhs.end_range_offset && ego_state_.s_offset >= rhs.start_range_offset;
  });

  if (it_group_idx == lane_groups.end()) {
    if (ego_state_.s_offset >= lane_groups.back().end_range_offset) {
      it_group_idx = --lane_groups.end();
    } else if (ego_state_.s_offset < lane_groups.back().start_range_offset) {
      it_group_idx = lane_groups.begin();
    } else {
      return;
    }
  }
  auto it_lane_group = sd_lane_groups_map_.find(it_group_idx->id);
  if (it_lane_group == sd_lane_groups_map_.end() || ((it_lane_group->second) == nullptr) || it_lane_group->second->lane_info.empty()) {
    return;
  }
  bool ego_is_last_wait_ = std::all_of(it_lane_group->second->lane_info.begin(), it_lane_group->second->lane_info.end(),
                                       [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_LEFT_WAIT; });
  if (ego_is_last_wait_) {
    NEW_LOG << fmt::format("ego_lane_group_id:{} has_left_wait.", it_lane_group->second->id);
    return;
  }

  constexpr double standard_lane_width_max = 3.75;

  std::size_t lane_size      = it_lane_group->second->lane_info.size();
  double      max_lane_width = standard_lane_width_max * static_cast<double>(lane_size);
  double      width_tmp      = lane_size <= 2 ? GetWidthOffset() : 2.0;
  if (ego_left_ids_.size() + ego_right_ids_.size() + 1 == lane_size) {
    ego_width_left_    = standard_lane_width_max * static_cast<double>(ego_left_ids_.size()) + width_tmp;
    ego_width_right_   = -standard_lane_width_max * static_cast<double>(ego_right_ids_.size()) - width_tmp;
    lane_match_bev_sd_ = true;
  } else {
    if (ego_width_left_ > max_lane_width) {
      ego_width_left_ = max_lane_width + width_tmp;
    }
    if (ego_width_right_ < -max_lane_width) {
      ego_width_right_ = -max_lane_width - width_tmp;
    }
  }
  NEW_LOG << fmt::format("ego_lane_group_id:{}  lane_size:{}  max_lane_width:{:.2f} bev_ego_left_width:{:.2f}  right_width:{:.2f}",
                         it_lane_group->second->id, lane_size, max_lane_width, ego_width_left_, ego_width_right_);
}

void TrafficLightDiscern::JunctionLeftRightBaseEdge() {
  constexpr double width_offset_t = 0.0;

  double ego_width_left  = ego_width_left_ + width_offset_t;
  double ego_width_right = ego_width_right_ - width_offset_t;
  for (auto &[id, junc] : sd_junctions_) {
    double ori_left  = junc.left_bound;
    double ori_right = junc.right_bound;
    if (auto val = GetLaneGroupWidth(junc.junction_id, std::numeric_limits<double>::infinity()); val) {
      junc.left_bound  = std::min(junc.left_bound, val->first);
      junc.right_bound = std::max(junc.right_bound, val->second);
    }
    junc.left_bound  = std::min(junc.left_bound, ego_width_left);
    junc.right_bound = std::max(junc.right_bound, ego_width_right);
    NEW_LOG << fmt::format("jun_id:{}  left:{:.2f}  right:{:.2f}  orin:[{:.2f},{:.2f}]", junc.junction_id, junc.left_bound,
                           junc.right_bound, ori_left, ori_right);
  }
}

bool TrafficLightDiscern::SetJunctionBaseOnPolygon(const PolygonInfo &polygon_info) {
  MultiLineSegment   line_seg;
  std::vector<Vec2d> points;
  for (const auto &point : *polygon_info.section_ptr_start->points) {
    Vec2d point_tmp{point.x(), point.y()};
    if (!points.empty() && point_tmp.DistanceTo(points.back()) < 0.2) {
      continue;
    }
    NEW_LOG << fmt::format("point_tmp:{:.2f} {:.2f}", point_tmp.x(), point_tmp.y());
    points.emplace_back(point_tmp);
  }
  line_seg.Init(points);
  double proj_s  = 0.0;
  double proj_l  = 0.0;
  double min_dis = 0.0;
  int    min_idx = 0.0;
  bool   proj_suc{false};
  proj_suc = line_seg.GetProjection(polygon_info.inter_start, &proj_s, &proj_l, &min_dis, &min_idx);
  if (!proj_suc) {
    return false;
  }
  double                   offset_prev = 5.0;
  std::pair<Vec2d, double> dis_vec;
  if (proj_s > offset_prev) {
    dis_vec = line_seg.GetPointAtDistance(proj_s - offset_prev);
  } else {
    int section_idx = 0;
    for (std::size_t idx = 0; idx < mpp_sections_.size(); idx++) {
      if (mpp_sections_[idx].id == polygon_info.section_ptr_start->id) {
        section_idx = static_cast<int>(idx);
        break;
      }
    }

    double dis_length  = 0.0;
    double sec_min_len = offset_prev - proj_s;
    points.clear();
    for (int idx = static_cast<int>(section_idx); idx >= 1; idx--) {
      if (dis_length > sec_min_len) {
        break;
      }
      for (auto it_rev = mpp_sections_[idx - 1].points->rbegin(); it_rev != mpp_sections_[idx - 1].points->rend(); it_rev++) {
        Vec2d point_tmp{it_rev->x(), it_rev->y()};
        if (points.empty()) {
          points.emplace_back(point_tmp);
          continue;
        }
        double dis_t = point_tmp.DistanceTo(points.back());
        NEW_LOG << fmt::format("point_tmp:{:.2f} {:.2f}", point_tmp.x(), point_tmp.y());
        points.emplace_back(point_tmp);
        dis_length += dis_t;
      }
    }
    if (dis_length > sec_min_len) {
      dis_vec = line_seg.GetPointAtDistance(sec_min_len);
      if (dis_vec.first.x() > points.back().x()) {
        LineSegment2d seg_temp{points.back(), dis_vec.first};
        dis_vec.second = seg_temp.heading();
      } else {
        LineSegment2d seg_temp{dis_vec.first, points.back()};
        dis_vec.second = seg_temp.heading();
      }
    } else {
      dis_vec = std::make_pair(points.back(), 0.0);
    }
  }
  NEW_LOG << fmt::format(
      "proj_section_id:{} end_id:{} proj_s:{:.2f} proj_l:{:.2f} min_dis:{:.2f} min_indx:{} dis_vec:{:.2f},{:.2f} heading:{:.2f}",
      polygon_info.section_ptr_start->id, polygon_info.section_ptr_end->id, proj_s, proj_l, min_dis, min_idx, dis_vec.first.x(),
      dis_vec.first.y(), dis_vec.second);

  JunctionInfo junc_t;
  junc_t.come_from_polygon = true;
  junc_t.junction_id       = polygon_info.section_ptr_start->id;
  bool find_start          = false;
  for (const auto &sec : mpp_sections_) {
    if (sec.id == polygon_info.section_ptr_start->id) {
      find_start = true;
      junc_t.section_ids.emplace_back(sec.id);
      continue;
    }
    auto it_end = std::find(junc_t.section_ids.begin(), junc_t.section_ids.end(), polygon_info.section_ptr_end->id);
    if (it_end != junc_t.section_ids.end()) {
      break;
    }
    if (sec.id == polygon_info.section_ptr_end->id) {
      junc_t.section_ids.emplace_back(sec.id);
      break;
    }
    if (find_start) {
      junc_t.section_ids.emplace_back(sec.id);
    }
  }
  constexpr double dis_temp = 10.0;
  junc_t.lowest_bound       = -dis_temp;
  junc_t.upper_bound        = junc_t.lowest_bound + default_jun_upper + dis_temp;

  junc_t.orin_coordinate.tx    = dis_vec.first.x();
  junc_t.orin_coordinate.ty    = dis_vec.first.y();
  junc_t.orin_coordinate.theta = dis_vec.second;
  sd_junctions_.insert({polygon_info.section_ptr_start->id, junc_t});
  NEW_LOG << "sd_junction_polygon " << junc_t.DebugString();
  return true;
}

void TrafficLightDiscern::JunctionBasePolygon() {
  if (!bev_map_ptr_) {
    return;
  }

  std::vector<LineSegInfo> section_segs;

  Vec2d point_0{};
  Vec2d point_1{};
  bool  is_first_point{true};

  constexpr double min_dis = 0.01;
  for (auto &mpp_section : mpp_sections_) {
    const auto &section_points = *mpp_section.points;
    if (section_points.empty()) {
      continue;
    }
    for (const auto &section_point : section_points) {
      if (!is_first_point) {
        point_0 = point_1;
        point_1.set_x(section_point.x());
        point_1.set_y(section_point.y());
        if (point_0.DistanceTo(point_1) < min_dis) {
          continue;
        }
        section_segs.emplace_back(LineSegInfo{LineSegment2d{point_0, point_1}, &mpp_section});
      } else {
        point_1.set_x(section_point.x());
        point_1.set_y(section_point.y());
        is_first_point = false;
      }
    }
  }

  double ego_dis_polygon = std::numeric_limits<double>::infinity();
  double max_x           = std::numeric_limits<double>::lowest();

  byd::common::math::Polygon2d polygon;
  std::vector<Vec2d>           points;
  Vec2d                        ego_point{0.0, 0.0};
  for (const auto &junc : bev_map_ptr_->junctions) {
    if (junc.type != static_cast<uint32_t>(message::sensor::BevLaneMarkerType::RM_TYPE_INTERSECTION_ZONE)) {
      continue;
    }
    NEW_LOG << fmt::format("polygon_bev_id:{}", junc.id);
    points.clear();
    for (const auto &point : junc.line_points) {
      points.emplace_back(point.x, point.y);
      max_x = max_x > point.x ? max_x : point.x;
    }
    if (points.size() < 3 || max_x < 0) {
      NEW_LOG << "size:" << points.size() << "  max_x:" << max_x;
      continue;
    }
    polygon.Init(points);
    ego_dis_polygon = polygon.DistanceTo(ego_point);
    std::vector<std::tuple<Vec2d, SDSectionInfo *>> start_vec;
    std::vector<std::tuple<Vec2d, SDSectionInfo *>> end_vec;

    PolygonInfo polygon_info;

    double dis{0.0};
    for (const auto &seg_tmp : section_segs) {
      if (seg_tmp.section_ptr == nullptr) {
        continue;
      }
      const auto &line_seg = seg_tmp.line_seg;
      dis += line_seg.length();
      Vec2d start;
      Vec2d end;
      if (!polygon.GetOverlap(line_seg, &start, &end)) {
        continue;
      };
      start_vec.emplace_back(start, seg_tmp.section_ptr);
      end_vec.emplace_back(end, seg_tmp.section_ptr);
      NEW_LOG << fmt::format("inter_sec start_point:[{:.2f},{:.2f}]  end_point:[{:.2f},{:.2f}]  id:{}", start.x(), start.y(), end.x(),
                             end.y(), seg_tmp.section_ptr->id);
    }
    if (start_vec.empty() || end_vec.empty()) {
      continue;
    }

    polygon_info.bev_id            = junc.id;
    polygon_info.polygon           = polygon;
    polygon_info.inter_start       = std::get<0>(start_vec.front());
    polygon_info.section_ptr_start = std::get<1>(start_vec.front());
    polygon_info.inter_end         = std::get<0>(end_vec.back());
    polygon_info.section_ptr_end   = std::get<1>(end_vec.back());
    if ((polygon_info.section_ptr_start == nullptr) || (polygon_info.section_ptr_end == nullptr)) {
      continue;
    }

    if (!SetJunctionBaseOnPolygon(polygon_info)) {
      continue;
    }

    polygon_info_.push_back(polygon_info);
    NEW_LOG << polygon_info_.back().DebugString();
  }
}

void TrafficLightDiscern::JunctionSquare() {
  // JunctionLeftRightBasePedestrian();
  JunctionLeftRightBaseEdge();
  JunctionBaseStopline();
}

bool TrafficLightDiscern::IsSectionJunction(const std::vector<SDSectionInfo> &route_sections, std::size_t idx) {
  auto lane_group = route_sections[idx].lane_group_idx;
  if (lane_group.empty()) {
    return false;
  }
  std::sort(lane_group.begin(), lane_group.end(),
            [](const SDLaneGroupIndex &lhs, const SDLaneGroupIndex &rhs) { return lhs.end_range_offset < rhs.end_range_offset; });

  auto it_lane_group = sd_lane_groups_map_.find(lane_group.back().id);
  if (it_lane_group == sd_lane_groups_map_.end()) {
    return false;
  }
  bool is_last_wait = std::all_of(it_lane_group->second->lane_info.begin(), it_lane_group->second->lane_info.end(),
                                  [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_LEFT_WAIT; });
  if (is_last_wait) {
    return true;
  }
  uint64_t turn_val = 0;
  for (const auto &lane_info : it_lane_group->second->lane_info) {
    turn_val |= GetTurnTypeVal(lane_info.plan_turn_type);
  }
  NEW_LOG << fmt::format("section_id:{}  turn:{}", route_sections[idx].id, turn_val);

  return (turn_val & ~GetBaseTurnVal(TurnType::NO_TURN)) != 0U;
}

std::pair<int, double> TrafficLightDiscern::JunctionNextIds(const std::vector<SDSectionInfo> &route_sections, std::size_t start_idx) {
  if (route_sections.size() == start_idx + 1) {
    return std::make_pair(-1, 0);
  }
  double dis = 0.0;
  for (std::size_t idx = start_idx + 1; idx < route_sections.size(); idx++) {
    if (IsSectionJunction(route_sections, idx)) {
      return std::make_pair(static_cast<int>(idx), dis);
    }
    dis += route_sections[idx].length;
    if (dis > junction_search_dis) {
      return std::make_pair(-1, 0);
    }
  }
  return std::make_pair(-1, 0);
}

std::pair<int, double> TrafficLightDiscern::JunctionPreviousIds(const std::vector<SDSectionInfo> &route_sections, std::size_t start_idx) {
  if (start_idx == 0) {
    return std::make_pair(-1, 0);
  }
  double dis = 0.0;
  for (int idx = static_cast<int>(start_idx) - 1; idx >= 0 && idx < static_cast<int>(route_sections.size()); idx--) {
    if (IsSectionJunction(route_sections, idx)) {
      return std::make_pair(static_cast<int>(idx), dis);
    }
    dis += route_sections[idx].length;
    if (dis > junction_search_dis) {
      return std::make_pair(-1, 0);
    }
  }

  return std::make_pair(-1, 0);
}

void TrafficLightDiscern::EventJunctions() {
  if (!routing_map_ptr_) {
    return;
  }
  const auto       &route_sections = mpp_sections_;
  std::vector<bool> vec_int;
  vec_int.reserve(route_sections.size());

  auto AddSectionIds = [](JunctionInfo &junction, const std::vector<SDSectionInfo> &sections, size_t start_idx, size_t end_idx) {
    for (size_t i = start_idx; i <= end_idx && i < sections.size(); ++i) {
      junction.section_ids.emplace_back(sections[i].id);
    }
  };

  for (std::size_t idx = 0; idx < route_sections.size(); idx++) {
    auto it_t = section_aux_map_infos_.find(route_sections[idx].id);
    if (it_t == section_aux_map_infos_.end() || it_t->second.navi_actions.empty()) {
      continue;
    }
    auto action_t = it_t->second.navi_actions.front();
    NEW_LOG << fmt::format("raw_event_junction section_id:{} navi_main_action:{} assist:{} dis:{:.2f}", route_sections[idx].id,
                           static_cast<int>(action_t.navi_action.main_action), action_t.navi_action.assistant_action,
                           action_t.navi_action.action_dis);
    if (action_t.navi_action.main_action == NaviMainAction::NMA_MERGE_RIGHT) {
      NEW_LOG << fmt::format("skip_this_action.");
      continue;
    }

    double s_offset = action_t.s_offset;
    double s_remain = route_sections[idx].length - s_offset;

    JunctionInfo junction_t;
    junction_t.junction_type = static_cast<uint64_t>(JunctionTypeSource::Event);

    auto [prev_junction_idx, dis_prev] = JunctionPreviousIds(route_sections, idx);
    auto [next_junction_idx, dis_next] = JunctionNextIds(route_sections, idx);
    if (prev_junction_idx == -1) {
      junction_t.junction_id = route_sections[idx].id;
      if (next_junction_idx == -1) {
        junction_t.section_ids.emplace_back(junction_t.junction_id);
        if (s_offset > s_remain && IsSectionJunction(route_sections, idx) && idx + 1 < route_sections.size()) {
          junction_t.section_ids.emplace_back(route_sections.at(idx + 1).id);
        } else if (s_offset <= s_remain && idx > 0) {
          junction_t.junction_id = route_sections[idx - 1].id;
          junction_t.section_ids.emplace_back(route_sections.at(idx - 1).id);
          junction_t.section_ids.emplace_back(junction_t.junction_id);
        }
      } else {
        AddSectionIds(junction_t, route_sections, idx, next_junction_idx);
      }
    } else {
      if (route_sections[idx].lane_group_idx.empty()) {
        junction_t.junction_id = route_sections[prev_junction_idx].id;
        AddSectionIds(junction_t, route_sections, prev_junction_idx, idx);
      } else if (next_junction_idx == -1) {
        junction_t.junction_id = route_sections[prev_junction_idx].id;
        AddSectionIds(junction_t, route_sections, prev_junction_idx, idx);
      } else if (dis_prev + s_offset < dis_next + s_remain) {
        junction_t.junction_id = route_sections[prev_junction_idx].id;
        AddSectionIds(junction_t, route_sections, prev_junction_idx, idx);
      } else {
        junction_t.junction_id = route_sections[idx].id;
        AddSectionIds(junction_t, route_sections, idx, next_junction_idx);
      }
    }

    bool find_cross_junction{false};
    for (auto &[junction_id, junction_info_t] : sd_junctions_) {
      auto it_j = std::find_if(junction_info_t.section_ids.begin(), junction_info_t.section_ids.end(),
                               [&](uint64_t junction_lane_id) { return junction_lane_id == junction_t.junction_id; });
      if (it_j == junction_info_t.section_ids.end()) {
        continue;
      }
      uint64_t junction_id_t = junction_info_t.junction_id;
      auto     sec_temp      = std::find_if(route_sections.begin(), route_sections.end(),
                                            [junction_id_t](const SDSectionInfo &rhs) { return rhs.id == junction_id_t; });
      if (sec_temp == route_sections.end() || sec_temp->points->empty()) {
        continue;
      }
      double event_dis = std::fabs(sec_temp->points->back().x() - action_t.navi_action.action_dis);
      NEW_LOG << fmt::format("event_find_cross_junction_id:{} jun_id:{}  ids:{} back_x:{:.2f}  event_dis:{:.2f}", junction_t.junction_id,
                             junction_id, junction_info_t.section_ids, sec_temp->points->back().x(), event_dis);
      if (event_dis > 50.0) {
        continue;
      }
      junction_info_t.junction_type |= static_cast<uint64_t>(JunctionTypeSource::Event);
      find_cross_junction = true;
      break;
    }
    if (!find_cross_junction) {
      sd_junctions_.insert({junction_t.junction_id, junction_t});
    }
    NEW_LOG << fmt::format(
        "cross_event find_cross_junction:{:d} prev:[{},{:.2f}]  next:[{},{:.2f}] junction_id:{}  ids:{}  s_offset:{:.2f}  s_remain:{:.2f}",
        find_cross_junction, prev_junction_idx, dis_prev, next_junction_idx, dis_next, junction_t.junction_id, junction_t.section_ids,
        s_offset, s_remain);
  }
}

void TrafficLightDiscern::CrossLinkJunctions() {
  if (!routing_map_ptr_) {
    return;
  }
  const auto       &route_sections = mpp_sections_;
  std::vector<bool> vec_int;
  vec_int.reserve(route_sections.size());
  for (std::size_t idx = 0; idx < route_sections.size(); idx++) {
    vec_int[idx] = LinkTypeHasCross(route_sections[idx].link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  }
  std::vector<uint64_t> ids;
  std::uint64_t         counter  = 0;
  const std::uint64_t   loop_max = 10000;

  const int ext_idx   = 6;
  int       idx_start = static_cast<int>(section_ranges_.idx_start);
  idx_start           = idx_start < ext_idx ? 0 : idx_start - ext_idx;
  NEW_LOG << fmt::format("idx_start:{}  idx_end:{}  ext_idx:{}", idx_start, section_ranges_.idx_end, ext_idx);

  for (std::size_t idx = std::max(0, idx_start);
       idx < route_sections.size() && idx < section_ranges_.idx_end + ext_idx && counter < loop_max; counter++) {
    ids.clear();
    ids.emplace_back(route_sections[idx].id);
    if (!vec_int[idx]) {
      idx++;
      continue;
    }
    std::size_t idx_j = idx + 1;
    for (; idx_j < route_sections.size(); idx_j++) {
      ids.emplace_back(route_sections[idx_j].id);
      if (!vec_int[idx_j]) {
        break;
      }
    }
    JunctionInfo junction_info;
    if (idx > 0) {
      junction_info.junction_id = route_sections[idx - 1].id;
      ids.insert(ids.begin(), junction_info.junction_id);
    } else {
      junction_info.junction_id = route_sections[idx].id;
    }

    junction_info.junction_type |= static_cast<uint64_t>(JunctionTypeSource::CrossLink);
    junction_info.section_ids = ids;
    NEW_LOG << fmt::format("cross_junc_id:{}   ids:{}", junction_info.junction_id, ids);
    if (sd_junctions_.count(junction_info.junction_id) == 1) {
      NEW_LOG << fmt::format("find_loop_id:", junction_info.junction_id);
      return;
    }
    sd_junctions_.insert({junction_info.junction_id, junction_info});

    idx = idx_j + 1;
  }
}

void TrafficLightDiscern::IsInJunction() {
  ;
}

void TrafficLightDiscern::GetLeastTopics() {
  SensorDataManager::Instance()->GetLatestSensorFrame(sd_traffic_light_raw_ptr_);

  MapEventPtr map_event_ptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr);
  if (map_event_ptr) {
    navi_actions_ = GetActionEvent(map_event_ptr);
  }

  navi_actions_.erase(std::remove_if(navi_actions_.begin(), navi_actions_.end(),
                                     [](const NaviActionInfo &rhs) {
                                       return rhs.main_action == NaviMainAction::NMA_NONE ||
                                              rhs.main_action == NaviMainAction::NMA_UNKNOWN ||
                                              rhs.main_action == NaviMainAction::NMA_MERGE_RIGHT ||
                                              rhs.assistant_action == NaviAssistantAction::NAA_VIA1 ||
                                              rhs.assistant_action == NaviAssistantAction::NAA_VIA2 ||
                                              rhs.assistant_action == NaviAssistantAction::NAA_VIA3 ||
                                              rhs.assistant_action == NaviAssistantAction::NAA_VIA4;
                                     }),
                      navi_actions_.end());
  // TODO(lingpeng): recieve e2e topics.
  // SensorDataManager::Instance()->GetLatestSensorFrame(e2e_map_ptr_);
}

bool TrafficLightDiscern::CollectSDSections() {
  for (const auto &sec : routing_map_ptr_->sd_route.mpp_sections) {
    sd_sections_map_.insert({sec.id, &sec});
  }
  for (const auto &sub_path : routing_map_ptr_->sd_route.subpaths) {
    for (const auto &sec : sub_path.sections) {
      sd_sections_map_.insert({sec.id, &sec});
    }
  }

  ego_state_  = routing_map_ptr_->sd_route.navi_start;
  auto ego_it = sd_sections_map_.find(ego_state_.section_id);

  if (ego_it == sd_sections_map_.end()) {
    NEW_LOG << fmt::format("ego_section_navi_section_id:{}  s_offset:{:.2f}", ego_state_.section_id, ego_state_.s_offset);
    return false;
  }

  for (const auto &lane_group : routing_map_ptr_->sd_lane_groups) {
    sd_lane_groups_map_.insert({lane_group.id, &lane_group});
    for (const auto &lane_info : lane_group.lane_info) {
      sd_lanes_map_.insert({lane_info.id, &lane_info});
    }
  }

  for (auto it_t = junction_corrects_.begin(); it_t != junction_corrects_.begin();) {
    if (sd_sections_map_.count(it_t->first) == 0) {
      it_t = junction_corrects_.erase(it_t);
    } else {
      ++it_t;
    }
  }

  return true;
}

std::optional<std::size_t> TrafficLightDiscern::GetEgoIndexInSDSections() {
  if (!routing_map_ptr_) {
    NEW_LOG << "routing_map_ptr_is_nullptr. Can't Get Ego Index.";
    return nullopt;
  }
  if (ego_state_.section_id == 0) {
    NEW_LOG << "ego_state_section_id==0. Can't Get Ego Index.";
    return nullopt;
  }

  auto it_ego_opt = FindTheElement(mpp_sections_, ego_state_.section_id);
  if (!it_ego_opt) {
    NEW_LOG << fmt::format("ego_state_section_id:{}. Can't Get Ego Index.", ego_state_.section_id);
    return nullopt;
  }

  return std::distance(mpp_sections_.cbegin(), *it_ego_opt);
}

void TrafficLightDiscern::ProjectRawTrafficLightToSD() {
  if (!sd_traffic_light_raw_ptr_ || !sd_traffic_light_raw_ptr_->has_traffic_light_dist()) {
    return;
  }
  double dis_next_traffic_light = sd_traffic_light_raw_ptr_->traffic_light_dist();
  if (dis_next_traffic_light < 1.0) {
    return;
  }
  const auto &route_sections = mpp_sections_;
  auto        ego_index_opt  = GetEgoIndexInSDSections();
  if (!ego_index_opt || *ego_index_opt >= route_sections.size()) {
    return;
  }

  double dis = route_sections[*ego_index_opt].length - ego_state_.s_offset;
  if (dis < dis_next_traffic_light && dis + route_sections[*ego_index_opt].length > dis_next_traffic_light) {
    section_aux_map_infos_[route_sections[*ego_index_opt].id].traffic_light_raw = dis_next_traffic_light - dis;
    NEW_LOG << fmt::format("inserted_sd_traffic_light section_id:{}  traffic_lights_dis:{}", route_sections[*ego_index_opt].id,
                           dis_next_traffic_light);
    return;
  }
  for (std::size_t idx = *ego_index_opt; idx < route_sections.size(); idx++) {
    if (dis >= dis_max_section_length) {
      return;
    }
    if (dis <= dis_next_traffic_light && dis + route_sections[idx].length >= dis_next_traffic_light) {
      section_aux_map_infos_[route_sections[idx].id].traffic_light_raw = dis_next_traffic_light - dis;
      NEW_LOG << fmt::format("inserted_sd_traffic_light section_id:{}  traffic_lights_dis:{}", route_sections[*ego_index_opt].id,
                             dis_next_traffic_light);
      return;
    }
    dis += route_sections[idx].length;
  }
}

void TrafficLightDiscern::ProjectEventLightToSD() {
  if (navi_actions_.empty()) {
    return;
  }
  auto it_ego_opt = FindTheElement(mpp_sections_, ego_state_.section_id);
  if (!it_ego_opt) {
    return;
  }
  const auto &route_sections = mpp_sections_;
  auto        ego_index_opt  = GetEgoIndexInSDSections();
  if (!ego_index_opt || *ego_index_opt >= route_sections.size()) {
    return;
  }

  double      dis = route_sections[*ego_index_opt].length - ego_state_.s_offset;
  std::size_t idx = *ego_index_opt;
  for (const auto &navi_action : navi_actions_) {
    if (navi_action.action_dis > dis_max_section_length) {
      return;
    }
    if (navi_action.action_dis <= dis) {
      section_aux_map_infos_[route_sections[idx].id].navi_actions.emplace_back<NaviActionInfoSection>(
          {navi_action, navi_action.action_dis + ego_state_.s_offset});
      NEW_LOG << fmt::format("inserted_navi_action route_section_id:{}  navi_action:{}  assis_action:{} dis:{:.2f}", route_sections[idx].id,
                             navi_action.main_action, navi_action.assistant_action, navi_action.action_dis + ego_state_.s_offset);
      continue;
    }
    for (idx++; idx < route_sections.size(); idx++) {
      if (dis <= navi_action.action_dis && dis + route_sections[idx].length >= navi_action.action_dis) {
        section_aux_map_infos_[route_sections[idx].id].navi_actions.emplace_back<NaviActionInfoSection>(
            {navi_action, navi_action.action_dis - dis});
        NEW_LOG << fmt::format("inserted_navi_action route_section_id:{}  navi_action:{}  assis_action:{} dis:{:.2f} action_s:{:.2f}",
                               route_sections[idx].id, navi_action.main_action, navi_action.assistant_action, navi_action.action_dis,
                               navi_action.action_dis - dis);
        break;
      }
      dis += route_sections[idx].length;
    }
  }
}

void TrafficLightDiscern::ValidSectionRange() {
  const auto &route_sections = mpp_sections_;
  auto        ego_index_opt  = GetEgoIndexInSDSections();
  if (!ego_index_opt || *ego_index_opt >= route_sections.size()) {
    return;
  }
  double      dis           = route_sections[*ego_index_opt].length - ego_state_.s_offset;
  std::size_t idx           = *ego_index_opt;
  section_ranges_.idx_start = idx > 0 ? idx - 1 : idx;
  section_ranges_.idx_end   = idx;
  if (dis > dis_max_section_length) {
    return;
  }
  // auto it = section_aux_map_infos_.find(route_sections[idx].id);
  // if (it != section_aux_map_infos_.end()) {
  //   return;
  // }
  const auto &section_ego = route_sections[idx];

  fmt::memory_buffer buf;
  fmt::format_to(buf, "section_id-angle:");
  for (idx++; idx < route_sections.size(); idx++) {
    const auto &section_idx = route_sections[idx];
    dis += route_sections[idx].length;
    // auto        it_t        = section_aux_map_infos_.find(section_idx.id);
    // if (it_t != section_aux_map_infos_.end() && !it_t->second.navi_actions.empty() &&
    //     std::any_of(it_t->second.navi_actions.begin(), it_t->second.navi_actions.end(),
    //                 [&it_t](const NaviActionInfoSection &action_info_section) {
    //                   if (E2EActionConvertTurnType(action_info_section.navi_action.main_action) != TurnType::NO_TURN) {
    //                     NEW_LOG << fmt::format("find_id:{} contain_action:{}", it_t->first, action_info_section.navi_action.main_action);
    //                     return true;
    //                   }
    //                   return false;
    //                 })) {
    //   break;
    // }
    if (dis > dis_max_section_length) {
      break;
    }
    auto val_opt = navigation::AngleDiffBetweenSection(section_ego, section_idx, false, true);
    if (val_opt) {
      fmt::format_to(buf, "[{},{:.2f}],", section_idx.id, val_opt.value());
    } else {
      break;
    }
    if (std::fabs(val_opt.value()) > navigation::Angle_75) {
      // NEW_LOG << "stop_at_this_section.";
      if (idx > *ego_index_opt) {
        idx--;
      }
      break;
    }
  }
  NEW_LOG << std::string_view(buf.data(), buf.size());
  section_ranges_.idx_end = idx;
  if (section_ranges_.idx_start >= 0 && section_ranges_.idx_end >= 0 && section_ranges_.idx_start < route_sections.size() &&
      section_ranges_.idx_end < route_sections.size()) {
    NEW_LOG << fmt::format("valid_range:{} --->>> {}  idx:[{},{}]", route_sections[section_ranges_.idx_start].id,
                           route_sections[section_ranges_.idx_end].id, section_ranges_.idx_start, section_ranges_.idx_end);
  } else {
    NEW_LOG << fmt::format("valid_range idx:{} --->>> {}  size:{}", section_ranges_.idx_start, section_ranges_.idx_end,
                           route_sections.size());
  }
}

void TrafficLightDiscern::E2eMapSdRoutingMap() {
  if (e2e_map_ptr_ && !routing_map_ptr_) {
    routing_map_ptr_                                 = std::make_shared<RoutingMap>();
    routing_map_ptr_->sd_route.id                    = e2e_map_ptr_->route_info().id();
    routing_map_ptr_->sd_route.navi_start.section_id = e2e_map_ptr_->route_info().navi_start().section_id();
    routing_map_ptr_->sd_route.navi_start.s_offset   = e2e_map_ptr_->route_info().navi_start().s_offset();
    //...
  }
}

void TrafficLightDiscern::ClustorTrafficLights() {
  auto cluster_obj_opt = ClusterObjects(perception_traffic_lights_);
  if (!cluster_obj_opt) {
    NEW_LOG << "cluster_obj_is_empty.";
    return;
  }
  cluster_traffics_ = cluster_obj_opt.value();
}

void TrafficLightDiscern::ProjectTrafficLightsToSD() {
  if (cluster_traffics_.clusters.empty()) {
    NEW_LOG << "cluster is empty.";
    return;
  }
  const auto &route_sections = mpp_sections_;

  std::map<uint64_t, MultiLineSegment> section_segment_map;

  std::vector<Vec2d> points;
  MultiLineSegment   segment_temp;
  // Point3DD           point{};
  NEW_LOG << fmt::format("section_id     section_length    segment_length");
  for (std::size_t idx = section_ranges_.idx_start; idx <= section_ranges_.idx_end && idx < route_sections.size(); idx++) {
    points.clear();
    const auto &section_t = route_sections[idx];
    for (const auto &p_0 : *section_t.points) {
      // point.x = p_0.x();
      // point.y = p_0.y();
      // point   = TransformCoordinate(point, ego_coordinate_);

      points.emplace_back(p_0.x(), p_0.y());
    }
    if (points.size() < 2) {
      continue;
    }
    segment_temp.Init(points);
    section_segment_map.insert({section_t.id, segment_temp});
    NEW_LOG << fmt::format("{}  {:.2f}  {:.2f}", section_t.id, section_t.length, segment_temp.Length());
    // NEW_LOG << fmt::format("section_id:{}  length:{:.2f}  seg_len:{:.2f} point:[{:.2f},{:.2f}]-->[{:.2f},{:.2f}]", section_t.id,
    //                        section_t.length, segment_temp.Length(), points.begin()->x(), points.begin()->y(), points.rbegin()->x(),
    //                        points.rbegin()->y());
  }
  Vec2d  traffic_pos;
  double pos_s{0.0};
  double pos_l{0.0};
  double min_dis{0.0};

  struct SectionProjValue {
    uint64_t             section_id{0};
    double               dis_min{std::numeric_limits<double>::infinity()};
    TrfObjectInfoSection obj_t;
  };
  std::vector<SectionProjValue> vec_section_projs;
  SectionProjValue              section_temp;

  std::vector<uint64_t> traffic_ids;
  constexpr double      extend_dis     = 50.0;
  uint32_t              vision_counter = vision_traffic_light_ptr_ ? vision_traffic_light_ptr_->header.cycle_counter : 0;
  for (const auto &traffic_lights : cluster_traffics_.clusters) {
    traffic_ids.clear();
    for (const auto &obj : traffic_lights) {
      traffic_ids.emplace_back(obj.id);
    }
    const auto &point = ClusterTrafficLights::GetClusterPosition(traffic_lights);
    traffic_pos.set_x(point.x);
    traffic_pos.set_y(point.y);
    bool is_inserted{false};
    NEW_LOG << fmt::format("cluster_traffic_ids:{} position:[{:.2f},{:.2f},{:.2f}]", traffic_ids, point.x, point.y, point.z);
    for (std::size_t idx = section_ranges_.idx_start; idx <= section_ranges_.idx_end && idx < route_sections.size(); idx++) {
      auto it_seg = section_segment_map.find(route_sections[idx].id);
      if (it_seg == section_segment_map.end()) {
        NEW_LOG << fmt::format("can't find the segment:{}", route_sections[idx].id);
        continue;
      }
      bool        is_proj  = it_seg->second.GetProjection(traffic_pos, &pos_s, &pos_l, &min_dis);
      const auto &points_t = it_seg->second.GetPathPoints();
      NEW_LOG << fmt::format("route_id:{} sl:[{:.2f},{:.2f}]  is_proj:{:d} len:{:.2f} point:[{:.2f},{:.2f}]-->[{:.2f},{:.2f}]",
                             route_sections[idx].id, pos_s, pos_l, is_proj, it_seg->second.Length(), points_t.front().x(),
                             points_t.front().y(), points_t.back().x(), points_t.back().y());
      if (!is_proj) {
        continue;
      }
      TrfObjectInfoSection trf_obj_tmp = TrfObjectInfoSection{traffic_lights, pos_s, pos_l, vision_counter};
      if (pos_s >= k_epsilon && pos_s <= it_seg->second.Length() - k_epsilon) {
        is_inserted = true;
        section_aux_map_infos_[route_sections[idx].id].traffic_light_perception.emplace_back(trf_obj_tmp);
        NEW_LOG << fmt::format("inserted_perception_obj section_id:{}", route_sections[idx].id, traffic_ids);
        break;
      }
      if (idx == section_ranges_.idx_end && pos_s > it_seg->second.Length() && pos_s < it_seg->second.Length() + extend_dis) {
        is_inserted = true;
        NEW_LOG << fmt::format("inserted_perception_obj section_id:{}", route_sections[idx].id, traffic_ids);
        section_aux_map_infos_[route_sections[idx].id].traffic_light_perception.emplace_back(trf_obj_tmp);
        break;
      }
      section_temp.obj_t      = trf_obj_tmp;
      section_temp.section_id = route_sections[idx].id;
      section_temp.dis_min    = pos_s < k_epsilon ? pos_s : pos_s - it_seg->second.Length();
      vec_section_projs.push_back(section_temp);
    }
    if (!is_inserted) {
      std::sort(vec_section_projs.begin(), vec_section_projs.end(),
                [](const SectionProjValue &lhs, const SectionProjValue &rhs) { return std::fabs(lhs.dis_min) < std::fabs(rhs.dis_min); });
      if (!vec_section_projs.empty() && std::fabs(vec_section_projs.front().dis_min) < 0.5) {
        section_aux_map_infos_[vec_section_projs.front().section_id].traffic_light_perception.emplace_back(vec_section_projs.front().obj_t);
        NEW_LOG << fmt::format("find_min_dis:{:.2f} inserted_perception_obj section_id:{}", vec_section_projs.front().dis_min,
                               vec_section_projs.front().section_id, traffic_ids);
      } else {
        NEW_LOG << fmt::format("no_find_related_section_id.");
      }
    }
  }
}

void TrafficLightDiscern::TransformEgoPosBaseSD() {
  const auto &section_ego = *sd_sections_map_[ego_state_.section_id];
  ego_coordinate_         = GetPositionOnSection(section_ego, ego_state_.s_offset);
  NEW_LOG << fmt::format("ego_position_in_sd:[{:.2f},{:.2f}] heading:{:.2f} section_id:{}  s_offset:{:.2f}", ego_coordinate_.tx,
                         ego_coordinate_.ty, ego_coordinate_.theta, ego_state_.section_id, ego_state_.s_offset);

  // for (auto &per_obj : perception_traffic_lights_) {
  // per_obj = TransformTrafficObject(per_obj, ego_coordinate_);
  // }
}

bool TrafficLightDiscern::ValidCheck() {
  if (!routing_map_ptr_) {
    NEW_LOG << "routing_map_ptr_is_nullptr.";
    return false;
  }
  if (routing_map_ptr_->sd_route.navi_start.section_id == 0) {
    NEW_LOG << fmt::format("routing_map_counter:{}  navi_section_id:0", routing_map_ptr_->header.cycle_counter);
    return false;
  }

  return true;
}

void TrafficLightDiscern::Reset() {
  routing_map_ptr_.reset();
  bev_map_ptr_.reset();
  vision_traffic_light_ptr_.reset();
  perception_traffic_lights_.clear();
  e2e_map_ptr_.reset();
  navi_actions_.clear();
  sd_sections_map_.clear();
  sd_lane_groups_map_.clear();
  sd_lanes_map_.clear();
  sd_traffic_light_raw_ptr_.reset();
  dynamic_objects_.reset();
  section_ranges_ = SectionRange();
  section_aux_map_infos_.clear();
  ego_dis_ = std::numeric_limits<double>::infinity();
  mpp_sections_.clear();
  sd_junctions_.clear();
  next_junction_   = std::nullopt;
  ego_width_left_  = default_ego_left;
  ego_width_right_ = default_ego_right;
  ego_back_        = std::nullopt;
  stopline_info_.clear();
  traffic_lights_ = TrafficLights();
  deal_per_traffic_light_objects_.clear();
  ego_left_ids_.clear();
  ego_right_ids_.clear();
  polygon_info_.clear();
  lane_match_bev_sd_ = false;
  ego_is_last_wait_  = false;
  while (junction_previous_.size() > max_deque_size) {
    junction_previous_.pop_front();
  }

  while (section_app_info_previous_.size() > max_deque_size) {
    section_app_info_previous_.pop_front();
  }
  if (junction_corrects_.size() > 5000) {
    junction_corrects_.clear();
  }
}

void TrafficLightDiscern::FilterVisionFusionTrafficLights() {
  SensorDataManager::Instance()->GetLatestSensorFrame(vision_traffic_light_ptr_);
  if (!vision_traffic_light_ptr_) {
    return;
  }
  SensorDataManager::Instance()->GetLatestSensorFrame(objects_fusion_ptr_);
  if (!objects_fusion_ptr_) {
    NEW_LOG << "object_fusion_is_nullptr.";
  }

  // if (fabs(timestamp_ - traffic_light_info->header.timestamp) < 1e-3) {
  //   return;
  // }

  NEW_LOG << fmt::format("traffic_light_info_cycle_counter:{}  publish_time:{:.3f}  measurement_time:{:.3f} ",
                         vision_traffic_light_ptr_->header.cycle_counter, vision_traffic_light_ptr_->header.recv_timestamp,
                         vision_traffic_light_ptr_->header.timestamp);
  perception_traffic_lights_.clear();
  for (const auto &obj : vision_traffic_light_ptr_->objects) {
    if (obj.type != ObjectType::TRAFFIC_LIGHT) {
      continue;
    }
    // if (obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK &&
    //     obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_LEFT_BACK &&
    //     obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_RIGHT_BACK) {
    //   NEW_LOG << fmt::format("Remove_Trf!!!  direction_reason id:{}  direction:{}", obj.id, obj.attributes.traffic_light_direction);
    //   continue;
    // }
    if (std::isnan(obj.position.x) || std::isnan(obj.position.y)) {
      NEW_LOG << fmt::format("Remove_Trf!!! nan_reason obj id:{} is pos_nan_value. pos x:{} y:{}  z:{}", obj.id, obj.position.x,
                             obj.position.y, obj.position.z);
      continue;
    }

    if (FilterTrafficLightShape(obj.attributes.traffic_light_shape)) {
      NEW_LOG << fmt::format("Remove_Trf!!!  shape_reason id:{}  shape:{}", obj.id, obj.attributes.traffic_light_shape);
      continue;
    }

    if (obj.position.x > dis_max_longitudinal || obj.position.x < dis_min_longitudinal) {
      NEW_LOG << fmt::format("Remove_Trf!!! long_dis_reason id:{}  obj_x:{:.2f}  max_lon:{:.2f} min_lon:{:.2f}", obj.id, obj.position.x,
                             dis_max_longitudinal, dis_min_longitudinal);
      continue;
    }

    // if (obj.position.y > dis_max_lateral) {
    // NEW_LOG << fmt::format("filter_obj_x:{:.2f}  more_than_threshold:{:.2f}", obj.position.x, dis_max_lateral);
    // continue;
    // }

    NEW_LOG << fmt::format(
        "normal_traffic_light_id:{}  position:[{:.2f},{:.2f},{:.2f}] shape:{}  color:{}  num:{} is_flash:{:d} direction:{}", obj.id,
        obj.position.x, obj.position.y, obj.position.z, magic_enum::enum_name(obj.attributes.traffic_light_shape),
        magic_enum::enum_name(obj.attributes.traffic_light_color), obj.attributes.traffic_light_num, obj.attributes.traffic_light_flashing,
        magic_enum::enum_name(obj.attributes.traffic_light_direction));

    perception_traffic_lights_.push_back(obj);
    perception_traffic_lights_.back().publish_time = vision_traffic_light_ptr_->header.timestamp;
  }
  traffic_lights_.header.timestamp      = vision_traffic_light_ptr_->header.timestamp;
  traffic_lights_.header.recv_timestamp = vision_traffic_light_ptr_->header.recv_timestamp;
  traffic_lights_.header.cycle_counter  = vision_traffic_light_ptr_->header.cycle_counter;
}

}  // namespace cem::fusion