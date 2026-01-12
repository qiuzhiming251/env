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
 * @file traffic_light_common.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-26
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <math/math.h>
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/localmap_construction/cloud_config_traffic_light.h"
#include "modules/perception/env/src/lib/message/common/geometry.h"
#include "modules/perception/env/src/lib/message/env_model/routing_map/map_event.h"
#include "modules/perception/env/src/lib/message/env_model/routing_map/routing_map.h"
#include "modules/perception/env/src/lib/message/sensor/vision/tsrmobject.h"

#include "fmt/core.h"
#include "fmt/format.h"
#include "fmt/ranges.h"
#include "magic_enum/magic_enum.hpp"

namespace cem::fusion {
using byd::common::math::LineSegment2d;
using byd::common::math::Polygon2d;
using byd::common::math::Vec2d;
using cem::message::sensor::TrafficLightShapeType;
using message::common::Point3DD;
using message::env_model::LightStatus;
using message::env_model::NaviActionInfo;
using message::env_model::NaviMainAction;
using message::env_model::SDSectionInfo;
using message::env_model::TurnType;
using message::sensor::TrafficLightColorType;
using message::sensor::TrfObjectInfo;
using PerTrfObjectInfoMaps = std::unordered_map<uint32_t, TrfObjectInfo>;

struct ClusterTrafficLights {
  std::vector<std::vector<TrfObjectInfo>> clusters;
  std::unordered_map<uint32_t, size_t>    object_to_cluster;  // 对象ID到聚类索引的映射

  void DebugString(const char *title = "Traffic Light Clustering Result") const;

  static message::common::Point3DD GetClusterPosition(const std::vector<TrfObjectInfo> &traffic_lights);  // z maybe nan value
};
struct TransformParams {
  double           tx{0.0};
  double           ty{0.0};
  double           theta{0.0};
  double           scale{1.0};
  MultiLineSegment ego_line{};
  double           width{0.0};
  double           length{0.0};
};

inline double CalculateDistance(const Point3DD &p_1, const Point3DD &p_2, bool is3D = false) {
  if (is3D) {
    return std::sqrt(std::pow(p_2.x - p_1.x, 2) + std::pow(p_2.y - p_1.y, 2) + std::pow(p_2.z - p_1.z, 2));
  }
  return std::sqrt(std::pow(p_2.x - p_1.x, 2) + std::pow(p_2.y - p_1.y, 2));
}

LightStatus LightColorConvert(const TrafficLightColorType &color_type, bool is_blinking = false);

TransformParams GetPositionOnSection(const SDSectionInfo &section, double position_from_start);

TrfObjectInfo TransformTrafficObject(const TrfObjectInfo &obj, const TransformParams &params);

inline bool LinkTypeHasCross(uint32_t link_type, message::env_model::SDLinkTypeMask link_mask) {
  return (link_type & static_cast<uint32_t>(link_mask)) == static_cast<uint32_t>(link_mask);
}

template <typename T>
Polygon2d GetTheElementOfMap(const std::vector<T> &ele_pool, uint64_t ele_id, double stopline_pan = 0.0) {
  auto it = std::find_if(ele_pool.begin(), ele_pool.end(), [ele_id](const auto &rhs) { return rhs.id == ele_id; });
  if (it == ele_pool.end()) {
    return Polygon2d{};
  }
  std::vector<Vec2d> points;
  if (it->points.size() < 3) {
    if (stopline_pan < 0.1 || it->points.size() < 2) {
      return Polygon2d{};
    }
    for (auto it_p = it->points.rbegin(); it_p != it->points.rend(); it_p++) {
      points.emplace_back(it_p->x - stopline_pan, it_p->y);
    }
  }
  for (auto point : it->points) {
    points.emplace_back(point.x, point.y);
  }
  return Polygon2d{points};
}

inline bool FilterTrafficLightShape(const TrafficLightShapeType &shape) {
  return shape == TrafficLightShapeType::TLS_DOWN_ARROW || shape == TrafficLightShapeType::TLS_BICYCLE ||
         shape == TrafficLightShapeType::TLS_CLOSE_TO_TRAFFIC || shape == TrafficLightShapeType::TLS_SLOW_DOWN ||
         shape == TrafficLightShapeType::TLS_HEART_SHAPE || shape == TrafficLightShapeType::TLS_NUMBER ||
         shape == TrafficLightShapeType::TLS_PROCESS_BAR || shape == TrafficLightShapeType::TLS_BUS_ONLY_SHAPE ||
         shape == TrafficLightShapeType::TLS_RECTANGLE || shape == TrafficLightShapeType::TLS_PEDESTRIAN;
}

std::optional<ClusterTrafficLights> ClusterObjects(const std::vector<TrfObjectInfo> &traffic_light_objects, double threshold_m = 5.0);

inline cem::message::env_model::TurnType E2EActionConvertTurnType(const NaviMainAction &navi_main_action) {
  switch (navi_main_action) {
    case NaviMainAction::NMA_TURN_LEFT:
    case NaviMainAction::NMA_SLIGHT_LEFT:
    case NaviMainAction::NMA_TURN_HARD_LEFT:
      return cem::message::env_model::TurnType::LEFT_TURN;
    case NaviMainAction::NMA_TURN_RIGHT:
    case NaviMainAction::NMA_ENTRY_RING:
    case NaviMainAction::NMA_SLIGHT_RIGHT:
    case NaviMainAction::NMA_TURN_HARD_RIGHT:
    case NaviMainAction::NMA_RIGHT_UTURN:
      return cem::message::env_model::TurnType::RIGHT_TURN;
    case NaviMainAction::NMA_LEFT_UTURN:
      return cem::message::env_model::TurnType::U_TURN;
    default:
      return cem::message::env_model::TurnType::NO_TURN;
  }
}

inline uint32_t GetBaseTurnVal(TurnType turn_type) {
  uint32_t turn_type_value = 0b0000;
  if (turn_type == TurnType::NO_TURN) {
    turn_type_value = 0b0001;
  } else if (turn_type == TurnType::LEFT_TURN) {
    turn_type_value = 0b0010;
  } else if (turn_type == TurnType::RIGHT_TURN) {
    turn_type_value = 0b0100;
  } else if (turn_type == TurnType::U_TURN) {
    turn_type_value = 0b1000;
  } else {
    turn_type_value = 0b0001;
  }
  return turn_type_value;
}

message::common::Point3DD TransformCoordinate(const message::common::Point3DD &global_point, const TransformParams &params);

inline uint32_t GetTurnTypeVal(TurnType turn_type) {
  switch (turn_type) {
    case message::env_model::TurnType::NO_TURN:
      return 0b0001;
    case message::env_model::TurnType::LEFT_TURN:
      return 0b0010;
    case message::env_model::TurnType::RIGHT_TURN:
      return 0b0100;
    case message::env_model::TurnType::U_TURN:
      return 0b1000;
    case message::env_model::TurnType::STRAIGHT_AND_LEFT_TURN:
      return 0b0011;
    case message::env_model::TurnType::STRAIGHT_AND_RIGHT_TURN:
      return 0b0101;
    case message::env_model::TurnType::STRAIGHT_AND_U_TURN:
      return 0b1001;
    case message::env_model::TurnType::LEFT_TURN_AND_U_TURN:
      return 0b1010;
    case message::env_model::TurnType::RIGHT_TURN_AND_U_TURN:
      return 0b1100;
    case message::env_model::TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN:
      return 0b0111;
    case message::env_model::TurnType::LEFT_TURN_AND_RIGHT_TURN:
      return 0b0110;
    case message::env_model::TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN:
      return 0b1011;
    case message::env_model::TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN:
      return 0b1101;
    case message::env_model::TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN:
      return 0b1110;
    case message::env_model::TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN:
      return 0b1111;
    case message::env_model::TurnType::OTHER_UNKNOWN:
    default:
      return 0b0000;
  }
};

Eigen::Vector2f CalculatePointAlongLine(const Eigen::Vector2f &start, const Eigen::Vector2f &end, double len);

}  // namespace cem::fusion

namespace fmt {
template <>
struct formatter<cem::message::env_model::Point> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::env_model::Point &vec, FormatContext &ctx) const {
    return format_to(ctx.out(), "({:.2f}, {:.2f})", vec.x, vec.y);
  }
};

template <>
struct formatter<cem::message::env_model::Point2D> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::env_model::Point2D &rhs, FormatContext &ctx) const {
    return format_to(ctx.out(), "({:.2f}, {:.2f})", rhs.x, rhs.y);
  }
};

template <>
struct formatter<cem::message::env_model::TrafficLightStatus> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::env_model::TrafficLightStatus &rhs, FormatContext &ctx) const {
    return format_to(ctx.out(), "(light_status:{}, turn:{} num:{} lanes:{} is_navi:{})", rhs.light_status, rhs.turn_type,
                     rhs.traffic_light_num, rhs.lane_ids, rhs.is_navi_light);
  }
};

}  // namespace fmt