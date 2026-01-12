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
 * @file traffic_light_common.cpp
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-06-26
 */

#include "modules/perception/env/src/lib/localmap_construction/traffic_light_common.h"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <Eigen/Core>
#include <localmap_construction/traffic_light_map_topo.h>
#include <message/common/geometry.h>
#include <message/sensor/vision/tsrmobject.h>
namespace cem::fusion {
LightStatus LightColorConvert(const TrafficLightColorType &color_type, bool is_blinking) {
  switch (color_type) {
    case cem::message::sensor::TLC_RED_FLASHING:
    case message::sensor::TLC_RED: {
      return message::env_model::LightStatus::RED_LIGHT;
    }
    case message::sensor::TLC_GREEN: {
      return is_blinking ? message::env_model::LightStatus::GREEN_BLINKING : message::env_model::LightStatus::GREEN_LIGHT;
    }
    case message::sensor::TLC_YELLOW: {
      return is_blinking ? message::env_model::LightStatus::YELLOW_BLINKING : message::env_model::LightStatus::YELLOW_LIGHT;
    }
    case cem::message::sensor::TLC_GREEN_FLASHING:
      return message::env_model::LightStatus::GREEN_BLINKING;
    case cem::message::sensor::TLC_YELLOW_FLASHING:
      return message::env_model::LightStatus::YELLOW_BLINKING;
    case cem::message::sensor::TLC_BLURRING_MODE:
      return message::env_model::LightStatus::BLURRING_MODE;
    case cem::message::sensor::TLC_BLOCK_FAILED:
      return message::env_model::LightStatus::BLOCK_FAIL;
    case cem::message::sensor::TLC_CLOUD_NOMATCH:
    case cem::message::sensor::TLC_NOT_MATCH:
      return message::env_model::LightStatus::CLOUD_NOT_MATCH;
    default: {
      return message::env_model::LightStatus::UNKNOWN_LIGHT;
    }
  }
}

message::common::Point3DD ClusterTrafficLights::GetClusterPosition(const std::vector<TrfObjectInfo> &traffic_lights) {
  if (traffic_lights.empty()) {
    return message::common::Point3DD{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                                     std::numeric_limits<double>::infinity()};
  }
  double dis_x = std::accumulate(traffic_lights.begin(), traffic_lights.end(), 0.0,
                                 [](double sum, const auto &trf_light) { return sum + trf_light.position.x; });
  double dis_y = std::accumulate(traffic_lights.begin(), traffic_lights.end(), 0.0,
                                 [](double sum, const auto &trf_light) { return sum + trf_light.position.y; });
  double dis_z = std::accumulate(traffic_lights.begin(), traffic_lights.end(), 0.0,
                                 [](double sum, const auto &trf_light) { return sum + trf_light.position.z; });

  const auto size = static_cast<double>(traffic_lights.size());

  return message::common::Point3DD{dis_x / size, dis_y / size, dis_z / size};
}

void ClusterTrafficLights::DebugString(const char *title) const {
  fmt::memory_buffer buffer;

  fmt::format_to(std::back_inserter(buffer), "{:=^60}\n", title);

  size_t total_objects = 0;
  for (const auto &cluster : clusters) {
    total_objects += cluster.size();
  }

  fmt::format_to(std::back_inserter(buffer), "• Clusters: {:5d}   |   Objects: {:5d}   |   Unique mappings: {:5d}\n{:-^60}\n",
                 clusters.size(), total_objects, object_to_cluster.size(), "");

  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto &cluster = clusters[i];
    fmt::format_to(std::back_inserter(buffer), "Cluster #{:03d} ({} objects)\n", i, cluster.size());

    for (const auto &obj : cluster) {
      fmt::format_to(std::back_inserter(buffer), "  ├─ ID:{:5d} → Pos:({:6.2f}, {:6.2f})", obj.id, obj.position.x, obj.position.y);

      if (auto it = object_to_cluster.find(obj.id); it != object_to_cluster.end() && it->second == i) {
        fmt::format_to(std::back_inserter(buffer), " ✓\n");
      } else {
        fmt::format_to(std::back_inserter(buffer), " ✗ (mapping error!)\n");
      }
    }
    fmt::format_to(std::back_inserter(buffer), "\n");
  }

  fmt::format_to(std::back_inserter(buffer), "{:-^60}\n", "Object-Cluster Mapping");
  size_t counter = 0;
  for (const auto &[obj_id, cluster_idx] : object_to_cluster) {
    fmt::format_to(std::back_inserter(buffer), "({:4} → {:2}) ", obj_id, cluster_idx);
    counter++;

    if (counter % 10 == 0) {
      fmt::format_to(std::back_inserter(buffer), "\n");
    }
  }
  if (counter % 10 != 0) {
    fmt::format_to(std::back_inserter(buffer), "\n");
  }

  fmt::format_to(std::back_inserter(buffer), "{:=^60}", "End");

  buffer.push_back('\0');
  const char *debug_cstr = buffer.data();

  NEW_LOG << debug_cstr;
}

Point3DD TransformCoordinate(const Point3DD &global_point, const TransformParams &params) {
  double delta_x = global_point.x - params.tx;
  double delta_y = global_point.y - params.ty;

  double inv_scale = 1.0 / params.scale;
  double cos_theta = std::cos(params.theta);
  double sin_theta = std::sin(params.theta);

  Point3DD local_point{};
  local_point.x = (delta_x * cos_theta + delta_y * sin_theta) * inv_scale;
  local_point.y = (-delta_x * sin_theta + delta_y * cos_theta) * inv_scale;
  local_point.z = global_point.z * inv_scale;

  return local_point;
}

TrfObjectInfo TransformTrafficObject(const TrfObjectInfo &obj, const TransformParams &params) {
  TrfObjectInfo transformed = obj;

  auto point_new = TransformCoordinate(obj.position, params);

  NEW_LOG << fmt::format("obj_id:{}  old_point:[{:.2f},{:.2f},{:.2f}]  new_point:[{:.2f},{:.2f},{:.2f}]", obj.id, obj.position.x,
                         obj.position.y, obj.position.z, point_new.x, point_new.y, point_new.z);

  transformed.position = point_new;
  transformed.length *= static_cast<float>(params.scale);
  transformed.width *= static_cast<float>(params.scale);
  transformed.height *= static_cast<float>(params.scale);

  return transformed;
}

TransformParams GetPositionOnSection(const SDSectionInfo &section, double position_from_start) {
  TransformParams default_val{};
  if (position_from_start < 0.0) {
    return default_val;
  }

  if (position_from_start >= section.length) {
    if (section.points && !section.points->empty()) {
      const auto &points   = *section.points;
      size_t      last_idx = points.size() - 1;
      float       theta    = (points.size() > 1)
                                 ? std::atan2(points[last_idx].y() - points[last_idx - 1].y(), points[last_idx].x() - points[last_idx - 1].x())
                                 : 0.0F;
      return {points.back().x(), points.back().y(), theta, 1.0F};
    }
    return default_val;
  }

  if (!section.points || section.points->size() < 2) {
    if (section.points && !section.points->empty()) {
      return {section.points->front().x(), section.points->front().y(), 0.0F, 1.0F};
    }
    return default_val;
  }

  double      accumulated_distance = 0.0;
  const auto &points               = *section.points;

  for (size_t i = 1; i < points.size(); ++i) {
    const auto     &point_0        = points[i - 1];
    const auto     &point_1        = points[i];
    Eigen::Vector2f segment_vec    = point_1 - point_0;
    float           segment_length = segment_vec.norm();

    if (position_from_start <= accumulated_distance + segment_length) {
      double          offset_on_segment = position_from_start - accumulated_distance;
      double          ratio             = offset_on_segment / segment_length;
      Eigen::Vector2f position          = point_0 + ratio * segment_vec;
      float           theta             = std::atan2(segment_vec.y(), segment_vec.x());
      return {position.x(), position.y(), theta, 1.0F};
    }

    accumulated_distance += segment_length;
  }

  size_t last_idx = points.size() - 1;
  float  theta    = (points.size() > 1)
                        ? std::atan2(points[last_idx].y() - points[last_idx - 1].y(), points[last_idx].x() - points[last_idx - 1].x())
                        : 0.0F;
  return {points.back().x(), points.back().y(), theta, 1.0F};
}

inline double CalcDistance2D(const TrfObjectInfo &obj_a, const TrfObjectInfo &obj_b) noexcept {
  const double delta_x = obj_a.position.x - obj_b.position.x;
  const double delta_y = obj_a.position.y - obj_b.position.y;
  return std::hypot(delta_x, delta_y);
}

std::optional<ClusterTrafficLights> ClusterObjects(const std::vector<TrfObjectInfo> &traffic_light_objects, double threshold_m) {
  ClusterTrafficLights result;
  if (traffic_light_objects.empty()) {
    return result;
  }

  std::vector<bool>                  processed(traffic_light_objects.size(), false);
  std::vector<std::vector<uint64_t>> obs_ids;

  for (size_t i = 0; i < traffic_light_objects.size(); ++i) {
    if (processed[i]) {
      continue;
    }

    std::vector<TrfObjectInfo> current_cluster;
    std::vector<size_t>        to_process{i};
    std::vector<uint64_t>      objs_id;

    while (!to_process.empty()) {
      const size_t current_idx = to_process.back();
      to_process.pop_back();

      if (processed[current_idx]) {
        continue;
      }

      processed[current_idx] = true;
      current_cluster.push_back(traffic_light_objects[current_idx]);
      objs_id.emplace_back(traffic_light_objects[current_idx].id);
      result.object_to_cluster[traffic_light_objects[current_idx].id] = result.clusters.size();

      for (size_t j = 0; j < traffic_light_objects.size(); ++j) {
        if (!processed[j]) {
          const double dist = CalcDistance2D(traffic_light_objects[current_idx], traffic_light_objects[j]);
          // NEW_LOG << fmt::format("cluster_obj_id:{}  obj_id:{}  dis:{:.2f}", traffic_light_objects[current_idx].id,
          //  traffic_light_objects[j].id, dist);
          if (dist <= threshold_m) {
            to_process.push_back(j);
          }
        }
      }
    }

    if (!current_cluster.empty()) {
      result.clusters.push_back(std::move(current_cluster));
      obs_ids.emplace_back(objs_id);
    }
  }
  NEW_LOG << fmt::format("obj_cluster:{}", obs_ids);

  return result;
}

Eigen::Vector2f CalculatePointAlongLine(const Eigen::Vector2f &start, const Eigen::Vector2f &end, double len) {
  Eigen::Vector2f direction      = end - start;
  double          total_distance = direction.norm();

  if (total_distance <= std::numeric_limits<float>::epsilon()) {
    return start;
  }

  Eigen::Vector2f unit_vector = direction / total_distance;

  return start + unit_vector * len;
}

}  // namespace cem::fusion