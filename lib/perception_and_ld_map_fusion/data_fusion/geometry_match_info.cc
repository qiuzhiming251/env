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
 * @file geometry_match_info.cc
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-11
 */
#include <log_custom.h>
#include <sys/types.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include <cyber/common/log.h>
#include <message/sensor/camera/bev_lane/bev_lane.h>

#include "fmt/core.h"

#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"

namespace cem::fusion {
void GeometryMatchInfo::SetBevInfo(const BevLaneInfo &raw_bev_lane, cem::message::sensor::BevLanePosition position_type) {
  auto [iterator, inserted] = map_geometry_match_info_.insert({position_type, {}});
  auto &bev_lane_ref        = iterator->second.bev_lane_.emplace_back();
  bev_lane_ref.lane_id      = raw_bev_lane.id;
  bev_lane_ref.left_boundary_id_.emplace_back(raw_bev_lane.left_lane_marker_id);
  bev_lane_ref.right_boundary_id_.emplace_back(raw_bev_lane.right_lane_marker_id);
  GEOMETRY_LOG << "position:" << static_cast<int>(position_type) << "  lane_id:" << bev_lane_ref.lane_id
               << "  left_bound_size:" << bev_lane_ref.left_boundary_id_.size()
               << "  right_bound_size:" << bev_lane_ref.right_boundary_id_.size();
}

void GeometryMatchInfo::InsertBevLane(const std::shared_ptr<BevMapInfo> &bev_map_info, const BevLaneInfo &bev_lane) {
  SetBevInfo(bev_lane, cem::message::sensor::BevLanePosition::LANE_LOC_EGO);
  if (static_cast<bool>(bev_lane.left_lane_id)) {
    auto it_left_lane = FindTheElement(bev_map_info->lane_infos, bev_lane.left_lane_id);
    if (it_left_lane.has_value()) {
      SetBevInfo(*(it_left_lane.value()), cem::message::sensor::BevLanePosition::LANE_LOC_LEFT_FIRST);
    }
  }
  if (static_cast<bool>(bev_lane.right_lane_id)) {
    auto it_right_lane = FindTheElement(bev_map_info->lane_infos, bev_lane.right_lane_id);
    if (it_right_lane.has_value()) {
      SetBevInfo(*(it_right_lane.value()), cem::message::sensor::BevLanePosition::LANE_LOC_RIGHT_FIRST);
    }
  }
}

void GeometryMatchInfo::SetBevLaneIds(const std::shared_ptr<BevMapInfo> &bev_map_info) {
  struct NextLaneOverview {
    uint64_t                                    id{0};
    std::vector<cem::message::common::Point2DF> line_points;
    double                                      lat{0.0};
  };

  BevLaneInfo                      bev_lane{};
  std::vector<const BevLaneInfo *> ego_lanes;
  for (const auto &lane : bev_map_info->lane_infos) {
    // position==0 mean ego lane,filter others.
    if (lane.position == 0) {
      auto it_left_bound  = FindTheElement(bev_map_info->lanemarkers, lane.left_lane_marker_id);
      auto it_right_bound = FindTheElement(bev_map_info->lanemarkers, lane.right_lane_marker_id);
      if (it_left_bound.has_value() && it_right_bound.has_value()) {
        auto it_l_p = std::find_if(it_left_bound.value()->line_points.begin(), it_left_bound.value()->line_points.end(),
                                   [](const auto &poi) { return poi.x > 0.0; });
        auto it_r_p = std::find_if(it_right_bound.value()->line_points.begin(), it_right_bound.value()->line_points.end(),
                                   [](const auto &poi) { return poi.x > 0.0; });
        if (it_l_p != it_left_bound.value()->line_points.end() && it_r_p != it_right_bound.value()->line_points.end() &&
            it_l_p->y * it_r_p->y < 0.0) {
          bev_lane = lane;
        } else {
          ego_lanes.emplace_back(&lane);
        }
      }
      break;
    }
  }
  ego_lane_bev_ = bev_lane;
  if (bev_lane.id == 0) {
    GEOMETRY_LOG << "bev_lane can't find the ego lane. bev_lane_counter:" << bev_map_info->header.cycle_counter;
    for (const auto *ego_ptr : ego_lanes) {
      for (const auto succ_id : ego_ptr->next_lane_ids) {
        auto it_lane = FindTheElement(bev_map_info->lane_infos, succ_id);
        if (!it_lane) {
          continue;
        }
        auto it_left_bound  = FindTheElement(bev_map_info->lanemarkers, it_lane.value()->left_lane_marker_id);
        auto it_right_bound = FindTheElement(bev_map_info->lanemarkers, it_lane.value()->right_lane_marker_id);
        if (it_left_bound.has_value() && it_right_bound.has_value()) {
          auto it_l_p = std::find_if(it_left_bound.value()->line_points.begin(), it_left_bound.value()->line_points.end(),
                                     [](const auto &poi) { return poi.x > 0.0; });
          auto it_r_p = std::find_if(it_right_bound.value()->line_points.begin(), it_right_bound.value()->line_points.end(),
                                     [](const auto &poi) { return poi.x > 0.0; });
          if (it_l_p != it_left_bound.value()->line_points.end() && it_r_p != it_right_bound.value()->line_points.end() &&
              it_l_p->y * it_r_p->y < 0.0) {
            bev_lane = *ego_ptr;
            break;
          }
        }
      }
      if (bev_lane.id == 0) {
        GEOMETRY_LOG << "can't_find_the_ego_lane after_next.";
        return;
      }
    }
  }
  InsertBevLane(bev_map_info, bev_lane);
  uint8_t counter = 0;
  while (counter++ < 2) {
    GEOMETRY_LOG << "bev_ego_lane:" << bev_lane.id;
    const size_t next_lane_size = bev_lane.next_lane_ids.size();
    if (next_lane_size == 0) {
      break;
    }
    auto it_next = FindTheElement(bev_map_info->lane_infos, bev_lane.next_lane_ids.front());
    if (!it_next.has_value()) {
      break;
    }
    if (next_lane_size == 1) {
      bev_lane = *(it_next.value());
      InsertBevLane(bev_map_info, bev_lane);
      continue;
    }
    //sort next lane base on angle
    std::vector<NextLaneOverview> next_lanes_overview;
    next_lanes_overview.reserve(next_lane_size);
    for (const auto &id : bev_lane.next_lane_ids) {
      auto it_next_lane_info = FindTheElement(bev_map_info->lane_infos, id);
      if (!it_next_lane_info.has_value() || it_next_lane_info.value()->line_points.size() < 2) {
        continue;
      }
      const auto &start_p = it_next_lane_info.value()->line_points.front();
      const auto &end_p   = it_next_lane_info.value()->line_points.back();
      double      delta_x = end_p.x - start_p.x;
      double      delta_y = end_p.y - start_p.y;
      // NOLINTBEGIN
      double angle = 90 - (std::fabs(delta_x) > 1e-6 ? (std::atan(delta_y / delta_x) * 180 / M_PI) : (delta_y > 0 ? 90 : -90));
      // NOLINTEND
      next_lanes_overview.emplace_back(NextLaneOverview{id, {start_p, end_p}, angle});
    }
    if (next_lanes_overview.empty()) {
      break;
    }
    std::sort(next_lanes_overview.begin(), next_lanes_overview.end(), [](const auto &lhs, const auto &rhs) { return lhs.lat < rhs.lat; });

    // get appropriate lane.
    if (bev_lane.split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) {
      it_next = FindTheElement(bev_map_info->lane_infos, next_lanes_overview.front().id);
    } else if (bev_lane.split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
      it_next = FindTheElement(bev_map_info->lane_infos, next_lanes_overview.back().id);
    } else {
      break;
    }
    if (it_next.has_value()) {
      bev_lane = *(it_next.value());
      InsertBevLane(bev_map_info, bev_lane);
    } else {
      break;
    }
  }
}

void GeometryMatchInfo::SetLDLaneIds(const std::shared_ptr<RoutingMap> &route_map_info, const LaneInfo &lane_info) {
  auto it_ego_loc = map_geometry_match_info_.find(cem::message::sensor::BevLanePosition::LANE_LOC_EGO);
  if (it_ego_loc == map_geometry_match_info_.end()) {
    GEOMETRY_LOG << "cant_find_the_bev_ego_lane.";
    auto [it, inserted] = map_geometry_match_info_.insert({cem::message::sensor::BevLanePosition::LANE_LOC_EGO, {}});
    it_ego_loc          = it;
  }
  auto &ld_lane_ref              = it_ego_loc->second.ld_lane_.emplace_back();
  ld_lane_ref.lane_id            = lane_info.id;
  ld_lane_ref.left_boundary_id_  = lane_info.left_lane_boundary_ids;
  ld_lane_ref.right_boundary_id_ = lane_info.right_lane_boundary_ids;
  GEOMETRY_LOG << "ld_map_lane_id:" << lane_info.id << "  left_bound_id:" << lane_info.left_lane_boundary_ids.front()
               << "  right_bound_id:" << lane_info.right_lane_boundary_ids.front();
  auto it_left_loc    = map_geometry_match_info_.find(cem::message::sensor::BevLanePosition::LANE_LOC_LEFT_FIRST);
  auto it_left_lane_t = FindTheElement(route_map_info->lanes, lane_info.left_lane_id);
  if (it_left_loc != map_geometry_match_info_.end() && it_left_lane_t.has_value()) {
    auto  left_lane_t                = it_left_lane_t.value();
    auto &left_lane_ref              = it_left_loc->second.ld_lane_.emplace_back();
    left_lane_ref.lane_id            = left_lane_t->id;
    left_lane_ref.left_boundary_id_  = left_lane_t->left_lane_boundary_ids;
    left_lane_ref.right_boundary_id_ = left_lane_t->right_lane_boundary_ids;
  }
  auto it_right_loc    = map_geometry_match_info_.find(cem::message::sensor::BevLanePosition::LANE_LOC_RIGHT_FIRST);
  auto it_right_lane_t = FindTheElement(route_map_info->lanes, lane_info.right_lane_id);
  if (it_right_loc != map_geometry_match_info_.end() && it_right_lane_t.has_value()) {
    auto  right_lane_t                = it_right_lane_t.value();
    auto &right_lane_ref              = it_right_loc->second.ld_lane_.emplace_back();
    right_lane_ref.lane_id            = right_lane_t->id;
    right_lane_ref.left_boundary_id_  = right_lane_t->left_lane_boundary_ids;
    right_lane_ref.right_boundary_id_ = right_lane_t->right_lane_boundary_ids;
  }
}


void GeometryMatchInfo::PrintLineSegmentGeometryMatch(const LineSegmentGeometryMatch& match)const {
  AINFO << "=== LineSegmentGeometryMatch ===" ;

  AINFO << "-- BEV Lanes --" ;
  for (size_t i = 0; i < match.bev_lane_.size(); ++i) {
    const auto& lane = match.bev_lane_[i];
    AINFO << "  [" << i << "] lane_id: " << lane.lane_id ;
    AINFO << "    left_boundary_ids: ";
    for (auto id : lane.left_boundary_id_) AINFO << id << " ";
    AINFO << "    right_boundary_ids: ";
    for (auto id : lane.right_boundary_id_) AINFO << id << " ";
  }

  AINFO << "-- LD Lanes --" ;
  for (size_t i = 0; i < match.ld_lane_.size(); ++i) {
    const auto& lane = match.ld_lane_[i];
    AINFO << "  [" << i << "] lane_id: " << lane.lane_id ;
    AINFO << "    left_boundary_ids: ";
    for (auto id : lane.left_boundary_id_) AINFO << id << " ";
    AINFO << "    right_boundary_ids: ";
    for (auto id : lane.right_boundary_id_) AINFO << id << " ";
  }

  // AINFO << "-- Left Matchers (s, lat) --" ;
  // for (const auto& [s, lat] : match.left_matchers_) {
  //   AINFO << "  s: " << s << ", lat: " << lat ;
  // }

  // AINFO << "-- Right Matchers (s, lat) --" ;
  // for (const auto& [s, lat] : match.right_matchers_) {
  //   AINFO << "  s: " << s << ", lat: " << lat ;
  // }

  // AINFO << "-- Left Match Marks --" ;
  // AINFO << "  avg: " << match.left_match_markers_.average
  //           << ", max_lat: " << match.left_match_markers_.max_lat
  //           << ", rate: " << match.left_match_markers_.rate
  //           << ", start_point: [" << match.left_match_markers_.start_point.x()
  //           << ", " << match.left_match_markers_.start_point.y() << "]" ;

  // AINFO << "-- Right Match Marks --" ;
  // AINFO << "  avg: " << match.right_match_markers_.average
  //           << ", max_lat: " << match.right_match_markers_.max_lat
  //           << ", rate: " << match.right_match_markers_.rate
  //           << ", start_point: [" << match.right_match_markers_.start_point.x()
  //           << ", " << match.right_match_markers_.start_point.y() << "]" ;
}

bool GeometryMatchInfo::FindLDEgoLane(const std::shared_ptr<RoutingMap> &route_map_info, uint64_t section_id) {
  auto it_section = std::find_if(route_map_info->route.sections.begin(), route_map_info->route.sections.end(),
                                 [section_id](const auto &section_t) { return section_t.id == section_id; });
  if (it_section == route_map_info->route.sections.end()) {
    GEOMETRY_LOG << "No_Find_section_id:" << section_id;
    return false;
  }

  for (auto lane_id_t : it_section->lane_ids) {
    auto it_lane_t = FindTheElement(route_map_info->lanes, lane_id_t);
    if (!it_lane_t.has_value()) {
      GEOMETRY_LOG << "No_Find_section_id:" << it_section->id << "  lane_id:" << lane_id_t;
      continue;
    }
    auto it_lane = it_lane_t.value();
    if (it_lane->left_lane_boundary_ids.empty() || it_lane->right_lane_boundary_ids.empty()) {
      GEOMETRY_LOG << "No_Find_section_id:" << it_section->id << "  lane_id:" << lane_id_t
                   << "  left_ids_is_empty:" << it_lane->left_lane_boundary_ids.empty()
                   << "  right_empty:" << it_lane->right_lane_boundary_ids.empty();
      continue;
    }
    auto it_left_boundary =
        std::find_if(route_map_info->lane_boundaries.begin(), route_map_info->lane_boundaries.end(),
                     [&it_lane](const auto &boundary) { return boundary.id == it_lane->left_lane_boundary_ids.front(); });
    auto it_right_boundary =
        std::find_if(route_map_info->lane_boundaries.begin(), route_map_info->lane_boundaries.end(),
                     [&it_lane](const auto &boundary) { return boundary.id == it_lane->right_lane_boundary_ids.front(); });
    if (it_left_boundary == route_map_info->lane_boundaries.end() || it_left_boundary->points.empty() ||
        it_right_boundary == route_map_info->lane_boundaries.end() || it_right_boundary->points.empty()) {
      continue;
    }
    auto it_left_boundary_point =
        std::find_if(it_left_boundary->points.begin(), it_left_boundary->points.end(), [](const auto &point) { return point.x > 0.0; });
    auto it_right_boundary_point =
        std::find_if(it_right_boundary->points.begin(), it_right_boundary->points.end(), [](const auto &point) { return point.x > 0.0; });
    if (it_left_boundary_point == it_left_boundary->points.end() || it_right_boundary_point == it_right_boundary->points.end()) {
      continue;
    }
    // AWARN << "it_lane_id:" << it_lane->id << "  boundary_point:["
    // << it_left_boundary_point->y << "," << it_right_boundary_point->y
    // << "]";
    if (it_left_boundary_point == it_left_boundary->points.begin() ||
        it_right_boundary_point == it_right_boundary->points.begin()) {
      if (it_left_boundary_point->y * it_right_boundary_point->y >= 0) {
      // if (it_lane->id == 474007505) {
      //   fmt::memory_buffer buf_left;
      //   fmt::memory_buffer buf_right;
      //   fmt::format_to(buf_left, " left_bound_x:[");
      //   fmt::format_to(buf_right, " right_bound_x:[");
      //   for (const auto &p : it_left_boundary->points) {
      //     fmt::format_to(buf_left, "[{:5.1f},{:5.1f}],", p.x, p.y);
      //   }
      //   for (const auto &p : it_right_boundary->points) {
      //     fmt::format_to(buf_right, "[{:5.1f},{:5.1f}],", p.x, p.y);
      //   }
      //   fmt::format_to(buf_left, "]");
      //   fmt::format_to(buf_right, "]");
      //   GEOMETRY_LOG << std::string_view(buf_left.data(), buf_left.size());
      //   GEOMETRY_LOG << std::string_view(buf_right.data(), buf_right.size());
      //   GEOMETRY_LOG << "left_y:" << it_left_boundary_point->y << "  right_y:" << it_right_boundary_point->y;
      // }
        continue;
      }
    } else {
      auto calc_y_intercept =
          [](const std::vector<cem::message::env_model::Point2D>::iterator p2) {
            auto p1 = p2 - 1;
            return p1->y - (p2->y - p1->y) * p1->x / (p2->x - p1->x);
          };
      if (calc_y_intercept(it_left_boundary_point) *
              calc_y_intercept(it_right_boundary_point) >=
          0) {
        continue;
      }
    }
    // AWARN << "find the ego lane.";
    double                  distance         = it_section->length - route_map_info->route.navi_start.s_offset;
    static constexpr double max_distance_get = 80;
    ego_lane_ld_                             = *it_lane;
    SetLDLaneIds(route_map_info, *it_lane);
    while (distance < max_distance_get && !it_lane->next_lane_ids.empty()) {
      it_lane_t = FindTheElement(route_map_info->lanes, it_lane->next_lane_ids.front());
      GEOMETRY_LOG << "lane_id:" << it_lane->id << "  next_lane:" << it_lane->next_lane_ids.front();
      if (!it_lane_t) {
        break;
      }
      it_lane = *it_lane_t;
      distance += it_lane->length;
      SetLDLaneIds(route_map_info, *it_lane);
      if (it_lane->next_lane_ids.size() >= 2) {
        break;
      }
    }
    return true;
  }
  return false;
}

const auto DebugPoint = [](const auto &points, const std::string &name, bool debug_flag = false) {
  if (!debug_flag || points.empty()) {
    return;
  }
  fmt::memory_buffer info_x;
  fmt::memory_buffer info_y;
  auto               it = points.begin();
  fmt::format_to(info_x, "Size:{}   {}_x += [{:5.1f}", points.size(), name, it->x);
  fmt::format_to(info_y, "Size:{}   {}_y += [{:5.1f}", points.size(), name, it->y);
  for (++it; it != points.end(); ++it) {
    fmt::format_to(info_x, ",{:5.1f}", it->x);
    fmt::format_to(info_y, ",{:5.1f}", it->y);
  }
  fmt::format_to(info_x, "];");
  fmt::format_to(info_y, "];");
  GEOMETRY_LOG << std::string_view(info_x.data(), info_x.size());
  GEOMETRY_LOG << std::string_view(info_y.data(), info_y.size());
};

const auto CollectPoints = [](size_t loop_i, const auto &points, std::vector<Eigen::Vector2d> *const collect_set) {
  size_t idx = 0;
  if (loop_i == 0) {
    auto it = std::find_if(points.begin(), points.end(), [](const auto &poi) { return poi.x > 0; });
    if (it == points.end()) {
      return;
    }
    idx = static_cast<size_t>(std::distance(points.begin(), it));
    if (idx > 0) {
      GEOMETRY_LOG << "idx:" << idx << " prev_point:[" << points[idx - 1].x << "," << points[idx - 1].y << "]  current_point:["
                   << points[idx].x << "," << points[idx].y << "]";
    }
  }
  if (idx > 0) {
    idx--;
  }
  for (; idx < points.size(); idx++) {
    collect_set->emplace_back(Eigen::Vector2d{points.at(idx).x, points.at(idx).y});
  }
};

bool GeometryMatchInfo::LdMapIsValid(double length) const {
  if (!ld_map_) {
    return false;
  }

  const auto &ego_state      = ld_map_->route.navi_start;
  const auto  it_section_opt = FindTheElement(ld_map_->route.sections, ego_state.section_id);
  if (!it_section_opt || it_section_opt.value()->lane_ids.empty()) {
    return false;
  }

  auto   it_section = it_section_opt.value();
  double distance   = it_section->length - ego_state.s_offset;

  const auto has_valid_lane = [this](const auto &section) {
    return std::any_of(section.lane_ids.begin(), section.lane_ids.end(),
                       [this](uint64_t lane_id) { return FindTheElement(ld_map_->lanes, lane_id); });
  };

  for (++it_section; it_section != ld_map_->route.sections.end(); ++it_section) {
    if (!has_valid_lane(*it_section) || it_section->lane_ids.empty()) {
      break;
    }
    distance += it_section->length;
    // if (distance += it_section->length; distance > length) {
    // return true;
    // }
  }
  GEOMETRY_LOG << fmt::format("section_remain_length:{:.2f}", distance);
  return distance > length;
}

void GeometryMatchInfo::CalculateMatchInfo(  // NOLINT
    const std::vector<Eigen::Vector2d> &bev_left_points, const std::vector<Eigen::Vector2d> &bev_right_points,
    const std::vector<Eigen::Vector2d> &ld_left_points, const std::vector<Eigen::Vector2d> &ld_right_points,
    LineSegmentGeometryMatch &ego_match_info) {
  size_t                  bev_left_size               = bev_left_points.size();
  size_t                  bev_right_size              = bev_right_points.size();
  size_t                  ld_left_size                = ld_left_points.size();
  size_t                  ld_right_size               = ld_right_points.size();
  static constexpr size_t min_point_threshold         = 3;
  static constexpr double min_start_point_x_threshold = 0.0;
  if (bev_left_size < min_point_threshold || bev_right_size < min_point_threshold || ld_left_size < min_point_threshold ||
      ld_right_size < min_point_threshold) {
    GEOMETRY_LOG << fmt::format("bev_left:{} bev_right:{} ld_left:{} ld_right:{}", bev_left_size, bev_right_size, ld_left_size,
                                ld_right_size);
    return;
  }
  if (bev_left_points.front().x() > min_start_point_x_threshold || bev_right_points.front().x() > min_start_point_x_threshold ||
      ld_left_points.front().x() > min_start_point_x_threshold || ld_right_points.front().x() > min_start_point_x_threshold) {
    GEOMETRY_LOG << fmt::format(
        "bev_left_start:{:.2f}  bev_right_start:{:.2f} ld_left_start:{:.2f} "
        "ld_right_start:{:.2f}",
        bev_left_points.front().x(), bev_right_points.front().x(), ld_left_points.front().x(), ld_right_points.front().x());
    return;
  }
  double            lat_dist = 0.0;
  Eigen::Vector2d   foot{0, 0};
  int               out_index{0};
  const double      opt_dis = 0.001;
  std::stringstream lat_dis_info;           // debug info lateral
  std::stringstream lon_dis_info;           // debug info longitudinal
  double            dis_s           = 0.0;  // distance has been projected
  double            max_lateral_dis = std::numeric_limits<double>::lowest();
  double            dis_sum         = 0.0;
  const double      max_bev_dis     = 50.0;  // maximum projection distance.
  lat_dis_info << "Size:" << bev_left_points.size() << "  left_bound_lat_dis = [";
  lon_dis_info << "Size:" << bev_left_points.size() << "  left_bound_lon_dis = [";
  for (size_t i = 0; dis_s < max_bev_dis && i < bev_left_points.size(); i++) {
    if (i > 0) {
      dis_s += (bev_left_points[i] - bev_left_points[i - 1]).norm();
    }
    bool flag = cem::fusion::GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(
        bev_left_points[i], ld_left_points, 0, static_cast<int>(ld_left_points.size()) - 1, false, foot, lat_dist, out_index, opt_dis);
    if (flag) {
      ego_match_info.left_matchers_.emplace_back(dis_s, lat_dist);
      max_lateral_dis = max_lateral_dis < std::fabs(lat_dist) ? std::fabs(lat_dist) : max_lateral_dis;
      dis_sum += std::abs(lat_dist);
      lat_dis_info << lat_dist << ",";
      lon_dis_info << dis_s << ",";
    }
  }
  ego_match_info.left_match_markers_.start_point = ld_left_points.front();
  ego_match_info.left_match_markers_.max_lat     = max_lateral_dis;
  ego_match_info.left_match_markers_.average     = dis_sum / static_cast<double>(ego_match_info.left_matchers_.size());
  GEOMETRY_LOG << lat_dis_info.str() << "]";
  GEOMETRY_LOG << lon_dis_info.str() << "]";
  dis_s = 0;
  lat_dis_info.str("");
  lat_dis_info << "Size:" << bev_right_points.size() << "  right_bound_lat_dis = [";
  lon_dis_info.str("");
  lon_dis_info << "Size:" << bev_right_points.size() << "  right_bound_lon_dis = [";
  max_lateral_dis = std::numeric_limits<double>::lowest();
  dis_sum         = 0.0;
  for (size_t i = 0; dis_s < max_bev_dis && i < bev_right_points.size(); i++) {
    if (i > 0) {
      dis_s += (bev_right_points[i] - bev_right_points[i - 1]).norm();
    }
    bool flag = cem::fusion::GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(
        bev_right_points[i], ld_right_points, 0, static_cast<int>(ld_right_points.size()) - 1, false, foot, lat_dist, out_index, opt_dis);
    if (flag) {
      ego_match_info.right_matchers_.emplace_back(dis_s, lat_dist);
      max_lateral_dis = max_lateral_dis < std::fabs(lat_dist) ? std::fabs(lat_dist) : max_lateral_dis;
      dis_sum += std::abs(lat_dist);
      lat_dis_info << lat_dist << ",";
      lon_dis_info << dis_s << ",";
    }
  }
  ego_match_info.right_match_markers_.start_point = ld_right_points.front();
  ego_match_info.right_match_markers_.max_lat     = max_lateral_dis;
  ego_match_info.right_match_markers_.average     = dis_sum / static_cast<double>(ego_match_info.right_matchers_.size());
  GEOMETRY_LOG << lat_dis_info.str() << "]";
  GEOMETRY_LOG << lon_dis_info.str() << "]";
  GEOMETRY_LOG << "left_average:" << ego_match_info.left_match_markers_.average
               << "  max_dis:" << ego_match_info.left_match_markers_.max_lat
               << "  right_average:" << ego_match_info.right_match_markers_.average
               << "  max_dis:" << ego_match_info.right_match_markers_.max_lat;
}

void GeometryMatchInfo::GetBoundaryPoints() {
  auto        it_ego     = map_geometry_match_info_.find(cem::message::sensor::BevLanePosition::LANE_LOC_EGO);
  const bool  debug_flag = false;
  const auto &bev_map    = bev_map_;
  const auto &ld_map     = ld_map_;
  if (it_ego == map_geometry_match_info_.end()) {
    return;
  }
  auto &ego_match_info = it_ego->second;
  if (!ego_match_info.bev_lane_.empty()) {
    for (size_t i = 0; i < ego_match_info.bev_lane_.size(); i++) {
      const auto &bev_lane        = ego_match_info.bev_lane_[i];
      auto        it_central_lane = FindTheElement(bev_map->lane_infos, bev_lane.lane_id);
      if (!it_central_lane.has_value()) {
        GEOMETRY_LOG << "can't find ld the central_lane points.";
        continue;
      }
      // DebugPoint(it_central_lane.value()->line_points, "bev_ego_central_point", false);
      CollectPoints(i, it_central_lane.value()->line_points, &bev_central_points_);
    }
  }
  if (!ego_match_info.ld_lane_.empty()) {
    for (size_t i = 0; i < ego_match_info.ld_lane_.size(); i++) {
      const auto &ld_lane         = ego_match_info.ld_lane_[i];
      auto        it_central_lane = FindTheElement(ld_map->lanes, ld_lane.lane_id);
      if (!it_central_lane.has_value()) {
        GEOMETRY_LOG << "can't find ld the central_lane points.";
        continue;
      }
      DebugPoint(it_central_lane.value()->points, "ld_ego_central_point", true);
      CollectPoints(i, it_central_lane.value()->points, &ld_central_points_);
    }
  }
  if (ego_match_info.bev_lane_.empty() || ego_match_info.ld_lane_.empty()) {
    GEOMETRY_LOG << "ego_lane:" << ego_match_info.bev_lane_.empty() << "  ld_lane:" << ego_match_info.ld_lane_.empty();
    return;
  }
  // bev map debug bound points.
  std::vector<Eigen::Vector2d> bev_left_points;
  std::vector<Eigen::Vector2d> bev_right_points;
  std::vector<Eigen::Vector2d> ld_left_points;
  std::vector<Eigen::Vector2d> ld_right_points;
  for (const auto &ld_lane : ego_match_info.ld_lane_) {
    for (std::size_t i = 0; i < ld_lane.left_boundary_id_.size(); i++) {
      auto it_ego_left_bound_ld = FindTheElement(ld_map->lane_boundaries, ld_lane.left_boundary_id_.at(i));
      if (!it_ego_left_bound_ld.has_value()) {
        GEOMETRY_LOG << "can't find ld the left bound.";
        continue;
      }
      DebugPoint(it_ego_left_bound_ld.value()->points, "ld_ego_left_point", debug_flag);
      CollectPoints(i, it_ego_left_bound_ld.value()->points, &ld_left_points);
    }
    for (std::size_t i = 0; i < ld_lane.right_boundary_id_.size(); i++) {
      auto it_ego_right_bound_ld = FindTheElement(ld_map->lane_boundaries, ld_lane.right_boundary_id_.at(i));
      if (!it_ego_right_bound_ld.has_value()) {
        GEOMETRY_LOG << "can't find ld the right bound.";
        continue;
      }
      DebugPoint(it_ego_right_bound_ld.value()->points, "ld_ego_right_point", debug_flag);
      CollectPoints(i, it_ego_right_bound_ld.value()->points, &ld_right_points);
    }
  }
  for (const auto &bev_lane : ego_match_info.bev_lane_) {
    for (size_t i = 0; i < bev_lane.left_boundary_id_.size(); i++) {
      auto it_ego_left_bound_bev = FindTheElement(bev_map->lanemarkers, bev_lane.left_boundary_id_.at(i));
      if (!it_ego_left_bound_bev.has_value()) {
        GEOMETRY_LOG << "can't find bev the left bound.";
        continue;
      }
      DebugPoint(it_ego_left_bound_bev.value()->line_points, "bev_ego_left_point", debug_flag);
      CollectPoints(i, it_ego_left_bound_bev.value()->line_points, &bev_left_points);
    }
    for (size_t i = 0; i < bev_lane.right_boundary_id_.size(); i++) {
      auto it_ego_right_bound_bev = FindTheElement(bev_map->lanemarkers, bev_lane.right_boundary_id_.at(i));
      if (!it_ego_right_bound_bev.has_value()) {
        GEOMETRY_LOG << "can't find bev the right bound.";
        continue;
      }
      DebugPoint(it_ego_right_bound_bev.value()->line_points, "bev_ego_right_point", debug_flag);
      CollectPoints(i, it_ego_right_bound_bev.value()->line_points, &bev_right_points);
    }
  }

  CalculateMatchInfo(bev_left_points, bev_right_points, ld_left_points, ld_right_points, ego_match_info);
}

void GeometryMatchInfo::CalculateMapGeometryMatchInfo(const std::shared_ptr<BevMapInfo> &bev_map_info,
                                                      const std::shared_ptr<RoutingMap> &route_map_info) {
  if (!bev_map_info || !route_map_info) {
    return;
  }
  Reset();
  bev_map_ = bev_map_info;
  ld_map_  = route_map_info;
  SetBevLaneIds(bev_map_info);

  const uint64_t start_id = route_map_info->route.navi_start.section_id;
  bool           find_ld  = FindLDEgoLane(route_map_info, start_id);
  if (!find_ld) {
    auto it_secs = FindTheElement(route_map_info->route.sections, start_id);
    if (it_secs.has_value()) {
      for (const auto &sec_id : it_secs.value()->successor_section_id_list) {
        GEOMETRY_LOG << "test_section_id:" << sec_id;
        if (find_ld = FindLDEgoLane(route_map_info, sec_id); find_ld) {
          break;
        }
      }
    }
  }
  {
    auto it_secs = FindTheElement(route_map_info->route.sections, start_id);
    if (it_secs.has_value()) {
      auto sec = it_secs.value();
      auto len = -route_map_info->route.navi_start.s_offset + sec->length;
      while (true) {
        if (len >= 50.0) {
          break;
        }
        if (sec->successor_section_id_list.empty()) {
          break;
        }
        bool not_found = true;
        for (auto id : sec->successor_section_id_list) {
          auto it_sec_tmp = FindTheElement(route_map_info->route.sections, id);
          if (it_sec_tmp.has_value()) {
            sec = it_sec_tmp.value();
            len += sec->length;
            not_found = false;
            break;
          }
        }
        if (not_found) {
          break;
        }
      }
      if (len < 50.0) {
        is_ld_map_close_end = true;
      }
    } else {
      is_ld_map_close_end = true;
    }
  }
  GEOMETRY_LOG << "routing_map_counter:" << route_map_info->header.cycle_counter << "  start_id:" << start_id
               << "   end_find_ld:" << find_ld;

  GetBoundaryPoints();
}

bool GeometryMatchInfo::IsLaneBoundaryMatch(
    GeometryMatchResult &result,
    const cem::message::sensor::BevLanePosition &position_type) {
  result.Reset();
  auto it_ego = map_geometry_match_info_.find(position_type);
  if (it_ego == map_geometry_match_info_.end()) {
    result.fail_reason = MatchFailReason::kLostAllLane;
    debug_info_ = "cant find the ego ld lane.";
    GEOMETRY_LOG << debug_info_;
    return false;
  }
  const auto &geometry_match = it_ego->second;
  if (geometry_match.bev_lane_.empty() || geometry_match.ld_lane_.empty()) {
    if (!geometry_match.bev_lane_.empty()) {
      result.fail_reason = MatchFailReason::kLostHdMapLane;
    } else if (!geometry_match.ld_lane_.empty()) {
      result.fail_reason = MatchFailReason::kLostBevLane;
    } else {
      result.fail_reason = MatchFailReason::kLostAllLane;
    }
    debug_info_ = fmt::format("bev_lane.size:{}  ld_lane_size:{}", geometry_match.bev_lane_.size(), geometry_match.ld_lane_.size());
    return false;
  }
  if (is_ld_map_close_end) {
    result.fail_reason = MatchFailReason::kLostHdMapLane;
    return false;
  }
  const auto             &left_match                 = geometry_match.left_match_markers_;
  const auto             &right_match                = geometry_match.right_match_markers_;
  static constexpr double boundary_threshold_average = 0.4;
  static constexpr double boundary_threshold_max     = 0.7;
  const double            max_lateral                = 100.0;  // prevent value too big in log.
  const double            lane_straddling_threshold  = 0.3;

  bool is_lane_straddling{false};  // based on ld_map
  bool is_lane_num_matched{true};

  const auto &ego_lane_bev = geometry_match.bev_lane_.front();
  const auto &ego_lane_ld  = geometry_match.ld_lane_.front();

  if (position_type == cem::message::sensor::BevLanePosition::LANE_LOC_EGO) {
    is_lane_straddling =
        left_match.start_point.y() <= lane_straddling_threshold || right_match.start_point.y() >= -lane_straddling_threshold;
    if (is_lane_straddling) {
      GEOMETRY_LOG << fmt::format(
          "is_lane_straddling:{:d}  left_match_start_point:{}  "
          "right_match_start_point:{}",
          is_lane_straddling, left_match.start_point, right_match.start_point);
    }

    if (!ego_lane_bev.left_boundary_id_.empty() && ego_lane_ld.left_boundary_id_.empty()) {
      is_lane_num_matched = false;
    }
    if (!ego_lane_bev.right_boundary_id_.empty() && ego_lane_ld.right_boundary_id_.empty()) {
      is_lane_num_matched = false;
    }
    if (!is_lane_num_matched) {
      AWARN << fmt::format(
          "is_lane_num_matched:{:d}  bev_ego_lane_id:{}  left_lane_ids:{}  "
          "right_lane_ids:{}  bev_ego_ld_id:{}    left_lane_ids:{}  "
          "right_lane_ids:{} ",
          is_lane_num_matched, ego_lane_bev.lane_id, ego_lane_bev.left_boundary_id_, ego_lane_bev.right_boundary_id_, ego_lane_ld.lane_id,
          ego_lane_ld.left_boundary_id_, ego_lane_ld.right_boundary_id_);
    }
  }

  bool is_boundary_matched = left_match.average < boundary_threshold_average && left_match.max_lat < boundary_threshold_max &&
                             right_match.average < boundary_threshold_average && right_match.max_lat < boundary_threshold_max;
  if (!is_boundary_matched) {
    result.fail_reason = MatchFailReason::kMatchFailed;
  } else {
    result.is_matched = true;
  }
  result.left_offset_average = left_match.average;
  result.right_offset_average = right_match.average;
  result.left_offset_max = left_match.max_lat;
  result.right_offset_max = right_match.max_lat;

  GEOMETRY_LOG << fmt::format(
      "position:{} bev_ld_map_geometry_  ld_lane:[{}] bev_lane:[{}]   is_boundary_matched:{:d}  "
      "is_lane_straddling:{:d} is_lane_num_matched:{:d} left_average:{:.2f} "
      "max_dis:{:.2f}  right_average:{:.2f}  max_dis:{:.2f}",
      position_type, ego_lane_ld_.id, ego_lane_bev_.id, is_boundary_matched, is_lane_straddling, is_lane_num_matched, left_match.average,
      std::min(left_match.max_lat, max_lateral), right_match.average, std::min(right_match.max_lat, max_lateral));
  debug_info_ = fmt::format("is_match:{:d}  ld_lane:{}  bev_lane:{}  left:{:.2f}-{:.1f}  right_bound:{:.2f}-{:.1f}  ", is_boundary_matched,
                            ego_lane_ld.lane_id, ego_lane_bev.lane_id, left_match.average, std::min(left_match.max_lat, max_lateral),
                            right_match.average, std::min(right_match.max_lat, max_lateral));

  return result.is_matched;
}
}  // namespace cem::fusion
