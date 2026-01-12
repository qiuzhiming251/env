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
 * @file geometry_match_info.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-11
 */

#ifndef GEOMETRY_MATCH_INFO_H_
#define GEOMETRY_MATCH_INFO_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "Eigen/src/Core/Matrix.h"
#include "cyber/common/log.h"

#include "lib/common/log_custom.h"
#include "lib/common/utils/GeoMathUtil.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/sensor/camera/bev_lane/bev_lane.h"
#include "modules/perception/env/src/lib/common/utility.h"

// 为 Eigen::Matrix<double, 2, 1> 实现格式化器
namespace fmt {
template <>
struct formatter<Eigen::Matrix<double, 2, 1>> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  // format 方法实现
  template <typename FormatContext>
  auto format(const Eigen::Matrix<double, 2, 1> &vec, FormatContext &ctx) const {
    return format_to(ctx.out(), "({:.2f}, {:.2f})", vec[0],
                     vec[1]);  // 示例格式化为 (x, y)
  }
};

// 为 std::vector<unsigned long> 实现格式化器
template <>
struct formatter<std::vector<uint64_t>> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const std::vector<uint64_t> &vec, FormatContext &ctx) {
    auto out = format_to(ctx.out(), "[");
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i > 0) {
        out = format_to(out, ", ");
      }
      out = format_to(out, "{}", vec[i]);
    }
    return format_to(out, "]");
  }
};
}  // namespace fmt

namespace cem::fusion {

enum class MapConvertType {
  BEV_2_LD          = 0,
  BEV_2_UNKNOWN     = 1,
  BEV_2_BEV         = 2,
  LD_2_BEV          = 3,
  LD_2_LD           = 4,
  UNKNOWN_2_LD      = 5,
  UNKNOWN_2_BEV     = 6,
  UNKNOWN_2_UNKNOWN = 7,
  DEFAULT_BEV       = 8,
};

inline const char *to_string(MapConvertType type) {
  switch (type) {
    case MapConvertType::BEV_2_LD:
      return "BEV_2_LD";
    case MapConvertType::BEV_2_UNKNOWN:
      return "BEV_2_UNKNOWN";
    case MapConvertType::BEV_2_BEV:
      return "BEV_2_BEV";
    case MapConvertType::LD_2_BEV:
      return "LD_2_BEV";
    case MapConvertType::LD_2_LD:
      return "LD_2_LD";
    case MapConvertType::UNKNOWN_2_LD:
      return "UNKNOWN_2_LD";
    case MapConvertType::UNKNOWN_2_BEV:
      return "UNKNOWN_2_BEV";
    case MapConvertType::UNKNOWN_2_UNKNOWN:
      return "UNKNOWN_2_UNKNOWN";
    case MapConvertType::DEFAULT_BEV:
      return "DEFAULT_BEV";
    default:
      return "UNKNOWN_MapConvertType";
  }
}

struct LineSegment {
  std::vector<uint64_t> left_boundary_id_{};
  std::vector<uint64_t> right_boundary_id_{};
  uint64_t              lane_id{0};
};

struct LineSegmentGeometryMatch {
  struct MatchMarks {
    double          average{0.0};
    double          max_lat{std::numeric_limits<double>::max()};
    double          rate{0.0};
    Eigen::Vector2d start_point{};
  };

  std::vector<LineSegment>               bev_lane_;
  std::vector<LineSegment>               ld_lane_;
  std::vector<std::pair<double, double>> left_matchers_{};
  std::vector<std::pair<double, double>> right_matchers_{};
  MatchMarks                             left_match_markers_;
  MatchMarks                             right_match_markers_;
};

constexpr double distance_threshold = 30.0;

enum MatchFailReason {
  kNotFailed = 0,
  kLostBevLane = 1,
  kLostHdMapLane = 2,
  kLostAllLane = 3,
  kMatchFailed = 4
};

struct GeometryMatchResult {
  MatchFailReason fail_reason;
  bool is_matched;
  double left_offset_average;
  double left_offset_max;
  double right_offset_average;
  double right_offset_max;
  double ld_line_c0;
  double bev_line_c0;
  void Reset() {
    fail_reason = MatchFailReason::kNotFailed;
    is_matched = false;
    left_offset_average = 0.0;
    left_offset_max = 0.0;
    right_offset_average = 0.0;
    right_offset_max = 0.0;
  };
};

struct GeometryMatchInfo {
 public:
  using BevMapInfo    = cem::message::sensor::BevMapInfo;
  using BevLaneInfo   = cem::message::sensor::BevLaneInfo;
  using BevLaneMarker = cem::message::sensor::BevLaneMarker;

  using RoutingMap       = cem::message::env_model::RoutingMap;
  using LaneInfo         = cem::message::env_model::LaneInfo;
  using LaneBoundaryInfo = cem::message::env_model::LaneBoundaryInfo;
  /**
     * @brief get ego lane(bev,ld);collects points(bev,ld);projection.
     * all in all, complete bev_ld_map_geometry_match_info_ info.
     * @param bev_map_info
     * @param route_map_info
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void CalculateMapGeometryMatchInfo(const std::shared_ptr<BevMapInfo> &bev_map_info, const std::shared_ptr<RoutingMap> &route_map_info);

  [[nodiscard]] bool IsLaneBoundaryMatch(
      GeometryMatchResult &result,
      const cem::message::sensor::BevLanePosition &position_type =
          cem::message::sensor::BevLanePosition::LANE_LOC_EGO);

  [[nodiscard]] BevLaneInfo GetEgoBevLane() const { return ego_lane_bev_; }

  [[nodiscard]] LaneInfo GetLdBevLane() const { return ego_lane_ld_; }
  std::shared_ptr<BevMapInfo> GetBevMap() const { return bev_map_; }
  std::shared_ptr<RoutingMap> GetLdMap() const { return ld_map_; }

  [[nodiscard]] const std::string &GetDebugInfo() const { return debug_info_; }

  [[nodiscard]] std::optional<LineSegmentGeometryMatch> GetEgoLanes(
      cem::message::sensor::BevLanePosition lane_pos = cem::message::sensor::BevLanePosition::LANE_LOC_EGO) const {
    auto it = map_geometry_match_info_.find(lane_pos);
    if (it != map_geometry_match_info_.end()) {
      return it->second;
    };
    return std::nullopt;
  }
  [[nodiscard]] std::map<cem::message::sensor::BevLanePosition, LineSegmentGeometryMatch> GetLanePairs() const {
    return map_geometry_match_info_;
  }

  // [[nodiscard]] std::shared_ptr<BevMapInfo> GetBevMap() const { return bev_map_; }
  // [[nodiscard]] std::shared_ptr<RoutingMap> GetLdMap() const { return ld_map_; }

  [[nodiscard]] bool LdMapIsValid(double length = distance_threshold) const;

  [[nodiscard]] bool LDHasEnoughPoints(double length = distance_threshold) const {
    if (!ld_central_points_.empty()) {
      GEOMETRY_LOG << fmt::format("ld_points size:{}  front_x:{:.2f}  back_x:{:.2f}", ld_central_points_.size(),
                                  ld_central_points_.front().x(), ld_central_points_.back().x());
    } else {
      GEOMETRY_LOG << "ld_point_is_empty.";
    }
    bool IsLdValid      = LdMapIsValid(length);
    bool IsEgoLaneValid = IsMapValid(ld_central_points_, length);
    GEOMETRY_LOG << "IsEgoLaneValid:" << IsEgoLaneValid << "  IsLdValid:" << IsLdValid;
    return IsLdValid;
  }

  [[nodiscard]] bool BEVHasEnoughPoints(double length = distance_threshold) const {
    if (!bev_central_points_.empty()) {
      GEOMETRY_LOG << fmt::format("bev_points size:{}  front_x:{:.2f}  back_x:{:.2f}", bev_central_points_.size(),
                                  bev_central_points_.front().x(), bev_central_points_.back().x());
    } else {
      GEOMETRY_LOG << "bev_point_is_empty.";
    }
    return IsMapValid(bev_central_points_, length);
  }

  [[nodiscard]] const std::vector<Eigen::Vector2d> &GetLdLanePoints() const { return ld_central_points_; }

  [[nodiscard]] const std::vector<Eigen::Vector2d> &GetBevLanePoints() const { return bev_central_points_; }

  void PrintLineSegmentGeometryMatch(const LineSegmentGeometryMatch &match)const;

 private:
  /**
     * @brief Set the Bev Lane Ids (ego lane,left lane, right lane)
     * @param bev_map_info
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void SetBevLaneIds(const std::shared_ptr<BevMapInfo> &bev_map_info);

  static bool IsMapValid(const std::vector<Eigen::Vector2d> &points, double length = distance_threshold) {
    if (points.empty()) {
      return false;
    }
    double total_length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
      total_length += (points[i] - points[i - 1]).norm();
      if (total_length > length) {
        return true;
      }
    }
    return false;
  }

  /**
     * @brief Set the Ld Lane Ids(ego lane, left lane, right lane)
     * @param route_map_info
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  bool FindLDEgoLane(const std::shared_ptr<RoutingMap> &route_map_info, uint64_t section_id);

  /**
     * @brief assign the lane id to map_geometry_match_info_.ld_lane_
     * @param route_map_info
     * @param lane_info
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void SetLDLaneIds(const std::shared_ptr<RoutingMap> &route_map_info, const LaneInfo &lane_info);

  /**
     * @brief 1.Get the Boundary Points object 2.project points(bev) to points(ld)
     * 3. calculate the averager lateral distance,get max distance
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void GetBoundaryPoints();

  /**
     * @brief projection bev points to ld points, calculate left_matchers_, right_matchers_,
      left_match_markers_, right_match_markers_
     * @param bev_left_points
     * @param bev_right_points
     * @param ld_left_points
     * @param ld_right_points
     * @param ego_match_info_ptr
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void CalculateMatchInfo(const std::vector<Eigen::Vector2d> &bev_left_points, const std::vector<Eigen::Vector2d> &bev_right_points,
                          const std::vector<Eigen::Vector2d> &ld_left_points, const std::vector<Eigen::Vector2d> &ld_right_points,
                          LineSegmentGeometryMatch &ego_match_info_ptr);
  /**
     * @brief 1. get the position objection 2. set bev id(lane,boundary) based on raw_bev_lane
     * @param raw_bev_lane
     * @param position_type
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void SetBevInfo(const BevLaneInfo &raw_bev_lane, cem::message::sensor::BevLanePosition position_type);
  /**
     * @brief 1. set bev ego info(call SetBevInfo) 2. set bev ego info(call SetBevInfo)
     * 3. set bev ego info(call SetBevInfo)
     * @param bev_map_info
     * @param bev_lane
     *
     * @author lingpeng (ling.peng3@byd.com)
     * @date 2025-03-11
     */
  void InsertBevLane(const std::shared_ptr<BevMapInfo> &bev_map_info, const BevLaneInfo &bev_lane);

  bool is_ld_map_close_end;
  std::shared_ptr<BevMapInfo> bev_map_{nullptr};
  std::shared_ptr<RoutingMap> ld_map_{nullptr};

  std::map<cem::message::sensor::BevLanePosition, LineSegmentGeometryMatch> map_geometry_match_info_;

  BevLaneInfo                  ego_lane_bev_;
  LaneInfo                     ego_lane_ld_;
  std::string                  debug_info_;
  std::vector<Eigen::Vector2d> ld_central_points_;
  std::vector<Eigen::Vector2d> bev_central_points_;

  void Reset() {
    is_ld_map_close_end = false;
    bev_map_ = nullptr;
    ld_map_  = nullptr;
    map_geometry_match_info_.clear();
    ego_lane_bev_ = BevLaneInfo{};
    ego_lane_ld_  = LaneInfo{};
    debug_info_.clear();
    ld_central_points_.clear();
    bev_central_points_.clear();
  }
};

}  // namespace cem::fusion
#endif
