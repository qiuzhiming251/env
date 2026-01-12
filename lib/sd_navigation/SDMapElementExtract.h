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
 * @file SDMapElementExtract.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-09
 */

#ifndef SD_MAP_ELEMENT_EXTRACT_H_
#define SD_MAP_ELEMENT_EXTRACT_H_

#include <cstddef>
#include <cstdint>
#include <optional>

#include <message/env_model/routing_map/routing_map.h>
#include "modules/msg/orin_msgs/routing_map.pb.h"

#include "lib/common/log_custom.h"
#include "lib/sd_navigation/routing_map_debug.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/common/math/math.h"

namespace cem::fusion::navigation {
#if defined(SD_EXTRACT_MAP_ELEMENT_LOG) && (SD_EXTRACT_MAP_ELEMENT_LOG)
#define SD_EXTRACT_MAP_ELEMENT AINFO
#else
#define SD_EXTRACT_MAP_ELEMENT NULL_LOG
#endif

#if defined(SD_JUNCTION_CONVERTE) && (SD_JUNCTION_CONVERTE)
#define SD_JUNCTION_CONVERTE_LOG AINFO
#else
#define SD_JUNCTION_CONVERTE_LOG NULL_LOG
#endif

using byd::common::math::LineSegment2d;
using byd::common::math::Vec2d;
using cem::message::env_model::angleDifference;
using cem::message::env_model::LaneInfo;
using cem::message::env_model::LaneType;
using cem::message::env_model::RoutingMap;
using cem::message::env_model::SDLaneGroupIndex;
using cem::message::env_model::SDLaneGroupInfo;
using cem::message::env_model::SDLinkTypeMask;
using cem::message::env_model::SDSectionInfo;
using cem::message::env_model::SDSubPath;

constexpr double Angle_180 = 180.0;
constexpr double Angle_90  = 90.0;
constexpr double Angle_75  = 75.0;
constexpr double Angle_110 = 110.0;
constexpr double Angle_179 = 179.0;
constexpr double Angle_35  = 35.0;
constexpr double Angle_140 = 140.0;
constexpr double Angle_25  = 25.0;
constexpr double Angle_15  = 15.0;
constexpr double Angle_5   = 5.0;
constexpr double Angle_45  = 45.0;

struct SdSubPathInfo {
  std::shared_ptr<const SDSubPath>                  sub_path_{nullptr};
  std::vector<std::shared_ptr<const SDSectionInfo>> sd_sction_infos_;
};
inline std::optional<double> LD_AngleDiffBetweenSection(const cem::message::env_model::SectionInfo &lhs,
                                                        const cem::message::env_model::SectionInfo &rhs, bool connected = true,
                                                        bool normal_direction = true) {
  const auto &point_ptr_current = lhs.points;
  const auto &point_ptr_next    = rhs.points;
  if (point_ptr_current.size() < 2 || point_ptr_next.size() < 2) {
    return std::nullopt;
  }
  std::size_t size_lhs = lhs.points.size();
  std::size_t size_rhs = rhs.points.size();

  // Vec2d seg1_start_0{point_ptr_current->at(0).x(), point_ptr_current->at(0).y()};
  // Vec2d seg1_start_1{point_ptr_current->at(1).x(), point_ptr_current->at(1).y()};
  Vec2d seg1_end_0{point_ptr_current.at(size_lhs - 2).x, point_ptr_current.at(size_lhs - 2).y};
  Vec2d seg1_end_1{point_ptr_current.at(size_lhs - 1).x, point_ptr_current.at(size_lhs - 1).y};
  Vec2d seg1_start_0{point_ptr_current.at(0).x, point_ptr_current.at(0).y};
  Vec2d seg1_start_1{point_ptr_current.at(1).x, point_ptr_current.at(1).y};

  Vec2d seg2_start_0{point_ptr_next.at(0).x, point_ptr_next.at(0).y};
  Vec2d seg2_start_1{point_ptr_next.at(1).x, point_ptr_next.at(1).y};
  Vec2d seg2_end_0{point_ptr_next.at(size_rhs - 2).x, point_ptr_next.at(size_rhs - 2).y};
  Vec2d seg2_end_1{point_ptr_next.at(size_rhs - 1).x, point_ptr_next.at(size_rhs - 1).y};

  auto  seg1_it_back       = point_ptr_current.rbegin();
  auto  seg1_it_back_prev  = std::next(seg1_it_back);
  auto  seg2_it_begin      = point_ptr_next.begin();
  auto  seg2_it_begin_next = std::next(seg2_it_begin);
  Vec2d seg1_start{seg1_it_back_prev->x, seg1_it_back_prev->y};
  Vec2d seg1_end{seg1_it_back->x, seg1_it_back->y};
  Vec2d seg2_start{seg2_it_begin->x, seg2_it_begin->y};
  Vec2d seg2_end{seg2_it_begin_next->x, seg2_it_begin_next->y};

  LineSegment2d line_segment_current{seg1_end_0, seg1_end_1};
  if (!normal_direction) {
    LineSegment2d line_segment_current{seg1_start_0, seg1_start_1};
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }
  if (!connected) {
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }

  // double dis_00 = seg1_start_0.DistanceTo(seg2_start_0);
  // double dis_01 = seg1_start_0.DistanceTo(seg2_end_1);
  double dis_10 = seg1_end_1.DistanceTo(seg2_start_0);
  double dis_11 = seg1_end_1.DistanceTo(seg2_end_1);
  if (dis_10 < dis_11) {
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }

  // SD_EXTRACT_MAP_ELEMENT << fmt::format(
  //     "seg1_id:{}  {{[{:.2f},{:.2f}],[{:.2f},{:.2f}]}}  seg2_id:{}  {{[{:.2f},{:.2f}],[{:.2f},{:.2f}]}}  dis_end_start:{:.2f}  "
  //     "distance_end_end:{:.2f} connected:{:d}  normal_direction:{:d}",
  //     lhs.id, seg1_start.x(), seg1_start.y(), seg1_end.x(), seg1_end.y(), rhs.id, seg2_start.x(), seg2_start.y(), seg2_end.x(),
  //     seg2_end.y(), dis_end_start, dis_end_end, connected, normal_direction);
  LineSegment2d line_segment_next{seg2_end_0, seg2_end_1};
  return angleDifference(line_segment_current.heading(), line_segment_next.heading());
}

/**
 * @brief 计算2个section之间的角度
 * @param lhs 
 * @param rhs 
 * @return std::optional<double> 2个section之间的角度，正值表示rhs在lhs的左边，负值表示rhs在lhs的右边
 * 
 * @author lingpeng (ling.peng3@byd.com)
 * @date 2025-04-10
 */
inline std::optional<double> AngleDiffBetweenSection(const SDSectionInfo &lhs, const SDSectionInfo &rhs, bool connected = true,
                                                     bool not_Calculate_Uturn_Angle = true) {
  const auto &point_ptr_current = lhs.points;
  const auto &point_ptr_next    = rhs.points;
  if (!point_ptr_current || point_ptr_current->size() < 2 || !point_ptr_next || point_ptr_next->size() < 2) {
    return std::nullopt;
  }
  std::size_t size_lhs = lhs.points->size();
  std::size_t size_rhs = rhs.points->size();

  // Vec2d seg1_start_0{point_ptr_current->at(0).x(), point_ptr_current->at(0).y()};
  // Vec2d seg1_start_1{point_ptr_current->at(1).x(), point_ptr_current->at(1).y()};
  Vec2d seg1_end_0{point_ptr_current->at(size_lhs - 2).x(), point_ptr_current->at(size_lhs - 2).y()};
  Vec2d seg1_end_1{point_ptr_current->at(size_lhs - 1).x(), point_ptr_current->at(size_lhs - 1).y()};
  Vec2d seg1_start_0{point_ptr_current->at(0).x(), point_ptr_current->at(0).y()};
  Vec2d seg1_start_1{point_ptr_current->at(1).x(), point_ptr_current->at(1).y()};

  Vec2d seg2_start_0{point_ptr_next->at(0).x(), point_ptr_next->at(0).y()};
  Vec2d seg2_start_1{point_ptr_next->at(1).x(), point_ptr_next->at(1).y()};
  Vec2d seg2_end_0{point_ptr_next->at(size_rhs - 2).x(), point_ptr_next->at(size_rhs - 2).y()};
  Vec2d seg2_end_1{point_ptr_next->at(size_rhs - 1).x(), point_ptr_next->at(size_rhs - 1).y()};

  auto  seg1_it_back       = point_ptr_current->rbegin();
  auto  seg1_it_back_prev  = std::next(seg1_it_back);
  auto  seg2_it_begin      = point_ptr_next->begin();
  auto  seg2_it_begin_next = std::next(seg2_it_begin);
  Vec2d seg1_start{seg1_it_back_prev->x(), seg1_it_back_prev->y()};
  Vec2d seg1_end{seg1_it_back->x(), seg1_it_back->y()};
  Vec2d seg2_start{seg2_it_begin->x(), seg2_it_begin->y()};
  Vec2d seg2_end{seg2_it_begin_next->x(), seg2_it_begin_next->y()};
  // if (!normal_direction) {
  //   LineSegment2d line_segment_current_reverse{seg1_start_0, seg1_start_1};
  //   double        dis_00 = seg1_start_0.DistanceTo(seg2_start_0);
  //   double        dis_01 = seg1_start_0.DistanceTo(seg2_end_1);
  //   if (dis_00 < dis_01) {
  //     LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
  //     return angleDifference(line_segment_current_reverse.heading(), line_segment_next.heading());
  //   }
  //   LineSegment2d line_segment_next{seg2_end_0, seg2_end_1};
  //   return angleDifference(line_segment_current_reverse.heading(), line_segment_next.heading());
  // }

  if (!not_Calculate_Uturn_Angle) {
    LineSegment2d line_segment_current{seg1_start_0, seg1_start_1};
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }
  LineSegment2d line_segment_current{seg1_end_0, seg1_end_1};
  if (!connected) {
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }

  // double dis_00 = seg1_start_0.DistanceTo(seg2_start_0);
  // double dis_01 = seg1_start_0.DistanceTo(seg2_end_1);
  double dis_10 = seg1_end_1.DistanceTo(seg2_start_0);
  double dis_11 = seg1_end_1.DistanceTo(seg2_end_1);
  if (dis_10 < dis_11) {
    LineSegment2d line_segment_next{seg2_start_0, seg2_start_1};
    return angleDifference(line_segment_current.heading(), line_segment_next.heading());
  }

  // SD_EXTRACT_MAP_ELEMENT << fmt::format(
  //     "seg1_id:{}  {{[{:.2f},{:.2f}],[{:.2f},{:.2f}]}}  seg2_id:{}  {{[{:.2f},{:.2f}],[{:.2f},{:.2f}]}}  dis_end_start:{:.2f}  "
  //     "distance_end_end:{:.2f} connected:{:d}  normal_direction:{:d}",
  //     lhs.id, seg1_start.x(), seg1_start.y(), seg1_end.x(), seg1_end.y(), rhs.id, seg2_start.x(), seg2_start.y(), seg2_end.x(),
  //     seg2_end.y(), dis_end_start, dis_end_end, connected, normal_direction);
  LineSegment2d line_segment_next{seg2_end_0, seg2_end_1};
  return angleDifference(line_segment_current.heading(), line_segment_next.heading());
}

inline bool LinkTypeMaskType(uint32_t link_type, SDLinkTypeMask linktype_mask) {
  return (link_type & static_cast<uint32_t>(linktype_mask)) == static_cast<uint32_t>(linktype_mask);
}

inline bool IsInDegreeTarget(double degree, double degree_target, double normal_degree_threshold = Angle_25) {
  return std::fabs(degree - degree_target) < normal_degree_threshold;
}

class SDMapElementExtract {
 public:
  SDMapElementExtract() = default;

  ~SDMapElementExtract() = default;

  struct JunctionHelpInfo {
    uint32_t         idx_start_{0};
    uint32_t         idx_end_{0};
    JunctionInfoCity junction_city_;
    bool             valid_{false};
  };
  struct LDAngleJunction {
    std::shared_ptr<const cem::message::env_model::SectionInfo> section{nullptr};

    double angle{0.0};
    double raw_angle{0.0};
    enum class PositionDirection {
      LEFT    = 0,
      RIGHT   = 1,
      DEFAULT = 2,
    };
    bool next_suc{false};
    bool min_angle{false};

    PositionDirection         position{PositionDirection::DEFAULT};
    [[nodiscard]] std::string GetPositionInfo() const {
      if (position == PositionDirection::LEFT) {
        return "left";
      }
      if (position == PositionDirection::RIGHT) {
        return "right";
      }
      return "default";
    }
  };

  struct AngleJunction {
    const SDSectionInfo *section{nullptr};

    double angle{0.0};
    double raw_angle{0.0};
    enum class PositionDirection {
      LEFT    = 0,
      RIGHT   = 1,
      DEFAULT = 2,
    };
    bool next_suc{false};
    bool min_angle{false};

    PositionDirection         position{PositionDirection::DEFAULT};
    [[nodiscard]] std::string GetPositionInfo() const {
      if (position == PositionDirection::LEFT) {
        return "left";
      }
      if (position == PositionDirection::RIGHT) {
        return "right";
      }
      return "default";
    }
  };

  void Proc(const std::shared_ptr<RoutingMap> &routing_map);

  void SectionsProcess(const std::shared_ptr<RoutingMap> &routing_map);

  [[nodiscard]] const std::vector<JunctionInfoCity> &GetJunctionsInfo() const { return junctions_info_; }

  [[nodiscard]] bool SectionIsRightTurnDedicated(const SDSectionInfo &section_t);

  static void JunctionAngleSectionDirection(const SDSectionInfo &section, std::vector<AngleJunction> &res, bool normal_direction = true);
  static void JunctionAngleLDSectionDirection(const cem::message::env_model::SectionInfo &section, std::vector<LDAngleJunction> &res,
                                              bool normal_direction = true);

  [[nodiscard]] const std::map<double, RecommendLane> &GetSdRecommendLane() const { return map_recommend_lane_; }

 private:
  void Reset();

  void FillMapsInfo();

  void DebugSdRouteInfo();

  void SetSdRecommendLane();

  int GetSectionLaneNum(const std::vector<SDSectionInfo> &route_section, size_t idx, bool is_back = true) {
    const auto &lane_group = LaneGroups(route_section[idx], is_back);
    std::size_t target_num = lane_group.size();
    for (auto lane_info : lane_group) {
      if (lane_info.type == LaneType::LANE_NON_MOTOR || lane_info.type == LaneType::LANE_DIVERSION) {
        target_num--;
      }
    }
    target_num = target_num != 0 ? target_num : route_section[idx].lane_num;
    return static_cast<int>(target_num);
  }

  [[nodiscard]] std::optional<std::vector<LaneInfo>> FindValidLaneGroups(const std::vector<SDSectionInfo> &route_section, std::size_t idx,
                                                                         bool is_back = true);

  /**
 * @brief 
 * @param section 
 * @param method 0,cross,T,small. 1.succ 2.prev
 * @return std::vector<AngleJunction> 
 * 
 * @author lingpeng (ling.peng3@byd.com)
 * @date 2025-04-12
 */
  std::vector<AngleJunction> JunctionAnglesAroundJunction(const std::vector<SDSectionInfo> &route_section, size_t idx, int method = 0);

  static bool SeemsOneRoad(const SDSectionInfo &lhs, const SDSectionInfo &rhs);

  static bool Is2LineCross(const SDSectionInfo &section, const std::vector<SDMapElementExtract::AngleJunction> &angle_junctions);

  [[nodiscard]] bool IsJunctionCross(size_t idx_start, const std::vector<SDSectionInfo> &route_section);
  [[nodiscard]] bool IsUTurn(size_t idx_start, const std::vector<SDSectionInfo> &route_section);
  [[nodiscard]] bool IsJunctionT(size_t idx_start, const std::vector<SDSectionInfo> &route_section);
  [[nodiscard]] bool IsJunctionTSmall(size_t idx_start, const std::vector<SDSectionInfo> &route_section);
  [[nodiscard]] bool IsJunctionSplit(size_t idx, const std::vector<SDSectionInfo> &route_section);
  [[nodiscard]] bool IsJunctionMerge(size_t idx, const std::vector<SDSectionInfo> &route_section);

  [[nodiscard]] bool SectionIsUTurn(const SDSectionInfo &section_t);
  [[nodiscard]] bool SectionIsRightTurn(const SDSectionInfo &section_t);

  [[nodiscard]] JunctionAction SetJunctionAction(const std::vector<SDSectionInfo> &route_section);

  [[nodiscard]] std::vector<LaneInfo> LaneGroups(const SDSectionInfo &section, bool is_back = true);

  [[nodiscard]] std::vector<RoadMainType> JunctionsMainRamp(const std::vector<AngleJunction> &junctions_angle, const SDSectionInfo &rhs);

  static void FilterOneRoad(const SDSectionInfo &section, std::vector<AngleJunction> &res);

  void GetJunctionType(const std::vector<SDSectionInfo> &route_sections, double ego_offset, std::vector<JunctionInfoCity> &junction_infos);

  void MergeSections(const std::vector<SDSectionInfo> &route_sections, std::vector<JunctionInfoCity> &junction_infos);

  void FillJunctionInfo(const std::vector<SDSectionInfo> &route_sections, std::vector<JunctionInfoCity> &junction_infos);
  void FillJunctionInfo_Uturn(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);
  void FillJunctionInfo_CrossRoad(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);
  void FillJunctionInfo_T(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);
  void FillJunctionInfo_SmallT(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);
  void FillJunctionInfo_Split(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);
  void FillJunctionInfo_Merge(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info);

  void SetMainLaneIndex(JunctionInfoCity &junction_info);
  void CheckMainLaneIndex(double action_angle, JunctionInfoCity &junction_info);

  static bool IsSubpathBidirectional(const std::vector<AngleJunction> &section_angles_succ,
                                     const std::vector<SDSectionInfo> &route_sections, size_t idx);

  static bool TempFilterSplit_01(const std::vector<AngleJunction> &succ_junction_angles);
  static bool TempFilterSplit_02(const std::vector<AngleJunction> &succ_junction_angles);
  static bool TempFilterSplit_03(const std::vector<AngleJunction> &succ_junction_angles);

  // bool CheckIfUsedBefore();

  static void JudgeIsNeedMatchArrow(std::vector<JunctionInfoCity> &junctions_info);
  void        JudgeTjunctionIsOnlyUturn(std::vector<JunctionInfoCity> &junctions_info, const std::vector<SDSectionInfo> &sd_route_sections);

  std::shared_ptr<RoutingMap> routing_map_{nullptr};

  std::unordered_map<uint64_t, const SDLaneGroupInfo *> map_sd_lane_groups_info_;
  //std::unordered_map<uint64_t, std::shared_ptr<const LaneInfo>>                 map_sd_lanes_info_;
  std::unordered_map<uint64_t, const SDSectionInfo *> map_sd_sections_;
  //std::unordered_map<uint64_t, SdSubPathInfo>                                   map_sd_subpaths_;
  std::unordered_map<uint64_t, std::vector<SDMapElementExtract::AngleJunction>> map_section_angles_;

  // std::unordered_map<uint64_t, std::shared_ptr<const SDLaneGroupInfo>>          map_sd_lane_groups_info_;
  // //std::unordered_map<uint64_t, std::shared_ptr<const LaneInfo>>                 map_sd_lanes_info_;
  // std::unordered_map<uint64_t, std::shared_ptr<const SDSectionInfo>>            map_sd_sections_;
  // std::unordered_map<uint64_t, SdSubPathInfo>                                   map_sd_subpaths_;
  // std::unordered_map<uint64_t, std::vector<SDMapElementExtract::AngleJunction>> map_section_angles_;

  JunctionHelpInfo junction_help_;

  std::vector<JunctionInfoCity>   junctions_info_;
  std::map<double, RecommendLane> map_recommend_lane_;
};
}  // namespace cem::fusion::navigation

#endif
