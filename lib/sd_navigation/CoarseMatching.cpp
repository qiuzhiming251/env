#include "CoarseMatching.h"
#include <algorithm>
#include <cmath>
#include "base/params_manager/params_manager.h"
#include "lib/sd_navigation/routing_map_debug.h"

namespace cem {
namespace fusion {
namespace navigation {

// Type1  = 1,  // 非直行的 小T型路口
std::vector<uint64_t> CoarseMatching::CoarseMatchingType1(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected) {
  std::vector<uint64_t> guide_lanes = {};
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "road_selected is empty, returning empty guide_lanes.";
    return guide_lanes;
  }
  if (junction.junction_action == JunctionAction::TurnRight) {
    guide_lanes = {road_selected.back()};
  } else if (junction.junction_action == JunctionAction::TurnLeft) {
    guide_lanes = {road_selected.front()};
  } else {
    guide_lanes = road_selected;
  }
  return guide_lanes;
}

// Type2  = 2,  // 主辅路分叉 RoadSplit
std::vector<uint64_t> CoarseMatching::CoarseMatchingType2(const std::vector<JunctionInfoCity> &junctions_info_city,
                                                          const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                                          bool isSplitRoadSelectWorked, const BevRouteInfo &complete_section_info) {

  std::vector<uint64_t> guide_lanes = {};
  if (road_selected.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "road_selected is empty, returning empty guide_lanes.";
    return {};
  }
  SD_COARSE_MATCH_TYPE2_LOG << " [CoarseMatchingType2]road_selected: " << fmt::format(" [{}]  ", fmt::join(road_selected, ", "));
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2]isSplitRoadSelectWorked: {}", isSplitRoadSelectWorked);

  bool is_right_turn = (junction.split_merge_direction == DirectionSplitMerge::Right);
  int  mapLaneNum    = CoarseMatching::GetCurrentLaneNum();
  int  bevLaneNum    = bev_processor_.CountLanesPassedZero();
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] mapLaneNum: {}, bevLaneNum: {}", mapLaneNum, bevLaneNum);
  auto &navi_debug_infos                   = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_infos.bevLaneNum              = bevLaneNum;
  navi_debug_infos.mapLaneNum              = mapLaneNum;
  navi_debug_infos.isSplitRoadSelectWorked = isSplitRoadSelectWorked;
  // Find the matching current section based on road_selected lane_ids
  const BevSectionInfo *current_sec = nullptr;
  for (const auto &sec : complete_section_info.sections) {
    bool match = false;
    for (uint64_t lane_id : sec.lane_ids) {
      if (std::find(road_selected.begin(), road_selected.end(), lane_id) != road_selected.end()) {
        match = true;
        break;
      }
    }
    if (match) {
      current_sec = &sec;
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Found matching section ID: {} with lane_ids: [{}]", sec.id,
                                               fmt::join(sec.lane_ids, ", "));
      break;
    }
  }
  double       left_edge_dist     = -1.0;
  double       right_edge_dist    = -1.0;
  const double single_line_missed = 5.5;
  const double double_line_missed = 9.0;
  if (current_sec) {
    left_edge_dist  = current_sec->left_edge_distance;
    right_edge_dist = current_sec->right_edge_distance;
  } else {
    SD_COARSE_MATCH_TYPE2_LOG
        << "[CoarseMatchingType2] No matching section found in complete_section_info_, using default edge distances -1.0.";
  }
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] left_edge_dist: {:.2f}, right_edge_dist: {:.2f}", left_edge_dist,
                                           right_edge_dist);

  int right_already_exclusived = navi_debug_infos.target_section.size() - navi_debug_infos.sections_without_bus.size();

  if (IsRoadSplitMain2Ramp(junction)) {
    SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] RoadSplit_Ramp.";
    std::pair<int, int> lane_counts             = GetReachableLaneCountToRamp(junction);
    int                 last_lg_reachable_count = lane_counts.first;
    int                 ego_reachable_count     = lane_counts.second;
    int                 max_map_count           = GetMaxNormalLaneCountToJunction(junction);
    int                 recommended_lane_count  = 0;
    SD_COARSE_MATCH_TYPE2_LOG << "Max normal lane count from ego to target junction: " << max_map_count;

    if (junction.offset > 100.0) {
      recommended_lane_count = ego_reachable_count;
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Distance > 100m, using ego_reachable_count: " << ego_reachable_count;
    } else if (junction.offset <= 100.0 && last_lg_reachable_count < ego_reachable_count) {
      recommended_lane_count = last_lg_reachable_count;
      SD_COARSE_MATCH_TYPE2_LOG
          << "[CoarseMatchingType2] Distance <= 100m and last_lg_reachable_count < ego_reachable_count, using last_lg_reachable_count: "
          << last_lg_reachable_count;
    } else {
      recommended_lane_count = ego_reachable_count;
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Default case, using ego_reachable_count: " << ego_reachable_count;
    }
    navi_debug_infos.ReachableLaneCountToRamp = recommended_lane_count;

    if (isSplitRoadSelectWorked && road_selected.size() <= recommended_lane_count) {
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
          "[CoarseMatchingType2] Skipping RoadSplit_Ramp logic, because isSplitRoadSelectWorked: {} and bevsize: {} < "
          "recommended_lane_count {}",
          isSplitRoadSelectWorked, road_selected.size(), recommended_lane_count);
      guide_lanes = road_selected;
      return guide_lanes;
    }

    // Adjust for missed detection on the ramp side
    int    missed_lanes = 0;
    double check_dist   = (junction.split_merge_direction == DirectionSplitMerge::Left) ? left_edge_dist : right_edge_dist;
    if (check_dist > double_line_missed) {
      missed_lanes = 2;
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Detected 2 missed lanes on {} side.",
                                               (junction.split_merge_direction == DirectionSplitMerge::Left ? "left" : "right"));
    } else if (check_dist > single_line_missed) {
      missed_lanes = 1;
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Detected 1 missed lane on {} side.",
                                               (junction.split_merge_direction == DirectionSplitMerge::Left ? "left" : "right"));
    }
    if (recommended_lane_count >= 3 && missed_lanes == 0 && mapLaneNum > road_selected.size() && !isSplitRoadSelectWorked)
      missed_lanes = mapLaneNum - road_selected.size();
    recommended_lane_count = std::max(0, recommended_lane_count - missed_lanes);
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Adjusted recommended_lane_count: {}", recommended_lane_count);

    if (junction.split_merge_direction == DirectionSplitMerge::Left) {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] DirectionSplitMerge::Left.";
      if (recommended_lane_count == 1 && mapLaneNum != 0) {
        apply_gradual_strategy(junction, road_selected, mapLaneNum, is_right_turn, guide_lanes);
      } else if (recommended_lane_count > 1) {
        int lanes_to_select = std::min(recommended_lane_count, static_cast<int>(road_selected.size()));
        guide_lanes         = std::vector<uint64_t>(road_selected.begin(), road_selected.begin() + lanes_to_select);
      } else {
        guide_lanes = {road_selected.front()};  // 默认选择最左侧车道
        apply_gradual_strategy(junction, road_selected, mapLaneNum, is_right_turn, guide_lanes);
      }
    } else if (junction.split_merge_direction == DirectionSplitMerge::Right) {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2]DirectionSplitMerge::Right.";
      // if (right_already_exclusived != 0) {
      //   recommended_lane_count = std::max(0, recommended_lane_count - right_already_exclusived);
      //   SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
      //       "[CoarseMatchingType2] Because right_already_exclusived is:{}, Update recommended_lane_count to: {}", right_already_exclusived,
      //       recommended_lane_count);
      // }
      std::vector<uint64_t> filtered_road_selected = road_selected;
      if (road_selected.size() > max_map_count && !road_selected.empty()) {
        uint64_t rightmost_lane_id = road_selected.back();
        auto     lane_info         = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(rightmost_lane_id);
        if (lane_info && !lane_info->geos->empty()) {
          double lane_start_x = lane_info->geos->front().x();
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Rightmost lane {} start x: {:.2f}, max_map_count: {}",
                                                   rightmost_lane_id, lane_start_x, max_map_count);
          if (lane_start_x > 0.0) {
            // Remove the rightmost lane as it starts too far ahead (likely misdetection)
            filtered_road_selected.pop_back();
            SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Removed rightmost lane due to start x > 20.0";
          }
        }
      }

      if (recommended_lane_count == 1 && mapLaneNum != 0) {
        apply_gradual_strategy(junction, filtered_road_selected, mapLaneNum, is_right_turn, guide_lanes);
      } else if (recommended_lane_count > 1) {
        int lanes_to_select = std::min(recommended_lane_count, static_cast<int>(road_selected.size()));
        guide_lanes         = std::vector<uint64_t>(filtered_road_selected.end() - lanes_to_select, filtered_road_selected.end());
      } else {
        guide_lanes = {filtered_road_selected.back()};  // 默认选择最右侧车道
        apply_gradual_strategy(junction, filtered_road_selected, mapLaneNum, is_right_turn, guide_lanes);
      }
    } else if (junction.split_merge_direction == DirectionSplitMerge::Straight) {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2]DirectionSplitMerge::Straight.";
      if (junction.succ_road_class.size() == 3 && junction.split_direction_ids.size() >= 3) {
        if (junction.succ_road_class[0] == RoadMainType::RoadRamp) {
          uint64_t left_sec_id    = junction.split_direction_ids[0];
          int      left_exclusive = GetExclusiveLaneCountToRamp(junction, left_sec_id, bevLaneNum);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2]DirectionSplitMerge::Straight exclusive left {} lanes.",
                                                   left_exclusive);
          if (road_selected.size() > left_exclusive) {
            for (unsigned int i = left_exclusive; i < road_selected.size(); ++i) {
              guide_lanes.push_back(road_selected[i]);
              if (guide_lanes.size() >= recommended_lane_count) {
                break;
              }
            }
          }
        }
      }

    } else {
      guide_lanes = road_selected;
    }
  } else {
    SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] RoadSplit_Straight.";
    // 收紧排除法进入的条件
    if (junction.offset < -15.0) {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Junction offset is less than 0m, skipping exclusion logic.";
      guide_lanes = road_selected;
      return guide_lanes;
    }

    bool has_intermediate_merge = HasIntermediateRoadMerge(junctions_info_city, junction);
    // if (isSplitRoadSelectWorked == true && road_selected.size() >= junction.main_road_lane_nums) {
    //   SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] road_selected is not sperate rightly, set isSplitRoadSelectWorked to false.";
    //   isSplitRoadSelectWorked = false;
    // }
    //当前section车道数小于目标道路车道数才跳过排除法
    if ((isSplitRoadSelectWorked || has_intermediate_merge) &&
        (road_selected.size() <= junction.target_road_lane_nums))  // 如果有合适的中间 RoadMerge 路口，跳过排除法
    {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Skipping exclusion logic (RoadMerge or isSplitRoadSelectWorked).";
      guide_lanes = road_selected;
    } else {
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] perform lane exclusion logic.";
      int left_exclusive  = 0;
      int right_exclusive = 0;
      if (junction.succ_road_class.size() == 2 && junction.split_direction_ids.size() >= 2) {
        if (junction.split_merge_direction == DirectionSplitMerge::Left) {
          // 目标主在左, 分叉辅在右, exclusive到辅的在右
          uint64_t fork_sec_id                      = junction.split_direction_ids[1];
          right_exclusive                           = GetExclusiveLaneCountToRamp(junction, fork_sec_id, bevLaneNum);
          navi_debug_infos.ExclusiveLaneCountToRamp = right_exclusive;
        } else if (junction.split_merge_direction == DirectionSplitMerge::Right) {
          // 目标主在右, 分叉辅在左, exclusive到辅的在左
          uint64_t fork_sec_id                      = junction.split_direction_ids[0];
          left_exclusive                            = GetExclusiveLaneCountToRamp(junction, fork_sec_id, bevLaneNum);
          navi_debug_infos.ExclusiveLaneCountToRamp = left_exclusive;
        }
      } else if (junction.succ_road_class.size() == 3 && junction.split_direction_ids.size() >= 3) {
        if (junction.split_merge_direction == DirectionSplitMerge::Straight) {
          if (junction.succ_road_class[0] == RoadMainType::RoadRamp) {
            uint64_t left_sec_id                      = junction.split_direction_ids[0];
            left_exclusive                            = GetExclusiveLaneCountToRamp(junction, left_sec_id, bevLaneNum);
            navi_debug_infos.ExclusiveLaneCountToRamp = left_exclusive;
          }
          if (junction.succ_road_class[2] == RoadMainType::RoadRamp) {
            uint64_t right_sec_id                     = junction.split_direction_ids[2];
            right_exclusive                           = GetExclusiveLaneCountToRamp(junction, right_sec_id, bevLaneNum);
            navi_debug_infos.ExclusiveLaneCountToRamp = right_exclusive;
          }
        }
      } else {
        SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Invalid succ_road_class or split_direction_ids size, use all road_selected.";
        guide_lanes = road_selected;
        return guide_lanes;
      }

      // 判断左右侧路岩的情况
      bool leftEdgeDetected  = (left_edge_dist > 0.0);
      bool rightEdgeDetected = (right_edge_dist > 0.0);
      int  missed_left       = 0;
      int  missed_right      = 0;

      // 如果自车脚下的感知车道数多于脚下地图车道数->脚下跟脚下对比
      if ((bevLaneNum >= mapLaneNum) && (road_selected.size() >= junction.target_road_lane_nums)) {
        guide_lanes = SelectLanesBasedOnDirection(junction, road_selected);
      }
      // 如果感知车道数少于地图车道数，并且某一侧没有路岩
      else {
        int missingCount = mapLaneNum - bevLaneNum;

        // 如果两侧都没有路岩：跳过排除法，沿用之前target_road_lane_nums推荐逻辑
        if (!leftEdgeDetected && !rightEdgeDetected) {
          SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Both edges missing & bevLaneNum < mapLaneNum, skip exclusion logic.";
          left_exclusive  = 0;
          right_exclusive = 0;
        }
        // 如果右侧没有路岩：假定右侧丢失
        else if (leftEdgeDetected && !rightEdgeDetected) {
          missed_right    = std::max(0, missingCount);
          right_exclusive = std::max(0, right_exclusive - missed_right);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Right edge missing, Adjusted left_exclusive: {} (missed: {})",
                                                   right_exclusive, missed_right);
        }
        // 如果左侧没有路岩：假定左侧丢失
        else if (!leftEdgeDetected && rightEdgeDetected) {
          missed_left    = std::max(0, missingCount);
          left_exclusive = std::max(0, left_exclusive - missed_left);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Left edge missing, Adjusted left_exclusive: {} (missed: {})",
                                                   left_exclusive, missed_left);
        }
        // 如果两侧都有路岩，继续执行原排除法逻辑
        else {
          SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Both edges detected, continuing exclusion logic.";
          if (left_edge_dist > double_line_missed) {
            missed_left = 2;
            SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Detected 2 missed lanes on left side.";
          } else if (left_edge_dist > single_line_missed) {
            missed_left = 1;
            SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Detected 1 missed lane on left side.";
          }
          left_exclusive = std::max(0, left_exclusive - missed_left);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Adjusted left_exclusive: {} (missed: {})", left_exclusive,
                                                   missed_left);

          if (right_edge_dist > double_line_missed) {
            missed_right = 2;
            SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Detected 2 missed lanes on right side.";
          } else if (right_edge_dist > single_line_missed) {
            missed_right = 1;
            SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2] Detected 1 missed lane on right side.";
          }
          right_exclusive = std::max(0, right_exclusive - missed_right);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Adjusted right_exclusive: {} (missed: {})", right_exclusive,
                                                   missed_right);
        }
        if (right_already_exclusived != 0) {
          right_exclusive = std::max(0, right_exclusive - right_already_exclusived);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
              "[CoarseMatchingType2] Because right_already_exclusived is:{}, Update right_exclusive to: {}", right_already_exclusived,
              right_exclusive);
        }
        if (left_exclusive == 0 && right_exclusive == 0 && junction.offset < 150.0) {
          guide_lanes = SelectLanesBasedOnDirection(junction, road_selected);
        } else {
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Enter exclusive loggic, left_exclusive: {}, right_exclusive:{}.",
                                                   left_exclusive, right_exclusive);
          if (road_selected.size() > left_exclusive + right_exclusive) {
            for (unsigned int i = left_exclusive; i < road_selected.size() - right_exclusive; ++i) {
              guide_lanes.push_back(road_selected[i]);
            }
          } else {
            guide_lanes = road_selected;
          }
        }
      }
    }
  }
  return guide_lanes;
}

std::vector<uint64_t> CoarseMatching::SelectLanesBasedOnDirection(const JunctionInfoCity      &junction,
                                                                  const std::vector<uint64_t> &road_selected) {
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[CoarseMatchingType2] Enter targetnum loggic");
  int target_road_lane_nums = junction.target_road_lane_nums;
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("target_road_lane_nums: {}", target_road_lane_nums);
  if (target_road_lane_nums == 0) {
    target_road_lane_nums = road_selected.size();
  }
  if (junction.split_merge_direction == DirectionSplitMerge::Left) {
    SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2]DirectionSplitMerge::Left.";
    return SelectLanesByCount(road_selected, target_road_lane_nums, false);
  } else if (junction.split_merge_direction == DirectionSplitMerge::Right) {
    SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2]DirectionSplitMerge::Right.";
    return SelectLanesByCount(road_selected, target_road_lane_nums, true);
  } else if (junction.split_merge_direction == DirectionSplitMerge::Straight) {
    SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatchingType2]DirectionSplitMerge::Straight.";
    std::vector<uint64_t> guide_lanes;
    if (road_selected.size() > 2) {
      for (unsigned int i = 1; i < road_selected.size() - 1; i++) {
        guide_lanes.push_back(road_selected[i]);
      }
    } else {
      guide_lanes = road_selected;
    }
    return guide_lanes;
  }
  return road_selected;
}

// Type3  = 3,  // 路口 粗匹配
std::vector<uint64_t> CoarseMatching::CoarseMatchingType3(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected) {
  std::vector<uint64_t> guide_lanes = {};
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "road_selected is empty, returning empty guide_lanes.";
    return {};
  }

  SD_COARSE_MATCH_LOG << "Handling TJunction or CrossRoad";
  int         arrow_count       = junction.map_lane_arrows_plan.size();
  const auto &main_lane_indexs  = junction.main_lane_indexs;
  int         target_lane_count = main_lane_indexs.size();
  SD_COARSE_MATCH_LOG << "main_lane_indexs: " << fmt::format("{}", main_lane_indexs);
  int  LaneNum       = (arrow_count > 0) ? arrow_count : (CoarseMatching::GetCurrentLaneNum());
  bool is_right_turn = (junction.junction_action == JunctionAction::TurnRight);

  if (arrow_count == 0 && !main_lane_indexs.empty()) {
    AWARN << "[CoarseMatching] arrow_count is 0 but main_lane_indexs is not empty";
    guide_lanes = road_selected;
  } else {
    // 计算左侧连续左转车道数量
    int left_turn_count = 0;
    for (size_t i = 0; i < arrow_count; ++i) {
      TurnType turn_type = junction.map_lane_arrows_plan[i];
      if (turn_type == TurnType::LEFT_TURN || turn_type == TurnType::LEFT_TURN_AND_U_TURN || turn_type == TurnType::U_TURN) {
        left_turn_count++;
      } else {
        break;
      }
    }
    // 计算右侧连续右转车道数量
    int right_turn_count = 0;
    for (int i = arrow_count - 1; i >= 0; --i) {
      TurnType turn_type = junction.map_lane_arrows_plan[i];
      if (turn_type == TurnType::RIGHT_TURN || turn_type == TurnType::RIGHT_TURN_AND_U_TURN) {
        right_turn_count++;
      } else {
        break;
      }
    }
    SD_COARSE_MATCH_LOG << "[CoarseMatchingType3] left_turn_count: " << left_turn_count << ", right_turn_count: " << right_turn_count;
    // 直行时排除左侧连续左转车道和右侧连续右转车道,未判断箭头数与section的车道数的对比，待优化
    if (junction.junction_action == JunctionAction::GoStraight && road_selected.size() >= 3 &&
        junction.offset <= NavigationConfig::max_distance) {
      SD_COARSE_MATCH_LOG << "[CoarseMatchingType3] Management of Multi-Lane Straight Crossroads ";
#if 0
        bool exclude_left  = (arrow_count > 0 && (junction.map_lane_arrows_plan[0] == TurnType::LEFT_TURN ||
                                                 junction.map_lane_arrows_plan[0] == TurnType::LEFT_TURN_AND_U_TURN));
        bool exclude_right = (arrow_count > 0 && (junction.map_lane_arrows_plan.back() == TurnType::RIGHT_TURN ||
                                                  junction.map_lane_arrows_plan.back() == TurnType::RIGHT_TURN_AND_U_TURN));
        for (size_t i = 0; i < road_selected.size(); ++i) {
          if ((i == 0 && exclude_left) || (i == road_selected.size() - 1 && exclude_right))
            continue;
          guide_lanes.push_back(road_selected[i]);
        }
#else

      // 滤除感知车道中左侧的 left_turn_count 个车道和右侧的 right_turn_count 个车道
      for (size_t i = 0; i < road_selected.size(); ++i) {
        if (i < left_turn_count || i >= road_selected.size() - right_turn_count) {
          continue;
        }
        guide_lanes.push_back(road_selected[i]);
      }
#endif
      SD_COARSE_MATCH_LOG << "Excluding turn lanes for GoStraight: " << fmt::format("{}", guide_lanes);
    } else {
      /*处理车道数较少时的直行推荐,后续调整顺序*/
      SD_COARSE_MATCH_LOG << "[CoarseMatchingType3] Management of situations involving fewer than 3 lanes ";
      if (junction.junction_action == JunctionAction::GoStraight) {
        if (road_selected.size() >= 3) {  // 计算左侧连续左转车道数量
#if 0
            int left_turn_count = 0;
            for (size_t i = 0; i < arrow_count; ++i) {
              TurnType turn_type = junction.map_lane_arrows_plan[i];
              if (turn_type == TurnType::LEFT_TURN || turn_type == TurnType::LEFT_TURN_AND_U_TURN) {
                left_turn_count++;
              } else {
                break;
              }
            }
            // 计算右侧连续右转车道数量
            int right_turn_count = 0;
            for (int i = arrow_count - 1; i >= 0; --i) {
              TurnType turn_type = junction.map_lane_arrows_plan[i];
              if (turn_type == TurnType::RIGHT_TURN || turn_type == TurnType::RIGHT_TURN_AND_U_TURN) {
                right_turn_count++;
              } else {
                break;
              }
            }
            // 滤除感知车道中左侧的 left_turn_count 个车道和右侧的 right_turn_count 个车道
            for (size_t i = 0; i < road_selected.size(); ++i) {
              if (i < left_turn_count || i >= road_selected.size() - right_turn_count) {
                continue;
              }
              guide_lanes.push_back(road_selected[i]);
            }
#endif

          /*后续根据箭头进行调整*/
          guide_lanes = road_selected;
        } else {
          guide_lanes = road_selected;
        }

        /* code */
      } else {
        bool is_leftmost = !main_lane_indexs.empty() && *std::min_element(main_lane_indexs.begin(), main_lane_indexs.end()) == 0;
        bool is_rightmost =
            !main_lane_indexs.empty() && *std::max_element(main_lane_indexs.begin(), main_lane_indexs.end()) == arrow_count - 1;

        if ((arrow_count == 2 && target_lane_count == 1) || (arrow_count == 3 && target_lane_count == 1 && (is_leftmost || is_rightmost)) ||
            (arrow_count >= 2 && target_lane_count >= 1 && (is_leftmost || is_rightmost))) {
          // apply_gradual_strategy(junction.junction_action == JunctionAction::TurnRight);
          apply_gradual_strategy(junction, road_selected, LaneNum, is_right_turn, guide_lanes);
          //guide_lanes = SelectSingleSideLanes(road_selected, junction);
          if (guide_lanes.empty()) {
            // 若返回空，默认推荐所有车道
            SD_COARSE_MATCH_LOG << "[SelectSingleSideLanes] empty!";
            guide_lanes = road_selected;
          }
        } else {
          SD_COARSE_MATCH_LOG << "Conditions not met, recommending all lanes.";
          guide_lanes = road_selected;
        }
      }
    }
  }
  return guide_lanes;
}

// Type7  = 7,  // UTurn
std::vector<uint64_t> CoarseMatching::CoarseMatchingType7(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected) {
  std::vector<uint64_t> guide_lanes = {};
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "road_selected is empty, returning empty guide_lanes.";
    return {};
  }
  guide_lanes = {road_selected.front()};
  return guide_lanes;
}

std::vector<uint64_t> CoarseMatching::SelectTargetLanes(const std::vector<uint64_t> &road_selected,
                                                        const std::vector<int>      &main_lane_indexs) {
  SD_COARSE_MATCH_LOG << "Entering SelectTargetLanes with road_selected size: " << road_selected.size()
                      << ", main_lane_indexs size: " << main_lane_indexs.size();
  std::vector<uint64_t> selected;
  for (int idx : main_lane_indexs) {
    if (idx >= 0 && idx < road_selected.size()) {
      selected.push_back(road_selected[idx]);
    }
  }
  SD_COARSE_MATCH_LOG << "Selected target lanes: " << fmt::format("{}", selected);
  return selected;
}

int CoarseMatching::GetCurrentLaneNum() {
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    AWARN << "[CoarseMatching] sd_route is null";
    SD_COARSE_MATCH_TYPE2_LOG << "sd_route is null";
    return 0;
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "[CoarseMatching] mpp_sections is empty";
    SD_COARSE_MATCH_TYPE2_LOG << "mpp_sections is empty";
    return 0;
  }

  uint64_t current_section_id = sd_route->navi_start.section_id;

  const SDSectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[CoarseMatching] current section not found";
    SD_COARSE_MATCH_TYPE2_LOG << "current section not found";
    return 0;
  }

  double current_s_offset = sd_route->navi_start.s_offset;
  SD_COARSE_MATCH_TYPE2_LOG << "current_section_id: " << current_section_id << ", current_s_offset: " << current_s_offset
                            << ", section_length: " << current_section->length;

  const double            kTol                   = 1.0;  // 1m 公差，按需调整
  const SDLaneGroupIndex *current_lane_group_idx = nullptr;
  for (const auto &lg_idx : current_section->lane_group_idx) {
    if (current_s_offset >= lg_idx.start_range_offset && current_s_offset < lg_idx.end_range_offset) {
      current_lane_group_idx = &lg_idx;
      break;
    }
  }

  if (!current_lane_group_idx) {
    const SDLaneGroupIndex *first_lg = nullptr;
    const SDLaneGroupIndex *last_lg  = nullptr;
    for (const auto &lg : current_section->lane_group_idx) {
      if (!first_lg || lg.start_range_offset < first_lg->start_range_offset)
        first_lg = &lg;
      if (!last_lg || lg.end_range_offset > last_lg->end_range_offset)
        last_lg = &lg;
    }

    if (current_s_offset <= (first_lg ? first_lg->start_range_offset - kTol : 0.0)) {
      current_lane_group_idx = first_lg;
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatching] s before first lanegroup, clamp to FIRST id=" << (first_lg ? first_lg->id : 0);
    } else if (current_s_offset >= (current_section->length - kTol) || (last_lg && current_s_offset >= last_lg->end_range_offset - kTol)) {
      current_lane_group_idx = last_lg;
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatching] s beyond section length, clamp to LAST lanegroup id=" << (last_lg ? last_lg->id : 0);
    } else {
      double                  best    = std::numeric_limits<double>::infinity();
      const SDLaneGroupIndex *nearest = nullptr;
      for (const auto &lg : current_section->lane_group_idx) {
        double d = 0.0;
        if (current_s_offset < lg.start_range_offset)
          d = lg.start_range_offset - current_s_offset;
        else
          d = current_s_offset - lg.end_range_offset;
        if (d < best) {
          best    = d;
          nearest = &lg;
        }
      }
      current_lane_group_idx = nearest;
      SD_COARSE_MATCH_TYPE2_LOG << "[CoarseMatching] s in gap, pick NEAREST lanegroup id=" << (nearest ? nearest->id : 0);
    }
  }

  if (!current_lane_group_idx) {
    AWARN << "[CoarseMatching] current lane group still not found (unexpected)";
    SD_COARSE_MATCH_TYPE2_LOG << "current lane group not found";
    return 0;
  }

  const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(current_lane_group_idx->id);
  if (!lane_group) {
    AWARN << "[CoarseMatching] lane group not found";
    SD_COARSE_MATCH_LOG << "lane group not found";
    return 0;
  }

  int lane_num = static_cast<int>(lane_group->lane_num);
  return lane_num;
}

std::vector<uint64_t> CoarseMatching::SelectSingleSideLanes(const std::vector<uint64_t> &road_selected, const JunctionInfoCity &junction) {
  SD_COARSE_MATCH_TYPE2_LOG << "Entering SelectSingleSideLanes with road_selected size: " << road_selected.size()
                            << ", junction_action: " << StrJunctionAction(junction.junction_action);
  if (road_selected.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "road_selected is empty";
    return {};
  }
  if (junction.junction_action == JunctionAction::TurnRight) {
    SD_COARSE_MATCH_TYPE2_LOG << "Selecting rightmost lane: " << road_selected.back();
    return {road_selected.back()};  // 最右侧车道
  } else if (junction.junction_action == JunctionAction::TurnLeft || junction.junction_action == JunctionAction::UTurn) {
    SD_COARSE_MATCH_TYPE2_LOG << "Selecting leftmost lane: " << road_selected.front();
    return {road_selected.front()};  // 最左侧车道
  } else if (junction.junction_action == JunctionAction::GoStraight) {
    SD_COARSE_MATCH_TYPE2_LOG << "[SelectSingleSideLanes]  junction action: " << static_cast<int>(junction.junction_action);
    //if (junction.succ_road_class.size() == 3 && junction.split_direction_ids.size() >= 3) {
    SD_COARSE_MATCH_TYPE2_LOG << "[SelectSingleSideLanes] Enter 1split3 logic, using DirectionSplitMerge for singleSideLanes.";
    if (junction.split_merge_direction == DirectionSplitMerge::Left) {
      return {road_selected.front()};
    } else if (junction.split_merge_direction == DirectionSplitMerge::Right) {
      return {road_selected.back()};
    } else if (junction.split_merge_direction == DirectionSplitMerge::Straight) {
      return road_selected;
    }
    //}
  } else {
    SD_COARSE_MATCH_TYPE2_LOG << "Unsupported junction action";
    return {};  // 返回空向量
  }
  return road_selected;
}

std::vector<uint64_t> CoarseMatching::SelectLanesByCount(const std::vector<uint64_t> &road_selected, int count, bool from_right) {
  SD_COARSE_MATCH_TYPE2_LOG << "Entering SelectLanesByCount with road_selected size: " << road_selected.size() << ", count: " << count
                            << ", from_right: " << from_right;
  if (road_selected.empty() || count <= 0) {
    SD_COARSE_MATCH_TYPE2_LOG << "road_selected is empty or count <= 0";
    return {};
  }

  std::vector<uint64_t> selected;
  if (from_right) {
    // 从右侧选择 count 个车道
    int start = std::max(0, static_cast<int>(road_selected.size()) - count);
    for (int i = start; i < road_selected.size(); ++i) {
      selected.push_back(road_selected[i]);
    }
    SD_COARSE_MATCH_TYPE2_LOG << "Selected lanes from right: " << fmt::format("{}", selected);
  } else {
    // 从左侧选择 count 个车道
    int end = std::min(static_cast<int>(road_selected.size()), count);
    for (int i = 0; i < end; ++i) {
      selected.push_back(road_selected[i]);
    }
    SD_COARSE_MATCH_TYPE2_LOG << "Selected lanes from left: " << fmt::format("{}", selected);
  }
  return selected;
}

// 检查 succ_road_class 是否为 {RoadMain, RoadRamp}
bool CoarseMatching::HasMainAndRamp(const std::vector<RoadMainType> &succ_road_class) {
  if (succ_road_class.size() == 2) {
    if (succ_road_class[0] == RoadMainType::RoadMain && succ_road_class[1] == RoadMainType::RoadRamp) {
      return true;
    }
  }
  return false;
}

//RoadSplit场景中 判断是否是主路到辅路场景
bool CoarseMatching::IsRoadSplitMain2Ramp(const JunctionInfoCity &junction) {

  if (junction.junction_type_city != JunctionTypeCity::RoadSplit) {
    return false;
  }

  if (junction.succ_road_class.size() == 2) {
    if ((junction.split_merge_direction == DirectionSplitMerge::Left && junction.succ_road_class.front() == RoadMainType::RoadMain) ||
        (junction.split_merge_direction == DirectionSplitMerge::Right && junction.succ_road_class.back() == RoadMainType::RoadMain)) {
      return false;
    } else {
      return true;
    }
  } else if (junction.succ_road_class.size() == 3) {
    if (junction.split_merge_direction == DirectionSplitMerge::Straight && junction.succ_road_class[1] == RoadMainType::RoadMain) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

void CoarseMatching::apply_gradual_strategy(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected_in, int LaneNum,
                                            bool is_right_turn, std::vector<uint64_t> &guide_lanes) {
  double offset = junction.offset;

  if (LaneNum == 1) {
    int bev_lane_count = road_selected_in.size();
    SD_COARSE_MATCH_TYPE2_LOG << "LaneNum == 1, BEV lane count: " << bev_lane_count;
    if (bev_lane_count > 1) {
      guide_lanes = SelectSingleSideLanes(road_selected_in, junction);
    } else {
      guide_lanes = road_selected_in;
    }
  } else if (LaneNum == 2) {
    SD_COARSE_MATCH_TYPE2_LOG << "LaneNum == 2, offset: " << junction.offset;
    if (offset > NavigationConfig::single_guide_lane_distance && offset <= NavigationConfig::max_distance) {
      guide_lanes = SelectLanesByCount(road_selected_in, 2, is_right_turn);
    } else if (offset <= NavigationConfig::single_guide_lane_distance) {
      guide_lanes = SelectSingleSideLanes(road_selected_in, junction);
    }
  } else if (LaneNum == 3) {
    SD_COARSE_MATCH_TYPE2_LOG << "LaneNum == 3, offset: " << offset;
    if (offset > NavigationConfig::single_guide_lane_distance && offset <= NavigationConfig::max_distance) {
      guide_lanes = SelectLanesByCount(road_selected_in, 2, is_right_turn);
    } else if (offset <= NavigationConfig::single_guide_lane_distance) {
      guide_lanes = SelectSingleSideLanes(road_selected_in, junction);
    }
  } else if (LaneNum >= 4) {
    SD_COARSE_MATCH_TYPE2_LOG << "LaneNum >= 4, offset: " << offset;
    if (offset > 600.0 && offset <= NavigationConfig::max_distance) {
      int half_lanes = static_cast<int>(std::ceil(static_cast<double>(LaneNum) / 2.0));
      guide_lanes    = SelectLanesByCount(road_selected_in, half_lanes, is_right_turn);
    } else if (offset >= NavigationConfig::single_guide_lane_distance && offset <= 600.0) {
      guide_lanes = SelectLanesByCount(road_selected_in, 2, is_right_turn);
    } else if (offset < NavigationConfig::single_guide_lane_distance) {
      guide_lanes = SelectSingleSideLanes(road_selected_in, junction);
    }
  } else {
    guide_lanes = SelectSingleSideLanes(road_selected_in, junction);
  }
}

int CoarseMatching::GetMaxNormalLaneCountToJunction(const JunctionInfoCity &junction) {
  uint64_t    ego_lg_id = topology_extractor_.GetCurrentLaneGroupId();
  const auto *ego_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(ego_lg_id);
  if (!ego_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find ego lane group: " << ego_lg_id;
    return 0;
  }

  if (junction.junction_ids.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "junction.junction_ids empty, cannot determine max lane count.";
    return 0;
  }

  uint64_t current_junction_id = junction.junction_ids.front();
  auto     target_section      = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(current_junction_id);
  if (!target_section) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find section for target junction id: " << current_junction_id;
    return 0;
  }

  std::queue<uint64_t>                 lg_queue;
  std::unordered_set<uint64_t>         visited_lgs;
  std::vector<const SDLaneGroupInfo *> path_lgs;

  lg_queue.push(ego_lg_id);
  visited_lgs.insert(ego_lg_id);
  path_lgs.push_back(ego_lg);

  uint64_t target_last_lg_id = 0;
  if (!target_section->lane_group_idx.empty()) {
    target_last_lg_id = target_section->lane_group_idx.back().id;
  }

  bool target_last_reached = false;
  while (!lg_queue.empty() && !target_last_reached) {
    uint64_t current_lg_id = lg_queue.front();
    lg_queue.pop();

    const SDLaneGroupInfo *current_lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(current_lg_id);
    if (!current_lg)
      continue;

    if (std::find(path_lgs.begin(), path_lgs.end(), current_lg) == path_lgs.end()) {
      path_lgs.push_back(current_lg);
    }

    if (current_lg_id == target_last_lg_id) {
      target_last_reached = true;
      break;
    }

    for (uint64_t succ_lg_id : current_lg->successor_lane_group_ids) {
      if (visited_lgs.count(succ_lg_id) == 0) {
        const auto *succ_lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(succ_lg_id);
        if (succ_lg) {
          lg_queue.push(succ_lg_id);
          visited_lgs.insert(succ_lg_id);
        }
      }
    }
  }

  if (!target_last_reached) {
    std::queue<uint64_t>         remaining_lg_queue;
    std::unordered_set<uint64_t> remaining_visited_lgs;
    for (const auto &lg : path_lgs) {
      remaining_lg_queue.push(lg->id);
      remaining_visited_lgs.insert(lg->id);
    }

    while (!remaining_lg_queue.empty()) {
      uint64_t current_lg_id = remaining_lg_queue.front();
      remaining_lg_queue.pop();

      const SDLaneGroupInfo *current_lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(current_lg_id);
      if (!current_lg)
        continue;

      for (uint64_t succ_lg_id : current_lg->successor_lane_group_ids) {
        if (remaining_visited_lgs.count(succ_lg_id) == 0) {
          const auto *succ_lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(succ_lg_id);
          if (succ_lg) {
            remaining_lg_queue.push(succ_lg_id);
            remaining_visited_lgs.insert(succ_lg_id);
            path_lgs.push_back(succ_lg);
          }
        }
      }
    }
  }

  int max_normal_lane_count = 0;
  for (const auto *lg : path_lgs) {
    int normal_lane_count = 0;
    for (const auto &lane : lg->lane_info) {
      const auto *sd_lane = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(lane.id);
      if (sd_lane && sd_lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
        SD_COARSE_MATCH_TYPE2_LOG << "Skip harbor-stop lane: " << lane.id << " in lane group: " << lg->id;
        continue;
      }
      normal_lane_count++;
    }

    if (normal_lane_count > max_normal_lane_count) {
      max_normal_lane_count = normal_lane_count;
      SD_COARSE_MATCH_TYPE2_LOG << "Found new max normal lane count: " << normal_lane_count << " in lane group: " << lg->id;
    }
  }

  return max_normal_lane_count;
}

std::pair<int, int> CoarseMatching::GetReachableLaneCountToRamp(const JunctionInfoCity &junction) {
  if (junction.junction_ids.size() < 2) {
    SD_COARSE_MATCH_TYPE2_LOG << "junction.junction_ids size < 2, cannot determine recommended lane count.";
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }

  uint64_t current_junction_id    = junction.junction_ids.front();
  uint64_t associated_junction_id = junction.junction_ids.back();
  auto    &navi_debug_infos       = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  auto current_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(current_junction_id);
  if (!current_section) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find section for current junction id: " << current_junction_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No junction id {}", current_junction_id);
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }

  auto associated_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(associated_junction_id);
  if (!associated_section) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find section for associated junction id: " << associated_junction_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No associated junction  {}", associated_junction_id);
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }

  uint64_t    ego_lg_id = topology_extractor_.GetCurrentLaneGroupId();
  const auto *ego_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(ego_lg_id);
  if (!ego_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find ego lane group: " << ego_lg_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No ego_lg  {}", ego_lg_id);
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }

  // 获取当前 section 的最后一个 lane_group
  if (current_section->lane_group_idx.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "Current section has no lane groups.";
    navi_debug_infos.lanegroup_failed = fmt::format("Current section {} no lane groups", current_junction_id);
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }
  uint64_t    last_lg_id = current_section->lane_group_idx.back().id;
  const auto *last_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(last_lg_id);
  if (!last_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find last lane group: " << last_lg_id;
    navi_debug_infos.lanegroup_failed = fmt::format("Current section {} last_lg_id {} not exit", current_junction_id, last_lg_id);
    return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
  }

  int section_lane_num = associated_section->lane_num;
  SD_COARSE_MATCH_TYPE2_LOG << "Target section ID: " << associated_section->id << " has lane_num: " << section_lane_num;

  const SDLaneGroupInfo *target_lg   = nullptr;
  bool                   is_valid_lg = false;
  if (section_lane_num > 0) {
    for (const auto &lg_idx : associated_section->lane_group_idx) {
      const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (lane_group && lane_group->lane_num <= section_lane_num) {  // 修改为 <= section_lane_num
        // 只检查该lane_group是否不在subpaths的任何section中
        bool in_subpath = false;
        for (const auto &subpath : INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->subpaths) {
          for (const auto &sub_sec : subpath.sections) {
            for (const auto &sub_lg_idx : sub_sec.lane_group_idx) {
              if (sub_lg_idx.id == lg_idx.id) {
                in_subpath = true;
                break;
              }
            }
            if (in_subpath)
              break;
          }
          if (in_subpath)
            break;
        }
        if (!in_subpath) {
          target_lg   = lane_group;
          is_valid_lg = true;
          SD_COARSE_MATCH_TYPE2_LOG << "Found valid lane group ID: " << lg_idx.id << " with lane_num: " << lane_group->lane_num
                                    << " (not in subpaths)";
          break;
        } else {
          SD_COARSE_MATCH_TYPE2_LOG << "Lane group ID: " << lg_idx.id << " appears in subpaths, skipping.";
        }
      }
    }
  }

  if (!target_lg) {
    if (associated_section->lane_group_idx.empty()) {
      SD_COARSE_MATCH_TYPE2_LOG << "Associated section has no lane groups.";
      return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
    }
    uint64_t first_lg_id = associated_section->lane_group_idx.front().id;
    target_lg            = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(first_lg_id);
    if (!target_lg) {
      SD_COARSE_MATCH_TYPE2_LOG << "Cannot find first lane group: " << first_lg_id;
      navi_debug_infos.lanegroup_failed = fmt::format("associated_section {} target_lg not exit", associated_section->id, first_lg_id);
      return std::make_pair(junction.target_road_lane_nums, junction.target_road_lane_nums);
    }
    // 检查第一个lane_group是否不在subpaths中
    bool in_subpath = false;
    for (const auto &subpath : INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->subpaths) {
      for (const auto &sub_sec : subpath.sections) {
        for (const auto &sub_lg_idx : sub_sec.lane_group_idx) {
          if (sub_lg_idx.id == first_lg_id) {
            in_subpath = true;
            break;
          }
        }
        if (in_subpath)
          break;
      }
      if (in_subpath)
        break;
    }
    if (!in_subpath && target_lg->lane_num <= section_lane_num) {
      is_valid_lg = true;
      SD_COARSE_MATCH_TYPE2_LOG << "Falling back to valid first lane group ID: " << first_lg_id << " with lane_num: " << target_lg->lane_num
                                << " (not in subpaths)";
    } else {
      SD_COARSE_MATCH_TYPE2_LOG << "First lane group ID: " << first_lg_id
                                << " does not meet criteria (in subpaths or lane_num > section_lane_num).";
      return std::make_pair(0, 0);
    }
  }

  // Build lane successor relationship map
  std::unordered_map<uint64_t, std::vector<uint64_t>> lane_successor_map;
  for (const auto &sec : INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->mpp_sections) {
    for (const auto &lg_idx : sec.lane_group_idx) {
      const auto *lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (lg) {
        for (const auto &lane : lg->lane_info) {
          lane_successor_map[lane.id] = lane.next_lane_ids;
        }
      }
    }
  }

  std::unordered_set<uint64_t> target_lg_lane_ids;
  for (const auto &lane : target_lg->lane_info) {
    target_lg_lane_ids.insert(lane.id);
  }

  // Use BFS to count lanes from last_lg that can reach target_lg
  std::unordered_set<uint64_t> reachable_lanes;
  for (const auto &start_lane : last_lg->lane_info) {
    const auto *sd_lane = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(start_lane.id);
    if (sd_lane && sd_lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
      SD_COARSE_MATCH_TYPE2_LOG << "Skip harbor-stop start lane: " << start_lane.id;
      continue;
    }

    std::queue<uint64_t>         q;
    std::unordered_set<uint64_t> visited;
    q.push(start_lane.id);
    visited.insert(start_lane.id);

    while (!q.empty()) {
      uint64_t current = q.front();
      q.pop();

      // If the current lane belongs to target_lg, mark the starting lane as reachable
      if (target_lg_lane_ids.count(current)) {
        reachable_lanes.insert(start_lane.id);
        break;
      }

      for (uint64_t next_id : lane_successor_map[current]) {
        if (visited.find(next_id) == visited.end()) {
          q.push(next_id);
          visited.insert(next_id);
        }
      }
    }
  }

  int last_lg_reachable_count = static_cast<int>(reachable_lanes.size());
  SD_COARSE_MATCH_TYPE2_LOG << "Reachable lane count to ramp from last LG: " << last_lg_reachable_count;

  constexpr double kMinLgLen = 15.0;
  // ego_lg 太短，则寻找第一个“长度>=阈值”的后继 lane group
  const SDLaneGroupInfo *start_lg    = ego_lg;
  uint64_t               start_lg_id = ego_lg_id;
  double                 ego_lg_len  = ego_lg->length;
  if (ego_lg_len < kMinLgLen) {
    SD_COARSE_MATCH_TYPE2_LOG << "ego_lg(" << ego_lg_id << ") length=" << ego_lg_len << " < " << kMinLgLen
                              << "m, search successor lane group...";
    std::queue<uint64_t>         q;
    std::unordered_set<uint64_t> vis;
    for (uint64_t succ_id : ego_lg->successor_lane_group_ids) {
      if (!vis.count(succ_id)) {
        vis.insert(succ_id);
        q.push(succ_id);
      }
    }

    bool found = false;
    while (!q.empty() && !found) {
      uint64_t cand_id = q.front();
      q.pop();
      const auto *cand_lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(cand_id);
      if (!cand_lg)
        continue;

      double L = cand_lg->length;
      SD_COARSE_MATCH_TYPE2_LOG << "Check successor LG " << cand_id << " len=" << L;
      if (L >= kMinLgLen) {
        start_lg    = cand_lg;
        start_lg_id = cand_id;
        found       = true;
        SD_COARSE_MATCH_TYPE2_LOG << "Use successor LG " << cand_id << " (len=" << L << "m) as start.";
        break;
      }

      for (uint64_t nx : cand_lg->successor_lane_group_ids) {
        if (!vis.count(nx)) {
          vis.insert(nx);
          q.push(nx);
        }
      }
    }

    if (!found) {
      SD_COARSE_MATCH_TYPE2_LOG << "No successor LG >= " << kMinLgLen << "m found; fallback to last_lg count=" << last_lg_reachable_count
                                << ".";
      return std::make_pair(last_lg_reachable_count, last_lg_reachable_count);  // 回退：用前面 last_lg→target_lg 的统计
    }
  }

  // Use BFS to count lanes from start_lg that can reach target_lg
  std::unordered_set<uint64_t> ego_reachable_lanes;
  for (const auto &start_lane : start_lg->lane_info) {
    const auto *sd_lane = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(start_lane.id);
    if (sd_lane && sd_lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
      SD_COARSE_MATCH_TYPE2_LOG << "Skip harbor-stop start lane: " << start_lane.id;
      continue;
    }
    std::queue<uint64_t>         q;
    std::unordered_set<uint64_t> visited;
    q.push(start_lane.id);
    visited.insert(start_lane.id);

    while (!q.empty()) {
      uint64_t current = q.front();
      q.pop();

      // If the current lane belongs to ego_lg, mark the starting lane as reachable
      if (target_lg_lane_ids.count(current)) {
        ego_reachable_lanes.insert(start_lane.id);
        break;
      }

      for (uint64_t next_id : lane_successor_map[current]) {
        if (visited.find(next_id) == visited.end()) {
          q.push(next_id);
          visited.insert(next_id);
        }
      }
    }
  }
  int ego_reachable_count = static_cast<int>(ego_reachable_lanes.size());
  SD_COARSE_MATCH_TYPE2_LOG << "Reachable lane count from start_lg(" << start_lg_id
                            << ") to ramp (skip harbor stop): " << ego_reachable_count;
  return std::make_pair(last_lg_reachable_count, ego_reachable_count);
}

int CoarseMatching::GetExclusiveLaneCountToRamp(const JunctionInfoCity &junction, uint64_t target_section_id, uint64_t bev_size) {
  if (junction.junction_ids.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "junction.junction_ids empty, cannot determine exclusive lane count.";
    return 0;
  }

  uint64_t current_junction_id = junction.junction_ids.front();

  auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  auto current_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(current_junction_id);
  if (!current_section) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find section for current junction id: " << current_junction_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No junction id {}", current_junction_id);
    return 0;
  }

  auto target_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(target_section_id);
  if (!target_section) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find section for target section id: " << target_section_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No target_section_id  {}", target_section_id);
    return 0;
  }

  uint64_t    ego_lg_id = topology_extractor_.GetCurrentLaneGroupId();
  const auto *ego_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(ego_lg_id);
  if (!ego_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find ego lane group: " << ego_lg_id;
    navi_debug_infos.lanegroup_failed = fmt::format("No ego_lg  {}", ego_lg_id);
    return 0;
  }

  // 获取当前 section 的最后一个 lane_group
  if (current_section->lane_group_idx.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "Current section has no lane groups.";
    navi_debug_infos.lanegroup_failed = fmt::format("Current section {} no lane groups", current_junction_id);
    return 0;
  }
  uint64_t    last_lg_id = current_section->lane_group_idx.back().id;
  const auto *last_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(last_lg_id);
  if (!last_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find last lane group: " << last_lg_id;
    navi_debug_infos.lanegroup_failed = fmt::format("Current section {} last_lg_id {} not exit", current_junction_id, last_lg_id);
    return 0;
  }

  int section_lane_num = target_section->lane_num;
  SD_COARSE_MATCH_TYPE2_LOG << "Target subpath section ID: " << target_section->id << " has lane_num: " << section_lane_num;

  const SDLaneGroupInfo *target_lg   = nullptr;
  bool                   is_valid_lg = false;
  if (section_lane_num > 0) {
    for (const auto &lg_idx : target_section->lane_group_idx) {
      const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (lane_group) {
        // 只检查该lane_group是否不在mppsection中
        bool in_mpppath = false;
        for (const auto &section : INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->mpp_sections) {
          for (const auto &mpp_lg_idx : section.lane_group_idx) {
            if (mpp_lg_idx.id == lg_idx.id) {
              in_mpppath = true;
              break;
            }
          }
          if (in_mpppath)
            break;
        }
        if (!in_mpppath) {
          target_lg   = lane_group;
          is_valid_lg = true;
          SD_COARSE_MATCH_TYPE2_LOG << "Found valid lane group ID: " << lg_idx.id << " with lane_num: " << lane_group->lane_num
                                    << " (not in mpppaths)";
          break;
        } else {
          SD_COARSE_MATCH_TYPE2_LOG << "Lane group ID: " << lg_idx.id << " appears in mpppaths, skipping.";
        }
      }
    }
  }

  if (!target_lg) {
    SD_COARSE_MATCH_TYPE2_LOG << "Cannot find valid target first lane group.";
    navi_debug_infos.lanegroup_failed = fmt::format("Targe section {} target_lg not exit", target_section->id);
    return 0;
  }

  std::unordered_set<uint64_t> target_lg_lane_ids;
  target_lg_lane_ids.reserve(target_lg->lane_info.size() * 2);
  for (const auto &ln : target_lg->lane_info)
    target_lg_lane_ids.insert(ln.id);
  // 记忆化：-1=未知, 0=非独占(存在脱离target的路径), 1=独占(所有路径都会进入target_lg)
  std::unordered_map<uint64_t, int8_t> memo;
  // 避免递归环；若遇到环，保守视为非独占
  std::unordered_set<uint64_t> onstack;

  constexpr int kMaxDepth = 1000;  // 防御性限深，避免异常数据

  std::function<int8_t(uint64_t, int)> onlyToTarget = [&](uint64_t u, int depth) -> int8_t {
    // 已算过
    auto it = memo.find(u);
    if (it != memo.end())
      return it->second;

    // 进入 target_lg：视为“成功吸收态”，不再展开后继
    if (target_lg_lane_ids.count(u))
      return memo[u] = 1;

    // 异常：环/过深/取不到 lane 信息 ⇒ 无法保证“只能到” ⇒ 非独占
    if (onstack.count(u) || depth > kMaxDepth)
      return memo[u] = 0;
    const auto *ln = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(u);
    if (!ln)
      return memo[u] = 0;

    // 叶子且不在 target_lg ⇒ 有一条路径终止于非 target ⇒ 非独占
    if (ln->next_lane_ids.empty())
      return memo[u] = 0;

    onstack.insert(u);
    // 所有后继都必须“onlyToTarget”，任一失败则失败
    for (uint64_t v : ln->next_lane_ids) {
      if (onlyToTarget(v, depth + 1) == 0) {
        onstack.erase(u);
        return memo[u] = 0;
      }
    }
    onstack.erase(u);
    return memo[u] = 1;
  };

  int exclusive_cnt = 0;
  for (const auto &start_ln : ego_lg->lane_info) {
    int8_t ok = onlyToTarget(start_ln.id, 0);
    SD_COARSE_MATCH_TYPE2_LOG << "[Ego Only→TargetLG] start_lane=" << start_ln.id << " -> " << int(ok);
    if (ok == 1)
      ++exclusive_cnt;
  }

  SD_COARSE_MATCH_TYPE2_LOG << "Exclusive lane count (ego_lg only-to-target_lg): " << exclusive_cnt;

  // uint64_t ego_lg_laneNum = ego_lg->lane_num;
  // uint64_t lane_num       = ego_lg_laneNum - exclusive_cnt;
  // uint64_t count          = bev_size - lane_num;

  // size_t exclusive_count = exclusive_cnt;

  // uint64_t map_lane_num        = ego_lg->lane_num;
  // uint64_t remaining_map_lanes = (map_lane_num > exclusive_count) ? (map_lane_num - exclusive_count) : 0;
  // SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Map lane group lane num: {}, Calculated remaining map lanes: {}", map_lane_num, remaining_map_lanes);

  // uint64_t adjusted_exclusive_count;
  // if (bev_size >= remaining_map_lanes) {
  //   adjusted_exclusive_count = bev_size - remaining_map_lanes;
  // } else {
  //   // If perception lanes are fewer than remaining map lanes, minimize exclusive count
  //   adjusted_exclusive_count = 0;
  //   SD_COARSE_MATCH_TYPE2_LOG << "Warning: Perception lane size (" << bev_size << ") < remaining map lanes (" << remaining_map_lanes
  //                       << "), setting adjusted exclusive count to 0";
  // }

  // SD_COARSE_MATCH_TYPE2_LOG << "Adjusted exclusive count based on perception size (" << bev_size << "): " << adjusted_exclusive_count;
  // count = adjusted_exclusive_count;

  return exclusive_cnt;
}

// Type3  = 3,  // 路口 粗匹配 新方案
std::vector<uint64_t> CoarseMatching::CoarseMatchingType33(const std::vector<JunctionInfoCity> &junctions_info_city,
                                                           const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                                           const BevRouteInfo &route_info) {
  std::vector<int>      selected_lanes      = {};
  std::vector<uint64_t> guide_lanes         = {};
  std::vector<uint64_t> guide_lanes_mapless = {};
  bool                  check_flag          = false;
  auto                 &navi_debug_info     = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  SD_COARSE_MATCH_LOG << " Start road_selected lanes: " << fmt::format(" [{}]  ", fmt::join(road_selected, ", "));

  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "road_selected is empty, returning empty guide_lanes.";
    return {};
  }
  guide_lanes = GetRecommendedLanes(junction, road_selected, route_info, check_flag);

  guide_lanes_mapless = CrossLaneRecommendation(junction, road_selected, route_info);

  SD_COARSE_MATCH_LOG << " Final recommended lanes: " << fmt::format(" [{}]  ", fmt::join(guide_lanes, ", "));
  SD_COARSE_MATCH_LOG << " Final recommended lane2: " << fmt::format(" [{}]  ", fmt::join(guide_lanes_mapless, ", "));
  navi_debug_info.guide_lanes_topo    = guide_lanes;
  navi_debug_info.guide_lanes_mapless = guide_lanes_mapless;

  /* 兜底策略 */
  if (guide_lanes.empty() || !check_flag) {
    if (!guide_lanes_mapless.empty()) {
      guide_lanes = guide_lanes_mapless;
    } else {
      guide_lanes = SelectSingleSideLanes(road_selected, junction);
    }
  }

  if (guide_lanes.empty()) {
    guide_lanes = road_selected;
  }

  // ============= 步骤1：查找下一个路口 =============
  const JunctionInfoCity *next_junction = nullptr;
  bool                    found_current = false;

  // 遍历路口列表定位当前路口位置
  for (size_t i = 0; i < junctions_info_city.size(); ++i) {
    if (junctions_info_city[i].junction_id == junction.junction_id) {
      found_current = true;
      // 检查是否存在下一个路口
      if ((i + 1) < junctions_info_city.size()) {
        next_junction = &junctions_info_city[i + 1];
      }
      break;
    }
  }

  // ============= 步骤2：距离和类型检查 =============
  if (found_current && next_junction != nullptr) {
    // 使用offset计算距离
    double distance = next_junction->offset - junction.offset;
    SD_COARSE_MATCH_LOG << "Distance to next junction: " << distance << "m";

    // 检查距离和路口类型条件
    if (distance > 0 && distance < 300.0 && IsInterestedJunction(*next_junction)) {
      SD_COARSE_MATCH_LOG << "Next junction detected: id=" << next_junction->junction_id
                          << ", action= " << StrJunctionAction(next_junction->junction_action);

      // ============= 步骤3：按转向规则调整车道 =============
      size_t         lane_count  = guide_lanes.size();
      JunctionAction curr_action = junction.junction_action;
      JunctionAction next_action = next_junction->junction_action;

      // 直行+转向组合
      if (curr_action == JunctionAction::GoStraight) {
        if (next_action == JunctionAction::TurnLeft) {
          size_t keep_count = (lane_count + 1) / 2;  // 向上取整
          guide_lanes       = std::vector<uint64_t>(guide_lanes.begin(), guide_lanes.begin() + keep_count);
        } else if (next_action == JunctionAction::TurnRight) {
          size_t keep_count = (lane_count + 1) / 2;
          guide_lanes       = std::vector<uint64_t>(guide_lanes.end() - keep_count, guide_lanes.end());
        }
      }
      // 转向+转向组合
      else if (curr_action == JunctionAction::TurnLeft || curr_action == JunctionAction::TurnRight) {
        if (next_action == JunctionAction::TurnLeft || next_action == JunctionAction::TurnRight) {
          size_t keep_count = lane_count;  //(lane_count + 1) / 2; /* 暂时不处理左右转的情况 */
          if (next_action == JunctionAction::TurnLeft) {
            guide_lanes = std::vector<uint64_t>(guide_lanes.begin(), guide_lanes.begin() + keep_count);
          } else {
            guide_lanes = std::vector<uint64_t>(guide_lanes.end() - keep_count, guide_lanes.end());
          }
        }
      }

      SD_COARSE_MATCH_LOG << "Adjusted lanes: " << fmt::format("[{}]", fmt::join(guide_lanes, ", "));
    }
  }

  return guide_lanes;
}
//从感知车道组中提取推荐车道的ID
std::vector<uint64_t> CoarseMatching::extractGuideLanes(const std::vector<int>      &selected_lanes,
                                                        const std::vector<uint64_t> &road_selected) {
  std::vector<uint64_t> guide_lanes;

  // 检查输入
  if (selected_lanes.empty() || road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[extractGuideLanes]: selected_lanes or road_selected is empty";
    return guide_lanes;
  }

  std::unordered_set<uint64_t> processed_lanes;
  std::vector<uint64_t>        overlap_lanes;

  for (const auto &lane_id : road_selected) {
    auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (!lane_info || lane_info->geos->empty()) {
      continue;
    }
    auto [lane_start_s, lane_end_s] = GetLaneStartEndS2(lane_id, processed_lanes);
    if (lane_start_s > lane_end_s) {
      std::swap(lane_start_s, lane_end_s);
    }
    double lane_length = lane_end_s - lane_start_s;

    // 记录满足条件的车道
    SD_COARSE_MATCH_LOG << "[extractGuideLanes] Lane " << lane_id << " lane_start_s " << lane_start_s << " lane_end_s " << lane_end_s
                        << " lane_length " << lane_length;
    if (lane_length > 15.0 && lane_start_s < 5.0) {
      overlap_lanes.push_back(lane_id);
    }
  }
  SD_COARSE_MATCH_LOG << "[extractGuideLanes]  overlap_lanes: " << fmt::format(" [{}]  ", fmt::join(overlap_lanes, ", "));

#if 0
  // 从overlap_lanes中提取selected_lanes索引对应的车道ID
    for (int index : selected_lanes) {
    // 检查索引是否有效
    if (index >= 0 && index < static_cast<int>(overlap_lanes.size())) {
      guide_lanes.push_back(overlap_lanes[index]);
      SD_COARSE_MATCH_LOG << "[extractGuideLanes]: Extracted guide lane: index=" << index << ", ID=" << overlap_lanes[index];
    } else {
      SD_COARSE_MATCH_LOG << "[extractGuideLanes]:  Invalid selected lane index " << index
                          << ", overlap_lanes size=" << overlap_lanes.size();
    }
  }
#else
  // 从road_selected中提取selected_lanes索引对应的车道ID
  for (int index : selected_lanes) {
    // 检查索引是否有效
    if (index >= 0 && index < static_cast<int>(road_selected.size())) {
      guide_lanes.push_back(road_selected[index]);
      SD_COARSE_MATCH_LOG << "[extractGuideLanes]: Extracted guide lane: index=" << index << ", ID=" << road_selected[index];
    } else {
      SD_COARSE_MATCH_LOG << "[extractGuideLanes]:  Invalid selected lane index " << index
                          << ", road_selected size=" << road_selected.size();
    }
  }
#endif

  SD_COARSE_MATCH_LOG << "[extractGuideLanes]: Extracted " << guide_lanes.size() << " guide lanes";
  return guide_lanes;
}

// 根据路口动作和转向类型获取推荐的路口车道索引
std::vector<int> CoarseMatching::getRecommendedJunctionLanes(const JunctionInfoCity &junction) {

  std::vector<int> recommended_junction_lanes;
  JunctionAction   action = junction.junction_action;

  // 获取当前车道数和路口前车道数
  int current_lane_count  = CoarseMatching::GetCurrentLaneNum();
  int junction_lane_count = junction.map_lane_arrows_plan.size();

  // 确定路口推荐的转向类型
  std::vector<TurnType> recommended_turn_types;
  switch (action) {
    case JunctionAction::TurnLeft:
      recommended_turn_types = {TurnType::LEFT_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN,
                                TurnType::LEFT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN,
                                TurnType::LEFT_TURN_AND_RIGHT_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN,
                                TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN};
      break;
    case JunctionAction::GoStraight:
      recommended_turn_types = {TurnType::NO_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN,
                                TurnType::STRAIGHT_AND_RIGHT_TURN,
                                TurnType::STRAIGHT_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN};
      break;
    case JunctionAction::TurnRight:
      recommended_turn_types = {TurnType::RIGHT_TURN,
                                TurnType::STRAIGHT_AND_RIGHT_TURN,
                                TurnType::RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN,
                                TurnType::LEFT_TURN_AND_RIGHT_TURN,
                                TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN};
      break;
    case JunctionAction::UTurn:
      recommended_turn_types = {TurnType::U_TURN,
                                TurnType::STRAIGHT_AND_U_TURN,
                                TurnType::LEFT_TURN_AND_U_TURN,
                                TurnType::RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN,
                                TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN};
      break;
    default:
      SD_COARSE_MATCH_LOG << "[GetRecommendedJunctionLanes]  Unsupported junction action: " << static_cast<int>(junction.junction_action);
      return {};
  }

  // 找出路口推荐的车道

  for (int j = 0; j < junction_lane_count; ++j) {
    TurnType turn_type = junction.map_lane_arrows_plan[j];
    if (std::find(recommended_turn_types.begin(), recommended_turn_types.end(), turn_type) != recommended_turn_types.end()) {
      recommended_junction_lanes.push_back(j);
      SD_COARSE_MATCH_LOG << "[GetRecommendedJunctionLanes]  Recommended junction lane: " << j << ", type=" << static_cast<int>(turn_type);
    }
  }

  if (recommended_junction_lanes.empty()) {
    SD_COARSE_MATCH_LOG << "[GetRecommendedJunctionLanes]  No recommended junction lanes found";
    return {};
  }

  SD_COARSE_MATCH_LOG << "[GetRecommendedJunctionLanes] Recommended junction lanes count: " << recommended_junction_lanes.size();
  return recommended_junction_lanes;
}
// 检查路口是否只有一个左转车道
bool CoarseMatching::hasSingleLeftTurnLane(const std::vector<TurnType> &junction_turn_types) {
  int left_turn_count = 0;

  for (const auto &turn_type : junction_turn_types) {
    if (turn_type == TurnType::LEFT_TURN || turn_type == TurnType::LEFT_TURN_AND_U_TURN || turn_type == TurnType::U_TURN) {
      left_turn_count++;
    }
  }

  return (left_turn_count == 1);
}
/**
 * 检查转向类型是否包含右转
 * 
 * @param type 
 * @return true or false 
 */
bool CoarseMatching::IsAdjacentToDedicatedRight(uint64_t current_junction_id) {
  // 遍历相邻路口信息数组
  for (const auto &junction_info : adjacent_junctions_info_) {
    // 检查是否是当前路口
    if (junction_info.junction_id == current_junction_id) {
      // 返回是否与右转专用道相邻
      return junction_info.is_adjacent_to_dedicated_right;
    }
  }

  // 如果没有找到匹配的路口信息，返回false
  return false;
}
/**
 * 检查转向类型是否包含右转
 * 
 * @param type 
 * @return true or false 
 */
bool CoarseMatching::containsRightTurn(TurnType type) {
  return type == TurnType::RIGHT_TURN || type == TurnType::STRAIGHT_AND_RIGHT_TURN || type == TurnType::RIGHT_TURN_AND_U_TURN ||
         type == TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN || type == TurnType::LEFT_TURN_AND_RIGHT_TURN ||
         type == TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN || type == TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN ||
         type == TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN;
}

/**
 * 检查路口是否无右转车道
 * 
 * @param junction_turn_types 
 * @return true or false 
 */
bool CoarseMatching::hasNoRightTurnLane(const std::vector<TurnType> &junction_turn_types) {
  for (const auto &turn_type : junction_turn_types) {
    if (containsRightTurn(turn_type)) {
      return false;  // 找到右转车道
    }
  }

  return true;  // 没有找到右转车道
}

/**
 * 计算两个区间的重叠率
 * 
 * @param type 
 * @return true or false 
 */
double CoarseMatching::calculateOverlapRatio(double c_start, double c_end, double j_start, double j_end) {
  double overlap_start  = std::max(c_start, j_start);
  double overlap_end    = std::min(c_end, j_end);
  double overlap_length = std::max(0.0, overlap_end - overlap_start);
  return overlap_length / (c_end - c_start);
}
/**
 * 检查路口箭头类型是否全部相同
 * 
 * @param junction 
 * @return true or false 
 */
bool CoarseMatching::allSameTurnType(const JunctionInfoCity &junction) {
  if (junction.map_lane_arrows_plan.empty()) {
    return true;
  }
  TurnType first_type = junction.map_lane_arrows_plan[0];
  for (size_t i = 1; i < junction.map_lane_arrows_plan.size(); ++i) {
    if (junction.map_lane_arrows_plan[i] != first_type) {
      return false;
    }
  }
  return true;
}
// 处理特殊场景的辅助函数
void CoarseMatching::processSpecialCase(int current_lane_count, int junction_lane_count, int first_lane_coverage,
                                        const std::vector<int> &recommended_junction_lanes, std::vector<TurnLaneOverlap> &overlaps) {
  SD_COARSE_MATCH_LOG << "[processSpecialCase] Processing special case with "
                      << "current_lane_count=" << current_lane_count << ", "
                      << "junction_lane_count=" << junction_lane_count << ", "
                      << "first_lane_coverage=" << first_lane_coverage;

  // 计算当前车道0的新范围，使其与路口前first_lane_coverage个车道完全重叠
  double current_lane_0_start = 0.0;
  double current_lane_0_end   = first_lane_coverage * (1.0 / junction_lane_count);

  SD_COARSE_MATCH_LOG << "[processSpecialCase] Adjusted current lane 0 range: "
                      << "[" << current_lane_0_start << ", " << current_lane_0_end << "]";

  // 处理当前车道0与路口车道的重叠
  for (int j : recommended_junction_lanes) {
    double j_start = static_cast<double>(j) / junction_lane_count;
    double j_end   = static_cast<double>(j + 1) / junction_lane_count;

    double overlap_ratio = calculateOverlapRatio(current_lane_0_start, current_lane_0_end, j_start, j_end);
    if (overlap_ratio > 0.0) {
      overlaps.push_back({0, j, overlap_ratio});
      SD_COARSE_MATCH_LOG << "[processSpecialCase] Current lane 0 overlaps with junction lane " << j << " by " << overlap_ratio * 100
                          << "%";
    }
  }

  // 处理当前车道1及以后的车道 - 平均分配剩余宽度
  double remaining_width = 1.0 - (current_lane_0_end - current_lane_0_start);
  double lane_width      = remaining_width / (current_lane_count - 1);

  for (int c = 1; c < current_lane_count; ++c) {
    double c_start = current_lane_0_end + (c - 1) * lane_width;
    double c_end   = current_lane_0_end + c * lane_width;

    SD_COARSE_MATCH_LOG << "[processSpecialCase] Current lane " << c << " range: "
                        << "[" << c_start << ", " << c_end << "], width: " << (c_end - c_start);

    for (int j : recommended_junction_lanes) {
      double j_start = static_cast<double>(j) / junction_lane_count;
      double j_end   = static_cast<double>(j + 1) / junction_lane_count;

      double overlap_ratio = calculateOverlapRatio(c_start, c_end, j_start, j_end);
      if (overlap_ratio > 0.0) {
        overlaps.push_back({c, j, overlap_ratio});
        SD_COARSE_MATCH_LOG << "[processSpecialCase] Current lane " << c << " overlaps with junction lane " << j << " by "
                            << overlap_ratio * 100 << "%";
      }
    }
  }
}

// 处理右转专用道场景的辅助函数
void CoarseMatching::processRightTurnLaneCase(int current_lane_count, int junction_lane_count,
                                              const std::vector<int> &recommended_junction_lanes, std::vector<TurnLaneOverlap> &overlaps) {
  SD_COARSE_MATCH_LOG << "[processRightTurnLaneCase] Processing right turn lane case with "
                      << "current_lane_count=" << current_lane_count << ", "
                      << "junction_lane_count=" << junction_lane_count;

  // 处理当前车道0到current_lane_count-2（除最右车道外的所有车道）
  for (int c = 0; c < current_lane_count - 1; ++c) {
    double c_start = static_cast<double>(c) / (current_lane_count - 1);
    double c_end   = static_cast<double>(c + 1) / (current_lane_count - 1);

    SD_COARSE_MATCH_LOG << "[processRightTurnLaneCase] Current lane " << c << " range: "
                        << "[" << c_start << ", " << c_end << "]";

    for (int j : recommended_junction_lanes) {
      double j_start = static_cast<double>(j) / junction_lane_count;
      double j_end   = static_cast<double>(j + 1) / junction_lane_count;

      double overlap_ratio = calculateOverlapRatio(c_start, c_end, j_start, j_end);
      if (overlap_ratio > 0.0) {
        overlaps.push_back({c, j, overlap_ratio});
        SD_COARSE_MATCH_LOG << "[processRightTurnLaneCase] Current lane " << c << " overlaps with junction lane " << j << " by "
                            << overlap_ratio * 100 << "%";
      }
    }
  }

  // 处理当前最右车道（current_lane_count-1），使其与所有路口车道的重叠率为0
  // 方法：将最右车道的宽度压缩到接近0，但保持在最右侧位置
  int    rightmost_lane       = current_lane_count - 1;
  double rightmost_lane_width = 0.0001;  // 非常小的宽度，几乎为0
  double rightmost_lane_start = 1.0 - rightmost_lane_width;
  double rightmost_lane_end   = 1.0;

  SD_COARSE_MATCH_LOG << "[processRightTurnLaneCase] Adjusted rightmost lane " << rightmost_lane << " range: [" << rightmost_lane_start
                      << ", " << rightmost_lane_end << "], width: " << rightmost_lane_width;

  // 由于最右车道宽度几乎为0，它与任何路口车道的重叠率都将接近于0
  // 为了确保，我们明确将其与所有路口车道的重叠率设为0
  for (int j : recommended_junction_lanes) {
    overlaps.push_back({rightmost_lane, j, 0.0});
    SD_COARSE_MATCH_LOG << "[processRightTurnLaneCase] Current rightmost lane " << rightmost_lane << " overlaps with junction lane " << j
                        << " by 0%";
  }
}
/**
 * 计算重叠率总和并筛选推荐车道
 * 
 * @param junction 
 * @return selected_lanes 
 */
std::vector<int> CoarseMatching::calculateAndSelectLanesByOverlap(int current_lane_count, int junction_lane_count,
                                                                  const std::vector<int>      &recommended_junction_lanes,
                                                                  const std::vector<TurnType> &junction_turn_types, bool b_right_turn_jct) {
  std::vector<TurnLaneOverlap> overlaps;
  std::vector<int>             selected_lanes;
  double                       overlap_threshold = 0.3;
  SD_COARSE_MATCH_LOG << "[CalculateAndSelectLanesByOverlap] Start calculating overlap ratios";
  // 检查特殊场景1：当前车道数等于路口车道数减一，且路口只有一个左转车道，且路口无右转车道
  bool is_special_case1 = (current_lane_count == junction_lane_count - 1) && (current_lane_count > 1) &&
                          hasSingleLeftTurnLane(junction_turn_types) && hasNoRightTurnLane(junction_turn_types);

  // 左侧车道新增场景：路口无右转车道，且当前车道数比路口车道数要少
  bool is_left_lane_add = (current_lane_count < junction_lane_count) && (current_lane_count > 1) &&
                          (hasNoRightTurnLane(junction_turn_types) || b_right_turn_jct);
  // left lane add
  // right-turn only lane
  // 检查特殊场景3：路口车道无右转车道，且当前车道数和路口车道数相等,特殊右转专用道，避开最右车道
  bool is_special_case3 = false;
  //(current_lane_count == junction_lane_count) && hasNoRightTurnLane(junction_turn_types);

  if (is_special_case1) {
    // 特殊处理1：将当前的第一车道宽度调整到和路口的第一、第二车道重叠率100%
    // 计算当前车道0的新范围，使其与路口车道0和1完全重叠
    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Special case 1 detected. ";
    double current_lane_0_start = 0.0;
    double current_lane_0_end   = 2.0 / junction_lane_count;  // 覆盖路口的前两个车道

    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Adjusted current lane 0 range: "
                        << "[" << current_lane_0_start << ", " << current_lane_0_end << "]";

    // 处理当前车道0与路口车道的重叠
    for (int j : recommended_junction_lanes) {
      double j_start = static_cast<double>(j) / junction_lane_count;
      double j_end   = static_cast<double>(j + 1) / junction_lane_count;

      double overlap_ratio = calculateOverlapRatio(current_lane_0_start, current_lane_0_end, j_start, j_end);
      if (overlap_ratio > 0.0) {
        overlaps.push_back({0, j, overlap_ratio});
        SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane 0 overlaps with junction lane " << j << " by "
                            << overlap_ratio * 100 << "%";
      }
    }

    // 处理当前车道1及以后的车道 - 平均分配剩余宽度
    double remaining_width = 1.0 - (current_lane_0_end - current_lane_0_start);
    double lane_width      = remaining_width / (current_lane_count - 1);

    for (int c = 1; c < current_lane_count; ++c) {
      double c_start = current_lane_0_end + (c - 1) * lane_width;
      double c_end   = current_lane_0_end + c * lane_width;

      SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane " << c << " range: "
                          << "[" << c_start << ", " << c_end << "], width: " << (c_end - c_start);

      for (int j : recommended_junction_lanes) {
        double j_start = static_cast<double>(j) / junction_lane_count;
        double j_end   = static_cast<double>(j + 1) / junction_lane_count;

        double overlap_ratio = calculateOverlapRatio(c_start, c_end, j_start, j_end);
        if (overlap_ratio > 0.0) {
          overlaps.push_back({c, j, overlap_ratio});
          SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane " << c << " overlaps with junction lane " << j << " by "
                              << overlap_ratio * 100 << "%";
        }
      }
    }
  } else if (is_left_lane_add) {
    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Special case 2 detected: "
                        << "current_lane_count=" << current_lane_count << ", "
                        << "junction_lane_count=" << junction_lane_count << ", "
                        << "no right turn lane";

    // 特殊处理2：让当前第一车道与路口的多出来的车道匹配
    // 计算当前第一车道需要覆盖的路口车道数量
    int extra_junction_lanes = junction_lane_count - current_lane_count;
    int first_lane_coverage  = extra_junction_lanes + 1;  // 当前第一车道需要覆盖的路口车道数

    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current first lane needs to cover " << first_lane_coverage
                        << " junction lanes";

    // 计算当前车道0的新范围，使其与路口前first_lane_coverage个车道完全重叠
    double current_lane_0_start = 0.0;
    double current_lane_0_end   = first_lane_coverage * (1.0 / junction_lane_count);

    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Adjusted current lane 0 range: "
                        << "[" << current_lane_0_start << ", " << current_lane_0_end << "]";

    // 处理当前车道0与路口车道的重叠
    for (int j : recommended_junction_lanes) {
      double j_start = static_cast<double>(j) / junction_lane_count;
      double j_end   = static_cast<double>(j + 1) / junction_lane_count;

      double overlap_ratio = calculateOverlapRatio(current_lane_0_start, current_lane_0_end, j_start, j_end);
      if (overlap_ratio > 0.0) {
        overlaps.push_back({0, j, overlap_ratio});
        SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane 0 overlaps with junction lane " << j << " by "
                            << overlap_ratio * 100 << "%";
      }
    }

    // 处理当前车道1及以后的车道 - 平均分配剩余宽度
    double remaining_width = 1.0 - (current_lane_0_end - current_lane_0_start);
    double lane_width      = remaining_width / (current_lane_count - 1);

    for (int c = 1; c < current_lane_count; ++c) {
      double c_start = current_lane_0_end + (c - 1) * lane_width;
      double c_end   = current_lane_0_end + c * lane_width;

      SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane " << c << " range: "
                          << "[" << c_start << ", " << c_end << "], width: " << (c_end - c_start);

      for (int j : recommended_junction_lanes) {
        double j_start = static_cast<double>(j) / junction_lane_count;
        double j_end   = static_cast<double>(j + 1) / junction_lane_count;

        double overlap_ratio = calculateOverlapRatio(c_start, c_end, j_start, j_end);
        if (overlap_ratio > 0.0) {
          overlaps.push_back({c, j, overlap_ratio});
          SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Current lane " << c << " overlaps with junction lane " << j << " by "
                              << overlap_ratio * 100 << "%";
        }
      }
    }
  } else if (is_special_case3) {
    SD_COARSE_MATCH_LOG << "[calculateAndSelectLanesByOverlap] Special case 3 detected: "
                        << "current_lane_count=" << current_lane_count << ", "
                        << "junction_lane_count=" << junction_lane_count << ", "
                        << "no right turn lane";

    // 调用辅助函数处理特殊场景3
    processRightTurnLaneCase(current_lane_count, junction_lane_count, recommended_junction_lanes, overlaps);
  } else {

    // 常规处理：计算每个当前车道与路口推荐车道的重叠率
    for (int c = 0; c < current_lane_count; ++c) {
      double c_start = static_cast<double>(c) / current_lane_count;
      double c_end   = static_cast<double>(c + 1) / current_lane_count;

      for (int j : recommended_junction_lanes) {
        double j_start = static_cast<double>(j) / junction_lane_count;
        double j_end   = static_cast<double>(j + 1) / junction_lane_count;

        double overlap_ratio = calculateOverlapRatio(c_start, c_end, j_start, j_end);
        if (overlap_ratio > 0.0) {
          overlaps.push_back({c, j, overlap_ratio});
          SD_COARSE_MATCH_LOG << "[CalculateAndSelectLanesByOverlap] Overlap: current lane " << c << " -> junction lane " << j
                              << ", ratio=" << overlap_ratio;
        }
      }
    }
  }

  SD_COARSE_MATCH_LOG << "[CalculateAndSelectLanesByOverlap] Start calculating total overlap for each current lane";
  // 计算重叠率总和并筛选推荐车道
  for (int c = 0; c < current_lane_count; ++c) {
    double total_overlap = 0.0;
    for (const auto &overlap : overlaps) {
      if (overlap.current_lane_index == c) {
        total_overlap += overlap.overlap_ratio;
      }
    }

    if (total_overlap >= overlap_threshold) {
      selected_lanes.push_back(c);
      SD_COARSE_MATCH_LOG << "[CalculateAndSelectLanesByOverlap] Selected current lane: " << c << ", total overlap=" << total_overlap;
    }
  }

  SD_COARSE_MATCH_LOG << "[CalculateAndSelectLanesByOverlap] Selected " << selected_lanes.size() << " lanes";
  return selected_lanes;
}

// 匹配路口SectionInfo
bool CoarseMatching::matchJunctionSection(const JunctionInfo &junction_info, const BevRouteInfo &route_info,
                                          const std::vector<uint64_t> &road_selected) {
  SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Start matching junction section";
  SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Junction offset: " << junction_info.offset;
  if (route_info.sections.empty() || road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[matchJunctionSection] Warning: route_info or road_selected is empty";
    return false;
  }
  bool     found_match        = false;
  uint64_t matched_section_id = 0;

  // 遍历所有SectionInfo，寻找匹配的路口section
  for (const auto &section : route_info.sections) {
    double distance = std::abs(section.end_range_offset - junction_info.offset);

    SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Checking section ID: " << section.id
                        << ", end_range_offset: " << section.end_range_offset << ", distance: " << distance;

    // 如果距离小于30米，认为可能匹配,检查车道id
    if (distance < 60.0) {
      // 遍历section中的每个车道ID
      for (uint64_t lane_id : section.lane_ids) {
        // 检查该车道ID是否在road_selected中
        if (std::find(road_selected.begin(), road_selected.end(), lane_id) != road_selected.end()) {
          found_match        = true;
          matched_section_id = section.id;
          SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Potential match found: section ID=" << section.id << ", distance=" << distance;
          break;
        }
      }
    }
  }

  // 如果找到匹配的section
  if (found_match) {
    // 如果与上一帧匹配的section相同，增加匹配帧数计数
    if (matched_section_id == g_junction_section_id) {
      g_matching_frames++;
      SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Continuing match, frame count: " << g_matching_frames;
    } else {
      // 如果是新的匹配section，重置计数
      g_junction_section_id = matched_section_id;
      g_matching_frames     = 1;
      SD_COARSE_MATCH_LOG << "[MatchJunctionSection] New match found, reset frame count to 1";
    }

    // 如果连续匹配5帧，认为匹配成功
    if (g_matching_frames >= 5) {
      SD_COARSE_MATCH_LOG << "[MatchJunctionSection] Match confirmed after " << g_matching_frames
                          << " frames, junction section ID: " << g_junction_section_id;
      return true;
    }
  } else {
    // 如果没有找到匹配的section，重置状态
    g_junction_section_id = 0;
    g_matching_frames     = 0;
    SD_COARSE_MATCH_LOG << "[MatchJunctionSection] No match found, resetting state";
  }

  return false;
}

/**
 * 车道变化确认
 * @param current_lane_count 当前车道数
 * @param junction_lane_count 路口车道数
 * @return  确认状态
 */
bool CoarseMatching::confirmLanesChange(int current_lane_count, int junction_lane_count) {
  if (!g_laneschange_status.confirmed) {
    if (g_laneschange_status.current_lane_count == -1 || g_laneschange_status.junction_lane_count == -1) {
      g_laneschange_status.current_lane_count  = current_lane_count;
      g_laneschange_status.junction_lane_count = junction_lane_count;
      g_laneschange_status.confirmation_frames = 1;
      SD_COARSE_MATCH_LOG << "[confirmLanesChange] Initializing confirmation, frames: 1";
    } else if (current_lane_count == g_laneschange_status.current_lane_count &&
               junction_lane_count == g_laneschange_status.junction_lane_count) {
      g_laneschange_status.confirmation_frames++;
      SD_COARSE_MATCH_LOG << "[confirmLanesChange] Lane counts match, frames: " << g_laneschange_status.confirmation_frames;
    } else {
      g_laneschange_status.current_lane_count  = current_lane_count;
      g_laneschange_status.junction_lane_count = junction_lane_count;
      g_laneschange_status.confirmation_frames = 1;
      SD_COARSE_MATCH_LOG << "[confirmLanesChange] Lane counts changed, reset frames: 1";
    }

    if (g_laneschange_status.confirmation_frames >= 5) {
      g_laneschange_status.confirmed = true;
      SD_COARSE_MATCH_LOG << "[confirmLanesChange] LANES CHANGE CONFIRMED after 5 frames";
    }
  } else {
    SD_COARSE_MATCH_LOG << "[confirmLanesChange]current current=" << current_lane_count << ", current junction=" << junction_lane_count
                        << " global current=" << g_laneschange_status.current_lane_count
                        << ", global junction=" << g_laneschange_status.junction_lane_count;
  }
  return g_laneschange_status.confirmed;
}

// 从路口section中提取推荐车道
std::vector<uint64_t> CoarseMatching::extractGuideLanesFromJunctionSection(const std::vector<int> &recommended_junction_lanes,
                                                                           const BevRouteInfo     &route_info) {
  std::vector<uint64_t> guide_lanes;

  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Extracting guide lanes from junction section";
  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Junction section ID: " << g_junction_section_id;

  // 查找路口section
  bool                  found_section    = false;
  const BevSectionInfo *junction_section = nullptr;

  for (const auto &section : route_info.sections) {
    if (section.id == g_junction_section_id) {
      found_section    = true;
      junction_section = &section;
      break;
    }
  }

  if (!found_section || junction_section == nullptr) {
    SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Error: Junction section not found";
    return guide_lanes;
  }

  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Junction section found, lane count: " << junction_section->lane_num;

  // 打印推荐的路口车道索引
  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Recommended junction lanes (0-based index): ";
  for (int lane : recommended_junction_lanes) {
    SD_COARSE_MATCH_LOG << lane << " ";
  }
  //SD_COARSE_MATCH_LOG << std::endl;

  // 打印路口section的车道ID
  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Junction section lane IDs: ";
  for (uint64_t id : junction_section->lane_ids) {
    SD_COARSE_MATCH_LOG << id << " ";
  }
  //SD_COARSE_MATCH_LOG << std::endl;

  // 从路口section中提取推荐车道ID
  for (int index : recommended_junction_lanes) {
    // 检查索引是否有效
    if (index >= 0 && index < static_cast<int>(junction_section->lane_ids.size())) {
      guide_lanes.push_back(junction_section->lane_ids[index]);
      SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Extracted guide lane: index=" << index
                          << ", ID=" << junction_section->lane_ids[index];
    } else {
      SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Warning: Invalid recommended junction lane index " << index
                          << ", junction section lane count=" << junction_section->lane_ids.size();
    }
  }

  SD_COARSE_MATCH_LOG << "[ExtractGuideLanesFromJunctionSection] Extracted " << guide_lanes.size() << " guide lanes";
  return guide_lanes;
}
/**
 * 重置车道变化状态
 * 
 * @param g_laneschange_status 车道变化状态全局变量
 * @return 
 */
void CoarseMatching::resetLanesChangeStatus() {
  SD_COARSE_MATCH_LOG << "[resetLanesChangeStatus] Resetting lanes change status to default values";

  g_laneschange_status.confirmed           = false;
  g_laneschange_status.current_lane_count  = -1;
  g_laneschange_status.junction_lane_count = -1;
  g_laneschange_status.confirmation_frames = 0;
  g_laneschange_status.recommended_junction_lanes.clear();
  g_laneschange_status.guide_lanes.clear();

  g_junction_section_id = 0;
  g_matching_frames     = 0;
}
/**
 * 十字路口自车位置的推荐车道计算
 * 
 * @param junction_info 路口信息结构体
 * @param road_selected
 * @param route_info
 * @return 符合条件的当前车道索引列表 
 */
std::vector<uint64_t> CoarseMatching::CrossLaneRecommendation(const JunctionInfoCity      &junction_info,
                                                              const std::vector<uint64_t> &road_selected, const BevRouteInfo &route_info) {
  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] ===== Start lane recommendation process =====";
  // 距离路口小于15米时重置状态
  if (junction_info.offset < 15.0) {
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] reset lanes change status";
    resetLanesChangeStatus();
  }
  // 判断路口有无右转专用道
  bool b_right_turn_jct = (IsAdjacentToDedicatedRight(junction_info.junction_id) || junction_info.is_dedicated_right_turn_lane);
  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] junction_info is right_turn_jct.";
  // 获取当前车道数
  int              current_lane_count  = CoarseMatching::GetCurrentLaneNum();
  int              junction_lane_count = junction_info.map_lane_arrows_plan.size();
  std::vector<int> selected_lanes;

  if (current_lane_count <= 0 || junction_lane_count <= 0) {
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Invalid lane counts: current=" << current_lane_count
                        << ", junction=" << junction_lane_count;
    if (g_laneschange_status.current_lane_count > 0 && g_laneschange_status.junction_lane_count > 0) {
      current_lane_count  = g_laneschange_status.current_lane_count;
      junction_lane_count = g_laneschange_status.junction_lane_count;
      SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] use pre frame lane counts: current=" << current_lane_count
                          << ", junction=" << junction_lane_count;
    } else {
      return {};
    }
  }

  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Current lanes: " << current_lane_count << ", Junction lanes: " << junction_lane_count
                      << ",  id: " << junction_info.junction_id << ",  offset: " << junction_info.offset
                      << ",  action: " << StrJunctionAction(junction_info.junction_action);

  // 获取路口的推荐车道索引
  std::vector<int> recommended_junction_lanes = getRecommendedJunctionLanes(junction_info);
  if (recommended_junction_lanes.empty()) {
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Error: No recommended junction lanes found, returning empty list";
    return {};
  }

  // 打印推荐的路口车道
  std::stringstream ss;
  ss << "Recommended junction lanes (0-based): ";
  for (int lane : recommended_junction_lanes) {
    ss << lane << " ";
  }
  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] " << ss.str();
  // 300-500米：确认车道变化
  if (junction_info.offset >= 300.0 && junction_info.offset <= 700.0) {
    confirmLanesChange(current_lane_count, junction_lane_count);
    if (g_laneschange_status.confirmed) {
      std::vector<int> selected_lanes;
      if (current_lane_count <= junction_lane_count) {
        SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Case 1: Current lane count <= junction lane count, using overlap ratio method";
        selected_lanes = calculateAndSelectLanesByOverlap(current_lane_count, junction_lane_count, recommended_junction_lanes,
                                                          junction_info.map_lane_arrows_plan, b_right_turn_jct);
      } else {
        /* 车道数多变少的情况待细化 */
        SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Case 2: Current lane count > junction lane count, using junction_lanes";
        selected_lanes = recommended_junction_lanes;
      }

      g_laneschange_status.guide_lanes = extractGuideLanes(selected_lanes, road_selected);
    } else {
      SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] NOT confirm yet,no guide lanes.";
      return {};
    }
  }
  // 150-300米：匹配路口section
  else if (junction_info.offset >= 150.0 && junction_info.offset < 300.0) {
    /* 先进行路口section的匹配 */
    bool match_status = matchJunctionSection(junction_info, route_info, road_selected);
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] junction and route_info match_status: " << match_status;
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] g_laneschange_status.confirmed: " << g_laneschange_status.confirmed;
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] g_laneschange_status.confirmation_frames: "
                        << g_laneschange_status.confirmation_frames;
    if (match_status) {
      g_laneschange_status.current_lane_count  = junction_lane_count;
      g_laneschange_status.junction_lane_count = junction_lane_count;
      g_laneschange_status.confirmed           = true;
    } else if (!g_laneschange_status.confirmed) {
      confirmLanesChange(current_lane_count, junction_lane_count);
    } else {
    }

    if (g_laneschange_status.confirmed) {
      if (match_status) {
        SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] extractGuideLanes From Junction Section.";
        g_laneschange_status.guide_lanes = extractGuideLanesFromJunctionSection(recommended_junction_lanes, route_info);
      } else {
        std::vector<int> selected_lanes;
        if (current_lane_count <= junction_lane_count) {
          SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Case 1: Current lane count <= junction lane count, using overlap ratio method";
          selected_lanes = calculateAndSelectLanesByOverlap(current_lane_count, junction_lane_count, recommended_junction_lanes,
                                                            junction_info.map_lane_arrows_plan, b_right_turn_jct);
        } else {
          /* 车道数多变少的情况待细化 */
          SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] Case 2: Current lane count > junction lane count, using junction_lanes";
          selected_lanes = recommended_junction_lanes;
        }
        g_laneschange_status.guide_lanes = extractGuideLanes(selected_lanes, road_selected);
      }
    }
  } /* 小于150m时 */
  else {
    SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] default case: Current lane count > junction lane count, using junction_lanes";
    g_laneschange_status.guide_lanes = extractGuideLanes(recommended_junction_lanes, road_selected);
  }

  // 最终推荐结果
  std::stringstream result_ss;
  result_ss << "Final selected lanes (0-based): ";
  for (int lane : g_laneschange_status.guide_lanes) {
    result_ss << lane << " ";
  }
  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] " << result_ss.str();
  SD_COARSE_MATCH_LOG << "[CrossLaneRecommendation] ===== End lane recommendation process =====";

  return g_laneschange_status.guide_lanes;
}

/**
 * 基于拓扑与感知匹配后的推荐车道
 * @param junction   路口
 * @param road_selected  选路结果
 * @param route_info  感知路段
 * @param check_flag  校验标志位
 * @return guide_lanes 
 */
std::vector<uint64_t> CoarseMatching::GetRecommendedLanes(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                                          const BevRouteInfo &route_info, bool &check_flag) {

  std::vector<uint64_t> guide_lanes     = {};
  auto                 &navi_debug_info = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  // 步骤1: 构建自车到路口的虚拟路段
  SD_COARSE_MATCH_TYPE2_LOG << "[GetRecommendedLanes] Extracting virtual sections for junction: " << junction.junction_id;
  std::vector<VirtualSection> virtual_sections = topology_extractor_.ExtractVirtualSections(junction);

  if (virtual_sections.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "[GetRecommendedLanes] No virtual sections extracted!";
    check_flag = false;
    return guide_lanes;
  }
  if (virtual_sections.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("No virtual sections found for junction ID: {}", junction.junction_id);
  } else {
    for (const auto &section : virtual_sections) {
      std::string indices_str = fmt::format("{}", fmt::join(section.target_lane_indices, ", "));
      std::string section_str = fmt::format(
          "VirtualSection ID: {}, Start S: {:.2f}, End S: {:.2f}, Lane Num: {}, Target Indices: [{}], "
          "Left Lane Change: {}, Right Lane Change: {}, Mid Lane Change: {}",
          section.id, section.start_s, section.end_s, section.lane_num, indices_str, section.left_lane_change, section.right_lane_change,
          section.mid_lane_change);
      SD_COARSE_MATCH_TYPE2_LOG << section_str;
      SD_COARSE_MATCH_LOG << section_str;
      /*debug 信息添加*/
      navi_debug_info.virtual_sections.push_back(section);
    }
  }

  // 步骤2: 筛选感知的主路径sections
  std::vector<BevSectionInfo> main_path_sections;
  for (const auto &section : route_info.sections) {
    for (const auto &lane_id : section.lane_ids) {
      if (std::find(road_selected.begin(), road_selected.end(), lane_id) != road_selected.end()) {
        main_path_sections.push_back(section);
        break;
      }
    }
  }

  SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Found " << main_path_sections.size() << " main path sections from "
                      << route_info.sections.size() << " total sections";

  // 步骤3: 计算每条车道线与每个虚拟路段的重叠率
  // 每个虚拟路段对应的重叠车道ID
  std::vector<std::vector<uint64_t>> vs_overlap_lanes(virtual_sections.size());
  std::unordered_set<uint64_t>       processed_lanes;

  for (size_t vs_idx = 0; vs_idx < virtual_sections.size(); vs_idx++) {
    const auto &vs = virtual_sections[vs_idx];
    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Processing virtual section " << vs_idx << " (ID: " << vs.id << ", start_s: " << vs.start_s
                        << ", end_s: " << vs.end_s << ", lane_num: " << vs.lane_num << ")";

    for (const auto &section : main_path_sections) {
      for (const auto &lane_id : section.lane_ids) {
        auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
        if (!lane_info || lane_info->geos->empty()) {
          continue;
        }

        // 计算车道线与虚拟路段的重叠长度
        //double lane_start_s = lane_info->geos->front().x();
        //double lane_end_s = lane_info->geos->back().x();
        // 计算车道线起终点，对多段线的情况进行连接
        auto [lane_start_s, lane_end_s] = GetLaneStartEndS(lane_id, processed_lanes);
        if (lane_start_s > lane_end_s) {
          std::swap(lane_start_s, lane_end_s);
        }

        double overlap_start  = std::max(lane_start_s, vs.start_s);
        double overlap_end    = std::min(lane_end_s, vs.end_s);
        double overlap_length = (overlap_start < overlap_end) ? (overlap_end - overlap_start) : 0.0;

        double lane_length         = lane_end_s - lane_start_s;
        double vs_length           = vs.end_s - vs.start_s;
        double overlap_ratio       = (lane_length > 0) ? (overlap_length / lane_length) : 0.0;
        double overlap_ratio_to_vs = (lane_length > 0) ? (overlap_length / vs_length) : 0.0;

        // 记录满足条件的车道
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Lane " << lane_id << " lane_start_s " << lane_start_s << " lane_end_s " << lane_end_s
                            << " overlaps with VS " << vs_idx << " (length: " << overlap_length << ", ratio: " << overlap_ratio
                            << ", overlap_ratio_to_vs: " << overlap_ratio_to_vs << ")";
        //if (overlap_ratio > 0.5 || overlap_ratio_to_vs > 0.5|| overlap_length > 15.0) {
        if (overlap_ratio > 0.5 || overlap_length > 15.0) {
          vs_overlap_lanes[vs_idx].push_back(lane_id);
        }
      }
    }

    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Virtual section " << vs_idx << " has " << vs_overlap_lanes[vs_idx].size()
                        << " overlapping lanes";
  }

#if 0
  // 步骤4: 从后向前遍历虚拟路段，进行校验
  bool validation_passed = false;
  int  selected_vs_idx   = -1;

  for (int i = virtual_sections.size() - 1; i >= 0; i--) {
    const auto &vs            = virtual_sections[i];
    const auto &overlap_lanes = vs_overlap_lanes[i];

    // 校验：感知车道数与虚拟路段车道数差异不超过2
    int lane_diff = static_cast<int>(overlap_lanes.size()) - static_cast<int>(vs.lane_num);
    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Validating VS " << i << ": overlap lanes = " << overlap_lanes.size()
                        << ", VS lane_num = " << vs.lane_num << ", lane_diff = " << lane_diff;

    if (overlap_lanes.size() > 0 && (std::abs(lane_diff) <= 1 || lane_diff >= 0) && vs.start_s < 90) {
      validation_passed = true;
      selected_vs_idx   = i;
      break;
    }
  }
#else
  // 步骤4: 使用优化的Cost值选择最优匹配的虚拟路段
  bool validation_passed = false;
  int selected_vs_idx = -1;
  double min_cost = DBL_MAX;
  const double COST_THRESHOLD = 6.5;  // Cost阈值

  // 权重系数
  const double WEIGHT_LANE_NUM = 1.0;   // 车道数匹配权重
  const double WEIGHT_START_DIS = 1.0;  // 起点距离匹配权重
  const double WEIGHT_LANE_START = 10;  // 车道起点位置匹配权重（更高优先级）

  for (int i = virtual_sections.size() - 1; i >= 0; i--) {
    const auto &vs = virtual_sections[i];
    const auto &overlap_lanes = vs_overlap_lanes[i];

    // 1. 车道数匹配度 (差异越大，匹配度越低)
    double lane_num_diff = std::abs(static_cast<int>(overlap_lanes.size()) - static_cast<int>(vs.lane_num));
    double lane_num_match = 1.0 / (1.0 + lane_num_diff);  // 匹配度范围 [0,1]

    // 2. 虚拟路段起点距离匹配度 (距离越远，匹配度越低)
    double vs_start_dis = vs.start_s;  // 虚拟路段起点到自车的距离
    double vs_start_match = 0.0;
    if (vs_start_dis < 0) {
      // 自车后方的虚拟路段，匹配度默认0.6（近距离特殊处理）
      vs_start_match = 0.6;
    } else if (vs_start_dis <= 20.0) {
      // 0~20m：从0.6缓慢升高到0.9
      vs_start_match = 0.6 + (vs_start_dis / 20.0) * 0.3;  // 线性递增
    } else if (vs_start_dis <= 40.0) {
      // 30~40m：保持高匹配度（0.9）
      vs_start_match = 0.9;
    } else if (vs_start_dis <= 120.0) {
      // 50~120m：从0.9逐渐降低到0
      const double range = 120.0 - 40.0;                             // 70m区间
      vs_start_match = 0.9 - ((vs_start_dis - 40.0) / range) * 0.9;  // 线性递减
    } else {
      // 120m以外：匹配度为0
      vs_start_match = 0.0;
    }
    // 确保匹配度在[0,1]范围内
    vs_start_match = std::max(0.0, std::min(1.0, vs_start_match));

    // 3. 感知车道起点位置匹配度
    double lane_start_match = 0.5;        // 默认基础匹配度
    bool has_valid_distant_lane = false;  // 是否有前方的车道
    uint64_t distant_lane_id = 0;
    double distant_lane_start = 0;
    bool has_split_topo = false;
    int vs_left_split_num = 0;
    if (i >= 1) {
      vs_left_split_num = virtual_sections[i - 1].left_lane_change;
    }

    // 检查是否存在起点在远处的车道
    for (const auto &lane_id : overlap_lanes) {

      auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
      if (lane_info && !lane_info->geos->empty()) {
#if 1
        auto [lane_start_s, lane_end_s] = GetLaneStartEndS2(lane_id, processed_lanes);
        double final_lane_start = lane_start_s;  // 车道起点距离
#else
        bool   has_pre_lane     = (lane_info->previous_lane_ids.size() > 0);
        double lane_start       = lane_info->geos->front().x();  // 车道起点距离
        double full_lane_start  = lane_info->geos->front().x();  // 车道起点距离
        double final_lane_start = lane_info->geos->front().x();  // 车道起点距离
        auto   full_geometry    = bev_processor_.GetFullLaneGeometry(lane_info);
        if (full_geometry && !full_geometry->empty()) {
          full_lane_start = full_geometry->front().x();
        }

        if (has_pre_lane) {
          if (lane_info->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT ||
              lane_info->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
            final_lane_start = lane_start;
          } else {
            final_lane_start = full_lane_start;
          }
        }
#endif

        //检查第一条线是否是新增车道
        if (lane_id == overlap_lanes[0]) {
          if (final_lane_start < 0 && vs_left_split_num > 0 && vs.start_s > 0) {
            SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] mismatch between bev left lane and map ";
            lane_start_match = 0.5;  //default value
          }
        }
        // SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] lane_id: "<<lane_id<<" lane_start:  " << lane_start <<" split_topo: "<< static_cast<int>(lane_info->split_topo_extend);
        if ((final_lane_start >= 10.0 && overlap_lanes.size() > 2 && lane_id != overlap_lanes[overlap_lanes.size() - 1]) ||
            (final_lane_start >= 10.0 && overlap_lanes.size() <= 2)) {  // 车道起点在40->25->10米外的线
          double start_diff = std::abs(final_lane_start - vs.start_s);
          if (start_diff <= 18.0) {  // 与VS起点差≤15m->18
            lane_start_match = 1.0;  // 高匹配度
            has_valid_distant_lane = true;
            distant_lane_id = lane_id;
            distant_lane_start = final_lane_start;
            break;
          }
        } else if (lane_info->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) {  //或者有split拓扑
          lane_start_match = 1.0;                                                               // 高匹配度
          has_valid_distant_lane = true;
          distant_lane_id = lane_id;
          distant_lane_start = final_lane_start;
          has_split_topo = true;
          break;
        } else if (HasCommonAncestorForThreeLanes(overlap_lanes)) {  //一分三场景,直接与最后一段vs匹配
          auto start = virtual_sections[virtual_sections.size() - 1].start_s;
          double start_diff = std::abs(final_lane_start - start);
          if (start_diff <= 20.0) {  // 与VS起点差≤15m
            lane_start_match = 1.0;  // 高匹配度
            has_valid_distant_lane = true;
            distant_lane_id = lane_id;
            distant_lane_start = final_lane_start;
            break;
          }
        }
      }
    }

    // 计算最终Cost (值越小越匹配)
    double cost =
        (1.0 - lane_num_match) * WEIGHT_LANE_NUM + (1.0 - vs_start_match) * WEIGHT_START_DIS + (1.0 - lane_start_match) * WEIGHT_LANE_START;

    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Validating VS " << i << ": overlap lanes = " << overlap_lanes.size()
                        << ", VS lane_num = " << vs.lane_num << ", cost = " << cost << ", factors = [" << lane_num_match << ", "
                        << vs_start_match << ", " << lane_start_match << "]"
                        << (has_valid_distant_lane ? "存在有效远距离车道： " : "无有效远距离车道 ") << distant_lane_id
                        << " start: " << distant_lane_start << " m. "
                        << "is_split: " << has_split_topo;

    // 选择Cost最小的虚拟路段
    if (cost < min_cost) {
      min_cost = cost;
      selected_vs_idx = i;
      validation_passed = (min_cost <= COST_THRESHOLD);
    }
  }
#endif

  // 步骤5: 根据校验结果提取推荐车道
  if (validation_passed) {
    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Validation passed for VS " << selected_vs_idx;
    check_flag = true;
  } else {
    // 如果所有校验都失败，使用第一个虚拟路段
    SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] All validations failed, using first virtual section";
    if (!virtual_sections.empty()) {
      selected_vs_idx = 0;
    }
    check_flag = false;
  }
  SD_COARSE_MATCH_LOG << "[GetRecommendedLanes]  road_selected: " << fmt::format(" [{}]  ", fmt::join(road_selected, ", "));
  if (selected_vs_idx >= 0 && selected_vs_idx < virtual_sections.size()) {
    const auto &vs            = virtual_sections[selected_vs_idx];
    const auto &overlap_lanes = vs_overlap_lanes[selected_vs_idx];
    // 提取推荐车道，使用选路结果，而非重叠率计算的结果,受感知识别能力影响较大
    for (uint64_t target_idx : vs.target_lane_indices) {
      // if (target_idx >= 0 && target_idx < overlap_lanes.size()) {
      //guide_lanes.push_back(overlap_lanes[target_idx]);
      if (target_idx >= 0 && target_idx < road_selected.size()) {
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes]  road_selected[target_idx]  " << road_selected[target_idx];
        guide_lanes.push_back(road_selected[target_idx]);
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Recommended lane " << road_selected[target_idx] << " from target index "
                            << target_idx;
      } else {
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Invalid target index " << target_idx << " for VS " << selected_vs_idx
                            << " (overlap lanes size: " << overlap_lanes.size() << ")";
      }
    }
  } else if (selected_vs_idx == 0 && selected_vs_idx < virtual_sections.size()) {
    const auto &vs            = virtual_sections[selected_vs_idx];
    const auto &overlap_lanes = vs_overlap_lanes[selected_vs_idx];
    // 提取推荐车道，当前段使用重叠率计算的结果,已使用
    for (uint64_t target_idx : vs.target_lane_indices) {
      if (target_idx >= 0 && target_idx < overlap_lanes.size()) {
        guide_lanes.push_back(overlap_lanes[target_idx]);
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes]  overlap_lanes[target_idx]  " << overlap_lanes[target_idx];
        //if (target_idx >= 0 && target_idx < road_selected.size()) {
        // SD_COARSE_MATCH_LOG << "[GetRecommendedLanes]  road_selected[target_idx]  " << road_selected[target_idx];
        // guide_lanes.push_back(road_selected[target_idx]);
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Recommended lane " << overlap_lanes[target_idx] << " from target index "
                            << target_idx;
      } else {
        SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Invalid target index " << target_idx << " for VS " << selected_vs_idx
                            << " (overlap lanes size: " << overlap_lanes.size() << ")";
      }
    }
  } else {
  }

  /*debug 信息添加*/
  navi_debug_info.selected_vs_idx = selected_vs_idx;
  SD_COARSE_MATCH_LOG << "[GetRecommendedLanes] Final recommended lanes: " << fmt::format(" [{}]  ", fmt::join(guide_lanes, ", "));

  return guide_lanes;
}

// 获取车道完整几何形状的起点和终点x坐标，避免重复计算相同前继车道
std::pair<double, double> CoarseMatching::GetLaneStartEndS(uint64_t lane_id, std::unordered_set<uint64_t> &processed_lanes) {

  //  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Processing lane ID: " << lane_id;
  bev_processor_.Clear();
#if 1
  // 处理当前车道的完整几何形状
  auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
  if (!lane_info) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Lane " << lane_id << " not found, returning {0.0, 0.0}";
    return {0.0, 0.0};
  }

  // SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Calculating full geometry for lane " << lane_id;
  auto full_geometry = bev_processor_.GetFullLaneGeometry(lane_info);

  if (full_geometry && !full_geometry->empty()) {
    double start_s = full_geometry->front().x();
    double end_s   = full_geometry->back().x();

    //  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Successfully got full geometry for lane " << lane_id << ": start_s=" << start_s
    //                      << ", end_s=" << end_s << ", point count=" << full_geometry->size();

    return {start_s, end_s};
  }
#else

  // 检查是否已处理过该车道
  if (processed_lanes.count(lane_id) > 0) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Lane " << lane_id << " already processed, checking predecessors";

    // 尝试从未处理的前继车道获取完整几何形状
    auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (lane_info && !lane_info->previous_lane_ids.empty()) {
      SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Lane " << lane_id << " has " << lane_info->previous_lane_ids.size()
                          << " predecessors, checking for unprocessed ones";

      for (uint64_t pre_id : lane_info->previous_lane_ids) {
        if (processed_lanes.count(pre_id) == 0) {
          SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Found unprocessed predecessor: " << pre_id;

          auto pre_lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(pre_id);
          if (!pre_lane_info) {
            SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Predecessor lane " << pre_id << " not found, skipping";
            continue;
          }

          auto full_geometry = bev_processor_.GetFullLaneGeometry(pre_lane_info);
          if (full_geometry && !full_geometry->empty()) {
            processed_lanes.insert(pre_id);
            double start_s = full_geometry->front().x();
            double end_s = full_geometry->back().x();

            SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Using predecessor " << pre_id << " full geometry: start_s=" << start_s
                                << ", end_s=" << end_s << ". processed_lanes state: [" << PrintProcessedLanes(processed_lanes) << "]";

            return {start_s, end_s};
          } else {
            SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Failed to get valid geometry for predecessor " << pre_id;
          }
        }
      }
    } else {
      SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Lane " << lane_id << " has no valid predecessors, using original geometry";
    }
  }

  // 处理当前车道的完整几何形状
  auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
  if (!lane_info) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Lane " << lane_id << " not found, returning {0.0, 0.0}";
    return {0.0, 0.0};
  }

  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Calculating full geometry for lane " << lane_id;
  auto full_geometry = bev_processor_.GetFullLaneGeometry(lane_info);

  if (full_geometry && !full_geometry->empty()) {
    processed_lanes.insert(lane_id);
    double start_s = full_geometry->front().x();
    double end_s = full_geometry->back().x();

    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Successfully got full geometry for lane " << lane_id << ": start_s=" << start_s
                        << ", end_s=" << end_s << ", point count=" << full_geometry->size() << ". processed_lanes state: ["
                        << PrintProcessedLanes(processed_lanes) << "]";

    return {start_s, end_s};
  }
#endif
  // 默认：使用原始几何点
  //SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Failed to get full geometry for lane " << lane_id << ", using original geometry points";

  if (!lane_info->geos || lane_info->geos->empty()) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Original geometry for lane " << lane_id << " is empty, returning {0.0, 0.0}";
    return {0.0, 0.0};
  }

  double start_s = lane_info->geos->front().x();
  double end_s   = lane_info->geos->back().x();

  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS] Using original geometry for lane " << lane_id << ": start_s=" << start_s << ", end_s=" << end_s
                      << ", point count=" << lane_info->geos->size();

  return {start_s, end_s};
}

std::pair<double, double> CoarseMatching::GetLaneStartEndS2(uint64_t lane_id, std::unordered_set<uint64_t> &processed_lanes) {
  // 步骤1：检查当前车道或其前继是否已被处理
  bool is_pre_lane_processced = IsLaneOrPredecessorsProcessed(lane_id, processed_lanes);
  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Lane or pre_lane is processced: " << is_pre_lane_processced;
  if (IsLaneOrPredecessorsProcessed(lane_id, processed_lanes)) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Lane " << lane_id << " or its predecessors are already processed, using original geometry";
    // 直接使用原始几何数据
    auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (!lane_info || !lane_info->geos || lane_info->geos->empty()) {
      SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Original geometry for lane " << lane_id << " is invalid, returning {0.0, 0.0}";
      return {0.0, 0.0};
    }
    double start_s = lane_info->geos->front().x();
    double end_s   = lane_info->geos->back().x();
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Using original geometry for lane " << lane_id << ": start_s=" << start_s
                        << ", end_s=" << end_s;
    return {start_s, end_s};
  }

  // 步骤2：若未被处理，获取前继车道
  auto current_ancestors = CollectPreLanes(lane_id);

  // 步骤3：检测是否存在共享前继（当前车道的前继链中存在已被处理的其他车道的前继）
  bool has_shared_ancestor = false;
  for (uint64_t ancestor : current_ancestors) {
    if (processed_lanes.count(ancestor) > 0) {
      has_shared_ancestor = true;
      break;
    }
  }
  if (has_shared_ancestor) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Lane " << lane_id
                        << " shares ancestors with processed lanes (分叉线), using original geometry";
    auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (!lane_info || !lane_info->geos || lane_info->geos->empty()) {
      return {0.0, 0.0};
    }
    return {lane_info->geos->front().x(), lane_info->geos->back().x()};
  }
  // 步骤4：尝试使用前继的完整几何（优先未被处理的直接前继）
  auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
  if (!lane_info) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Lane " << lane_id << " not found, returning {0.0, 0.0}";
    return {0.0, 0.0};
  }

  // 遍历直接前继（第1层），寻找可用的完整几何
  for (uint64_t pre_id : lane_info->previous_lane_ids) {
    if (processed_lanes.count(pre_id) == 0) {
      auto pre_lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(pre_id);
      if (!pre_lane_info)
        continue;
      auto full_geometry = bev_processor_.GetFullLaneGeometry(pre_lane_info);
      if (full_geometry && !full_geometry->empty()) {
        // 标记当前车道及其3层前继为已处理（避免分叉线重复计算）
        processed_lanes.insert(current_ancestors.begin(), current_ancestors.end());
        SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Using predecessor " << pre_id << " full geometry";
        return {full_geometry->front().x(), full_geometry->back().x()};
      }
    }
  }
  // 步骤5：前继无有效几何，尝试当前车道的完整几何
  auto full_geometry = bev_processor_.GetFullLaneGeometry(lane_info);
  if (full_geometry && !full_geometry->empty()) {
    // 标记当前车道及其3层前继为已处理
    processed_lanes.insert(current_ancestors.begin(), current_ancestors.end());
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Using current lane " << lane_id << " full geometry";
    return {full_geometry->front().x(), full_geometry->back().x()};
  }

  // 步骤6：完整几何获取失败，使用原始几何（兜底）
  if (!lane_info->geos || lane_info->geos->empty()) {
    SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Original geometry for lane " << lane_id << " is empty, returning {0.0, 0.0}";
    return {0.0, 0.0};
  }
  double start_s = lane_info->geos->front().x();
  double end_s   = lane_info->geos->back().x();
  SD_COARSE_MATCH_LOG << "[GetLaneStartEndS2] Using original geometry for lane " << lane_id << ": start_s=" << start_s
                      << ", end_s=" << end_s;
  return {start_s, end_s};
}

bool CoarseMatching::HasIntermediateRoadMerge(const std::vector<JunctionInfoCity> &junctions_info_city,
                                              const JunctionInfoCity              &current_junction) {
  for (const auto &adj_junction : junctions_info_city) {
    if (adj_junction.junction_type_city == JunctionTypeCity::RoadMerge && adj_junction.offset < current_junction.offset &&
        adj_junction.offset > 0 && adj_junction.junction_state_city != JunctionStateCity::PASSED) {
      SD_COARSE_MATCH_LOG << fmt::format("[HasIntermediateRoadMerge] Found RoadMerge id={} offset={}", adj_junction.junction_id,
                                         adj_junction.offset);
      return true;
    }
  }
  return false;
}

std::string CoarseMatching::PrintProcessedLanes(const std::unordered_set<uint64_t> &lanes) {
  std::stringstream ss;
  for (auto id : lanes) {
    if (ss.tellp() > 0) {
      ss << ", ";
    }
    ss << id;
  }
  return ss.str();
}

bool CoarseMatching::IsLaneOrPredecessorsProcessed(uint64_t lane_id, const std::unordered_set<uint64_t> &processed_lanes) {
  SD_COARSE_MATCH_LOG << "[IsLaneOrPredecessorsProcessed] lane " << lane_id << ". processed_lanes state: ["
                      << PrintProcessedLanes(processed_lanes) << "]";

  // 当前层待检查的车道集合（初始为当前车道，即第0层）
  std::unordered_set<uint64_t> current_lanes = {lane_id};
  // 最多检查3层前继
  for (int level = 0; level <= 5; ++level) {
    // 检查当前层是否有已处理的车道
    for (uint64_t lane : current_lanes) {
      if (processed_lanes.count(lane)) {
        SD_COARSE_MATCH_LOG << "[IsLaneOrPredecessorsProcessed] Found processed lane " << lane << " at level " << level;
        return true;
      }
    }

    // 若已到第3层，停止遍历
    if (level == 5) {
      break;
    }

    // 收集下一层前继（当前层所有车道的前继）
    std::unordered_set<uint64_t> next_lanes;
    for (uint64_t current_lane : current_lanes) {
      auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(current_lane);
      if (!lane_info) {
        continue;
      }
      for (uint64_t pre_id : lane_info->previous_lane_ids) {
        next_lanes.insert(pre_id);
      }
    }

    // 下一层无可用前继，提前终止
    if (next_lanes.empty()) {
      break;
    }
    current_lanes = next_lanes;  // 进入下一层
  }

  return false;
}
//收集前继车道id
std::unordered_set<uint64_t> CoarseMatching::CollectPreLanes(uint64_t start_id) const {
  std::unordered_set<uint64_t> prelanes;
  std::unordered_set<uint64_t> current_level = {start_id};  // 第0层：当前车道

  for (int level = 0; level <= 5; ++level) {  // 最多3层（0~3）
    // 将当前层车道加入祖先集合（第0层是当前车道本身）
    for (uint64_t lane : current_level) {
      prelanes.insert(lane);
    }

    // 若已到第3层，停止收集
    if (level == 5)
      break;

    // 收集下一层前继（当前层所有车道的前继）
    std::unordered_set<uint64_t> next_level;
    for (uint64_t current_lane : current_level) {
      auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(current_lane);
      if (!lane_info)
        continue;  // 车道信息不存在则跳过
      for (uint64_t pre_id : lane_info->previous_lane_ids) {
        next_level.insert(pre_id);  // 自动去重
      }
    }

    // 下一层无可用前继，提前终止
    if (next_level.empty())
      break;
    current_level = next_level;  // 进入下一层
  }
  return prelanes;
}

///判断是否是关注路口
bool CoarseMatching::IsInterestedJunction(const JunctionInfoCity &junction_info) {

  if (junction_info.junction_type_city == JunctionTypeCity::CrossRoad || junction_info.junction_type_city == JunctionTypeCity::TJunction ||
      junction_info.junction_type_city == JunctionTypeCity::SmallTJunction ||
      (junction_info.junction_type_city == JunctionTypeCity::UTurn && junction_info.junction_action == JunctionAction::UTurn)) {
    return true;
  }

  if (junction_info.junction_type_city == JunctionTypeCity::RoadSplit) {
    if (IsRoadSplitMain2Ramp(junction_info)) {
      return true;
    } else {
      return false;
    }
  }
  return false;
};

bool CoarseMatching::HasCommonAncestorForThreeLanes(const std::vector<uint64_t> &overlap_lanes) {
  // 车道数量不足3条时直接返回false
  if (overlap_lanes.size() < 3) {
    SD_COARSE_MATCH_LOG << "[HasCommonAncestorForThreeLanes] Insufficient lanes (" << overlap_lanes.size() << "), return false";
    return false;
  }

  // 获取前三个车道ID
  uint64_t lane1 = overlap_lanes[0];
  uint64_t lane2 = overlap_lanes[1];
  uint64_t lane3 = overlap_lanes[2];

  // 收集各车道的前继集合（包含自身及最多3层前继）
  std::unordered_set<uint64_t> ancestors1 = CollectPreLanes(lane1);
  std::unordered_set<uint64_t> ancestors2 = CollectPreLanes(lane2);
  std::unordered_set<uint64_t> ancestors3 = CollectPreLanes(lane3);

  // 检查是否存在共同前继（排除自身后的交集）
  for (uint64_t ancestor : ancestors1) {
    if (ancestors2.count(ancestor) && ancestors3.count(ancestor)) {
      SD_COARSE_MATCH_LOG << "[HasCommonAncestorForThreeLanes] Found common ancestor: " << ancestor << " for lanes {" << lane1 << ", "
                          << lane2 << ", " << lane3 << "}";
      return true;
    }
  }

  SD_COARSE_MATCH_LOG << "[HasCommonAncestorForThreeLanes] No common ancestor found for first three lanes";
  return false;
}

// 检查两条车道是否共享前继车道
bool CoarseMatching::CheckLanesShareAncestor(uint64_t lane1, uint64_t lane2) {
  // 收集车道1的前继（包含自身及最多3层前继）
  std::unordered_set<uint64_t> ancestors1 = CollectPreLanes(lane1);
  // 收集车道2的前继（包含自身及最多3层前继）
  std::unordered_set<uint64_t> ancestors2 = CollectPreLanes(lane2);

  // 调试输出前继集合
  SD_COARSE_MATCH_LOG << "[CheckLanesShareAncestor] Lane " << lane1 << " ancestors: " << PrintProcessedLanes(ancestors1);
  SD_COARSE_MATCH_LOG << "[CheckLanesShareAncestor] Lane " << lane2 << " ancestors: " << PrintProcessedLanes(ancestors2);

  // 检查前继集合交集（排除自身）
  for (uint64_t ancestor : ancestors1) {
    // 跳过自身ID
    if (ancestor == lane1 || ancestor == lane2)
      continue;

    if (ancestors2.count(ancestor)) {
      SD_COARSE_MATCH_LOG << "[CheckLanesShareAncestor] Found shared ancestor: " << ancestor << " between lanes " << lane1 << " and "
                          << lane2;
      return true;
    }
  }

  SD_COARSE_MATCH_LOG << "[CheckLanesShareAncestor] No shared ancestor found between lanes " << lane1 << " and " << lane2;
  return false;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem