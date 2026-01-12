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
 * @file SDMapElementExtract.cpp
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-09
 */
#include <log_custom.h>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <cyber/common/log.h>
#include <message/env_model/navigation/navigation.h>
#include <message/env_model/routing_map/routing_map.h>
#include "fmt/core.h"
#include "fmt/ranges.h"

#include "lib/sd_navigation/SDMapElementExtract.h"
#include "lib/sd_navigation/routing_map_debug.h"
#include "cyber/time/time.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"

namespace cem::fusion::navigation {
using message::env_model::StrLinkType;

bool HasInInsertJunction(const SDMapElementExtract::JunctionHelpInfo &junction_help_prev, size_t idx) {
  return junction_help_prev.junction_city_.junction_type_city == JunctionTypeCity::SmallTJunction ||
         junction_help_prev.junction_city_.junction_type_city == JunctionTypeCity::TJunction ||
         junction_help_prev.junction_city_.junction_type_city == JunctionTypeCity::CrossRoad ||
         junction_help_prev.junction_city_.junction_type_city == JunctionTypeCity::RoadSplit;
}

uint32_t GetTurnTypeVal(TurnType turn_type) {
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

uint32_t GetActionTurnVal(JunctionAction junction_action) {
  uint32_t turn_type = 0b0000;
  if (junction_action == JunctionAction::GoStraight) {
    turn_type = 0b0001;
  } else if (junction_action == JunctionAction::TurnLeft) {
    turn_type = 0b0010;
  } else if (junction_action == JunctionAction::TurnRight) {
    turn_type = 0b0100;
  } else if (junction_action == JunctionAction::UTurn) {
    turn_type = 0b1000;
  }
  return turn_type;
}

void SDMapElementExtract::SetMainLaneIndex(JunctionInfoCity &junction_info) {

  uint32_t turn_type = GetActionTurnVal(junction_info.junction_action);

  junction_info.main_lane_indexs.clear();

  // RoadSplit && 直行 && 导航车道数减少
  if (junction_info.junction_type_city == JunctionTypeCity::RoadSplit && junction_info.junction_action == JunctionAction::GoStraight &&
      junction_info.main_road_lane_nums > junction_info.target_road_lane_nums) {
    size_t idx_start = junction_info.idx_start != 0 ? junction_info.idx_start : junction_help_.idx_start_;
    if (idx_start + 1 >= routing_map_->sd_route.mpp_sections.size()) {
      SD_EXTRACT_MAP_ELEMENT << "Invalid target section index for RoadSplit.";
      return;
    }
    const auto &source_section = routing_map_->sd_route.mpp_sections[idx_start];
    const auto &target_section = routing_map_->sd_route.mpp_sections[idx_start + 1];

    auto source_lanes = LaneGroups(source_section);
    auto target_lanes = LaneGroups(target_section, false);

    std::set<size_t> main_lane_set;
    for (const auto &target_lane : target_lanes) {
      for (const auto &pred_id : target_lane.previous_lane_ids) {
        for (size_t idx = 0; idx < source_lanes.size(); ++idx) {
          if (source_lanes[idx].id == pred_id) {
            main_lane_set.insert(idx);
          }
        }
      }
    }
    for (size_t idx : main_lane_set) {
      junction_info.main_lane_indexs.emplace_back(idx);
    }
  } else {
    // 其他场景的默认处理逻辑
    for (std::size_t idx = 0; idx < junction_info.map_lane_arrows_plan.size(); idx++) {
      SD_EXTRACT_MAP_ELEMENT << fmt::format(
          "turn_type:{}  arrow_type:{} get_action:{} res:{}", turn_type,
          junction_info.map_lane_arrows_plan[idx],
          GetTurnTypeVal(junction_info.map_lane_arrows_plan[idx]),
          turn_type & GetTurnTypeVal(junction_info.map_lane_arrows_plan[idx]));
      if ((turn_type & GetTurnTypeVal(junction_info.map_lane_arrows_plan[idx])) != 0U) {
        junction_info.main_lane_indexs.emplace_back(idx);
      }
    }
  }
}
// bool SDMapElementExtract::CheckIfUsedBefore(const std::shared_ptr<RoutingMap> &routing_map) {
//   if (routing_map == nullptr || routing_map_ == nullptr) {
//     return false;
//   }
//   const auto &sd_route_sections_curr = routing_map->sd_route.mpp_sections;
//   const auto &sd_route_sections_prev = routing_map_->sd_route.mpp_sections;
//   std::size_t section_size           = sd_route_sections_curr.size();
//   if (section_size != sd_route_sections_prev.size()) {
//     return false;
//   }

//   constexpr double min_gap = 0.001;
//   for (std::size_t idx = 0; idx < section_size; idx++) {
//     const auto &sec_cur  = sd_route_sections_curr[idx];
//     const auto &sec_prev = sd_route_sections_prev[idx];
//     if (sec_cur.id != sec_prev.id || sec_cur.lane_num != sec_prev.lane_num) {
//       return false;
//     }
//     if (std::fabs(sec_cur.length - sec_prev.length) > min_gap) {
//       return false;
//     }map_lane_arrows_plan
//   }

//   return true;
// }

void SDMapElementExtract::Proc(const std::shared_ptr<RoutingMap> &routing_map) {
  Reset();
  if (routing_map == nullptr) {
    SD_EXTRACT_MAP_ELEMENT << "routing_map is nullptr.";
    return;
  }
  SD_EXTRACT_MAP_ELEMENT << "";
  SD_EXTRACT_MAP_ELEMENT << "start_extract_map_element. routing_map_counter:" << routing_map->header.cycle_counter;
  routing_map_ = routing_map;
  DebugSdRouteInfo();
  FillMapsInfo();
  SetSdRecommendLane();
  const auto      &sd_route_sections     = routing_map->sd_route.mpp_sections;
  const auto      &ego_in_route          = routing_map->sd_route.navi_start;
  double           ego_s_offset_in_route = 0.0;
  bool             ego_founded{false};
  std::set<size_t> idx_looped;
  if (sd_route_sections.empty()) {
    SD_EXTRACT_MAP_ELEMENT << "sd_route_section is empty.";
    return;
  }

  JunctionHelpInfo junction_help_prev;
  // for (size_t idx = 0; idx < sd_route_sections.size(); idx++) {
  for (const auto &section_t : sd_route_sections) {
    if (!ego_founded && section_t.id == ego_in_route.section_id) {
      ego_founded = true;
    }
    if (ego_founded) {
      ego_s_offset_in_route += ego_in_route.s_offset;
      break;
    }
    ego_s_offset_in_route += section_t.length;
  }
  if (!ego_founded) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("can't find the ego_route:{}", ego_in_route.section_id);
    return;
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("ego_section_id:{} offset:{:.2f} ego_s_offset_in_route:{:.2f}", ego_in_route.section_id,
                                        ego_in_route.s_offset, ego_s_offset_in_route);
  /*
  auto GetLength = [&](size_t idx_end) {
    if (sd_route_sections.empty()) {
      return 0.0;
    }
    const size_t end_idx = std::min(idx_end, sd_route_sections.size() - 1);
    return std::accumulate(sd_route_sections.begin(),
                           sd_route_sections.begin() + end_idx + 1,  // 包含end_idx
                           0.0, [](double sum, const auto &section) { return sum + section.length; });
  };
  for (size_t idx = 0; idx < sd_route_sections.size(); idx++) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("section_id:{}  offset:{:.2f}", sd_route_sections[idx].id, GetLength(idx));
  }

  
  for (size_t idx = 0; idx < sd_route_sections.size(); idx++) {
    const auto &section_t = sd_route_sections[idx];
    junction_help_        = JunctionHelpInfo();
    // if (!ego_founded && section_t.id == ego_in_route.section_id) {
    //   ego_founded = true;
    // }
    // if (!ego_founded) {
    //   continue;
    // }
    size_t orin_idx = idx;
    SD_EXTRACT_MAP_ELEMENT << "\n\n";
    SD_EXTRACT_MAP_ELEMENT << fmt::format("junction_judge_section_id:{}  succ:{}  has_junction:{:d}  idx:{}", section_t.id,
                                          section_t.successor_section_id_list, section_t.has_junction, idx);
    if (IsUTurn(idx, sd_route_sections)) {
      idx = junction_help_.idx_end_ - 1;
    } else if ((IsJunctionCross(idx, sd_route_sections) || IsJunctionT(idx, sd_route_sections) ||
                IsJunctionTSmall(idx, sd_route_sections) || IsJunctionSplit(idx, sd_route_sections))) {
      idx = junction_help_.idx_end_ != idx ? junction_help_.idx_end_ - 1 : junction_help_.idx_end_;
    } else if (IsJunctionMerge(idx, sd_route_sections)) {
      idx = junction_help_.idx_end_ != idx ? junction_help_.idx_end_ - 1 : junction_help_.idx_end_;
    }
    junction_help_prev = junction_help_;
    if (junction_help_.junction_city_.junction_type_city == JunctionTypeCity::Unknown) {
      SD_EXTRACT_MAP_ELEMENT << fmt::format("unknown_type.");
      continue;
    }
    junction_help_.junction_city_.offset = GetLength(orin_idx) - ego_s_offset_in_route;
    if (junction_help_.junction_city_.junction_type_city == JunctionTypeCity::RoadMerge) {
      junction_help_.junction_city_.offset = GetLength(orin_idx) - ego_s_offset_in_route;
    }
    if (idx + 1 < sd_route_sections.size()) {
      auto val = AngleDiffBetweenSection(sd_route_sections[idx], sd_route_sections[idx + 1]);

      junction_help_.junction_city_.next_angle = val ? *val : 0.0;
    }

    size_t idx_t        = junction_help_.idx_start_;
    auto  &junction_ids = junction_help_.junction_city_.junction_ids;
    for (; idx_t <= junction_help_.idx_end_ && idx_t < sd_route_sections.size(); idx_t++) {
      junction_ids.emplace_back(sd_route_sections[idx_t].id);
    }
    if (junction_help_.junction_city_.junction_type_city == JunctionTypeCity::RoadSplit ||
        junction_help_.junction_city_.junction_type_city == JunctionTypeCity::SmallTJunction) {
      junction_ids.emplace_back(sd_route_sections[junction_help_.idx_start_ + 1].id);
    } else if (junction_help_.junction_city_.junction_type_city == JunctionTypeCity::RoadMerge) {
      // junction_ids.insert(junction_ids.begin(), sd_route_sections[junction_help_.idx_start_].id);
    }
    if (junction_ids.empty()) {
      SD_EXTRACT_MAP_ELEMENT << fmt::format("junction idx_start:{}  idx_end:{} route_size:{}", junction_help_.idx_start_,
                                            junction_help_.idx_end_, sd_route_sections.size());
      continue;
    }
    junction_help_.junction_city_.time = apollo::cyber::Time::Now().ToSecond();

    SetMainLaneIndex(junction_help_.junction_city_);
    if (junction_help_.junction_city_.main_lane_indexs.empty() &&
        junction_help_.junction_city_.junction_type_city == JunctionTypeCity::UTurn) {
      junction_help_.junction_city_.main_lane_indexs.emplace_back(0);
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format("{}", junction_help_.junction_city_)
                           << fmt::format("  idx_range:[{},{}]  idx:{}", junction_help_.idx_start_, junction_help_.idx_end_, idx);

    junctions_info_.emplace_back(junction_help_.junction_city_);
  }
  SD_JUNCTION_CONVERTE_LOG << fmt::format("junction_help_size:{}  sd_route_sections_size:{}", junctions_info_.size(),
                                          sd_route_sections.size());*/
  constexpr double kDisFusionJunctionThreshold = Angle_90;
  constexpr double next_angle_threshold        = 35.0;
  constexpr double offset_between_T_split      = 50.0;
  SectionsProcess(routing_map);

  for (auto it = junctions_info_.begin(); it != junctions_info_.end() && std::next(it) != junctions_info_.end(); ++it) {
    auto  &junction_cur        = *it;
    auto  &junction_next       = *std::next(it);
    double offset_diff         = junction_next.offset - junction_cur.offset;
    junction_cur.diff_offset   = offset_diff;
    bool   is_within_threshold = offset_diff < kDisFusionJunctionThreshold;

    if (junction_cur.junction_type_city == JunctionTypeCity::RoadSplit &&
        (junction_next.junction_type_city == JunctionTypeCity::CrossRoad ||
         junction_next.junction_type_city == JunctionTypeCity::TJunction) &&
        is_within_threshold) {
      auto junctions_angle = JunctionAnglesAroundJunction(sd_route_sections, junction_cur.idx_start, 1);
      junctions_angle.erase(std::remove_if(junctions_angle.begin(), junctions_angle.end(),
                                           [](const AngleJunction &rhs) { return !rhs.section || std::fabs(rhs.angle) > Angle_75; }),
                            junctions_angle.end());
      if (junctions_angle.size()>0 && std::any_of(junctions_angle.begin(), junctions_angle.end(), [](auto const &lhs) { return lhs.angle < -10; })) {
        junction_cur.is_dedicated_right_turn_lane  = true;
        junction_next.is_dedicated_right_turn_lane = true;
      }

      // if (offset_diff < next_angle_threshold) {
      //   junction_cur.is_valid = false;
      //   SD_EXTRACT_MAP_ELEMENT << fmt::format("invalid RoadSplit junction.   {}", junction_cur);
      // }
      continue;
    }
    if (junction_cur.junction_type_city == JunctionTypeCity::SmallTJunction && std::fabs(junction_cur.next_angle) < next_angle_threshold &&
        junction_next.junction_type_city == JunctionTypeCity::RoadSplit && is_within_threshold) {
      auto it_t = std::find_if(sd_route_sections.begin(), sd_route_sections.end(),
                               [&](const auto &sec) { return sec.id == junction_cur.junction_id; });
      if (it_t == sd_route_sections.end()) {
        continue;
      }
      auto angles = JunctionAnglesAroundJunction(sd_route_sections, static_cast<size_t>(std::distance(sd_route_sections.begin(), it_t)));
      if (std::any_of(angles.begin(), angles.end(), [](const AngleJunction &rhs) { return IsInDegreeTarget(rhs.angle, Angle_90); })) {
        junction_cur.is_valid = false;
        SD_EXTRACT_MAP_ELEMENT<<"junction_cur"<<junction_cur.next_angle;
        SD_EXTRACT_MAP_ELEMENT << fmt::format("invalid junction.   {}", junction_cur);
      }
      continue;
    }
    if (junction_cur.junction_type_city == JunctionTypeCity::CrossRoad && junction_next.junction_type_city == JunctionTypeCity::RoadSplit &&
        junction_next.is_dedicated_right_turn_lane && junction_next.offset - junction_cur.offset < offset_between_T_split) {
      junction_cur.junction_type_city = JunctionTypeCity::SmallTJunction;
      continue;
    }
  }

  JudgeIsNeedMatchArrow(junctions_info_);
  JudgeTjunctionIsOnlyUturn(junctions_info_, sd_route_sections);

  for (const auto &junction : junctions_info_) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("{}", junction);
  }
}

void SDMapElementExtract::SectionsProcess(const std::shared_ptr<RoutingMap> &routing_map) {
  const auto &route_sections = routing_map->sd_route.mpp_sections;
  for (size_t i = 1; i < static_cast<size_t>(route_sections.size()); i++) {
    if (route_sections[i].direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
      if (route_sections[i].points->size() > (size_t)2) {
        size_t front_point = route_sections[i].points->front().x() * route_sections[i].points->front().x() +
                             route_sections[i].points->front().y() * route_sections[i].points->front().y();
        size_t back_point = route_sections[i].points->back().x() * route_sections[i].points->back().x() +
                            route_sections[i].points->back().y() * route_sections[i].points->back().y();
        if(front_point>back_point)
        {
          std::reverse(route_sections[i].points->begin(),route_sections[i].points->end());
        }
      }
    }
  }
  const auto &ego_in_route   = routing_map->sd_route.navi_start;
  double      ego_offset     = 0.0;
  bool        is_find_ego{false};
  for (const auto &sec : route_sections) {
    if (ego_in_route.section_id == sec.id) {
      ego_offset += ego_in_route.s_offset;
      is_find_ego = true;
      break;
    }
    ego_offset += sec.length;
  }
  if (!is_find_ego) {
    return;
  }

  std::vector<JunctionInfoCity> junction_infos(route_sections.size());

  junctions_info_.clear();
  GetJunctionType(route_sections, ego_offset, junction_infos);
  MergeSections(route_sections, junction_infos);
  FillJunctionInfo(route_sections, junction_infos);
  JudgeIsNeedMatchArrow(junctions_info_);
}

void SDMapElementExtract::GetJunctionType(const std::vector<SDSectionInfo> &route_sections, double ego_offset,
                                          std::vector<JunctionInfoCity> &junction_infos) {
  SD_EXTRACT_MAP_ELEMENT << "\n\n";
  double offset = 0.0;
  for (size_t idx = 0; idx < route_sections.size() - 1; idx++) {
    const auto &section_t_curr = route_sections[idx];
    const auto &section_t_next = route_sections[idx + 1];
    offset += section_t_curr.length;
    auto section_angles_all = JunctionAnglesAroundJunction(route_sections, idx);
    if (section_angles_all.size() <= 1) {
      SD_EXTRACT_MAP_ELEMENT << "the_sections_less_or_equal_one skip_it. size:" << section_angles_all.size() << "\n\n";
      SD_EXTRACT_MAP_ELEMENT << "section_t_currids:" << section_t_curr.id;
      continue;
    }

    std::vector<AngleJunction> section_angles_succ = section_t_curr.successor_section_id_list.size() >= 2
                                                         ? JunctionAnglesAroundJunction(route_sections, idx, 1)
                                                         : std::vector<AngleJunction>{};

    auto next_angle = AngleDiffBetweenSection(route_sections[idx], route_sections[idx + 1], false);
    bool exist_uturn{false};
    bool section_curr_exist_Only_U_TURN{false};
    // erase the uturn of non-route road.
    if (section_angles_all.size() > 2) {
      section_angles_all.erase(std::remove_if(section_angles_all.begin(), section_angles_all.end(),
                                              [&](const AngleJunction &rhs) { return rhs.section && SectionIsUTurn(*rhs.section); }),
                               section_angles_all.end());
    }
    if (section_t_curr.lane_group_idx.size()) {
      auto section_lanegroups_id = section_t_curr.lane_group_idx.back().id;
      if (map_sd_lane_groups_info_.count(section_lanegroups_id) != 0) {
        auto lanes = map_sd_lane_groups_info_[section_lanegroups_id]->lane_info;
        if (lanes.size() > 0) {
          if (section_angles_succ.size() > 0) {
            if (std::any_of(section_angles_succ.begin(), section_angles_succ.end(),
                            [&](const auto &rhs) { return rhs.section && SectionIsUTurn(*rhs.section); })) {
              exist_uturn = true;
              if (lanes[0].next_lane_ids.size() == 1) {
                for (auto it : section_angles_succ) {
                  if (SectionIsUTurn(*it.section)) {
                    auto lane_group_info = LaneGroups(*it.section, false);
                    if (lane_group_info.size() > 0 && lanes[0].next_lane_ids[0] == lane_group_info[0].id) {
                      section_curr_exist_Only_U_TURN = true;
                      break;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    // if (section_angles_succ.size() >= 2 && !section_curr_exist_Only_U_TURN) {
    //   section_angles_succ.erase(std::remove_if(section_angles_succ.begin(), section_angles_succ.end(),
    //                                            [&](const AngleJunction &rhs) { return rhs.section && SectionIsUTurn(*rhs.section); }),
    //                             section_angles_succ.end());
    // }

    auto section_angles_succ_filtered = section_angles_succ;
    if (section_angles_succ_filtered.size() >= 2) {
      section_angles_succ_filtered.erase(std::remove_if(section_angles_succ_filtered.begin(), section_angles_succ_filtered.end(),
                                                        [](const AngleJunction &rhs) { return std::fabs(rhs.angle) > Angle_75; }),
                                         section_angles_succ_filtered.end());
    }

    bool is_exsit_90_left  = std::any_of(section_angles_all.begin(), section_angles_all.end(), [](const AngleJunction &rhs) {
      return rhs.position == AngleJunction::PositionDirection::LEFT && IsInDegreeTarget(rhs.angle, Angle_90);
    });
    bool is_exsit_90_right = std::any_of(section_angles_all.begin(), section_angles_all.end(), [](const AngleJunction &rhs) {
      return rhs.position == AngleJunction::PositionDirection::RIGHT && IsInDegreeTarget(rhs.angle, -Angle_90);
    });
    bool is_exsit_0        = std::any_of(section_angles_all.begin(), section_angles_all.end(), [](const AngleJunction &rhs) {
      return IsInDegreeTarget(rhs.angle, 0.0) || IsInDegreeTarget(rhs.angle, Angle_180) || IsInDegreeTarget(rhs.angle, -Angle_180);
    });
    int  exsit_val_nums    = static_cast<int>(is_exsit_0) + static_cast<int>(is_exsit_90_right) + static_cast<int>(is_exsit_90_left);
    bool line_2_cross      = section_angles_all.size() >= 3 && Is2LineCross(route_sections[idx], section_angles_all);
    bool ego_is_in_right   = SectionIsRightTurnDedicated(section_t_curr) || SectionIsRightTurn(section_t_curr);
    if (section_t_curr.id) {
      SD_EXTRACT_MAP_ELEMENT << "section_t_curr.id" << section_t_curr.id;
      SD_EXTRACT_MAP_ELEMENT << "exsit_val_nums :" << exsit_val_nums;
      SD_EXTRACT_MAP_ELEMENT << "line_2_cross" << line_2_cross << ego_is_in_right;
      SD_EXTRACT_MAP_ELEMENT << "ection_angles_all.size() " << section_angles_all.size();
      SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
      SD_EXTRACT_MAP_ELEMENT << "section_angles_succ_filtered" << section_angles_succ_filtered.size();
    }
    // judge the section is uturn or not.
    std::string reason;
    if (std::any_of(section_angles_all.begin(), section_angles_all.end(), [](const AngleJunction &rhs) {
          return rhs.section && LinkTypeMaskType(rhs.section->link_type, SDLinkTypeMask::SDLT_RING);
        })) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RING;
      reason                                 = "next_section_is_RING.";
    } else if (section_angles_succ_filtered.size() >= 2 && !ego_is_in_right && SectionIsRightTurnDedicated(section_t_next)) {
      junction_infos[idx].junction_type_city           = JunctionTypeCity::RoadSplit;
      junction_infos[idx].is_dedicated_right_turn_lane = true;

      reason = "next_section_is_dedicated.";
    } else if (section_angles_succ_filtered.size() >= 2 && !ego_is_in_right && SectionIsRightTurn(section_t_next)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadSplit;

      reason = "next_section_is_right_turn";
    } else if (SectionIsUTurn(section_t_next)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::UTurn;

      reason = "next_section_uturn";
    } else if (exist_uturn) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::UTurn;
      if (section_curr_exist_Only_U_TURN) {
        junction_infos[idx].is_only_Turn = true;
      }

      reason = "exist_uturn,exist_only_Uturn_can't_go_straight";
    } else if (section_angles_succ_filtered.size() >= 2 && next_angle && next_angle.value() < -Angle_25 &&
               route_sections[idx + 1].direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::TJunction;

      reason = "next_section_is_bidirection_passable.";
    } else if (IsSubpathBidirectional(section_angles_succ, route_sections, idx)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::TJunction;

      reason = "next_subpath_section_is_bidirection_passable.";
    } else if (section_angles_succ_filtered.size() >= 2 && next_angle && IsInDegreeTarget(next_angle.value(), Angle_90)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::TJunction;

      reason = "next_section_has_near_90.";
    } else if (section_angles_succ_filtered.size() >= 2 && !ego_is_in_right &&
               std::any_of(section_angles_succ.begin(), section_angles_succ.end(), [&](const AngleJunction &rhs) {
                 return rhs.section && (SectionIsRightTurn(*rhs.section) || SectionIsRightTurnDedicated(*rhs.section));
               })) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadSplit;

      reason = "subpath_lane_has_right_turn_lane.";
    } else if (std::any_of(section_angles_all.begin(), section_angles_all.end(), [&](const AngleJunction &rhs) {
                 return rhs.section && LinkTypeMaskType(rhs.section->link_type, SDLinkTypeMask::SDLT_CROSSLINK) &&
                        (!SectionIsUTurn(*(rhs.section)));//add SectionIsUTurn to fix CNOAC2-78260 not affect other logic
               })) {
      if (section_angles_all.size() >= 3 && (exsit_val_nums == 3 || line_2_cross)) {
        junction_infos[idx].junction_type_city = JunctionTypeCity::CrossRoad;
      } else if (LinkTypeMaskType(section_t_next.link_type, SDLinkTypeMask::SDLT_CROSSLINK) && idx + 2 < route_sections.size() &&
                 LinkTypeMaskType(route_sections[idx + 2].link_type, SDLinkTypeMask::SDLT_CROSSLINK)) {
        junction_infos[idx].junction_type_city = JunctionTypeCity::CrossRoad;
      } else {
        junction_infos[idx].junction_type_city = JunctionTypeCity::TJunction;
      }

      reason = "has_lane_is_cross_link_type.";
    } else if (section_t_curr.successor_section_id_list.size() >= 2 && section_angles_succ.size() >= 2 &&
               TempFilterSplit_01(section_angles_succ) &&
               std::all_of(section_angles_succ.begin(), section_angles_succ.end(), [](const AngleJunction &rhs) {
                 return std::fabs(rhs.angle) < Angle_75 || rhs.position == AngleJunction::PositionDirection::DEFAULT;
               })) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadSplit;
      junction_infos[idx].is_dedicated_right_turn_lane =
          std::any_of(section_angles_succ.begin(), section_angles_succ.end(),
                      [&](const AngleJunction &rhs) { return rhs.section && SectionIsRightTurnDedicated(*rhs.section); });

      reason = "all_successor_lane_angle_less_75.";
    } else if (section_t_curr.successor_section_id_list.size() >= 2 && section_angles_succ.size() >= 2 &&
               TempFilterSplit_02(section_angles_succ)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadSplit;
      junction_infos[idx].is_dedicated_right_turn_lane =
          std::any_of(section_angles_succ.begin(), section_angles_succ.end(),
                      [&](const AngleJunction &rhs) { return rhs.section && SectionIsRightTurnDedicated(*rhs.section); });

      reason = "all_successor_in_one_direction.";
    } else if (auto section_angles_prev = JunctionAnglesAroundJunction(route_sections, idx + 1, 2);
               section_t_next.predecessor_section_id_list.size() >= 2 &&
               std::all_of(section_angles_prev.begin(), section_angles_prev.end(), [](const AngleJunction &rhs) {
                 return std::fabs(rhs.angle) > 105 || std::fabs(rhs.angle) < Angle_5 ||
                        rhs.position == AngleJunction::PositionDirection::DEFAULT;
               })) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadMerge;
      junction_infos[idx].is_dedicated_right_turn_lane =
          std::any_of(section_angles_prev.begin(), section_angles_prev.end(),
                      [&](const AngleJunction &rhs) { return rhs.section && SectionIsRightTurnDedicated(*rhs.section); });

      reason = "all_previous_lane_angle_less_75.";
    } else if (section_angles_all.size() >= 3 && (exsit_val_nums == 3 || line_2_cross)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::CrossRoad;

      reason = " line_2_cross:" + std::to_string(static_cast<int>(line_2_cross));
    } else if (exsit_val_nums == 2) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::SmallTJunction;

      reason = "exist_val_reason.";
    } else if (section_t_curr.successor_section_id_list.size() >= 2 && section_angles_succ_filtered.size() >= 2 &&
               TempFilterSplit_03(section_angles_succ_filtered)) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::RoadSplit;
      junction_infos[idx].is_dedicated_right_turn_lane =
          std::any_of(section_angles_succ.begin(), section_angles_succ.end(),
                      [&](const AngleJunction &rhs) { return rhs.section && SectionIsRightTurnDedicated(*rhs.section); });

      reason = "except_BIDIRECTIONAL_PASSABLE_and_angle_big_75_all_succ_section_in_one_direction.";
    }
    if (junction_infos[idx].junction_type_city != JunctionTypeCity::Unknown) {
      junction_infos[idx].offset      = offset - ego_offset;
      SD_EXTRACT_MAP_ELEMENT<<"offset"<<offset<<" ego_offset"<<ego_offset<<" offset - ego_offset"<<offset - ego_offset;
      junction_infos[idx].junction_id = section_t_curr.id;
      junction_infos[idx].idx_start   = idx;
      junction_infos[idx].idx_end     = idx + 1;
      junction_infos[idx].time        = apollo::cyber::Time::Now().ToSecond();
      junction_infos[idx].junction_ids.emplace_back(route_sections[idx].id);
      junction_infos[idx].junction_ids.emplace_back(route_sections[idx + 1].id);
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format(
        "junction_id:{}  offset:{:.2f}  type:{}  is_right_dedicated:{:d} [0,left,right;{:d},{:d},{:d}] idx:{}-{} all_succ_size:{}-{}  "
        "line_2_cross:{:d} next_angle:{:.2f}  reason:[{}]\n\n",
        section_t_curr.id, junction_infos[idx].offset, StrJunctionTypeCity(junction_infos[idx].junction_type_city),
        junction_infos[idx].is_dedicated_right_turn_lane, is_exsit_0, is_exsit_90_left, is_exsit_90_right, idx, idx + 1,
        section_angles_all.size(), section_angles_succ.size(), line_2_cross, next_angle ? next_angle.value() : 0.0, reason);
  }
}

constexpr double kEpsilon = 1e-6;

inline bool DoubleEqual(double a, double b, double epsilon = kEpsilon) {
  return std::abs(a - b) < epsilon;
}

bool operator==(const JunctionInfo &lhs, const JunctionInfo &rhs) {
  return lhs.junction_id == rhs.junction_id && DoubleEqual(lhs.offset, rhs.offset) && lhs.main_road_lane_nums == rhs.main_road_lane_nums &&
         lhs.target_road_lane_nums == rhs.target_road_lane_nums;
}

bool operator==(const JunctionInfoCity &lhs, const JunctionInfoCity &rhs) {
  if (!(static_cast<const JunctionInfo &>(lhs) == static_cast<const JunctionInfo &>(rhs))) {
    return false;
  }

  if (lhs.junction_type_city != rhs.junction_type_city || lhs.junction_action != rhs.junction_action ||
      lhs.split_merge_direction != rhs.split_merge_direction || lhs.junction_state_city != rhs.junction_state_city) {
    return false;
  }

  if (lhs.map_lane_arrows_plan != rhs.map_lane_arrows_plan || lhs.map_lane_arrows_default != rhs.map_lane_arrows_default) {
    return false;
  }

  if (lhs.main_lane_indexs != rhs.main_lane_indexs) {
    return false;
  }

  if (lhs.succ_road_class != rhs.succ_road_class || lhs.prev_road_class != rhs.prev_road_class) {
    return false;
  }

  if (lhs.junction_ids != rhs.junction_ids) {
    return false;
  }

  if (lhs.is_dedicated_right_turn_lane != rhs.is_dedicated_right_turn_lane || lhs.is_valid != rhs.is_valid) {
    return false;
  }

  return true;
}

void SDMapElementExtract::MergeSections(const std::vector<SDSectionInfo> &route_sections, std::vector<JunctionInfoCity> &junction_infos) {
  if (route_sections.size() != junction_infos.size()) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("route_sections:{}  junction_infos:{}", route_sections.size(), junction_infos.size());
    return;
  }
  for (std::size_t idx = 0; idx < junction_infos.size(); idx++) {
    if (junction_infos[idx].junction_type_city == JunctionTypeCity::Unknown) {
      continue;
    }
    SD_EXTRACT_MAP_ELEMENT << "merge_id:" << junction_infos[idx].junction_id;
    if (idx + 1 < route_sections.size()) {
      if (SectionIsUTurn(route_sections[idx + 1])) {
        if (idx + 1 < junction_infos.size() && junction_infos[idx].junction_type_city == JunctionTypeCity::UTurn &&
            (junction_infos[idx + 1].junction_type_city == JunctionTypeCity::TJunction ||
             junction_infos[idx + 1].junction_type_city == JunctionTypeCity::SmallTJunction)) {
          SD_EXTRACT_MAP_ELEMENT << fmt::format("junction_id:{}  reset_unknown:", junction_infos[idx + 1].junction_id);
          junction_infos[idx + 1] = JunctionInfoCity();
          if (idx + 2 < junction_infos.size()) {
            junction_infos[idx].junction_ids.emplace_back(route_sections[idx + 2].id);
          }
          continue;
        }
      }
    }

    if (!LinkTypeMaskType(route_sections[idx].link_type, SDLinkTypeMask::SDLT_CROSSLINK) &&
        !LinkTypeMaskType(route_sections[idx + 1].link_type, SDLinkTypeMask::SDLT_CROSSLINK)) {
      continue;
    }
    bool is_exist_cross = junction_infos[idx].junction_type_city == JunctionTypeCity::CrossRoad;
    bool is_exist_T     = junction_infos[idx].junction_type_city == JunctionTypeCity::TJunction ||
                      junction_infos[idx].junction_type_city == JunctionTypeCity::SmallTJunction;
    std::size_t idx_end = idx + 1;
    for (; idx_end < junction_infos.size(); idx_end++) {
      if (!LinkTypeMaskType(route_sections[idx_end].link_type, SDLinkTypeMask::SDLT_CROSSLINK)) {
        break;
      }
      if (junction_infos[idx_end].junction_type_city == JunctionTypeCity::CrossRoad) {
        is_exist_cross = true;
      } else if (junction_infos[idx_end].junction_type_city == JunctionTypeCity::TJunction ||
                 junction_infos[idx_end].junction_type_city == JunctionTypeCity::SmallTJunction) {
        is_exist_T = true;
      }
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format("idx:{}-{}  is_exist_cross:{:d}  is_exist_T:{:d}", idx, idx_end, is_exist_cross, is_exist_T);
    if (idx_end == idx + 1) {
      continue;
    }
    if (is_exist_cross) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::CrossRoad;
    } else if (is_exist_T) {
      junction_infos[idx].junction_type_city = JunctionTypeCity::TJunction;
    } else {
      continue;
    }
    if (idx_end == junction_infos.size()) {
      idx_end--;
    }
    junction_infos[idx].idx_start = idx;
    junction_infos[idx].idx_end   = idx_end;
    junction_infos[idx].is_valid  = true;
    junction_infos[idx].junction_ids.clear();
    for (std::size_t idx_t = idx; idx_t <= idx_end; idx_t++) {
      if (idx_t > idx && idx_t < idx_end) {
        junction_infos[idx_t] = JunctionInfoCity();
      }
      junction_infos[idx].junction_ids.emplace_back(route_sections[idx_t].id);
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format("junction_id:{}  type:{}  ids:{}  is_exist_cross:{:d} idx:{}-{}", junction_infos[idx].junction_id,
                                          StrJunctionTypeCity(junction_infos[idx].junction_type_city), junction_infos[idx].junction_ids,
                                          is_exist_cross, idx, idx_end);
  }
}

void SDMapElementExtract::FillJunctionInfo(const std::vector<SDSectionInfo> &route_sections,
                                           std::vector<JunctionInfoCity>    &junction_infos) {
  for (auto &junction : junction_infos) {
    if (junction.junction_type_city == JunctionTypeCity::Unknown || junction.junction_type_city == JunctionTypeCity::InvalidJunction) {
      continue;
    }
    switch (junction.junction_type_city) {
      case JunctionTypeCity::RoadSplit:
        FillJunctionInfo_Split(route_sections, junction);
        break;
      case JunctionTypeCity::RoadMerge:
        FillJunctionInfo_Merge(route_sections, junction);
        break;
      case JunctionTypeCity::CrossRoad:
        FillJunctionInfo_CrossRoad(route_sections, junction);
        break;
      case JunctionTypeCity::TJunction:
      case JunctionTypeCity::SmallTJunction:
        FillJunctionInfo_T(route_sections, junction);
        break;
      case JunctionTypeCity::UTurn:
        FillJunctionInfo_Uturn(route_sections, junction);
        break;
      default:
        break;
    }
    if (junction.junction_type_city == JunctionTypeCity::Unknown || junction.junction_type_city == JunctionTypeCity::InvalidJunction) {
      continue;
    }
    SetMainLaneIndex(junction);
    if (junction.main_lane_indexs.empty() && junction.junction_type_city == JunctionTypeCity::UTurn) {
      junction.main_lane_indexs.emplace_back(0);
    }
    auto it = std::find_if(junctions_info_.begin(), junctions_info_.end(),
                           [&junction](const JunctionInfoCity &rhs) { return rhs.junction_id == junction.junction_id; });
    if (it != junctions_info_.end() && !(*it == junction)) {
      SD_EXTRACT_MAP_ELEMENT << fmt::format("raw_{}", *it);
    }
    const auto idx_start = junction.idx_start;
    auto       section   = route_sections[idx_start];
    if (section.lane_group_idx.empty() && idx_start - 1 >=0 && idx_start - 1 < route_sections.size() ) {
      section = route_sections[idx_start - 1];
      if (section.lane_group_idx.empty()) {
        if (junction.junction_type_city == JunctionTypeCity::CrossRoad && idx_start - 2 >= 0 && idx_start - 2 < route_sections.size() ) {
          section = route_sections[idx_start - 2];
          if (section.lane_group_idx.empty()) {
            continue;
          }
        } else {
          continue;
        }
      }
    }

    auto group_idx = section.lane_group_idx;
    std::sort(group_idx.begin(), group_idx.end(),
              [](const SDLaneGroupIndex &lhs, const SDLaneGroupIndex &rhs) { return lhs.end_range_offset < rhs.end_range_offset; });

    const auto &search_range = std::vector<SDLaneGroupIndex>(group_idx.rbegin(), group_idx.rend());

    for (const auto &idx : search_range) {
      const auto it = map_sd_lane_groups_info_.find(idx.id);
      if (it != map_sd_lane_groups_info_.end() && it->second && !it->second->lane_info.empty()) {
        bool is_all_wait = std::all_of(it->second->lane_info.begin(), it->second->lane_info.end(), [](const LaneInfo &rhs) {
          return rhs.type == LaneType::LANE_LEFT_WAIT || rhs.type == LaneType::LANE_RIGHT_TURN_AREA ||
                 rhs.type == LaneType::LANE_U_TURN_AREA || rhs.type == LaneType::LANE_NO_TURN_AREA;
        });
        if (is_all_wait) {
          continue;
        }
        SD_EXTRACT_MAP_ELEMENT << "find_the_lane_index:" << idx.id;
        junction.map_plan_turn_type_lane_group_id = idx.id;
        break;
      }
    }
    junctions_info_.emplace_back(junction);
    SD_EXTRACT_MAP_ELEMENT << fmt::format("new_{}\n\n", junction);
  }
}

void SDMapElementExtract::FillJunctionInfo_Uturn(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info) {
  const std::size_t idx_start = junction_info.idx_start;
  const std::size_t idx_end   = junction_info.idx_end;
  const auto &lane_group        = LaneGroups(route_sections[idx_start]);
  auto       &arrow_vec         = junction_info.map_lane_arrows_plan;
  auto       &arrow_vec_default = junction_info.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }
  bool        not_Calculate_Uturn_Angle{false};
  std::size_t kk = 0;
  if (idx_start < route_sections.size() && route_sections[idx_start].length >= 150) {
    not_Calculate_Uturn_Angle = true;
  }
  auto angle_val_start_end = AngleDiffBetweenSection(route_sections[idx_start], route_sections[idx_end], true, not_Calculate_Uturn_Angle);
  if (angle_val_start_end.has_value()) {
    double angle = angle_val_start_end.value();
    SD_EXTRACT_MAP_ELEMENT << "idx_start" << idx_start << "idx_end" << idx_end;
    SD_EXTRACT_MAP_ELEMENT << "route_sections[idx_start]:.id" << route_sections[idx_start].id;
    SD_EXTRACT_MAP_ELEMENT << "route_sections[idx_end]:.id" << route_sections[idx_end].id;
    if (SectionIsUTurn(route_sections[idx_end]) && fabs(angle) > 30) {
      junction_info.junction_action = JunctionAction::UTurn;
      SD_EXTRACT_MAP_ELEMENT << "JunctionAction::UTurn";
    } else {
      std::vector<AngleJunction> section_angles_succ = std::vector<AngleJunction>{};
      if (angle_val_start_end.has_value()) {
        junction_info.next_angle  = angle_val_start_end.value();
        const auto section_end_id = route_sections[idx_end].id;
        if (map_sd_sections_.count(section_end_id)) {
          section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[section_end_id], *angle_val_start_end});
        }
        SD_EXTRACT_MAP_ELEMENT << "base_section_id:" << route_sections[idx_start].id;
        SD_EXTRACT_MAP_ELEMENT << "section_id:" << section_end_id;
        SD_EXTRACT_MAP_ELEMENT << "angle_val_start_end:" << angle;
        if (angle >= 160 || angle <= -170) {
          junction_info.junction_action = JunctionAction::UTurn;
        } else {
          if (route_sections[idx_start].successor_section_id_list.size() != 0) {
            if (route_sections[idx_start].successor_section_id_list.size() >= 2) {
              for (int i = idx_start; i < idx_end; i++) {
                for (const auto &id_succ : route_sections[i].successor_section_id_list) {

                  if (map_sd_sections_.count(id_succ) != 0 && map_sd_sections_[id_succ]->id != route_sections[i + 1].id &&
                      !SectionIsUTurn(*map_sd_sections_[id_succ])) {

                    auto angle_val =
                        AngleDiffBetweenSection(route_sections[idx_start], *map_sd_sections_[id_succ], true, not_Calculate_Uturn_Angle);
                    section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[id_succ], *angle_val});
                  }
                }
              }
            } else {
              if (idx_start + 1 < route_sections.size() && route_sections[idx_start].successor_section_id_list.size() == 1 &&
                  route_sections[idx_start].successor_section_id_list[0] == route_sections[idx_start + 1].id) {
                if (angle_val_start_end.value() < 30 && angle_val_start_end.value() > -30) {
                  junction_info.junction_action = JunctionAction::GoStraight;

                } else if (angle_val_start_end.has_value() && angle_val_start_end.value() < -30) {
                  junction_info.junction_action = JunctionAction::TurnRight;
                } else if (angle_val_start_end.has_value() && angle_val_start_end.value() > 30) {
                  junction_info.junction_action = JunctionAction::TurnLeft;
                }
              } else {
                junction_info.junction_action = JunctionAction::Unknown;
              }
            }
            // if (junction_info.junction_id == 15270229220) {
            //   SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
            //   SD_EXTRACT_MAP_ELEMENT << "kk" << kk;
            // }
            section_angles_succ.erase(std::remove_if(section_angles_succ.begin(), section_angles_succ.end(),
                                                     [](const auto &rhs) { return std::fabs(rhs.angle) > 160; }),
                                      section_angles_succ.end());
            auto section_angles_succ_nomal = section_angles_succ;
            std::sort(section_angles_succ.begin(), section_angles_succ.end(),
                      [&](const auto &a, const auto &b) { return fabs(a.angle) < fabs(b.angle); });
            std::sort(section_angles_succ_nomal.begin(), section_angles_succ_nomal.end(),
                      [&](const auto &a, const auto &b) { return (a.angle) > (b.angle); });

            for (int i = 0; i < section_angles_succ.size(); i++) {
              if (idx_end < route_sections.size() && route_sections[idx_end].id == section_angles_succ[i].section->id) {
                if (i == 0 && fabs(angle_val_start_end.value()) < 45.0) {
                  junction_info.junction_action = JunctionAction::GoStraight;
                  kk                            = 1;
                  break;
                } else if (angle_val_start_end.value() > 0 && section_angles_succ_nomal.size() &&
                           route_sections[idx_end].id == section_angles_succ_nomal[0].section->id) {
                  junction_info.junction_action = JunctionAction::TurnLeft;
                  kk                            = 2;
                  break;

                } else if (angle_val_start_end.value() < 0 && section_angles_succ_nomal.size() &&
                           route_sections[idx_end].id == section_angles_succ_nomal.back().section->id) {
                  junction_info.junction_action = JunctionAction::TurnRight;
                  kk                            = 3;
                  break;
                } else {
                  junction_info.junction_action = JunctionAction::Unknown;
                  kk                            = 4;
                  break;
                }
              }
            }
            SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
            for (auto i : section_angles_succ) {
              SD_EXTRACT_MAP_ELEMENT << "i.id" << i.section->id;
              SD_EXTRACT_MAP_ELEMENT << "i.angle" << i.angle;
            }

            SD_EXTRACT_MAP_ELEMENT << "kk" << kk;
            SD_EXTRACT_MAP_ELEMENT << "junction_info.junction_action:" << StrJunctionAction(junction_info.junction_action);
            SD_EXTRACT_MAP_ELEMENT << "angle_val_start_end" << angle_val_start_end.value();
          }
        }
      }
    }

    junction_info.main_road_lane_nums   = GetSectionLaneNum(route_sections, idx_start);
    junction_info.target_road_lane_nums = GetSectionLaneNum(route_sections, idx_end, false);
  }
}

void SDMapElementExtract::FillJunctionInfo_CrossRoad(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info) {
  const std::size_t idx_start = junction_info.idx_start;
  const std::size_t idx_end   = junction_info.idx_end;

  auto action_angle = AngleDiffBetweenSection(route_sections[idx_start], route_sections[idx_end], false);
  for (std::size_t idx = idx_start; idx < idx_end; idx++) {
    if (auto action_angle_t = AngleDiffBetweenSection(route_sections[idx], route_sections[idx + 1]); action_angle_t) {
      if (!action_angle || std::fabs(*action_angle) < std::fabs(action_angle_t.value())) {
        action_angle = action_angle_t;
      }
    }
  }
  /*
    pjc
    1.first judge action is or not Uturn :find no crosslink mpp_section,Calculate the angle,if fabs(angle)-180<15 or fabs(angle)-180>-15,is uturn.
    2.judge TUrn right or turn left or go straight

  */

  //judge action is or not uturn
  std::size_t kk=0;
  std::size_t section_num=idx_start;
  for( int i  = idx_start+1;i<route_sections.size();i++)
  {
    section_num = i;
     if(!LinkTypeMaskType(route_sections[i].link_type,SDLinkTypeMask::SDLT_CROSSLINK))
     {
       break;
     }
  }
  bool not_Calculate_Uturn_Angle = false;
  if (idx_start < route_sections.size() && route_sections[idx_start].length >= 150) {
    not_Calculate_Uturn_Angle = true;
  }
  auto angle_val_start_end = AngleDiffBetweenSection(route_sections[idx_start], route_sections[section_num] ,true, not_Calculate_Uturn_Angle);
  std::vector<AngleJunction> section_angles_succ = std::vector<AngleJunction>{};
  if (angle_val_start_end.has_value()) {
      const auto section_end_id = route_sections[section_num].id;
    if (map_sd_sections_.count(section_end_id)) {
      section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[section_end_id], *angle_val_start_end});
    }
    double angle = angle_val_start_end.value();
    SD_EXTRACT_MAP_ELEMENT<<"base_section_id:"<<route_sections[idx_start].id;
    SD_EXTRACT_MAP_ELEMENT<<"section_id:"<<section_end_id;
    SD_EXTRACT_MAP_ELEMENT<<"angle_val_start_end:"<<angle;
    if (angle >= 160 || angle <= -170) {
      junction_info.junction_action = JunctionAction::UTurn;
    } else {
      if (route_sections[idx_start].successor_section_id_list.size() != 0) {
        if (route_sections[idx_start].successor_section_id_list.size() >= 2) {
          for (int i = idx_start; i < idx_end; i++) {
            for (const auto &id_succ : route_sections[i].successor_section_id_list) {

              if (map_sd_sections_.count(id_succ) != 0  &&
                  map_sd_sections_[id_succ]->id != route_sections[i + 1].id) {

                auto angle_val =
                    AngleDiffBetweenSection(route_sections[idx_start], *map_sd_sections_[id_succ], true, not_Calculate_Uturn_Angle);
                section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[id_succ], *angle_val});
              }
            }
          }


        } else {
          if (idx_start + 1 < route_sections.size() && route_sections[idx_start].successor_section_id_list.size() == 1 &&
              route_sections[idx_start].successor_section_id_list[0] == route_sections[idx_start + 1].id) {
            if ( angle_val_start_end.value() < 30 && angle_val_start_end.value() > -30) {
              junction_info.junction_action = JunctionAction::GoStraight;

            } else if (angle_val_start_end.has_value() && angle_val_start_end.value() < -30) {
              junction_info.junction_action = JunctionAction::TurnRight;
            } else if (angle_val_start_end.has_value() && angle_val_start_end.value() > 30) {
              junction_info.junction_action = JunctionAction::TurnLeft;
            }
          } else {
            junction_info.junction_action = JunctionAction::Unknown;
          }
        }
        if (junction_info.junction_id == 15270229220) {
          SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
          SD_EXTRACT_MAP_ELEMENT << "kk" << kk;
        }
        section_angles_succ.erase(std::remove_if(section_angles_succ.begin(), section_angles_succ.end(),
                                                 [](const auto &rhs) { return std::fabs(rhs.angle) > 160 ;}),
                                  section_angles_succ.end());
        auto section_angles_succ_nomal = section_angles_succ;
        std::sort(section_angles_succ.begin(), section_angles_succ.end(),
                  [&](const auto &a, const auto &b) { return fabs(a.angle) < fabs(b.angle); });
        std::sort(section_angles_succ_nomal.begin(), section_angles_succ_nomal.end(),
                  [&](const auto &a, const auto &b) { return (a.angle) > (b.angle); });

        for (int i = 0; i < section_angles_succ.size(); i++) {
          if ( idx_end < route_sections.size() &&
              route_sections[idx_end].id == section_angles_succ[i].section->id) {
            if (i == 0 && fabs(angle_val_start_end.value()) < 45.0) {
              junction_info.junction_action = JunctionAction::GoStraight;
              kk                            = 1;
              break;
            } else if (angle_val_start_end.value() > 0 && section_angles_succ_nomal.size() &&
                       route_sections[idx_end].id == section_angles_succ_nomal[0].section->id) {
              junction_info.junction_action = JunctionAction::TurnLeft;
              kk                            = 2;
              break;

            } else if (angle_val_start_end.value() < 0 && section_angles_succ_nomal.size() &&
                       route_sections[idx_end].id == section_angles_succ_nomal.back().section->id) {
              junction_info.junction_action = JunctionAction::TurnRight;
              kk                            = 3;
              break;
            } else {
              junction_info.junction_action = JunctionAction::Unknown;
              kk                            = 4;
              break;
            }
          }
        }
        SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
        SD_EXTRACT_MAP_ELEMENT << "kk:" << kk;
      }
    }
  }
  // if (action_angle) {
  //   if (action_angle.value() > Angle_45 && action_angle.value() <= Angle_140) {
  //     junction_info.junction_action = JunctionAction::TurnLeft;
  //   } else if (action_angle.value() < -Angle_45 && action_angle.value() > -Angle_140) {
  //     junction_info.junction_action = JunctionAction::TurnRight;
  //   } else if (action_angle.value() <= -Angle_140 || action_angle.value() >= Angle_140) {
  //     junction_info.junction_action = JunctionAction::UTurn;
  //   }
  // }
  junction_info.main_road_lane_nums   = GetSectionLaneNum(route_sections, idx_start);
  junction_info.target_road_lane_nums = GetSectionLaneNum(route_sections, idx_end, false);

  auto &arrow_vec         = junction_info.map_lane_arrows_plan;
  auto &arrow_vec_default = junction_info.map_lane_arrows_default;
  auto  val               = FindValidLaneGroups(route_sections, idx_start);
  if (val) {
    for (const auto &lane : val.value()) {
      arrow_vec.emplace_back(lane.plan_turn_type);
      arrow_vec_default.emplace_back(lane.turn_type);
    }
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("cross_action_angle:{:.2f} action:{} lane_id:{} start_id:{}", action_angle.value(),
                                        StrJunctionAction(junction_info.junction_action),
                                        val && !val.value().empty() ? val.value().front().id : 0, route_sections[idx_start].id);
  if (action_angle) {
    CheckMainLaneIndex(action_angle.value(), junction_info);                                
  }
}

void SDMapElementExtract::FillJunctionInfo_T(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info) {
  const std::size_t idx_start = junction_info.idx_start;
  const std::size_t idx_end   = junction_info.idx_end;
  bool not_Calculate_Uturn_Angle{false};
  std::size_t kk=0;
  if (idx_start < route_sections.size() && route_sections[idx_start].length >= 150) {
    not_Calculate_Uturn_Angle = true;
  }
  if (route_sections[idx_start].successor_section_id_list.size() == (size_t)2) {
    for (auto succ_id : route_sections[idx_start].successor_section_id_list) {
      if (map_sd_sections_.count(succ_id)) {
        if (map_sd_sections_[succ_id]->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
          not_Calculate_Uturn_Angle = true;
          break;
        }
      }
    }
  }
  auto angle_val_start_end = AngleDiffBetweenSection(route_sections[idx_start], route_sections[idx_end] ,true, not_Calculate_Uturn_Angle);
  std::vector<AngleJunction> section_angles_succ = std::vector<AngleJunction>{};
  if (angle_val_start_end.has_value()) {
      junction_info.next_angle = angle_val_start_end.value();
      const auto section_end_id = route_sections[idx_end].id;
    if (map_sd_sections_.count(section_end_id)) {
      section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[section_end_id], *angle_val_start_end});
    }
    double angle = angle_val_start_end.value();
    SD_EXTRACT_MAP_ELEMENT << "base_section_id:" << route_sections[idx_start].id;
    SD_EXTRACT_MAP_ELEMENT << "section_id:" << section_end_id;
    SD_EXTRACT_MAP_ELEMENT << "angle_val_start_end:" << angle;
    if (angle >= 160 || angle <= -170) {
      junction_info.junction_action = JunctionAction::UTurn;
    } else {
      if (route_sections[idx_start].successor_section_id_list.size() != 0) {
        if (route_sections[idx_start].successor_section_id_list.size() >= 2) {
          for (int i = idx_start; i < idx_end; i++) {
            for (const auto &id_succ : route_sections[i].successor_section_id_list) {

              if (map_sd_sections_.count(id_succ) != 0  &&
                  map_sd_sections_[id_succ]->id != route_sections[i + 1].id) {

                auto angle_val =
                    AngleDiffBetweenSection(route_sections[idx_start], *map_sd_sections_[id_succ], true, not_Calculate_Uturn_Angle);
                section_angles_succ.emplace_back((AngleJunction){map_sd_sections_[id_succ], *angle_val});
              }
            }
          }
        } else {
          if (idx_start + 1 < route_sections.size() && route_sections[idx_start].successor_section_id_list.size() == 1 &&
              route_sections[idx_start].successor_section_id_list[0] == route_sections[idx_start + 1].id) {
            if ( angle_val_start_end.value() < 30 && angle_val_start_end.value() > -30) {
              junction_info.junction_action = JunctionAction::GoStraight;

            } else if (angle_val_start_end.has_value() && angle_val_start_end.value() < -30) {
              junction_info.junction_action = JunctionAction::TurnRight;
            } else if (angle_val_start_end.has_value() && angle_val_start_end.value() > 30) {
              junction_info.junction_action = JunctionAction::TurnLeft;
            }
          } else {
            junction_info.junction_action = JunctionAction::Unknown;
          }
        }
        // if (junction_info.junction_id == 15270229220) {
        //   SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
        //   SD_EXTRACT_MAP_ELEMENT << "kk" << kk;
        // }
        section_angles_succ.erase(std::remove_if(section_angles_succ.begin(), section_angles_succ.end(),
                                                 [](const auto &rhs) { return std::fabs(rhs.angle) > 160 ;}),
                                  section_angles_succ.end());
        auto section_angles_succ_nomal = section_angles_succ;
        std::sort(section_angles_succ.begin(), section_angles_succ.end(),
                  [&](const auto &a, const auto &b) { return fabs(a.angle) < fabs(b.angle); });
        std::sort(section_angles_succ_nomal.begin(), section_angles_succ_nomal.end(),
                  [&](const auto &a, const auto &b) { return (a.angle) > (b.angle); });

        for (int i = 0; i < section_angles_succ.size(); i++) {
          if (idx_end < route_sections.size() &&
              route_sections[idx_end].id == section_angles_succ[i].section->id) {
            if (i == 0 && fabs(angle_val_start_end.value()) < 45.0) {
              junction_info.junction_action = JunctionAction::GoStraight;
              kk                            = 1;
              break;
            } else if (angle_val_start_end.value() > 0 && section_angles_succ_nomal.size() &&
                       route_sections[idx_end].id == section_angles_succ_nomal[0].section->id) {
              junction_info.junction_action = JunctionAction::TurnLeft;
              kk                            = 2;
              break;

            } else if (angle_val_start_end.value() < 0 && section_angles_succ_nomal.size() &&
                       route_sections[idx_end].id == section_angles_succ_nomal.back().section->id) {
              junction_info.junction_action = JunctionAction::TurnRight;
              kk                            = 3;
              break;
            } else {
              junction_info.junction_action = JunctionAction::Unknown;
              kk                            = 4;
              break;
            }
          }
        }
        if (junction_info.junction_id == 148845893) {
          SD_EXTRACT_MAP_ELEMENT << "section_angles_succ" << section_angles_succ.size();
          for (auto i : section_angles_succ) {
            SD_EXTRACT_MAP_ELEMENT << "i.id" << i.section->id;
            SD_EXTRACT_MAP_ELEMENT << "i.angle" << i.angle;
          }

          SD_EXTRACT_MAP_ELEMENT << "kk" << kk;
          SD_EXTRACT_MAP_ELEMENT << "angle_val_start_end" << angle_val_start_end.value();
        }
      }
    }
  }
  junction_info.main_road_lane_nums   = GetSectionLaneNum(route_sections, idx_start);
  junction_info.target_road_lane_nums = GetSectionLaneNum(route_sections, idx_end, false);

  auto &arrow_vec         = junction_info.map_lane_arrows_plan;
  auto &arrow_vec_default = junction_info.map_lane_arrows_default;
  auto  val               = FindValidLaneGroups(route_sections, idx_start);
  if (val) {
    for (const auto &lane : val.value()) {
      arrow_vec.emplace_back(lane.plan_turn_type);
      arrow_vec_default.emplace_back(lane.turn_type);
    }
  }
}

void SDMapElementExtract::FillJunctionInfo_Split(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info) {
  const std::size_t idx_start = junction_info.idx_start;
  const std::size_t idx_end   = junction_info.idx_end;

  auto junctions_angle = JunctionAnglesAroundJunction(route_sections, idx_start, 1);
  junctions_angle.erase(std::remove_if(junctions_angle.begin(), junctions_angle.end(),
                                       [](const AngleJunction &rhs) { return !rhs.section || std::fabs(rhs.angle) > Angle_75; }),
                        junctions_angle.end());
  if (junctions_angle.empty()) {
    junction_info.junction_type_city = JunctionTypeCity::Unknown;
    return;
  }
  for (const AngleJunction &obj : junctions_angle) {
    junction_info.split_direction_ids.emplace_back(obj.section->id);
  }
  junction_info.junction_action = JunctionAction::GoStraight;
  if (junctions_angle.front().section->id == route_sections[idx_end].id) {
    junction_info.split_merge_direction = DirectionSplitMerge::Left;
  } else if (junctions_angle.back().section->id == route_sections[idx_end].id) {
    junction_info.split_merge_direction = DirectionSplitMerge::Right;
  } else {
    junction_info.split_merge_direction = DirectionSplitMerge::Straight;
  }
  const auto &lane_group              = LaneGroups(route_sections[idx_start]);
  junction_info.main_road_lane_nums   = GetSectionLaneNum(route_sections, idx_start);
  junction_info.target_road_lane_nums = GetSectionLaneNum(route_sections, idx_end, false);
  auto &arrow_vec                     = junction_info.map_lane_arrows_plan;
  auto &arrow_vec_default             = junction_info.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }

  junction_info.is_left_ahead_of_time = LinkTypeMaskType(route_sections[idx_end].link_type, SDLinkTypeMask::SDLT_LEFTTURN);
  junction_info.succ_road_class       = JunctionsMainRamp(junctions_angle, route_sections[idx_start]);
  if (junction_info.succ_road_class.size() == junctions_angle.size()) {
    int right_idx = -1;
    for (std::size_t idx = 0; idx < junctions_angle.size(); idx++) {
      if (junctions_angle[idx].section && LinkTypeMaskType(junctions_angle[idx].section->link_type, SDLinkTypeMask::SDLT_RIGHTTURN)) {
        SD_EXTRACT_MAP_ELEMENT << fmt::format("orin_type:{}", junction_info.succ_road_class[idx]);
        right_idx                          = static_cast<int>(idx);
        junction_info.succ_road_class[idx] = RoadMainType::RoadRamp;
      }
    }
    if (junctions_angle.size() == 2 && right_idx != -1) {
      if (right_idx == 1) {
        junction_info.succ_road_class[0] = RoadMainType::RoadMain;
      } else {
        junction_info.succ_road_class[1] = RoadMainType::RoadMain;
      }
    }
  }
  double      min_angle = std::numeric_limits<double>::infinity();
  std::size_t min_idx   = 0;
  std::size_t next_idx  = 0;
  for (std::size_t i = 0; i < junctions_angle.size(); i++) {
    if (std::fabs(min_angle) > std::fabs(junctions_angle[i].angle)) {
      min_angle = junctions_angle[i].angle;
      min_idx   = i;
    }
    if (junctions_angle[i].section->id == route_sections[idx_end].id) {
      next_idx = i;
    }
  }
  junction_info.junction_action = JunctionAction::GoStraight;
  if (next_idx < min_idx) {
    junction_info.junction_action = JunctionAction::TurnLeft;
  } else if (next_idx > min_idx) {
    junction_info.junction_action = JunctionAction::TurnRight;
  }
  auto IsDedicatedRight = [&](const SDSectionInfo &rhs) {
    const auto &lanes_begin = LaneGroups(rhs, false);
    const auto &lanes_back  = LaneGroups(rhs);
    return std::any_of(lanes_begin.begin(), lanes_begin.end(),
                       [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_RIGHT_TURN_LANE; }) ||
           std::any_of(lanes_back.begin(), lanes_back.end(),
                       [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_RIGHT_TURN_LANE; });
  };

  for (const auto &angle : junctions_angle) {
    if (angle.angle < 0.0 && angle.section && IsDedicatedRight(*angle.section)) {
      junction_info.is_dedicated_right_turn_lane = true;
      break;
    }
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("split_section:{}.  is_dedicated_right_turn_lane:{:d}  ids:{}  left_ahead:{:d}",
                                        junction_info.junction_id, junction_info.is_dedicated_right_turn_lane,
                                        junction_info.split_direction_ids, junction_info.is_left_ahead_of_time);
}

void SDMapElementExtract::FillJunctionInfo_Merge(const std::vector<SDSectionInfo> &route_sections, JunctionInfoCity &junction_info) {
  const std::size_t idx_start = junction_info.idx_start;
  const std::size_t idx_end   = junction_info.idx_end;

  auto junctions_angle = JunctionAnglesAroundJunction(route_sections, idx_end, 2);
  if (junctions_angle.size() < 2) {
    return;
  }
  for (auto k : junctions_angle) {
    SD_EXTRACT_MAP_ELEMENT << "id:" << k.section->id << "angle:" << k.angle;
  }
  std::reverse(junctions_angle.begin(), junctions_angle.end());
  for (auto k : junctions_angle) {
    SD_EXTRACT_MAP_ELEMENT << "id:" << k.section->id << "angle:" << k.angle;
  }
  junction_info.junction_action       = JunctionAction::GoStraight;
  const auto &lane_group              = LaneGroups(route_sections[idx_start]);
  junction_info.main_road_lane_nums   = GetSectionLaneNum(route_sections, idx_start);
  float max_angle = Angle_180;
  const SDSectionInfo* main_section = nullptr;
  for (auto main_road : junctions_angle) {
    if (fabs(main_road.angle) < 90) {
      main_road.angle = 180 - fabs(main_road.angle);
    }
    if (180 - fabs(main_road.angle) < (max_angle)) {
      max_angle    = 180 - fabs(main_road.angle);
      main_section = main_road.section;
    }
  }
  if (main_section) {
    const auto &main_road_lane_group = LaneGroups(*main_section);
    std::size_t main_road_lane_num           = main_road_lane_group.size();
    for (auto lane_info : main_road_lane_group) {
      if (lane_info.type == LaneType::LANE_NON_MOTOR || lane_info.type == LaneType::LANE_DIVERSION) {
        main_road_lane_num--;
      }
    }
    main_road_lane_num                                   = main_road_lane_num != 0 ? main_road_lane_num : main_section->lane_num;
    junction_info.road_merge_main_road_lane_nums  = main_road_lane_num;
    SD_EXTRACT_MAP_ELEMENT << "main_section->lane_num:" << main_section->lane_num;
    SD_EXTRACT_MAP_ELEMENT << "main_road_lane_num:" << main_road_lane_num;
    auto &prev_road_class = junction_info.prev_road_class ;
    for (auto main_road : junctions_angle) {
      if (main_road.section->id == main_section->id) {
        prev_road_class.emplace_back(RoadMainType::RoadMain);
        SD_EXTRACT_MAP_ELEMENT << "section_id:" << main_section->id << ":roadtype:RoadMain";
      } else {
        prev_road_class.emplace_back(RoadMainType::RoadRamp);
        SD_EXTRACT_MAP_ELEMENT << "section_id:" << main_road.section->id << ":roadtype:RoadRamp";
      }
    }
  }
  junction_info.target_road_lane_nums = GetSectionLaneNum(route_sections, idx_end, false);
  SD_EXTRACT_MAP_ELEMENT << "targe_num:" << junction_info.target_road_lane_nums << "  next_id:" << route_sections[idx_end].id;
  auto &arrow_vec         = junction_info.map_lane_arrows_plan;
  auto &arrow_vec_default = junction_info.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }

  if (junctions_angle.front().section->id == route_sections[idx_start].id) {
    junction_info.split_merge_direction = DirectionSplitMerge::Left;
  } else if (junctions_angle.back().section->id == route_sections[idx_start].id) {
    junction_info.split_merge_direction = DirectionSplitMerge::Right;
  } else {
    junction_info.split_merge_direction = DirectionSplitMerge::Straight;
  }
}
//pjc
void SDMapElementExtract::JudgeTjunctionIsOnlyUturn(std::vector<JunctionInfoCity>    &junctions_info_,
                                                    const std::vector<SDSectionInfo> &sd_route_sections) {
  for (auto &junction : junctions_info_) {
    SD_EXTRACT_MAP_ELEMENT << "junctions_info_.size()" << junctions_info_.size();
    if (junction.junction_type_city == JunctionTypeCity::TJunction) {

      SD_EXTRACT_MAP_ELEMENT << "unction.junction_type_city == JunctionTypeCity::TJunction()";

      uint64_t junction_section_id = junction.junction_id;  //rout_section[idx]
      size_t   idx                 = junction.idx_start;
      size_t   idx_next            = junction.idx_end;
      if (idx >= sd_route_sections.size()) {
        continue;
      }
      SDSectionInfo SD_section_info      = sd_route_sections[idx];
      SDSectionInfo SD_next_section_info = sd_route_sections[idx_next];
      SD_EXTRACT_MAP_ELEMENT << "IDX" << idx << "sd_route_sections.size()" << sd_route_sections.size() << "junction.junction_id:" << junction.junction_id
            << "SD_section_info" << SD_section_info.id;
      std::vector<uint64_t> temp;
      temp.clear();
      if (junction_section_id == SD_section_info.id) {
        for (auto successor_id : SD_section_info.successor_section_id_list) {
          if (successor_id != SD_next_section_info.id) {
            temp.emplace_back(successor_id);
          }
        }
        for (int i = 0; i < static_cast<int>(temp.size()); i++) {
          auto Uturnsection = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(temp[i]);
          if (Uturnsection != nullptr) {
            if (static_cast<int>(Uturnsection->lane_group_idx.size()) > 0) {
              auto size_lane_group = static_cast<int>(Uturnsection->lane_group_idx.size()) - 1;
              auto lane_group_id   = Uturnsection->lane_group_idx[size_lane_group].id;
              int  lanegroup_size  = map_sd_lane_groups_info_.count(lane_group_id);
              if (lanegroup_size > 0) {
                int lane_size = map_sd_lane_groups_info_[lane_group_id]->lane_info.size();
                if (lanegroup_size > 0 && lane_size > 0) {
                  auto lanes_turn_type = map_sd_lane_groups_info_[lane_group_id]->lane_info[lane_size - 1].type;
                  if (lanes_turn_type == LaneType::LANE_U_TURN_LANE) {
                    junction.is_only_Turn = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }
}

void SDMapElementExtract::JudgeIsNeedMatchArrow(std::vector<JunctionInfoCity> &junctions_info) {
  for (auto &junction : junctions_info) {
    if (junction.junction_type_city == JunctionTypeCity::RoadSplit || junction.junction_type_city == JunctionTypeCity::RoadMerge) {
      continue;
    }
    uint32_t junction_action_val = GetActionTurnVal(junction.junction_action);
    bool     all_lane_include_no_turn{true};
    for (const auto &turn_type_t : junction.map_lane_arrows_plan) {
      uint32_t turn_val = GetTurnTypeVal(turn_type_t);
      if (junction_action_val != 0b0001 || ((junction_action_val & turn_val) == 0U)) {
        all_lane_include_no_turn = false;
      }
    }
    if (all_lane_include_no_turn) {
      SD_EXTRACT_MAP_ELEMENT << "junction_id:" << junction.junction_id << "  no_need_match_arrow.";
      junction.need_match_arrow = false;
    }
  }
  // junctions_info_.erase(
  //     std::remove_if(junctions_info_.begin(), junctions_info_.end(), [](const JunctionInfoCity &rhs) { return !rhs.need_match_arrow; }),
  //     junctions_info_.end());
}

void SDMapElementExtract::DebugSdRouteInfo() {
  SD_EXTRACT_MAP_ELEMENT << fmt::format("sd_route_id:{}  navi_start section_id:{}  s_offset:{:.2f}", routing_map_->sd_route.id,
                                        routing_map_->sd_route.navi_start.section_id, routing_map_->sd_route.navi_start.s_offset);
  // for (const auto &sd_section : routing_map_->sd_route.mpp_sections) {
  // SD_EXTRACT_MAP_ELEMENT << fmt::format(
  //     "section id:{}  length:{} lane_num:{} point_size:{}  link_type:{} road_class:{} prev:{} succ:{} speed_limit:{:.2f}  "
  //     "has_toll_station:{:d} direction:{} is_mpp_section:{:d} has_junction:{:d} lane_group_index:{} \n",
  //     sd_section.id, sd_section.length, sd_section.lane_num, sd_section.points ? sd_section.points->size() : 0,
  //     cem::message::env_model::StrLinkType(sd_section.link_type), StrSDRoadClass(sd_section.road_class),
  //     sd_section.predecessor_section_id_list, sd_section.successor_section_id_list, sd_section.speed_limit, sd_section.has_toll_station,
  //     StrSDDirectionType(sd_section.direction), sd_section.is_mpp_section, sd_section.has_junction, sd_section.lane_group_idx);
  // }
  SD_EXTRACT_MAP_ELEMENT << "\n\n";
  // for (const auto &sd_subpath : routing_map_->sd_route.subpaths) {
  //   SD_EXTRACT_MAP_ELEMENT << fmt::format("enter_section_id:{} section_size:{}", sd_subpath.enter_section_id, sd_subpath.sections.size());
  //   fmt::memory_buffer buf;
  //   fmt::format_to(buf, "sub_section_info ");
  //   for (const auto &sd_section : sd_subpath.sections) {
  //     fmt::format_to(buf,
  //                    "section id:{}  length:{} lane_num:{} point_size:{}  link_type:{} road_class:{} prev:{} succ:{} speed_limit:{:.2f}  "
  //                    "has_toll_station:{:d} direction:{} is_mpp_section:{:d} has_junction:{:d} lane_group_index:{}",
  //                    sd_section.id, sd_section.length, sd_section.lane_num, sd_section.points ? sd_section.points->size() : 0,
  //                    StrLinkType(sd_section.link_type), StrSDRoadClass(sd_section.road_class), sd_section.predecessor_section_id_list,
  //                    sd_section.successor_section_id_list, sd_section.speed_limit, sd_section.has_toll_station,
  //                    StrSDDirectionType(sd_section.direction), sd_section.is_mpp_section, sd_section.has_junction,
  //                    sd_section.lane_group_idx);
  //   }
  //   SD_EXTRACT_MAP_ELEMENT << std::string_view(buf.data(), buf.size()) << "\n\n";
  // }
  // for (const auto &lane_group : routing_map_->sd_lane_groups) {
  //   bool is_lane_equal = lane_group.lane_num == lane_group.lane_info.size();
  //   SD_EXTRACT_MAP_ELEMENT << fmt::format("lane_group_id:{} length:{:.2f} succ:{}  prev:{} lane_num:{} lane_num_equal:{:d}", lane_group.id,
  //                                         lane_group.length, lane_group.successor_lane_group_ids, lane_group.predecessor_lane_group_ids,
  //                                         lane_group.lane_num, is_lane_equal);
  //   for (const auto &lane_info : lane_group.lane_info) {
  //     SD_EXTRACT_MAP_ELEMENT << fmt::format("{}", lane_info);
  //   }
  //   SD_EXTRACT_MAP_ELEMENT << "\n\n";
  // }
}

void SDMapElementExtract::FillMapsInfo() {
  for (const auto &lane_group : routing_map_->sd_lane_groups) {
    map_sd_lane_groups_info_.insert({lane_group.id, &lane_group});
    // for (const auto &lane_info : lane_group.lane_info) {
    //   map_sd_lanes_info_.insert({lane_info.id, std::make_shared<const LaneInfo>(lane_info)});
    // }
  }
  // fmt::memory_buffer buf;
  // fmt::format_to(buf, "map_sd_sections:");
  for (const auto &sd_section : routing_map_->sd_route.mpp_sections) {
    if (!sd_section.points || sd_section.points->size() < 2) {
      continue;
    }
    map_sd_sections_.insert({sd_section.id, &sd_section});
    // fmt::format_to(buf, "{},", sd_section.id);
  }
  // SD_EXTRACT_MAP_ELEMENT << std::string_view(buf.data(), buf.size());
  // buf.clear();
  // fmt::format_to(buf, "map_sd_subpath:");
  for (const auto &sd_subpath : routing_map_->sd_route.subpaths) {
    //SdSubPathInfo        sub_path_info;
    static constexpr int invalid_road_class = 7;
  //  sub_path_info.sub_path_                 = std::make_shared<const SDSubPath>(sd_subpath);
    for (const auto &sd_section : sd_subpath.sections) {
      if (static_cast<int>(sd_section.road_class) >= invalid_road_class || !sd_section.points || sd_section.points->size() < 2) {
        continue;
      }
     // auto ptr_t = std::make_shared<const SDSectionInfo>(sd_section);
      map_sd_sections_.insert({sd_section.id, &sd_section});
      // fmt::format_to(buf, "{},", sd_section.id);
   //   sub_path_info.sd_sction_infos_.emplace_back(ptr_t);
    }
    // if (sub_path_info.sd_sction_infos_.empty()) {
    //   continue;
    // }
    // map_sd_subpaths_.insert({sd_subpath.enter_section_id, std::move(sub_path_info)});
    // fmt::format_to(buf, "{},", sd_subpath.enter_section_id);
  }
  // SD_EXTRACT_MAP_ELEMENT << std::string_view(buf.data(), buf.size());
}

void SDMapElementExtract::SetSdRecommendLane() {
  const auto sd_map_info = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_map_info) {
    // SD_FINE_MATCH_LOG << "sd_map_info is null";
    return;
  }

  const auto &mpp_sections        = sd_map_info->mpp_sections;
  const auto &recommend_lane_list = sd_map_info->recommend_lane_list;
  if (mpp_sections.empty() || recommend_lane_list.empty()) {
    SD_FINE_MATCH_LOG << "mpp_sections or recommend_lane_list is empty";
    return;
  }

  const SDSectionInfo *ego_section    = nullptr;
  uint64_t             ego_section_id = sd_map_info->navi_start.section_id;
  ego_section                         = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(ego_section_id);
  int ego_section_idx                 = -1;

  if ((ego_section != nullptr) && (ego_section->is_mpp_section)) {
    for (size_t i = 0; i < mpp_sections.size(); i++) {
      if (mpp_sections[i].id == ego_section->id) {
        ego_section_idx = static_cast<int>(i);
        break;
      }
    }
  }

  if ((ego_section == nullptr) || (ego_section_idx == -1)) {
    // SD_FINE_MATCH_LOG << "ego_section not found";
    return;
  }

  double ego_s_offset = sd_map_info->navi_start.s_offset;
  double ego_s_global = 0.0;
  for (int i = 0; i < ego_section_idx; ++i) {
    ego_s_global += mpp_sections[i].length;
  }
  ego_s_global += ego_s_offset;

  double min_s      = -50;
  double max_s      = 200;
  double current_s  = 0.0;
  bool   start_once = true;
  double current_lg_s;
  for (size_t i = 0; i < mpp_sections.size(); ++i) {
    const auto &section         = mpp_sections[i];
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    if (section_end_s < ego_s_global + min_s || section_start_s > ego_s_global + max_s) {
      current_s += section.length;
      continue;
    }
    SD_FINE_MATCH_LOG << "--------------------------------";
    SD_FINE_MATCH_LOG << "Section ID: " << section.id << " start from s: " << section_start_s << " to: " << section_end_s;

    if (start_once) {
      current_lg_s = section_start_s;
      start_once   = false;
    }

    for (const auto &lg_idx : section.lane_group_idx) {
      const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (!lane_group) {
        continue;
      }
      uint64_t lg_id          = lg_idx.id;
      double   lg_start_s     = current_lg_s;
      double   lg_end_s       = current_lg_s + lane_group->length;
      double   lane_start_ego = lg_start_s - ego_s_global;
      double   lane_end_ego   = lg_end_s - ego_s_global;
      if ((lane_end_ego < min_s) || (lane_start_ego > max_s)) {
        current_lg_s += lane_group->length;
        continue;
      }

      SD_FINE_MATCH_LOG << "LaneGroup ID: " << lg_id << " start from s: " << lg_start_s << " to: " << lg_end_s
                        << ", lane size: " << lane_group->lane_info.size();

      SD_FINE_MATCH_LOG << "Lane group in ego start, end: " << lane_start_ego << ", " << lane_end_ego;

      SD_FINE_MATCH_LOG << "recommend_lane list size: " << recommend_lane_list.size();

      auto recommend_lane_info =
          std::find_if(recommend_lane_list.begin(), recommend_lane_list.end(),
                       [lg_id](const SDRecommendLaneGroup &recommend_lane_list) { return recommend_lane_list.lane_group_id == lg_id; });
      if (recommend_lane_info != recommend_lane_list.end()) {
        for (const auto &recommend_lane : recommend_lane_info->recommend_lane) {
          cem::fusion::navigation::RecommendLane recommend_info;
          recommend_info.lanegroup_id = lg_id;
          recommend_info.lane_num     = recommend_lane.lane_num;
          double rm_start_ego         = lg_start_s + recommend_lane.start_offset - ego_s_global;
          double rm_end_ego           = lg_start_s + recommend_lane.end_offset - ego_s_global;
          recommend_info.start_offset = rm_start_ego;
          recommend_info.end_offset   = rm_end_ego;
          recommend_info.lane_seqs    = recommend_lane.lane_seqs;
          SD_FINE_MATCH_LOG << "recommend_lane start end ego: " << rm_start_ego << ", " << rm_end_ego;

          for (auto &lane_seq : recommend_lane.lane_seqs) {
            SD_FINE_MATCH_LOG << "recommend_lane lane_seq: " << lane_seq;
          }
          map_recommend_lane_[rm_start_ego] = recommend_info;
        }
      }
      current_lg_s += lane_group->length;
    }
    current_s += section.length;
  }

  // for (const auto &recommend_lane : map_recommend_lane_) {
  //   const double rm_start_ego   = recommend_lane.first;
  //   const auto  &recommend_info = recommend_lane.second;
  //   SD_FINE_MATCH_LOG << "lane_goup_id = " << recommend_info.lanegroup_id << ", lane_start_offset = " << recommend_info.start_offset
  //                     << ", lane_end_offset = " << recommend_info.end_offset << ", lane_num = " << recommend_info.lane_num;
  //   for (const auto &lane_id : recommend_info.lane_seqs) {
  //     SD_FINE_MATCH_LOG << "lane_seqs: " << lane_id;
  //   }
  // }

  if (map_recommend_lane_.empty())
    return;
  auto it      = map_recommend_lane_.begin();
  auto prev_it = it++;
  //qu chong
  for (; it != map_recommend_lane_.end();) {
    const RecommendLane &prev_lane = prev_it->second;
    const RecommendLane &curr_lane = it->second;
    if (prev_lane.lane_num == curr_lane.lane_num && prev_lane.lane_seqs == curr_lane.lane_seqs) {
      it = map_recommend_lane_.erase(it);
    } else {
      prev_it = it++;
    }
  }

  SD_FINE_MATCH_LOG << "Merge Duplicate RecommendLanes";
  // for (const auto &recommend_lane : map_recommend_lane_) {
  //   const double rm_start_ego   = recommend_lane.first;
  //   const auto  &recommend_info = recommend_lane.second;
  //   SD_FINE_MATCH_LOG << "lane_goup_id = " << recommend_info.lanegroup_id << ", lane_start_offset = " << recommend_info.start_offset
  //                     << ", lane_end_offset = " << recommend_info.end_offset << ", lane_num = " << recommend_info.lane_num;
  //   for (const auto &lane_id : recommend_info.lane_seqs) {
  //     SD_FINE_MATCH_LOG << "lane_seqs: " << lane_id;
  //   }
  // }
  auto &navi_debug_infos             = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_infos.sd_recommend_lane = map_recommend_lane_;
}

void SDMapElementExtract::Reset() {
  //routing_map_.reset();
  map_sd_lane_groups_info_.clear();
 // map_sd_lanes_info_.clear();
  map_sd_sections_.clear();
 // map_sd_subpaths_.clear();
  map_section_angles_.clear();
  junctions_info_.clear();
  map_recommend_lane_.clear();
}

bool SDMapElementExtract::Is2LineCross(const SDSectionInfo                                   &section,
                                       const std::vector<SDMapElementExtract::AngleJunction> &angle_junctions) {
  constexpr double threshold_angle_line = 10.0;
  // constexpr double threshold_same_line_point = 3.0;

  auto it = std::find_if(angle_junctions.begin(), angle_junctions.end(), [](const AngleJunction &rhs) {
    return (IsInDegreeTarget(rhs.angle, 0, threshold_angle_line) || IsInDegreeTarget(rhs.angle, -Angle_180, threshold_angle_line) ||
            IsInDegreeTarget(rhs.angle, Angle_180, threshold_angle_line));
  });
  if (it == angle_junctions.end()) {
    SD_EXTRACT_MAP_ELEMENT << "not_find_line_within_0_degree.";
    return false;
  };
  // auto IsInLine = [](const SDSectionInfo &lhs, const SDSectionInfo &rhs) {
  //   if (lhs.points->size() <= 1 || rhs.points->size() <= 1) {
  //     return false;
  //   }
  //   double delta_x_01 = lhs.points->back().x() - rhs.points->front().x();
  //   double delta_y_01 = lhs.points->back().y() - rhs.points->front().y();
  //   double delta_x_02 = lhs.points->front().x() - rhs.points->back().x();
  //   double delta_y_02 = lhs.points->front().y() - rhs.points->back().y();
  //   double dis_01     = std::sqrt(delta_x_01 * delta_x_01 + delta_y_01 * delta_y_01);
  //   double dis_02     = std::sqrt(delta_x_02 * delta_x_02 + delta_y_02 * delta_y_02);
  // // SD_EXTRACT_MAP_ELEMENT << fmt::format("section_01:{}  section_02:{}  dis_01:{:.2f}  dis_02:{:.2f}", lhs.id, rhs.id, dis_01, dis_02);
  //   return dis_01 < threshold_same_line_point && dis_02 > threshold_same_line_point;
  // };

  for (size_t idx_i = 0; idx_i < angle_junctions.size(); idx_i++) {
    for (size_t idx_j = idx_i + 1; idx_j < angle_junctions.size(); idx_j++) {
      if (((angle_junctions[idx_i].position == AngleJunction::PositionDirection::LEFT &&
            angle_junctions[idx_j].position == AngleJunction::PositionDirection::RIGHT) ||
           (angle_junctions[idx_i].position == AngleJunction::PositionDirection::RIGHT &&
            angle_junctions[idx_j].position == AngleJunction::PositionDirection::LEFT)) &&
          std::fabs(std::fabs(angle_junctions[idx_i].angle - angle_junctions[idx_j].angle) - Angle_180) < threshold_angle_line) {
        SD_EXTRACT_MAP_ELEMENT << fmt::format("Is2LineCross section_i:{}  section_j:{}  angle_i:{:.2f}  angle_j:{:.2f}",
                                              angle_junctions[idx_i].section->id, angle_junctions[idx_j].section->id,
                                              angle_junctions[idx_i].angle, angle_junctions[idx_j].angle);
        return true;
      }
      // if (!IsInDegreeTarget(angle_junctions[idx_i].angle, angle_junctions[idx_j].angle, threshold_angle_line)) {
      //   continue;
      // }
      // if ((IsInLine(*angle_junctions[idx_i].section, *angle_junctions[idx_j].section) ||
      //      IsInLine(*angle_junctions[idx_j].section, *angle_junctions[idx_i].section))) {
      // return true;
      // }
    }
  }

  return false;
}

bool SDMapElementExtract::SeemsOneRoad(const SDSectionInfo &lhs, const SDSectionInfo &rhs) {
  SD_EXTRACT_MAP_ELEMENT <<"id"<<lhs.id<< "lhs.points->size():" << lhs.points->size();
  SD_EXTRACT_MAP_ELEMENT <<"id"<<rhs.id<< "rhs.points->size():" << rhs.points->size();
  if (!lhs.points || !rhs.points || lhs.points->size() < 2 || rhs.points->size() < 2) {
    return false;
  }

  size_t size_lhs = lhs.points->size();
  Vec2d  p1_start_curr{lhs.points->at(0).x(), lhs.points->at(0).y()};
  Vec2d  p1_start_next{lhs.points->at(1).x(), lhs.points->at(1).y()};
  Vec2d  p1_end_curr{lhs.points->at(size_lhs - 1).x(), lhs.points->at(size_lhs - 1).y()};
  Vec2d  p1_end_prev{lhs.points->at(size_lhs - 2).x(), lhs.points->at(size_lhs - 2).y()};

  size_t size_rhs = rhs.points->size();
  Vec2d  p2_start_curr{rhs.points->at(0).x(), rhs.points->at(0).y()};
  Vec2d  p2_start_next{rhs.points->at(1).x(), rhs.points->at(1).y()};
  Vec2d  p2_end_curr{rhs.points->at(size_rhs - 1).x(), rhs.points->at(size_rhs - 1).y()};
  Vec2d  p2_end_next{rhs.points->at(size_rhs - 2).x(), rhs.points->at(size_rhs - 2).y()};

  constexpr double kMaxStartDistance   = 1.0;
  constexpr double kMaxLateralDistance = 0.1;

  auto CheckConnection = [&](const Vec2d &start1, const Vec2d &end1, const Vec2d &start2, const Vec2d &end2) {
    double dis_t = start1.DistanceTo(start2);
    // if (lhs.id == 16642339101) {
    //   SD_EXTRACT_MAP_ELEMENT << fmt::format(
    //       "lhs_id:{}  rhs_id:{}  start_1:{:.2f}-{:.2f}  end_1:{:.2f}-{:.2f}  start_2:{:.2f}-{:.2f} end_2:{:.2f}-{:.2f} dis_t:{:.2f}",
    //       lhs.id, rhs.id, start1.x(), start1.y(), end1.x(), end1.y(), start2.x(), start2.y(), end2.x(), end2.y(), dis_t);
    // }
    if (dis_t >= kMaxStartDistance) {
      return false;
    }
    LineSegment2d seg1(start1, end1);
    auto          lhs_v_x    = end1.x() - start1.x();
    auto          lhs_v_y    = end1.y() - start1.y();
    auto          lhs_length = sqrt(lhs_v_x * lhs_v_x + lhs_v_y * lhs_v_y);
    if (lhs_length > 0) {
      lhs_v_x = lhs_v_x / lhs_length;
      lhs_v_y = lhs_v_y / lhs_length;
    }

    auto   rhs_v_x              = end2.x() - start2.x();
    auto   rhs_v_y              = end2.y() - start2.y();
    double vector_dot_product   = lhs_v_x * rhs_v_x + lhs_v_y * rhs_v_y;  //Vector dot product >0,same direction
    double Vector_cross_product = lhs_v_x * rhs_v_y - rhs_v_x * lhs_v_y;  //平行四边形面积
    bool   res                  = vector_dot_product > 0 && std::fabs(Vector_cross_product) < kMaxLateralDistance;
    SD_EXTRACT_MAP_ELEMENT << fmt::format("lhs_id:{}  rhs_id:{}  vector_dot_product:{:.2f}  Vector_cross_product:{:.5f} issameroad:{:d}",
                                          lhs.id, rhs.id, vector_dot_product, Vector_cross_product, res);

    // double lon_proj = seg1.ProjectOntoUnit(end2);
    // double lat_proj = seg1.ProductOntoUnit(end2);
    // SD_EXTRACT_MAP_ELEMENT << "x0:" << start1.x() << "y0:" << start1.y() << "x1:" << end1.x() << "y1:"
    //                        << end1.y();
    // SD_EXTRACT_MAP_ELEMENT << "x2:" << start1.x() << "y2:" << start1.y() << "x3:" << end2.x() << "y3:"
    //                        << end2.y();
    // auto angle_ = AngleDiffBetweenSection(lhs,rhs);
    // double angle    = INFINITY;
    // if (angle_.has_value()) {
    //   angle = angle_.value();
    // }
    // bool   res      = lon_proj > 0 && std::fabs(lat_proj) < kMaxLateralDistance && fabs(angle)<Angle_15;
    // SD_EXTRACT_MAP_ELEMENT << fmt::format("lhs_id:{}  rhs_id:{}  s:{:.2f}  l:{:.5f} proj_final:{:d} angle:{:.2f}", lhs.id, rhs.id, lon_proj, lat_proj,
    //                                       res,angle);
    return res;
  };

  return CheckConnection(p1_end_curr, p1_end_prev, p2_start_curr, p2_start_next) ||
         CheckConnection(p1_end_curr, p1_end_prev, p2_end_curr, p2_end_next);
}
void SDMapElementExtract::JunctionAngleLDSectionDirection(const cem::message::env_model::SectionInfo &section, std::vector<LDAngleJunction> &res,
                                                        bool normal_direction) {
  constexpr double         later_valid_dis = 0.5;
  size_t                   section_length  = section.points.size();
  if (section_length < 2) {
    return;
  }
  byd::common::math::Vec2d section_end{section.points.at(section_length - 1).x, section.points.at(section_length - 1).y};
  byd::common::math::Vec2d section_end_prev{section.points.at(section_length - 2).x, section.points.at(section_length - 2).y};
  if (!normal_direction) {
    section_end_prev.set_x(section.points.at(0).x);
    section_end_prev.set_y(section.points.at(0).y);
    section_end.set_x(section.points.at(1).x);
    section_end.set_y(section.points.at(1).y);
  }

  const byd::common::math::LineSegment2d line_section{section_end_prev, section_end};
  for (auto &junction_angle : res) {
    double                   dis_t = 0.0;
    byd::common::math::Vec2d point_vec{0.0, 0.0};
    junction_angle.raw_angle = junction_angle.angle;

    auto       &points = junction_angle.section->points;
    const auto &range  = normal_direction ? points : std::vector<cem::message::env_model::Point>(points.rbegin(), points.rend());

    int    idx      = -1;
    double dis_prev = 0.0;
    bool   is_double_direction{false};
    for (const auto &point : range) {
      idx++;
      dis_prev = dis_t;
      point_vec.set_x(point.x);
      point_vec.set_y(point.y);
      dis_t = line_section.ProductOntoUnit(point_vec);
      if (dis_t * dis_prev < -1e-9) {
        is_double_direction = true;
        SD_EXTRACT_MAP_ELEMENT << fmt::format("double_direction dis_prev:{:.4f}  dis:{:.4f}", dis_prev, dis_t);
        break;
      }
      if (dis_t > later_valid_dis && std::fabs(junction_angle.angle) > Angle_5) {
        junction_angle.position = LDAngleJunction::PositionDirection::LEFT;
        if (junction_angle.angle < -Angle_180) {
          junction_angle.angle = -junction_angle.angle - Angle_180;
        } else if (junction_angle.angle < 0) {
          junction_angle.angle = Angle_180 + junction_angle.angle;
        } else if (junction_angle.angle > Angle_180) {
          junction_angle.angle = junction_angle.angle - Angle_180;
        }
        break;
      }
      if (dis_t < -later_valid_dis && std::fabs(junction_angle.angle) > Angle_5) {
        junction_angle.position = LDAngleJunction::PositionDirection::RIGHT;
        if (junction_angle.angle > Angle_180) {
          junction_angle.angle = -junction_angle.angle + Angle_180;
        } else if (junction_angle.angle > 0) {
          junction_angle.angle = junction_angle.angle - Angle_180;
        } else if (junction_angle.angle < -Angle_180) {
          junction_angle.angle = junction_angle.angle + Angle_180;
        }
        break;
      }
      if (junction_angle.angle < -Angle_180) {
        junction_angle.angle += Angle_180;
      }
      if (junction_angle.angle > Angle_180) {
        junction_angle.angle -= Angle_180;
      }
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format(
        "raw_section_base:{} section:{}  angle:{:.2f} is_double_direction:{:d} direction:{}  unction_raw_angle:{:.2f} "
        "dis_t:{:.2f} normal_direction:{:d} is_big_than:{:d} point_x:{:.2f}-{:.2f} idx:{}",
        section.id, junction_angle.section->id, junction_angle.angle, is_double_direction, junction_angle.GetPositionInfo(),
        junction_angle.raw_angle, dis_t, normal_direction, std::fabs(junction_angle.angle) > Angle_179,
        point_vec.x(), point_vec.y(), idx);
  }
}

void SDMapElementExtract::JunctionAngleSectionDirection(const SDSectionInfo &section, std::vector<AngleJunction> &res,
                                                        bool normal_direction) {
  constexpr double         later_valid_dis = 0.5;
  size_t                   section_length  = section.points->size();
  if (section_length < 2) {
    return;
  }
  byd::common::math::Vec2d section_end{section.points->at(section_length - 1).x(), section.points->at(section_length - 1).y()};
  byd::common::math::Vec2d section_end_prev{section.points->at(section_length - 2).x(), section.points->at(section_length - 2).y()};
  if (!normal_direction) {
    section_end_prev.set_x(section.points->at(0).x());
    section_end_prev.set_y(section.points->at(0).y());
    section_end.set_x(section.points->at(1).x());
    section_end.set_y(section.points->at(1).y());
  }

  const byd::common::math::LineSegment2d line_section{section_end_prev, section_end};
  for (auto &junction_angle : res) {
    double                   dis_t = 0.0;
    byd::common::math::Vec2d point_vec{0.0, 0.0};
    junction_angle.raw_angle = junction_angle.angle;

    auto       &points = *junction_angle.section->points;
    const auto &range  = normal_direction ? points : std::vector<Eigen::Vector2f>(points.rbegin(), points.rend());

    int    idx      = -1;
    double dis_prev = 0.0;
    bool   is_double_direction{false};
    for (const auto &point : range) {
      idx++;
      dis_prev = dis_t;
      point_vec.set_x(point.x());
      point_vec.set_y(point.y());
      dis_t = line_section.ProductOntoUnit(point_vec);
      if (dis_t * dis_prev < -1e-9) {
        is_double_direction = true;
        SD_EXTRACT_MAP_ELEMENT << fmt::format("double_direction dis_prev:{:.4f}  dis:{:.4f}", dis_prev, dis_t);
        break;
      }
      if (dis_t > later_valid_dis && std::fabs(junction_angle.angle) > Angle_5) {
        junction_angle.position = AngleJunction::PositionDirection::LEFT;
        if (junction_angle.angle < -Angle_180) {
          junction_angle.angle = -junction_angle.angle - Angle_180;
        } else if (junction_angle.angle < 0) {
          junction_angle.angle = Angle_180 + junction_angle.angle;
        } else if (junction_angle.angle > Angle_180) {
          junction_angle.angle = junction_angle.angle - Angle_180;
        }
        break;
      }
      if (dis_t < -later_valid_dis && std::fabs(junction_angle.angle) > Angle_5) {
        junction_angle.position = AngleJunction::PositionDirection::RIGHT;
        if (junction_angle.angle > Angle_180) {
          junction_angle.angle = -junction_angle.angle + Angle_180;
        } else if (junction_angle.angle > 0) {
          junction_angle.angle = junction_angle.angle - Angle_180;
        } else if (junction_angle.angle < -Angle_180) {
          junction_angle.angle = junction_angle.angle + Angle_180;
        }
        break;
      }
      if (junction_angle.angle < -Angle_180) {
        junction_angle.angle += Angle_180;
      }
      if (junction_angle.angle > Angle_180) {
        junction_angle.angle -= Angle_180;
      }
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format(
        "raw_section_base:{} section:{}  angle:{:.2f} is_double_direction:{:d} direction:{} link_type:{} unction_raw_angle:{:.2f} "
        "dis_t:{:.2f} normal_direction:{:d} is_big_than:{:d} point_x:{:.2f}-{:.2f} idx:{}",
        section.id, junction_angle.section->id, junction_angle.angle, is_double_direction, junction_angle.GetPositionInfo(),
        junction_angle.section->link_type, junction_angle.raw_angle, dis_t, normal_direction, std::fabs(junction_angle.angle) > Angle_179,
        point_vec.x(), point_vec.y(), idx);
  }
}

void SDMapElementExtract::FilterOneRoad(const SDSectionInfo &section, std::vector<AngleJunction> &res) {
  std::unordered_set<size_t> invalid_ids_set;
  std::vector<size_t>        invalid_ids_vec;
  for (size_t i = 0; i < res.size(); i++) {
    if (res[i].section && SeemsOneRoad(section, *res[i].section)) {
      auto [it, inserted] = invalid_ids_set.insert(i);
      if (inserted) {
        invalid_ids_vec.emplace_back(i);
      }
      SD_EXTRACT_MAP_ELEMENT << fmt::format("junction_id:[{}-{}] is_one_road.", section.id, res[i].section->id);
    }
    for (size_t j = i + 1; j < res.size(); j++) {
      bool has_i_to_j = std::find(res[i].section->successor_section_id_list.begin(), res[i].section->successor_section_id_list.end(),
                                  res[j].section->id) != res[i].section->successor_section_id_list.end();
      bool has_j_to_i = std::find(res[j].section->successor_section_id_list.begin(), res[j].section->successor_section_id_list.end(),
                                  res[i].section->id) != res[j].section->successor_section_id_list.end();

      bool is_one_road = SeemsOneRoad(*res[i].section, *res[j].section);
      bool loop_lane   = (has_i_to_j && has_j_to_i);
      if (loop_lane || is_one_road) {
        auto [it, inserted] = invalid_ids_set.insert(i);
        if (inserted) {
          invalid_ids_vec.emplace_back(i);
        }
        SD_EXTRACT_MAP_ELEMENT << fmt::format("junction_id:[{}-{}] is_loop:{:d}  one_road:{:d}.", res[i].section->id, res[j].section->id,
                                              loop_lane, is_one_road);
      }
    }
  }
  for (auto it = invalid_ids_vec.rbegin(); it != invalid_ids_vec.rend(); ++it) {
    if (*it < res.size()) {
      res.erase(res.begin() + *it);
    }
  }
}

std::vector<SDMapElementExtract::AngleJunction> SDMapElementExtract::JunctionAnglesAroundJunction(
    const std::vector<SDSectionInfo> &route_section, size_t idx_input, int method) {
  if (idx_input >= route_section.size()) {
    return {};
  }
  std::vector<AngleJunction> res;
  std::set<uint64_t>         looped_ids;
  const SDSectionInfo       &section = route_section[idx_input];
  SD_EXTRACT_MAP_ELEMENT << fmt::format("calculate_the_section:{} method:{}", section.id, method);

  auto FindSectionInRoute = [&](uint64_t id, bool normal_direction = true) {
    if (looped_ids.count(id) != 0) {
      // SD_EXTRACT_MAP_ELEMENT << fmt::format("id:{} has in set.", id);
      return false;
    }
    looped_ids.insert(id);
    if (map_sd_sections_.count(id) == 0) {// when section roadclass>=7,no section in map_sd_sections_
      SD_EXTRACT_MAP_ELEMENT << "can't find the lane:" << id;
      return false;
    }
    if (id == section.id) {
      return false;
    }
    if (auto val = AngleDiffBetweenSection(section, *map_sd_sections_[id], true, normal_direction); val) {
      res.emplace_back((AngleJunction){map_sd_sections_[id], *val});
      // SD_EXTRACT_MAP_ELEMENT << fmt::format(
      //     "route_angle section_1:{}  section_2:{} angle_diff:{:.2f}", section.id,
      //     map_sd_sections_[id]->id, *val);
      return true;
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format("can't calc the angle.");
    return false;
  };
  if (method == 0) {
    auto find = map_section_angles_.find(section.id);
    if (find != map_section_angles_.end()) {
      for (const auto &junction_angle : find->second) {
        SD_EXTRACT_MAP_ELEMENT << fmt::format("raw_section_base:{} section:{}  angle:{:.2f}  direction:{}  link_type:{}", section.id,
                                              junction_angle.section->id, junction_angle.angle, junction_angle.GetPositionInfo(),
                                              junction_angle.section->link_type);
      }
      return find->second;
    }
    for (const auto &id_succ : section.successor_section_id_list) {
      if (!FindSectionInRoute(id_succ)) {
        continue;
      }
      for (const auto &id_prev : map_sd_sections_[id_succ]->predecessor_section_id_list) {
        FindSectionInRoute(id_prev);
      }
    }
  } else if (method == 1) {
    for (const auto &id_succ : section.successor_section_id_list) {
      FindSectionInRoute(id_succ);
    }
  } else if (method == 2) {
    for (const auto &id_prev : section.predecessor_section_id_list) {
      FindSectionInRoute(id_prev);
    }
  } 
  std::sort(res.begin(), res.end(), [](const AngleJunction &lhs, const AngleJunction &rhs) { return lhs.angle > rhs.angle; });
  FilterOneRoad(section, res);

  JunctionAngleSectionDirection(section, res, method != 2);
  if (method == 0) {
    map_section_angles_.insert({section.id, res});
  }

  return res;
}

bool SDMapElementExtract::SectionIsUTurn(const SDSectionInfo &section_t) {
  if (section_t.lane_group_idx.empty()) {
    return false;
  }
  bool is_exit_u = false;
  for (const auto &group_info : section_t.lane_group_idx) {
    auto group_it = map_sd_lane_groups_info_.find(group_info.id);
    if (group_it == map_sd_lane_groups_info_.end()) {
      continue;
    }
    const auto &lanes = group_it->second->lane_info;
    if (std::any_of(lanes.begin(), lanes.end(), [](const auto &lane_t) { return lane_t.type == LaneType::LANE_U_TURN_LANE; })) {
      is_exit_u = true;
      SD_EXTRACT_MAP_ELEMENT<<"section_t.id:"<<section_t.id<<"lane_group_id:"<<group_info.id;
      break;
    }
  }
  return is_exit_u;
};

bool SDMapElementExtract::SectionIsRightTurnDedicated(const SDSectionInfo &section_t) {
  if (section_t.lane_group_idx.empty()) {
    return false;
  }
  bool is_right_dedicated = false;
  for (const auto &group_info : section_t.lane_group_idx) {
    auto group_it = map_sd_lane_groups_info_.find(group_info.id);
    if (group_it == map_sd_lane_groups_info_.end()) {
      continue;
    }
    const auto &lanes = group_it->second->lane_info;
    if (std::any_of(lanes.begin(), lanes.end(), [](const auto &lane_t) { return lane_t.type == LaneType::LANE_RIGHT_TURN_LANE; })) {
      is_right_dedicated = true;
      break;
    }
  }
  return is_right_dedicated;
};

bool SDMapElementExtract::SectionIsRightTurn(const SDSectionInfo &section_t) {
  if (section_t.lane_group_idx.empty()) {
    return false;
  }
  bool is_right_turn = false;
  for (const auto &group_info : section_t.lane_group_idx) {
    auto group_it = map_sd_lane_groups_info_.find(group_info.id);
    if (group_it == map_sd_lane_groups_info_.end()) {
      continue;
    }
    const auto &lanes = group_it->second->lane_info;
    if (std::any_of(lanes.begin(), lanes.end(), [](const LaneInfo &lane_t) { return lane_t.turn_type == TurnType::RIGHT_TURN; })) {
      is_right_turn = true;
      break;
    }
  }
  return is_right_turn;
};

bool SDMapElementExtract::IsUTurn(size_t idx_start, const std::vector<SDSectionInfo> &route_section) {
  if (route_section.size() <= 2) {
    return false;
  }
  if (idx_start >= route_section.size() - 1) {
    return false;
  }
  const auto &section_current = route_section[idx_start];

  if (!SectionIsUTurn(route_section[idx_start + 1])) {
    return false;
  }
  size_t idx = idx_start + 2;
  for (; idx < route_section.size(); idx++) {
    if (!SectionIsUTurn(route_section[idx])) {
      break;
    }
  }
  if (idx >= route_section.size()) {
    idx = route_section.size() - 1;
  }

  junction_help_.idx_start_ = idx_start;
  junction_help_.idx_end_   = idx;
  junction_help_.valid_     = true;

  const auto &lane_group        = LaneGroups(route_section[idx_start]);
  auto       &arrow_vec         = junction_help_.junction_city_.map_lane_arrows_plan;
  auto       &arrow_vec_default = junction_help_.junction_city_.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }

  junction_help_.junction_city_.junction_type_city = JunctionTypeCity::UTurn;
  junction_help_.junction_city_.junction_id        = section_current.id;
  junction_help_.junction_city_.junction_action    = JunctionAction::UTurn;

  junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx_start);
  junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx, false);

  return true;
}

void SDMapElementExtract::CheckMainLaneIndex(double action_angle, JunctionInfoCity &junction_info) {
  SetMainLaneIndex(junction_info);
  if (junction_info.main_lane_indexs.empty()) {
    std::array<JunctionAction, 3> actions{JunctionAction::GoStraight};  // 默认包含直行
    if (action_angle > 0) {
      actions = {JunctionAction::TurnLeft, JunctionAction::GoStraight, JunctionAction::TurnRight};
    } else if (action_angle < 0) {
      actions = {JunctionAction::TurnRight, JunctionAction::GoStraight, JunctionAction::TurnLeft};
    }
    for (auto action : actions) {
      junction_info.junction_action = action;
      SetMainLaneIndex(junction_info);
      if (!junction_info.main_lane_indexs.empty()) {
        break;
      }
    }
    if (junction_info.main_lane_indexs.empty()) {
      SD_EXTRACT_MAP_ELEMENT << "CAN'T Find Any Appropriate Action.";
      junction_info.junction_action = JunctionAction::GoStraight;
    }
  }
  junction_info.main_lane_indexs.clear();
}
bool SDMapElementExtract::IsSubpathBidirectional(const std::vector<AngleJunction> &section_angles_succ,
                                                 const std::vector<SDSectionInfo> &route_sections, size_t idx) {
  if (idx + 1 >= route_sections.size()) {
    return false;
  }
  if (section_angles_succ.size() != 2) {
    return false;
  }
  const auto &next_section = route_sections[idx + 1];
  if (section_angles_succ.begin()->section && section_angles_succ.begin()->section->id == next_section.id) {
    return section_angles_succ.rbegin()->section &&
           section_angles_succ.rbegin()->section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE;
  }

  return false;
}

bool SDMapElementExtract::TempFilterSplit_01(const std::vector<AngleJunction> &succ_junction_angles) {
  if (succ_junction_angles.size() != 2) {
    return true;
  }
  const auto &front = succ_junction_angles.at(0);
  const auto &next  = succ_junction_angles.at(1);
  if (std::fabs(front.angle) < Angle_25 && next.angle < -Angle_25 && next.position == AngleJunction::PositionDirection::RIGHT &&
      next.section && next.section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("the_next_lane_id:{} be_filted", next.section ? next.section->id : 0);
    return false;
  }

  return true;
}

bool SDMapElementExtract::TempFilterSplit_02(const std::vector<AngleJunction> &succ_junction_angles) {
  if (succ_junction_angles.size() <= 2) {
    return false;
  }

  if (succ_junction_angles.front().angle > Angle_25) {
    return false;
  }
  auto succ_angles_info = succ_junction_angles;

  succ_angles_info.erase(std::remove_if(succ_angles_info.begin(), succ_angles_info.end(),
                                        [](const AngleJunction &rhs) {
                                          return rhs.section && rhs.section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE &&
                                                 IsInDegreeTarget(rhs.angle, -Angle_90);
                                        }),
                         succ_angles_info.end());

  return (std::all_of(succ_angles_info.begin(), succ_angles_info.end(),
                     [](const AngleJunction &rhs) { return std::fabs(rhs.angle) < Angle_75; })&&succ_angles_info.size()>=2);
}

bool SDMapElementExtract::TempFilterSplit_03(const std::vector<AngleJunction> &section_angles_succ_filtered) {
  if (section_angles_succ_filtered.size() < 2) {
    return false;
  }

  auto succ_angles_info = section_angles_succ_filtered;

  succ_angles_info.erase(std::remove_if(succ_angles_info.begin(), succ_angles_info.end(),
                                        [](const AngleJunction &rhs) {
                                          return rhs.section && rhs.section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE;
                                        }),
                         succ_angles_info.end());

  return std::all_of(succ_angles_info.begin(), succ_angles_info.end(),
                     [](const AngleJunction &rhs) { return std::fabs(rhs.angle) < Angle_75; });
}

bool SDMapElementExtract::IsJunctionCross(size_t idx_start, const std::vector<SDSectionInfo> &route_section) {
  if (idx_start == route_section.size() - 1) {
    return false;
  }
  // const auto &section = route_section[idx_start];
  // if (!section.has_junction) {
  //   return false;
  // }
  bool   idx_has_cross      = LinkTypeMaskType(route_section[idx_start].link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  bool   idx_next_has_cross = LinkTypeMaskType(route_section[idx_start + 1].link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  size_t group_size         = route_section[idx_start + 1].lane_group_idx.size();
  auto   lanes              = FindValidLaneGroups(route_section, idx_start);
  bool   is_exsit_left_turn{false};
  bool   is_exsit_straight_turn{false};
  bool   is_exsit_right_turn{false};

  std::vector<uint64_t> lane_ids_temp;
  if (lanes) {
    for (const auto &lane : lanes.value()) {
      lane_ids_temp.emplace_back(lane.id);
      auto turn_val = GetTurnTypeVal(lane.plan_turn_type);
      if ((GetActionTurnVal(JunctionAction::TurnLeft) & turn_val) != 0U) {
        is_exsit_left_turn = true;
      }
      if ((GetActionTurnVal(JunctionAction::GoStraight) & turn_val) != 0U) {
        is_exsit_straight_turn = true;
      }
      if ((GetActionTurnVal(JunctionAction::TurnRight) & turn_val) != 0U) {
        is_exsit_right_turn = true;
      }
    }
  }
  bool is_exist_cross = is_exsit_left_turn && is_exsit_straight_turn && is_exsit_right_turn;
  SD_EXTRACT_MAP_ELEMENT << fmt::format(
      "idx_has_cross:{}  next_cross:{} is_exist_cross:{}  is_exsit_left_turn:{}  is_exsit_straight_turn:{}  is_exsit_right_turn:{}  "
      "group_size:{} ids:{}",
      idx_has_cross, idx_next_has_cross, is_exist_cross, is_exsit_left_turn, is_exsit_straight_turn, is_exsit_right_turn, group_size,
      lane_ids_temp);

  if (!idx_has_cross && idx_next_has_cross && group_size == 0) {
    size_t idx{idx_start + 2};
    for (; idx < route_section.size(); idx++) {
      if (LinkTypeMaskType(route_section[idx].link_type, SDLinkTypeMask::SDLT_CROSSLINK) && route_section[idx].lane_group_idx.empty()) {
        continue;
      }
      break;
    }
    if (idx == route_section.size()) {
      SD_EXTRACT_MAP_ELEMENT << "not find end section.";
      return false;
    }
    SD_EXTRACT_MAP_ELEMENT << fmt::format("idx_start:{}  idx_end:{} idx:[{},{}]  is_exist_cross:{:d}", route_section[idx_start].id,
                                          route_section[idx].id, idx_start, idx, is_exist_cross);
    size_t idx_end = idx;

    for (idx = idx_start; idx != idx_end; idx++) {
      const auto &junctions_angle = JunctionAnglesAroundJunction(route_section, idx);

      bool is_exsit_90_left  = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return rhs.position == AngleJunction::PositionDirection::LEFT && IsInDegreeTarget(rhs.angle, Angle_90, Angle_35);
      });
      bool is_exsit_90_right = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return rhs.position == AngleJunction::PositionDirection::RIGHT && IsInDegreeTarget(rhs.angle, -Angle_90, Angle_35);
      });
      bool is_exsit_0        = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return IsInDegreeTarget(rhs.angle, 0.0, Angle_35) || IsInDegreeTarget(rhs.angle, Angle_180) ||
               IsInDegreeTarget(rhs.angle, -Angle_180);
      });
      bool line_2_cross      = Is2LineCross(route_section[idx], junctions_angle);
      SD_EXTRACT_MAP_ELEMENT << fmt::format("is_left:{:d} is_right:{:d} is_exist_0:{:d} line_2_cross:{:d}", is_exsit_90_left,
                                            is_exsit_90_right, is_exsit_0, line_2_cross);
      if (junctions_angle.size() >= 3 && ((is_exsit_0 && is_exsit_90_left && is_exsit_90_right) || is_exist_cross || line_2_cross)) {
        SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{}->{} junction_found cross_junction", route_section[idx_start].id,
                                              route_section[idx_end].id);
        junction_help_.idx_start_ = idx_start;
        junction_help_.idx_end_   = idx_end;
        junction_help_.valid_     = true;

        junction_help_.junction_city_.junction_id        = route_section[idx_start].id;
        junction_help_.junction_city_.junction_type_city = JunctionTypeCity::CrossRoad;
        auto action_angle                                = AngleDiffBetweenSection(route_section[idx_start], route_section[idx_end], false);
        for (std::size_t idx = idx_start; idx < idx_end; idx++) {
          if (auto action_angle_t = AngleDiffBetweenSection(route_section[idx], route_section[idx + 1]); action_angle_t) {
            if (!action_angle || std::fabs(*action_angle) < std::fabs(action_angle_t.value())) {
              action_angle = action_angle_t;
            }
          }
        }
        junction_help_.junction_city_.junction_action = JunctionAction::GoStraight;
        if (action_angle) {
          SD_EXTRACT_MAP_ELEMENT << fmt::format("cross_action_angle:{:.2f}", action_angle.value());
          if (IsInDegreeTarget(*action_angle, Angle_90)) {
            junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
          } else if (IsInDegreeTarget(*action_angle, -Angle_90)) {
            junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
          } else if (*action_angle < -Angle_140 || *action_angle > Angle_140) {
            junction_help_.junction_city_.junction_action = JunctionAction::UTurn;
          }
        }
        junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx_start);
        junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx_end, false);
        auto &arrow_vec                                     = junction_help_.junction_city_.map_lane_arrows_plan;
        auto &arrow_vec_default                             = junction_help_.junction_city_.map_lane_arrows_default;

        auto val = FindValidLaneGroups(route_section, idx_start);
        if (val) {
          for (const auto &lane : val.value()) {
            arrow_vec.emplace_back(lane.plan_turn_type);
            arrow_vec_default.emplace_back(lane.turn_type);
          }
        }
        if (action_angle) {
          CheckMainLaneIndex(action_angle.value(), junction_help_.junction_city_);
        }

        return true;
      }
    }
  } else {
    size_t idx_end = idx_start + 1;
    // for (; idx_end < route_section.size(); ++idx_end) {
    //   if (!route_section[idx_end].has_junction) {
    //     break;
    //   }
    // }
    // idx_end = idx_end == route_section.size() ? --idx_end : idx_end;
    // oss://oss-byd-sh-roadtest/byd-hc25-033252-sz/20250430/20250430_wanban_qingtian_hc25-033252/2025-04-30_22-07-38/2025-04-30_22-07-39.record.00005.22-12-35
    // so can't be recursive until has_junction be false.

    for (size_t idx = idx_start; idx < idx_end && idx < route_section.size(); idx++) {
      const auto &junctions_angle = JunctionAnglesAroundJunction(route_section, idx);

      bool is_exsit_90_left  = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return rhs.position == AngleJunction::PositionDirection::LEFT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
      });
      bool is_exsit_90_right = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return rhs.position == AngleJunction::PositionDirection::RIGHT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
      });
      bool is_exsit_0        = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
        return IsInDegreeTarget(rhs.angle, 0.0) || IsInDegreeTarget(rhs.angle, Angle_180) || IsInDegreeTarget(rhs.angle, -Angle_180);
      });
      bool line_2_cross      = Is2LineCross(route_section[idx], junctions_angle);
      SD_EXTRACT_MAP_ELEMENT << fmt::format("cross_judge is_left:{:d}  is_right:{:d}  is_0:{:d}  line_2_cross:{:d}", is_exsit_90_left,
                                            is_exsit_90_right, is_exsit_0, line_2_cross);
      if (junctions_angle.size() >= 3 && ((is_exsit_0 && is_exsit_90_left && is_exsit_90_right) || is_exist_cross || line_2_cross)) {
        SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{}->{} junction_found cross_junction", route_section[idx_start].id,
                                              route_section[idx_end].id);
        junction_help_.idx_start_ = idx_start;
        junction_help_.idx_end_   = idx_end;
        junction_help_.valid_     = true;

        junction_help_.junction_city_.junction_id        = route_section[idx_start].id;
        junction_help_.junction_city_.junction_type_city = JunctionTypeCity::CrossRoad;
        auto action_angle                                = AngleDiffBetweenSection(route_section[idx_start], route_section[idx_end], false);
        for (std::size_t idx = idx_start; idx < idx_end; idx++) {
          if (auto action_angle_t = AngleDiffBetweenSection(route_section[idx], route_section[idx + 1]); action_angle_t) {
            if (!action_angle || std::fabs(*action_angle) < std::fabs(action_angle_t.value())) {
              action_angle = action_angle_t;
            }
          }
        }
        junction_help_.junction_city_.junction_action = JunctionAction::GoStraight;
        if (action_angle) {
          if (IsInDegreeTarget(*action_angle, Angle_90)) {
            junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
          } else if (IsInDegreeTarget(*action_angle, -Angle_90)) {
            junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
          }
        }

        junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx_start);
        junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx_end, false);
        auto &arrow_vec                                     = junction_help_.junction_city_.map_lane_arrows_plan;
        auto &arrow_vec_default                             = junction_help_.junction_city_.map_lane_arrows_default;

        auto val = FindValidLaneGroups(route_section, idx_start);
        if (val) {
          for (const auto &lane : val.value()) {
            arrow_vec.emplace_back(lane.plan_turn_type);
            arrow_vec_default.emplace_back(lane.turn_type);
          }
        }
        if (action_angle) {
          CheckMainLaneIndex(action_angle.value(), junction_help_.junction_city_);
        }

        return true;
      }
    }
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("link_type {:d}  {:d}  group_size:{:d} is_exist_cross:{:d}  not_junction cross.", idx_has_cross,
                                        idx_next_has_cross, group_size, is_exist_cross);
  // TODO(lingpeng): what if ego has been in junction ???

  return false;
}

bool SDMapElementExtract::IsJunctionT(size_t idx_start, const std::vector<SDSectionInfo> &route_section) {
  if (idx_start == route_section.size() - 1) {
    return false;
  }
  const auto &section = route_section[idx_start];
  if (!section.has_junction) {
    return false;
  }
  const auto &junctions_angle = JunctionAnglesAroundJunction(route_section, idx_start);

  bool exist_cross        = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
    return rhs.section && LinkTypeMaskType(rhs.section->link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  });
  bool is_next_right_lane = SectionIsRightTurn(route_section[idx_start + 1]) || SectionIsRightTurnDedicated(route_section[idx_start + 1]);

  bool idx_has_cross      = LinkTypeMaskType(route_section[idx_start].link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  bool idx_next_has_cross = LinkTypeMaskType(route_section[idx_start + 1].link_type, SDLinkTypeMask::SDLT_CROSSLINK);
  bool pre_not_succ_is    = !idx_has_cross && idx_next_has_cross;
  if ((!pre_not_succ_is && !exist_cross) || is_next_right_lane) {
    return false;
  }
  size_t idx{idx_start + 1};
  for (; idx < route_section.size(); idx++) {
    if (!LinkTypeMaskType(route_section[idx].link_type, SDLinkTypeMask::SDLT_CROSSLINK)) {
      break;
    }
  }
  if (idx == route_section.size()) {
    return false;
  }
  size_t idx_end{idx};
  for (idx = idx_start; idx < idx_end; idx++) {
    const auto &junctions_angle   = JunctionAnglesAroundJunction(route_section, idx);
    bool        is_exsit_90_left  = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
      return rhs.position == AngleJunction::PositionDirection::LEFT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
    });
    bool        is_exsit_90_right = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
      return rhs.position == AngleJunction::PositionDirection::RIGHT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
    });
    bool        is_exsit_0        = std::any_of(junctions_angle.begin(), junctions_angle.end(), [](const AngleJunction &rhs) {
      return IsInDegreeTarget(rhs.angle, 0.0) || IsInDegreeTarget(rhs.angle, Angle_180) || IsInDegreeTarget(rhs.angle, -Angle_180);
    });
    if (is_exsit_90_left && is_exsit_0 && is_exsit_90_right) {
      continue;
    }

    // case 1. ego in T down
    bool is_ego_in_T_down = is_exsit_90_left && is_exsit_90_right;
    // case 2. ego in T left
    bool is_ego_in_T_left = is_exsit_0 && is_exsit_90_right;
    // case 3. ego in T right
    bool is_ego_in_T_right = is_exsit_0 && is_exsit_90_left;
    bool alway_true        = true;
    if (alway_true || is_ego_in_T_down || is_ego_in_T_left || is_ego_in_T_right) {
      SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{}->{} junction_found T_junction.", route_section[idx_start].id,
                                            route_section[idx_end].id);
      junction_help_.idx_start_                        = idx_start;
      junction_help_.idx_end_                          = idx_end;
      junction_help_.valid_                            = true;
      junction_help_.junction_city_.junction_id        = route_section[idx_start].id;
      junction_help_.junction_city_.junction_type_city = JunctionTypeCity::TJunction;
      const auto action                                = SetJunctionAction(route_section);
      junction_help_.junction_city_.junction_action    = action;
      auto action_angle                                = AngleDiffBetweenSection(route_section[idx_start], route_section[idx_end], false);
      junction_help_.junction_city_.junction_action    = JunctionAction::GoStraight;
      if (action_angle) {
        if (IsInDegreeTarget(*action_angle, Angle_90)) {
          junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
        } else if (IsInDegreeTarget(*action_angle, -Angle_90)) {
          junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
        }
      }

      junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx_start);
      junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx_end, false);
      auto &arrow_vec                                     = junction_help_.junction_city_.map_lane_arrows_plan;
      auto &arrow_vec_default                             = junction_help_.junction_city_.map_lane_arrows_default;

      auto val = FindValidLaneGroups(route_section, idx_start);
      if (val) {
        for (const auto &lane : val.value()) {
          arrow_vec.emplace_back(lane.plan_turn_type);
          arrow_vec_default.emplace_back(lane.turn_type);
        }
      }

      return true;
    }
  }
  SD_EXTRACT_MAP_ELEMENT << "not_junction T.";

  return false;
}

bool SDMapElementExtract::IsJunctionTSmall(size_t idx_start, const std::vector<SDSectionInfo> &route_section) {
  if (idx_start == route_section.size() - 1) {
    return false;
  }
  const auto &junctions_angles = JunctionAnglesAroundJunction(route_section, idx_start, 0);

  bool is_exsit_90_left   = std::any_of(junctions_angles.begin(), junctions_angles.end(), [](const AngleJunction &rhs) {
    return rhs.position == AngleJunction::PositionDirection::LEFT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
  });
  bool is_exsit_90_right  = std::any_of(junctions_angles.begin(), junctions_angles.end(), [](const AngleJunction &rhs) {
    return rhs.position == AngleJunction::PositionDirection::RIGHT && IsInDegreeTarget(std::fabs(rhs.angle), Angle_90);
  });
  bool is_exsit_0         = std::any_of(junctions_angles.begin(), junctions_angles.end(), [](const AngleJunction &rhs) {
    return IsInDegreeTarget(rhs.angle, 0.0) || IsInDegreeTarget(rhs.angle, Angle_180) || IsInDegreeTarget(rhs.angle, -Angle_180);
  });
  bool is_next_right_lane = false;
  if (idx_start < route_section.size() - 1) {
    is_next_right_lane = SectionIsRightTurn(route_section[idx_start + 1]) || SectionIsRightTurnDedicated(route_section[idx_start + 1]);
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("90_left:{:d}  0:{:d} right:{:d}", is_exsit_90_left, is_exsit_0, is_exsit_90_right);
  if ((is_exsit_90_left && is_exsit_0 && is_exsit_90_right) || is_next_right_lane) {
    return false;
  }

  // case 1. ego in T down
  bool is_ego_in_T_down = is_exsit_90_left && is_exsit_90_right;
  // case 2. ego in T left
  bool is_ego_in_T_left = is_exsit_0 && is_exsit_90_right;
  // case 3. ego in T right
  bool is_ego_in_T_right = is_exsit_0 && is_exsit_90_left;
  if (is_ego_in_T_down || is_ego_in_T_left || is_ego_in_T_right) {
    SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{} junction_found T_small_junction.", route_section[idx_start].id);
    size_t idx_end{idx_start};
    if (idx_start < route_section.size() - 1) {
      idx_end++;
    }
    junction_help_.idx_start_                        = idx_start;
    junction_help_.idx_end_                          = idx_start;
    junction_help_.valid_                            = true;
    junction_help_.junction_city_.junction_id        = route_section[idx_start].id;
    junction_help_.junction_city_.junction_type_city = JunctionTypeCity::SmallTJunction;
    junction_help_.junction_city_.junction_id        = route_section[idx_start].id;
    const auto action                                = SetJunctionAction(route_section);
    junction_help_.junction_city_.junction_action    = action;
    auto action_angle                                = AngleDiffBetweenSection(route_section[idx_start], route_section[idx_end], false);
    junction_help_.junction_city_.junction_action    = JunctionAction::GoStraight;
    if (action_angle) {
      if (IsInDegreeTarget(*action_angle, Angle_90)) {
        junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
      } else if (IsInDegreeTarget(*action_angle, -Angle_90)) {
        junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
      }
    }
    const auto &lane_group                              = LaneGroups(route_section[idx_start]);
    junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx_start);
    junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx_end, false);
    auto &arrow_vec                                     = junction_help_.junction_city_.map_lane_arrows_plan;
    auto &arrow_vec_default                             = junction_help_.junction_city_.map_lane_arrows_default;
    for (const auto &lane : lane_group) {
      arrow_vec.emplace_back(lane.plan_turn_type);
      arrow_vec_default.emplace_back(lane.turn_type);
    }
    return true;
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("is_ego_in_T_down:{:d}  is_ego_in_T_left:{:d}  is_ego_in_T_right:{:d} not_junction SmallT.",
                                        is_ego_in_T_down, is_ego_in_T_left, is_ego_in_T_right);

  return false;
}

std::vector<RoadMainType> SDMapElementExtract::JunctionsMainRamp(const std::vector<AngleJunction> &junctions_angle,
                                                                 const SDSectionInfo              &rhs) {
  const size_t num_junctions = junctions_angle.size();
  if (num_junctions == 0) {
    return {};
  }

  std::vector<int>      roads_class;
  std::vector<uint64_t> road_lanes_num;
  roads_class.reserve(num_junctions);
  road_lanes_num.reserve(num_junctions);
  std::vector<int> same_rhs_road_class_idx;

  double angle_min     = std::numeric_limits<double>::infinity();
  int    angle_min_idx = -1;

  // First pass traversal:​​ Collect data and find the minimum road class
  int min_road_class = std::numeric_limits<int>::max();
  for (std::size_t idx = 0; idx < num_junctions; idx++) {
    const auto &junc = junctions_angle[idx];

    if (std::fabs(angle_min) > std::fabs(junc.angle)) {
      angle_min     = junc.angle;
      angle_min_idx = static_cast<int>(idx);
    }

    if (rhs.road_class == junc.section->road_class) {
      same_rhs_road_class_idx.emplace_back(static_cast<int>(idx));
    }
    const int current_class = static_cast<int>(junc.section->road_class);
    roads_class.push_back(current_class);
    size_t lane_counter = 0;
    for (const auto &group_info : junc.section->lane_group_idx) {
      auto it = map_sd_lane_groups_info_.find(group_info.id);
      if (it != map_sd_lane_groups_info_.end() && !it->second->lane_info.empty()) {
        lane_counter = it->second->lane_info.size();
        break;
      }
    }
    road_lanes_num.push_back(lane_counter > 0 ? lane_counter : junc.section->lane_num);

    if (current_class < min_road_class) {
      min_road_class = current_class;
    }
  }

  SD_EXTRACT_MAP_ELEMENT << "angle_min_idx:" << angle_min_idx << "  " << fmt::format("{}", same_rhs_road_class_idx)
                         << "  rhs.road_class:" << static_cast<int>(rhs.road_class);
  if (!same_rhs_road_class_idx.empty() && angle_min_idx != -1) {
    auto it = std::find(same_rhs_road_class_idx.begin(), same_rhs_road_class_idx.end(), angle_min_idx);
    if (it != same_rhs_road_class_idx.end()) {
      std::vector<RoadMainType> main_type;
      main_type.reserve(num_junctions);
      for (int idx = 0; idx < static_cast<int>(num_junctions); idx++) {
        main_type.emplace_back(idx == *it ? RoadMainType::RoadMain : RoadMainType::RoadRamp);
      }
      return main_type;
    }
  }

  // Count occurrences of the minimum road class​
  const auto min_count = std::count(roads_class.cbegin(), roads_class.cend(), min_road_class);

  // Case 1:​​ Minimum road class is unique
  if (min_count == 1) {
    std::vector<RoadMainType> main_type;
    main_type.reserve(num_junctions);
    for (int road_class_t : roads_class) {
      main_type.emplace_back(road_class_t == min_road_class ? RoadMainType::RoadMain : RoadMainType::RoadRamp);
    }
    return main_type;
  }

  // Case 2:​​ Multiple minimum road classes, find the maximum lane count
  uint64_t max_lane_num = 0;
  for (size_t i = 0; i < num_junctions; ++i) {
    if (roads_class[i] == min_road_class && road_lanes_num[i] > max_lane_num) {
      max_lane_num = road_lanes_num[i];
    }
  }

  // ​​Count occurrences of the maximum lane count​
  const auto max_lane_count = std::count(road_lanes_num.cbegin(), road_lanes_num.cend(), max_lane_num);

  // ​​Subcase 2a:​​ Maximum lane count is unique
  if (max_lane_count == 1) {
    std::vector<RoadMainType> main_type;
    main_type.reserve(num_junctions);
    for (uint64_t lane_num : road_lanes_num) {
      main_type.emplace_back(lane_num == max_lane_num ? RoadMainType::RoadMain : RoadMainType::RoadRamp);
    }
    return main_type;
  }

  // Subcase 2b:​​ Multiple maximum lane counts, select the one with the smallest absolute angle
  double min_angle_abs = std::numeric_limits<double>::max();
  size_t min_angle_idx = 0;
  for (size_t i = 0; i < num_junctions; ++i) {
    if (roads_class[i] == min_road_class && road_lanes_num[i] == max_lane_num) {
      const double angle_abs = std::fabs(junctions_angle[i].angle);
      if (angle_abs < min_angle_abs) {
        min_angle_abs = angle_abs;
        min_angle_idx = i;
      }
    }
  }

  // Generate final result:​​ Mark only one as RoadMain
  std::vector<RoadMainType> main_type(num_junctions, RoadMainType::RoadRamp);
  main_type[min_angle_idx] = RoadMainType::RoadMain;
  return main_type;
}

bool SDMapElementExtract::IsJunctionSplit(size_t idx, const std::vector<SDSectionInfo> &route_section) {
  if (idx == route_section.size() - 1 || route_section[idx].successor_section_id_list.size() <= 1) {
    return false;
  }
  const auto &junctions_angle = JunctionAnglesAroundJunction(route_section, idx, 1);
  if (junctions_angle.size() <= 1) {
    return false;
  }
  if (junctions_angle.back().angle < -Angle_75 && junctions_angle.back().position == AngleJunction::PositionDirection::RIGHT) {
    return false;
  }
  if (junctions_angle.front().position == AngleJunction::PositionDirection::LEFT && junctions_angle.front().angle > Angle_75) {
    return false;
  }

  SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{} junction_found Split.", route_section[idx].id);
  junction_help_.idx_start_                        = idx;
  junction_help_.idx_end_                          = idx;
  junction_help_.valid_                            = true;
  junction_help_.junction_city_.junction_type_city = JunctionTypeCity::RoadSplit;
  junction_help_.junction_city_.junction_id        = route_section[idx].id;
  junction_help_.junction_city_.junction_action    = JunctionAction::GoStraight;
  if (junctions_angle.front().section->id == route_section[idx + 1].id) {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Left;
  } else if (junctions_angle.back().section->id == route_section[idx + 1].id) {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Right;
  } else {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Straight;
  }
  const auto &lane_group                              = LaneGroups(route_section[idx]);
  junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx);
  junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, idx + 1, false);
  auto &arrow_vec                                     = junction_help_.junction_city_.map_lane_arrows_plan;
  auto &arrow_vec_default                             = junction_help_.junction_city_.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }

  junction_help_.junction_city_.succ_road_class = JunctionsMainRamp(junctions_angle, route_section[idx]);
  double      min_angle                         = std::numeric_limits<double>::infinity();
  std::size_t min_idx                           = 0;
  std::size_t next_idx                          = 0;
  for (std::size_t i = 0; i < junctions_angle.size(); i++) {
    if (std::fabs(min_angle) > std::fabs(junctions_angle[i].angle)) {
      min_angle = junctions_angle[i].angle;
      min_idx   = i;
    }
    if (junctions_angle[i].section->id == route_section[idx + 1].id) {
      next_idx = i;
    }
  }
  junction_help_.junction_city_.junction_action = JunctionAction::GoStraight;
  if (next_idx < min_idx) {
    junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
  } else if (next_idx > min_idx) {
    junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
  }
  auto IsDedicatedRight = [&](const SDSectionInfo &rhs) {
    const auto &lanes_begin = LaneGroups(rhs, false);
    const auto &lanes_back  = LaneGroups(rhs);
    return std::any_of(lanes_begin.begin(), lanes_begin.end(),
                       [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_RIGHT_TURN_LANE; }) ||
           std::any_of(lanes_back.begin(), lanes_back.end(),
                       [](const LaneInfo &rhs) { return rhs.type == LaneType::LANE_RIGHT_TURN_LANE; });
  };

  for (const auto &angle : junctions_angle) {
    if (angle.angle < 0.0 && angle.section && IsDedicatedRight(*angle.section)) {
      junction_help_.junction_city_.is_dedicated_right_turn_lane = true;
      break;
    }
  }
  SD_EXTRACT_MAP_ELEMENT << fmt::format("split_section:{}.  is_dedicated_right_turn_lane:{:d}", junction_help_.junction_city_.junction_id,
                                        junction_help_.junction_city_.is_dedicated_right_turn_lane);

  return true;
}

bool SDMapElementExtract::IsJunctionMerge(size_t idx, const std::vector<SDSectionInfo> &route_section) {
  if (idx >= route_section.size() - 1) {
    return false;
  }
  size_t      next_idx     = idx + 1;
  const auto &next_section = route_section[next_idx];
  if (next_section.predecessor_section_id_list.size() <= 1) {
    SD_EXTRACT_MAP_ELEMENT << "prev size <=1";
    return false;
  }
  auto junctions_angle = JunctionAnglesAroundJunction(route_section, next_idx, 2);
  if (junctions_angle.size() <= 1) {
    return false;
  }
  std::reverse(junctions_angle.begin(), junctions_angle.end());

  SD_EXTRACT_MAP_ELEMENT << fmt::format("section:{} junction_found Merge.", route_section[next_idx].id);
  junction_help_.idx_start_                        = idx;
  junction_help_.idx_end_                          = next_idx;
  junction_help_.valid_                            = true;
  junction_help_.junction_city_.junction_type_city = JunctionTypeCity::RoadMerge;
  junction_help_.junction_city_.junction_action    = JunctionAction::GoStraight;
  junction_help_.junction_city_.junction_id        = route_section[idx].id;

  const auto &lane_group                              = LaneGroups(route_section[idx]);
  junction_help_.junction_city_.main_road_lane_nums   = GetSectionLaneNum(route_section, idx);
  junction_help_.junction_city_.target_road_lane_nums = GetSectionLaneNum(route_section, next_idx, false);
  SD_EXTRACT_MAP_ELEMENT << "targe_num:" << junction_help_.junction_city_.target_road_lane_nums
                         << "  next_id:" << route_section[next_idx].id;
  auto &arrow_vec         = junction_help_.junction_city_.map_lane_arrows_plan;
  auto &arrow_vec_default = junction_help_.junction_city_.map_lane_arrows_default;
  for (const auto &lane : lane_group) {
    arrow_vec.emplace_back(lane.plan_turn_type);
    arrow_vec_default.emplace_back(lane.turn_type);
  }

  if (junctions_angle.front().section->id == route_section[idx].id) {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Left;
  } else if (junctions_angle.back().section->id == route_section[idx].id) {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Right;
  } else {
    junction_help_.junction_city_.split_merge_direction = DirectionSplitMerge::Straight;
  }
  auto prev_road_class                          = JunctionsMainRamp(junctions_angle, route_section[idx + 1]);
  junction_help_.junction_city_.prev_road_class = prev_road_class;
  // for (std::size_t i = 0; i < prev_road_class.size(); i++) {
  // if (prev_road_class[i] == RoadMainType::RoadMain) {
  // junction_help_.junction_city_.target_road_lane_nums = static_cast<int>(LaneGroups(*(junctions_angle[i].section), false).size());
  // }
  // }
  double      min_angle = std::numeric_limits<double>::infinity();
  std::size_t min_idx   = 0;  // NOLINT
  std::size_t prev_idx  = 0;  // NOLINT
  for (std::size_t i = 0; i < junctions_angle.size(); i++) {
    if (std::fabs(min_angle) > std::fabs(junctions_angle[i].angle)) {
      min_angle = junctions_angle[i].angle;
      min_idx   = i;
    }
    if (junctions_angle[i].section->id == route_section[idx - 1].id) {
      prev_idx = i;
    }
  }
  // junction_help_.junction_city_.junction_action = JunctionAction::GoStraight;
  // if (prev_idx < min_idx) {
  //   junction_help_.junction_city_.junction_action = JunctionAction::TurnRight;
  // } else if (prev_idx > min_idx) {
  //   junction_help_.junction_city_.junction_action = JunctionAction::TurnLeft;
  // }

  return true;
}

JunctionAction SDMapElementExtract::SetJunctionAction(const std::vector<SDSectionInfo> &route_section) {
  if (junction_help_.valid_ || junction_help_.idx_start_ < 0 || junction_help_.idx_start_ >= route_section.size()) {
    return JunctionAction::Unknown;
  }
  const auto &lane_group_idx = route_section[junction_help_.idx_start_].lane_group_idx;
  if (lane_group_idx.empty()) {
    return JunctionAction::Unknown;
  }
  if (map_sd_lane_groups_info_.count(lane_group_idx.back().id) == 0) {
    return JunctionAction::Unknown;
  }
  const auto &back_lane_group = lane_group_idx.back();
  if (map_sd_lane_groups_info_.count(back_lane_group.id) == 0) {
    return JunctionAction::Unknown;
  }

  for (const auto &lane_info : map_sd_lane_groups_info_[back_lane_group.id]->lane_info) {
    if (lane_info.plan_turn_type == TurnType::LEFT_TURN) {
      return JunctionAction::TurnLeft;
    }
    if (lane_info.plan_turn_type == TurnType::RIGHT_TURN) {
      return JunctionAction::TurnRight;
    }
  }

  return JunctionAction::GoStraight;
}

std::vector<LaneInfo> SDMapElementExtract::LaneGroups(const SDSectionInfo &section, bool is_back) {
  if (section.lane_group_idx.empty()) {
    return {};
  }

  auto group_idx = section.lane_group_idx;
  std::sort(group_idx.begin(), group_idx.end(),
            [](const SDLaneGroupIndex &lhs, const SDLaneGroupIndex &rhs) { return lhs.end_range_offset < rhs.end_range_offset; });

  const auto &search_range = is_back ? std::vector<SDLaneGroupIndex>(group_idx.rbegin(), group_idx.rend()) : group_idx;

  for (const auto &idx : search_range) {
    const auto it = map_sd_lane_groups_info_.find(idx.id);
    if (it != map_sd_lane_groups_info_.end() && it->second && !it->second->lane_info.empty()) {
      bool is_all_wait = std::all_of(it->second->lane_info.begin(), it->second->lane_info.end(), [](const LaneInfo &rhs) {
        return rhs.type == LaneType::LANE_LEFT_WAIT || rhs.type == LaneType::LANE_RIGHT_TURN_AREA ||
               rhs.type == LaneType::LANE_U_TURN_AREA || rhs.type == LaneType::LANE_NO_TURN_AREA;
      });
      if (is_all_wait) {
        continue;
      }
      SD_EXTRACT_MAP_ELEMENT << "find_the_lane_index:" << idx.id;
      return it->second->lane_info;
    }
  }

  return {};
}

[[nodiscard]] std::optional<std::vector<LaneInfo>> SDMapElementExtract::FindValidLaneGroups(const std::vector<SDSectionInfo> &route_section,
                                                                                            std::size_t idx, bool is_back) {
  if (route_section.empty() || idx >= route_section.size()) {
    return std::nullopt;
  }

  int idx_temp = static_cast<int>(idx);
  while (idx_temp >= 0) {
    const auto &lane_groups = LaneGroups(route_section[idx_temp], is_back);
    if (lane_groups.empty()) {
      idx_temp--;
    } else {
      return lane_groups;
    }
  }
  return std::nullopt;
}

}  // namespace cem::fusion::navigation
