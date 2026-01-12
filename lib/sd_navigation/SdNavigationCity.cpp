#include "SdNavigationCity.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <string_view>
#include <type_traits>
#include <base/params_manager/params_manager.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <message/common/geometry.h>
#include <message/env_model/navigation/navigation.h>
#include "fmt/core.h"
#include "fmt/format.h"
#include "lib/common/log_custom.h"
#include "lib/sd_navigation/routing_map_debug.h"
#include "modules/msg/environment_model_msgs/local_map_info.pb.h"
#include "cyber_release/include/cyber/time/time.h"
#include "modules/common/math/line_segment2d.h"

namespace cem {
namespace fusion {
namespace navigation {

void SdNavigationCity::Proc(const std::shared_ptr<RoutingMap> &raw_routing_map, BevMapInfoConstPtr &raw_bev_map,
                            BevMapInfoPtr &GlobalBevMapOutPut, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                            std::vector<uint64_t> &sd_guide_lane_ids, AccLaneInfo &sd_acc_lane_info) {
  // AINFO << fmt::format("Enter SdNavigationCity::Proc logic.");
  INTERNAL_PARAMS.navigation_info_data.ClearContainer();
  guide_lane_result.clear();
  sd_guide_lane_ids.clear();
  cut_map_zone_.active = false;
  if (!raw_routing_map || !raw_bev_map || !GlobalBevMapOutPut) {
    return;
  }
  Reset();
  //////////////////////// SD信息提取 ////////////////////////////
  sd_map_element_extract_.Proc(raw_routing_map);
  junctions_info_city_ = sd_map_element_extract_.GetJunctionsInfo();
  sd_recommend_lane_   = sd_map_element_extract_.GetSdRecommendLane();
  JunctionsStateProc(raw_bev_map);
  UpdateJunctionStateQueue();
  SetJunctionInfo();
  SetACCLaneInfo(sd_acc_lane_info);
  SetReversibleLaneType(raw_routing_map ,raw_bev_map, GlobalBevMapOutPut);
  for (const auto &junction_tmp : junctions_info_city_) {
    SD1LOG << "[JUNCTION INFO] id: " << junction_tmp.junction_id << " type:" << static_cast<int>(junction_tmp.junction_type_city)
           << " offset:" << junction_tmp.offset << " state:" << static_cast<int>(junction_tmp.junction_state_city);
    SD_JUNCTION_CONVERTE_LOG << fmt::format("{}", junction_tmp);
  }

  /////////////////////// 根据路口信息更新导航推荐切图标志 ///////////////////////////////
  UpdateCutMapFlagByHardScenes();
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
      "cut_map_zone_ - active: {}, sceneType: {}, anchorJunctionId: {}, anchorType: {}, anchorOffset: {:.2f}, winBefore: {:.2f}, winAfter: "
      "{:.2f}",
      cut_map_zone_.active, cut_map_zone_.sceneType, cut_map_zone_.anchorJunctionId, static_cast<int>(cut_map_zone_.anchorType),
      cut_map_zone_.anchorOffset, cut_map_zone_.winBefore, cut_map_zone_.winAfter);

  /////////////////////// BEV感知预处理 ///////////////////////////////
  uint64_t bev_counter                                                = raw_bev_map == nullptr ? 0 : raw_bev_map->header.cycle_counter;
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().bev_counter = bev_counter;
  bev_data_processor_.Process(junctions_info_city_, raw_bev_map, bev_candidate_lanes_, bev_candidate_road_edges_, bev_line_sorts_,
                              bev_left_road_edge_indexs_, bev_right_road_edge_indexs_, complete_section_info_, bev_egoRoadEdgeIndexPair_,
                              bev_ego_lane_related_, bev_ego_root_section_x0_);
  SD_BEV_PROCESS << fmt::format("{}", complete_section_info_);
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("{}", complete_section_info_);
  INTERNAL_PARAMS.navigation_info_data.set_bev_root_section_ids(bev_ego_root_section_x0_);
  // 临时最小化改动SdNavigationCity成员函数，从 complete_section_info_ 同步 lane IDs 到 bev_sections_
  bev_sections_.clear();
  for (const auto &section : complete_section_info_.sections) {
    bev_sections_.push_back(section.lane_ids);
  }
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().all_sections_in = bev_sections_;
  auto &navi_debug_info                                                   = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_info.all_separators.clear();
  for (const auto &sep_group : complete_section_info_.separators) {
    std::vector<std::pair<uint64_t, int>> group;
    size_t count = std::min({sep_group.edge_ids.size(), sep_group.types.size(), sep_group.edge_points.size()});
    for (size_t i = 0; i < count; ++i) {
      group.emplace_back(sep_group.edge_ids[i], static_cast<int>(sep_group.types[i]));
    }
    navi_debug_info.all_separators.push_back(group);
  }

  ///////////////////////  从sdmap获取匹配Merge车道属性  //////////////////////////////////
  BevMapInfo bev_map = *raw_bev_map;
  AssignMergeAttributesToBevLanes(bev_map);
  std::unordered_map<uint64_t, BevLaneInfo *> lane_map;
  lane_map.clear();
  for (auto &lane : bev_map.lane_infos) {
    lane_map[lane.id] = &lane;
  }

  ///////////////////////  导航选路  //////////////////////////////////
  road_selected_      = road_selector_.SelectOptRoad(junctions_info_city_, bev_ego_lane_related_, history_guide_laneids_, bev_sections_,
                                                     complete_section_info_, info_buffer_, isSplitRoadSelectWorked_);
  hold_road_selected_ = road_selector_.HoldOptRoad(bev_sections_, complete_section_info_, road_selected_);
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("isSplitRoadSelectWorked :{}", isSplitRoadSelectWorked_);
  SD_JUNCTION_CONVERTE_LOG << fmt::format(
      "routing_map_counter:{}  time:{:.3f} junction_id:{} idx_route:{} "
      "state:{} road_selected:{} bev_section:{}",
      raw_routing_map ? raw_routing_map->header.cycle_counter : 0, raw_routing_map ? raw_routing_map->header.timestamp : 0.0,
      junction_current_.junction_id, IdxInAllMppSection(junction_current_.junction_id),
      cem::fusion::navigation::StrJunctionStateCity(junction_current_.junction_state_city), hold_road_selected_, bev_sections_);
  SD_JUNCTION_CONVERTE_LOG << std::string_view(info_buffer_.data(), info_buffer_.size());
  std::ostringstream os_section;
  for (auto &section : road_selected_) {
    os_section << section << ",";
  }
  SD1LOG << "[Target BevSection]" << os_section.str();
  SD_FINE_MATCH_LOG << "[Target BevSection] " << os_section.str();
  std::ostringstream hold_os_section;
  for (auto &section : hold_road_selected_) {
    hold_os_section << section << ",";
  }
  SD_FINE_MATCH_LOG << "[Hold BevSection] " << hold_os_section.str();

  SD_COARSE_MATCH_LOG << "[Target BevSection]" << os_section.str();
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().target_section      = road_selected_;
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().hold_target_section = hold_road_selected_;

  ////////////////////// 感知车道index的计算 /////////////////////////
  CalcBevIndex(hold_road_selected_, lane_index_left_, lane_index_right_);

  //////////////////////  特殊车道的过滤处理 /////////////////////////
  /*过滤公交车道*/
  road_selected_without_buslane_ = FilterSpecialLane(raw_routing_map, hold_road_selected_, raw_bev_map, GlobalBevMapOutPut);
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().target_section_filter = road_selected_without_buslane_;

  /* 过滤merge属性车道 */
  road_selected_without_merge_.clear();
  road_selected_without_merge_ = road_selected_without_buslane_;
  // for (const auto &lane_id : road_selected_without_buslane_) {
  //   // SD_MERGE_LOG << fmt::format("Lane id: {}.", lane_id);
  //   auto it = lane_map.find(lane_id);
  //   if (it != lane_map.end()) {
  //     BevLaneInfo *lane = it->second;
  //     if (lane->merge_topo_extend ==
  //     MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
  //       road_selected_without_merge_.push_back(lane_id);
  //     } else {
  //       SD_MERGE_LOG << fmt::format("Lane: {} is merge line, delete for
  //       navigation.", lane_id);
  //     }
  //   } else {
  //     SD_MERGE_LOG << fmt::format("Lane id: {} not found in bev_map.",
  //     lane_id);
  //   }
  // }

  // 提取目标路口信息
  ExtractTargetJunctions(junctions_info_city_);

  ////////////////////  导航选择车道 /////////////////////////////////
  if (IsCutMapActive()) {
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Enter SelectGuideLanesWithSd wiht bevsize: {}.", bev_ego_root_section_x0_.size());
    // auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
    // navi_debug_infos.match_type.push_back(StrNaviMatchType(NaviMatchType::Type10));
    // SelectGuideLanesWithSd(bev_ego_root_section_x0_, sd_recommend_lane_,
    // GlobalBevMapOutPut, guide_lane_result, sd_guide_lane_ids);
    SelectGuideLanes(junctions_info_city_, road_selected_without_merge_, lane_index_left_, lane_index_right_, GlobalBevMapOutPut,
                     guide_lane_result, sd_guide_lane_ids);
  } else {
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Enter SelectGuideLanes wiht bevsize: {}.", road_selected_without_merge_.size());
    SelectGuideLanes(junctions_info_city_, road_selected_without_merge_, lane_index_left_, lane_index_right_, GlobalBevMapOutPut,
                     guide_lane_result, sd_guide_lane_ids);
  }
  SetNaviInterfaceAndDebugInfos(GlobalBevMapOutPut);
}

void SdNavigationCity::UpdateCutMapFlagByHardScenes() {
  cut_map_zone_ = CutMapZone{};

  if (junctions_info_city_.empty())
    return;

  auto isSplit = [](const JunctionInfoCity &j) -> bool {
    return (j.junction_type_city == JunctionTypeCity::RoadSplit || (j.junction_type_city == JunctionTypeCity::UTurn && j.is_only_Turn));
  };

  auto isCross = [](const JunctionInfoCity &j) -> bool {
    const bool type_ok = (j.junction_type_city == JunctionTypeCity::CrossRoad) || (j.junction_type_city == JunctionTypeCity::TJunction) ||
                         (j.junction_type_city == JunctionTypeCity::SmallTJunction);
    return type_ok && j.need_match_arrow;
  };

  auto inCutWindowAround = [](double offset) -> bool {
    return (offset < 150.0 && offset > -50.0);
  };

  auto hasLeftTurnOnSplit = [&](const JunctionInfoCity &j) -> bool {
    if (!j.split_direction_ids.empty()) {
      for (uint64_t sec_id : j.split_direction_ids) {
        auto sec = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(sec_id);
        if (sec) {
          uint32_t lt = sec->link_type;
          if ((lt & static_cast<uint32_t>(SDLinkTypeMask::SDLT_LEFTTURN)) != 0U) {
            return true;
          }
        }
      }
    }
    return false;
  };

  // 构建 offset 递增索引，便于“连续split”配对
  std::vector<int> ord(junctions_info_city_.size());
  std::iota(ord.begin(), ord.end(), 0);
  std::sort(ord.begin(), ord.end(), [&](int a, int b) { return junctions_info_city_[a].offset < junctions_info_city_[b].offset; });

  // ========= 场景1：RoadSplit 的 1分3 三叉路口 =========
  for (int idx : ord) {
    const auto &j = junctions_info_city_[idx];
    if (isSplit(j) && j.succ_road_class.size() == 3) {
      if (inCutWindowAround(j.offset)) {
        cut_map_zone_.active           = true;
        cut_map_zone_.sceneType        = 1;
        cut_map_zone_.anchorJunctionId = j.junction_id;
        cut_map_zone_.anchorType       = j.junction_type_city;
        cut_map_zone_.anchorOffset     = j.offset;
        // debug 信息
        fmt::format_to(info_buffer_, "[CUT] scene=1 1->3 split id:{} off:{:.1f}; ", j.junction_id, j.offset);
        return;
      }
    }
  }

  // ========= 场景2：（两个 split 间距 < 150m），以“前一个”路口为锚点, 不要求连续 =========
  for (size_t i = 0; i + 1 < ord.size(); ++i) {
    const auto &s = junctions_info_city_[ord[i]];  // 第一个 split
    if (!isSplit(s))
      continue;

    const JunctionInfoCity *t_best = nullptr;  // 你也可以改成挑最近的；这里默认找到“第一个满足条件的”就触发
    for (int j = i + 1; j < static_cast<int>(ord.size()); ++j) {
      const auto  &cand = junctions_info_city_[ord[j]];
      const double gap  = cand.offset - s.offset;
      if (gap < 0.0)
        continue;
      if (gap >= 150.0)
        break;
      if (isSplit(cand)) {
        t_best = &cand;
        break;
      }
    }

    if (!t_best)
      continue;

    // 桥接窗口触发条件：进入 s 前150m（s.offset<150）并且尚未超过 t_best 后50m（t_best.offset>-50）
    const bool active_bridge = (s.offset < 150.0) && (t_best->offset > -50.0);
    if (active_bridge) {
      cut_map_zone_.active           = true;
      cut_map_zone_.sceneType        = 2;
      cut_map_zone_.anchorJunctionId = std::fabs(s.offset) < std::fabs(t_best->offset) ? s.junction_id : t_best->junction_id;
      cut_map_zone_.anchorType       = s.junction_type_city;
      cut_map_zone_.anchorOffset     = s.offset;
      cut_map_zone_.winBefore        = 150.0;
      cut_map_zone_.winAfter         = std::max(50.0, (t_best->offset + 50.0) - s.offset);

      fmt::format_to(
          info_buffer_,
          "[CUT] scene=2 continuous split prev_id:{} next_id:{} gap:{:.1f} off_prev:{:.1f} off_next:{:.1f} win=[prev-150, next+50]; ",
          s.junction_id, t_best->junction_id, (t_best->offset - s.offset), s.offset, t_best->offset);
      return;
    }
  }

  // ========= 场景3：带“提前左转”的 split + 十字（Cross/T/SmallT，且 need_match_arrow）距 < 150m =========
  for (int i = 0; i < static_cast<int>(ord.size()); ++i) {
    const auto &s = junctions_info_city_[ord[i]];
    if (!isSplit(s))
      continue;
    if (!hasLeftTurnOnSplit(s))
      continue;

    // 寻找后续最近十字（CrossRoad），间距<150m
    for (int j = i + 1; j < static_cast<int>(ord.size()); ++j) {
      const auto &c = junctions_info_city_[ord[j]];
      if (!isCross(c))
        continue;

      const double gap = c.offset - s.offset;
      if (gap < 0.0 || gap >= 150.0)
        break;

      const JunctionInfoCity *c_best = nullptr;
      for (int j = i + 1; j < static_cast<int>(ord.size()); ++j) {
        const auto  &cand = junctions_info_city_[ord[j]];
        const double gap  = cand.offset - s.offset;
        if (gap < 0.0)
          continue;
        if (gap >= 150.0)
          break;
        if (isCross(cand)) {
          c_best = &cand;
          break;
        }
      }
      if (!c_best)
        continue;

      const bool active_bridge = (s.offset < 150.0) && (c_best->offset > -50.0);
      if (active_bridge) {
        cut_map_zone_.active           = true;
        cut_map_zone_.sceneType        = 3;
        cut_map_zone_.anchorJunctionId = std::abs(s.offset) < std::abs(c_best->offset) ? s.junction_id : c_best->junction_id;
        cut_map_zone_.anchorType       = s.junction_type_city;
        cut_map_zone_.anchorOffset     = s.offset;
        cut_map_zone_.winBefore        = 150.0;
        cut_map_zone_.winAfter         = std::max(50.0, (c_best->offset + 50.0) - s.offset);

        fmt::format_to(info_buffer_,
                       "[CUT] scene=3 split(left-turn)+cross split_id:{} cross_id:{} gap:{:.1f} off_split:{:.1f} off_cross:{:.1f} "
                       "win=[split-150, cross+50]; ",
                       s.junction_id, c_best->junction_id, (c_best->offset - s.offset), s.offset, c_best->offset);
        return;
      }
    }
  }

  // ========= 场景4：UTurn 且 is_only_Turn 的路口，在 offset < 150.0 && offset > -50.0 范围内 =========
  for (int idx : ord) {
    const auto &j = junctions_info_city_[idx];
    if (j.junction_type_city == JunctionTypeCity::UTurn && j.is_only_Turn) {
      if (inCutWindowAround(j.offset)) {
        cut_map_zone_.active           = true;
        cut_map_zone_.sceneType        = 4;
        cut_map_zone_.anchorJunctionId = j.junction_id;
        cut_map_zone_.anchorType       = j.junction_type_city;
        cut_map_zone_.anchorOffset     = j.offset;
        fmt::format_to(info_buffer_, "[CUT] scene=4 UTurn is_only_Turn id:{} off:{:.1f}; ", j.junction_id, j.offset);
        return;
      }
    }
  }

  // ========= 场景5：RoadSplit主路到辅路，且自车脚下可顺下的车道数>1，且与路口可顺下的车道数不一致 =========
  // 记忆之前识别到的复杂场景
  static std::unordered_set<uint64_t> remembered_scene5_junctions;
  for (int idx : ord) {
    const auto &j = junctions_info_city_[idx];
    if (j.junction_type_city == JunctionTypeCity::RoadSplit && IsRoadSplitMain2Ramp(j)) {
      bool is_remembered            = remembered_scene5_junctions.count(j.junction_id) > 0;
      int  junction_reachable_count = 0;
      if (j.junction_ids.size() >= 2) {
        uint64_t current_junction_id    = j.junction_ids.front();
        uint64_t associated_junction_id = j.junction_ids.back();

        auto current_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(current_junction_id);
        auto associated_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(associated_junction_id);

        if (current_section && associated_section && !current_section->lane_group_idx.empty()) {
          uint64_t    last_lg_id = current_section->lane_group_idx.back().id;
          const auto *last_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(last_lg_id);

          if (last_lg) {
            uint64_t    ego_lg_id = topology_extractor_.GetCurrentLaneGroupId();
            const auto *ego_lg    = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(ego_lg_id);

            if (ego_lg) {
              int section_lane_num = associated_section->lane_num;

              const SDLaneGroupInfo *target_lg = nullptr;
              if (section_lane_num > 0) {
                for (const auto &lg_idx : associated_section->lane_group_idx) {
                  const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
                  if (lane_group && lane_group->lane_num <= section_lane_num) {
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
                      target_lg = lane_group;
                      break;
                    }
                  }
                }
              }

              if (!target_lg && !associated_section->lane_group_idx.empty()) {
                uint64_t first_lg_id = associated_section->lane_group_idx.front().id;
                target_lg            = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(first_lg_id);
              }

              if (target_lg) {
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
                    continue;
                  }

                  std::queue<uint64_t>         q;
                  std::unordered_set<uint64_t> visited;
                  q.push(start_lane.id);
                  visited.insert(start_lane.id);

                  while (!q.empty()) {
                    uint64_t current = q.front();
                    q.pop();

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

                junction_reachable_count = static_cast<int>(reachable_lanes.size());

                // 计算自车脚下顺下去的车道数 (ego_reachable_lanes.size())
                int ego_reachable_count = 0;

                constexpr double       kMinLgLen   = 15.0;
                const SDLaneGroupInfo *start_lg    = ego_lg;
                uint64_t               start_lg_id = ego_lg_id;
                double                 ego_lg_len  = ego_lg->length;

                if (ego_lg_len < kMinLgLen) {
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
                    if (L >= kMinLgLen) {
                      start_lg    = cand_lg;
                      start_lg_id = cand_id;
                      found       = true;
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
                    ego_reachable_count = junction_reachable_count;  // 回退使用junction_reachable_count
                  }
                }

                if (ego_reachable_count == 0) {  // 没有回退或不需要回退
                  std::unordered_set<uint64_t> ego_reachable_lanes;
                  for (const auto &start_lane : start_lg->lane_info) {
                    const auto *sd_lane = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(start_lane.id);
                    if (sd_lane && sd_lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
                      continue;
                    }

                    std::queue<uint64_t>         q;
                    std::unordered_set<uint64_t> visited;
                    q.push(start_lane.id);
                    visited.insert(start_lane.id);

                    while (!q.empty()) {
                      uint64_t current = q.front();
                      q.pop();

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

                  ego_reachable_count = static_cast<int>(ego_reachable_lanes.size());
                }

                // 自车脚下可顺下的车道数>1，且与路口可顺下的车道数不一致
                // 或者已经记住这个路口是复杂场景
                if ((ego_reachable_count > 1 && ego_reachable_count != junction_reachable_count) || is_remembered) {
                  if (inCutWindowAround(j.offset)) {
                    cut_map_zone_.active           = true;
                    cut_map_zone_.sceneType        = 5;
                    cut_map_zone_.anchorJunctionId = j.junction_id;
                    cut_map_zone_.anchorType       = j.junction_type_city;
                    cut_map_zone_.anchorOffset     = j.offset;

                    //记忆该路口
                    remembered_scene5_junctions.insert(j.junction_id);

                    fmt::format_to(info_buffer_, "[CUT] scene=5 RoadSplit Main2Ramp ego_count:{} junction_count:{} id:{} off:{:.1f}; ",
                                   ego_reachable_count, junction_reachable_count, j.junction_id, j.offset);
                    return;
                  }
                } else if (!inCutWindowAround(j.offset)) {
                  remembered_scene5_junctions.erase(j.junction_id);
                }
              }
            }
          }
        }
      }
    }
  }

  // 清理
  std::vector<uint64_t> to_erase;
  for (const auto &junction_id : remembered_scene5_junctions) {
    bool found = false;
    for (const auto &j : junctions_info_city_) {
      if (j.junction_id == junction_id) {
        found = true;
        if (j.offset < -100.0) {
          to_erase.push_back(junction_id);
        }
        break;
      }
    }
    if (!found) {
      to_erase.push_back(junction_id);
    }
  }
  for (const auto &junction_id : to_erase) {
    remembered_scene5_junctions.erase(junction_id);
  }
  // 未命中
  fmt::format_to(info_buffer_, "[CUT] scene=none; ");
}

void SdNavigationCity::AssignMergeAttributesToBevLanes(BevMapInfo &bev_map) {
  std::vector<MergeDetail> merge_details_new;
  merge_details_new.clear();
  topology_extractor_.SetIsHighway(false);
  topology_extractor_.GetMergeTopologiesFromMap(merge_details_new);
  // topology_extractor_.PrintMergeDetails(merge_details_new);
  topology_extractor_.SetBevLaneMergeTopoUseMap(merge_details_new, bev_map);
  SD_MERGE_LOG << fmt::format("[SdNavigationCity] Now BEV lane size: {}", bev_map.lane_infos.size());
  for (const auto &lane : bev_map.lane_infos) {
    SD_MERGE_LOG << fmt::format("{}", lane);
  }
}

bool SdNavigationCity::IsEgoInDedicatedRightLane() const {
  if (junctions_info_city_.empty()) {
    return false;
  }
  auto it = std::find_if(junctions_info_city_.begin(), junctions_info_city_.end(),
                         [](const JunctionInfoCity &city) { return city.junction_state_city != JunctionStateCity::PASSED; });
  if (it == junctions_info_city_.end()) {
    return false;
  }
  if (it->junction_type_city == JunctionTypeCity::RoadSplit && it->is_dedicated_right_turn_lane &&
      it->junction_action == JunctionAction::TurnRight && it->offset < 80) {
    return true;
  }
  if (it == junctions_info_city_.begin()) {
    return false;
  }
  auto it_prev = std::prev(it);
  return it->junction_type_city == JunctionTypeCity::RoadMerge && it->offset > 0 && it_prev->junction_action == JunctionAction::TurnRight &&
         it_prev->junction_type_city == JunctionTypeCity::RoadSplit && it_prev->is_dedicated_right_turn_lane;
}

void SdNavigationCity::Reset() {
  bev_candidate_lanes_.clear();
  bev_candidate_road_edges_.clear();
  bev_line_sorts_.clear();
  bev_left_road_edge_indexs_.clear();
  bev_right_road_edge_indexs_.clear();
  last_intersect_bev_laneids_.clear();
  bev_sections_.clear();
  bev_ego_lane_related_.clear();
  lane_index_left_.clear();
  lane_index_right_.clear();
  road_selected_repeat_.clear();
  road_selected_candidate_.clear();
  road_selected_map_.clear();
  guide_lanes_.clear();
  junctions_around_.clear();
  road_selected_without_buslane_.clear();
  road_selected_without_merge_.clear();
  bev_data_processor_.Clear();
  // 初始化 pair
  bev_egoRoadEdgeIndexPair_ = {-1, -1};
  main_lane_pos_            = MainLanePos::Unknow;
  junction_current_         = JunctionInfoCity();
  info_buffer_.clear();
}

void SdNavigationCity::ExtractTargetJunctions(const std::vector<JunctionInfoCity> &junctions_info_city) {
  target_junction_vct_.clear();

  // 检测与右转专用道路口相邻的路口
  DetectAdjacentRightTurnJunctions(junctions_info_city, coarse_matching_.adjacent_junctions_info_);

  auto MergeJunction = [&](const JunctionInfoCity &current, const JunctionInfoCity &next) -> bool {
    if (current.junction_type_city == JunctionTypeCity::TJunction || current.junction_type_city == JunctionTypeCity::SmallTJunction ||
        current.junction_type_city == JunctionTypeCity::CrossRoad) {
      if (current.junction_action == JunctionAction::GoStraight) {
        if (current.map_lane_arrows_plan.size() >= 2 && current.main_lane_indexs.size() - 1 == current.map_lane_arrows_plan.size() &&
            (current.main_lane_indexs.front() == 1 || current.main_lane_indexs.back() == current.map_lane_arrows_plan.size() - 2)) {

          if (next.junction_state_city != JunctionStateCity::PASSED && next.offset < NavigationConfig::max_distance &&
              next.need_match_arrow) {
            if (next.junction_type_city == JunctionTypeCity::TJunction || next.junction_type_city == JunctionTypeCity::SmallTJunction ||
                next.junction_type_city == JunctionTypeCity::CrossRoad) {
              if (fabs(next.offset - current.offset) < 100) {
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  };

  for (unsigned int i = 0; i < junctions_info_city.size(); i++) {
    auto &junction = junctions_info_city[i];
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("{}", junction);
    if (junction.junction_state_city == JunctionStateCity::PASSED) {
      continue;
    }
    if (junction.offset > NavigationConfig::max_distance) {
      break;
    }
    if (!junction.need_match_arrow) {
      continue;
    }

    if (IsInterestedJunction(junction)) {
      bool need_merge_junction = false;
      if (i + 1 < junctions_info_city.size()) {
        need_merge_junction = MergeJunction(junction, junctions_info_city[i + 1]);
      }
      if (need_merge_junction) {
        target_junction_vct_.push_back(junctions_info_city[i + 1]);
      } else {
        target_junction_vct_.push_back(junction);
      }
      break;
    }
    if (IsEffectedJunction(junction)) {
      target_junction_vct_.push_back(junction);
    }
  }

  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("target_junction_vct_.size: {}", target_junction_vct_.size());
  if (target_junction_vct_.size() >= 2) {
    auto &junction = target_junction_vct_.back();
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("{}", junction);
    if (IsInterestedJunction(target_junction_vct_.back())) {
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("junction {} is interested junction.", junction.junction_id);
      std::vector<JunctionInfoCity> junctions = {};
      junctions.push_back(target_junction_vct_.front());
      junctions.push_back(target_junction_vct_.back());
      target_junction_vct_.clear();
      target_junction_vct_ = junctions;
    } else {
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("junction {} is not interested junction.", junction.junction_id);
      std::vector<JunctionInfoCity> junctions = {};
      junctions.push_back(target_junction_vct_.front());
      target_junction_vct_.clear();
      target_junction_vct_ = junctions;
    }
  }
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Now target_junction_vct_.size: {}", target_junction_vct_.size());
}

void SdNavigationCity::SelectGuideLanes(const std::vector<JunctionInfoCity> &junctions_info_city,
                                        const std::vector<uint64_t> &road_selected_in, const std::vector<int> &lane_index_left,
                                        const std::vector<int> &lane_index_right, BevMapInfoPtr &GlobalBevMapOutPut,
                                        std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                        std::vector<uint64_t>                                 &sd_guide_lane_ids) {

  guide_lane_result.clear();
  sd_guide_lane_ids.clear();
  std::vector<uint64_t>         guide_laneids     = {};
  std::vector<std::vector<int>> guide_lane_length = {};

  auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  if (target_junction_vct_.empty()) {

    guide_laneids = road_selected_in;
    for (auto &laneid_tmp : guide_laneids) {
      guide_lane_result.push_back({laneid_tmp, {-1, 0}});
      guide_lane_length.push_back({-1, 0});
    }
    navi_debug_infos.guide_laneids      = guide_laneids;
    navi_debug_infos.guide_lanes_length = guide_lane_length;
    navi_debug_infos.match_type.push_back(StrNaviMatchType(NaviMatchType::Type6));
    sd_guide_lane_ids = guide_laneids;

  } else if (target_junction_vct_.size() == 1) {

    NaviMatchType navi_match_type = NaviMatchType::Unknow;
    JunctionNaviMatch(target_junction_vct_.front(), road_selected_in, lane_index_left, lane_index_right, GlobalBevMapOutPut,
                      navi_match_type, guide_laneids);
    for (auto &laneid_tmp : guide_laneids) {
      guide_lane_result.push_back({laneid_tmp, {-1, 0}});
      guide_lane_length.push_back({-1, 0});
    }
    navi_debug_infos.guide_laneids      = guide_laneids;
    navi_debug_infos.guide_lanes_length = guide_lane_length;
    navi_debug_infos.match_type.push_back(StrNaviMatchType(navi_match_type));
    sd_guide_lane_ids = guide_laneids;

  } else {

    JunctionInfoCity     &first_junction      = target_junction_vct_.front();
    JunctionInfoCity     &second_junction     = target_junction_vct_.back();
    std::vector<uint64_t> first_guide_result  = {};
    std::vector<uint64_t> second_guide_result = {};

    NaviMatchType navi_match_type = NaviMatchType::Unknow;
    JunctionNaviMatch(first_junction, road_selected_in, lane_index_left, lane_index_right, GlobalBevMapOutPut, navi_match_type,
                      first_guide_result);
    navi_debug_infos.match_type.push_back(StrNaviMatchType(navi_match_type));
    navi_match_type = NaviMatchType::Unknow;
    JunctionNaviMatch(second_junction, first_guide_result, lane_index_left, lane_index_right, GlobalBevMapOutPut, navi_match_type,
                      second_guide_result);
    navi_debug_infos.match_type.push_back(StrNaviMatchType(navi_match_type));

    for (auto &laneid_tmp : first_guide_result) {
      auto find_second = std::find(second_guide_result.begin(), second_guide_result.end(), laneid_tmp);
      if (find_second != second_guide_result.end()) {
        std::pair<int, int> guide_dist = {(int)second_junction.offset, -1};
        guide_lane_result.emplace_back(laneid_tmp, guide_dist);
        guide_lane_length.push_back({(int)second_junction.offset, -1});
      } else {
        std::pair<int, int> guide_dist = {(int)second_junction.offset, 0};
        guide_lane_result.emplace_back(laneid_tmp, guide_dist);
        guide_lane_length.push_back({(int)second_junction.offset, 0});
      }
    }
    navi_debug_infos.guide_laneids      = first_guide_result;
    navi_debug_infos.guide_lanes_length = guide_lane_length;
    sd_guide_lane_ids                   = second_guide_result;
  }
  for (auto &target_junction : target_junction_vct_) {
    navi_debug_infos.target_junction_ids.push_back(target_junction.junction_id);
    navi_debug_infos.junction_type.push_back(StrJunctionTypeCity(target_junction.junction_type_city));
    navi_debug_infos.split_merge.push_back(StrDirectionSplitMerge(target_junction.split_merge_direction));
    navi_debug_infos.junction_action.push_back(StrJunctionAction(target_junction.junction_action));
    navi_debug_infos.junction_offset.push_back((int)target_junction.offset);
  }

  history_guide_laneids_.clear();
  history_guide_laneids_ = sd_guide_lane_ids;
  std::ostringstream os1;
  for (auto guide_laneid_tmp : sd_guide_lane_ids) {
    os1 << guide_laneid_tmp << ",";
  }
  SD_COARSE_MATCH_TYPE2_LOG << "[SelectGuideLanes]  : " << os1.str();
  navi_debug_infos.guide_laneids_refer = sd_guide_lane_ids;
  INTERNAL_PARAMS.navigation_info_data.set_v2_junction_ids(navi_debug_infos.target_junction_ids);
}

void SdNavigationCity::SelectGuideLanesWithSd(const std::vector<uint64_t>           &bev_ego_root_section_x0,
                                              const std::map<double, RecommendLane> &sd_recommend_lane, BevMapInfoPtr &GlobalBevMapOutPut,
                                              std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                              std::vector<uint64_t>                                 &sd_guide_lane_ids) {
  auto                 &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  std::vector<uint64_t> target_junction_ids;
  for (const auto &junction : target_junction_vct_) {
    target_junction_ids.push_back(junction.junction_id);
  }
  INTERNAL_PARAMS.navigation_info_data.set_v2_junction_ids(target_junction_ids);
  std::set<uint64_t> bev_guide_lanes_with_sd;
  guide_lane_result.clear();
  sd_guide_lane_ids.clear();
  if (bev_ego_root_section_x0.empty() || sd_recommend_lane.empty()) {
    SD_FINE_MATCH_LOG << "bev or sd is empty, return: " << bev_ego_root_section_x0.empty() << ", " << sd_recommend_lane.empty();
    return;
  }

  for (auto &bev_ego_id : bev_ego_root_section_x0) {
    SD_FINE_MATCH_LOG << "bev_ego_id: " << bev_ego_id;
  }

  RecommendLane sd_target_lane;
  bool          is_found = false;
  for (auto it = sd_recommend_lane.begin(); it != sd_recommend_lane.end(); ++it) {
    if (it->first < 0.0) {
      auto next_it = std::next(it);
      if (next_it != sd_recommend_lane.end() && next_it->first > 0.0) {
        sd_target_lane = it->second;
        is_found       = true;
        break;
      }

      if (next_it == sd_recommend_lane.end()) {
        sd_target_lane = it->second;
        is_found       = true;
        break;
      }
    }
  }

  if (!is_found) {
    return;
  }
  constexpr double kSdGuideRemmRange  = 200.0;
  int              sd_ego_lane_num    = sd_target_lane.lane_num;
  int              bev_ego_lane_num   = bev_ego_root_section_x0.size();
  int              lane_num_diff      = std::abs(sd_ego_lane_num - bev_ego_lane_num);
  bool             sd_match_from_left = true;  // 1 left, 0 right
  double           nearest_distance   = std::numeric_limits<double>::max();
  // 自车脚下bev车道和地图车道数对不上
  if (bev_ego_lane_num != sd_ego_lane_num) {
    auto it_target_lane =
        std::find_if(sd_recommend_lane.begin(), sd_recommend_lane.end(), [&](const auto &pair) { return pair.second == sd_target_lane; });
    if (it_target_lane != sd_recommend_lane.end()) {
      for (auto it = it_target_lane; it != sd_recommend_lane.end(); ++it) {
        if ((it->second.lane_num == bev_ego_lane_num) && (it->first < kSdGuideRemmRange)) {
          // case1:自车脚下未匹配，合理范围内匹配上
          SD_FINE_MATCH_LOG << "Case1: bev and sd lane num match at a distance";
          sd_target_lane     = it->second;
          sd_match_from_left = true;
          break;
        }
        // case2:合理范围未匹配上，且地图和感知车道数相差较大
        if ((std::next(it) == sd_recommend_lane.end()) && (lane_num_diff > 1)) {
          // int ego_bev_id, ego_bev_index = -1;
          int ego_bev_id = -1, ego_bev_index = -1;
          for (auto &bev_root_id : bev_ego_root_section_x0) {
            auto lane_r = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(bev_root_id);
            if (!lane_r) {
              continue;
            }
            if (lane_r->position == (int)BevLanePosition::LANE_LOC_EGO) {
              ego_bev_id = bev_root_id;
              break;
            } else {
              int nearest_index = lane_r->indexed_geos.FindNearestPoint(0.0, 0.0);
              if (nearest_index >= 0 && nearest_index < static_cast<int>(lane_r->geos->size())) {
                double lane_distance = fabs(0.0 - lane_r->geos->at(nearest_index).y());
                SD_FINE_MATCH_LOG << "lane_distance: " << lane_distance;
                if (lane_distance < nearest_distance) {
                  nearest_distance = lane_distance;
                  ego_bev_id       = bev_root_id;
                }
              }
            }
          }
          SD_FINE_MATCH_LOG << "ego_bev_id: " << ego_bev_id;
          auto it = std::find(bev_ego_root_section_x0.begin(), bev_ego_root_section_x0.end(), ego_bev_id);
          if (it != bev_ego_root_section_x0.end()) {
            ego_bev_index = std::distance(bev_ego_root_section_x0.begin(), it);
            if (ego_bev_index < static_cast<double>(bev_ego_lane_num) / 2.0f) {
              sd_match_from_left = true;
            } else {
              sd_match_from_left = false;
            }
          } else {
            sd_match_from_left = true;
          }
          SD_FINE_MATCH_LOG << "Case2: bev and sd lane num not match";
        }
      }
    }
  }
  SD_FINE_MATCH_LOG << "lanegroup_id: " << sd_target_lane.lanegroup_id;
  std::vector<int> ego_lane_index_left, ego_lane_index_right;
  ego_lane_index_left.reserve(bev_ego_root_section_x0.size());
  // ego_lane_index_right.reserve(bev_ego_root_section_x0.size());
  for (int i = 1; i <= bev_ego_root_section_x0.size(); ++i) {
    ego_lane_index_left.emplace_back(i);
    // ego_lane_index_right.emplace_back(bev_ego_root_section_x0.size() - i +
    // 1);
  }
  // for (auto &lane_left : ego_lane_index_left) {
  //   SD_FINE_MATCH_LOG << "lane_left: " << int(lane_left);
  // }
  // for (auto &lane_right : ego_lane_index_right) {
  //   SD_FINE_MATCH_LOG << "lane_right: " << int(lane_right);
  // }

  for (auto &sd_target_index : sd_target_lane.lane_seqs) {
    SD_FINE_MATCH_LOG << "sd_target_index: " << sd_target_index;
    if (sd_match_from_left) {
      auto find_bev = std::find(ego_lane_index_left.begin(), ego_lane_index_left.end(), sd_target_index);
      if (find_bev != ego_lane_index_left.end()) {
        auto &bev_road_id = bev_ego_root_section_x0.at(std::distance(ego_lane_index_left.begin(), find_bev));
        SD_FINE_MATCH_LOG << "from left bev_road_id: " << bev_road_id;
        bev_guide_lanes_with_sd.insert(bev_road_id);
      } else {
        auto find_closest =
            std::min_element(ego_lane_index_left.begin(), ego_lane_index_left.end(), [&sd_target_index](const int &a, const int &b) {
              return std::abs(a - static_cast<int>(sd_target_index)) < std::abs(b - static_cast<int>(sd_target_index));
            });
        if (find_closest != ego_lane_index_left.end()) {
          auto &bev_road_id = bev_ego_root_section_x0.at(std::distance(ego_lane_index_left.begin(), find_closest));
          SD_FINE_MATCH_LOG << "from left closest bev_road_id: " << bev_road_id;
          bev_guide_lanes_with_sd.insert(bev_road_id);
        }
      }
    } else {
      auto bev_ego_root_section_x0_right = std::vector<uint64_t>(
          bev_ego_root_section_x0.end() - std::min(sd_target_lane.lane_num, bev_ego_root_section_x0.size()), bev_ego_root_section_x0.end());
      for (int i = 1; i <= bev_ego_root_section_x0_right.size(); ++i) {
        ego_lane_index_right.emplace_back(i);
        SD_FINE_MATCH_LOG << "bev_ego_id_right: " << bev_ego_root_section_x0_right.at(i - 1);
      }
      auto find_bev = std::find(ego_lane_index_right.begin(), ego_lane_index_right.end(), sd_target_index);
      if (find_bev != ego_lane_index_right.end()) {
        auto &bev_road_id = bev_ego_root_section_x0_right.at(std::distance(ego_lane_index_right.begin(), find_bev));
        SD_FINE_MATCH_LOG << "from right bev_road_id: " << bev_road_id;
        bev_guide_lanes_with_sd.insert(bev_road_id);
      } else {
        auto find_closest =
            std::min_element(ego_lane_index_right.begin(), ego_lane_index_right.end(), [&sd_target_index](const int &a, const int &b) {
              return std::abs(a - static_cast<int>(sd_target_index)) < std::abs(b - static_cast<int>(sd_target_index));
            });
        if (find_closest != ego_lane_index_right.end()) {
          auto &bev_road_id = bev_ego_root_section_x0_right.at(std::distance(ego_lane_index_right.begin(), find_closest));
          SD_FINE_MATCH_LOG << "from right closest bev_road_id: " << bev_road_id;
          bev_guide_lanes_with_sd.insert(bev_road_id);
        }
      }
    }
  }
  std::unordered_map<uint64_t, BevLaneInfo *> lane_map;
  lane_map.clear();
  for (auto &lane : GlobalBevMapOutPut->lane_infos) {
    lane_map[lane.id] = &lane;
  }
  std::vector<uint64_t> match_guide_lanes;
  for (auto &ego_start_id : bev_guide_lanes_with_sd) {
    match_guide_lanes.clear();
    if (lane_map.find(ego_start_id) == lane_map.end()) {
      return;
    }

    std::queue<uint64_t>         q;
    std::unordered_set<uint64_t> visited;

    q.push(ego_start_id);
    visited.insert(ego_start_id);

    while (!q.empty()) {
      uint64_t cur = q.front();
      q.pop();

      const BevLaneInfo *info = lane_map.at(cur);
      if (info->next_lane_ids.empty()) {
        match_guide_lanes.push_back(cur);
      } else {
        for (uint64_t nxt : info->next_lane_ids) {
          if (lane_map.find(nxt) == lane_map.end())
            continue;
          if (visited.insert(nxt).second) {
            q.push(nxt);
          }
        }
      }
    }
  }

  sd_guide_lane_ids = match_guide_lanes;
  for (auto &laneid_tmp : sd_guide_lane_ids) {
    guide_lane_result.push_back({laneid_tmp, {-1, 0}});
  }

  std::ostringstream os1;
  for (auto guide_laneid_tmp : sd_guide_lane_ids) {
    os1 << guide_laneid_tmp << ",";
  }
  SD_FINE_MATCH_LOG << "[SelectGuideLanesWithSd]  : " << os1.str();
  navi_debug_infos.guide_laneids_refer = sd_guide_lane_ids;
}

void SdNavigationCity::JunctionNaviMatch(const JunctionInfoCity &junction, const std::vector<uint64_t> &candidate_lanes_in,
                                         const std::vector<int> &lane_index_left, const std::vector<int> &lane_index_right,
                                         BevMapInfoPtr &GlobalBevMapOutPut, NaviMatchType &match_type, std::vector<uint64_t> &guide_lanes) {
  guide_lanes.clear();
  match_type = NaviMatchType::Unknow;

  // 主辅路分叉：RoadSplit
  if (junction.junction_type_city == JunctionTypeCity::RoadSplit) {
    std::vector<std::string> succ_road_class_str;
    for (const auto &type : junction.succ_road_class) {
      succ_road_class_str.push_back(StrRoadMainType(type));
    }
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
        "Entering RoadSplit branch: id: {}  type: {}  action: {}  "
        "succ_road_class: [{}]",
        junction.junction_id, StrJunctionTypeCity(junction.junction_type_city), StrJunctionAction(junction.junction_action),
        fmt::join(succ_road_class_str, ", "));
    guide_lanes = coarse_matching_.CoarseMatchingType2(junctions_info_city_, junction, candidate_lanes_in, isSplitRoadSelectWorked_,
                                                       complete_section_info_);
    if (IsRoadSplitMain2Ramp(junction)) {
      match_type = NaviMatchType::Type2;
    } else {
      match_type = NaviMatchType::Type9;
    }
  }
  // UTurn 路口
  else if (junction.junction_type_city == JunctionTypeCity::UTurn && junction.junction_action == JunctionAction::UTurn) {
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Entering UTurn branch: id: {}  type: {}  action: {}", junction.junction_id,
                                             StrJunctionTypeCity(junction.junction_type_city), StrJunctionAction(junction.junction_action));
    guide_lanes = coarse_matching_.CoarseMatchingType7(junction, candidate_lanes_in);
    match_type  = NaviMatchType::Type7;  // UTurn 粗匹配
  }
  // 十字路口 或 大 T 型路口
  else if (junction.junction_type_city == JunctionTypeCity::CrossRoad || junction.junction_type_city == JunctionTypeCity::TJunction ||
           junction.junction_type_city == JunctionTypeCity::SmallTJunction) {
    if (junction.junction_state_city == JunctionStateCity::UNREACHED) {
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format(
          "Entering CrossRoad or TJunction branch: id: {}  type: {}  state: "
          "{}  offset: {:.2f}",
          junction.junction_id, StrJunctionTypeCity(junction.junction_type_city), StrJunctionStateCity(junction.junction_state_city),
          junction.offset);
      if (junction.offset > NavigationConfig::fine_matching_threshold) {
        // 距离 > 配置的精匹配阈值：粗匹配
        // guide_lanes = coarse_matching_.CoarseMatchingType3(junction,
        // candidate_lanes_in);
        guide_lanes = coarse_matching_.CoarseMatchingType33(junctions_info_city_, junction, candidate_lanes_in, complete_section_info_);
        match_type  = NaviMatchType::Type3;
      } else {
        // 距离 <= 配置的精匹配阈值：精匹配
        SD_COARSE_MATCH_TYPE2_LOG << "[JunctionNaviMatch] PerformFineMatching...";
        guide_lanes = PerformFineMatching(junction, candidate_lanes_in, lane_index_left, lane_index_right, GlobalBevMapOutPut);
        match_type  = NaviMatchType::Type4;
      }
      auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
      auto &arrows_tmp       = junction.map_lane_arrows_plan;
      for (auto &arrow : arrows_tmp) {
        navi_debug_infos.sd_arrows.push_back((int)arrow);
      }
      navi_debug_infos.target_lanes_index = junction.main_lane_indexs;

    } else if (junction.junction_state_city == JunctionStateCity::PASSING) {
      match_type = NaviMatchType::Type5;
      // 输出虚拟引导线
    }
  }
}

void SdNavigationCity::CalcBevIndex(const std::vector<uint64_t> &road_selected, std::vector<int> &lane_index_left,
                                    std::vector<int> &lane_index_right) {
  lane_index_left.clear();
  lane_index_right.clear();
  if (road_selected.size() > 1) {
    for (size_t i = 0; i < road_selected.size() - 1; i++) {
      int  nearest_count = 0;
      auto lane_i        = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected.at(i));
      auto lane_j        = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected.at(i + 1));
      if (!lane_i || !lane_j || lane_i->geos->empty() || lane_j->geos->empty() || lane_i->line_points.size() < 2 ||
          lane_j->line_points.size() < 2) {
        continue;
      }
      double overlap = LaneGeometry::CalcLinesOverlap(*lane_i->geos, *lane_j->geos, 2);
      SD_FINE_MATCH_LOG << road_selected.at(i) << "---" << road_selected.at(i + 1) << " overlap: " << overlap;
      if (overlap > 0.6) {
        for (const auto &lane_pt_i : lane_i->line_points) {
          int nearest_index = lane_j->indexed_geos.FindNearestPoint(lane_pt_i.x, lane_pt_i.y);
          if (nearest_index >= 0 && nearest_index < static_cast<int>(lane_j->geos->size())) {
            double nearest_distance = fabs(lane_pt_i.y - lane_j->geos->at(nearest_index).y());
            // SD_FINE_MATCH_LOG << "nearest_distance: " << nearest_distance;
            if (nearest_distance < 1.2) {
              nearest_count++;
            }
          }
        }
        // SD_FINE_MATCH_LOG << "nearest_count: " << nearest_count;
        // SD_FINE_MATCH_LOG << "lane_i size: " << lane_i->line_points.size();
        if ((static_cast<float>(nearest_count) / lane_i->line_points.size()) > 0.5) {
          // SD_FINE_MATCH_LOG << "has repeated lane";
          float smoothness1 = CalcLineSmoothness(*lane_i);
          float smoothness2 = CalcLineSmoothness(*lane_j);
          // SD_FINE_MATCH_LOG << "smoothness: " << smoothness1 << ", "
          //                   << smoothness2;
          if (smoothness1 < smoothness2) {
            road_selected_repeat_.insert(road_selected.at(i + 1));
            road_selected_map_[road_selected.at(i)].insert(road_selected.at(i + 1));
          } else {
            road_selected_repeat_.insert(road_selected.at(i));
            road_selected_map_[road_selected.at(i + 1)].insert(road_selected.at(i));
          }
        }
      }
    }
  }

  for (auto &road_id : road_selected) {
    if (road_selected_repeat_.count(road_id) > 0) {
      continue;
    }
    road_selected_candidate_.emplace_back(road_id);
  }

  int road_num = road_selected_candidate_.size();
  if (bev_egoRoadEdgeIndexPair_.first != -1 && bev_egoRoadEdgeIndexPair_.second != -1) {
    lane_index_left.reserve(road_num);
    lane_index_right.reserve(road_num);
    // auto ego_left_road_edge_id  =
    // bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id; auto
    // ego_right_road_edge_id =
    // bev_line_sorts_[bev_egoRoadEdgeIndexPair_.second].id; auto
    // roadEdge_left          =
    // INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_right_road_edge_id);
    // auto roadEdge_right         =
    // INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_right_road_edge_id);

    // Todo:和路沿/车道间的距离判断
    for (int i = 0; i < road_num; ++i) {
      lane_index_left.emplace_back(i);
      lane_index_right.emplace_back(road_num - i - 1);
    }
    return;
  }

  /*
  if (bev_egoRoadEdgeIndexPair_.first != -1 &&
  bev_egoRoadEdgeIndexPair_.second == -1) {
  lane_index_left.reserve(road_num);
  auto ego_left_road_edge_id =
  bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id;
  auto roadEdge_left =
  INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_left_road_edge_id);
  // Todo:和路沿/车道间的距离判断
  for (int i = 0; i < road_num; ++i) {
  lane_index_left.emplace_back(i);
  }
  }
  */
  // 临时优化：应对右侧转向专用道
  lane_index_left.reserve(road_num);
  lane_index_right.reserve(road_num);
  // Todo:和路沿/车道间的距离判断
  for (int i = 0; i < road_num; ++i) {
    lane_index_left.emplace_back(i);
    lane_index_right.emplace_back(road_num - i - 1);
  }
  /*
    if (bev_egoRoadEdgeIndexPair_.first == -1 &&
    bev_egoRoadEdgeIndexPair_.second != -1) {
      lane_index_right.reserve(road_num);
      auto ego_right_road_edge_id =
    bev_line_sorts_[bev_egoRoadEdgeIndexPair_.second].id; auto roadEdge_right
    = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_right_road_edge_id);
      // Todo:和路沿/车道间的距离判断
      for (int i = 0; i < road_num; ++i) {
        lane_index_right.emplace_back(road_num - i - 1);
      }
    }
  */
}

float SdNavigationCity::CalcLineSmoothness(const BevLaneInfo &line) {
  float smoothness = 0.0f;
  for (size_t i = 2; i < line.line_points.size(); ++i) {
    float dx1   = line.line_points[i - 1].x - line.line_points[i - 2].x;
    float dy1   = line.line_points[i - 1].y - line.line_points[i - 2].y;
    float dx2   = line.line_points[i].x - line.line_points[i - 1].x;
    float dy2   = line.line_points[i].y - line.line_points[i - 1].y;
    float angle = std::acos((dx1 * dx2 + dy1 * dy2) / (std::sqrt(dx1 * dx1 + dy1 * dy1) * std::sqrt(dx2 * dx2 + dy2 * dy2)));
    smoothness += angle;
  }
  return smoothness;
}

/*
 * @brief 打印edge信息
 */
void SdNavigationCity::Edge_log_print(const std::vector<BevLaneMarker> &all_bev_road_edges) {
  for (const auto edge : all_bev_road_edges) {
    if (!edge.geos->empty()) {
      SD_COARSE_MATCH_LOG << "edge_lane_id: " << edge.id << " edge_start_x: " << edge.geos->front().x()
                          << " edge_start_y: " << edge.geos->front().y() << " ledge_end_x: " << edge.geos->back().x()
                          << " edge_end_y: " << edge.geos->back().y();
      switch (edge.road_edeg_pos) {
        case BevRoadEdgePosition::BEV_REP__UNKNOWN:
          SD_COARSE_MATCH_LOG << "BEV_REP__UNKNOWN ";
          break;
        case BevRoadEdgePosition::BEV_REP__LEFT:
          SD_COARSE_MATCH_LOG << "BEV_REP__LEFT";
          break;
        case BevRoadEdgePosition::BEV_REP__RIGHT:
          SD_COARSE_MATCH_LOG << "BEV_REP__RIGHT ";
          break;
        default:
          SD_COARSE_MATCH_LOG << "BEV_REP__UNKNOWN ";
          break;
      }
#if 0
         if ((!edge.geos->empty())) {

        const auto &bdry_points = *(edge.geos);
        for (size_t i = 0; i < bdry_points.size(); ++i) {
          const Eigen::Vector2f &point = bdry_points[i];
          SD_COARSE_MATCH_LOG << "点 " << i << ": (" << point.x() << ", " << point.y() << ")" << std::endl;
        }
      }
#endif
    }
  }
}
/*
  * @brief 根据港湾车道信息，去除感知的港湾车道id
  * @param harbor  港湾车道信息
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
void SdNavigationCity::Erase_Harborlane_Ids(const HarborStopInfo             &sd_harbor_stop_info_,
                                            std::vector<uint64_t>            &road_selected_buslane_filtered,
                                            const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list) {

  /*获取当前section的车道线属性，包括起终点、merge topo、split  topo信息*/
  /*遍历 road_selected_buslane_filtered, 取出其中的每个id*/
  std::vector<uint64_t> indices_id_to_remove   = {};
  size_t                lane_count             = 0;
  double                min_overlap_threshold  = 20.0f;  // 最小重叠长度阈值(米)
  double                max_distance_threshold = 2.8f;   // 最大距离阈值(米)

  for (auto &id : road_selected_buslane_filtered) {
    auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
    if ((single_lane) && (!single_lane->geos->empty())) {
      SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << single_lane->geos->front().x()
                          << " lane_start_y: " << single_lane->geos->front().y() << " lane_end_x: " << single_lane->geos->back().x()
                          << " lane_end_y: " << single_lane->geos->back().y();

      //通过switch  判断所有SplitTopoExtendType，并打印日志
      SD_COARSE_MATCH_LOG << StrSplitTopologyExtendType(single_lane->split_topo_extend);
      SD_COARSE_MATCH_LOG << StrMergeTopologyExtendType(single_lane->merge_topo_extend);

      double overlap_dis = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
      SD_COARSE_MATCH_LOG << "overlap_dis:" << overlap_dis;
      if ((overlap_dis > min_overlap_threshold)) {
        lane_count++;
        if ((single_lane->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) ||
            (single_lane->merge_topo_extend == cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_LEFT)) {
          SD_COARSE_MATCH_LOG << "harbor single_lane id is  :" << single_lane->id;
          indices_id_to_remove.push_back(single_lane->id);
        } else if ((sd_harbor_stop_info_.is_right_most) && (id == road_selected_buslane_filtered.back()) &&
                   HasAncestorWithSplitRight(single_lane)) {
          /* 前继车道的topo是否分叉 */
          indices_id_to_remove.push_back(single_lane->id);
          SD_COARSE_MATCH_LOG << "harbor single_lane (by pre) id is  :" << single_lane->id;

        } else {
        }
      }
    }
  }

  /* 自行判断split场景 */
  if (road_selected_buslane_filtered.size() > 2) {
    size_t   last_idx         = road_selected_buslane_filtered.size() - 1;
    uint64_t last_lane        = road_selected_buslane_filtered[last_idx];
    uint64_t second_last_lane = road_selected_buslane_filtered[last_idx - 1];
    if (coarse_matching_.CheckLanesShareAncestor(last_lane, second_last_lane)) {
      indices_id_to_remove.push_back(last_lane);
    }
  }

  /*若没有通过拓扑找到港湾车道，则利用每条线到右边界的距离进行判断*/
  //Edge_log_print(all_bev_edges);
  /* 添加 obstacle_harbor的判断 */
  if (indices_id_to_remove.empty() && sd_harbor_stop_info_.exists && !sd_harbor_stop_info_.is_obstacle_harbor) {
    // 步骤1: 找出与公交港湾重叠最长且大于阈值的边界线
    // int           best_boundary_idx = -1;
    BevLaneMarker best_edge;
    double        max_overlap = -1.0;
    for (auto &edge : all_bev_edges) {
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() < 0))) {
        double overlap_dis = calculateOverlapDistance(sd_harbor_stop_info_, *edge.geos);
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] bdry_2_harbor overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge   = edge;
        }
      }
    }
    // 步骤2: 找出与该边界线宽度最小且小于阈值的车道线
    uint64_t best_lane_id = 0;
    float    min_distance = std::numeric_limits<float>::max();
    for (auto &id : road_selected_buslane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_edge.geos->empty())) {
        double overlap_dis = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
        //double dis_to_rightbdry = cem::fusion::LaneGeometry::GetDistanceBetweenLines(*single_lane->geos, *best_edge.geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*single_lane->geos, *best_edge.geos);

        // double overlap_dis      = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] dis line " << single_lane->id << " to " << best_edge.id
                            << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry > 0.5 && dis_to_rightbdry < min_distance && dis_to_rightbdry <= max_distance_threshold &&
            overlap_dis >= min_overlap_threshold) {
          min_distance = dis_to_rightbdry;
          best_lane_id = id;
        }
      }
    }
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]best_edge id: " << best_edge.id << "  max_overlap: " << max_overlap;
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]best_lane_id: " << best_lane_id << "  min_distance: " << min_distance;
    if (best_lane_id != 0) {
      indices_id_to_remove.push_back(best_lane_id);
    }
  }

  // 使用remove-erase移除指定ID的元素
  if (sd_harbor_stop_info_.exists) {
    // 获取第一个港湾车道范围 [start_s, end_s]
    std::pair<double, double> firstRange = {0, 0};
    // 获取第一个港湾车道范围 [start_s, end_s]
    for (auto range : sd_harbor_stop_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
    //    const auto &firstRange   = sd_harbor_stop_info_.ranges[0];
    double harborStart   = firstRange.first;
    double harborEnd     = firstRange.second;
    bool   remove_enable = RemoveHarborlaneCheck(sd_harbor_stop_info_, road_selected_buslane_filtered);
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] harborStart:" << harborStart << ", harborEnd:" << harborEnd
                        << " remove_enable: " << remove_enable;
    bool is_last_elem = IsContainsLastElement(road_selected_buslane_filtered, indices_id_to_remove);
    if ((harborStart < 80.0) && (harborEnd > 0.0) && (lane_count > 2) && remove_enable) {  //50->80
      if ((!indices_id_to_remove.empty()) && is_last_elem) {
        for (int id : indices_id_to_remove) {
          road_selected_buslane_filtered.erase(
              std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
              road_selected_buslane_filtered.end());
        }
      } /* 若仍未找到港湾车道，利用车道数来判断 */
      else if ((lane_count > lanetype_list.size()) && ((harborStart < 50.0) && (harborStart > 0.0))) {
        road_selected_buslane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]  pop_back in front ";
      } else if ((lane_count >= lanetype_list.size()) && ((harborStart < 0.0) && (harborEnd > 0.0))) {
        road_selected_buslane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] pop_back between";
      }
    } else if ((harborStart < 80.0) && (harborEnd > 0.0) && (lane_count == 2) && remove_enable) {  //50->80
      if ((!indices_id_to_remove.empty()) && is_last_elem) {  //保守处理，只过滤最右筛选出的港湾车道
        for (int id : indices_id_to_remove) {
          road_selected_buslane_filtered.erase(
              std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
              road_selected_buslane_filtered.end());
        }
      }
    } else {
    }
  }
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]Current bevlane num  :" << lane_count;
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]road_selected_buslane_filtered: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_buslane_filtered, ", "))
                      << " size: " << road_selected_buslane_filtered.size();
}
/*
  * @brief 根据港湾车道信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
double SdNavigationCity::calculateOverlapDistance(const HarborStopInfo &harbor, const std::vector<Eigen::Vector2f> geos) {
  //SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Entering...]";
  // 检查港湾车道是否存在且有范围
  if (!harbor.exists || harbor.ranges.empty()) {
    return 0.0;
  }

  // 获取第一个港湾车道范围 [start_s, end_s]
  std::pair<double, double> firstRange = {0, 0};
  for (auto range : harbor.ranges) {
    if (range.second > 0) {
      firstRange = range;
      break;
    }
  }

  // const auto &firstRange  = harbor.ranges[0];
  double harborStart = firstRange.first;
  double harborEnd   = firstRange.second;

  // 确保范围有效
  if (harborStart >= harborEnd) {
    return 0.0;
  }

  // 获取车道线的geos点集
  //const auto& geos = *singlelane.geos;
  if (geos.size() < 2) {
    return 0.0;
  }

  // 直接使用第一个点和最后一个点的x坐标作为车道线的起点和终点
  double laneStart = geos.front().x();
  double laneEnd   = geos.back().x();

  // 确保车道线起点和终点的顺序正确
  if (laneStart > laneEnd) {
    std::swap(laneStart, laneEnd);
  }

  // 计算重叠部分
  double overlapStart = std::max(laneStart, harborStart);
  double overlapEnd   = std::min(laneEnd, harborEnd);
  //SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Exiting...]"
  //                    << " overlapStart: " << overlapStart << " overlapEnd: " << overlapEnd;
  // 如果存在重叠，返回重叠距离
  return (overlapStart < overlapEnd) ? (overlapEnd - overlapStart) : 0.0;
}

/* 
@brief 根据地图车道列表，获取公交车道信息
*/
void SdNavigationCity::GetBusLaneInfo() {
  auto sd_map_info = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_map_info) {
    AWARN << "[GetBusLaneInfo] Failed: sd_map_info is null!";
    bus_lane_info_.exists = false;  // 确保全局变量重置
    return;
  }

  uint64_t ego_section_id = sd_map_info->navi_start.section_id;
  double   ego_s_offset   = sd_map_info->navi_start.s_offset;

  // 定位自车所在section
  int ego_section_index = -1;
  for (size_t i = 0; i < sd_map_info->mpp_sections.size(); ++i) {
    if (sd_map_info->mpp_sections[i].id == ego_section_id) {
      ego_section_index = static_cast<int>(i);
      break;
    }
  }
  if (ego_section_index == -1) {
    AWARN << "[GetBusLaneInfo] Failed: ego_section not found!";
    bus_lane_info_.exists = false;
    return;
  }

  // 计算自车全局s坐标
  double ego_global_s = 0.0;
  for (int i = 0; i < ego_section_index; ++i) {
    ego_global_s += sd_map_info->mpp_sections[i].length;
  }
  ego_global_s += ego_s_offset;

  // 设置搜索范围（相对自车位置）
  const double min_s = -100.0;
  const double max_s = 300.0;

  // 初始化结果
  bus_lane_info_.exists = false;
  bus_lane_info_.ranges.clear();
  bus_lane_info_.is_left_most  = false;
  bus_lane_info_.is_right_most = false;
  bus_lane_info_.index         = -1;

  std::vector<std::pair<double, double>> bus_ranges;

  // 临时存储公交车道区间及属性
  std::vector<BusLaneInfo> temp_bus_lanes;
  double                   current_global_s = 0.0;

  // 遍历所有section
  for (size_t i = 0; i < sd_map_info->mpp_sections.size(); ++i) {
    const auto  &section       = sd_map_info->mpp_sections[i];
    const double section_end_s = current_global_s + section.length;

    // 跳过超出搜索范围的section
    if (section_end_s < ego_global_s + min_s || current_global_s > ego_global_s + max_s) {
      current_global_s = section_end_s;
      continue;
    }

    // 遍历当前section的车道组
    for (const auto &lg_idx : section.lane_group_idx) {
      const auto lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (!lane_group || lane_group->lane_info.empty())
        continue;

      // 计算车道组全局s范围
      const double lg_start_s = current_global_s + lg_idx.start_range_offset;
      const double lg_end_s   = current_global_s + lg_idx.end_range_offset;

      // 检查车道组是否在搜索范围内
      if (lg_end_s < ego_global_s + min_s || lg_start_s > ego_global_s + max_s)
        continue;

      // 遍历车道组内的每条车道
      for (int j = 0; j < lane_group->lane_info.size(); ++j) {
        const auto &lane = lane_group->lane_info[j];
        const bool  is_bus_lane =
            (lane.type == cem::message::env_model::LaneType::LANE_BUS_NORMAL || lane.type == cem::message::env_model::LaneType::LANE_BRT);

        if (is_bus_lane) {
          // 计算相对自车的s范围
          const double lane_start_s = std::max(lg_start_s - ego_global_s, min_s);
          const double lane_end_s   = std::min(lg_end_s - ego_global_s, max_s);

          if (lane_start_s < lane_end_s) {
            BusLaneInfo bus_lane;
            bus_lane.ranges.emplace_back(lane_start_s, lane_end_s);
            bus_lane.exists        = true;
            bus_lane.is_left_most  = (j == 0);                                 // 是否最左车道
            bus_lane.is_right_most = (j == lane_group->lane_info.size() - 1);  // 是否最右车道
            bus_lane.is_passable   = lane.restricted_info.is_passable;         // 是否可通行
            // bus_lane.passable_env_state = (lane.restricted_info.is_passable == true) ? 1 : 2;  // 是否可通行
            bus_lane.passable_env_state = lane.restricted_info.passable_env_state;
            bus_lane.index              = j;  // 从左到右的索引
            if (lane_end_s > 0) {
              temp_bus_lanes.push_back(bus_lane);
            }

            // 动态合并区间
            if (!bus_ranges.empty() && lane_start_s <= bus_ranges.back().second) {
              bus_ranges.back().second = std::max(bus_ranges.back().second, lane_end_s);
            } else {
              bus_ranges.emplace_back(lane_start_s, lane_end_s);
            }
          }
        }
      }
    }
    current_global_s = section_end_s;
  }

  // 合并结果到全局变量
  if (!temp_bus_lanes.empty()) {
    bus_lane_info_.exists = true;
    // 取第一个检测到的公交车道属性（可根据需求调整）
    bus_lane_info_.is_left_most       = temp_bus_lanes[0].is_left_most;
    bus_lane_info_.is_right_most      = temp_bus_lanes[0].is_right_most;
    bus_lane_info_.is_passable        = temp_bus_lanes[0].is_passable;
    bus_lane_info_.passable_env_state = temp_bus_lanes[0].passable_env_state;
    bus_lane_info_.index              = temp_bus_lanes[0].index;
    bus_lane_info_.ranges             = bus_ranges;

    // 日志输出
    SD_COARSE_MATCH_LOG << "[GetBusLaneInfo] Bus lanes detected:";
    for (const auto &range : bus_lane_info_.ranges) {
      SD_COARSE_MATCH_LOG << " [" << range.first << ", " << range.second << "]";
    }
    SD_COARSE_MATCH_LOG << " LeftMost: " << bus_lane_info_.is_left_most << ", RightMost: " << bus_lane_info_.is_right_most
                        << ", is_passable: " << bus_lane_info_.is_passable << ", passable_env_state: " << bus_lane_info_.passable_env_state
                        << ", Index: " << bus_lane_info_.index;
  } else {
    SD_COARSE_MATCH_LOG << "[GetBusLaneInfo] No bus lanes found.";
  }
}

/*
  * @brief 根据地图车道列表，获取港湾车道信息
  * @return sd_harbor_stop_info_  港湾车道信息
*/
void SdNavigationCity::GetHarborStopInfo() {
  auto sd_map_info = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_map_info) {
    AWARN << "[GetHarborStopInfo] Failed: sd_map_info is null!";
    return;
  }

  uint64_t ego_section_id = sd_map_info->navi_start.section_id;
  double   ego_s_offset   = sd_map_info->navi_start.s_offset;

  // SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  int ego_section_index = -1;
  for (size_t i = 0; i < sd_map_info->mpp_sections.size(); ++i) {
    if (sd_map_info->mpp_sections[i].id == ego_section_id) {
      ego_section_index = static_cast<int>(i);
      break;
    }
  }
  if (ego_section_index == -1) {
    AWARN << "[GetHarborStopInfo] Failed: ego_section not found!";
    return;
  }

  // 计算自车的全局 s 坐标
  double ego_global_s = 0.0;
  for (int i = 0; i < ego_section_index; ++i) {
    ego_global_s += sd_map_info->mpp_sections[i].length;
  }
  ego_global_s += ego_s_offset;

  // SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego global s: " << ego_global_s;

  double min_s = -100;  // -100.0
  double max_s = 300;   // 300.0

  sd_harbor_stop_info_.exists = false;
  sd_harbor_stop_info_.ranges.clear();

  std::vector<std::pair<double, double>> harbor_stop_ranges;
  bool                                   current_leftmost  = false;
  bool                                   current_rightmost = false;

  double current_s = 0.0;
  /*遍历section*/
  for (size_t i = 0; i < sd_map_info->mpp_sections.size(); ++i) {
    const auto &section         = sd_map_info->mpp_sections[i];
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s) {
      current_s += section.length;
      continue;
    }

    /* SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Section ID: " << section.id
                       << " start from s: " << section_start_s 
                       << " to s: " << section_end_s; */

    // 遍历 Section 中的 LaneGroup
    for (const auto &lg_idx : section.lane_group_idx) {
      const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (lane_group && !lane_group->lane_info.empty()) {
        double lg_start_s = section_start_s + lg_idx.start_range_offset;
        double lg_end_s   = section_start_s + lg_idx.end_range_offset;

        /* SD_COARSE_MATCH_LOG << "  [GetHarborStopInfo] LaneGroup ID: " << lg_idx.id
                            << " start from s: " << lg_start_s
                            << " to s: " << lg_end_s
                            << ", lanes: " << lane_group->lane_info.size(); */

        // 遍历车道组，检测港湾停靠站
        //for (const auto &lane : lane_group->lane_info) {
        for (size_t j = 0; j < lane_group->lane_info.size(); ++j) {  // 改为 size_t j，方便访问索引
          const auto &lane = lane_group->lane_info[j];
          if (lane.type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
            double lane_start_s = std::max(lg_start_s - ego_global_s, min_s);
            double lane_end_s   = std::min(lg_end_s - ego_global_s, max_s);
            if (lane_start_s < lane_end_s) {
              // 动态合并港湾停靠站区间
              if (!harbor_stop_ranges.empty() && lane_start_s <= harbor_stop_ranges.back().second) {
                harbor_stop_ranges.back().second = std::max(harbor_stop_ranges.back().second, lane_end_s);
              } else {
                harbor_stop_ranges.emplace_back(lane_start_s, lane_end_s);
              }

              // 检查是否在最左或最右位置
              if (j == 0) {
                current_leftmost = true;
              }
              if (j == lane_group->lane_info.size() - 1) {
                current_rightmost = true;
              }

              // 新增判断：检查左侧车道是否为 LANE_OBSTACLE
              if (j > 0) {  // 存在左侧车道
                const auto &left_lane = lane_group->lane_info[j - 1];
                if (left_lane.type == cem::message::env_model::LaneType::LANE_OBSTACLE) {
                  sd_harbor_stop_info_.is_obstacle_harbor = true;
                }
              }
              /*SD_COARSE_MATCH_LOG << "    Harbor stop lane detected, range: [" 
                                  << lane_start_s << ", " << lane_end_s << "]";*/
            }
          }
        }
      }
    }
    current_s += section.length;
  }

  sd_harbor_stop_info_.ranges        = harbor_stop_ranges;
  sd_harbor_stop_info_.exists        = !harbor_stop_ranges.empty();
  sd_harbor_stop_info_.is_left_most  = current_leftmost;
  sd_harbor_stop_info_.is_right_most = current_rightmost;

  if (sd_harbor_stop_info_.exists) {
    SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Harbor stop exists , is_obstacle_harbor:  " << sd_harbor_stop_info_.is_obstacle_harbor
                        << " in merged ranges:";
    for (const auto &range : sd_harbor_stop_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    SD_COARSE_MATCH_LOG << "  is_left_most: " << sd_harbor_stop_info_.is_left_most
                        << ", is_right_most: " << sd_harbor_stop_info_.is_right_most;
  } else {
    SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] No harbor stop found.";
  }
}
/*
  * @brief 根据地图车道列表，去除感知的公交车道id
  * @param lanetype_list  地图车道列表
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
void SdNavigationCity::Erase_Buslane_Ids(std::vector<LaneType> &lanetype_list, std::vector<uint64_t> &road_selected) {

  // 收集所有公交车道/港湾车道的索引，车道数较少(<=3)时，只过滤港湾
  std::vector<size_t> indices_to_remove;
  for (size_t idx = 0; idx < lanetype_list.size(); ++idx) {
    if (lanetype_list.size() <= 3) {
      if (lanetype_list[idx] == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
        indices_to_remove.push_back(idx);
        SD_COARSE_MATCH_LOG << "[Erase_Buslane_Ids]map idx: " << idx << " is harbor lane";
      }
    } else {

      if ((lanetype_list[idx] == cem::message::env_model::LaneType::LANE_BUS_NORMAL)) {
        indices_to_remove.push_back(idx);
        SD_COARSE_MATCH_LOG << "[Erase_Buslane_Ids]map idx: " << idx << " is  bus lane";
      }
    }
  }
  // 确保两个vector数量相等
  if (lanetype_list.size() == road_selected.size()) {
    SD_COARSE_MATCH_LOG << "[Erase_Buslane_Ids]lanetype_list.size() == road_selected.size(): " << lanetype_list.size();
    // 从后向前删除road_selected中对应索引的元素
    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
      if (*it < road_selected.size()) {
        road_selected.erase(road_selected.begin() + *it);
      }
    }
  } else if (lanetype_list.size() > road_selected.size()) {
    SD_COARSE_MATCH_LOG << "[Erase_Buslane_Ids]lanetype_list.size() > road_selected.size(): " << lanetype_list.size();
    // 使用反向迭代器
    for (auto it = indices_to_remove.rbegin(); it != indices_to_remove.rend(); ++it) {
      if (*it < road_selected.size()) {
        road_selected.erase(road_selected.begin() + *it);
      }
    }
  } else {
    // 无需处理 lanetype_list.size() < road_selected.size() 的情况
    //road_selected.resize(lane_type_list.size());
  }

  return;
}
/*
  * @brief 获取当前地图的LaneType列表
  * @param raw_routing_map  SDMap的信息
  * @return 当前地图的LaneType列表
*/
std::vector<LaneType> SdNavigationCity::GetCurLaneTypeList() {
  //定义输出为默认值
  std::vector<LaneType> lane_type_list = {};
  // SD_COARSE_MATCH_LOG << "Entering GetCurLaneTypeList";
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    AWARN << "[GetCurLaneTypeList] sd_route is null";
    SD_COARSE_MATCH_LOG << "sd_route is null";
    return {};
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "[GetCurLaneTypeList] mpp_sections is empty";
    SD_COARSE_MATCH_LOG << "mpp_sections is empty";
    return {};
  }

  uint64_t current_section_id = sd_route->navi_start.section_id;
  SD_COARSE_MATCH_LOG << "current_section_id: " << current_section_id;

  const SDSectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[GetCurLaneTypeList] current section not found";
    SD_COARSE_MATCH_LOG << "current section not found";
    return {};
  }

  double current_s_offset = sd_route->navi_start.s_offset;
  // SD_COARSE_MATCH_LOG << "current_s_offset: " << current_s_offset;
  /*获取当前group idx*/
  const SDLaneGroupIndex *current_lane_group_idx = nullptr;
  const SDLaneGroupIndex *next_lane_group_idx    = nullptr;
  // 使用迭代器遍历lane_group_idx容器
  auto it  = current_section->lane_group_idx.begin();
  auto end = current_section->lane_group_idx.end();

  for (; it != end; ++it) {
    const auto &lg_idx = *it;
    if (current_s_offset >= lg_idx.start_range_offset && current_s_offset < lg_idx.end_range_offset) {
      current_lane_group_idx = &lg_idx;

      // 获取下一个元素的迭代器
      auto next_it = std::next(it);
      if (next_it != end) {
        // 下一个元素存在，处理它
        next_lane_group_idx = &(*next_it);
        // 使用next_lg_idx进行后续操作
      } else {
        // 已经是最后一个元素，没有下一个元素
      }

      break;
    }
  }

  if (!current_lane_group_idx) {
    AWARN << "[GetCurLaneTypeList] current lane group not found";
    SD_COARSE_MATCH_LOG << "current lane group not found";
    return {};
  } else {
    // SD_COARSE_MATCH_LOG << "end_range_offset: " << current_lane_group_idx->end_range_offset;
    // SD_COARSE_MATCH_LOG << "left_offset: " << current_lane_group_idx->end_range_offset - current_s_offset;
  }

  const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(current_lane_group_idx->id);
  // SD_COARSE_MATCH_LOG << "current lane group id: " << current_lane_group_idx->id;

  if (!lane_group) {
    AWARN << "[GetCurLaneTypeList] lane group not found";
    SD_COARSE_MATCH_LOG << "lane group not found";
    return {};
  }
#if 0
 /*后续调整lane_group_next位置*/
 /*获取后继lanegroup，获取其车道数*/
  std::vector<uint64_t> successors =lane_group->successor_lane_group_ids;
  if  (successors.empty()) {
    AWARN << "[GetCurLaneTypeList] successors lane group not found";
    SD_COARSE_MATCH_LOG << "successors lane group not found";
     return {};
  }else{
    /*遍历successors，打印其id*/
    for (auto &successor : successors) {
     SD_COARSE_MATCH_LOG << "successor lane group id: " << successor;
    }
  }
    if (!next_lane_group_idx) {
    AWARN << "[GetCurLaneTypeList]next lane group not found";
    SD_COARSE_MATCH_LOG << "next lane group not found";
    return {};
  }else{
    SD_COARSE_MATCH_LOG << "successor lane group id2: " << next_lane_group_idx->id;
  }

  uint64_t lane_group_id_next = successors[0];
  const auto *lane_group_next = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lane_group_id_next);
  const auto *lane_group_next_2 = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(next_lane_group_idx->id);
  
  if (!lane_group_next) {
    SD_COARSE_MATCH_LOG << "lane_group_next not found";
    return {};
  }
  size_t lane_num_next = static_cast<size_t>(lane_group_next->lane_num);  /* to be deleted*/
  size_t lane_num_next2 = static_cast<size_t>(lane_group_next_2->lane_num);



  SD_COARSE_MATCH_LOG << " GetCurLaneTypeList cur lane_num: " << lane_num<< " next lane_num: " << lane_num_next;
/*车道数信息处理*/
  lane_num_info.cur_lg_id = lane_group->id;
  lane_num_info.cur_lane_num = lane_group->lane_num;
  lane_num_info.next_lg_id = lane_group_id_next;
  lane_num_info.next_lane_num = lane_group_next_2->lane_num;
  lane_num_info.next_lg_offset = current_lane_group_idx->end_range_offset-current_s_offset;
  /*tbd*/
  lane_num_info.next_lane_num_change; 
  lane_num_info.next_lane_num_change_offset; 

      SD_COARSE_MATCH_LOG << "cur_lg_id: "<< lane_num_info.cur_lg_id 
                        <<  " cur_lane_num: "<< lane_num_info.cur_lane_num
                        << " next_lg_id: "<< lane_num_info.next_lg_id
                        << " next_lane_num: "<< lane_num_info.next_lane_num
                        << " next_lg_offset: "<< lane_num_info.next_lg_offset;

#endif
  size_t lane_num         = static_cast<size_t>(lane_group->lane_num);
  auto  &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  bool   is_already_have  = !navi_debug_infos.lane_type.empty();
  for (const auto &lane : lane_group->lane_info) {
    if (lane.type != cem::message::env_model::LaneType::LANE_UNKNOWN) {  //LANE_NON_MOTOR
      lane_type_list.emplace_back(lane.type);
      /*添加debug信息*/
      if (!is_already_have) {
        navi_debug_infos.lane_type.push_back(StrLaneType(lane.type));
      }
      SD_COARSE_MATCH_LOG << "LaneType: " << StrLaneType(lane.type);
    }
  }
  /*判断车道数与车道类型的数量，两者相等或车道数比车道类型数量多1，则打印日志并且返回车道类型列表*/
  if (lane_num == lane_type_list.size() || lane_num == lane_type_list.size() + 1) {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num << " lane_type_list.size(): " << lane_type_list.size();
  } else {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num << " lane_type_list.size(): " << lane_type_list.size() << " (Mismatch detected)";
  }

  // SD_COARSE_MATCH_LOG << "Exiting GetCurLaneTypeList";
  return lane_type_list;
}
/*
   * @brief 过滤掉特殊车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉特殊车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationCity::FilterSpecialLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                          const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                          BevMapInfoPtr &GlobalBevMapOutPut) {
  SD_COARSE_MATCH_LOG << "[FilterSpecialLane]Entering FilterSpecialLane...";
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    return road_selected;
  }
  std::vector<uint64_t> filtered_lanes_without_harbor = {};
  std::vector<uint64_t> filtered_lanes_without_acc    = {};
  std::vector<uint64_t> final_filtered_lanes          = {};
  uint64_t              acc_adj_lane_id               = 0;
  JunctionInfoCity      merge_jct;
  bool                  approaching_merge = IsApproachMergeJCT(merge_jct);  //途径汇入口
  /* 港湾车道的过滤 */
  filtered_lanes_without_harbor = FilterBusLane(raw_routing_map, road_selected, raw_bev_map, GlobalBevMapOutPut);

  UpdateHarborLaneType(road_selected, filtered_lanes_without_harbor, GlobalBevMapOutPut);

  if (approaching_merge) {
    SD_COARSE_MATCH_LOG << "[FilterSpecialLane] FilterACCLane...";
    filtered_lanes_without_acc =
        FilterACCLane(raw_routing_map, filtered_lanes_without_harbor, raw_bev_map, approaching_merge, merge_jct, acc_adj_lane_id);
  } else {
    SD_COARSE_MATCH_LOG << "[FilterSpecialLane] other situation maybe ramp merge or main road. acc filter is not valid.";
  }
  SD_COARSE_MATCH_LOG << "[FilterSpecialLane]   acc_adj_lane_id: " << acc_adj_lane_id;

  if (GlobalBevMapOutPut) {
    SetBevAccAdjLane(*GlobalBevMapOutPut, acc_adj_lane_id, merge_jct);
  }

  /* 双向单车道的过滤 */
  final_filtered_lanes = FilterOppositeLane(raw_routing_map, filtered_lanes_without_harbor, raw_bev_map);
  /* 车道方向属性赋值 */
  UpdateOppoLaneDir(filtered_lanes_without_harbor, final_filtered_lanes, GlobalBevMapOutPut);
  SD_COARSE_MATCH_LOG << " [FilterSpecialLane]  final_filtered_lanes : " << fmt::format(" [{}]  ", fmt::join(final_filtered_lanes, ", "));

  return final_filtered_lanes;
}
/*
   * @brief 过滤掉公交车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉公交车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationCity::FilterBusLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                      const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                      BevMapInfoPtr &GlobalBevMapOutPut) {
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    return road_selected;
  }
  auto &all_bev_road_edges = raw_bev_map->edges;

  /*全局debug*/
  auto                 &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  std::vector<uint64_t> road_selected_buslane_filtered(road_selected);
  SD_COARSE_MATCH_LOG << "Entering FilterBusLane function with inputs: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_buslane_filtered, ", ")) << " size: " << road_selected.size();
  // 获取车道列表
  SDLaneNumInfo         lane_num_info;
  std::vector<LaneType> lane_type_list = GetCurLaneTypeList();
  //lane_type_list判空
  if (lane_type_list.empty()) {
    SD_COARSE_MATCH_LOG << "FilterBusLane lane_type_list is empty.";
    navi_debug_infos.sections_without_bus = road_selected_buslane_filtered;
    return road_selected_buslane_filtered;
  }
  /*获取港湾的信息*/
  GetHarborStopInfo();
  GetBusLaneInfo();
  if (sd_harbor_stop_info_.exists) {
    SD_COARSE_MATCH_LOG << "harbor_stop is exists.";
    for (const auto &range : sd_harbor_stop_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    /*检测港湾分叉线，并移除该线*/
    Erase_Harborlane_Ids(sd_harbor_stop_info_, road_selected_buslane_filtered, all_bev_road_edges, lane_type_list);

    // 判断车道列表中是否存在公交车道和港湾车道，若存在，计算其标志位和个数
    bool has_bus_lane    = false;
    bool has_harbor_lane = false;
    int  bus_lane_cnt    = 0;
    int  harbor_lane_cnt = 0;
    int  idx_buslane     = -1;  //公交车道在列表中的索引

    for (size_t i = 0; i < lane_type_list.size(); i++) {
      /*   cem::message::env_model::LaneType::LANE_HOV_NORMAL 暂不考虑，后续看情况使用 */
      if (lane_type_list[i] == cem::message::env_model::LaneType::LANE_BUS_NORMAL ||
          lane_type_list[i] == cem::message::env_model::LaneType::LANE_BRT) {
        has_bus_lane = true;
        bus_lane_cnt++;
        idx_buslane = i;
      }
      if (lane_type_list[i] == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
        has_harbor_lane = true;
        harbor_lane_cnt++;
      }
    }
    SD_COARSE_MATCH_LOG << "has_bus_lane: " << has_bus_lane << " has_harbor_lane: " << has_harbor_lane << " bus_lane_cnt: " << bus_lane_cnt
                        << " harbor_lane_cnt: " << harbor_lane_cnt;
  }
  /* 处理公交车道 */
  if (bus_lane_info_.exists) {
    MatchBusLaneType(bus_lane_info_, road_selected_buslane_filtered, all_bev_road_edges, lane_type_list, GlobalBevMapOutPut);
  }

  // Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
#if 0
    /*判断车道数与road_selected的数量一致性判断，
    如果一致，默认公交车道在最右侧
    且只存在一个公交车道，则将road_selected的最后一个元素去除；
    若只存在一个港湾车道，则将road_selected的最后一个元素去除；
    若存在两个公交车道，则将road_selected的最后两个元素去除；
    若存在两个港湾车道，则将road_selected的最后两个元素去除；
    若存在一个公交车道和一个港湾车道，则将road_selected的最后两个元素去除；
    可能存在感知车道数不完备或者异常的情况，需要上游进行优化
    */
    if ((lane_type_list.size() == road_selected.size()) && (lane_type_list.size() >= 2)) { 
       SD_COARSE_MATCH_LOG << "map lane_num = bev lane_num ";
      if (((has_bus_lane)||(has_harbor_lane))
      &&(bus_lane_cnt + harbor_lane_cnt==1))
      {
        //road_selected_buslane_filtered.pop_back();
        //待地图车道顺序正常后打开
        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
      else if (((has_bus_lane)&&(bus_lane_cnt>1))
      ||((has_harbor_lane)&&(harbor_lane_cnt>1)))
      {
        //road_selected_buslane_filtered.pop_back();
        //road_selected_buslane_filtered.pop_back();
         //待地图车道顺序正常后打开
        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
      else if(((has_bus_lane)&&(has_harbor_lane))
       &&(bus_lane_cnt + harbor_lane_cnt>1))
      {
        //road_selected_buslane_filtered.pop_back();
        //road_selected_buslane_filtered.pop_back();
         //待地图车道顺序正常后打开
        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
    }/*不一致的情况下，默认公交车道在最右侧，和上面类似操作，需要做车道数的约束，细节待细化*/
    /* 不一致分两种情况，一种是地图比感知多，说明感知漏检或上游误过滤了，由于不确定过滤掉的是哪条线，无法进行特殊处理,暂时按照左排序进行过滤；
    另一种是感知比地图多，说明感知多识别出车道的情况了，后续考虑完善处理*/
    else if((lane_type_list.size() > road_selected.size()) && (road_selected.size() >= 2)){
      SD_COARSE_MATCH_LOG << "map lane_num > bev lane_num ";
      if (((has_bus_lane)||(has_harbor_lane))
      &&(bus_lane_cnt + harbor_lane_cnt==1))
      {

        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
        //road_selected_buslane_filtered.pop_back();
      }
      else if (((has_bus_lane)&&(bus_lane_cnt>1))
      ||((has_harbor_lane)&&(harbor_lane_cnt>1)))
      {
        //road_selected_buslane_filtered.pop_back();
        //road_selected_buslane_filtered.pop_back();
        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
      else if(((has_bus_lane)&&(has_harbor_lane))
       &&(bus_lane_cnt + harbor_lane_cnt>1))
      {
        //road_selected_buslane_filtered.pop_back();
        //road_selected_buslane_filtered.pop_back();
        Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
    }
    else if((road_selected.size() > lane_type_list.size()) && (lane_type_list.size() >= 2)){
      SD_COARSE_MATCH_LOG << "map lane_num < bev lane_num ";
      if ((((has_bus_lane)||(has_harbor_lane))&&(bus_lane_cnt + harbor_lane_cnt==1))
      ||((sd_harbor_stop_info_.exists)&& (harborStart < 50)))
      {
        SD_COARSE_MATCH_LOG << "road_selected_buslane_filtered.size "<<road_selected_buslane_filtered.size();
        road_selected_buslane_filtered.pop_back();
        //待地图车道顺序正常，且能提前识别车道变化后开发
       // road_selected_buslane_filtered.resize(lane_type_list.size());
       // Erase_Buslane_Ids(lane_type_list, road_selected_buslane_filtered);
      }
      else if (((has_bus_lane)&&(bus_lane_cnt>1))
      ||((has_harbor_lane)&&(harbor_lane_cnt>1)))
      {
        road_selected_buslane_filtered.pop_back();
        road_selected_buslane_filtered.pop_back();
      }
      else if(((has_bus_lane)&&(has_harbor_lane))
       &&(bus_lane_cnt + harbor_lane_cnt>1))
      {
        road_selected_buslane_filtered.pop_back();
        road_selected_buslane_filtered.pop_back();
      }

    }/*后续需要考虑感知提前输出多车道的情况*/
    else{
      /*nothing todo */
    }
#endif

  /*添加debug信息*/
  navi_debug_infos.sections_without_bus = road_selected_buslane_filtered;
  // 最终结果日志输出
  SD_COARSE_MATCH_LOG << "FilterBusLane Filtered result: " << fmt::format("[{}]", fmt::join(road_selected_buslane_filtered, ", "));
  SD_COARSE_MATCH_LOG << "Exiting FilterBusLane function ";
  return road_selected_buslane_filtered;
}
void SdNavigationCity::UpdateJunctionStateQueue() {
  static constexpr size_t max_queue_size = 50;
  if (junction_current_.junction_type_city == JunctionTypeCity::Unknown) {
    return;
  }
  if (junction_history_state_.size() > max_queue_size) {
    junction_history_state_.pop_front();
  }
  // SD_JUNCTION_CONVERTE_LOG << fmt::format("final_junction_info:{}", junction_current_);
  junction_history_state_.emplace_back(junction_current_);
  junction_previous_ = junction_current_;
}
int SdNavigationCity::IdxInAllMppSection(uint64_t section_id) {
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    return -1;
  }
  const auto &junctions = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->mpp_sections;
  if (route_section_ids_.empty()) {
    std::transform(junctions.begin(), junctions.end(), std::back_inserter(route_section_ids_), [](const auto &sec) { return sec.id; });
  }

  auto it_start_t = std::find(route_section_ids_.begin(), route_section_ids_.end(), junctions.begin()->id);
  auto it_end_t   = std::find(route_section_ids_.begin(), route_section_ids_.end(), junctions.back().id);
  if (it_start_t == route_section_ids_.end()) {
    route_section_ids_.clear();
    std::transform(junctions.begin(), junctions.end(), std::back_inserter(route_section_ids_), [](const auto &sec) { return sec.id; });
  } else if (it_end_t == route_section_ids_.end()) {
    route_section_ids_.erase(it_start_t, route_section_ids_.end());
    std::transform(junctions.begin(), junctions.end(), std::back_inserter(route_section_ids_), [](const auto &sec) { return sec.id; });
  }

  auto it =
      std::find_if(route_section_ids_.begin(), route_section_ids_.end(), [&](const auto &section_t) { return section_t == section_id; });

  if (it != route_section_ids_.end()) {
    return static_cast<int>(std::distance(route_section_ids_.begin(), it));
  }
  return -1;
}

int SdNavigationCity::IdxInMppSection(uint64_t section_id) {
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    return -1;
  }
  const auto &junctions = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->mpp_sections;
  auto it = std::find_if(junctions.begin(), junctions.end(), [&](const SDSectionInfo &section_t) { return section_t.id == section_id; });
  if (it != junctions.end()) {
    return static_cast<int>(std::distance(junctions.begin(), it));
  }
  return -1;
};

bool SdNavigationCity::FindCurrentJunction() {
  // find previous junction
  int previous_idx_in_junctions = -1;
  if (junction_previous_.junction_type_city != JunctionTypeCity::Unknown) {
    auto it_t = std::find_if(junctions_info_city_.begin(), junctions_info_city_.end(),
                             [&](const JunctionInfoCity &info_city) { return info_city.junction_id == junction_previous_.junction_id; });
    if (it_t != junctions_info_city_.end()) {
      previous_idx_in_junctions = static_cast<int>(std::distance(junctions_info_city_.begin(), it_t));
      SD_JUNCTION_CONVERTE_LOG<<"previous_idx_in_junctions:"<<previous_idx_in_junctions;
    }
  }
  uint64_t ego_section_id  = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->navi_start.section_id;
  double   ego_offset      = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->navi_start.s_offset;
  int      ego_index       = IdxInMppSection(ego_section_id);
  int      prev_idx_in_mpp = IdxInMppSection(junction_previous_.junction_id);
  if (prev_idx_in_mpp == -1 || ego_index - prev_idx_in_mpp > 3) {
    previous_idx_in_junctions = -1;
    SD_JUNCTION_CONVERTE_LOG<<"previous_idx_in_junctions:"<<previous_idx_in_junctions;
  }

  std::vector<uint64_t> section_ids;
  for (const auto &sec : INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->mpp_sections) {
    section_ids.emplace_back(sec.id);
  }

  if (ego_index == -1) {
    SD_JUNCTION_CONVERTE_LOG << fmt::format("can't find the ego idx ego_section_id:{}", ego_section_id);
    return false;
  }
  SD_JUNCTION_CONVERTE_LOG << fmt::format(
      "ego_junction_id:{} offset:{:.2f} ego_index_in_mpp:{} prev_idx_in_mpp:{} previous_id:{}  prev_idx_in_junctions:{} section_ids:{}  ",
      ego_section_id, ego_offset, ego_index, prev_idx_in_mpp, junction_previous_.junction_id, previous_idx_in_junctions, section_ids);

  // static CONSTEXPR double ratio_filter = 2.0;
  for (size_t idx = 0; idx < junctions_info_city_.size(); idx++) {
    auto &junction_info_t = junctions_info_city_[idx];
    // int    junction_start_idx_in_mpp = IdxInMppSection(junction_info_t.junction_ids.front());
    // int    junction_end_idx_in_mpp   = IdxInMppSection(junction_info_t.junction_ids.back());
    // double dis_in  = junction_type_dis_in_[junction_info_t.junction_type_city];
    double dis_out = junction_type_dis_out_[junction_info_t.junction_type_city];

    // const auto &sec_start = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(junction_info_t.junction_ids.front());
    // const auto &sec_end = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(junction_info_t.junction_ids.back());

    // SD_JUNCTION_CONVERTE_LOG << fmt::format(
    //     "junction_id:{}  id_start:{}  id_end:{} junction_start_idx_in_mpp:{} junction_end_idx_in_mpp:{}",
    //     junction_info_t.junction_id, junction_info_t.junction_ids.front(),
    //     junction_info_t.junction_ids.back(), junction_start_idx_in_mpp, junction_end_idx_in_mpp);

    if (junction_info_t.offset > 0.0) {
      junction_info_t.junction_state_city = JunctionStateCity::UNREACHED;
      continue;
    }
    if (previous_idx_in_junctions != -1 && (static_cast<int>(idx) < previous_idx_in_junctions)) {
      junction_info_t.junction_state_city = JunctionStateCity::PASSED;
      continue;
    }
    if (junction_info_t.offset < -1.5 * dis_out) {
      junction_info_t.junction_state_city = JunctionStateCity::PASSED;
      continue;
    }
    junctions_around_.emplace_back(idx);
    SD_JUNCTION_CONVERTE_LOG << fmt::format("junction_around_emplace_back_junction:{}", junction_info_t);
  }

  size_t junctions_affected_size = junctions_around_.size();
  SD_JUNCTION_CONVERTE_LOG << fmt::format("junctions_info_city_size:{}  junctions_around_ego_size:{}", junctions_info_city_.size(),
                                          junctions_affected_size);
  fmt::format_to(info_buffer_, "junction_around_size:{}  ", junctions_affected_size);
  if (junctions_affected_size == 0) {
    // auto it = std::find_if(junctions_info_city_.rbegin(), junctions_info_city_.rend(),
    //                        [](const JunctionInfoCity &junc_t) { return junc_t.junction_state_city == JunctionStateCity::PASSED; });
    // if (it != junctions_info_city_.rend()) {
    //   junction_current_ = *it;
    // }
    // SD_JUNCTION_CONVERTE_LOG << "junctions_affected_size_is_empty. set:" << junction_current_.junction_id;
  } else if (previous_idx_in_junctions != -1 && junction_previous_.junction_state_city == JunctionStateCity::PASSED &&
             previous_idx_in_junctions + 1 < static_cast<int>(junctions_info_city_.size())) {
    junction_current_ = junctions_info_city_[previous_idx_in_junctions + 1];
  } else if (previous_idx_in_junctions == -1) {
    for (size_t i = 0; i + 1 < static_cast<size_t>(junctions_info_city_.size()); i++) {
      if (junctions_info_city_[i].offset < 0 && junctions_info_city_[i + 1].offset > 0) {
        if (std::fabs(junctions_info_city_[i + 1].offset) > std::fabs(junctions_info_city_[i].offset)) {
          junction_previous_ = junctions_info_city_[i];
          SD_JUNCTION_CONVERTE_LOG << "i:" << i;
        } else {
          junction_previous_ = junctions_info_city_[i + 1];
        }
        junction_current_ = junction_previous_;
        break;
      }
    }
    SD_JUNCTION_CONVERTE_LOG << "junction_previous_id:" << junction_previous_.junction_id;
    SD_JUNCTION_CONVERTE_LOG << "junction_type_city:" << static_cast<int>(junction_previous_.junction_type_city);
  } else {
    junction_current_ = junctions_info_city_[junctions_around_.front()];
  }

  return junctions_affected_size != 0;
}

bool SdNavigationCity::JunctionStateCalculate(BevMapInfoConstPtr &raw_bev_map, JunctionInfoCity &junction_current) {
  if (junction_current_.junction_id == 0 || junction_current_.junction_type_city == JunctionTypeCity::Unknown) {
    return false;
  }
  std::vector<Vec2d> points;
  for (const auto &junc : raw_bev_map->junctions) {
    if (junc.type == static_cast<uint32_t>(::byd::msg::env_model::LaneMarkerType::RM_TYPE_INTERSECTION_ZONE)) {
      for (const auto &point : junc.line_points) {
        points.emplace_back(point.x, point.y);
      }
      break;
    }
  }
  const auto &junction_type = junction_current_.junction_type_city;
  SD_JUNCTION_CONVERTE_LOG << fmt::format("junction_current:{}", junction_current_);
  // set prev junction passed
  // set succ junctions unreached
  const auto &ego_state       = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->navi_start;
  const auto *ego_section_ptr = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(ego_state.section_id);
  int         ego_index       = IdxInMppSection(ego_state.section_id);
  if (ego_index == -1) {
    SD_JUNCTION_CONVERTE_LOG << fmt::format("can't find the ego idx ego_section_id:{}", ego_state.section_id);
  }
  if (ego_section_ptr == nullptr || ego_section_ptr->points->size() < 2) {
    SD_JUNCTION_CONVERTE_LOG << fmt::format("can't find the ego section_id:{}", ego_state.section_id);
    return false;
  }
  const auto &ego_section = *ego_section_ptr;

  double ratio{ego_state.s_offset / ego_section.length};
  ratio = ratio > 1.0 ? 1.0 : ratio;
  Vec2d section_start{ego_section.points->front().x(), ego_section.points->front().y()};
  Vec2d section_end{ego_section.points->back().x(), ego_section.points->back().y()};
  Vec2d ego_point{0.0, 0.0};

  bool   is_ego_in_envelope  = false;
  double dis_ego_to_envelope = std::numeric_limits<double>::infinity();
  if (points.size() > 3 && junction_current.offset < 80.0) {
    if (junction_current.junction_type_city == JunctionTypeCity::CrossRoad ||
        junction_current.junction_type_city == JunctionTypeCity::TJunction ||
        junction_current.junction_type_city == JunctionTypeCity::SmallTJunction ||
        std::fabs(junction_current_.offset - dis_ego_to_envelope) < 50.0) {
      Polygon2d junction_envelope(points);
      dis_ego_to_envelope                        = junction_envelope.DistanceTo(ego_point);
      is_ego_in_envelope                         = junction_envelope.IsPointIn(ego_point);
      junction_current_.junction_envelope_points = std::move(points);
      junction_current_.distance_to_envelope     = dis_ego_to_envelope;
    }
  }

  // 2nd step: set passed,passing,unreached.
  bool   is_ego_in_start_index      = ego_state.section_id == junction_current_.junction_ids.front();
  bool   is_ego_in_end_index        = ego_state.section_id == junction_current_.junction_ids.back();
  int    junction_start_idx_in_mpp  = IdxInMppSection(junction_current_.junction_ids.front());
  int    junction_end_idx_in_mpp    = IdxInMppSection(junction_current_.junction_ids.back());
  double junction_dis_threshold_in  = junction_type_dis_in_[junction_type];
  double junction_dis_threshold_out = junction_type_dis_out_[junction_type];
  if (junction_current.diff_offset < junction_dis_threshold_out*1.5) {
    junction_dis_threshold_out = junction_current.diff_offset / 2.0;
  }
  constexpr double envelope_dis_in = 15.0;
  // constexpr double envelope_dis_out           = 5.0;
  bool is_in_envelope     = is_ego_in_envelope || dis_ego_to_envelope < envelope_dis_in;
  bool sd_offset_fullfill = false;
  bool is_in_start_sec    = false;
  bool is_in_mid_sec      = false;
  bool is_in_end_sec      = false;
  if (junction_start_idx_in_mpp != -1 && junction_end_idx_in_mpp != -1) {
    is_in_start_sec    = ego_index <= junction_start_idx_in_mpp && junction_current_.offset < junction_dis_threshold_in;
    is_in_mid_sec      = ego_index > junction_start_idx_in_mpp  && ego_index < junction_end_idx_in_mpp;
    is_in_end_sec      = ego_index == junction_end_idx_in_mpp && ego_state.s_offset <= junction_dis_threshold_out;
    sd_offset_fullfill = is_in_start_sec || is_in_mid_sec || is_in_end_sec;
  }

  SD_JUNCTION_CONVERTE_LOG << fmt::format(
      "ego_section_id:{} ego_idx:{} s_offset:{:.2f}  ids:{} is_ego_in_start_index:{:d}  is_ego_in_end_index:{:d} is_in_envelope:{:d} "
      "sd_offset_fullfill:{:d} offset:{:.2f} length:{:.2f} ratio_length:{:.2f} junction_start_idx_in_mpp:{}  junction_end_idx_in_mpp:{}  "
      "junction_type:{} is_ego_in_envelop:{:d}  dis_ego_to_envelope:{:.2f}  dis_shreshold_in:{:.2f}  dis_shreshold_out:{:.2f} "
      "ego_point:{:.2f},{:.2f} is_in_begin_mid_end:{:d} {:d} {:d}",
      ego_state.section_id, ego_index, ego_state.s_offset, junction_current_.junction_ids, is_ego_in_start_index, is_ego_in_end_index,
      is_in_envelope, sd_offset_fullfill, junction_current_.offset, ego_section.length, ratio * ego_section.length,
      junction_start_idx_in_mpp, junction_end_idx_in_mpp, junction_type, is_ego_in_envelope, dis_ego_to_envelope, junction_dis_threshold_in,
      junction_dis_threshold_out, ego_point.x(), ego_point.y(), is_in_start_sec, is_in_mid_sec, is_in_end_sec);

  auto IsBevBoundaryChange = [&]() {
    return false;
  };

  auto StayPreviousState = [&]() {
    if (junction_current_.junction_id == junction_previous_.junction_id) {
      junction_current_.junction_state_city = junction_previous_.junction_state_city;
    }
  };

  switch (junction_type) {
    case JunctionTypeCity::CrossRoad:
    case JunctionTypeCity::TJunction: {
      if (is_in_envelope || sd_offset_fullfill) {
        junction_current_.junction_state_city = JunctionStateCity::PASSING;
      } else if (junction_current_.offset > 0.0) {
        junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
      } else if ((!is_ego_in_envelope && (junction_end_idx_in_mpp != -1 && junction_end_idx_in_mpp < ego_index)) ||
                 (ego_index > junction_end_idx_in_mpp) ||
                 (ego_index == junction_end_idx_in_mpp && ego_state.s_offset > junction_dis_threshold_out)) {
        junction_current_.junction_state_city = JunctionStateCity::PASSED;
      } else {
        StayPreviousState();
      }
      break;
    }
    case JunctionTypeCity::RoadSplit: {
      if (sd_offset_fullfill) {
        junction_current_.junction_state_city = JunctionStateCity::PASSING;
      } else if (junction_current_.offset > junction_dis_threshold_in) {
        junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
      } else if (IsBevBoundaryChange() || (junction_end_idx_in_mpp != -1 && ego_index > junction_end_idx_in_mpp) ||
                 (ego_index > junction_end_idx_in_mpp) ||
                 (ego_index == junction_end_idx_in_mpp && ego_state.s_offset > junction_dis_threshold_out)) {
        junction_current_.junction_state_city = JunctionStateCity::PASSED;
      } else {
        StayPreviousState();
      }
      break;
    }
    case JunctionTypeCity::UTurn:
    case JunctionTypeCity::RoadMerge: {
      if (sd_offset_fullfill) {
        junction_current_.junction_state_city = JunctionStateCity::PASSING;
      } else if (junction_current_.offset > junction_dis_threshold_in) {
        junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
      } else if (IsBevBoundaryChange() || (junction_end_idx_in_mpp != -1 && ego_index > junction_end_idx_in_mpp) ||
                 (ego_index > junction_end_idx_in_mpp) ||
                 (ego_index == junction_end_idx_in_mpp && ego_state.s_offset > junction_dis_threshold_out)) {
        junction_current_.junction_state_city = JunctionStateCity::PASSED;
      } else {
        StayPreviousState();
      }
      break;
    }
    case JunctionTypeCity::SmallTJunction: {
      if (sd_offset_fullfill) {
        junction_current_.junction_state_city = JunctionStateCity::PASSING;
      } else if (junction_current_.offset > junction_dis_threshold_in) {
        junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
      } else if (IsBevBoundaryChange() || (junction_end_idx_in_mpp != -1 && junction_end_idx_in_mpp < ego_index) ||
                 (ego_index > junction_end_idx_in_mpp) ||
                 (ego_index == junction_end_idx_in_mpp && ego_state.s_offset > junction_dis_threshold_out)) {
        junction_current_.junction_state_city = JunctionStateCity::PASSED;
      } else {
        StayPreviousState();
      }
      break;
    }
    default: {
      ;
    }
  }
  if (junction_current_.junction_state_city == JunctionStateCity::UNKNOWN) {
    SD_JUNCTION_CONVERTE_LOG << "junction_current_state is unknown.";
    junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
  }
  return true;
}
std::vector<Eigen::Vector2f> SdNavigationCity::GetNextRightPoints(const std::shared_ptr<RoutingMap> &routing_map) {
  if (routing_map == nullptr || junctions_info_city_.empty()) {
    return {};
  }
  auto it = std::find_if(junctions_info_city_.begin(), junctions_info_city_.end(),
                         [](const JunctionInfoCity &rhs) { return rhs.junction_state_city != JunctionStateCity::PASSED; });
  if (it == junctions_info_city_.end()) {
    return {};
  }
  if (it->junction_type_city == JunctionTypeCity::RoadMerge && it != junctions_info_city_.begin()) {
    auto it_prev = std::prev(it);
    if (it_prev->junction_type_city == JunctionTypeCity::RoadSplit && it_prev->is_dedicated_right_turn_lane) {
      it = it_prev;
    }
  }
  auto             it_split         = it;
  uint64_t         split_id         = 0;
  uint64_t         merge_id         = 0;
  constexpr double max_gap_length   = 120.0;
  constexpr double unreached_length = 200.0;
  for (; it != junctions_info_city_.end(); ++it) {
    if (split_id == 0 && it->junction_type_city == JunctionTypeCity::RoadSplit && it->is_dedicated_right_turn_lane) {
      if (it->offset > unreached_length) {
        return {};
      }
      split_id = it->junction_id;
      it_split = it;
    }
    if (split_id != 0 && it->junction_type_city == JunctionTypeCity::RoadMerge && it->offset - it_split->offset < max_gap_length) {
      merge_id = it->junction_id;
      break;
    }
  }
  if (split_id == 0 || merge_id == 0) {
    return {};
  }
  const auto &sections = routing_map->sd_route.mpp_sections;
  auto        it_begin = std::find_if(sections.begin(), sections.end(), [&](const SDSectionInfo &rhs) { return rhs.id == split_id; });
  auto        it_end   = std::find_if(sections.begin(), sections.end(), [&](const SDSectionInfo &rhs) { return rhs.id == merge_id; });
  if (it_begin == sections.end() || it_end == sections.end()) {
    return {};
  }
  std::vector<Eigen::Vector2f> res;
  for (++it_begin, ++it_end; it_begin != sections.end() && it_begin != it_end; ++it_begin) {
    if (!sd_map_element_extract_.SectionIsRightTurnDedicated(*it_begin)) {
      return {};
    }
    if (it_begin->points) {
      res.insert(res.end(), it_begin->points->begin(), it_begin->points->end());
    }
  }
  SD_JUNCTION_CONVERTE_LOG << "split_id:" << split_id << "  merge_id:" << merge_id;
  return res;
}

void SdNavigationCity::JunctionsStateProc(BevMapInfoConstPtr &raw_bev_map) {
  if (!INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()) {
    SD_JUNCTION_CONVERTE_LOG << "route_ptr is nullptr.";
    return;
  }

  if (!FindCurrentJunction()) {
    SD_JUNCTION_CONVERTE_LOG << "can't find current";
  }
  if (junction_current_.junction_type_city == JunctionTypeCity::Unknown) {
    auto it = std::find_if(junctions_info_city_.begin(), junctions_info_city_.end(),
                           [](const JunctionInfoCity &city) { return city.junction_state_city == JunctionStateCity::UNREACHED; });
    if (it != junctions_info_city_.end()) {
      junction_current_ = *it;
      fmt::format_to(info_buffer_, "current_unreachd:{} ", junction_current_.junction_id);
    } else {
      auto it = std::find_if(junctions_info_city_.rbegin(), junctions_info_city_.rend(),
                             [](const JunctionInfoCity &city) { return city.junction_state_city == JunctionStateCity::PASSED; });
      if (it != junctions_info_city_.rend()) {
        junction_current_ = *it;
        SD_JUNCTION_CONVERTE_LOG << "all junction has passed. find prev pass_id:" << junction_current_.junction_id;
        fmt::format_to(info_buffer_, "prev_pass:{} ", junction_current_.junction_id);
      } else {
        SD_JUNCTION_CONVERTE_LOG << "can't find the current";
        fmt::format_to(info_buffer_, " warning!!! no current.");
        return;
      }
    }
  }

  auto FindCurrentJunction = [&](const JunctionInfoCity &junction_city_t) -> std::optional<decltype(junctions_info_city_)::iterator> {
    auto junction_it =
        std::find_if(junctions_info_city_.begin(), junctions_info_city_.end(), [&junction_city_t](const JunctionInfoCity &junction_city) {
          return junction_city.junction_id == junction_city_t.junction_id && std::abs(junction_city.offset - junction_city_t.offset) < 1.0;
        });
    if (junction_it != junctions_info_city_.end()) {
      return junction_it;
    }
    return std::nullopt;
  };
  auto GetValidNextJunction = [&](const JunctionInfoCity &junction) -> std::optional<decltype(junctions_info_city_)::iterator> {
    const auto current_pos = FindCurrentJunction(junction);
    if (!current_pos) {
      return std::nullopt;
    }

    auto it = *current_pos;
    for (++it; it != junctions_info_city_.end(); ++it) {
      if (it->is_valid) {
        return it;
      }
    }
    return std::nullopt;
  };
  if (!junction_current_.is_valid) {
    auto junction_val = GetValidNextJunction(junction_current_);
    if (!junction_val) {
      return;
    }
    junction_current_ = **junction_val;
  }

  // 2. 主逻辑
  JunctionStateCalculate(raw_bev_map, junction_current_);
  if (auto current_iter = FindCurrentJunction(junction_current_); current_iter) {
    // 检查下一个元素是否有效
    auto next_iter = GetValidNextJunction(junction_current_);
    if (junction_current_.junction_state_city == JunctionStateCity::PASSED && next_iter) {
      junction_current_ = **next_iter;
      JunctionStateCalculate(raw_bev_map, junction_current_);
      current_iter = next_iter;
    }
    if (junction_previous_.junction_id == junction_current_.junction_id &&
        junction_previous_.junction_state_city != junction_current_.junction_state_city) {
      SD_JUNCTION_CONVERTE_LOG << fmt::format("state_no_equal_id:{}  {}-{}", junction_current_.junction_id,
                                              junction_current_.junction_state_city, junction_previous_.junction_state_city);
      fmt::format_to(info_buffer_, "state_no_equal_id:{}  state:{}-{}  ", junction_current_.junction_id,
                     junction_current_.junction_state_city, junction_previous_.junction_state_city);
      switch (junction_previous_.junction_state_city) {
        case JunctionStateCity::PASSED:
          junction_current_.junction_state_city = JunctionStateCity::PASSED;
          break;
        case JunctionStateCity::PASSING:
          if (junction_current_.junction_state_city == JunctionStateCity::UNKNOWN ||
              junction_current_.junction_state_city == JunctionStateCity::UNREACHED) {
            junction_current_.junction_state_city = JunctionStateCity::PASSING;
          }
          break;
        case JunctionStateCity::UNREACHED:
          if (junction_current_.junction_state_city == JunctionStateCity::UNKNOWN) {
            junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
          }
          break;
        default:
          junction_current_.junction_state_city = JunctionStateCity::UNREACHED;
          break;
      }
    }

    // 3. 更新容器状态
    // 当前元素之前的状态设为 PASSED
    std::for_each(junctions_info_city_.begin(), *current_iter, [](auto &item) { item.junction_state_city = JunctionStateCity::PASSED; });

    // 更新当前元素
    **current_iter = junction_current_;

    // 当前元素之后的状态设为 UNREACHED
    std::for_each(std::next(*current_iter), junctions_info_city_.end(),
                  [](auto &item) { item.junction_state_city = JunctionStateCity::UNREACHED; });
  } else {
    // 未找到当前元素的处理
    std::for_each(junctions_info_city_.begin(), junctions_info_city_.end(),
                  [](auto &item) { item.junction_state_city = JunctionStateCity::UNREACHED; });
  }
}

void SdNavigationCity::SetJunctionInfo() {
  fmt::format_to(info_buffer_, "is_only_Turn:{} ", junction_current_.is_only_Turn);
  fmt::format_to(info_buffer_, "PlanTurnTypeLaneGroupID:{} ", junction_current_.map_plan_turn_type_lane_group_id);
  if (junction_current_.junction_type_city == JunctionTypeCity::Unknown) {
    fmt::format_to(info_buffer_, "junction_current is unknown.");
  } else {
    fmt::format_to(info_buffer_, "{}", junction_current_);
  }
  fmt::format_to(info_buffer_, "  all_jun_info:[");
  for (const auto &jun : junctions_info_city_) {
    fmt::format_to(info_buffer_, "{},{},{},{},{:.0f},{:d},{:d},{:d},{:d},{:d};", jun.junction_id, jun.junction_type_city,
                   jun.junction_state_city, jun.junction_action, jun.offset, jun.is_valid, jun.is_dedicated_right_turn_lane,
                   jun.need_match_arrow, jun.idx_start, jun.idx_end);
  }
  fmt::format_to(info_buffer_, "]  ");
}

std::vector<uint64_t> SdNavigationCity::PerformFineMatching(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                                            const std::vector<int> &lane_index_left,
                                                            const std::vector<int> &lane_index_right, BevMapInfoPtr &GlobalBevMapOutPut) {
  SD_FINE_MATCH_LOG << "cycle_counter: " << GlobalBevMapOutPut->header.cycle_counter;
  auto            &bev_lane_info = GlobalBevMapOutPut->lane_infos;
  std::vector<int> main_lane_reverse_indexs;
  main_lane_reverse_indexs.clear();
  int total_lane_size  = junction.map_lane_arrows_plan.size();
  int target_lane_size = junction.main_lane_indexs.size();
  // for (auto &arrow : junction.map_lane_arrows_plan) {
  //   SD_FINE_MATCH_LOG << "arrow: " << int(arrow);
  // }
  // for (auto &target : junction.main_lane_indexs) {
  //   SD_FINE_MATCH_LOG << "target index: " << int(target);
  // }
  // for (auto &road_select : road_selected) {
  //   SD_FINE_MATCH_LOG << "road_select: " << int(road_select);
  // }
  // for (auto &road_candidate : road_selected_candidate_) {
  //   SD_FINE_MATCH_LOG << "road_candidate: " << int(road_candidate);
  // }
  // for (auto &road_repeat : road_selected_repeat_) {
  //   SD_FINE_MATCH_LOG << "road_repeat: " << int(road_repeat);
  // }
  // for (auto &road_map : road_selected_map_) {
  //   SD_FINE_MATCH_LOG << "road_map: " << int(road_map.first) << " -> ";
  //   for (auto &road_repeat : road_map.second) {
  //     SD_FINE_MATCH_LOG << int(road_repeat) << " ";
  //   }
  // }

  // for (auto &lane_left : lane_index_left) {
  //   SD_FINE_MATCH_LOG << "lane_left: " << int(lane_left);
  // }

  // for (auto &lane_right : lane_index_right) {
  //   SD_FINE_MATCH_LOG << "lane_right: " << int(lane_right);
  // }

  std::vector<int>      lane_index_remain;
  std::vector<uint64_t> guide_lanes;
  if ((lane_index_left.empty()) && (lane_index_right.empty())) {
    return {};
  }

  if ((total_lane_size > 0) && (target_lane_size > 0)) {
    // main lane position
    JudgeMainLanePos(junction, total_lane_size, target_lane_size);
    SD_FINE_MATCH_LOG << "main_lane_pos_: " << int(main_lane_pos_);
    // reverse index
    for (auto &main_lane_index : junction.main_lane_indexs) {
      int main_lane_reverse_index = total_lane_size - main_lane_index - 1;
      main_lane_reverse_indexs.emplace_back(main_lane_reverse_index);
    }

    std::vector<uint64_t> road_selected_candidate_reverse(road_selected_candidate_);
    std::reverse(road_selected_candidate_reverse.begin(), road_selected_candidate_reverse.end());

    // fine matching
    SD_FINE_MATCH_LOG << "is_dedicated_right_turn_lane: " << junction.is_dedicated_right_turn_lane;
    if ((!lane_index_left.empty()) || (!lane_index_right.empty())) {
      if (((main_lane_pos_ == MainLanePos::NearLeftRight) || (main_lane_pos_ == MainLanePos::NearLeft) ||
           (junction.is_dedicated_right_turn_lane)) &&
          (!lane_index_left.empty())) {
        SD_FINE_MATCH_LOG << "From left fine match";
        lane_index_remain = lane_index_left;
        for (auto &main_index : junction.main_lane_indexs) {
          auto find_bev = std::find(lane_index_left.begin(), lane_index_left.end(), main_index);
          if (find_bev != lane_index_left.end()) {
            auto &bev_road_id = road_selected_candidate_.at(std::distance(lane_index_left.begin(), find_bev));
            lane_index_remain.at(std::distance(lane_index_left.begin(), find_bev)) = -100;
            guide_lanes.emplace_back(bev_road_id);
            auto bev_lane = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                         [bev_road_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == bev_road_id; });
            if (bev_lane != bev_lane_info.end()) {
              bev_lane->plan_turn_type = static_cast<cem::message::sensor::BevTurnType>(junction.map_lane_arrows_plan.at((main_index)));
              auto it                  = JunctionAction2Bev_.find(junction.junction_action);
              if (it != JunctionAction2Bev_.end()) {
                bev_lane->navi_action = it->second;
              } else {
                bev_lane->navi_action = cem::message::sensor::BevAction::UNKNOWN;
              }
            }
          }
        }
        if (guide_lanes.empty()) {
          for (auto &main_index : junction.main_lane_indexs) {
            auto find_bev = std::find(lane_index_left.begin(), lane_index_left.end(), main_index);
            if (find_bev == lane_index_left.end()) {
              auto find_closest = std::min_element(lane_index_remain.begin(), lane_index_remain.end(), [main_index](int a, int b) {
                return std::abs(a - main_index) < std::abs(b - main_index);
              });

              if (find_closest != lane_index_remain.end()) {
                auto &bev_road_id = road_selected_candidate_.at(std::distance(lane_index_remain.begin(), find_closest));
                guide_lanes.emplace_back(bev_road_id);
                auto bev_lane = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                             [bev_road_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == bev_road_id; });
                if (bev_lane != bev_lane_info.end()) {
                  bev_lane->plan_turn_type = static_cast<cem::message::sensor::BevTurnType>(junction.map_lane_arrows_plan.at((main_index)));
                  auto it                  = JunctionAction2Bev_.find(junction.junction_action);
                  if (it != JunctionAction2Bev_.end()) {
                    bev_lane->navi_action = it->second;
                  } else {
                    bev_lane->navi_action = cem::message::sensor::BevAction::UNKNOWN;
                  }
                }
              }
            }
          }
        }
      } else if (((main_lane_pos_ == MainLanePos::NearRight) || ((main_lane_pos_ == MainLanePos::NearLeftRight))) &&
                 (!lane_index_right.empty())) {
        SD_FINE_MATCH_LOG << "From right fine match";
        lane_index_remain = lane_index_right;
        for (auto &main_index : main_lane_reverse_indexs) {
          auto find_bev = std::find(lane_index_right.begin(), lane_index_right.end(), main_index);
          if (find_bev != lane_index_right.end()) {
            auto &bev_road_id = road_selected_candidate_reverse.at(std::distance(lane_index_right.begin(), find_bev));
            lane_index_remain.at(std::distance(lane_index_right.begin(), find_bev)) = -100;
            guide_lanes.emplace_back(bev_road_id);
            auto bev_lane = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                         [bev_road_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == bev_road_id; });
            if (bev_lane != bev_lane_info.end()) {
              bev_lane->plan_turn_type = static_cast<cem::message::sensor::BevTurnType>(junction.map_lane_arrows_plan.at(main_index));
              auto it                  = JunctionAction2Bev_.find(junction.junction_action);
              if (it != JunctionAction2Bev_.end()) {
                bev_lane->navi_action = it->second;
              } else {
                bev_lane->navi_action = cem::message::sensor::BevAction::UNKNOWN;
              }
            }
          }
        }
        if (guide_lanes.empty()) {
          for (auto &main_index : main_lane_reverse_indexs) {
            auto find_bev = std::find(lane_index_right.begin(), lane_index_right.end(), main_index);
            if (find_bev == lane_index_right.end()) {
              auto find_closest = std::min_element(lane_index_remain.begin(), lane_index_remain.end(), [main_index](int a, int b) {
                return std::abs(a - main_index) < std::abs(b - main_index);
              });

              if (find_closest != lane_index_remain.end()) {
                auto &bev_road_id = road_selected_candidate_reverse.at(std::distance(lane_index_remain.begin(), find_closest));
                guide_lanes.emplace_back(bev_road_id);
                auto bev_lane = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                             [bev_road_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == bev_road_id; });
                if (bev_lane != bev_lane_info.end()) {
                  bev_lane->plan_turn_type = static_cast<cem::message::sensor::BevTurnType>(junction.map_lane_arrows_plan.at(main_index));
                  auto it                  = JunctionAction2Bev_.find(junction.junction_action);
                  if (it != JunctionAction2Bev_.end()) {
                    bev_lane->navi_action = it->second;
                  } else {
                    bev_lane->navi_action = cem::message::sensor::BevAction::UNKNOWN;
                  }
                }
              }
            }
          }
        }
      } else {  // Unknow
        guide_lanes = road_selected;
      }
    }
  }

  // repeated lane process
  bool exit_once = false;
  for (const auto &pair : road_selected_map_) {
    exit_once = false;
    auto it   = std::find(guide_lanes.begin(), guide_lanes.end(), pair.first);
    if (it == guide_lanes.end()) {
      exit_once = true;
    }
    for (const auto &value_id : pair.second) {
      uint64_t key_id         = pair.first;
      auto     bev_lane_key   = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                             [key_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == key_id; });
      auto     bev_lane_value = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                             [value_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == value_id; });
      if (bev_lane_value != bev_lane_info.end()) {
        bev_lane_value->plan_turn_type = bev_lane_key->plan_turn_type;
        bev_lane_value->navi_action    = bev_lane_key->navi_action;
        if (!exit_once) {
          guide_lanes.emplace_back(value_id);
        }
      }
    }
  }

  // remain not main lane arrow in road_selected
  if (total_lane_size > target_lane_size) {
    for (size_t i = 0; i < junction.map_lane_arrows_plan.size(); i++) {
      auto arrow_find = std::find(junction.main_lane_indexs.begin(), junction.main_lane_indexs.end(), i);
      if (arrow_find != junction.main_lane_indexs.end()) {
        continue;
      }
      if (i >= road_selected.size()) {
        continue;
      }
      size_t bev_other_index = road_selected.at(i);
      auto   bev_lane_other = std::find_if(bev_lane_info.begin(), bev_lane_info.end(), [bev_other_index](const BevLaneInfo &bev_lane_info) {
        return bev_lane_info.id == bev_other_index;
      });
      if (bev_lane_other != bev_lane_info.end()) {
        bev_lane_other->plan_turn_type = static_cast<cem::message::sensor::BevTurnType>(junction.map_lane_arrows_plan.at(i));
        auto it                        = JunctionAction2Bev_.find(junction.junction_action);
        if (it != JunctionAction2Bev_.end()) {
          bev_lane_other->navi_action = it->second;
        } else {
          bev_lane_other->navi_action = cem::message::sensor::BevAction::UNKNOWN;
        }
      }
    }
  }

  // junction has no arrow / has no main lane
  if (((total_lane_size == 0) || (target_lane_size == 0)) && (road_selected.size() != 0)) {
    guide_lanes.clear();
    for (size_t i = 0; i < road_selected.size(); i++) {
      if (junction.junction_action == JunctionAction::TurnLeft) {
        if (i == 0) {
          size_t left_id       = road_selected.at(0);
          auto   bev_lane_left = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                              [left_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == left_id; });
          if (bev_lane_left != bev_lane_info.end()) {
            bev_lane_left->plan_turn_type = cem::message::sensor::BevTurnType::LEFT_TURN;
            auto action_left              = JunctionAction2Bev_.find(junction.junction_action);
            if (action_left != JunctionAction2Bev_.end()) {
              bev_lane_left->navi_action = action_left->second;
            } else {
              bev_lane_left->navi_action = cem::message::sensor::BevAction::UNKNOWN;
            }
            bev_lane_left->is_default_arrow = true;
            guide_lanes.emplace_back(left_id);
          }
        }
      } else if (junction.junction_action == JunctionAction::TurnRight) {
        if (i == road_selected.size() - 1) {
          size_t right_id       = road_selected.at(road_selected.size() - 1);
          auto   bev_lane_right = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                               [right_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == right_id; });
          if (bev_lane_right != bev_lane_info.end()) {
            bev_lane_right->plan_turn_type = cem::message::sensor::BevTurnType::RIGHT_TURN;
            auto action_right              = JunctionAction2Bev_.find(junction.junction_action);
            if (action_right != JunctionAction2Bev_.end()) {
              bev_lane_right->navi_action = action_right->second;
            } else {
              bev_lane_right->navi_action = cem::message::sensor::BevAction::UNKNOWN;
            }
            bev_lane_right->is_default_arrow = true;
            guide_lanes.emplace_back(right_id);
          }
        }
      } else {
        if (i == 0) {
          guide_lanes = road_selected;
        }
      }
      size_t other_id       = road_selected.at(i);
      auto   bev_lane_other = std::find_if(bev_lane_info.begin(), bev_lane_info.end(),
                                           [other_id](const BevLaneInfo &bev_lane_info) { return bev_lane_info.id == other_id; });
      if (bev_lane_other != bev_lane_info.end()) {
        bev_lane_other->plan_turn_type = cem::message::sensor::BevTurnType::NO_TURN;
        auto action_other              = JunctionAction2Bev_.find(junction.junction_action);
        if (action_other != JunctionAction2Bev_.end()) {
          bev_lane_other->navi_action = action_other->second;
        } else {
          bev_lane_other->navi_action = cem::message::sensor::BevAction::UNKNOWN;
        }
        bev_lane_other->is_default_arrow = true;
      }
    }
    return guide_lanes;
  }

  for (auto &bev_lane : bev_lane_info) {
    SD_FINE_MATCH_LOG << "bev_lane.id: " << bev_lane.id;
    SD_FINE_MATCH_LOG << "bev_lane.plan_turn_type: " << int(bev_lane.plan_turn_type);
    SD_FINE_MATCH_LOG << "bev_lane.navi_action: " << int(bev_lane.navi_action);
  }

  std::unordered_set<uint64_t> is_existing;
  std::vector<uint64_t>        guide_lanes_tmp;
  for (int guide_lane : guide_lanes) {
    if (is_existing.find(guide_lane) == is_existing.end()) {
      guide_lanes_tmp.emplace_back(guide_lane);
      is_existing.insert(guide_lane);
    }
  }

  guide_lanes = guide_lanes_tmp;
  for (auto &guide_lane : guide_lanes) {
    SD_FINE_MATCH_LOG << "fine match guide_lane: " << guide_lane;
  }
  return guide_lanes;
}

void SdNavigationCity::JudgeMainLanePos(const JunctionInfoCity &junction, int &size_sum_lane, int &size_target_lane) {
  const std::vector<int> &target_lane = junction.main_lane_indexs;
  if (size_target_lane > size_sum_lane) {
    main_lane_pos_ = MainLanePos::Unknow;
    return;
  }

  if (size_target_lane == size_sum_lane) {
    main_lane_pos_ = MainLanePos::NearLeftRight;
    return;
  }

  bool             isContinuous       = true;
  std::vector<int> sorted_target_lane = target_lane;
  std::sort(sorted_target_lane.begin(), sorted_target_lane.end());

  for (size_t i = 1; i < sorted_target_lane.size(); ++i) {
    if (sorted_target_lane.at(i) - sorted_target_lane.at(i - 1) != 1) {
      isContinuous = false;
      break;
    }
  }

  if (!isContinuous) {
    main_lane_pos_ = MainLanePos::Unknow;
    return;
  }

  bool isOdd = true;
  if (size_sum_lane % 2 == 0)
    isOdd = false;

  int halfN       = size_sum_lane / 2;
  int left_count  = 0;
  int right_count = 0;

  if (isOdd) {
    for (auto &target_index : target_lane) {
      if (target_index < halfN) {
        left_count++;
      } else if (target_index > halfN) {
        right_count++;
      }
    }
  } else {
    for (auto &target_index : target_lane) {
      if (target_index < halfN) {
        left_count++;
      } else {
        right_count++;
      }
    }
  }

  if (left_count > right_count) {
    main_lane_pos_ = MainLanePos::NearLeft;
  } else if (left_count < right_count) {
    main_lane_pos_ = MainLanePos::NearRight;
  } else {
    main_lane_pos_ = MainLanePos::NearLeftRight;
  }

  // 特殊场景，最左侧车道非导航车道，强制靠左匹配
  if (target_lane.at(0) != 0) {
    main_lane_pos_ = MainLanePos::NearLeft;
  }
}

///判断是否是导航作用路口
bool SdNavigationCity::IsEffectedJunction(const JunctionInfoCity &junction_info) {

  if (IsInterestedJunction(junction_info)) {
    return true;
  }
  if (junction_info.junction_type_city == JunctionTypeCity::RoadSplit) {
    return true;
  }
  return false;
}

//RoadSplit场景中 判断是否是主路到辅路场景
bool SdNavigationCity::IsRoadSplitMain2Ramp(const JunctionInfoCity &junction) {

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

///判断是否是关注路口
bool SdNavigationCity::IsInterestedJunction(const JunctionInfoCity &junction_info) {

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

void SdNavigationCity::SetNaviInterfaceAndDebugInfos(BevMapInfoPtr &GlobalBevMapOutPut) {

  ///导航接口赋值
  auto navigation_info_interface = std::make_shared<std::vector<JunctionInfoCity>>();
  *navigation_info_interface     = this->junctions_info_city_;
  INTERNAL_PARAMS.navigation_info_data.SetJunctionInfoCityPtr(navigation_info_interface);

  json debug_json;
  auto navi_debug_info = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  // 获取枚举的字符串表示
  debug_json["ause_ld"]              = (int)this->cut_map_zone_.active;
  debug_json["acity"]                = "888";
  debug_json["alanes"]               = navi_debug_info.guide_laneids;
  debug_json["arefer"]               = navi_debug_info.guide_laneids_refer;
  debug_json["avirtual"]             = navi_debug_info.guide_lanes_length;
  debug_json["j_id"]                 = navi_debug_info.target_junction_ids;
  debug_json["j_type"]               = navi_debug_info.junction_type;
  debug_json["j_offset"]             = navi_debug_info.junction_offset;
  debug_json["split_merge"]          = navi_debug_info.split_merge;
  debug_json["action"]               = navi_debug_info.junction_action;
  debug_json["am_type"]              = navi_debug_info.match_type;
  debug_json["bev_counter"]          = navi_debug_info.bev_counter;
  debug_json["bev_section"]          = navi_debug_info.all_sections_in;
  debug_json["bev_separators"]       = navi_debug_info.all_separators;
  debug_json["btarget_section"]      = navi_debug_info.target_section;
  debug_json["bhole_target_section"] = navi_debug_info.hold_target_section;
  debug_json["road_j_id"]            = navi_debug_info.select_road_junction;
  debug_json["filter_section"]       = navi_debug_info.target_section_filter;
  debug_json["lane_type"]            = navi_debug_info.lane_type;
  debug_json["section_nobus"]        = navi_debug_info.sections_without_bus;
  debug_json["arrow"]                = navi_debug_info.sd_arrows;
  debug_json["index"]                = navi_debug_info.target_lanes_index;

  json CoarseMatchingType2;
  CoarseMatchingType2["ReachableLaneCountToRamp"] = navi_debug_info.ReachableLaneCountToRamp;
  CoarseMatchingType2["ExclusiveLaneCountToRamp"] = navi_debug_info.ExclusiveLaneCountToRamp;
  CoarseMatchingType2["bevLaneNum"]               = navi_debug_info.bevLaneNum;
  CoarseMatchingType2["mapLaneNum"]               = navi_debug_info.mapLaneNum;
  CoarseMatchingType2["lanegroup_failed"]         = navi_debug_info.lanegroup_failed;
  CoarseMatchingType2["isSplitRoadSelectWorked"]  = navi_debug_info.isSplitRoadSelectWorked;
  debug_json["coarseMatchingType2"]               = CoarseMatchingType2;

  /* 粗匹配地图推荐车道的debug信息 */
  json vs_array = json::array();
  for (const auto &vs : navi_debug_info.virtual_sections) {
    json vs_obj;
    vs_obj["id"]                  = vs.id;
    vs_obj["start_s"]             = std::round(vs.start_s * 100) / 100;
    vs_obj["end_s"]               = std::round(vs.end_s * 100) / 100;
    vs_obj["lane_num"]            = vs.lane_num;
    vs_obj["target_lane_indices"] = vs.target_lane_indices;
    vs_array.push_back(vs_obj);
  }
  debug_json["virtual_sections"] = vs_array;

  /* 地图推荐车道的debug信息 */
  json sd_remm_array = json::array();
  for (const auto &sd_remm : navi_debug_info.sd_recommend_lane) {
    json sd_remm_obj;
    sd_remm_obj["start_ego"]        = std::round(sd_remm.first * 100) / 100;
    sd_remm_obj["lane_num"]         = sd_remm.second.lane_num;
    sd_remm_obj["target_lane_seqs"] = sd_remm.second.lane_seqs;
    sd_remm_array.push_back(sd_remm_obj);
  }
  debug_json["sd_recommend_lane"] = sd_remm_array;

  // 新增其他字段
  debug_json["selected_vs_idx"]     = navi_debug_info.selected_vs_idx;
  debug_json["guide_lanes_topo"]    = navi_debug_info.guide_lanes_topo;
  debug_json["guide_lanes_mapless"] = navi_debug_info.guide_lanes_mapless;
  debug_json["lane_backward"]       = navi_debug_info.lane_backward;
  std::string debug_str             = debug_json.dump();
  SD_BEV_PROCESS << "===json:" << debug_str;
  SD_COARSE_MATCH_TYPE2_LOG << "===json:" << debug_str;
  SD_ENV_INFO_LOG << "===json:" << debug_str;
  SD_MERGE_LOG << "===json:" << debug_str;
  SD_FINE_MATCH_LOG << "===json:" << debug_str;
  GlobalBevMapOutPut->debug_infos = debug_str;
}

//判断是否是右转专用道路口
void SdNavigationCity::DetectAdjacentRightTurnJunctions(const std::vector<JunctionInfoCity> &junctions_info_city,
                                                        std::vector<AdjacentJunctionInfo>   &adjacent_junctions_info) {

  // 清空之前存储的相邻路口信息
  adjacent_junctions_info.clear();

  // 检测十字路口/T型路口与右转专用道相邻的情况
  for (size_t i = 0; i < junctions_info_city.size(); ++i) {
    const auto &current_junction = junctions_info_city[i];

    // 检查是否为未到达的十字路口或T型路口
    if ((current_junction.junction_type_city == JunctionTypeCity::CrossRoad ||
         current_junction.junction_type_city == JunctionTypeCity::TJunction ||
         current_junction.junction_type_city == JunctionTypeCity::SmallTJunction) &&
        current_junction.junction_state_city == JunctionStateCity::UNREACHED) {

      // 向前查找前一个RoadSplit类型的路口
      for (int j = i - 1; j >= 0; --j) {
        const auto &previous_junction = junctions_info_city[j];

        // 检查是否为RoadSplit类型且是右转专用道
        if (previous_junction.junction_type_city == JunctionTypeCity::RoadSplit && previous_junction.is_dedicated_right_turn_lane) {

          // 计算两个路口之间的距离
          double distance = current_junction.offset - previous_junction.offset;

          // 如果距离小于60米，则记录该路口
          if (distance > 0 && distance < 60.0) {
            AdjacentJunctionInfo info;
            info.junction_id                    = current_junction.junction_id;
            info.offset                         = current_junction.offset;
            info.junction_type                  = current_junction.junction_type_city;
            info.is_adjacent_to_dedicated_right = true;

            adjacent_junctions_info.push_back(info);
            SD_COARSE_MATCH_LOG << "[DetectAdjacentRightTurnJunctions] Found adjacent junction: ID=" << info.junction_id
                                << ", Type=" << StrJunctionTypeCity(info.junction_type) << ", Offset=" << info.offset;
          }
          break;  // 找到第一个符合条件的RoadSplit路口
        }
      }
    }
  }
}

/*
  * @brief 根据
  * @param harbor  港湾车道信息
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
void SdNavigationCity::Erase_Oppolane_Ids(const HarborStopInfo &sd_harbor_stop_info_, std::vector<uint64_t> &road_selected_buslane_filtered,
                                          const std::vector<BevLaneMarker> &all_bev_edges) {

  /*遍历 road_selected_buslane_filtered, 取出其中的每个id*/
  std::vector<uint64_t> indices_id_to_remove = {};
  // bool                  is_remove            = false;

  for (auto &id : road_selected_buslane_filtered) {
    auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
    if ((single_lane) && (!single_lane->geos->empty())) {
      SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << single_lane->geos->front().x()
                          << " lane_start_y: " << single_lane->geos->front().y() << " lane_end_x: " << single_lane->geos->back().x()
                          << " lane_end_y: " << single_lane->geos->back().y();
    }
  }

  /*利用每条线到左边界的距离进行判断*/
  //Edge_log_print(all_bev_edges);
  double min_overlap_threshold  = 15.0f;  // 最小长度阈值(米) 20->15
  double max_distance_threshold = 3.0f;   // 最大距离阈值(米)  后续改为3.5
  if (indices_id_to_remove.empty()) {
    // 步骤1: 找出最长且大于阈值的边界线
    BevLaneMarker         best_edge;
    double                max_length = -1.0;
    const Eigen::Vector2f ego_pt     = {0, 0};
    for (auto &edge : all_bev_edges) {
      bool is_left_of_line = IsPointLeftOfPolyline(ego_pt, *edge.geos);
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && !is_left_of_line)) {
        double edge_length = edge.geos->back().x() - edge.geos->front().x();
        SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids] bdry id: " << edge.id << " edge_length:" << edge_length;
        if (edge_length > max_length && edge_length >= min_overlap_threshold &&
            edge.type != static_cast<uint32_t>(BoundaryType::FENCE_BOUNDARY)) {
          max_length = edge_length;
          best_edge  = edge;
        }
      }
    }
    // 步骤2: 找出与该边界线宽度最小且小于阈值的车道线
    uint64_t best_lane_id = 0;
    float    min_distance = std::numeric_limits<float>::max();
    for (auto &id : road_selected_buslane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);

      if ((id == road_selected_buslane_filtered.front()) && (single_lane) && (!single_lane->geos->empty()) && (!best_edge.geos->empty())) {
        // double dis_to_rightbdry = cem::fusion::LaneGeometry::GetDistanceBetweenLines(*single_lane->geos, *best_edge.geos);
        /* 计算车道垂线宽度均值 */
        //double dis_to_rightbdry = CalculateCurvedLaneWidth(*single_lane->geos, *best_edge.geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*single_lane->geos, *best_edge.geos);
        /* 计算投影距离的平均宽度 */
        // double dis_to_rightbdry = CalculateLaneWidth(*single_lane->geos, *best_edge.geos);
        SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids] dis line " << single_lane->id << " to " << best_edge.id
                            << "  leftbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry > 0.5 && dis_to_rightbdry < min_distance && dis_to_rightbdry <= max_distance_threshold) {
          min_distance = dis_to_rightbdry;
          best_lane_id = id;
        }
      }
    }
    SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids]best_edge id: " << best_edge.id << "  max_length: " << max_length;
    SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids]best_lane_id: " << best_lane_id << "  min_distance: " << min_distance;
    if (best_lane_id != 0) {
      indices_id_to_remove.push_back(best_lane_id);
    }
  }

  // 使用remove-erase移除指定ID的元素
  if (!indices_id_to_remove.empty() && road_selected_buslane_filtered.size() >= 2) {
    for (int id : indices_id_to_remove) {
      road_selected_buslane_filtered.erase(std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
                                           road_selected_buslane_filtered.end());
    }
    pre_oppo_lanes_to_move_.oppo_lane_ids = indices_id_to_remove;
    pre_oppo_lanes_to_move_.keep_count    = 0;
  } else if (indices_id_to_remove.empty() && !pre_oppo_lanes_to_move_.oppo_lane_ids.empty()) /* precious is not empty */
  {
    for (int id : pre_oppo_lanes_to_move_.oppo_lane_ids) {
      road_selected_buslane_filtered.erase(std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
                                           road_selected_buslane_filtered.end());
    }
    pre_oppo_lanes_to_move_.keep_count++;
  } else {
    /* null */
  }

  if (pre_oppo_lanes_to_move_.keep_count > NavigationConfig::oppo_lane_keep_cnt_threshold) {
    pre_oppo_lanes_to_move_.keep_count    = 0;
    pre_oppo_lanes_to_move_.oppo_lane_ids = {};
  }

  SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
  SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids]pre_oppo_lanes_to_move_: "
                      << fmt::format(" [{}]  ", fmt::join(pre_oppo_lanes_to_move_.oppo_lane_ids, ", "))
                      << " keep_count: " << pre_oppo_lanes_to_move_.keep_count;
  SD_COARSE_MATCH_LOG << "[Erase_Oppolane_Ids]road_selected_buslane_filtered: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_buslane_filtered, ", "))
                      << " size: " << road_selected_buslane_filtered.size();
}

/*
   * @brief 过滤掉对向车道
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉对向车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationCity::FilterOppositeLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                           std::vector<uint64_t>             &filtered_lanes_without_harbor,
                                                           BevMapInfoConstPtr                &raw_bev_map) {
  SD_COARSE_MATCH_LOG << "Entering [FilterOppositeLane]...";
  // 判断road_selected是否为空，空则返回
  if (filtered_lanes_without_harbor.empty()) {
    return {};
  }
  std::vector<uint64_t> filtered_lanes     = filtered_lanes_without_harbor;
  auto                 &all_bev_road_edges = raw_bev_map->edges;

  bool filter_flag = DetectBidirSingleLane();
  SD_COARSE_MATCH_LOG << " [FilterOppositeLane] filter_flag: " << filter_flag;

  if (filter_flag && filtered_lanes_without_harbor.size() >= 2) {
    //filtered_lanes_without_harbor.erase(filtered_lanes_without_harbor.begin());
    Erase_Oppolane_Ids(sd_harbor_stop_info_, filtered_lanes, all_bev_road_edges);
    SD_COARSE_MATCH_LOG << " [FilterOppositeLane] erase the front lane...";
  }

  return filtered_lanes;
}
/*
  * @brief 判断当前是否是双向单车道
  * @param raw_routing_map  SDMap的信息
  * @return 
*/
bool SdNavigationCity::DetectBidirSingleLane() {

  SD_COARSE_MATCH_LOG << "Entering DetectBidirSingleLane";
  int                   current_lane_count = coarse_matching_.GetCurrentLaneNum();
  auto                  sd_route           = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  std::vector<LaneType> lane_type_list     = GetCurLaneTypeList();
  int                   normal_lane_count  = std::count(lane_type_list.begin(), lane_type_list.end(), LaneType::LANE_NORMAL);
  if (!sd_route) {
    AWARN << "[DetectBidirSingleLane] sd_route is null";
    SD_COARSE_MATCH_LOG << "sd_route is null";
    return false;
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "[DetectBidirSingleLane] mpp_sections is empty";
    SD_COARSE_MATCH_LOG << "mpp_sections is empty";
    return false;
  }

  uint64_t current_section_id = sd_route->navi_start.section_id;
  SD_COARSE_MATCH_LOG << "current_section_id: " << current_section_id;

  const SDSectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[DetectBidirSingleLane] current section not found";
    SD_COARSE_MATCH_LOG << "current section not found";
    return false;
  }

  /* 获取对向section id */
  uint64_t opposite_section_id = current_section->id % 10 ? (current_section->id - 1) : (current_section->id + 1);
  auto    *opposite_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(opposite_section_id);
  if (!opposite_section) {
    // AWARN << "[DetectBidirSingleLane] opposite_section  not found";
    // SD_COARSE_MATCH_LOG << "opposite_section  not found";
    return false;
  }

  SD_COARSE_MATCH_LOG << "[DetectBidirSingleLane] direction： " << StrSDDirectionType(current_section->direction)
                      << " current_lane_count: " << current_lane_count << " opposite_lane_count: " << opposite_section->lane_num;

  bool b_within_harbor_range = false;

  if (sd_harbor_stop_info_.exists) {
    // 获取第一个港湾车道范围 [start_s, end_s]
    const auto &firstRange  = sd_harbor_stop_info_.ranges[0];
    double      harborStart = firstRange.first;
    double      harborEnd   = firstRange.second;
    if (harborStart < 0 && harborEnd > 0) {
      b_within_harbor_range = true;
    }
  }

  if (current_section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE && opposite_section->lane_num == 1) {
    if (current_lane_count == 1) {
      return true;
    } else if (current_lane_count == 2 && b_within_harbor_range) {
      return true;
    } else if (current_lane_count == 2 && normal_lane_count == 1) {
      return true;
    }
  } else {
  }

  return false;
}

// 判断点在单条线段的左侧（返回true）或右侧（返回false）
bool SdNavigationCity::IsPointLeftOfSegment(const Eigen::Vector2f &pt, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
  // 向量 P1->P2 和 P1->pt 的叉积
  float cross = (p2.x() - p1.x()) * (pt.y() - p1.y()) - (p2.y() - p1.y()) * (pt.x() - p1.x());
  return cross > 0;  // 正数：左侧；负数：右侧
}

// 判断点在折线的左侧（返回true）或右侧（返回false）
bool SdNavigationCity::IsPointLeftOfPolyline(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &polyline) {
  if (polyline.size() < 2)
    return false;  // 折线至少需要2个点
  int left_cnt = 0, right_cnt = 0;
  for (size_t i = 0; i < polyline.size() - 1; ++i) {
    if (polyline[i].x() < 0.0) {
      continue;
    }

    if (IsPointLeftOfSegment(pt, polyline[i], polyline[i + 1])) {
      left_cnt++;
    } else {
      right_cnt++;
    }
  }
  SD_COARSE_MATCH_LOG << "[IsPointLeftOfPolyline] left_cnt: " << left_cnt << " right_cnt: " << right_cnt;
  return left_cnt > right_cnt;  // 以多数线段的判断结果为准
}

// 计算带符号的距离（左侧为正，右侧为负）
double SdNavigationCity::GetSignedDistanceToLine(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line, bool need_extent) {
  // 1. 计算距离大小（非负）
  double distance = cem::fusion::LaneGeometry::GetDistToLine(pt, line, false);
  // 2. 判断方向（左侧/右侧）
  bool is_left = IsPointLeftOfPolyline(pt, line);
  // 3. 带符号距离（左侧正，右侧负）
  return is_left ? distance : -distance;
}

// 计算两点连线的单位方向向量（切线方向）
Eigen::Vector2f SdNavigationCity::GetTangentDirection(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
  Eigen::Vector2f dir = p2 - p1;
  if (dir.norm() < 1e-6) {
    return Eigen::Vector2f(0, 0);  // 无效方向
  }
  return dir.normalized();
}

// 计算垂直于切线方向的单位向量（宽度方向）
Eigen::Vector2f SdNavigationCity::GetNormalDirection(const Eigen::Vector2f &tangent) {
  return Eigen::Vector2f(-tangent.y(), tangent.x());  // 顺时针旋转90度（垂直方向）
}

double SdNavigationCity::CalculateCurvedLaneWidth(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2) {
  if (lane1.size() < 2 || lane2.size() < 2) {
    return 0.0;
  }

  std::vector<double> widths;
  // 沿车道线均匀采样（取较短线段的点，避免越界）
  size_t sample_count = std::min(lane1.size(), lane2.size()) - 1;
  for (size_t i = 0; i < sample_count; ++i) {
    // 取lane1的切线方向
    Eigen::Vector2f tangent = GetTangentDirection(lane1[i], lane1[i + 1]);
    if (tangent.norm() < 1e-6) {
      continue;
    }
    Eigen::Vector2f normal = GetNormalDirection(tangent);  // 垂直方向

    // 计算lane1[i]在lane2上的投影（沿垂直方向）
    Eigen::Vector2f proj_pt = cem::fusion::LaneGeometry::PointProjectToLineSegment(lane2.front(), lane2.back(), lane1[i]);
    // 计算垂直距离（沿normal方向的距离）
    double dist = (proj_pt - lane1[i]).dot(normal);  // 点积得到垂直分量
    widths.push_back(std::fabs(dist));
  }

  // 取平均宽度（过滤无效值）
  if (widths.empty()) {
    return 0.0;
  }
  double avg_width = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();
  return avg_width;
}
//通过投影计算车道平均宽度
double SdNavigationCity::CalculateLaneWidth(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2) {
  if (lane1.size() < 2 || lane2.size() < 2) {
    return 0.0;  // 无效车道线
  }
  // 使用投影距离的平均值（更接近垂直方向）
  double avg_dist = cem::fusion::LaneGeometry::GetProjDistanceBetweenLinesAverage(lane1, lane2);
  // 过滤异常值（例如距离过小或过大）
  if (avg_dist < 0 || avg_dist > 99.0) {  // 可根据实际场景调整
    return 0.0;
  }
  return avg_dist;
}

double SdNavigationCity::CalculateCurvedLaneWidth2(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2) {
  if (lane1.size() < 2 || lane2.size() < 2) {
    return 0.0;
  }

  // 1. 计算两条车道线的重叠区域
  float min_x = std::max(lane1.front().x(), lane2.front().x());
  float max_x = std::min(lane1.back().x(), lane2.back().x());

  // 如果没有重叠区域，返回0
  if (min_x >= max_x) {
    return 0.0;
  }

  std::vector<double> valid_widths;
  double              sample_step = 0.2;  // 采样步长，单位为米

  // 2. 在重叠区域内均匀采样
  for (double x = min_x; x <= max_x; x += sample_step) {
    // 找到lane1上最接近x的点
    auto it1 = std::lower_bound(lane1.begin(), lane1.end(), x, [](const Eigen::Vector2f &p, double val) { return p.x() < val; });

    if (it1 == lane1.end() || it1 == lane1.begin())
      continue;
    const Eigen::Vector2f &p1 = *(it1 - 1);
    const Eigen::Vector2f &p2 = *it1;

    // 线性插值计算x处的精确位置和切线方向
    double          t              = (x - p1.x()) / (p2.x() - p1.x());
    Eigen::Vector2f point_on_lane1 = p1 + t * (p2 - p1);
    Eigen::Vector2f tangent        = (p2 - p1).normalized();
    Eigen::Vector2f normal(-tangent.y(), tangent.x());  // 垂直方向

    // 找到lane2上最接近x的点
    auto it2 = std::lower_bound(lane2.begin(), lane2.end(), x, [](const Eigen::Vector2f &p, double val) { return p.x() < val; });

    if (it2 == lane2.end() || it2 == lane2.begin())
      continue;
    const Eigen::Vector2f &q1 = *(it2 - 1);
    const Eigen::Vector2f &q2 = *it2;

    // 线性插值计算x处的精确位置
    double          s              = (x - q1.x()) / (q2.x() - q1.x());
    Eigen::Vector2f point_on_lane2 = q1 + s * (q2 - q1);

    // 计算两点之间的垂直距离
    double width = (point_on_lane2 - point_on_lane1).dot(normal);
    valid_widths.push_back(std::fabs(width));
  }

  // 3. 计算有效宽度的平均值
  if (valid_widths.empty()) {
    return 0.0;
  }

  // 过滤异常值（可选）
  std::sort(valid_widths.begin(), valid_widths.end());
  size_t start_idx = valid_widths.size() * 0.1;  // 忽略最小的10%
  size_t end_idx   = valid_widths.size() * 0.9;  // 忽略最大的10%

  double sum = 0.0;
  for (size_t i = start_idx; i < end_idx; ++i) {
    sum += valid_widths[i];
  }

  return sum / (end_idx - start_idx);
}

bool SdNavigationCity::IsContainsLastElement(const std::vector<uint64_t> &road_selected_buslane_filtered,
                                             const std::vector<uint64_t> &indices_id_to_remove) {

  if (indices_id_to_remove.empty()) {
    return false;
  }

  if (road_selected_buslane_filtered.empty()) {
    return false;
  }

  uint64_t last_element = road_selected_buslane_filtered.back();

  auto it = std::find(indices_id_to_remove.begin(), indices_id_to_remove.end(), last_element);

  return (it != indices_id_to_remove.end());
}

void SdNavigationCity::UpdateHarborLaneType(const std::vector<uint64_t> &road_selected,
                                            std::vector<uint64_t> &filtered_lanes_without_harbor, BevMapInfoPtr &GlobalBevMapOutPut) {
  // 遍历road_selected，寻找不在filtered_lanes中的id
  uint64_t target_harbor_id = 0;
  for (uint64_t id : road_selected) {
    // 检查当前id是否存在于filtered_lanes_without_harbor中
    auto it = std::find(filtered_lanes_without_harbor.begin(), filtered_lanes_without_harbor.end(), id);
    if (it == filtered_lanes_without_harbor.end()) {
      target_harbor_id = id;
      break;
    }
  }

  if (target_harbor_id == 0)
    return;

  // 更新对应车道的类型
  for (auto &lane : GlobalBevMapOutPut->lane_infos) {
    if (lane.id == target_harbor_id) {
      lane.lane_type = BevLaneType::LANE_TYPE_HARBOR_STOP;
      SD_COARSE_MATCH_LOG << "  [UpdateHarborLaneType] Setting rightmost BEV lane ID: " << lane.id << " to LANE_TYPE_HARBOR_STOP";
      break;
    }
  }
}

void SdNavigationCity::UpdateOppoLaneDir(const std::vector<uint64_t> &road_selected, std::vector<uint64_t> &filtered_lanes_without_harbor,
                                         BevMapInfoPtr &GlobalBevMapOutPut) {
  SD_COARSE_MATCH_LOG << "  [UpdateOppoLaneDir] input: before filter " << fmt::format(" [{}]  ", fmt::join(road_selected, ", "))
                      << "input: after filter " << fmt::format(" [{}]  ", fmt::join(filtered_lanes_without_harbor, ", "));
  // 遍历road_selected，寻找不在filtered_lanes中的id
  uint64_t target_harbor_id = 0;
  for (uint64_t id : road_selected) {
    // 检查当前id是否存在于filtered_lanes_without_harbor中
    auto it = std::find(filtered_lanes_without_harbor.begin(), filtered_lanes_without_harbor.end(), id);
    if (it == filtered_lanes_without_harbor.end()) {
      target_harbor_id = id;
      break;
    }
  }

  if (target_harbor_id == 0)
    return;
  SD_COARSE_MATCH_LOG << "  [UpdateOppoLaneDir] target_harbor_id: " << target_harbor_id;
  // 更新对应车道的类型
  for (auto &lane : GlobalBevMapOutPut->lane_infos) {
    if (lane.id == target_harbor_id) {
      lane.direction = BevLaneDirection::DIRECTION_BACKWARD;
      SD_COARSE_MATCH_LOG << "  [UpdateOppoLaneDir] Setting oppo BEV lane ID: " << lane.id << " to DIRECTION_BACKWARD";
      break;
    }
  }
  /*全局debug*/
  auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_infos.lane_backward.push_back(target_harbor_id);
}

void SdNavigationCity::MatchBusLaneType(const BusLaneInfo &bus_lane_info_, std::vector<uint64_t> &road_selected_buslane_filtered,
                                        const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                                        BevMapInfoPtr &GlobalBevMapOutPut) {
  SD_COARSE_MATCH_LOG << "entering MatchBusLaneType with " << fmt::format(" [{}]  ", fmt::join(road_selected_buslane_filtered, ", "));
  std::pair<double, double> firstRange = {0, 0};
  // 获取第一个港湾车道范围 [start_s, end_s]
  for (auto range : bus_lane_info_.ranges) {
    if (range.second > 0) {
      firstRange = range;
      break;
    }
  }
  double rangeStart = firstRange.first;
  double rangeEnd   = firstRange.second;

  /*遍历 road_selected_buslane_filtered, 取出其中的每个id*/
  std::vector<uint64_t> indices_id_to_remove = {};
  std::vector<uint64_t> overlap_lanes        = {};
  // bool                  is_remove            = false;
  size_t lane_count = 0;
  for (auto &id : road_selected_buslane_filtered) {
    auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
    if ((single_lane) && (!single_lane->geos->empty())) {
      auto full_geometry = bev_data_processor_.GetFullLaneGeometry(single_lane);
      SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << full_geometry->front().x()
                          << " lane_start_y: " << full_geometry->front().y() << " lane_end_x: " << full_geometry->back().x()
                          << " lane_end_y: " << full_geometry->back().y();

      double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *full_geometry);
      SD_COARSE_MATCH_LOG << "overlap_dis:" << overlap_dis;
      if ((overlap_dis > 20.0)) {
        lane_count++;
        overlap_lanes.push_back(id);
      }
    }
  }

  /*利用每条线到边界的距离进行判断*/
  //Edge_log_print(all_bev_edges);
  double min_overlap_threshold  = 20.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 2.8f;   // 最大距离阈值(米)2.5->2.8
  if (bus_lane_info_.exists && bus_lane_info_.is_left_most) {
    // 步骤1: 找出与公交重叠最长且大于阈值的边界线

    BevLaneMarker         best_edge;
    double                max_overlap = -1.0;
    const Eigen::Vector2f ego_pt      = {0, 0};
    for (auto &edge : all_bev_edges) {
      bool is_left_of_line = IsPointLeftOfPolyline(ego_pt, *edge.geos);
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && !is_left_of_line)) {

        double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] bdry_2_harbor overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge   = edge;
        }
      }
    }
    // 步骤2: 找出与该边界线宽度最小且小于阈值的车道线
    uint64_t best_lane_id = 0;
    float    min_distance = std::numeric_limits<float>::max();
    for (auto &id : road_selected_buslane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_edge.geos->empty())) {
        double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *single_lane->geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*single_lane->geos, *best_edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] dis line " << single_lane->id << " to " << best_edge.id
                            << "  leftbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry > 0.5 && dis_to_rightbdry < min_distance && dis_to_rightbdry <= max_distance_threshold &&
            overlap_dis >= min_overlap_threshold) {
          min_distance = dis_to_rightbdry;
          best_lane_id = id;
        }
      }
    }
    SD_COARSE_MATCH_LOG << "[MatchBusLaneType]best_edge id: " << best_edge.id << "  max_overlap: " << max_overlap;
    SD_COARSE_MATCH_LOG << "[MatchBusLaneType]best_lane_id: " << best_lane_id << "  min_distance: " << min_distance;
    if (best_lane_id != 0) {
      indices_id_to_remove.push_back(best_lane_id);
    }
  } else if (bus_lane_info_.exists && bus_lane_info_.is_right_most) {
    // 步骤1: 找出与公交重叠最长且大于阈值的边界线

    BevLaneMarker         best_edge;
    double                max_overlap = -1.0;
    const Eigen::Vector2f ego_pt      = {0, 0};
    for (auto &edge : all_bev_edges) {
      bool is_left_of_line = IsPointLeftOfPolyline(ego_pt, *edge.geos);  //判断自车在线的左侧，即true则为自车右侧线
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && is_left_of_line)) {

        double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] bdry_2_harbor overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge   = edge;
        }
      }
    }
    // 步骤2: 找出与该边界线宽度最小且小于阈值的车道线
    uint64_t best_lane_id = 0;
    float    min_distance = std::numeric_limits<float>::max();
    for (auto &id : road_selected_buslane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_edge.geos->empty())) {
        double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *single_lane->geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*single_lane->geos, *best_edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] dis line " << single_lane->id << " to " << best_edge.id
                            << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry > 0.5 && dis_to_rightbdry < min_distance && dis_to_rightbdry <= max_distance_threshold &&
            overlap_dis >= min_overlap_threshold) {
          min_distance = dis_to_rightbdry;
          best_lane_id = id;
        }
      }
    }
    SD_COARSE_MATCH_LOG << "[MatchBusLaneType]best_edge id: " << best_edge.id << "  max_overlap: " << max_overlap;
    SD_COARSE_MATCH_LOG << "[MatchBusLaneType]best_lane_id: " << best_lane_id << "  min_distance: " << min_distance;
    if (best_lane_id != 0) {
      indices_id_to_remove.push_back(best_lane_id);
    }
  } else if (bus_lane_info_.exists) {
    /*对非最左和最右的车道进行选择*/
    if (!overlap_lanes.empty()) {
      SD_COARSE_MATCH_LOG << "[MatchBusLaneType] overlap_lanes : " << fmt::format(" [{}]  ", fmt::join(overlap_lanes, ", "));
      // 直接根据公交车道索引选择对应位置的车道
      size_t target_index = static_cast<size_t>(bus_lane_info_.index);

      // 边界保护：索引超出范围时选择最右侧车道
      if (target_index < overlap_lanes.size()) {

        // 添加选定车道的ID,后续添加保持策略
        //indices_id_to_remove.push_back(overlap_lanes[target_index]);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] Selected bus lane index: " << bus_lane_info_.index << ", Actual index: " << target_index
                            << ", Lane ID: " << overlap_lanes[target_index] << " But not push_back";
      } else {
        //target_index = overlap_lanes.size() - 1;
        //indices_id_to_remove.push_back(overlap_lanes[target_index]);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] No overlapping lanes found for bus lane index: " << bus_lane_info_.index;
      }

    } else {
      SD_COARSE_MATCH_LOG << "[MatchBusLaneType] No overlapping lanes found for bus lane index: " << bus_lane_info_.index;
    }
  } else {
  }
  /* check indices and calculate lat dis to left bdry */
  if (indices_id_to_remove.empty() && bus_lane_info_.exists) {
    //找出左侧边界线
    BevLaneMarker         best_edge;
    double                max_overlap = -1.0;
    const Eigen::Vector2f ego_pt      = {0, 0};
    for (auto &edge : all_bev_edges) {
      bool is_left_of_line = IsPointLeftOfPolyline(ego_pt, *edge.geos);
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && !is_left_of_line)) {

        double overlap_dis = calculateOverlapDisinRange(rangeStart, rangeEnd, *edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] bdry_2_BRT overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge   = edge;
        }
      }
    }
    // 找出与左边界线宽度符合车道索引阈值的车道线
    uint64_t best_lane_id = 0;
    for (auto &id : road_selected_buslane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_edge.geos->empty())) {
        auto   full_geometry = bev_data_processor_.GetFullLaneGeometry(single_lane);
        double overlap_dis   = calculateOverlapDisinRange(rangeStart, rangeEnd, *full_geometry);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*full_geometry, *best_edge.geos);
        SD_COARSE_MATCH_LOG << "[MatchBusLaneType] dis line " << single_lane->id << " to " << best_edge.id
                            << "  left bdry is : " << dis_to_rightbdry << " range overlap: " << overlap_dis;
        if (dis_to_rightbdry > 0.5 && overlap_dis >= min_overlap_threshold && dis_to_rightbdry > bus_lane_info_.index * 3.5 &&
            dis_to_rightbdry < (bus_lane_info_.index + 1) * 3.5) {
          best_lane_id = id;
          /* tbd  */
          // break;
        }
      }
    }
    if (best_lane_id != 0) {
      indices_id_to_remove.push_back(best_lane_id);
      SD_COARSE_MATCH_LOG << "[MatchBusLaneType] bus lane detected by lat dis to left bdry & lane index: "
                          << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
    }
  }

  if (!indices_id_to_remove.empty()) {
    SD_COARSE_MATCH_LOG << "[MatchBusLaneType] indices_id_to_remove : " << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
    UpdateBusLaneType(indices_id_to_remove[0], GlobalBevMapOutPut);
  }
}

void SdNavigationCity::UpdateBusLaneType(uint64_t target_buslane_id, BevMapInfoPtr &GlobalBevMapOutPut) {

  if (target_buslane_id == 0)
    return;

  // 更新对应车道的类型
  for (auto &lane : GlobalBevMapOutPut->lane_infos) {
    if (lane.id == target_buslane_id) {
      lane.lane_type = BevLaneType::LANE_BRT;
      //lane.lane_type = BevLaneType::LANE_TYPE_HARBOR_STOP; //debug
      lane.restricted_info.restricted_type    = Bev_RestrictedType::RT_BRT;
      lane.restricted_info.is_passable        = bus_lane_info_.is_passable;
      lane.restricted_info.passable_env_state = bus_lane_info_.passable_env_state;
      SD_COARSE_MATCH_LOG << "  [UpdateBusLaneType] Setting  BEV lane ID: " << lane.id
                          << " to LANE_BRT ,is_passable: " << lane.restricted_info.is_passable
                          << " passable_env_state: " << lane.restricted_info.passable_env_state;
    }
  }
}

/*
  * @brief 根据range信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
double SdNavigationCity::calculateOverlapDisinRange(double rangeStart, double rangeEnd, const std::vector<Eigen::Vector2f> geos) {
  //SD_COARSE_MATCH_LOG << "[calculateOverlapDisinRange]Entering...";

  // 确保范围有效
  if (rangeStart >= rangeEnd) {
    return 0.0;
  }

  // 获取车道线的geos点集
  //const auto& geos = *singlelane.geos;
  if (geos.size() < 2) {
    return 0.0;
  }

  // 直接使用第一个点和最后一个点的x坐标作为车道线的起点和终点
  double laneStart = geos.front().x();
  double laneEnd   = geos.back().x();

  // 确保车道线起点和终点的顺序正确
  if (laneStart > laneEnd) {
    std::swap(laneStart, laneEnd);
  }

  // 计算重叠部分
  double overlapStart = std::max(laneStart, rangeStart);
  double overlapEnd   = std::min(laneEnd, rangeEnd);
  //SD_COARSE_MATCH_LOG << "[calculateOverlapDisinRange] Exiting..."
  //                    << " overlapStart: " << overlapStart << " overlapEnd: " << overlapEnd;
  // 如果存在重叠，返回重叠距离
  return (overlapStart < overlapEnd) ? (overlapEnd - overlapStart) : 0.0;
}

void SdNavigationCity::SetACCLaneInfo(AccLaneInfo &sd_acc_lane_info) {

  SD_COARSE_MATCH_LOG << "[SetACCLaneInfo] Normal lane id: " << sd_acc_lane_info.ego_normal_lane_id;
}
void SdNavigationCity::SetReversibleLaneType(const std::shared_ptr<RoutingMap> &raw_routing_map, 
                                             BevMapInfoConstPtr &raw_bev_map, 
                                             BevMapInfoPtr &GlobalBevMapOutPut) {
  if(!raw_routing_map || !raw_bev_map || !GlobalBevMapOutPut) {
     return;
  }
  std::map<uint64_t, LaneInfo*> ld_lane_type_map;
  for(auto ld_lane : raw_routing_map->lanes) {
      if(ld_lane.type == cem::message::env_model::LaneType::LANE_REVERSIBLE) {
         ld_lane_type_map.insert({ld_lane.id, &ld_lane});
      }
  }
  if(ld_lane_type_map.empty()) {
     return;
  }
  auto& ld_match_infos = INTERNAL_PARAMS.ld_match_info_data.GetMatchInfo();
  for(auto &lane : GlobalBevMapOutPut->lane_infos) {
    uint64_t bev_id = lane.id;
    if(ld_match_infos.find(bev_id) != ld_match_infos.end()) {
      auto &matched_ld_ids = ld_match_infos.at(bev_id);
      for(auto map_id : matched_ld_ids) {
          if(ld_lane_type_map.find(map_id) != ld_lane_type_map.end()) {
            lane.lane_type = BevLaneType::LANE_REVERSIBLE;
            lane.restricted_info.restricted_type = cem::message::sensor::Bev_RestrictedType::RT_IDAL;
            lane.restricted_info.is_passable = true; //不在使用这个字段
            lane.restricted_info.passable_env_state = ld_lane_type_map[map_id]->restricted_info.passable_env_state;
            break;
          }
      }
    }
  }
}
/**
     * @brief 回溯检查当前车道的主前继链中是否在最多5层内存在 split_topo_extend == TOPOLOGY_SPLIT_RIGHT
     *        
     * @param start_lane 起始车道指针
     * @return true 如果发现符合条件的前继车道
     */
bool SdNavigationCity::HasAncestorWithSplitRight(const BevLaneInfo *start_lane) const {
  if (!start_lane)
    return false;

  std::set<uint64_t> visited;
  const BevLaneInfo *current   = start_lane;
  int                depth     = 0;
  const int          max_depth = 5;

  while (current && depth <= max_depth) {
    // 防止重复访问（环路检测）
    if (!visited.insert(current->id).second) {
      break;  // 已访问过，退出避免死循环
    }

    // 检查当前车道是否为右分叉出口
    if (current->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
      SD_COARSE_MATCH_LOG << "[HasAncestorWithSplitRight] Found TOPOLOGY_SPLIT_RIGHT at lane: " << current->id << " (depth: " << depth
                          << ")";
      return true;
    }

    // 移动到下一个前驱（只取第一个，作为主流向）
    if (current->previous_lane_ids.empty()) {
      break;
    }

    uint64_t prev_id   = current->previous_lane_ids[0];
    auto     prev_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(prev_id);
    if (!prev_lane) {
      SD_COARSE_MATCH_LOG << "[HasAncestorWithSplitRight] Failed to get previous lane info: " << prev_id;
      break;
    }

    current = prev_lane;
    ++depth;
  }

  return false;
}

/* harbor 车道能否过滤的校验 */
bool SdNavigationCity::RemoveHarborlaneCheck(const HarborStopInfo  &sd_harbor_stop_info_,
                                             std::vector<uint64_t> &road_selected_buslane_filtered) {
  bool remove_enable      = false;
  bool right_lenght_check = false;
  bool left_lenght_check  = false;
  if (sd_harbor_stop_info_.exists) {
    std::pair<double, double> firstRange = {0, 0};
    // 获取第一个港湾车道范围 [start_s, end_s]
    for (auto range : sd_harbor_stop_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
    double harborStart     = firstRange.first;
    double harborEnd       = firstRange.second;
    double harbor_length   = harborEnd - harborStart;
    auto   right_most_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.back());
    auto   left_most_lane  = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.front());
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] harborStart:" << harborStart << ", harborEnd:" << harborEnd
                        << " , harbor_length: " << harbor_length;

    if ((sd_harbor_stop_info_.is_left_most) && (left_most_lane) && (left_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
      double left_lane_length = left_most_lane->geos->back().x() - left_most_lane->geos->front().x();
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] harbor_length :  " << harbor_length << "left_lane_length " << left_lane_length;
      if (left_lane_length < harbor_length + 50) {
        left_lenght_check = true;
      } else {
        SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] left check fail for lane length is :  " << left_lane_length << " <  "
                            << harbor_length << " + 50 in city.";
      }
    } else {
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] left check fail for lane position is :  " << left_most_lane->position
                          << " not ego: " << (int)BevLanePosition::LANE_LOC_EGO;
    }
    if ((sd_harbor_stop_info_.is_right_most) && (right_most_lane) && (right_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO) &&
        (right_most_lane->previous_lane_ids.size() < 2)) {
      bool   is_split_right    = HasAncestorWithSplitRight(right_most_lane);
      double right_lane_length = right_most_lane->geos->back().x() - right_most_lane->geos->front().x();
      if (right_lane_length < harbor_length + 50 || is_split_right) {
        right_lenght_check = true;
      } else {
        SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] right check fail for lane length is :  " << right_lane_length << " <  "
                            << harbor_length << " + 50 in city.";
      }
    } else {
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] right check fail for lane position is :  " << right_most_lane->position
                          << " not ego: " << (int)BevLanePosition::LANE_LOC_EGO;
    }

    remove_enable = (left_lenght_check || right_lenght_check);
  }
  SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] length & position check result :  " << remove_enable;
  return remove_enable;
}

/*
   * @brief 过滤掉加速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉加速车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationCity::FilterACCLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                      const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                      bool &approaching_merge, JunctionInfoCity &merge_jct, uint64_t &acc_adj_lane_id) {
  // 判断road_selected是否为空，空则返回 2025.10.15
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[FilterACCLane]  road_selected is empty! ";
    return road_selected;
  }
  std::vector<uint64_t> road_selected_acclane_filtered(road_selected);
  SD_COARSE_MATCH_LOG << "Entering FilterACCLane function with inputs: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_acclane_filtered, ", ")) << " size: " << road_selected.size();
  // 获取车道列表

  size_t lane_count             = 0;
  size_t current_lane_count     = 0;
  size_t map_normal_lane_count  = 0;
  double min_overlap_threshold  = 15.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 3.0f;   // 最大距离阈值(米)

  if (merge_jct.junction_id != 0 && merge_jct.offset < 300) {
    map_normal_lane_count = merge_jct.target_road_lane_nums;

    for (auto &id : road_selected_acclane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
      if ((single_lane) && (!single_lane->geos->empty())) {

        double overlap_dis     = calculateOverlapDisinRange(-50.0, 150.0, *single_lane->geos);
        double cur_overlap_dis = calculateOverlapDisinRange(-50.0, 150.0, *single_lane->geos);
        SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << single_lane->geos->front().x()
                            << " lane_end_x: " << single_lane->geos->back().x() << "overlap_dis: " << overlap_dis;
        if ((overlap_dis > min_overlap_threshold)) {
          lane_count++;
        }
        if ((cur_overlap_dis > min_overlap_threshold)) {
          current_lane_count++;
        }
      }
    }
  }
  SD_COARSE_MATCH_LOG << "[FilterACCLane] acc overlap lane_count :" << lane_count << " ,current_lane_count: " << current_lane_count
                      << " Current maplane num(normal): " << map_normal_lane_count;
  /* 300m内出现合流路口 */
  if (merge_jct.offset < 300) {
    /* 找到最右车道 */
    uint64_t right_most_laneid = road_selected_acclane_filtered.back();

    /* 自车在主路 感知车道数大于地图普通车道数 acc范围内 */
    if (approaching_merge && (merge_jct.offset < 300.0) && (merge_jct.offset > 0.0) && (map_normal_lane_count > 0)) {
      if ((current_lane_count >= map_normal_lane_count) && (road_selected_acclane_filtered.size() >= map_normal_lane_count)) {
        right_most_laneid == road_selected_acclane_filtered[map_normal_lane_count - 1]
            ? acc_adj_lane_id = road_selected_acclane_filtered[map_normal_lane_count - 1]
            : acc_adj_lane_id = 0;
        SD_COARSE_MATCH_LOG << "[FilterACCLane]extra index: " << map_normal_lane_count - 1 << " acc_adj_lane_id: " << acc_adj_lane_id;

      } else {
        /* 后续添加路沿校验 */
        acc_adj_lane_id = right_most_laneid;
        SD_COARSE_MATCH_LOG << "[FilterACCLane] right_most: acc_adj_lane_id: " << acc_adj_lane_id;
      }
    } else {
    }
  }

  SD_COARSE_MATCH_LOG << "[FilterACCLane] acc_adj_lane_id: " << acc_adj_lane_id;

  return road_selected_acclane_filtered;
}

// 加速车道邻车道
void SdNavigationCity::SetBevAccAdjLane(BevMapInfo &bev_map, const uint64_t &AccAdjLane_id, JunctionInfoCity &merge_jct) {
  auto &bev_lane = bev_map.lane_infos;
  if (sd_acc_lane_info_.exists) {
    // 获取第一个 加速车道范围 [start_s, end_s]
    std::pair<double, double> firstRange = {0, 0};
    for (auto range : sd_acc_lane_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }

    double mergeStart = firstRange.first;
    double mergeEnd   = firstRange.second;

    if (AccAdjLane_id != 0) {
      auto bev_map_lane = std::find_if(bev_lane.begin(), bev_lane.end(),
                                       [AccAdjLane_id](const BevLaneInfo &bev_lane_in) { return bev_lane_in.id == AccAdjLane_id; });
      if (bev_map_lane != bev_lane.end()) {
        // bev_map_lane->lane_type = BevLaneType::LANE_ACC_ADJ;
        bev_map_lane->is_acc_adj_lane = true;
        bev_map_lane->merge_start_dis = mergeStart;
        bev_map_lane->merge_end_dis   = mergeEnd;

        SD_COARSE_MATCH_LOG << "  [SetBevAccAdjLane] Setting  BEV lane ID: " << AccAdjLane_id << " to LANE_ACC_ADJ"
                            << " merge_start_dis: " << mergeStart << " mergeEnd: " << mergeEnd;
      }
    }
  } else if (AccAdjLane_id != 0 && merge_jct.junction_id != 0 && 0 < merge_jct.offset && merge_jct.offset < 300) {
    /* 非加速车道场景的途径汇入的车道属性赋值 */
    auto bev_map_lane = std::find_if(bev_lane.begin(), bev_lane.end(),
                                     [AccAdjLane_id](const BevLaneInfo &bev_lane_in) { return bev_lane_in.id == AccAdjLane_id; });
    if (bev_map_lane != bev_lane.end()) {
      // bev_map_lane->lane_type = BevLaneType::LANE_ACC_ADJ;
      bev_map_lane->is_acc_adj_lane = true;
      bev_map_lane->merge_start_dis = merge_jct.offset;
      bev_map_lane->merge_end_dis   = merge_jct.offset;

      SD_COARSE_MATCH_LOG << "  [SetBevAccAdjLane] Setting  BEV lane ID: " << AccAdjLane_id << " to LANE_ACC_ADJ"
                          << " merge_start_dis: " << merge_jct.offset << " mergeEnd: " << merge_jct.offset;
    }
  } else {
  }
}

/*
   * @brief 判断是否是途径汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
bool SdNavigationCity::IsApproachMergeJCT(JunctionInfoCity &merge_jct) {
  bool approaching_merge = false;  //途径汇入口

  for (auto jct : junctions_info_city_) {
    /* SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge ,offset: " << junction.offset
                        << StrJunctionType(junction.junction_type) << "  junction's main_road_lane_nums:  " << junction.main_road_lane_nums
                        << "  road_merge_main_road_lane_nums:  " << junction.road_merge_main_road_lane_nums
                        << "  target_road_lane_nums:  " << junction.target_road_lane_nums;*/
    SD_COARSE_MATCH_LOG << fmt::format("{}", jct);
  }
  JunctionInfoCity candidate_jct;
  for (auto junction : junctions_info_city_) {
    //auto junction = junctions_info_city_.front();

    /* 自车在主路、前方合流、车道数未增加 */
    if (junction.offset < 300 && junction.offset > 0.0 && junction.junction_state_city != JunctionStateCity::PASSED) {
      SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] prev_road_class size: " << junction.prev_road_class.size()
                          << " ,road_merge_main_road_lane_nums: " << junction.road_merge_main_road_lane_nums
                          << " target_road_lane_nums: " << junction.target_road_lane_nums;
      candidate_jct = junction;
      break;
    }
  }

  if (candidate_jct.junction_id != 0 && candidate_jct.junction_type_city == JunctionTypeCity::RoadMerge &&
      candidate_jct.prev_road_class.size() == 2 && candidate_jct.road_merge_main_road_lane_nums == candidate_jct.target_road_lane_nums) {
    SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] split_merge_direction: " << (int)candidate_jct.split_merge_direction;
    /* left 指自车在左侧支路汇入主路 */
    if ((candidate_jct.split_merge_direction == DirectionSplitMerge::Left && candidate_jct.prev_road_class.front() == RoadMainType::RoadMain) /*||
        (candidate_jct.split_merge_direction == DirectionSplitMerge::Right && candidate_jct.prev_road_class.back() == RoadMainType::RoadMain)*/) {
      approaching_merge = true;
      merge_jct         = candidate_jct;
      SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge ,offset: " << candidate_jct.offset
                          << StrJunctionType(candidate_jct.junction_type)
                          << "  junction's main_road_lane_nums:  " << candidate_jct.main_road_lane_nums
                          << "  road_merge_main_road_lane_nums:  " << candidate_jct.road_merge_main_road_lane_nums
                          << "  target_road_lane_nums:  " << candidate_jct.target_road_lane_nums;
    }
  }
  SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge:  " << approaching_merge;
  return approaching_merge;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem