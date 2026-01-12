#include "SdNavigationHighway.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <iterator>
#include <base/sensor_data_manager.h>
#include <common/utils/lane_geometry.h>
#include <fmt/ranges.h>
#include <message/env_model/navigation/navigation.h>
#include <message/env_model/routing_map/routing_map.h>
#include "fmt/core.h"
#include "fmt/format.h"
#include "routing_map_debug.h"
#include "modules/msg/environment_model_msgs/local_map_info.pb.h"
#include "cyber_release/include/cyber/time/time.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/perception/env/src/lib/sd_navigation/SDMapElementExtract.h"

// #define ENABLE_DEBUG_SDNAVIGATION 1

namespace cem {
namespace fusion {
namespace navigation {

SdNavigationHighway::SdNavigationHighway() {}

void SdNavigationHighway::Proc(const std::shared_ptr<RoutingMap> &raw_routing_map, BevMapInfoConstPtr &raw_bev_map,
                               BevMapInfoPtr &GlobalBevMapOutPut, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                               std::vector<uint64_t> &sd_guide_lane_ids, AccLaneInfo &sd_acc_lane_info) {
  AINFO << fmt::format("Enter SdNavigationHighway::Proc logic.");
#ifdef ENABLE_DEBUG_SDNAVIGATION
  AINFO << fmt::format("{}", raw_routing_map->sd_route);
#endif
  INTERNAL_PARAMS.navigation_info_data.ClearContainer();
  if (!raw_routing_map || !raw_bev_map || !GlobalBevMapOutPut) {
    return;
  }
  auto& ld_match_infos = INTERNAL_PARAMS.ld_match_info_data.GetMatchInfo();


  uint64_t bev_counter                                                = raw_bev_map == nullptr ? 0 : raw_bev_map->header.cycle_counter;
  INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().bev_counter = bev_counter;
  // AINFO << "[SDNOA] ============start================";
  sd_junction_infos_.clear();
  ego_section_id_ = 0;
  bev_candidate_lanes_.clear();
  bev_line_sorts_.clear();
  bev_left_road_edge_indexs_.clear();
  bev_right_road_edge_indexs_.clear();
  bev_egoRoadEdgeIndexPair_ = {-1, -1};
  bev_sections_.clear();
  bev_ego_lane_related_.clear();
  guide_lane_result.clear();
  sd_guide_lane_ids.clear();
  ld_info_buffer_.clear();

  ///获取LD导航动作
  GetLDJunctionInfos();
  SetLDJunctionInfo();
  ///Bev感知数据处理
  BevMapProcess();

  ///更新导航动作事件
  int current_target_junction_index = -1;
  ManageJunctionEvent(current_target_junction_index);
  ///更新last_passed
  JunctionEvent_last_passed_junction_id_ = 0;
  for (auto it = sd_junction_infos_.rbegin(); it != sd_junction_infos_.rend(); ++it) {
    if (it->has_passed_flag) {
      JunctionEvent_last_passed_junction_id_ = it->junction_id;
      break;
    }
  }

  for (auto &sd_junction_tmp : sd_junction_infos_) {
    SD_BEV_PROCESS << "[SDNOA Junction] ," << sd_junction_tmp.junction_id << "," << (int)sd_junction_tmp.junction_type << ",  "
                   << sd_junction_tmp.offset << "  ," << sd_junction_tmp.main_road_lane_nums << "," << sd_junction_tmp.target_road_lane_nums
                   << "," << (int)sd_junction_tmp.has_passed_flag << "," << (int)sd_junction_tmp.has_effected_flag;
  }

  SD_BEV_PROCESS << fmt::format("Before SelectBevNavigationLanes: sections size: {}", bev_sections_.size());
  for (size_t i = 0; i < bev_sections_.size(); ++i) {
    SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(bev_sections_[i], ", "));
  }

  std::pair<uint64_t, uint64_t> fusion_emergency_laneid = {};

  SelectBevNavigationLanes(raw_routing_map, current_target_junction_index, bev_sections_, fusion_emergency_laneid.second,
                           fusion_emergency_laneid.first, guide_lane_result, sd_guide_lane_ids, GlobalBevMapOutPut);

  SetACCLaneInfo(sd_acc_lane_info);

  if (GlobalBevMapOutPut) {
    SetBevEmergencyLane(*GlobalBevMapOutPut, fusion_emergency_laneid);
  }

  HoldGuideLanes(bev_sections_, guide_lane_result, sd_guide_lane_ids, GlobalBevMapOutPut);

  history_guide_laneids_.clear();
  history_guide_laneids_ = sd_guide_lane_ids;
  // AINFO << "[SDNOA Emergency id] " << history_bev_left_emergency_laneid_ << "," << history_bev_right_emergency_laneid_;

  // if (IsCutMapActive()) {
  //SetLdRecommendLane();
  // SelectGuideLanesWithLd(bev_ego_root_section_x0_, map_ld_recommend_lane_,
  //                        GlobalBevMapOutPut, guide_lane_result,
  //                        sd_guide_lane_ids);
  // }

  std::ostringstream os;
  for (auto &guide_laneid : sd_guide_lane_ids) {
    os << guide_laneid << ",";
  }
  // AINFO << "[SDNOA GUIDE] " << os.str();

  if (GlobalBevMapOutPut) {
    SetNaviInterfaceAndDebugInfos(GlobalBevMapOutPut);
  }

  SD_BEV_PROCESS << fmt::format("  Output final_guide_laneids: [{}]", sd_guide_lane_ids);
}
void SdNavigationHighway::SetLDJunctionInfo() {
  fmt::format_to(ld_info_buffer_, "  all_ld_jun_info:[");
  for (const auto &jun : sd_junction_infos_) {
    fmt::format_to(ld_info_buffer_, "{},{},{:.0f},{},{},{};", jun.junction_id, StrNaviJunctionHighway(static_cast<int>(jun.junction_type)),
                   jun.offset, StrNaviSplitDirectionHighway(jun.split_direction), jun.main_road_navigation_lane_num, jun.split_num);
    SD_HIGHJUNCTION_CONVERTE_LOG << fmt::format("{}", jun);
  }
  fmt::format_to(ld_info_buffer_, "]  ");
}

void SdNavigationHighway::SelectBevNavigationLanes(const std::shared_ptr<RoutingMap> &raw_routing_map, int current_target_junction_index,
                                                   const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                   uint64_t                                 &current_right_bev_emergency_laneid,
                                                   uint64_t                                 &current_left_bev_emergency_laneid,
                                                   std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                                   std::vector<uint64_t> &guide_laneids_ref, BevMapInfoPtr &GlobalBevMapOutPut) {

  current_right_bev_emergency_laneid = 0;
  current_left_bev_emergency_laneid  = 0;
  guide_lane_result.clear();
  guide_laneids_ref.clear();

  if (bev_sections_in.empty()) {
    return;
  }
  SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] GetEmergencyLaneInfo ";
  ///获取sdmap中的应急车道信息
  EmergencyLaneInfo emergency_lane_info = GetEmergencyLaneInfo();
  SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] GetHarborStopInfo ";
  GetHarborStopInfo();
  //GetAccLaneInfo();
  GetDecLaneInfo();

  std::vector<JunctionInfo *> target_sd_junctions = {};
  if (current_target_junction_index != -1) {
    for (unsigned int i = current_target_junction_index; i < sd_junction_infos_.size(); i++) {
      if (sd_junction_infos_[i].offset > 2000) {
        break;
      }
      if (IsInterestedJunction(sd_junction_infos_[i])) {
        target_sd_junctions.push_back(&sd_junction_infos_[i]);
        break;
      }
      target_sd_junctions.push_back(&sd_junction_infos_[i]);
    }
  }
  SD_COARSE_MATCH_LOG << " [junction filter] jcts: ";
  for (const auto &junction : target_sd_junctions) {
    SD_COARSE_MATCH_LOG << fmt::format("{}", junction);
  }
  SD_COARSE_MATCH_LOG << " [junction filter] end ================================== ";
  /* CNOAC2-100318 */
  if (target_sd_junctions.size() >= 3) {
    if (target_sd_junctions.back()->junction_type == JunctionType::RampInto &&
        target_sd_junctions[0]->junction_type == JunctionType::ApproachRampInto &&
        target_sd_junctions[1]->junction_type == JunctionType::ApproachRampInto) {
      int ReferenceLaneNum0 = std::min(target_sd_junctions[0]->target_road_lane_nums, target_sd_junctions[0]->main_road_lane_nums);
      int ReferenceLaneNum1 = std::min(target_sd_junctions[1]->target_road_lane_nums, target_sd_junctions[1]->main_road_lane_nums);
      SD_COARSE_MATCH_LOG << "[junction filter] :  "
                          << "ReferenceLaneNum0: " << ReferenceLaneNum0 << " , ReferenceLaneNum1: " << ReferenceLaneNum1;
      if (ReferenceLaneNum0 > ReferenceLaneNum1) {
        target_sd_junctions.erase(target_sd_junctions.begin());
      }
    }
  }
  if (target_sd_junctions.size() >= 2) {
    std::vector<JunctionInfo *> new_target_sd_junctions = {};
    if (IsInterestedJunction(*target_sd_junctions.back())) {
      if (IsV2IgnoredJunction(*target_sd_junctions.front())) {
        new_target_sd_junctions.push_back(target_sd_junctions.front());
        for (unsigned int i = 1; i < target_sd_junctions.size() - 1; i++) {
          if (!IsV2IgnoredJunction(*target_sd_junctions[i])) {
            new_target_sd_junctions.push_back(target_sd_junctions[i]);
            break;
          }
        }
        new_target_sd_junctions.push_back(target_sd_junctions.back());
      } else {
        new_target_sd_junctions.push_back(target_sd_junctions.front());
        new_target_sd_junctions.push_back(target_sd_junctions.back());
      }

    } else {

      if (IsV2IgnoredJunction(*target_sd_junctions.front())) {
        new_target_sd_junctions.push_back(target_sd_junctions.front());
        for (unsigned int i = 1; i < target_sd_junctions.size(); i++) {
          if (!IsV2IgnoredJunction(*target_sd_junctions[i])) {
            new_target_sd_junctions.push_back(target_sd_junctions[i]);
            break;
          }
        }
      } else {
        new_target_sd_junctions.push_back(target_sd_junctions.front());
      }
    }
    target_sd_junctions = new_target_sd_junctions;
  }

  int selected_section_index = -1;

  //////////////////////1 无导航信息 导航车道全给
  auto &navi_debug_info = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_info.all_separators.clear();
  for (size_t m = 1; m < bev_line_sorts_.size() - 1; m++) {
    if (bev_line_sorts_[m].is_road_boundary && !bev_line_sorts_[m - 1].is_road_boundary && !bev_line_sorts_[m + 1].is_road_boundary) {
      std::vector<std::pair<uint64_t, int>> group;
      group.emplace_back(bev_line_sorts_[m].id, 1);
      navi_debug_info.all_separators.push_back(group);
    }
  }
  if (target_sd_junctions.empty()) {

    //选道路
    selected_section_index = SelectMainRoadSectionNoNavi(bev_sections_in);
    auto &target_section   = bev_sections_in[selected_section_index];
    SD_COARSE_MATCH_LOG << " [SelectBevNavigationLanes]target_section: " << fmt::format(" [{}]  ", fmt::join(target_section, ", "));

    ///排除应急车道
    FusionCurrentBevEmergencyLaneid(target_section, emergency_lane_info, selected_section_index, current_left_bev_emergency_laneid,
                                    current_right_bev_emergency_laneid);

    std::vector<uint64_t> bev_section_tmp = target_section;
    if (current_left_bev_emergency_laneid != 0 && bev_section_tmp.size() > 1) {
      bev_section_tmp.erase(bev_section_tmp.begin());
      SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] erase the left emergency_lane .";
    }
    if (current_right_bev_emergency_laneid != 0 && bev_section_tmp.size() > 1) {
      bev_section_tmp.pop_back();
      SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] erase the right emergency_lane .";
    }

    /* 港湾临停车道的过滤 */
    auto bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
    if (!bev_map_ptr) {
      SD_COARSE_MATCH_LOG << fmt::format("[SelectBevNavigationLanes] bev_map is null, exiting .");
      return;
    }
    std::vector<uint64_t> target_section_without_harbor;

    target_section_without_harbor =
        FilterSpecialLane(raw_routing_map, bev_section_tmp, bev_map_ptr, target_sd_junctions, current_right_bev_emergency_laneid,
                          current_left_bev_emergency_laneid, emergency_lane_info, GlobalBevMapOutPut);

    for (auto &laneid_tmp : target_section_without_harbor) {
      guide_lane_result.push_back({laneid_tmp, {-1, 0}});
      navi_debug_info.guide_lanes_length.push_back({-1, 0});
    }
    guide_laneids_ref = target_section_without_harbor;

    SD_COARSE_MATCH_LOG << " [SelectBevNavigationLanes]guide_laneids: " << fmt::format(" [{}]  ", fmt::join(guide_laneids_ref, ", "));

    navi_debug_info.all_sections_in       = bev_sections_in;
    navi_debug_info.target_section        = target_section;
    navi_debug_info.guide_laneids         = target_section_without_harbor;
    navi_debug_info.guide_laneids_refer   = guide_laneids_ref;
    navi_debug_info.sections_without_bus  = target_section_without_harbor;
    navi_debug_info.target_section_filter = target_section_without_harbor;
    navi_debug_info.match_type.push_back(StrNaviMatchTypeHighway(1));
    return;
  }

  /////////////2 有导航信息 按照junction类型给导航车道
  /*****选道路*****/
  { /*选路的输入*/
    SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] hghiway navi RoadSelect start... ";
    SD_COARSE_MATCH_LOG << "Before SelectOptRoad: bev_sections size: " << bev_sections_in.size();
    for (size_t i = 0; i < bev_sections_in.size(); ++i) {
      std::ostringstream section_str;
      for (const auto &lane_id : bev_sections_in[i]) {
        section_str << lane_id << ",";
      }
      SD_COARSE_MATCH_LOG << "  Section " << i << ": " << section_str.str();
    }

    // SD_COARSE_MATCH_LOG << "Entering SelectBevNavigationLanes inputs: "
    //                << fmt::format("road_selected: [{}] ", fmt::join(road_selected_in, ", "));
    /*高速路口信息 */
    SD_COARSE_MATCH_LOG << "junctions_info_highway details:";
    SD_COARSE_MATCH_LOG << "target_sd_junctions.size :" << target_sd_junctions.size();

    for (const auto &junction : target_sd_junctions) {
      SD_COARSE_MATCH_LOG << fmt::format("{}", junction);
    }

    if (bev_sections_in.size() > 1) {
      //SD_JUNCTION_CONVERTE_LOG << fmt::format("{}", bev_sections_in);
      //SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes]  : " << fmt::format("[{}]", fmt::join(bev_sections_in, ", "));
      JunctionInfo *sd_junction = nullptr;
      for (const auto &junction : target_sd_junctions) {
        if (Junction_select_road_dist_map_.find(junction->junction_type) != Junction_select_road_dist_map_.end()) {
          sd_junction = junction;
          break;
        }
      }
      if (sd_junction != nullptr) {
        SD_COARSE_MATCH_LOG << "[RoadSelect] sd_junction offset: " << sd_junction->offset << "  选路生效距离： "
                            << Junction_select_road_dist_map_[sd_junction->junction_type];
        if (sd_junction->offset < Junction_select_road_dist_map_[sd_junction->junction_type]) {
          // 使用代价函数选择最优section
          std::vector<int> ego_section_indexs = {};  /// 自车所在的section index
          for (unsigned int i = 0; i < bev_sections_in.size(); i++) {
            for (unsigned int j = 0; j < bev_sections_in[i].size(); j++) {
              if (bev_ego_lane_related_.find(bev_sections_in[i][j]) != bev_ego_lane_related_.end()) {
                ego_section_indexs.emplace_back(i);
                break;
              }
            }
          }
          // 根据路口类型确定候选section
          std::vector<int> candidate_sections;
          if (sd_junction->junction_type == JunctionType::RampInto) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] RampInto junction select road. ";

            candidate_sections = {static_cast<int>(bev_sections_in.size() - 1)};

          } else if (sd_junction->junction_type == JunctionType::RampSplitLeft) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] RampSplitLeft junction select road. ";

            candidate_sections = {0};

          } else if (sd_junction->junction_type == JunctionType::RampSplitRight) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] RampSplitRight junction select road. ";

            candidate_sections = {static_cast<int>(bev_sections_in.size() - 1)};

          } else if (sd_junction->junction_type == JunctionType::RampSplitMiddle) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] RampSplitMiddle junction select road. ";

            if (bev_sections_in.size() > 2) {
              candidate_sections = {1};
            } else {
              candidate_sections = {0, static_cast<int>(bev_sections_in.size() - 1)};
            }

          } else if (sd_junction->junction_type == JunctionType::ApproachRampInto) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] ApproachRampInto junction select road. ";
            // 对于ApproachRampInto，考虑所有section作为候选
            for (size_t i = 0; i < bev_sections_in.size(); ++i) {
              candidate_sections.push_back(static_cast<int>(i));
            }
          } else if (sd_junction->junction_type == JunctionType::ApproachRampMerge) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] ApproachRampMerge junction select road. ";
            // 对于ApproachRampMerge，考虑所有section作为候选
            for (size_t i = 0; i < bev_sections_in.size(); ++i) {
              candidate_sections.push_back(static_cast<int>(i));
            }
          } else if (sd_junction->junction_type == JunctionType::MainRoadSplitLeft) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] MainRoadSplitLeft junction select road. ";
            candidate_sections = {0};
          } else if (sd_junction->junction_type == JunctionType::MainRoadSplitRight) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] MainRoadSplitRight junction select road. ";
            candidate_sections = {static_cast<int>(bev_sections_in.size() - 1)};
          } else if (sd_junction->junction_type == JunctionType::MainRoadSplitMiddle) {
            SD_COARSE_MATCH_LOG << "[RoadSelect] MainRoadSplitMiddle junction select road. ";
            if (bev_sections_in.size() > 2) {
              candidate_sections = {1};
            } else {
              candidate_sections = {0, static_cast<int>(bev_sections_in.size() - 1)};
            }
          }

          // 使用代价函数选择最优section
          double min_cost = std::numeric_limits<double>::max();
          for (int section_index : candidate_sections) {
            double cost = CalculateSectionCost(section_index, bev_sections_in, ego_section_indexs, target_sd_junctions);
            SD_COARSE_MATCH_LOG << " [RoadSelect] section " << section_index << " cost: " << cost;

            if (cost < min_cost) {
              min_cost               = cost;
              selected_section_index = section_index;
            }
          }

          if (selected_section_index != -1) {
            navi_debug_info.select_road_junction.push_back(sd_junction->junction_id);
          }
        }
      }
      if (selected_section_index == -1) {
        SD_COARSE_MATCH_LOG << "[RoadSelect] no section is selected , select cruise  section";
        selected_section_index = SelectMainRoadSectionNoNavi(bev_sections_in);
      }

    } else {
      SD_COARSE_MATCH_LOG << "[RoadSelect] default case , select index 0";
      selected_section_index = 0;
    }
  }

  /**********选车道*******/
  {
    std::vector<uint64_t> target_section = bev_sections_in[selected_section_index];
    std::vector<uint64_t> target_section_without_harbor;

    navi_debug_info.all_sections_in = bev_sections_in;
    navi_debug_info.target_section  = target_section;

    /// 排除应急车道
    FusionCurrentBevEmergencyLaneid(target_section, emergency_lane_info, selected_section_index, current_left_bev_emergency_laneid,
                                    current_right_bev_emergency_laneid);
    if (current_left_bev_emergency_laneid != 0 && target_section.size() > 1) {
      target_section.erase(target_section.begin());
      SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] erase the left emergency_lane . id: " << current_right_bev_emergency_laneid;
    }
    if (current_right_bev_emergency_laneid != 0 && target_section.size() > 1) {
      target_section.pop_back();
      SD_COARSE_MATCH_LOG << "[SelectBevNavigationLanes] erase the right emergency_lane . id: " << current_right_bev_emergency_laneid;
    }

    /* 港湾临停车道的过滤 */
    auto bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
    if (!bev_map_ptr) {
      SD_COARSE_MATCH_LOG << fmt::format("[SelectBevNavigationLanes] bev_map is null, exiting .");
      return;
    }
    target_section_without_harbor =
        FilterSpecialLane(raw_routing_map, target_section, bev_map_ptr, target_sd_junctions, current_right_bev_emergency_laneid,
                          current_left_bev_emergency_laneid, emergency_lane_info, GlobalBevMapOutPut);
    auto &navi_debug_infos                 = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
    navi_debug_infos.sections_without_bus  = target_section_without_harbor;
    navi_debug_infos.target_section_filter = target_section_without_harbor;

    for (auto &target_junction_tmp : target_sd_junctions) {
      navi_debug_infos.target_junction_ids.emplace_back(target_junction_tmp->junction_id);
      navi_debug_infos.junction_offset.emplace_back(target_junction_tmp->offset);
      navi_debug_info.main_road_navigation_lane_num.emplace_back(target_junction_tmp->main_road_navigation_lane_num);
    }

    for (const auto &junction : target_sd_junctions) {
      SD_COARSE_MATCH_LOG << fmt::format("{}", junction);
    }

    /* 根据路口数量进行选道 */
    if (target_sd_junctions.size() == 1) {
      SD_COARSE_MATCH_LOG << "[guidelanes] deal with  1 junction: ";

      std::vector<uint64_t> guide_laneids = {};
      int                   match_type    = 0;
      JunctionNaviMatch(target_sd_junctions.front(), target_section_without_harbor, match_type, guide_laneids);
      for (auto &laneid_tmp : guide_laneids) {
        guide_lane_result.push_back({laneid_tmp, {-1, 0}});
        navi_debug_info.guide_lanes_length.push_back({-1, 0});
      }
      guide_laneids_ref = guide_laneids;

      navi_debug_info.guide_laneids       = guide_laneids;
      navi_debug_info.guide_laneids_refer = guide_laneids_ref;
      navi_debug_info.match_type.push_back(StrNaviMatchTypeHighway(match_type));
      //v2_junction_ids.push_back(target_sd_junctions.front()->junction_id);

    } else {

      std::vector<uint64_t> first_guide_laneids  = {};
      std::vector<uint64_t> second_guide_laneids = {};
      std::vector<uint64_t> third_guide_laneids  = {};
      int                   match_type1, match_type2, match_type3 = 0;
      JunctionNaviMatch(target_sd_junctions.front(), target_section_without_harbor, match_type1, first_guide_laneids);
      SD_COARSE_MATCH_LOG << "[guidelanes]   junctions size : " << target_sd_junctions.size();
      if (target_sd_junctions.size() >= 2) {
        SD_COARSE_MATCH_LOG << "[guidelanes] deal with  more than 2 junctions: ";
        JunctionNaviMatch(target_sd_junctions[1], first_guide_laneids, match_type2, second_guide_laneids);
      }

      if (target_sd_junctions.size() >= 3) {
        SD_COARSE_MATCH_LOG << "[guidelanes] deal with  more than 3 junctions: ";
        JunctionNaviMatch(target_sd_junctions[2], second_guide_laneids, match_type3, third_guide_laneids);
      }

      if (target_sd_junctions.size() == 3) {
        SD_COARSE_MATCH_LOG << "[guidelanes] deal with  3 junctions: ";

        for (auto &laneid_tmp : second_guide_laneids) {
          auto find_itor = std::find(third_guide_laneids.begin(), third_guide_laneids.end(), laneid_tmp);
          if (find_itor != third_guide_laneids.end()) {
            std::pair<int, int> guide_dist = {(int)target_sd_junctions[2]->offset, -1};
            guide_lane_result.emplace_back(laneid_tmp, guide_dist);
            navi_debug_info.guide_lanes_length.push_back({(int)target_sd_junctions[2]->offset, -1});
          } else {
            std::pair<int, int> guide_dist = {(int)target_sd_junctions[2]->offset, 0};
            guide_lane_result.emplace_back(laneid_tmp, guide_dist);
            navi_debug_info.guide_lanes_length.push_back({(int)target_sd_junctions[2]->offset, 0});
          }
        }
        navi_debug_info.guide_laneids = second_guide_laneids;
        guide_laneids_ref             = third_guide_laneids;
        navi_debug_info.guide_laneids_refer = guide_laneids_ref;
        navi_debug_info.match_type    = {StrNaviMatchTypeHighway(match_type1), StrNaviMatchTypeHighway(match_type2),
                                         StrNaviMatchTypeHighway(match_type3)};

        //v2_junction_ids = {target_sd_junctions[1]->junction_id, target_sd_junctions[2]->junction_id};
      }

      if (target_sd_junctions.size() == 2) {
        SD_COARSE_MATCH_LOG << "[guidelanes] deal with  2 junctions: ";
        navi_debug_info.match_type = {StrNaviMatchTypeHighway(match_type1), StrNaviMatchTypeHighway(match_type2)};

        if (IsV2IgnoredJunction(*target_sd_junctions[0])) {

          for (auto &laneid_tmp : second_guide_laneids) {
            guide_lane_result.push_back({laneid_tmp, {-1, 0}});
            navi_debug_info.guide_lanes_length.push_back({-1, 0});
          }
          navi_debug_info.guide_laneids = second_guide_laneids;
          guide_laneids_ref             = second_guide_laneids;
          //v2_junction_ids               = {target_sd_junctions[1]->junction_id};

        } else {

          for (auto &laneid_tmp : first_guide_laneids) {
            auto find_itor = std::find(second_guide_laneids.begin(), second_guide_laneids.end(), laneid_tmp);
            if (find_itor != second_guide_laneids.end()) {
              std::pair<int, int> guide_dist = {(int)target_sd_junctions[1]->offset, -1};
              guide_lane_result.emplace_back(laneid_tmp, guide_dist);
              navi_debug_info.guide_lanes_length.push_back({(int)target_sd_junctions[1]->offset, -1});
            } else {
              std::pair<int, int> guide_dist = {(int)target_sd_junctions[1]->offset, 0};
              guide_lane_result.emplace_back(laneid_tmp, guide_dist);
              navi_debug_info.guide_lanes_length.push_back({(int)target_sd_junctions[1]->offset, 0});
            }
          }
          navi_debug_info.guide_laneids = first_guide_laneids;
          guide_laneids_ref             = second_guide_laneids;
          //v2_junction_ids               = {target_sd_junctions[0]->junction_id, target_sd_junctions[1]->junction_id};
        }
        navi_debug_info.guide_laneids_refer = guide_laneids_ref;
      }
    }
  }

  std::vector<uint64_t> v2_junction_ids = {};
  for (auto &junction_tmp : target_sd_junctions) {
    if (IsV2IgnoredJunction(*junction_tmp)) {
      continue;
    }
    v2_junction_ids.push_back(junction_tmp->junction_id);
  }

  INTERNAL_PARAMS.navigation_info_data.set_v2_junction_ids(v2_junction_ids);
}

void SdNavigationHighway::JunctionNaviMatch(const JunctionInfo *junction_info_ptr, const std::vector<uint64_t> &candidate_lanes_in,
                                            int &match_type, std::vector<uint64_t> &guide_laneids) {

  guide_laneids.clear();
  SD_COARSE_MATCH_LOG << "candidate_lanes_in.size()" << candidate_lanes_in.size();
  if (candidate_lanes_in.empty()) {
    return;
  }
  if (junction_info_ptr->junction_type == JunctionType::RampInto) {

    guide_laneids = {candidate_lanes_in.back()};
    match_type    = 2;
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] RampInto: recommand the last lane  ";
    int ReferenceLaneNum = junction_info_ptr->main_road_lane_nums;
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] RampInto:  "
                        << "main_road_lane_nums: " << ReferenceLaneNum << " offset: " << junction_info_ptr->offset;

  } else if (junction_info_ptr->junction_type == JunctionType::RampSplitLeft) {
    int ReferenceLaneNum = junction_info_ptr->main_road_lane_nums;
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] RampSplitLeft:  "
                        << "main_road_lane_nums: " << ReferenceLaneNum << " offset: " << junction_info_ptr->offset;
    if (candidate_lanes_in.size() > 1) {
      if (candidate_lanes_in.size() >= ReferenceLaneNum && ReferenceLaneNum != 0 && junction_info_ptr->offset > 500) {
        for (unsigned int n = 0; n < ReferenceLaneNum; n++) {
          guide_laneids.emplace_back(candidate_lanes_in[n]);
        }
      } else {
        guide_laneids = {candidate_lanes_in.front()};
      }
    } else {
      guide_laneids = {candidate_lanes_in.front()};
    }
    match_type = 3;

  } else if (junction_info_ptr->junction_type == JunctionType::RampSplitMiddle) {

    if (candidate_lanes_in.size() > 2) {
      guide_laneids = {candidate_lanes_in[1]};
    } else {
      guide_laneids = {candidate_lanes_in.front()};
    }
    match_type = 4;

  } else if (junction_info_ptr->junction_type == JunctionType::RampSplitRight) {

    int ReferenceLaneNum = junction_info_ptr->main_road_lane_nums;
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] RampSplitRight:  "
                        << "main_road_lane_nums: " << ReferenceLaneNum << " offset: " << junction_info_ptr->offset;
    if (candidate_lanes_in.size() > 1) {
      if (candidate_lanes_in.size() >= ReferenceLaneNum && ReferenceLaneNum != 0 && junction_info_ptr->offset > 500) {
        for (unsigned int n = 0; n < ReferenceLaneNum; n++) {
          guide_laneids.emplace_back(candidate_lanes_in[n]);
        }
      } else {
        guide_laneids = {candidate_lanes_in.back()};
      }
    } else {
      guide_laneids = {candidate_lanes_in.back()};
    }
    match_type = 5;

  } else if (junction_info_ptr->junction_type == JunctionType::ApproachRampInto) {

    std::vector<std::vector<uint64_t>> non_navi_ids = topology_extractor_.ExtractNonNaviLanesFromJunction(junction_info_ptr);
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Result: [");
    for (size_t i = 0; i < non_navi_ids.size(); ++i) {
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("  [{}]", fmt::join(non_navi_ids[i], ", "));
      if (i < non_navi_ids.size() - 1) {
        SD_COARSE_MATCH_TYPE2_LOG << ",";
      }
    }
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("]");

    int ReferenceLaneNum = std::min(junction_info_ptr->target_road_lane_nums, junction_info_ptr->main_road_lane_nums);
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] ApproachRampInto:  "
                        << "main_road_lane_nums: " << ReferenceLaneNum << " offset: " << junction_info_ptr->offset;
    if (candidate_lanes_in.size() > 1) {
      if (candidate_lanes_in.size() >= ReferenceLaneNum && ReferenceLaneNum != 0 && junction_info_ptr->offset < 1000) {
        for (unsigned int n = 0; n < ReferenceLaneNum; n++) {
          guide_laneids.emplace_back(candidate_lanes_in[n]);
        }
      } else {
        guide_laneids = candidate_lanes_in;
      }
    } else {
      guide_laneids = {candidate_lanes_in.front()};
    }
    match_type = 6;

  } else if (junction_info_ptr->junction_type == JunctionType::ApproachRampMerge) {
    int ReferenceLaneNum = junction_info_ptr->main_road_lane_nums;
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] ApproachRampMerge:  "
                        << "main_road_lane_nums: " << ReferenceLaneNum << " offset: " << junction_info_ptr->offset;
    if (candidate_lanes_in.size() > 1) {
      if (candidate_lanes_in.size() >= ReferenceLaneNum && ReferenceLaneNum != 0 && junction_info_ptr->offset < 1000) {
        for (unsigned int n = 0; n < ReferenceLaneNum; n++) {
          guide_laneids.emplace_back(candidate_lanes_in[n]);
        }
      } else {
        guide_laneids = candidate_lanes_in;
      }
    } else {
      guide_laneids = {candidate_lanes_in.front()};
    }
    match_type = 7;

  } else if (junction_info_ptr->junction_type == JunctionType::RampMerge) {
    SD_COARSE_MATCH_LOG << "[JunctionNaviMatch] RampMerge:  "
                        << "main_road_lane_nums: " << junction_info_ptr->main_road_lane_nums << " offset: " << junction_info_ptr->offset
                        << " EgoInMainRoad: " << EgoInMainRoad();

    if (candidate_lanes_in.size() > 1 && junction_info_ptr->offset > -50 && junction_info_ptr->offset < 200) {
      for (unsigned int m = 0; m < candidate_lanes_in.size() - 1; m++) {
        guide_laneids.emplace_back(candidate_lanes_in[m]);
      }
    } else {
      guide_laneids = candidate_lanes_in;
    }
    if (junction_info_ptr->main_road_lane_nums != 0 && junction_info_ptr->main_road_lane_nums < guide_laneids.size() &&
        junction_info_ptr->offset > -50 && junction_info_ptr->offset < 400) { /* CNOAC2-100184 */
      for (unsigned int n = 0; n <= (guide_laneids.size() - junction_info_ptr->main_road_lane_nums); n++) {
        guide_laneids.pop_back();
      }
    }
    match_type = 8;
  } else if (junction_info_ptr->junction_type == JunctionType::MainRoadSplitLeft) {
    if (junction_info_ptr->main_road_navigation_lane_num > 1) {
      for (int n = 0; n < candidate_lanes_in.size() && n < junction_info_ptr->main_road_navigation_lane_num; n++) {
        guide_laneids.emplace_back(candidate_lanes_in[n]);
      }
    } else {
      if (!candidate_lanes_in.empty()) {
        guide_laneids = {candidate_lanes_in.front()};
      }
    }
    match_type = 9;
  } else if (junction_info_ptr->junction_type == JunctionType::MainRoadSplitRight) {
    if (junction_info_ptr->main_road_navigation_lane_num > 1) {
      for (int n = candidate_lanes_in.size() - 1;
           n >= 0 && (candidate_lanes_in.size() - n) <= junction_info_ptr->main_road_navigation_lane_num; n--) {
        guide_laneids.emplace_back(candidate_lanes_in[n]);
      }
      std::reverse(guide_laneids.begin(), guide_laneids.end());
    } else {
      if (!candidate_lanes_in.empty()) {
        guide_laneids = {candidate_lanes_in.back()};
      }
    }
    match_type = 10;
  } else if (junction_info_ptr->junction_type == JunctionType::MainRoadSplitMiddle) {
    guide_laneids = candidate_lanes_in;
    match_type    = 11;
  } else {
    guide_laneids = candidate_lanes_in;
  }
  SD_COARSE_MATCH_LOG << " [JunctionNaviMatch]guide_laneids: " << fmt::format(" [{}]  ", fmt::join(guide_laneids, ", "));
}

double SdNavigationHighway::CalculateSectionCost(int section_index, const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                 const std::vector<int>            &ego_section_indexs,
                                                 const std::vector<JunctionInfo *> &target_sd_junctions) {
  double cost                = 0.0;
  double ego_in_section_cost = 0.0;
  double lane_diff_cost      = 0.0;
  double history_lane_cost   = 0.0;
  double single_lane_cost    = 0.0;

  const double kEgoInSectionCost        = 0.0;
  const double kEgoNotInSectionCostHigh = 5.0;
  const double kEgoNotInSectionCostLow  = 1.0;
  const double kLaneDiffThreshold       = 2.0;
  const double kLaneDiffCostFactor      = 1.0;
  const double kHistoryLaneCostFactor   = 2.0;
  const double kSingleLaneCost          = 2.0;

  // 获取自车所在section的车道数
  uint64_t main_road_lane_num = 0;
  if (ego_section_id_ != 0) {
    auto ego_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(ego_section_id_);
    if (ego_section) {
      main_road_lane_num = ego_section->lane_ids.size();
    }
  }

  // 获取当前section的车道数
  uint64_t current_section_lane_num = bev_sections_in[section_index].size();

  // 如果section_index跟属于自车section，给予较低的代价
  bool is_ego_in_section = std::find(ego_section_indexs.begin(), ego_section_indexs.end(), section_index) != ego_section_indexs.end();
  if (target_sd_junctions.empty()) {
    // 无导航关注路口
    if (is_ego_in_section) {
      ego_in_section_cost += kEgoInSectionCost;
    } else {
      ego_in_section_cost += kEgoNotInSectionCostHigh;  // 自车不在该section中，增加代价
    }
  } else {
    // 有导航关注路口
    if (is_ego_in_section) {
      ego_in_section_cost += kEgoInSectionCost;
    } else {
      // 靠左侧行驶的路口，左侧section降低cost
      bool has_special_junction = false;
      for (const auto &junction : target_sd_junctions) {
        if (junction->junction_type == JunctionType::ApproachRampInto || junction->junction_type == JunctionType::ApproachRampMerge) {
          has_special_junction = true;
          break;
        }
      }

      if (has_special_junction && section_index == 0) {
        ego_in_section_cost += kEgoNotInSectionCostLow;  // 特殊路口类型且section_index为0时，代价较低
      } else {
        ego_in_section_cost += kEgoNotInSectionCostHigh;  // 其他情况，自车不在该section中，增加代价
      }
    }
  }

  // 考虑车道数差异
  if (!target_sd_junctions.empty()) {
    const JunctionInfo *junction_info         = target_sd_junctions.front();
    int                 target_road_lane_nums = junction_info->target_road_lane_nums;
    if (target_road_lane_nums > 0) {
      int lane_diff = std::abs(static_cast<int>(current_section_lane_num) - target_road_lane_nums);
      // 只有车道数差异大于2时才计入cost
      if (lane_diff > kLaneDiffThreshold) {
        lane_diff_cost += lane_diff * kLaneDiffCostFactor;  // 车道数差异越大，代价越高
      }
    }
  } else if (ego_section_id_ != 0) {
    // 无导航关注路口
    auto ego_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(ego_section_id_);
    if (ego_section) {
      uint64_t main_road_lane_num = ego_section->lane_ids.size();
      if (main_road_lane_num > 0) {
        int lane_diff = std::abs(static_cast<int>(current_section_lane_num) - static_cast<int>(main_road_lane_num));
        SD_COARSE_MATCH_LOG << " [CalculateSectionCost] current_section_lane_num " << current_section_lane_num
                            << " main_road_lane_num: " << main_road_lane_num << " lane_diff: " << lane_diff;
        // 只有车道数差异大于2时才计入cost
        if (lane_diff > kLaneDiffThreshold) {
          lane_diff_cost += lane_diff * kLaneDiffCostFactor;
        }
      }
    }
  }

  // 考虑是否存在历史推荐车道
  int history_lane_count = 0;
  for (const auto &lane_id : bev_sections_in[section_index]) {
    if (std::find(history_guide_laneids_.begin(), history_guide_laneids_.end(), lane_id) != history_guide_laneids_.end()) {
      history_lane_count++;
    }
  }
  history_lane_cost -= history_lane_count * kHistoryLaneCostFactor;

  // 考虑当前section车道数
  if (current_section_lane_num == 1) {
    single_lane_cost += kSingleLaneCost;  // 单车道section增加代价
  }

  // 总cost代价
  cost = ego_in_section_cost + lane_diff_cost + history_lane_cost + single_lane_cost;
  SD_COARSE_MATCH_LOG << " [CalculateSectionCost] section " << section_index << " ego_in_section_cost: " << ego_in_section_cost
                      << " lane_diff_cost: " << lane_diff_cost << " history_lane_cost: " << history_lane_cost
                      << " single_lane_cost: " << single_lane_cost << " total_cost: " << cost;
  return cost;
}

int SdNavigationHighway::SelectMainRoadSectionNoNavi(const std::vector<std::vector<uint64_t>> &bev_sections_in) {
  if (bev_sections_in.empty()) {
    return -1;
  }
  std::vector<int> ego_section_indexs = {};  /// 自车所在的section index
  for (unsigned int i = 0; i < bev_sections_in.size(); i++) {
    for (unsigned int j = 0; j < bev_sections_in[i].size(); j++) {
      if (bev_ego_lane_related_.find(bev_sections_in[i][j]) != bev_ego_lane_related_.end()) {
        SD_COARSE_MATCH_LOG << " [SelectMainRoadSectionNoNavi] ego_section_indexs: " << i;
        ego_section_indexs.emplace_back(i);
        break;
      }
    }
  }

  // 使用代价函数选择最优section
  int    selected_section_index = -1;
  double min_cost               = std::numeric_limits<double>::max();
  // 无导航场景
  std::vector<JunctionInfo *> empty_junctions = {};

  for (unsigned int i = 0; i < bev_sections_in.size(); i++) {
    double cost = CalculateSectionCost(i, bev_sections_in, ego_section_indexs, empty_junctions);
    SD_COARSE_MATCH_LOG << " [SelectMainRoadSectionNoNavi] section " << i << " [" << fmt::format("{}", fmt::join(bev_sections_in[i], ", "))
                        << "] cost: " << cost;

    if (cost < min_cost) {
      min_cost               = cost;
      selected_section_index = i;
    }
  }

  if (selected_section_index == -1) {
    selected_section_index = 0;  /// 默认选择最左侧的section
    SD_COARSE_MATCH_LOG << " [SelectMainRoadSectionNoNavi] default case, index: " << selected_section_index;
  } else {
    SD_COARSE_MATCH_LOG << " [SelectMainRoadSectionNoNavi] selected section index: " << selected_section_index
                        << " with cost: " << min_cost;
  }

  return selected_section_index;
};

void SdNavigationHighway::BevMapProcess() {
  auto bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
  if (!bev_map_ptr) {
    SD_BEV_PROCESS << fmt::format("bev_map is null, exiting BevMapProcess.");
    return;
  }
  const auto &all_bev_lane_infos = bev_map_ptr->lane_infos;
  auto       &all_bev_road_edges = bev_map_ptr->edges;
  auto       &diversion_zones    = bev_map_ptr->diversion_zone;
  SD_BEV_PROCESS << fmt::format("raw_bev_map: {}", bev_map_ptr->header.cycle_counter);
  SD_BEV_PROCESS << fmt::format("Before GetCandidateBevLanes: all_bev_lane_infos size: {}", all_bev_lane_infos.size());

  for (const auto &lane : all_bev_lane_infos) {
    SD_BEV_PROCESS << fmt::format("{}", lane);
  }

  GetCandidateBevLanes(all_bev_lane_infos, bev_candidate_lanes_);

  SD_BEV_PROCESS << fmt::format("After GetCandidateBevLanes: bev_candidate_lanes size: {}", bev_candidate_lanes_.size());
  for (const auto &lane : bev_candidate_lanes_) {
    SD_BEV_PROCESS << fmt::format("  Candidate Lane ID: {}", lane->id);
  }
  static constexpr int kMaxNextDepth = 10;
  for (auto &bev_lane_info : all_bev_lane_infos) {
    if (bev_lane_info.position == (int)BevLanePosition::LANE_LOC_EGO) {
      bev_ego_lane_related_.insert(bev_lane_info.id);
      bev_ego_lane_related_.insert(bev_lane_info.previous_lane_ids.begin(), bev_lane_info.previous_lane_ids.end());
      std::vector<uint64_t> next{bev_lane_info.id};
      int                   depth = 0;
      while (!next.empty() && depth < kMaxNextDepth) {
        std::vector<uint64_t> tmp;
        for (uint64_t cur_id : next) {
          const auto cur_lane_ptr = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(cur_id);
          if (cur_lane_ptr == nullptr) {
            continue;
          }
          for (uint64_t nxt_id : cur_lane_ptr->next_lane_ids) {
            if (bev_ego_lane_related_.insert(nxt_id).second) {
              tmp.push_back(nxt_id);
            }
          }
        }
        next.swap(tmp);
        ++depth;
      }
      break;
    }
  }

  SD_BEV_PROCESS << fmt::format("After GetEgoLaneRelated: bev_ego_lane_related IDs: [{}]", fmt::join(bev_ego_lane_related_, ", "));

  for (const auto &edge : all_bev_road_edges) {
    SD_BEV_PROCESS << fmt::format("{}", edge);
  }

  std::vector<BevLaneMarker> filtered_bev_road_edges;
  for (const auto &edge : all_bev_road_edges) {
    if (IsValidRoadEdge(edge)) {
      filtered_bev_road_edges.push_back(edge);
    }
  }

  for (const auto &edge : filtered_bev_road_edges) {
    SD_BEV_PROCESS << fmt::format("{}", edge);
  }
  SD_BEV_PROCESS << fmt::format("After filtering road edges: original size: {}, filtered size: {}", all_bev_road_edges.size(),
                                filtered_bev_road_edges.size());

  SortBevLaneInfoAndRoadEdges(bev_candidate_lanes_, filtered_bev_road_edges, bev_line_sorts_, bev_left_road_edge_indexs_,
                              bev_right_road_edge_indexs_);

  SD_BEV_PROCESS << fmt::format("After SortBevLaneInfoAndRoadEdges: bev_line_sorts: ");
  for (const auto &sort_tmp : bev_line_sorts_) {
    SD_BEV_PROCESS << fmt::format("  [{}, {}]", sort_tmp.id, (int)sort_tmp.is_road_boundary);
  }
  SD_BEV_PROCESS << fmt::format("bev_left_road_edge_indexs: [{}]", fmt::join(bev_left_road_edge_indexs_, ", "));
  SD_BEV_PROCESS << fmt::format("bev_right_road_edge_indexs: [{}]", fmt::join(bev_right_road_edge_indexs_, ", "));

  BevLanesFilterByRoadEdges(bev_line_sorts_, bev_left_road_edge_indexs_, bev_right_road_edge_indexs_, bev_candidate_lanes_);

  SD_BEV_PROCESS << fmt::format("After BevLanesFilterByRoadEdges: lanes size: {}", bev_candidate_lanes_.size());

  DivideBevSectionsByRoadEdges(bev_candidate_lanes_, bev_sections_);

  SD_BEV_PROCESS << fmt::format("After DivideBevSectionsByRoadEdges: sections size: {}", bev_sections_.size());
  for (size_t i = 0; i < bev_sections_.size(); ++i) {
    SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(bev_sections_[i], ", "));
  }

  FilterBevBlockedLanes(bev_candidate_lanes_, bev_sections_);

  SD_BEV_PROCESS << fmt::format("After FilterBevBlockedLanes: lanes size: {}", bev_candidate_lanes_.size());

  bev_egoRoadEdgeIndexPair_ = GetEgoLeftRightRoadEdges(bev_line_sorts_, bev_left_road_edge_indexs_, bev_right_road_edge_indexs_);

  SD_BEV_PROCESS << fmt::format("After GetEgoLeftRightRoadEdges: Left_EgoEdge_Index: {}, Right_EgoEdge_Index: {}",
                                bev_egoRoadEdgeIndexPair_.first, bev_egoRoadEdgeIndexPair_.second);

  std::vector<LineSort>      original_bev_line_sorts = bev_line_sorts_;
  std::vector<BevLaneMarker> filtered_virtual_edges;
#if 0
  if (!diversion_zones.empty()) {
    SD_BEV_PROCESS << fmt::format("Processing diversion zones (highway): {}", diversion_zones.size());
    for (const auto &zone : diversion_zones) {
      if (zone.geos && zone.geos->size() == 4) {
        SD_BEV_PROCESS << fmt::format(
            "Diversion Zone ID: {}, Type: {}, Points: [({:.3f}, {:.3f}), ({:.3f}, {:.3f}), ({:.3f}, {:.3f}), ({:.3f}, {:.3f})]", zone.id,
            static_cast<int>(zone.type), (*zone.geos)[0].x(), (*zone.geos)[0].y(), (*zone.geos)[1].x(), (*zone.geos)[1].y(),
            (*zone.geos)[2].x(), (*zone.geos)[2].y(), (*zone.geos)[3].x(), (*zone.geos)[3].y());
      } else {
        SD_BEV_PROCESS << fmt::format("Diversion Zone ID: {} has invalid geometry (size != 4)", zone.id);
      }
    }
    auto virtual_road_edges = bev_data_processor_.CreateVirtualRoadEdgesFromDiversionZones(diversion_zones);
    for (const auto &virtual_edge : virtual_road_edges) {
      SD_BEV_PROCESS << fmt::format("virtual_edge: {}", virtual_edge);
    }

    for (const auto &ve : virtual_road_edges) {
      if (!ve.geos || ve.geos->empty()) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} has invalid or empty geos, skipping.", ve.id);
        continue;
      }
      // 过滤距离自车大于 60 米或者自车后方的虚拟路岩
      if (ve.geos->front().x() > 60.0) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} filtered: closest x > 60m).", ve.id);
        continue;
      }
      if (ve.geos->back().x() < 0.0) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} filtered: end x < 0m).", ve.id);
        continue;
      }
      if (!bev_data_processor_.HasXOverlap(ve, bev_candidate_lanes_)) {
        continue;
      }
      filtered_virtual_edges.push_back(ve);
    }

    if (!filtered_virtual_edges.empty()) {
      //  把高速 LineSort -> 城区 LineSortCity
      std::vector<LineSortCity> final_line_sorts_city;
      final_line_sorts_city.reserve(bev_line_sorts_.size() + filtered_virtual_edges.size());
      for (const auto &s : bev_line_sorts_) {
        final_line_sorts_city.emplace_back(s.id, s.is_road_boundary ? LineSortType::RoadBoundary : LineSortType::LaneLine);
      }
      bev_data_processor_.SortLanesRelativeToVirtualEdges(bev_candidate_lanes_, filtered_virtual_edges, final_line_sorts_city);

      // 把含虚拟边的排序映射 城区LineSortCity -> 高速 LineSort
      std::vector<LineSort> line_sorts_with_virtual;
      line_sorts_with_virtual.reserve(final_line_sorts_city.size());
      for (const auto &c : final_line_sorts_city) {
        const bool is_boundary = (c.type == LineSortType::RoadBoundary || c.type == LineSortType::VirtualEdge);
        line_sorts_with_virtual.emplace_back(c.id, is_boundary);
      }

      bev_sections_.clear();
      bev_line_sorts_ = std::move(line_sorts_with_virtual);
      DivideBevSectionsByRoadEdges(bev_candidate_lanes_, bev_sections_);

      std::vector<std::string> updated_sorts;
      for (const auto &sort_tmp : bev_line_sorts_) {
        updated_sorts.push_back(fmt::format("[{}, {}]", sort_tmp.id, static_cast<int>(sort_tmp.is_road_boundary)));
      }
      SD_BEV_PROCESS << fmt::format("Updated bev_line_sorts: [{}]", fmt::join(updated_sorts, " "));

      SD_BEV_PROCESS << fmt::format("After diversion-zone re-division (HW): sections={}", bev_sections_.size());
      for (size_t i = 0; i < bev_sections_.size(); ++i) {
        SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(bev_sections_[i], ", "));
      }
    }
  }
#else

  if (!ld_merge_points.empty()) {
    SD_BEV_PROCESS << fmt::format("Processing map-based diversion points: {}", ld_merge_points.size());
    // for (auto point : ld_merge_points) {
    //   SD_BEV_PROCESS << "x:" << point.x() << " y:" << point.y();
    // }

    std::vector<BevLaneMarker> virtual_road_edges;
    if (ld_merge_points.size() >= 2) {
      std::vector<Eigen::Vector2f> sorted_points = ld_merge_points;
      std::sort(sorted_points.begin(), sorted_points.end(),
                [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });

      // 创建虚拟边界线
      BevLaneMarker virtual_edge;
      virtual_edge.id   = 999999;
      virtual_edge.geos = std::make_shared<std::vector<Eigen::Vector2f>>();

      virtual_edge.geos->insert(virtual_edge.geos->end(), sorted_points.begin(), sorted_points.end());
      virtual_road_edges.push_back(virtual_edge);
      SD_BEV_PROCESS << fmt::format("Created virtual edge from map points, start: ({:.3f}, {:.3f}), end: ({:.3f}, {:.3f})",
                                    sorted_points.front().x(), sorted_points.front().y(), sorted_points.back().x(),
                                    sorted_points.back().y());
    }

    for (const auto &virtual_edge : virtual_road_edges) {
      SD_BEV_PROCESS << fmt::format("map_virtual_edge: {}", virtual_edge);
    }

    for (const auto &ve : virtual_road_edges) {
      if (!ve.geos || ve.geos->empty()) {
        SD_BEV_PROCESS << fmt::format("Map Virtual Edge ID: {} has invalid or empty geos, skipping.", ve.id);
        continue;
      }
      // 过滤距离自车大于 60 米或者自车后方的虚拟路岩
      if (ve.geos->front().x() > 60.0) {
        SD_BEV_PROCESS << fmt::format("Map Virtual Edge ID: {} filtered: closest x > 60m).", ve.id);
        continue;
      }
      if (ve.geos->back().x() < 0.0) {
        SD_BEV_PROCESS << fmt::format("Map Virtual Edge ID: {} filtered: end x < 0m).", ve.id);
        continue;
      }
      if (!bev_data_processor_.HasXOverlap(ve, bev_candidate_lanes_)) {
        continue;
      }
      filtered_virtual_edges.push_back(ve);
    }

    if (!filtered_virtual_edges.empty()) {
      //  把高速 LineSort -> 城区 LineSortCity
      std::vector<LineSortCity> final_line_sorts_city;
      final_line_sorts_city.reserve(bev_line_sorts_.size() + filtered_virtual_edges.size());
      for (const auto &s : bev_line_sorts_) {
        final_line_sorts_city.emplace_back(s.id, s.is_road_boundary ? LineSortType::RoadBoundary : LineSortType::LaneLine);
      }
      bev_data_processor_.SortLanesRelativeToVirtualEdges(bev_candidate_lanes_, filtered_virtual_edges, final_line_sorts_city);

      // 把含虚拟边的排序映射 城区LineSortCity -> 高速 LineSort
      std::vector<LineSort> line_sorts_with_virtual;
      line_sorts_with_virtual.reserve(final_line_sorts_city.size());
      for (const auto &c : final_line_sorts_city) {
        const bool is_boundary = (c.type == LineSortType::RoadBoundary || c.type == LineSortType::VirtualEdge);
        line_sorts_with_virtual.emplace_back(c.id, is_boundary);
      }

      bev_sections_.clear();
      bev_line_sorts_ = std::move(line_sorts_with_virtual);
      DivideBevSectionsByRoadEdges(bev_candidate_lanes_, bev_sections_);

      std::vector<std::string> updated_sorts;
      for (const auto &sort_tmp : bev_line_sorts_) {
        updated_sorts.push_back(fmt::format("[{}, {}]", sort_tmp.id, static_cast<int>(sort_tmp.is_road_boundary)));
      }
      SD_BEV_PROCESS << fmt::format("Updated bev_line_sorts: [{}]", fmt::join(updated_sorts, " "));

      SD_BEV_PROCESS << fmt::format("After map-based diversion points re-division (HW): sections={}", bev_sections_.size());
      for (size_t i = 0; i < bev_sections_.size(); ++i) {
        SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(bev_sections_[i], ", "));
      }
    }
  }
#endif

  bev_line_sorts_ = std::move(original_bev_line_sorts);

  bev_ego_root_section_x0_.clear();
  auto merged              = MergeSectionLaneIdsLeft2Right(bev_sections_);
  bev_ego_root_section_x0_ = FindRootsAtX0AndSort(*bev_map_ptr, merged);
  SD_FINE_MATCH_LOG << fmt::format("merged:[{}]  roots:[{}]", fmt::join(merged, ", "), fmt::join(bev_ego_root_section_x0_, ", "));
}

std::vector<uint64_t> SdNavigationHighway::MergeSectionLaneIdsLeft2Right(
    const std::vector<std::vector<uint64_t>> &complete_section_info) const {
  std::vector<uint64_t>        merged;
  std::unordered_set<uint64_t> seen;
  for (const auto &sec : complete_section_info) {
    for (const auto id : sec) {
      if (seen.insert(id).second)
        merged.push_back(id);
    }
  }
  return merged;
}

std::vector<uint64_t> SdNavigationHighway::FindRootsAtX0AndSort(const BevMapInfo            &bev_map,
                                                                const std::vector<uint64_t> &merged_lane_ids) const {
  std::unordered_map<uint64_t, const BevLaneInfo *> lane_map;
  for (const auto &l : bev_map.lane_infos)
    lane_map[l.id] = &l;

  auto startx_lt0 = [](const BevLaneInfo *lane) -> bool {
    if (!lane || !lane->geos || lane->geos->empty())
      return false;
    double sx = std::min(lane->geos->front().x(), lane->geos->back().x());
    return sx < 0.0;
  };

  static constexpr int kMaxPrevDepth = 8;
  // 溯源到“无前驱”节点（包含多层前继也）
  std::unordered_set<uint64_t>       roots;
  std::function<void(uint64_t, int)> dfs_to_root = [&](uint64_t id, int depth) {
    if (depth >= kMaxPrevDepth) {
      roots.insert(id);
      return;
    }
    auto it = lane_map.find(id);
    if (it == lane_map.end())
      return;
    const auto *lane = it->second;

    // 早停条件：如果该 lane 的“起点 x”(startx) <
    // 0，则认为脚下已覆盖，不再回溯前继
    if (startx_lt0(lane)) {
      roots.insert(lane->id);
      return;
    }

    if (lane->previous_lane_ids.empty()) {
      roots.insert(lane->id);
      return;
    }

    for (uint64_t pre : lane->previous_lane_ids) {
      dfs_to_root(pre, depth + 1);
    }
  };
  for (auto id : merged_lane_ids)
    dfs_to_root(id, 0);

  std::vector<uint64_t> out(roots.begin(), roots.end());
  auto                  y_at_x0 = [&](uint64_t id) -> double {
    auto it = lane_map.find(id);
    if (it == lane_map.end())
      return std::numeric_limits<double>::infinity();
    const auto &geosPtr = it->second->geos;
    if (geosPtr && !geosPtr->empty()) {
      double y = InterpolateYAtX(*geosPtr, 0.0);
      if (!std::isfinite(y))
        y = geosPtr->front().y();
      return y;
    }
    return std::numeric_limits<double>::infinity();
  };
  std::sort(out.begin(), out.end(), [&](uint64_t a, uint64_t b) { return y_at_x0(a) > y_at_x0(b); });
  return out;
}

double SdNavigationHighway::InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x) noexcept {
  if (geos.empty())
    return std::numeric_limits<double>::quiet_NaN();
  if (x < geos.front().x() || x > geos.back().x())
    return std::numeric_limits<double>::quiet_NaN();

  for (size_t i = 0; i < geos.size() - 1; ++i) {
    if (geos[i].x() <= x && x <= geos[i + 1].x()) {
      double t = (x - geos[i].x()) / (geos[i + 1].x() - geos[i].x());
      return geos[i].y() + t * (geos[i + 1].y() - geos[i].y());
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void SdNavigationHighway::ManageJunctionEvent(int &current_target_junction_index) {
  current_target_junction_index = -1;
  /// 基于历史更新本周期的passed
  int last_passed_junction_index = -1;
  if (JunctionEvent_last_passed_junction_id_ != 0) {
    for (unsigned i = 0; i < sd_junction_infos_.size(); i++) {
      if (JunctionEvent_last_passed_junction_id_ == sd_junction_infos_[i].junction_id) {
        last_passed_junction_index = i;
        break;
      }
    }
  }
  if (last_passed_junction_index != -1) {
    for (unsigned i = 0; i < sd_junction_infos_.size(); i++) {
      if (i <= last_passed_junction_index) {
        sd_junction_infos_[i].has_passed_flag = true;
      } else {
        sd_junction_infos_[i].has_passed_flag = false;
      }
    }
  }
  if (last_passed_junction_index != -1 && last_passed_junction_index + 1 < sd_junction_infos_.size()) {
    current_target_junction_index = last_passed_junction_index + 1;
  } else if (last_passed_junction_index == -1 && !sd_junction_infos_.empty()) {
    current_target_junction_index = 0;
  } else {
    current_target_junction_index = -1;
  }

  /// 使用min_passed_dist更新本周期passed
  if (current_target_junction_index != -1) {
    for (unsigned i = current_target_junction_index; i < sd_junction_infos_.size(); i++) {
      if (sd_junction_infos_[i].offset < 0) {
        if (JunctionEvent_min_passed_dist_map_.find(sd_junction_infos_[i].junction_type) != JunctionEvent_min_passed_dist_map_.end()) {
          if (sd_junction_infos_[i].offset < JunctionEvent_min_passed_dist_map_[sd_junction_infos_[i].junction_type]) {
            sd_junction_infos_[i].has_passed_flag = true;
            if (current_target_junction_index + 1 < sd_junction_infos_.size()) {
              current_target_junction_index += 1;
            } else {
              current_target_junction_index = -1;
              break;
            }
          }
        }
      }
    }
  }
  /// 使用max_start_effect_dist更新本周期的effected
  if (current_target_junction_index != -1) {
    for (unsigned i = current_target_junction_index; i < sd_junction_infos_.size(); i++) {
      if (JunctionEvent_max_start_effect_dist_map_more_lane_num_.find(sd_junction_infos_[i].junction_type) !=
              JunctionEvent_max_start_effect_dist_map_more_lane_num_.end() &&
          sd_junction_infos_[i].main_road_lane_nums >= 4) {
        if (sd_junction_infos_[i].offset < JunctionEvent_max_start_effect_dist_map_more_lane_num_[sd_junction_infos_[i].junction_type]) {
          sd_junction_infos_[i].has_effected_flag = true;
        }
      } else if (JunctionEvent_max_start_effect_dist_map_less_lane_num_.find(sd_junction_infos_[i].junction_type) !=
                     JunctionEvent_max_start_effect_dist_map_less_lane_num_.end() &&
                 (sd_junction_infos_[i].main_road_lane_nums == 1 || sd_junction_infos_[i].main_road_lane_nums == 2)) {
        if (sd_junction_infos_[i].offset < JunctionEvent_max_start_effect_dist_map_less_lane_num_[sd_junction_infos_[i].junction_type]) {
          sd_junction_infos_[i].has_effected_flag = true;
        }
      } else if (JunctionEvent_max_start_effect_dist_map_.find(sd_junction_infos_[i].junction_type) !=
                 JunctionEvent_max_start_effect_dist_map_.end()) {
        if (sd_junction_infos_[i].offset < JunctionEvent_max_start_effect_dist_map_[sd_junction_infos_[i].junction_type]) {
          sd_junction_infos_[i].has_effected_flag = true;
        }
      }
    }
  }

  /// 针对首个非passed且effected的 RampMerge ApproachRampMerge,更新本周期passed
  if (current_target_junction_index != -1 && sd_junction_infos_[current_target_junction_index].has_effected_flag &&
      current_target_junction_index + 1 < sd_junction_infos_.size()) {
    auto &junction_info_effected = sd_junction_infos_[current_target_junction_index];
    if (junction_info_effected.offset < -50 && (junction_info_effected.junction_type == JunctionType::RampMerge ||
                                                junction_info_effected.junction_type == JunctionType::ApproachRampMerge)) {
      if ((sd_junction_infos_[current_target_junction_index + 1].junction_type == JunctionType::RampInto &&
           sd_junction_infos_[current_target_junction_index + 1].offset < 240) ||
          sd_junction_infos_[current_target_junction_index + 1].offset < 30) {
        junction_info_effected.has_passed_flag = true;
        current_target_junction_index += 1;
      }
    }
  }

  /// 使用感知特征变化 只针对首个非passed且effected 更新本周期的passed
  if (current_target_junction_index != -1 && sd_junction_infos_[current_target_junction_index].has_effected_flag) {
    auto &junction_info_effected = sd_junction_infos_[current_target_junction_index];

    auto check_static3roadedges_turnRight = [&]() -> bool {
      if (!junction_info_effected.has_passed_flag) {
        if (bev_left_road_edge_indexs_.size() == 2 && bev_right_road_edge_indexs_.size() == 1) {
          if (bev_egoRoadEdgeIndexPair_.first == bev_left_road_edge_indexs_.back() &&
              bev_egoRoadEdgeIndexPair_.second == bev_right_road_edge_indexs_.front()) {
            auto   L1Geos    = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(bev_line_sorts_[bev_left_road_edge_indexs_.front()].id)->geos;
            auto   L2Geos    = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id)->geos;
            double Gap_L1_L2 = LaneGeometry::GetDistanceBetweenLinesThin(*L1Geos, *L2Geos);
            if (L2Geos->front().x() - L1Geos->front().x() > 10 && Gap_L1_L2 > 5) {
              junction_info_effected.has_passed_flag   = true;
              junction_info_effected.has_effected_flag = false;
              if (current_target_junction_index + 1 < sd_junction_infos_.size()) {
                current_target_junction_index += 1;
              } else {
                current_target_junction_index = -1;
              }
              return true;
            }
          }
        }
      }
      return false;
    };

    auto check_static3roadedges_turnLeft = [&]() -> bool {
      if (!junction_info_effected.has_passed_flag) {
        if (bev_left_road_edge_indexs_.size() == 1 && bev_right_road_edge_indexs_.size() == 2) {
          if (bev_egoRoadEdgeIndexPair_.second == bev_right_road_edge_indexs_.front() &&
              bev_egoRoadEdgeIndexPair_.first == bev_left_road_edge_indexs_.front()) {
            auto   R1Geos    = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(bev_line_sorts_[bev_right_road_edge_indexs_.front()].id)->geos;
            auto   R2Geos    = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(bev_line_sorts_[bev_right_road_edge_indexs_.back()].id)->geos;
            double Gap_R1_R2 = LaneGeometry::GetDistanceBetweenLinesThin(*R1Geos, *R2Geos);
            if (R1Geos->front().x() - R2Geos->front().x() > 10 && Gap_R1_R2 > 5) {
              junction_info_effected.has_passed_flag   = true;
              junction_info_effected.has_effected_flag = false;
              if (current_target_junction_index + 1 < sd_junction_infos_.size()) {
                current_target_junction_index += 1;
              } else {
                current_target_junction_index = -1;
              }
              return true;
            }
          }
        }
      }
      return false;
    };

    auto check_change_egoleftroadedge = [&]() -> bool {
      if (!junction_info_effected.has_passed_flag) {
        if (JunctionEvent_last_junction_bev_feature_.junction_id == junction_info_effected.junction_id &&
            JunctionEvent_last_junction_bev_feature_.junction_type == junction_info_effected.junction_type) {
          if (bev_egoRoadEdgeIndexPair_.first != -1 && bev_egoRoadEdgeIndexPair_.second != -1) {
            auto ego_left_road_edge_id  = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id;
            auto ego_right_road_edge_id = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.second].id;

            if (ego_right_road_edge_id == JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id &&
                ego_left_road_edge_id != JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id &&
                JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id != 0) {
              auto roadEdge_tmp  = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_left_road_edge_id);
              int  nearest_index = roadEdge_tmp->indexed_geos.FindNearestPoint(0, 0);
              if (nearest_index != -1 && nearest_index < roadEdge_tmp->geos->size() &&
                  JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point.y() - roadEdge_tmp->geos->at(nearest_index).y() > 5) {
                junction_info_effected.has_passed_flag   = true;
                junction_info_effected.has_effected_flag = false;
                if (current_target_junction_index + 1 < sd_junction_infos_.size()) {
                  current_target_junction_index += 1;
                } else {
                  current_target_junction_index = -1;
                }
                return true;
              }
            }
          }
        }
      }
      return false;
    };

    auto check_change_egorightroadedge = [&]() -> bool {
      if (!junction_info_effected.has_passed_flag) {
        if (JunctionEvent_last_junction_bev_feature_.junction_id == junction_info_effected.junction_id &&
            JunctionEvent_last_junction_bev_feature_.junction_type == junction_info_effected.junction_type) {
          if (bev_egoRoadEdgeIndexPair_.first != -1 && bev_egoRoadEdgeIndexPair_.second != -1) {
            auto ego_left_road_edge_id  = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id;
            auto ego_right_road_edge_id = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.second].id;

            if (ego_right_road_edge_id != JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id &&
                JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id != 0 &&
                ego_left_road_edge_id == JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id) {
              auto roadEdge_tmp  = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(ego_right_road_edge_id);
              int  nearest_index = roadEdge_tmp->indexed_geos.FindNearestPoint(0, 0);
              if (nearest_index != -1 && nearest_index < roadEdge_tmp->geos->size() &&
                  roadEdge_tmp->geos->at(nearest_index).y() - JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_point.y() > 5) {
                junction_info_effected.has_passed_flag   = true;
                junction_info_effected.has_effected_flag = false;
                if (current_target_junction_index + 1 < sd_junction_infos_.size()) {
                  current_target_junction_index += 1;
                } else {
                  current_target_junction_index = -1;
                }
                return true;
              }
            }
          }
        }
      }
      return false;
    };

    if (junction_info_effected.junction_type == JunctionType::RampInto) {
      if (junction_info_effected.offset < 0) {
        /// 3 road boundary
        bool res1 = check_static3roadedges_turnRight();
        /// ego left roadboundary changed
        bool res2 = check_change_egoleftroadedge();
        // AINFO << "[SDNOA] BevFeatureEvent: " << (int)res1 << "," << (int)res2;
      }
    } else if (junction_info_effected.junction_type == JunctionType::RampSplitLeft) {
      if (junction_info_effected.offset < 10) {
        bool res1 = check_static3roadedges_turnLeft();
        bool res2 = check_change_egorightroadedge();
        // AINFO << "[SDNOA] BevFeatureEvent: " << (int)res1 << "," << (int)res2;
      }
    } else if (junction_info_effected.junction_type == JunctionType::RampSplitRight) {
      if (junction_info_effected.offset < 10) {
        bool res1 = check_static3roadedges_turnRight();
        bool res2 = check_change_egoleftroadedge();
        // AINFO << "[SDNOA] BevFeatureEvent: " << (int)res1 << "," << (int)res2;
      }
    }
  }

  /// 更新本周期JunctionBevFeature
  if (current_target_junction_index != -1 && sd_junction_infos_[current_target_junction_index].has_effected_flag) {
    auto &junction_info_effected = sd_junction_infos_[current_target_junction_index];

    if (bev_egoRoadEdgeIndexPair_.first != -1) {
      JunctionEvent_last_junction_bev_feature_.left_update_offset    = junction_info_effected.offset;
      JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.first].id;
      auto roadEdge_tmp  = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id);
      int  nearest_index = roadEdge_tmp->indexed_geos.FindNearestPoint(0, 0);
      if (nearest_index != -1 && nearest_index < roadEdge_tmp->geos->size()) {
        JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point = roadEdge_tmp->geos->at(nearest_index);
      } else {
        JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point = {0, 0};
      }
    } else {
      if (junction_info_effected.junction_id != JunctionEvent_last_junction_bev_feature_.junction_id) {
        JunctionEvent_last_junction_bev_feature_.left_update_offset       = 0;
        JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id    = 0;
        JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point = {0, 0};
      }
    }

    if (bev_egoRoadEdgeIndexPair_.second != -1) {
      JunctionEvent_last_junction_bev_feature_.right_update_offset    = junction_info_effected.offset;
      JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id = bev_line_sorts_[bev_egoRoadEdgeIndexPair_.second].id;
      JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_point =
          INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id)->geos->back();
    } else {
      if (junction_info_effected.junction_id != JunctionEvent_last_junction_bev_feature_.junction_id) {
        JunctionEvent_last_junction_bev_feature_.right_update_offset       = 0;
        JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id    = 0;
        JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_point = {0, 0};
      }
    }

    JunctionEvent_last_junction_bev_feature_.junction_id   = junction_info_effected.junction_id;
    JunctionEvent_last_junction_bev_feature_.junction_type = junction_info_effected.junction_type;
  } else {
    JunctionEvent_last_junction_bev_feature_.junction_id   = 0;
    JunctionEvent_last_junction_bev_feature_.junction_type = JunctionType::Unknow;

    JunctionEvent_last_junction_bev_feature_.left_update_offset        = 0;
    JunctionEvent_last_junction_bev_feature_.right_update_offset       = 0;
    JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id     = 0;
    JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point  = {0, 0};
    JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_id    = 0;
    JunctionEvent_last_junction_bev_feature_.ego_right_road_edge_point = {0, 0};
  }

  // AINFO << "[SDNOA FEATURE LEFT] id:" << JunctionEvent_last_junction_bev_feature_.junction_id << ",left_id:" << JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_id << ", x:" <<
  //   JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point.x() << ", y:" << JunctionEvent_last_junction_bev_feature_.ego_left_road_edge_point.y();
}

void SdNavigationHighway::GetCandidateBevLanes(const std::vector<BevLaneInfo> &lane_infos_in,
                                               std::vector<BevLaneInfo *>     &candidate_bev_lanes_out) {
  candidate_bev_lanes_out.clear();

  constexpr double kDefaultFrontXThresh = 40.0;
  constexpr double kRampFrontXThresh    = 80.0;
  constexpr double kNearMinOffset       = -50.0;
  constexpr double kNearMaxOffset       = 250.0;

  double front_x_threshold = kDefaultFrontXThresh;
  bool   ramp_nearby       = false;

  for (const auto &j : sd_junction_infos_) {
    if ((j.junction_type == JunctionType::RampInto || j.junction_type == JunctionType::ApproachRampInto) && j.offset > kNearMinOffset &&
        j.offset < kNearMaxOffset) {
      ramp_nearby = true;
      break;
    }
  }
  if (ramp_nearby) {
    SD_BEV_PROCESS << "Nearby ramp (-50~250m), "
                      "relax short-mess front.x threshold: 40 -> 80";
    front_x_threshold = kRampFrontXThresh;
  }

  for (auto &bev_lane_info_tmp : lane_infos_in) {
    if (bev_lane_info_tmp.geos && bev_lane_info_tmp.geos->size() > 1) {
      /// 是否是short mess lane
      bool is_bev_short_mess_lane = false;
      if (bev_lane_info_tmp.previous_lane_ids.empty() && bev_lane_info_tmp.next_lane_ids.empty()) {
        const double front_x = bev_lane_info_tmp.geos->front().x();
        const double dx      = std::fabs(bev_lane_info_tmp.geos->back().x() - bev_lane_info_tmp.geos->front().x());
        if (front_x > front_x_threshold && dx < 50.0) {
          is_bev_short_mess_lane = true;
        }
        if (front_x > front_x_threshold + 10 && dx < 80) {
          is_bev_short_mess_lane = true;
        }
      }
      /// 是否有后继
      bool has_suc_lanes_flag = false;
      for (auto &suc_laneid_tmp : bev_lane_info_tmp.next_lane_ids) {
        auto laneinfo_ptr = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(suc_laneid_tmp);
        if (laneinfo_ptr) {
          has_suc_lanes_flag = true;
          break;
        }
      }

      if (bev_lane_info_tmp.geos->back().x() < -5) {
        SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: end x < -5.", bev_lane_info_tmp.id);
        continue;
      }
      if (!has_suc_lanes_flag && !is_bev_short_mess_lane) {
        candidate_bev_lanes_out.push_back(const_cast<BevLaneInfo *>(&bev_lane_info_tmp));
      }
    }
  }
}

// 检查是否为有效道路边缘
bool SdNavigationHighway::IsValidRoadEdge(const BevLaneMarker &edge) {
  if (!edge.geos || edge.geos->empty()) {
    return false;
  }

  if (edge.type == static_cast<uint32_t>(BoundaryType::FENCE_BOUNDARY)) {
    SD_BEV_PROCESS << fmt::format("[Edge {}] FENCE_BOUNDARY (cone-like), skip as road-edge for section division.", edge.id);
    return false;
  }

  double length = fabs(edge.geos->back().x() - edge.geos->front().x());
  //过滤部分激光雷达路岩
  if (edge.id > 100 && (length < 50.0 || edge.geos->front().x() > 50.0 || edge.geos->back().x() < 20)) {
    return false;
  }

  // Filter road edges starting beyond 80m from ego vehicle
  if (edge.geos->front().x() > 80.0) {
    SD_BEV_PROCESS << fmt::format("Edge ID: {} filtered: start x >80m.", edge.id);
    return false;
  }

  //数据发现track提供的路岩存在Edge ID: 42, PointSize: 7255, StartPointX: -707.0023, StartPointY: -148.6161, EndPointX: 21.8407, EndPointY: -6.7597
  if (edge.geos->front().x() < -150.0) {
    SD_BEV_PROCESS << fmt::format("Edge ID: {} filtered: start x < -150.", edge.id);
    return false;
  }

  // 检查是否为横向道路边缘
  if (edge.geos->size() >= 2) {
    double delta_x = fabs(edge.geos->back().x() - edge.geos->front().x());
    double delta_y = fabs(edge.geos->back().y() - edge.geos->front().y());
    if (delta_x < delta_y) {
      return false;
    }
  }

  if (length < 15.0) {  // 短线过滤
    SD_BEV_PROCESS << fmt::format("Edge ID: {} filtered: length < 15 m.", edge.id);
    return false;
  }

  if (edge.geos->back().x() < 0) {
    SD_BEV_PROCESS << fmt::format("Edge ID: {} filtered: end x < 0.", edge.id);
    return false;
  }

  return true;
}

bool SdNavigationHighway::HasCommonAncestor(const LineSort &l1, const LineSort &l2) {
  if (l1.is_road_boundary || l2.is_road_boundary) {
    return false;
  }

  std::unordered_set<uint64_t> ancestors_l1, ancestors_l2;

  auto getAncestors = [](const BevLaneInfo *lane) -> std::unordered_set<uint64_t> {
    std::unordered_set<uint64_t> ancestors;
    std::vector<uint64_t>        to_visit = lane->previous_lane_ids;
    while (!to_visit.empty()) {
      uint64_t current_id = to_visit.back();
      to_visit.pop_back();
      if (ancestors.find(current_id) != ancestors.end()) {
        continue;
      }
      ancestors.insert(current_id);
      auto current_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(current_id);
      if (current_lane) {
        to_visit.insert(to_visit.end(), current_lane->previous_lane_ids.begin(), current_lane->previous_lane_ids.end());
      }
    }
    return ancestors;
  };

  auto lane1 = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(l1.id);
  auto lane2 = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(l2.id);
  if (!lane1 || !lane2) {
    SD_BEV_PROCESS << fmt::format("Error: Lane {} or Lane {} is invalid.", l1.id, l2.id);
    return false;
  }
  ancestors_l1 = getAncestors(lane1);
  ancestors_l2 = getAncestors(lane2);

  for (const auto &ancestor : ancestors_l1) {
    if (ancestors_l2.find(ancestor) != ancestors_l2.end()) {
      return true;
    }
  }

  return false;
}

void SdNavigationHighway::SortBevLaneInfoAndRoadEdges(const std::vector<BevLaneInfo *> &bev_lane_infos_in,
                                                      const std::vector<BevLaneMarker> &road_edges_in, std::vector<LineSort> &line_sorts,
                                                      std::vector<int> &left_road_edge_indexs, std::vector<int> &right_road_edge_indexs) {
  if (bev_lane_infos_in.size() >= 16) {
    SD_BEV_PROCESS << fmt::format("bev_lane_infos_in size: {} > 16.", bev_lane_infos_in.size());
    AWARN << "Input lane size too much, exit from SortBevLaneInfoAndRoadEdges";
    return;
  }
  line_sorts.clear();
  left_road_edge_indexs.clear();
  right_road_edge_indexs.clear();
  std::unordered_map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> road_edge_geo_map;
  std::unordered_map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> laneinfo_geo_map;
  for (auto bev_lane_info_tmp : bev_lane_infos_in) {
    if (bev_lane_info_tmp->geos && !bev_lane_info_tmp->geos->empty()) {
      std::shared_ptr<std::vector<Eigen::Vector2f>> geos_tmp = bev_lane_info_tmp->geos;
      std::vector<double>                           geo_x_vec, geo_y_vec;
      geo_x_vec.clear();
      geo_y_vec.clear();
      bev_data_processor_.Clear();
      auto cur_full_geos = bev_data_processor_.GetFullLaneGeometry(bev_lane_info_tmp);
      for (auto &pt : *cur_full_geos) {
        geo_x_vec.push_back(pt.x());
        geo_y_vec.push_back(pt.y());
      }
      LaneGeometry::PolynomialFitting fitting = (geo_x_vec.size() < 10) ? LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1)
                                                                        : LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3);
      laneinfo_geo_map[bev_lane_info_tmp->id] = {cur_full_geos, fitting};
      line_sorts.emplace_back(bev_lane_info_tmp->id, false);
      SD_BEV_PROCESS << fmt::format("Lane ID: {}, y_at_0: {:.3f}, x-range: [{:.3f}, {:.3f}]", bev_lane_info_tmp->id, fitting.GetValue(0),
                                    geo_x_vec.front(), geo_x_vec.back());
    }
  };

  for (auto &road_edge_tmp : road_edges_in) {
    if (road_edge_tmp.geos && road_edge_tmp.geos->size() > 1) {
      if (IsValidRoadEdge(road_edge_tmp)) {
        std::vector<double> geo_x_vec, geo_y_vec;
        for (auto &pt : *road_edge_tmp.geos) {
          geo_x_vec.push_back(pt.x());
          geo_y_vec.push_back(pt.y());
        }
        if (geo_x_vec.size() < 20) {
          road_edge_geo_map.insert({road_edge_tmp.id, {road_edge_tmp.geos, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1)}});
        } else {
          road_edge_geo_map.insert({road_edge_tmp.id, {road_edge_tmp.geos, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
        }
        line_sorts.emplace_back(road_edge_tmp.id, true);
      }
    }
  }

  if (road_edge_geo_map.empty() && laneinfo_geo_map.empty()) {
    // AINFO << "[SDNOA] road_edge_geo_map size:" << road_edge_geo_map.size() << ", laneinfo_geo_map size:" << laneinfo_geo_map.size();
    return;
  }

  ///////////////////////车道和道路边界整体排序//////////////////
  std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
    LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
    LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
    Vec2fVector                      l1_geos               = nullptr;
    Vec2fVector                      l2_geos               = nullptr;

    if (l1.is_road_boundary) {
      polynomial_fitting_l1 = &road_edge_geo_map[l1.id].second;
      l1_geos               = road_edge_geo_map[l1.id].first;
    } else {
      polynomial_fitting_l1 = &laneinfo_geo_map[l1.id].second;
      l1_geos               = laneinfo_geo_map[l1.id].first;
    }

    if (l2.is_road_boundary) {
      polynomial_fitting_l2 = &road_edge_geo_map[l2.id].second;
      l2_geos               = road_edge_geo_map[l2.id].first;
    } else {
      polynomial_fitting_l2 = &laneinfo_geo_map[l2.id].second;
      l2_geos               = laneinfo_geo_map[l2.id].first;
    }

    if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
      bool has_common_ancestor = HasCommonAncestor(l1, l2);
      if (has_common_ancestor) {
        SD_BEV_PROCESS << fmt::format("id1={} and id2={} has same ancestor", l1.id, l2.id);
        auto lane1 = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(l1.id);
        auto lane2 = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(l2.id);
        if (!lane1 || !lane2) {
          SD_BEV_PROCESS << fmt::format("Error: Lane {} or Lane {} is invalid.", l1.id, l2.id);
          return false;
        }
        std::shared_ptr<std::vector<Eigen::Vector2f>> raw_geos_l1 = lane1->geos;
        std::shared_ptr<std::vector<Eigen::Vector2f>> raw_geos_l2 = lane2->geos;

        //前继相同，split的两条子路，用原始形点来排序；1.有overlap用原始形点；2.没有overlap用线段的中点的y方向数值判断左右
        double start_x = std::max(raw_geos_l1->front().x(), raw_geos_l2->front().x());
        double end_x   = std::min(raw_geos_l1->back().x(), raw_geos_l2->back().x());

        if (end_x > start_x) {
          bool res = LaneGeometry::JudgeIsLeft(*raw_geos_l1, *raw_geos_l2);
          SD_BEV_PROCESS << fmt::format("JudgeIsLeft (has same ancestor and overlap): id1={} vs id2={} -> {} ", l1.id, l2.id,
                                        res ? "left" : "right");
          return res;
        } else {
          Eigen::Vector2f l1_center_pt(0, 0), l2_center_pt(0, 0);
          int             l1_pts_size = raw_geos_l1->size();
          int             l2_pts_size = raw_geos_l2->size();
          l1_center_pt                = raw_geos_l1->at(l1_pts_size / 2);
          l2_center_pt                = raw_geos_l2->at(l2_pts_size / 2);
          bool res                    = l1_center_pt.y() > l2_center_pt.y();
          SD_BEV_PROCESS << fmt::format(
              "JudgeIsLeft (has same ancestor but has no overlap): id1={} centerPt.y={} vs id2={} centterPt.y={} -> {} ", l1.id,
              l1_center_pt.y(), l2.id, l2_center_pt.y(), res ? "left" : "right");
          return res;
        }
      } else {
        bool res = LaneGeometry::JudgeIsLeft(*l1_geos, *l2_geos, *polynomial_fitting_l1, *polynomial_fitting_l2);
        SD_BEV_PROCESS << fmt::format("JudgeIsLeft: id1={} vs id2={} -> {}", l1.id, l2.id, res ? "left" : "right");
        return res;
      }
    } else {
      return false;
    }
  });

  size_t ego_lane_index = line_sorts.size();
  for (unsigned i = 0; i < line_sorts.size(); i++) {
    if (!line_sorts[i].is_road_boundary && bev_ego_lane_related_.count(line_sorts[i].id)) {
      ego_lane_index = i;
      break;
    }
  }
  // 根据 ego lane 位置分类道路边缘
  if (ego_lane_index != line_sorts.size()) {
    for (size_t i = 0; i < line_sorts.size(); ++i) {
      if (line_sorts[i].is_road_boundary) {
        if (i < ego_lane_index) {
          left_road_edge_indexs.push_back(i);
        } else if (i > ego_lane_index) {
          right_road_edge_indexs.push_back(i);
        }
        // SD_BEV_PROCESS << fmt::format("line_sorts[{}].id: {} classified as {}", i, line_sorts[i].id,
        //                               (i < ego_lane_index ? "left" : "right"));
      }
    }
  }
}

void SdNavigationHighway::BevLanesFilterByRoadEdges(const std::vector<LineSort> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                                    const std::vector<int>     &right_road_edge_indexs,
                                                    std::vector<BevLaneInfo *> &candidateBevLanes) {
  std::vector<uint32_t> left_delete_laneids      = {};
  std::vector<uint32_t> right_delete_laneids     = {};
  int                   left_divider_edge_index  = -1;
  int                   right_divider_edge_index = -1;
  /////////////////自车左侧车道过滤逻辑////////////////////////
  {
    std::set<uint64_t> current_intersect_laneids;
    current_intersect_laneids.clear();
    const double l_longitude_dist_threshold_front = 10.0f;
    const double l_longitude_dist_threshold_back  = 50.0f;
    const double l_min_road_edge_length           = 5.0f;
    if (!left_road_edge_indexs.empty()) {
      for (int n = left_road_edge_indexs.size() - 1; n >= 0; n--) {
        auto road_edge_ptr = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(line_sorts[left_road_edge_indexs[n]].id);
        if (!road_edge_ptr) {
          continue;
        }
        auto   road_edge_geo_ptr     = road_edge_ptr->geos;
        double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
        double start_longitude_range = road_edge_geo_ptr->front().x() - l_longitude_dist_threshold_front;
        double end_longitude_range   = road_edge_geo_ptr->back().x() + l_longitude_dist_threshold_back;
        if (road_edge_length > l_min_road_edge_length) {
          if (start_longitude_range < 0 && end_longitude_range > 0) {
            left_divider_edge_index = left_road_edge_indexs[n];
            break;
          }
        }
      }
    }
    if (left_divider_edge_index != -1) {
      ////寻找left_divider右侧第一个不是roadEdge的lane
      uint32_t left_divider_first_lane_id = 0;
      for (int p = left_divider_edge_index + 1; p < line_sorts.size(); p++) {
        if (!line_sorts[p].is_road_boundary) {
          left_divider_first_lane_id = line_sorts[p].id;
          break;
        }
      }

      ////删除left_divider左侧的无拓扑关系的所有lane
      for (unsigned k = 0; k < left_divider_edge_index; k++) {
        if (!line_sorts[k].is_road_boundary) {
          bool delete_flag  = true;
          auto lane_id_tmp  = line_sorts[k].id;
          auto lane_ptr_tmp = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id_tmp);
          if (left_divider_first_lane_id != 0 && lane_ptr_tmp) {
            for (auto &pre_laneid_tmp : lane_ptr_tmp->previous_lane_ids) {
              if (pre_laneid_tmp == left_divider_first_lane_id) {
                delete_flag = false;
                break;
              }
            }
            for (auto &next_laneid_tmp : lane_ptr_tmp->next_lane_ids) {
              if (next_laneid_tmp == left_divider_first_lane_id) {
                delete_flag = false;
                break;
              }
            }
          }
          if (delete_flag) {
            left_delete_laneids.emplace_back(lane_id_tmp);
          }
        }
      }

      ////判断left_divider_first_lane是否和left_divider交叉 交叉且不经过00点则删除
      // if (left_divider_first_lane_id != 0) {
      //     Vec2fVector left_divider_first_lane_geos = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(left_divider_first_lane_id)->geos;
      //     Vec2fVector left_divider_geos =  INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(line_sorts[left_divider_edge_index].id)->geos;
      //     if (left_divider_first_lane_geos && left_divider_geos && !left_divider_first_lane_geos->empty()) {
      //         if (!(left_divider_first_lane_geos->front().x() < 0 && left_divider_geos->back().x() > 0)) {
      //             std::vector<int> intersection_indexs = {};
      //             LaneGeometry::IsIntersectForTwoLines(
      //                 *left_divider_geos, *left_divider_first_lane_geos, intersection_indexs);
      //             if (!intersection_indexs.empty()) {
      //                 current_intersect_laneids.insert(left_divider_first_lane_id);
      //                 if(last_intersect_bev_laneids_.find(left_divider_first_lane_id) != last_intersect_bev_laneids_.end()) {
      //                     if(last_intersect_bev_laneids_[left_divider_first_lane_id] >= 5) {
      //                         left_delete_laneids.emplace_back(left_divider_first_lane_id);
      //                     }
      //                 }
      //             }
      //         }
      //     }
      // }
    }

    // if(current_intersect_laneids.empty()) {
    //     last_intersect_bev_laneids_.clear();
    // }else {
    //    for(auto it = last_intersect_bev_laneids_.begin(); it != last_intersect_bev_laneids_.end();) {
    //        if(current_intersect_laneids.find(it->first) == current_intersect_laneids.end()) {
    //            it = last_intersect_bev_laneids_.erase(it);
    //        }else {
    //            ++it;
    //        }
    //    }
    //
    //    for(auto& current_id: current_intersect_laneids) {
    //        if(last_intersect_bev_laneids_.find(current_id) == last_intersect_bev_laneids_.end()) {
    //            last_intersect_bev_laneids_.insert({current_id, 1});
    //        }else {
    //            last_intersect_bev_laneids_[current_id] += 1;
    //        }
    //    }
    // }
  }

  /////////////////自车右侧车道过滤逻辑////////////////////////
  {
    const double r_longitude_dist_threshold = 10.0f;
    const double r_min_road_edge_length     = 50.0f;
    if (!right_road_edge_indexs.empty()) {
      for (int n = 0; n < right_road_edge_indexs.size(); n++) {
        auto   road_edge_id          = line_sorts[right_road_edge_indexs[n]].id;
        auto   road_edge_geo_ptr     = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(road_edge_id)->geos;
        double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
        double start_longitude_range = road_edge_geo_ptr->front().x() - r_longitude_dist_threshold;
        double end_longitude_range   = road_edge_geo_ptr->back().x() + r_longitude_dist_threshold;
        if (road_edge_length > r_min_road_edge_length) {
          if (start_longitude_range < 0 && end_longitude_range > 0) {
            right_divider_edge_index = right_road_edge_indexs[n];
            break;
          }
        }
      }
    }
    if (right_divider_edge_index != -1) {
      ////删除right_divider右侧的无拓扑、不经过00点的车道
      for (unsigned k = right_divider_edge_index + 1; k < line_sorts.size(); k++) {
        if (!line_sorts[k].is_road_boundary) {
          auto lane_id_tmp  = line_sorts[k].id;
          auto lane_ptr_tmp = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id_tmp);
          if (lane_ptr_tmp && lane_ptr_tmp->geos && !lane_ptr_tmp->geos->empty() && lane_ptr_tmp->geos->front().x() > 10) {
            if (lane_ptr_tmp->previous_lane_ids.empty() && lane_ptr_tmp->next_lane_ids.empty()) {
              right_delete_laneids.emplace_back(lane_id_tmp);
            }
          }
        }
      }
    }
  }

  std::ostringstream os1;
  for (auto &lane_id_tmp : left_delete_laneids) {
    os1 << lane_id_tmp << ",";
  }
  // AINFO << "[POSTMATCH BOUNDARY FILTER LEFT] " << os1.str();
  std::ostringstream os2;
  for (auto &lane_id_tmp : right_delete_laneids) {
    os2 << lane_id_tmp << ",";
  }
  // AINFO << "[POSTMATCH BOUNDARY FILTER RIGHT]" << os2.str();
  std::set<uint32_t> delete_laneids_set;
  delete_laneids_set.insert(left_delete_laneids.begin(), left_delete_laneids.end());
  delete_laneids_set.insert(right_delete_laneids.begin(), right_delete_laneids.end());

  //////////////////Update Candidate////////////////////
  for (auto it = candidateBevLanes.begin(); it != candidateBevLanes.end();) {
    if (delete_laneids_set.find((*it)->id) != delete_laneids_set.end()) {
      it = candidateBevLanes.erase(it);
    } else {
      ++it;
    }
  }
}

/// 获取EgoRoadEdges的index
std::pair<int, int> SdNavigationHighway::GetEgoLeftRightRoadEdges(const std::vector<LineSort> &line_sorts,
                                                                  const std::vector<int>      &left_road_edge_indexs,
                                                                  const std::vector<int>      &right_road_edge_indexs) {
  int Left_EgoEdge_Index  = -1;
  int Right_EgoEdge_Index = -1;
  if (left_road_edge_indexs.empty() && right_road_edge_indexs.empty()) {
    return {Left_EgoEdge_Index, Right_EgoEdge_Index};
  }

  // Left
  const double l_longitude_dist_threshold = 13.0f;
  const double l_min_road_edge_length     = 5.0f;
  if (!left_road_edge_indexs.empty()) {
    for (int n = left_road_edge_indexs.size() - 1; n >= 0; n--) {
      auto road_edge_ptr = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(line_sorts[left_road_edge_indexs[n]].id);
      if (!road_edge_ptr) {
        continue;
      }
      auto   road_edge_geo_ptr     = road_edge_ptr->geos;
      double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
      double start_longitude_range = road_edge_geo_ptr->front().x() - l_longitude_dist_threshold;
      double end_longitude_range   = road_edge_geo_ptr->back().x() + l_longitude_dist_threshold;
      if (road_edge_length > l_min_road_edge_length) {
        if (start_longitude_range < 0 && end_longitude_range > 0) {
          Left_EgoEdge_Index = left_road_edge_indexs[n];
          break;
        }
      }
    }
  }

  // Right
  const double r_longitude_dist_threshold = 13.0f;
  const double r_min_road_edge_length     = 5.0f;
  if (!right_road_edge_indexs.empty()) {
    for (int n = 0; n < right_road_edge_indexs.size(); n++) {
      auto   road_edge_id          = line_sorts[right_road_edge_indexs[n]].id;
      auto   road_edge_geo_ptr     = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(road_edge_id)->geos;
      double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
      double start_longitude_range = road_edge_geo_ptr->front().x() - r_longitude_dist_threshold;
      double end_longitude_range   = road_edge_geo_ptr->back().x() + r_longitude_dist_threshold;
      if (road_edge_length > r_min_road_edge_length) {
        if (start_longitude_range < 0 && end_longitude_range > 0) {
          Right_EgoEdge_Index = right_road_edge_indexs[n];
          break;
        }
      }
    }
  }
  return {Left_EgoEdge_Index, Right_EgoEdge_Index};
}

void SdNavigationHighway::DivideBevSectionsByRoadEdges(const std::vector<BevLaneInfo *>   &bev_candidate_lanes_in,
                                                       std::vector<std::vector<uint64_t>> &sections) {
  std::set<uint64_t> bev_candidate_laneids_set = {};
  for (auto &bev_lane_info : bev_candidate_lanes_in) {
    bev_candidate_laneids_set.insert(bev_lane_info->id);
  }

  std::vector<LineSort> bev_line_sorts_new = {};
  for (auto &line_sort_tmp : bev_line_sorts_) {
    if (!line_sort_tmp.is_road_boundary) {
      if (bev_candidate_laneids_set.find(line_sort_tmp.id) != bev_candidate_laneids_set.end()) {
        bev_line_sorts_new.push_back(line_sort_tmp);
      }
    } else {
      bev_line_sorts_new.push_back(line_sort_tmp);
    }
  }
  for (unsigned m = 0; m < bev_line_sorts_new.size(); m++) {
    if (bev_line_sorts_new[m].is_road_boundary) {
      sections.emplace_back(std::vector<uint64_t>());
    } else {
      if (sections.empty()) {
        sections.emplace_back(std::vector<uint64_t>());
      }
      sections.back().push_back(bev_line_sorts_new[m].id);
    }
  }

  for (auto it = sections.begin(); it != sections.end();) {
    if (it->empty()) {
      it = sections.erase(it);
    } else {
      ++it;
    }
  }
}

/// bev blocked车道的过滤
void SdNavigationHighway::FilterBevBlockedLanes(std::vector<BevLaneInfo *>         &bev_candidate_lanes_in,
                                                std::vector<std::vector<uint64_t>> &sections_in) {
  std::vector<std::vector<uint64_t>> sections_new = {};
  for (auto &section_tmp : sections_in) {
    /// section中多于1条lane才删除blocked车道
    std::vector<uint64_t> laneids_new = {};
    for (auto &laneid_tmp : section_tmp) {
      auto lane_tmp = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(laneid_tmp);
      // 高速的应急车道，即使是blocked也不过滤
      bool is_emergency_lane = (lane_tmp->lane_type == BevLaneType::LANE_TYPE_EMERGENCY);
      if (lane_tmp->is_blocked && !is_emergency_lane) {
        for (auto it = bev_candidate_lanes_in.begin(); it != bev_candidate_lanes_in.end();) {
          if ((*it)->id == lane_tmp->id) {
            it = bev_candidate_lanes_in.erase(it);
          } else {
            ++it;
          }
        }
      } else {
        laneids_new.push_back(laneid_tmp);
      }
    }
    if (!laneids_new.empty()) {
      sections_new.push_back(laneids_new);
    } else {
      sections_new.push_back(section_tmp);
    }
  }
  sections_in = sections_new;
}

void SdNavigationHighway::AssignBevLaneTypes() {
  auto sd_map_info = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_map_info) {
    AWARN << "[SDNOA] Failed: sd_map_info is null!";
    return;
  }

  auto bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
  if (!bev_map_ptr || bev_candidate_lanes_.empty()) {
    AWARN << "[SDNOA] Failed: bev_map_ptr or bev_candidate_lanes_ is empty!";
    return;
  }

  // Step 1: 根据 bev_line_sorts_ 构建按左右顺序排列的 BEV 车道线列表
  std::vector<BevLaneInfo *> sorted_bev_lanes;
  for (const auto &line_sort : bev_line_sorts_) {
    if (!line_sort.is_road_boundary) {  // 过滤掉边界线，只保留车道线
      for (const auto &bev_lane : bev_candidate_lanes_) {
        if (bev_lane->id == line_sort.id) {
          sorted_bev_lanes.push_back(bev_lane);
          break;
        }
      }
    }
  }
  size_t bev_lane_count = sorted_bev_lanes.size();
  if (bev_lane_count == 0) {
    AWARN << "[SDNOA] No valid BEV lanes found in bev_line_sorts_!";
    return;
  }

  // 打印 sorted_bev_lanes 的信息
  AINFO << "[SDNOA] Sorted BEV lanes count: " << bev_lane_count;
  for (const auto &lane : sorted_bev_lanes) {
    AINFO << "  [SDNOA] Lane ID: " << lane->id << ", Length: " << lane->length;
  }

  // Step 2: 找到覆盖范围最远的 BEV 车道线
  const BevLaneInfo *max_range_lane = nullptr;
  double             max_range      = std::numeric_limits<double>::lowest();
  double             x_min = 0.0, x_max = 0.0;  // 定义变量以便后续打印
  for (auto &bev_lane : sorted_bev_lanes) {
    if (bev_lane->geos && !bev_lane->geos->empty()) {
      x_min        = bev_lane->geos->front().x();
      x_max        = bev_lane->geos->back().x();
      double range = x_max - std::max(0.0, x_min);  // 计算覆盖范围
      if (range > max_range) {
        max_range      = range;
        max_range_lane = bev_lane;
      }
    }
  }
  if (!max_range_lane) {
    AWARN << "[SDNOA] No valid BEV lanes with geos!";
    return;
  }

  // 打印 max_range_lane 的信息
  AINFO << "[SDNOA] Max range lane ID: " << max_range_lane->id << ", x_min: " << x_min << ", x_max: " << x_max << ", range: " << max_range;

  // Step 3: 为该车道线匹配覆盖长度最大的 LaneGroup
  uint64_t ego_section_id = sd_map_info->navi_start.section_id;
  double   ego_s_offset   = sd_map_info->navi_start.s_offset;
  double   bev_start_s    = ego_s_offset + std::max(0.0, x_min);
  double   bev_end_s      = ego_s_offset + x_max;

  // 打印 ego_section_id 和 ego_s_offset 以及 BEV 车道线的 s 范围
  AINFO << "[SDNOA] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;
  AINFO << "[SDNOA] BEV lane s range: [" << bev_start_s << ", " << bev_end_s << "]";

  // 计算与每个 LaneGroup 的重叠长度
  std::unordered_map<uint64_t, double> lanegroup_coverage;
  double                               current_s  = 0.0;
  bool                                 processing = false;  // 是否开始处理自车所在 Section
  for (const auto &section : sd_map_info->mpp_sections) {
    if (section.id == ego_section_id) {
      processing = true;  // 从自车所在 Section 开始
    }
    if (!processing) {
      continue;  // 跳过自车所在 Section 之前的部分
    }
    double section_start_s = current_s;                   // 当前 Section 的起点 s
    double section_end_s   = current_s + section.length;  // 当前 Section 的终点 s

    // 如果 Section 起点已超出 BEV 车道线范围，停止遍历
    if (section_start_s > bev_end_s) {
      break;
    }

    // 计算 BEV 车道线与当前 Section 的重叠区间
    double overlap_start = std::max(bev_start_s, section_start_s);
    double overlap_end   = std::min(bev_end_s, section_end_s);

    // 打印当前处理的 Section 信息
    AINFO << "[SDNOA] Processing section ID: " << section.id << ", s range: [" << section_start_s << ", " << section_end_s << "]";

    if (overlap_start < overlap_end) {
      // 打印 Section 与 BEV 车道线的重叠区间
      AINFO << "  Overlap with BEV lane: [" << overlap_start << ", " << overlap_end << "]";

      for (const auto &lg_idx : section.lane_group_idx) {
        double lg_start_s = current_s + lg_idx.start_range_offset;
        double lg_end_s   = current_s + lg_idx.end_range_offset;

        // 打印 LaneGroup 的信息
        AINFO << "    LaneGroup ID: " << lg_idx.id << ", s range: [" << lg_start_s << ", " << lg_end_s << "]";

        double lg_overlap_start = std::max(overlap_start, lg_start_s);
        double lg_overlap_end   = std::min(overlap_end, lg_end_s);
        if (lg_overlap_start < lg_overlap_end) {
          double overlap_length = lg_overlap_end - lg_overlap_start;
          lanegroup_coverage[lg_idx.id] += overlap_length;
          // 打印重叠长度
          AINFO << "      Overlap length: " << overlap_length;
        } else {
          AINFO << "      No overlap with this LaneGroup.";
        }
      }
    } else {
      AINFO << "  No overlap with this section.";
    }
    current_s = section_end_s;
  }

  // 打印 lanegroup_coverage 的总结信息
  AINFO << "[SDNOA] LaneGroup coverage summary:";
  for (const auto &[lg_id, length] : lanegroup_coverage) {
    AINFO << "  LaneGroup ID: " << lg_id << ", Total coverage: " << length;
  }

  // 选择覆盖长度最大的 LaneGroup
  uint64_t best_lanegroup_id = 0;
  double   max_coverage      = 0.0;
  for (const auto &[lg_id, length] : lanegroup_coverage) {
    if (length > max_coverage) {
      max_coverage      = length;
      best_lanegroup_id = lg_id;
    }
  }
  if (best_lanegroup_id == 0) {
    AWARN << "[SDNOA] No matching LaneGroup for max range BEV lane!";
    return;
  }

  // 打印 best_lanegroup_id 的信息
  AINFO << "[SDNOA] Best LaneGroup ID: " << best_lanegroup_id << ", Max coverage: " << max_coverage;

  // 获取 LaneGroup 信息
  const SDLaneGroupInfo *best_lanegroup = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(best_lanegroup_id);
  if (!best_lanegroup || best_lanegroup->lane_info.empty()) {
    AWARN << "[SDNOA] LaneGroup " << best_lanegroup_id << " not found or empty!";
    return;
  }

  // 打印 best_lanegroup 的信息
  size_t lanegroup_lane_count = best_lanegroup->lane_info.size();
  AINFO << "[SDNOA] Best LaneGroup info:";
  AINFO << "  Lane count: " << lanegroup_lane_count;
  if (lanegroup_lane_count > 0) {
    AINFO << "  Last lane type: " << static_cast<int>(best_lanegroup->lane_info.back().type);
    if (best_lanegroup->lane_info.back().type == LaneType::LANE_EMERGENCY) {
      AINFO << "  Contains emergency lane.";
    } else {
      AINFO << "  Does not contain emergency lane.";
    }
  }

  // Step 4: 根据车道数和 bev_line_sorts_ 的顺序进行属性赋值
  if (bev_lane_count == lanegroup_lane_count) {
    // 车道数相等，按 bev_line_sorts_ 的顺序赋值
    AINFO << "[SDNOA] Assigning lane types (equal lane count):";
    for (size_t i = 0; i < bev_lane_count; ++i) {
      auto assigned_type = mapLaneType(best_lanegroup->lane_info[i].type);
      AINFO << "  BEV Lane ID: " << sorted_bev_lanes[i]->id << ", Assigned type: " << static_cast<int>(assigned_type);
      sorted_bev_lanes[i]->lane_type = assigned_type;
    }
  } else {
    // 车道数不等，检查 LaneGroup 是否有应急车道
    AINFO << "[SDNOA] Lane counts differ: BEV lanes: " << bev_lane_count << ", LaneGroup lanes: " << lanegroup_lane_count;
    if (lanegroup_lane_count > 0 && best_lanegroup->lane_info.back().type == LaneType::LANE_EMERGENCY) {
      if (!sorted_bev_lanes.empty()) {
        AINFO << "  Setting rightmost BEV lane ID: " << sorted_bev_lanes.back()->id << " to EMERGENCY";
        sorted_bev_lanes.back()->lane_type = BevLaneType::LANE_TYPE_EMERGENCY;
      }
    } else {
      AINFO << "[SDNOA] LaneGroup has no emergency lane, keeping BEV lane types unchanged.";
    }
  }

  AINFO << "[SDNOA] Successfully assigned types to BEV lanes based on bev_line_sorts_.";
}

EmergencyLaneInfo SdNavigationHighway::GetEmergencyLaneInfo() {
  EmergencyLaneInfo result;
  auto              ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[LDNOA] Failed: ld_route_info is null!";
    return result;
  }

  // 获取自车位置信息
  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;
  SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  // 计算自车全局s坐标
  double ego_global_s      = 0.0;
  bool   ego_section_found = false;
  for (const auto &section : ld_route_info->sections) {
    if (section.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      ego_section_found = true;
      break;
    }
    ego_global_s += section.length;
  }
  if (!ego_section_found) {
    AWARN << "[LDNOA] Failed: ego_section not found!";
    return result;
  }
  SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Ego global s: " << ego_global_s;

  // 定义检测范围
  const double min_s = rear_distance_;
  const double max_s = front_distance_;

  // 存储应急车道区间
  std::vector<std::pair<double, double>> left_ranges;
  std::vector<std::pair<double, double>> right_ranges;
  int                                    left_lane_count  = 0;
  int                                    right_lane_count = 0;

  // 遍历所有路段
  double current_s = 0.0;
  for (const auto &section : ld_route_info->sections) {
    const double section_start_s = current_s;
    const double section_end_s   = current_s + section.length;
    current_s                    = section_end_s;  // 更新当前s位置

    // 跳过不在检测范围内的路段
    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s)
      continue;

    //SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Section ID: " << section.id
    //                    << " start s: " << section_start_s << " end s: " << section_end_s << "  section.laneids.size(): "<<section.lane_ids.size();

    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {

      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
        //SD_COARSE_MATCH_LOG << "  [GetEmergencyLaneInfo] lane id:  " <<lane_id;
      }
    }

    // 按车道序列号排序 (lane_seq从左到右递增)
    // std::sort(section_lanes.begin(), section_lanes.end(),
    //           [](const cem::message::env_model::LaneInfo* a,
    //              const cem::message::env_model::LaneInfo* b) {
    //             return a->lane_seq < b->lane_seq;
    //           });

    if (section_lanes.empty())
      continue;

    // 检查最左侧车道
    const cem::message::env_model::LaneInfo *leftmost_lane = section_lanes.front();
    if (leftmost_lane->type == cem::message::env_model::LaneType::LANE_EMERGENCY) {
      const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
      const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

      if (lane_start_s < lane_end_s) {
        // 合并连续区间
        if (!left_ranges.empty() && lane_start_s <= left_ranges.back().second) {
          left_ranges.back().second = std::max(left_ranges.back().second, lane_end_s);
        } else {
          left_ranges.emplace_back(lane_start_s, lane_end_s);
        }
        //left_lane_count = section_lanes.size();
        if (left_lane_count < section_lanes.size()) {
          left_lane_count = section_lanes.size();
        }
        SD_COARSE_MATCH_LOG << "  Left emergency lane detected, range: [" << lane_start_s << ", " << lane_end_s << "]";
      }
    }

    // 检查最右侧车道
    const cem::message::env_model::LaneInfo *rightmost_lane = section_lanes.back();
    if (rightmost_lane->type == cem::message::env_model::LaneType::LANE_EMERGENCY) {
      const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
      const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

      if (lane_start_s < lane_end_s) {
        // 合并连续区间
        if (!right_ranges.empty() && lane_start_s <= right_ranges.back().second) {
          right_ranges.back().second = std::max(right_ranges.back().second, lane_end_s);
        } else {
          right_ranges.emplace_back(lane_start_s, lane_end_s);
        }
        //right_lane_count = section_lanes.size();
        if (right_lane_count < section_lanes.size()) {
          right_lane_count = section_lanes.size();
        }
        SD_COARSE_MATCH_LOG << "  Right emergency lane detected, range: [" << lane_start_s << ", " << lane_end_s << "]";
      }
    }
  }

  // 填充结果
  result.left.ranges    = left_ranges;
  result.right.ranges   = right_ranges;
  result.left.exists    = !left_ranges.empty();
  result.right.exists   = !right_ranges.empty();
  result.left.lane_num  = left_lane_count;
  result.right.lane_num = right_lane_count;
  // 全局
  g_emergency_lane_info_.left.ranges    = left_ranges;
  g_emergency_lane_info_.right.ranges   = right_ranges;
  g_emergency_lane_info_.left.exists    = !left_ranges.empty();
  g_emergency_lane_info_.right.exists   = !right_ranges.empty();
  g_emergency_lane_info_.left.lane_num  = left_lane_count;
  g_emergency_lane_info_.right.lane_num = right_lane_count;

  // 调试日志
  if (result.right.exists) {
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Right emergency lane exists in merged ranges:";
    for (const auto &range : result.right.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo]  lane num: " << result.right.lane_num;
  } else {
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] No right emergency lane found.";
  }

  return result;
}

void SdNavigationHighway::GetHarborStopInfo() {
  // 使用LD地图数据替换SD地图数据
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[LDNOA] Failed: ld_route_info is null!";
    return;
  }

  // 获取自车位置信息
  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;
  SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  // 计算自车全局s坐标
  double ego_global_s      = 0.0;
  bool   ego_section_found = false;
  for (const auto &section : ld_route_info->sections) {
    if (section.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      ego_section_found = true;
      break;
    }
    ego_global_s += section.length;
  }
  if (!ego_section_found) {
    AWARN << "[LDNOA] Failed: ego_section not found!";
    return;
  }
  SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Ego global s: " << ego_global_s;

  // 定义检测范围
  const double min_s = rear_distance_;   // -100.0
  const double max_s = front_distance_;  // 300.0

  // 初始化港湾停靠站信息
  sd_harbor_stop_info_.exists        = false;
  sd_harbor_stop_info_.is_left_most  = false;
  sd_harbor_stop_info_.is_right_most = false;
  sd_harbor_stop_info_.ranges.clear();

  // 存储港湾停靠站区间
  std::vector<std::pair<double, double>> harbor_stop_ranges;
  bool                                   current_leftmost  = false;
  bool                                   current_rightmost = false;

  // 遍历所有路段
  double current_s = 0.0;
  for (const auto &section : ld_route_info->sections) {
    const double section_start_s = current_s;
    const double section_end_s   = current_s + section.length;
    current_s                    = section_end_s;  // 更新当前s位置

    // 跳过不在检测范围内的路段
    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s) {
      continue;
    }
    //SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Section ID: " << section.id
    //                   << " start s: " << section_start_s << " end s: " << section_end_s;

    // 收集当前section的所有车道,直接处理车道
    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {
      // 使用LD地图获取车道信息
      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
      }
    }

    if (section_lanes.empty())
      continue;

    // 检测港湾停靠站并记录位置
    for (size_t idx = 0; idx < section_lanes.size(); ++idx) {
      const auto *lane = section_lanes[idx];
      if (lane->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP) {
        // 计算车道在相对坐标系中的位置
        const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
        const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

        if (lane_start_s < lane_end_s) {
          // 合并连续区间
          if (!harbor_stop_ranges.empty() && lane_start_s <= harbor_stop_ranges.back().second) {
            harbor_stop_ranges.back().second = std::max(harbor_stop_ranges.back().second, lane_end_s);
          } else {
            harbor_stop_ranges.emplace_back(lane_start_s, lane_end_s);
          }

          // 检查是否在最左或最右位置
          if (idx == 0) {
            current_leftmost = true;
          }
          if (idx == section_lanes.size() - 1) {
            current_rightmost = true;
          }

          SD_COARSE_MATCH_LOG << " [GetHarborStopInfo] Harbor stop lane detected, lane_seq: " << lane->lane_seq << ", range: ["
                              << lane_start_s << ", " << lane_end_s << "]";
        }
      }
    }
  }

  // 填充结果
  sd_harbor_stop_info_.ranges        = harbor_stop_ranges;
  sd_harbor_stop_info_.exists        = !harbor_stop_ranges.empty();
  sd_harbor_stop_info_.is_left_most  = current_leftmost;
  sd_harbor_stop_info_.is_right_most = current_rightmost;

  // 调试日志
  if (sd_harbor_stop_info_.exists) {
    SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] Harbor stop exists in merged ranges:";
    for (const auto &range : sd_harbor_stop_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    SD_COARSE_MATCH_LOG << "  is_left_most: " << sd_harbor_stop_info_.is_left_most
                        << ", is_right_most: " << sd_harbor_stop_info_.is_right_most;
  } else {
    SD_COARSE_MATCH_LOG << "[GetHarborStopInfo] No harbor stop found.";
  }
}

// bool SdNavigationHighway::IsRampSection(const SDSectionInfo &section_info) {
//   if (section_info.direction == SDDirectionType::BIDIRECTIONAL_PASSABLE) {
//     return false;
//   }
//   uint32_t section_type = section_info.link_type;
//   if ((section_type & (uint32_t)SDLinkTypeMask::SDLT_JCT) == (uint32_t)SDLinkTypeMask::SDLT_JCT ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_IC) == (uint32_t)SDLinkTypeMask::SDLT_IC ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_SERVICE) == (uint32_t)SDLinkTypeMask::SDLT_SERVICE ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_PARK) == (uint32_t)SDLinkTypeMask::SDLT_PARK ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_RAMP) == (uint32_t)SDLinkTypeMask::SDLT_RAMP ||
//       (section_info.road_class == SDRoadClass::SD_OTHER_ROAD &&
//        (section_type & (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION) == (uint32_t)SDLinkTypeMask::SDLT_POICONNECTION)) {
//     return true;
//   } else {
//     return false;
//   }
// }
bool SdNavigationHighway::LD_IsRampSection(const cem::message::env_model::SectionInfo &section_info) {
  uint32_t section_type = section_info.link_type;
  if ((section_type & (uint32_t)LDLinkTypeMask::LT_JCT) == (uint32_t)LDLinkTypeMask::LT_JCT ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_IC) == (uint32_t)LDLinkTypeMask::LT_IC ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_SAPA) == (uint32_t)LDLinkTypeMask::LT_SAPA ||
      (section_info.road_class == cem::message::env_model::RoadClass::OTHERS &&
       (section_type & (uint32_t)LDLinkTypeMask::LT_MAINROAD_CONNECTION) == (uint32_t)LDLinkTypeMask::LT_MAINROAD_CONNECTION)) {
    return true;
  }
  if (section_info.lane_ids.size() > 0) {
    for (auto &lane_id_tmp : section_info.lane_ids) {
      auto lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id_tmp);
      if (lane) {
        if (lane->type == LaneType::LANE_RAMP) {
          return true;
        }
      }
    }
  }
  return false;
}

bool SdNavigationHighway::LD_IsMainRoadSection(const cem::message::env_model::SectionInfo &section_info) {
  //ld routing_map proto no DirectionType
  if (section_info.road_class == cem::message::env_model::RoadClass::EXPRESSWAY ||
      section_info.road_class == cem::message::env_model::RoadClass::URBAN_EXPRESSWAY) {
    if (!LD_IsRampSection(section_info)) {
      return true;
    }
  }
  return false;
}

// bool SdNavigationHighway::IsMainRoadSection(const SDSectionInfo &section_info) {
//   if (section_info.road_class == SDRoadClass::SD_HIGHWAY || section_info.road_class == SDRoadClass::SD_CITY_FAST_WAY) {
//     if (!IsRampSection(section_info)) {
//       return true;
//     }
//   }
//   return false;
// }
bool SdNavigationHighway::LD_IsCityRoadSection(const cem::message::env_model::SectionInfo &section_info) {
  uint32_t section_type = section_info.link_type;
  if ((section_type & (uint32_t)LDLinkTypeMask::LT_WITHIN_INTERSECTION) == (uint32_t)LDLinkTypeMask::LT_WITHIN_INTERSECTION ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_TOLLGATE) == (uint32_t)LDLinkTypeMask::LT_TOLLGATE ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_ABANDONED_TOLLGATE) == (uint32_t)LDLinkTypeMask::LT_ABANDONED_TOLLGATE ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_ROUNDABOUT) == (uint32_t)LDLinkTypeMask::LT_ROUNDABOUT ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_LEFT) == (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_LEFT ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_RIGHT) == (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_RIGHT ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_BOTH) == (uint32_t)LDLinkTypeMask::LT_S_INTERSECTION_BOTH ||
      (section_type & (uint32_t)LDLinkTypeMask::LT_INJUNCTION) == (uint32_t)LDLinkTypeMask::LT_INJUNCTION) {
    return true;
  }
  for (auto &suc_laneid : section_info.successor_section_id_list) {
    auto section_ptr_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(suc_laneid);
    if (section_ptr_tmp) {
      uint32_t suc_section_type = section_ptr_tmp->link_type;
      if ((suc_section_type & (uint32_t)LDLinkTypeMask::LT_WITHIN_INTERSECTION) == (uint32_t)LDLinkTypeMask::LT_WITHIN_INTERSECTION) {
        return true;
      }
    }
  }
  return false;
}
// bool SdNavigationHighway::IsCityRoadSection(const SDSectionInfo &section_info) {
//   uint32_t section_type = section_info.link_type;
//   if ((section_type & (uint32_t)SDLinkTypeMask::SDLT_CROSSLINK) == (uint32_t)SDLinkTypeMask::SDLT_CROSSLINK ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_RING) == (uint32_t)SDLinkTypeMask::SDLT_RING ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_SERVICE) == (uint32_t)SDLinkTypeMask::SDLT_SERVICE ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_PARK) == (uint32_t)SDLinkTypeMask::SDLT_PARK ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_RIGHTTURN) == (uint32_t)SDLinkTypeMask::SDLT_RIGHTTURN ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_LEFTTURN) == (uint32_t)SDLinkTypeMask::SDLT_LEFTTURN ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_UTURN) == (uint32_t)SDLinkTypeMask::SDLT_UTURN ||
//       (section_type & (uint32_t)SDLinkTypeMask::SDLT_PARKLINK) == (uint32_t)SDLinkTypeMask::SDLT_PARKLINK) {
//     return true;
//   }
//   for (auto &suc_laneid : section_info.successor_section_id_list) {
//     auto section_ptr_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(suc_laneid);
//     if (section_ptr_tmp) {
//       uint32_t suc_section_type = section_ptr_tmp->link_type;
//       if ((suc_section_type & (uint32_t)SDLinkTypeMask::SDLT_CROSSLINK) == (uint32_t)SDLinkTypeMask::SDLT_CROSSLINK) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

//split use
void SdNavigationHighway::judge_left_or_rightsection(std::vector<uint64_t>                      &sort_section_id,
                                                     const cem::message::env_model::SectionInfo &SectionInfo) {
  if (SectionInfo.successor_section_id_list.size() != 2 || sort_section_id.size() != 2) {
    return;
  }
  SD_HIGHJUNCTION_CONVERTE_LOG << "SectionInfoid:" << SectionInfo.id;
  SD_HIGHJUNCTION_CONVERTE_LOG << "处理qian的id:";
  for (auto id : sort_section_id) {
    SD_HIGHJUNCTION_CONVERTE_LOG << id;
  }
  int mpp_section_leftlaneid  = 0;
  int mpp_section_rightlaneid = 0;

  for (int i = 0; i < SectionInfo.lane_ids.size(); i++) {
    auto mpp_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(SectionInfo.lane_ids[i]);
    if (mpp_lane_info && (mpp_lane_info->type == LaneType::LANE_EMERGENCY || mpp_lane_info->type == LaneType::LANE_DIVERSION)) {
      continue;
    }
    mpp_section_leftlaneid = SectionInfo.lane_ids[i];
    break;
  }
  for (int i = SectionInfo.lane_ids.size() - 1; i >= 0; i--) {
    auto mpp_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(SectionInfo.lane_ids[i]);
    if (mpp_lane_info && (mpp_lane_info->type == LaneType::LANE_EMERGENCY || mpp_lane_info->type == LaneType::LANE_DIVERSION)) {
      continue;
    }
    mpp_section_rightlaneid = SectionInfo.lane_ids[i];
    break;
  }
  if (mpp_section_leftlaneid == mpp_section_rightlaneid) {
    return;  //prof the section lane has only y, this function is invalid
  }
  for (auto lane_info_id : sort_section_id) {
    bool flag_find = 0;
    auto section_  = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(lane_info_id);
    if (section_) {
      for (auto &lane_info_id : section_->lane_ids) {
        auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_info_id);
        if (lane_info && (lane_info->type == LaneType::LANE_EMERGENCY || lane_info->type == LaneType::LANE_DIVERSION)) {
          continue;
        }
        if (lane_info && lane_info->previous_lane_ids.size() > 0 && lane_info->previous_lane_ids[0] == mpp_section_leftlaneid) {
          if (sort_section_id[0] != section_->id) {
            auto temp          = sort_section_id[0];
            sort_section_id[0] = section_->id;
            sort_section_id[1] = temp;
            flag_find          = 1;
          }
        }
        break;
      }
    }
    if (flag_find == 1) {
      break;
    }
  }
  SD_HIGHJUNCTION_CONVERTE_LOG << "处理之后的id:";
  for (auto id : sort_section_id) {
    SD_HIGHJUNCTION_CONVERTE_LOG << id;
  }
  return;
}

void SdNavigationHighway::GetLDJunctionInfos() {
  auto ld_map_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_map_info) {
    return;
  }
  ld_merge_points.clear();
  double ego_dist_s         = 0;
  double section_length_tmp = 0;
  bool   use_topo_flag      = 0;
  ego_section_id_           = ld_map_info->navi_start.section_id;
  for (auto &mpp_section : ld_map_info->sections) {
    if (mpp_section.id == ld_map_info->navi_start.section_id) {
      ego_dist_s = section_length_tmp + ld_map_info->navi_start.s_offset;
      break;
    }
    section_length_tmp += mpp_section.length;
  }

  double all_mpp_length = 0;

  const auto                                                                &mpp_sections = ld_map_info->sections;
  std::unordered_map<uint64_t, const cem::message::env_model::SectionInfo *> section_map;
  std::vector<uint64_t>navi_lane_ids;
  for (const auto &section : mpp_sections) {
    section_map.insert({section.id, &section});
    for(const auto laneid : section.lane_ids)
    {
      navi_lane_ids.emplace_back(laneid);
    }
  }
  for (const auto &subpath : ld_map_info->subpaths) {
    for (const auto &section : subpath.sections) {
      section_map.insert({section.id, &section});
    }
  }

  auto GetSuccInfos = [&](int idx) -> std::vector<SDMapElementExtract::LDAngleJunction> {
    if (idx < 0 || idx + 1 >= static_cast<int>(mpp_sections.size())) {
      return {};
    }
    const auto &section_curr = mpp_sections[idx];
    const auto &section_next = mpp_sections[idx + 1];
    if (section_curr.successor_section_id_list.size() <= 1) {
      return {};
    }
    std::vector<SDMapElementExtract::LDAngleJunction> res;

    double      min_angle = std::numeric_limits<double>::infinity();
    std::size_t min_idx   = 0;
    for (const auto &succ_id : section_curr.successor_section_id_list) {
      auto it =
          std::find_if(section_map.begin(), section_map.end(), [&](const auto &rhs) { return rhs.second && rhs.second->id == succ_id; });
      if (static_cast<int>(succ_id / 10) == static_cast<int>(section_curr.id / 10)) {
        // AINFO << "skip_same_road_id:" << succ_id;
        continue;
      }
      if (it == section_map.end()) {
        continue;
      }
      auto angle_diff = LD_AngleDiffBetweenSection(section_curr, *it->second);
      if (!angle_diff) {
        continue;
      }
      auto &succ_info    = res.emplace_back();
      succ_info.section  = std::make_shared<cem::message::env_model::SectionInfo>(*it->second);
      succ_info.next_suc = succ_id == section_next.id;
      succ_info.angle    = angle_diff.value();
      if (min_angle > std::fabs(angle_diff.value())) {
        min_angle = std::fabs(angle_diff.value());
        min_idx   = res.size() - 1;
      }
    }
    if (res.size() > 1) {
      res[min_idx].min_angle = true;
      SDMapElementExtract::JunctionAngleLDSectionDirection(section_curr, res);
      std::sort(res.begin(), res.end(), [](const auto &lhs, const auto &rhs) { return lhs.angle > rhs.angle; });
    }
    return res;
  };

  int idx = -1;
  //when passed junction ,ld no send sections,delete it.
  //std::erase_if(navi_lane_num, [](const auto &pair) { return section_map.count(pair.first) == 0; });//c++20
  // 遍历并删除：section_map中不存在的键
  for (auto it = navi_lane_num.begin(); it != navi_lane_num.end();) {
    if (section_map.count(it->first) == 0) {
      // erase返回下一个有效迭代器，直接赋值避免失效
      it = navi_lane_num.erase(it);
    } else {
      // 不删除则正常递增迭代器
      ++it;
    }
  }
  SD_HIGHJUNCTION_CONVERTE_LOG << "navi_lane_num.size()" << navi_lane_num.size();
  for (const auto &pair : navi_lane_num) {
    SD_HIGHJUNCTION_CONVERTE_LOG << "junctionid:" << pair.first << " main_road_navigation_lane_num:" << pair.second;
  }
  for (auto &mpp_section : ld_map_info->sections) {
    idx++;
    all_mpp_length += mpp_section.length;
    if (LD_IsCityRoadSection(mpp_section)) {
      if (all_mpp_length - ego_dist_s <= 0) {
        sd_junction_infos_.clear();
        continue;
      } else {
        break;
      }
    }
    if ((mpp_section.link_type & (uint32_t)LDLinkTypeMask::LT_TOLLBOOTH )== (uint32_t)LDLinkTypeMask::LT_TOLLBOOTH) {
      break;
    }
    uint64_t erase_id = 0;
    bool     use_cur_section_end_points{false};
    if (LD_IsMainRoadSection(mpp_section) && idx + 1 < mpp_sections.size()) {
      for (auto &succ_id : mpp_section.successor_section_id_list) {
        if (mpp_sections[idx + 1].id != succ_id && INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(succ_id)) {

          auto suc_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(succ_id);
          auto angle_      = LD_AngleDiffBetweenSection(mpp_section, *suc_section, true, use_cur_section_end_points);
          if (angle_.has_value() && angle_.value() > 0) {
            auto   distance = 0;
            int8_t j        = 0;
            while (suc_section && suc_section->successor_section_id_list.size() > 0) {
              j++;
              distance = distance + suc_section->length;
              if (suc_section && (distance > 100 || j > 6)) {
                auto angle_100success = LD_AngleDiffBetweenSection(mpp_section, *suc_section, true, use_cur_section_end_points);
                if (angle_100success.has_value() && fabs(angle_100success.value()) > 150) {
                  erase_id = succ_id;
                }
                break;
              }
              suc_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(suc_section->successor_section_id_list.front());
            }
          }
        }
        if (erase_id == succ_id) {
          break;
        }
      }
    }
    if (erase_id) {
      continue;
    }
    /*Filter split intersections meeting the following criteria:
      1.Not on the navigation route;
      2.Located on the left;
      3.Not classified as highways.
      4.only have one lane
   */
    if (LD_IsMainRoadSection(mpp_section) && idx + 1 < mpp_sections.size() && LD_IsMainRoadSection(mpp_sections[idx + 1])) {
      for (auto &succ_id : mpp_section.successor_section_id_list) {

        if (mpp_sections[idx + 1].id != succ_id && INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(succ_id)) {

          auto suc_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(succ_id);
          auto angle_      = LD_AngleDiffBetweenSection(mpp_section, *suc_section, true, use_cur_section_end_points);
          if (angle_.has_value() && angle_.value() > 10 && angle_.value() < 100 &&
              suc_section->road_class != cem::message::env_model::RoadClass::EXPRESSWAY &&
              suc_section->road_class != cem::message::env_model::RoadClass::URBAN_EXPRESSWAY && suc_section->lane_ids.size() == 1) {
            erase_id = succ_id;
          }
        }
        if (erase_id) {
          break;
        }
      }
    }
    auto successor_section_id_list = mpp_section.successor_section_id_list;
    if (erase_id) {
      auto it = std::find(successor_section_id_list.begin(), successor_section_id_list.end(), erase_id);

      if (it != successor_section_id_list.end()) {

        successor_section_id_list.erase(it);
      }
      continue;
    }
    /// 下匝道
    bool has_ramp_into_flag = false;
    if (LD_IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 2) {
      use_topo_flag = 0;
      for (auto &suc_section_id_tmp : mpp_section.successor_section_id_list) {
        auto suc_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(suc_section_id_tmp);
        if (suc_section_tmp) {
          if (suc_section_tmp->is_mpp_section && LD_IsRampSection(*suc_section_tmp)) {
            std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
            std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
              auto section_tmp_a = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(a);
              auto section_tmp_b = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(b);
              if (section_tmp_a && section_tmp_b && section_tmp_a->points.size() >= 2 && section_tmp_b->points.size() >= 2) {
                Eigen::Vector2f              section_tmp_a_points0(section_tmp_a->points.at(0).x, section_tmp_a->points.at(0).y);
                Eigen::Vector2f              section_tmp_a_points1(section_tmp_a->points.at(1).x, section_tmp_a->points.at(1).y);
                Eigen::Vector2f              section_tmp_b_points0(section_tmp_b->points.at(0).x, section_tmp_b->points.at(0).y);
                Eigen::Vector2f              section_tmp_b_points1(section_tmp_b->points.at(1).x, section_tmp_b->points.at(1).y);
                std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a_points0, section_tmp_a_points1};
                std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b_points0, section_tmp_b_points1};
                return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
              } else {
                use_topo_flag = 1;
              }
              return a < b;
            });
            if (use_topo_flag == 1) {
              SD_HIGHJUNCTION_CONVERTE_LOG << "FUNCTION RampInto:";
              judge_left_or_rightsection(split_section_ids, mpp_section);
            }
            if (split_section_ids.back() == suc_section_tmp->id) {
              JunctionInfo junction_info;
              junction_info.junction_id           = mpp_section.id;
              junction_info.junction_type         = JunctionType::RampInto;
              junction_info.offset                = all_mpp_length - ego_dist_s;
              junction_info.main_road_lane_nums   = GetLDSectionMinLaneNumNOTEmergency(mpp_section);
              junction_info.target_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(*suc_section_tmp);
              junction_info.split_num             = 2;
              junction_info.junction_ids.clear();
              junction_info.junction_ids.emplace_back(mpp_section.id);
              junction_info.junction_ids.emplace_back(suc_section_id_tmp);
              sd_junction_infos_.push_back(junction_info);
              has_ramp_into_flag = true;
              break;
            }
          }
        }
      }
    }
    if (has_ramp_into_flag) {
      continue;
    }

    /// 途径下匝道
    if (LD_IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 2) {
      use_topo_flag         = 0;
      auto suc_section_tmp1 = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_section.successor_section_id_list.front());
      auto suc_section_tmp2 = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_section.successor_section_id_list.back());
      if (suc_section_tmp1 && suc_section_tmp2) {
        int main_road_index = 0;
        if (suc_section_tmp2->is_mpp_section && LD_IsRampSection(*suc_section_tmp1) && LD_IsMainRoadSection(*suc_section_tmp2)) {
          main_road_index = 2;
        }
        if (suc_section_tmp1->is_mpp_section && LD_IsRampSection(*suc_section_tmp2) && LD_IsMainRoadSection(*suc_section_tmp1)) {
          main_road_index = 1;
        }
        //case http://jira-irc.byd.com:8080/browse/CNOAC2-107675
        if (suc_section_tmp1->is_mpp_section && LD_IsMainRoadSection(*suc_section_tmp1)) {
          if (suc_section_tmp2->road_class >= cem::message::env_model::RoadClass::NATION_ROAD) {
            main_road_index = 1;
          }
        }
        if (suc_section_tmp2->is_mpp_section && LD_IsMainRoadSection(*suc_section_tmp2)) {
          if (suc_section_tmp1->road_class >= cem::message::env_model::RoadClass::NATION_ROAD) {
            main_road_index = 2;
          }
        }

        std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
        std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
          auto section_tmp_a = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(a);
          auto section_tmp_b = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(b);
          if (section_tmp_a && section_tmp_b && section_tmp_a->points.size() >= 2 && section_tmp_b->points.size() >= 2) {
            Eigen::Vector2f              section_tmp_a_points0(section_tmp_a->points.at(0).x, section_tmp_a->points.at(0).y);
            Eigen::Vector2f              section_tmp_a_points1(section_tmp_a->points.at(1).x, section_tmp_a->points.at(1).y);
            Eigen::Vector2f              section_tmp_b_points0(section_tmp_b->points.at(0).x, section_tmp_b->points.at(0).y);
            Eigen::Vector2f              section_tmp_b_points1(section_tmp_b->points.at(1).x, section_tmp_b->points.at(1).y);
            std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a_points0, section_tmp_a_points1};
            std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b_points0, section_tmp_b_points1};
            return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
          } else {
            use_topo_flag = 1;
          }
          return a < b;
        });
        if (use_topo_flag == 1) {
          SD_HIGHJUNCTION_CONVERTE_LOG << "FUNCTION ApproachRampInto:";
          judge_left_or_rightsection(split_section_ids, mpp_section);
        }

        if (main_road_index != 0 && mpp_section.successor_section_id_list[main_road_index - 1] == split_section_ids.front()) {
          JunctionInfo junction_info;
          junction_info.junction_id         = mpp_section.id;
          junction_info.junction_type       = JunctionType::ApproachRampInto;
          junction_info.offset              = all_mpp_length - ego_dist_s;
          junction_info.main_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(mpp_section);
          if (main_road_index == 1) {
            junction_info.target_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(*suc_section_tmp1);
          } else if (main_road_index == 2) {
            junction_info.target_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(*suc_section_tmp2);
          }
          junction_info.split_num = 2;

          junction_info.junction_ids.clear();
          junction_info.junction_ids.emplace_back(mpp_section.id);
          if (main_road_index == 1) {
            junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());
          } else if (main_road_index == 2) {
            junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.back());
          }

          sd_junction_infos_.push_back(junction_info);
          continue;
        }
      }
    }

    /// 匝道汇入
    bool has_ramp_merge_flag = false;
    if (LD_IsRampSection(mpp_section) && mpp_section.successor_section_id_list.size() == 1) {
      auto suc_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_section.successor_section_id_list.front());
      if (suc_section_tmp && LD_IsMainRoadSection(*suc_section_tmp) && suc_section_tmp->is_mpp_section) {
        if (suc_section_tmp->predecessor_section_id_list.size() == 2) {
          for (auto &merge_id_tmp : suc_section_tmp->predecessor_section_id_list) {
            if (merge_id_tmp != mpp_section.id) {
              auto merge_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(merge_id_tmp);
              if (merge_section_tmp && LD_IsMainRoadSection(*merge_section_tmp) && !merge_section_tmp->is_mpp_section) {
                JunctionInfo junction_info;
                junction_info.junction_id = mpp_section.id;
                if (mpp_section.lane_ids.size() > 0) {
                  for (auto lane_info_id : mpp_section.lane_ids) {
                    auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_info_id);
                    if (lane_info && (lane_info->type == LaneType::LANE_EMERGENCY || lane_info->type == LaneType::LANE_DIVERSION)) {
                      continue;
                    }
                    if (!lane_info) {
                      continue;
                    }
                    unsigned long succ_id = 0;
                    for (auto succ_lane_info_id : suc_section_tmp->lane_ids) {
                      auto succ_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(succ_lane_info_id);
                      if (succ_lane_info &&
                          (succ_lane_info->type == LaneType::LANE_EMERGENCY || succ_lane_info->type == LaneType::LANE_DIVERSION)) {
                        continue;
                      } else {
                        if (succ_lane_info) {
                          succ_id = succ_lane_info_id;
                          break;
                        }
                      }
                    }
                    //first method :use topo
                    if (succ_id != 0 && lane_info->next_lane_ids.size() > 0 && lane_info->next_lane_ids[0] == succ_id) {
                      junction_info.junction_type = JunctionType::Unknow;
                    }
                    //second method : use cross vec to judge
                    if (mpp_section.points.size() >= 2 && merge_section_tmp->points.size() >= 2) {
                      auto x0             = mpp_section.points.back().x;
                      auto y0             = mpp_section.points.back().y;
                      auto x1             = merge_section_tmp->points[0].x;
                      auto y1             = merge_section_tmp->points[0].y;
                      auto x2             = mpp_section.points[0].x;
                      auto y2             = mpp_section.points[0].y;
                      auto vector_mpp_x   = x2 - x0;
                      auto vector_mpp_y   = y2 - y0;
                      auto vector_merge_x = x1 - x0;
                      auto vector_merge_y = y1 - y0;
                      //mpp_section is the first vector,cross_v>0,merge is right,cross_v merge is left
                      auto cross_v = vector_mpp_x * vector_merge_y - vector_merge_x * vector_mpp_y;
                      if (cross_v > 0) {
                        junction_info.junction_type = JunctionType::Unknow;
                      } else {
                        junction_info.junction_type = JunctionType::RampMerge;
                      }
                    }
                    break;
                  }
                }
                if (junction_info.junction_type != JunctionType::Unknow) {
                  junction_info.junction_type = JunctionType::RampMerge;
                }
                junction_info.offset                = all_mpp_length - ego_dist_s;
                junction_info.main_road_lane_nums   = GetLDSectionMinLaneNumNOTEmergency(*merge_section_tmp);
                junction_info.target_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(*suc_section_tmp);

                junction_info.junction_ids.clear();
                junction_info.junction_ids.emplace_back(mpp_section.id);
                junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());

                sd_junction_infos_.push_back(junction_info);
                has_ramp_merge_flag = true;
                //output  diversion points
                Point frontpoint;
                if (junction_info.offset > -50.0 && junction_info.offset < 300.0) {
                  for (auto point : mpp_section.points) {
                    Eigen::Vector2f temp(point.x, point.y);
                    ld_merge_points.emplace_back(temp);
                    SD_HIGHJUNCTION_CONVERTE_LOG << "mppsection_x:" << point.x << " y:" << point.y;
                  }
                  if (mpp_section.points.size() > 0) {
                    frontpoint = mpp_section.points[0];  //qu chong
                  }
                  int min_length = 300;
                  int length_sum = mpp_section.length;
                  if (mpp_section.predecessor_section_id_list.size() == 1) {
                    auto pre_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_section.predecessor_section_id_list[0]);

                    while (length_sum < min_length && pre_section) {
                      length_sum = length_sum + pre_section->length;
                      for (auto point : pre_section->points) {
                        if (point.x == frontpoint.x) {
                          continue;
                        }
                        Eigen::Vector2f temp(point.x, point.y);
                        ld_merge_points.emplace_back(temp);
                        SD_HIGHJUNCTION_CONVERTE_LOG << "mppsection_x:" << point.x << " y:" << point.y;
                      }
                      if (pre_section->points.size() > 0) {
                        frontpoint = pre_section->points[0];  //qu chong
                      }
                      if (pre_section->predecessor_section_id_list.size() == 1) {
                        pre_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(pre_section->predecessor_section_id_list[0]);

                      } else {
                        break;
                      }
                    }
                  }

                  if (ld_merge_points.size() > 0) {
                    std::sort(ld_merge_points.begin(), ld_merge_points.end(),
                              [](const Eigen::Vector2f &lhs, const Eigen::Vector2f &rhs) { return lhs.x() < rhs.x(); });
                  }
                  for (auto point : ld_merge_points) {
                    SD_HIGHJUNCTION_CONVERTE_LOG << "x:" << point.x() << " y:" << point.y();
                  }
                }
                break;
              }
            }
          }
        }
      }
    }
    if (has_ramp_merge_flag) {
      continue;
    }

    /// 途径匝道汇入
    bool has_approach_ramp_merge_flag = false;
    if (LD_IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 1) {
      auto suc_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_section.successor_section_id_list.front());
      if (suc_section_tmp && LD_IsMainRoadSection(*suc_section_tmp) && suc_section_tmp->is_mpp_section) {
        if (suc_section_tmp->predecessor_section_id_list.size() == 2) {
          for (auto &merge_id_tmp : suc_section_tmp->predecessor_section_id_list) {
            if (merge_id_tmp != mpp_section.id) {
              auto merge_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(merge_id_tmp);
              if (merge_section_tmp && !merge_section_tmp->is_mpp_section && LD_IsRampSection(*merge_section_tmp)) {
                JunctionInfo junction_info;
                junction_info.junction_id = mpp_section.id;
                if (merge_section_tmp->lane_ids.size() > 0) {
                  for (auto lane_info_id : merge_section_tmp->lane_ids) {
                    auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_info_id);
                    if (lane_info && (lane_info->type == LaneType::LANE_EMERGENCY || lane_info->type == LaneType::LANE_DIVERSION)) {
                      continue;
                    }
                    if (!lane_info) {
                      continue;
                    }
                    unsigned long succ_id = 0;
                    for (auto succ_lane_info_id : suc_section_tmp->lane_ids) {
                      auto succ_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(succ_lane_info_id);
                      if (succ_lane_info &&
                          (succ_lane_info->type == LaneType::LANE_EMERGENCY || succ_lane_info->type == LaneType::LANE_DIVERSION)) {
                        continue;
                      } else {
                        if (succ_lane_info) {
                          succ_id = succ_lane_info_id;
                          break;
                        }
                      }
                    }
                    //first method :use topo
                    if (succ_id != 0 && lane_info->next_lane_ids.size() > 0 && lane_info->next_lane_ids[0] == succ_id) {
                      junction_info.junction_type = JunctionType::ApproachRampMergeLeft;
                    }
                    //second method : use cross vec to judge
                    if (mpp_section.points.size() >= 2 && merge_section_tmp->points.size() >= 2) {
                      auto x0             = mpp_section.points.back().x;
                      auto y0             = mpp_section.points.back().y;
                      auto x1             = merge_section_tmp->points[0].x;
                      auto y1             = merge_section_tmp->points[0].y;
                      auto x2             = mpp_section.points[0].x;
                      auto y2             = mpp_section.points[0].y;
                      auto vector_mpp_x   = x2 - x0;
                      auto vector_mpp_y   = y2 - y0;
                      auto vector_merge_x = x1 - x0;
                      auto vector_merge_y = y1 - y0;
                      //mpp_section is the first vector,cross_v>0,merge is right,cross_v merge is left
                      auto cross_v = vector_mpp_x * vector_merge_y - vector_merge_x * vector_mpp_y;
                      if (cross_v < 0) {
                        junction_info.junction_type = JunctionType::ApproachRampMergeLeft;
                      } else {
                        junction_info.junction_type = JunctionType::ApproachRampMerge;
                      }
                    }
                    break;
                  }
                }
                if (junction_info.junction_type != JunctionType::ApproachRampMergeLeft) {
                  junction_info.junction_type = JunctionType::ApproachRampMerge;
                }
                if (junction_info.junction_type == JunctionType::ApproachRampMergeLeft) {
                  junction_info.junction_type = JunctionType::Unknow;
                }
                junction_info.offset                = all_mpp_length - ego_dist_s;
                junction_info.main_road_lane_nums   = GetLDSectionMinLaneNumNOTEmergency(mpp_section);
                junction_info.target_road_lane_nums = GetLDSectionMinLaneNumNOTEmergency(*suc_section_tmp);

                junction_info.junction_ids.clear();
                junction_info.junction_ids.emplace_back(mpp_section.id);
                junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());

                sd_junction_infos_.push_back(junction_info);
                has_approach_ramp_merge_flag = true;
                break;
              }
            }
          }
        }
      }
    }
    if (has_approach_ramp_merge_flag) {
      continue;
    }
    /// 主路的分流
    bool has_main_road_split_merge_flag = 0;
    if (!LD_IsCityRoadSection(mpp_section) && LD_IsMainRoadSection(mpp_section) &&
        (mpp_section.successor_section_id_list.size() == 2 || mpp_section.successor_section_id_list.size() == 3)) {
      use_topo_flag                      = 0;
      uint64_t mpp_split_section_id      = 0;
      int      mpp_split_section_lanenum = 0;
      int      valid_subpath_num         = 0;
      int      main_road_num             = 0;
      for (auto &suc_section_id_tmp : mpp_section.successor_section_id_list) {
        auto suc_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(suc_section_id_tmp);
        if (suc_section_tmp) {
          // if(suc_section_tmp->direction != SDDirectionType::BIDIRECTIONAL_PASSABLE) {
          valid_subpath_num++;
          if (LD_IsMainRoadSection(*suc_section_tmp)) {
            main_road_num++;
          }

          // }
          if (suc_section_tmp->is_mpp_section) {
            mpp_split_section_id      = suc_section_tmp->id;
            mpp_split_section_lanenum = suc_section_tmp->lane_num;
          }
        }
      }
      if (valid_subpath_num >= 2 && main_road_num == valid_subpath_num) {
        std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
        std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
          auto section_tmp_a = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(a);
          auto section_tmp_b = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(b);
          if (section_tmp_a && section_tmp_b && section_tmp_a->points.size() >= 2 && section_tmp_b->points.size() >= 2) {
            Eigen::Vector2f              section_tmp_a_points0(section_tmp_a->points.at(0).x, section_tmp_a->points.at(0).y);
            Eigen::Vector2f              section_tmp_a_points1(section_tmp_a->points.at(1).x, section_tmp_a->points.at(1).y);
            Eigen::Vector2f              section_tmp_b_points0(section_tmp_b->points.at(0).x, section_tmp_b->points.at(0).y);
            Eigen::Vector2f              section_tmp_b_points1(section_tmp_b->points.at(1).x, section_tmp_b->points.at(1).y);
            std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a_points0, section_tmp_a_points1};
            std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b_points0, section_tmp_b_points1};
            return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
          } else {
            use_topo_flag = 1;
          }
          return a < b;
        });
        if (use_topo_flag == 1) {
          SD_HIGHJUNCTION_CONVERTE_LOG << "FUNCTION MainRoadSplit:";
          judge_left_or_rightsection(split_section_ids, mpp_section);
        }

        JunctionInfo junction_info;
        junction_info.junction_id = mpp_section.id;
        if (mpp_split_section_id == split_section_ids.front()) {
          junction_info.junction_type = JunctionType::MainRoadSplitLeft;
        } else if (mpp_split_section_id == split_section_ids.back()) {
          junction_info.junction_type = JunctionType::MainRoadSplitRight;
        } else if (split_section_ids.size() == 3 && mpp_split_section_id == split_section_ids[1]) {
          junction_info.junction_type = JunctionType::MainRoadSplitMiddle;
        }
        junction_info.split_num             = split_section_ids.size();
        junction_info.offset                = all_mpp_length - ego_dist_s;
        junction_info.main_road_lane_nums   = mpp_section.lane_num;
        junction_info.target_road_lane_nums = mpp_split_section_lanenum;
        auto success_mppsection             = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(mpp_split_section_id);
        if (navi_lane_num.count(junction_info.junction_id) == 0) {
          navi_lane_num.insert({junction_info.junction_id, 0});
        }
        if (success_mppsection && navi_lane_num[junction_info.junction_id] == 0) {
          if (success_mppsection->lane_ids.size() == 1) {
            navi_lane_num[junction_info.junction_id] = 1;
          } else if (success_mppsection->lane_ids.size() > 1) {
            int min_length = 400;
            if (junction_info.offset < 0.0) {
              min_length = 300;
            }
            int    nums               = 0;
            double one_to_two         = 0;
            double one_to_three       = 0;
            double one_to_four        = 0;
            bool   use_junctionoffset = 0;  //use when length <400 or 300;
            for (auto lane_id : success_mppsection->lane_ids) {
              auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
              int  j         = 0;
              int  length    = 0;
              if (lane_info && lane_info->type != cem::message::env_model::LaneType::LANE_EMERGENCY &&
                  lane_info->type != cem::message::env_model::LaneType::LANE_DIVERSION) {
                SD_HIGHJUNCTION_CONVERTE_LOG << "junction_info.id:" << junction_info.junction_id;
                if (lane_info->previous_lane_ids.size() == 1) {
                  auto pre_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_info->previous_lane_ids[0]);
                  auto cur_lane      = lane_info;
                  while (cur_lane && length < min_length && j < 100 && pre_lane_info && pre_lane_info->next_lane_ids.size() >= 1 &&
                         std::find(pre_lane_info->next_lane_ids.begin(), pre_lane_info->next_lane_ids.end(), cur_lane->id) !=
                             pre_lane_info->next_lane_ids.end() &&
                         cur_lane->previous_lane_ids.size() == 1 && cur_lane->previous_lane_ids[0] == pre_lane_info->id) {
                    j++;
                    length = length + pre_lane_info->length;
                    SD_HIGHJUNCTION_CONVERTE_LOG << "lengh:" << length;
                    SD_HIGHJUNCTION_CONVERTE_LOG << "j:" << j;
                    uint64_t curr_id = cur_lane->id;
                    cur_lane         = pre_lane_info;
                    if (length >= min_length || j >= 50) {
                      nums++;
                      use_junctionoffset = 1;
                      SD_HIGHJUNCTION_CONVERTE_LOG << "nums" << nums;
                      break;
                    } else {
                      int cur_nextlane_size = cur_lane->next_lane_ids.size();
                      if (cur_nextlane_size >= 2) {
                        for (auto id : cur_lane->next_lane_ids) {
                          if (id == curr_id) {
                            continue;
                          }
                          auto cur_lane_info_succ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(id);
                          if (!cur_lane_info_succ) {
                            cur_nextlane_size--;
                          } else {
                            if (cur_lane_info_succ->type == cem::message::env_model::LaneType::LANE_EMERGENCY ||
                                cur_lane_info_succ->type == cem::message::env_model::LaneType::LANE_DIVERSION ||
                                std::find(navi_lane_ids.begin(), navi_lane_ids.end(), cur_lane_info_succ->id) == navi_lane_ids.end()) {
                              cur_nextlane_size--;
                              continue;
                            }
                            auto cur_lane_ = cur_lane_info_succ;
                            int  flag_i    = 0;
                            while (cur_lane_) {
                              flag_i++;
                              if (flag_i == 8) {
                                break;
                              }
                              if (cur_lane_->next_lane_ids.size() == 1) {
                                if (std::find(navi_lane_ids.begin(), navi_lane_ids.end(), cur_lane_->id) != navi_lane_ids.end()) {
                                  cur_lane_ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(cur_lane_->next_lane_ids[0]);
                                } else {
                                  cur_nextlane_size--;
                                  break;
                                }

                              } else if (cur_lane_->next_lane_ids.size() == 0) {
                                break;
                              } else {
                                for (auto idd : cur_lane_->next_lane_ids) {
                                  auto lane_ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(idd);
                                  if (lane_ && lane_->next_lane_ids.size() == 1) {
                                    if (std::find(navi_lane_ids.begin(), navi_lane_ids.end(), lane_->next_lane_ids[0]) !=
                                        navi_lane_ids.end()) {
                                      continue;
                                    } else {
                                      cur_nextlane_size--;
                                      break;
                                    }
                                  } else {
                                    continue;
                                  }
                                }
                                break;
                              }
                            }
                          }
                        }
                      }
                      SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane_info.id:" << cur_lane->id << "cur_nextlane_size:" << cur_nextlane_size;
                      if (cur_nextlane_size == 2) {
                        one_to_two++;
                        SD_HIGHJUNCTION_CONVERTE_LOG << "one_to_two:" << one_to_two;
                        break;
                      } else if (cur_nextlane_size == 3) {
                        one_to_three++;
                        break;
                      } else if (cur_nextlane_size == 4) {
                        one_to_four++;
                        break;
                      } else {
                        //on code;
                      }
                    }
                    SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane->id:" << cur_lane->id;
                    SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane->previous_lane_ids.size():" << cur_lane->previous_lane_ids.size();
                    if (cur_lane->previous_lane_ids.size() <= 1) {
                      if (cur_lane->previous_lane_ids.size() == 1) {
                        pre_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(cur_lane->previous_lane_ids[0]);
                      } else {
                        pre_lane_info = nullptr;
                      }
                    } else {
                      break;
                    }
                  }
                }
              }
            }
            if (use_junctionoffset == 0) {
              if (junction_info.offset < 0) {
                for (auto lane_id : success_mppsection->lane_ids) {
                  auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
                  if (lane_info && lane_info->type != cem::message::env_model::LaneType::LANE_EMERGENCY &&
                      lane_info->type != cem::message::env_model::LaneType::LANE_DIVERSION) {
                    navi_lane_num[junction_info.junction_id]++;
                  }
                }
              } else {
                min_length           = junction_info.offset + 10;
                int    nums1         = 0;
                double one_to_two1   = 0;
                double one_to_three1 = 0;
                double one_to_four1  = 0;
                for (auto lane_id : success_mppsection->lane_ids) {
                  auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
                  int  j         = 0;
                  int  length    = 0;
                  if (lane_info && lane_info->type != cem::message::env_model::LaneType::LANE_EMERGENCY &&
                      lane_info->type != cem::message::env_model::LaneType::LANE_DIVERSION) {
                    SD_HIGHJUNCTION_CONVERTE_LOG << "junction_info.id:" << junction_info.junction_id;
                    if (lane_info->previous_lane_ids.size() == 1) {
                      auto pre_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_info->previous_lane_ids[0]);
                      auto cur_lane      = lane_info;
                      while (cur_lane && length < min_length && j < 100 && pre_lane_info && pre_lane_info->next_lane_ids.size() == 1 &&
                             pre_lane_info->next_lane_ids[0] == cur_lane->id && cur_lane->previous_lane_ids.size() == 1 &&
                             cur_lane->previous_lane_ids[0] == pre_lane_info->id) {
                        j++;
                        length = length + pre_lane_info->length;
                        SD_HIGHJUNCTION_CONVERTE_LOG << "lengh:" << length;
                        SD_HIGHJUNCTION_CONVERTE_LOG << "j:" << j;
                        uint64_t curr_id = cur_lane->id;
                        cur_lane = pre_lane_info;
                        if (length >= min_length || j >= 50) {
                          nums1++;
                          SD_HIGHJUNCTION_CONVERTE_LOG << "nums1" << nums1;
                          break;
                        } else {
                          int cur_nextlane_size = cur_lane->next_lane_ids.size();
                          if (cur_nextlane_size >= 2) {
                            for (auto id : cur_lane->next_lane_ids) {
                              if (id == curr_id) {
                                continue;
                              }
                              auto cur_lane_info_succ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(id);
                              if (!cur_lane_info_succ) {
                                cur_nextlane_size--;
                              } else {
                                if (cur_lane_info_succ->type == cem::message::env_model::LaneType::LANE_EMERGENCY ||
                                    cur_lane_info_succ->type == cem::message::env_model::LaneType::LANE_DIVERSION ||
                                    std::find(navi_lane_ids.begin(), navi_lane_ids.end(), cur_lane_info_succ->id) == navi_lane_ids.end()) {
                                  cur_nextlane_size--;
                                  continue;
                                }
                                auto cur_lane_ = cur_lane_info_succ;
                                int  flag_i    = 0;
                                while (cur_lane_) {
                                  flag_i++;
                                  if (flag_i == 8) {
                                    break;
                                  }
                                  if (cur_lane_->next_lane_ids.size() == 1) {
                                    if (std::find(navi_lane_ids.begin(), navi_lane_ids.end(), cur_lane_->id) != navi_lane_ids.end()) {
                                      cur_lane_ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(cur_lane_->next_lane_ids[0]);
                                    } else {
                                      cur_nextlane_size--;
                                      break;
                                    }

                                  } else if (cur_lane_->next_lane_ids.size() == 0) {
                                    break;
                                  } else {
                                    for (auto idd : cur_lane_->next_lane_ids) {
                                      auto lane_ = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(idd);
                                      if (lane_ && lane_->next_lane_ids.size() == 1) {
                                        if (std::find(navi_lane_ids.begin(), navi_lane_ids.end(), lane_->next_lane_ids[0]) !=
                                            navi_lane_ids.end()) {
                                          continue;
                                        } else {
                                          cur_nextlane_size--;
                                          break;
                                        }
                                      } else {
                                        continue;
                                      }
                                    }
                                    break;
                                  }
                                }
                              }
                            }
                          }
                          SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane_info.id:" << cur_lane->id << "cur_nextlane_size:" << cur_nextlane_size;
                          if (cur_nextlane_size == 2) {
                            one_to_two1++;
                            break;
                          } else if (cur_nextlane_size == 3) {
                            one_to_three1++;
                            break;
                          } else if (cur_nextlane_size == 4) {
                            one_to_four1++;
                            break;
                          } else {
                            //on code;
                          }
                        }
                        SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane->id:" << cur_lane->id;
                        SD_HIGHJUNCTION_CONVERTE_LOG << "cur_lane->previous_lane_ids.size():" << cur_lane->previous_lane_ids.size();
                        if (cur_lane->previous_lane_ids.size() <= 1) {
                          if (cur_lane->previous_lane_ids.size() == 1) {
                            pre_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(cur_lane->previous_lane_ids[0]);
                          } else {
                            pre_lane_info = nullptr;
                          }
                        } else {
                          break;
                        }
                      }
                    }
                  }
                }
                navi_lane_num[junction_info.junction_id] = nums1 + static_cast<int>(std::ceil(one_to_two1 / 2)) +
                                                           static_cast<int>(std::ceil(one_to_three1 / 3)) +
                                                           static_cast<int>(std::ceil(one_to_four1 / 4));
              }
            } else {
              navi_lane_num[junction_info.junction_id] = nums + static_cast<int>(std::ceil(one_to_two / 2.0)) +
                                                         static_cast<int>(std::ceil(one_to_three / 3.0)) +
                                                         static_cast<int>(std::ceil(one_to_four / 4.0));
              SD_HIGHJUNCTION_CONVERTE_LOG << "nums:" << nums << "std::ceil(one_to_two / 2.0):" << std::ceil(one_to_two / 2.0);
              SD_HIGHJUNCTION_CONVERTE_LOG << "use_junctionoffset:" << use_junctionoffset;
            }
          }
        }
        junction_info.main_road_navigation_lane_num = navi_lane_num[junction_info.junction_id];
        SD_HIGHJUNCTION_CONVERTE_LOG << "main_road_navigation_lane_num:" << junction_info.main_road_navigation_lane_num;
        auto SplitSuccInfo = GetSuccInfos(idx);
        if (SplitSuccInfo.size() > 1) {
          std::optional<size_t> min_angle_idx;
          std::optional<size_t> succ_idx;

          for (size_t idx_t = 0; idx_t < SplitSuccInfo.size(); ++idx_t) {
            const auto &info = SplitSuccInfo[idx_t];
            // AINFO << "info_id:" << info.section->id << " angle:" << info.angle << "  min_angle:" << info.min_angle
            //       << "  succ:" << info.next_suc;
            if (info.min_angle) {
              min_angle_idx = idx_t;
            }
            if (info.next_suc) {
              succ_idx = idx_t;
            }
          }

          if (min_angle_idx.has_value() && succ_idx.has_value()) {
            if (*min_angle_idx == *succ_idx) {
              junction_info.split_direction = SplitDirection::Straight;
            } else if (*min_angle_idx < *succ_idx) {
              junction_info.split_direction = SplitDirection::SplitRight;
            } else {
              junction_info.split_direction = SplitDirection::SplitLeft;
            }
            // if (idx < static_cast<int>(mpp_sections.size())) {
            //   junction_info.is_left_ahead_of_time = LinkTypeMaskType(mpp_sections[idx + 1].link_type, SDLinkTypeMask::SDLT_LEFTTURN);
            // }
          }
          // AINFO << "split_id:" << junction_info.junction_id << "  split_direction:" << static_cast<int>(junction_info.split_direction)
          // << "  left_time:" << junction_info.is_left_ahead_of_time;
        }

        junction_info.junction_ids.clear();
        junction_info.junction_ids.emplace_back(mpp_section.id);
        junction_info.junction_ids.emplace_back(mpp_split_section_id);

        sd_junction_infos_.push_back(junction_info);
        has_main_road_split_merge_flag = true;
      }
    }
    if (has_main_road_split_merge_flag) {
      continue;
    }
    // 非主路的分流
    if (!LD_IsCityRoadSection(mpp_section) &&
        (mpp_section.successor_section_id_list.size() == 2 || mpp_section.successor_section_id_list.size() == 3)) {
      uint64_t mpp_split_section_id      = 0;
      int      mpp_split_section_lanenum = 0;
      int      valid_subpath_num         = 0;
      for (auto &suc_section_id_tmp : mpp_section.successor_section_id_list) {
        auto suc_section_tmp = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(suc_section_id_tmp);
        if (suc_section_tmp) {
          // if(suc_section_tmp->direction != SDDirectionType::BIDIRECTIONAL_PASSABLE) {
          valid_subpath_num++;
          // }
          if (suc_section_tmp->is_mpp_section) {
            mpp_split_section_id      = suc_section_tmp->id;
            mpp_split_section_lanenum = suc_section_tmp->lane_ids.size();
          }
        }
      }
      if (valid_subpath_num >= 2) {
        std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
        std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
          use_topo_flag      = 0;
          auto section_tmp_a = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(a);
          auto section_tmp_b = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(b);
          if (section_tmp_a && section_tmp_b && section_tmp_a->points.size() >= 2 && section_tmp_b->points.size() >= 2) {
            Eigen::Vector2f section_tmp_a_points0(section_tmp_a->points.at(0).x, section_tmp_a->points.at(0).y);
            Eigen::Vector2f section_tmp_a_points1(section_tmp_a->points.at(1).x, section_tmp_a->points.at(1).y);
            Eigen::Vector2f section_tmp_b_points0(section_tmp_b->points.at(0).x, section_tmp_b->points.at(0).y);
            Eigen::Vector2f section_tmp_b_points1(section_tmp_b->points.at(1).x, section_tmp_b->points.at(1).y);
            // if (mpp_section.id == 96300977) {
            //   std::cout << "section_tmp_a.id:" << a << std::endl;
            //   std::cout << "section_tmp_a.x0::" << section_tmp_a->points.at(0).x << "section_tmp_a.y0::" << section_tmp_a->points.at(0).y
            //             << std::endl;
            //   std::cout << "section_tmp_a.x1::" << section_tmp_a->points.at(1).x << "section_tmp_a.y1::" << section_tmp_a->points.at(1).y
            //             << std::endl;

            //   std::cout << "section_tmp_b.id:" << b << std::endl;
            //   std::cout << "section_tmp_b.x0::" << section_tmp_b->points.at(0).x << "section_tmp_b.y0::" << section_tmp_b->points.at(0).y
            //             << std::endl;
            //   std::cout << "section_tmp_b.x1::" << section_tmp_b->points.at(1).x << "section_tmp_b.y1::" << section_tmp_b->points.at(1).y
            //             << std::endl;
            // }
            std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a_points0, section_tmp_a_points1};
            std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b_points0, section_tmp_b_points1};
            return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
          } else {
            use_topo_flag = 1;
          }
          return a < b;
        });
        if (use_topo_flag == 1) {
          SD_HIGHJUNCTION_CONVERTE_LOG << "FUNCTION rampsplit:";
          judge_left_or_rightsection(split_section_ids, mpp_section);
        }

        JunctionInfo junction_info;
        junction_info.junction_id = mpp_section.id;
        // if (mpp_section.id == 96300977) {
        //   for (auto splitid : split_section_ids) {
        //     std::cout << "split_section_ids::" << splitid << std::endl;
        //   }
        //   std::cout << "mpp_split_section_id:" << mpp_split_section_id << std::endl;
        // }
        if (mpp_split_section_id == split_section_ids.front()) {
          junction_info.junction_type = JunctionType::RampSplitLeft;
        } else if (mpp_split_section_id == split_section_ids.back()) {
          junction_info.junction_type = JunctionType::RampSplitRight;
        } else if (split_section_ids.size() == 3 && mpp_split_section_id == split_section_ids[1]) {
          junction_info.junction_type = JunctionType::RampSplitMiddle;
        }
        junction_info.split_num             = split_section_ids.size();
        junction_info.offset                = all_mpp_length - ego_dist_s;
        junction_info.main_road_lane_nums   = mpp_section.lane_ids.size();
        junction_info.target_road_lane_nums = mpp_split_section_lanenum;

        auto SplitSuccInfo = GetSuccInfos(idx);
        if (SplitSuccInfo.size() > 1) {
          std::optional<size_t> min_angle_idx;
          std::optional<size_t> succ_idx;

          for (size_t idx_t = 0; idx_t < SplitSuccInfo.size(); ++idx_t) {
            const auto &info = SplitSuccInfo[idx_t];
            // AINFO << "info_id:" << info.section->id << " angle:" << info.angle << "  min_angle:" << info.min_angle
            //       << "  succ:" << info.next_suc;
            if (info.min_angle) {
              min_angle_idx = idx_t;
            }
            if (info.next_suc) {
              succ_idx = idx_t;
            }
          }

          if (min_angle_idx.has_value() && succ_idx.has_value()) {
            if (*min_angle_idx == *succ_idx) {
              junction_info.split_direction = SplitDirection::Straight;
            } else if (*min_angle_idx < *succ_idx) {
              junction_info.split_direction = SplitDirection::SplitRight;
            } else {
              junction_info.split_direction = SplitDirection::SplitLeft;
            }
            if (idx < static_cast<int>(mpp_sections.size())) {
              junction_info.is_left_ahead_of_time = LinkTypeMaskType(mpp_sections[idx + 1].link_type, SDLinkTypeMask::SDLT_LEFTTURN);
            }
          }
          // AINFO << "split_id:" << junction_info.junction_id << "  split_direction:" << static_cast<int>(junction_info.split_direction)
          // << "  left_time:" << junction_info.is_left_ahead_of_time;
        }

        junction_info.junction_ids.clear();
        junction_info.junction_ids.emplace_back(mpp_section.id);
        junction_info.junction_ids.emplace_back(mpp_split_section_id);

        sd_junction_infos_.push_back(junction_info);
      }
    }
  }

  /// 下匝道前相邻太近的匝道口过滤掉
  if (sd_junction_infos_.size() > 1) {
    std::set<uint64_t> deleted_section_ids = {};
    for (unsigned i = 1; i < sd_junction_infos_.size(); i++) {
      if (sd_junction_infos_[i].junction_type == JunctionType::RampInto) {
        if (sd_junction_infos_[i - 1].junction_type == JunctionType::ApproachRampInto) {
          if (fabs(sd_junction_infos_[i].offset - sd_junction_infos_[i - 1].offset) < 60) {
            deleted_section_ids.insert(sd_junction_infos_[i - 1].junction_id);
          }
        }
      }
    }
    if (!deleted_section_ids.empty()) {
      for (auto it = sd_junction_infos_.begin(); it != sd_junction_infos_.end();) {
        if (deleted_section_ids.find(it->junction_id) != deleted_section_ids.end()) {
          it = sd_junction_infos_.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}
// void SdNavigationHighway::GetSDJunctionInfos() {
//   auto sd_map_info = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
//   if (!sd_map_info) {
//     return;
//   }
//
//   double ego_dist_s         = 0;
//   double section_length_tmp = 0;
//   ego_section_id_           = sd_map_info->navi_start.section_id;
//   for (auto &mpp_section : sd_map_info->mpp_sections) {
//     if (mpp_section.id == sd_map_info->navi_start.section_id) {
//       ego_dist_s = section_length_tmp + sd_map_info->navi_start.s_offset;
//       break;
//     }
//     section_length_tmp += mpp_section.length;
//   }
//
//   double all_mpp_length = 0;
//
//   const auto                               &mpp_sections = sd_map_info->mpp_sections;
//   std::map<uint64_t, const SDSectionInfo *> section_map;
//   for (const auto &section : mpp_sections) {
//     section_map.insert({section.id, &section});
//   }
//   for (const auto &subpath : sd_map_info->subpaths) {
//     for (const auto &section : subpath.sections) {
//       section_map.insert({section.id, &section});
//     }
//   }
//
//   auto GetSuccInfos = [&](int idx) -> std::vector<SDMapElementExtract::AngleJunction> {
//     if (idx < 0 || idx + 1 >= static_cast<int>(mpp_sections.size())) {
//       return {};
//     }
//     const auto &section_curr = mpp_sections[idx];
//     const auto &section_next = mpp_sections[idx + 1];
//     if (section_curr.successor_section_id_list.size() <= 1) {
//       return {};
//     }
//     std::vector<SDMapElementExtract::AngleJunction> res;
//
//     double      min_angle = std::numeric_limits<double>::infinity();
//     std::size_t min_idx   = 0;
//     for (const auto &succ_id : section_curr.successor_section_id_list) {
//       auto it =
//           std::find_if(section_map.begin(), section_map.end(), [&](const auto &rhs) { return rhs.second && rhs.second->id == succ_id; });
//       if (static_cast<int>(succ_id / 10) == static_cast<int>(section_curr.id / 10)) {
//         // AINFO << "skip_same_road_id:" << succ_id;
//         continue;
//       }
//       if (it == section_map.end()) {
//         continue;
//       }
//       auto angle_diff = AngleDiffBetweenSection(section_curr, *it->second);
//       if (!angle_diff) {
//         continue;
//       }
//       auto &succ_info    = res.emplace_back();
//       succ_info.section  = std::make_shared<SDSectionInfo>(*it->second);
//       succ_info.next_suc = succ_id == section_next.id;
//       succ_info.angle    = angle_diff.value();
//       if (min_angle > std::fabs(angle_diff.value())) {
//         min_angle = std::fabs(angle_diff.value());
//         min_idx   = res.size() - 1;
//       }
//     }
//     if (res.size() > 1) {
//       res[min_idx].min_angle = true;
//       SDMapElementExtract::JunctionAngleSectionDirection(section_curr, res);
//       std::sort(res.begin(), res.end(), [](const auto &lhs, const auto &rhs) { return lhs.angle > rhs.angle; });
//     }
//     return res;
//   };
//
//   int idx = -1;
//   for (auto &mpp_section : sd_map_info->mpp_sections) {
//     idx++;
//     all_mpp_length += mpp_section.length;
//
//     if (IsCityRoadSection(mpp_section)) {
//       if (all_mpp_length - ego_dist_s <= 0) {
//         sd_junction_infos_.clear();
//         continue;
//       } else {
//         break;
//       }
//     }
//
//     /// 下匝道
//     bool has_ramp_into_flag = false;
//     if (IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 2) {
//       for (auto &suc_section_id_tmp : mpp_section.successor_section_id_list) {
//         auto suc_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(suc_section_id_tmp);
//         if (suc_section_tmp) {
//           if (suc_section_tmp->is_mpp_section && IsRampSection(*suc_section_tmp)) {
//             std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
//             std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
//               auto section_tmp_a = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(a);
//               auto section_tmp_b = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(b);
//               if (section_tmp_a && section_tmp_b && section_tmp_a->points->size() >= 2 && section_tmp_b->points->size() >= 2) {
//                 std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a->points->at(0), section_tmp_a->points->at(1)};
//                 std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b->points->at(0), section_tmp_b->points->at(1)};
//                 return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
//               }
//               return a < b;
//             });
//
//             if (split_section_ids.back() == suc_section_tmp->id) {
//               JunctionInfo junction_info;
//               junction_info.junction_id           = mpp_section.id;
//               junction_info.junction_type         = JunctionType::RampInto;
//               junction_info.offset                = all_mpp_length - ego_dist_s;
//               junction_info.main_road_lane_nums   = GetSDSectionMinLaneNumNOTEmergency(mpp_section);
//               junction_info.target_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(*suc_section_tmp);
//               junction_info.split_num             = 2;
//               junction_info.junction_ids.clear();
//               junction_info.junction_ids.emplace_back(mpp_section.id);
//               junction_info.junction_ids.emplace_back(suc_section_id_tmp);
//               sd_junction_infos_.push_back(junction_info);
//               has_ramp_into_flag = true;
//               break;
//             }
//           }
//         }
//       }
//     }
//     if (has_ramp_into_flag) {
//       continue;
//     }
//
//     /// 途径下匝道
//     if (IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 2) {
//       auto suc_section_tmp1 = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(mpp_section.successor_section_id_list.front());
//       auto suc_section_tmp2 = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(mpp_section.successor_section_id_list.back());
//       if (suc_section_tmp1 && suc_section_tmp2) {
//         int main_road_index = 0;
//         if (suc_section_tmp2->is_mpp_section && IsRampSection(*suc_section_tmp1) && IsMainRoadSection(*suc_section_tmp2)) {
//           main_road_index = 2;
//         }
//         if (suc_section_tmp1->is_mpp_section && IsRampSection(*suc_section_tmp2) && IsMainRoadSection(*suc_section_tmp1)) {
//           main_road_index = 1;
//         }
//
//         std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
//         std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
//           auto section_tmp_a = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(a);
//           auto section_tmp_b = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(b);
//           if (section_tmp_a && section_tmp_b && section_tmp_a->points->size() >= 2 && section_tmp_b->points->size() >= 2) {
//             std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a->points->at(0), section_tmp_a->points->at(1)};
//             std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b->points->at(0), section_tmp_b->points->at(1)};
//             return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
//           }
//           return a < b;
//         });
//
//         if (main_road_index != 0 && mpp_section.successor_section_id_list[main_road_index - 1] == split_section_ids.front()) {
//           JunctionInfo junction_info;
//           junction_info.junction_id         = mpp_section.id;
//           junction_info.junction_type       = JunctionType::ApproachRampInto;
//           junction_info.offset              = all_mpp_length - ego_dist_s;
//           junction_info.main_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(mpp_section);
//           if (main_road_index == 1) {
//             junction_info.target_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(*suc_section_tmp1);
//           } else if (main_road_index == 2) {
//             junction_info.target_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(*suc_section_tmp2);
//           }
//           junction_info.split_num = 2;
//
//           junction_info.junction_ids.clear();
//           junction_info.junction_ids.emplace_back(mpp_section.id);
//           if (main_road_index == 1) {
//             junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());
//           } else if (main_road_index == 2) {
//             junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.back());
//           }
//
//           sd_junction_infos_.push_back(junction_info);
//           continue;
//         }
//       }
//     }
//
//     /// 匝道汇入
//     bool has_ramp_merge_flag = false;
//     if (IsRampSection(mpp_section) && mpp_section.successor_section_id_list.size() == 1) {
//       auto suc_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(mpp_section.successor_section_id_list.front());
//       if (suc_section_tmp && IsMainRoadSection(*suc_section_tmp) && suc_section_tmp->is_mpp_section) {
//         if (suc_section_tmp->predecessor_section_id_list.size() == 2) {
//           for (auto &merge_id_tmp : suc_section_tmp->predecessor_section_id_list) {
//             if (merge_id_tmp != mpp_section.id) {
//               auto merge_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(merge_id_tmp);
//               if (merge_section_tmp && IsMainRoadSection(*merge_section_tmp) && !merge_section_tmp->is_mpp_section) {
//                 JunctionInfo junction_info;
//                 junction_info.junction_id           = mpp_section.id;
//                 junction_info.junction_type         = JunctionType::RampMerge;
//                 junction_info.offset                = all_mpp_length - ego_dist_s;
//                 junction_info.main_road_lane_nums   = GetSDSectionMinLaneNumNOTEmergency(*merge_section_tmp);
//                 junction_info.target_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(*suc_section_tmp);
//
//                 junction_info.junction_ids.clear();
//                 junction_info.junction_ids.emplace_back(mpp_section.id);
//                 junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());
//
//                 sd_junction_infos_.push_back(junction_info);
//                 has_ramp_merge_flag = true;
//                 break;
//               }
//             }
//           }
//         }
//       }
//     }
//     if (has_ramp_merge_flag) {
//       continue;
//     }
//
//     /// 途径匝道汇入
//     bool has_approach_ramp_merge_flag = false;
//     if (IsMainRoadSection(mpp_section) && mpp_section.successor_section_id_list.size() == 1) {
//       auto suc_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(mpp_section.successor_section_id_list.front());
//       if (suc_section_tmp && IsMainRoadSection(*suc_section_tmp) && suc_section_tmp->is_mpp_section) {
//         if (suc_section_tmp->predecessor_section_id_list.size() == 2) {
//           for (auto &merge_id_tmp : suc_section_tmp->predecessor_section_id_list) {
//             if (merge_id_tmp != mpp_section.id) {
//               auto merge_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(merge_id_tmp);
//               if (merge_section_tmp && !merge_section_tmp->is_mpp_section && IsRampSection(*merge_section_tmp)) {
//                 JunctionInfo junction_info;
//                 junction_info.junction_id           = mpp_section.id;
//                 junction_info.junction_type         = JunctionType::ApproachRampMerge;
//                 junction_info.offset                = all_mpp_length - ego_dist_s;
//                 junction_info.main_road_lane_nums   = GetSDSectionMinLaneNumNOTEmergency(mpp_section);
//                 junction_info.target_road_lane_nums = GetSDSectionMinLaneNumNOTEmergency(*suc_section_tmp);
//
//                 junction_info.junction_ids.clear();
//                 junction_info.junction_ids.emplace_back(mpp_section.id);
//                 junction_info.junction_ids.emplace_back(mpp_section.successor_section_id_list.front());
//
//                 sd_junction_infos_.push_back(junction_info);
//                 has_approach_ramp_merge_flag = true;
//                 break;
//               }
//             }
//           }
//         }
//       }
//     }
//     if (has_approach_ramp_merge_flag) {
//       continue;
//     }
//
//     /// 非主路的分流
//     if (!IsCityRoadSection(mpp_section) &&
//         (mpp_section.successor_section_id_list.size() == 2 || mpp_section.successor_section_id_list.size() == 3)) {
//       uint64_t mpp_split_section_id      = 0;
//       int      mpp_split_section_lanenum = 0;
//       int      valid_subpath_num         = 0;
//       for (auto &suc_section_id_tmp : mpp_section.successor_section_id_list) {
//         auto suc_section_tmp = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(suc_section_id_tmp);
//         if (suc_section_tmp) {
//           if (suc_section_tmp->direction != SDDirectionType::BIDIRECTIONAL_PASSABLE) {
//             valid_subpath_num++;
//           }
//           if (suc_section_tmp->is_mpp_section) {
//             mpp_split_section_id      = suc_section_tmp->id;
//             mpp_split_section_lanenum = suc_section_tmp->lane_num;
//           }
//         }
//       }
//       if (valid_subpath_num >= 2) {
//         std::vector<uint64_t> split_section_ids = mpp_section.successor_section_id_list;
//         std::sort(split_section_ids.begin(), split_section_ids.end(), [&](uint64_t a, uint64_t b) {
//           auto section_tmp_a = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(a);
//           auto section_tmp_b = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(b);
//           if (section_tmp_a && section_tmp_b && section_tmp_a->points->size() >= 2 && section_tmp_b->points->size() >= 2) {
//             std::vector<Eigen::Vector2f> section_pt_a = {section_tmp_a->points->at(0), section_tmp_a->points->at(1)};
//             std::vector<Eigen::Vector2f> section_pt_b = {section_tmp_b->points->at(0), section_tmp_b->points->at(1)};
//             return LaneGeometry::JudgeIsLeftUseRawPt(section_pt_a, section_pt_b);
//           }
//           return a < b;
//         });
//
//         JunctionInfo junction_info;
//         junction_info.junction_id = mpp_section.id;
//         if (mpp_split_section_id == split_section_ids.front()) {
//           junction_info.junction_type = JunctionType::RampSplitLeft;
//         } else if (mpp_split_section_id == split_section_ids.back()) {
//           junction_info.junction_type = JunctionType::RampSplitRight;
//         } else if (split_section_ids.size() == 3 && mpp_split_section_id == split_section_ids[1]) {
//           junction_info.junction_type = JunctionType::RampSplitMiddle;
//         }
//         junction_info.split_num             = split_section_ids.size();
//         junction_info.offset                = all_mpp_length - ego_dist_s;
//         junction_info.main_road_lane_nums   = mpp_section.lane_num;
//         junction_info.target_road_lane_nums = mpp_split_section_lanenum;
//
//         auto SplitSuccInfo = GetSuccInfos(idx);
//         if (SplitSuccInfo.size() > 1) {
//           std::optional<size_t> min_angle_idx;
//           std::optional<size_t> succ_idx;
//
//           for (size_t idx_t = 0; idx_t < SplitSuccInfo.size(); ++idx_t) {
//             const auto &info = SplitSuccInfo[idx_t];
//             // AINFO << "info_id:" << info.section->id << " angle:" << info.angle << "  min_angle:" << info.min_angle
//             //       << "  succ:" << info.next_suc;
//             if (info.min_angle) {
//               min_angle_idx = idx_t;
//             }
//             if (info.next_suc) {
//               succ_idx = idx_t;
//             }
//           }
//
//           if (min_angle_idx.has_value() && succ_idx.has_value()) {
//             if (*min_angle_idx == *succ_idx) {
//               junction_info.split_direction = SplitDirection::Straight;
//             } else if (*min_angle_idx < *succ_idx) {
//               junction_info.split_direction = SplitDirection::SplitRight;
//             } else {
//               junction_info.split_direction = SplitDirection::SplitLeft;
//             }
//             if (idx < static_cast<int>(mpp_sections.size())) {
//               junction_info.is_left_ahead_of_time = LinkTypeMaskType(mpp_sections[idx + 1].link_type, SDLinkTypeMask::SDLT_LEFTTURN);
//             }
//           }
//           // AINFO << "split_id:" << junction_info.junction_id << "  split_direction:" << static_cast<int>(junction_info.split_direction)
//           // << "  left_time:" << junction_info.is_left_ahead_of_time;
//         }
//
//         junction_info.junction_ids.clear();
//         junction_info.junction_ids.emplace_back(mpp_section.id);
//         junction_info.junction_ids.emplace_back(mpp_split_section_id);
//
//         sd_junction_infos_.push_back(junction_info);
//       }
//     }
//   }
//
//   /// 下匝道前相邻太近的匝道口过滤掉
//   if (sd_junction_infos_.size() > 1) {
//     std::set<uint64_t> deleted_section_ids = {};
//     for (unsigned i = 1; i < sd_junction_infos_.size(); i++) {
//       if (sd_junction_infos_[i].junction_type == JunctionType::RampInto) {
//         if (sd_junction_infos_[i - 1].junction_type == JunctionType::ApproachRampInto) {
//           if (fabs(sd_junction_infos_[i].offset - sd_junction_infos_[i - 1].offset) < 60) {
//             deleted_section_ids.insert(sd_junction_infos_[i - 1].junction_id);
//           }
//         }
//       }
//     }
//     if (!deleted_section_ids.empty()) {
//       for (auto it = sd_junction_infos_.begin(); it != sd_junction_infos_.end();) {
//         if (deleted_section_ids.find(it->junction_id) != deleted_section_ids.end()) {
//           it = sd_junction_infos_.erase(it);
//         } else {
//           ++it;
//         }
//       }
//     }
//   }
// }

void SdNavigationHighway::FusionCurrentBevEmergencyLaneid(const std::vector<uint64_t> &target_section,
                                                          const EmergencyLaneInfo     &sd_emergency_lane_info,
                                                          const int &selected_section_index, uint64_t &current_left_bev_emergency_laneid,
                                                          uint64_t &current_right_bev_emergency_laneid) {
  std::ostringstream right_range;
  std::ostringstream left_range;
  auto              &navi_debug_info = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  for (auto &range_tmp : sd_emergency_lane_info.right.ranges) {
    right_range << "[" << range_tmp.first << "," << range_tmp.second << "]";
  }
  for (auto &range_tmp : sd_emergency_lane_info.left.ranges) {
    left_range << "[" << range_tmp.first << "," << range_tmp.second << "]";
  }
  std::pair<double, double> firstRange = {0, 0};
  if (g_emergency_lane_info_.right.exists) {
    // 获取第一个 应急车道范围 [start_s, end_s] CNOAC2-124418
    for (auto range : g_emergency_lane_info_.right.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
  }
  SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] SDMAP RES  R:" << right_range.str() << "; L:" << left_range.str();

  auto SDHasEMLane = [](const BevLaneInfo *bev_lane_info, const std::vector<std::pair<double, double>> &sd_ranges) -> bool {
    if (!bev_lane_info && bev_lane_info->geos->empty()) {
      return false;
    }
    float start_x             = bev_lane_info->geos->front().x();
    float end_x               = bev_lane_info->geos->back().x();
    float intersection_length = 0;
    for (auto &range_tmp : sd_ranges) {
      SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] range_tmp.first: " << range_tmp.first
                          << "  range_tmp.second: " << range_tmp.second;
      if (range_tmp.first < end_x && range_tmp.second > start_x) {
        if (range_tmp.first < start_x) {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - start_x);
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] intersection_length: " << intersection_length;
          } else {
            intersection_length += fabs(end_x - start_x);
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] intersection_length: " << intersection_length;
          }
        } else {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - range_tmp.first);
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] intersection_length: " << intersection_length;
          } else {
            intersection_length += fabs(end_x - range_tmp.first);
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] intersection_length: " << intersection_length;
          }
        }
      }
    }
    if (fabs(end_x - start_x) > 5) {
      float rate = intersection_length / fabs(end_x - start_x);
      SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] start_x: " << start_x << "end_x: " << end_x;
      SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] intersection_length: " << intersection_length << "/end_x - start_x "
                          << end_x - start_x << " = " << rate;
      if (rate > 0.5) {
        SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] rate: " << rate << " > " << 0.5 << " SDHasEMLane: is true. ";
        return true;
      }
    }
    return false;
  };

  ////Right Emergency Lane
  {
    auto rightmost = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(target_section.back());
    if (rightmost) {
      /// 感知检出应急车道
      if (rightmost->lane_type == BevLaneType::LANE_TYPE_EMERGENCY) {
        current_right_bev_emergency_laneid = target_section.back();
        navi_debug_info.bev_emergency_id   = target_section.back();
        SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : the right most lane is bev emergency lane.";

        /// 感知没有检出应急车道
      } else {
        if (rightmost->position != (int)BevLanePosition::LANE_LOC_EGO && rightmost->left_lane_marker_id != 0 &&
            rightmost->right_lane_marker_id != 0 && rightmost->previous_lane_ids.size() < 2 && firstRange.first < -20 &&
            firstRange.second > 0) {
          SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] rightmost valid check ok.."
                              << "  rightmost->position: " << rightmost->position;
          if (SDHasEMLane(rightmost, sd_emergency_lane_info.right.ranges)) {
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] SDHasEMLane check ok > 50%  bev lane overlap..";
            bool valid_flag          = true;
            auto left_lane_boundary  = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(rightmost->left_lane_marker_id);
            auto right_lane_boundary = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(rightmost->right_lane_marker_id);
            if (left_lane_boundary) {
              int dash_line_count = 0;
              int all_index_count = 0;
              for (auto &type_tmp : left_lane_boundary->type_segs) {
                all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
                if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED || type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
                  dash_line_count += fabs(type_tmp.end_index - type_tmp.start_index);
                }
              }
              if (all_index_count != 0 && (float)dash_line_count / (float)all_index_count > 0.2) {
                valid_flag = false;
                SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] emergency lane check fail cause > 20%  bev lane dashline..";
              }
            }
            if (right_lane_boundary) {
              double dist2rightRoadEdge = 9999;
              auto   bev_map_ptr        = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
              if (bev_map_ptr) {
                for (auto &road_edge : bev_map_ptr->edges) {
                  double gap_right_boundary = LaneGeometry::GetDistanceBetweenLinesThin(*road_edge.geos, *right_lane_boundary->geos);
                  if (gap_right_boundary != 9999 && gap_right_boundary < dist2rightRoadEdge) {
                    dist2rightRoadEdge = gap_right_boundary;
                  }
                }
              }
              SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : the dis right most lane to right bdry: " << dist2rightRoadEdge;
              if (dist2rightRoadEdge > 1) {
                valid_flag = false;
              }
            }
            if (valid_flag) {
              current_right_bev_emergency_laneid = target_section.back();
              SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : emergency_laneid: " << current_right_bev_emergency_laneid;
            }
          }
        }
      }
    }
  }

  ////Left Emergency Lane
  {
    auto leftmost = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(target_section.front());
    if (leftmost) {
      SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] leftmost->position: " << leftmost->position
                          << "selected_section_index: " << selected_section_index;
      /// 感知检出应急车道
      if (leftmost->lane_type == BevLaneType::LANE_TYPE_EMERGENCY) {
        current_left_bev_emergency_laneid = target_section.front();
        navi_debug_info.bev_emergency_id  = target_section.front();
        SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : the left most lane is bev emergency lane.";

        /// 感知没有检出应急车道 //CNOAC2-65271
      } else {
        if (leftmost->position != (int)BevLanePosition::LANE_LOC_EGO && leftmost->left_lane_marker_id != 0 && selected_section_index == 0 &&
            leftmost->right_lane_marker_id != 0 && leftmost->previous_lane_ids.size() < 2) {
          SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] leftmost->position: " << leftmost->position << " is not ego : 0";
          if (SDHasEMLane(leftmost, sd_emergency_lane_info.left.ranges)) {
            SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] SDHasEMLane check ok > 50%  bev lane overlap..";
            bool valid_flag          = true;
            auto left_lane_boundary  = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(leftmost->left_lane_marker_id);
            auto right_lane_boundary = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(leftmost->right_lane_marker_id);
            if (right_lane_boundary) {
              int dash_line_count = 0;
              int all_index_count = 0;
              for (auto &type_tmp : right_lane_boundary->type_segs) {
                all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
                if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED || type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
                  dash_line_count += fabs(type_tmp.end_index - type_tmp.start_index);
                }
              }
              if (all_index_count != 0 && (float)dash_line_count / (float)all_index_count > 0.2) {
                valid_flag = false;
                SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] emergency lane check fail cause > 20%  bev lane dashline..";
              }
            }
            if (left_lane_boundary) {
              double dist2leftRoadEdge = 9999;
              auto   bev_map_ptr       = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
              if (bev_map_ptr) {
                for (auto &road_edge : bev_map_ptr->edges) {
                  double gap_left_boundary = LaneGeometry::GetDistanceBetweenLinesThin(*road_edge.geos, *left_lane_boundary->geos);
                  if (gap_left_boundary != 9999 && gap_left_boundary < dist2leftRoadEdge) {
                    dist2leftRoadEdge = gap_left_boundary;
                  }
                }
              }
              SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : the dis left most lane to left bdry: " << dist2leftRoadEdge;
              if (dist2leftRoadEdge > 1) {
                valid_flag = false;
              }
            }
            if (valid_flag) {
              current_left_bev_emergency_laneid = target_section.front();
              SD_COARSE_MATCH_LOG << "[FusionCurrentBevEmergencyLaneid] : emergency_laneid: " << current_left_bev_emergency_laneid;
            }
          }
        }
      }
    }
  }
}

int SdNavigationHighway::GetLDSectionMinLaneNumNOTEmergency(const cem::message::env_model::SectionInfo &section_info) {
  if (section_info.lane_ids.empty()) {
    return section_info.lane_ids.size() != 0 ? section_info.lane_ids.size() : 0;
  }
  int min_lane_num         = std::numeric_limits<int>::max();
  int emergency_lane_count = 0;
  for (auto &lane_id_tmp : section_info.lane_ids) {
    auto lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id_tmp);
    if (lane) {
      if (lane->type == LaneType::LANE_EMERGENCY || lane->type == LaneType::LANE_DIVERSION || lane->type == LaneType::LANE_NON_MOTOR) {
        emergency_lane_count++;
      }
    }
  }
  int current_lane_num = section_info.lane_ids.size() - emergency_lane_count;
  if (current_lane_num < min_lane_num) {
    min_lane_num = current_lane_num;
  }
  if (min_lane_num == std::numeric_limits<int>::max()) {
    return section_info.lane_ids.size() != 0 ? section_info.lane_ids.size() : 0;
  }
  if (min_lane_num == 0) {
    return section_info.lane_ids.size() != 0 ? section_info.lane_ids.size() : 0;
  } else {
    return min_lane_num;
  }
}

// int SdNavigationHighway::GetSDSectionMinLaneNumNOTEmergency(const SDSectionInfo &section_info) {
//   if (section_info.lane_group_idx.empty()) {
//     return section_info.lane_num != 0 ? section_info.lane_num : 0;
//   }
//   int min_lane_num = std::numeric_limits<int>::max();
//   for (auto &lane_group_id_tmp : section_info.lane_group_idx) {
//     auto lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lane_group_id_tmp.id);
//     if (lane_group) {
//       int emergency_lane_count = 0;
//       for (auto &lane_info_tmp : lane_group->lane_info) {
//         if (lane_info_tmp.type == LaneType::LANE_EMERGENCY) {
//           emergency_lane_count++;
//         }
//       }
//       int current_lane_num = lane_group->lane_info.size() - emergency_lane_count;
//       if (current_lane_num < min_lane_num) {
//         min_lane_num = current_lane_num;
//       }
//     }
//   }
//   if (min_lane_num == std::numeric_limits<int>::max()) {
//     return section_info.lane_num != 0 ? section_info.lane_num : 0;
//   }
//   if (min_lane_num == 0) {
//     return section_info.lane_num != 0 ? section_info.lane_num : 0;
//   } else {
//     return min_lane_num;
//   }
// }

void SdNavigationHighway::SetLdRecommendLane() {
  const auto ld_map_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_map_info) {
    // SD_FINE_MATCH_LOG << "ld_map_info is null";
    return;
  }
  const auto &mpp_sections = ld_map_info->sections;
  if (mpp_sections.empty()) {
    // SD_FINE_MATCH_LOG << "mpp_sections is empty";
    return;
  }
  uint64_t   ego_section_id  = ld_map_info->navi_start.section_id;
  const auto ego_section     = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(ego_section_id);
  int        ego_section_idx = -1;
  if (!ego_section)
    return;
  for (size_t i = 0; i < mpp_sections.size(); i++) {
    if (mpp_sections[i].id == ego_section->id) {
      ego_section_idx = static_cast<int>(i);
      break;
    }
  }
  if (ego_section_idx == -1)
    return;
  double ego_s_offset = ld_map_info->navi_start.s_offset;
  double ego_s_global = 0.0;
  for (int i = 0; i < ego_section_idx; ++i) {
    ego_s_global += mpp_sections[i].length;
  }
  ego_s_global += ego_s_offset;
  SD_FINE_MATCH_LOG << "[GetLdRecommendLane] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset
                    << ", Ego s_global: " << ego_s_global;

  double current_s = 0.0;
  double min_s     = -50;
  double max_s     = 200;
  for (size_t i = 0; i < mpp_sections.size(); ++i) {
    const auto &section         = mpp_sections.at(i);
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    if (section_end_s < ego_s_global + min_s || section_start_s > ego_s_global + max_s) {
      current_s += section.length;
      continue;
    }
    SD_FINE_MATCH_LOG << "Section ID: " << section.id << " start from s: " << section_start_s << " to: " << section_end_s;
    const auto                            &lane_id = section.lane_ids;
    cem::fusion::navigation::RecommendLane recommend_info;
    if (i >= 1) {
      const auto &prev_section  = mpp_sections.at(i - 1);
      const auto &curr_section  = mpp_sections.at(i);
      recommend_info.section_id = curr_section.id;
      std::unordered_set<uint64_t> prev_lane_set(prev_section.lane_ids.begin(), prev_section.lane_ids.end());

      for (uint64_t curr_lane_id : curr_section.lane_ids) {
        auto curr_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(curr_lane_id);
        if (!curr_lane_info) {
          SD_ENV_INFO_LOG << fmt::format("Get curr_lane_id: {} failed.", curr_lane_id);
          continue;
        }
        auto prev_lanes = curr_lane_info->previous_lane_ids;
        for (uint64_t prev_lane : prev_lanes) {
          if (prev_lane_set.count(prev_lane)) {
            recommend_info.lane_ids.emplace_back(curr_lane_id);
            recommend_info.lane_seqs.emplace_back(curr_lane_info->lane_seq);
            break;
          }
        }
      }
      recommend_info.lane_num                             = recommend_info.lane_ids.size();
      recommend_info.start_offset                         = section_start_s - ego_s_global;
      recommend_info.end_offset                           = section_end_s - ego_s_global;
      map_ld_recommend_lane_[recommend_info.start_offset] = recommend_info;
    }
    current_s += section.length;
  }

  for (const auto &recommend_lane : map_ld_recommend_lane_) {
    const double rm_start_ego   = recommend_lane.first;
    const auto  &recommend_info = recommend_lane.second;
    SD_FINE_MATCH_LOG << "section_id = " << recommend_info.section_id << ", lane_start_offset = " << recommend_info.start_offset
                      << ", lane_end_offset = " << recommend_info.end_offset << ", lane_num = " << recommend_info.lane_num;
    for (const auto &lane_id : recommend_info.lane_ids) {
      SD_FINE_MATCH_LOG << "lane_id: " << lane_id;
    }
    for (const auto &lane_seq : recommend_info.lane_seqs) {
      SD_FINE_MATCH_LOG << "lane_seq: " << lane_seq;
    }
  }

  if (map_ld_recommend_lane_.empty())
    return;
  auto it      = map_ld_recommend_lane_.begin();
  auto prev_it = it++;
  for (; it != map_ld_recommend_lane_.end();) {
    const RecommendLane &prev_lane = prev_it->second;
    const RecommendLane &curr_lane = it->second;
    if (prev_lane.lane_num == curr_lane.lane_num && prev_lane.lane_seqs == curr_lane.lane_seqs) {
      it = map_ld_recommend_lane_.erase(it);
    } else {
      prev_it = it++;
    }
  }

  SD_FINE_MATCH_LOG << "Merge Duplicate RecommendLanes";
  for (const auto &recommend_lane : map_ld_recommend_lane_) {
    const double rm_start_ego   = recommend_lane.first;
    const auto  &recommend_info = recommend_lane.second;
    SD_FINE_MATCH_LOG << "section_id = " << recommend_info.section_id << ", lane_start_offset = " << recommend_info.start_offset
                      << ", lane_end_offset = " << recommend_info.end_offset << ", lane_num = " << recommend_info.lane_num;
    for (const auto &lane_id : recommend_info.lane_ids) {
      SD_FINE_MATCH_LOG << "lane_id: " << lane_id;
    }
    for (const auto &lane_id : recommend_info.lane_seqs) {
      SD_FINE_MATCH_LOG << "lane_seqs: " << lane_id;
    }
  }
}

void SdNavigationHighway::SelectGuideLanesWithLd(const std::vector<uint64_t>                           &bev_ego_root_section_x0,
                                                 const std::unordered_map<double, RecommendLane>       &ld_recommend_lane,
                                                 BevMapInfoPtr                                         &GlobalBevMapOutPut,
                                                 std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                                 std::vector<uint64_t>                                 &sd_guide_lane_ids) {
  std::set<uint64_t> bev_guide_lanes_with_ld;
  guide_lane_result.clear();
  sd_guide_lane_ids.clear();
  if (bev_ego_root_section_x0.empty() || ld_recommend_lane.empty()) {
    SD_FINE_MATCH_LOG << "bev or ld is empty, return: " << bev_ego_root_section_x0.empty() << ", " << ld_recommend_lane.empty();
    return;
  }

  for (auto &bev_ego_id : bev_ego_root_section_x0) {
    SD_FINE_MATCH_LOG << "bev_ego_id: " << bev_ego_id;
  }

  RecommendLane ld_target_lane;
  bool          is_found = false;
  for (auto it = ld_recommend_lane.begin(); it != ld_recommend_lane.end(); ++it) {
    if (it->first < 0.0) {
      auto next_it = std::next(it);
      if (next_it != ld_recommend_lane.end() && next_it->first > 0.0) {
        ld_target_lane = it->second;
        is_found       = true;
        break;
      }

      if (next_it == ld_recommend_lane.end()) {
        ld_target_lane = it->second;
        is_found       = true;
        break;
      }
    }
  }

  if (!is_found) {
    return;
  }
  constexpr double kLdGuideRemmRange  = 200.0;
  int              ld_ego_lane_num    = ld_target_lane.lane_num;
  int              bev_ego_lane_num   = bev_ego_root_section_x0.size();
  int              lane_num_diff      = std::abs(ld_ego_lane_num - bev_ego_lane_num);
  bool             ld_match_from_left = true;  // 1 left, 0 right
  double           nearest_distance   = std::numeric_limits<double>::max();
  // 自车脚下bev车道和地图车道数对不上
  if (bev_ego_lane_num != ld_ego_lane_num) {
    auto it_target_lane =
        std::find_if(ld_recommend_lane.begin(), ld_recommend_lane.end(), [&](const auto &pair) { return pair.second == ld_target_lane; });
    if (it_target_lane != ld_recommend_lane.end()) {
      for (auto it = it_target_lane; it != ld_recommend_lane.end(); ++it) {
        if ((it->second.lane_num == bev_ego_lane_num) && (it->first < kLdGuideRemmRange)) {
          // case1:自车脚下未匹配，合理范围内匹配上
          SD_FINE_MATCH_LOG << "Case1: bev and ld lane num match at a distance";
          ld_target_lane     = it->second;
          ld_match_from_left = true;
          break;
        }
        // case2:合理范围未匹配上，且地图和感知车道数相差较大
        if ((std::next(it) == ld_recommend_lane.end()) && (lane_num_diff > 1)) {
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
              ld_match_from_left = true;
            } else {
              ld_match_from_left = false;
            }
          } else {
            ld_match_from_left = true;
          }
          SD_FINE_MATCH_LOG << "Case2: bev and ld lane num not match";
        }
      }
    }
  }
  SD_FINE_MATCH_LOG << "section_id: " << ld_target_lane.section_id;
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

  for (auto &ld_target_index : ld_target_lane.lane_seqs) {
    SD_FINE_MATCH_LOG << "ld_target_index: " << ld_target_index;
    if (ld_match_from_left) {
      auto find_bev = std::find(ego_lane_index_left.begin(), ego_lane_index_left.end(), ld_target_index);
      if (find_bev != ego_lane_index_left.end()) {
        auto &bev_road_id = bev_ego_root_section_x0.at(std::distance(ego_lane_index_left.begin(), find_bev));
        SD_FINE_MATCH_LOG << "from left bev_road_id: " << bev_road_id;
        bev_guide_lanes_with_ld.insert(bev_road_id);
      } else {
        auto find_closest =
            std::min_element(ego_lane_index_left.begin(), ego_lane_index_left.end(), [&ld_target_index](const int &a, const int &b) {
              return std::abs(a - static_cast<int>(ld_target_index)) < std::abs(b - static_cast<int>(ld_target_index));
            });
        if (find_closest != ego_lane_index_left.end()) {
          auto &bev_road_id = bev_ego_root_section_x0.at(std::distance(ego_lane_index_left.begin(), find_closest));
          SD_FINE_MATCH_LOG << "from left closest bev_road_id: " << bev_road_id;
          bev_guide_lanes_with_ld.insert(bev_road_id);
        }
      }
    } else {
      auto bev_ego_root_section_x0_right = std::vector<uint64_t>(
          bev_ego_root_section_x0.end() - std::min(ld_target_lane.lane_num, bev_ego_root_section_x0.size()), bev_ego_root_section_x0.end());
      for (int i = 1; i <= bev_ego_root_section_x0_right.size(); ++i) {
        ego_lane_index_right.emplace_back(i);
        SD_FINE_MATCH_LOG << "bev_ego_id_right: " << bev_ego_root_section_x0_right.at(i - 1);
      }
      auto find_bev = std::find(ego_lane_index_right.begin(), ego_lane_index_right.end(), ld_target_index);
      if (find_bev != ego_lane_index_right.end()) {
        auto &bev_road_id = bev_ego_root_section_x0_right.at(std::distance(ego_lane_index_right.begin(), find_bev));
        SD_FINE_MATCH_LOG << "from right bev_road_id: " << bev_road_id;
        bev_guide_lanes_with_ld.insert(bev_road_id);
      } else {
        auto find_closest =
            std::min_element(ego_lane_index_right.begin(), ego_lane_index_right.end(), [&ld_target_index](const int &a, const int &b) {
              return std::abs(a - static_cast<int>(ld_target_index)) < std::abs(b - static_cast<int>(ld_target_index));
            });
        if (find_closest != ego_lane_index_right.end()) {
          auto &bev_road_id = bev_ego_root_section_x0_right.at(std::distance(ego_lane_index_right.begin(), find_closest));
          SD_FINE_MATCH_LOG << "from right closest bev_road_id: " << bev_road_id;
          bev_guide_lanes_with_ld.insert(bev_road_id);
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
  for (auto &ego_start_id : bev_guide_lanes_with_ld) {
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

  auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  sd_guide_lane_ids = match_guide_lanes;
  for (auto &laneid_tmp : sd_guide_lane_ids) {
    guide_lane_result.push_back({laneid_tmp, {-1, 0}});
  }

  std::ostringstream os1;
  for (auto guide_laneid_tmp : sd_guide_lane_ids) {
    os1 << guide_laneid_tmp << ",";
  }
  SD_FINE_MATCH_LOG << "[SelectGuideLanesWithLd]  : " << os1.str();
  navi_debug_infos.guide_laneids_refer = sd_guide_lane_ids;
}

void SdNavigationHighway::SetBevEmergencyLane(BevMapInfo &bev_map, const std::pair<uint64_t, uint64_t> &fusion_emergency_lane_id) {
  auto &bev_lane = bev_map.lane_infos;
  if (fusion_emergency_lane_id.first != 0) {
    auto bev_map_lane = std::find_if(bev_lane.begin(), bev_lane.end(), [fusion_emergency_lane_id](const BevLaneInfo &bev_lane_in) {
      return bev_lane_in.id == fusion_emergency_lane_id.first;
    });
    if (bev_map_lane != bev_lane.end()) {
      bev_map_lane->lane_type = BevLaneType::LANE_TYPE_EMERGENCY;
    }
  }

  if (fusion_emergency_lane_id.second != 0) {
    auto bev_map_lane = std::find_if(bev_lane.begin(), bev_lane.end(), [fusion_emergency_lane_id](const BevLaneInfo &bev_lane_in) {
      return bev_lane_in.id == fusion_emergency_lane_id.second;
    });
    if (bev_map_lane != bev_lane.end()) {
      bev_map_lane->lane_type = BevLaneType::LANE_TYPE_EMERGENCY;
    }
  }
}

void SdNavigationHighway::HoldGuideLanes(const std::vector<std::vector<uint64_t>>              &bev_sections_in,
                                         std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                         std::vector<uint64_t> &guide_laneids_ref, BevMapInfoPtr &GlobalBevMapOutPut) {
  std::vector<uint64_t> bev_sections_out;
  for (const auto &lane : bev_sections_in)
    bev_sections_out.insert(bev_sections_out.end(), lane.begin(), lane.end());
  for (const auto &bev_lane : bev_sections_out) {
    SD_FINE_MATCH_LOG << "bev_sections_out: " << bev_lane;
  }
  auto &navi_debug_info                            = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  navi_debug_info.enter_hold_lanes_flag            = false;
  std::vector<std::vector<int>> hold_guides_length = {};
  // Case1: ego非推荐车道场景
  uint64_t ego_bev_id = 0;
  for (auto &bev_lane_info : GlobalBevMapOutPut->lane_infos) {
    if (bev_lane_info.position == (int)BevLanePosition::LANE_LOC_EGO) {
      ego_bev_id = bev_lane_info.id;
      break;
    }
  }
  static uint64_t         case1_enter_counter, case1_active_counter = 1;
  static constexpr size_t kCase1_enter_thres  = 20;
  static constexpr size_t kCase1_active_thres = 10;
  static bool             ego_executed        = false;
  bool                    multi_guide         = (guide_laneids_ref.size() > 1);
  bool ego_in_guide = std::find(guide_laneids_ref.begin(), guide_laneids_ref.end(), ego_bev_id) != guide_laneids_ref.end();
  auto ego_bev_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(ego_bev_id);
  if (multi_guide && ego_bev_lane) {
    bool ego_no_topo = (ego_bev_lane->previous_lane_ids.empty()) && (ego_bev_lane->next_lane_ids.empty());
    if (ego_in_guide && ego_no_topo) {
      if (ego_bev_id != hist_ego_id_ && !ego_executed) {
        hist_ego_id_           = ego_bev_id;
        case1_enter_counter    = 0;
        case1_active_counter   = 0;
        active_last_ego_guide_ = false;
        ego_executed           = true;
      }
      SD_FINE_MATCH_LOG << "case1_enter_counter: " << case1_enter_counter;
      if (++case1_enter_counter >= kCase1_enter_thres) {
        active_last_ego_guide_ = true;
      } else {
        active_last_ego_guide_ = false;
      }
    } else if ((active_last_ego_guide_) && (++case1_active_counter < kCase1_active_thres) && ego_no_topo) {
      SD_FINE_MATCH_LOG << "case1_active_counter: " << case1_active_counter;
      auto rank = [&bev_sections_out](uint64_t lane_id) -> size_t {
        auto it = std::find(bev_sections_out.begin(), bev_sections_out.end(), lane_id);
        return it == bev_sections_out.end() ? SIZE_MAX : static_cast<size_t>(it - bev_sections_out.begin());
      };

      auto safe_distance = [&bev_sections_out, &rank](uint64_t lane_id) -> size_t {
        return rank(lane_id);
      };
      size_t ego_rank = rank(ego_bev_id);
      if (ego_rank == SIZE_MAX)
        return;

      auto pos = std::lower_bound(guide_laneids_ref.begin(), guide_laneids_ref.end(), ego_bev_id,
                                  [&safe_distance](uint64_t a, uint64_t b) { return safe_distance(a) < safe_distance(b); });
      if (pos == guide_laneids_ref.end() || *pos != ego_bev_id) {
        guide_laneids_ref.insert(pos, ego_bev_id);
      }

      auto cmp_elem = [&safe_distance](const auto &e, size_t r) {
        return safe_distance(e.first) < r;
      };

      auto it_ego =
          std::find_if(guide_lane_result.begin(), guide_lane_result.end(), [ego_bev_id](const auto &e) { return e.first == ego_bev_id; });
      if (it_ego != guide_lane_result.end())
        return;
      for (size_t i = 0; i < guide_lane_result.size(); ++i) {
        const auto &[lane_id, inner_pair] = guide_lane_result.at(i);
        int  key                          = inner_pair.first;
        int  value                        = inner_pair.second;
        auto cmp                          = [&rank](const auto &elem, size_t r) {
          return rank(elem.first) < r;
        };
        auto insert_it = std::lower_bound(guide_lane_result.begin(), guide_lane_result.end(), ego_rank, cmp);
        if (key == -1 && value == 0) {
          guide_lane_result.insert(insert_it, {ego_bev_id, {-1, 0}});
          hold_guides_length.push_back({-1, 0});
          navi_debug_info.enter_hold_lanes_flag = true;
          break;
        }
        if (value == -1) {
          guide_lane_result.insert(insert_it, {ego_bev_id, {key, -1}});
          hold_guides_length.push_back({key, -1});
          navi_debug_info.enter_hold_lanes_flag = true;
          break;
        }
      }
      SD_FINE_MATCH_LOG << "Case 1: Append ego lane to guide result";
    } else {
      case1_enter_counter    = 1;
      case1_active_counter   = 1;
      hist_ego_id_           = ego_bev_id;
      active_last_ego_guide_ = false;
    }
  } else {
    case1_enter_counter    = 1;
    case1_active_counter   = 1;
    hist_ego_id_           = ego_bev_id;
    active_last_ego_guide_ = false;
  }
  //// Case2: 单一推荐车道，下匝道逆导航方向场景
  static uint64_t         case2_enter_counter, case2_active_counter = 1;
  static constexpr size_t kCase2_enter_thres  = 20;
  static constexpr size_t kCase2_active_thres = 10;
  const auto             &junction_type       = navi_debug_info.match_type;
  // for (const auto &type : junction_type) {
  //   AINFO << "type: " << type;
  // }
  bool is_ramp_into = (!junction_type.empty()) && (junction_type.back() == "ramp-into");
  bool single_guide = (guide_laneids_ref.size() == 1);
  if (single_guide && is_ramp_into) {
    uint64_t    curr_guide_id   = guide_laneids_ref.front();
    auto        curr_guide_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(curr_guide_id);
    bool        single_no_next  = curr_guide_lane && curr_guide_lane->next_lane_ids.empty();
    static bool executed        = false;
    if (curr_guide_id != hist_guide_id_ && single_no_next && !executed) {
      hist_guide_id_       = curr_guide_id;
      case2_enter_counter  = 0;
      case2_active_counter = 0;
      active_hist_guide_   = false;
      executed             = true;
    }
    SD_FINE_MATCH_LOG << "single_no_next: " << single_no_next;
    SD_FINE_MATCH_LOG << "curr_guide_id: " << curr_guide_id;
    SD_FINE_MATCH_LOG << "hist_guide_id_: " << hist_guide_id_;
    if (single_no_next && curr_guide_id == hist_guide_id_) {
      SD_FINE_MATCH_LOG << "case2_enter_counter: " << case2_enter_counter;
      if (++case2_enter_counter >= kCase2_enter_thres) {
        active_hist_guide_ = true;
      } else {
        active_hist_guide_ = false;
      }
    } else if (active_hist_guide_ && single_no_next) {
      bool find_hist_id = std::find(bev_sections_out.begin(), bev_sections_out.end(), hist_guide_id_) != bev_sections_out.end();
      bool is_left_curr = IsLeftGuidePos(bev_sections_out, curr_guide_id, hist_guide_id_);
      SD_FINE_MATCH_LOG << "find_hist_id: " << find_hist_id;
      SD_FINE_MATCH_LOG << "is_left_curr: " << is_left_curr;
      if (find_hist_id && is_left_curr && ++case2_active_counter < kCase2_active_thres) {
        SD_FINE_MATCH_LOG << "case2_active_counter: " << case2_active_counter;
        guide_laneids_ref.clear();
        guide_laneids_ref.emplace_back(hist_guide_id_);
        for (size_t i = 0; i < guide_lane_result.size(); ++i) {
          auto &[lane_id, inner_pair] = guide_lane_result.at(i);
          int key                     = inner_pair.first;
          int value                   = inner_pair.second;
          if (key == -1 && value == 0) {
            guide_lane_result.clear();
            guide_lane_result.push_back({hist_guide_id_, {-1, 0}});
            hold_guides_length.push_back({-1, 0});
            navi_debug_info.enter_hold_lanes_flag = true;
            break;
          }
          if (value == -1) {
            lane_id = hist_guide_id_;
            hold_guides_length.push_back({key, -1});
            navi_debug_info.enter_hold_lanes_flag = true;
            break;
          }
        }
        SD_FINE_MATCH_LOG << "Case 2: Replace hist lane to guide result";
      } else {
        case2_enter_counter  = 1;
        case2_active_counter = 1;
        hist_guide_id_       = curr_guide_id;
        active_hist_guide_   = false;
      }
    } else {
      case2_enter_counter  = 1;
      case2_active_counter = 1;
      hist_guide_id_       = curr_guide_id;
      active_hist_guide_   = false;
    }
  }

  for (auto &[id, wi] : guide_lane_result)
    SD_FINE_MATCH_LOG << id << ":{" << wi.first << "," << wi.second << "} ";

  navi_debug_info.hold_guide_lanes   = guide_laneids_ref;
  navi_debug_info.hold_guides_length = hold_guides_length;
}

bool SdNavigationHighway::IsLeftGuidePos(std::vector<uint64_t> &bev_sections_out, uint64_t curr_id, uint64_t hist_id) {
  auto curr_it = std::find(bev_sections_out.begin(), bev_sections_out.end(), curr_id);
  auto hist_it = std::find(bev_sections_out.begin(), bev_sections_out.end(), hist_id);

  if (curr_it == bev_sections_out.end() || hist_it == bev_sections_out.end()) {
    return false;
  }

  int curr_idx = static_cast<int>(std::distance(bev_sections_out.begin(), curr_it));
  int hist_idx = static_cast<int>(std::distance(bev_sections_out.begin(), hist_it));

  // curr 在左返回 true，否则 false
  return curr_idx < hist_idx;
}

// 加速车道
void SdNavigationHighway::SetBevAccLane(BevMapInfo &bev_map, const uint64_t &AccLane_id) {
  auto &bev_lane = bev_map.lane_infos;
  if (AccLane_id != 0) {
    auto bev_map_lane = std::find_if(bev_lane.begin(), bev_lane.end(),
                                     [AccLane_id](const BevLaneInfo &bev_lane_in) { return bev_lane_in.id == AccLane_id; });
    if (bev_map_lane != bev_lane.end()) {
      bev_map_lane->lane_type = BevLaneType::LANE_ACC;
    }
  }
}

// 加速车道邻车道
void SdNavigationHighway::SetBevAccAdjLane(BevMapInfo &bev_map, const uint64_t &AccAdjLane_id, JunctionInfo &merge_jct) {
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
  } else if (AccAdjLane_id != 0 && merge_jct.junction_id != 0 && merge_jct.offset < 800) {
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

bool SdNavigationHighway::IsInterestedJunction(const JunctionInfo &junction_info) {
  if (junction_info.junction_type == JunctionType::RampInto || junction_info.junction_type == JunctionType::RampSplitLeft ||
      junction_info.junction_type == JunctionType::RampSplitRight || junction_info.junction_type == JunctionType::RampSplitMiddle ||
      ((junction_info.junction_type == JunctionType::MainRoadSplitRight) && junction_info.main_road_navigation_lane_num < 2)) {
    return true;
  } else {
    return false;
  }
}

bool SdNavigationHighway::IsV2IgnoredJunction(const JunctionInfo &junction_info) {
  if (junction_info.junction_type == JunctionType::RampMerge || junction_info.junction_type == JunctionType::ApproachRampMerge) {
    return true;
  } else {
    return false;
  }
}

void SdNavigationHighway::SetNaviInterfaceAndDebugInfos(BevMapInfoPtr &GlobalBevMapOutPut) {
  /// 导航接口赋值
  auto navigation_info_interface = std::make_shared<std::vector<navigation::JunctionInfo>>();
  *navigation_info_interface     = this->sd_junction_infos_;
  INTERNAL_PARAMS.navigation_info_data.SetJunctionInfoPtr(navigation_info_interface);

  json debug_json;
  auto navi_debug_info = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  // 获取枚举的字符串表示
  debug_json["highway"]            = "useLD";
  debug_json["alanes"]             = navi_debug_info.guide_laneids;
  debug_json["arefer"]             = navi_debug_info.guide_laneids_refer;
  debug_json["avirtual"]           = navi_debug_info.guide_lanes_length;
  debug_json["am_type"]            = navi_debug_info.match_type;
  debug_json["bev_section"]        = navi_debug_info.all_sections_in;
  debug_json["bev_separators"]     = navi_debug_info.all_separators;
  debug_json["btarget_section"]    = navi_debug_info.target_section;
  debug_json["hold_guide_lanes"]   = navi_debug_info.hold_guide_lanes;
  debug_json["hold_guides_length"] = navi_debug_info.hold_guides_length;
  debug_json["enter_hold_lanes"]   = navi_debug_info.enter_hold_lanes_flag;
  debug_json["lane_type"]          = navi_debug_info.lane_type;
  debug_json["filter_section"]     = navi_debug_info.target_section_filter;
  debug_json["road_j_id"]          = navi_debug_info.select_road_junction;
  debug_json["j_id"]               = navi_debug_info.target_junction_ids;
  debug_json["j_offset"]           = navi_debug_info.junction_offset;
  debug_json["nav_lane_num"]       = navi_debug_info.main_road_navigation_lane_num;
  debug_json["bev_counter"]        = navi_debug_info.bev_counter;
  debug_json["bev_emergency_id"]   = navi_debug_info.bev_emergency_id;
  std::string debug_str            = debug_json.dump();
  // AINFO << "===json:" << debug_str;
  SD_BEV_PROCESS << "===json:" << debug_str;
  SD_COARSE_MATCH_LOG << "===json:" << debug_str;
  SD_ENV_INFO_LOG << "===json:" << debug_str;
  SD_MERGE_LOG << "===json:" << debug_str;
  SD_FINE_MATCH_LOG << "===json:" << debug_str;
  GlobalBevMapOutPut->debug_infos = debug_str;
}
/*
 * @brief 打印edge信息
 */
void SdNavigationHighway::Edge_log_print(const std::vector<BevLaneMarker> &all_bev_road_edges) {
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
      switch (edge.type) {
        case 0:
          SD_COARSE_MATCH_LOG << "UNKNOWN_BOUNDARY ";
          break;
        case 1:
          SD_COARSE_MATCH_LOG << "FLAT_BOUNDARY";
          break;
        case 2:
          SD_COARSE_MATCH_LOG << "LOW_BOUNDARY ";
          break;
          case 3:
          SD_COARSE_MATCH_LOG << "HIGH_BOUNDARY ";
          break;
          case 4:
          SD_COARSE_MATCH_LOG << "FENCE_BOUNDARY ";
          break;
          case 12:
          SD_COARSE_MATCH_LOG << "FENCE_BOUNDARY ";
          break;
        default:
          SD_COARSE_MATCH_LOG << "VIRTUAL ";
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
void SdNavigationHighway::Erase_Harborlane_Ids(const HarborStopInfo             &sd_harbor_stop_info_,
                                               std::vector<uint64_t>            &road_selected_buslane_filtered,
                                               const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                                               uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid) {
  /*获取当前section的车道线属性，包括起终点、merge topo、split  topo信息*/
  /*遍历 road_selected_buslane_filtered, 取出其中的每个id*/
  std::vector<uint64_t> indices_id_to_remove        = {};
  std::vector<uint64_t> tunnel_indices_id_to_remove = {};
  // bool                  is_remove            = false;
  size_t lane_count             = 0;
  size_t map_normal_lane_count  = std::count(lanetype_list.begin(), lanetype_list.end(), LaneType::LANE_NORMAL) + std::count(lanetype_list.begin(), lanetype_list.end(), LaneType::LANE_BRT);
  double min_overlap_threshold  = 18.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 3.0f;   // 最大距离阈值(米)

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
        if ((sd_harbor_stop_info_.is_right_most) && (id == road_selected_buslane_filtered.back()) &&
            (single_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT)) {
          SD_COARSE_MATCH_LOG << "harbor single_lane id is  :" << single_lane->id;
          indices_id_to_remove.push_back(single_lane->id);
        } else if ((sd_harbor_stop_info_.is_right_most) && (id == road_selected_buslane_filtered.back()) &&
                   (!single_lane->previous_lane_ids.empty())) {
          /* 前继车道的topo是否分叉 */
          for (auto segment_lane_id : single_lane->previous_lane_ids) {
            auto segment_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(segment_lane_id);
            if (segment_lane && segment_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
              indices_id_to_remove.push_back(single_lane->id);
              SD_COARSE_MATCH_LOG << "harbor single_lane (by pre) id is  :" << single_lane->id;
            }
          }

        } else {
        }
      }
    }
  }

  /*若没有通过拓扑找到港湾车道，则利用每条线到右边界的距离进行判断*/
  Edge_log_print(all_bev_edges);

  if (indices_id_to_remove.empty() && sd_harbor_stop_info_.exists) {
    // 步骤1: 找出与公交港湾重叠最长且大于阈值的边界线
    // int           best_boundary_idx = -1;
    BevLaneMarker best_edge;
    double        max_overlap = -1.0;
    for (auto &edge : all_bev_edges) {
      if (sd_harbor_stop_info_.is_right_most &&
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
           ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() < 0)))) {
        double overlap_dis = calculateOverlapDistance(sd_harbor_stop_info_, *edge.geos);
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] bdry_2_harbor overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge   = edge;
        }
      } else if (sd_harbor_stop_info_.is_left_most && ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
                                                       ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) &&
                                                        (!edge.geos->empty()) && (edge.geos->front().y() > 0)))) {
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
        // double dis_to_rightbdry = cem::fusion::LaneGeometry::GetDistanceBetweenLines(*single_lane->geos, *best_edge.geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry = CalculateCurvedLaneWidth2(*single_lane->geos, *best_edge.geos);
        // double overlap_dis      = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] dis line " << single_lane->id << " to " << best_edge.id
                            << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry < min_distance && dis_to_rightbdry <= max_distance_threshold && overlap_dis >= min_overlap_threshold) {
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
  bool b_in_tunnel = EgoIsInTunnel();
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]  EgoIsInTunnel: " << b_in_tunnel;
  if (sd_harbor_stop_info_.exists && !b_in_tunnel) {
    // 获取第一个港湾车道范围 [start_s, end_s]
    const auto &firstRange           = sd_harbor_stop_info_.ranges[0];
    double      harborStart          = firstRange.first;
    double      harborEnd            = firstRange.second;
    bool        remove_enable        = RemoveHarborlaneCheck(sd_harbor_stop_info_, road_selected_buslane_filtered);
    bool        is_only_harbor_scene = OnlyHarborSceneCheck(sd_harbor_stop_info_, g_emergency_lane_info_);
    bool        filter_enable = !(is_only_harbor_scene && right_bev_emergency_laneid); /* 单harbor场景且存在应急车道，则不过滤，反之需要过滤 */

    SD_COARSE_MATCH_LOG << "harborStart:" << harborStart << ", harborEnd:" << harborEnd << "remove_enable: " << remove_enable;
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
    if ((harborStart < 100.0) && (harborEnd > -10.0) && (lane_count > 2) && (lane_count > map_normal_lane_count) &&
        (map_normal_lane_count > 0) && remove_enable && filter_enable) {

      if (!indices_id_to_remove.empty()) {
        for (int id : indices_id_to_remove) {
          auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]single_lane: " << single_lane->id
                              << "single_lane->position: " << single_lane->position;
          if ((single_lane) && (single_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
            road_selected_buslane_filtered.erase(
                std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
                road_selected_buslane_filtered.end());
          }
        }
      } /* 若仍未找到港湾车道，利用车道数来判断 */
      else if ((lane_count > lanetype_list.size()) && ((harborStart < 80.0) && (harborStart > 0.0)) &&
               (sd_harbor_stop_info_.is_right_most)) {
        road_selected_buslane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]  pop_back in front ";
      } else if ((lane_count >= lanetype_list.size()) && ((harborStart < 0.0) && (harborEnd > 0.0)) &&
                 (sd_harbor_stop_info_.is_right_most)) {
        road_selected_buslane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] pop_back between";
      }
    } else {
    }
  } else if (sd_harbor_stop_info_.exists && b_in_tunnel) {
    // 获取第一个港湾车道范围 [start_s, end_s]
    const auto &firstRange      = sd_harbor_stop_info_.ranges[0];
    double      harborStart     = firstRange.first;
    double      harborEnd       = firstRange.second;
    auto        right_most_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.back());
    auto        left_most_lane  = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.front());
    /* 右侧临停 */
    if (sd_harbor_stop_info_.is_right_most && (right_bev_emergency_laneid == 0)) {
      for (auto &id : road_selected_buslane_filtered) {
        auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
        if ((single_lane) && (!single_lane->geos->empty())) {

          if ((single_lane->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) ||
              (single_lane->merge_topo_extend == cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_LEFT)) {
            SD_COARSE_MATCH_LOG << "tunnel harbor single_lane id is  :" << single_lane->id;
            tunnel_indices_id_to_remove.push_back(single_lane->id);
          } else {
          }
        }
      }
      if (!tunnel_indices_id_to_remove.empty()) {
        for (uint64_t id : tunnel_indices_id_to_remove) {
          auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
          if ((single_lane) && (single_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
            road_selected_buslane_filtered.erase(
                std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
                road_selected_buslane_filtered.end());
          }
        }
      } else if ((harborStart < 150.0) && (harborEnd > -100.0) && (road_selected_buslane_filtered.size() > 2) && (right_most_lane) &&
                 (right_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
        SD_COARSE_MATCH_LOG << "harborStart:" << harborStart << ", harborEnd:" << harborEnd;
        /* 隧道内的港湾车道，利用车道数来判断 */
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] handle harbor in tunnel... ";
        double lane_length   = right_most_lane->geos->back().x() - right_most_lane->geos->front().x();
        double harbor_length = harborEnd - harborStart;
        if ((lane_count > lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
            ((harborStart < 150.0) && (harborStart > 0.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.pop_back();
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]  pop_back in front ";
        } else if ((lane_count >= lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
                   ((harborStart < 0.0) && (harborEnd > 0.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.pop_back();
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] pop_back between";
        } else if ((lane_count > lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
                   ((harborEnd < 0.0) && (harborEnd > -100.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.pop_back();
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] pop_back behind";
        }

      } else {
      }
    } /* 左侧临停 */  //CNOAC2-43115 隧道感知有检出应急车道的情况下，不再过滤
    else if (sd_harbor_stop_info_.is_left_most && (left_bev_emergency_laneid == 0)) {
      for (auto &id : road_selected_buslane_filtered) {
        auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
        if ((single_lane) && (!single_lane->geos->empty())) {

          if ((single_lane->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) ||
              (single_lane->merge_topo_extend == cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT)) {
            SD_COARSE_MATCH_LOG << "tunnel harbor single_lane id is  :" << single_lane->id;
            tunnel_indices_id_to_remove.push_back(single_lane->id);
          } else {
          }
        }
      }
      if (!tunnel_indices_id_to_remove.empty()) {
        for (uint64_t id : tunnel_indices_id_to_remove) {
          auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
          if ((single_lane) && (single_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
            road_selected_buslane_filtered.erase(
                std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
                road_selected_buslane_filtered.end());
          }
        }
      } else if ((harborStart < 150.0) && (harborEnd > -100.0) && (road_selected_buslane_filtered.size() > 2) && (left_most_lane) &&
                 (left_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
        SD_COARSE_MATCH_LOG << "harborStart:" << harborStart << ", harborEnd:" << harborEnd;
        /* 隧道内的港湾车道，利用车道数来判断 */
        double lane_length   = left_most_lane->geos->back().x() - left_most_lane->geos->front().x();
        double harbor_length = harborEnd - harborStart;
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] handle harbor in tunnel... ";
        if ((lane_count > lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
            ((harborStart < 150.0) && (harborStart > 0.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.erase(road_selected_buslane_filtered.begin());
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]  erase_begin in front ";
        } else if ((lane_count >= lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
                   ((harborStart < 0.0) && (harborEnd > 0.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.erase(road_selected_buslane_filtered.begin());
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] erase_begin between";
        } else if ((lane_count > lanetype_list.size() || road_selected_buslane_filtered.size() > map_normal_lane_count) &&
                   ((harborEnd < 0.0) && (harborEnd > -100.0)) && (lane_length < harbor_length + 50)) {
          road_selected_buslane_filtered.erase(road_selected_buslane_filtered.begin());
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] erase_begin behind";
        }

      } else {
      }
    }

  } else if (b_in_tunnel) {
    /* 隧道内,地图未输出港湾车道 */
    bool has_junction_distant = false;
    for (auto &sd_junction : sd_junction_infos_) {
      if (!sd_junction.has_passed_flag && sd_junction.offset < 300 && sd_junction.offset > -30) {
        has_junction_distant = true;
        break;
      }
    }
    SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] has_junction_distant in 300m : " << has_junction_distant;
    if (!has_junction_distant) {
      /* 判断临停 */
      for (auto &id : road_selected_buslane_filtered) {
        auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
        if ((single_lane) && (!single_lane->geos->empty())) {

          if ((single_lane->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) ||
              (single_lane->merge_topo_extend == cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_LEFT) ||
              (single_lane->split_topo_extend == cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) ||
              (single_lane->merge_topo_extend == cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT)) {
            SD_COARSE_MATCH_LOG << "tunnel harbor single_lane id is  :" << single_lane->id;
            tunnel_indices_id_to_remove.push_back(single_lane->id);
          } else {
          }
        }
      }
      if (!tunnel_indices_id_to_remove.empty()) {
        for (uint64_t id : tunnel_indices_id_to_remove) {
          road_selected_buslane_filtered.erase(
              std::remove(road_selected_buslane_filtered.begin(), road_selected_buslane_filtered.end(), id),
              road_selected_buslane_filtered.end());
        }
      } else if (road_selected_buslane_filtered.size() > 2) {
        /* 隧道内的港湾车道，利用车道数来判断 */
        SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] handle harbor in tunnel,but no map harbor... ";
        if ((lane_count > lanetype_list.size() || road_selected_buslane_filtered.size() > lanetype_list.size())) {
          road_selected_buslane_filtered.pop_back();
          SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids] default right harbor pop_back  ";
        }

      } else {
      }
    }
  } else {
  }

  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]Current bevlane num  :" << lane_count
                      << " Current maplane num(normal): " << map_normal_lane_count;
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(indices_id_to_remove, ", "));
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]tunnel_indices_id_to_remove: "
                      << fmt::format(" [{}]  ", fmt::join(tunnel_indices_id_to_remove, ", "));
  SD_COARSE_MATCH_LOG << "[Erase_Harborlane_Ids]road_selected_buslane_filtered: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_buslane_filtered, ", "))
                      << " size: " << road_selected_buslane_filtered.size();
}
/*
  * @brief 根据港湾车道信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
double SdNavigationHighway::calculateOverlapDistance(const HarborStopInfo &harbor, const std::vector<Eigen::Vector2f> geos) {
  SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Entering...]";
  // 检查港湾车道是否存在且有范围
  if (!harbor.exists || harbor.ranges.empty()) {
    return 0.0;
  }

  // 获取第一个港湾车道范围 [start_s, end_s]
  const auto &firstRange  = harbor.ranges[0];
  double      harborStart = firstRange.first;
  double      harborEnd   = firstRange.second;

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
  SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Exiting...]"
                      << " overlapStart: " << overlapStart << " overlapEnd: " << overlapEnd;
  // 如果存在重叠，返回重叠距离
  return (overlapStart < overlapEnd) ? (overlapEnd - overlapStart) : 0.0;
}
/*
  * @brief 根据地图车道列表，去除感知的公交车道id
  * @param lanetype_list  地图车道列表
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
void SdNavigationHighway::Erase_Buslane_Ids(std::vector<LaneType> &lanetype_list, std::vector<uint64_t> &road_selected) {
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
#if 0
std::vector<LaneType> SdNavigationHighway::GetCurLaneTypeList(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                              SDLaneNumInfo                     &lane_num_info) {
  //定义输出为默认值
  std::vector<LaneType> lane_type_list = {};
  SD_COARSE_MATCH_LOG << "Entering GetCurLaneTypeList";
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
  SD_COARSE_MATCH_LOG << "current_s_offset: " << current_s_offset;
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
    SD_COARSE_MATCH_LOG << "end_range_offset: " << current_lane_group_idx->end_range_offset;
    SD_COARSE_MATCH_LOG << "left_offset: " << current_lane_group_idx->end_range_offset - current_s_offset;
  }

  const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(current_lane_group_idx->id);
  SD_COARSE_MATCH_LOG << "current lane group id: " << current_lane_group_idx->id;

  if (!lane_group) {
    AWARN << "[GetCurLaneTypeList] lane group not found";
    SD_COARSE_MATCH_LOG << "lane group not found";
    return {};
  }
  size_t lane_num         = static_cast<size_t>(lane_group->lane_num);
  auto  &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();
  for (const auto &lane : lane_group->lane_info) {
    if (lane.type != cem::message::env_model::LaneType::LANE_NON_MOTOR) {
      lane_type_list.emplace_back(lane.type);
      /*添加debug信息*/
      navi_debug_infos.lane_type.push_back(StrLaneType(lane.type));
      //通过switch  判断所有LaneType，并打印日志
      switch (lane.type) {
        case cem::message::env_model::LaneType::LANE_NON_MOTOR:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_NON_MOTOR";
          break;
        case cem::message::env_model::LaneType::LANE_NORMAL:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_NORMAL";
          break;
        case cem::message::env_model::LaneType::LANE_BUS_NORMAL:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_BUS_NORMAL";
          break;
        case cem::message::env_model::LaneType::LANE_HARBOR_STOP:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_HARBOR_STOP";
          break;
        case cem::message::env_model::LaneType::LANE_UNKNOWN:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_UNKNOWN";
          break;
        case cem::message::env_model::LaneType::LANE_EMERGENCY:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_EMERGENCY";
          break;
        default:
          SD_COARSE_MATCH_LOG << "LaneType::LANE_OTHER_ELSE: "<< StrLaneType(lane.type);
      }
    }
  }
  /*判断车道数与车道类型的数量，两者相等或车道数比车道类型数量多1，则打印日志并且返回车道类型列表*/
  if (lane_num == lane_type_list.size() || lane_num == lane_type_list.size() + 1) {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num << " lane_type_list.size(): " << lane_type_list.size();
  } else {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num << " lane_type_list.size(): " << lane_type_list.size() << " (Mismatch detected)";
  }

  SD_COARSE_MATCH_LOG << "Exiting GetCurLaneTypeList";
  return lane_type_list;
}
#endif
std::vector<LaneType> SdNavigationHighway::GetCurLaneTypeList(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                              SDLaneNumInfo                     &lane_num_info) {
  //定义输出为默认值
  std::vector<LaneType> lane_type_list = {};
  SD_COARSE_MATCH_LOG << "Entering GetCurLaneTypeList with LD...";
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[GetCurLaneTypeList] ld_route_info is null";
    SD_COARSE_MATCH_LOG << "ld_route_info is null";
    return {};
  }

  const auto &mpp_sections = ld_route_info->sections;
  if (mpp_sections.empty()) {
    AWARN << "[GetCurLaneTypeList] mpp_sections is empty";
    SD_COARSE_MATCH_LOG << "mpp_sections is empty";
    return {};
  }

  uint64_t current_section_id = ld_route_info->navi_start.section_id;
  SD_COARSE_MATCH_LOG << "current_section_id: " << current_section_id;

  const SectionInfo *current_section = nullptr;
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

  double current_s_offset = ld_route_info->navi_start.s_offset;
  SD_COARSE_MATCH_LOG << "current_s_offset: " << current_s_offset;
  // 获取当前section的车道ID列表
  const auto &lane_ids = current_section->lane_ids;
  if (lane_ids.empty()) {
    AWARN << "[GetCurLaneTypeList] no lanes in current section";
    SD_COARSE_MATCH_LOG << "no lanes in current section";
    return {};
  }

  // 设置车道数量信息
  lane_num_info.cur_lane_num = lane_ids.size();

  auto &navi_debug_infos = INTERNAL_PARAMS.navigation_info_data.navi_debug_infos();

  // 遍历所有车道ID并获取车道类型
  for (uint64_t lane_id : lane_ids) {
    ConstLaneInfo lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
    if (!lane) {
      AWARN << "[GetCurLaneTypeList] lane not found: " << lane_id;
      SD_COARSE_MATCH_LOG << "lane not found: " << lane_id;
      continue;
    }

    if (lane->type != cem::message::env_model::LaneType::LANE_NON_MOTOR) {
      lane_type_list.emplace_back(lane->type);
      /*添加debug信息*/
      navi_debug_infos.lane_type.push_back(StrLaneType(lane->type));
      //通过switch  判断所有LaneType，并打印日志
      SD_COARSE_MATCH_LOG << "LaneType: " << StrLaneType(lane->type);
    } else {
      SD_COARSE_MATCH_LOG << "Skipped non-motor lane: " << lane_id;
    }
  }
  /*判断车道数与车道类型的数量，两者相等或车道数比车道类型数量多1，则打印日志并且返回车道类型列表*/
  if (lane_num_info.cur_lane_num == lane_type_list.size() || lane_num_info.cur_lane_num == lane_type_list.size() + 1) {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num_info.cur_lane_num << " lane_type_list.size(): " << lane_type_list.size();
  } else {
    SD_COARSE_MATCH_LOG << "lane_num: " << lane_num_info.cur_lane_num << " lane_type_list.size(): " << lane_type_list.size()
                        << " (Mismatch detected)";
  }

  SD_COARSE_MATCH_LOG << "Exiting GetCurLaneTypeList";
  return lane_type_list;
}
/*
   * @brief 过滤掉特殊车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉特殊车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterSpecialLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                             const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                             std::vector<JunctionInfo *> &target_sd_junctions,
                                                             uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid,
                                                             EmergencyLaneInfo &emergency_lane_info, BevMapInfoPtr &GlobalBevMapOutPut) {
  SD_COARSE_MATCH_LOG << "[FilterSpecialLane]Entering FilterSpecialLane...";
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    return road_selected;
  }
  bool is_near_exit = false;  //接近汇出口
                              // JunctionInfo exit_jct;
  if (!target_sd_junctions.empty()) {
    for (auto junction : target_sd_junctions) {
      if (junction->offset < 300 && junction->offset > 0.0 && (junction->junction_type == JunctionType::RampInto)) {
        is_near_exit = true;
        SD_COARSE_MATCH_LOG << " [FilterSpecialLane] APPROACHING junction RampInto ,offset: " << junction->offset
                            << StrJunctionType(junction->junction_type);
        break;
      }
    }
  }
  JunctionInfo merge_jct;
  bool         approaching_merge = IsApproachMergeJCT(target_sd_junctions, merge_jct);  //途径汇入口
  bool         ego_merge         = IsRampMergeJCT(target_sd_junctions, merge_jct);      //自车汇入口
  SD_COARSE_MATCH_LOG << " [FilterSpecialLane] approaching_merge: " << approaching_merge;
  GetAccLaneInfo(approaching_merge);

  std::vector<uint64_t> filtered_lanes_without_emergency = {};
  std::vector<uint64_t> filtered_lanes_without_harbor    = {};
  std::vector<uint64_t> filtered_lanes_without_acc       = {};
  std::vector<uint64_t> final_filtered_without_split     = {};
  std::vector<uint64_t> final_filtered_lanes             = {};
  uint64_t              acc_lane_id                      = 0;
  uint64_t              acc_adj_lane_id                  = 0;
  auto                 &all_bev_edges                    = raw_bev_map->edges;
  SDLaneNumInfo         lane_num_info;
  std::vector<LaneType> lanetype_list = GetCurLaneTypeList(raw_routing_map, lane_num_info);
  bool                  ConstructionZone =
      LeftConstructionZoneCheck(road_selected, all_bev_edges, lanetype_list, right_bev_emergency_laneid, left_bev_emergency_laneid);
  SD_COARSE_MATCH_LOG << " [FilterSpecialLane] ConstructionZone: " << ConstructionZone;

  /* 应急车道的过滤兜底 */
  filtered_lanes_without_emergency = road_selected;  //FilterEmergencyLane(raw_routing_map, road_selected, raw_bev_map);
  /* 港湾车道的过滤 */
  filtered_lanes_without_harbor =
      FilterBusLane(raw_routing_map, filtered_lanes_without_emergency, raw_bev_map, right_bev_emergency_laneid, left_bev_emergency_laneid);
  /* 港湾车道的赋值 */
  UpdateHarborLaneType(filtered_lanes_without_emergency, filtered_lanes_without_harbor, GlobalBevMapOutPut);
  /* 加速车道的过滤 */
  if (is_near_exit) {
    filtered_lanes_without_acc = filtered_lanes_without_harbor;
    SD_COARSE_MATCH_LOG << "[FilterSpecialLane] ignore acc lane cause the rampinto jct in 1200m.";
  } else if (approaching_merge || !ego_merge) {
    /* CNOAC2-51557 */
    SD_COARSE_MATCH_LOG << "[FilterSpecialLane] FilterACCLane...";
    filtered_lanes_without_acc = FilterACCLane(raw_routing_map, filtered_lanes_without_harbor, raw_bev_map, approaching_merge, ego_merge,
                                               acc_lane_id, acc_adj_lane_id);
    /* 无加速车道场景的acc_adj_lane_id的匹配 */
    if (!sd_acc_lane_info_.exists && EgoInMainRoad() && approaching_merge && merge_jct.offset < 800) {
      if (acc_adj_lane_id == 0 && filtered_lanes_without_acc.size() >= merge_jct.main_road_lane_nums && merge_jct.main_road_lane_nums > 0) {
        //acc_adj_lane_id = filtered_lanes_without_acc[merge_jct.main_road_lane_nums - 1];
      }
    }
  } else {
    filtered_lanes_without_acc = filtered_lanes_without_harbor;
    SD_COARSE_MATCH_LOG << "[FilterSpecialLane] other situation maybe ramp merge or main road. acc filter is not valid.";
  }

  SD_COARSE_MATCH_LOG << "[FilterSpecialLane] acc_lane_id: " << acc_lane_id << ", acc_adj_lane_id: " << acc_adj_lane_id;

  if (GlobalBevMapOutPut) {
    SetBevAccLane(*GlobalBevMapOutPut, acc_lane_id);
    SetBevAccAdjLane(*GlobalBevMapOutPut, acc_adj_lane_id, merge_jct);
  }

  /* split车道的过滤 */
  final_filtered_without_split = FilterSplitLane(raw_routing_map, filtered_lanes_without_acc, raw_bev_map, acc_lane_id, GlobalBevMapOutPut);

  /* 减速车道的过滤 */
  if (!pre_split_lanes_to_move_.split_lane_ids.empty()) {
    final_filtered_lanes = final_filtered_without_split;
  } else if (ConstructionZone) {
    final_filtered_lanes = FilterDecLane(raw_routing_map, final_filtered_without_split, raw_bev_map, GlobalBevMapOutPut);
  } else {
    final_filtered_lanes = final_filtered_without_split;
  }


  SD_COARSE_MATCH_LOG << " [FilterSpecialLane]  final_filtered_lanes : " << fmt::format(" [{}]  ", fmt::join(final_filtered_lanes, ", "));

  return final_filtered_lanes;
}
/*
   * @brief 过滤掉公交车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉公交车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterBusLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                         const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                         uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid) {
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
  std::vector<LaneType> lane_type_list = GetCurLaneTypeList(raw_routing_map, lane_num_info);
  //lane_type_list判空
  if (lane_type_list.empty()) {
    SD_COARSE_MATCH_LOG << "FilterBusLane lane_type_list is empty.";
    navi_debug_infos.sections_without_bus = road_selected_buslane_filtered;
    return road_selected_buslane_filtered;
  }
  /*获取港湾的信息*/
  //GetHarborStopInfo();
  if (sd_harbor_stop_info_.exists) {
    SD_COARSE_MATCH_LOG << "harbor_stop is exists.";
    for (const auto &range : sd_harbor_stop_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    /*检测港湾分叉线，并移除该线*/
    Erase_Harborlane_Ids(sd_harbor_stop_info_, road_selected_buslane_filtered, all_bev_road_edges, lane_type_list,
                         right_bev_emergency_laneid, left_bev_emergency_laneid);

  } else {
    SD_COARSE_MATCH_LOG << "harbor_stop is not found in map,try to detect harbor in tunnel.";
    /*检测港湾分叉线，并移除该线*/
    Erase_Harborlane_Ids(sd_harbor_stop_info_, road_selected_buslane_filtered, all_bev_road_edges, lane_type_list,
                         right_bev_emergency_laneid, left_bev_emergency_laneid);
  }

  /*添加debug信息*/
  //navi_debug_infos.sections_without_bus = road_selected_buslane_filtered;
  // 最终结果日志输出
  SD_COARSE_MATCH_LOG << "FilterBusLane Filtered result: " << fmt::format("[{}]", fmt::join(road_selected_buslane_filtered, ", "));
  SD_COARSE_MATCH_LOG << "Exiting FilterBusLane function ";
  return road_selected_buslane_filtered;
}
#if 0
/*
  * @brief 当前是否在隧道内
  * @return 
*/
bool SdNavigationHighway::EgoIsInTunnel() {
  SD_COARSE_MATCH_LOG << "Entering EgoIsInTunnel";
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    AWARN << "[EgoIsInTunnel] sd_route is null";
    SD_COARSE_MATCH_LOG << "sd_route is null";
    return false;
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "[EgoIsInTunnel] mpp_sections is empty";
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
    AWARN << "[EgoIsInTunnel] current section not found";
    SD_COARSE_MATCH_LOG << "current section not found";
    return false;
  }
  bool is_tunnel = (current_section->link_type & static_cast<uint32_t>(SDLinkTypeMask::SDLT_TUNNEL)) != 0;
  SD_COARSE_MATCH_LOG << "[EgoIsInTunnel] link_type: " << StrLinkType(current_section->link_type) << "result: " << is_tunnel;
  return is_tunnel;
}
#endif
/*
  * @brief 当前是否在隧道内
  * @return
*/
bool SdNavigationHighway::EgoIsInTunnel() {
  SD_COARSE_MATCH_LOG << "Entering EgoIsInTunnel with LD";
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[EgoIsInTunnel LD] ld_route_info is null";
    SD_COARSE_MATCH_LOG << "ld_route_info is null";
    return false;
  }

  const auto &mpp_sections = ld_route_info->sections;
  if (mpp_sections.empty()) {
    AWARN << "[EgoIsInTunnel LD] mpp_sections is empty";
    SD_COARSE_MATCH_LOG << "mpp_sections is empty";
    return false;
  }

  uint64_t current_section_id = ld_route_info->navi_start.section_id;
  SD_COARSE_MATCH_LOG << "current_section_id: " << current_section_id;

  const SectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[EgoIsInTunnel LD] current section not found";
    SD_COARSE_MATCH_LOG << "current section not found";
    return false;
  }
  SD_COARSE_MATCH_LOG << "current link type: " << current_section->link_type;
  bool is_tunnel = (current_section->link_type & static_cast<uint32_t>(LDLinkTypeMask::LT_TUNNEL)) != 0;
  SD_COARSE_MATCH_LOG << "[EgoIsInTunnel LD] link_type: " << StrLinkType(current_section->link_type) << "result: " << is_tunnel;
  return is_tunnel;
}

// 获取加速车道信息 (LD地图适配)
void SdNavigationHighway::GetAccLaneInfo(bool approaching_merge) {
  // 获取地图路径信息指针
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[GetAccLaneInfo] GetAccLaneInfo Failed: ld_route_info is null!";
    return;
  }

  // 自车位置参数
  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;

  SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] GetAccLaneInfo Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  // 查找自车所在路段索引
  int ego_section_index = -1;
  for (size_t i = 0; i < ld_route_info->sections.size(); ++i) {
    if (ld_route_info->sections[i].id == ego_section_id) {
      ego_section_index = static_cast<int>(i);
      break;
    }
  }
  if (ego_section_index == -1) {
    AWARN << "[GetAccLaneInfo] GetAccLaneInfo Failed: ego_section not found!";
    return;
  }

  // 计算自车全局s坐标
  double ego_global_s = 0.0;
  for (int i = 0; i < ego_section_index; ++i) {
    ego_global_s += ld_route_info->sections[i].length;
  }
  ego_global_s += ego_s_offset;

  SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] GetAccLaneInfo Ego global s: " << ego_global_s;

  // 定义搜索范围
  double min_s = acc_rear_distance_;   // 后向搜索范围（-100m）
  double max_s = acc_front_distance_;  // 前向搜索范围（800m）

  // 初始化加速车道信息
  sd_acc_lane_info_.exists          = false;
  sd_acc_lane_info_.is_left_most    = false;
  sd_acc_lane_info_.is_right_most   = false;
  sd_acc_lane_info_.normal_lane_num = 0;
  sd_acc_lane_info_.ranges.clear();
  sd_acc_lane_info_.merge_dir = AccLaneMergeDir::ACC_MERGE_UNKNOWN;  // ← 显式初始化

  std::vector<std::pair<double, double>> acc_lane_ranges;                      // 临时存储加速车道区间
  bool                                   current_leftmost            = false;  // 当前车道组是否最左加速车道
  bool                                   current_rightmost           = false;  // 当前车道组是否最右加速车道
  size_t                                 normal_lane_num             = 0;
  size_t                                 ego_section_normal_lane_num = 0;  // 自车所在section的普通车道数

  double current_s = 0.0;  // 全局s坐标累加变量
  // 遍历所有路段
  for (size_t i = 0; i < ld_route_info->sections.size(); ++i) {
    const auto &section         = ld_route_info->sections[i];
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    // 跳过与搜索范围无交集的路段
    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s) {
      current_s += section.length;
      continue;
    }

    // SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] GetAccLaneInfo Section ID: " << section.id
    //       << " start from s: " << section_start_s << " to s: " << section_end_s;

    // 收集当前section的所有车道,直接处理车道
    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {
      // 使用LD地图获取车道信息
      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
      }
    }

    if (section_lanes.empty())
      continue;

    /* CNOAC2-111458 普通车道数的计算优化为自车位置的车道数 */
    if (section.id == ego_section_id) {
      size_t ego_normal_lane_count = 0;
      for (const auto *lane : section_lanes) {
        if (lane->type == cem::message::env_model::LaneType::LANE_NORMAL) {
          ego_normal_lane_count++;
        }
      }
      ego_section_normal_lane_num = ego_normal_lane_count;
      SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Ego section " << ego_section_id << " normal lane count: " << ego_section_normal_lane_num;
    }
    //  SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Acceleration: section: "<<section.id<<" lane num:  "<<section_lanes.size();
    // 遍历当前路段的车道组
    for (size_t idx = 0; idx < section_lanes.size(); ++idx) {
      const auto *lane = section_lanes[idx];
      // SD_COARSE_MATCH_LOG << "LaneType: " << StrLaneType(lane->type);
      // 检测加速车道
      if (lane->type == cem::message::env_model::LaneType::LANE_ACC || lane->type == cem::message::env_model::LaneType::LANE_ENTRY) {
        //  判断 merge_dir
        if (lane->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_LEFT) {
          sd_acc_lane_info_.merge_dir = AccLaneMergeDir::ACC_MERGE_LEFT;
          double start_s              = std::max(section_start_s - ego_global_s, min_s);
          SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Acceleration lane detected, ACC_MERGE_LEFT. id: " << lane->id << " start_s: " << start_s;
        } else if (lane->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_RIGHT) {
          sd_acc_lane_info_.merge_dir = AccLaneMergeDir::ACC_MERGE_RIGHT;
        } /*else{
          sd_acc_lane_info_.merge_dir = AccLaneMergeDir::ACC_MERGE_UNKNOWN;
        }*/

        // 计算加速车道相对于自车的s区间（裁剪到搜索范围）
        const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
        const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);
        if (lane_start_s < lane_end_s) {
          // 动态合并重叠区间
          if (!acc_lane_ranges.empty() && lane_start_s <= acc_lane_ranges.back().second) {
            acc_lane_ranges.back().second = std::max(acc_lane_ranges.back().second, lane_end_s);
          } else {
            acc_lane_ranges.emplace_back(lane_start_s, lane_end_s);
          }
          SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Acceleration lane detected, range: [" << lane_start_s << ", " << lane_end_s << "]";
        }
      }
    }
    // 遍历当前路段的车道组
    for (size_t idx = 0; idx < section_lanes.size(); ++idx) {
      const auto *lane = section_lanes[idx];
      // 检测加速车道
      if (lane->type == cem::message::env_model::LaneType::LANE_ACC || lane->type == cem::message::env_model::LaneType::LANE_ENTRY) {
        //  调用函数获取源头普通车道ID
        std::pair<double, double> target_range;
        bool                      found_forward = false;
        // 计算加速车道相对于自车的s区间（裁剪到搜索范围）
        const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
        const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

        // uint64_t ego_normal_lane_ids;
        if (!acc_lane_ranges.empty()) {
          for (auto range : acc_lane_ranges) {
            if (range.second > 0) {
              target_range  = range;
              found_forward = true;
              break;
            }
          }
        }
        if (!found_forward || lane_start_s > target_range.second)
          continue;
        SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Target acc lane range: [" << target_range.first << ", " << target_range.second << "] ";
        SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] approaching_merge  is: " << approaching_merge << " EgoInMainRoad():  " << EgoInMainRoad()
                            << "found_forward: " << found_forward;

        if (approaching_merge && EgoInMainRoad() && found_forward) {
          ego_normal_lane_id_ = FindOriginNormalLaneIdFromAccLane2(lane, ego_section_id, 800);
          SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] ego_normal_lane_id_  is: " << ego_normal_lane_id_ << " section_lanes idx " << idx;
        }
        if (ego_normal_lane_id_ != 0) {
          break;
          SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Origin normal lane ID from acc lane " << lane->id << " is: " << ego_normal_lane_id_;
        } else {
          SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Could not trace back to normal lane in ego section for acc lane " << lane->id;
        }
      }
    }

    // 标记加速车道是否为车道组最左/最右
    size_t lane_count        = section_lanes.size();
    size_t normal_lane_count = 0;
    bool   has_acc_lane      = false;
    for (size_t idx = 0; idx < lane_count; ++idx) {
      const auto *lane = section_lanes[idx];
      //  SD_COARSE_MATCH_LOG << "GetAccLaneInfo LaneType: " << StrLaneType(lane->type);
      if (lane->type == cem::message::env_model::LaneType::LANE_ACC || lane->type == cem::message::env_model::LaneType::LANE_ENTRY) {
        if (idx == 0)
          current_leftmost = true;  // 车道组第一个车道是加速车道→最左
        if (idx == lane_count - 1)
          current_rightmost = true;  // 车道组最后一个→最右
        has_acc_lane = true;
      }
      if (lane->type == cem::message::env_model::LaneType::LANE_NORMAL) {
        normal_lane_count++;
      }
    }
    if (has_acc_lane) {
      normal_lane_num = normal_lane_count;
    }
    current_s += section.length;  // 累加全局s坐标
  }

  // 输出结果
  sd_acc_lane_info_.ranges             = acc_lane_ranges;
  sd_acc_lane_info_.exists             = !acc_lane_ranges.empty();
  sd_acc_lane_info_.is_left_most       = current_leftmost;
  sd_acc_lane_info_.is_right_most      = current_rightmost;
  sd_acc_lane_info_.normal_lane_num    = ego_section_normal_lane_num;
  sd_acc_lane_info_.ego_normal_lane_id = ego_normal_lane_id_;

  std::pair<double, double> firstRange = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  if (sd_acc_lane_info_.exists) {
    for (auto range : sd_acc_lane_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
    sd_acc_lane_info_.merge_start_dis = firstRange.first;
    sd_acc_lane_info_.merge_end_dis   = firstRange.second;
  }

  if (sd_acc_lane_info_.exists) {
    SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] Acceleration lane exists in merged ranges:";
    for (const auto &range : sd_acc_lane_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "] "
                          << "  is_left_most: " << sd_acc_lane_info_.is_left_most << "  is_right_most: " << sd_acc_lane_info_.is_right_most
                          << "  normal_lane_num: " << sd_acc_lane_info_.normal_lane_num
                          << " merge_start_dis: " << sd_acc_lane_info_.merge_start_dis
                          << " merge_end_dis: " << sd_acc_lane_info_.merge_end_dis
                          << " merge_dir: " << static_cast<int>(sd_acc_lane_info_.merge_dir);
    }
  } else {
    SD_COARSE_MATCH_LOG << "[GetAccLaneInfo] No acceleration lane found.";
  }
}

//获取减速车道信息
void SdNavigationHighway::GetDecLaneInfo() {
  // 逻辑与GetAccLaneInfo一致，仅车道类型改为LANE_DEC
  auto ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[GetDecLaneInfo]  Failed: ld_route_info is null!";
    return;
  }

  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;
#ifdef ENABLE_DEBUG_SDNAVIGATION
  SD_COARSE_MATCH_LOG << "[GetDecLaneInfo]  Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;
#endif

  int ego_section_index = -1;
  for (size_t i = 0; i < ld_route_info->sections.size(); ++i) {
    if (ld_route_info->sections[i].id == ego_section_id) {
      ego_section_index = static_cast<int>(i);
      break;
    }
  }
  if (ego_section_index == -1) {
    AWARN << "[GetDecLaneInfo]  Failed: ego_section not found!";
    return;
  }

  double ego_global_s = 0.0;
  for (int i = 0; i < ego_section_index; ++i) {
    ego_global_s += ld_route_info->sections[i].length;
  }
  ego_global_s += ego_s_offset;
#ifdef ENABLE_DEBUG_SDNAVIGATION
  AINFO << "[GetDecLaneInfo]  Ego global s: " << ego_global_s;
#endif

  double min_s = dec_rear_distance_;   // 后向搜索范围
  double max_s = dec_front_distance_;  // 前向搜索范围

  sd_dec_lane_info_.exists          = false;
  sd_dec_lane_info_.is_left_most    = false;
  sd_dec_lane_info_.is_right_most   = false;
  sd_dec_lane_info_.normal_lane_num = 0;
  sd_dec_lane_info_.ranges.clear();

  std::vector<std::pair<double, double>> dec_lane_ranges;
  bool                                   current_leftmost  = false;
  bool                                   current_rightmost = false;
  size_t                                 normal_lane_num   = 0;
  double                                 current_s         = 0.0;
  for (size_t i = 0; i < ld_route_info->sections.size(); ++i) {
    const auto &section         = ld_route_info->sections[i];
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s) {
      current_s += section.length;
      continue;
    }

    //SD_COARSE_MATCH_LOG << "[GetDecLaneInfo]  Section ID: " << section.id
    //      << " start from s: " << section_start_s << " to s: " << section_end_s;
    // 收集当前section的所有车道,直接处理车道
    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {
      // 使用LD地图获取车道信息
      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
      }
    }

    if (section_lanes.empty())
      continue;

    // 遍历当前路段的车道组
    for (size_t idx = 0; idx < section_lanes.size(); ++idx) {
      const auto *lane = section_lanes[idx];
      // 检测加速车道
      if (lane->type == cem::message::env_model::LaneType::LANE_DEC || lane->type == cem::message::env_model::LaneType::LANE_EXIT) {
        // 计算加速车道相对于自车的s区间（裁剪到搜索范围）
        const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
        const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);
        if (lane_start_s < lane_end_s) {
          // 动态合并重叠区间
          if (!dec_lane_ranges.empty() && lane_start_s <= dec_lane_ranges.back().second) {
            dec_lane_ranges.back().second = std::max(dec_lane_ranges.back().second, lane_end_s);
          } else {
            dec_lane_ranges.emplace_back(lane_start_s, lane_end_s);
          }
          //SD_COARSE_MATCH_LOG << "[GetDecLaneInfo] Acceleration lane detected, range: ["<< lane_start_s << ", " << lane_end_s << "]";
        }
      }
    }

    // 标记加速车道是否为车道组最左/最右
    size_t lane_count        = section_lanes.size();
    size_t normal_lane_count = 0;
    for (size_t idx = 0; idx < lane_count; ++idx) {
      const auto *lane = section_lanes[idx];
      if (lane->type == cem::message::env_model::LaneType::LANE_DEC) {
        SD_COARSE_MATCH_LOG << "[GetDecLaneInfo]  Dec lane exists :" << section_lanes.size() << "index: " << idx;
        if (idx == 0)
          current_leftmost = true;  // 车道组第一个车道是加速车道→最左
        if (idx == lane_count - 1)
          current_rightmost = true;  // 车道组最后一个→最右
      }
      if (lane->type == cem::message::env_model::LaneType::LANE_NORMAL) {
        normal_lane_count++;
      }
      normal_lane_num = normal_lane_count;
    }
    current_s += section.length;
  }

  sd_dec_lane_info_.ranges          = dec_lane_ranges;
  sd_dec_lane_info_.exists          = !dec_lane_ranges.empty();
  sd_dec_lane_info_.is_left_most    = current_leftmost;
  sd_dec_lane_info_.is_right_most   = current_rightmost;
  sd_dec_lane_info_.normal_lane_num = normal_lane_num;

  if (sd_dec_lane_info_.exists) {
    SD_COARSE_MATCH_LOG << "[GetDecLaneInfo]  Dec lane exists in merged ranges:";
    for (const auto &range : sd_dec_lane_info_.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "] "
                          << "  is_left_most: " << sd_dec_lane_info_.is_left_most << "  is_right_most: " << sd_dec_lane_info_.is_right_most
                          << "  normal_lane_num: " << sd_dec_lane_info_.normal_lane_num;
    }
  } else {
    SD_COARSE_MATCH_LOG << "[GetDecLaneInfo] No deceleration lane found.";
  }
}

/*
   * @brief 过滤掉应急车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉应急车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterEmergencyLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                               const std::vector<uint64_t>       &road_selected,
                                                               BevMapInfoConstPtr                &raw_bev_map) {
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[FilterEmergencyLane]  road_selected is empty! ";
    return road_selected;
  }
  std::vector<uint64_t> road_selected_emlane_filtered(road_selected);
  SD_COARSE_MATCH_LOG << "Entering FilterEmLane function with inputs: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_emlane_filtered, ", ")) << " size: " << road_selected.size();
  // 获取车道列表

  size_t lane_count             = 0;
  size_t map_normal_lane_count  = 0;
  size_t map_lane_count         = 0;
  double min_overlap_threshold  = 15.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 3.0f;   // 最大距离阈值(米)

  EmergencyLaneInfo emergency_lane_info = GetEmergencyLaneInfo();
  if (emergency_lane_info.right.exists) {
    map_normal_lane_count = (emergency_lane_info.right.lane_num - 1) > 0 ? (emergency_lane_info.right.lane_num - 1) : 0;
    map_lane_count        = emergency_lane_info.right.lane_num;

    for (auto &id : road_selected_emlane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
      if ((single_lane) && (!single_lane->geos->empty())) {
        SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << single_lane->geos->front().x()
                            << " lane_start_y: " << single_lane->geos->front().y() << " lane_end_x: " << single_lane->geos->back().x()
                            << " lane_end_y: " << single_lane->geos->back().y();

        double overlap_dis = calculateOverlapDisinRange(emergency_lane_info.right.ranges[0].first,
                                                        emergency_lane_info.right.ranges[0].second, *single_lane->geos);
        SD_COARSE_MATCH_LOG << "overlap_dis:" << overlap_dis;
        if ((overlap_dis > min_overlap_threshold)) {
          lane_count++;
        }
      }
    }
  }

  if (emergency_lane_info.right.exists) {
    // 获取第一个 应急车道范围 [start_s, end_s]
    const auto &firstRange     = emergency_lane_info.right.ranges[0];
    double      emergencyStart = firstRange.first;
    double      emergencyEnd   = firstRange.second;
    auto        rightmost      = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_emlane_filtered.back());
    double      rightmost_overlap_dis =
        calculateOverlapDisinRange(emergency_lane_info.right.ranges[0].first, emergency_lane_info.right.ranges[0].second, *rightmost->geos);

    if ((emergencyStart < 100.0) && (emergencyEnd > 0.0) && (lane_count > 2) && (lane_count > map_normal_lane_count) &&
        (map_lane_count > 0) && (rightmost) && (rightmost->position != (int)BevLanePosition::LANE_LOC_EGO) &&
        (rightmost_overlap_dis < 50)) {
      SD_COARSE_MATCH_LOG << "emergencyStart:" << emergencyStart << ", emergencyEnd:" << emergencyEnd
                          << " , rightmost_overlap_dis: " << rightmost_overlap_dis;
      /*  利用车道数来判断 */
      if ((lane_count > map_lane_count) && ((emergencyStart < 80.0) && (emergencyStart > 0.0))) {
        road_selected_emlane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[FilterEmergencyLane]  pop_back in front ";
      } else if ((lane_count >= map_lane_count) && ((emergencyStart < 0.0) && (emergencyEnd > 0.0))) {
        road_selected_emlane_filtered.pop_back();
        SD_COARSE_MATCH_LOG << "[FilterEmergencyLane] pop_back between";
      }
    } else {
    }
  } else {
  }

  SD_COARSE_MATCH_LOG << "[FilterEmergencyLane]Current bevlane num  :" << lane_count
                      << " Current maplane num(normal): " << map_normal_lane_count;
  SD_COARSE_MATCH_LOG << "[FilterEmergencyLane]road_selected_emlane_filtered: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_emlane_filtered, ", "))
                      << " size: " << road_selected_emlane_filtered.size();
  return road_selected_emlane_filtered;
}

/*
  * @brief 根据range信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
double SdNavigationHighway::calculateOverlapDisinRange(double rangeStart, double rangeEnd, const std::vector<Eigen::Vector2f> geos) {
  SD_COARSE_MATCH_LOG << "[calculateOverlapDisinRange]Entering...";

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
  SD_COARSE_MATCH_LOG << "[calculateOverlapDisinRange] Exiting..."
                      << " overlapStart: " << overlapStart << " overlapEnd: " << overlapEnd;
  // 如果存在重叠，返回重叠距离
  return (overlapStart < overlapEnd) ? (overlapEnd - overlapStart) : 0.0;
}

/*
   * @brief 过滤掉加速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉加速车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterACCLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                         const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                         bool &approaching_merge, bool &ego_merge, uint64_t &acc_lane_id,
                                                         uint64_t &acc_adj_lane_id) {
  // 判断road_selected是否为空，空则返回 2025.10.15
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[FilterACCLane]  road_selected is empty! ";
    return road_selected;
  }
  std::vector<uint64_t> road_selected_acclane_filtered(road_selected);
  std::vector<uint64_t> road_selected_normal_lanes = {};
  SD_COARSE_MATCH_LOG << "Entering FilterACCLane function with inputs: "
                      << fmt::format(" [{}]  ", fmt::join(road_selected_acclane_filtered, ", ")) << " size: " << road_selected.size();
  // 获取车道列表

  size_t lane_count             = 0;
  size_t current_lane_count     = 0;
  size_t map_normal_lane_count  = 0;
  size_t map_lane_count         = 0;
  double min_overlap_threshold  = 15.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 3.0f;   // 最大距离阈值(米)

  if (sd_acc_lane_info_.exists) {
    std::pair<double, double> frontRange = {0, 0};
    for (auto range : sd_acc_lane_info_.ranges) {
      if (range.second > 0) {
        frontRange = range;
        break;
      }
    }
    map_normal_lane_count = sd_acc_lane_info_.normal_lane_num;
    map_lane_count        = map_normal_lane_count + 1;

    for (auto &id : road_selected_acclane_filtered) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
      if ((single_lane) && (!single_lane->geos->empty())) {

        double overlap_dis     = calculateOverlapDisinRange(frontRange.first, frontRange.second, *single_lane->geos);
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
  /* 800m内出现加速车道 */
  if (sd_acc_lane_info_.exists) {
    // 获取第一个 加速车道范围 [start_s, end_s]
    std::pair<double, double> firstRange = {0, 0};
    for (auto range : sd_acc_lane_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
    //const auto &firstRange     = sd_acc_lane_info_.ranges[0];
    double mergeStart     = firstRange.first;
    double mergeEnd       = firstRange.second;
    map_normal_lane_count = sd_acc_lane_info_.normal_lane_num;
    SD_COARSE_MATCH_LOG << "acc  Start:" << mergeStart << ", acc End:" << mergeEnd;
    SD_COARSE_MATCH_LOG << "map_normal_lane_count:" << map_normal_lane_count;
    /* 先匹配加速车道 */ /* CNOAC2-8901 */
    if ((mergeStart < 100.0) && (mergeEnd > 0.0) && (lane_count > 2) &&
        (lane_count > map_normal_lane_count || road_selected_acclane_filtered.size() > map_normal_lane_count) &&
        (map_normal_lane_count > 0)) {
      acc_lane_id = road_selected_acclane_filtered[map_normal_lane_count];  // 普通车道右侧
      /* 若acc_lane_id 的车道是merge车道，继续进行过滤，否则不过滤  CNOAC2-41924 */
      if (acc_lane_id != 0 && sd_acc_lane_info_.merge_dir == AccLaneMergeDir::ACC_MERGE_LEFT) {
        for (size_t i = 0; i < map_normal_lane_count; i++) {
          road_selected_normal_lanes.push_back(road_selected_acclane_filtered[i]);
        }
        road_selected_acclane_filtered = road_selected_normal_lanes;
      } else {
        /* 保持原有输出 */
      }
    } else {
    }

    /* 再考虑加速车道邻车道 */
    /* 自车在主路 感知车道数大于地图普通车道数 acc范围内 */
    if ((approaching_merge || !ego_merge) && EgoInMainRoad() && (mergeStart < 800.0) && (mergeEnd > 0.0) &&
        (current_lane_count >= map_normal_lane_count) && (road_selected_acclane_filtered.size() >= map_normal_lane_count) &&
        (map_normal_lane_count > 0)) {
      /* 后续添加路沿校验 */
      acc_adj_lane_id = road_selected_acclane_filtered[map_normal_lane_count - 1];  // 普通车道右侧
    } else {
    }
  }

  SD_COARSE_MATCH_LOG << "[FilterACCLane] acc_lane_id: " << acc_lane_id << ", acc_adj_lane_id: " << acc_adj_lane_id;
  SD_COARSE_MATCH_LOG << "[FilterACCLane]final lanes: " << fmt::format(" [{}]  ", fmt::join(road_selected_acclane_filtered, ", "))
                      << " size: " << road_selected_acclane_filtered.size();
  return road_selected_acclane_filtered;
}

/*
   * @brief 过滤掉分叉车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉分叉车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterSplitLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                           const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                           uint64_t acc_lane_id, BevMapInfoPtr &GlobalBevMapOutPut) {
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[FilterSplitLane]  road_selected is empty! ";
    return road_selected;
  }
  auto                 &all_bev_edges = raw_bev_map->edges;
  std::vector<uint64_t> rsplit_lane_filtered(road_selected);
  std::vector<uint64_t> split_indices_id_to_remove;
  SD_COARSE_MATCH_LOG << "Entering FilterSplitLane function with inputs: " << fmt::format(" [{}]  ", fmt::join(rsplit_lane_filtered, ", "))
                      << " size: " << road_selected.size();

  bool has_junction_distant          = false; /* 300m内存在路口 */
  bool has_junction_app_ramp         = false; /* [0,300m]的最近路口为途径下匝道 */
  bool has_junction_rampinto         = false; /* [0,800m]的最近路口为下匝道 */
  bool has_junction_rampinto_distant = false; /* [0,800m]的最近路口为下匝道，但距离减速车道起点大于一定距离 */
  bool has_junction_app_merge        = false; /* [0,300m]的最近路口为途径汇入 */

  std::unordered_set<uint64_t> processed_lanes;
  for (auto &sd_junction_tmp : sd_junction_infos_) {
    SD_COARSE_MATCH_LOG << "[SDNOA Junction] ,id : " << sd_junction_tmp.junction_id
                        << " ,type: " << StrNaviJunctionHighway(static_cast<int>(sd_junction_tmp.junction_type))
                        << " ,offset:  " << sd_junction_tmp.offset << "  ,main_road_lane_nums: " << sd_junction_tmp.main_road_lane_nums
                        << " ,direction: " << StrNaviSplitDirectionHighway(sd_junction_tmp.split_direction)
                        << " ,target_road_lane_nums: " << sd_junction_tmp.target_road_lane_nums
                        << " ,passed_flag: " << (int)sd_junction_tmp.has_passed_flag
                        << " ,effected_flag: " << (int)sd_junction_tmp.has_effected_flag;
  }

  for (auto &sd_junction : sd_junction_infos_) {
    if (!sd_junction.has_passed_flag && sd_junction.offset < 300 && sd_junction.offset > 0) {
      has_junction_distant = true;
      break;
    }
  }

  for (auto &sd_junction : sd_junction_infos_) {
    /*  CNOAC2-38580  160M */
    if (!sd_junction.has_passed_flag && sd_junction.offset < 300 && sd_junction.offset > -30 &&
        sd_junction.junction_type == JunctionType::ApproachRampInto) {
      has_junction_app_ramp = true;
      break;
    }
  }

  for (auto &sd_junction : sd_junction_infos_) {
    /*  CNOAC2-121271  */
    if (!sd_junction.has_passed_flag && sd_junction.offset < 300 && sd_junction.offset > -30 &&
        sd_junction.junction_type == JunctionType::ApproachRampMerge) {
      has_junction_app_merge = true;
      break;
    }
  }

  // CNOAC2-43671
  JunctionInfo exit_jct;
  for (auto &sd_junction : sd_junction_infos_) {
    if (!sd_junction.has_passed_flag && sd_junction.offset < 900 && sd_junction.offset > 0 &&
        sd_junction.junction_type == JunctionType::RampInto) {
      exit_jct              = sd_junction;
      has_junction_rampinto = true;
      SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   has_junction_rampinto. "
                          << "exit_jct.offset : " << exit_jct.offset;
      break;
    }
  }

  std::pair<double, double> firstRange = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  if (sd_dec_lane_info_.exists) {
    for (auto range : sd_dec_lane_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
    /* 出口和减速车道匹配 */
    /* 距离匹配的减速车道大于80 */
    if (has_junction_rampinto && std::abs(exit_jct.offset - firstRange.second) < 150 && firstRange.first > 80) {

      has_junction_rampinto_distant = true;
      SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   exit_jct matched dec lane, has_junction_rampinto_distant: "
                          << has_junction_rampinto_distant;
    } else if (has_junction_rampinto) {

      //has_junction_rampinto_distant = true;
      SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   exit_jct dismatched dec lane, has_junction_rampinto_distant: "
                          << has_junction_rampinto_distant;
    } else {
    }
  }
  SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   exit_jct.offset : " << exit_jct.offset << " firstRange.start: " << firstRange.first
                      << " firstRange.end:  " << firstRange.second;

  SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   has_junction_distant: " << has_junction_distant
                      << " has_junction_app_ramp: " << has_junction_app_ramp << " has_junction_rampinto: " << has_junction_rampinto
                      << " has_junction_rampinto_distant: " << has_junction_rampinto_distant
                      << " has_junction_app_merge: " << has_junction_app_merge;
  /* 远处无路口且距离下匝道还远，或者 远处无路口且也没有下匝道，或者 前方有途径匝道，正常过滤 */
  if ((!has_junction_distant && has_junction_rampinto_distant) || (!has_junction_distant && !has_junction_rampinto) ||
      has_junction_app_ramp || has_junction_app_merge) {
    /* 判断分叉 */
    DetectSplitLanes(rsplit_lane_filtered, all_bev_edges, split_indices_id_to_remove, has_junction_app_ramp, has_junction_app_merge,
                     acc_lane_id);
  }

  if (!split_indices_id_to_remove.empty()) {
    for (uint64_t id : split_indices_id_to_remove) {
      rsplit_lane_filtered.erase(std::remove(rsplit_lane_filtered.begin(), rsplit_lane_filtered.end(), id), rsplit_lane_filtered.end());
    }
    pre_split_lanes_to_move_.split_lane_ids = split_indices_id_to_remove;
    pre_split_lanes_to_move_.keep_count     = 0;
    // 拆解对应车道的前继及其前继车道的后继
    uint64_t prelane_id = 0;
    uint64_t pre_prelane_id = 0;
    for (auto &lane : GlobalBevMapOutPut->lane_infos) {
      if (lane.id == split_indices_id_to_remove[0]) {
        // lane.lane_type = BevLaneType::LANE_TYPE_HARBOR_STOP;  //debug
        lane.is_topo_disassembled = true;
        if (!lane.previous_lane_ids.empty()) {
          prelane_id = lane.previous_lane_ids[0];
          lane.previous_lane_ids.clear();  // 清空目标车道的前继
          SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   BEV lane ID: " << split_indices_id_to_remove[0] << " 's pre: " << prelane_id
                              << " is clear. ";
        }
        SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   BEV lane ID: " << split_indices_id_to_remove[0] << " is split lane ."
                            << " is_topo_disassembled : " << lane.is_topo_disassembled;
      }
    }
    // 前继的前继
    for (auto &lane : GlobalBevMapOutPut->lane_infos) {
      if (lane.id == prelane_id && !lane.previous_lane_ids.empty()) {
        lane.is_topo_disassembled = true;
        pre_prelane_id            = lane.previous_lane_ids[0];
        lane.previous_lane_ids.clear();  // 清空前继车道的前继
        SD_COARSE_MATCH_LOG << "  [FilterSplitLane]   BEV lane ID: " << lane.id << " is_topo_disassembled : " << lane.is_topo_disassembled
                            << " pre: " << pre_prelane_id << "is clear. ";
      }
    }
    // 删除prepre的后继，即pre
    for (auto &lane : GlobalBevMapOutPut->lane_infos) {
      if (lane.id == pre_prelane_id && !lane.next_lane_ids.empty()) {
        lane.next_lane_ids.erase(std::remove(lane.next_lane_ids.begin(), lane.next_lane_ids.end(), prelane_id), lane.next_lane_ids.end());
        SD_COARSE_MATCH_LOG << "[FilterSplitLane]   pre pre lane. " << pre_prelane_id
                            << " next_lane_ids: " << fmt::format(" [{}]  ", fmt::join(lane.next_lane_ids, ", ")) << " removed "
                            << prelane_id;
      }
    }
  } else if (split_indices_id_to_remove.empty() && !pre_split_lanes_to_move_.split_lane_ids.empty()) /* previous is not empty */
  {
    bool has_common_element = false;
    // 检查rsplit_lane_filtered中是否存在pre_split_lanes_to_move_.split_lane_ids中的元素
    for (int id : pre_split_lanes_to_move_.split_lane_ids) {
        if (std::find(rsplit_lane_filtered.begin(), rsplit_lane_filtered.end(), id) != rsplit_lane_filtered.end()) {
            has_common_element = true;
            break;
        }
    }
    if (has_common_element) {
        // 存在共同元素，执行删除操作并增加keep_count
        for (int id : pre_split_lanes_to_move_.split_lane_ids) {
            rsplit_lane_filtered.erase(std::remove(rsplit_lane_filtered.begin(), rsplit_lane_filtered.end(), id), rsplit_lane_filtered.end());
        }
        pre_split_lanes_to_move_.keep_count++;
    } else {
        // 不存在共同元素，清空split_lane_ids并将keep_count置为0
        pre_split_lanes_to_move_.split_lane_ids.clear();
        pre_split_lanes_to_move_.keep_count = 0;
    }
  } else {
    /* null */
  }
  /* 根据路口状态进行重置 */
  if (pre_split_lanes_to_move_.keep_count > 50 || !has_junction_app_ramp) {
    pre_split_lanes_to_move_.keep_count     = 0;
    pre_split_lanes_to_move_.split_lane_ids = {};
  }
  SD_COARSE_MATCH_LOG << "[FilterSplitLane]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(split_indices_id_to_remove, ", "));
  SD_COARSE_MATCH_LOG << "[FilterSplitLane]pre_split_lanes_to_move_: "
                      << fmt::format(" [{}]  ", fmt::join(pre_split_lanes_to_move_.split_lane_ids, ", "))
                      << " keep_count: " << pre_split_lanes_to_move_.keep_count;
  SD_COARSE_MATCH_LOG << "[FilterSplitLane] split_lane_filtered: " << fmt::format(" [{}]  ", fmt::join(rsplit_lane_filtered, ", "))
                      << " size: " << rsplit_lane_filtered.size();
  return rsplit_lane_filtered;
}

// 检测车道集合中首尾车道的分叉情况
void SdNavigationHighway::DetectSplitLanes(const std::vector<uint64_t>      &rsplit_lane_filtered,
                                           const std::vector<BevLaneMarker> &all_bev_edges,
                                           std::vector<uint64_t> &split_indices_id_to_remove, bool has_junction_app_ramp,
                                           bool has_junction_app_merge, uint64_t acc_lane_id) {
  // 车道数量不足时直接返回
  if (rsplit_lane_filtered.size() < 2) {
    SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Insufficient lanes (" << rsplit_lane_filtered.size() << "), skip detection";
    return;
  }

  // 特殊情况：只有两条车道
  if (rsplit_lane_filtered.size() == 2) {
    uint64_t lane1 = rsplit_lane_filtered[0];
    uint64_t lane2 = rsplit_lane_filtered[1];

    // 检查是否共享前继（分叉情况）  //CNOAC2-43213
    if ((acc_lane_id == 0) && CheckLanesShareAncestor(lane1, lane2)) {
      auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane2);
      if (HasAncestorWithSplitRight(lane_info)) {
        // 只移除最后一条车道（最右车道）
        split_indices_id_to_remove.push_back(lane2);
        SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Two-lane split detected. Removing last lane (rightmost): " << lane2;
      } else {
        SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Two-lane split detected. But the right lane is not split right.";
      }
    } else {
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Two-lane split is not detected.";
    }
    return;
  }

  //找出左侧和右侧边界
  BevLaneMarker best_left_edge;
  BevLaneMarker best_right_edge;
  double        max_left_overlap      = -1.0;
  double        max_right_overlap     = -1.0;
  double        min_overlap_threshold = 15;  //m
  for (auto &edge : all_bev_edges) {
    if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
        ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() < 0))) {
      if (edge.geos->size() < 2) {
        break;
      }

      // 直接使用第一个点和最后一个点的x坐标作为车道线的起点和终点
      double laneStart = edge.geos->front().x();
      double laneEnd   = edge.geos->back().x();

      // 确保车道线起点和终点的顺序正确
      if (laneStart > laneEnd) {
        std::swap(laneStart, laneEnd);
      }
      double overlap_dis = laneEnd - laneStart;
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] bdry  length: " << overlap_dis;
      if (overlap_dis > max_right_overlap && overlap_dis >= min_overlap_threshold) {
        max_right_overlap = overlap_dis;
        best_right_edge   = edge;
      }
    } else if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
               ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() > 0))) {
      if (edge.geos->size() < 2) {
        break;
      }

      // 直接使用第一个点和最后一个点的x坐标作为车道线的起点和终点
      double laneStart = edge.geos->front().x();
      double laneEnd   = edge.geos->back().x();

      // 确保车道线起点和终点的顺序正确
      if (laneStart > laneEnd) {
        std::swap(laneStart, laneEnd);
      }
      double overlap_dis = laneEnd - laneStart;
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] bdry  length: " << overlap_dis;
      if (overlap_dis > max_left_overlap && overlap_dis >= min_overlap_threshold) {
        max_left_overlap = overlap_dis;
        best_left_edge   = edge;
      }
    }
  }
  SD_COARSE_MATCH_LOG << "[DetectSplitLanes] best_left_edge: " << best_left_edge.id << " best_right_edge: " << best_right_edge.id;
  /* 途径汇出口 */
  if (has_junction_app_ramp) {
    // 检测最后一条车道是否分叉（与倒数第二条车道共享前继）
    size_t   last_idx         = rsplit_lane_filtered.size() - 1;
    uint64_t last_lane        = rsplit_lane_filtered[last_idx];
    uint64_t second_last_lane = rsplit_lane_filtered[last_idx - 1];
    //uint64_t third_last_lane  = rsplit_lane_filtered[last_idx - 2];
    if (CheckLanesShareAncestor(last_lane, second_last_lane)) {
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Last lane " << last_lane << " is split type before app_ramp junction";
      split_indices_id_to_remove.push_back(last_lane);
    }
  } else if (has_junction_app_merge) {
    // 检测第一条车道是否分叉（与第二条车道共享前继）
    uint64_t first_lane  = rsplit_lane_filtered[0];
    uint64_t second_lane = rsplit_lane_filtered[1];
    uint64_t third_lane  = rsplit_lane_filtered[2];
    if (CheckLanesShareAncestor(first_lane, second_lane)) {
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] First lane " << first_lane << " is split type";
      split_indices_id_to_remove.push_back(first_lane);
    }
  } else {
    // 检测第一条车道是否分叉（与第二条车道共享前继）
    uint64_t first_lane  = rsplit_lane_filtered[0];
    uint64_t second_lane = rsplit_lane_filtered[1];
    uint64_t third_lane  = rsplit_lane_filtered[2];
    if (CheckLanesShareAncestor(first_lane, second_lane)) {
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] First lane " << first_lane << " is split type";
      split_indices_id_to_remove.push_back(first_lane);
    }
    //如果左二车道距离左边界很近，则判断左二左三是否分叉

    if (split_indices_id_to_remove.empty()) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(second_lane);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_left_edge.geos->empty())) {
        //double overlap_dis      = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
        double dis_to_rightbdry = cem::fusion::LaneGeometry::GetDistanceBetweenLines(*single_lane->geos, *best_left_edge.geos);

        SD_COARSE_MATCH_LOG << "[DetectSplitLanes] dis line " << single_lane->id << " to " << best_left_edge.id
                            << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry < 1.5 && CheckLanesShareAncestor(second_lane, third_lane)) {
          split_indices_id_to_remove.push_back(second_lane);
        }
      }
    }

    // 检测最后一条车道是否分叉（与倒数第二条车道共享前继）
    size_t   last_idx         = rsplit_lane_filtered.size() - 1;
    uint64_t last_lane        = rsplit_lane_filtered[last_idx];
    uint64_t second_last_lane = rsplit_lane_filtered[last_idx - 1];
    uint64_t third_last_lane  = rsplit_lane_filtered[last_idx - 2];
    if (CheckLanesShareAncestor(last_lane, second_last_lane)) {
      SD_COARSE_MATCH_LOG << "[DetectSplitLanes] Last lane " << last_lane << " is split type";
      split_indices_id_to_remove.push_back(last_lane);
    }
    //如果右二车道距离左边界很近，则判断右二右三是否分叉

    if (split_indices_id_to_remove.empty()) {
      auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(second_last_lane);

      if ((single_lane) && (!single_lane->geos->empty()) && (!best_right_edge.geos->empty())) {
        double overlap_dis      = calculateOverlapDistance(sd_harbor_stop_info_, *single_lane->geos);
        double dis_to_rightbdry = cem::fusion::LaneGeometry::GetDistanceBetweenLines(*single_lane->geos, *best_right_edge.geos);

        SD_COARSE_MATCH_LOG << "[DetectSplitLanes] dis line " << single_lane->id << " to " << best_right_edge.id
                            << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry < 1.5 && CheckLanesShareAncestor(second_last_lane, third_last_lane)) {
          split_indices_id_to_remove.push_back(second_last_lane);
        }
      }
    }
  }
}

// 检查两条车道是否共享前继车道
bool SdNavigationHighway::CheckLanesShareAncestor(uint64_t lane1, uint64_t lane2) {
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

//收集前继车道id
std::unordered_set<uint64_t> SdNavigationHighway::CollectPreLanes(uint64_t start_id) const {
  std::unordered_set<uint64_t> prelanes;
  std::unordered_set<uint64_t> current_level = {start_id};  // 第0层：当前车道

  for (int level = 0; level <= 3; ++level) {  // 最多3层（0~3）
    // 将当前层车道加入祖先集合（第0层是当前车道本身）
    for (uint64_t lane : current_level) {
      prelanes.insert(lane);
    }

    // 若已到第3层，停止收集
    if (level == 3)
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

std::string SdNavigationHighway::PrintProcessedLanes(const std::unordered_set<uint64_t> &lanes) {
  std::stringstream ss;
  for (auto id : lanes) {
    if (ss.tellp() > 0) {
      ss << ", ";
    }
    ss << id;
  }
  return ss.str();
}

void SdNavigationHighway::UpdateHarborLaneType(const std::vector<uint64_t> &road_selected,
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

bool SdNavigationHighway::EgoInMainRoad() {
  // 高速场景：走 LD
  auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route)
    return false;
  const uint64_t ego_sec = ld_route->navi_start.section_id;
  for (const auto &sec : ld_route->sections) {
    if (sec.id == ego_sec) {
      const bool isInMainRoad = LD_IsMainRoadSection(sec);
      SD_COARSE_MATCH_LOG << fmt::format("[LD] Section {} isInMainRoad: {}", ego_sec, isInMainRoad);
      return isInMainRoad;
    }
  }
  return false;
}
/* 查找LD地图的加速车道邻车道在自车脚下的车道id */
uint64_t SdNavigationHighway::FindOriginNormalLaneIdFromAccLane(const cem::message::env_model::LaneInfo *acc_lane,
                                                                uint64_t                                 ego_section_id) {
  // Step 1: 获取加速车道左侧的车道（通过 left_lane_id）
  uint64_t left_lane_id = acc_lane->left_lane_id;
  if (left_lane_id == 0) {
    SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] No left lane for acc lane " << acc_lane->id;
    return 0;
  }

  auto left_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(left_lane_id);
  if (!left_lane_info) {
    SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Failed to get left lane info for id: " << left_lane_id;
    return 0;
  }

  // 确保是普通车道
  if (left_lane_info->type != cem::message::env_model::LaneType::LANE_NORMAL) {
    SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Left lane is not normal type, type: " << static_cast<int>(left_lane_info->type);
    return 0;
  }

  SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Found left normal lane: " << left_lane_id
                      << " in section: " << left_lane_info->section_id;

  // Step 2: 回溯 previous_lane_ids 链，查找是否属于 ego_section_id
  std::set<uint64_t>                       visited;  // 防止环路
  const cem::message::env_model::LaneInfo *current = left_lane_info;

  while (current && visited.insert(current->id).second) {
    // 判断当前车道是否属于自车所在 section
    if (current->section_id == ego_section_id) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Found origin lane in ego section: " << current->id << " at section "
                          << current->section_id;
      return current->id;
    }

    // 移动到前驱车道（注意：可能有多个前驱，这里取第一个作为主路径）
    if (current->previous_lane_ids.empty()) {
      break;
    }

    uint64_t prev_id   = current->previous_lane_ids[0];  // 认为第一个为主流向前驱
    auto     prev_lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(prev_id);
    if (!prev_lane) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Failed to get previous lane info: " << prev_id;
      break;
    }

    current = prev_lane;
  }

  SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] No matching lane found in ego section from acc lane " << acc_lane->id;
  return 0;  // 未找到
}
/* 查找LD地图的加速车道邻车道在自车脚下的车道id,添加退出距离 */
uint64_t SdNavigationHighway::FindOriginNormalLaneIdFromAccLane2(const cem::message::env_model::LaneInfo *acc_lane, uint64_t ego_section_id,
                                                                 double max_search_distance /* = 800.0 */) {
  // Step 1: 获取加速车道左侧的车道
  uint64_t left_lane_id = acc_lane->left_lane_id;
  if (left_lane_id == 0) {
    SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] No left lane for acc lane " << acc_lane->id;
    return 0;
  }

  auto left_lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(left_lane_id);
  if (!left_lane_info || left_lane_info->type != cem::message::env_model::LaneType::LANE_NORMAL) {
    SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Left lane is invalid or not normal type, id: " << left_lane_id;
    return 0;
  }

  SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Start tracing from left normal lane: " << left_lane_id << " in section "
                      << left_lane_info->section_id;

  // Step 2: 回溯 previous_lane_ids，累计长度不超过 max_search_distance
  std::set<uint64_t>                       visited;
  const cem::message::env_model::LaneInfo *current            = left_lane_info;
  double                                   accumulated_length = 0.0;  // 累计回溯的车道总长度

  while (current && visited.insert(current->id).second) {
    // 检查当前车道是否在自车所在的 section
    if (current->section_id == ego_section_id) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Found origin lane in ego section: " << current->id
                          << " (accumulated backtrack: " << accumulated_length << " m)";
      return current->id;
    }

    // 累加当前车道长度
    accumulated_length += current->length;

    // 判断是否超过最大搜索距离
    if (accumulated_length >= max_search_distance) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Max search distance (" << max_search_distance << "m) exceeded. Stopping search.";
      break;
    }

    // 移动到前驱车道（取第一个为主流路径）
    if (current->previous_lane_ids.empty()) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] No more previous lanes. Search ended.";
      break;
    }

    uint64_t prev_id = 0;

    // 如果前驱车道数量大于1，需要根据merge属性选择合适的前驱车道
    if (current->previous_lane_ids.size() > 1) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Found " << current->previous_lane_ids.size()
                          << " previous lanes, checking merge attributes for lane " << current->id;

      // 遍历所有前驱车道，优先选择无merge属性的车道
      for (uint64_t candidate_prev_id : current->previous_lane_ids) {
        auto candidate_prev_lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(candidate_prev_id);
        if (!candidate_prev_lane) {
          SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Failed to get candidate previous lane info: " << candidate_prev_id;
          continue;
        }

        // 检查merge属性，如果无merge属性则选择该车道
        if (candidate_prev_lane->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_NONE) {
          prev_id = candidate_prev_id;
          SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Selected non-merge previous lane: " << prev_id;
          break;
        }
      }

      // 如果没有找到无merge属性的前驱车道，则使用第一个前驱车道作为默认选择
      if (prev_id == 0) {
        prev_id = current->previous_lane_ids[0];
        SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] No non-merge previous lane found, using first: " << prev_id;
      }
    } else {
      // 如果只有一个前驱车道，直接使用
      prev_id = current->previous_lane_ids[0];
    }
    auto     prev_lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(prev_id);
    if (!prev_lane) {
      SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Failed to get previous lane info: " << prev_id;
      break;
    }

    current = prev_lane;
  }

  SD_COARSE_MATCH_LOG << "[FindOriginNormalLaneId] Could not find lane in ego section within " << max_search_distance << "m backtracking.";
  return 0;  // 未找到符合条件的车道
}

void SdNavigationHighway::SetACCLaneInfo(AccLaneInfo &sd_acc_lane_info) {
  sd_acc_lane_info = sd_acc_lane_info_;
  SD_COARSE_MATCH_LOG << "[SetACCLaneInfo] Normal lane id: " << sd_acc_lane_info_.ego_normal_lane_id;
}

/**
     * @brief 回溯检查当前车道的主前继链中是否在最多5层内存在 split_topo_extend == TOPOLOGY_SPLIT_RIGHT
     *        
     * @param start_lane 起始车道指针
     * @return true 如果发现符合条件的前继车道
     */
bool SdNavigationHighway::HasAncestorWithSplitRight(const BevLaneInfo *start_lane) const {
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

/*
   * @brief 判断是否是途径汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
bool SdNavigationHighway::IsApproachMergeJCT(std::vector<JunctionInfo *> &target_sd_junctions, JunctionInfo &merge_jct) {
  bool   approaching_merge = false;  //途径汇入口
  bool   near_exit         = false;  //接近汇出口
  double exit_offset       = 2001;   //接近汇出口的距离
  if (!target_sd_junctions.empty()) {
    for (auto junction : target_sd_junctions) {
      if (junction->offset < 800 && junction->offset > 0.0 && (junction->junction_type == JunctionType::RampInto)) {
        near_exit   = true;
        exit_offset = junction->offset;
      }
      if (junction->offset < 800 && junction->offset > 0.0 && (junction->junction_type == JunctionType::ApproachRampMerge)) {
        /* 无汇出口或汇出口距离大于途径汇入 */
        if ((!near_exit) || (near_exit && exit_offset > junction->offset)) {
          approaching_merge = true;
          merge_jct         = *junction;
          SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge ,offset: " << junction->offset
                              << StrJunctionType(junction->junction_type)
                              << "  junction's normal lane num:  " << junction->main_road_lane_nums;
          break;
        }
      }
    }
  }
  if (!sd_junction_infos_.empty()) {
    for (auto junction : sd_junction_infos_) {
      if (junction.offset < 800 && junction.offset > 0.0 && (junction.junction_type == JunctionType::RampInto)) {
        near_exit   = true;
        exit_offset = junction.offset;
      }
      if (junction.offset < 800 && junction.offset > 0.0 && (junction.junction_type == JunctionType::ApproachRampMerge)) {
        /* 无汇出口或汇出口距离大于途径汇入 */
        if ((!near_exit) || (near_exit && exit_offset > junction.offset)) {
          approaching_merge = true;
          merge_jct         = junction;
          SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge ,offset: " << junction.offset
                              << StrJunctionType(junction.junction_type)
                              << "  junction's normal lane num:  " << junction.main_road_lane_nums;
          break;
        }
      }
    }
  }
  SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT] APPROACHING junction merge:  " << approaching_merge;
  return approaching_merge;
}

/*
   * @brief 判断是否是匝道汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
bool SdNavigationHighway::IsRampMergeJCT(std::vector<JunctionInfo *> &target_sd_junctions, JunctionInfo &merge_jct) {
  bool   ramp_merge  = false;  //途径汇入口
  bool   near_exit   = false;  //接近汇出口
  double exit_offset = 2001;   //接近汇出口的距离
  if (!target_sd_junctions.empty()) {
    for (auto junction : target_sd_junctions) {
      if (junction->offset < 200 && junction->offset > -300.0 && (junction->junction_type == JunctionType::RampMerge)) {
        ramp_merge = true;
        break;
      }
    }
  }
  SD_COARSE_MATCH_LOG << " [IsApproachMergeJCT]  junction ramp_merge:  " << ramp_merge;
  return ramp_merge;
}

double SdNavigationHighway::CalculateCurvedLaneWidth2(const std::vector<Eigen::Vector2f> &lane1,
                                                      const std::vector<Eigen::Vector2f> &lane2) {
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

/* harbor 车道能否过滤的校验 */
bool SdNavigationHighway::RemoveHarborlaneCheck(const HarborStopInfo  &sd_harbor_stop_info_,
                                                std::vector<uint64_t> &road_selected_buslane_filtered) {
  bool remove_enable      = false;
  bool right_lenght_check = false;
  bool left_lenght_check  = false;
  if (sd_harbor_stop_info_.exists) {
    const auto &firstRange      = sd_harbor_stop_info_.ranges[0];
    double      harborStart     = firstRange.first;
    double      harborEnd       = firstRange.second;
    double      harbor_length   = harborEnd - harborStart;
    auto        right_most_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.back());
    auto        left_most_lane  = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(road_selected_buslane_filtered.front());

    if ((sd_harbor_stop_info_.is_left_most) && (left_most_lane) && (left_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO)) {
      double left_lane_length = left_most_lane->geos->back().x() - left_most_lane->geos->front().x();
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] harbor_length :  " << harbor_length << "left_lane_length " << left_lane_length;
      if (left_lane_length < harbor_length + 100) {
        left_lenght_check = true;
      } else {
        SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck]  check fail for lane length is :  " << left_lane_length << " <  " << harbor_length
                            << " + 100";
      }
    } else {
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck]  check fail for lane position is :  " << left_most_lane->position
                          << " not ego: " << (int)BevLanePosition::LANE_LOC_EGO;
    }
    if ((sd_harbor_stop_info_.is_right_most) && (right_most_lane) && (right_most_lane->position != (int)BevLanePosition::LANE_LOC_EGO) &&
        (right_most_lane->previous_lane_ids.size() < 2)) {
      double right_lane_length = right_most_lane->geos->back().x() - right_most_lane->geos->front().x();
      if (right_lane_length < harbor_length + 100) {
        right_lenght_check = true;
      } else {
        SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck]  check fail for lane length is :  " << right_lane_length << " <  " << harbor_length
                            << " + 100";
      }
    } else {
      SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck]  check fail for lane position is :  " << right_most_lane->position
                          << " not ego: " << (int)BevLanePosition::LANE_LOC_EGO;
    }

    remove_enable = (left_lenght_check || right_lenght_check);
  }
  SD_COARSE_MATCH_LOG << " [RemoveHarborlaneCheck] length & position check result :  " << remove_enable;
  return remove_enable;
}

/* 单 harbor 场景的判断,暂时只看右侧 */
bool SdNavigationHighway::OnlyHarborSceneCheck(const HarborStopInfo    &sd_harbor_stop_info_,
                                               const EmergencyLaneInfo &g_emergency_lane_info_) {
  bool result      = false;
  bool right_check = false;
  bool left_check  = false;
  if (sd_harbor_stop_info_.exists && !g_emergency_lane_info_.right.exists) {
    result = true;
    SD_COARSE_MATCH_LOG << " [OnlyHarborSceneCheck] OnlyHarborSceneCheck check result (right side) :  " << result;
  } else {
    SD_COARSE_MATCH_LOG << " [OnlyHarborSceneCheck]  check fail cause emergency is: " << g_emergency_lane_info_.right.exists;
  }
  SD_COARSE_MATCH_LOG << " [OnlyHarborSceneCheck]   check result :  " << result;
  return result;
}

/* 左侧锥筒施工场景的判断：仅判断左侧成排锥筒将左侧车道拦住的场景 */
/*
   * @brief 左侧锥筒施工场景的判断
   * @param  
   * @return bool
  */
bool SdNavigationHighway::LeftConstructionZoneCheck(const std::vector<uint64_t>      &road_selected_buslane_filtered,
                                                    const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                                                    uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid) {
  bool   result                = false;
  bool   final_result                = false;
  int    lane_count            = 0;
  double min_overlap_threshold = 15.0;
  for (auto &id : road_selected_buslane_filtered) {
    auto single_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(id);
    if ((single_lane) && (!single_lane->geos->empty())) {
      SD_COARSE_MATCH_LOG << "single_lane_id: " << single_lane->id << " lane_start_x: " << single_lane->geos->front().x()
                          << " lane_start_y: " << single_lane->geos->front().y() << " lane_end_x: " << single_lane->geos->back().x()
                          << " lane_end_y: " << single_lane->geos->back().y();


      double overlap_dis = calculateOverlapDisinRange(-50.0, 150.0, *single_lane->geos);
      SD_COARSE_MATCH_LOG << "overlap_dis:" << overlap_dis;
      if ((overlap_dis > min_overlap_threshold)) {
        lane_count++;
      }
    }
  }

  /*若没有通过拓扑找到港湾车道，则利用每条线到右边界的距离进行判断*/
  Edge_log_print(all_bev_edges);

    // 步骤1: 左侧的正常路沿和锥筒路沿

    BevLaneMarker best_left_edge;
    BevLaneMarker best_right_edge;
    BevLaneMarker best_cone_edge;
    double        max_overlap_left  = -1.0;
    double        max_overlap_right = -1.0;
    double        max_cone_overlap  = -1.0;
    /* 左侧路沿 */
    for (auto &edge : all_bev_edges) {
      if (((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
           ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() > 0))) &&
          edge.type != static_cast<uint32_t>(BoundaryType::FENCE_BOUNDARY)) {
        double overlap_dis = calculateOverlapDisinRange(-50.0, 150.0, *edge.geos);
        SD_COARSE_MATCH_LOG << "[LeftConstructionZoneCheck]  overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap_left && overlap_dis >= min_overlap_threshold) {
          max_overlap_left = overlap_dis;
          best_left_edge   = edge;
        }
      }
      if (((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
           ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() < 0))) &&
          edge.type != static_cast<uint32_t>(BoundaryType::FENCE_BOUNDARY)) {
        double overlap_dis = calculateOverlapDisinRange(-50.0, 150.0, *edge.geos);
        SD_COARSE_MATCH_LOG << "[LeftConstructionZoneCheck]  overlap_dis:" << overlap_dis;
        if (overlap_dis > max_overlap_right && overlap_dis >= min_overlap_threshold) {
          max_overlap_right = overlap_dis;
          best_right_edge   = edge;
        }
      }
      if (((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__LEFT) ||
           ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) && (!edge.geos->empty()) && (edge.geos->front().y() > 0))) &&
          edge.type == static_cast<uint32_t>(BoundaryType::FENCE_BOUNDARY)) {
        double overlap_dis = calculateOverlapDisinRange(-50.0, 150.0, *edge.geos);
        SD_COARSE_MATCH_LOG << "[LeftConstructionZoneCheck]  overlap_dis:" << overlap_dis;
        if (overlap_dis > max_cone_overlap && overlap_dis >= min_overlap_threshold) {
          max_cone_overlap = overlap_dis;
          best_cone_edge   = edge;
        }
      }
    }
    // 步骤2: 计算两条边界线的宽度
    uint64_t best_lane_id = 0;
    float    min_distance = std::numeric_limits<float>::max();

    if ( 0 /*(!best_left_edge.geos->empty()) && (!best_cone_edge.geos->empty())*/) {
      /* 判断左侧路沿距离锥筒是否在大于一个车道 */
      double dis_to_rightbdry = CalculateCurvedLaneWidth2(*best_cone_edge.geos, *best_left_edge.geos);
      SD_COARSE_MATCH_LOG << "[LeftConstructionZoneCheck] dis line " << best_cone_edge.id << " to " << best_left_edge.id
                          << "  left bdry is : " << dis_to_rightbdry;
      if (3 < dis_to_rightbdry && dis_to_rightbdry < 5.5) {
        SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] check by left_edge & cone edge. ";
        result = true;
      }
    }else if((!best_right_edge.geos->empty()) && (!best_cone_edge.geos->empty())){
      /* 判断左侧锥筒和右侧路沿距离是否小于实际车道数*宽度 */
      double road_width = CalculateCurvedLaneWidth2(*best_cone_edge.geos, *best_right_edge.geos);
      SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] road_width: "<<road_width<<"lanetype_list.size()*3.5: "<<lanetype_list.size()*3.5;
      if(lanetype_list.size()*3.5 > road_width){
        SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] check by road_width. ";
        result = true;
      }
    }else if(!best_cone_edge.geos->empty()){
      /* 判断左侧存在锥筒路沿 */
      SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] check only by left cone edge. ";
      result = true;
    }
SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] result: " << result;
  

/* 5帧check，50帧keep */
if (result) {
  if (construction_zone_info_.pre_ConstructionZone) {
    final_result = result;
    construction_zone_info_.check_count++;
    construction_zone_info_.keep_count = 0;
  } else {
    final_result                        = result;
    construction_zone_info_.check_count = 1;
    construction_zone_info_.keep_count  = 0;
  }

} else if (construction_zone_info_.check_count > 5 && !result && construction_zone_info_.pre_ConstructionZone) {
  final_result = construction_zone_info_.pre_ConstructionZone;
  construction_zone_info_.keep_count++;
}else if (!result && !construction_zone_info_.pre_ConstructionZone && construction_zone_info_.keep_count > 0) {
  final_result = true;
  construction_zone_info_.keep_count++;
} else if (!result && !construction_zone_info_.pre_ConstructionZone) {
  /* 默认情况 */
  construction_zone_info_.pre_final_ConstructionZone = false;
  construction_zone_info_.pre_ConstructionZone = false;
  construction_zone_info_.check_count          = 0;
  construction_zone_info_.keep_count           = 0;
} else {
}
 construction_zone_info_.pre_final_ConstructionZone = final_result;
 construction_zone_info_.pre_ConstructionZone = result;


 /* 保持超过50帧，清空状态 */
 if (construction_zone_info_.keep_count > 100) // 50-> 100
 {
  construction_zone_info_.pre_final_ConstructionZone = false;
  construction_zone_info_.pre_ConstructionZone = false;
  construction_zone_info_.check_count = 0;
  construction_zone_info_.keep_count = 0;
 }

 SD_COARSE_MATCH_LOG << " [LeftConstructionZoneCheck] final_result: " << final_result
                     << " pre_final_result: " << construction_zone_info_.pre_final_ConstructionZone << " result " << result
                     << " pre_result " << construction_zone_info_.pre_ConstructionZone << " check_count "
                     << construction_zone_info_.check_count << " keep_count " << construction_zone_info_.keep_count;
 return final_result;
}

/*
   * @brief 过滤掉减速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉减速车道的车道 ID 列表
  */
std::vector<uint64_t> SdNavigationHighway::FilterDecLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                                         const std::vector<uint64_t> &road_selected, BevMapInfoConstPtr &raw_bev_map,
                                                         BevMapInfoPtr &GlobalBevMapOutPut) {
  // 判断road_selected是否为空，空则返回
  if (road_selected.empty()) {
    SD_COARSE_MATCH_LOG << "[FilterDecLane]  road_selected is empty! ";
    return road_selected;
  }
  auto                 &all_bev_edges = raw_bev_map->edges;
  std::vector<uint64_t> dec_lane_filtered(road_selected);
  std::vector<uint64_t> dec_indices_id_to_remove;
  SD_COARSE_MATCH_LOG << "Entering FilterDecLane  with inputs: " << fmt::format(" [{}]  ", fmt::join(road_selected, ", "))
                      << " size: " << road_selected.size();

  bool has_junction_app_ramp = false; /* [0,300m]的最近路口为途径下匝道 */
  bool is_ramp_lane          = false; /*  最右车道是下匝道的bev车道 */
  const JunctionInfo *junction_info_ptr = nullptr;
  for (auto &sd_junction_tmp : sd_junction_infos_) {
    SD_COARSE_MATCH_LOG << "[SDNOA Junction] ,id : " << sd_junction_tmp.junction_id
                        << " ,type: " << StrNaviJunctionHighway(static_cast<int>(sd_junction_tmp.junction_type))
                        << " ,offset:  " << sd_junction_tmp.offset << "  ,main_road_lane_nums: " << sd_junction_tmp.main_road_lane_nums
                        << " ,direction: " << StrNaviSplitDirectionHighway(sd_junction_tmp.split_direction)
                        << " ,target_road_lane_nums: " << sd_junction_tmp.target_road_lane_nums
                        << " ,passed_flag: " << (int)sd_junction_tmp.has_passed_flag
                        << " ,effected_flag: " << (int)sd_junction_tmp.has_effected_flag;
  }

  for (auto &sd_junction : sd_junction_infos_) {
    if (!sd_junction.has_passed_flag && sd_junction.offset < 200 && sd_junction.offset > -30 &&
        sd_junction.junction_type == JunctionType::ApproachRampInto) {
      has_junction_app_ramp = true;
      junction_info_ptr = &sd_junction;
      break;
    }
  }

  std::pair<double, double> firstRange = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  if (sd_dec_lane_info_.exists) {
    for (auto range : sd_dec_lane_info_.ranges) {
      if (range.second > 0) {
        firstRange = range;
        break;
      }
    }
  }

  std::vector<std::vector<uint64_t>> non_navi_ids;
  if (junction_info_ptr) {
    non_navi_ids = topology_extractor_.ExtractNonNaviLanesFromJunction(junction_info_ptr);

    SD_COARSE_MATCH_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Result: [");
    for (size_t i = 0; i < non_navi_ids.size(); ++i) {
      SD_COARSE_MATCH_LOG << fmt::format("  [{}]", fmt::join(non_navi_ids[i], ", "));
      if (i < non_navi_ids.size() - 1) {
        SD_COARSE_MATCH_LOG << ",";
      }
    }
    SD_COARSE_MATCH_LOG << fmt::format("]");
  } else {
    SD_MERGE_LOG << "[ExtractNonNaviLanesFromJunction] junction_info_ptr is null";
  }

  SD_COARSE_MATCH_LOG << "  [FilterDecLane] dec lane info: "<< " has_junction_app_ramp: " << has_junction_app_ramp<<" firstRange.start: " << firstRange.first
                      << " firstRange.end:  " << firstRange.second;

  /* 减速车道80m前或者路口200m前 */
  if (has_junction_app_ramp) {
    /* 判断最右车道 */
    if(dec_lane_filtered.size() > 1){
      dec_indices_id_to_remove.push_back(dec_lane_filtered.back());
    }
  }

  if (!dec_indices_id_to_remove.empty()) {
    /* 判断最右车道是否是下匝道的车道 */
    auto &ld_match_infos = INTERNAL_PARAMS.ld_match_info_data.GetMatchInfo();

    // 打印完整的ld_match_infos信息
    SD_COARSE_MATCH_LOG << "[FilterDecLane] ld_match_infos size: " << ld_match_infos.size();
    for (const auto &[bev_id, matched_ld_ids] : ld_match_infos) {
      SD_COARSE_MATCH_LOG << "[FilterDecLane] bev_id: " << bev_id << " matched_ld_ids: [";
      for (size_t i = 0; i < matched_ld_ids.size(); ++i) {
        SD_COARSE_MATCH_LOG << matched_ld_ids[i];
        if (i < matched_ld_ids.size() - 1) {
          SD_COARSE_MATCH_LOG << ", ";
        }
      }
      SD_COARSE_MATCH_LOG << "]";
    }

    uint64_t bev_id = dec_indices_id_to_remove.front();

    /* 检查bev_id是否存在 */
    if (ld_match_infos.find(bev_id) != ld_match_infos.end()) {
      auto &matched_ld_ids = ld_match_infos.at(bev_id);
      for (auto &vec : non_navi_ids) {
        for (auto &element : vec) {
          if (std::find(matched_ld_ids.begin(), matched_ld_ids.end(), element) != matched_ld_ids.end()) {
            SD_COARSE_MATCH_LOG << "  [FilterDecLane] dec lane is matched, bev_id: " << bev_id << " map_id: " << element;
            is_ramp_lane = true;
          } else {
            
          }
        }
      }
    } else {
      // 处理键不存在的情况（记录错误/跳过）
      SD_COARSE_MATCH_LOG << "  [FilterDecLane] bev_id " << bev_id << " not found in ld_match_infos!";
    }
  }

  if (!dec_indices_id_to_remove.empty() && is_ramp_lane) {

    for (uint64_t id : dec_indices_id_to_remove) {
      dec_lane_filtered.erase(std::remove(dec_lane_filtered.begin(), dec_lane_filtered.end(), id), dec_lane_filtered.end());
    }
    SD_COARSE_MATCH_LOG << "[FilterDecLane]erase the back ramp/dec lane  ";
  } else {

  }

  SD_COARSE_MATCH_LOG << "[FilterDecLane]indices_id_to_remove: " << fmt::format(" [{}]  ", fmt::join(dec_indices_id_to_remove, ", "));

  SD_COARSE_MATCH_LOG << "[FilterDecLane] dec_lane_filtered result: " << fmt::format(" [{}]  ", fmt::join(dec_lane_filtered, ", "))
                      << " size: " << dec_lane_filtered.size();
  return dec_lane_filtered;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem
