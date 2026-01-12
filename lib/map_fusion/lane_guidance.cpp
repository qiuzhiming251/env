#include "lane_guidance.h"
#include <log_custom.h>
#include <cmath>
#include <limits>
#include <Eigen/QR>
#include <common/utils/lane_geometry.h>
#include <fmt/format.h>
#include "base/sensor_data_manager.h"
#include "lib/common/utility.h"
#include "lib/sd_navigation/routing_map_debug.h"
#include "opencv2/opencv.hpp"

namespace cem {
namespace fusion {

namespace {
constexpr float    kSwitchMapThreshold               = 150;
constexpr float    kVirtualLaneLength                = 1000;
constexpr int      kMaxExtendedLength                = 100;
constexpr int      kMaxLoopNumber                    = 20;
constexpr uint64_t kMaxBevTrackID                    = 110;
constexpr uint64_t kVirtualLaneID                    = std::numeric_limits<uint64_t>::max();
constexpr uint64_t kVirtualJunctionID                = std::numeric_limits<uint64_t>::max();
constexpr double   kDistanceThreshold                = 1e-3;
constexpr double   kCutPointHierarchicalThreshold    = 3;
constexpr float    kEmergencyFilterMaxWidth          = 5.0;
constexpr double   kStoplineFilterMaxCos             = 0.324;  // 0.324 及自车车道与停止线夹角小于 70°则stopline不参与切割车道线
constexpr double   kLaneAssociationJunctionThreshold = 50;
constexpr double   kMaxDistanceToJunction            = 100.0;
constexpr double   kToggleGuidanceThreshold          = 25.0;
}  // namespace

LaneGuidance::LaneGuidance() {}

void LaneGuidance::Process(const RoutingMapPtr routing_map_ptr, 
                           const cem::fusion::SdJunctionInfoCityPtr junctions_ptr, 
                           DetectBevMap &aligned_map,
                           const std::vector<cem::message::env_model::StopLine>  &stop_lines,
                           std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result) {
  if (aligned_map.bev_map_ptr == nullptr) {
    return;
  }

  BevMapInfo bev_map = *aligned_map.bev_map_ptr;
  bev_map.map_type = cem::message::sensor::MapType::BEV_MAP;

  SD_MERGE_LOG << fmt::format("[LaneGuidanceCity] Raw BEV lane size: {}", bev_map.lane_infos.size());
  for (const auto &lane : bev_map.lane_infos) {
    SD_MERGE_LOG << fmt::format("{}", lane);
  }

  // 获取merge拓扑信息
  is_highway_ = bev_map.env_info.is_on_highway_;
  SD_MERGE_LOG << fmt::format("[LaneGuidanceCity] is_highway: {}", is_highway_);
  // std::vector<MergeDetail> merge_details_new;
  // topology_extractor_.SetIsHighway(is_highway_);
  // topology_extractor_.GetMergeTopologiesFromMap(merge_details_new);
  // topology_extractor_.PrintMergeDetails(merge_details_new);

  if (routing_map_ptr) {
    SDStateFusion(bev_map, routing_map_ptr);
    // SetBevEmergencyLane(bev_map, fusion_emergency_lane_id);
  }

  T_local_ego_ = aligned_map.Twb;
  T_ego_local_ = T_local_ego_.inverse();

  stop_lines_ = stop_lines;

  InitRecommendedLaneAction(bev_map, guide_lane_result, routing_map_ptr, junctions_ptr);

  // AINFO << "*******JunctionInfoBefore******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // for (auto &lane : bev_map.lane_infos){
  //   AINFO << "lane ID: " << lane.id << ", position: " << static_cast<int>(lane.position) << ", turn_type: " << static_cast<int>(lane.bev_turn_type);
  // }
  // AINFO << "*************************";

  InitLaneTurnType(bev_map, junctions_ptr);

  // AINFO << "*******JunctionInfoAfter******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // for (auto &lane : bev_map.lane_infos){
  //   AINFO << "lane ID: " << lane.id << ", position: " << static_cast<int>(lane.position) << ", turn_type: " << static_cast<int>(lane.bev_turn_type);
  // }
  // AINFO << "*************************";

  InsertMergeSplitPoints(bev_map);

  topology_extractor_.SetBevLaneMergeTopoNew(routing_map_ptr ,bev_map);

  SD_MERGE_LOG << fmt::format("[LaneGuidanceCity] Now BEV lane size: {}", bev_map.lane_infos.size());
  for (const auto &lane : bev_map.lane_infos) {
    SD_MERGE_LOG << fmt::format("{}", lane);
  }

  SpecialLaneAssignment(bev_map);
  RemoveSpecialLaneTopo(bev_map, routing_map_ptr, guide_lane_result);

  //  Remove non-existent successor index
  RemoveNonExistentSuccessor(bev_map);
  RemoveNonExistentPredecessor(bev_map);

  TraceFarRecommendLane(bev_map, guide_lane_result);

  // Horozontal road divider
  road_divider_.T_local_ego_ = T_local_ego_;
  road_divider_.is_highway_ = is_highway_;
  road_divider_.Process(bev_map);

  std::vector<std::pair<uint64_t, uint64_t>> roads_span;
  RoadSpan(bev_map.lane_infos, roads_span);

  bev_map.route.id = 1;
  std::vector<BevLaneInfo>::iterator lane_begin_iter, lane_end_iter;
  for (size_t road_index = 0; road_index < roads_span.size(); road_index++) {
    lane_begin_iter = bev_map.lane_infos.begin() + roads_span[road_index].first;
    lane_end_iter   = bev_map.lane_infos.begin() + roads_span[road_index].second;
    std::vector<BevMapSectionInfo> sections;
    uint64_t cur_road_id = lane_begin_iter->road_id;
    uint64_t total_section_id = cur_road_id * 100;

    // Search for multi cutting points
    std::vector<std::vector<LaneGuidance::CutPoint>> multi_layer_cut_points = FindMultiLayerCutPoints(
        bev_map, lane_begin_iter, lane_end_iter);
    sections.resize(multi_layer_cut_points.size() + 1);
    for (size_t i = 0; i < sections.size(); i++) {
      sections[i].id = ++total_section_id;
    }

    Graph<uint64_t> lane_graph, lanemarker_graph;
    std::vector<std::set<int>> lanemarker_ids_in_routes;
    std::set<int>              lanemarker_ids_in_section;
    for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
      if (it->line_points.size() < 2) {
        continue;
      }

      // 只处理在当前road组没有前继的车道中心线
      bool exist_previous_lane = false;
      for(const auto &pre_id : it->previous_lane_ids) {
        auto pre_lane = std::find_if(lane_begin_iter, lane_end_iter,
                                    [pre_id](BevLaneInfo &lane) { return lane.id == pre_id; });
        if (pre_lane != lane_end_iter) {
          exist_previous_lane = true;
          break;
        }
      }
      if(exist_previous_lane){
        continue;
      }

      if (!multi_layer_cut_points.empty()) {
        Eigen::Vector3d point_1st(it->line_points.front().x, it->line_points.front().y, 0);
        point_1st = T_local_ego_.inverse() * point_1st;
        Eigen::Vector3d point_2nd(it->line_points.back().x, it->line_points.back().y, 0);
        point_2nd             = T_local_ego_.inverse() * point_2nd;
        bool is_insert_vector = false;
        // 如果这条车道中心线有一部分在第n层的切割点之前，那么放到第n个section中
        for (size_t j = 0; j < multi_layer_cut_points.size(); j++) {
          if (point_1st.x() <= multi_layer_cut_points.at(j).back().ego_point.x() ||
              point_2nd.x() <= multi_layer_cut_points.at(j).back().ego_point.x()) {
            is_insert_vector = true;
            lane_graph.AddNode(it->id);
            lanemarker_ids_in_section.insert(it->left_lane_marker_id);
            lanemarker_ids_in_section.insert(it->right_lane_marker_id);
            sections.at(j).lane_infos.emplace_back(*it);
            break;
          }
        }
        // 如果这条车道中心线在所有切割点之后，那么放到最后一个section
        if (!is_insert_vector) {
          lane_graph.AddNode(it->id);
          lanemarker_ids_in_section.insert(it->left_lane_marker_id);
          lanemarker_ids_in_section.insert(it->right_lane_marker_id);
          sections.back().lane_infos.emplace_back(*it);
        }
        continue;
      }

      // 如果multi_layer_cut_points为空，那么所有车道中心线放在第一个section
      lane_graph.AddNode(it->id);
      lanemarker_ids_in_section.insert(it->left_lane_marker_id);
      lanemarker_ids_in_section.insert(it->right_lane_marker_id);
      sections.at(0).lane_infos.emplace_back(*it);
    }

    lanemarker_ids_in_routes.push_back(lanemarker_ids_in_section);
    AddLaneMarkersToSection(bev_map, sections, lanemarker_ids_in_routes, lanemarker_graph);

    // cut lane marker and lane
    for (size_t i = 0; i < sections.size() - 1; i++) {
      if (sections.size() < 2) {
        break;
      }
      LaneGuidance::CutPoint    lane_cut_point   = multi_layer_cut_points[i].back();
      std::pair<double, double> line_cofficients = {0, 0};

      CalculateCutLine(sections.at(i), lane_cut_point, line_cofficients);
      sections.at(i + 1).front_left = sections.at(i).back_left;
      sections.at(i + 1).front_right = sections.at(i).back_right;

      CutLaneMarkers(line_cofficients, sections.at(i).lanemarkers, sections.at(i), sections.at(i + 1), lanemarker_graph);

      CutLanes(line_cofficients, multi_layer_cut_points.at(i), bev_map, cur_road_id, sections.at(i), sections.at(i + 1),
              lane_graph, lanemarker_graph);
    }

    // 由于对导流区添加cut_point，同时对导流区进行了跟踪，所以可能存在空section，需要过滤一下
    std::vector<BevMapSectionInfo> new_sections;
    total_section_id = cur_road_id * 100;
    for(auto &section : sections) {
      if(!section.lane_infos.empty()) {
        section.id = ++total_section_id;
        new_sections.emplace_back(section);
      }
    }
    sections = new_sections;

#if 1   //是否根据导流区划分主辅路
    if (road_index == 0) {
      if(road_divider_.FindDiversionRouteAndSubpath(bev_map, sections)) {
        AssignSectionIDToLane(bev_map.route.subpaths[0].sections);
        for (auto &section : bev_map.route.subpaths[0].sections) {
          CalculateLengthOfLanes(section.lane_infos);
        }
        for (auto &section : bev_map.route.subpaths[0].sections) {
          for (auto &lane : section.lane_infos) {
            if (lane.is_virtual) {
              lane.lane_type = BevLaneType::LANE_TYPE_VIRTUAL_JUNCTION;
            }
          }
        }
        CalculateLengthOfSection(bev_map.route.subpaths[0].sections);
        for (auto &section : bev_map.route.subpaths[0].sections) {
          ResetLanePosition(section, bev_map.edges);
          CaculateLanePosition(section, bev_map.edges, lane_graph);
        }
      }
    }
#endif

    AssignSectionIDToLane(sections);

    for (auto &section : sections) {
      CalculateLengthOfLanes(section.lane_infos);
    }
    CalculateLengthOfLanes(lane_begin_iter, lane_end_iter);

    // 将所有is_virtual属性的lane_type转化为LANE_TYPE_VIRTUAL_JUNCTION
    for (auto &section : sections) {
      for (auto &lane : section.lane_infos) {
        if (lane.is_virtual) {
          lane.lane_type = BevLaneType::LANE_TYPE_VIRTUAL_JUNCTION;
        }
      }
    }
    for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
      if (it->is_virtual) {
        it->lane_type = BevLaneType::LANE_TYPE_VIRTUAL_JUNCTION;
      }
    }

    if (road_index == 0) {
      CalculateNaviStart(bev_map, sections);
    }

    CalculateLengthOfSection(sections);

    for (size_t i = 0; i < sections.size(); i++) {
      ResetLanePosition(sections.at(i), bev_map.edges);
      CaculateLanePosition(sections.at(i), bev_map.edges, lane_graph);
    }

    if (road_index == 0) {
      bev_map.route.sections = sections;
      // CheckSectionMergeTopo(bev_map);
      LaneGuidanceByRouteLanes(bev_map, lane_graph, guide_lane_result, total_section_id);
    } else {
      BevMapSubPathInfo subpath;
      subpath.sections = sections;
      if(!sections.empty()) {
        subpath.enter_section_id = sections.front().id;
      }
      bev_map.route.subpaths.emplace_back(subpath);
    }
  }

  LaneAssociationWithJunction(bev_map);

  CutLaneMarkerWithTypeSeg(bev_map);

  CutSectionWithTypeSeg(bev_map);

  AssignLaneMarkerColor(bev_map);

  AssignSectionLaneMarkerColor(bev_map);

  BoundLaneMarkerWithLane(bev_map);

  BoundSectionLaneMarkerWithLane(bev_map);

  RemoveRepeatPoint(bev_map);

  RemoveLanemarkerRepeatPoint(bev_map);

  RemoveSectionRepeatPoint(bev_map.route.sections);

  RemoveSectionLanemarkerRepeatPoint(bev_map.route.sections);

  RemoveAbnormalLane(bev_map.route.sections);

  for (auto &subpath : bev_map.route.subpaths) {
    RemoveSectionRepeatPoint(subpath.sections);

    RemoveSectionLanemarkerRepeatPoint(subpath.sections);

    RemoveAbnormalLane(subpath.sections);

    ReverseBackwardLane(subpath.sections);
  }

  GetMarkerPositionEgoFrame(bev_map);

  last_map_info_ptr_ = std::make_shared<BevMapInfo>(std::move(bev_map));

  // AINFO << "*******JunctionInfo******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // for (auto &lane : bev_map.lane_infos){
  //   AINFO << "lane ID: " << lane.id;
  //   AINFO << "lane turn_type: " << static_cast<int>(lane.bev_turn_type);
  // }
  // AINFO << "*************************";

  return;
}

void LaneGuidance::CheckSectionMergeTopo(BevMapInfo &bev_map) {
  if (bev_map.route.sections.size() < 2) {
    return;
  }

  for (size_t i = 1; i < bev_map.route.sections.size(); i++) {
    for (auto &lane : bev_map.route.sections[i].lane_infos) {
      if (lane.previous_lane_ids.size() != 2) {
        continue;
      }
      auto prev_id_1 = lane.previous_lane_ids[0];
      auto prev_id_2 = lane.previous_lane_ids[1];

      auto temp_lane_1 = std::find_if(bev_map.route.sections[i - 1].lane_infos.begin(), bev_map.route.sections[i - 1].lane_infos.end(),
                                      [prev_id_1](const cem::message::sensor::BevLaneInfo &lane) { return prev_id_1 == lane.id; });
      auto temp_lane_2 = std::find_if(bev_map.route.sections[i - 1].lane_infos.begin(), bev_map.route.sections[i - 1].lane_infos.end(),
                                      [prev_id_2](const cem::message::sensor::BevLaneInfo &lane) { return prev_id_2 == lane.id; });
      if (temp_lane_1 == bev_map.route.sections[i - 1].lane_infos.end() ||
          temp_lane_2 == bev_map.route.sections[i - 1].lane_infos.end()) {
        continue;
      }
      if ((temp_lane_1->line_points.size() < 2) ||
          (temp_lane_2->line_points.size() < 2)) {
        continue;
      }
      uint64_t mainlane_left_id = 0;
      std::set<uint64_t> temp_lane_ids = {lane.id, temp_lane_1->id, temp_lane_2->id};

      if (lane.left_lane_id > 0 && !temp_lane_ids.count(lane.left_lane_id)) {
        mainlane_left_id = lane.left_lane_id;
      } else if (temp_lane_1->left_lane_id > 0 && !temp_lane_ids.count(temp_lane_1->left_lane_id)) {
        mainlane_left_id = temp_lane_1->left_lane_id;
      } else if (temp_lane_2->left_lane_id > 0 && !temp_lane_ids.count(temp_lane_2->left_lane_id)) {
        mainlane_left_id = temp_lane_2->left_lane_id;
      }

      auto mainlane_left = std::find_if(bev_map.route.sections[i].lane_infos.begin(), bev_map.route.sections[i].lane_infos.end(),
                           [mainlane_left_id](const cem::message::sensor::BevLaneInfo &lane) { return mainlane_left_id == lane.id; });
      if (mainlane_left != bev_map.route.sections[i].lane_infos.end()) {
        // AINFO << "main lane left ID: " << mainlane_left->id;
        std::pair<float, float> main_left_near_lane_dist = {0.0, 0.0};
        std::vector<Eigen::Vector2f> mainlane_pts = *(lane.geos);
        std::vector<Eigen::Vector2f> mainlane_left_pts = *(mainlane_left->geos);
        if (!mainlane_pts.empty() && !mainlane_left_pts.empty()) {
          bool main_dist_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(mainlane_pts, mainlane_left_pts, main_left_near_lane_dist);
          // AINFO << "mean distance between lane " << lane.id << " and lane " << mainlane_left->id << ": " << main_left_near_lane_dist.first;
          if (main_left_near_lane_dist.first > 2 && main_left_near_lane_dist.first < 5.1) {
            std::pair<float, float> sub1_left_near_lane_dist = {0.0, 0.0};
            std::vector<Eigen::Vector2f> sub1lane_pts = *(temp_lane_1->geos);
            bool sub1_dist_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub1lane_pts, mainlane_left_pts, sub1_left_near_lane_dist);

            std::pair<float, float> sub2_left_near_lane_dist = {0.0, 0.0};
            std::vector<Eigen::Vector2f> sub2lane_pts = *(temp_lane_2->geos);
            bool sub2_dist_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(sub2lane_pts, mainlane_left_pts, sub2_left_near_lane_dist);

            if (sub1_left_near_lane_dist.first < sub2_left_near_lane_dist.first) {
              if (sub1_left_near_lane_dist.first > 2 && sub1_left_near_lane_dist.first < 5.0) {
                temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
              }
            } else {
              if (sub2_left_near_lane_dist.first > 2 && sub2_left_near_lane_dist.first < 5.0) {
                temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
                temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              }
            }
          }
        }
      }
    }
  }
}

void LaneGuidance::SpecialLaneAssignment(BevMapInfo &bev_map) {
  for (auto &lane : bev_map.lane_infos) {
    if (lane.lane_type != BevLaneType::LANE_TYPE_EMERGENCY && lane.lane_type != BevLaneType::LANE_TYPE_HARBOR_STOP) {
      continue;
    }
    for (auto pre_id : lane.previous_lane_ids) {
      auto pre_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                  [pre_id](BevLaneInfo &lane) { return lane.id == pre_id; });
      if (pre_lane != bev_map.lane_infos.end() && pre_lane->is_topological_connection) {
        pre_lane->lane_type = lane.lane_type;
        break;
      }
    }
    for (auto next_id : lane.next_lane_ids) {
      auto next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                   [next_id](BevLaneInfo &lane) { return lane.id == next_id; });
      if (next_lane != bev_map.lane_infos.end() && next_lane->is_topological_connection) {
        next_lane->lane_type = lane.lane_type;
        break;
      }
    }
  }
}

void LaneGuidance::RemoveSpecialLaneTopo(BevMapInfo &bev_map, const RoutingMapPtr routing_map_ptr,
    std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result) {
  if (routing_map_ptr && routing_map_ptr->is_on_highway) {
    std::set<uint64_t> highway_split_ids;
    std::set<uint64_t> recommend_lane_ids;
    for (const auto &item : guide_lane_result) {
      recommend_lane_ids.insert(item.first);
    }

    for (auto &lane : bev_map.lane_infos) {
      if (!lane.next_lane_ids.empty() || lane.previous_lane_ids.size() != 1) {
        continue;
      }
      if (recommend_lane_ids.find(lane.id) != recommend_lane_ids.end()) {
        continue;
      }

      auto pre_id = lane.previous_lane_ids.front();
      auto pre_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                  [pre_id](BevLaneInfo &lane) { return lane.id == pre_id; });
      if (pre_lane == bev_map.lane_infos.end()) {
        continue;
      }
      if (pre_lane->next_lane_ids.size() != 1 && pre_lane->next_lane_ids.front() != lane.id) {
        continue;
      }

      if (pre_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT ||
          pre_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
        highway_split_ids.insert(lane.id);
        lane.is_topo_disassembled = true;
        lane.previous_lane_ids.clear();
        lane.next_lane_ids.clear();
        highway_split_ids.insert(pre_lane->id);
        pre_lane->is_topo_disassembled = true;
        pre_lane->previous_lane_ids.clear();
        pre_lane->next_lane_ids.clear();
      }
    }
    for (auto &lane : bev_map.lane_infos) {
      for (auto it = lane.previous_lane_ids.begin(); it != lane.previous_lane_ids.end();) {
        if (highway_split_ids.find(*it) != highway_split_ids.end()) {
          it = lane.previous_lane_ids.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = lane.next_lane_ids.begin(); it != lane.next_lane_ids.end();) {
        if (highway_split_ids.find(*it) != highway_split_ids.end()) {
          it = lane.next_lane_ids.erase(it);
        } else {
          ++it;
        }
      }
    }
  }

  std::set<uint64_t> special_lane_ids;
  for (auto &lane : bev_map.lane_infos) {
    if (lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY || lane.lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
      if (!lane.previous_lane_ids.empty() || !lane.next_lane_ids.empty()) {
        lane.is_topo_disassembled = true;
      }
      lane.previous_lane_ids.clear();
      lane.next_lane_ids.clear();
      special_lane_ids.insert(lane.id);
    }
  }

  for (auto &lane : bev_map.lane_infos) {
    for (auto it = lane.previous_lane_ids.begin(); it != lane.previous_lane_ids.end();) {
      if (special_lane_ids.find(*it) != special_lane_ids.end()) {
        it = lane.previous_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
    for (auto it = lane.next_lane_ids.begin(); it != lane.next_lane_ids.end();) {
      if (special_lane_ids.find(*it) != special_lane_ids.end()) {
        it = lane.next_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void LaneGuidance::TraceFarRecommendLane(BevMapInfo &bev_map, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result) {
  if (guide_lane_result.empty()) {
    return;
  }

  for (auto& item : guide_lane_result) {
    auto lane_iter = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                 [&item](BevLaneInfo &lane) { return lane.id == item.first; });
    if (lane_iter == bev_map.lane_infos.end()) {
      continue;
    }
    int cycle_count = 0;
    uint64_t end_lane_id = std::numeric_limits<uint64_t>::max();
    while (lane_iter->next_lane_ids.size() == 1 && cycle_count < kMaxLoopNumber) {
      cycle_count++;
      auto lane_id = lane_iter->next_lane_ids.front();
      lane_iter = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                 [&lane_id](BevLaneInfo &lane) { return lane.id == lane_id; });
      if (lane_iter == bev_map.lane_infos.end()) {
        break;
      }
      end_lane_id = lane_id;
    }

    if (end_lane_id < std::numeric_limits<uint64_t>::max()) {
      item.first = end_lane_id;
    }
  }
}

inline bool FindCurveLaneIntersection(const std::vector<cem::message::common::Point2DF>& lane_points,
                                      const std::optional<std::pair<uint64_t, uint64_t>>& ind_pair,
                                      const cem::message::common::Point2DF& pt1,
                                      const cem::message::common::Point2DF& pt2,
                                      cem::message::common::Point2DF& intersection_point) {
  if (!ind_pair) {
    return false;
  }

  bool intersection_flag = false;
  auto start_ind = ind_pair->first;
  auto end_ind  = ind_pair->second;
  cem::message::common::Point2DF intersection_line;

  for (auto i = start_ind; i < end_ind; ++i) {
        auto pt_lst = lane_points.at(start_ind);
        auto pt_led = lane_points.at(end_ind);
        if (!IsSegmentLineIntersect(pt_lst, pt_led, pt1, pt2, intersection_line)) {
            break;
        }
        auto mid_ind = (start_ind + end_ind) / 2;
        if (mid_ind == start_ind || mid_ind == end_ind) {
            break;
        }
        auto pt_lmid = lane_points.at(mid_ind);
        bool is_1st_intersection = IsSegmentsIntersect(pt1, pt2, pt_lst, pt_lmid, intersection_point);
        bool is_2nd_intersection = IsSegmentsIntersect(pt1, pt2, pt_lmid, pt_led, intersection_point);
        if (is_1st_intersection) {
            intersection_flag = true;
            break;
        }

        if (is_2nd_intersection) {
            intersection_flag = true;
            break;
        }

        if (IsSegmentLineIntersect(pt_lst, pt_lmid, pt1, pt2, intersection_line)) {
            end_ind = mid_ind;
        } else if (IsSegmentLineIntersect(pt_lmid, pt_led, pt1, pt2, intersection_line)) {
            start_ind = mid_ind;
        } else {
            break;
        }
    }

  return intersection_flag;
}

void LaneGuidance::CutLaneInfoOutLaneMarker(BevMapInfo &bev_map) {
  for (auto &lane : bev_map.lane_infos) {
    auto                                        lanePoints = lane.line_points;
    std::vector<cem::message::common::Point2DF> lanePtLocVec;
    for (const auto &pt : lanePoints) {
      Eigen::Vector3d                lanePt3d{pt.x, pt.y, 0.0};
      Eigen::Vector3d                lanePt3dLoc = T_ego_local_ * lanePt3d;
      cem::message::common::Point2DF point2d     = pt;
      point2d.x                                  = lanePt3dLoc.x();
      point2d.y                                  = lanePt3dLoc.y();
      lanePtLocVec.emplace_back(point2d);
    }
    if (lanePtLocVec.empty()) {
      continue;
    }
    std::vector<Eigen::Vector3d> leftLocPts;
    std::vector<Eigen::Vector3d> rightLocPts;
    for (const auto &lanemarker : bev_map.lanemarkers) {
      auto leftIdx = std::find(lane.left_lane_boundary_ids.begin(), lane.left_lane_boundary_ids.end(), lanemarker.id);
      if (leftIdx != lane.left_lane_boundary_ids.end()) {
        for (const auto &pt : lanemarker.line_points) {
          Eigen::Vector3d lanemarkerPt3d{pt.x, pt.y, 0.0};
          Eigen::Vector3d lanemarkerPtLoc = T_ego_local_ * lanemarkerPt3d;
          leftLocPts.emplace_back(lanemarkerPtLoc);
        }
      }
      auto rightIdx = std::find(lane.right_lane_boundary_ids.begin(), lane.right_lane_boundary_ids.end(), lanemarker.id);
      if (rightIdx != lane.right_lane_boundary_ids.end()) {
        for (const auto &pt : lanemarker.line_points) {
          Eigen::Vector3d lanemarkerPt3d{pt.x, pt.y, 0.0};
          Eigen::Vector3d lanemarkerPt3dLoc = T_ego_local_ * lanemarkerPt3d;
          rightLocPts.emplace_back(lanemarkerPt3dLoc);
        }
      }
    }
    for (int idx = leftLocPts.size() - 1; idx > 0; idx--) {
      auto pt = leftLocPts[idx];
      while ((fabs(lanePtLocVec.back().x - pt.x()) < 1.0) && (lanePtLocVec.back().y > pt.y()) && (!lanePtLocVec.empty()) &&
             static_cast<int>(lanePtLocVec.back().point_source) > 2) {
        lanePtLocVec.erase(lanePtLocVec.end() - 1);
      }
    }
    for (int idx = rightLocPts.size() - 1; idx > 0; idx--) {
      auto pt = rightLocPts[idx];
      while ((fabs(lanePtLocVec.back().x - pt.x()) < 1.0) && (lanePtLocVec.back().y < pt.y()) && (!lanePtLocVec.empty()) &&
             static_cast<int>(lanePtLocVec.back().point_source) > 2) {
        lanePtLocVec.erase(lanePtLocVec.end() - 1);
      }
    }
    std::vector<cem::message::common::Point2DF> outputLanePts;
    for (int idx = 0; idx < lanePtLocVec.size(); idx++) {
      outputLanePts.emplace_back(lanePoints[idx]);
    }
    lane.line_points.clear();
    lane.line_points = outputLanePts;
  }
}

double ComputeCurvature(Eigen::VectorXd &coeff, Eigen::Vector3d &point) {
  double a           = coeff[0];
  double b           = coeff[1];
  double x           = point.x();
  double numerator   = std::abs(2 * a);
  double denominator = pow(1 + pow(2 * a * x + b, 2), 1.5);
  double currKappa;
  currKappa = numerator / denominator;
  return currKappa;
}

void AdjustCurvature(Eigen::VectorXd &coeff, Eigen::Vector3d &point, double &preKappa) {
  const double learningRate  = 0.01;
  const int    maxIterations = 100;
  double       targetKappa   = preKappa * 1.15;
  for (int i = 0; i < maxIterations; i++) {
    double currKappa = ComputeCurvature(coeff, point);
    double error     = currKappa - targetKappa;
    coeff.array() -= learningRate * error;
    if (std::fabs(error) < 0.01) {
      break;
    }
  }
}

void LaneGuidance::AssignSectionIDToLane(std::vector<BevMapSectionInfo>& sections) {
  for (auto &section : sections) {
    for (auto &lane : section.lane_infos) {
      lane.section_id = section.id;
    }
  }
}

void LaneGuidance::RemoveSectionRepeatPoint(std::vector<BevMapSectionInfo>& sections) {
  for (auto &section : sections) {
    for (auto &line : section.lane_infos) {
      auto points = line.line_points;
      if (points.empty()) {
        continue;
      }
      std::set<int> match_idx;
      for (int pre_idx = 0; pre_idx < points.size() - 1; pre_idx++) {
        const Eigen::Vector3d pre_pt{points[pre_idx].x, points[pre_idx].y, 0.0};
        for (int next_idx = pre_idx + 1; next_idx < points.size(); next_idx++) {
          const Eigen::Vector3d next_pt{points[next_idx].x, points[next_idx].y, 0.0};
          const double          dist = (pre_pt - next_pt).norm();
          if (dist < kDistanceThreshold) {
            match_idx.insert(next_idx);
          }
        }
      }
      std::vector<cem::message::common::Point2DF> filted_points;
      for (int idx = 0; idx < points.size(); idx++) {
        if (match_idx.find(idx) == match_idx.end()) {
          filted_points.emplace_back(points[idx]);
        }
        line.line_points.clear();
        line.line_points = filted_points;
      }
    }
  }
}

template <typename T>
bool LaneGuidance::HaveIntersection(const std::vector<T> &vec1, const std::vector<T> &vec2) {
  std::set<T> set1(vec1.begin(), vec1.end());
  std::set<T> set2(vec2.begin(), vec2.end());

  std::vector<T> intersection;
  std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), std::back_inserter(intersection));

  return !intersection.empty();
}

void LaneGuidance::RemoveNonExistentSuccessor(BevMapInfo &bev_map) {
  for (size_t i = 0; i < bev_map.lane_infos.size(); i++) {
    if (bev_map.lane_infos[i].next_lane_ids.empty()) {
      continue;
    }
    for (std::vector<uint64_t>::iterator it = bev_map.lane_infos[i].next_lane_ids.begin();
         it != bev_map.lane_infos[i].next_lane_ids.end();) {
      auto next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                    [find_id = *it](BevLaneInfo &lane) { return lane.id == find_id; });

      if (next_lane == bev_map.lane_infos.end()) {
        it = bev_map.lane_infos[i].next_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void LaneGuidance::RemoveNonExistentPredecessor(BevMapInfo &bev_map) {
  for (size_t i = 0; i < bev_map.lane_infos.size(); i++) {
    if (bev_map.lane_infos[i].previous_lane_ids.empty()) {
      continue;
    }
    for (std::vector<uint64_t>::iterator it = bev_map.lane_infos[i].previous_lane_ids.begin();
         it != bev_map.lane_infos[i].previous_lane_ids.end();) {
      auto next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                    [find_id = *it](BevLaneInfo &lane) { return lane.id == find_id; });

      if (next_lane == bev_map.lane_infos.end()) {
        it = bev_map.lane_infos[i].previous_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
  }
}

std::vector<std::vector<LaneGuidance::CutPoint>> LaneGuidance::FindMultiLayerCutPoints(const BevMapInfo &bev_map,
    const std::vector<BevLaneInfo>::iterator &lane_begin_iter, const std::vector<BevLaneInfo>::iterator &lane_end_iter) {
  std::vector<std::vector<CutPoint>> multi_layers_cut_points;
  std::vector<CutPoint>              cut_points;
  Eigen::Vector2d                    dir_stop;

  // 搜索ego_lane，作为起始的node，添加进入queue中
  // AINFO << "*******FindCutpoint******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  std::queue<std::pair<std::vector<BevLaneInfo>::iterator, std::vector<std::vector<BevLaneInfo>::iterator>>> ego_lane_q;
  std::set<uint64_t> visited_id_set;
  std::vector<std::vector<std::vector<BevLaneInfo>::iterator>> target_ego_lane_path_container;
  Eigen::Vector2d dir_lane_temp;
  std::vector<Eigen::Vector2d> target_ego_lane_dir_container;
  std::vector<std::unique_ptr<BevLaneInfo>> target_ego_lane_container;
  int N_max = 100;
  for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
    if (it->line_points.size() < 2) {
      continue;
    }

    if (it->position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
      ego_lane_q.push({it, {it}});
    }
  }

  if (!ego_lane_q.empty()){
    std::unique_ptr<BevLaneInfo> target_lane = std::make_unique<BevLaneInfo>(*ego_lane_q.front().first);
    Eigen::Vector3d target_line_start_ego;
    Eigen::Vector3d target_line_start;
    target_line_start.x() = target_lane->line_points.front().x;
    target_line_start.y() = target_lane->line_points.front().y;
    target_line_start_ego = T_ego_local_ * target_line_start;

    if (target_line_start_ego.x() >= 0){
      dir_lane_temp.x()           = target_lane->line_points[1].x - target_lane->line_points[0].x;
      dir_lane_temp.y()           = target_lane->line_points[1].y - target_lane->line_points[0].y;
      dir_lane_temp               = dir_lane_temp / dir_lane_temp.norm();

      target_ego_lane_dir_container.push_back(dir_lane_temp);
      target_ego_lane_container.push_back(std::move(target_lane));
    } else {
      // 使用BFS的算法，搜索ego_lane的后继，输出所有到无后继节点的路径
      for (int i=0; i<N_max; ++i){
        if(ego_lane_q.empty()){
          break;
        }

        auto target_lane = ego_lane_q.front().first;
        auto target_lane_path = ego_lane_q.front().second;
        ego_lane_q.pop();

        if (visited_id_set.count(target_lane->id)){
          continue;
        }

        if (target_lane == lane_end_iter){
          continue;
        }

        visited_id_set.insert(target_lane->id);

        if (target_lane->line_points.size() < 2) {
          continue;
        }

        if (target_lane->next_lane_ids.empty()) {
          target_ego_lane_path_container.push_back(target_lane_path);
        }

        for (const auto &next_id : target_lane->next_lane_ids) {
          if (visited_id_set.count(next_id)) {
            continue;
          }

          auto next_lane = std::find_if(lane_begin_iter, lane_end_iter, [id = next_id](BevLaneInfo &lane) { return lane.id == id; });
          if (next_lane == lane_end_iter){
            continue;
          }

          // 创建新路径
          auto next_lane_path = target_lane_path;
          next_lane_path.push_back(next_lane);

          ego_lane_q.push({next_lane, next_lane_path});
        }
      }

      // 根据搜索到的路径拼接形成ego_lane，只记录其形点用于stopline，crosswalk的筛选
      for (const auto &target_lane_path : target_ego_lane_path_container){
        // 找到目标的初始ego_lane的iter，拼接其与后继的行点形成最终使用的ego_lane
        int target_start_ind = -1;
        for (int i=0; i<target_lane_path.size(); i++){
          Eigen::Vector3d line_start_ego, line_end_ego;
          Eigen::Vector3d line_start, line_end;
          auto target_lane = target_lane_path[i];
          line_start.x() = target_lane->line_points.front().x;
          line_start.y() = target_lane->line_points.front().y;
          line_start_ego = T_ego_local_ * line_start;

          line_end.x() = target_lane->line_points.back().x;
          line_end.y() = target_lane->line_points.back().y;
          line_end_ego = T_ego_local_ * line_end;

          // AINFO << "current lane ID: " << target_lane->id << " with ego start x: " << line_start_ego.x();

          if (line_start_ego.x() < 0 && line_end_ego.x() > 0) {
            target_start_ind = i;
            break; 
          }
        }

        // 将对应ego_lane形点与其后继其他形点拼接形成问题中使用判定stop line crosswalk是否形成可靠断点使用的ego_lane
        std::unique_ptr<BevLaneInfo> target_lane = std::make_unique<BevLaneInfo>();
        if (target_start_ind >= 0) {
          for (int i=target_start_ind; i<target_lane_path.size(); i++){
            auto temp_lane = target_lane_path[i];
            target_lane->line_points.insert(target_lane->line_points.end(), temp_lane->line_points.begin(),  temp_lane->line_points.end());
          }

          size_t ind = target_lane->line_points.size();
          for (size_t i2 = 1; i2 < target_lane->line_points.size(); ++i2) {
            Eigen::Vector3d line_point_ego;
            Eigen::Vector3d line_point;
            line_point.x() = target_lane->line_points[i2].x;
            line_point.y() = target_lane->line_points[i2].y;
            line_point_ego = T_ego_local_ * line_point;
            if (line_point_ego.x() > 0) {
              ind = i2;
              break;
            }
          }

          if (ind < target_lane->line_points.size()) {
            dir_lane_temp.x()           = target_lane->line_points[ind].x - target_lane->line_points[ind - 1].x;
            dir_lane_temp.y()           = target_lane->line_points[ind].y - target_lane->line_points[ind - 1].y;
            dir_lane_temp               = dir_lane_temp / dir_lane_temp.norm();

            target_ego_lane_dir_container.push_back(dir_lane_temp);
            target_ego_lane_container.push_back(std::move(target_lane));
          }
        }
      }
    }
  }

  // for (int i=0; i<target_ego_lane_container.size(); ++i) {
  //   auto& target_lane = target_ego_lane_container[i];
  //   auto target_dir_lane = target_ego_lane_dir_container[i];
  //   AINFO << "target lane start: (" << target_lane->line_points.front().x << ", " << target_lane->line_points.front().y << ")";
  //   AINFO << "target lane end: (" << target_lane->line_points.back().x << ", " << target_lane->line_points.back().y << ")";
  //   AINFO << "target lane direction: " << target_dir_lane;
  // }

  // for (const auto &visited_id : visited_id_set){
  //   AINFO << "visited ID: " << visited_id;
  // }

  for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
    if (it->next_lane_ids.empty() || it->line_points.size() < 2) {
      continue;
    }

    // 如果切割点与车道中心线有相同的后继，那么将车道中心线的id加入切割点的merge_lane_ids
    bool is_the_same_point = false;
    for (auto &point : cut_points) {
      is_the_same_point = HaveIntersection(point.next_lane_ids, it->next_lane_ids);
      if (is_the_same_point) {
        point.merge_lane_ids.emplace_back(it->id);
        break;
      }
    }

    if (is_the_same_point) {
      continue;
    }

    // 否则，采用车道中心线最后一个点，创建一个新的切割点
    CutPoint cut_point;
    cut_point.lane_id           = it->id;
    cut_point.previous_lane_ids = it->previous_lane_ids;
    cut_point.next_lane_ids     = it->next_lane_ids;
    cut_point.dr_point.x()      = it->line_points.back().x;
    cut_point.dr_point.y()      = it->line_points.back().y;
    cut_point.ego_point         = T_local_ego_.inverse() * cut_point.dr_point;
    cut_point.merge_lane_ids.emplace_back(it->id);
    cut_points.emplace_back(cut_point);
  }

  has_virtual_junction_lane_ = false;
  for (const auto &lane : bev_map.lane_infos) {
    if (lane.is_virtual) {
      has_virtual_junction_lane_ = true;
    }
  }

  // 计算车辆前方的车道中心线与停止线的交点
  std::vector<CutPoint> stopline_cut_point;
  for (const auto &stop_line : stop_lines_) {
    // if (has_virtual_junction_lane_) {
    //   break;
    // }
    if (stop_line.dr_line_points.size() < 2) {
      continue;
    }
    CutPoint cut_point;
    bool     is_valid_stop_line = false;
    // for (auto &lane : bev_map.lane_infos) {
    for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
      if (it->line_points.size() < 2) {
        continue;
      }
      auto id_pair = ValidLaneSegment(*it);
      if (id_pair == std::nullopt) {
        continue;
      }
      cem::message::common::Point2DF intersection;
      if (!IsSegmentsIntersect(stop_line.dr_line_points.front(), stop_line.dr_line_points.back(), it->line_points.at(id_pair->first),
                               it->line_points.at(id_pair->second), intersection)) {
        cem::message::common::Point2DF intersection_curve;
        bool intersection_flag = FindCurveLaneIntersection(it->line_points, id_pair, stop_line.dr_line_points.front(), stop_line.dr_line_points.back(), intersection_curve);
        if (!intersection_flag) {
          continue;
        }
        intersection = intersection_curve;
      }
      is_valid_stop_line     = true;
      cut_point.lane_id      = it->id;
      cut_point.is_stop_line = true;
      cut_point.dr_point.x() = intersection.x;
      cut_point.dr_point.y() = intersection.y;
      cut_point.ego_point    = T_ego_local_ * cut_point.dr_point;
      it->traffic_stop_lines.emplace_back(static_cast<uint64_t>(stop_line.id));
    }

    // 新增分支：假设找到成功定位到了ego_lane_direction，则利用其的值来判断是否emplace_back cut_point
    for (int i=0; i<target_ego_lane_container.size(); ++i) {
      // 判断ego_lane 线段是否与stopline相交，如果不相交就进行向量夹角判断，向量夹角的cos_threshold可以设置的很小
      bool is_ego_lane_intersect = false;
      // auto ego_lane              = bev_map.lane_infos[ego_lane_ind];
      auto& target_ego_lane_iter = target_ego_lane_container[i];
      auto target_ego_lane_dir = target_ego_lane_dir_container[i];
      auto id_pair               = ValidLaneSegment(*target_ego_lane_iter);

      if (id_pair != std::nullopt) {
        cem::message::common::Point2DF intersection;
        if (IsSegmentsIntersect(stop_line.dr_line_points.front(), stop_line.dr_line_points.back(), target_ego_lane_iter->line_points.at(id_pair->first), target_ego_lane_iter->line_points.at(id_pair->second), intersection)) {
          is_ego_lane_intersect = true;
        }
        else if (FindCurveLaneIntersection(target_ego_lane_iter->line_points, id_pair, stop_line.dr_line_points.front(), stop_line.dr_line_points.back(), intersection)){
          is_ego_lane_intersect = true;
        }
      }

      if (!is_ego_lane_intersect) {
        // 计算stopline的方向向量（尾端 - 首端）
        dir_stop.x()     = stop_line.dr_line_points.back().x - stop_line.dr_line_points.front().x;
        dir_stop.y()     = stop_line.dr_line_points.back().y - stop_line.dr_line_points.front().y;
        dir_stop         = dir_stop / dir_stop.norm();
        double cos_theta = dir_stop.dot(target_ego_lane_dir);
        if (std::abs(cos_theta) > kStoplineFilterMaxCos) {
          is_valid_stop_line = false;
          break;
        }
      }
    }

    if (is_valid_stop_line) {
      stopline_cut_point.emplace_back(cut_point);
    }
  }

  // 计算车辆前方的车道中心线与人行横道的交点
  std::vector<CutPoint> crosswalk_cut_point;
  for (const auto &crosswalk : bev_map.crosswalks) {
    // if (has_virtual_junction_lane_) {
    //   break;
    // }
    if (crosswalk.line_points.size() != 4) {
      continue;
    }

    // 根据自车车道与纵向车道位置，判定crosswalk是否valid。
    // 临时规则：如果有多条查找到的ego_lane, 当前要求对于每条lane选择到的pt1, pt2一致。
    // 搜索自车车道，找到所有在自车坐标系x>0的点集
    // if (ego_lane_ind >= bev_map.lane_infos.size()) {
    if (target_ego_lane_container.empty()) {
      // AINFO << "No Ego lane found!!! Now crosswalk ID:  " << crosswalk.id;
      continue;
    }
    // auto                                        ego_lane = bev_map.lane_infos[ego_lane_ind];
    // 如果target_ego_lane_iter_container有多条车道线结果，选取第一条确认pt1, pt2
    auto& target_ego_lane_iter = target_ego_lane_container[0];
    std::vector<cem::message::common::Point2DF> target_ego_line_points;
    for (size_t i = 1; i < target_ego_lane_iter->line_points.size(); ++i) {
      Eigen::Vector3d line_point;
      Eigen::Vector3d ego_point;
      line_point.x() = target_ego_lane_iter->line_points[i].x;
      line_point.y() = target_ego_lane_iter->line_points[i].y;
      ego_point      = T_local_ego_.inverse() * line_point;
      if (ego_point.x() > 0) {
        if (target_ego_line_points.empty()){
          target_ego_line_points.push_back(target_ego_lane_iter->line_points[i-1]);
          target_ego_line_points.push_back(target_ego_lane_iter->line_points[i]);
        } else {
          target_ego_line_points.push_back(target_ego_lane_iter->line_points[i]);
        }
      }
    }

    // 设置crosswalk的polygon对象
    std::vector<cem::message::common::Point2DF> crosswalk_polygon;
    for (const auto &point : crosswalk.line_points) {
      crosswalk_polygon.push_back(point);
    }
    crosswalk_polygon.push_back(crosswalk.line_points[0]);
    // 求解egolane切线与crosswalk的polygon是否有交点，若有则记录该交点
    std::vector<cem::message::common::Point2DF> intersect_seg_st;
    std::vector<cem::message::common::Point2DF> intersect_seg_ed;
    std::vector<cem::message::common::Point2DF> intersections;
    cem::message::common::Point2DF              line_st, line_ed;
    bool                                        is_intersect = false;
    for (size_t i = 1; i < target_ego_line_points.size(); ++i) {
      line_st = target_ego_line_points[i - 1];
      line_ed = target_ego_line_points[i];
      intersect_seg_st.clear();
      intersect_seg_ed.clear();
      intersections.clear();

      is_intersect = IsLinePolygonIntersect(line_st, line_ed, crosswalk_polygon, intersect_seg_st, intersect_seg_ed, intersections);
      if (is_intersect) {
        break;
      }
    }

    if (!is_intersect) {
      continue;
    }

    // 根据intersection point与ego_lane目标切线起点的远近，找到对应的pt1, pt2
    double min_distance  = std::numeric_limits<double>::max();
    size_t closest_index = -1;
    for (size_t i = 0; i < intersections.size(); ++i) {
      auto   current_point = intersections[i];
      double distance      = std::sqrt(std::pow(current_point.x - line_st.x, 2) + std::pow(current_point.y - line_st.y, 2));
      if (distance < min_distance) {
        min_distance  = distance;
        closest_index = i;
      }
    }

    cem::message::common::Point2DF pt1, pt2;
    pt1 = intersect_seg_st[closest_index];
    pt2 = intersect_seg_ed[closest_index];

    bool valid_crosswalk_flag = true;
    // 临时规则：剩余的其他可能选取的ego_lane必须与pt1, pt2相交
    for (int i=1; i<target_ego_lane_container.size(); ++i) {
      auto& target_ego_lane_iter = target_ego_lane_container[i];
      std::vector<cem::message::common::Point2DF> target_ego_line_points;
      for (size_t i = 1; i < target_ego_lane_iter->line_points.size(); ++i) {
        Eigen::Vector3d line_point;
        Eigen::Vector3d ego_point;
        line_point.x() = target_ego_lane_iter->line_points[i].x;
        line_point.y() = target_ego_lane_iter->line_points[i].y;
        ego_point      = T_local_ego_.inverse() * line_point;
        if (ego_point.x() > 0) {
          if (target_ego_line_points.empty()){
            target_ego_line_points.push_back(target_ego_lane_iter->line_points[i-1]);
            target_ego_line_points.push_back(target_ego_lane_iter->line_points[i]);
          } else {
            target_ego_line_points.push_back(target_ego_lane_iter->line_points[i]);
          }
        }
      }

      bool is_intersect = false;
      for (size_t i = 1; i < target_ego_line_points.size(); ++i) {
        auto line_st = target_ego_line_points[i - 1];
        auto line_ed = target_ego_line_points[i];
        cem::message::common::Point2DF intersection;
        is_intersect = IsSegmentLineIntersect(pt1, pt2, line_st, line_ed, intersection);
        if (is_intersect) {
          break;
        }
      }

      if (!is_intersect) {
        valid_crosswalk_flag = false;
        break;
      }
    }

    if (!valid_crosswalk_flag){
      continue;
    }

    // AINFO << "Crosswalk ID: " << crosswalk.id << " is valid!!!!!"; 

    CutPoint cut_point;
    bool     is_valid_crosswalk = false;
    // for (const auto &lane : bev_map.lane_infos) {
    for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
      if (it->line_points.size() < 2) {
        continue;
      }
      auto id_pair = ValidLaneSegment(*it);
      if (id_pair == std::nullopt) {
        continue;
      }
      cem::message::common::Point2DF intersection;
      if (!IsSegmentsIntersect(pt1, pt2, it->line_points.at(id_pair->first), it->line_points.at(id_pair->second), intersection)) {
        cem::message::common::Point2DF intersection_curve;
        bool intersection_flag = FindCurveLaneIntersection(it->line_points, id_pair, pt1, pt2, intersection_curve);
        if (!intersection_flag) {
          continue;
        }
        intersection = intersection_curve;
      }
      is_valid_crosswalk     = true;
      cut_point.lane_id      = it->id;
      cut_point.is_stop_line = true;
      cut_point.dr_point.x() = intersection.x;
      cut_point.dr_point.y() = intersection.y;
      cut_point.ego_point    = T_ego_local_ * cut_point.dr_point;
      it->traffic_crosswalks.emplace_back(static_cast<uint64_t>(crosswalk.id));
    }
    if (is_valid_crosswalk) {
      crosswalk_cut_point.emplace_back(cut_point);
    }
  }

  if (!stopline_cut_point.empty()) {
    cut_points.emplace_back(stopline_cut_point.front());
  } else if (!crosswalk_cut_point.empty() && stopline_cut_point.empty()) {
    cut_points.emplace_back(crosswalk_cut_point.front());
  }

  // 导流区首尾点生成cut_point
  for(const auto& diver_zone : bev_map.diversion_zone) {
    if(diver_zone.geos && diver_zone.geos->size()==4) {
      std::vector<Eigen::Vector2f> points = *diver_zone.geos;
      std::sort(points.begin(), points.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });
      std::vector<Eigen::Vector2f> diver_line = {(points[0] + points[1]) / 2.0f, (points[2] + points[3]) / 2.0f};

      for(auto &diver_point : diver_line) {
        CutPoint cut_point;
        cut_point.lane_id        = 0;
        cut_point.ego_point.x()  = diver_point.x();
        cut_point.ego_point.y()  = diver_point.y();
        cut_point.dr_point       = T_local_ego_ * cut_point.ego_point;
        cut_points.emplace_back(cut_point);
      }
    }
  }
  // AINFO << "*****************************";

  // 对所有切割点（车道中心线末端、与停止线交点、与人行横道交点、导流区首尾点）由近到远进行排序
  std::sort(cut_points.begin(), cut_points.end(), [](const CutPoint &a, const CutPoint &b) { return a.ego_point.x() < b.ego_point.x(); });

  for (size_t i = 0; i < cut_points.size(); i++) {
    // 第1个切割点，创建第1层切割点
    if (i == 0) {
      std::vector<CutPoint> one_layer_cut_points;
      one_layer_cut_points.emplace_back(cut_points[i]);
      multi_layers_cut_points.emplace_back(one_layer_cut_points);
      continue;
    }

    // 如果切割点是最后一层的后继lane的尾点，那么必然是下一级拓扑
    bool is_cascade_topology = false;
    if (!cut_points[i].is_stop_line) {
      for (auto last_layer_point : multi_layers_cut_points.back()) {
        for (auto next_lane_id : last_layer_point.next_lane_ids) {
          if (next_lane_id == cut_points[i].lane_id) {
            is_cascade_topology = true;
            break;
          }
        }
      }
    }

    double delta_x = cut_points[i].ego_point.x() - multi_layers_cut_points.back().front().ego_point.x();
    // 如果不是下一级拓扑，且新的切割点在10m范围内，那么属于同一层的切割点
    if (!is_cascade_topology && delta_x < kCutPointHierarchicalThreshold) {
      multi_layers_cut_points.back().emplace_back(cut_points[i]);
    }
    // 如果是下一级拓扑，或者新的切割点在10m之外，那么创建为新一层的切割点
    else {
      std::vector<CutPoint> one_layer_cut_points;
      one_layer_cut_points.emplace_back(cut_points[i]);
      multi_layers_cut_points.emplace_back(one_layer_cut_points);
    }
  }

  return multi_layers_cut_points;
}

std::optional<std::pair<uint64_t, uint64_t>> LaneGuidance::ValidLaneSegment(const BevLaneInfo &lane) {
  if (lane.line_points.size() < 2) {
    return std::nullopt;
  }
  Eigen::Vector3d front_point(lane.line_points.front().x, lane.line_points.front().y, 0);
  Eigen::Vector3d back_point(lane.line_points.back().x, lane.line_points.back().y, 0);
  front_point = T_ego_local_ * front_point;
  back_point  = T_ego_local_ * back_point;
  if (back_point.x() < 0 || back_point.x() < front_point.x()) {
    return std::nullopt;
  }

  size_t begin_id = 0;
  for (size_t i = 0; i < lane.line_points.size(); i++) {
    Eigen::Vector3d ego_point(lane.line_points[i].x, lane.line_points[i].y, 0);
    ego_point = T_ego_local_ * ego_point;
    if (ego_point.x() >= 0) {
      begin_id = i;
      break;
    }
  }

  if (lane.line_points.size() - 1 > begin_id) {
    return std::make_pair(begin_id, lane.line_points.size() - 1);
  }

  return std::nullopt;
}

void LaneGuidance::RemoveLanemarkerRepeatPoint(BevMapInfo &bev_map) {
  for (auto &lanemarker : bev_map.lanemarkers) {
    std::vector<cem::message::common::Point2DF> points;
    points = lanemarker.line_points;
    if (points.empty()) {
      continue;
    }
    std::set<int> matchIdx;
    for (int preIdx = 0; preIdx < (points.size() - 1); preIdx++) {
      const Eigen::Vector3d prePt{points[preIdx].x, points[preIdx].y, 0.0};
      for (int nextIdx = preIdx + 1; nextIdx < points.size(); nextIdx++) {
        const Eigen::Vector3d nextPt{points[nextIdx].x, points[nextIdx].y, 0.0};
        const double          dist = (nextPt - prePt).norm();
        if (dist < kDistanceThreshold) {
          matchIdx.insert(nextIdx);
        }
      }
    }
    std::vector<cem::message::common::Point2DF> filtedPoints;
    for (int idx = 0; idx < points.size(); idx++) {
      if (matchIdx.find(idx) == matchIdx.end()) {
        filtedPoints.emplace_back(points[idx]);
      }
    }
    lanemarker.line_points.clear();
    lanemarker.line_points = filtedPoints;
  }
}

void LaneGuidance::RemoveSectionLanemarkerRepeatPoint(std::vector<BevMapSectionInfo>& sections) {
  for (auto &section : sections) {
    for (auto lanemarker : section.lanemarkers) {
      auto points = lanemarker.line_points;
      if (points.empty()) {
        continue;
      }
      std::set<int> match_idx;
      for (int pre_idx = 0; pre_idx < points.size() - 1; pre_idx++) {
        const Eigen::Vector3d pre_pt{points[pre_idx].x, points[pre_idx].y, 0.0};
        for (int next_idx = pre_idx + 1; next_idx < points.size(); next_idx++) {
          const Eigen::Vector3d next_pt{points[next_idx].x, points[next_idx].y, 0.0};
          const double          dist = (pre_pt - next_pt).norm();
          if (dist < kDistanceThreshold) {
            match_idx.insert(next_idx);
          }
        }
      }
      std::vector<cem::message::common::Point2DF> fitled_points;
      for (int idx = 0; idx < points.size(); idx++) {
        if (match_idx.find(idx) == match_idx.end()) {
          fitled_points.emplace_back(points[idx]);
        }
        lanemarker.line_points.clear();
        lanemarker.line_points = fitled_points;
      }
    }
  }
}

bool RansacFitting(const std::vector<Eigen::Vector3d> &pos_vec, std::vector<Eigen::Vector3d> *selected_points,
                   Eigen::Matrix<double, 4, 1> *coeff, const int max_iters = 100, const int N = 5,
                   const double inlier_thres = static_cast<double>(0.1)) {
  if (coeff == nullptr) {
    return false;
  }
  selected_points->clear();
  int n  = static_cast<int>(pos_vec.size());
  int q1 = static_cast<int>(n / 4);
  int q2 = static_cast<int>(n / 2);
  int q3 = static_cast<int>(n * 3 / 4);
  if (n < N) {
    return false;
  }
  std::vector<int> index(3, 0);
  int              max_inliers      = 0;
  double           min_residual     = std::numeric_limits<double>::max();
  double           early_stop_ratio = 0.95f;
  double           good_lane_ratio  = 0.666f;
  for (int j = 0; j < max_iters; ++j) {
    index[0] = std::rand() % q2;
    index[1] = q2 + std::rand() % q1;
    index[2] = q3 + std::rand() % q1;
    Eigen::Matrix<double, 3, 3> matA;
    matA << pos_vec[index[0]](0) * pos_vec[index[0]](0), pos_vec[index[0]](0), 1, pos_vec[index[1]](0) * pos_vec[index[1]](0),
        pos_vec[index[1]](0), 1, pos_vec[index[2]](0) * pos_vec[index[2]](0), pos_vec[index[2]](0), 1;

    Eigen::FullPivLU<Eigen::Matrix<double, 3, 3>> mat(matA);
    mat.setThreshold(1e-5f);
    if (mat.rank() < 3) {
      continue;
    }
    Eigen::Matrix<double, 3, 1> matB;
    matB << pos_vec[index[0]](1), pos_vec[index[1]](1), pos_vec[index[2]](1);
    Eigen::Vector3d c = static_cast<Eigen::Matrix<double, 3, 1>>(matA.inverse() * matB);
    if (!(matA * c).isApprox(matB)) {
      continue;
    }
    int    num_inliers = 0;
    double residual    = 0;
    double y           = 0;
    for (int i = 0; i < n; ++i) {
      y = pos_vec[i](0) * pos_vec[i](0) * c(0) + pos_vec[i](0) * c(1) + c(2);
      if (std::abs(y - pos_vec[i](1)) <= inlier_thres) {
        ++num_inliers;
      }
      residual += std::abs(y - pos_vec[i](1));
    }
    if (num_inliers > max_inliers || (num_inliers == max_inliers && residual < min_residual)) {
      (*coeff)(3)  = 0;
      (*coeff)(2)  = c(0);
      (*coeff)(1)  = c(1);
      (*coeff)(0)  = c(2);
      max_inliers  = num_inliers;
      min_residual = residual;
    }
    if (max_inliers > early_stop_ratio * n) {
      break;
    }
  }
  if (static_cast<double>(max_inliers) / n < good_lane_ratio) {
    return false;
  }
  for (int i = 0; i < n; ++i) {
    double y = pos_vec[i](0) * pos_vec[i](0) * (*coeff)(2) + pos_vec[i](0) * (*coeff)(1) + (*coeff)(0);
    if (std::abs(y - pos_vec[i](1)) <= inlier_thres) {
      selected_points->push_back(pos_vec[i]);
    }
  }
  return true;
}

void LaneGuidance::AssignLaneMarkerColor(BevMapInfo &bev_map) {
  for (auto &laneMarker : bev_map.lanemarkers) {
    auto             points = laneMarker.line_points;
    std::vector<int> cutIdx{0};
    if (points.empty()) {
      continue;
    }
    for (int ptIdx = 0; ptIdx < points.size() - 1; ptIdx++) {
      cem::message::common::Point2DF prePt;
      cem::message::common::Point2DF currPt;
      if (prePt.color != currPt.color && prePt.color != PointColor::BEV_LMC__UNKNOWN && currPt.color != PointColor::BEV_LMC__UNKNOWN) {
        cutIdx.emplace_back(ptIdx);
        cutIdx.emplace_back(ptIdx + 1);
      }
    }
    int endIdx = static_cast<int>(points.size() - 1);
    cutIdx.emplace_back(endIdx);
    std::vector<BevLmColorSeg> colorSeg;
    if (cutIdx.size() < 3) {
      BevLmColorSeg seg;
      seg.color       = static_cast<BevLaneMarkerColor>(laneMarker.line_points.front().color);
      seg.start_index = 0;
      seg.end_index   = endIdx;
      colorSeg.emplace_back(seg);
      laneMarker.color_segs.clear();
      laneMarker.color_segs = colorSeg;
      continue;
    }
    laneMarker.color_segs.clear();
    for (int idx = 0; idx < cutIdx.size(); idx += 2) {
      BevLmColorSeg seg;
      seg.color       = static_cast<BevLaneMarkerColor>(laneMarker.line_points[idx].color);
      seg.start_index = cutIdx[idx];
      seg.end_index   = cutIdx[idx + 1];
      laneMarker.color_segs.emplace_back(seg);
    }
  }
}

void LaneGuidance::AssignSectionLaneMarkerColor(BevMapInfo &bev_map) {
  AssignSectionLaneMarkerColorImplement(bev_map.route.sections);
  for (auto &subpath : bev_map.route.subpaths) {
    AssignSectionLaneMarkerColorImplement(subpath.sections);
  }
}

void LaneGuidance::AssignSectionLaneMarkerColorImplement(std::vector<BevMapSectionInfo> &sections) {
  for (auto &section : sections) {
    for (auto &lanemarker : section.lanemarkers) {
      auto points = lanemarker.line_points;
      std::vector<int> cut_idx{0};
      if (points.size() < 2) {
        continue;
      }

      for (size_t pt_idx = 0; pt_idx < points.size() - 1; pt_idx++) {
        if (points[pt_idx + 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
          points[pt_idx + 1].color = points[pt_idx].color;
        }
      }

      for (size_t pt_idx = points.size() - 1; pt_idx > 0; pt_idx--) {
        if (points[pt_idx - 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
          points[pt_idx - 1].color = points[pt_idx].color;
        }
      }

      for (int pt_idx = 0; pt_idx < points.size() - 1; pt_idx++) {
        auto pre_pt  = points[pt_idx];
        auto curr_pt = points[pt_idx + 1];
        if (pre_pt.color != curr_pt.color && curr_pt.color != PointColor::BEV_LMC__UNKNOWN && pre_pt.color != PointColor::BEV_LMC__UNKNOWN) {
          cut_idx.emplace_back(pt_idx);
          cut_idx.emplace_back(pt_idx + 1);
        }
      }
      int end_idx = static_cast<int>(points.size() - 1);
      cut_idx.emplace_back(end_idx);
      std::vector<BevLmColorSeg> color_segs;
      if (cut_idx.size() < 3) {
        BevLmColorSeg seg;
        if (points.front().color == PointColor::BEV_LMC__UNKNOWN) {
          seg.color = static_cast<BevLaneMarkerColor>(PointColor::BEV_LMC__WHITE);
        } else {
          seg.color = static_cast<BevLaneMarkerColor>(points.front().color);
        }
        seg.start_index = 0;
        seg.end_index   = end_idx;
        color_segs.emplace_back(seg);
        lanemarker.color_segs.clear();
        lanemarker.color_segs = color_segs;
        continue;
      }
      lanemarker.color_segs.clear();
      for (int idx = 0; idx <= cut_idx.size() - 2; idx += 2) {
        BevLmColorSeg seg;
        seg.color       = static_cast<BevLaneMarkerColor>(points[cut_idx[idx]].color);
        seg.start_index = cut_idx[idx];
        seg.end_index   = cut_idx[idx + 1];
        lanemarker.color_segs.emplace_back(seg);
      }
    }
  }
}

void LaneGuidance::BoundSectionLaneMarkerWithLane(BevMapInfo &bev_map) {
  BoundSectionLaneMarkerWithLaneImplement(bev_map.route.sections, bev_map);
  // for (auto &subpath : bev_map.route.subpaths) {
  //   BoundSectionLaneMarkerWithLaneImplement(subpath.sections, bev_map);
  // }
}

void LaneGuidance::BoundSectionLaneMarkerWithLaneImplement(std::vector<BevMapSectionInfo> &sections, const BevMapInfo &bev_map) {
  for (auto &section : sections) {
    for (auto &lane : section.lane_infos) {
      // 查找感知的左右车道线id
      uint32_t leftLaneMarkerId = 0;
      uint32_t rightLaneMarkerId = 0;
      uint32_t lane_remainder = lane.id % static_cast<uint32_t>(1e3);
      for (auto &bev_lane : bev_map.lane_infos) {
        if (lane_remainder == bev_lane.id) {
          leftLaneMarkerId = bev_lane.left_lane_marker_id;
          rightLaneMarkerId = bev_lane.right_lane_marker_id;
        }
      }
      // 计算切割后的左右车道线id
      std::set<uint64_t> leftLaneIds;
      std::set<uint64_t> rightLaneIds;
      for (auto &laneMarker : section.lanemarkers) {
        uint32_t remainder = laneMarker.id % static_cast<uint32_t>(1e3);
        if (remainder == leftLaneMarkerId) {
          leftLaneIds.insert(laneMarker.id);
        }
        if (remainder == rightLaneMarkerId) {
          rightLaneIds.insert(laneMarker.id);
        }
      }
      // 取出去重后的左右车道线id
      lane.left_lane_boundary_ids.clear();
      for (auto idx : leftLaneIds) {
        lane.left_lane_boundary_ids.emplace_back(idx);
      }
      lane.right_lane_boundary_ids.clear();
      for (auto idx : rightLaneIds) {
        lane.right_lane_boundary_ids.emplace_back(idx);
      }
    }
  }
}

void LaneGuidance::BoundLaneMarkerWithLane(BevMapInfo &bev_map) {
  for (auto &lane : bev_map.lane_infos) {
    uint32_t leftLaneMarkerId;
    leftLaneMarkerId = lane.left_lane_marker_id;
    uint32_t rightLaneMarkerId;
    rightLaneMarkerId = lane.right_lane_marker_id;
    std::set<uint64_t> leftLaneIds;
    std::set<uint64_t> rightLaneIds;
    for (auto laneMarker : bev_map.lanemarkers) {
      int remainder = laneMarker.id % static_cast<uint32_t>(1e3);
      if (remainder == leftLaneMarkerId) {
        leftLaneIds.insert(laneMarker.id);
      }
      if (remainder == rightLaneMarkerId) {
        rightLaneIds.insert(laneMarker.id);
      }
    }
    for (auto &section : bev_map.route.sections) {
      for (auto &laneMarker : section.lanemarkers) {
        int remainder = laneMarker.id % static_cast<uint32_t>(1e3);
        if (remainder == leftLaneMarkerId) {
          leftLaneIds.insert(laneMarker.id);
        }
        if (remainder == rightLaneMarkerId) {
          rightLaneIds.insert(laneMarker.id);
        }
      }
    }
    lane.left_lane_boundary_ids.clear();
    for (auto idx : leftLaneIds) {
      lane.left_lane_boundary_ids.emplace_back(idx);
    }
    lane.right_lane_boundary_ids.clear();
    for (auto idx : rightLaneIds) {
      lane.right_lane_boundary_ids.emplace_back(idx);
    }
  }
}

void LaneGuidance::CutSectionWithTypeSeg(BevMapInfo &bev_map) {
  // 存储所有laneMarker的id，用于判断id是否重复
  std::vector<BevMapSectionInfo> all_sections;
  size_t num_sections = bev_map.route.sections.size();
  for (const auto &subpath : bev_map.route.subpaths) {
    num_sections += subpath.sections.size();
  }
  all_sections.reserve(num_sections);
  all_sections.insert(all_sections.end(), bev_map.route.sections.begin(), bev_map.route.sections.end());
  for (const auto &subpath : bev_map.route.subpaths) {
    all_sections.insert(all_sections.end(), subpath.sections.begin(), subpath.sections.end());
  }

  std::set<uint32_t> laneMarkerIds;
  for (auto &section : all_sections) {
    for (const auto &laneMarker : section.lanemarkers) {
      laneMarkerIds.insert(laneMarker.id);
    }
  }

  CutSectionWithTypeSegImplement(bev_map.route.sections, laneMarkerIds);
  for (auto &subpath : bev_map.route.subpaths) {
    CutSectionWithTypeSegImplement(subpath.sections, laneMarkerIds);
  }
}

void LaneGuidance::CutSectionWithTypeSegImplement(std::vector<BevMapSectionInfo> &sections, std::set<uint32_t> &laneMarkerIds) {
  for (auto &section : sections) {
    std::vector<BevLaneMarker> lanemarker_vector;
    for (auto laneMarker : section.lanemarkers) {
      auto             points = laneMarker.line_points;
      if (points.empty()) {
        continue;
      }
      // 存储分段首尾点的索引
      std::vector<int> cutIdx{0};
      for (int ptIdx = 0; ptIdx < points.size() - 1; ptIdx++) {
        auto prePt  = points[ptIdx];
        auto currPt = points[ptIdx + 1];
        if ((prePt.type != currPt.type) && (currPt.type != PointType::BEV_LMT__UNDECIDED) &&
            (prePt.type != PointType::BEV_LMT__UNDECIDED)) {
          cutIdx.emplace_back(ptIdx + 1);
          cutIdx.emplace_back(ptIdx + 1);
        }
      }
      int endIdx = static_cast<int>(points.size() - 1);
      cutIdx.emplace_back(endIdx);

      // 如果没有分段，那么将第1个点的类型赋给车道线
      if (cutIdx.size() < 3) {
        for (const auto &pt : laneMarker.line_points) {
          if (pt.type != PointType::BEV_LMT__UNDECIDED) {
            laneMarker.type = static_cast<BevLaneMarkerType>(pt.type);
            break;
          } else {
            laneMarker.type = cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
          }
        }
        lanemarker_vector.emplace_back(laneMarker);
        continue;
      }

      // 根据首尾点索引进行分段，并赋上类型和ID
      for (int idx = 0; idx < cutIdx.size(); idx += 2) {
        BevLaneMarker                               seg = laneMarker;
        std::vector<cem::message::common::Point2DF> segPoints;
        segPoints.assign(points.begin() + cutIdx[idx], points.begin() + cutIdx[idx + 1] + 1);
        seg.line_points.clear();
        seg.line_points = segPoints;
        for (const auto &pt : segPoints) {
          if (pt.type != PointType::BEV_LMT__UNDECIDED) {
            seg.type = static_cast<BevLaneMarkerType>(pt.type);
            break;
          }
        }
        seg.id = idx * 1e3 + laneMarker.id;
        while (laneMarkerIds.find(seg.id) != laneMarkerIds.end() && idx != 0) {
          seg.id += 1e3;
        }
        laneMarkerIds.insert(seg.id);
        lanemarker_vector.emplace_back(seg);
      }
    }
    section.lanemarkers.clear();
    section.lanemarkers = lanemarker_vector;
  }
}

void LaneGuidance::CutLaneMarkerWithTypeSeg(BevMapInfo &bevMap) {
  std::vector<BevLaneMarker> laneMarkerVec;
  for (auto &laneMarker : bevMap.lanemarkers) {
    auto             points = laneMarker.line_points;
    if (points.empty()) {
      continue;
    }

    // 存储分段首尾点的索引
    std::vector<int> cutIdx{0};
    for (int ptIdx = 0; ptIdx < points.size() - 1; ptIdx++) {
      auto prePt  = points[ptIdx];
      auto currPt = points[ptIdx + 1];
      if ((prePt.type != currPt.type) && (prePt.type != PointType::BEV_LMT__UNDECIDED) && (currPt.type != PointType::BEV_LMT__UNDECIDED)) {
        cutIdx.emplace_back(ptIdx + 1);
        cutIdx.emplace_back(ptIdx + 1);
      }
    }
    int endIdx = static_cast<int>(points.size() - 1);
    cutIdx.emplace_back(endIdx);

    // 对laneMarker进行分段，并赋上类型和ID
    if (cutIdx.size() < 3) {
      for (const auto &pt : laneMarker.line_points) {
        if (pt.type != PointType::BEV_LMT__UNDECIDED) {
          laneMarker.type = static_cast<BevLaneMarkerType>(pt.type);
          break;
        } else {
          laneMarker.type = cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
        }
      }
      laneMarkerVec.emplace_back(laneMarker);
      continue;
    }
    for (int idx = 0; idx < cutIdx.size(); idx += 2) {
      BevLaneMarker                               seg = laneMarker;
      std::vector<cem::message::common::Point2DF> segPoints;
      segPoints.assign(points.begin() + cutIdx[idx], points.begin() + cutIdx[idx + 1] + 1);
      seg.line_points.clear();
      seg.line_points = segPoints;
      for (auto pt : segPoints) {
        if (pt.type != PointType::BEV_LMT__UNDECIDED) {
          seg.type = static_cast<BevLaneMarkerType>(pt.type);
          break;
        }
      }
      seg.id = idx * 1e3 + laneMarker.id;
      laneMarkerVec.emplace_back(seg);
    }
  }
  bevMap.lanemarkers.clear();
  bevMap.lanemarkers = laneMarkerVec;

  // 将每个点的类型统一为laneMarker的类型
  for (auto &laneMarker : bevMap.lanemarkers) {
    PointType laneType = PointType::BEV_LMT__UNDECIDED;
    for (auto point : laneMarker.line_points) {
      if (point.type != PointType::BEV_LMT__UNDECIDED) {
        laneType = point.type;
        break;
      }
    }
    for (auto &point : laneMarker.line_points) {
      point.type = laneType;
    }
  }
}

void LaneGuidance::RemoveRepeatPoint(BevMapInfo &bevMap) {
  for (auto &line : bevMap.lane_infos) {
    std::vector<cem::message::common::Point2DF> points = line.line_points;
    if (points.empty()) {
      continue;
    }
    std::set<int> matchIdx;
    for (int firstIdx = 0; firstIdx < (points.size() - 1); firstIdx++) {
      const Eigen::Vector3d firstPt{points[firstIdx].x, points[firstIdx].y, 0.0};
      for (int secondIdx = (firstIdx + 1); secondIdx < points.size(); secondIdx++) {
        const Eigen::Vector3d secondPt{points[secondIdx].x, points[secondIdx].y, 0.0};
        const double          dist = (secondPt - firstPt).norm();
        if (dist < kDistanceThreshold) {
          matchIdx.insert(secondIdx);
        }
      }
    }
    std::vector<cem::message::common::Point2DF> filtedPoints;
    for (int idx = 0; idx < points.size(); idx++) {
      if (matchIdx.find(idx) == matchIdx.end()) {
        filtedPoints.emplace_back(points[idx]);
      }
    }
    line.line_points.clear();
    line.line_points = filtedPoints;
  }
}

void LaneGuidance::CalculateCutLine(BevMapSectionInfo &section, const LaneGuidance::CutPoint &lane_cut_point,
                                    std::pair<double, double> &line_cofficients) {
  cem::message::common::Point2DF cut_point;
  cut_point.x = lane_cut_point.dr_point.x();
  cut_point.y = lane_cut_point.dr_point.y();

  Eigen::Vector2d                project_point = {cut_point.x, cut_point.y};
  cem::message::common::Point2DF nearest_pt1, nearest_pt2;
  bool                           is_pt_assigned = false;
  for (const auto &lane : section.lane_infos) {
    if (lane.line_points.size() < 2) {
      continue;
    }
    if (std::find(lane_cut_point.merge_lane_ids.begin(), lane_cut_point.merge_lane_ids.end(), lane.id) !=
        lane_cut_point.merge_lane_ids.end()) {
      continue;  //如果属于切割点的合并车道，那么不在该车道上找投影点
    }
    for (size_t i = 0; i < lane.line_points.size() - 1; i++) {
      double dist1 = std::sqrt(std::pow(lane.line_points[i].x - cut_point.x, 2) + std::pow(lane.line_points[i].y - cut_point.y, 2));
      double dist2 = std::sqrt(std::pow(lane.line_points[i + 1].x - cut_point.x, 2) + std::pow(lane.line_points[i + 1].y - cut_point.y, 2));
      if (dist1 <= dist2 && dist1 < 10.0 && dist2 < 10.0) {
        is_pt_assigned = true;
        nearest_pt1    = lane.line_points[i];
        nearest_pt2    = lane.line_points[i + 1];
        break;  //在10米范围内，查找每条车道的最近点
      }
    }
    if (is_pt_assigned && lane.previous_lane_ids.empty() && lane.next_lane_ids.empty()) {
      break;  //如果找到最近点的车道没有前后继，那么优先用该点计算切割线
    }
  }

  if (is_pt_assigned) {
    // 将找到的2个最近点连成直线，计算切割点到该直线的垂足作为投影点
    project_point = GetFootPoint(cut_point, nearest_pt1, nearest_pt2);
  } else {
    // 否则，将车道起点和终点连成直线，计算切割点到该直线的垂足作为投影点
    for (const auto &lane : section.lane_infos) {
      if (lane.line_points.size() < 2) {
        continue;
      }
      Eigen::Vector2d lane_begin(lane.line_points.front().x, lane.line_points.front().y);
      Eigen::Vector2d lane_vector(lane.line_points.back().x - lane_begin.x(), lane.line_points.back().y - lane_begin.y());
      Eigen::Vector2d lane_to_cut_point(cut_point.x - lane.line_points.front().x, cut_point.y - lane.line_points.front().y);
      double          lane_length = lane_vector.norm();
      double          value       = lane_vector.dot(lane_to_cut_point) / lane_length;
      if (value > 0 && value < lane_length) {
        project_point = lane_begin + value * lane_vector.normalized();
        break;
      }
    }
  }

  // 计算世界坐标系的直线方程y=kx+b
  if (fabs(cut_point.x - project_point.x()) < 1e-5 && fabs(cut_point.y - project_point.y()) < 1e-5) {  //两点重合，直线无效
    line_cofficients.first  = std::numeric_limits<double>::max();
    line_cofficients.second = std::numeric_limits<double>::max();
  } else {
    line_cofficients.first  = std::fabs(cut_point.x - project_point.x()) < 1e-5
                                  ? std::numeric_limits<double>::max()  //斜率无穷大，即直线为x=cut_point.x
                                  : (cut_point.y - project_point.y()) / (cut_point.x - project_point.x());
    line_cofficients.second = std::fabs(cut_point.x - project_point.x()) < 1e-5 ? cut_point.x  //斜率无穷大，使用b记录cut_point.x的值
                                                                                : cut_point.y - line_cofficients.first * cut_point.x;
  }

  // 计算自车坐标系的直线方程Ax+By+C=0
  Eigen::Vector3d line_func_ego = {0, 0, 0};
  Eigen::Vector3d cut_pt_ego    = {cut_point.x, cut_point.y, 0};
  cut_pt_ego                    = T_local_ego_.inverse() * cut_pt_ego;
  Eigen::Vector3d proj_pt_ego   = {project_point.x(), project_point.y(), 0};
  proj_pt_ego                   = T_local_ego_.inverse() * proj_pt_ego;
  if (fabs(cut_point.x - project_point.x()) < 1e-5 && fabs(cut_point.y - project_point.y()) < 1e-5) {  //两点重合，直线无效
    line_func_ego.x() = 0;
    line_func_ego.y() = 0;
    line_func_ego.z() = cut_pt_ego.x();  //使用C记录cut_pt_ego.x的值
    section.back_left = cut_pt_ego + Eigen::Vector3d(0, 1, 0);
    section.back_right = cut_pt_ego - Eigen::Vector3d(0, 1, 0);
  } else {
    line_func_ego.x() = cut_pt_ego.y() - proj_pt_ego.y();
    line_func_ego.y() = proj_pt_ego.x() - cut_pt_ego.x();
    line_func_ego.z() = (proj_pt_ego.y() - cut_pt_ego.y()) * cut_pt_ego.x() - (proj_pt_ego.x() - cut_pt_ego.x()) * cut_pt_ego.y();
    section.back_left = cut_pt_ego.y() > proj_pt_ego.y() ? cut_pt_ego : proj_pt_ego;
    section.back_right = cut_pt_ego.y() < proj_pt_ego.y() ? cut_pt_ego : proj_pt_ego;
  }
  section.line_func_ego = line_func_ego;
}

bool LaneGuidance::IsLaneInPreviousSection(const BevLaneInfo &lane_infos, const cem::message::common::Point2DF &cut_point,
                                           const std::pair<double, double> &line_cofficients) {
  if (lane_infos.line_points.size() < 2) {
    return false;
  }
  double dx = 1.0;
  double dy = line_cofficients.first * dx;

  double          magnitude = std::sqrt(dx * dx + dy * dy);
  Eigen::Vector2d line_direction(dx / magnitude, dy / magnitude);

  Eigen::Vector2d clockwise_vector(-line_direction.y(), line_direction.x());
  Eigen::Vector2d counter_clockwise_vector(line_direction.y(), -line_direction.x());

  Eigen::Vector2d lane_direction(lane_infos.line_points.back().x - lane_infos.line_points.front().x,
                                 lane_infos.line_points.back().y - lane_infos.line_points.front().y);

  double dot_1st = clockwise_vector.dot(clockwise_vector);
  double dot_2nd = counter_clockwise_vector.dot(clockwise_vector);

  Eigen::Vector2d principal_direction = (dot_1st > dot_2nd) ? clockwise_vector : counter_clockwise_vector;

  Eigen::Vector2d cut_point_2_lane_back_point(lane_infos.line_points.back().x - cut_point.x, lane_infos.line_points.back().y - cut_point.y);

  Eigen::Vector2d cut_point_2_lane_front_point(lane_infos.line_points.back().x - cut_point.x,
                                               lane_infos.line_points.back().y - cut_point.y);

  if (cut_point_2_lane_back_point.dot(principal_direction) < 0 && cut_point_2_lane_front_point.dot(principal_direction) < 0) {
    return true;
  }

  return false;
}

bool LaneGuidance::IsEgoInPreviousSection(const cem::message::common::Point2DF &cut_point,
                                          const std::pair<double, double>      &line_cofficients) {
  double dx = 1.0;
  double dy = line_cofficients.first * dx;

  double          magnitude = std::sqrt(dx * dx + dy * dy);
  Eigen::Vector2d line_direction(dx / magnitude, dy / magnitude);

  Eigen::Vector2d clockwise_vector(-line_direction.y(), line_direction.x());
  Eigen::Vector2d counter_clockwise_vector(line_direction.y(), -line_direction.x());

  Eigen::Vector2d lane_direction(T_local_ego_.rotation()(0, 0), T_local_ego_.rotation()(1, 0));

  // double dot_1st = clockwise_vector.dot(clockwise_vector);
  // double dot_2nd = counter_clockwise_vector.dot(clockwise_vector);

  // Eigen::Vector2d principal_direction = (dot_1st > dot_2nd) ? clockwise_vector : counter_clockwise_vector;

  Eigen::Vector2d cut_point_2_ego(T_local_ego_.translation().x() - cut_point.x, T_local_ego_.translation().y() - cut_point.y);

  Eigen::Vector3d ego_cut_point(cut_point.x, cut_point.y, 0);
  ego_cut_point = T_local_ego_.inverse() * ego_cut_point;

  // TODO: 优化只有车道2合1场景的line_cofficients生成
  // if (cut_point_2_ego.dot(principal_direction) < 0)
  // {
  //     return true;
  // }
  if (ego_cut_point.x() > 0) {
    return true;
  }

  return false;
}

void LaneGuidance::InitPose() {
  LocalizationPtr local_pose_ptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(local_pose_ptr);
  if (!local_pose_ptr) {
    return;
  }

  T_local_ego_.translation() = Eigen::Vector3d(local_pose_ptr->posne_dr.at(0), local_pose_ptr->posne_dr.at(1), 0);

  T_local_ego_.linear() = Eigen::AngleAxisd(local_pose_ptr->attitude_dr * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void LaneGuidance::CutLaneMarkers(const std::pair<double, double> &line_cofficients, std::vector<BevLaneMarker> &lanemarkers,
                                  BevMapSectionInfo &section_1st, BevMapSectionInfo &section_2nd, Graph<uint64_t> &lanemarkers_graph) {
  if (line_cofficients.first == std::numeric_limits<double>::max() && line_cofficients.second == std::numeric_limits<double>::max()) {
    return;
  }
  for (size_t i = 0; i < lanemarkers.size(); i++) {
    if (lanemarkers[i].line_points.size() < 2) {
      continue;
    }
    size_t cut_id = 0;
    for (size_t j = 0; j < lanemarkers[i].line_points.size() - 1; j++) {
      float pt1dir = line_cofficients.first * lanemarkers[i].line_points[j].x + line_cofficients.second - lanemarkers[i].line_points[j].y;
      float pt2dir =
          line_cofficients.first * lanemarkers[i].line_points[j + 1].x + line_cofficients.second - lanemarkers[i].line_points[j + 1].y;
      if (pt1dir * pt2dir > 0) {
        continue;
      }
      cut_id = j;  //如果这一小段lanemarker与line相交，那么记为分割点
      break;
    }

    if (cut_id == 0 || cut_id == lanemarkers[i].line_points.size() - 1) {  //如果分割点是第1个点或最后1个点，那么分割无效
      continue;
    }
    std::vector<cem::message::common::Point2DF>::iterator split_iter;
    split_iter = lanemarkers[i].line_points.begin() + cut_id;

    auto segmented_lanemarker = lanemarkers[i];
    segmented_lanemarker.line_points.assign(split_iter, lanemarkers[i].line_points.end());  //新分割的lanemarker是从分割点到终点
    segmented_lanemarker.number_of_points = segmented_lanemarker.line_points.size();

    lanemarkers[i].line_points.assign(lanemarkers[i].line_points.begin(), split_iter + 1);  //原来的lanemarker是从起点到分割点
    lanemarkers[i].number_of_points = lanemarkers[i].line_points.size();

    // rename id
    const Graph<uint64_t>::Node *lanemarker_1st_node = lanemarkers_graph.GetNode(lanemarkers[i].id);
    if (lanemarker_1st_node && !lanemarker_1st_node->successors.empty()) {
      segmented_lanemarker.id = lanemarker_1st_node->successors.front()->id;
    } else {
      segmented_lanemarker.id = 1e3 + lanemarkers[i].id;  //取1000+原来的id作为新分割的lanemarker的id
    }
    section_2nd.lanemarkers.push_back(segmented_lanemarker);
    lanemarkers_graph.AddEdge(lanemarkers[i].id, segmented_lanemarker.id);
  }
}

void LaneGuidance::CutLanes(const std::pair<double, double> &line_cofficients, const std::vector<LaneGuidance::CutPoint> &cut_points,
                            BevMapInfo &bev_map, uint64_t cur_road_id, BevMapSectionInfo &section_1st, BevMapSectionInfo &section_2nd,
                            Graph<uint64_t> &lanes_graph, Graph<uint64_t> &lanemarkers_graph) {
  std::set<uint64_t> lane_ids_within_section;
  std::set<uint32_t> lane_marker_ids_within_section;
  for (size_t i = 0; i < section_1st.lane_infos.size(); i++) {
    bool   is_lane_with_successor = false;
    bool   is_stop_line           = false;

    for (const auto &cut_point : cut_points) {
      if (cut_point.is_stop_line) {
        is_stop_line = true;
      }

      if (section_1st.lane_infos[i].id <= kMaxBevTrackID) {
        if (cut_point.is_stop_line) {
          continue;
        }
        // 如果是合并到切割点的车道之一，那么必然有后继
        for (size_t j = 0; j < cut_point.merge_lane_ids.size(); j++) {
          if (section_1st.lane_infos[i].id == cut_point.merge_lane_ids[j]) {
            is_lane_with_successor = true;
            break;
          }
        }
        if (is_lane_with_successor) {
          break;
        }
      } else {
        const auto         section_1st_lane_node = lanes_graph.GetNode(section_1st.lane_infos[i].id);
        std::set<uint64_t> predecessors;

        if (section_1st_lane_node) {
          int depth = 0;
          lanes_graph.TraversePredecessors(section_1st_lane_node, predecessors, depth);
        }

        for (size_t j = 0; j < cut_point.merge_lane_ids.size(); j++) {
          if (predecessors.count(cut_point.merge_lane_ids[j])) {
            is_lane_with_successor = true;
            break;
          }
        }
        if (is_lane_with_successor) {
          break;
        }
      }
    }

    if (is_lane_with_successor && !section_1st.lane_infos[i].next_lane_ids.empty()) {
      for (size_t j = 0; j < section_1st.lane_infos[i].next_lane_ids.size(); j++) {
        if (lane_ids_within_section.count(section_1st.lane_infos[i].next_lane_ids[j])) {
          continue;
        }

        auto next_lane =
            std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                         [find_id = section_1st.lane_infos[i].next_lane_ids[j]](BevLaneInfo &lane) { return lane.id == find_id; });

        if (next_lane != bev_map.lane_infos.end() && next_lane->road_id == cur_road_id) {
          section_2nd.lane_infos.push_back(*next_lane);
          lane_ids_within_section.insert(next_lane->id);
          lane_marker_ids_within_section.insert(next_lane->left_lane_marker_id);
          lane_marker_ids_within_section.insert(next_lane->right_lane_marker_id);
          lanes_graph.AddEdge(section_1st.lane_infos[i].id, next_lane->id);
        }
      }
      continue;  //如果这条lane有后继，那么直接把后继放到section_2nd，不再进行下面的分割
    }

    if (section_1st.lane_infos[i].line_points.size() < 2) {
      continue;
    }

    size_t cut_id = std::numeric_limits<size_t>::max();
    cem::message::common::Point2DF inserted_point;
    inserted_point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
    inserted_point.x = 0;
    inserted_point.y = 0;
    if (line_cofficients.first == std::numeric_limits<double>::max() && line_cofficients.second == std::numeric_limits<double>::max()) {
      if(!section_1st.lane_infos[i].next_lane_ids.empty()) { //保证后继的section_id!=0
        cut_id = 0;
        inserted_point.x = (section_1st.lane_infos[i].line_points[cut_id].x + section_1st.lane_infos[i].line_points[cut_id+1].x) / 2.0;
        inserted_point.y = (section_1st.lane_infos[i].line_points[cut_id].y + section_1st.lane_infos[i].line_points[cut_id+1].y) / 2.0;
      } else {
        continue;
      }
    }
    else {
      for (size_t j = 0; j < section_1st.lane_infos[i].line_points.size() - 1; j++) {
        float pt1dir = 0;
        float pt2dir = 0;
        if (line_cofficients.first != std::numeric_limits<double>::max()) {
          pt1dir = line_cofficients.first * section_1st.lane_infos[i].line_points[j].x + line_cofficients.second -
                  section_1st.lane_infos[i].line_points[j].y;
          pt2dir = line_cofficients.first * section_1st.lane_infos[i].line_points[j + 1].x + line_cofficients.second -
                  section_1st.lane_infos[i].line_points[j + 1].y;
        } else {
          pt1dir = line_cofficients.second - section_1st.lane_infos[i].line_points[j].x;
          pt2dir = line_cofficients.second - section_1st.lane_infos[i].line_points[j + 1].x;
        }

        if (pt1dir * pt2dir > 0) {
          continue;
        }

        cut_id = j;  //如果这一小段lane与line相交，那么记为分割点
        break;
      }

      // 计算分割线与车道中心线的交点
      if (cut_id == std::numeric_limits<size_t>::max()) { //如果切割不到lane，那么判断lane位于切割线的前方还是后方
        double k = line_cofficients.first;
        double b = line_cofficients.second;
        double x = section_1st.lane_infos[i].line_points.front().x;
        double y = section_1st.lane_infos[i].line_points.front().y;
        if (std::abs(k) < 1e-5 || k == std::numeric_limits<double>::max()) {
          if(!section_1st.lane_infos[i].next_lane_ids.empty()) { //保证后继的section_id!=0
            cut_id = 0;
          } else {
            continue;
          }
        }
        double line_x = (y - b) / k;
        Eigen::Vector3d lane_point_ego = T_local_ego_.inverse() * Eigen::Vector3d(x, y, 0);
        Eigen::Vector3d line_point_ego = T_local_ego_.inverse() * Eigen::Vector3d(line_x, y, 0);
        if(line_point_ego.x() < lane_point_ego.x()) { //如果切割线位于lane前方，那么切割lane起始一小段
          cut_id = 0;
        }
        else { //如果切割线位于lane后方，那么切割lane末尾一小段
          if(!section_1st.lane_infos[i].next_lane_ids.empty()) { //保证后继的section_id!=0
            cut_id = section_1st.lane_infos[i].line_points.size() - 2;
          } else {
            continue;
          }
        }
        inserted_point.x = (section_1st.lane_infos[i].line_points[cut_id].x + section_1st.lane_infos[i].line_points[cut_id+1].x) / 2.0;
        inserted_point.y = (section_1st.lane_infos[i].line_points[cut_id].y + section_1st.lane_infos[i].line_points[cut_id+1].y) / 2.0;
      }
      else {
        double k1 = line_cofficients.first, b1 = line_cofficients.second;
        double x1 = section_1st.lane_infos[i].line_points[cut_id].x, y1 = section_1st.lane_infos[i].line_points[cut_id].y;
        double x2 = section_1st.lane_infos[i].line_points[cut_id + 1].x, y2 = section_1st.lane_infos[i].line_points[cut_id + 1].y;
        if (k1 != std::numeric_limits<double>::max()) {
          if (fabs(x2 - x1) > 1e-5) {
            double k2 = (y2 - y1) / (x2 - x1), b2 = y2 - k2 * x2;
            double x = 0;
            if(fabs(k1 - k2) > 1e-5) {
              x = (b2 - b1) / (k1 - k2);
            }
            else {
              x = (x1 + x2) / 2.0;
            }
            inserted_point.x = x;
            inserted_point.y = k1 * x + b1;
          } else {
            inserted_point.x = x1;
            inserted_point.y = k1 * x1 + b1;
          }
        } else {
          if (fabs(x2 - x1) > 1e-5) {
            double k2 = (y2 - y1) / (x2 - x1), b2 = y2 - k2 * x2;
            inserted_point.x = b1;
            inserted_point.y = k2 * b1 + b2;
          } else {
            inserted_point.x = (x1 + x2) / 2.0;
            inserted_point.y = (y1 + y2) / 2.0;
          }
        }
      }
    }
    if(std::abs(inserted_point.x - section_1st.lane_infos[i].line_points.front().x) < 1e-3 &&
       std::abs(inserted_point.y - section_1st.lane_infos[i].line_points.front().y) < 1e-3) {
        cut_id = 0;
        inserted_point.x = (section_1st.lane_infos[i].line_points[cut_id].x + section_1st.lane_infos[i].line_points[cut_id+1].x) / 2.0;
        inserted_point.y = (section_1st.lane_infos[i].line_points[cut_id].y + section_1st.lane_infos[i].line_points[cut_id+1].y) / 2.0;
    } else if(std::abs(inserted_point.x - section_1st.lane_infos[i].line_points.back().x) < 1e-3 &&
              std::abs(inserted_point.y - section_1st.lane_infos[i].line_points.back().y) < 1e-3) {
        cut_id = section_1st.lane_infos[i].line_points.size() - 2;
        inserted_point.x = (section_1st.lane_infos[i].line_points[cut_id].x + section_1st.lane_infos[i].line_points[cut_id+1].x) / 2.0;
        inserted_point.y = (section_1st.lane_infos[i].line_points[cut_id].y + section_1st.lane_infos[i].line_points[cut_id+1].y) / 2.0;
    }

    std::vector<cem::message::common::Point2DF>::iterator split_iter;
    split_iter = section_1st.lane_infos[i].line_points.begin() + cut_id + 1;

    // 新分割的lane是从分割后点到终点，并且在最前面插入交点
    auto segmented_lane = section_1st.lane_infos[i];
    segmented_lane.line_points.assign(split_iter, section_1st.lane_infos[i].line_points.end());
    segmented_lane.line_points.insert(segmented_lane.line_points.begin(), inserted_point);
    segmented_lane.number_of_points = segmented_lane.line_points.size();
    if (is_stop_line) {
      segmented_lane.is_virtual = true;
    }

    // 原来的lane是从起点到分割前点，并且在最后面插入交点
    section_1st.lane_infos[i].line_points.assign(section_1st.lane_infos[i].line_points.begin(), split_iter);
    section_1st.lane_infos[i].line_points.push_back(inserted_point);
    section_1st.lane_infos[i].number_of_points = section_1st.lane_infos[i].line_points.size();

    if (section_1st.lane_infos[i].merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
      section_1st.lane_infos[i].merge_topo_extend             = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      section_1st.lane_infos[i].merge_info_extend.merge_valid = 0;
    }

    if (section_1st.lane_infos[i].split_topo_extend != SplitTopoExtendType::TOPOLOGY_SPLIT_NONE) {
      segmented_lane.split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
    }

    const Graph<uint64_t>::Node *lane_1st_node = lanes_graph.GetNode(section_1st.lane_infos[i].id);
    if (lane_1st_node && !lane_1st_node->successors.empty()) {
      segmented_lane.id = lane_1st_node->successors.front()->id;
    } else {
      segmented_lane.id = 1e3 + section_1st.lane_infos[i].id; //取1000+原来的id作为新分割的lane的id

      // Modified the lane ID, all next lanes' predecessor IDs need to be updated.
      for (auto next_lane_id : segmented_lane.next_lane_ids) {
        auto next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                      [id = next_lane_id](BevLaneInfo &lane) { return lane.id == id; });
        if (next_lane != bev_map.lane_infos.end()) {
          next_lane->previous_lane_ids.erase(
              std::remove(next_lane->previous_lane_ids.begin(), next_lane->previous_lane_ids.end(), section_1st.lane_infos[i].id),
              next_lane->previous_lane_ids.end());
          next_lane->previous_lane_ids.push_back(segmented_lane.id);
          // lanes_graph.AddEdge(segmented_lane.id, next_lane->id);
        }
      }
    }

    // next and prev lane id
    section_1st.lane_infos[i].next_lane_ids.clear();
    section_1st.lane_infos[i].next_lane_ids.push_back(segmented_lane.id);
    segmented_lane.previous_lane_ids.clear();
    segmented_lane.previous_lane_ids.push_back(section_1st.lane_infos[i].id);

    lanes_graph.AddEdge(section_1st.lane_infos[i].id, segmented_lane.id);

    // lanemarker
    const Graph<uint64_t>::Node *lane_1st_left_lanemarker = lanemarkers_graph.GetNode(section_1st.lane_infos[i].left_lane_marker_id);

    if (lane_1st_left_lanemarker && !lane_1st_left_lanemarker->successors.empty()) {
      segmented_lane.left_lane_marker_id = lane_1st_left_lanemarker->successors.front()->id;
    }
    const Graph<uint64_t>::Node *lane_1st_right_lanemarker = lanemarkers_graph.GetNode(section_1st.lane_infos[i].right_lane_marker_id);
    if (lane_1st_right_lanemarker && !lane_1st_right_lanemarker->successors.empty()) {
      segmented_lane.right_lane_marker_id = lane_1st_right_lanemarker->successors.front()->id;
    }

    lane_ids_within_section.insert(segmented_lane.id);
    section_2nd.lane_infos.push_back(segmented_lane);
  }

  for (const auto lane_marker_id : lane_marker_ids_within_section) {
    const auto lanemarker_iter =
        std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                     [find_id = lane_marker_id](const BevLaneMarker &lanemarker) { return lanemarker.id == find_id; });

    if (lanemarker_iter == bev_map.lanemarkers.end()) {
      continue;
    }
    section_2nd.lanemarkers.push_back(*lanemarker_iter);
  }
}

void LaneGuidance::AddLaneMarkersToSection(const BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections,
                                           std::vector<std::set<int>> lanemarker_ids, Graph<uint64_t> &lanemarker_graph) {
  for (size_t i = 0; i < lanemarker_ids.size(); i++) {
    sections.at(i).lanemarkers.reserve(lanemarker_ids[i].size());
    for (auto id : lanemarker_ids[i]) {
      uint32_t   find_id          = id;
      const auto found_lanemarker = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                 [find_id](const BevLaneMarker &lane_marker) { return lane_marker.id == find_id; });
      if (found_lanemarker != bev_map.lanemarkers.end()) {
        lanemarker_graph.AddNode(found_lanemarker->id);
        sections.at(i).lanemarkers.push_back(*found_lanemarker);
      }
    }
  }
}

void LaneGuidance::SDStateFusion(BevMapInfo &bev_map, const RoutingMapPtr routing_map_ptr) {
  // double   speed_limit    = 20;
  uint64_t ego_section_id = routing_map_ptr->sd_route.navi_start.section_id;
  auto     sd_ego_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(ego_section_id);

  // if (sd_ego_section) {
  //   speed_limit = sd_ego_section->speed_limit;
  // }

  // for (auto &lane : bev_map.lane_infos) {
  //   lane.speed_limit = speed_limit;
  // }

  bev_map.road_class    = cem::message::sensor::RoadClass(routing_map_ptr->cur_road_class);
  bev_map.is_on_highway = routing_map_ptr->is_on_highway;
}

void LaneGuidance::SetBevEmergencyLane(BevMapInfo &bev_map, const std::pair<uint64_t, uint64_t> &fusion_emergency_lane_id) {
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

// void LaneGuidance::CalculateNaviStart(BevMapInfo &bev_map, std::vector<std::vector<LaneGuidance::CutPoint>> &multi_layer_cut_points) {
//   if (bev_map.route.sections.empty()) {
//     return;
//   }

//   // 由前往后遍历，如果section有一条lane的起点在自车后方，那么为自车所在section
//   size_t ego_section_id = 0;
//   if (bev_map.route.sections.size() < 2) {
//     ego_section_id = 0;
//   } else {
//     for (int i = bev_map.route.sections.size() - 1; i >= 0; --i) {
//       bool is_found_section = false;
//       for (const auto &lane : bev_map.route.sections[i].lane_infos) {
//         if (lane.line_points.empty()) {
//           continue;
//         }
//         Eigen::Vector3d point(lane.line_points.front().x, lane.line_points.front().y, 0);
//         point = T_local_ego_.inverse() * point;
//         if (point.x() < 0) {
//           ego_section_id   = i;
//           is_found_section = true;
//           break;
//         }
//       }
//       if (is_found_section) {
//         break;
//       }
//     }
//   }
//   bev_map.route.navi_start.section_id = bev_map.route.sections.at(ego_section_id).id;

//   // 找到该section在自车最后方的lane，该lane起点作为section的起点
//   if (!bev_map.route.sections.at(ego_section_id).lane_infos.empty()) {
//     size_t start_lane_id = 0;
//     double min_dist      = DBL_MAX;
//     for (size_t i = 0; i < bev_map.route.sections.at(ego_section_id).lane_infos.size(); i++) {
//       const auto &lane = bev_map.route.sections.at(ego_section_id).lane_infos.at(i);
//       if (lane.line_points.empty()) {
//         continue;
//       }
//       Eigen::Vector3d point(lane.line_points.front().x, lane.line_points.front().y, 0);
//       point = T_local_ego_.inverse() * point;
//       if (point.x() < min_dist) {
//         min_dist      = point.x();
//         start_lane_id = i;
//       }
//     }
//     const auto &lane = bev_map.route.sections.at(ego_section_id).lane_infos.at(start_lane_id);

//     // 计算offset，为lane起点到自车原点的x方向距离
//     if (!lane.line_points.empty()) {
//       Eigen::Vector3d point(lane.line_points.front().x, lane.line_points.front().y, 0);
//       point                             = T_local_ego_.inverse() * point;
//       bev_map.route.navi_start.s_offset = std::abs(point.x());
//     }
//   }
// }

void LaneGuidance::CalculateNaviStart(BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections) {
  if (sections.empty()) {
    return;
  }

  // 由前往后遍历，判断自车所在的section
  int ego_section_id = 0;
  for (int i = sections.size() - 2; i >= 0; --i) {
    double A = sections[i].line_func_ego.x(), B = sections[i].line_func_ego.y(), C = sections[i].line_func_ego.z();
    if (fabs(A) < 1e-5 && fabs(B) < 1e-5) {  //如果切割线无效，那么根据切割点判断所在section
      if (C < 0) {
        ego_section_id = i + 1;  //section_id比切割点id多1，如第1个切割点之后为第2个section
        break;
      }
    } else if (fabs(A) < 1e-5) {  //如果切割线平行于x轴，那么自车位于掉头线的拐点，取最后一个section
      ego_section_id = sections.size() - 1;
      break;
    } else {  //如果有切割线，那么根据与自车坐标系x轴的交点判断所在section
      double x = -C / A;
      if (x < 0) {
        ego_section_id = i + 1;
        break;
      }
    }
  }
  bev_map.route.navi_start.section_id = sections.at(ego_section_id).id;

  if (!sections.at(ego_section_id).lane_infos.empty()) {
    int                            turn_lane_id = -1;
    cem::message::common::Point2DF vehicle_position;
    vehicle_position.x = T_local_ego_.translation()(0);
    vehicle_position.y = T_local_ego_.translation()(1);
    // 遍历所在section的每条lane，如果自车在左转或掉头lane的弧上，那么记录该lane_id
    for (int i = 0; i < sections.at(ego_section_id).lane_infos.size(); i++) {
      const auto &lane = sections.at(ego_section_id).lane_infos.at(i);
      if (lane.line_points.empty()) {
        continue;
      }
      if (lane.navi_action == cem::message::sensor::BevAction::LEFT_TURN || lane.navi_action == cem::message::sensor::BevAction::U_TURN) {
        double normal = CrossProduct(lane.line_points.front(), lane.line_points.back(), vehicle_position);
        if (normal <= 0) {
          turn_lane_id = i;
          break;
        }
      }
    }

    // 左转和掉头时，计算自车到lane起点的弧长作为offset
    if (turn_lane_id >= 0) {
      //找到弧上最近的点
      const auto &lane     = sections.at(ego_section_id).lane_infos.at(turn_lane_id);
      double      min_dist = DBL_MAX;
      int         min_idx  = 0;
      for (int i = 0; i < lane.line_points.size(); i++) {
        double dist = std::sqrt(std::pow(T_local_ego_.translation()(0) - lane.line_points[i].x, 2) +
                                std::pow(T_local_ego_.translation()(1) - lane.line_points[i].y, 2));
        if (dist < min_dist) {
          min_dist = dist;
          min_idx  = i;
        }
      }
      //最近点可能在车前方也可能在车后方，而最近点的上一个点一定在车后方，所以计算起点到该点的弧长
      bev_map.route.navi_start.s_offset = 0;
      for (int i = 0; i < min_idx - 1; i++) {
        bev_map.route.navi_start.s_offset += std::sqrt(std::pow(lane.line_points[i].x - lane.line_points[i + 1].x, 2) +
                                                       std::pow(lane.line_points[i].y - lane.line_points[i + 1].y, 2));
      }
      //最后加上最近点的上一个点到车身的距离
      if (min_idx > 0) {
        bev_map.route.navi_start.s_offset += std::sqrt(std::pow(T_local_ego_.translation()(0) - lane.line_points[min_idx - 1].x, 2) +
                                                       std::pow(T_local_ego_.translation()(1) - lane.line_points[min_idx - 1].y, 2));
      } else {
        bev_map.route.navi_start.s_offset += std::sqrt(std::pow(T_local_ego_.translation()(0) - lane.line_points[min_idx].x, 2) +
                                                       std::pow(T_local_ego_.translation()(1) - lane.line_points[min_idx].y, 2));
      }
    }
    // 直行和右转时，计算自车到section起点的距离作为offset
    else {
      //如果有切割线，那么计算自车到切割线的距离作为offset
      if (ego_section_id > 0) {
        double A = sections[ego_section_id-1].line_func_ego.x(), B = sections[ego_section_id-1].line_func_ego.y(),
               C = sections[ego_section_id-1].line_func_ego.z();
        if (fabs(A) > 1e-5 || fabs(B) > 1e-5) {
          bev_map.route.navi_start.s_offset = std::abs(C) / std::sqrt(A * A + B * B);
          return;
        }
      }
      //如果没有切割线，那么找到该section在自车最后方的lane，计算自车到该lane起点的x方向距离作为offset
      double min_dist = 0;
      for (const auto &lane : sections.at(ego_section_id).lane_infos) {
        if (lane.line_points.empty()) {
          continue;
        }
        Eigen::Vector3d point(lane.line_points.front().x, lane.line_points.front().y, 0);
        point = T_local_ego_.inverse() * point;
        if (point.x() < min_dist) {
          min_dist = point.x();
        }
      }
      bev_map.route.navi_start.s_offset = std::abs(min_dist);
    }
  }
}

void LaneGuidance::CalculateLengthOfSection(std::vector<BevMapSectionInfo> &sections) {
  for (auto &section : sections) {
    for (auto &lane : section.lane_infos) {
      if (lane.line_points.size() < 2) {
        continue;
      }
      for (size_t i = 1; i < lane.line_points.size(); i++) {
        double distance = std::sqrt(std::pow(lane.line_points[i].x - lane.line_points[i - 1].x, 2) +
                                    std::pow(lane.line_points[i].y - lane.line_points[i - 1].y, 2));
        section.length += distance;
      }
      break;
    }
  }
}

Eigen::Vector2d LaneGuidance::GetFootPoint(const cem::message::common::Point2DF &pt, const cem::message::common::Point2DF &begin,
                                           const cem::message::common::Point2DF &end) {
  Eigen::Vector2d footpoint;
  double          dx = begin.x - end.x;
  double          dy = begin.y - end.y;
  if (std::fabs(dx) < 1e-5 && std::fabs(dy) < 1e-5) {
    footpoint << begin.x, begin.y;
    return footpoint;
  }
  double u      = (pt.x - begin.x) * (begin.x - end.x) + (pt.y - begin.y) * (begin.y - end.y);
  u             = u / ((dx * dx) + (dy * dy));
  footpoint.x() = begin.x + u * dx;
  footpoint.y() = begin.y + u * dy;
  return footpoint;
};

void LaneGuidance::CalculateLengthOfLanes(std::vector<BevLaneInfo> &lane_infos) {
  for (auto &lane : lane_infos) {
    if (lane.line_points.size() < 2) {
      continue;
    }
    double lane_length = 0;
    for (size_t i = 1; i < lane.line_points.size(); i++) {
      lane_length += std::sqrt(std::pow(lane.line_points[i].x - lane.line_points[i - 1].x, 2) +
                               std::pow(lane.line_points[i].y - lane.line_points[i - 1].y, 2));
    }
    lane.length = lane_length;
  }
}

void LaneGuidance::CalculateLengthOfLanes(const std::vector<BevLaneInfo>::iterator &lane_begin_iter, 
                                          const std::vector<BevLaneInfo>::iterator &lane_end_iter) {
  for (std::vector<BevLaneInfo>::iterator it = lane_begin_iter; it != lane_end_iter; ++it) {
    if (it->line_points.size() < 2) {
      continue;
    }
    double lane_length = 0;
    for (size_t i = 1; i < it->line_points.size(); i++) {
      lane_length += std::sqrt(std::pow(it->line_points[i].x - it->line_points[i - 1].x, 2) +
                               std::pow(it->line_points[i].y - it->line_points[i - 1].y, 2));
    }
    it->length = lane_length;
  }
}

void LaneGuidance::DistanceToJunction(BevMapInfo &bev_map) {
  distance_to_junction_    = std::numeric_limits<double>::max();
  Eigen::Vector3d ego_pose = Eigen::Vector3d::Zero();
  for (const auto &junction : bev_map.junctions) {
    if (junction.line_points.size() < 3) {
      continue;
    }

    Eigen::Vector3d              ego_junction_point;
    std::vector<Eigen::Vector3d> ego_junction_points;
    for (size_t i = 0; i < junction.line_points.size(); i++) {
      ego_junction_point << junction.line_points[i].x, junction.line_points[i].y, 0;
      ego_junction_point = T_ego_local_ * ego_junction_point;
      ego_junction_points.push_back(ego_junction_point);
    }
    ego_junction_points.push_back(ego_junction_points.front());
    bool is_inside_polygon = IsPointInPolygon(ego_pose, ego_junction_points);
    if (is_inside_polygon) {
      distance_to_junction_ = 0;
      return;
    }

    for (size_t i = 0; i < ego_junction_points.size() - 1; i++) {
      if (ego_junction_points[i].x() > 0 && ego_junction_points[i].x() < kMaxDistanceToJunction) {
        distance_to_junction_ = std::min(distance_to_junction_, ego_junction_points[i].x());
      }
    }
  }
}

std::optional<std::pair<std::size_t, size_t>> LaneGuidance::FindMaximumSectionAndLanePosition(uint64_t id, BevMapInfo &bev_map,
                                                                                              Graph<uint64_t> &lane_graph) {
  const auto &node = lane_graph.GetNode(id);
  if (!node) {
    return {};
  }

  bool               is_found = false;
  std::set<uint64_t> sucessssor_index;
  int                depth = 0;
  lane_graph.TraverseSuccessors(node, sucessssor_index, depth);
  sucessssor_index.insert(node->id);
  std::pair<int, int> section_and_lane_position = std::make_pair(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
  for (auto sucessssor_id : sucessssor_index) {
    for (size_t i = 0; i < bev_map.route.sections.size(); i++) {
      for (size_t j = 0; j < bev_map.route.sections[i].lane_infos.size(); j++) {
        if (bev_map.route.sections[i].lane_infos[j].id != sucessssor_id) {
          continue;
        }
        if ((int)i > section_and_lane_position.first) {
          is_found                         = true;
          section_and_lane_position.first  = i;
          section_and_lane_position.second = j;
        }
      }
    }
  }

  if (is_found) {
    return section_and_lane_position;
  }

  return {};
}

void LaneGuidance::AddSuccessorLaneToSection(BevLaneInfo &lane_1st, BevLaneInfo &lane_2nd, BevMapSectionInfo &section,
                                             Graph<uint64_t> &lane_graph) {
  lane_1st.next_lane_ids.push_back(lane_2nd.id);
  lane_2nd.previous_lane_ids.push_back(lane_1st.id);
  lane_2nd.section_id = section.id;
  lane_graph.AddEdge(lane_1st.id, lane_2nd.id);

  if (lane_graph.GetNode(lane_2nd.id) && !lane_2nd.previous_lane_ids.empty() && lane_graph.GetNode(lane_2nd.previous_lane_ids.front()) &&
      lane_graph.GetNode(lane_2nd.previous_lane_ids.front())->left) {
    auto left_bottom_node = lane_graph.GetNode(lane_graph.GetNode(lane_2nd.previous_lane_ids.front())->left->id);
    if (left_bottom_node && !left_bottom_node->successors.empty()) {
      lane_2nd.left_lane_id = left_bottom_node->successors.front()->id;
    }
  }

  if (lane_graph.GetNode(lane_2nd.id) && !lane_2nd.previous_lane_ids.empty() && lane_graph.GetNode(lane_2nd.previous_lane_ids.back()) &&
      lane_graph.GetNode(lane_2nd.previous_lane_ids.back())->right) {
    auto right_bottom_node = lane_graph.GetNode(lane_graph.GetNode(lane_2nd.previous_lane_ids.back())->right->id);
    if (right_bottom_node && !right_bottom_node->successors.empty()) {
      lane_2nd.right_lane_id = right_bottom_node->successors.back()->id;
    }
  }

  section.lane_infos.push_back(lane_2nd);
}

void LaneGuidance::LaneGuidanceByRouteLanes(BevMapInfo &bev_map, Graph<uint64_t> &lane_graph,
                                            std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                            uint64_t &total_section_id) {
  /*
  bev_map：BEV地图信息
  route_lane_index: 推荐车道index的集合
  lane_graph: 车道图
  */
  if (bev_map.route.sections.empty()) {
    return;
  }

  BevMapSectionInfo virtual_section_1st, virtual_section_2nd;
  virtual_section_1st.id = ++total_section_id;
  BevLaneInfo virtual_lane;
  virtual_lane.length     = 1000;
  virtual_lane.id         = kVirtualLaneID;
  virtual_lane.is_mounted = true;

  virtual_lane.section_id    = virtual_section_1st.id;
  virtual_section_1st.length = virtual_lane.length;
  if (!bev_map.route.sections.front().lane_infos.empty()) {
    virtual_lane.speed_limit = bev_map.route.sections.front().lane_infos.front().speed_limit;
  }
  virtual_section_2nd = virtual_section_1st;

  if (has_virtual_junction_lane_) {
    for (const auto &lane : bev_map.lane_infos) {
      if (lane.is_virtual) {
        Eigen::Vector3d point(lane.line_points.front().x, lane.line_points.front().y, 0);
        point = T_ego_local_ * point;
        if (point.x() < kToggleGuidanceThreshold) {
          guide_lane_result.clear();
          guide_lane_result.push_back({lane.id, {-1, 0}});  // 只添加一个虚拟车道
        }
        break;
      }
    }
  }

  // 如果route_lane_index为空，则只添加一个虚拟车道到virtual_section，virtual_section只含一个vitual_lane
  if (guide_lane_result.empty()) {
    virtual_section_1st.lane_infos.emplace_back(virtual_lane);
    bev_map.route.sections.emplace_back(virtual_section_1st);
    return;
  }

  BevLaneInfo compensation_lane     = virtual_lane;
  compensation_lane.is_mounted      = false;
  compensation_lane.is_compensation = true;
  size_t max_section_position       = bev_map.route.sections.size();
  for (auto guide_lane : guide_lane_result) {
    if (max_section_position < 2) {
      break;
    }
    auto section_and_lane_position = FindMaximumSectionAndLanePosition(guide_lane.first, bev_map, lane_graph);
    if (section_and_lane_position == std::nullopt) {
      continue;
    }
    if (section_and_lane_position->first >= max_section_position) {
      continue;
    }
    for (size_t i = section_and_lane_position->first; i < max_section_position - 1; i++) {
      compensation_lane.id--;
      compensation_lane.previous_lane_ids.clear();
      compensation_lane.next_lane_ids.clear();
      auto &current_section = bev_map.route.sections.at(i);
      auto &next_section    = bev_map.route.sections.at(i + 1);
      if (i == section_and_lane_position->first) {
        AddSuccessorLaneToSection(current_section.lane_infos.at(section_and_lane_position->second), compensation_lane, next_section,
                                  lane_graph);
      } else {
        AddSuccessorLaneToSection(current_section.lane_infos.back(), compensation_lane, next_section, lane_graph);
      }
    }
  }

  bool is_two_layer_section = false;
  for (size_t i = 0; i < guide_lane_result.size(); i++) {
    if (guide_lane_result[i].second.second == -1) {
      is_two_layer_section = true;
      break;
    }
  }

  double pre_length = 0;
  for (const auto &section : bev_map.route.sections) {
    if (bev_map.route.navi_start.section_id == section.id) {
      pre_length += bev_map.route.navi_start.s_offset;
      break;
    }
    pre_length += section.length;
  }

  compensation_lane.is_mounted = true;
  if (is_two_layer_section) {
    virtual_section_2nd.id = ++total_section_id;
    // 如果是两层section，则需要将compensation_lane添加到virtual_section_2nd中
    for (size_t i = 0; i < guide_lane_result.size(); i++) {
      auto finded_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                      [find_id = guide_lane_result[i].first](const BevLaneInfo &p) { return p.id == find_id; });
      if (finded_lane == bev_map.lane_infos.end()) {
        continue;
      }
      const auto &node = lane_graph.GetNode(guide_lane_result[i].first);
      if (!node) {
        continue;
      }

      double lane_length = finded_lane->length;
      if (guide_lane_result[i].second.first > 0) {
        compensation_lane.id--;
        compensation_lane.length = 1000;
        compensation_lane.previous_lane_ids.clear();
        compensation_lane.next_lane_ids.clear();
        compensation_lane.left_lane_id  = 0;
        compensation_lane.right_lane_id = 0;
        compensation_lane.section_id    = virtual_section_2nd.id;
        double delta_length             = 0;
        if (lane_length > pre_length) {
          delta_length = lane_length - pre_length;
        }
        if (guide_lane_result[i].second.first > delta_length) {
          compensation_lane.length = guide_lane_result[i].second.first - delta_length;
        }
      }

      int                depth = 0;
      std::set<uint64_t> successor_index;
      lane_graph.TraverseSuccessors(node, successor_index, depth);
      successor_index.insert(node->id);
      for (size_t j = 0; j < bev_map.route.sections.back().lane_infos.size(); j++) {
        if (successor_index.count(bev_map.route.sections.back().lane_infos[j].id)) {
          compensation_lane.previous_lane_ids.push_back(bev_map.route.sections.back().lane_infos[j].id);
          bev_map.route.sections.back().lane_infos[j].next_lane_ids.push_back(compensation_lane.id);
          lane_graph.AddEdge(bev_map.route.sections.back().lane_infos[j].id, compensation_lane.id);
          break;
        }
      }

      virtual_section_2nd.lane_infos.emplace_back(compensation_lane);
    }

    double max_length = std::numeric_limits<double>::min();
    for (auto &lane : virtual_section_2nd.lane_infos) {
      if (lane.length > max_length) {
        max_length = lane.length;
      }
      // 如果车道的前驱车道存在，则将其左侧和右侧车道ID设置为前驱车道的左侧和右侧车道的后继id；
      // 注意的是多个前继或者后继场景，左边界取最左侧的lane判定，右侧取最右侧的lane判定。
      if (lane_graph.GetNode(lane.id) && !lane.previous_lane_ids.empty() && lane_graph.GetNode(lane.previous_lane_ids.front()) &&
          lane_graph.GetNode(lane.previous_lane_ids.front())->left) {
        auto left_bottom_node = lane_graph.GetNode(lane_graph.GetNode(lane.previous_lane_ids.front())->left->id);
        if (left_bottom_node && !left_bottom_node->successors.empty()) {
          lane.left_lane_id = left_bottom_node->successors.front()->id;
        }
      }

      if (lane_graph.GetNode(lane.id) && !lane.previous_lane_ids.empty() && lane_graph.GetNode(lane.previous_lane_ids.back()) &&
          lane_graph.GetNode(lane.previous_lane_ids.back())->right) {
        auto right_bottom_node = lane_graph.GetNode(lane_graph.GetNode(lane.previous_lane_ids.back())->right->id);
        if (right_bottom_node && !right_bottom_node->successors.empty()) {
          lane.right_lane_id = right_bottom_node->successors.back()->id;
        }
      }
    }
    virtual_section_2nd.length = max_length;

    bev_map.route.sections.emplace_back(virtual_section_2nd);
  }

  // 在BEV地图的最后一个section中添加一个virtual_lane，并将那些与推荐车道有前驱关系的车道与这个虚拟车道连接起来，从而形成一个完整的导航路径。
  for (size_t i = 0; i < bev_map.route.sections.back().lane_infos.size(); i++) {
    uint64_t    find_id = bev_map.route.sections.back().lane_infos[i].id;
    const auto &node    = lane_graph.GetNode(find_id);
    if (!node) {
      continue;
    }

    int                depth = 0;
    std::set<uint64_t> preprocessor_index;
    lane_graph.TraversePredecessors(node, preprocessor_index, depth);
    preprocessor_index.insert(find_id);

    std::vector<std::pair<uint64_t, std::pair<int, int>>>::iterator route_lane_result;
    for (const auto preprocessor_id : preprocessor_index) {
      if (preprocessor_id > kMaxBevTrackID) {
        continue;
      }
      route_lane_result = std::find_if(guide_lane_result.begin(), guide_lane_result.end(),
                                       [find_id = preprocessor_id](const std::pair<uint64_t, std::pair<int, int>> &lane_result) {
                                         return lane_result.first == find_id;
                                       });

      if (route_lane_result != guide_lane_result.end()) {
        break;
      }
    }

    if (route_lane_result == guide_lane_result.end()) {
      continue;
    }

    compensation_lane.id--;
    compensation_lane.section_id = virtual_section_1st.id;
    compensation_lane.length     = 1000;
    compensation_lane.previous_lane_ids.clear();
    compensation_lane.next_lane_ids.clear();
    compensation_lane.left_lane_id  = 0;
    compensation_lane.right_lane_id = 0;

    if (is_two_layer_section && route_lane_result->second.second != -1) {
      continue;
    }

    compensation_lane.previous_lane_ids.push_back(bev_map.route.sections.back().lane_infos[i].id);
    bev_map.route.sections.back().lane_infos[i].next_lane_ids.push_back(compensation_lane.id);
    virtual_section_1st.lane_infos.push_back(compensation_lane);
  }

  bev_map.route.sections.emplace_back(virtual_section_1st);
}

void LaneGuidance::InsertMergeSplitPoints(BevMapInfo &bev_map) {
  for (auto &lane : bev_map.lane_infos) {
    if (lane.is_bev_topo_connected) {
      continue;
    }
    std::set<uint64_t> next_id;
    for (auto &id : lane.next_lane_ids) {
      next_id.insert(id);
    }
    if (next_id.size() >= 2) {
      // AINFO << "go inside split";
      int  bot_lane_index            = -1;
      int  bot_leftlanemarker_index  = -1;
      int  bot_rightlanemarker_index = -1;
      auto find_bot_llanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
      auto find_bot_rlanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
      for (auto it = next_id.begin(); it != next_id.end(); it++) {
        auto find_next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                           [find_id = *it](const BevLaneInfo &p) { return p.id == find_id; });

        if (find_next_lane == bev_map.lane_infos.end()) {
          continue;
        }
        std::vector<cem::message::common::Point2DF> control_points = {};

        int top_index = -1;

        if (InsertBezierPoints(lane.line_points, find_next_lane->line_points, control_points, bot_lane_index, top_index)) {
          // AINFO<<"lane insert success";
          // AINFO<<"topo index is "<<top_index<<" lane id is "<<find_next_lane->id;

          if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
            for (auto &pt : control_points) {
              pt.mse          = NAN;
              pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
            }
            find_next_lane->line_points.erase(find_next_lane->line_points.begin(), find_next_lane->line_points.begin() + top_index + 1);
            find_next_lane->line_points.insert(find_next_lane->line_points.begin(), control_points.begin(), control_points.end());
          }
        }

        auto find_top_llanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_next_lane->left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
        auto find_top_rlanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_next_lane->right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });

        if (find_bot_llanemarker != bev_map.lanemarkers.end() && find_top_llanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         top_index      = -1;
          // AINFO<<"lanemarker insert success 1";
          if (InsertBezierPoints(find_bot_llanemarker->line_points, find_top_llanemarker->line_points, control_points,
                                 bot_leftlanemarker_index, top_index)) {
            if (bot_leftlanemarker_index >= 0 && top_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_top_llanemarker->line_points.erase(find_top_llanemarker->line_points.begin(),
                                                      find_top_llanemarker->line_points.begin() + top_index + 1);
              find_top_llanemarker->line_points.insert(find_top_llanemarker->line_points.begin(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
        if (find_bot_rlanemarker != bev_map.lanemarkers.end() && find_top_rlanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         top_index      = -1;
          // AINFO<<"lanemarker insert success 2";
          if (InsertBezierPoints(find_bot_rlanemarker->line_points, find_top_rlanemarker->line_points, control_points,
                                 bot_rightlanemarker_index, top_index)) {
            if (bot_rightlanemarker_index >= 0 && top_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_top_rlanemarker->line_points.erase(find_top_rlanemarker->line_points.begin(),
                                                      find_top_rlanemarker->line_points.begin() + top_index + 1);
              find_top_rlanemarker->line_points.insert(find_top_rlanemarker->line_points.begin(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
      }
      // AINFO<<"success lane and lane marker,bot index is"<<bot_lane_index;
      if (bot_lane_index > 0) {
        lane.line_points.erase(lane.line_points.begin() + bot_lane_index + 1, lane.line_points.end());
      }
      if (bot_leftlanemarker_index > 0) {
        find_bot_llanemarker->line_points.erase(find_bot_llanemarker->line_points.begin() + bot_leftlanemarker_index + 1,
                                                find_bot_llanemarker->line_points.end());
      }
      if (bot_rightlanemarker_index > 0) {
        find_bot_rlanemarker->line_points.erase(find_bot_rlanemarker->line_points.begin() + bot_rightlanemarker_index + 1,
                                                find_bot_rlanemarker->line_points.end());
      }
    }
    // AINFO << "go inside merge";
    std::set<uint64_t> pre_id;
    for (auto &id : lane.previous_lane_ids) {
      pre_id.insert(id);
    }
    if (pre_id.size() >= 2) {
      std::vector<cem::message::common::Point2DF> last_control_points;

      int  top_lane_index            = -1;
      int  top_leftlanemarker_index  = -1;
      int  top_rightlanemarker_index = -1;
      auto find_top_llanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
      auto find_top_rlanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
      for (auto it = pre_id.begin(); it != pre_id.end(); it++) {
        auto find_pre_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                          [find_id = *it](const BevLaneInfo &p) { return p.id == find_id; });

        if (find_pre_lane == bev_map.lane_infos.end()) {
          continue;
        }
        if (find_pre_lane->is_bev_topo_connected) {
          continue;
        }
        // AINFO << "  pre_id: " << *it << " size: " << find_pre_lane->line_points.size();
        std::vector<cem::message::common::Point2DF> control_points = {};

        int bot_index = -1;

        if (InsertBezierPoints(find_pre_lane->line_points, lane.line_points, control_points, bot_index, top_lane_index)) {
          // AINFO << "  InsertBezierPoints: bot_index: " << bot_index << " top_lane_index: " << top_lane_index;
          if (bot_index >= 0 && top_lane_index >= 0 && control_points.size() > 0) {
            for (auto &pt : control_points) {
              pt.mse          = NAN;
              pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
            }
            find_pre_lane->line_points.erase(find_pre_lane->line_points.begin() + bot_index, find_pre_lane->line_points.end());
            find_pre_lane->line_points.insert(find_pre_lane->line_points.end(), control_points.begin(), control_points.end());
          }
          last_control_points = control_points;
        }

        auto find_bot_llanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_pre_lane->left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
        auto find_bot_rlanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_pre_lane->right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });

        if (find_bot_llanemarker != bev_map.lanemarkers.end() && find_top_llanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         bot_index      = -1;
          if (InsertBezierPoints(find_bot_llanemarker->line_points, find_top_llanemarker->line_points, control_points, bot_index,
                                 top_leftlanemarker_index)) {
            // AINFO << "   find_bot_llanemarker id: " << find_bot_llanemarker->id << " size: " << find_bot_llanemarker->line_points.size();
            // AINFO << "  InsertBezierPoints: bot_index: " << bot_index << " top_leftlanemarker_index: " << top_leftlanemarker_index;
            if (top_leftlanemarker_index >= 0 && bot_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_bot_llanemarker->line_points.erase(find_bot_llanemarker->line_points.begin() + bot_index,
                                                      find_bot_llanemarker->line_points.end());
              find_bot_llanemarker->line_points.insert(find_bot_llanemarker->line_points.end(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
        if (find_bot_rlanemarker != bev_map.lanemarkers.end() && find_top_rlanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         bot_index      = -1;
          if (InsertBezierPoints(find_bot_rlanemarker->line_points, find_top_rlanemarker->line_points, control_points, bot_index,
                                 top_rightlanemarker_index)) {
            // AINFO << "   find_bot_rlanemarker id: " << find_bot_rlanemarker->id << " size: " << find_bot_rlanemarker->line_points.size();
            // AINFO << "  InsertBezierPoints: bot_index: " << bot_index << " top_rightlanemarker_index: " << top_rightlanemarker_index;
            if (top_rightlanemarker_index >= 0 && bot_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_bot_rlanemarker->line_points.erase(find_bot_rlanemarker->line_points.begin() + bot_index,
                                                      find_bot_rlanemarker->line_points.end());
              find_bot_rlanemarker->line_points.insert(find_bot_rlanemarker->line_points.end(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
      }
      // AINFO << "top_lane_index :" << top_lane_index;
      if (lane.line_points.size() == 2 && top_lane_index > 0) {
        // 删除第一个点 P0
        lane.line_points.erase(lane.line_points.begin(), lane.line_points.begin() + 1);

        // 插入 control_points 的倒数第二个点
        if (!last_control_points.empty() && last_control_points.size() >= 2) {
          auto insert_point         = last_control_points[last_control_points.size() - 2];
          insert_point.mse          = NAN;
          insert_point.point_source = cem::message::common::PointSource::TOPO_INSETRT;
          lane.line_points.insert(lane.line_points.begin(), insert_point);
          // AINFO << "Inserted control point into Lane ID: " << lane.id << ", new size: " << lane.line_points.size();
        }
      } else if (top_lane_index > 0) {
        // 按照之间逻辑处理normal case
        lane.line_points.erase(lane.line_points.begin(), lane.line_points.begin() + top_lane_index);
      }
      // AINFO << "lane size: " << lane.line_points.size();
      if (top_leftlanemarker_index > 0) {
        find_top_llanemarker->line_points.erase(find_top_llanemarker->line_points.begin(),
                                                find_top_llanemarker->line_points.begin() + top_leftlanemarker_index);
      }
      if (top_rightlanemarker_index > 0) {
        find_top_rlanemarker->line_points.erase(find_top_rlanemarker->line_points.begin(),
                                                find_top_rlanemarker->line_points.begin() + top_rightlanemarker_index);
      }
    }
  }

  for (auto &lane : bev_map.lane_infos) {
    if (lane.is_bev_topo_connected) {
      continue;
    }

    std::set<uint64_t> next_id;
    for (auto &id : lane.next_lane_ids) {
      next_id.insert(id);
    }
    if (1 == next_id.size()) {
      // AINFO << "Processing 1to1 case...";
      int  bot_lane_index            = -1;
      int  bot_leftlanemarker_index  = -1;
      int  bot_rightlanemarker_index = -1;
      auto find_bot_llanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
      auto find_bot_rlanemarker      = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                                                    [find_id = lane.right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });

      auto it = next_id.begin();

      auto find_next_lane = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                                         [find_id = *it](const BevLaneInfo &p) { return p.id == find_id; });

      if (find_next_lane == bev_map.lane_infos.end()) {
        continue;
      }

      if (1 == find_next_lane->previous_lane_ids.size()) {
        std::vector<cem::message::common::Point2DF> control_points = {};

        int top_index = -1;

        if (InsertBezierPoints(lane.line_points, find_next_lane->line_points, control_points, bot_lane_index, top_index)) {
          if (bot_lane_index >= 0 && top_index >= 0 && control_points.size() > 0) {
            for (auto &pt : control_points) {
              pt.mse          = NAN;
              pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
            }
            find_next_lane->line_points.erase(find_next_lane->line_points.begin(), find_next_lane->line_points.begin() + top_index + 1);
            find_next_lane->line_points.insert(find_next_lane->line_points.begin(), control_points.begin(), control_points.end());
          }
        }

        auto find_top_llanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_next_lane->left_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });
        auto find_top_rlanemarker =
            std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                         [find_id = find_next_lane->right_lane_marker_id](const BevLaneMarker &p) { return p.id == find_id; });

        if (find_bot_llanemarker != bev_map.lanemarkers.end() && find_top_llanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         top_index      = -1;
          // AINFO<<"lanemarker insert success 1";
          if (InsertBezierPoints(find_bot_llanemarker->line_points, find_top_llanemarker->line_points, control_points,
                                 bot_leftlanemarker_index, top_index)) {
            if (bot_leftlanemarker_index > 0 && top_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_top_llanemarker->line_points.erase(find_top_llanemarker->line_points.begin(),
                                                      find_top_llanemarker->line_points.begin() + top_index + 1);
              find_top_llanemarker->line_points.insert(find_top_llanemarker->line_points.begin(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
        if (find_bot_rlanemarker != bev_map.lanemarkers.end() && find_top_rlanemarker != bev_map.lanemarkers.end()) {
          std::vector<cem::message::common::Point2DF> control_points = {};
          int                                         top_index      = -1;
          // AINFO<<"lanemarker insert success 2";
          if (InsertBezierPoints(find_bot_rlanemarker->line_points, find_top_rlanemarker->line_points, control_points,
                                 bot_rightlanemarker_index, top_index)) {
            if (bot_rightlanemarker_index >= 0 && top_index >= 0 && control_points.size() > 0) {
              for (auto &pt : control_points) {
                pt.mse          = NAN;
                pt.point_source = cem::message::common::PointSource::TOPO_INSETRT;
              }
              find_top_rlanemarker->line_points.erase(find_top_rlanemarker->line_points.begin(),
                                                      find_top_rlanemarker->line_points.begin() + top_index + 1);
              find_top_rlanemarker->line_points.insert(find_top_rlanemarker->line_points.begin(), control_points.begin(),
                                                       control_points.end());
            }
          }
        }
        // AINFO<<"success lane and lane marker,bot index is"<<bot_lane_index;
        if (bot_lane_index > 0) {
          lane.line_points.erase(lane.line_points.begin() + bot_lane_index + 1, lane.line_points.end());
        }
        if (bot_leftlanemarker_index > 0) {
          find_bot_llanemarker->line_points.erase(find_bot_llanemarker->line_points.begin() + bot_leftlanemarker_index + 1,
                                                  find_bot_llanemarker->line_points.end());
        }
        if (bot_rightlanemarker_index > 0) {
          find_bot_rlanemarker->line_points.erase(find_bot_rlanemarker->line_points.begin() + bot_rightlanemarker_index + 1,
                                                  find_bot_rlanemarker->line_points.end());
        }
      }
    }
  }
}

typedef std::shared_ptr<std::vector<Eigen::Vector2f>> Vec2fVector;
struct LineSort {
  LineSort(uint32_t ID, bool IsRoadBoundary) : id(ID), is_road_boundary(IsRoadBoundary) {}

  uint32_t id;
  bool     is_road_boundary;
};
void LaneGuidance::CaculateLanePosition(BevMapSectionInfo &section, std::vector<BevLaneMarker> &edges, Graph<uint64_t> &lane_graph) {
  if (section.lane_infos.size() <= 1) {
    return;
  }
  std::map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> geo_map1;
  std::map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> geo_map2;
  std::map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> geo_map;
  std::vector<LineSort>                                                       line_sorts1;
  std::vector<LineSort>                                                       line_sorts2;
  std::vector<LineSort>                                                       line_sorts;
  auto                                                                        T_ego_local = T_local_ego_.inverse();
  auto                                                                        lane_infos  = section.lane_infos;

  for (auto &lane : lane_infos) {
    Vec2fVector geo1 = std::make_shared<std::vector<Eigen::Vector2f>>();
    Vec2fVector geo2 = std::make_shared<std::vector<Eigen::Vector2f>>();
    Vec2fVector geo  = std::make_shared<std::vector<Eigen::Vector2f>>();
    // 遍历line_points 使用迭代器

    std::transform(lane.line_points.begin(), lane.line_points.end(), lane.line_points.begin(), [T_ego_local](auto &pt) {
      Eigen::Vector3d pt_vcs(pt.x, pt.y, 0);
      pt_vcs = T_ego_local * pt_vcs;
      cem::message::common::Point2DF ret;
      {
        ret.x            = pt_vcs[0];
        ret.y            = pt_vcs[1];
        ret.point_source = pt.point_source;
      };
      return ret;
    });
    lane.line_points.erase(std::remove_if(lane.line_points.begin(), lane.line_points.end(),
                                          [](const auto &pt) {
                                            return (pt.point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_FIT ||
                                                    pt.point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANE ||
                                                    pt.x < -30.0f);
                                          }),
                           lane.line_points.end());
    // 计算line_points 两点之间距离的积分
    if (lane.line_points.size() < 2) {  // 空点
      continue;
    };
    double total_distance = std::accumulate(
        lane.line_points.begin() + 1, lane.line_points.end(), 0.0,
        [previous_point = lane.line_points.front()](double total, const cem::message::common::Point2DF &current_point) mutable -> double {
          double dx       = current_point.x - previous_point.x;
          double dy       = current_point.y - previous_point.y;
          double distance = std::sqrt(dx * dx + dy * dy);
          previous_point  = current_point;
          return total + distance;
        });
    // 远端误检导流线短线
    Eigen::Vector3d start_point(lane.line_points.front().x, lane.line_points.front().y, 0);
    auto            skip_condition = (total_distance < 20.0 && start_point(0) > 30.0 && lane.previous_lane_ids.size() == 0) ? true : false;

    if (skip_condition) {
      continue;
    };

    // 分近端20m以内 和其他 两部分计算
    std::vector<double> geo_x_vec1, geo_y_vec1;
    std::vector<double> geo_x_vec2, geo_y_vec2;
    std::vector<double> geo_x_vec, geo_y_vec;

    for (auto &vcs_point : lane.line_points) {
      if (vcs_point.x < 50.0f) {
        geo_x_vec1.push_back(vcs_point.x);
        geo_y_vec1.push_back(vcs_point.y);
        geo1->push_back(Eigen::Vector2f(vcs_point.x, vcs_point.y));
      } else {
        geo_x_vec2.push_back(vcs_point.x);
        geo_y_vec2.push_back(vcs_point.y);
        geo2->push_back(Eigen::Vector2f(vcs_point.x, vcs_point.y));
      }
      geo_x_vec.push_back(vcs_point.x);
      geo_y_vec.push_back(vcs_point.y);
      geo->push_back(Eigen::Vector2f(vcs_point.x, vcs_point.y));
    }
    if (geo_x_vec1.size() > 2) {
      geo_map1.insert({lane.id, {geo1, LaneGeometry::PolynomialFitting(geo_x_vec1, geo_y_vec1, 3)}});
      line_sorts1.emplace_back(lane.id, false);
    }
    if (geo_x_vec2.size() > 3 || (geo_x_vec2.size() > 2 && !lane.previous_lane_ids.empty())) {
      geo_map2.insert({lane.id, {geo2, LaneGeometry::PolynomialFitting(geo_x_vec2, geo_y_vec2, 3)}});
      line_sorts2.emplace_back(lane.id, false);
    }
    if (geo_x_vec.size() > 3) {
      geo_map.insert({lane.id, {geo, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
      line_sorts.emplace_back(lane.id, false);
    }
  }

  // 近端排序
  if (line_sorts1.size() > 1) {
    std::sort(line_sorts1.begin(), line_sorts1.end(), [&](const auto &l1, const auto &l2) {
      LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
      LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
      Vec2fVector                      l1_geos               = nullptr;
      Vec2fVector                      l2_geos               = nullptr;

      polynomial_fitting_l1 = &geo_map1[l1.id].second;
      l1_geos               = geo_map1[l1.id].first;
      polynomial_fitting_l2 = &geo_map1[l2.id].second;
      l2_geos               = geo_map1[l2.id].first;
      if (l1_geos && l2_geos) {
        auto node_1st = lane_graph.GetNode(l1.id);
        auto node_2nd = lane_graph.GetNode(l2.id);
        if (node_1st && node_2nd && node_1st->predecessors.size() == 1 && node_2nd->predecessors.size() == 1 &&
            node_1st->predecessors.front() == node_2nd->predecessors.front() && l1_geos->size() > 1 && l2_geos->size() > 1) {
          Eigen::Vector3d vec_1st = Eigen::Vector3d(l1_geos->back().x() - l1_geos->front().x(), l1_geos->back().y() - l1_geos->front().y(), 0);
          Eigen::Vector3d vec_2nd = Eigen::Vector3d(l2_geos->back().x() - l2_geos->front().x(), l2_geos->back().y() - l2_geos->front().y(), 0);
          double cross_product = vec_1st.cross(vec_2nd).z();
          if (cross_product > 0) {
            return false;
          } else if (cross_product < 0) {
            return true;
          }
        }

        bool result = false;
        float sum_y1 = 0;
        float sum_y2 = 0;
        std::vector<Eigen::Vector2f> truncated_lane_points;
        if (is_highway_) {
          if (l1_geos->front().x() - l2_geos->front().x() > 25.0f && l2_geos->front().x() < -25) {
            std::copy_if(l2_geos->begin(), l2_geos->end(), std::back_inserter(truncated_lane_points),
              [](const Eigen::Vector2f& point) {
                  return point.x() > 0.0f;
              });
              if (truncated_lane_points.size() >= 2) {
                return IsLeftLane(*l1_geos, truncated_lane_points);
              }
          } else if (l2_geos->front().x() - l1_geos->front().x() > 25.0f && l1_geos->front().x() < -25) {
            std::copy_if(l1_geos->begin(), l1_geos->end(), std::back_inserter(truncated_lane_points),
              [](const Eigen::Vector2f& point) {
                  return point.x() > 0.0f;
              });
              if (truncated_lane_points.size() >= 2) {
                return IsLeftLane(truncated_lane_points, *l2_geos);
              }
          }
        }
        return IsLeftLane(*l1_geos, *l2_geos);
      } else {
        return false;
      }
    });
  } else {
    line_sorts1.clear();  // 只有一个元素排序无意义
    geo_map1.clear();
  }

  // 远端排序
  std::sort(line_sorts2.begin(), line_sorts2.end(), [&](const auto &l1, const auto &l2) {
    LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
    LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
    Vec2fVector                      l1_geos               = nullptr;
    Vec2fVector                      l2_geos               = nullptr;

    polynomial_fitting_l1 = &geo_map2[l1.id].second;
    l1_geos               = geo_map2[l1.id].first;
    polynomial_fitting_l2 = &geo_map2[l2.id].second;
    l2_geos               = geo_map2[l2.id].first;

    if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
      bool ret = LaneGeometry::JudgeIsLeft(*l1_geos, *l2_geos, *polynomial_fitting_l1, *polynomial_fitting_l2);
      return ret;
    } else {
      return false;
    }
  });

  // 近端赋值
  if (!line_sorts1.empty()) {
    for (auto iter = line_sorts1.begin() + 1; iter != line_sorts1.end(); iter++) {
      auto prevIt = std::prev(iter);
      auto nextIt = std::next(iter);
      if (nextIt != line_sorts1.end()) {
        auto lane_iter = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                                   [iter](const BevLaneInfo &lane) { return lane.id == iter->id; });
        if (lane_iter != section.lane_infos.end()) {
          lane_iter->left_lane_id  = prevIt->id;
          lane_iter->right_lane_id = nextIt->id;
          if (lane_graph.GetNode(lane_iter->id)) {
            lane_graph.AddLeftNode(lane_graph.GetNode(lane_iter->id), lane_iter->left_lane_id);
            lane_graph.AddRightNode(lane_graph.GetNode(lane_iter->id), lane_iter->right_lane_id);
          }
        }
      }
    }
    // 处理第一个
    auto lane_iter = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                               [&line_sorts1](const BevLaneInfo &lane) { return lane.id == line_sorts1.begin()->id; });
    if ((lane_iter != section.lane_infos.end()) && (line_sorts1.size() > 1)) {
      lane_iter->left_lane_id  = 0;
      lane_iter->right_lane_id = std::next(line_sorts1.begin())->id;
      if (lane_graph.GetNode(lane_iter->id)) {
        lane_graph.AddRightNode(lane_graph.GetNode(lane_iter->id), lane_iter->right_lane_id);
      }
    }
    lane_iter = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                          [&line_sorts1](const BevLaneInfo &lane) { return lane.id == (line_sorts1.end() - 1)->id; });
    if ((lane_iter != section.lane_infos.end()) && (line_sorts1.size() > 1)) {
      lane_iter->right_lane_id = 0;
      lane_iter->left_lane_id  = std::prev(line_sorts1.end() - 1)->id;
      if (lane_graph.GetNode(lane_iter->id)) {
        lane_graph.AddLeftNode(lane_graph.GetNode(lane_iter->id), lane_iter->left_lane_id);
      }
    }
  }

  // 远端赋值 只对相对于近端新增的
  if (!line_sorts2.empty()) {
    for (auto iter = line_sorts2.begin() + 1; iter != line_sorts2.end(); iter++) {
      auto prev_iter = std::prev(iter);
      auto next_iter = std::next(iter);
      if (next_iter != line_sorts2.end()) {
        auto lane_iter = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                                   [iter](const BevLaneInfo &lane) { return lane.id == iter->id; });
        if (lane_iter != section.lane_infos.end() && geo_map1.find(lane_iter->id) == geo_map1.end()) {
          lane_iter->left_lane_id  = prev_iter->id;
          lane_iter->right_lane_id = next_iter->id;
          if (lane_graph.GetNode(lane_iter->id)) {
            lane_graph.AddLeftNode(lane_graph.GetNode(lane_iter->id), lane_iter->left_lane_id);
            lane_graph.AddRightNode(lane_graph.GetNode(lane_iter->id), lane_iter->right_lane_id);
          }
        }
      }
    }
    // 处理第一个
    auto laneIt = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                               [&line_sorts2](const BevLaneInfo &lane) { return lane.id == line_sorts2.begin()->id; });
    if (laneIt != section.lane_infos.end() && geo_map1.find(laneIt->id) == geo_map1.end()) {
      laneIt->left_lane_id = 0;
      if (line_sorts2.size() > 1) {
        laneIt->right_lane_id = std::next(line_sorts2.begin())->id;
        if (lane_graph.GetNode(laneIt->id)) {
          lane_graph.AddRightNode(lane_graph.GetNode(laneIt->id), laneIt->right_lane_id);
        }
      }

      auto laneIt1 = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                                  [&laneIt](const BevLaneInfo &lane) { return lane.id == laneIt->right_lane_id; });
      if (laneIt1 != section.lane_infos.end() && laneIt1->left_lane_id == 0) {
        laneIt1->left_lane_id = laneIt->id;
        if (lane_graph.GetNode(laneIt1->id)) {
          lane_graph.AddLeftNode(lane_graph.GetNode(laneIt1->id), laneIt1->left_lane_id);
        }
      }
    }
    // 和最后一个
    laneIt = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                          [&line_sorts2](const BevLaneInfo &lane) { return lane.id == (line_sorts2.end() - 1)->id; });
    if (laneIt != section.lane_infos.end() && geo_map1.find(laneIt->id) == geo_map1.end()) {
      laneIt->right_lane_id = 0;
      if (line_sorts2.size() > 1) {
        laneIt->left_lane_id = std::prev(line_sorts2.end() - 1)->id;
        if (lane_graph.GetNode(laneIt->id)) {
          lane_graph.AddLeftNode(lane_graph.GetNode(laneIt->id), laneIt->left_lane_id);
        }
      };
      auto laneIt1 = std::find_if(section.lane_infos.begin(), section.lane_infos.end(),
                                  [&laneIt](const BevLaneInfo &lane) { return lane.id == laneIt->left_lane_id; });
      if (laneIt1 != section.lane_infos.end() && laneIt1->right_lane_id == 0) {
        laneIt1->right_lane_id = laneIt->id;
        if (lane_graph.GetNode(laneIt->id)) {
          lane_graph.AddRightNode(lane_graph.GetNode(laneIt->id), laneIt->right_lane_id);
        }
      }
    }
  }

  // 整体排序
  std::sort(section.lane_infos.begin(), section.lane_infos.end(), [&](const auto &l1, const auto &l2) {
    if (geo_map.count(l1.id) == 0 || geo_map.count(l2.id) == 0) {
      return false;
    }
    LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
    LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
    Vec2fVector                      l1_geos               = nullptr;
    Vec2fVector                      l2_geos               = nullptr;
    polynomial_fitting_l1                                  = &geo_map[l1.id].second;
    l1_geos                                                = geo_map[l1.id].first;
    polynomial_fitting_l2                                  = &geo_map[l2.id].second;
    l2_geos                                                = geo_map[l2.id].first;

    if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
      bool ret = LaneGeometry::JudgeIsLeft(*l1_geos, *l2_geos, *polynomial_fitting_l1, *polynomial_fitting_l2);
      return ret;
    } else {
      return false;
    }
  });

  return;
}

bool LaneGuidance::IsLeftLane(std::vector<Eigen::Vector2f>& l1_geos, std::vector<Eigen::Vector2f>& l2_geos) {
    bool result = LaneGeometry::JudgeIsLeftUseRawPtFilter(l1_geos, l2_geos);
    if (-1 == result) {  // 可以比较的点全部在距离范围内 l1_geos  l2_geos比较y坐标
      double sum_y1 =
          std::accumulate(l1_geos.begin(), l1_geos.end(), 0.0, [](double a, const Eigen::Vector2f &b) { return a + b.y(); });
      double sum_y2 =
          std::accumulate(l2_geos.begin(), l2_geos.end(), 0.0, [](double a, const Eigen::Vector2f &b) { return a + b.y(); });
      if (!l1_geos.empty() && !l2_geos.empty()) {
        sum_y1 /= l1_geos.size();
        sum_y2 /= l2_geos.size();
      }
      return sum_y1 > sum_y2;
    } else if (1 == result) {
      return true;
    } else {
      return false;
    }
}

void LaneGuidance::ResetLanePosition(BevMapSectionInfo &section, std::vector<BevLaneMarker> &edges) {
  // left_lane_id  right_lane_id 清零
  for (auto &lane : section.lane_infos) {
    lane.left_lane_id  = 0;
    lane.right_lane_id = 0;
  }
  return;
}

void LaneGuidance::RemoveAbnormalLane(std::vector<BevMapSectionInfo>& sections) {
  for (auto &section : sections) {
    for (std::vector<BevLaneInfo>::iterator it = section.lane_infos.begin(); it != section.lane_infos.end();) {
      // 先移除每个lane中的无效点，再判断是否删除该车道
      for (std::vector<cem::message::common::Point2DF>::iterator it_pt = it->line_points.begin(); it_pt != it->line_points.end();) {
        if (std::isnan(it_pt->x) || std::isnan(it_pt->y)) {
          it_pt = it->line_points.erase(it_pt);
        } else {
          ++it_pt;
        }
      }

      if (it->line_points.size() < 2 && !it->is_compensation) {
        it = section.lane_infos.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void LaneGuidance::ReverseBackwardLane(std::vector<BevMapSectionInfo>& sections) {
  for (auto &section : sections) {
    for (auto &lane : section.lane_infos) {
      if(lane.road_id == 2) {
        std::reverse(lane.line_points.begin(), lane.line_points.end());
      }
    }
  }
}

void LaneGuidance::LaneAssociationWithJunction(BevMapInfo &bev_map) {
  for (auto &section : bev_map.route.sections) {
    for (auto &lane : section.lane_infos) {
      if (!lane.is_virtual) {
        continue;
      }
      for (const auto &junction : bev_map.junctions) {
        lane.junction_id = junction.id;
        break;
      }
      if (lane.junction_id == 0) {
        lane.junction_id = kVirtualJunctionID;
      }
    }
  }
}

void LaneGuidance::InitRecommendedLaneAction(BevMapInfo &bev_map, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                             const RoutingMapPtr routing_map_ptr, const SdJunctionInfoCityPtr junctions_ptr) {
  // if (guide_lane_result.empty()) {
  //   return;
  // }
  // 设置该变量存储之前推荐车道的导航动作，默认为未知
  recommend_action_ = BevAction::UNKNOWN;

  for (auto &lane : bev_map.lane_infos) {
    auto id_iter = std::find_if(guide_lane_result.begin(), guide_lane_result.end(),
                                [find_id = lane.id](std::pair<uint64_t, std::pair<int, int>> result) { return result.first == find_id; });

    if (id_iter == guide_lane_result.end()) {
      continue;
    }
    // 推荐车道导航动作不为空，否则默认为直行
    if (lane.navi_action != BevAction::UNKNOWN) {
      recommend_action_ = lane.navi_action;
      // AINFO << "*******RecommendActionOrigin******"; 
      // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
      // AINFO << "lane ID: " << lane.id;
      // AINFO << "lane navi_action: " << static_cast<uint32_t>(lane.navi_action); 
      // AINFO << "****************************";
    }
    break;
  }

  // 如果处于右转专用道，直接将推荐动作设置为turn right
  // AINFO << "*******RightTurnOnlyBefore******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // AINFO << "route ID: " << route_id_;
  // AINFO << "is right turn only lane: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_; 
  // AINFO << "****************************";

  // AINFO << "*******IsRightTurnOnly******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // AINFO << "sequence number: " << bev_map.header.cycle_counter;
  // AINFO << "current recommend action: " << static_cast<int>(recommend_action_);
  // AINFO << "is right turn only before: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_;
  if (recommend_action_ == BevAction::UNKNOWN && routing_map_ptr && junctions_ptr){
    IsRightTurnOnly(junctions_ptr, routing_map_ptr);
    if (is_right_turn_only_){
      recommend_action_ = BevAction::RIGHT_TURN;
    }
  }
  // AINFO << "is right turn only after: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_;
  // AINFO << "****************************";

  // AINFO << "*******RightTurnOnlyAfter******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // AINFO << "route ID: " << route_id_;
  // AINFO << "is right turn only lane: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_; 
  // AINFO << "****************************";


  if (recommend_action_ == BevAction::UNKNOWN && routing_map_ptr) {
    // 如果没有推荐车道导航动作，则从图中获取推荐车道的导航动作
    uint64_t current_sd_section_id = routing_map_ptr->sd_route.navi_start.section_id;
    double current_sd_navi_offset =  routing_map_ptr->sd_route.navi_start.s_offset; 
    // find current section in sd_route
    auto sd_section = std::find_if(routing_map_ptr->sd_route.mpp_sections.begin(), routing_map_ptr->sd_route.mpp_sections.end(),
                                   [find_id = current_sd_section_id](SDSectionInfo &sd_section) { return sd_section.id == find_id; });

    uint64_t target_lane_group_id = 0;
    if (sd_section != routing_map_ptr->sd_route.mpp_sections.end() && !sd_section->lane_group_idx.empty()) {
      // 根据navi_start的s_offset确认lane_group的ID，找到正确的lane_group
      for (const auto &sd_lane_group : sd_section->lane_group_idx){
        if (current_sd_navi_offset >= sd_lane_group.start_range_offset && current_sd_navi_offset < sd_lane_group.end_range_offset){
          target_lane_group_id = sd_lane_group.id; 
          // AINFO << "*******RecommendAction******"; 
          // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
          // AINFO << "sd lane ID: " << sd_lane_group.id; 
          // AINFO << "sd lane start: " << sd_lane_group.start_range_offset; 
          // AINFO << "sd lane end: " << sd_lane_group.end_range_offset; 
          // AINFO << "****************************"; 
          break; 
        }
      }

      auto target_lane_group = std::find_if(
          routing_map_ptr->sd_lane_groups.begin(), routing_map_ptr->sd_lane_groups.end(),
          [find_id = target_lane_group_id](SDLaneGroupInfo &sd_lane_group) { return sd_lane_group.id == find_id; });
      if (target_lane_group != routing_map_ptr->sd_lane_groups.end() && !target_lane_group->lane_info.empty()) {
        // AINFO << "*******RecommendActionSD******"; 
        // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
        // AINFO << "sd section id from navi_start: " << current_sd_section_id;
        // AINFO << "section ID: " << sd_section->id;
        // AINFO << "lane_group_idx target ID: " << target_lane_group->id;
        // for (const auto &lane : target_lane_group->lane_info){
        //   AINFO << "lane ID: " << lane.id;
        //   AINFO << "lane turn_type: " << static_cast<uint32_t>(lane.turn_type); 
        // }
        // AINFO << "****************************";
        switch (target_lane_group->lane_info.back().turn_type) {
          case TurnType::NO_TURN:
            recommend_action_ = BevAction::STRAIGHT;
            break;
          case TurnType::LEFT_TURN:
            recommend_action_ = BevAction::LEFT_TURN;
            break;
          case TurnType::RIGHT_TURN:
            recommend_action_ = BevAction::RIGHT_TURN;
            break;
          default:
            break;
        }
      }
    }
  }
  // AINFO << "*******RecommendAction******"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // AINFO << "recommend_action_: " << static_cast<uint32_t>(recommend_action_); 
  // AINFO << "****************************"; 

  if (recommend_action_ == BevAction::UNKNOWN) {
    // 如果还是没有推荐车道导航动作，则默认为直行
    recommend_action_ = BevAction::STRAIGHT;
  }

  // 设置自车车道导航动作，默认为推荐车道动作
  for (auto &lane : bev_map.lane_infos) {
    if (lane.position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
      continue;
    }

    // 推荐车道导航动作不为空，否则默认为直行
    if (lane.navi_action == BevAction::UNKNOWN) {
      lane.navi_action = recommend_action_;
    }
  }

  for (auto &lane : bev_map.lane_infos) {
    if (!lane.is_virtual || lane.id > kMaxBevTrackID) {
      continue;
    }
    if (lane.navi_action == BevAction::UNKNOWN) {
      lane.navi_action = recommend_action_;
    }
  }
}

void LaneGuidance::InitLaneTurnType(BevMapInfo &bev_map, const cem::fusion::SdJunctionInfoCityPtr junctions_ptr) {
  constexpr double TurnTypeSetThreshold = 10.0;  // 设置turn_type的阈值
  if (junctions_ptr == nullptr) {
    return;
  }

  auto ego_lane_action_type = BevAction::STRAIGHT;
  double min_offset_junction = std::numeric_limits<double>::max();
  cem::fusion::navigation::JunctionInfoCity target_junction; 
  auto junctions = *junctions_ptr; 
  int target_index = junctions.size(); 

  for (int i=0; i<junctions.size(); i++){
    // 过滤掉以及行驶过去的路口信息
    if (junctions[i].junction_state_city == cem::fusion::navigation::JunctionStateCity::PASSED){
      continue;
    }

    // 找到当前最近的路口offset
    if (junctions[i].offset < min_offset_junction){
      target_junction = junctions[i];
      target_index = i; 
      min_offset_junction = junctions[i].offset;
    }
  }

  // 如果小于一定的阈值，将会对于ego_lane的turn_type做出对应的导航修改
  if (min_offset_junction > TurnTypeSetThreshold){
    for (auto &lane : bev_map.lane_infos){
      if (lane.position != 0 || lane.bev_turn_type != BevAction::UNKNOWN){ // not BEVLanePositon::LANE_LOC_EGO
        continue;
      }
      lane.bev_turn_type = ego_lane_action_type;
    }
    return;
  }

  // 临时策略：假设junction_state_city==0（passing）路口距离下一个路口offset大于当前路口(绝对值)，将junctions的结果为passed，替换target_junction，用下一个路口导航信息填充turn_type打灯
  if (target_junction.junction_state_city == cem::fusion::navigation::JunctionStateCity::PASSING && 
      (target_junction.junction_type_city == cem::fusion::navigation::JunctionTypeCity::TJunction ||
      target_junction.junction_type_city == cem::fusion::navigation::JunctionTypeCity::SmallTJunction) &&
      target_index < junctions.size() - 1){

    double offset_prev = target_junction.offset; 
    double offset_next = junctions[target_index+1].offset;

    if (abs(offset_prev) > abs(offset_next) && 
        junctions[target_index+1].junction_state_city == cem::fusion::navigation::JunctionStateCity::UNREACHED){
      target_junction = junctions[target_index+1];

      // 更新target_junction需要再进行一次判定是否小于Threshold
      if (target_junction.offset > TurnTypeSetThreshold){
        for (auto &lane : bev_map.lane_infos){
          if (lane.position != 0 || lane.bev_turn_type != BevAction::UNKNOWN){ // not BEVLanePositon::LANE_LOC_EGO
            continue;
          }
          lane.bev_turn_type = ego_lane_action_type;
        }
        return;
      }
    }
  }

  // 对于ego_lane的turn_type进行赋值
  switch (target_junction.junction_action) {
    case cem::fusion::navigation::JunctionAction::TurnLeft:
      ego_lane_action_type = BevAction::LEFT_TURN;
      break;
    case cem::fusion::navigation::JunctionAction::TurnRight:
      ego_lane_action_type = BevAction::RIGHT_TURN;
      break;
    default:
      break;
  }

  for (auto &lane : bev_map.lane_infos){
    if (lane.position != 0 || lane.bev_turn_type != BevAction::UNKNOWN){ // not BEVLanePositon::LANE_LOC_EGO
      continue;
    }
    lane.bev_turn_type = ego_lane_action_type;
  }
  // AINFO << "*******JunctionInfo******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  // AINFO << "junction ID: " << target_junction.junction_id;
  // AINFO << "junction state: " << static_cast<int>(target_junction.junction_state_city);
  // AINFO << "junction offset: " << target_junction.offset;
  // AINFO << "junction action: " << static_cast<int>(target_junction.junction_action);
  // AINFO << "ego lane action: " << static_cast<int>(ego_lane_action_type);
  // AINFO << "*************************";
}

void LaneGuidance::RoadSpan(const std::vector<BevLaneInfo> &lane_infos, std::vector<std::pair<uint64_t, uint64_t>> &roads_span) {
  if (lane_infos.empty()) {
    return;
  }
  std::pair<uint64_t, uint64_t> road_span;
  if (lane_infos.size() == 1) {
    road_span.first  = 0;
    road_span.second = 1;
    roads_span.push_back(road_span);
    return;
  }

  uint64_t current_value = lane_infos[0].road_id;
  road_span.first = 0;
  for (size_t i = 1; i <= lane_infos.size(); i++) {
    if (i == lane_infos.size() || lane_infos[i].road_id != current_value) {
      road_span.second = i;
      roads_span.push_back(road_span);
      if (i < lane_infos.size()) {
          current_value = lane_infos[i].road_id;
          road_span.first = i;
      }
    }
  }
}

void LaneGuidance::GetMarkerPositionEgoFrame(BevMapInfo &bev_map) {
  auto T_ego_local = T_local_ego_.inverse();
  // AINFO << "*********MarkerEgoFrame*********"; 
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << bev_map.header.timestamp;
  for (auto &junction : bev_map.junctions){
    // AINFO << "Junction ID: " << junction.id;
    for (auto point : junction.line_points){
      Eigen::Vector3d point_dr(point.x, point.y, 0); 
      Eigen::Vector3d point_ego_3d = T_ego_local * point_dr; 
      cem::message::common::Point2DF point_ego;
      point_ego.x = point_ego_3d.x();
      point_ego.y = point_ego_3d.y();
      junction.line_points_ego.push_back(point_ego);
      // AINFO << "Junction Point: (" << point_ego.x << ", " << point_ego.y << ")";
    }
  }
  for (auto &stop_line: bev_map.stop_lines){
    // AINFO << "Stop Line ID: " << stop_line.id;
    for (auto point : stop_line.line_points){
      Eigen::Vector3d point_dr(point.x, point.y, 0); 
      Eigen::Vector3d point_ego_3d = T_ego_local * point_dr; 
      cem::message::common::Point2DF point_ego;
      point_ego.x = point_ego_3d.x();
      point_ego.y = point_ego_3d.y();
      stop_line.line_points_ego.push_back(point_ego);
      // AINFO << "Stop line Point: (" << point_ego.x << ", " << point_ego.y << ")";
    }
  }
  for (auto &crosswalk : bev_map.crosswalks){
    // AINFO << "Crosswalk ID: " << crosswalk.id;
    for (auto point : crosswalk.line_points){
      Eigen::Vector3d point_dr(point.x, point.y, 0); 
      Eigen::Vector3d point_ego_3d = T_ego_local * point_dr; 
      cem::message::common::Point2DF point_ego;
      point_ego.x = point_ego_3d.x();
      point_ego.y = point_ego_3d.y();
      crosswalk.line_points_ego.push_back(point_ego);
      // AINFO << "Crosswalk Point: (" << point_ego.x << ", " << point_ego.y << ")";
    }
  }
  // AINFO << "*******************************"; 
}

void LaneGuidance::DebugLog() {
  AINFO << "bev_map.bev_map_ptr->header.timestamp:  " << std::setprecision(20) << last_map_info_ptr_->header.timestamp;
  for (const auto &lane : last_map_info_ptr_->lane_infos) {
    AINFO << "lane.id:  " << lane.id;
    AINFO << "lane.navi_action:   " << (int)lane.navi_action;
  }
}

void LaneGuidance::IsRightTurnOnly(const SdJunctionInfoCityPtr &junctions_ptr, const RoutingMapPtr &routing_map_raw) {
  if (junctions_ptr == nullptr || routing_map_raw == nullptr) {
    return;
  }

  // Initialization upon switching navigation state.
  if (routing_map_raw->sd_route.id != route_id_) {
    route_id_ = routing_map_raw->sd_route.id;
    is_right_turn_only_ = false;
    next_junction_id_ = 0;
    return;
  }

  // // check whether is passing junction
  // bool is_passing_junction = false;
  // for (const auto &junction : *junctions_ptr) {
  //   if (junction.junction_state_city == navigation::JunctionStateCity::PASSING) {
  //     AINFO << "Passing junction ID: " << junction.junction_id;
  //     is_passing_junction = true;
  //     break;
  //   }
  // }
  // AINFO << "is passing junction: " << is_passing_junction;

  // Judgment before entering a right turn dedicated lane scenario.
  bool first_unreached = false;
  for (size_t i = 0; i < junctions_ptr->size(); i++) {
    // AINFO << junctions_ptr->at(i).junction_id << "  is_dedicated_right_turn_lane:  " << junctions_ptr->at(i).is_dedicated_right_turn_lane
    //       << "  junction_state:  " << (int)junctions_ptr->at(i).junction_state_city
    //       << "  junction_type:   " << (int)junctions_ptr->at(i).junction_type_city
    //       << "  junction_action:   " << (int)junctions_ptr->at(i).junction_action 
    //       << "  junction offset:   " << junctions_ptr->at(i).offset;
    if (is_right_turn_only_) {
      break;
    }

    if (junctions_ptr->at(i).junction_state_city == navigation::JunctionStateCity::UNREACHED) {
      first_unreached = true;
    }

    if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
      if (junctions_ptr->at(i).is_dedicated_right_turn_lane && !is_right_turn_only_ &&
          junctions_ptr->at(i).offset < 0 && junctions_ptr->at(i+1).offset > 0 &&
          junctions_ptr->at(i).junction_action == navigation::JunctionAction::TurnRight) {
        is_right_turn_only_ = true;
        next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
      }
    }

    // if (junctions_ptr->at(i).is_dedicated_right_turn_lane && first_unreached && !is_right_turn_only_ &&
    //     junctions_ptr->at(i).junction_type_city == navigation::JunctionTypeCity::RoadSplit && !is_passing_junction &&
    //     junctions_ptr->at(i).junction_action == navigation::JunctionAction::TurnRight) {
    //   is_right_turn_only_ = true;

    //   if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
    //     next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
    //   }

    //   break;
    // }

    if (first_unreached) {
      break;
    }
  }

  // Judgment of exiting a right turn dedicated lane scenario
  for (size_t i = 0; i < junctions_ptr->size(); i++) {
    auto junction = junctions_ptr->at(i);
    if (!is_right_turn_only_) {
      break;
    }
    if (junction.junction_id == next_junction_id_ && junction.offset < 0 && junction.junction_type_city == navigation::JunctionTypeCity::RoadMerge) {
      is_right_turn_only_ = false;
      break;
    }
    else if (junction.junction_id == next_junction_id_ && junction.offset < 0){
      if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
        next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
      }
      break;
    }
  }
}

}  // namespace fusion
}  // namespace cem
