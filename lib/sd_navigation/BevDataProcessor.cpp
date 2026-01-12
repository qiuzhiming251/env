#include "BevDataProcessor.h"
#include "lib/sd_navigation/routing_map_debug.h"

namespace cem {
namespace fusion {
namespace navigation {

BevDataProcessor::BevDataProcessor() {
  geometry_cache_.clear();
  history_separators_.clear();
  last_confirmed_section_info_.sections.clear();
  last_confirmed_section_info_.separators.clear();
}

void BevDataProcessor::Clear() {
  geometry_cache_.clear();
}

void BevDataProcessor::Process(const std::vector<JunctionInfoCity> &junctions_info_city, BevMapInfoConstPtr &bev_map,
                               std::vector<BevLaneInfo *> &bev_candidate_lanes, std::vector<BevLaneMarker *> &bev_candidate_road_edges,
                               std::vector<LineSortCity> &bev_line_sorts, std::vector<int> &bev_left_road_edge_indexs,
                               std::vector<int> &bev_right_road_edge_indexs, BevRouteInfo &complete_section_info,
                               std::pair<int, int> &bev_egoRoadEdgeIndexPair, std::set<uint64_t> &bev_ego_lane_related,
                               std::vector<uint64_t> &bev_ego_root_section_x0) {
  if (!bev_map) {
    SD_BEV_PROCESS << fmt::format("bev_map is null, exiting Process.");
    return;
  }
  SD_BEV_PROCESS << fmt::format("raw_bev_map: {}", bev_map->header.cycle_counter);
  Clear();

  passedZeroLaneNum_ = CountLanesPassedZero();
  // SD_BEV_PROCESS << fmt::format("[passedZeroLaneNum_] {}", passedZeroLaneNum_);

  // 筛选视觉范围内、未通过RoadSplit 路口
  std::vector<JunctionInfoCity> nearby_RoadSplits;
  std::vector<JunctionInfoCity> nearby_RoadMerges;
  for (const auto &junction : junctions_info_city) {
    if (junction.offset <= 200.0 &&
        (junction.junction_state_city == JunctionStateCity::UNREACHED || junction.junction_state_city == JunctionStateCity::PASSING)) {
      if (junction.junction_type_city == JunctionTypeCity::RoadSplit) {
        nearby_RoadSplits.push_back(junction);
      } else if (junction.junction_type_city == JunctionTypeCity::RoadMerge) {
        nearby_RoadMerges.push_back(junction);
      }
    }
  }

  const auto &all_bev_lane_infos = bev_map->lane_infos;
  auto       &all_bev_road_edges = bev_map->edges;
  auto       &bev_junctions      = bev_map->junctions;
  auto       &diversion_zones    = bev_map->diversion_zone;
  auto       &crosswalks         = bev_map->crosswalks;

  // Step 1: GetCandidateLanes
  SD_BEV_PROCESS << fmt::format("Before GetCandidateLanes: all_bev_lane_infos size: {}", all_bev_lane_infos.size());
  for (const auto &lane : all_bev_lane_infos) {
    SD_BEV_PROCESS << fmt::format("{}", lane);
  }
  GetCandidateLanes(all_bev_lane_infos, bev_junctions, bev_candidate_lanes);
  SD_BEV_PROCESS << fmt::format("After GetCandidateLanes: bev_candidate_lanes size: {}", bev_candidate_lanes.size());
  if (bev_candidate_lanes.empty()) {
    SD_BEV_PROCESS << "No candidate lanes found, skipping further processing.";
    return;
  }
  for (const auto &lane : bev_candidate_lanes) {
    SD_BEV_PROCESS << fmt::format("  Candidate Lane ID: {}", lane->id);
  }

  // Step 2: GetCandidateRoadEdges
  SD_BEV_PROCESS << fmt::format("Before GetCandidateRoadEdges: all_bev_road_edges size: {}", all_bev_road_edges.size());
  for (const auto &edge : all_bev_road_edges) {
    SD_BEV_PROCESS << fmt::format("{}", edge);
  }
  GetCandidateRoadEdges(all_bev_road_edges, bev_candidate_lanes, bev_junctions, bev_candidate_road_edges);
  SD_BEV_PROCESS << fmt::format("After GetCandidateRoadEdges: bev_candidate_road_edges size: {}", bev_candidate_road_edges.size());
  for (const auto &edge : bev_candidate_road_edges) {
    SD_BEV_PROCESS << fmt::format("  Candidate Edge ID: {}", edge->id);
  }

  // Step 3: 纵向聚类
  auto clusters = ClusterLanesByX(bev_candidate_lanes, bev_candidate_road_edges);
  SD_BEV_PROCESS << fmt::format("Clustered lanes into {} groups", clusters.size());
  for (size_t i = 0; i < clusters.size(); ++i) {
    std::vector<uint64_t> ids;
    for (const auto *lane : clusters[i].lanes) {
      ids.push_back(lane->id);
    }
    SD_BEV_PROCESS << fmt::format("Cluster {}: [{}]", i, fmt::join(ids, ", "));
  }

  // Step 4: 使用 crosswalk 验证聚类结果
  std::set<double> crosswalk_centers;
  for (const auto &crosswalk : crosswalks) {
    double center_x = CalculateCrosswalkCenterX(crosswalk);
    if (center_x != std::numeric_limits<double>::max()) {
      crosswalk_centers.insert(center_x);
    }
  }

  std::vector<ClusterInfo> validated_clusters;
  if (clusters.size() > 1) {
    ClusterInfo current_cluster = clusters[0];
    for (size_t i = 1; i < clusters.size(); ++i) {
      auto &cluster2              = clusters[i];
      bool  has_crosswalk_between = false;
      for (double center_x : crosswalk_centers) {
        if (center_x > current_cluster.max_x && center_x < cluster2.min_x) {
          has_crosswalk_between = true;
          break;
        }
      }

      if (has_crosswalk_between) {
        validated_clusters.push_back(current_cluster);
        current_cluster = cluster2;
      } else {
        // 合并 cluster
        current_cluster.lanes.insert(current_cluster.lanes.end(), cluster2.lanes.begin(), cluster2.lanes.end());
        current_cluster.road_edges.insert(current_cluster.road_edges.end(), cluster2.road_edges.begin(), cluster2.road_edges.end());
        current_cluster.min_x = std::min(current_cluster.min_x, cluster2.min_x);
        current_cluster.max_x = std::max(current_cluster.max_x, cluster2.max_x);
      }
    }
    validated_clusters.push_back(current_cluster);
  } else if (clusters.size() == 1) {
    validated_clusters = clusters;
  }

  SD_BEV_PROCESS << fmt::format("Validated clusters: {}", validated_clusters.size());
  for (size_t i = 0; i < validated_clusters.size(); ++i) {
    std::vector<uint64_t> ids;
    for (const auto *lane : validated_clusters[i].lanes) {
      ids.push_back(lane->id);
    }
    SD_BEV_PROCESS << fmt::format("Validated Cluster {}: [{}]", i, fmt::join(ids, ", "));
  }

  // Step 5: 识别自车所在的 cluster
  int ego_cluster_idx = -1;
  if (validated_clusters.size() == 1) {
    ego_cluster_idx = 0;
    SD_BEV_PROCESS << fmt::format("Only one cluster, using it as ego cluster.");
  } else {
    for (size_t i = 0; i < validated_clusters.size(); ++i) {
      const auto &cluster = validated_clusters[i];
      if (cluster.min_x <= 0 && 0 <= cluster.max_x) {
        ego_cluster_idx = i;
        SD_BEV_PROCESS << fmt::format("Ego cluster found: index={}, x-range: [{}, {}]", i, cluster.min_x, cluster.max_x);
        break;
      }
    }
  }

  // 检查 validated_clusters 是否为空
  if (validated_clusters.empty()) {
    SD_BEV_PROCESS << fmt::format("No valid clusters found, exiting Process.");
    return;
  }

  if (ego_cluster_idx == -1) {
    SD_BEV_PROCESS << fmt::format("No cluster covers x=0, defaulting to first cluster.");
    ego_cluster_idx = 0;
  }

  // Step 6: 处理自车所在的 cluster
  auto &cluster_lanes      = validated_clusters[ego_cluster_idx].lanes;
  auto &cluster_road_edges = validated_clusters[ego_cluster_idx].road_edges;
  SD_BEV_PROCESS << fmt::format("Processing ego cluster: lanes size={}, road_edges size={}", cluster_lanes.size(),
                                cluster_road_edges.size());

  // Step 7: GetEgoLaneRelated
  bev_ego_lane_related_.clear();
  bev_ego_lane_related_ = GetEgoLaneRelated(bev_map->lane_infos);
  bev_ego_lane_related  = bev_ego_lane_related_;
  SD_BEV_PROCESS << fmt::format("After GetEgoLaneRelated: bev_ego_lane_related IDs: [{}]", fmt::join(bev_ego_lane_related, ", "));

  //  Step 8:排序车道线和道路边缘
  std::vector<LineSortCity> cluster_line_sorts;
  std::vector<int>          cluster_left_road_edge_indexs, cluster_right_road_edge_indexs;
  SortBevLaneInfoAndRoadEdges(cluster_lanes, cluster_road_edges, cluster_line_sorts, cluster_left_road_edge_indexs,
                              cluster_right_road_edge_indexs);
  SD_BEV_PROCESS << fmt::format("After SortBevLaneInfoAndRoadEdges: cluster_line_sorts: ");
  for (const auto &sort_tmp : cluster_line_sorts) {
    SD_BEV_PROCESS << fmt::format("  [{}, {}]", sort_tmp.id, static_cast<int>(sort_tmp.type));
  }
  SD_BEV_PROCESS << fmt::format("cluster_left_road_edge_indexs: [{}]", fmt::join(cluster_left_road_edge_indexs, ", "));
  SD_BEV_PROCESS << fmt::format("cluster_right_road_edge_indexs: [{}]", fmt::join(cluster_right_road_edge_indexs, ", "));

  // Step 9: 过滤车道线
  bool filtered_non_ego_lanes = false;
  BevLanesFilterByRoadEdges(cluster_line_sorts, cluster_left_road_edge_indexs, cluster_right_road_edge_indexs, cluster_lanes,
                            filtered_non_ego_lanes);
  complete_section_info.filtered_non_ego_lanes = filtered_non_ego_lanes;
  SD_BEV_PROCESS << fmt::format("After BevLanesFilterByRoadEdges: lanes size: {}", cluster_lanes.size());

  // Step 10: 划分 sections
  DivideBevSectionsByRoadEdges(cluster_lanes, cluster_line_sorts, complete_section_info);
  SD_BEV_PROCESS << fmt::format("After DivideBevSectionsByRoadEdges: sections size: {}", complete_section_info.sections.size());

  // 更新输出结果
  // bev_sections               = std::move(cluster_sections);
  bev_line_sorts             = std::move(cluster_line_sorts);
  bev_left_road_edge_indexs  = std::move(cluster_left_road_edge_indexs);
  bev_right_road_edge_indexs = std::move(cluster_right_road_edge_indexs);
  bev_candidate_lanes        = cluster_lanes;

  SD_BEV_PROCESS << fmt::format("After clustering and division: sections size: {}", complete_section_info.sections.size());
  for (size_t i = 0; i < complete_section_info.sections.size(); ++i) {
    SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(complete_section_info.sections[i].lane_ids, ", "));
  }

  // Step 11: GetEgoLeftRightRoadEdges
  bev_egoRoadEdgeIndexPair = GetEgoLeftRightRoadEdges(bev_line_sorts, bev_left_road_edge_indexs, bev_right_road_edge_indexs);
  SD_BEV_PROCESS << fmt::format("After GetEgoLeftRightRoadEdges: Left_EgoEdge_Index: {}, Right_EgoEdge_Index: {}",
                                bev_egoRoadEdgeIndexPair.first, bev_egoRoadEdgeIndexPair.second);

  // Backup original bev_line_sorts before modifying it
  std::vector<LineSortCity> original_bev_line_sorts = bev_line_sorts;

  // Step 12: Further divide sections using diversion zones
  std::vector<BevLaneMarker> filtered_virtual_edges;
  if (!diversion_zones.empty()) {
    SD_BEV_PROCESS << fmt::format("Processing diversion zones: size = {}", diversion_zones.size());
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
    auto virtual_road_edges = CreateVirtualRoadEdgesFromDiversionZones(diversion_zones);
    for (const auto &virtual_edge : virtual_road_edges) {
      SD_BEV_PROCESS << fmt::format("virtual_edge: {}", virtual_edge);
    }

    std::vector<LineSortCity> temp_line_sorts = bev_line_sorts;
    SortLanesRelativeToVirtualEdges(bev_candidate_lanes, virtual_road_edges, temp_line_sorts);

    // Filter virtual edges that have x-overlap with lanes and is_merge_zone is false

    for (const auto &virtual_edge : virtual_road_edges) {
      if (!virtual_edge.geos || virtual_edge.geos->empty()) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} has invalid or empty geos, skipping.", virtual_edge.id);
        continue;
      }
      // 过滤距离自车大于 60 米或者自车后方的虚拟路岩
      if (virtual_edge.geos->front().x() > 60.0) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} filtered: closest x > 60m).", virtual_edge.id);
        continue;
      }
      if (virtual_edge.geos->back().x() < 0.0) {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} filtered: end x < 0m).", virtual_edge.id);
        continue;
      }

      if (HasXOverlap(virtual_edge, bev_candidate_lanes)) {
        bool is_merge_zone = IsMergeZoneByJunction(virtual_edge, nearby_RoadMerges) ||
                             IsMergeZoneByLaneSpacing(virtual_edge, temp_line_sorts, bev_candidate_lanes);
        bool is_split_zone = IsSplitZoneByJunction(virtual_edge, nearby_RoadSplits);
        if (!is_merge_zone || is_split_zone) {
          filtered_virtual_edges.push_back(virtual_edge);
          SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} identified as split or undetermined, included for section division.",
                                        virtual_edge.id);
        } else {
          SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} identified as merge zone, skipping section division.", virtual_edge.id);
        }
      } else {
        SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} has no x-overlap with lanes, skipping.", virtual_edge.id);
      }
    }

    if (!filtered_virtual_edges.empty()) {
      // Re-sort with updated road edges
      SortLanesRelativeToVirtualEdges(bev_candidate_lanes, filtered_virtual_edges, bev_line_sorts);

      std::vector<std::string> updated_sorts;
      for (const auto &sort_tmp : bev_line_sorts) {
        updated_sorts.push_back(fmt::format("[{}, {}]", sort_tmp.id, static_cast<int>(sort_tmp.type)));
      }
      SD_BEV_PROCESS << fmt::format("Updated bev_line_sorts: [{}]", fmt::join(updated_sorts, " "));
      SD_BEV_PROCESS << fmt::format("Updated left_road_edge_indexs: [{}]", fmt::join(bev_left_road_edge_indexs, ", "));
      SD_BEV_PROCESS << fmt::format("Updated right_road_edge_indexs: [{}]", fmt::join(bev_right_road_edge_indexs, ", "));

      DivideBevSectionsByDiversionZones(bev_candidate_lanes, filtered_virtual_edges, bev_line_sorts, complete_section_info);

      SD_BEV_PROCESS << fmt::format("After DivideBevSectionsByDiversionZones: sections size: {}", complete_section_info.sections.size());
      for (size_t i = 0; i < complete_section_info.sections.size(); ++i) {
        SD_BEV_PROCESS << fmt::format("  Section {}: [{}]", i, fmt::join(complete_section_info.sections[i].lane_ids, ", "));
      }
    }
  }

  // Step 13: 过滤被阻挡的车道
  const double max_distance          = 50.0;
  bool         skip_filterBlockLanes = false;
  if (!nearby_RoadSplits.empty()) {
    for (const auto &junction : nearby_RoadSplits) {
      if (junction.is_dedicated_right_turn_lane && junction.split_merge_direction == DirectionSplitMerge::Right) {
        skip_filterBlockLanes = true;
        break;
      }
    }
  }
  if (!skip_filterBlockLanes) {
    FilterBlockedLanesWithDistance(bev_candidate_lanes, complete_section_info, max_distance);
    SD_BEV_PROCESS << fmt::format("Executed FilterBlockedLanesWithDistance.");
  } else {
    SD_BEV_PROCESS << fmt::format("Skipped FilterBlockedLanesWithDistance due to RoadSplit with dedicated right turn lane and turn right.");
  }

  // CalculateSectionEdgeDistances(complete_section_info, bev_line_sorts, bev_left_road_edge_indexs, bev_right_road_edge_indexs);
  CalculateSectionEdgeDistances(complete_section_info, bev_line_sorts, filtered_virtual_edges);

  // Restore original bev_line_sorts to exclude virtual edges
  bev_line_sorts = std::move(original_bev_line_sorts);
  SD_BEV_PROCESS << fmt::format("Restored original bev_line_sorts: ");
  for (const auto &sort_tmp : bev_line_sorts) {
    SD_BEV_PROCESS << fmt::format("  [{}, {}]", sort_tmp.id, static_cast<int>(sort_tmp.type));
  }

  ensureEgoLaneInSingleSection(complete_section_info);

  //待完善
  // SmoothOutputBasedOnSeparators(complete_section_info);

  bev_ego_root_section_x0.clear();
  if (bev_map) {
    auto merged             = MergeSectionLaneIdsLeft2Right(complete_section_info);
    bev_ego_root_section_x0 = FindRootsAtX0AndSort(*bev_map, merged);
    SD_BEV_PROCESS << fmt::format("merged:[{}]  roots:[{}]", fmt::join(merged, ", "), fmt::join(bev_ego_root_section_x0, ", "));
  }
}

std::vector<uint64_t> BevDataProcessor::MergeSectionLaneIdsLeft2Right(const BevRouteInfo &complete_section_info) const {
  std::vector<uint64_t>        merged;
  std::unordered_set<uint64_t> seen;
  for (const auto &sec : complete_section_info.sections) {
    for (auto id : sec.lane_ids) {
      if (seen.insert(id).second)
        merged.push_back(id);
    }
  }
  return merged;
}

std::vector<uint64_t> BevDataProcessor::FindRootsAtX0AndSort(const BevMapInfo            &bev_map,
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

    // 早停条件：如果该 lane 的“起点 x”(startx) < 0，则认为脚下已覆盖，不再回溯前继
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

Eigen::Vector2f BevDataProcessor::CalculateMidpoint(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
  return (p1 + p2) / 2.0f;
}

static std::atomic<int> id_counter(0);

int BevDataProcessor::GenerateUniqueID() {
  auto now       = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  int id = static_cast<int>(timestamp % 1000000) + id_counter++;

  //  虚拟ID 范围（ 200 到 1000199）
  return (id % 1000000) + 200;
}

std::vector<BevLaneMarker> BevDataProcessor::CreateVirtualRoadEdgesFromDiversionZones(const std::vector<BevLaneMarker> &diversion_zones) {
  std::vector<BevLaneMarker> virtual_road_edges;
  for (const auto &zone : diversion_zones) {
    if (zone.geos && zone.geos->size() == 4) {
      std::vector<Eigen::Vector2f> points = *zone.geos;

      // 按 x 坐标从小到大排序
      std::sort(points.begin(), points.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });

      // 底部
      Eigen::Vector2f bottom_left  = points[0];
      Eigen::Vector2f bottom_right = points[1];

      // 顶部
      Eigen::Vector2f top_left  = points[2];
      Eigen::Vector2f top_right = points[3];

      // 计算底部和顶部边中点
      Eigen::Vector2f bottom_mid = CalculateMidpoint(bottom_left, bottom_right);
      Eigen::Vector2f top_mid    = CalculateMidpoint(top_left, top_right);

      // 生成虚拟边界线
      BevLaneMarker virtual_edge;
      // virtual_edge.id   = GenerateUniqueID();
      virtual_edge.id   = zone.id;  // 使用导流区id，便于区分
      virtual_edge.geos = std::make_shared<std::vector<Eigen::Vector2f>>();
      virtual_edge.geos->push_back(bottom_mid);
      virtual_edge.geos->push_back(top_mid);
      virtual_road_edges.push_back(virtual_edge);
    }
  }
  return virtual_road_edges;
}

// 检查是否为短乱线
bool BevDataProcessor::IsShortMessLane(const BevLaneInfo &lane) {
  if (lane.geos && lane.geos->size() > 1) {
    if (lane.previous_lane_ids.empty() && lane.next_lane_ids.empty()) {
      double start_x = lane.geos->front().x();
      double length  = fabs(lane.geos->back().x() - start_x);
      if ((start_x > 40 && length < 50) || (start_x > 50 && length < 80)) {
        return true;
      }
    }
  }
  return false;
}

// 是否是对向车道
bool BevDataProcessor::IsOppositeLane(const BevLaneInfo &lane) {
  if (lane.direction == BevLaneDirection::DIRECTION_BACKWARD) {
    return true;
  }
  if (lane.geos && lane.geos->size() > 1) {
    int opposite_point_count = 0;
    for (unsigned int i = 1; i < lane.geos->size(); ++i) {
      if (lane.geos->at(i).x() < lane.geos->at(i - 1).x()) {
        opposite_point_count++;
      }
    }
    double rate = (float)opposite_point_count / (float)lane.geos->size();
    if (rate > 0.8) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

// 检查是否为有效车道线（无后继且非短乱线）
bool BevDataProcessor::IsValidLane(const BevLaneInfo &lane) {
  if (!lane.geos || lane.geos->empty()) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: no geometry.", lane.id);
    return false;
  }

  // 检查是否是对向车道
  if (IsOppositeLane(lane)) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: opposite lane.", lane.id);
    return false;
  }

  // 应急车道检车
  if (lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: emergency lane.", lane.id);
    return false;
  }

  // 检查是否为横向车道线，并且无前继车道时过滤
  if (lane.geos->size() >= 2) {
    double delta_x = fabs(lane.geos->back().x() - lane.geos->front().x());
    double delta_y = fabs(lane.geos->back().y() - lane.geos->front().y());
    if (delta_x < delta_y && lane.previous_lane_ids.empty()) {
      SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: lateral lane without previous lanes (delta_x={:.3f}, delta_y={:.3f}).", lane.id,
                                    delta_x, delta_y);
      return false;
    }
  }

  // 首点 x > 60 且无前继车道线时无效
  if (lane.geos->front().x() > 60 && lane.previous_lane_ids.empty()) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: start x > 60 and no previous lanes.", lane.id);
    return false;
  }

  //车身后的车道线过滤
  if (lane.geos->back().x() < -5) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: end x < -5.", lane.id);
    return false;
  }

  if (IsShortMessLane(lane)) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: short mess lane.", lane.id);
    return false;
  }
  for (const auto &next_id : lane.next_lane_ids) {
    auto next_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(next_id);
    if (next_lane) {
      SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: has next lane ID: {}.", lane.id, next_id);
      return false;  // 有后继，无效
    }
  }
  return true;
}

// 检查是否为有效道路边缘
bool BevDataProcessor::IsValidRoadEdge(const BevLaneMarker &edge) {
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

  // Filter road edges starting beyond 50m from ego vehicle
  if (edge.geos->front().x() > 50.0) {
    SD_BEV_PROCESS << fmt::format("Edge ID: {} filtered: start x > 50m.", edge.id);
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

  if (length < 6.0) {  // 短线过滤
    return false;
  }

  if (edge.geos->back().x() < 0 && edge.geos->front().x() < edge.geos->back().x()) {  // 自车后向远距离
    return false;
  }
  return true;
}

void BevDataProcessor::GetCandidateLanes(const std::vector<BevLaneInfo> &lane_infos_in, const std::vector<BevLaneMarker> &bev_junctions,
                                         std::vector<BevLaneInfo *> &candidate_bev_lanes_out) {
  candidate_bev_lanes_out.clear();

  // 获取车前的第一个路口
  JunctionRange junction_info = GetFirstFrontJunction(bev_junctions);
  if (junction_info.valid) {
    SD_BEV_PROCESS << fmt::format("First front junction: x_min={:.3f}, center_x={:.3f}", junction_info.x_min, junction_info.center_x);
  } else {
    SD_BEV_PROCESS << fmt::format("No upcoming junction found ahead of the vehicle.");
  }

  // 过滤车道线
  std::vector<uint64_t> filtered_ids;
  for (const auto &lane : lane_infos_in) {
    if (lane.geos && lane.geos->size() > 1 && IsValidLane(lane) && IsWithinJunctionRange(lane, junction_info)) {
      candidate_bev_lanes_out.push_back(const_cast<BevLaneInfo *>(&lane));
    } else {
      filtered_ids.push_back(lane.id);
    }
  }
  SD_BEV_PROCESS << fmt::format("Filtered {} lanes, Filtered lane ids: [{}]", filtered_ids.size(), fmt::join(filtered_ids, ", "));
}

std::shared_ptr<std::vector<Eigen::Vector2f>> BevDataProcessor::GetFullLaneGeometry(const BevLaneInfo *lane) {
  std::unordered_set<uint64_t> visited_lanes;
  visited_lanes.clear();
  int depth = 0;
  return GetFullLaneGeometry(lane, visited_lanes, depth);
}

std::shared_ptr<std::vector<Eigen::Vector2f>> BevDataProcessor::GetFullLaneGeometry(const BevLaneInfo            *lane,
                                                                                    std::unordered_set<uint64_t> &visited_lanes,
                                                                                    int                          &depth) {
  std::shared_ptr<std::vector<Eigen::Vector2f>> full_geos = std::make_shared<std::vector<Eigen::Vector2f>>();
  if (!lane || !lane->geos || lane->geos->empty()) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} has no geometry or invalid.", lane ? lane->id : 0);
    return full_geos;
  }

  //  SD_BEV_PROCESS << fmt::format("[Recursive] Lane ID: {}, Geometry size: {}", lane->id, lane->geos->size());

  static constexpr int kMaxDepth = 4;
  // 保护：最大深度
  if (depth > kMaxDepth) {
    SD_BEV_PROCESS << fmt::format("Depth overflow on Lane ID: {}, depth={}", lane->id, depth);
    return full_geos;
  }

  // 检查缓存
  auto it = geometry_cache_.find(lane->id);
  if (it != geometry_cache_.end()) {
    // SD_BEV_PROCESS << fmt::format("Cache hit for Lane ID: {}", lane->id);
    return it->second;
  }

  // 检测环路
  if (visited_lanes.find(lane->id) != visited_lanes.end()) {
    SD_BEV_PROCESS << fmt::format("[Warning] Detected loop! Lane ID: {}", lane->id);
    return full_geos;  // 直接返回，避免无限递归
  }

  visited_lanes.insert(lane->id);  // 先插入当前，避免自环

  if (!lane->previous_lane_ids.empty()) {
    auto GetNormalizedDirection = [](const std::vector<Eigen::Vector2f> &geos, bool at_end) -> Eigen::Vector2f {
      if (geos.size() < 2)
        return Eigen::Vector2f(0, 0);
      Eigen::Vector2f vec  = at_end ? (geos.back() - geos[geos.size() - 2]) : (geos[1] - geos[0]);
      double          norm = vec.norm();
      return (norm > 1e-6) ? (vec / norm) : Eigen::Vector2f(0, 0);
    };

    Eigen::Vector2f current_start_dir = GetNormalizedDirection(*lane->geos, false);
    double          best_dot          = -1.0;
    uint64_t        best_pre_id       = 0;

    for (uint64_t pre_id : lane->previous_lane_ids) {
      if (visited_lanes.find(pre_id) != visited_lanes.end()) {
        SD_BEV_PROCESS << fmt::format("Cycle detected involving predecessor Lane ID: {}", pre_id);
        continue;  // 跳过有环的前继，避免无限递归
      }
      auto pre_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(pre_id);
      if (pre_lane && pre_lane->geos && pre_lane->geos->size() >= 2) {
        Eigen::Vector2f pre_end_dir = GetNormalizedDirection(*pre_lane->geos, true);
        if (pre_end_dir.norm() > 1e-6) {
          double dot_product = pre_end_dir.dot(current_start_dir);
          if (dot_product > best_dot) {
            best_dot    = dot_product;
            best_pre_id = pre_id;
          }
        }
      }
    }

    if (best_pre_id != 0) {
      auto best_pre_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(best_pre_id);
      if (best_pre_lane) {
        // SD_BEV_PROCESS << fmt::format("[Recursive] Recursively fetching geometry for Predecessor Lane ID: {}", best_pre_id);
        auto pre_full_geos = GetFullLaneGeometry(best_pre_lane, visited_lanes, depth);  // 递归调用
        if (pre_full_geos && !pre_full_geos->empty()) {
          // SD_BEV_PROCESS << fmt::format("Inserting pre_full_geos for Lane ID: {}, size: {}, x-range: [{:.3f}, {:.3f}]", best_pre_id,
          //                               pre_full_geos->size(), pre_full_geos->front().x(), pre_full_geos->back().x());
          SD_BEV_PROCESS << fmt::format("[Recursive] Inserting geometry from Predecessor Lane ID: {}, Size: {}", best_pre_id,
                                        pre_full_geos->size());
          full_geos->insert(full_geos->end(), pre_full_geos->begin(), pre_full_geos->end());
        }
        // 递归后插入 visited，防止环
        visited_lanes.insert(best_pre_id);
        depth++;
      } else {
        SD_BEV_PROCESS << fmt::format("Null predecessor for Lane ID: {}", lane->id);
      }
    }
  }

  // 插入当前车道几何形点
  // SD_BEV_PROCESS << fmt::format("Inserting current Lane ID: {} geometry, size: {}, x-range: [{:.3f}, {:.3f}]", lane->id, lane->geos->size(),
  //                               lane->geos->front().x(), lane->geos->back().x());
  // SD_BEV_PROCESS << fmt::format("[Recursive] Inserting current Lane ID: {}, Geometry size: {}", lane->id, lane->geos->size());
  full_geos->insert(full_geos->end(), lane->geos->begin(), lane->geos->end());

  // 计算结果存入缓存--优化后，从不同地方调用相同lane的GetFullLaneGeometry，模块只需计算一次即可
  geometry_cache_[lane->id] = full_geos;
  // SD_BEV_PROCESS << fmt::format("Final full_geos for Lane ID: {} size: {}, x-range: [{:.3f}, {:.3f}]", lane->id, full_geos->size(),
  //                               full_geos->front().x(), full_geos->back().x());

  visited_lanes.erase(lane->id);
  return full_geos;
}

void BevDataProcessor::GetCandidateRoadEdges(const std::vector<BevLaneMarker> &bev_road_edges_in,
                                             const std::vector<BevLaneInfo *> &candidate_bev_lanes,
                                             const std::vector<BevLaneMarker> &bev_junctions,
                                             std::vector<BevLaneMarker *>     &candidate_bev_road_edges_out) {
  candidate_bev_road_edges_out.clear();

  // 过滤道路边缘
  std::vector<uint64_t> filtered_ids;
  for (const auto &edge : bev_road_edges_in) {
    if (IsValidRoadEdge(edge)) {
      bool has_overlap = false;
      for (const auto &lane : candidate_bev_lanes) {
        auto full_lane_geos = GetFullLaneGeometry(lane);
        if (full_lane_geos && !full_lane_geos->empty() && edge.geos && !edge.geos->empty()) {
          double overlap = LaneGeometry::CalcLinesOverlap(*edge.geos, *full_lane_geos, 0);
          if (overlap > 0.0) {  // 重叠长度大于0
            has_overlap = true;
            break;
          }
        }
      }
      if (has_overlap) {
        candidate_bev_road_edges_out.push_back(const_cast<BevLaneMarker *>(&edge));
      } else {
        SD_BEV_PROCESS << fmt::format("Edge ID: {} does not have overlap with all lanes!", edge.id);
        filtered_ids.push_back(edge.id);
      }
    } else {
      filtered_ids.push_back(edge.id);
    }
  }
  SD_BEV_PROCESS << fmt::format("Filtered {} road edges, Filtered edge ids: [{}]", filtered_ids.size(), fmt::join(filtered_ids, ", "));
}

bool BevDataProcessor::HasCommonAncestor(const LineSortCity &l1, const LineSortCity &l2) {
  if (l1.type == LineSortType::RoadBoundary || l2.type == LineSortType::RoadBoundary) {
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

void BevDataProcessor::SortBevLaneInfoAndRoadEdges(const std::vector<BevLaneInfo *>   &bev_lane_infos_in,
                                                   const std::vector<BevLaneMarker *> &road_edges_in, std::vector<LineSortCity> &line_sorts,
                                                   std::vector<int> &left_road_edge_indexs, std::vector<int> &right_road_edge_indexs) {
  line_sorts.clear();
  left_road_edge_indexs.clear();
  right_road_edge_indexs.clear();
  std::unordered_map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> road_edge_geo_map;
  std::unordered_map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> laneinfo_geo_map;
  std::unordered_map<uint32_t, std::pair<double, double>>                               road_edge_x_range_map;
  for (auto bev_lane_info_tmp : bev_lane_infos_in) {
    if (bev_lane_info_tmp->geos && !bev_lane_info_tmp->geos->empty()) {
      // std::shared_ptr<std::vector<Eigen::Vector2f>> geos_tmp = bev_lane_info_tmp->geos;
      std::vector<double> geo_x_vec, geo_y_vec;
      geo_x_vec.clear();
      geo_y_vec.clear();
      auto cur_full_geos = GetFullLaneGeometry(bev_lane_info_tmp);
      for (auto &pt : *cur_full_geos) {
        geo_x_vec.push_back(pt.x());
        geo_y_vec.push_back(pt.y());
      }

#if 0
      for (auto &pt : *bev_lane_info_tmp->geos) {
        geo_x_vec.push_back(pt.x());
        geo_y_vec.push_back(pt.y());
      }
      if (!geo_x_vec.empty()) {

        std::vector<uint64_t> pre_laneid_tmp = bev_lane_info_tmp->previous_lane_ids;
        std::sort(pre_laneid_tmp.begin(), pre_laneid_tmp.end(), [](uint64_t a, uint64_t b) {
          auto lane_a = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(a);
          auto lane_b = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(b);
          if (lane_a && lane_b && !lane_a->geos->empty() && !lane_b->geos->empty()) {
            return fabs(lane_a->geos->front().x() - lane_a->geos->back().x()) < fabs(lane_b->geos->front().x() - lane_b->geos->back().x());
          }
          return false;
        });
        if (!pre_laneid_tmp.empty()) {
          auto pre_bev_lane_tmp = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(pre_laneid_tmp.back());
          if (pre_bev_lane_tmp) {
            geos_tmp = std::make_shared<std::vector<Eigen::Vector2f>>();
            geos_tmp->insert(geos_tmp->end(), pre_bev_lane_tmp->geos->begin(), pre_bev_lane_tmp->geos->end());
            geos_tmp->insert(geos_tmp->end(), bev_lane_info_tmp->geos->begin(), bev_lane_info_tmp->geos->end());

            //重新构建geo_x_vec，geo_y_vec，先放前继，否则geo_x_vec.front()以及geo_x_vec.back()不对
            geo_x_vec.clear();
            geo_y_vec.clear();
            for (auto &pt : *pre_bev_lane_tmp->geos) {
              geo_x_vec.push_back(pt.x());
              geo_y_vec.push_back(pt.y());
            }
            for (auto &pt : *bev_lane_info_tmp->geos) {
              geo_x_vec.push_back(pt.x());
              geo_y_vec.push_back(pt.y());
            }
          }
        }
      }
#endif
      LaneGeometry::PolynomialFitting fitting = (geo_x_vec.size() < 10) ? LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1)
                                                                        : LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3);
      laneinfo_geo_map[bev_lane_info_tmp->id] = {cur_full_geos, fitting};
      line_sorts.emplace_back(bev_lane_info_tmp->id, LineSortType::LaneLine);
      SD_BEV_PROCESS << fmt::format("Lane ID: {}, Geometry size: {}, y_at_0: {:.3f}, x-range: [{:.3f}, {:.3f}]", bev_lane_info_tmp->id,
                                    cur_full_geos->size(), fitting.GetValue(0), geo_x_vec.front(), geo_x_vec.back());
    }
  };

  for (auto &road_edge_tmp : road_edges_in) {
    if (road_edge_tmp->geos && road_edge_tmp->geos->size() > 1) {
      std::vector<double> geo_x_vec, geo_y_vec;
      double              x_min = std::numeric_limits<double>::max();
      double              x_max = std::numeric_limits<double>::lowest();
      for (auto &pt : *road_edge_tmp->geos) {
        geo_x_vec.push_back(pt.x());
        geo_y_vec.push_back(pt.y());
        x_min = std::min(x_min, static_cast<double>(pt.x()));
        x_max = std::max(x_max, static_cast<double>(pt.x()));
      }

      LaneGeometry::PolynomialFitting fitting = (geo_x_vec.size() < 10) ? LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1)
                                                                        : LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3);
      road_edge_geo_map[road_edge_tmp->id]    = {road_edge_tmp->geos, fitting};
      line_sorts.emplace_back(road_edge_tmp->id, LineSortType::RoadBoundary);
      // SD_BEV_PROCESS << fmt::format("Road Edge ID: {}, y_at_0: {:.3f}, x-range: [{:.3f}, {:.3f}]", road_edge_tmp->id, fitting.GetValue(0),
      //                               geo_x_vec.front(), geo_x_vec.back());
    }
  }

  if (laneinfo_geo_map.empty() && road_edge_geo_map.empty()) {
    SD_BEV_PROCESS << "No valid lanes or road edges to sort.";
    return;
  }

  // // 自定义位置判断函数
  // auto JudgeIsLeftWithRange = [&](const Vec2fVector &line1_geos, const Vec2fVector &line2_geos, LaneGeometry::PolynomialFitting &fitting1,
  //                                 LineSortType type1, uint32_t id1, uint32_t id2, LaneGeometry::PolynomialFitting &fitting2) -> bool {
  //   if (type1 == LineSortType::RoadBoundary && road_edge_x_range_map.count(id1)) {  // 点数 < 10，使用点位判断
  //     SD_BEV_PROCESS << fmt::format("JudgeIsLeftWithRange with points, because id1={} points less than 10", id1);
  //     // Get x-range of road edge (line1)
  //     double x_min1 = road_edge_x_range_map[id1].first;
  //     double x_max1 = road_edge_x_range_map[id1].second;

  //     // Get x-range of lane (line2)
  //     double x_min2 = line2_geos->front().x();
  //     double x_max2 = line2_geos->back().x();

  //     // Compute overlapping x-range
  //     double x_min = std::max(x_min1, x_min2);
  //     double x_max = std::min(x_max1, x_max2);

  //     if (x_min > x_max) {  // 无重叠，比较最近点
  //       double min_dist_left  = std::numeric_limits<double>::max();
  //       double min_dist_right = std::numeric_limits<double>::max();
  //       for (const auto &pt1 : *line1_geos) {
  //         for (const auto &pt2 : *line2_geos) {
  //           double dist = (pt1 - pt2).norm();
  //           if (pt1.y() > pt2.y()) {
  //             min_dist_left = std::min(min_dist_left, dist);
  //           } else {
  //             min_dist_right = std::min(min_dist_right, dist);
  //           }
  //         }
  //       }
  //       bool result = min_dist_left < min_dist_right;
  //       SD_BEV_PROCESS << fmt::format("No overlap, id1={} vs id2={}: nearest point result = {}", id1, id2, result ? "left" : "right");
  //       return result;
  //     }

  //     // Compute avg_y1 for line1 (road edge) in overlapping range
  //     double sum_y1 = 0.0;
  //     int    count1 = 0;
  //     for (const auto &pt : *line1_geos) {
  //       if (pt.x() >= x_min && pt.x() <= x_max) {
  //         sum_y1 += pt.y();
  //         count1++;
  //       }
  //     }
  //     if (count1 == 0) {  // Fallback to full line if no points in overlap (shouldn't happen for road edge)
  //       for (const auto &pt : *line1_geos) {
  //         sum_y1 += pt.y();
  //       }
  //       count1 = line1_geos->size();
  //     }
  //     double avg_y1 = sum_y1 / count1;

  //     // Compute avg_y2 for line2 (lane) in overlapping range
  //     double sum_y2 = 0.0;
  //     int    count2 = 0;
  //     for (const auto &pt : *line2_geos) {
  //       if (pt.x() >= x_min && pt.x() <= x_max) {
  //         sum_y2 += pt.y();
  //         count2++;
  //       }
  //     }
  //     if (count2 == 0) {  // Shouldn't happen since overlap exists, but for safety
  //       for (const auto &pt : *line2_geos) {
  //         sum_y2 += pt.y();
  //       }
  //       count2 = line2_geos->size();
  //     }
  //     double avg_y2 = sum_y2 / count2;
  //     bool   result = avg_y1 > avg_y2;
  //     SD_BEV_PROCESS << fmt::format("Point-based: id1={}, avg_y1={:.3f}, id2={}, avg_y2={:.3f}, result={}", id1, avg_y1, id2, avg_y2,
  //                                   result ? "left" : "right");
  //     return result;
  //   } else {  // 使用多项式拟合
  //     bool is_left = LaneGeometry::JudgeIsLeft(*line1_geos, *line2_geos, fitting1, fitting2);
  //     SD_BEV_PROCESS << fmt::format("JudgeIsLeft result: id1={} is {} of id2={}", id1, is_left ? "left" : "right", id2);
  //     return is_left;
  //   }
  // };

  // // 排序逻辑
  // std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
  //   LaneGeometry::PolynomialFitting *polynomial_fitting_l1 = nullptr;
  //   LaneGeometry::PolynomialFitting *polynomial_fitting_l2 = nullptr;
  //   Vec2fVector                      l1_geos               = nullptr;
  //   Vec2fVector                      l2_geos               = nullptr;

  //   if (l1.type == LineSortType::RoadBoundary || l1.type == LineSortType::VirtualEdge) {
  //     polynomial_fitting_l1 = &road_edge_geo_map[l1.id].second;
  //     l1_geos               = road_edge_geo_map[l1.id].first;
  //   } else {
  //     polynomial_fitting_l1 = &laneinfo_geo_map[l1.id].second;
  //     l1_geos               = laneinfo_geo_map[l1.id].first;
  //   }

  //   if (l2.type == LineSortType::RoadBoundary || l2.type == LineSortType::VirtualEdge) {
  //     polynomial_fitting_l2 = &road_edge_geo_map[l2.id].second;
  //     l2_geos               = road_edge_geo_map[l2.id].first;
  //   } else {
  //     polynomial_fitting_l2 = &laneinfo_geo_map[l2.id].second;
  //     l2_geos               = laneinfo_geo_map[l2.id].first;
  //   }

  //   if (polynomial_fitting_l1 && polynomial_fitting_l2 && l1_geos && l2_geos) {
  //     bool result = JudgeIsLeftWithRange(l1_geos, l2_geos, *polynomial_fitting_l1, l1.type, l1.id, l2.id, *polynomial_fitting_l2);
  //     SD_BEV_PROCESS << fmt::format("Sort comparison: [{},{}] vs [{},{}] -> {} < {}", l1.id, l1.type, l2.id, l2.type,
  //                                   result ? "true" : "false", result ? "left" : "right");
  //     return result;
  //   } else {
  //     SD_BEV_PROCESS << fmt::format("Sort comparison skipped: [{},{}] vs [{},{}] (invalid geos)", l1.id, l1.type, l2.id, l2.type);
  //     return false;
  //   }
  // });

  auto JudgeIsLeftSimple = [&](const LineSortCity &l1, const LineSortCity &l2) -> bool {
    auto &[geo1, fitting1] = (l1.type == LineSortType::LaneLine) ? laneinfo_geo_map[l1.id] : road_edge_geo_map[l1.id];
    auto &[geo2, fitting2] = (l2.type == LineSortType::LaneLine) ? laneinfo_geo_map[l2.id] : road_edge_geo_map[l2.id];

    if (!geo1 || !geo2) {
      SD_BEV_PROCESS << fmt::format("Error: Lane {} or Lane {} has no valid geometry.", l1.id, l2.id);
      return false;
    }
    // 两条线都有足够的点，使用拟合曲线判断
    if (geo1->size() >= 10 && geo2->size() >= 10) {
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
          if (l1_center_pt.y() > l2_center_pt.y()) {
            return true;
          } else {
            return false;
          }
        }
      } else {
        bool is_left = LaneGeometry::JudgeIsLeft(*geo1, *geo2, fitting1, fitting2);
        SD_BEV_PROCESS << fmt::format("JudgeIsLeft (both >=10, fitting): id1={} vs id2={} -> {}", l1.id, l2.id, is_left ? "left" : "right");
        return is_left;
      }
    }
    // 两条线点数都不足10，比较平均 Y 值
    else if (geo1->size() < 10 && geo2->size() < 10) {
      double avg_y1 = 0.0, avg_y2 = 0.0;
      for (const auto &pt : *geo1)
        avg_y1 += pt.y();
      for (const auto &pt : *geo2)
        avg_y2 += pt.y();
      avg_y1 /= geo1->size();
      avg_y2 /= geo2->size();
      bool result = avg_y1 > avg_y2;
      SD_BEV_PROCESS << fmt::format("JudgeIsLeft (both <10, avg_y): id1={}, avg_y1={:.3f}, id2={}, avg_y2={:.3f}, result={}", l1.id, avg_y1,
                                    l2.id, avg_y2, result ? "left" : "right");
      return result;
    }
    // l1 点数 < 10，用 l1 的点判断是否在 l2 左侧
    else if (geo1->size() < 10) {
      int left_count = 0;
      for (const auto &pt : *geo1) {
        if (LaneGeometry::JudgeIsLeft_navigation(pt, *geo2)) {
          left_count++;
        }
      }
      double fraction_left = static_cast<double>(left_count) / geo1->size();
      bool   result        = fraction_left > 0.5;
      SD_BEV_PROCESS << fmt::format("JudgeIsLeft (l1 <10): id1={} vs id2={}, fraction_left={:.2f}, result={}", l1.id, l2.id, fraction_left,
                                    result ? "left" : "right");
      return result;
    }
    // l2 点数 < 10，用 l2 的点判断是否在 l1 右侧
    else {  // geo2->size() < 10
      int right_count = 0;
      for (const auto &pt : *geo2) {
        if (!LaneGeometry::JudgeIsLeft_navigation(pt, *geo1)) {  // pt 在 geo1 右侧
          right_count++;
        }
      }
      double fraction_right = static_cast<double>(right_count) / geo2->size();
      bool   result         = fraction_right > 0.5;
      SD_BEV_PROCESS << fmt::format("JudgeIsLeft (l2 <10): id1={} vs id2={}, fraction_right={:.2f}, result={}", l1.id, l2.id,
                                    fraction_right, result ? "left" : "right");
      return result;
    }
  };

  if (line_sorts.size() >= 16) {
    SD_MERGE_LOG << fmt::format("line_sorts size: {} > 16.", line_sorts.size());
    AWARN << "Input lane size too much, exit from SortBevLaneInfoAndRoadEdges";
    return;
  }
  std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) { return JudgeIsLeftSimple(l1, l2); });

  size_t ego_lane_index = line_sorts.size();
  for (unsigned i = 0; i < line_sorts.size(); i++) {
    if (line_sorts[i].type == LineSortType::LaneLine && bev_ego_lane_related_.count(line_sorts[i].id)) {
      ego_lane_index = i;
      break;
    }
  }
  // 根据 ego lane 位置分类道路边缘
  if (ego_lane_index != line_sorts.size()) {
    for (size_t i = 0; i < line_sorts.size(); ++i) {
      if (line_sorts[i].type == LineSortType::RoadBoundary) {
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

void BevDataProcessor::BevLanesFilterByRoadEdges(const std::vector<LineSortCity> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                                 const std::vector<int>     &right_road_edge_indexs,
                                                 std::vector<BevLaneInfo *> &candidateBevLanes, bool &filtered_non_ego_lanes) {
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
      for (size_t p = left_divider_edge_index + 1; p < line_sorts.size(); p++) {
        if (line_sorts[p].type == LineSortType::LaneLine) {
          left_divider_first_lane_id = line_sorts[p].id;
          break;
        }
      }

      ////删除left_divider左侧的无拓扑关系的所有lane
      for (int k = 0; k < left_divider_edge_index; k++) {
        if (line_sorts[k].type == LineSortType::LaneLine) {
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
    const double r_min_road_edge_length     = 40.0f;
    if (!right_road_edge_indexs.empty()) {
      for (size_t n = 0; n < right_road_edge_indexs.size(); n++) {
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
        if (line_sorts[k].type == LineSortType::LaneLine) {
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

  filtered_non_ego_lanes = !delete_laneids_set.empty();
}

void BevDataProcessor::InsertVirtualEdge(const BevLaneMarker &virtual_edge, std::vector<LineSortCity> &bev_line_sorts,
                                         const std::vector<BevLaneInfo *> &bev_candidate_lanes) {
  std::unordered_map<uint64_t, const BevLaneInfo *> lane_map;
  for (const auto *lane : bev_candidate_lanes) {
    if (lane && lane->geos && !lane->geos->empty()) {
      lane_map[lane->id] = lane;
    }
  }

  if (!virtual_edge.geos || virtual_edge.geos->empty()) {
    // bev_line_sorts.emplace_back(virtual_edge.id, LineSortType::VirtualEdge);
    return;
  }

  // Eigen::Vector2f virtual_midpoint = CalculateMidpoint(virtual_edge.geos->front(), virtual_edge.geos->back());
  std::vector<LineSortCity> new_sorts;
  bool                      inserted = false;

  for (const auto &sort : bev_line_sorts) {
    if (!inserted && sort.type == LineSortType::LaneLine) {
      const auto *lane          = lane_map[sort.id];
      auto        cur_full_geos = GetFullLaneGeometry(lane);
      if (lane && cur_full_geos && !cur_full_geos->empty()) {
        // Eigen::Vector2f lane_midpoint = CalculateMidpoint(lane->geos->front(), lane->geos->back());
        if (LaneGeometry::JudgeIsLeft(*virtual_edge.geos, *cur_full_geos)) {
          new_sorts.emplace_back(virtual_edge.id, LineSortType::VirtualEdge);
          inserted = true;
          SD_BEV_PROCESS << fmt::format("[InsertVirtualEdge] edge={} vs lane={} -> left", virtual_edge.id, lane->id);
        } else {
          SD_BEV_PROCESS << fmt::format("[InsertVirtualEdge] edge={} vs lane={} -> right", virtual_edge.id, lane->id);
        }
      }
    }
    new_sorts.push_back(sort);
  }

  if (!inserted) {
    // new_sorts.emplace_back(virtual_edge.id, true);
    new_sorts.emplace_back(virtual_edge.id, LineSortType::VirtualEdge);
  }

  bev_line_sorts = std::move(new_sorts);
}

void BevDataProcessor::SortLanesRelativeToVirtualEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes,
                                                       const std::vector<BevLaneMarker> &virtual_road_edges,
                                                       std::vector<LineSortCity>        &bev_line_sorts) {
  SD_BEV_PROCESS << fmt::format("[SortLanesRelativeToVirtualEdges] virtual_road_edges size ={}", virtual_road_edges.size());
  for (const auto &edge : virtual_road_edges) {
    SD_BEV_PROCESS << fmt::format("[SortLanesRelativeToVirtualEdges] virtual_edge: {}", edge);
    InsertVirtualEdge(edge, bev_line_sorts, bev_candidate_lanes);
  }
}

bool BevDataProcessor::HasXOverlap(const BevLaneMarker &virtual_edge, const std::vector<BevLaneInfo *> &bev_candidate_lanes) {
  if (!virtual_edge.geos || virtual_edge.geos->size() < 2) {
    return false;
  }

  double virtual_min_x = std::min(virtual_edge.geos->front().x(), virtual_edge.geos->back().x());
  double virtual_max_x = std::max(virtual_edge.geos->front().x(), virtual_edge.geos->back().x());

  for (const auto &lane : bev_candidate_lanes) {
    auto full_lane_geos = GetFullLaneGeometry(lane);
    if (full_lane_geos && !full_lane_geos->empty()) {
      double lane_min_x = full_lane_geos->front().x();
      double lane_max_x = full_lane_geos->back().x();
      for (const auto &pt : *full_lane_geos) {
        if (pt.x() < lane_min_x)
          lane_min_x = pt.x();
        if (pt.x() > lane_max_x)
          lane_max_x = pt.x();
      }
      // Check overlap
      if (!(virtual_max_x < lane_min_x || virtual_min_x > lane_max_x)) {
        return true;  // Overlap found
      }
    }
  }
  return false;  // No overlap with any lane
}

void BevDataProcessor::ProcessDiversionZones(const std::vector<BevLaneInfo *>   &bev_candidate_lanes,
                                             std::vector<std::vector<uint64_t>> &bev_sections,
                                             const std::vector<BevLaneMarker>   &virtual_road_edges) {
  if (virtual_road_edges.empty() || bev_sections.empty()) {
    return;
  }

  std::unordered_map<uint64_t, const BevLaneInfo *> lane_map;
  for (const auto *lane : bev_candidate_lanes) {
    if (lane && lane->geos && !lane->geos->empty()) {
      lane_map[lane->id] = lane;
    }
  }

  for (const auto &virtual_edge : virtual_road_edges) {
    if (!virtual_edge.geos || virtual_edge.geos->size() < 2) {
      continue;
    }

    // Eigen::Vector2f                    rep_point = CalculateMidpoint(virtual_edge.geos->front(), virtual_edge.geos->back());
    std::vector<std::vector<uint64_t>> new_sections;

    for (const auto &section : bev_sections) {
      std::vector<uint64_t> left_section, right_section;
      for (const auto &lane_id : section) {
        const auto *lane = lane_map[lane_id];
        if (!lane || !lane->geos || lane->geos->empty()) {
          continue;
        }

        // bool is_left = LaneGeometry::JudgeIsLeft(rep_point, *lane->geos);
        bool is_left = LaneGeometry::JudgeIsLeft(*virtual_edge.geos, *lane->geos);
        if (is_left) {
          right_section.push_back(lane_id);
        } else {
          left_section.push_back(lane_id);
        }
      }

      if (!left_section.empty()) {
        new_sections.push_back(left_section);
      }
      if (!right_section.empty()) {
        new_sections.push_back(right_section);
      }
    }

    bev_sections = std::move(new_sections);
  }
}

void BevDataProcessor::DivideBevSectionsByRoadEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes,
                                                    const std::vector<LineSortCity> &bev_line_sorts, BevRouteInfo &complete_section_info) {
  std::unordered_map<uint32_t, BevLaneMarker *> edge_map = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeList();
  DivideBevSectionsByEdges(bev_candidate_lanes, bev_line_sorts, edge_map, complete_section_info);
}

void BevDataProcessor::DivideBevSectionsByDiversionZones(const std::vector<BevLaneInfo *> &bev_candidate_lanes,
                                                         const std::vector<BevLaneMarker> &virtualEdges,
                                                         const std::vector<LineSortCity>  &bev_line_sorts,
                                                         BevRouteInfo                     &complete_section_info) {
  std::unordered_map<uint32_t, BevLaneMarker *> edge_map;
  for (const auto &edge : virtualEdges) {
    edge_map[edge.id] = const_cast<BevLaneMarker *>(&edge);
  }
  DivideBevSectionsByEdges(bev_candidate_lanes, bev_line_sorts, edge_map, complete_section_info);
}

void BevDataProcessor::DivideBevSectionsByEdges(const std::vector<BevLaneInfo *>                    &bev_candidate_lanes,
                                                const std::vector<LineSortCity>                     &bev_line_sorts,
                                                const std::unordered_map<uint32_t, BevLaneMarker *> &edge_map,
                                                BevRouteInfo                                        &complete_section_info) {
  complete_section_info.sections.clear();
  complete_section_info.separators.clear();

  std::unordered_map<uint64_t, BevLaneInfo *> lane_map;
  for (auto lane : bev_candidate_lanes) {
    lane_map[lane->id] = lane;
  }

  std::vector<LineSortCity> bev_line_sorts_new;
  for (const auto &line_sort_tmp : bev_line_sorts) {
    if (line_sort_tmp.type == LineSortType::LaneLine) {
      if (lane_map.find(line_sort_tmp.id) != lane_map.end()) {
        bev_line_sorts_new.push_back(line_sort_tmp);
      }
    } else {
      bev_line_sorts_new.push_back(line_sort_tmp);
    }
  }

  std::vector<uint64_t> current_section;
  double                current_section_min_x = std::numeric_limits<double>::max();
  double                current_section_max_x = std::numeric_limits<double>::lowest();
  uint64_t              section_id            = 1;
  SeparatorInfo         current_separator;  // Single SeparatorInfo to collect multiple edges
  bool                  in_section            = false;
  bool                  collecting_separators = false;

  auto get_x_range = [&](uint64_t id, LineSortType type) -> std::pair<double, double> {
    if (type == LineSortType::LaneLine) {
      auto lane = lane_map[id];
      if (lane && lane->geos && !lane->geos->empty()) {
        return CalculateLaneXRange(lane);
      }
    } else {
      auto edge_it = edge_map.find(id);
      if (edge_it != edge_map.end() && edge_it->second->geos && !edge_it->second->geos->empty()) {
        double min_x = std::min_element(edge_it->second->geos->begin(), edge_it->second->geos->end(),
                                        [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); })
                           ->x();
        double max_x = std::max_element(edge_it->second->geos->begin(), edge_it->second->geos->end(),
                                        [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); })
                           ->x();
        return {min_x, max_x};
      }
    }
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  };

  // for (size_t i = 0; i < bev_line_sorts_new.size(); ++i) {
  //   const auto &current = bev_line_sorts_new[i];
  //   if (current.type == LineSortType::RoadBoundary || current.type == LineSortType::VirtualEdge) {
  //     if (!current_section.empty()) {
  //       BevSectionInfo section;
  //       section.id                 = section_id++;
  //       section.lane_ids           = current_section;
  //       section.lane_num           = current_section.size();
  //       section.start_range_offset = current_section_min_x;
  //       section.end_range_offset   = current_section_max_x;
  //       section.length             = current_section_max_x - current_section_min_x;

  //       // 预留字段留空
  //       section.predecessor_section_id_list.clear();
  //       section.successor_section_id_list.clear();

  //       complete_section_info.sections.push_back(section);
  //       current_section.clear();
  //       current_section_min_x = std::numeric_limits<double>::max();
  //       current_section_max_x = std::numeric_limits<double>::lowest();
  //     }
  //     // 记录分隔符信息
  //     if (i > 0 && bev_line_sorts_new[i - 1].type == LineSortType::LaneLine) {
  //       auto edge_ptr = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(current.id);
  //       if (edge_ptr && edge_ptr->geos && !edge_ptr->geos->empty()) {
  //         Eigen::Vector2f start_point = edge_ptr->geos->front();
  //         Eigen::Vector2f end_point   = edge_ptr->geos->back();
  //         complete_section_info.separators.emplace_back(current.id, start_point, end_point, current.type);
  //       }
  //     }
  //     continue;
  //   }

  //   current_section.push_back(current.id);

  //   // 更新当前section的X-range
  //   auto current_lane = lane_map[current.id];
  //   if (current_lane && current_lane->geos && !current_lane->geos->empty()) {
  //     auto [lane_min_x, lane_max_x] = CalculateLaneXRange(current_lane);
  //     current_section_min_x         = std::min(current_section_min_x, lane_min_x);
  //     current_section_max_x         = std::max(current_section_max_x, lane_max_x);
  //   }

  //   // 检查与下一条线的关系（如果存在且不是道路边界）
  //   if (i + 1 < bev_line_sorts_new.size() && bev_line_sorts_new[i + 1].type == LineSortType::LaneLine) {
  //     const auto &next      = bev_line_sorts_new[i + 1];
  //     auto        next_lane = lane_map[next.id];

  //     // 获取完整的车道几何信息
  //     auto current_full_geos = GetFullLaneGeometry(current_lane);
  //     auto next_full_geos    = GetFullLaneGeometry(next_lane);

  //     if (next_lane && next_lane->geos && !next_lane->geos->empty()) {
  //       // Calculate next_lane's X-range
  //       // auto [next_min_x, next_max_x] = CalculateLaneXRange(next_lane);

  //       // Calculate overlap between current_section’s X-range and next_lane’s X-range
  //       // double overlap_start = std::max(current_section_min_x, next_min_x);
  //       // double overlap_end   = std::min(current_section_max_x, next_max_x);
  //       // double overlap       = (overlap_end > overlap_start) ? (overlap_end - overlap_start) : 0.0;

  //       // Calculate lateral distance between current_lane and next_lane
  //       auto   current_full_geos = GetFullLaneGeometry(current_lane);
  //       auto   next_full_geos    = GetFullLaneGeometry(next_lane);
  //       double lateral_dist      = LaneGeometry::GetDistanceBetweenLines(*current_full_geos, *next_full_geos);

  //       // SD_BEV_PROCESS << fmt::format(
  //       //     "current_section x_range=[{:.3f}, {:.3f}], next_lane id={}, x_range=[{:.3f}, {:.3f}], overlap={:.3f}, lateral_dist={:.3f}",
  //       //     current_section_min_x, current_section_max_x, next_lane->id, next_min_x, next_max_x, overlap, lateral_dist);

  //       // 如果无重叠或横向距离过大，切分 section---ClusterLanesByX已做了纵向的聚类，横向section划分不再参考lane的overlap
  //       // if (overlap <= 0.0 || lateral_dist > 7.0) {
  //       if (lateral_dist > 7.0) {
  //         BevSectionInfo section;
  //         section.id                 = section_id++;
  //         section.lane_ids           = current_section;
  //         section.lane_num           = current_section.size();
  //         section.start_range_offset = current_section_min_x;
  //         section.end_range_offset   = current_section_max_x;
  //         section.length             = current_section_max_x - current_section_min_x;

  //         section.predecessor_section_id_list.clear();
  //         section.successor_section_id_list.clear();

  //         complete_section_info.sections.push_back(section);

  //         // 间隔过大，用特殊的id=0，作为分割信息
  //         if (i > 0 && bev_line_sorts_new[i - 1].type == LineSortType::LaneLine) {
  //           Eigen::Vector2f start_point(current_section_min_x, 0.0f);
  //           Eigen::Vector2f end_point(current_section_max_x, 0.0f);
  //           complete_section_info.separators.emplace_back(0, start_point, end_point, LineSortType::LateralDist);
  //         }

  //         current_section.clear();
  //         current_section_min_x = std::numeric_limits<double>::max();
  //         current_section_max_x = std::numeric_limits<double>::lowest();
  //       }
  //     }
  //   }
  // }

  for (size_t i = 0; i < bev_line_sorts_new.size(); ++i) {
    const auto &current = bev_line_sorts_new[i];

    if (current.type == LineSortType::LaneLine) {
      if (collecting_separators && !current_separator.edge_ids.empty()) {
        complete_section_info.separators.push_back(current_separator);
        current_separator     = SeparatorInfo();  // Reset for next group
        collecting_separators = false;
      }
      if (!in_section) {
        in_section      = true;
        current_section = {current.id};
        // auto current_lane             = lane_map[current.id];
        auto [lane_min_x, lane_max_x] = get_x_range(current.id, current.type);
        current_section_min_x         = lane_min_x;
        current_section_max_x         = lane_max_x;
      } else {
        current_section.push_back(current.id);
        auto [lane_min_x, lane_max_x] = get_x_range(current.id, current.type);
        current_section_min_x         = std::min(current_section_min_x, lane_min_x);
        current_section_max_x         = std::max(current_section_max_x, lane_max_x);
        // Check lateral distance with next lane line if applicable
        if (i + 1 < bev_line_sorts_new.size() && bev_line_sorts_new[i + 1].type == LineSortType::LaneLine) {
          const auto &next      = bev_line_sorts_new[i + 1];
          auto        next_lane = lane_map[next.id];
          if (next_lane && next_lane->geos && !next_lane->geos->empty()) {
            auto   current_full_geos = GetFullLaneGeometry(lane_map[current.id]);
            auto   next_full_geos    = GetFullLaneGeometry(next_lane);
            double lateral_dist      = LaneGeometry::GetDistanceBetweenLines(*current_full_geos, *next_full_geos);

            double cur_min_x = current_full_geos->front().x();
            double cur_max_x = current_full_geos->back().x();
            SD_BEV_PROCESS << fmt::format("current_section x_range=[{:.3f}, {:.3f}], cur_lane_id={}, cur_min_x={:.3f}, cur_max_x={:.3f}",
                                          current_section_min_x, current_section_max_x, current.id, cur_min_x, cur_max_x);

            SD_BEV_PROCESS << fmt::format("next_lane_id={}, lateral_dist={:.3f}", next_lane->id, lateral_dist);

            if (lateral_dist > 15.0) {
              BevSectionInfo section;
              section.id                 = section_id++;
              section.lane_ids           = current_section;
              section.lane_num           = current_section.size();
              section.start_range_offset = current_section_min_x;
              section.end_range_offset   = current_section_max_x;
              section.length             = current_section_max_x - current_section_min_x;
              section.predecessor_section_id_list.clear();
              section.successor_section_id_list.clear();
              complete_section_info.sections.push_back(section);

              SeparatorInfo separator;
              separator.edge_ids.push_back(0);
              separator.types.push_back(LineSortType::LateralDist);
              separator.edge_points.push_back({Eigen::Vector2f(current_section_min_x, 0.0f), Eigen::Vector2f(current_section_max_x, 0.0f)});
              complete_section_info.separators.push_back(separator);

              current_section.clear();
              current_section_min_x = std::numeric_limits<double>::max();
              current_section_max_x = std::numeric_limits<double>::lowest();
              in_section            = false;
            }
          }
        }
      }
    } else if (current.type == LineSortType::RoadBoundary || current.type == LineSortType::VirtualEdge) {
      size_t prev_lane_idx = i;
      while (prev_lane_idx > 0 && bev_line_sorts_new[prev_lane_idx - 1].type != LineSortType::LaneLine) {
        --prev_lane_idx;
      }
      size_t next_lane_idx = i;
      while (next_lane_idx < bev_line_sorts_new.size() - 1 && bev_line_sorts_new[next_lane_idx + 1].type != LineSortType::LaneLine) {
        ++next_lane_idx;
      }

      if (prev_lane_idx > 0 && next_lane_idx < bev_line_sorts_new.size() - 1) {
        auto [edge_min_x, edge_max_x]           = get_x_range(current.id, current.type);
        auto [prev_lane_min_x, prev_lane_max_x] = get_x_range(bev_line_sorts_new[prev_lane_idx - 1].id, LineSortType::LaneLine);
        auto [next_lane_min_x, next_lane_max_x] = get_x_range(bev_line_sorts_new[next_lane_idx + 1].id, LineSortType::LaneLine);
        // SD_BEV_PROCESS << fmt::format("Edge id:{}, x_start:{}, x_end:{}", current.id, edge_min_x, edge_max_x);
        // SD_BEV_PROCESS << fmt::format("Lane id:{}, x_start:{}, x_end:{}", bev_line_sorts_new[prev_lane_idx - 1].id, prev_lane_min_x,
        //                               prev_lane_max_x);
        // SD_BEV_PROCESS << fmt::format("Lane id:{}, x_start:{}, x_end:{}", bev_line_sorts_new[next_lane_idx + 1].id, next_lane_min_x,
        //                               next_lane_max_x);
        bool overlaps_prev = !(edge_max_x < prev_lane_min_x || edge_min_x > prev_lane_max_x);
        bool overlaps_next = !(edge_max_x < next_lane_min_x || edge_min_x > next_lane_max_x);
        // SD_BEV_PROCESS << fmt::format("overlaps_prev:{}", overlaps_prev);
        // SD_BEV_PROCESS << fmt::format("overlaps_next:{}", overlaps_prev);
        auto   edge_it     = edge_map.find(current.id);
        double edge_startX = 0.0;
        if (edge_it != edge_map.end() && edge_it->second->geos && !edge_it->second->geos->empty()) {
          Eigen::Vector2f start_point = edge_it->second->geos->front();
          edge_startX                 = start_point.x();
        }
        if (!overlaps_prev && !overlaps_next) {
          SD_BEV_PROCESS << fmt::format("Edge id:{} have no overlap both with lane:{} and lane:{}", current.id,
                                        bev_line_sorts_new[prev_lane_idx - 1].id, bev_line_sorts_new[next_lane_idx + 1].id);
        }

        SD_BEV_PROCESS << fmt::format("Edge id:{} startx is :{}", current.id, edge_startX);
        if (overlaps_prev || overlaps_next || edge_startX < 20.0) {  // 如果与前一条或后一条车道线有重叠，则用作分隔符
          if (in_section) {
            BevSectionInfo section;
            section.id                 = section_id++;
            section.lane_ids           = current_section;
            section.lane_num           = current_section.size();
            section.start_range_offset = current_section_min_x;
            section.end_range_offset   = current_section_max_x;
            section.length             = current_section_max_x - current_section_min_x;
            section.predecessor_section_id_list.clear();
            section.successor_section_id_list.clear();
            complete_section_info.sections.push_back(section);

            current_section.clear();
            current_section_min_x = std::numeric_limits<double>::max();
            current_section_max_x = std::numeric_limits<double>::lowest();
            in_section            = false;
          }
          collecting_separators = true;
          current_separator     = SeparatorInfo();
          current_separator.edge_ids.emplace_back(current.id);
          current_separator.types.emplace_back(current.type);
          auto edge_it = edge_map.find(current.id);
          if (edge_it != edge_map.end() && edge_it->second->geos && !edge_it->second->geos->empty()) {
            Eigen::Vector2f start_point = edge_it->second->geos->front();
            Eigen::Vector2f end_point   = edge_it->second->geos->back();
            current_separator.edge_points.push_back({start_point, end_point});
          }
        }
      }
      // Edges before the first section are ignored
    }
  }

  if (!current_section.empty()) {
    BevSectionInfo section;
    section.id                 = section_id++;
    section.lane_ids           = current_section;
    section.lane_num           = current_section.size();
    section.start_range_offset = current_section_min_x;
    section.end_range_offset   = current_section_max_x;
    section.length             = current_section_max_x - current_section_min_x;

    section.predecessor_section_id_list.clear();
    section.successor_section_id_list.clear();

    complete_section_info.sections.push_back(section);
  }

  // Remove empty sections
  complete_section_info.sections.erase(std::remove_if(complete_section_info.sections.begin(), complete_section_info.sections.end(),
                                                      [](const BevSectionInfo &sec) { return sec.lane_ids.empty(); }),
                                       complete_section_info.sections.end());
}

std::pair<int, int> BevDataProcessor::GetEgoLeftRightRoadEdges(const std::vector<LineSortCity> &line_sorts,
                                                               const std::vector<int>          &left_road_edge_indexs,
                                                               const std::vector<int>          &right_road_edge_indexs) {
  int Left_EgoEdge_Index  = -1;
  int Right_EgoEdge_Index = -1;
  if (left_road_edge_indexs.empty() && right_road_edge_indexs.empty()) {
    return {Left_EgoEdge_Index, Right_EgoEdge_Index};
  }

  const double l_longitude_dist_threshold = 13.0f;
  const double l_min_road_edge_length     = 5.0f;
  if (!left_road_edge_indexs.empty()) {
    for (int n = left_road_edge_indexs.size() - 1; n >= 0; n--) {
      auto road_edge_ptr = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(line_sorts[left_road_edge_indexs[n]].id);
      if (!road_edge_ptr)
        continue;
      auto   road_edge_geo_ptr     = road_edge_ptr->geos;
      double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
      double start_longitude_range = road_edge_geo_ptr->front().x() - l_longitude_dist_threshold;
      double end_longitude_range   = road_edge_geo_ptr->back().x() + l_longitude_dist_threshold;
      if (road_edge_length > l_min_road_edge_length && start_longitude_range < 0 && end_longitude_range > 0) {
        Left_EgoEdge_Index = left_road_edge_indexs[n];
        break;
      }
    }
  }

  const double r_longitude_dist_threshold = 13.0f;
  const double r_min_road_edge_length     = 5.0f;
  if (!right_road_edge_indexs.empty()) {
    for (unsigned int n = 0; n < right_road_edge_indexs.size(); n++) {
      auto   road_edge_id          = line_sorts[right_road_edge_indexs[n]].id;
      auto   road_edge_geo_ptr     = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(road_edge_id)->geos;
      double road_edge_length      = fabs(road_edge_geo_ptr->back().x() - road_edge_geo_ptr->front().x());
      double start_longitude_range = road_edge_geo_ptr->front().x() - r_longitude_dist_threshold;
      double end_longitude_range   = road_edge_geo_ptr->back().x() + r_longitude_dist_threshold;
      if (road_edge_length > r_min_road_edge_length && start_longitude_range < 0 && end_longitude_range > 0) {
        Right_EgoEdge_Index = right_road_edge_indexs[n];
        break;
      }
    }
  }
  return {Left_EgoEdge_Index, Right_EgoEdge_Index};
}

std::set<uint64_t> BevDataProcessor::GetEgoLaneRelated(const std::vector<BevLaneInfo> &all_bev_lane_infos) {
  std::set<uint64_t> ego_lane_related;
  const BevLaneInfo *ego_lane = nullptr;

  for (const auto &lane : all_bev_lane_infos) {
    if (lane.position == static_cast<int>(BevLanePosition::LANE_LOC_EGO)) {
      ego_lane = &lane;
      break;
    }
  }
  if (!ego_lane) {
    return ego_lane_related;
  }
  ego_lane_related.insert(ego_lane->id);

  //递归插入所有后继车道
  std::function<void(uint64_t)> insert_successors = [&](uint64_t lane_id) {
    auto lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (lane) {
      for (uint64_t next_id : lane->next_lane_ids) {
        if (ego_lane_related.insert(next_id).second) {
          insert_successors(next_id);  // 递归处理后继车道
        }
      }
    }
  };
  insert_successors(ego_lane->id);

  for (uint64_t prev_id : ego_lane->previous_lane_ids) {
    ego_lane_related.insert(prev_id);
  }

  return ego_lane_related;
}

std::pair<double, double> BevDataProcessor::CalculateLaneXRange(const BevLaneInfo *lane) {
  if (!lane || !lane->geos || lane->geos->empty())
    return {0.0, 0.0};
  double min_x = lane->geos->front().x();
  double max_x = lane->geos->back().x();
  for (const auto &pt : *lane->geos) {
    if (pt.x() < min_x)
      min_x = pt.x();
    if (pt.x() > max_x)
      max_x = pt.x();
  }
  return {min_x, max_x};
}

double BevDataProcessor::CalculateCrosswalkCenterX(const BevLaneMarker &crosswalk) {
  if (!crosswalk.geos || crosswalk.geos->size() != 4)
    return std::numeric_limits<double>::max();
  double sum_x = 0.0;
  for (const auto &pt : *crosswalk.geos) {
    sum_x += pt.x();
  }
  return sum_x / 4.0;
}

std::vector<ClusterInfo> BevDataProcessor::ClusterLanesByX(const std::vector<BevLaneInfo *>   &lanes,
                                                           const std::vector<BevLaneMarker *> &bev_candidate_road_edges,
                                                           double                              gap_threshold) {
  std::vector<ClusterInfo> clusters;
  if (lanes.empty())
    return clusters;

  // Step 1: Calculate extended X-range for each lane including predecessors
  std::unordered_map<uint64_t, std::pair<double, double>> extended_x_ranges;
  for (const auto *lane : lanes) {
    if (lane && lane->geos && !lane->geos->empty()) {
      double min_x = lane->geos->front().x();
      double max_x = lane->geos->back().x();
      // for (const auto &pt : *lane->geos) {
      //   min_x = std::min(min_x, static_cast<double>(pt.x()));
      //   max_x = std::max(max_x, static_cast<double>(pt.x()));
      // }
      std::set<uint64_t>            visited;
      std::function<void(uint64_t)> get_prev_x_range = [&](uint64_t lane_id) {
        if (visited.count(lane_id))
          return;
        visited.insert(lane_id);
        auto prev_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
        if (prev_lane && prev_lane->geos && !prev_lane->geos->empty()) {
          for (const auto &pt : *prev_lane->geos) {
            min_x = std::min(min_x, static_cast<double>(pt.x()));
            max_x = std::max(max_x, static_cast<double>(pt.x()));
          }
          for (uint64_t prev_id : prev_lane->previous_lane_ids) {
            get_prev_x_range(prev_id);
          }
        }
      };
      for (uint64_t prev_id : lane->previous_lane_ids) {
        get_prev_x_range(prev_id);
      }
      extended_x_ranges[lane->id] = {min_x, max_x};
      // SD_BEV_PROCESS << fmt::format("Lane ID: {} extended x-range: [{}, {}]", lane->id, min_x, max_x);
    }
  }

  // 按起始 X 坐标排序
  // std::vector<BevLaneInfo *> sorted_lanes = lanes;
  // std::sort(sorted_lanes.begin(), sorted_lanes.end(),
  //           [](const BevLaneInfo *a, const BevLaneInfo *b) { return a->geos->front().x() < b->geos->front().x(); });

  // Step 2: Sort lanes by extended min_x
  std::vector<BevLaneInfo *> sorted_lanes = lanes;
  std::sort(sorted_lanes.begin(), sorted_lanes.end(),
            [&](const BevLaneInfo *a, const BevLaneInfo *b) { return extended_x_ranges[a->id].first < extended_x_ranges[b->id].first; });

  // Step 3: Cluster lanes based on extended X-range
  ClusterInfo current_cluster;
  current_cluster.lanes.push_back(sorted_lanes[0]);
  current_cluster.min_x = extended_x_ranges[sorted_lanes[0]->id].first;
  current_cluster.max_x = extended_x_ranges[sorted_lanes[0]->id].second;

  for (size_t i = 1; i < sorted_lanes.size(); ++i) {
    auto  *lane       = sorted_lanes[i];
    double lane_min_x = extended_x_ranges[lane->id].first;
    double lane_max_x = extended_x_ranges[lane->id].second;

    if (lane_min_x <= current_cluster.max_x + gap_threshold) {
      current_cluster.lanes.push_back(lane);
      current_cluster.min_x = std::min(current_cluster.min_x, lane_min_x);
      current_cluster.max_x = std::max(current_cluster.max_x, lane_max_x);
    } else {
      clusters.push_back(current_cluster);
      current_cluster.lanes.clear();
      current_cluster.lanes.push_back(lane);
      current_cluster.min_x = lane_min_x;
      current_cluster.max_x = lane_max_x;
    }
  }

  if (!current_cluster.lanes.empty()) {
    clusters.push_back(current_cluster);
  }

  // Step 4: Bind road edges to clusters
  const double overlap_threshold = 5.0;
  for (auto &cluster : clusters) {
    for (auto *edge : bev_candidate_road_edges) {
      if (edge->geos && !edge->geos->empty()) {
        double edge_min_x = std::numeric_limits<double>::max();
        double edge_max_x = std::numeric_limits<double>::lowest();
        for (const auto &pt : *edge->geos) {
          edge_min_x = std::min(edge_min_x, static_cast<double>(pt.x()));
          edge_max_x = std::max(edge_max_x, static_cast<double>(pt.x()));
        }
        double distance = 0.0;
        if (edge_max_x < cluster.min_x) {
          distance = cluster.min_x - edge_max_x;
        } else if (edge_min_x > cluster.max_x) {
          distance = edge_min_x - cluster.max_x;
        }
        if (distance <= overlap_threshold) {
          cluster.road_edges.push_back(edge);
          // SD_BEV_PROCESS << fmt::format("Edge ID: {} bound to cluster with x-range [{}, {}], edge x-range [{}, {}], distance: {:.3f}m",
          //                               edge->id, cluster.min_x, cluster.max_x, edge_min_x, edge_max_x, distance);
        } else {
          SD_BEV_PROCESS << fmt::format("Edge ID: {} not bound to cluster with x-range [{}, {}], edge x-range [{}, {}], distance: {:.3f}m",
                                        edge->id, cluster.min_x, cluster.max_x, edge_min_x, edge_max_x, distance);
        }
      }
    }
  }

  return clusters;
}

// 基于距离约束过滤被阻挡的车道
void BevDataProcessor::FilterBlockedLanesWithDistance(std::vector<BevLaneInfo *> &bev_candidate_lanes, BevRouteInfo &complete_section_info,
                                                      double max_distance) {
  // 统计每个 section 的车道数量
  std::unordered_map<uint64_t, int> lane_to_section_index;
  std::vector<int>                  section_lane_counts(complete_section_info.sections.size(), 0);
  for (size_t i = 0; i < complete_section_info.sections.size(); ++i) {
    for (const auto &lane_id : complete_section_info.sections[i].lane_ids) {
      auto [it, inserted] = lane_to_section_index.emplace(lane_id, i);
      if (!inserted) {
        SD_BEV_PROCESS << fmt::format("Error: Lane ID {} appears in multiple sections ({} and {}). Using first occurrence.", lane_id,
                                      it->second, i);
        continue;
      }
      section_lane_counts[i]++;
    }
  }

  // 过滤车道，保留 section 中唯一的车道
  std::vector<BevLaneInfo *> filtered_lanes;
  for (auto lane : bev_candidate_lanes) {
    bool should_filter = lane->is_blocked && lane->geos && !lane->geos->empty() && lane->geos->back().x() < max_distance;
    if (should_filter) {
      auto it = lane_to_section_index.find(lane->id);
      if (it != lane_to_section_index.end() && section_lane_counts[it->second] == 1) {
        SD_BEV_PROCESS << fmt::format("Retaining blocked lane ID: {} as it is the only lane in section {}.", lane->id, it->second);
        filtered_lanes.push_back(lane);
        continue;
      }
      SD_BEV_PROCESS << fmt::format("Filtering blocked lane ID: {} (end x: {} < {})", lane->id, lane->geos->back().x(), max_distance);
      continue;
    }
    filtered_lanes.push_back(lane);
  }
  bev_candidate_lanes = std::move(filtered_lanes);

  // 更新 bev_sections，移除已删除车道的 ID
  for (auto &section : complete_section_info.sections) {
    section.lane_ids.erase(std::remove_if(section.lane_ids.begin(), section.lane_ids.end(),
                                          [&](uint64_t id) {
                                            return std::find_if(bev_candidate_lanes.begin(), bev_candidate_lanes.end(),
                                                                [id](const BevLaneInfo *lane) { return lane->id == id; }) ==
                                                   bev_candidate_lanes.end();
                                          }),
                           section.lane_ids.end());
    // Todo：block车道是否在section里删除还是在下游删除看后续需求，暂时在section移除，删除后，更新车道数
    section.lane_num = section.lane_ids.size();
    // length, start_range_offset, end_range_offset 不变
  }

  // 移除空的 section, 因为可能存在删除block车道后，section为空的情况
  complete_section_info.sections.erase(std::remove_if(complete_section_info.sections.begin(), complete_section_info.sections.end(),
                                                      [](const BevSectionInfo &sec) { return sec.lane_ids.empty(); }),
                                       complete_section_info.sections.end());
}

// 获取感知车前的第一个路口
JunctionRange BevDataProcessor::GetFirstFrontJunction(const std::vector<BevLaneMarker> &bev_junctions) {
  JunctionRange first_front_junction;
  double        min_x_min = std::numeric_limits<double>::max();

  for (const auto &junction : bev_junctions) {
    if (junction.geos && !junction.geos->empty()) {
      double x_min       = std::numeric_limits<double>::max();
      double x_max       = std::numeric_limits<double>::lowest();
      double x_sum       = 0.0;
      size_t point_count = junction.geos->size();

      for (const auto &pt : *junction.geos) {
        double x = static_cast<double>(pt.x());
        x_min    = std::min(x_min, x);
        x_max    = std::max(x_max, x);
        x_sum += x;
      }

      double center_x = x_sum / point_count;
      if (x_min > 0 && x_min < min_x_min) {
        min_x_min                     = x_min;
        first_front_junction.x_min    = x_min;
        first_front_junction.x_max    = x_max;
        first_front_junction.center_x = center_x;
        first_front_junction.valid    = true;
      }
    }
  }

  return first_front_junction;
}

bool BevDataProcessor::IsWithinJunctionRange(const BevLaneInfo &lane, const JunctionRange &junction) {
  if (!lane.geos || lane.geos->empty()) {
    SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: no geometry.", lane.id);
    return false;
  }

  if (!junction.valid) {
    // SD_BEV_PROCESS << fmt::format("junction is invalid");
    return true;
  }
  auto full_geos = GetFullLaneGeometry(&lane);
  if (full_geos && !full_geos->empty()) {
    double lane_start_x = full_geos->front().x();
    // SD_BEV_PROCESS << fmt::format("Lane ID: {} lane_start_x={:.3f}", lane.id, lane_start_x);
    if (lane_start_x < junction.center_x) {
      return true;
    } else {
      SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: lane_start_x={:.3f} >= junction.center_x={:.3f}", lane.id, lane_start_x,
                                    junction.center_x);
      return false;
    }
  }
  SD_BEV_PROCESS << fmt::format("Lane ID: {} filtered: full_geos empty or null.", lane.id);
  return false;
}

bool BevDataProcessor::IsMergeZoneByJunction(const BevLaneMarker &diversion_zone, const std::vector<JunctionInfoCity> &nearby_RoadMerges) {
  if (nearby_RoadMerges.empty()) {
    return false;
  }

  double zone_x_max =
      std::max_element(diversion_zone.geos->begin(), diversion_zone.geos->end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
        return a.x() < b.x();
      })->x();

  for (const auto &merge_junction : nearby_RoadMerges) {
    if (std::abs(zone_x_max - merge_junction.offset) < 20.0) {
      SD_BEV_PROCESS << fmt::format("merge_junction :{}, offset: {}, zone_x_max: {}.", merge_junction.junction_id, merge_junction.offset,
                                    zone_x_max);
      return true;
    }
  }
  return false;
}

bool BevDataProcessor::IsSplitZoneByJunction(const BevLaneMarker &diversion_zone, const std::vector<JunctionInfoCity> &nearby_RoadSplits) {
  if (nearby_RoadSplits.empty()) {
    return false;
  }

  double zone_x_max =
      std::max_element(diversion_zone.geos->begin(), diversion_zone.geos->end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
        return a.x() < b.x();
      })->x();
  double zone_x_min =
      std::max_element(diversion_zone.geos->begin(), diversion_zone.geos->end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) {
        return a.x() > b.x();
      })->x();

  for (const auto &merge_junction : nearby_RoadSplits) {
    if (std::abs(zone_x_max - merge_junction.offset) < 50.0 || std::abs(zone_x_min - merge_junction.offset) < 50.0) {
      SD_BEV_PROCESS << fmt::format("split_junction :{}, offset: {}, zone_x_max: {}, zone_x_min: {}.", merge_junction.junction_id,
                                    merge_junction.offset, zone_x_max, zone_x_min);
      return true;
    }
  }
  return false;
}

bool BevDataProcessor::IsMergeZoneByLaneSpacing(const BevLaneMarker &virtual_edge, const std::vector<LineSortCity> &sorted_lines,
                                                const std::vector<BevLaneInfo *> &bev_candidate_lanes) {
  // Find the virtual edge in sorted_lines
  auto it = std::find_if(sorted_lines.begin(), sorted_lines.end(),
                         [&](const LineSortCity &sort) { return sort.id == virtual_edge.id && sort.type == LineSortType::VirtualEdge; });
  if (it == sorted_lines.end()) {
    SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} not found in sorted_lines.", virtual_edge.id);
    return false;
  }

  size_t edge_index = std::distance(sorted_lines.begin(), it);

  // Find adjacent lanes (left and right)
  const BevLaneInfo *left_lane  = nullptr;
  const BevLaneInfo *right_lane = nullptr;

  // Search left
  for (int i = edge_index - 1; i >= 0; --i) {
    if (sorted_lines[i].type == LineSortType::LaneLine) {
      left_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(sorted_lines[i].id);
      break;
    }
  }

  // Search right
  for (size_t i = edge_index + 1; i < sorted_lines.size(); ++i) {
    if (sorted_lines[i].type == LineSortType::LaneLine) {
      right_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(sorted_lines[i].id);
      break;
    }
  }

  if (!left_lane || !right_lane) {
    SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} lacks adjacent lanes (left: {}, right: {}).", virtual_edge.id,
                                  left_lane ? "found" : "missing", right_lane ? "found" : "missing");
    return false;
  }

  // Calculate far-end evaluation point
  double x_max  = std::max(virtual_edge.geos->front().x(), virtual_edge.geos->back().x());

  // Retrieve full lane geometries
  auto left_geos  = GetFullLaneGeometry(left_lane);
  auto right_geos = GetFullLaneGeometry(right_lane);

  // Check if lanes extend to eval_x
  double eval_x = x_max + 20.0;  // 20m beyond the virtual edge
  double left_max_x  = left_geos->back().x();
  double right_max_x = right_geos->back().x();
  eval_x = std::max(eval_x, std::min(left_max_x, right_max_x));  
  if (left_max_x < eval_x || right_max_x < eval_x) {
    SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} - lanes do not extend to eval_x={:.3f} (left_max_x={:.3f}, right_max_x={:.3f}).",
                                  virtual_edge.id, eval_x, left_max_x, right_max_x);
    return false;
  }
 

  // Interpolate y-coordinates at eval_x
  double y_left  = InterpolateYAtX(*left_geos, eval_x);
  double y_right = InterpolateYAtX(*right_geos, eval_x);
  if (std::isnan(y_left) || std::isnan(y_right)) {
    SD_BEV_PROCESS << fmt::format("Virtual Edge ID: {} - failed to interpolate y at x={:.3f} for lanes (left ID: {}, right ID: {}).",
                                  virtual_edge.id, eval_x, left_lane->id, right_lane->id);
    return false;
  }

  double       spacing        = std::abs(y_left - y_right);
  const double min_lane_width = 2.5;
  const double max_lane_width = 4.5;

  SD_BEV_PROCESS << fmt::format(
      "Virtual Edge ID: {} evaluated at x={:.3f}: left_lane ID: {}, y={:.3f}, right_lane ID: {}, y={:.3f}, spacing={:.3f}", virtual_edge.id,
      eval_x, left_lane->id, y_left, right_lane->id, y_right, spacing);

  return (spacing >= min_lane_width && spacing <= max_lane_width);
}

double BevDataProcessor::InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x) noexcept {
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

std::pair<uint64_t, std::unordered_set<uint64_t>> BevDataProcessor::findEgoAndSuccessorLanes(
    const std::vector<BevLaneInfo> &all_bev_lane_infos) {
  uint64_t                     ego_lane_id = 0;
  std::unordered_set<uint64_t> successor_ids;
  for (const auto &lane : all_bev_lane_infos) {
    if (lane.position == static_cast<int>(BevLanePosition::LANE_LOC_EGO)) {
      ego_lane_id = lane.id;
      successor_ids.insert(lane.next_lane_ids.begin(), lane.next_lane_ids.end());
      break;
    }
  }
  return {ego_lane_id, successor_ids};
}

void BevDataProcessor::ensureEgoLaneInSingleSection(BevRouteInfo &complete_section_info) {
  if (complete_section_info.sections.size() != 1) {
    SD_BEV_PROCESS << fmt::format("Sections size is {}, skipping ego lane addition.", complete_section_info.sections.size());
    return;
  }

  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    AWARN << "sd_route is null";
    SD_BEV_PROCESS << "sd_route is null";
    return;
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "mpp_sections is empty";
    SD_BEV_PROCESS << "mpp_sections is empty";
    return;
  }

  uint64_t current_section_id = sd_route->navi_start.section_id;
  SD_BEV_PROCESS << "current_section_id: " << current_section_id;

  const SDSectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "current section not found";
    SD_BEV_PROCESS << "current section not found";
    return;
  }

  if (current_section->direction == SDDirectionType::BIDIRECTIONAL_PASSABLE && current_section->lane_num <= 2) {
    SD_BEV_PROCESS << fmt::format("Current Section is BIDIRECTIONAL_PASSABLE and lane num is {}, skipping ego lane addition.",
                                  current_section->lane_num);
    return;
  }

  auto &section = complete_section_info.sections[0];

  const auto &ego_related_ids = bev_ego_lane_related_;
  if (ego_related_ids.empty()) {
    SD_BEV_PROCESS << "bev_ego_lane_related_ is empty, no action needed.";
    return;
  }

  // 检查 section.lane_ids 是否与 bev_ego_lane_related_ 有交集
  bool has_related_lane = false;
  for (const auto &lane_id : section.lane_ids) {
    if (ego_related_ids.count(lane_id)) {
      has_related_lane = true;
      break;
    }
  }

  // 如果 section 已包含 bev_ego_lane_related_ 中的车道，则不执行操作
  if (has_related_lane) {
    SD_BEV_PROCESS << "Section already contains at least one lane from bev_ego_lane_related_, no update needed.";
    return;
  }

  // 添加没有后继并且不在现section里面的车道id
  for (const auto &lane_id : ego_related_ids) {
    auto lane_info = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (lane_info && lane_info->next_lane_ids.empty()) {
      if (std::find(section.lane_ids.begin(), section.lane_ids.end(), lane_id) == section.lane_ids.end()) {
        section.lane_ids.push_back(lane_id);
        SD_BEV_PROCESS << fmt::format("Added lane ID: {} (no successors) to the single section.", lane_id);
      }
    }
  }

  std::unordered_map<uint64_t, double> y_at_zero;
  for (uint64_t lane_id : section.lane_ids) {
    auto lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
    if (lane && lane->geos && !lane->geos->empty()) {
      auto                full_geos = GetFullLaneGeometry(lane);
      std::vector<double> geo_x_vec, geo_y_vec;
      for (const auto &pt : *full_geos) {
        geo_x_vec.push_back(pt.x());
        geo_y_vec.push_back(pt.y());
      }
      LaneGeometry::PolynomialFitting fitting = LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3);
      double                          y       = fitting.GetValue(0.0);
      y_at_zero[lane_id]                      = y;
    }
  }

  // 根据 y_at_zero 对 section.lane_ids 进行排序
  std::sort(section.lane_ids.begin(), section.lane_ids.end(), [&](uint64_t id1, uint64_t id2) { return y_at_zero[id1] > y_at_zero[id2]; });

  section.lane_num = section.lane_ids.size();
  SD_BEV_PROCESS << fmt::format("Updated section lane_num: {}", section.lane_num);
  SD_BEV_PROCESS << fmt::format("Sorted lane IDs: [{}]", fmt::join(section.lane_ids, ", "));
}

bool BevDataProcessor::AreSeparatorsEqual(const std::vector<SeparatorInfo> &sep1, const std::vector<SeparatorInfo> &sep2) {
  if (sep1.size() != sep2.size())
    return false;
  for (size_t i = 0; i < sep1.size(); ++i) {
    if (sep1[i].edge_ids != sep2[i].edge_ids || sep1[i].types != sep2[i].types)
      return false;
  }
  return true;
}

void BevDataProcessor::SmoothOutputBasedOnSeparators(BevRouteInfo &current_section_info) {
  std::vector<SeparatorInfo> current_separators = current_section_info.separators;

  // 将当前分割器添加到历史记录
  history_separators_.push_back(current_separators);
  if (history_separators_.size() > consistency_threshold_) {
    history_separators_.pop_front();
  }

  // 检查最近三帧的分割器是否一致
  if (history_separators_.size() == consistency_threshold_) {
    bool consistent = true;
    for (size_t i = 1; i < consistency_threshold_; ++i) {
      if (!AreSeparatorsEqual(history_separators_[i], history_separators_[0])) {
        consistent = false;
        break;
      }
    }
    if (consistent) {
      last_confirmed_section_info_ = current_section_info;
    } else {
      current_section_info = last_confirmed_section_info_;
    }
  } else {
    // 对于初始帧，使用当前 section 信息
    last_confirmed_section_info_ = current_section_info;
  }
}

void BevDataProcessor::CalculateSectionEdgeDistances(BevRouteInfo &complete_section_info, const std::vector<LineSortCity> &line_sorts,
                                                     const std::vector<BevLaneMarker> &virtual_edges) {
  if (complete_section_info.sections.empty()) {
    return;
  }

  auto yAtX = [&](const BevLaneInfo *lane, double x_ref = 0.0) -> double {
    if (!lane || !lane->geos || lane->geos->empty())
      return std::numeric_limits<double>::quiet_NaN();
    double y = InterpolateYAtX(*lane->geos, x_ref);
    if (!std::isfinite(y))
      y = lane->geos->front().y();
    return y;
  };

  auto pickExtremePredecessor = [&](const BevLaneInfo *cur, bool want_leftmost) -> const BevLaneInfo * {
    if (!cur)
      return nullptr;
    if (cur->previous_lane_ids.empty())
      return cur;

    const BevLaneInfo *best   = nullptr;
    double             best_y = want_leftmost ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();

    for (uint64_t pid : cur->previous_lane_ids) {
      auto cand = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(pid);
      if (!cand || !cand->geos || cand->geos->empty())
        continue;
      double y = yAtX(cand, 0.0);
      if (!std::isfinite(y))
        continue;

      if ((want_leftmost && y > best_y) || (!want_leftmost && y < best_y)) {
        best_y = y;
        best   = cand;
      }
    }
    return best ? best : cur;
  };

  std::unordered_map<uint64_t, size_t> lane_to_sort_index;
  for (size_t i = 0; i < line_sorts.size(); ++i) {
    if (line_sorts[i].type == LineSortType::LaneLine) {
      lane_to_sort_index[line_sorts[i].id] = i;
    }
  }

  for (auto &section : complete_section_info.sections) {
    if (section.lane_ids.empty()) {
      continue;
    }

    // Find the leftmost and rightmost lanes in the section
    size_t leftmost_lane_index  = std::numeric_limits<size_t>::max();
    size_t rightmost_lane_index = 0;
    for (const auto &lane_id : section.lane_ids) {
      auto it = lane_to_sort_index.find(lane_id);
      if (it != lane_to_sort_index.end()) {
        leftmost_lane_index  = std::min(leftmost_lane_index, it->second);
        rightmost_lane_index = std::max(rightmost_lane_index, it->second);
      }
    }

    if (leftmost_lane_index == std::numeric_limits<size_t>::max()) {
      continue;
    }

    // Find the nearest left edge (RoadBoundary or VirtualEdge)
    int nearest_left_edge_index = -1;
    for (int j = 0; j < static_cast<int>(line_sorts.size()); ++j) {
      if ((line_sorts[j].type == LineSortType::RoadBoundary || line_sorts[j].type == LineSortType::VirtualEdge) &&
          j < static_cast<int>(leftmost_lane_index)) {
        nearest_left_edge_index = std::max(nearest_left_edge_index, j);
      }
    }

    // Find the nearest right edge (RoadBoundary or VirtualEdge)
    int nearest_right_edge_index = -1;
    for (int j = 0; j < static_cast<int>(line_sorts.size()); ++j) {
      if ((line_sorts[j].type == LineSortType::RoadBoundary || line_sorts[j].type == LineSortType::VirtualEdge) &&
          j > static_cast<int>(rightmost_lane_index)) {
        if (nearest_right_edge_index == -1 || j < nearest_right_edge_index) {
          nearest_right_edge_index = j;
        }
      }
    }

    // Calculate left distance
    double left_distance = 0.0;
    if (nearest_left_edge_index != -1) {
      const auto &left_sort         = line_sorts[nearest_left_edge_index];
      uint64_t    left_edge_id      = left_sort.id;
      uint64_t    base_left_lane_id = line_sorts[leftmost_lane_index].id;
      auto        base_left_lane    = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(base_left_lane_id);
      if (base_left_lane) {
        const BevLaneInfo                            *left_rep       = pickExtremePredecessor(base_left_lane, /*want_leftmost=*/true);
        auto                                          left_lane_geos = GetFullLaneGeometry(left_rep ? left_rep : base_left_lane);
        std::shared_ptr<std::vector<Eigen::Vector2f>> left_edge_geos;
        if (left_sort.type == LineSortType::RoadBoundary) {
          auto left_edge = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(left_edge_id);
          if (left_edge) {
            left_edge_geos = left_edge->geos;
          }
        } else if (left_sort.type == LineSortType::VirtualEdge) {
          auto it = std::find_if(virtual_edges.begin(), virtual_edges.end(),
                                 [left_edge_id](const BevLaneMarker &e) { return e.id == left_edge_id; });
          if (it != virtual_edges.end()) {
            left_edge_geos = it->geos;
          }
        }
        SD_BEV_PROCESS << fmt::format("LeftDist use lane {} (rep:{}) vs edge {}.", base_left_lane_id,
                                      left_rep ? left_rep->id : base_left_lane_id, left_edge_id);
        SD_COARSE_MATCH_TYPE2_LOG << fmt::format("LeftDist use lane {} (rep:{}) vs edge {}.", base_left_lane_id,
                                                 left_rep ? left_rep->id : base_left_lane_id, left_edge_id);
        if (left_edge_geos && left_lane_geos) {
          left_distance = LaneGeometry::GetDistanceBetweenLinesOverX(*left_edge_geos, *left_lane_geos);
        }
      }
      SD_BEV_PROCESS << fmt::format("Calculate left distance: Left lane ID: {}, Left edge ID: {}, Distance: {}", base_left_lane_id,
                                    left_edge_id, left_distance);
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Calculate left distance: Left lane ID: {}, Left edge ID: {}, Distance: {}",
                                               base_left_lane_id, left_edge_id, left_distance);
    }

    // Calculate right distance
    double right_distance = 0.0;
    if (nearest_right_edge_index != -1) {
      const auto &right_sort         = line_sorts[nearest_right_edge_index];
      uint64_t    right_edge_id      = right_sort.id;
      uint64_t    base_right_lane_id = line_sorts[rightmost_lane_index].id;
      auto        base_right_lane    = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(base_right_lane_id);
      if (base_right_lane) {
        const BevLaneInfo *right_rep       = pickExtremePredecessor(base_right_lane, /*want_leftmost=*/false);
        auto               right_lane_geos = GetFullLaneGeometry(right_rep ? right_rep : base_right_lane);

        std::shared_ptr<std::vector<Eigen::Vector2f>> right_edge_geos;
        if (right_sort.type == LineSortType::RoadBoundary) {
          auto right_edge = INTERNAL_PARAMS.raw_bev_data.GetRoadEdgeById(right_edge_id);
          if (right_edge) {
            right_edge_geos = right_edge->geos;
          }
        } else if (right_sort.type == LineSortType::VirtualEdge) {
          auto it = std::find_if(virtual_edges.begin(), virtual_edges.end(),
                                 [right_edge_id](const BevLaneMarker &e) { return e.id == right_edge_id; });
          if (it != virtual_edges.end()) {
            right_edge_geos = it->geos;
          }
        }
        SD_BEV_PROCESS << fmt::format("RightDist use lane {} (rep:{}) vs edge {}.", base_right_lane_id,
                                      right_rep ? right_rep->id : base_right_lane_id, right_edge_id);
        SD_COARSE_MATCH_TYPE2_LOG << fmt::format("RightDist use lane {} (rep:{}) vs edge {}.", base_right_lane_id,
                                                 right_rep ? right_rep->id : base_right_lane_id, right_edge_id);
        if (right_edge_geos && right_lane_geos) {
          right_distance = LaneGeometry::GetDistanceBetweenLinesOverX(*right_edge_geos, *right_lane_geos);
        }
      }
      SD_BEV_PROCESS << fmt::format("Calculate right distance: Right lane ID: {}, Right edge ID: {}, , Distance: {}", base_right_lane_id,
                                    right_edge_id, right_distance);
      SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Calculate right distance: Right lane ID: {}, Right edge ID: {}, , Distance: {}",
                                               base_right_lane_id, right_edge_id, right_distance);
    }

    section.left_edge_distance  = left_distance;
    section.right_edge_distance = right_distance;
  }
}

int BevDataProcessor::CountLanesPassedZero() const {
  auto raw_bev_map_ptr = INTERNAL_PARAMS.raw_bev_data.GetRawBevMapPtr();
  if (!raw_bev_map_ptr)
    return 0;
  int cnt = 0;
  for (const auto &lane : raw_bev_map_ptr->lane_infos) {
    if (!lane.geos || lane.geos->empty())
      continue;
    const auto &p0 = lane.geos->front();
    const auto &p1 = lane.geos->back();
    if (p0.x() < 0.0 && p1.x() > 0.0) {
      ++cnt;
    }
  }
  return cnt;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem