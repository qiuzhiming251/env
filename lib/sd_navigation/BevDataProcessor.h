#ifndef BEVDATAPROCESSOR_H
#define BEVDATAPROCESSOR_H

#include <map>
#include <set>
#include <vector>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include "common/utils/GeoMathUtil.h"
#include "common/utils/lane_geometry.h"
#include "lib/common/log_custom.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"

namespace cem {
namespace fusion {
namespace navigation {
struct JunctionRange {
  double x_min    = std::numeric_limits<double>::max();
  double x_max    = std::numeric_limits<double>::lowest();
  double center_x = 0.0;
  bool   valid    = false;
};
struct ClusterInfo {
  std::vector<BevLaneInfo *>   lanes;
  std::vector<BevLaneMarker *> road_edges;
  double                       min_x;
  double                       max_x;
};

class BevDataProcessor {
 public:
  BevDataProcessor();

  void Process(const std::vector<JunctionInfoCity> &junctions_info_city, BevMapInfoConstPtr &bev_map,
               std::vector<BevLaneInfo *> &bev_candidate_lanes, std::vector<BevLaneMarker *> &bev_candidate_road_edges,
               std::vector<LineSortCity> &bev_line_sorts, std::vector<int> &bev_left_road_edge_indexs,
               std::vector<int> &bev_right_road_edge_indexs, BevRouteInfo &complete_section_info,
               std::pair<int, int> &bev_egoRoadEdgeIndexPair, std::set<uint64_t> &bev_ego_lane_related,
               std::vector<uint64_t> &bev_ego_root_section_x0);

  void GetCandidateLanes(const std::vector<BevLaneInfo> &lane_infos_in, const std::vector<BevLaneMarker> &bev_junctions,
                         std::vector<BevLaneInfo *> &candidate_bev_lanes_out);
  void GetCandidateRoadEdges(const std::vector<BevLaneMarker> &bev_road_edges_in, const std::vector<BevLaneInfo *> &candidate_bev_lanes,
                             const std::vector<BevLaneMarker> &bev_junctions, std::vector<BevLaneMarker *> &candidate_bev_road_edges_out);

  void SortBevLaneInfoAndRoadEdges(const std::vector<BevLaneInfo *> &bev_lane_infos_in, const std::vector<BevLaneMarker *> &road_edges_in,
                                   std::vector<LineSortCity> &line_sorts, std::vector<int> &left_road_edge_indexs,
                                   std::vector<int> &right_road_edge_indexs);

  // void BevLanesFilterByRoadEdges(const std::vector<LineSortCity> &line_sorts, const std::vector<int> &left_road_edge_indexs,
  //                                const std::vector<int> &right_road_edge_indexs, std::vector<BevLaneInfo *> &candidateBevLanes);

  void BevLanesFilterByRoadEdges(const std::vector<LineSortCity> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                 const std::vector<int> &right_road_edge_indexs, std::vector<BevLaneInfo *> &candidateBevLanes,
                                 bool &filtered_non_ego_lanes);

  void DivideBevSectionsByRoadEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes, const std::vector<LineSortCity> &bev_line_sorts,
                                    BevRouteInfo &complete_section_info);

  void DivideBevSectionsByDiversionZones(const std::vector<BevLaneInfo *> &bev_candidate_lanes,
                                         const std::vector<BevLaneMarker> &virtualEdges, const std::vector<LineSortCity> &bev_line_sorts,
                                         BevRouteInfo &complete_section_info);

  void DivideBevSectionsByEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes, const std::vector<LineSortCity> &bev_line_sorts,
                                const std::unordered_map<uint32_t, BevLaneMarker *> &edge_map, BevRouteInfo &complete_section_info);

  std::pair<int, int> GetEgoLeftRightRoadEdges(const std::vector<LineSortCity> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                               const std::vector<int> &right_road_edge_indexs);

  std::set<uint64_t> GetEgoLaneRelated(const std::vector<BevLaneInfo> &all_bev_lane_infos);

  bool IsShortMessLane(const BevLaneInfo &lane);

  bool IsOppositeLane(const BevLaneInfo &lane);

  bool IsValidLane(const BevLaneInfo &lane);

  bool IsValidRoadEdge(const BevLaneMarker &edge);

  std::shared_ptr<std::vector<Eigen::Vector2f>> GetFullLaneGeometry(const BevLaneInfo *lane);

  std::shared_ptr<std::vector<Eigen::Vector2f>> GetFullLaneGeometry(const BevLaneInfo *lane, std::unordered_set<uint64_t> &visited_lanes,
                                                                    int &depth);

  Eigen::Vector2f CalculateMidpoint(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

  std::vector<BevLaneMarker> CreateVirtualRoadEdgesFromDiversionZones(const std::vector<BevLaneMarker> &diversion_zones);

  void SortLanesRelativeToVirtualEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes,
                                       const std::vector<BevLaneMarker> &virtual_road_edges, std::vector<LineSortCity> &bev_line_sorts);

  void InsertVirtualEdge(const BevLaneMarker &virtual_edge, std::vector<LineSortCity> &bev_line_sorts,
                         const std::vector<BevLaneInfo *> &bev_candidate_lanes);

  void ProcessDiversionZones(const std::vector<BevLaneInfo *> &bev_candidate_lanes, std::vector<std::vector<uint64_t>> &bev_sections,
                             const std::vector<BevLaneMarker> &virtual_road_edges);

  bool HasXOverlap(const BevLaneMarker &virtual_edge, const std::vector<BevLaneInfo *> &bev_candidate_lanes);

  int GenerateUniqueID();

  std::pair<double, double> CalculateLaneXRange(const BevLaneInfo *lane);
  double                    CalculateCrosswalkCenterX(const BevLaneMarker &crosswalk);

  std::vector<ClusterInfo> ClusterLanesByX(const std::vector<BevLaneInfo *>   &lanes,
                                           const std::vector<BevLaneMarker *> &bev_candidate_road_edges, double gap_threshold = 5.0);

  void FilterBlockedLanesWithDistance(std::vector<BevLaneInfo *> &bev_candidate_lanes, BevRouteInfo &complete_section_info,
                                      double max_distance);

  JunctionRange GetFirstFrontJunction(const std::vector<BevLaneMarker> &bev_junctions);

  bool IsWithinJunctionRange(const BevLaneInfo &lane, const JunctionRange &junction);

  bool IsSplitZoneByJunction(const BevLaneMarker &diversion_zone, const std::vector<JunctionInfoCity> &nearby_RoadSplits);

  bool IsMergeZoneByJunction(const BevLaneMarker &diversion_zone, const std::vector<JunctionInfoCity> &nearby_RoadMerges);

  bool IsMergeZoneByLaneSpacing(const BevLaneMarker &virtual_edge, const std::vector<LineSortCity> &sorted_lines,
                                const std::vector<BevLaneInfo *> &bev_candidate_lanes);

  // double InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x);
  static double InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x) noexcept;

  void ensureEgoLaneInSingleSection(BevRouteInfo &complete_section_info);

  std::pair<uint64_t, std::unordered_set<uint64_t>> findEgoAndSuccessorLanes(const std::vector<BevLaneInfo> &all_bev_lane_infos);

  bool AreSeparatorsEqual(const std::vector<SeparatorInfo> &sep1, const std::vector<SeparatorInfo> &sep2);

  void SmoothOutputBasedOnSeparators(BevRouteInfo &current_section_info);

  // void CalculateSectionEdgeDistances(BevRouteInfo &complete_section_info, const std::vector<LineSortCity> &bev_line_sorts,
  //                                    const std::vector<int> &bev_left_road_edge_indexs, const std::vector<int> &bev_right_road_edge_indexs);

  void CalculateSectionEdgeDistances(BevRouteInfo &complete_section_info, const std::vector<LineSortCity> &line_sorts,
                                     const std::vector<BevLaneMarker> &virtual_edges);

  void Clear();

  int CountLanesPassedZero() const;

  int GetpassedZeroLaneNum() const { return passedZeroLaneNum_; }

  std::vector<uint64_t> MergeSectionLaneIdsLeft2Right(const BevRouteInfo &complete_section_info) const;

  std::vector<uint64_t> FindRootsAtX0AndSort(const BevMapInfo &bev_map, const std::vector<uint64_t> &merged_lane_ids) const;

  bool HasCommonAncestor(const LineSortCity &l1, const LineSortCity &l2);

 private:
  std::map<uint64_t, int> last_intersect_bev_laneids_;  // To track intersecting lanes for filtering
  std::set<uint64_t>      bev_ego_lane_related_;
  std::unordered_map<uint64_t, std::shared_ptr<std::vector<Eigen::Vector2f>>> geometry_cache_;
  std::deque<std::vector<SeparatorInfo>>                                      history_separators_;
  const size_t                                                                consistency_threshold_ = 3;  // 要求连续3帧一致
  BevRouteInfo                                                                last_confirmed_section_info_;
  int                                                                         passedZeroLaneNum_ = 0;
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // BEVDATAPROCESSOR_H