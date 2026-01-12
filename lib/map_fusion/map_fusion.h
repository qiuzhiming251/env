#ifndef MAP_FUSION_H_
#define MAP_FUSION_H_

#include "base/sensor_data_manager.h"
#include "common/CommonDataType.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"

namespace cem {
namespace fusion {

enum class LaneSide { LEFT, RIGHT }; 

class MapFusion {
 public:
  MapFusion();
  void Process(const BevMapInfoPtr &bev_map_ptr, RoutingMapPtr &routing_map_ptr);
  void SetGeometryMatchInfo(uint64_t bev_ego_lane_id, std::vector<uint64_t> ld_ego_lane_ids) {
    if(bev_ego_lane_id==0||ld_ego_lane_ids.empty()) return;
    ld_ego_lane_ids_ = ld_ego_lane_ids;
    bev_ego_lane_id_ = bev_ego_lane_id;
  }

 private:
  void UseBevLmAttrTypeInsteadOfHdmap(const BevMapInfo &bev_map, RoutingMapPtr routing_map_ptr);
  bool ReplaceHdmapAttrSegWithBevAttrSeg(const BevLaneMarker *bev_lm, const LaneBoundaryInfo *route_lm,
                                         std::vector<std::unique_ptr<LaneBoundaryInfo>> &route_landmarkers);
  void UpdateLaneBoundaryBySide(uint32_t bev_lane_marker_id, const std::vector<BevLaneMarker> &bev_lane_markers, std::vector<LaneBoundaryInfo> &hdmap_lane_boundaries,
                                std::vector<LaneInfo> &hdmap_lanes, const LaneSide &side);
  bool FindCutPoint(const std::vector<Point2D> &route_points, const cem::message::common::Point2DF &position, int &best_index,
                    Point2D &point_out);
  void PrintBevLaneMarker(const BevLaneMarker &marker);
  void PrintLaneBoundaryInfo(const LaneBoundaryInfo& info);
  void UpdateRoutingMapArrows(const BevMapInfoPtr &bev_map_ptr, RoutingMapPtr &routing_map_ptr);

  void AssignLaneMarkerColor(BevMapInfo &bev_map);
  void AssignColor2LaneMarkerPts(BevMapInfo &bev_map);

 private:
  uint64_t bev_ego_lane_id_{0};
  std::vector<uint64_t> ld_ego_lane_ids_;
  std::atomic<u_int64_t> new_id_;
};

}  // namespace fusion
}  // namespace cem

#endif  // MAP_FUSION_H_
