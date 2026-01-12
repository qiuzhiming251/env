#ifndef SD_MAP_TOPO_EXTRACT_H_
#define SD_MAP_TOPO_EXTRACT_H_

#include <vector>
#include <common/utils/lane_geometry.h>
#include "base/params_manager/params_defination/internal_params.h"
#include "base/sensor_data_manager.h"
#include "message/env_model/navigation/navigation.h"
#include "message/env_model/routing_map/routing_map.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "lib/perception_and_ld_map_fusion/fusion_manager.h"

namespace cem {
namespace fusion {
namespace navigation {
struct MergeTrack {
  MergeTopoExtendType stable    = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
  int                 hits      = 0;
  int                 misses    = 0;
  double              last_dist = std::numeric_limits<double>::infinity();
  Eigen::Isometry3d   last_T_ego = Eigen::Isometry3d::Identity();
  std::shared_ptr<std::vector<Eigen::Vector2f>> last_geos        = std::make_shared<std::vector<Eigen::Vector2f>>();
};
class SDMapTopologyExtractor {
 public:
  SDMapTopologyExtractor();

  void GetMergeTopologiesFromMap(std::vector<MergeDetail> &merge_details);
  void GetMergeTopologiesFromSDMap(std::vector<MergeDetail> &merge_details);
  void GetMergeTopologiesFromLDMap(std::vector<MergeDetail> &merge_details);

  void SetBevLaneMergeTopoUseMap(const std::vector<MergeDetail> &sd_merge_details, BevMapInfo &bev_map);
  void SetBevLaneMergeTopoUseLDMap(const std::vector<MergeDetail> &ld_merge_details, BevMapInfo &bev_map);
  void SetBevLaneMergeTopoUseSDMap(const std::vector<MergeDetail> &sd_merge_details, BevMapInfo &bev_map);
  void PrintMergeDetails(const std::vector<MergeDetail> &merge_details);
  bool IsInLowPrecisionZone(const double merge_dist);
  void SetBevLaneMergeTopoNew(const RoutingMapPtr routing_map_ptr, BevMapInfo &bevMap);
  void CheckBevMergeTopo(const RoutingMapPtr routing_map_ptr, BevMapInfo &bevMap, std::map<uint64_t, MergeTopoInfo>& bev_matched_merge_infos);
  std::optional<Eigen::Isometry3d> GetTransform(const double& timestamp);

  bool IsValidLaneForMergeAssignment(const BevLaneInfo &lane);

  bool IsShortMessLane(const BevLaneInfo &lane);

  // 获取从自车位置到路口最后一个 lanegroup 的虚拟 section 列表
  std::vector<VirtualSection> ExtractVirtualSections(const JunctionInfoCity &junction_info);

  uint64_t GetCurrentLaneGroupId() const;

  inline void SetIsHighway(bool v) { is_on_highway_ = v; }

  std::vector<std::vector<uint64_t>> ExtractNonNaviLanesFromJunction(const JunctionInfo *junction_info_ptr) const;

  //Todo---后续待拓展
  // void GetSplitTopologies();
  // void GetLaneLevelTopologies();
 private:
  // ========= SD 专用实现 =========
  double GetEgoGlobalS(const SDRouteInfo &route_info) const;

  double GetSectionStartS(const SDSectionInfo &section, const SDRouteInfo &route_info) const;

  // 从 lanegroup 逆向追踪目标车道索引
  std::vector<int> TraceTargetLaneIndices(const SDLaneGroupInfo &prev_lg, const SDLaneGroupInfo &current_lg,
                                          const std::vector<int> &target_indices);

  // ========= LD 专用实现（新增） =========
  int    CountNonEmergencyLanesLD(uint64_t section_id) const;
  double GetSectionStartS(const cem::message::env_model::SectionInfo &sec, const cem::message::env_model::RouteInfo &ld_route) const;
  double GetEgoGlobalS(const cem::message::env_model::RouteInfo &ld_route) const;
  std::pair<double, LaneDetail> LD_BacktrackKeepTopoStart(uint64_t lane_id_merge) const;

  void SetBevLaneMergeTopoUseLDMapIndexBased(const MergeDetail &closest_merge, const std::vector<uint64_t> &sorted_bev_lane_ids,
                                             const std::unordered_map<uint64_t, BevLaneInfo *> &bev_lane_map);

  bool BuildLDTargetPolylineFromMerge(const MergeDetail &md, std::vector<Eigen::Vector2d> &out_pts) const;

  void SetMergeHysteresisParams(int promote, int demote, double first_promote_min_dist);
  void ResetHysteresisOnSceneChange(const std::vector<uint64_t> &current_bev_ids);

  void ReflectStableToBevLane(uint64_t bev_id, cem::message::sensor::BevLaneInfo *lane, const MergeTrack &trk, MergeSourceExtend src);

  static inline const char *StrMergeTopoExtendType(MergeTopoExtendType t) {
    switch (t) {
      case MergeTopoExtendType::TOPOLOGY_MERGE_LEFT:
        return "LEFT";
      case MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT:
        return "RIGHT";
      case MergeTopoExtendType::TOPOLOGY_MERGE_NONE:
        return "NONE";
      default:
        return "UNKNOWN";
    }
  }

 private:
  double merge_rear_distance_  = 0.0;
  double merge_front_distance_ = 500.0;
  bool   is_on_highway_        = false;

  uint64_t                                 last_matched_bev_id_ = 0;
  std::unordered_map<uint64_t, MergeTrack> bev_merge_trackers_;
  int                                      promote_hits_needed_    = 2;  // 无→有：至少连续命中 2 帧才提升
  int                                      demote_miss_allowed_    = 10;  // 有→无：最多允许连续丢失 5 帧再降级
  double                                   first_promote_min_dist_ = 100.0;
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // SD_MAP_TOPO_EXTRACT_H_