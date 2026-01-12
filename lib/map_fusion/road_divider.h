#ifndef ROAD_DIVIDER_H_
#define ROAD_DIVIDER_H_

#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>

namespace cem {
namespace fusion {

class RoadDivider {
 public:
  RoadDivider() = default;

  void Process(BevMapInfo& bev_map);

  void FindEgoLaneIds(BevMapInfo& bev_map);

  bool FindDiversionRouteAndSubpath(BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections);

  Eigen::Isometry3d T_local_ego_;

  bool is_highway_;

 private:
  std::set<uint64_t> ego_lane_ids_;

};

}  // namespace fusion
}  // namespace cem

#endif  // ROAD_DIVIDER_H_
