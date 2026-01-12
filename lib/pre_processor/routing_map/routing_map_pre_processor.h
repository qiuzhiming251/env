#ifndef ROUTING_MAP_PRE_PROCESSOR_H
#define ROUTING_MAP_PRE_PROCESSOR_H

#include <base/params_manager/params_manager.h>

#include "base/sensor_data_manager.h"
#include "common/CommonDataType.h"
namespace cem {
namespace fusion {
const double SAMPING_INTERVAL     = 1.0;
const float  MAX_SAMPING_INTERVAL = 500.0;
class RoutingMapPreProcessor {
 public:
  RoutingMapPreProcessor();
  ~RoutingMapPreProcessor();
  void Process(const DetectBevMap &bev_map, RoutingMapPtr ld_map = nullptr);
  RoutingMapPtr GetRoutingMap();

 private:
  void InsertLanePoint();

  void InsertLaneCenter();

  void InsertLaneBoundary();

  void BevReplaceMapEdge(const DetectBevMap &bev_map);

  void BoundBoundaryWithLane(const DetectBevMap &bev_map);

  void LDMapInfoProcess();

  void RoutingMapTransToWorld();

  std::optional<Eigen::Isometry3d> FindTransform(const double &timestamp);

  SDRouteInfoPtr sd_route_Info_ptr_{nullptr};

  RoutingMapPtr input_routing_map_ptr_{nullptr};

  RoutingMapPtr output_routing_map_ptr_{nullptr};
};

}  // namespace fusion
}  // namespace cem

#endif
