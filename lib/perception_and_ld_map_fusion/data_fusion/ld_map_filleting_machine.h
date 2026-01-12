#pragma once

#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/base_filleting_machine.h"

namespace cem {
namespace fusion {

class LdMapFilletingMachine
    : public FilletingMachine<cem::message::env_model::RoutingMap> {
 private:
  /* data */

 public:
  LdMapFilletingMachine(double slice_thickness = 1.0);
  ~LdMapFilletingMachine();

 private:
  void Slice() override;
  void JudgeLanePosition(const LaneInfo& lane_info,
                         const Eigen::Vector2d& intsect_pt);
  Eigen::Vector2d ldmap_slice_knife_direction_;
  Eigen::Vector2d ldmap_slice_start_point_;
  double ldmap_slice_thickness_;
  double ldmap_slice_totallength_;
  RoadSlice::LanePosition lane_position_;
};

}  // namespace fusion
}  // namespace cem