#pragma once

#include "lib/message/sensor/camera/bev_lane/bev_lane.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/base_filleting_machine.h"

namespace cem {
namespace fusion {

class BevFilletingMachine
    : public FilletingMachine<cem::message::sensor::BevMapInfo> {
 private:
  /* data */

 public:
  BevFilletingMachine(double slice_thickness = 1.0);
  ~BevFilletingMachine();
  const std::unordered_map<uint64_t, std::vector<bool>>&
  GetSamplingPointStatus() const {
    return sampling_points_valid_;
  }
  bool IsLaneAatTheLeftofLaneB(uint64_t a, uint64_t b, bool& valid);

  bool IsLaneInReliableRange(uint64_t id);

 private:
  void Slice() override;
  void JudgeLanePosition(const BevLaneInfo& lane_info,
                         const Eigen::Vector2d& intsect_pt);
  std::map<BevRoadEdgePosition, Eigen::Vector2d> SliceRoadEdege();
  double DistanceEuclidean(const Eigen::Vector2d& P1,
                           const Eigen::Vector2d& P2);
  Eigen::Vector2d bev_slice_knife_direction_;
  Eigen::Vector2d bev_slice_start_point_;
  Eigen::Vector2d bev_slice_new_point_;

  double bev_slice_thickness_;
  double bev_slice_totallength_;

  std::unordered_map<uint64_t, std::vector<bool>> sampling_points_valid_;

  const std::unordered_map<BevLaneMarkerType, LineType> LineTypeToRoutingMap = {
      {BEV_LMT__UNDECIDED, LineType::UNKNOWN},
      {BEV_LMT__SOLID, LineType::SOLID},
      {BEV_LMT__DASHED, LineType::DASHED},
      {BEV_LMT__DOUBLE_SOLID_SOLID, LineType::SOLID_SOLID},
      {BEV_LMT__DOUBLE_DASHED_DASHED, LineType::DASHED_DASHED},
      {BEV_LMT__DOUBLE_SOLID_DASHED, LineType::SOLID_DASHED},
      {BEV_LMT__DOUBLE_DASHED_SOLID, LineType::DASHED_SOLID}};

  RoadSlice::LanePosition lane_position_;
  std::map<BevRoadEdgePosition, Eigen::Vector2d> edge_map_;
  std::unordered_set<uint64_t> bev_lanes_in_reliable_range_;
  static constexpr double RELIABLE_RANGE_DISTANCE_THRESHOLD = 80.0;
};

}  // namespace fusion
}  // namespace cem
