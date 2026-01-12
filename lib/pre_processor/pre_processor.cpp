#include "pre_processor.h"

namespace cem {
namespace fusion {

PreProcessor::PreProcessor(std::shared_ptr<LandmarkBuffer> buffer) : landmark_buffer_(buffer) {
  bev_map_pre_processor_.reset(new BevMapPreProcessor(buffer));
  routing_map_pre_processor_.reset(new RoutingMapPreProcessor());
}

void PreProcessor::Process(BevMapInfoPtr bev_map, RoutingMapPtr ld_map) {
  bev_map_pre_processor_->Init();
  bev_map_pre_processor_->Process(bev_map);
/*
  bev_map_pre_processor_->GetCrossRoadFlag(is_cross_road_status_);
  bev_map_pre_processor_->GetCrossRoadDistance(cross_road_distance_);
*/

  routing_map_pre_processor_->Process(
      bev_map_pre_processor_->GetGlobalBevMapPtr(), ld_map);
}

void PreProcessor::SetRightTurnTrajectory(double timestamp, const std::vector<Eigen::Vector2f> &trajectory) {
  bev_map_pre_processor_->SetRightTurnTrajectory(timestamp, trajectory);
}

void PreProcessor::SetTraverseCrossWalkLaneInfo(
    std::vector<traverseCrossWalkLane>& crosswalk_lane_list) {
  bev_map_pre_processor_->SetTraverseCrossWalkLaneInfo(crosswalk_lane_list);
}

}  // namespace fusion
}  // namespace cem
