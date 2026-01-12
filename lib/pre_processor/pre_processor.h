#ifndef PRE_PROCESSOR_H_
#define PRE_PROCESSOR_H_

#include "common/utility.h"

#include "bev_map/bev_map_pre_processor.h"
#include "common/CommonDataType.h"
#include "lib/base/landmark_buffer.h"
#include "routing_map/routing_map_pre_processor.h"

namespace cem {
namespace fusion {

class PreProcessor {
 public:
  PreProcessor(std::shared_ptr<LandmarkBuffer> buffer);
  ~PreProcessor() = default;

  void Process();

  void Process(BevMapInfoPtr bev_map = nullptr, RoutingMapPtr ld_map = nullptr);

  void SetRightTurnTrajectory(double timestamp, const std::vector<Eigen::Vector2f> &trajectory);

  void SetTraverseCrossWalkLaneInfo(
      std::vector<traverseCrossWalkLane> &crosswalk_lane_list);

  DetectBevMap GetGlobalBevMapPtr() { return bev_map_pre_processor_->GetGlobalBevMapPtr(); }

  const std::vector<cem::message::sensor::BevLaneMarker> &GetLidarClusterRoadEdges() const {
    return bev_map_pre_processor_->GetLidarClusterRoadEdges();
  };

  // bool   is_cross_road_status_ = false;
  // double cross_road_distance_  = -1.0;

 public:
  std::unique_ptr<BevMapPreProcessor>              bev_map_pre_processor_;
  std::unique_ptr<RoutingMapPreProcessor>          routing_map_pre_processor_;
  std::shared_ptr<LandmarkBuffer> landmark_buffer_;
};

}  // namespace fusion
}  // namespace cem

#endif
