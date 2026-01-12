#ifndef TRAFFIC_FLOW_MAPPING_H_
#define TRAFFIC_FLOW_MAPPING_H_

#include <vector>
#include <optional>
#include <unordered_map>
#include <Eigen/Geometry>
#include "base/sensor_data_manager.h"

namespace cem {
namespace fusion {

class TrafficFlowMapping {
 public:
  TrafficFlowMapping();

   ~TrafficFlowMapping();

  void Process(const EnvInfo &env_info);

 private:
  static constexpr float kXAxisRange = -5;
  static constexpr float kTrajectorySampleDistance = 2;
  static constexpr size_t kMaxTrajectoryPointsSize = 75;

  struct ObjectTrajectory {
    double timestamp = 0;
    uint32_t id = 0;
    uint64_t tracked_lane_id = 0;
    size_t consecutive_invisible_count = 0;
    bool is_valid = false;
    double length = 0;
    std::deque<Eigen::Vector3d> points;
    std::deque<Eigen::Vector3d> ego_points;
    Eigen::Vector3d velocity;
    std::vector<uint64_t> associated_bev_lane_index;
    std::unordered_map<uint64_t, std::pair<size_t, size_t>> associated_bev_region_id_range;
    std::unordered_map<uint64_t, std::pair<size_t, size_t>> split_bev_region_id_range;
    std::unordered_map<uint64_t, size_t> split_id_in_trajectory;

    void AddTrack(const FusionObj& fusion_object, uint64_t lane_id) {
      timestamp = fusion_object.timestamp;
      if (points.empty()) {
        points.emplace_back(fusion_object.position);
      }
      id = fusion_object.id;
      tracked_lane_id = lane_id;
      velocity = fusion_object.velocity;
    }

    void Update(bool detected, const FusionObj& fusion_object) {
      if (detected) {
        consecutive_invisible_count = 0;
      } else {
        consecutive_invisible_count++;
        return;
      }

      timestamp = fusion_object.timestamp;
      velocity = fusion_object.velocity;
      if (points.empty()) {
        points.emplace_back(fusion_object.position);
        return;
      }
      double distance = (points.back() - fusion_object.position).norm();
      if (distance > kTrajectorySampleDistance) {
        points.emplace_back(fusion_object.position);
      }
      if (points.size() > kMaxTrajectoryPointsSize) {
        points.pop_front();
      }
    }
  };

  bool PreProcessor();

  void TrafficFlowTracking();

  bool ShouldErase(const ObjectTrajectory& object_trajectory);

  bool CalculateLocalToEgo();

  void UpdateEgoPoints();

  void FusionInLocalMap();

  template <typename T>
  std::optional<uint64_t> SearchUniqueID(T& markers);

  void SetValidTrafficFlow();

  bool IsOverlapping(const ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane);

  void AssociateBevLaneInSplitRegion(ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane);

  void AssociateBevLaneInNormalRegion(ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane);

  bool IsAssociatedSplitLane(const std::pair<size_t, size_t>& associated_region_id_range,
    const std::pair<size_t, size_t>& split_region_id_range);

  void ConstructTopology(const std::pair<uint32_t, uint64_t>& selected_traffic_flow_id,
      const ObjectTrajectory& trajectory, BevLaneInfo& fused_lane);

  bool RequireCompensationForSucessors(const BevLaneInfo& bev_lane);

  void CompensationTopologySucessor();

  void UpdateLength();

  uint64_t basic_mapping_lane_id_;
  uint64_t prev_bev_timestamp_ = 0;

  Eigen::Isometry3d T_local_ego_;
  Eigen::Isometry3d T_ego_local_;
  Eigen::Isometry3d T_ego_local_bev_;

  BevMapInfoPtr bev_map_ptr_ = nullptr;

  std::unordered_map<uint32_t, ObjectTrajectory> traffic_flow_;

  std::shared_ptr<FusObjInfo> fusion_object_ptr_ = nullptr;

};

}  // namespace fusion
}  // namespace cem

#endif  // TRAFFIC_FLOW_MAPPING_H_
