#ifndef CROSSWALK_TRACKER_H_
#define CROSSWALK_TRACKER_H_

#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "lib/common/log_custom.h"
#include "common/CommonDataType.h"
#include "common/utils/lane_geometry.h"

#include <thread>  
#include <shared_mutex>
#include <mutex>  
#include <condition_variable>  

namespace cem {
namespace fusion {

class CrossWalkTracker {
 public:
  struct UpdateInfo {
    std::vector<BevLaneMarker> detected_markers;
    Eigen::Isometry3d T_marker_to_ego;
  };
  struct TrackedCrossWalk {
    uint64_t id;
    int age = 0;
    int invisible_count = 0;
    bool confirmed = false;

    std::deque<std::vector<cem::message::common::Point2DF>> history_points;
    std::vector<cem::message::common::Point2DF> smoothed_points;

    void Update(const std::vector<cem::message::common::Point2DF>& new_points);
  };
  struct CrosswalkThreshold {
    int min_confirmed_age;
    int max_invisible_count;
  };

  CrossWalkTracker(int max_invisible = 3, int min_confirmed_age = 5, int smooth_window = 5);
  ~CrossWalkTracker();
  
  void PushUpdate(const std::vector<BevLaneMarker> &detected_markers, const Eigen::Isometry3d &T_marker_to_ego){
    UpdateInfo update_info;
    update_info.detected_markers = detected_markers;
    update_info.T_marker_to_ego  = T_marker_to_ego;

    std::unique_lock<std::mutex> lock(mutex_input_);
    update_info_.push_back(update_info);
    while (update_info_.size() > 10)
    {
      update_info_.pop_front();
    }    
    cv_input_.notify_all();
  };

  std::vector<BevLaneMarker> GetConfirmedCrosswalks() {
    std::shared_lock<std::shared_mutex> lock(mutex_result_);
    return result_crosswalks_;
  };

  void PrintInputMarkers(const std::vector<BevLaneMarker> &markers) const;
  void PrintTrackedCrosswalks() const;

  void Process(DetectBevMap& bev_map);

  bool AssociateBevAndMap(const BevLaneMarker& bev_crosswalk, const BevLaneMarker& noa_crosswalk);

 private:
  int max_invisible_count_;
  int min_confirmed_age_;
  int smooth_window_size_;

  std::unordered_map<uint64_t, TrackedCrossWalk> tracked_crosswalks_;
  void UpdateThread();
  double ComputeLineDistance(const std::vector<cem::message::common::Point2DF>& a,
                             const std::vector<cem::message::common::Point2DF>& b) const;
  void Update(const std::vector<BevLaneMarker> &detected_markers, const Eigen::Isometry3d &T_marker_to_ego);

  std::vector<cem::message::common::Point2DF> SmoothPolygon(
      const std::deque<std::vector<cem::message::common::Point2DF>>& history) const;

  template <typename T>
  bool IsPointInPolygon(const std::vector<T>& polygon, const T& p);

  std::unordered_map<uint64_t, CrosswalkThreshold> crosswalk_dynamic_thresholds_;
  std::deque<UpdateInfo> update_info_;

  std::thread process_threads_;
  bool initialized_;
  std::vector<BevLaneMarker> result_crosswalks_;

  std::shared_mutex mutex_result_;
  std::mutex mutex_input_;
  std::condition_variable cv_input_;

};

}  // namespace fusion
}  // namespace cem

#endif  // CROSSWALK_TRACKER_H_
