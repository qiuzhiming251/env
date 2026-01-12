#pragma once

#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "lib/common/log_custom.h"
#include "common/CommonDataType.h"
#include <common/utils/lane_geometry.h>

typedef cem::message::common::Point2DF Point2DF;

namespace cem {
namespace fusion {

class DiversionTracker {
 public:
  struct TrackedDiversion {
    uint64_t id;
    int age = 0;
    int invisible_count = 0;
    bool confirmed = false;

    std::deque<std::vector<Point2DF>> history_points;
    std::vector<Point2DF> smoothed_points;

    void Update(const std::vector<Point2DF>& new_points);
  };
  struct DiversionThreshold {
    int min_confirmed_age;
    int max_invisible_count;
  };

  DiversionTracker(int max_invisible = 3, int min_confirmed_age = 5, int smooth_window = 5);

  void Update(const std::vector<BevLaneMarker> &detected_markers, const Eigen::Isometry3d &T_marker_to_ego);

  std::vector<BevLaneMarker> GetConfirmedDiversions(float ego_distance_threshold = 30.0f) const;

  void PrintInputMarkers(const std::vector<BevLaneMarker> &markers) const;
  void PrintTrackedDiversions() const;

 private:
  int max_invisible_count_;
  int min_confirmed_age_;
  int smooth_window_size_;
  
  Eigen::Isometry3d T_local_ego_,T_ego_local_;

  std::unordered_map<uint64_t, TrackedDiversion> tracked_diversions_;

  std::vector<Point2DF> SmoothPolygon(
      const std::deque<std::vector<Point2DF>>& history) const;

  std::unordered_map<uint64_t, DiversionThreshold> diversion_dynamic_thresholds_;
};

}  // namespace fusion
}  // namespace cem