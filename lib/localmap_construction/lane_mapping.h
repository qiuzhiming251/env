#pragma once

#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "lib/common/log_custom.h"
#include "common/CommonDataType.h"
#include "common/utils/lane_geometry.h"

namespace cem {
namespace fusion {

class LaneTracker {
 public:
  struct UpdateInfo {
    std::vector<BevLaneInfo> detected_lanes;
    Eigen::Isometry3d T_laneinfo_to_ego;
  };
  struct TrackedCrossWalk {
    // uint64_t id;
    BevLaneInfo ori_lane_info;
    int age = 0;
    int invisible_count = 0;
    bool confirmed = false;
    std::optional<BevLaneMarker> left_lanemarker;
    std::optional<BevLaneMarker> right_lanemarker;

    // std::deque<std::vector<Point2DF>> history_lane;
    std::vector<Point2DF> smoothed_points;

    // void Update(const std::vector<Point2DF>& new_points);
  };
  struct LaneThreshold {
    int min_confirmed_age;
    int max_invisible_count;
  };

  LaneTracker(float min_invisible_dist = -30.f, int min_confirmed_age = 1, int smooth_window = 0, int min_lane_confirmed_size = 5);
  ~LaneTracker();
  void Update(const std::vector<BevLaneInfo> &detected_lanes, const std::vector<BevLaneMarker> &detected_lms,
              const Eigen::Isometry3d &T_local_ego, bool add_new, bool debug = false);

  std::vector<BevLaneInfo> getResult(std::vector<BevLaneMarker> &result_lanemarkers) {
    std::vector<BevLaneInfo> result_lanes;
    result_lanemarkers = landmarkers_;
    for (auto &lm : result_lanemarkers) {
      for (auto &point : lm.line_points) {
        TransformPoint(&point, T_ego_local_);
      }
    }

    for (const auto &[id, track] : tracked_lanes_) {
      if (track.confirmed && !track.ori_lane_info.line_points.empty()) {
        BevLaneInfo laneinfo = track.ori_lane_info;
        laneinfo.line_points = track.smoothed_points;
        for (auto &point : laneinfo.line_points) {
          TransformPoint(&point, T_ego_local_);
        }
        result_lanes.push_back(std::move(laneinfo));
      }
    }
    return result_lanes;
  }

  void PrintInputMarkers(const std::vector<BevLaneInfo> &lanes) const;
  void PrintTrackedLanes() const;

 private:
  void CleanOrShrinkTrack(std::unordered_map<uint64_t, TrackedCrossWalk>::iterator &it, const std::unordered_map<uint64_t, bool> &matched,
                          bool debug = false);
  bool  EgoOverlap(const std::vector<Point2DF> &new_pts, const std::vector<Point2DF> &old_pts);
  float min_invisible_dist_;
  int min_confirmed_age_;
  int smooth_window_size_;
  int min_lane_confirmed_size_;
  Eigen::Isometry3d T_local_ego_,T_ego_local_;

  std::unordered_map<uint64_t, TrackedCrossWalk> tracked_lanes_;
  std::vector<BevLaneMarker> landmarkers_;
  std::vector<Point2DF>    SmoothPoints(const std::vector<Point2DF> &update_lane,
                                                              const std::vector<Point2DF> &previous_smooth_lane,
                                                              bool                                               debug = false) const;

};

}  // namespace fusion
}  // namespace cem
