#include "crosswalk_mapping.h"
// #include <cmath>
// #include <algorithm>

namespace cem {
namespace fusion {

namespace {
constexpr float    MaxCrosswalkRange            = 80.0;
constexpr uint64_t MaxBevID                     = 200;
constexpr int      MaxCrosswalkPointSize        = 4;
}

void CrossWalkTracker::Process(DetectBevMap& bev_map) {
  if (!bev_map.bev_map_ptr) {
    return;
  }

  bev_map.bev_map_ptr->crosswalks.erase(std::remove_if(bev_map.bev_map_ptr->crosswalks.begin(), bev_map.bev_map_ptr->crosswalks.end(),
                                 [](const BevLaneMarker& t) { return t.id > MaxBevID; }),
                  bev_map.bev_map_ptr->crosswalks.end());

  RoutingMapPtr routing_map_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_ptr);
  if (!routing_map_ptr) {
    return;
  }
  LocalizationPtr local_pose_ptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_ptr->header.timestamp, 0.05, local_pose_ptr);
  if (!local_pose_ptr) {
    return;
  }

  Eigen::Isometry3d T_local_ego;
  T_local_ego.translation() = Eigen::Vector3d(
      local_pose_ptr->posne_dr.at(0), local_pose_ptr->posne_dr.at(1), 0);

  T_local_ego.linear() =
      Eigen::AngleAxisd(local_pose_ptr->attitude_dr * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<BevLaneMarker> map_crosswalks;
  for (size_t i = 0; i < routing_map_ptr->cross_walks.size(); i++) {
    if (routing_map_ptr->cross_walks[i].points.size() < MaxCrosswalkPointSize) {
      continue;
    }

    if (std::abs(routing_map_ptr->cross_walks[i].points.front().x) > MaxCrosswalkRange ||
        std::abs(routing_map_ptr->cross_walks[i].points.front().y) > MaxCrosswalkRange) {
      continue;
    }

    BevLaneMarker map_crosswalk;
    map_crosswalk.id = routing_map_ptr->cross_walks[i].id;
    for (size_t j = 0; j < MaxCrosswalkPointSize; j++) {
      Eigen::Vector3d pt(routing_map_ptr->cross_walks[i].points[j].x, routing_map_ptr->cross_walks[i].points[j].y, 0);
      pt = T_local_ego * pt;
      map_crosswalk.line_points.push_back(Point2DF(pt.x(), pt.y()));
    }
    map_crosswalks.emplace_back(map_crosswalk);
  }

  if (map_crosswalks.empty()) {
    return;
  }

  for (size_t i = 0; i < map_crosswalks.size(); i++) {
    map_crosswalks[i].id = MaxBevID + i + 1;
  }

  for (auto& bev_crosswalk : bev_map.bev_map_ptr->crosswalks) {
    for (auto& map_crosswalk : map_crosswalks) {
      if (AssociateBevAndMap(bev_crosswalk, map_crosswalk)) {
        map_crosswalk.is_associated_bev = true;
      }
    }
  }

  for (auto& map_crosswalk : map_crosswalks) {
    if (map_crosswalk.is_associated_bev) {
      continue;
    }
    bev_map.bev_map_ptr->crosswalks.emplace_back(map_crosswalk);
  }
}

CrossWalkTracker::CrossWalkTracker(int max_invisible, int min_confirmed_age, int smooth_window)
    : max_invisible_count_(max_invisible), min_confirmed_age_(min_confirmed_age), smooth_window_size_(smooth_window),initialized_(false) {
  process_threads_ = std::thread(std::bind(&CrossWalkTracker::UpdateThread, this));
  initialized_ = true;
}

CrossWalkTracker::~CrossWalkTracker() {
  initialized_ = false;
  cv_input_.notify_all();
  if (process_threads_.joinable()) {
    process_threads_.join();
  }
}

void CrossWalkTracker::UpdateThread() {
  std::deque<UpdateInfo> local_update_info;
  while (initialized_) {
    {
      std::unique_lock<std::mutex> lock(mutex_input_);
      cv_input_.wait(lock);

      if (update_info_.empty())
        continue;

      local_update_info = std::move(update_info_);
      update_info_.clear();
    }
    for (auto &update_info : local_update_info) {
      Update(update_info.detected_markers, update_info.T_marker_to_ego);
    }

    std::unique_lock<std::shared_mutex> lock(mutex_result_);
    result_crosswalks_.clear();
    for (const auto &[id, track] : tracked_crosswalks_) {
      if (track.confirmed && !track.smoothed_points.empty()) {
        BevLaneMarker marker;
        marker.id          = track.id;
        marker.line_points = track.smoothed_points;
        result_crosswalks_.push_back(marker);
      }
    }
  }  
}

void CrossWalkTracker::TrackedCrossWalk::Update(const std::vector<cem::message::common::Point2DF>& new_points) {
  history_points.push_back(new_points);
  if (history_points.size() > 5) {
    history_points.pop_front();
  }
}

double CrossWalkTracker::ComputeLineDistance(
    const std::vector<cem::message::common::Point2DF>& a,
    const std::vector<cem::message::common::Point2DF>& b) const {
  if (a.empty() || b.empty()) return 1e9;

  Eigen::Vector2f ca(0, 0), cb(0, 0);
  for (const auto& pt : a) ca += Eigen::Vector2f(pt.x, pt.y);
  for (const auto& pt : b) cb += Eigen::Vector2f(pt.x, pt.y);
  ca /= static_cast<float>(a.size());
  cb /= static_cast<float>(b.size());

  return (ca - cb).norm();
}

std::vector<cem::message::common::Point2DF> CrossWalkTracker::SmoothPolygon(
    const std::deque<std::vector<cem::message::common::Point2DF>>& history) const {
  if (history.empty()) return {};

  size_t point_size = history.front().size();
  std::vector<cem::message::common::Point2DF> smoothed(point_size, {0.0f, 0.0f});
  int count = static_cast<int>(history.size());

  for (int i = 0; i < point_size; ++i) {
    for (const auto& poly : history) {
      if (i >= poly.size()) continue;
      smoothed[i].x += poly[i].x;
      smoothed[i].y += poly[i].y;
    }
    smoothed[i].x /= count;
    smoothed[i].y /= count;
  }

  return smoothed;
}

void CrossWalkTracker::Update(const std::vector<BevLaneMarker>& detected_markers,
                               const Eigen::Isometry3d& T_marker_to_ego) {
  std::unordered_map<uint64_t, bool> matched;

  for (const auto& marker : detected_markers) {
    // 1. 坐标变换: 将 line_points 转换为 ego 坐标系
    std::vector<Eigen::Vector2f> ego_points;
    for (const auto& pt : marker.line_points) {
      Eigen::Vector3d p_world(pt.x, pt.y, 0.0);
      Eigen::Vector3d p_ego = T_marker_to_ego * p_world;
      ego_points.emplace_back(static_cast<float>(p_ego.x()), static_cast<float>(p_ego.y()));
    }

    // 2. 距离估计（基于所有点中最近的 x）
    float min_x = std::numeric_limits<float>::max();
    for (const auto& pt : ego_points) {
      if (fabs(pt.x()) < min_x) {
        min_x = pt.x();
      }
    }

    // 3. 动态阈值设定（越近越低）
    int dynamic_confirm_age = std::max(1, static_cast<int>(std::round(min_confirmed_age_ * std::min(1.0f, min_x / 20.0f))));
    int dynamic_invisible_limit = std::max(1, static_cast<int>(std::round((float)max_invisible_count_ / std::min(1.0f, min_x / 20.0f))));
    // if (marker.id == 71 || marker.id == 101)
    //   AINFO << "min_x: " << min_x << ", dynamic_confirm_age: " << dynamic_confirm_age
    //         << ", dynamic_invisible_limit: " << dynamic_invisible_limit;

    // 4. ID匹配与更新
    auto it = tracked_crosswalks_.find(marker.id);
    if (it != tracked_crosswalks_.end()) {
      it->second.Update(marker.line_points);  // 存原始点
      it->second.age++;
      it->second.invisible_count = 0;
      it->second.smoothed_points = SmoothPolygon(it->second.history_points);
      if (it->second.age >= dynamic_confirm_age) {
        it->second.confirmed = true;
      }
      matched[marker.id] = true;
    } else {
      bool        matched_by_geo      = false;
      uint64_t    matched_old_id      = 0;
      float       best_distance       = std::numeric_limits<float>::max();
      const float geo_match_threshold = 3.5f;  // 距离阈值，单位：米

      for (const auto &[old_id, track] : tracked_crosswalks_) {
        if (matched.find(old_id) != matched.end())
          continue;  // 当前帧已匹配的跳过

        float avg_dist = 0.0f;
        int   count    = 0;

        // 点对点欧氏距离匹配（可根据需要改为更鲁棒的重采样+Hausdorff）
        const auto &old_pts = track.smoothed_points;
        const auto &new_pts = marker.line_points;

        int pair_num = std::min(old_pts.size(), new_pts.size());
        if (pair_num < 3)
          continue;

        for (int i = 0; i < pair_num; ++i) {
          Eigen::Vector2f diff(old_pts[i].x - new_pts[i].x, old_pts[i].y - new_pts[i].y);
          avg_dist += diff.norm();
          count++;
        }

        if (count > 0)
          avg_dist /= count;

        if (avg_dist < geo_match_threshold && avg_dist < best_distance) {
          matched_by_geo = true;
          best_distance  = avg_dist;
          matched_old_id = old_id;
        }
      }

      if (matched_by_geo) {
        AINFO << "Crosswalk ID reassigned: old = " << matched_old_id << ", new = " << marker.id;
        auto old_track = tracked_crosswalks_[matched_old_id];
        tracked_crosswalks_.erase(matched_old_id);
        crosswalk_dynamic_thresholds_.erase(matched_old_id);

        old_track.id = marker.id;
        old_track.age++;
        old_track.invisible_count = 0;
        old_track.Update(marker.line_points);
        old_track.smoothed_points      = SmoothPolygon(old_track.history_points);
        tracked_crosswalks_[marker.id] = old_track;
        matched[marker.id]             = true;
      } else {
        // 正常新增目标
        TrackedCrossWalk track;
        track.id              = marker.id;
        track.age             = 1;
        track.invisible_count = 0;
        track.history_points.push_back(marker.line_points);
        track.smoothed_points          = marker.line_points;
        tracked_crosswalks_[marker.id] = track;
        matched[marker.id]             = true;
      }
    }

    // 缓存动态阈值
    crosswalk_dynamic_thresholds_[marker.id] = {
        .min_confirmed_age = dynamic_confirm_age,
        .max_invisible_count = dynamic_invisible_limit
    };
  }

  // 清理未匹配的轨迹
  for (auto it = tracked_crosswalks_.begin(); it != tracked_crosswalks_.end();) {
    if (matched.find(it->first) == matched.end()) {
      it->second.invisible_count++;
      int invisible_limit = crosswalk_dynamic_thresholds_[it->first].max_invisible_count;
      if (it->first == 71 || it->first == 101)
        AINFO << "invisible_count: " << it->second.invisible_count << " invisible_limit: " << invisible_limit;
      if (it->second.invisible_count > invisible_limit) {
        crosswalk_dynamic_thresholds_.erase(it->first);
        it = tracked_crosswalks_.erase(it);
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void CrossWalkTracker::PrintInputMarkers(const std::vector<BevLaneMarker>& markers) const {
  AINFO << "[CrossWalkTracker] Input BevLaneMarkers (count = " << markers.size() << "):";
  for (const auto& marker : markers) {
    if(marker.id != 71 && marker.id != 101)continue;
    AINFO << "  ID = " << marker.id << ", Points = [";
    for (const auto& pt : marker.line_points) {
      AINFO << "(" << pt.x << ", " << pt.y << "), ";
    }
    AINFO << "]";
  }
}

void CrossWalkTracker::PrintTrackedCrosswalks() const {
  AINFO << "[CrossWalkTracker] Tracked Crosswalks (count = " << tracked_crosswalks_.size() << "):";
  for (const auto& [id, track] : tracked_crosswalks_) {
    if(id != 71 && id != 101)continue;
    AINFO << "  ID = " << id
              << ", Age = " << track.age
              << ", Invisible = " << track.invisible_count
              << ", Confirmed = " << (track.confirmed ? "Yes" : "No")
              << ", Smoothed Points = [";
    for (const auto& pt : track.smoothed_points) {
      AINFO << "(" << pt.x << ", " << pt.y << "), ";
    }
    AINFO << "]";
  }
}

bool CrossWalkTracker::AssociateBevAndMap(const BevLaneMarker& bev_crosswalk,
                                          const BevLaneMarker& noa_crosswalk) {
  if (bev_crosswalk.line_points.size() < MaxCrosswalkPointSize || noa_crosswalk.line_points.size() < MaxCrosswalkPointSize) {
    return false;
  }

  Point2DF bev_center;
  for (size_t i = 0; i < MaxCrosswalkPointSize; i++) {
    bev_center.x += bev_crosswalk.line_points[i].x;
    bev_center.y += bev_crosswalk.line_points[i].y;
  }

  bev_center.x /= MaxCrosswalkPointSize;
  bev_center.y /= MaxCrosswalkPointSize;

  if (IsPointInPolygon(noa_crosswalk.line_points, bev_center)) {
    return true;
  }

  return false;
}

template <typename T>
bool CrossWalkTracker::IsPointInPolygon(const std::vector<T>& polygon, const T& p) {
  if (polygon.size() < 3) {
    return false;
  }

  int n = polygon.size();
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
      // 检查点是否在多边形的顶点上
      if ((polygon[i].x == p.x && polygon[i].y == p.y) ||
          (polygon[j].x == p.x && polygon[j].y == p.y)) {
          return true;
      }

      // 检查点是否在边的范围内
      bool yInRange = (polygon[i].y > p.y) != (polygon[j].y > p.y);
      if (!yInRange) {
        continue;
      }

      // 计算射线与边的交点x坐标
      double xIntersect = ((p.y - polygon[i].y) * (polygon[j].x - polygon[i].x)) /
                          (polygon[j].y - polygon[i].y) + polygon[i].x;

      // 如果交点在点的右侧，则射线与边相交
      if (p.x < xIntersect) {
          inside = !inside;
      }
  }

  return inside;
}

}  // namespace fusion
}  // namespace cem
