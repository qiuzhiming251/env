#include "diversion_mapping.h"

namespace cem {
namespace fusion {
constexpr bool DEBUG_INFO = true;

DiversionTracker::DiversionTracker(int max_invisible, int min_confirmed_age, int smooth_window)
    : max_invisible_count_(max_invisible),
      min_confirmed_age_(min_confirmed_age),
      smooth_window_size_(smooth_window) {}

void DiversionTracker::TrackedDiversion::Update(const std::vector<Point2DF>& new_points) {
  history_points.push_back(new_points);
  if (history_points.size() > 5) {
    history_points.pop_front();
  }
}

std::vector<Point2DF> DiversionTracker::SmoothPolygon(
    const std::deque<std::vector<Point2DF>>& history) const {
  if (history.empty()) return {};

  size_t point_size = history.front().size();
  std::vector<Point2DF> smoothed(point_size, {0.0f, 0.0f});
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

// 计算局部坐标系下的 AABB（min,max）。若 pts 为空返回 false。
bool ComputeAABBLocal(const std::vector<Eigen::Vector2f>& local_pts,
                      Eigen::Vector2f &min_pt, Eigen::Vector2f &max_pt) {
    if (local_pts.empty()) return false;
    float minx = local_pts[0].x(), miny = local_pts[0].y();
    float maxx = minx, maxy = miny;
    for (size_t i = 1; i < local_pts.size(); ++i) {
        minx = std::min(minx, local_pts[i].x());
        miny = std::min(miny, local_pts[i].y());
        maxx = std::max(maxx, local_pts[i].x());
        maxy = std::max(maxy, local_pts[i].y());
    }
    min_pt = Eigen::Vector2f(minx, miny);
    max_pt = Eigen::Vector2f(maxx, maxy);
    return true;
}

// 在同一坐标系下计算两个 AABB 的 IoU（局部坐标系下）
// 输入为 min/max（local coordinates）
float ComputeAABBIoU_Local(const Eigen::Vector2f &min1, const Eigen::Vector2f &max1,
                           const Eigen::Vector2f &min2, const Eigen::Vector2f &max2) {
    // 计算交集在每个轴的长度
    float ix = std::max(0.0f, std::min(max1.x(), max2.x()) - std::max(min1.x(), min2.x()));
    float iy = std::max(0.0f, std::min(max1.y(), max2.y()) - std::max(min1.y(), min2.y()));
    float inter_area = ix * iy;
    float area1 = std::max(0.0f, (max1.x() - min1.x())) * std::max(0.0f, (max1.y() - min1.y()));
    float area2 = std::max(0.0f, (max2.x() - min2.x())) * std::max(0.0f, (max2.y() - min2.y()));
    const float eps = 1e-12f;
    float union_area = area1 + area2 - inter_area;
    if (union_area <= eps) return 0.0f;
    return inter_area / union_area;
}

// 构建以 pts[0] 为原点，x 轴为 (pts[1]-pts[0]) 单位向量的局部坐标系。
// 返回 false 表示失败（例如 pts.size() < 2 或长边长度太小），这时 caller 可选择用单位旋转/用世界坐标系。
bool BuildLongEdgeFrame(const std::vector<Point2DF>& pts,
                        Eigen::Matrix2f &R,            // 输出: 列向量为局部 x,y 轴
                        Eigen::Vector2f &origin) {     // 输出: 局部原点（世界坐标）
    if (pts.size() < 2) return false;
    origin = Eigen::Vector2f(pts[0].x, pts[0].y);
    Eigen::Vector2f v = Eigen::Vector2f(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
    const float eps = 1e-6f;
    float norm = v.norm();
    if (norm < eps) {
        // 长度太小，无法构建有效长边方向
        return false;
    }
    Eigen::Vector2f x_axis = v / norm;
    // 右手正交构造 y_axis （旋转 90 deg）
    Eigen::Vector2f y_axis(-x_axis.y(), x_axis.x());
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    return true;
}

// 将一组点变换到给定局部坐标系（R, origin）中： local = R^T * (pt - origin)
std::vector<Eigen::Vector2f> TransformPointsToFrame(const std::vector<Point2DF>& pts,
                                                    const Eigen::Matrix2f &R,
                                                    const Eigen::Vector2f &origin) {
    std::vector<Eigen::Vector2f> out;
    out.reserve(pts.size());
    Eigen::Matrix2f Rt = R.transpose();
    for (const auto &p : pts) {
        Eigen::Vector2f v(p.x, p.y);
        out.push_back(Rt * (v - origin));
    }
    return out;
}

// 按第一个矩形的长边构造局部坐标系，将两个矩形都转换到该系，计算其 AABB 并返回 IoU。
// pts1, pts2 应分别为 4 点（或更多点），且 pts1[0]->pts1[1] 被视为长边方向。
// 返回 IoU in [0,1]. 若任一点集为空返回 0.
float ComputeIoU_ByAligningToFirstLongEdge(const std::vector<Point2DF>& pts1,
                                           const std::vector<Point2DF>& pts2) {
    if (pts1.empty() || pts2.empty()) return 0.0f;

    Eigen::Matrix2f R1;
    Eigen::Vector2f origin1;
    bool ok = BuildLongEdgeFrame(pts1, R1, origin1);
    if (!ok) {
        // 退回到以 pts1 的质心方向（或直接 AABB in world）——这里我们退回到世界坐标系（R=I, origin=0）
        R1 = Eigen::Matrix2f::Identity();
        origin1 = Eigen::Vector2f::Zero();
    }

    // 把两个点集都变换到 pts1 的局部系
    std::vector<Eigen::Vector2f> local1 = TransformPointsToFrame(pts1, R1, origin1);
    std::vector<Eigen::Vector2f> local2 = TransformPointsToFrame(pts2, R1, origin1);

    Eigen::Vector2f min1, max1, min2, max2;
    if (!ComputeAABBLocal(local1, min1, max1)) return 0.0f;
    if (!ComputeAABBLocal(local2, min2, max2)) return 0.0f;

    return ComputeAABBIoU_Local(min1, max1, min2, max2);
}

void DiversionTracker::Update(const std::vector<BevLaneMarker>& detected_markers,
                               const Eigen::Isometry3d& T_marker_to_ego) {
  T_local_ego_ = T_marker_to_ego;
  T_ego_local_ = T_marker_to_ego.inverse();

  std::unordered_map<uint64_t, bool> matched;
  const float                        iou_threshold = 0.2f;

  for (auto marker : detected_markers) {

    for (auto &point : marker.line_points) {
      TransformPoint(&point, T_local_ego_);
    }

    auto it = tracked_diversions_.find(marker.id);
    if (it != tracked_diversions_.end()) {
      it->second.Update(marker.line_points);  // 存原始点
      it->second.age++;
      it->second.invisible_count = 0;
      it->second.smoothed_points = marker.line_points;//SmoothPolygon(it->second.history_points);
      it->second.confirmed       = true;
      matched[marker.id]         = true;
    } else {
      bool        matched_by_geo = false;
      uint64_t    matched_old_id = 0;

      for (const auto &[old_id, track] : tracked_diversions_) {
        if (matched.find(old_id) != matched.end())
          continue;

        float iou = ComputeIoU_ByAligningToFirstLongEdge(marker.line_points, track.smoothed_points);

        if (DEBUG_INFO)
          AINFO << "iou:" << iou << " old_id:" << old_id << " marker.id:" << marker.id ;

        if (iou > iou_threshold) {
          matched_by_geo = true;
          // best_distance  = iou;  // 用 IoU 作为匹配度
          matched_old_id = old_id;
        }
      }

      if (matched_by_geo) {
        AINFO << "Diversion ID reassigned: old = " << matched_old_id << ", new = " << marker.id;
        auto old_track = tracked_diversions_[matched_old_id];
        tracked_diversions_.erase(matched_old_id);
        diversion_dynamic_thresholds_.erase(matched_old_id);

        old_track.id = marker.id;
        old_track.age++;
        old_track.invisible_count = 0;
        old_track.Update(marker.line_points);
        old_track.smoothed_points      = marker.line_points;//SmoothPolygon(old_track.history_points);
        tracked_diversions_[marker.id] = old_track;
        matched[marker.id]             = true;
      } else {
        // 正常新增目标
        TrackedDiversion track;
        track.id              = marker.id;
        track.age             = 1;
        track.invisible_count = 0;
        track.history_points.push_back(marker.line_points);
        track.smoothed_points          = marker.line_points;
        track.confirmed                = true;
        tracked_diversions_[marker.id] = track;
        matched[marker.id]             = true;
      }
    }
  }

  // 清理未匹配的轨迹
  for (auto it = tracked_diversions_.begin(); it != tracked_diversions_.end();) {
    if (matched.find(it->first) == matched.end()) {
      it->second.invisible_count++;
      
      //框子消失在车身后
      float x_max = -std::numeric_limits<float>::max();
      float y_min = std::numeric_limits<float>::max();
      for (auto point : it->second.smoothed_points) {
        TransformPoint(&point, T_ego_local_);
        x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, fabs(point.y));
      }

      if (x_max < -62.f|| y_min > 32.f) {
        it = tracked_diversions_.erase(it);
        continue;
      } else {
        ++it;
      }
    } else {
      ++it;
    }
  }
}

std::vector<BevLaneMarker> DiversionTracker::GetConfirmedDiversions(float ego_distance_threshold) const {
  std::vector<BevLaneMarker> result;
  for (const auto& [id, track] : tracked_diversions_) {
    if (track.confirmed && !track.smoothed_points.empty()) {
        BevLaneMarker marker;
        marker.id = track.id;
        marker.line_points = track.smoothed_points;
        std::vector<Eigen::Vector2f> geos;
        for (auto &point : marker.line_points) {
          TransformPoint(&point, T_ego_local_);
          geos.push_back(Eigen::Vector2f(point.x, point.y));
        }
        if (!geos.empty())
          marker.geos = std::make_unique<std::vector<Eigen::Vector2f>>(geos);
        else
          marker.geos.reset();
        result.push_back(marker);
    }
  }

  return result;
}

void DiversionTracker::PrintInputMarkers(const std::vector<BevLaneMarker>& markers) const {
  AINFO << "[DiversionTracker] Input BevLaneMarkers (count = " << markers.size() << "):";
  for (const auto& marker : markers) {
    // if(marker.id != 71 && marker.id != 101)continue;
    AINFO << "  ID = " << marker.id << ", Points = [";
    // for (const auto& pt : marker.line_points) {
    //   AINFO << "lp(" << pt.x << ", " << pt.y << "), ";
    // }
    for (const auto& pt : *marker.geos) {
      AINFO << "m(" << pt.x() << ", " << pt.y() << "), ";
    }
    AINFO << "]";
  }
}

void DiversionTracker::PrintTrackedDiversions() const {
  AINFO << "[DiversionTracker] Tracked Diversions (count = " << tracked_diversions_.size() << "):";
  for (const auto& [id, track] : tracked_diversions_) {
    // if(id != 71 && id != 101)continue;
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

}  // namespace fusion
}  // namespace cem
