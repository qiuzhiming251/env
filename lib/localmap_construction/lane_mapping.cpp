#include "lane_mapping.h"
// #include <cmath>
// #include <algorithm>

namespace cem {
namespace fusion {
constexpr bool DEBUG_INFO = false;

LaneTracker::LaneTracker(float min_invisible_dist, int min_confirmed_age, int smooth_window, int min_lane_confirmed_size)
    : min_invisible_dist_(min_invisible_dist),
      min_confirmed_age_(min_confirmed_age),
      smooth_window_size_(smooth_window),
      min_lane_confirmed_size_(min_lane_confirmed_size),
      T_local_ego_(Eigen::Isometry3d::Identity()),
      T_ego_local_(Eigen::Isometry3d::Identity()) {}

LaneTracker::~LaneTracker() {
}

bool IsTrackOverlapping(const std::vector<Point2DF> &new_pts, const std::vector<Point2DF> &old_pts,bool debug = false, int *matched_old_idx = nullptr,
                        int *matched_new_idx = nullptr) {
  if (old_pts.size() < 3 || new_pts.size() < 3)
    return false;

  constexpr float DIST_THRESH_SQ = 1.5f * 1.5f;
  size_t minsize = (old_pts.size()<new_pts.size()?old_pts.size():new_pts.size()) * 0.7;
  const size_t OVERLAP_THRESH = minsize > 8?8:minsize;
  // int close_count = 0;

  std::map<int, int> new_to_old;
  for (int new_idx = 0; new_idx < static_cast<int>(new_pts.size()); ++new_idx) {
    const auto &new_pt       = new_pts[new_idx];
    float       last_dist_sq = std::numeric_limits<float>::max();

    for (int old_idx = 0; old_idx < static_cast<int>(old_pts.size()); ++old_idx) {
      const auto &old_pt  = old_pts[old_idx];
      float       dx      = new_pt.x - old_pt.x;
      float       dy      = new_pt.y - old_pt.y;
      float       dist_sq = dx * dx + dy * dy;

      if (dist_sq < DIST_THRESH_SQ) {
        // close_count++;
        new_to_old[new_idx] = old_idx;
        if (matched_old_idx)
          *matched_old_idx = old_idx;
        if (matched_new_idx)
          *matched_new_idx = new_idx;

        break;  // 只需要一个最近点就够了
      }

      if (dist_sq > last_dist_sq)
        break;

      last_dist_sq = dist_sq;
    }

    if (new_to_old.size() > OVERLAP_THRESH){
      //筛掉短新线
      if (OVERLAP_THRESH < 8 && new_pts.size() < old_pts.size()) {
        return false;
      }
      if (debug) {
        AINFO << "OVERLAP_THRESH: " << OVERLAP_THRESH<<" new_pts.size() < old_pts.size()"<<new_pts.size() << " < " << old_pts.size();
      }
      return true;
    }
  }
  if (debug) {
    AINFO << "new_to_old.size(): " << new_to_old.size()<<" OVERLAP_THRESH: "<<OVERLAP_THRESH;
    AINFO  << " | new_pts: from (" << new_pts.front().x << "," << new_pts.front().y << ") to (" << new_pts.back().x << ","
          << new_pts.back().y << ")"
          << " | old_pts: from (" << old_pts.front().x << "," << old_pts.front().y << ") to (" << old_pts.back().x << ","
          << old_pts.back().y << ")";
  }

  return false;
}

bool LaneTracker::EgoOverlap(const std::vector<Point2DF> &new_pts, const std::vector<Point2DF> &old_pts) {
  const float EGO_RANGE_SQR = 1.f * 1.f;
  for (size_t i = 0; i < new_pts.size(); i++) {
    auto pt = new_pts[i];
    TransformPoint(&pt, T_ego_local_);
    if ((pt.x* pt.x+pt.y*pt.y)< EGO_RANGE_SQR) {
      if (DEBUG_INFO)
        AINFO << "pt: " << pt.x << "," << pt.y << " is in ego range";
      return true;
    }
  }
  return false;
}

void ShrinkPointsByDistance(std::vector<Point2DF>& points,
                            const Eigen::Isometry3d& T_ego_local,
                            float min_x_threshold) {
  while (!points.empty()) {
    auto pt = points.front();
    TransformPoint(&pt, T_ego_local);
    if (pt.x < min_x_threshold) {
      points.erase(points.begin());
    } else {
      break;
    }
  }
}

void LaneTracker::CleanOrShrinkTrack(
    std::unordered_map<uint64_t, TrackedCrossWalk>::iterator& it,
    const std::unordered_map<uint64_t, bool>& matched,bool debug) {
  bool should_erase = false;

  for (const auto& [id, track] : tracked_lanes_) {
    if (!matched.count(id)) continue;
    bool debug = (id == 19) && (it->second.ori_lane_info.id == 43);
    bool ego_overlap = (it->second.ori_lane_info.position == 0 && EgoOverlap(track.smoothed_points, it->second.smoothed_points));
    if (ego_overlap && DEBUG_INFO)
      AINFO << "old = " << it->first << ", new = " << id << "ego_overlap: " << ego_overlap;
    if (ego_overlap ||
        (it->second.ori_lane_info.position != 0 && IsTrackOverlapping(track.smoothed_points, it->second.smoothed_points, debug))) {
      if (DEBUG_INFO)
        AINFO << "erase lane " << it->first << " because it overlaps with " << id;
      should_erase = true;
      break;
    }
  }

  if (should_erase) {
    it = tracked_lanes_.erase(it);
  } else {
    if(debug)it->second.ori_lane_info.is_virtual = true;
    it->second.ori_lane_info.previous_lane_ids.clear();
    it->second.ori_lane_info.next_lane_ids.clear();
    ShrinkPointsByDistance(it->second.smoothed_points, T_ego_local_, min_invisible_dist_);
    ++it;
  }
}

std::vector<cem::message::common::Point2DF> LaneTracker::SmoothPoints(
    const std::vector<cem::message::common::Point2DF> &update_lane,
    const std::vector<cem::message::common::Point2DF> &previous_smooth_lane,bool debug) const {

  if(debug)AINFO << "smooth update_lane size: " << update_lane.size()
        << " previous_smooth_lane size: " << previous_smooth_lane.size();

  const float DIST_THRESHOLD_SQ = 1.0f * 1.0f;
  if (previous_smooth_lane.empty()) return update_lane;
  if (update_lane.size()<3) return previous_smooth_lane;

  std::vector<cem::message::common::Point2DF> result = update_lane;

  // 1. 找中点匹配（世界坐标系）
  int   match_idx_prev = -1;

  int  mid_idx_upd = update_lane.size() / 2;
  // auto mid_pt_upd  = update_lane[mid_idx_upd];

  // 双指针从中间向两边扩展找最近点
  for (long unsigned int offset = 0; offset < update_lane.size() / 2; ++offset) {
    // 左右对称扩展索引
    std::set<int> idxs;
    if (mid_idx_upd - offset >= 0)
      idxs.insert(mid_idx_upd - offset);
    if (mid_idx_upd + offset < update_lane.size())
      idxs.insert(mid_idx_upd + offset);

    for (int upd_idx : idxs) {
      float min_dist_sq    = std::numeric_limits<float>::max();
      const auto &upd_pt = update_lane[upd_idx];

      // 再遍历 previous_smooth_lane 找最近点
      for (long unsigned int prev_idx = 0; prev_idx < previous_smooth_lane.size(); ++prev_idx) {
        const auto &prev_pt = previous_smooth_lane[prev_idx];

        float dx      = prev_pt.x - upd_pt.x;
        float dy      = prev_pt.y - upd_pt.y;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < DIST_THRESHOLD_SQ && dist_sq < min_dist_sq) {
          match_idx_prev = prev_idx;
          mid_idx_upd    = upd_idx;  // 用新匹配上的 update idx 替换掉原中点
          min_dist_sq    = dist_sq;
        }
      }

      if (match_idx_prev != -1)
        break;  // 已匹配，立即退出
    }

    if (match_idx_prev != -1)
      break;  // 已匹配，立即退出
  }
  if (match_idx_prev<0)
  {
    return result;
  }
  

  // 2. 尾巴处理
  int remain_upd = update_lane.size() - mid_idx_upd - 1;
  int remain_prev = previous_smooth_lane.size() - match_idx_prev - 1;
  float y_offset = 0.0f;

  if (remain_upd < remain_prev) {
    auto tail_loc_upd = update_lane.back();
    TransformPoint(&tail_loc_upd, T_ego_local_);
    if(debug)
      AINFO << "update glb:" << update_lane.back().x << "," << update_lane.back().y << " loc:" << tail_loc_upd.x << "," << tail_loc_upd.y;

    for (long unsigned int i = match_idx_prev + 1; i < previous_smooth_lane.size(); ++i) {
      auto ego_loc_prev = previous_smooth_lane[i];
      TransformPoint(&ego_loc_prev, T_ego_local_);
      if (ego_loc_prev.x > tail_loc_upd.x) {
        if (y_offset == 0.0f)
          y_offset = tail_loc_upd.y - ego_loc_prev.y;
        ego_loc_prev.y += y_offset;
        if(debug)AINFO<<"add loc:"<<ego_loc_prev.x<<","<<ego_loc_prev.y;
        TransformPoint(&ego_loc_prev, T_local_ego_);
        ego_loc_prev.point_source = cem::message::common::PointSource::PS_BEV;
        result.push_back(ego_loc_prev);
      }
    }
    if(debug)AINFO<<"y_offset:"<<y_offset;
  }

  // 3. 头部处理
  int front_upd = mid_idx_upd;
  int front_prev = match_idx_prev;
  y_offset = 0.0f;

  auto head_ref_upd_loc = update_lane.front();
  TransformPoint(&head_ref_upd_loc, T_ego_local_);
  if (front_upd < front_prev && head_ref_upd_loc.x > 0) {
    std::vector<cem::message::common::Point2DF> prepend;

    for (int i = match_idx_prev; i >=0; --i) {
      auto ego_loc_prev = previous_smooth_lane[i];
      TransformPoint(&ego_loc_prev, T_ego_local_);
      if (ego_loc_prev.x < head_ref_upd_loc.x) {
        if (y_offset == 0.0f)
          y_offset = head_ref_upd_loc.y - ego_loc_prev.y;
        ego_loc_prev.y += y_offset;
        TransformPoint(&ego_loc_prev, T_local_ego_);
        ego_loc_prev.point_source = cem::message::common::PointSource::PS_BEV;
        prepend.push_back(ego_loc_prev);
      }
    }
    std::reverse(prepend.begin(),prepend.end());

    result.insert(result.begin(), prepend.begin(), prepend.end());
  }

  return result;
}

void LaneTracker::Update(const std::vector<BevLaneInfo> &detected_lanes,const std::vector<BevLaneMarker> &detected_lms, const Eigen::Isometry3d &T_local_ego,bool add_new,bool debug) {
  T_local_ego_ = T_local_ego;
  T_ego_local_ = T_local_ego.inverse();
  std::unordered_map<uint64_t, bool> matched;
  landmarkers_ = detected_lms;

  for (auto &lm : landmarkers_) {
    for (auto &point : lm.line_points) {
      TransformPoint(&point, T_local_ego_);
    }
  }

  for (auto laneinfo : detected_lanes) {
    for (auto &point : laneinfo.line_points) {
      TransformPoint(&point, T_local_ego_);
    }

    if (laneinfo.line_points.size() < 2) continue;
    
    auto it = tracked_lanes_.find(laneinfo.id);
    if (it != tracked_lanes_.end()) {
      // it->second.Update(laneinfo.line_points);  // 存原始点
      if(add_new)it->second.age++;
      if(add_new)it->second.invisible_count = 0;
      it->second.ori_lane_info   = laneinfo;
      it->second.smoothed_points = laneinfo.line_points;//laneinfo.next_lane_ids.empty()
                                      //  ? SmoothPoints(laneinfo.line_points, it->second.smoothed_points, laneinfo.id == 67)
                                      //  : laneinfo.line_points;
      if (it->second.age >= min_confirmed_age_) {
        it->second.confirmed = true;
      }
      auto left_lm_itr = std::find_if(landmarkers_.begin(), landmarkers_.end(),
                                      [&](const BevLaneMarker &lm) { return lm.id == laneinfo.left_lane_marker_id; });
      if (left_lm_itr != landmarkers_.end())
        it->second.left_lanemarker = *left_lm_itr;
      auto right_lm_itr = std::find_if(landmarkers_.begin(), landmarkers_.end(),
                                      [&](const BevLaneMarker &lm) { return lm.id == laneinfo.right_lane_marker_id; });
      if (right_lm_itr != landmarkers_.end())
        it->second.right_lanemarker = *right_lm_itr;

      matched[laneinfo.id] = true;
    } else {
      vector<int>             matched_old_ids;
      int max_age = 1;
      for (const auto &[old_id, track] : tracked_lanes_) {
        if (matched.find(old_id) != matched.end())
          continue;
        bool ego_overlap =
            track.ori_lane_info.position == 0 && EgoOverlap(laneinfo.line_points, track.smoothed_points);
        if (ego_overlap && DEBUG_INFO)
          AINFO << "old = " << old_id << ", new = " << laneinfo.id << "ego_overlap: " << ego_overlap;
        if (ego_overlap || (track.ori_lane_info.position != 0 &&
                            IsTrackOverlapping(track.smoothed_points, laneinfo.line_points))) {
          max_age = track.age > max_age ? track.age : max_age;
          matched_old_ids.push_back(old_id);
        }
      }

      if (!matched_old_ids.empty()) {
        for (const auto matched_old_id : matched_old_ids) {
          AINFO << "Lane ID reassigned: old = " << matched_old_id << ", new = " << laneinfo.id;
          tracked_lanes_.erase(matched_old_id);
        }
      }
      // 新增目标
      TrackedCrossWalk track;
      track.ori_lane_info   = laneinfo;
      track.age             = max_age;
      track.invisible_count = 0;
      // track.history_lane.push_back(laneinfo.line_points);
      track.smoothed_points = laneinfo.line_points;

      auto left_lm_itr = std::find_if(landmarkers_.begin(), landmarkers_.end(),
                                      [&](const BevLaneMarker &lm) { return lm.id == laneinfo.left_lane_marker_id; });
      if (left_lm_itr != landmarkers_.end())
        track.left_lanemarker = *left_lm_itr;
      auto right_lm_itr = std::find_if(landmarkers_.begin(), landmarkers_.end(),
                                       [&](const BevLaneMarker &lm) { return lm.id == laneinfo.right_lane_marker_id; });
      if (right_lm_itr != landmarkers_.end())
        track.right_lanemarker = *right_lm_itr;

      if (track.age >= min_confirmed_age_) {
        track.confirmed = true;
      }
      tracked_lanes_[laneinfo.id] = track;

      matched[laneinfo.id]        = true;
    }
  }

  // 清理未匹配的轨迹
  for (auto it = tracked_lanes_.begin(); it != tracked_lanes_.end();) {
    if (it->second.smoothed_points.empty()) {
      it = tracked_lanes_.erase(it);
      continue;
    }
    if (matched.find(it->first) == matched.end()) {
      it->second.invisible_count++;
      auto end_pnt_loc = it->second.smoothed_points.back();
      TransformPoint(&end_pnt_loc, T_ego_local_);
      auto front_pnt_loc = it->second.smoothed_points.front();
      TransformPoint(&front_pnt_loc, T_ego_local_);

      if (end_pnt_loc.x < min_invisible_dist_) {
        if (DEBUG_INFO)
          AINFO << "erase lane " << it->first;
        it = tracked_lanes_.erase(it);
      } else if (it->second.invisible_count > 0) {
        if (it->second.ori_lane_info.position != 0 && it->second.smoothed_points.size() < 10) {
          if (DEBUG_INFO)
            AINFO << "erase lane " << it->first;
          it = tracked_lanes_.erase(it);
        } else if (front_pnt_loc.x > 20 && fabs(front_pnt_loc.y) > 2) {
          if (DEBUG_INFO)
            AINFO << "erase lane " << it->first;
          it = tracked_lanes_.erase(it);
        } else if (it->second.ori_lane_info.position != 0 && it->second.smoothed_points.size() < 35 && it->second.invisible_count > (it->second.smoothed_points.size() * 0.4)) {
          if (DEBUG_INFO)
            AINFO << "erase lane " << it->first;
          it = tracked_lanes_.erase(it);
        } else if (it->second.ori_lane_info.position != 0 && it->second.invisible_count > it->second.age) {
          if (DEBUG_INFO)
            AINFO << "erase lane " << it->first;
          it = tracked_lanes_.erase(it);
        } else {
          // 定义 Lambda 函数：解决 ID 冲突并返回新 ID
          auto resolve_id_conflict = [&detected_lms](int original_id) -> std::pair<int, bool> {
            int new_id = original_id;
            int cnt    = 0;
            while (std::find_if(detected_lms.begin(), detected_lms.end(), [&new_id](const BevLaneMarker &lm) { return lm.id == new_id; }) !=
                   detected_lms.end()) {
              new_id++;
              cnt++;
              if (cnt >= 10) {  // 防止无限循环
                new_id = 0;
                break;
              }
            }
            return {new_id, cnt > 0};  // 返回新 ID 和是否发生变更
          };

          // 处理左车道标记 ID
          auto [left_new_id, left_changed] = resolve_id_conflict(it->second.ori_lane_info.left_lane_marker_id);
          if (left_changed) {
            it->second.ori_lane_info.left_lane_marker_id = left_new_id;
            if (it->second.left_lanemarker) {
              it->second.left_lanemarker->id = left_new_id;
            }
          }

          // 处理右车道标记 ID
          auto [right_new_id, right_changed] = resolve_id_conflict(it->second.ori_lane_info.right_lane_marker_id);
          if (right_changed) {
            it->second.ori_lane_info.right_lane_marker_id = right_new_id;
            if (it->second.right_lanemarker) {
              it->second.right_lanemarker->id = right_new_id;
            }
          }

          if(it->second.left_lanemarker.has_value())
            landmarkers_.push_back(it->second.left_lanemarker.value());
          if (it->second.right_lanemarker.has_value())
            landmarkers_.push_back(it->second.right_lanemarker.value());

          CleanOrShrinkTrack(it, matched, debug);
        }
      } else {
        AWARN << "why we come here? no match and no visible? " << it->second.ori_lane_info.id;
        if(debug)it->second.ori_lane_info.is_virtual = true;
        it->second.ori_lane_info.previous_lane_ids.clear();
        it->second.ori_lane_info.next_lane_ids.clear();
        ShrinkPointsByDistance(it->second.smoothed_points, T_ego_local_, min_invisible_dist_);
        ++it;
      }
    } else {
      ++it;
    }
  }
}

void LaneTracker::PrintInputMarkers(const std::vector<BevLaneInfo>& lanes) const {
  AINFO << "[LaneTracker] Input BevLaneInfos (count = " << lanes.size() << "):";
  for (const auto& laneinfo : lanes) {
    AINFO << "  ID = " << laneinfo.id <<",position = " << laneinfo.position;
    // if(laneinfo.id != 79 && laneinfo.id != 78)continue;
    AINFO << "size:"<< laneinfo.line_points.size() << " Points = [";
    AINFO << laneinfo.line_points[0].x << ", " << laneinfo.line_points[0].y << " -> " << laneinfo.line_points[laneinfo.line_points.size()-1].x << ", "
          << laneinfo.line_points[laneinfo.line_points.size()-1].y<< "]";

    for (const auto &next : laneinfo.next_lane_ids) {
      AINFO << "next lane id: " << next;
    }
    for (const auto &pre : laneinfo.previous_lane_ids) {
      AINFO << "previous lane id: " << pre;
    }
    // for (const auto& pt : laneinfo.line_points) {
    //   AINFO << "(" << pt.x << ", " << pt.y << "), ";
    // }
    // AINFO << "]";
  }
}

void LaneTracker::PrintTrackedLanes() const {
  AINFO << "[LaneTracker] Tracked Lanes (count = " << tracked_lanes_.size() << "):";
  for (const auto& [id, track] : tracked_lanes_) {
    AINFO << "  ID = " << id << ",position = " << track.ori_lane_info.position;
    // if (id != 79 && id != 78)
    //   continue;

    AINFO << "size:"<< track.smoothed_points.size() <<"Age = " << track.age << ", Invisible = " << track.invisible_count << ", Confirmed = " << (track.confirmed ? "Yes" : "No")
          << ", Smoothed Points = [";
    // AINFO << track.ori_lane_info.line_points[0].x << ", " << track.ori_lane_info.line_points[0].y << " -> " << track.ori_lane_info.line_points[track.ori_lane_info.line_points.size()-1].x << ", "
    //       << track.ori_lane_info.line_points[track.ori_lane_info.line_points.size()-1].y << "]";
    AINFO << track.smoothed_points[0].x << ", " << track.smoothed_points[0].y << " -> " << track.smoothed_points[track.smoothed_points.size()-1].x << ", "
          << track.smoothed_points[track.smoothed_points.size()-1].y << "]";
    // for (const auto& pt : track.smoothed_points) {
    //   AINFO << "(" << pt.x << ", " << pt.y << "), ";
    // }
    // AINFO << "]";
    AINFO << " track laneMark id:"<< track.left_lanemarker.value().id << "、"<< track.right_lanemarker.value().id;
  }
}

}  // namespace fusion
}  // namespace cem
