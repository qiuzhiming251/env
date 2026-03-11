#include "stopline_mapping_e2e.h"

namespace cem {
namespace fusion {
namespace e2e {

StoplineMapping::StoplineMapping() {}

StoplineMapping::~StoplineMapping() {}

void StoplineMapping::Process(const BevMapInfoPtr& bev_map_info) {
  if (!bev_map_info) {
    return;
  }
  if (bev_map_info->lane_infos.empty()) {
    return;
  }
  STOP_LINE_MAPPING_LOG << "Run stop_lines";
  STOP_LINE_MAPPING_LOG << "bev_map_sequence_id:"
    << bev_map_info->header.cycle_counter;
  Reset();
  if (CalcEgoToLocal(bev_map_info)) return;
  // LaneAssociation(bev_map_info);
  CalcDisToStopline(bev_map_info);
  SetStopLinesObjs(bev_map_info);
  Tracking(stop_line_objs_);
  GetDebugInfo(stop_line_objs_);
}

void StoplineMapping::Reset() {
  binding_success_ = false;
  maintain_frames_ = std::numeric_limits<int>::max();  // raw 10fps
  distance_to_stopline_ = std::numeric_limits<double>::lowest();
  stopline_map_.clear();
  stop_line_objs_.clear();
  lane_to_stopline_.clear();
  stopline_to_distance_.clear();
  stopline_debug_info_.clear();
  filter_stop_lines_.clear();
  stopline_debug_info_ = "";
}

bool StoplineMapping::CalcEgoToLocal(const BevMapInfoPtr& bev_info) {
  LocalizationPtr local_pose_ptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(
      bev_info->header.timestamp, 0.05, local_pose_ptr);
  if (!local_pose_ptr || prev_bev_timestamp_ == bev_info->header.timestamp) {
    prev_bev_timestamp_ = bev_info->header.timestamp;
    return true;
  }

  T_local_ego_.translation() = Eigen::Vector3d(
      local_pose_ptr->posne_dr.at(0), local_pose_ptr->posne_dr.at(1), 0);

  T_local_ego_.linear() =
      Eigen::AngleAxisd(local_pose_ptr->attitude_dr * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  T_ego_local_ = T_local_ego_.inverse();
  return false;
}

void StoplineMapping::LaneAssociation(const BevMapInfoPtr& bev_info) {
  auto& lane_infos = bev_info->lane_infos;
  for (auto& lane_info : lane_infos) {
    // // STOP_LINE_MAPPING_LOG << lane_info.line_points.front().x << ", "
    //       << lane_info.line_points.back().x;
    if (lane_info.line_points.size() < 2 ||
        lane_info.line_points.front().x > 0.0) {
      continue;
    }

    if (lane_to_stopline_.count(lane_info.id) > 0) {
      continue;
    }
    auto& lane_front_point = lane_info.line_points.back();
    std::vector<cem::message::common::Point2DF> stop_line_pts;
    int stop_line_id = -1;
    for (const auto& stop_line : bev_info->stop_lines) {
      //   // STOP_LINE_MAPPING_LOG << "stop_line: id:" << stop_line.id;
      if (stop_line.line_points.size() < 2) {
        continue;
      }

      auto& stop_line_pts = stop_line.line_points;
      if ((stop_line_pts.front().x < 0) && (stop_line_pts.back().x < 0)) {
        // // STOP_LINE_MAPPING_LOG << "no binding required: stopline is behind
        // ego veh";
        return;
      } else {
        // // STOP_LINE_MAPPING_LOG << "start binding required";
        if ((fabs(stop_line_pts.front().x) - fabs(stop_line_pts.back().x)) <
            2) {
          float stop_line_mid_x =
              (stop_line_pts.front().x + stop_line_pts.back().x) / 2;
          if ((fabs(lane_front_point.x) - fabs(stop_line_mid_x) < 5) &&
              ((lane_front_point.y > stop_line_pts.front().y) &&
               (lane_front_point.y < stop_line_pts.back().y))) {
            binding_success_ = true;
            // // STOP_LINE_MAPPING_LOG << "binding success";
          }
        }
      }

      if (binding_success_) {
        stop_line_id = stop_line.id;
        break;
      }
    }
    if (stop_line_id != -1) {
      lane_info.stopline_ids.emplace_back(stop_line_id);
      lane_to_stopline_.emplace(lane_info.id, stop_line_id);
      uint64_t lane_left_id = lane_info.left_lane_id;
      uint64_t lane_right_id = lane_info.right_lane_id;
      //   // STOP_LINE_MAPPING_LOG << "left lane id : " << lane_left_id
      //         << ", lane_right_id: " << lane_right_id;
      while (lane_left_id != 0) {
        auto it_left =
            std::find_if(lane_infos.begin(), lane_infos.end(),
                         [lane_left_id](const BevLaneInfo& lane_infos) {
                           return lane_infos.id == lane_left_id;
                         });
        if (it_left != lane_infos.end()) {
          it_left->stopline_ids.emplace_back(stop_line_id);
          lane_to_stopline_.emplace(it_left->id, stop_line_id);
          //   // STOP_LINE_MAPPING_LOG << "left lane id : " << it_left->id
          //         << ", stopline id: " << stop_line_id;
        } else {
          break;
        }
        lane_left_id = it_left->left_lane_id;
      }
      while (lane_right_id != 0) {
        auto it_right =
            std::find_if(lane_infos.begin(), lane_infos.end(),
                         [lane_right_id](const BevLaneInfo& lane_infos) {
                           return lane_infos.id == lane_right_id;
                         });
        if (it_right != lane_infos.end()) {
          it_right->stopline_ids.emplace_back(stop_line_id);
          lane_to_stopline_.emplace(it_right->id, stop_line_id);
          //   // STOP_LINE_MAPPING_LOG << "right lane id : " << it_right->id
          //         << ", stopline id: " << stop_line_id;
        } else {
          break;
        }
        lane_right_id = it_right->right_lane_id;
      }
    }
  }
  // // STOP_LINE_MAPPING_LOG << "lane_to_stopline_ :" <<
  // lane_to_stopline_.size(); for (const auto& pair : lane_to_stopline_) {
  //   // STOP_LINE_MAPPING_LOG << "lane id : " << pair.first << ", stopline id:
  //   "
  //   << pair.second;
  // }
}

void StoplineMapping::CalcDisToStopline(const BevMapInfoPtr& bev_info) {
  for (const auto& stop_line : bev_info->stop_lines) {
    auto& stop_line_pts = stop_line.line_points;
    distance_to_stopline_ =
        0.5 * (stop_line_pts.front().x + stop_line_pts.back().x);
    if (distance_to_stopline_ < 0) {
      distance_to_stopline_ = std::numeric_limits<double>::lowest();
      is_used_stopline_ = false;
    }
    // // STOP_LINE_MAPPING_LOG << "distance_to_stopline_: " <<
    // distance_to_stopline_;
    stopline_to_distance_[stop_line.id] = distance_to_stopline_;
    is_used_stopline_ = true;
  }
}

void StoplineMapping::SetStopLinesObjs(const BevMapInfoPtr& bev_info) {
  auto& stop_lines = bev_info->stop_lines;
  STOP_LINE_MAPPING_LOG << "raw stop_lines size: " << stop_lines.size();
  stopline_debug_info_ +=
      fmt::format(" bev_counter:{} ", bev_info->header.cycle_counter);
  stopline_debug_info_ +=
      fmt::format(" raw_stopline_size:{} ", stop_lines.size());
  for (const auto& stop_line : stop_lines) {
    stopline_map_[stop_line.id] = stop_line;
  }

  auto& lane_infos = bev_info->lane_infos;
  double sum = 0.0;
  double avg_lane_x =  std::numeric_limits<double>::max();
  int cnt = 0;
  std::map<double, BevLaneInfo*> lane_distance;
  for (const auto& lane : lane_infos) {
    if (lane.line_points.size() < 2 || lane.line_points.front().x > 0.0 ||
        lane.line_points.back().x < 0.0) {
      continue;
    }
    double distance = FindNearestPoint(lane.line_points, {0.0, 0.0});
    lane_distance[distance] = const_cast<BevLaneInfo*>(&lane);
  }

  for (const auto& pair : lane_distance) {
    if (pair.first > 5) {
      continue;
    }
    auto& lane = *(pair.second);
    // const auto& lane_back_point = (*(pair.second)).line_points.back();
    const auto& lane_front_point = lane.line_points.front();

    // STOP_LINE_MAPPING_LOG << "lane id: " << lane.id
    //                       << " lane distance: " << pair.first
    //                       << " lane_front_point.x: " << lane_front_point.x;

    // sum += lane_front_point.x;
    // cnt++;
  }
  // if (cnt > 0) {
  //   avg_lane_x = sum / cnt;
  // } else {
  //   STOP_LINE_MAPPING_LOG << "no lane to calc stopline";
  // }

  uint64_t nearest_stop_line_id = 0;
  uint64_t best_stop_line_id = 0;
  double min_x_distance = std::numeric_limits<double>::max();
  double max_length = -1.0;
  Eigen::Vector2d ego_lane_sp{0.0, 0.0};
  Eigen::Vector2d ego_lane_ep{0.0, 0.0};
  std::vector<uint64_t> candidate_stop_lines;
  uint64_t ego_lane_id = 0;
  for (const auto& lane : lane_infos) {
    if ((lane.position == (int)BevLanePosition::LANE_LOC_EGO) &&
        (lane.line_points.size() > 1)) {
      ego_lane_id = lane.id;
      STOP_LINE_MAPPING_LOG << "ego id: " << ego_lane_id;
      ego_lane_sp.x() = lane.line_points.front().x;
      ego_lane_ep.x() = lane.line_points.back().x;
      ego_lane_sp.y() = lane.line_points.front().y;
      ego_lane_ep.y() = lane.line_points.back().y;
      break;
    }
  }
  STOP_LINE_MAPPING_LOG << "ego start, end x: " << ego_lane_sp.x() << ", "
                        << ego_lane_ep.x();
  if ((ego_lane_sp.x() < 0) && (ego_lane_ep.x() > -10.0)) {
    for (const auto& stop_line : bev_info->stop_lines) {
      if (stopline_to_distance_.at(stop_line.id) < -3) {
        continue;
      }
      if (stopline_to_distance_.at(stop_line.id) > 200) {
        continue;
      }
      if (stop_line.line_points.size() < 2) {
        continue;
      }
      filter_stop_lines_.emplace_back(stop_line.id);
      for (auto& pt : stop_line.line_points) {
        STOP_LINE_MAPPING_LOG << "pt: " << pt.x << ", " << pt.y;
      }
      STOP_LINE_MAPPING_LOG << "stopline id: " << stop_line.id;
      Eigen::Vector2d mid_point = GetStoplineMidPoint(stop_line);
      // STOP_LINE_MAPPING_LOG << "stopline_midpoint_x, y: " << mid_point.x()
      //                       << ", " << mid_point.y();
      auto sort_stopline = stop_line.line_points;
      std::sort(sort_stopline.begin(), sort_stopline.end(),
                [](const Point2DF& a, const Point2DF& b) { return a.y > b.y; });
      double stopline_left_y = sort_stopline.front().y + 1.0;
      double stopline_right_y = sort_stopline.back().y - 1.0;
      STOP_LINE_MAPPING_LOG << "stopline left, right y: " << stopline_left_y
                            << ", " << stopline_right_y;
      stopline_debug_info_ += fmt::format(
          " stopline id:{} , stopline left y {:.2f}, right y {:.2f}",
          stop_line.id, stopline_left_y, stopline_right_y);
      if ((stopline_left_y * stopline_right_y) > 0) {
        continue;
      }

      if (fabs(mid_point.y()) > 10.0) {
        continue;
      }

      if (mid_point.x() < min_x_distance) {
        min_x_distance = mid_point.x();
        nearest_stop_line_id = stop_line.id;
      }
      candidate_stop_lines.emplace_back(stop_line.id);
    }
  }

  for (uint64_t candidate_id : candidate_stop_lines) {
    double stopline_x_thres = min_x_distance + 5.0;
    auto stop_line = stopline_map_.at(candidate_id);
    Eigen::Vector2d mid_point = GetStoplineMidPoint(stop_line);
    if (mid_point.x() < stopline_x_thres) {
      double stopline_length = std::fabs(stop_line.line_points.front().y -
                                         stop_line.line_points.back().y);
      if (stopline_length > max_length) {
        max_length = stopline_length;
        best_stop_line_id = candidate_id;
        STOP_LINE_MAPPING_LOG << "best_stop_line_id: " << best_stop_line_id
                              << ", " << stopline_length;
      }
    }
  }

  /// id lock
  STOP_LINE_MAPPING_LOG << "best and prev best id: " << best_stop_line_id
                        << ", " << prev_best_stop_line_id_;
  stopline_debug_info_ +=
      fmt::format(" pre_best_id:{}, best_id:{} ", prev_best_stop_line_id_,
                  best_stop_line_id);
  bool is_in_id_lock = false;
  auto it_prev = stopline_map_.find(prev_best_stop_line_id_);
  auto it_best = stopline_map_.find(best_stop_line_id);
  if ((it_prev != stopline_map_.end()) && (it_best != stopline_map_.end()) &&
      (best_stop_line_id != prev_best_stop_line_id_)) {
    if (stopline_to_distance_.at(best_stop_line_id) < 10.0) {
      auto best_stop_line = stopline_map_.at(best_stop_line_id);
      auto prev_best_stop_line = stopline_map_.at(prev_best_stop_line_id_);
      double delta_x = std::fabs(GetStoplineMidPoint(best_stop_line).x() -
                                 GetStoplineMidPoint(prev_best_stop_line).x());
      if (delta_x < 3.0) {
        is_in_id_lock = true;
        STOP_LINE_MAPPING_LOG << "enter in id lock";
        best_stop_line_id = prev_best_stop_line_id_;
      }
    }
  }

  stopline_debug_info_ += fmt::format(" enter_in_id_lock:{} ", is_in_id_lock);
  prev_best_stop_line_id_ = best_stop_line_id;

  // curve scene
  if ((best_stop_line_id == 0) && (!filter_stop_lines_.empty())) {
    STOP_LINE_MAPPING_LOG << "enter in curve scene";
    auto& lane_infos = bev_info->lane_infos;
    auto ego_lane = std::find_if(lane_infos.begin(), lane_infos.end(),
                                 [ego_lane_id](const BevLaneInfo& lane_infos) {
                                   return lane_infos.id == ego_lane_id;
                                 });
    if (ego_lane != lane_infos.end()) {
      for (auto& stop_line_id : filter_stop_lines_) {
        auto stop_line = stopline_map_.at(stop_line_id);
        float distance = Nearest_lane_point_to_stopline(ego_lane->line_points,
                                                        stop_line.line_points);
        STOP_LINE_MAPPING_LOG << "stopline id, dist: " << stop_line_id << ", "
                              << distance;
        if (distance < 1.2) {
          best_stop_line_id = stop_line_id;
        }
      }
    }
  }

  auto stop_line =
      std::find_if(stop_lines.begin(), stop_lines.end(),
                   [best_stop_line_id](const BevLaneMarker& stop_lines) {
                     return stop_lines.id == best_stop_line_id;
                   });
  if (stop_line != stop_lines.end()) {
    std::vector<BevLaneMarker> nearbyStopLines;
    std::vector<cem::fusion::Point2DF> total_stopline_pts;
    nearbyStopLines.clear();
    /*
    nearbyStopLines =
        SearchNearbyStopLines(*stop_line, stop_lines);
    nearbyStopLines.emplace_back(*stop_line);
    uint64_t left_stopline_id = 0;
    uint64_t right_stopline_id = 0;

    if (nearbyStopLines.size() > 1) {
      double min_mid_y = 1e9;
      double max_mid_y = -1e9;
      for (const auto& stopline : nearbyStopLines) {
        Eigen::Vector2d mid_point = GetStoplineMidPoint(stopline);
        if (mid_point.y() < min_mid_y) {
          min_mid_y = mid_point.y();
          right_stopline_id = stopline.id;
        }
        if (mid_point.y() > max_mid_y) {
          max_mid_y = mid_point.y();
          left_stopline_id = stopline.id;
        }
      }
      STOP_LINE_MAPPING_LOG << "left, right id: " << left_stopline_id << ", "
                            << right_stopline_id;
      if ((left_stopline_id != -1) && (right_stopline_id != -1)) {
        auto left_stopline_pts =
    stopline_map_.at(left_stopline_id).line_points; auto right_stopline_pts =
            stopline_map_.at(right_stopline_id).line_points;
        std::sort(
            left_stopline_pts.begin(), left_stopline_pts.end(),
            [](const Point2DF& a, const Point2DF& b) { return a.y > b.y; });
        std::sort(
            right_stopline_pts.begin(), right_stopline_pts.end(),
            [](const Point2DF& a, const Point2DF& b) { return a.y < b.y; });
        // for (auto& left_pts : left_stopline_pts) {
        //   STOP_LINE_MAPPING_LOG << "left_pts.x , y: (" << left_pts.x << ","
        //                         << left_pts.y << ")";
        // }
        // for (auto& right_pts : right_stopline_pts) {
        //   STOP_LINE_MAPPING_LOG << "right_pts.x , y: (" << right_pts.x <<
    ","
        //                         << right_pts.y << ")";
        // }
        // keep two stopline pts
        total_stopline_pts.emplace_back(left_stopline_pts.front());
        total_stopline_pts.emplace_back(right_stopline_pts.front());
      }
    }
    */
    StopLine stop_line_obj;
    stop_line_obj.id = stop_line->id;
    stop_line_obj.number_of_points = stop_line->number_of_points;
    stop_line_obj.position = stop_line->position;
    stop_line_obj.type = stop_line->type;
    stop_line_obj.color = stop_line->color;
    stop_line_obj.conf = stop_line->conf;
    stop_line_obj.is_virtual = stop_line->is_virtual;
    stop_line_obj.is_using_last_valid = false;
    std::vector<cem::fusion::Point2DF> final_stopline_pts;
    if (nearbyStopLines.size() > 1) {
      final_stopline_pts = total_stopline_pts;
    } else {
      auto sort_stopline_y = stop_line->line_points;
      std::sort(sort_stopline_y.begin(), sort_stopline_y.end(),
                [](const Point2DF& a, const Point2DF& b) { return a.y > b.y; });
      final_stopline_pts.emplace_back(sort_stopline_y.front());
      final_stopline_pts.emplace_back(sort_stopline_y.back());
    }
    for (const auto& stop_line_pts : final_stopline_pts) {
      stop_line_obj.ego_line_points.emplace_back(stop_line_pts);
      Eigen::Vector3d ego_stopline_pts(stop_line_pts.x, stop_line_pts.y, 0);
      Eigen::Vector3d dr_stopline_pts_3d = T_local_ego_ * ego_stopline_pts;
      cem::message::common::Point2DF dr_stopline_pts;
      dr_stopline_pts.x = dr_stopline_pts_3d.x();
      dr_stopline_pts.y = dr_stopline_pts_3d.y();
      stop_line_obj.dr_line_points.emplace_back(dr_stopline_pts);
    }

    stop_line_obj.distance_to_stopline =
        stopline_to_distance_.at(stop_line->id);
    stop_line_objs_.emplace_back(stop_line_obj);
  }
}

void StoplineMapping::Tracking(
    std::vector<cem::message::env_model::StopLine>& stop_line_objs) {
  if (!stop_line_objs.empty()) {
    last_valid_stop_lines_.clear();
    last_valid_stop_lines_ = stop_line_objs;
    STOP_LINE_MAPPING_LOG << "last_valid_stop_lines_.size(): "
                          << last_valid_stop_lines_.size();
    missing_frames_ = 0;
  } else {
    missing_frames_++;
    STOP_LINE_MAPPING_LOG << "missing_frames_: " << missing_frames_;
    if (missing_frames_ <= maintain_frames_) {
      last_valid_stop_lines_.erase(
          remove_if(last_valid_stop_lines_.begin(),
                    last_valid_stop_lines_.end(),
                    [](const StopLine& line) {
                      return line.distance_to_stopline < -3.0;
                    }),
          last_valid_stop_lines_.end());
      for (auto& last_valid_stop_line : last_valid_stop_lines_) {
        last_valid_stop_line.is_using_last_valid = true;
        last_valid_stop_line.ego_line_points.clear();
        for (auto& dr_pt : last_valid_stop_line.dr_line_points) {
          Eigen::Vector3d dr_stopline_pts_3d(dr_pt.x, dr_pt.y, 0);
          Eigen::Vector3d ego_stopline_pts_3d =
              T_ego_local_ * dr_stopline_pts_3d;
          last_valid_stop_line.ego_line_points.emplace_back(
              ego_stopline_pts_3d.x(), ego_stopline_pts_3d.y());
        }

        last_valid_stop_line.distance_to_stopline =
            0.5 * ((last_valid_stop_line.ego_line_points).front().x +
                   (last_valid_stop_line.ego_line_points).back().x);
      }
      stop_line_objs_ = last_valid_stop_lines_;
      STOP_LINE_MAPPING_LOG << "last_valid_stop_lines_.size(): "
                            << last_valid_stop_lines_.size();
    } else {
      stop_line_objs_.clear();
      last_valid_stop_lines_.clear();
    }
  }
}

void StoplineMapping::GetDebugInfo(
    const std::vector<cem::message::env_model::StopLine>& stop_line_objs) {
  for (const auto& stop_line_obj : stop_line_objs) {
    STOP_LINE_MAPPING_LOG << "------stop_line_obj------";
    STOP_LINE_MAPPING_LOG << "stop_line: id:" << stop_line_obj.id;
    STOP_LINE_MAPPING_LOG << "number of points:"
                          << int(stop_line_obj.number_of_points);
    STOP_LINE_MAPPING_LOG << "position: " << int(stop_line_obj.position);
    STOP_LINE_MAPPING_LOG << "type: " << int(stop_line_obj.type);
    STOP_LINE_MAPPING_LOG << "color: " << int(stop_line_obj.color);
    STOP_LINE_MAPPING_LOG << "conf: " << stop_line_obj.conf;
    STOP_LINE_MAPPING_LOG << "is_virtual: " << stop_line_obj.is_virtual;
    STOP_LINE_MAPPING_LOG << "is_using_last_valid: "
                          << stop_line_obj.is_using_last_valid;
    STOP_LINE_MAPPING_LOG << "distance_to_stopline: "
                          << stop_line_obj.distance_to_stopline;
    for (auto& ego_pts : stop_line_obj.ego_line_points) {
      STOP_LINE_MAPPING_LOG << "ego_line_points.x , y: (" << ego_pts.x << ","
                            << ego_pts.y << ")";
    }
    for (auto& dr_pts : stop_line_obj.dr_line_points) {
      STOP_LINE_MAPPING_LOG << "dr_line_points.x , y: (" << dr_pts.x << ","
                            << dr_pts.y << ")";
    }
    stopline_debug_info_ += fmt::format(" stop_line: id:{} ", stop_line_obj.id);
    stopline_debug_info_ +=
        fmt::format(" number_of_points:{} ", stop_line_obj.number_of_points);
    stopline_debug_info_ += fmt::format(" is_using_last_valid:{} ",
                                        stop_line_obj.is_using_last_valid);
    stopline_debug_info_ += fmt::format(" distance_to_stopline:{:.2f} ",
                                        stop_line_obj.distance_to_stopline);
    for (auto& ego_pts : stop_line_obj.ego_line_points) {
      stopline_debug_info_ += fmt::format(
          " ego_line_points:x ={:.2f}, y={:.2f}", ego_pts.x, ego_pts.y);
    }
    for (auto& dr_pts : stop_line_obj.dr_line_points) {
      stopline_debug_info_ +=
          fmt::format(" dr_line_points_x:{:.2f}, y={:.2f}", dr_pts.x, dr_pts.y);
    }
  }
}

std::vector<BevLaneMarker> StoplineMapping::SearchNearbyStopLines(
    const BevLaneMarker& known_stopline,
    const std::vector<BevLaneMarker>& stoplines) {
  std::vector<BevLaneMarker> result;
  if (stoplines.size() < 2) {
    return result;
  }

  std::queue<BevLaneMarker> q;
  std::set<int> visited;

  q.push(known_stopline);
  visited.insert(known_stopline.id);

  const double MAX_DISTANCE = 6.0;
  while (!q.empty()) {
    BevLaneMarker current_line = q.front();
    q.pop();

    for (const auto& near_line : stoplines) {
      if (visited.find(near_line.id) == visited.end()) {
        Eigen::Vector2d curr_mid_pt = GetStoplineMidPoint(current_line);
        Eigen::Vector2d near_mid_pt = GetStoplineMidPoint(near_line);
        double distance = sqrt(pow(curr_mid_pt.x() - near_mid_pt.x(), 2) +
                               pow(curr_mid_pt.y() - near_mid_pt.y(), 2));
        if (distance < MAX_DISTANCE) {
          result.emplace_back(near_line);
          visited.insert(near_line.id);
          q.push(near_line);
        }
      }
    }
  }
  for (auto& nearby_stopline : result) {
    STOP_LINE_MAPPING_LOG << "nearby_stopline: " << nearby_stopline.id;
  }

  return result;
}

double StoplineMapping::FindNearestPoint(
    const std::vector<Point2DF>& lane_points, const Point2DF& target_point) {
  double min_distance = std::numeric_limits<double>::max();
  Point2DF closest_point = lane_points.at(0);

  for (const auto& point : lane_points) {
    double distance = CalcDistance(point, target_point);
    if (distance < min_distance) {
      min_distance = distance;
      closest_point = point;
    }
  }

  return min_distance;
}

Eigen::Vector2d StoplineMapping::GetStoplineMidPoint(
    const BevLaneMarker& stopline) {
  if (stopline.line_points.size() < 2) {
    return Eigen::Vector2d{-1.0, 0.0};
  }
  double sum_x = 0;
  double sum_y = 0;
  for (const auto& point : stopline.line_points) {
    sum_x += point.x;
    sum_y += point.y;
  }
  double mid_x = sum_x / static_cast<int>(stopline.line_points.size());
  double mid_y = sum_y / static_cast<int>(stopline.line_points.size());

  return Eigen::Vector2d{mid_x, mid_y};
}

double StoplineMapping::CalcDistance(const Point2DF& p1, const Point2DF& p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y));
}

float StoplineMapping::Nearest_lane_point_to_stopline(
    const std::vector<Point2DF>& lane_points,
    const std::vector<Point2DF>& stopline_points) {
  float best_dist = std::numeric_limits<float>::max();
  for (size_t i = 0; i < lane_points.size(); ++i) {
    best_dist = std::min(
        best_dist, Point_to_segment(lane_points.at(i), stopline_points.front(),
                                    stopline_points.back()));
  }
  return best_dist;
}

float StoplineMapping::Point_to_segment(const Point2DF& p, const Point2DF& a,
                                        const Point2DF& b) {
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  float seg = dx * dx + dy * dy;
  if (seg < 1e-8f) return std::hypot(p.x - a.x, p.y - a.y);

  // 投影系数 t [0,1]
  float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / seg;
  t = std::max(0.0f, std::min(1.0f, t));

  float qx = a.x + t * dx;
  float qy = a.y + t * dy;
  return std::hypot(p.x - qx, p.y - qy);
}

}
}  // namespace fusion
}  // namespace cem

