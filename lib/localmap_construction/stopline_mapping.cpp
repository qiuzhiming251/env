#include "stopline_mapping.h"

namespace cem {
namespace fusion {

namespace {
constexpr float    MinDistanceStopline          = -10.0;
constexpr float    MaxStoplineRange             = 80.0;
constexpr uint64_t MaxBevID                     = 100;
constexpr double   MaxAssociationDistance       = 5;
constexpr double   MinStoplineLength            = 0.1;
}

StoplineMapping::StoplineMapping() {}

StoplineMapping::~StoplineMapping() {}

void StoplineMapping::Process() {
  BevProcessor();
  FusionNOAMap();
}

void StoplineMapping::FusionNOAMap() {
  stop_line_objs_.erase(std::remove_if(stop_line_objs_.begin(), stop_line_objs_.end(),
                                 [](const cem::message::env_model::StopLine& t) { return t.id > MaxBevID; }),
                  stop_line_objs_.end());

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

  std::vector<cem::message::env_model::StopLine> map_stoplines;
  for (size_t i = 0; i < routing_map_ptr->stop_lines.size(); i++) {
    if (routing_map_ptr->stop_lines[i].points.size() < 2) {
      continue;
    }

    if (std::abs(routing_map_ptr->stop_lines[i].points.front().x) > MaxStoplineRange ||
        std::abs(routing_map_ptr->stop_lines[i].points.front().y) > MaxStoplineRange) {
      continue;
    }

    cem::message::env_model::StopLine map_stopline;
    map_stopline.id = routing_map_ptr->stop_lines[i].id;
    for (const auto& point : routing_map_ptr->stop_lines[i].points) {
      Eigen::Vector3d pt(point.x, point.y, 0);
      map_stopline.ego_line_points.push_back(cem::message::common::Point2DF(pt.x(), pt.y()));
      pt = T_local_ego * pt;
      map_stopline.dr_line_points.push_back(cem::message::common::Point2DF(pt.x(), pt.y()));
    }
    map_stoplines.emplace_back(map_stopline);
  }

  if (map_stoplines.empty()) {
    return;
  }

  for (size_t i = 0; i < map_stoplines.size(); i++) {
    map_stoplines[i].id = MaxBevID + i + 1;
  }

  for (auto& bev_stopline : stop_line_objs_) {
    for (auto& map_stopline : map_stoplines) {
      if (AssociateBevAndMap(bev_stopline, map_stopline)) {
        bev_stopline.dr_line_points = map_stopline.dr_line_points;
        bev_stopline.ego_line_points = map_stopline.ego_line_points;
        map_stopline.is_associated_bev = true;
      }
    }
  }

  for (auto& map_stopline : map_stoplines) {
    if (map_stopline.is_associated_bev) {
      continue;
    }
    stop_line_objs_.emplace_back(map_stopline);
  }

  return;
}

bool StoplineMapping::AssociateBevAndMap(const cem::message::env_model::StopLine& bev_stopline,
                                         const cem::message::env_model::StopLine& noa_stopline) {
  if (bev_stopline.dr_line_points.size() < 2 || noa_stopline.dr_line_points.size() < 2) {
    return false;
  }

  Eigen::Vector2d noa_vector(noa_stopline.dr_line_points.at(1).x - noa_stopline.dr_line_points.at(0).x,
                             noa_stopline.dr_line_points.at(1).y - noa_stopline.dr_line_points.at(0).y);

  Eigen::Vector2d pt1_2_pt1(bev_stopline.dr_line_points.at(0).x - noa_stopline.dr_line_points.at(0).x,
                            bev_stopline.dr_line_points.at(0).y - noa_stopline.dr_line_points.at(0).y);

  Eigen::Vector2d pt1_2_pt2(bev_stopline.dr_line_points.at(1).x - noa_stopline.dr_line_points.at(0).x,
                            bev_stopline.dr_line_points.at(1).y - noa_stopline.dr_line_points.at(0).y);

  double noa_stopline_length = noa_vector.norm();
  if (noa_stopline_length < MinStoplineLength) {
    return false;
  }
  double pt1_projection = noa_vector.dot(pt1_2_pt1) / noa_stopline_length;
  double pt2_projection = noa_vector.dot(pt1_2_pt2) / noa_stopline_length;

  if ((pt1_projection > 0 && pt1_projection < noa_stopline_length) ||
      (pt2_projection > 0 && pt2_projection < noa_stopline_length)) {
    double vertical_distance = std::abs(pt1_2_pt1.x() * noa_vector.y() - pt1_2_pt1.y() * noa_vector.x()) / noa_stopline_length;
    if (vertical_distance <= MaxAssociationDistance) {
      return true;
    }
  }

  return false;
}

void StoplineMapping::BevProcessor() {
    BevMapInfoPtr bev_map_info;
    SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_info);
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
    if (CalcEgoToLocal(bev_map_info)) {
      return;
    }
    if (FeedLatestFrame(bev_map_info)) {
      return;
    }
    LaneAssociation(bev_map_info);
    CalcDisToStopline(bev_map_info);
    SetStopLinesObjs(bev_map_info);
}

void StoplineMapping::Reset() {
  timestamp_ = 0;
  binding_success_ = false;
  distance_to_stopline_ = std::numeric_limits<double>::lowest();
}

bool StoplineMapping::CalcEgoToLocal(cem::fusion::BevMapInfoPtr& bev_info) {
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

bool StoplineMapping::FeedLatestFrame(const cem::fusion::BevMapInfoPtr& bev_info) {
  if (bev_info->stop_lines.empty()) {
    // first empty or dis < MinDistanceStopline
    if (!is_used_stopline_) {
      stop_line_objs_.clear();
      lane_to_stopline_.clear();
      stopline_to_distance_.clear();
      return true;
    }

    // not first empty, dis > 0 ---> keep distance
    // 搜索和stoplines中各个stopline的距离，如果距离小于0，则删除该stopline，保留最小距离
    std::vector<Eigen::Vector2d> ego_stopline_pts;
    double min_dist = std::numeric_limits<double>::max();
    std::set<size_t> delete_ind_set;
    for (size_t ind = 0; ind < stop_line_objs_.size(); ++ind) {
      auto &stop_line_obj = stop_line_objs_[ind];
      for (auto& dr_pt : stop_line_obj.dr_line_points) {
        Eigen::Vector3d dr_stopline_pts_3d(dr_pt.x, dr_pt.y, 0);
        Eigen::Vector3d ego_stopline_pts_3d = T_ego_local_ * dr_stopline_pts_3d;
        ego_stopline_pts.emplace_back(ego_stopline_pts_3d.x(),
                                      ego_stopline_pts_3d.y());
      }
      stop_line_obj.distance_to_stopline =
          0.5 * (ego_stopline_pts.front().x() + ego_stopline_pts.back().x());
      STOP_LINE_MAPPING_LOG << "distance_to_stopline: "
                            << stop_line_obj.distance_to_stopline;

      if (stop_line_obj.distance_to_stopline < MinDistanceStopline) {
        delete_ind_set.insert(ind);
        continue;
      }

      if (stop_line_obj.distance_to_stopline < min_dist) {
        min_dist = stop_line_obj.distance_to_stopline;
      }
    }

    // erase delete_ind_set 中distance < MinDistanceStopline 的元素
    for (size_t i = stop_line_objs_.size(); i > 0; --i) {
      if (delete_ind_set.count(i - 1)) {
          // 删除route_lane_index中的元素
          stop_line_objs_.erase(stop_line_objs_.begin() + (i - 1));
      }
    }

    // 如果与最近的stoplines的距离大于10m，认为是丢失了stopline
    if (min_dist > 10.0) {
      lost_count_++;
    }

    // 如果丢失了50帧，认为是丢失了stopline
    if (lost_count_ > 50) {
      stop_line_objs_.clear();
      lane_to_stopline_.clear();
      stopline_to_distance_.clear();
      return true;
    }
    return true;
  } else {  // has stopline
    lost_count_ = 0;
    timestamp_ = bev_info->header.timestamp;
    stop_line_objs_.clear();
    lane_to_stopline_.clear();
    stopline_to_distance_.clear();
    return false;
  }
}

void StoplineMapping::LaneAssociation(cem::fusion::BevMapInfoPtr& bev_info) {
  auto& lane_infos = bev_info->lane_infos;
  for (auto& lane_info : lane_infos) {
    // STOP_LINE_MAPPING_LOG << lane_info.line_points.front().x << ", "
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
      //   STOP_LINE_MAPPING_LOG << "stop_line: id:" << stop_line.id;
      if (stop_line.line_points.size() < 2) {
        continue;
      }

      auto& stop_line_pts = stop_line.line_points;
      if ((stop_line_pts.front().x < MinDistanceStopline) && (stop_line_pts.back().x < MinDistanceStopline)) {
        // STOP_LINE_MAPPING_LOG << "no binding required: stopline is behind ego
        // veh";
        return;
      } else {
        // STOP_LINE_MAPPING_LOG << "start binding required";
        if ((fabs(stop_line_pts.front().x) - fabs(stop_line_pts.back().x)) <
            2) {
          float stop_line_mid_x =
              (stop_line_pts.front().x + stop_line_pts.back().x) / 2;
          if ((fabs(lane_front_point.x) - fabs(stop_line_mid_x) < 5) &&
              ((lane_front_point.y > stop_line_pts.front().y) &&
               (lane_front_point.y < stop_line_pts.back().y))) {
            binding_success_ = true;
            // STOP_LINE_MAPPING_LOG << "binding success";
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
      while (lane_left_id != 0) {
        auto it_left =
            std::find_if(lane_infos.begin(), lane_infos.end(),
                         [lane_left_id](const BevLaneInfo& lane_infos) {
                           return lane_infos.id == lane_left_id;
                         });
        if (it_left != lane_infos.end()) {
          it_left->stopline_ids.emplace_back(stop_line_id);
          lane_to_stopline_.emplace(it_left->id, stop_line_id);
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
          //   STOP_LINE_MAPPING_LOG << "right lane id : " << it_right->id
          //         << ", stopline id: " << stop_line_id;
        } else {
          break;
        }
        lane_right_id = it_right->right_lane_id;
      }
    }
  }
}

void StoplineMapping::CalcDisToStopline(cem::fusion::BevMapInfoPtr& bev_info) {
  Eigen::Vector2d lane_direction(1.0, 0.0);
  Eigen::Vector2d stopline_direction(0.0, 1.0);
  for (const auto& stop_line : bev_info->stop_lines) {
    auto& stop_line_pts = stop_line.line_points;
    distance_to_stopline_ =
        0.5 * (stop_line_pts.front().x + stop_line_pts.back().x);
    if (distance_to_stopline_ < MinDistanceStopline) {
      distance_to_stopline_ = std::numeric_limits<double>::lowest();
      is_used_stopline_ = false;
    }
    // STOP_LINE_MAPPING_LOG << "distance_to_stopline_: " <<
    // distance_to_stopline_;
    stopline_to_distance_[stop_line.id] = distance_to_stopline_;
    is_used_stopline_ = true;
  }
}

void StoplineMapping::SetStopLinesObjs(cem::fusion::BevMapInfoPtr& bev_info) {
  stop_line_objs_.reserve(bev_info->stop_lines.size());
  auto& stop_lines = bev_info->stop_lines;
  STOP_LINE_MAPPING_LOG << "raw stop_lines size: " << stop_lines.size();

  double min_distance = std::numeric_limits<double>::max();
  uint32_t nearest_stop_line_id = -1;
  for (const auto& stop_line : bev_info->stop_lines) {
    if (stopline_to_distance_.at(stop_line.id) < MinDistanceStopline) {
      continue;
    }
    double mid_point_x = 0.5 * (stop_line.line_points.front().x +
                                stop_line.line_points.back().x);
    double mid_point_y = 0.5 * (stop_line.line_points.front().y +
                                stop_line.line_points.back().y);
    double distance =
        std::sqrt(mid_point_x * mid_point_x + mid_point_y * mid_point_y);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_stop_line_id = stop_line.id;
    }
  }

  auto stop_line =
      std::find_if(stop_lines.begin(), stop_lines.end(),
                   [nearest_stop_line_id](const BevLaneMarker& stop_lines) {
                     return stop_lines.id == nearest_stop_line_id;
                   });
  if (stop_line != stop_lines.end()) {
    StopLine stop_line_obj;
    stop_line_obj.id = stop_line->id;
    stop_line_obj.number_of_points = stop_line->number_of_points;
    stop_line_obj.position = stop_line->position;
    stop_line_obj.type = stop_line->type;
    stop_line_obj.color = stop_line->color;
    stop_line_obj.conf = stop_line->conf;
    stop_line_obj.is_virtual = stop_line->is_virtual;
    for (const auto& stop_line_pts : stop_line->line_points) {
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

void StoplineMapping::Tracking(const cem::fusion::BevMapInfoPtr& bev_info) {
  // 目标：输出状态稳定的stopline目标。  points/id/..
  // TODO：获取单帧感知输入并校验关键感知数据。
}

}  // namespace fusion
}  // namespace cem
