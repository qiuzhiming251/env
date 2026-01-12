#include "lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.h"
// #define AddTopoDebug

using cem::message::common::Point2DF;
using cem::message::sensor::BevArrowType;
using cem::message::sensor::BevLaneInfo;
using cem::message::sensor::BevLaneMarker;
using cem::message::sensor::BevLaneMarkerType;
using cem::message::sensor::BevLanePosition;
using cem::message::sensor::BevMapInfo;

namespace cem {
namespace fusion {

namespace {

constexpr float kEmergencyLaneMinWidth = 2;
constexpr float kEmergencyLaneMaxWidth = 3.3;
constexpr float kEmergencyFilterMaxWidth = 5;

}  // namespace

BevMapProcessor::BevMapProcessor()
    : lane_tracker_ptr_(std::make_shared<cem::fusion::LaneTracker>()),
      diversion_ptr_(std::make_shared<cem::fusion::DiversionTracker>()){}

BevMapProcessor::~BevMapProcessor() {}

void BevMapProcessor::Proc() {
  lane_map_.clear();
}

std::optional<Eigen::Isometry3d> BevMapProcessor::FindTransform(
    const double& timestamp) {
  LocalizationPtr odom_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05,
                                                      odom_ptr);

  if (odom_ptr == nullptr) {
    return std::nullopt;
  }

  Eigen::Isometry3d T_local_ego = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd R_local_ego((odom_ptr->attitude_dr) * M_PI / 180.0,
                                Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d t(odom_ptr->posne_dr.at(0), odom_ptr->posne_dr.at(1), 0.0);
  T_local_ego.rotate(R_local_ego);
  T_local_ego.pretranslate(t);

  return T_local_ego;
}

void BevMapProcessor::FillNearLaneId() {
    // 构造从车道位置到车道信息的映射
    std::unordered_map<int, BevLaneInfo*> pos_to_lane;
    for (auto& lane : data_->lane_infos) {    
        if (lane.position == static_cast<int>(BevLanePosition::LANE_LOC_OTHER)) {
            lane.left_lane_id  = 0;
            lane.right_lane_id = 0;
        } else {
            // 检查是否存在重复 position
            if (pos_to_lane.find(lane.position) != pos_to_lane.end()) {
                AWARN << "Duplicate lane position detected: " << lane.position 
                      << " for lane ID " << lane.id;
            }
            pos_to_lane[lane.position] = &lane;
        }
    }
    const std::array<int, 17> pos_order = {-1, 14, 12, 10, 8, 5, 3, 1, 0, 2, 4, 6, 9, 11, 13, 15, -1};

    for (size_t i = 1; i < pos_order.size() - 1; ++i) {
        int cur_pos = pos_order[i];
        auto cur_it = pos_to_lane.find(cur_pos);
        if (cur_it == pos_to_lane.end()) {
            continue; 
        }

        // 查找左邻车道
        uint64_t left_id = 0;
        int left_pos = pos_order[i - 1];
        if (left_pos != -1) {
            auto left_it = pos_to_lane.find(left_pos);
            if (left_it != pos_to_lane.end()) {
                left_id = left_it->second->id;
            }
        }

        // 查找右邻车道
        uint64_t right_id = 0;
        int right_pos = pos_order[i + 1];
        if (right_pos != -1) {
            auto right_it = pos_to_lane.find(right_pos);
            if (right_it != pos_to_lane.end()) {
                right_id = right_it->second->id;
            }
        }
        cur_it->second->left_lane_id  = left_id;
        cur_it->second->right_lane_id = right_id;
        // AINFO << "Updated lane ID " << cur_it->second->id 
        //       << " at position " << cur_pos 
        //       << " with left_lane_id: " << left_id 
        //       << " and right_lane_id: " << right_id;
    }
}

bool BevMapProcessor::RotateAndTranslate(const Eigen::Isometry3d& T) {
  auto t_x = T.translation().x();
  auto lambda_lanemarker_trans = [&T, &t_x](BevLaneMarker& marker) {
    for (auto& point : marker.line_points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
    for (auto& type_seg : marker.type_segs) {
      type_seg.start_offset += t_x;
      type_seg.end_offset += t_x;
    }
    for (auto& color_seg : marker.type_segs) {
      color_seg.start_offset += t_x;
      color_seg.end_offset += t_x;
    }
    for (auto& merged_seg : marker.merged_segs) {
      merged_seg.start_offset += t_x;
      merged_seg.end_offset += t_x;
    }
  };

  for (auto& marker : data_->lanemarkers) {
    lambda_lanemarker_trans(marker);
  }

  for (auto& edge : data_->edges) {
    lambda_lanemarker_trans(edge);
  }

  for (auto& lane : data_->lane_infos) {
    for (auto& point : lane.line_points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
    lane.start_dx += t_x;
    lane.end_dx += t_x;
  }

  for (auto& stop_line : data_->stop_lines) {
    lambda_lanemarker_trans(stop_line);
  }

  for (auto& crosswalk : data_->crosswalks) {
    lambda_lanemarker_trans(crosswalk);
  }

  for (auto& arrow : data_->arrows) {
    lambda_lanemarker_trans(arrow);
  }

  data_->route.navi_start.s_offset += t_x;
  // for (auto& section : data_->route.sections) {
  //   for (auto& point : section.points) {
  //     Eigen::Vector3d point_src(point.x, point.y, 0);
  //     auto point_dst = T * point_src;
  //     point.x = point_dst.x();
  //     point.y = point_dst.y();
  //   }
  // }

  return true;
}

void BevMapProcessor::EsitmateSliceParameters(
    const Eigen::Vector2d& road_direction, Eigen::Vector2d& slice_start_point,
    double& slice_length) {
  double back_length = std::numeric_limits<double>::max();
  double front_length = std::numeric_limits<double>::min();
  bool has_lane = false;
  for (const auto& lane : data_->lane_infos) {
    if (lane.line_points.empty()) {
      continue;
    }
    has_lane = true;
    Eigen::Vector2d pt;
    pt.x() = lane.line_points.front().x;
    pt.y() = lane.line_points.front().y;
    double tmp = pt.dot(road_direction);
    back_length = tmp < back_length ? tmp : back_length;

    pt.x() = lane.line_points.back().x;
    pt.y() = lane.line_points.back().y;
    tmp = pt.dot(road_direction);
    front_length = tmp > front_length ? tmp : front_length;
  }
  if (!has_lane) {
    slice_start_point.x() = 0.0;
    slice_start_point.y() = 0.0;
    slice_length = 0.0;
  } else {
    back_length = back_length < -50.0 ? -50.0 : back_length;
    // front_length = front_length > 100.0 ? 100.0 : front_length;
    slice_start_point = back_length * road_direction;
    slice_length = front_length - back_length;
  }
}

void BevMapProcessor::ConstructLanesIndex() {
  lane_map_.clear();
  for (auto& lane : data_->lane_infos) {
    lane_map_.emplace(lane.id, &lane);
  }
}

void BevMapProcessor::ConfirmEmergencyLane(const std::function<LaneType(uint64_t)> &LaneTypeGetter, std::string &debug_infos,
                                           bool EgoIsEmergency) {
  if (data_ == nullptr) {
    return;
  }

  const bool DEBUG_EL = false;
  std::vector<BevLaneInfo*> emergency_lanes;
  std::unordered_map<uint64_t,bool> confirm_map;
  std::unordered_map<uint64_t, BevLaneInfo*> lane_map;
  BevLaneInfo* ego_lane_ptr = nullptr;
  emergency_lanes.reserve(data_->lane_infos.size());
  static const std::unordered_set<LaneType> kProhibitedTypes = {LaneType::LANE_EMERGENCY, LaneType::LANE_HARBOR_STOP,
                                                                LaneType::LANE_DIVERSION, LaneType::LANE_NON_MOTOR};
  debug_infos += "[emergency lane]before:";
  if (DEBUG_EL)
    AINFO << "before:";
  for (auto &lane : data_->lane_infos) {
    lane_map[lane.id] = &lane;
    if (lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY || lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY_STOP ||
        lane.lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
      debug_infos += std::to_string(lane.id) + ";";
      if (DEBUG_EL)
        AINFO << "lane id: " << lane.id;
      emergency_lanes.push_back(&lane);
    }
    if (!ego_lane_ptr && lane.position == 0) {
      ego_lane_ptr = &lane;
      debug_infos += "ego lane id: " + std::to_string(ego_lane_ptr->id) + ";";
      if (DEBUG_EL)
        AINFO << "ego lane id: " << ego_lane_ptr->id;
      //检查漏检的ego应急车道
      if(emergency_lanes.empty()||emergency_lanes.back()->id!=ego_lane_ptr->id){
        auto lane_type = LaneTypeGetter(ego_lane_ptr->id);
        if (DEBUG_EL)
          AINFO << ego_lane_ptr->id << ",lane_type: " << static_cast<int>(lane_type);
        debug_infos += std::to_string(ego_lane_ptr->id) + ",lane_type: " + std::to_string(static_cast<int>(lane_type)) + ";";
        if (kProhibitedTypes.find(lane_type) != kProhibitedTypes.end() && EgoIsEmergency) {
          ego_lane_ptr->lane_type = BevLaneType::LANE_TYPE_EMERGENCY;
          emergency_lanes.push_back(ego_lane_ptr);
          if (DEBUG_EL)
            AINFO << "ego changed to emergency lane!";
          debug_infos += "ego changed to emergency lane!;";
        }
      }
    }
  }
  if (emergency_lanes.empty())
    return;

  auto try_set_emergency_from_ids = [&](const std::vector<uint64_t> &ids) {
    for (auto id : ids) {
      BevLaneInfo *itr = lane_map[id];
      if (DEBUG_EL)
        AINFO << "lane:" << id;
      if (itr) {
        auto lane_type = LaneTypeGetter(id);
        debug_infos += std::to_string(id) + ",lane_type: " + std::to_string(static_cast<int>(lane_type)) + ";";
        if (DEBUG_EL)
          AINFO << id << ",lane_type: " << static_cast<int>(lane_type);
        if (kProhibitedTypes.find(lane_type) != kProhibitedTypes.end()) {
          // itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
          confirm_map[id] = true;
          auto another_id      = std::find_if(ids.begin(), ids.end(),
                                              [id](uint64_t aid) { return aid != id; });
          if (another_id != ids.end()) {
            BevLaneInfo *another_itr = lane_map[*another_id];
            if (another_itr) {
              another_itr->lane_type  = BevLaneType::LANE_TYPE_UNKNOWN;
              confirm_map[another_itr->id] = true;
              break;
            }
          }
        }
      }
    }
  };

  for (BevLaneInfo *lane_ptr : emergency_lanes) {
    if (!lane_ptr)
      continue;

    //一分二、二合一的一纠正
    if (lane_ptr->next_lane_ids.size() == 2 || lane_ptr->previous_lane_ids.size() == 2) {
      lane_ptr->lane_type = BevLaneType::LANE_TYPE_UNKNOWN;
      confirm_map[lane_ptr->id] = true;
      if (lane_ptr->next_lane_ids.size() == 2) {
        try_set_emergency_from_ids(lane_ptr->next_lane_ids);
      } else {
        try_set_emergency_from_ids(lane_ptr->previous_lane_ids);
      }
    }
    //地图校验
    if (!confirm_map[lane_ptr->id]) {
      auto lane_type = LaneTypeGetter(lane_ptr->id);
      debug_infos += std::to_string(lane_ptr->id) + ",lane_type: " + std::to_string(static_cast<int>(lane_type)) + ";";
      if (DEBUG_EL)
        AINFO << lane_ptr->id << ",lane_type: " << static_cast<int>(lane_type);

      if (kProhibitedTypes.find(lane_type) != kProhibitedTypes.end()) {
        confirm_map[lane_ptr->id] = true;
      } else if (lane_type != LaneType::LANE_UNKNOWN) {
        lane_ptr->lane_type = BevLaneType::LANE_TYPE_UNKNOWN;
      }
    }

    //其他拓扑纠正
    if (!confirm_map[lane_ptr->id] && lane_ptr->next_lane_ids.size() != 2 && lane_ptr->previous_lane_ids.size() != 2) {
      if (!ego_lane_ptr)
        continue;
      auto em_lane_type = LaneTypeGetter(lane_ptr->id);

      //ego == Emergency
      if (ego_lane_ptr->id == lane_ptr->id) {
        if (ego_lane_ptr->previous_lane_ids.size() != 1 && ego_lane_ptr->next_lane_ids.size() != 1) {
          if (DEBUG_EL)
            AINFO << lane_ptr->id << ",ego == Emergency, but previous/next size != 1";
          continue;
        }
        //ego为split/merge二之一
        bool has_confirm = false;
        if (ego_lane_ptr->previous_lane_ids.size() == 1) {
          BevLaneInfo *split_itr = lane_map[ego_lane_ptr->previous_lane_ids[0]];
          if (split_itr) {
            if (split_itr->next_lane_ids.size() == 2) {
              bool ego_on_right = std::find(split_itr->next_lane_ids.begin(), split_itr->next_lane_ids.end(), ego_lane_ptr->left_lane_id) !=
                                  split_itr->next_lane_ids.end();
              bool ego_on_left = std::find(split_itr->next_lane_ids.begin(), split_itr->next_lane_ids.end(), ego_lane_ptr->right_lane_id) !=
                                 split_itr->next_lane_ids.end();
              if (ego_on_right || ego_on_left) {
                has_confirm    = true;
                uint16_t another_id = 0;
                if (ego_on_right) {
                  another_id = ego_lane_ptr->left_lane_id;
                } else {
                  another_id = ego_lane_ptr->right_lane_id;
                }
                bool change = em_lane_type != LaneType::LANE_UNKNOWN;
                if (!change) {
                  auto lane_type = LaneTypeGetter(another_id);
                  if (DEBUG_EL)
                    AINFO << another_id << ",lane_type: " << static_cast<int>(lane_type);

                  if (kProhibitedTypes.find(lane_type) != kProhibitedTypes.end()) {
                    change = true;
                  }
                  else if (ego_on_left && ego_lane_ptr->left_lane_id != 0) {
                    change = true;
                  }
                  else if (ego_on_right && ego_lane_ptr->right_lane_id != 0) {
                    change = true;
                  }
                }
                if (!confirm_map[another_id] && change) {
                  BevLaneInfo *another_itr = lane_map[another_id];

                  if (another_itr) {
                    // another_itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
                    confirm_map[another_itr->id] = true;
                    lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                    confirm_map[lane_ptr->id]    = true;
                  }
                }
              }
            }
          }
        }
        if (!has_confirm && ego_lane_ptr->next_lane_ids.size() == 1) {
          BevLaneInfo *merge_itr = lane_map[ego_lane_ptr->next_lane_ids[0]];
          if (merge_itr) {
            if (merge_itr->previous_lane_ids.size() == 2) {
              bool ego_on_right = std::find(merge_itr->previous_lane_ids.begin(), merge_itr->previous_lane_ids.end(),
                                            ego_lane_ptr->left_lane_id) != merge_itr->previous_lane_ids.end();
              bool ego_on_left  = std::find(merge_itr->previous_lane_ids.begin(), merge_itr->previous_lane_ids.end(),
                                            ego_lane_ptr->right_lane_id) != merge_itr->previous_lane_ids.end();
              if (ego_on_right || ego_on_left) {
                has_confirm         = true;
                uint16_t another_id = 0;
                if (ego_on_right) {
                  another_id = ego_lane_ptr->left_lane_id;
                } else {
                  another_id = ego_lane_ptr->right_lane_id;
                }
                bool change = em_lane_type != LaneType::LANE_UNKNOWN;
                if (!change) {
                  auto lane_type = LaneTypeGetter(another_id);
                  if (DEBUG_EL)
                    AINFO << another_id << ",lane_type: " << static_cast<int>(lane_type);

                  if (kProhibitedTypes.find(lane_type) != kProhibitedTypes.end()) {
                    change = true;
                  }
                }

                if (!confirm_map[another_id] && change) {
                  BevLaneInfo *another_itr = lane_map[another_id];
                  if (another_itr) {
                    lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                    confirm_map[lane_ptr->id] = true;
                    // another_itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
                    confirm_map[another_itr->id] = true;
                  }
                }
              }
            }
          }
        }
      }
      //ego 后继有拓扑并包含Emergency
      else if (std::find(ego_lane_ptr->next_lane_ids.begin(), ego_lane_ptr->next_lane_ids.end(), lane_ptr->id) !=
               ego_lane_ptr->next_lane_ids.end()) {
        //ego看左/右应急
        if (ego_lane_ptr->left_lane_id != 0 && ego_lane_ptr->right_lane_id == 0) {
          if (DEBUG_EL)
            AINFO << lane_ptr->id << ",ego->Emergency:right ";

          //右应急
          bool     has_confirm  = false;
          uint16_t emergency_id = lane_ptr->id;
          //split
          if (ego_lane_ptr->next_lane_ids.size() == 2) {
            auto another_id = std::find_if(ego_lane_ptr->next_lane_ids.begin(), ego_lane_ptr->next_lane_ids.end(),
                                           [emergency_id](uint64_t id) { return id != emergency_id; });
            if (another_id != ego_lane_ptr->next_lane_ids.end()) {
              BevLaneInfo *another_itr = lane_map[*another_id];
              if (another_itr && !confirm_map[another_itr->id]) {
                bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                has_confirm  = true;
                if (is_left) {
                  lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                  confirm_map[lane_ptr->id]    = true;
                  // another_itr->lane_type = BevLaneType::LANE_TYPE_EMERGENCY;
                  confirm_map[another_itr->id] = true;
                }
              }
            }
          }
          //merge，找emergency的后继看有木有merge
          if (!has_confirm && lane_ptr->next_lane_ids.size() == 1) {
            BevLaneInfo *emer_next_itr = lane_map[lane_ptr->next_lane_ids[0]];
            if (emer_next_itr) {
              auto another_id = std::find_if(emer_next_itr->previous_lane_ids.begin(), emer_next_itr->previous_lane_ids.end(),
                                             [emergency_id](uint64_t id) { return id != emergency_id; });
              if (another_id != emer_next_itr->previous_lane_ids.end()) {
                BevLaneInfo *another_itr = lane_map[*another_id];
                if (another_itr && !confirm_map[another_itr->id]) {
                  bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                  has_confirm  = true;
                  if (is_left) {
                    lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                    confirm_map[lane_ptr->id]    = true;
                    // another_itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
                    confirm_map[another_itr->id] = true;
                  }
                }
              }
            }
          }
        } else if (ego_lane_ptr->left_lane_id == 0 && ego_lane_ptr->right_lane_id != 0) {
          if (DEBUG_EL)
            AINFO << lane_ptr->id << ",ego->Emergency:left ";

          //左应急
          bool     has_confirm  = false;
          uint16_t emergency_id = lane_ptr->id;
          //split
          if (ego_lane_ptr->next_lane_ids.size() == 2) {
            auto another_id = std::find_if(ego_lane_ptr->next_lane_ids.begin(), ego_lane_ptr->next_lane_ids.end(),
                                           [emergency_id](uint64_t id) { return id != emergency_id; });
            if (another_id != ego_lane_ptr->next_lane_ids.end()) {
              BevLaneInfo *another_itr = lane_map[*another_id];
              if (another_itr && !confirm_map[another_itr->id]) {
                bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                has_confirm  = true;
                if (!is_left) {
                  lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                  confirm_map[lane_ptr->id]    = true;
                  // another_itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
                  confirm_map[another_itr->id] = true;
                }
              }
            }
          }
          //merge，找emergency的后继看有木有merge
          if (!has_confirm && lane_ptr->next_lane_ids.size() == 1) {
            BevLaneInfo *emer_next_itr = lane_map[lane_ptr->next_lane_ids[0]];
            if (emer_next_itr) {
              auto another_id = std::find_if(emer_next_itr->previous_lane_ids.begin(), emer_next_itr->previous_lane_ids.end(),
                                             [emergency_id](uint64_t id) { return id != emergency_id; });
              if (another_id != emer_next_itr->previous_lane_ids.end()) {
                BevLaneInfo *another_itr = lane_map[*another_id];
                if (another_itr && !confirm_map[another_itr->id]) {
                  bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                  has_confirm  = true;
                  if (!is_left) {
                    lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                    confirm_map[lane_ptr->id]    = true;
                    // another_itr->lane_type       = BevLaneType::LANE_TYPE_EMERGENCY;
                    confirm_map[another_itr->id] = true;
                  }
                }
              }
            }
          }
        } else {
          if (DEBUG_EL)
            AINFO << lane_ptr->id<<",ego->Emergency, ego left lane:" << ego_lane_ptr->left_lane_id << ",ego right lane:" << ego_lane_ptr->right_lane_id;
          //地图找类型
          bool     has_confirm  = false;
          uint16_t emergency_id = lane_ptr->id;
          //split
          if (ego_lane_ptr->next_lane_ids.size() == 2) {
            try_set_emergency_from_ids(ego_lane_ptr->next_lane_ids);
            has_confirm = confirm_map[ego_lane_ptr->next_lane_ids[0]];
          }
          //merge，找emergency的后继看有木有merge
          if (!has_confirm && lane_ptr->next_lane_ids.size() == 1) {
            BevLaneInfo *emer_next_itr = lane_map[lane_ptr->next_lane_ids[0]];
            if (emer_next_itr) {
              try_set_emergency_from_ids(emer_next_itr->previous_lane_ids);
              has_confirm = confirm_map[emer_next_itr->previous_lane_ids[0]];
            }
          }
          //ego的两侧是否有Emergency
          if (!has_confirm && ego_lane_ptr->next_lane_ids.size() == 2) {
            //右侧有应急纠split左
            BevLaneInfo *right_itr = lane_map[ego_lane_ptr->right_lane_id];
            if (right_itr) {
              auto right_lane_type = right_itr->lane_type;
              if (right_lane_type == BevLaneType::LANE_TYPE_EMERGENCY || right_lane_type == BevLaneType::LANE_TYPE_EMERGENCY_STOP ||
                  right_lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
                auto another_id = std::find_if(ego_lane_ptr->next_lane_ids.begin(), ego_lane_ptr->next_lane_ids.end(),
                                               [emergency_id](uint64_t id) { return id != emergency_id; });
                if (another_id != ego_lane_ptr->next_lane_ids.end()) {
                  BevLaneInfo *another_itr = lane_map[*another_id];
                  if (another_itr) {
                    bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                    has_confirm  = true;
                    if (is_left) {
                      lane_ptr->lane_type       = BevLaneType::LANE_TYPE_UNKNOWN;
                      confirm_map[lane_ptr->id] = true;
                    }
                  }
                }
              }
            }
            //左侧有应急纠split右
            BevLaneInfo *left_itr = lane_map[ego_lane_ptr->left_lane_id];
            if (left_itr && !confirm_map[lane_ptr->id]) {
              auto left_lane_type = left_itr->lane_type;
              if (left_lane_type == BevLaneType::LANE_TYPE_EMERGENCY || left_lane_type == BevLaneType::LANE_TYPE_EMERGENCY_STOP ||
                  left_lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
                auto another_id = std::find_if(ego_lane_ptr->next_lane_ids.begin(), ego_lane_ptr->next_lane_ids.end(),
                                               [emergency_id](uint64_t id) { return id != emergency_id; });
                if (another_id != ego_lane_ptr->next_lane_ids.end()) {
                  BevLaneInfo *another_itr = lane_map[*another_id];
                  if (another_itr) {
                    bool is_left = LaneGeometry::JudgeIsLeft(*lane_ptr->geos, *another_itr->geos);
                    has_confirm  = true;
                    if (!is_left) {
                      lane_ptr->lane_type          = BevLaneType::LANE_TYPE_UNKNOWN;
                      confirm_map[lane_ptr->id]    = true;
                    }
                  }
                }
              }
            }
          }
        }
      } else {
        if (DEBUG_EL)
          AINFO << lane_ptr->id << ",ego lane not connected to Emergency lane";
        continue;
      }
    }
  }

  if (DEBUG_EL)
    AINFO << "after:";
  debug_infos += "after: ";
  for (const auto &lane : data_->lane_infos) {
    if (lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY || lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY_STOP ||
        lane.lane_type == BevLaneType::LANE_TYPE_HARBOR_STOP) {
      debug_infos += std::to_string(lane.id) + ";";
      if (DEBUG_EL)
        AINFO << "lane id: " << lane.id;
    }
  }
}

void BevMapProcessor::TopoProcessor(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                                    const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                                    const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                                    const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                    const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                                    const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                                    const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                                    const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                                    const bool& low_precision_flag,
                                    std::string& debug_infos) {
  RemoveDuplicateLaneIds();
  RemoveNonExistentSuccessor();
  RemoveNonExistentPredecessor();
  DeleteLaneTopology(LaneSplitGetter, LaneMergeGetter, LaneTypeGetter, ld_map, bev_ld_match, split_match_infos, merge_match_infos, connect_match_infos, low_precision_flag, debug_infos);
  AddLaneTopology(LaneSplitGetter, LaneMergeGetter, LaneTypeGetter, ld_map, bev_ld_match, split_match_infos, merge_match_infos, connect_match_infos, low_precision_flag, debug_infos);
  ConstructLanesIndex();
}

void BevMapProcessor::RemoveDuplicateLaneIds() {
  if (!data_) {
    return;
  }
  for (size_t i = 0; i < data_->lane_infos.size(); i++) {
    if (!data_->lane_infos[i].next_lane_ids.empty()) {
      std::sort(data_->lane_infos[i].next_lane_ids.begin(), data_->lane_infos[i].next_lane_ids.end());
      auto last_next = std::unique(data_->lane_infos[i].next_lane_ids.begin(), data_->lane_infos[i].next_lane_ids.end());
      data_->lane_infos[i].next_lane_ids.erase(last_next, data_->lane_infos[i].next_lane_ids.end());
    }
    if (!data_->lane_infos[i].previous_lane_ids.empty()) {
      std::sort(data_->lane_infos[i].previous_lane_ids.begin(), data_->lane_infos[i].previous_lane_ids.end());
      auto last_prev = std::unique(data_->lane_infos[i].previous_lane_ids.begin(), data_->lane_infos[i].previous_lane_ids.end());
      data_->lane_infos[i].previous_lane_ids.erase(last_prev, data_->lane_infos[i].previous_lane_ids.end());
    }
  }
}

void BevMapProcessor::RemoveNonExistentSuccessor() {
  if (!data_) {
    return;
  }
  for (size_t i = 0; i < data_->lane_infos.size(); i++) {
    if (data_->lane_infos[i].next_lane_ids.empty()) {
      continue;
    }
    for (std::vector<uint64_t>::iterator it = data_->lane_infos[i].next_lane_ids.begin();
         it != data_->lane_infos[i].next_lane_ids.end();) {
      auto next_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [find_id = *it](BevLaneInfo &lane) { return lane.id == find_id; });

      if (next_lane == data_->lane_infos.end()) {
        it = data_->lane_infos[i].next_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void BevMapProcessor::RemoveNonExistentPredecessor() {
  if (!data_) {
    return;
  }
  for (size_t i = 0; i < data_->lane_infos.size(); i++) {
    if (data_->lane_infos[i].previous_lane_ids.empty()) {
      continue;
    }
    for (std::vector<uint64_t>::iterator it = data_->lane_infos[i].previous_lane_ids.begin();
         it != data_->lane_infos[i].previous_lane_ids.end();) {
      auto next_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [find_id = *it](BevLaneInfo &lane) { return lane.id == find_id; });

      if (next_lane == data_->lane_infos.end()) {
        it = data_->lane_infos[i].previous_lane_ids.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void BevMapProcessor::AddLaneTopology(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                                      const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                                      const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                                      const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                      const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                                      const bool& low_precision_flag,
                                      std::string& debug_infos) {
  constexpr double JumpToNextThreshold = 15.0;  // ego_lane 跳到其next添加拓扑的阈值
  
  if (data_ == nullptr) {
    return;
  }

# ifdef AddTopoDebug
  AINFO << "*******TopoBeforeAdd******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  for (const auto lane : data_->lane_infos) {
    AINFO << "lane id: " << lane.id
          << ", lane type: " << static_cast<int>(lane.lane_type)
          << "; split_topology: " << static_cast<int>(lane.split_topo_extend)
          << "; merge_topology: " << static_cast<int>(lane.merge_topo_extend);
    if (lane.position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      AINFO << "ego lane: " << lane.id;
    }
    // AINFO << "lane start: (" << lane.line_points.front().x << ", " << lane.line_points.front().y << ")";
    // AINFO << "lane end: (" << lane.line_points.back().x << ", " << lane.line_points.back().y << ")";
    for (const auto prev_lane_id : lane.previous_lane_ids) {
      AINFO << "previous lane id: " << prev_lane_id;
    }
    for (const auto next_lane_id : lane.next_lane_ids) {
      AINFO << "next lane id: " << next_lane_id;
    }
  }
  AINFO << "************************";
# endif

  bool is_on_highway = false;
  RoutingMapPtr routing_map_ptr_{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_ptr_);
  if (routing_map_ptr_) {
    is_on_highway = routing_map_ptr_->is_on_highway;
  }

  T_local_ego_ = FindTransform(data_->header.timestamp);

  ego_lane_inds_.clear();
  filtered_lane_ids_.clear();
  // 找到当前的ego_laned，仅添加ego_lane中遗失的拓扑。要求ego_lane不为空，且有足够的点数。
  int ego_lane_ind = data_->lane_infos.size();
  for (int iter = 0; iter < data_->lane_infos.size(); ++iter) {
    if (data_->lane_infos[iter].position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      ego_lane_ind = iter;
      break;
    }
  }

  // std::set<uint64_t> emergency_lane_set;
  // std::set<uint64_t> harbor_lane_set;
  if (ego_lane_ind != data_->lane_infos.size()) {
    // 初始化ego_lane_ind容器，用于存储打断后的车道点。
    ego_lane_inds_.push_back(ego_lane_ind);

    // 过滤emergency lane等特殊车道的影响
    // navigation::EmergencyLaneInfo emergency_lane_info = GetEmergencyLaneInfo();
    // 根据位置找到最左边和最后边的车道
    // std::vector<int> left_most_idx, right_most_idx;
    // GetEmergencyLaneIds(emergency_lane_info, emergency_lane_set, left_most_idx, right_most_idx);

    // 根据地图匹配结果添加自车车道的遗漏拓扑，需要确认ld_map不为空
    if (ld_map.get() != nullptr) {
      // 获取当前所有lane_id，判断其是否与历史一致，如果一致现根据历史信息，填补不存在的拓扑点
      // ID与历史相比不变逻辑，提取当前id，与历史信息对比，对比结果相同且>3,
      std::set<uint64_t> current_lane_ids;
  # ifdef AddTopoDebug
      AINFO << "*******LaneIDRecords******";
      AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
      AINFO << "sequence number: " << data_->header.cycle_counter;
      std::stringstream lane_id_str;
      for (const auto& lane_id : lane_id_memory_result_.first){
        lane_id_str << lane_id << ", ";
      }
      AINFO << "lane ids: " << lane_id_str.str();
      AINFO << "records number: " << lane_id_memory_result_.second;

      lane_id_str.str("");
      lane_id_str.clear();
      for (const auto& lane_id : lane_id_memory_before_split_.first){
        lane_id_str << lane_id << ", ";
      }
      AINFO << "before split break lane ids: " << lane_id_str.str();
      AINFO << "records number: " << lane_id_memory_before_split_.second;

      lane_id_str.str("");
      lane_id_str.clear();
      for (const auto& lane_id : lane_id_memory_before_merge_.first){
        lane_id_str << lane_id << ", ";
      }
      AINFO << "before merge break lane ids: " << lane_id_str.str();
      AINFO << "records number: " << lane_id_memory_before_merge_.second;

      AINFO << "Split memory size: " << split_memory_result_.size();
      for (const auto& pair : split_memory_result_) {
        const BevMapProcessor::TopoRecord& topo_record = pair.second;
        AINFO << "mainlane ids: " << topo_record.main_lane_id;
        std::stringstream sub_id_str;
        for (const auto& lane_id : topo_record.sub_lane_ids){
          sub_id_str << lane_id << ", ";
        }
        AINFO << "sublane ids: " << sub_id_str.str();
        AINFO << "records number: " << topo_record.record_number;
      }

      AINFO << "Merge memory size: " << merge_memory_result_.size();
      for (const auto& pair : merge_memory_result_) {
        const BevMapProcessor::TopoRecord& topo_record = pair.second;
        AINFO << "mainlane ids: " << topo_record.main_lane_id;
        std::stringstream sub_id_str;
        for (const auto& lane_id : topo_record.sub_lane_ids){
          sub_id_str << lane_id << ", ";
        }
        AINFO << "sublane ids: " << sub_id_str.str();
        AINFO << "records number: " << topo_record.record_number;
      }

      AINFO << "Split break memory size: " << split_memory_break_result_.size();
      for (const auto& pair : split_memory_break_result_) {
        const BevMapProcessor::BreakTopoRecord& topo_record = pair.second;
        AINFO << "break_lane_id: " << topo_record.break_lane_id
              << "; break_lane_type: " << topo_record.break_lane_type
              << "; other_lane_id: " << topo_record.other_lane_id
              << "; other_lane_type: " << topo_record.other_lane_type
              << "; add_lane_type: " << topo_record.add_lane_type;
        AINFO << "; records number: " << topo_record.record_number;
      }

      AINFO << "Merge break memory size: " << merge_memory_break_result_.size();
      for (const auto& pair : merge_memory_break_result_) {
        const BevMapProcessor::BreakTopoRecord& topo_record = pair.second;
        AINFO << "break_lane_id: " << topo_record.break_lane_id
              << "; break_lane_type: " << topo_record.break_lane_type
              << "; other_lane_id: " << topo_record.other_lane_id
              << "; other_lane_type: " << topo_record.other_lane_type
              << "; add_lane_type: " << topo_record.add_lane_type;
        AINFO << "records number: " << topo_record.record_number;
      }
      AINFO << "*************************";
  # endif

      json topo_debug_info;

      auto ego_lane = &data_->lane_infos[ego_lane_inds_.at(0)];
      if (ego_lane->id != prev_ego_lane_id) {
        prev_ego_lane_trans_id = prev_ego_lane_id;
      }

      for (const auto &lane : data_->lane_infos){
        current_lane_ids.insert(lane.id);
      }

      std::optional<BreakLaneResult> modify_split_lane_ids;
      auto main_lane = ego_lane;
      bool change_to_next_flag = false;
      // 使用ego_lane的next_lane作为main_lane的规则
      // ***********************************
      if (main_lane->next_lane_ids.size() == 1 &&
          main_lane->line_points.size() >= 2 &&
          main_lane->line_points.back().x < JumpToNextThreshold) {
        auto next_lane_id = main_lane->next_lane_ids.front();
        auto main_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [next_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return next_lane_id == lane.id; });
        if (main_lane_iter != data_->lane_infos.end()) {
          main_lane = &(*main_lane_iter);
          change_to_next_flag = true;
        }
      }

      if (current_lane_ids == lane_id_memory_result_.first && lane_id_memory_result_.second >= 3) {
        AddSplitByHistory(main_lane->id, topo_debug_info);
      }

      if (current_lane_ids == lane_id_memory_before_split_.first && lane_id_memory_before_split_.second >= 3) {
        modify_split_lane_ids = AddSplitBreakByHistory(main_lane->id, topo_debug_info);
      }

      auto split_info_result = GetSplitInfoForTopoAdd(LaneSplitGetter, main_lane, ld_map);
      // 使用记忆的突变lane作为main_lane的规则
      // ***********************************
      // 1. lane_ids不发生变化
      // 2. split_info_result没有匹配结果
      if (!change_to_next_flag &&
          current_lane_ids == lane_id_memory_result_.first &&
          !split_info_result.has_value()) {
        // 3. prev_ego_lane_trans_id和当前ego_lane_id记忆中存在前后继关系
        auto it = split_memory_result_.find(prev_ego_lane_trans_id);
        if (it != split_memory_result_.end()) {
          const BevMapProcessor::TopoRecord& topo_record = it->second;
          if (topo_record.sub_lane_ids.count(ego_lane->id) > 0) {
            // AINFO << "prev_ego_lane_trans_id: " << prev_ego_lane_trans_id <<", ego id: " << ego_lane->id;
            for (auto& lane : data_->lane_infos) {
              if (lane.id == prev_ego_lane_trans_id) {
                  main_lane = &lane;
                  split_info_result = GetSplitInfoForTopoAdd(LaneSplitGetter, main_lane, ld_map);
              }
            }
          }
        }
      }
      // ***********************************
      // AINFO << "ego_lane id: " << ego_lane->id;
      if (!modify_split_lane_ids.has_value() && !low_precision_flag){
        modify_split_lane_ids = AddSplitTopoByMapInfo(split_info_result, main_lane, ld_map, bev_ld_match, split_match_infos, connect_match_infos, topo_debug_info);
      }
      bool update_split_lane_id_flag = UpadateLaneIDMemory(current_lane_ids, lane_id_memory_before_split_);
      UpadateBreakMemory(update_split_lane_id_flag, split_memory_break_result_, modify_split_lane_ids);
      auto main_lane_id = main_lane->id;

      std::string debug_str = topo_debug_info.dump();
      debug_infos = debug_str + debug_infos;

      current_lane_ids.clear();
      for (const auto &lane : data_->lane_infos){
        current_lane_ids.insert(lane.id);
      }

      std::optional<BreakLaneResult> modify_merge_lane_ids;
      auto main_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });

      if (main_lane_iter != data_->lane_infos.end() && main_lane_iter->next_lane_ids.empty()) {
        auto main_lane = &(*main_lane_iter);
        topo_debug_info.clear();
        if (current_lane_ids == lane_id_memory_result_.first && lane_id_memory_result_.second >= 3) {
          AddMergeByHistory(main_lane->id, topo_debug_info);
        }

        if (current_lane_ids == lane_id_memory_before_merge_.first && lane_id_memory_before_merge_.second >= 3) {
          modify_merge_lane_ids = AddMergeBreakByHistory(main_lane->id, topo_debug_info);
        }

        auto merge_info_result = GetMergeInfoForTopoAdd(LaneMergeGetter, main_lane, ld_map);

        if (!modify_merge_lane_ids.has_value() && !low_precision_flag){
          modify_merge_lane_ids = AddMergeTopoByMapInfo(merge_info_result, main_lane, ld_map, bev_ld_match, merge_match_infos, connect_match_infos, topo_debug_info);
        }
        debug_str = "";
        std::string debug_str = topo_debug_info.dump();
        debug_infos = debug_str + debug_infos;
      }
      else if (main_lane_iter != data_->lane_infos.end()) {
        auto main_lane = &(*main_lane_iter);
        for (const auto next_id : main_lane->next_lane_ids) {
          topo_debug_info.clear();
          if (modify_split_lane_ids.has_value() && next_id == modify_split_lane_ids.value().add_lane_id) {
            auto next_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [next_id](const cem::message::sensor::BevLaneInfo &lane) { return next_id == lane.id; });
            if (next_lane_iter == data_->lane_infos.end()){
              continue;
            }
            auto next_lane = &(*next_lane_iter);

            if (current_lane_ids == lane_id_memory_result_.first && lane_id_memory_result_.second >= 3) {
              AddMergeByHistory(next_lane->id, topo_debug_info);
            }

            if (current_lane_ids == lane_id_memory_before_merge_.first && lane_id_memory_before_merge_.second >= 3) {
              modify_merge_lane_ids = AddMergeBreakByHistory(next_lane->id, topo_debug_info);
            }

            auto merge_info_result = GetMergeInfoForTopoAdd(LaneMergeGetter, main_lane, ld_map);
            if (!merge_info_result.has_value() || !split_info_result.has_value()){
              continue;
            }
            if (split_info_result.value()->distance_to_ego < merge_info_result.value()->distance_to_ego) {
              std::unique_ptr<BevMergeInfo> merge_info_result_copy = std::make_unique<BevMergeInfo>(*merge_info_result.value());
              // 根据打断信息更新匹配结果
              auto it = merge_info_result_copy->bev_lanes_from.find(main_lane->id);
              if (it == merge_info_result_copy->bev_lanes_from.end()) {
                continue;
              }

              if (merge_info_result_copy->bev_lane_to == main_lane->id) {
                merge_info_result_copy->bev_lane_to == next_lane->id;
              }

              // 复制旧值并修改 bev_lane_from 为 next_id
              BevMergeSource new_source = it->second;  // 复制旧值
              new_source.bev_lane_from = next_lane->id;       // 同步修改bev_lane_from
              // 删除旧键值对
              merge_info_result_copy->bev_lanes_from.erase(it);
              // 插入新键值对（新键next_id + 修改后的值）
              merge_info_result_copy->bev_lanes_from.emplace(next_lane->id, new_source);

              std::optional<const BevMergeInfo*> merge_info_result_copy_opt;
              if (merge_info_result_copy) {  // 确保智能指针指向有效对象
                merge_info_result_copy_opt = merge_info_result_copy.get();  // 包装新对象的指针
              } else {
                merge_info_result_copy_opt = std::nullopt;  // 若拷贝失败，包装为空
              }

              if (!modify_merge_lane_ids.has_value() && !low_precision_flag){
                modify_merge_lane_ids = AddMergeTopoByMapInfo(merge_info_result_copy_opt, next_lane, ld_map, bev_ld_match, merge_match_infos, connect_match_infos, topo_debug_info);
              }
            }
          }
          else {
            auto next_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [next_id](const cem::message::sensor::BevLaneInfo &lane) { return next_id == lane.id; });
            if (next_lane_iter == data_->lane_infos.end()){
              continue;
            }
            auto next_lane = &(*next_lane_iter);
            if (current_lane_ids == lane_id_memory_result_.first && lane_id_memory_result_.second >= 3) {
              AddMergeByHistory(next_lane->id, topo_debug_info);
            }

            if (current_lane_ids == lane_id_memory_before_merge_.first && lane_id_memory_before_merge_.second >= 3) {
              modify_merge_lane_ids = AddMergeBreakByHistory(next_lane->id, topo_debug_info);
            }

            auto merge_info_result = GetMergeInfoForTopoAdd(LaneMergeGetter, next_lane, ld_map);
            if (!modify_merge_lane_ids.has_value() && !low_precision_flag){
              modify_merge_lane_ids = AddMergeTopoByMapInfo(merge_info_result, next_lane, ld_map, bev_ld_match, merge_match_infos, connect_match_infos, topo_debug_info);
            }
          }
          debug_str = "";
          debug_str = topo_debug_info.dump();
          debug_infos = debug_str + debug_infos;
        }
      }

      bool update_merge_lane_id_flag = UpadateLaneIDMemory(current_lane_ids, lane_id_memory_before_merge_);
      UpadateBreakMemory(update_merge_lane_id_flag, merge_memory_break_result_, modify_merge_lane_ids);
      
      // 更新laneID历史记录
      current_lane_ids.clear();
      for (const auto &lane : data_->lane_infos){
        current_lane_ids.insert(lane.id);
      }
      bool update_lane_id_flag = UpadateLaneIDMemory(current_lane_ids, lane_id_memory_result_);

      // 更新split拓扑记录
      UpadateSplitMemory(update_lane_id_flag);

      // 更新Merge拓扑记录
      UpadateMergeMemory(update_lane_id_flag);
      prev_ego_lane_id = ego_lane->id;
    }
  }

  if (T_local_ego_ == std::nullopt) {
    return;
  }

  // 更新lane
  bool debug = false;
  if (debug) {
    AINFO << "sequence_num: " << data_->header.cycle_counter;
    lane_tracker_ptr_->PrintInputMarkers(data_->lane_infos);
  }

  lane_tracker_ptr_->Update(data_->lane_infos, data_->lanemarkers, T_local_ego_.value(), true);
  data_->lane_infos = lane_tracker_ptr_->getResult(data_->lanemarkers);

  // diversion_ptr_->Update(data_->diversion_zone, T_local_ego_.value());
  // data_->diversion_zone = diversion_ptr_->GetConfirmedDiversions();
  if (debug) {
    lane_tracker_ptr_->PrintTrackedLanes();
    lane_tracker_ptr_->PrintInputMarkers(data_->lane_infos);
  }

  FillNearLaneId();

  lane_topology_processor_.SetTopoExtendInfo(data_->lane_infos);

  if (routing_map_ptr_) {
    if (!is_on_highway) {
      lane_topology_processor_.GetCrossParam(cross_road_status_,cross_road_distance_,is_turn_right_);
      lane_topology_processor_.SetAdc2Junction(adc_to_junction_);
      lane_topology_processor_.SetBrokenTopoInfo(data_);
    }
  }

  if (is_on_highway) {
    navigation::EmergencyLaneInfo emergency_lane_info = GetEmergencyLaneInfo();
    bool ego_lane_emergency_flag = CheckEgoLaneEmergencyConstraint(emergency_lane_info);

  # ifdef AddTopoDebug
    AINFO << "*******EgoLaneEmergency******";
    AINFO << "sequence number: " << data_->header.cycle_counter;
    for (const auto lane : data_->lane_infos) {
      if (lane.position == 0) {  // BEVLanePositon::LANE_LOC_EGO
        AINFO << "ego lane: " << lane.id
              << "; ego lane emergency: " << ego_lane_emergency_flag;
      }
    }
    AINFO << "************************";
  # endif

    //确认是否紧急车道
    ConfirmEmergencyLane(LaneTypeGetter, debug_infos, ego_lane_emergency_flag);
  }

# ifdef AddTopoDebug
  AINFO << "*******TopoAfterAdd******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  if (is_on_highway) {
    AINFO << "IS ON Highway !!!!";
  }
  for (const auto lane : data_->lane_infos){
    AINFO << "lane id: " << lane.id
          << ", lane type: " << static_cast<int>(lane.lane_type)
          << "; split_topology: " << static_cast<int>(lane.split_topo_extend)
          << "; merge_topology: " << static_cast<int>(lane.merge_topo_extend);
    if (lane.position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      AINFO << "ego lane: " << lane.id;
    }
    // AINFO << "lane start: (" << lane.line_points.front().x << ", " << lane.line_points.front().y << ")";
    // AINFO << "lane end: (" << lane.line_points.back().x << ", " << lane.line_points.back().y << ")";
    for (const auto prev_lane_id : lane.previous_lane_ids){
      AINFO << "previous lane id: " << prev_lane_id;
    }
    for (const auto next_lane_id : lane.next_lane_ids){
      AINFO << "next lane id: " << next_lane_id;
    }
  }
  AINFO << "************************";
# endif
}

void BevMapProcessor::DeleteLaneTopology(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                                      const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                                      const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                                      const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                      const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                                      const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                                      const bool& low_precision_flag,
                                      std::string& debug_infos) {
  constexpr float DistLatThreshold = 0.2;  // 掉头删除车道横向约束
  constexpr double DistLongThreshold = 20.0;  // 掉头删除车道纵向约束
  std::string debug_str;

  if (data_ == nullptr || ld_map.get() == nullptr) {
    return;
  }

  bool is_on_highway = false;
  RoutingMapPtr routing_map_ptr_{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_ptr_);
  if (routing_map_ptr_) {
    is_on_highway = routing_map_ptr_->is_on_highway;
  }
  if (is_on_highway) {
    return;
  }

  json topo_debug_info;

  if (low_precision_flag) {
    topo_debug_info["low_precision_flag"] = true;
    topo_debug_info["delete uturn lane"] = false;
    debug_str = topo_debug_info.dump();
    debug_infos += debug_str;
    return;
  }

# ifdef AddTopoDebug
  AINFO << "*******TopoBeforeDelete******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  for (const auto lane : data_->lane_infos) {
    AINFO << "lane id: " << lane.id
          << ", lane type: " << static_cast<int>(lane.lane_type)
          << "; split_topology: " << static_cast<int>(lane.split_topo_extend)
          << "; merge_topology: " << static_cast<int>(lane.merge_topo_extend);
    if (lane.position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      AINFO << "ego lane: " << lane.id;
    }
    // AINFO << "lane start: (" << lane.line_points.front().x << ", " << lane.line_points.front().y << ")";
    // AINFO << "lane end: (" << lane.line_points.back().x << ", " << lane.line_points.back().y << ")";
    for (const auto prev_lane_id : lane.previous_lane_ids) {
      AINFO << "previous lane id: " << prev_lane_id;
    }
    for (const auto next_lane_id : lane.next_lane_ids) {
      AINFO << "next lane id: " << next_lane_id;
    }
  }
  AINFO << "************************";
# endif

  // 找到当前的ego_laned，仅添加ego_lane中遗失的拓扑。要求ego_lane不为空，且有足够的点数。
  int ego_lane_ind = data_->lane_infos.size();
  for (int iter = 0; iter < data_->lane_infos.size(); ++iter) {
    if (data_->lane_infos[iter].position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      ego_lane_ind = iter;
      break;
    }
  }

  if (ego_lane_ind == data_->lane_infos.size()) {
    topo_debug_info["ego lane exit"] = false;
    topo_debug_info["delete uturn lane"] = false;
    debug_str = topo_debug_info.dump();
    debug_infos += debug_str;
    return;
  }

  auto ego_lane = &data_->lane_infos[ego_lane_ind];
  auto it = bev_ld_match.find(ego_lane->id);
  if (it == bev_ld_match.end()) {
    topo_debug_info["bev_ld_match exit"] = false;
    topo_debug_info["delete uturn lane"] = false;
    debug_str = topo_debug_info.dump();
    debug_infos += debug_str;
    return;
  }
  auto ego_lane_ld_ids = it->second;

  // std::stringstream lane_id_str;
  // for (const auto& lane_id : ego_lane_ld_ids){
  //   lane_id_str << lane_id << ", ";
  // }
  // AINFO << "ego_ld_map id: " << lane_id_str.str();

  std::vector<cem::message::env_model::LaneInfo> u_turn_lanes;
  FindUTurnLDLane(ego_lane_ld_ids, ld_map, u_turn_lanes);

  // lane_id_str.str("");
  // lane_id_str.clear();
  // for (const auto& lane : u_turn_lanes){
  //   lane_id_str << lane.id << ", ";
  // }
  // AINFO << "uturn_ld_map id: " << lane_id_str.str();

  // lane_id_str.str("");
  // lane_id_str.clear();
  for (const auto& u_turn_lane : u_turn_lanes) {
    if (u_turn_lane.points.size() < 2) {
      continue;
    }

    auto u_turn_lane_end = u_turn_lane.points.back();
    auto u_turn_lane_second_end = u_turn_lane.points[u_turn_lane.points.size() - 2];
    Eigen::Vector2d u_turn_lane_dir;
    u_turn_lane_dir.x() = u_turn_lane_end.x - u_turn_lane_second_end.x;
    u_turn_lane_dir.y() = u_turn_lane_end.y - u_turn_lane_second_end.y;
    u_turn_lane_dir = u_turn_lane_dir / u_turn_lane_dir.norm();

    double center2boundary_dist = CalculateCenterBoundaryDistanceFromMap(u_turn_lane, ld_map);
    // AINFO << "uturn lane id: " << u_turn_lane.id << "; center2boundary_dist: " << center2boundary_dist;

    // 反向遍历：从最后一个元素到第一个
    for (int i = data_->lane_infos.size() - 1; i >= 0; --i) {
      topo_debug_info.clear();
      auto& lane =  data_->lane_infos[i]; 
      if (lane.line_points.size() < 2) {
        continue;
      }
      // 首点距离约束
      Eigen::Vector2d lane_start_vec;
      lane_start_vec.x() = lane.line_points.front().x - u_turn_lane_end.x;
      lane_start_vec.y() = lane.line_points.front().y - u_turn_lane_end.y;
      double dist_lat_norm = (lane_start_vec - lane_start_vec.dot(u_turn_lane_dir) * u_turn_lane_dir).norm();
      double dist_long_norm =  std::abs(lane_start_vec.dot(u_turn_lane_dir));
      if (dist_long_norm > DistLongThreshold) {
        continue;
      }

      if (dist_lat_norm > center2boundary_dist + DistLatThreshold) {
        continue;
      }

      // 向左弯曲约束
      bool left_bending_flag = CheckLeftBending(lane, topo_debug_info);
      if (!left_bending_flag) {
        continue;
      }

      // lane_id_str << lane.id << ", ";

      // 删除lane何其前拓扑关系
      //处理前继车道 → 删除前继车道的next_lane_ids中的当前lane_id 
      uint64_t current_lane_id = lane.id;
      for (uint64_t prev_id : lane.previous_lane_ids) {
        auto prev_it = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), 
                                   [prev_id] (const cem::message::sensor::BevLaneInfo& lane) {return prev_id == lane.id;});
        if (prev_it == data_->lane_infos.end()) continue; // 前继车道不存在，跳过

        // 从prev_lane的next_lane_ids中删除current_lane_id
        prev_it->next_lane_ids.erase(
            std::remove(prev_it->next_lane_ids.begin(), prev_it->next_lane_ids.end(), current_lane_id),
            prev_it->next_lane_ids.end()
        );
      }

      // 处理后继车道 → 删除后继车道的previous_lane_ids中的当前lane_id =====
      for (uint64_t next_id : lane.next_lane_ids) {
        auto next_it = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(), 
                                   [next_id] (const cem::message::sensor::BevLaneInfo& lane) {return next_id == lane.id;});
        if (next_it == data_->lane_infos.end()) continue; // 后继车道不存在，跳过

        // 从next_lane的previous_lane_ids中删除current_lane_id
        next_it->previous_lane_ids.erase(
            std::remove(next_it->previous_lane_ids.begin(), next_it->previous_lane_ids.end(), current_lane_id),
            next_it->previous_lane_ids.end()
        );
      }

      // 删除当前车道
      data_->lane_infos.erase(data_->lane_infos.begin() + i);
      topo_debug_info["delete uturn lane"] = true;
      debug_str = "";
      debug_str = topo_debug_info.dump();
      debug_infos += debug_str;
    }
  }
  // AINFO << "target_delete_lane id: " << lane_id_str.str();

# ifdef AddTopoDebug
  AINFO << "*******TopoAfterDelete******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  for (const auto lane : data_->lane_infos) {
    AINFO << "lane id: " << lane.id
          << ", lane type: " << static_cast<int>(lane.lane_type)
          << "; split_topology: " << static_cast<int>(lane.split_topo_extend)
          << "; merge_topology: " << static_cast<int>(lane.merge_topo_extend);
    if (lane.position == 0) {  // BEVLanePositon::LANE_LOC_EGO
      AINFO << "ego lane: " << lane.id;
    }
    // AINFO << "lane start: (" << lane.line_points.front().x << ", " << lane.line_points.front().y << ")";
    // AINFO << "lane end: (" << lane.line_points.back().x << ", " << lane.line_points.back().y << ")";
    for (const auto prev_lane_id : lane.previous_lane_ids) {
      AINFO << "previous lane id: " << prev_lane_id;
    }
    for (const auto next_lane_id : lane.next_lane_ids) {
      AINFO << "next lane id: " << next_lane_id;
    }
  }
  AINFO << "************************";
# endif
}

navigation::EmergencyLaneInfo BevMapProcessor::GetEmergencyLaneInfo() {
  // 应急车道扫描范围
  constexpr double MinSConstraint  = -100.0;  // 自车后 100m
  constexpr double MaxSConstraint = 300.0;    // 自车前 300m
  static const std::unordered_set<LaneType> kProhibitedTypes = {LaneType::LANE_EMERGENCY, LaneType::LANE_HARBOR_STOP,
                                                                LaneType::LANE_DIVERSION, LaneType::LANE_NON_MOTOR};

  navigation::EmergencyLaneInfo result;
  auto              ld_route_info = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route_info) {
    AWARN << "[LDNOA] Failed: ld_route_info is null!";
    return result;
  }

  // 获取自车位置信息
  uint64_t ego_section_id = ld_route_info->navi_start.section_id;
  double   ego_s_offset   = ld_route_info->navi_start.s_offset;
  SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Ego section ID: " << ego_section_id << ", Ego s_offset: " << ego_s_offset;

  // 计算自车全局s坐标
  double ego_global_s      = 0.0;
  bool   ego_section_found = false;
  for (const auto &section : ld_route_info->sections) {
    if (section.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      ego_section_found = true;
      break;
    }
    ego_global_s += section.length;
  }
  if (!ego_section_found) {
    AWARN << "[LDNOA] Failed: ego_section not found!";
    return result;
  }
  SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Ego global s: " << ego_global_s;

  // 定义检测范围
  const double min_s = MinSConstraint;
  const double max_s = MaxSConstraint;

  // 存储应急车道区间
  std::vector<std::pair<double, double>> left_ranges {};
  std::vector<std::pair<double, double>> right_ranges {};
  int                                    left_lane_count  = 0;
  int                                    right_lane_count = 0;

  // 遍历所有路段
  double current_s = 0.0;
  for (const auto &section : ld_route_info->sections) {
    const double section_start_s = current_s;
    const double section_end_s   = current_s + section.length;
    current_s                    = section_end_s;  // 更新当前s位置

    // 跳过不在检测范围内的路段
    if (section_end_s < ego_global_s + min_s || section_start_s > ego_global_s + max_s)
      continue;

    //SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Section ID: " << section.id
    //                    << " start s: " << section_start_s << " end s: " << section_end_s << "  section.laneids.size(): "<<section.lane_ids.size();

    std::vector<const cem::message::env_model::LaneInfo *> section_lanes;
    for (uint64_t lane_id : section.lane_ids) {

      auto lane_info = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane_info) {
        section_lanes.push_back(lane_info);
        //SD_COARSE_MATCH_LOG << "  [GetEmergencyLaneInfo] lane id:  " <<lane_id;
      }
    }

    // 按车道序列号排序 (lane_seq从左到右递增)
    // std::sort(section_lanes.begin(), section_lanes.end(),
    //           [](const cem::message::env_model::LaneInfo* a,
    //              const cem::message::env_model::LaneInfo* b) {
    //             return a->lane_seq < b->lane_seq;
    //           });

    if (section_lanes.empty())
      continue;

    // 检查最左侧车道
    const cem::message::env_model::LaneInfo *leftmost_lane = section_lanes.front();
    if (kProhibitedTypes.find(leftmost_lane->type) != kProhibitedTypes.end()){
      const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
      const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

      if (lane_start_s < lane_end_s) {
        // 合并连续区间
        if (!left_ranges.empty() && lane_start_s <= left_ranges.back().second) {
          left_ranges.back().second = std::max(left_ranges.back().second, lane_end_s);
        } else {
          left_ranges.emplace_back(lane_start_s, lane_end_s);
        }
        //left_lane_count = section_lanes.size();
        if (left_lane_count < section_lanes.size()) {
          left_lane_count = section_lanes.size();
        }
        SD_COARSE_MATCH_LOG << "  Left emergency lane detected, range: [" << lane_start_s << ", " << lane_end_s << "]";
      }
    }

    // 检查最右侧车道
    const cem::message::env_model::LaneInfo *rightmost_lane = section_lanes.back();
    if (kProhibitedTypes.find(rightmost_lane->type) != kProhibitedTypes.end()){
      const double lane_start_s = std::max(section_start_s - ego_global_s, min_s);
      const double lane_end_s   = std::min(section_end_s - ego_global_s, max_s);

      if (lane_start_s < lane_end_s) {
        // 合并连续区间
        if (!right_ranges.empty() && lane_start_s <= right_ranges.back().second) {
          right_ranges.back().second = std::max(right_ranges.back().second, lane_end_s);
        } else {
          right_ranges.emplace_back(lane_start_s, lane_end_s);
        }
        //right_lane_count = section_lanes.size();
        if (right_lane_count < section_lanes.size()) {
          right_lane_count = section_lanes.size();
        }
        SD_COARSE_MATCH_LOG << "  Right emergency lane detected, range: [" << lane_start_s << ", " << lane_end_s << "]";
      }
    }
  }

  // 填充结果
  result.left.ranges    = left_ranges;
  result.right.ranges   = right_ranges;
  result.left.exists    = !left_ranges.empty();
  result.right.exists   = !right_ranges.empty();
  result.left.lane_num  = left_lane_count;
  result.right.lane_num = right_lane_count;

  // 调试日志
  if (result.right.exists) {
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] Right emergency lane exists in merged ranges:";
    for (const auto &range : result.right.ranges) {
      SD_COARSE_MATCH_LOG << "  [" << range.first << ", " << range.second << "]";
    }
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo]  lane num: " << result.right.lane_num;
  } else {
    SD_COARSE_MATCH_LOG << "[GetEmergencyLaneInfo] No right emergency lane found.";
  }

  return result;
}

void BevMapProcessor::GetEmergencyLaneIds(
    const navigation::EmergencyLaneInfo& sd_emergency_lane_info,
    std::set<uint64_t>& current_bev_emergency_laneid,
    std::vector<int>& target_emergency_left_idx,
    std::vector<int>& target_emergency_right_idx) {
  constexpr double MinDistThreshold = 0.5;
  constexpr double RoadEdgeDistThreshold = 1.0;
  // 内部函数：通过纵向信息判断Emergency_lane可行性
  auto SDHasEMLane =
      [](const BevLaneInfo* bev_lane_info,
         const std::vector<std::pair<double, double>>& sd_ranges) -> bool {
    if (!bev_lane_info && !bev_lane_info->geos->empty()) {
      return false;
    }
    float start_x = bev_lane_info->geos->front().x();
    float end_x = bev_lane_info->geos->back().x();
    float intersection_length = 0;
    for (auto& range_tmp : sd_ranges) {
      if (range_tmp.first < end_x && range_tmp.second > start_x) {
        if (range_tmp.first < start_x) {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - start_x);
          } else {
            intersection_length += fabs(end_x - start_x);
          }
        } else {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - range_tmp.first);
          } else {
            intersection_length += fabs(end_x - range_tmp.first);
          }
        }
      }
    }
    if (fabs(end_x - start_x) > 5) {
      float rate = intersection_length / fabs(end_x - start_x);
      if (rate > 0.5) {
        return true;
      }
    }
    return false;
  };

  // ego_lane 相关信息输入
  if (data_->lane_infos[ego_lane_inds_.front()].line_points.size() < 2) {
    return;
  }

  auto ego_lane_start =
      data_->lane_infos[ego_lane_inds_.front()].line_points.front();
  auto ego_lane_end =
      data_->lane_infos[ego_lane_inds_.back()].line_points.back();
  Eigen::Vector2d dir_ego_lane, ego_lane_mid;
  dir_ego_lane.x() = ego_lane_end.x - ego_lane_start.x;
  dir_ego_lane.y() = ego_lane_end.y - ego_lane_start.y;
  dir_ego_lane = dir_ego_lane / dir_ego_lane.norm();
  ego_lane_mid.x() = (ego_lane_start.x + ego_lane_end.x) / 2;
  ego_lane_mid.y() = (ego_lane_start.y + ego_lane_end.y) / 2;

  // 根据位置找到最左边和最右边的车道
  double min_val_lat = std::numeric_limits<double>::max();
  double max_val_lat = std::numeric_limits<double>::min();
  /// 第一遍：找到车道线横向的最大值和最小值
  for (int i = 0; i < data_->lane_infos.size(); ++i) {
    auto it = std::find(ego_lane_inds_.begin(), ego_lane_inds_.end(), i);
    if (it != ego_lane_inds_.end()) {
      continue;
    }
    const auto lane = data_->lane_infos[i];
    if (lane.line_points.size() < 2) {
      continue;
    }

    Eigen::Vector2d lane_mid;
    lane_mid.x() = (lane.line_points.front().x + lane.line_points.back().x) / 2;
    lane_mid.y() = (lane.line_points.front().y + lane.line_points.back().y) / 2;

    auto dist_vec = lane_mid - ego_lane_mid;
    double dist_lat_sign = copysign(
        1.0, dir_ego_lane.x() * dist_vec.y() - dir_ego_lane.y() * dist_vec.x());
    double dist_lat_val =
        dist_lat_sign *
        (dist_vec - dist_vec.dot(dir_ego_lane) * dir_ego_lane).norm();
    if (min_val_lat > dist_lat_val) {
      min_val_lat = dist_lat_val;
    }
    if (max_val_lat < dist_lat_val) {
      max_val_lat = dist_lat_val;
    }
  }
  /// 第二遍：根据最大值和最小值找到目标车道线（最左和最右），同时输出给下游输出港湾式车道使用
  for (int i = 0; i < data_->lane_infos.size(); i++) {
    const auto lane = data_->lane_infos[i];
    Eigen::Vector2d lane_mid;
    lane_mid.x() = (lane.line_points.front().x + lane.line_points.back().x) / 2;
    lane_mid.y() = (lane.line_points.front().y + lane.line_points.back().y) / 2;

    auto dist_vec = lane_mid - ego_lane_mid;
    double dist_lat_sign = copysign(
        1.0, dir_ego_lane.x() * dist_vec.y() - dir_ego_lane.y() * dist_vec.x());
    double dist_lat_val =
        dist_lat_sign *
        (dist_vec - dist_vec.dot(dir_ego_lane) * dir_ego_lane).norm();
    if (max_val_lat - dist_lat_val < MinDistThreshold) {
      target_emergency_left_idx.push_back(i);
    }
    if (dist_lat_val - min_val_lat < MinDistThreshold) {
      target_emergency_right_idx.push_back(i);
    }
  }

  // AINFO << "*******FusionEmergencyLane******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto &right_idx :
  // target_emergency_right_idx){
  //   const auto lane = data_->lane_infos[right_idx];
  //   AINFO << "target right emergency lane ID: " << lane.id;
  //   // if(SDHasEMLane(&lane, sd_emergency_lane_info.right.ranges)){
  //   //   AINFO << "right emergency lane ID: " << lane.id;
  //   // }
  // }
  // for (const auto &left_idx : target_emergency_left_idx){
  //   const auto lane = data_->lane_infos[left_idx];
  //   AINFO << "target left emergency lane ID: " << lane.id;
  //   // if(SDHasEMLane(&lane, sd_emergency_lane_info.left.ranges)){
  //   //   AINFO << "left emergency lane ID: " << lane.id;
  //   // }
  // }
  // AINFO << "*******************************";

  /// 联合感知、路沿信息和SD信息判断最右车道是否是应急车道
  /// 先记录感知中的应急车道
  for (const auto& lane : data_->lane_infos) {
    /// 感知检出应急车道直接添加
    if (lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY ||
        lane.lane_type == BevLaneType::LANE_TYPE_BLOCKED ||
        lane.lane_type == BevLaneType::LANE_TYPE_EMERGENCY_STOP) {
      current_bev_emergency_laneid.insert(lane.id);
    }
  }

  /// 在记录最左SD图判断的应急车道
  for (const auto& right_idx : target_emergency_right_idx) {
    const auto lane = data_->lane_infos[right_idx];
    auto it =
        std::find(ego_lane_inds_.begin(), ego_lane_inds_.end(), right_idx);
    if (it ==
            ego_lane_inds_
                .end() &&  // right
                           // lane的index不在ego_lane_inds_其中，及其不为ego_lane
        lane.left_lane_marker_id != 0 &&
        lane.right_lane_marker_id != 0) {
      /// 跳过已经添加的lane
      if (current_bev_emergency_laneid.count(lane.id)) {
        continue;
      }

      /// 感知没有检出应急车道使用SD地图信息和路沿信息做兜底判断
      if (SDHasEMLane(&lane, sd_emergency_lane_info.right.ranges)) {
        bool valid_flag = true;
        auto left_lane_boundary =
            INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(
                lane.left_lane_marker_id);
        auto right_lane_boundary =
            INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(
                lane.right_lane_marker_id);
        if (left_lane_boundary) {
          int dash_line_count = 0;
          int all_index_count = 0;
          for (auto& type_tmp : left_lane_boundary->type_segs) {
            all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
            if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED ||
                type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
              dash_line_count +=
                  fabs(type_tmp.end_index - type_tmp.start_index);
            }
          }
          if (all_index_count != 0 &&
              (float)dash_line_count / (float)all_index_count > 0.2) {
            valid_flag = false;
          }
        }
        if (right_lane_boundary) {
          double dist2rightRoadEdge = std::numeric_limits<double>::max();
          for (auto& road_edge : data_->edges) {
            double gap_right_boundary =
                LaneGeometry::GetDistanceBetweenLinesThin(
                    *road_edge.geos, *right_lane_boundary->geos);
            if (gap_right_boundary != std::numeric_limits<double>::max() &&
                gap_right_boundary < dist2rightRoadEdge) {
              dist2rightRoadEdge = gap_right_boundary;
            }
          }
          if (dist2rightRoadEdge > RoadEdgeDistThreshold) {
            valid_flag = false;
          }
        }
        if (valid_flag) {
          current_bev_emergency_laneid.insert(lane.id);
        }
      }
    }
  }

  /// 联合路沿信息和SD信息判断最左车道是否是应急车道
  for (const auto& left_idx : target_emergency_left_idx) {
    const auto lane = data_->lane_infos[left_idx];
    auto it = std::find(ego_lane_inds_.begin(), ego_lane_inds_.end(), left_idx);
    if (it ==
            ego_lane_inds_
                .end() &&  // left
                           // lane的index不在ego_lane_inds_其中，及其不为ego_lane
        lane.left_lane_marker_id != 0 &&
        lane.right_lane_marker_id != 0) {
      /// 跳过已经添加的lane
      if (current_bev_emergency_laneid.count(lane.id)) {
        continue;
      }

      /// 感知没有检出应急车道使用SD地图信息和路沿信息做兜底判断
      if (SDHasEMLane(&lane, sd_emergency_lane_info.right.ranges)) {
        bool valid_flag = true;
        auto left_lane_boundary =
            INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(
                lane.left_lane_marker_id);
        auto right_lane_boundary =
            INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(
                lane.right_lane_marker_id);
        if (left_lane_boundary) {
          int dash_line_count = 0;
          int all_index_count = 0;
          for (auto& type_tmp : left_lane_boundary->type_segs) {
            all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
            if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED ||
                type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
              dash_line_count +=
                  fabs(type_tmp.end_index - type_tmp.start_index);
            }
          }
          if (all_index_count != 0 &&
              (float)dash_line_count / (float)all_index_count > 0.2) {
            valid_flag = false;
          }
        }
        if (right_lane_boundary) {
          double dist2rightRoadEdge = std::numeric_limits<double>::max();
          for (auto& road_edge : data_->edges) {
            double gap_right_boundary =
                LaneGeometry::GetDistanceBetweenLinesThin(
                    *road_edge.geos, *right_lane_boundary->geos);
            if (gap_right_boundary != std::numeric_limits<double>::max() &&
                gap_right_boundary < dist2rightRoadEdge) {
              dist2rightRoadEdge = gap_right_boundary;
            }
          }
          if (dist2rightRoadEdge > RoadEdgeDistThreshold) {
            valid_flag = false;
          }
        }
        if (valid_flag) {
          current_bev_emergency_laneid.insert(lane.id);
        }
      }
    }
  }

  // AINFO << "*******FusionEmergencyLane******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto &lane_id :
  // current_bev_emergency_laneid){
  //   AINFO << "Emergency Lane ID: " << lane_id;
  // }
  // AINFO << "*******************************";
}

void BevMapProcessor::GetHarborLaneIds(
    const navigation::HarborStopInfo& sd_harbor_lane_info,
    std::set<uint64_t>& current_bev_harbor_laneid) {
  // AINFO << "*******FusionHarborLane******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp;
  auto all_bev_edges = data_->edges;
  double min_overlap_threshold = 20.0f;  // 最小重叠长度阈值(米)
  double max_distance_threshold = 2.5f;  // 最大距离阈值(米)
  if (current_bev_harbor_laneid.empty()) {
    // 步骤1: 找出与公交港湾重叠最长且大于阈值的边界线
    BevLaneMarker best_edge;
    double max_overlap = -1.0;
    for (auto& edge : all_bev_edges) {
      if ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__RIGHT) ||
          ((edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN) &&
           (!edge.geos->empty()) && (edge.geos->front().y() < 0))) {
        double overlap_dis =
            calculateOverlapDistance(sd_harbor_lane_info, *edge.geos);
        // AINFO << "[Erase_Harborlane_Ids] bdry_2_harbor overlap_dis:" <<
        // overlap_dis;
        if (overlap_dis > max_overlap && overlap_dis >= min_overlap_threshold) {
          max_overlap = overlap_dis;
          best_edge = edge;
        }
      }
    }
    // 步骤2: 找出与该边界线宽度最小且小于阈值的车道线
    uint64_t best_lane_id = 0;
    float min_distance = std::numeric_limits<float>::max();
    for (int i = 0; i < data_->lane_infos.size(); i++) {
      auto it = std::find(ego_lane_inds_.begin(), ego_lane_inds_.end(), i);
      if (it != ego_lane_inds_.end()) {
        continue;
      }

      const auto lane = data_->lane_infos[i];

      if (current_bev_harbor_laneid.count(lane.id)) {
        continue;
      }

      if ((!lane.geos->empty()) && (!best_edge.geos->empty())) {
        double overlap_dis =
            calculateOverlapDistance(sd_harbor_lane_info, *lane.geos);
        /* 重叠区域内计算车道垂线宽度均值，去除头尾值 */
        double dis_to_rightbdry =
            CalculateCurvedLaneWidth2(*lane.geos, *best_edge.geos);
        // AINFO << "[Erase_Harborlane_Ids] dis line " << lane.id << " to "
        //       << best_edge.id << "  rightbdry is : " << dis_to_rightbdry;
        if (dis_to_rightbdry > 0.5 && dis_to_rightbdry < min_distance &&
            dis_to_rightbdry <= max_distance_threshold &&
            overlap_dis >= min_overlap_threshold) {
          min_distance = dis_to_rightbdry;
          best_lane_id = lane.id;
        }
      }
    }
    // AINFO << "[Erase_Harborlane_Ids]best_edge id: " << best_edge.id << "
    // max_overlap: " << max_overlap; AINFO <<
    // "[Erase_Harborlane_Ids]best_lane_id: " << best_lane_id << " min_distance:
    // " << min_distance;
    if (best_lane_id != 0) {
      // AINFO << "best_lane_id insert in the set: " << best_lane_id;
      current_bev_harbor_laneid.insert(best_lane_id);
    }
  }
  // AINFO << "*******************************";
}

void BevMapProcessor::GetFilteredShortLaneIds(
    std::vector<int>& left_most_idx, std::vector<int>& right_most_idx) {
  constexpr double MaxDistConfine =
      2.0;  // 目标添加拓扑点的最小距离（中心线，精）
  for (int i = 0; i < data_->lane_infos.size(); i++) {
    // 过滤的车道不是最左和最右的车道
    auto it = std::find(left_most_idx.begin(), left_most_idx.end(), i);
    if (it != left_most_idx.end()) {
      continue;
    }
    it = std::find(right_most_idx.begin(), right_most_idx.end(), i);
    if (it != right_most_idx.end()) {
      continue;
    }

    const auto lane_check = data_->lane_infos[i];
    const auto lane_check_start = lane_check.line_points.front();
    const auto lane_check_end = lane_check.line_points.back();

    // 查找会打断车道, 首尾都会连上拓扑的车道，将其ID放在set容器中
    bool topo_start_flag = false;
    bool topo_end_flag = false;
    for (int j = 0; j < data_->lane_infos.size(); j++) {
      if (i == j) {
        continue;
      }

      const auto lane = data_->lane_infos[j];

      if (lane.line_points.size() < 4) {
        continue;
      }

      for (int k = 1; k < lane.line_points.size() - 2; k++) {
        auto segment_pt1 = lane.line_points[k];
        auto segment_pt2 = lane.line_points[k + 1];
        double temp_dist = CalculatePoint2SegmentDist(lane_check_start,
                                                      segment_pt1, segment_pt2);
        if (temp_dist < MaxDistConfine) {
          topo_start_flag = true;
          break;
        }
      }

      for (int k = 1; k < lane.line_points.size() - 2; k++) {
        auto segment_pt1 = lane.line_points[k];
        auto segment_pt2 = lane.line_points[k + 1];
        double temp_dist = CalculatePoint2SegmentDist(lane_check_end,
                                                      segment_pt1, segment_pt2);
        if (temp_dist < MaxDistConfine) {
          topo_end_flag = true;
          break;
        }
      }

      if (topo_start_flag && topo_end_flag) {
        filtered_lane_ids_.insert(lane_check.id);
      }
    }
  }
}

double BevMapProcessor::CalculateCurvedLaneWidth2(
    const std::vector<Eigen::Vector2f>& lane1,
    const std::vector<Eigen::Vector2f>& lane2) {
  if (lane1.size() < 2 || lane2.size() < 2) {
    return 0.0;
  }

  // 1. 计算两条车道线的重叠区域
  float min_x = std::max(lane1.front().x(), lane2.front().x());
  float max_x = std::min(lane1.back().x(), lane2.back().x());

  // 如果没有重叠区域，返回0
  if (min_x >= max_x) {
    return 0.0;
  }

  std::vector<double> valid_widths;
  double sample_step = 0.2;  // 采样步长，单位为米

  // 2. 在重叠区域内均匀采样
  for (double x = min_x; x <= max_x; x += sample_step) {
    // 找到lane1上最接近x的点
    auto it1 = std::lower_bound(
        lane1.begin(), lane1.end(), x,
        [](const Eigen::Vector2f& p, double val) { return p.x() < val; });

    if (it1 == lane1.end() || it1 == lane1.begin()) continue;
    const Eigen::Vector2f& p1 = *(it1 - 1);
    const Eigen::Vector2f& p2 = *it1;

    // 线性插值计算x处的精确位置和切线方向
    double t = (x - p1.x()) / (p2.x() - p1.x());
    Eigen::Vector2f point_on_lane1 = p1 + t * (p2 - p1);
    Eigen::Vector2f tangent = (p2 - p1).normalized();
    Eigen::Vector2f normal(-tangent.y(), tangent.x());  // 垂直方向

    // 找到lane2上最接近x的点
    auto it2 = std::lower_bound(
        lane2.begin(), lane2.end(), x,
        [](const Eigen::Vector2f& p, double val) { return p.x() < val; });

    if (it2 == lane2.end() || it2 == lane2.begin()) continue;
    const Eigen::Vector2f& q1 = *(it2 - 1);
    const Eigen::Vector2f& q2 = *it2;

    // 线性插值计算x处的精确位置
    double s = (x - q1.x()) / (q2.x() - q1.x());
    Eigen::Vector2f point_on_lane2 = q1 + s * (q2 - q1);

    // 计算两点之间的垂直距离
    double width = (point_on_lane2 - point_on_lane1).dot(normal);
    valid_widths.push_back(std::fabs(width));
  }

  // 3. 计算有效宽度的平均值
  if (valid_widths.empty()) {
    return 0.0;
  }

  // 过滤异常值（可选）
  std::sort(valid_widths.begin(), valid_widths.end());
  size_t start_idx = valid_widths.size() * 0.1;  // 忽略最小的10%
  size_t end_idx = valid_widths.size() * 0.9;    // 忽略最大的10%

  double sum = 0.0;
  for (size_t i = start_idx; i < end_idx; ++i) {
    sum += valid_widths[i];
  }

  return sum / (end_idx - start_idx);
}

double BevMapProcessor::calculateOverlapDistance(
    const navigation::HarborStopInfo& harbor,
    const std::vector<Eigen::Vector2f> geos) {
  SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Entering...]";
  // 检查港湾车道是否存在且有范围
  if (!harbor.exists || harbor.ranges.empty()) {
    return 0.0;
  }

  // 获取第一个港湾车道范围 [start_s, end_s]
  const auto& firstRange = harbor.ranges[0];
  double harborStart = firstRange.first;
  double harborEnd = firstRange.second;

  // 确保范围有效
  if (harborStart >= harborEnd) {
    return 0.0;
  }

  // 获取车道线的geos点集
  // const auto& geos = *singlelane.geos;
  if (geos.size() < 2) {
    return 0.0;
  }

  // 直接使用第一个点和最后一个点的x坐标作为车道线的起点和终点
  double laneStart = geos.front().x();
  double laneEnd = geos.back().x();

  // 确保车道线起点和终点的顺序正确
  if (laneStart > laneEnd) {
    std::swap(laneStart, laneEnd);
  }

  // 计算重叠部分
  double overlapStart = std::max(laneStart, harborStart);
  double overlapEnd = std::min(laneEnd, harborEnd);
  SD_COARSE_MATCH_LOG << "[calculateOverlapDistance Exiting...]"
                      << " overlapStart: " << overlapStart
                      << " overlapEnd: " << overlapEnd;
  // 如果存在重叠，返回重叠距离
  return (overlapStart < overlapEnd) ? (overlapEnd - overlapStart) : 0.0;
}

void BevMapProcessor::ProcessStartEndTopo(std::vector<int>& target_lane_inds) {
  constexpr double EgoXMinGuideLane =
      -20.0;  // 目标添加guide_lane起始点x的最小值（ego坐标系）
  constexpr double MaxDistLong =
      8.0;  // 目标添加拓扑点的最大距离（中心线，纵向）
  constexpr double MaxDistLat =
      2.0;  // 目标添加拓扑点的最大距离（中心线，横向）
  constexpr double LMDistThreshold =
      0.15;  // 连接vector距离LM的距离阈值，小于这个值就证明越过实线

  if (data_ == nullptr) {
    return;
  }

  // 遍历所有引导车道，仅添加引导车道和自车车道的拓扑关系，end-start只需要在当前车道线挂后继就可以。
  int start_sec = 0;
  int start_ind = target_lane_inds[start_sec];
  cem::message::sensor::BevLaneInfo* ego_lane = &data_->lane_infos[start_ind];
  auto& ego_lane_start_point = ego_lane->line_points.front();
  auto& ego_lane_second_point = ego_lane->line_points.at(1);
  Eigen::Vector2d dir_ego_lane, vec_ego_lane;
  Eigen::Vector2d dir_start_end, vec_start_end;

  // AINFO << "*******StartEndTopo******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; AINFO << "ego lane id:" <<
  // ego_lane->id; AINFO << "ego lane start point x:" << ego_lane_start_point.x;
  // AINFO << "*************************";

  if (ego_lane_start_point.x < EgoXMinGuideLane) {
    return;
  }

  for (const auto& guide_lane_idx : guide_lane_inds_) {
    // 找到guide_lane实例，并判断是否符合条件。
    if (guide_lane_idx >= data_->lane_infos.size()) {
      continue;
    }
    auto guide_lane = &data_->lane_infos[guide_lane_idx];
    auto guide_lane_id = guide_lane->id;
    if (guide_lane->line_points.empty()) {
      continue;
    }
    // 当前的line_points都是处于自车坐标系，直接使用
    Eigen::Vector3d guide_lane_end_point_vcs(guide_lane->line_points.back().x,
                                             guide_lane->line_points.back().y,
                                             0.0);
    if ((guide_lane_end_point_vcs.x() < EgoXMinGuideLane) ||
        (std::find(target_lane_inds.begin(), target_lane_inds.end(),
                   guide_lane_idx) != target_lane_inds.end()) ||
        (!guide_lane->next_lane_ids.empty())) {
      continue;
    }

    // AINFO << "*******StartEndTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "guide lane id:" <<
    // guide_lane->id; AINFO << "guide start point body: "; AINFO <<
    // guide_lane_end_point_vcs; AINFO << "*************************";

    // guide_lane的起点和ego_lane的终点进行匹配，guide_lane的起点在ego_lane的终点前方一个阈值中，将其挂到ego_lane的后继当中
    auto& guide_lane_end_point = guide_lane->line_points.back();

    vec_ego_lane.x() = ego_lane_second_point.x - ego_lane_start_point.x;
    vec_ego_lane.y() = ego_lane_second_point.y - ego_lane_start_point.y;
    dir_ego_lane = vec_ego_lane / vec_ego_lane.norm();

    vec_start_end.x() = ego_lane_start_point.x - guide_lane_end_point.x;
    vec_start_end.y() = ego_lane_start_point.y - guide_lane_end_point.y;
    dir_start_end = vec_start_end / vec_start_end.norm();

    double dist_long = dir_ego_lane.dot(vec_start_end);
    double dist_lat = (vec_start_end - dist_long * dir_ego_lane).norm();
    if (dist_long > MaxDistLong || dist_long < 0 || dist_lat > MaxDistLat) {
      continue;
    }

    // AINFO << "*******StartEndTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "ego lane id: " <<
    // ego_lane->id << ", guide lane id: " << guide_lane->id; AINFO << "dist
    // longitude:" << dist_long; AINFO << "dist lateral:" << dist_lat; AINFO <<
    // "guide lane end point: (" << guide_lane_end_point.x << ", " <<
    // guide_lane_end_point.y << ")"; AINFO << "ego lane start point: (" <<
    // ego_lane_start_point.x << ", " << ego_lane_start_point.y << ")"; AINFO <<
    // "ego lane second point: (" << ego_lane_second_point.x << ", " <<
    // ego_lane_second_point.y << ")"; AINFO << "**************************";

    // AINFO << "*******StartEndTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp;
    // 生成拓扑连接线段是否与左侧lanemarker相交，找到相交的segment index
    auto seg_start = guide_lane_end_point;
    auto seg_end = ego_lane_start_point;

    double min_left_lm_dist = std::numeric_limits<double>::max();
    std::vector<int> left_min_st_idx, left_min_ed_idx;
    auto left_lm_id = ego_lane->left_lane_marker_id;
    auto left_lm = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [left_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return left_lm_id == lane_marker.id;
        });
    if (left_lm != data_->lanemarkers.end()) {
      if (left_lm->line_points.size() > 2) {
        min_left_lm_dist =
            MinDistSegPolySeg(seg_start, seg_end, left_lm->line_points,
                              left_min_st_idx, left_min_ed_idx);
        // AINFO << "right lm id: " << left_lm->id << ", " << min_left_lm_dist;
      }
    }

    // 生成拓扑连接线段是否与右侧lanemarker相交，找到相交的segment index
    double min_right_lm_dist = std::numeric_limits<double>::max();
    std::vector<int> right_min_st_idx, right_min_ed_idx;
    auto right_lm_id = ego_lane->right_lane_marker_id;
    auto right_lm = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [right_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return right_lm_id == lane_marker.id;
        });
    if (right_lm != data_->lanemarkers.end()) {
      if (right_lm->line_points.size() > 2) {
        min_right_lm_dist =
            MinDistSegPolySeg(seg_start, seg_end, right_lm->line_points,
                              right_min_st_idx, right_min_ed_idx);
        // AINFO << "right lm id: " << right_lm->id << ", " <<
        // min_right_lm_dist;
      }
    }

    // 查看对应相交segment的属性，如果有一个是BEV_LMT__SOLID，直接跳过不添加拓扑
    auto start_type =
        cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
    auto end_type = cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
    if (min_left_lm_dist < LMDistThreshold) {
      for (const auto& lm_type : left_lm->type_segs) {
        if (left_min_st_idx.front() >= lm_type.start_index &&
            left_min_st_idx.front() <= lm_type.end_index) {
          start_type = lm_type.type;
          break;
        }
      }

      for (const auto& lm_type : left_lm->type_segs) {
        if (left_min_ed_idx.front() >= lm_type.start_index &&
            left_min_ed_idx.front() <= lm_type.end_index) {
          end_type = lm_type.type;
          break;
        }
      }

      // AINFO << "left lm intersection start: (" <<
      // left_lm->line_points[left_min_st_idx.front()].x << ", " <<
      // left_lm->line_points[left_min_st_idx.front()].y << "): " << start_type;
      // AINFO << "left lm intersection end: ("<<
      // left_lm->line_points[left_min_ed_idx.front()].x << ", " <<
      // left_lm->line_points[left_min_ed_idx.front()].y << "): " << end_type;
    }

    if (min_right_lm_dist < LMDistThreshold) {
      for (const auto& lm_type : right_lm->type_segs) {
        if (right_min_st_idx.front() >= lm_type.start_index &&
            right_min_st_idx.front() <= lm_type.end_index) {
          start_type = lm_type.type;
          break;
        }
      }

      for (const auto& lm_type : right_lm->type_segs) {
        if (right_min_ed_idx.front() >= lm_type.start_index &&
            right_min_ed_idx.front() <= lm_type.end_index) {
          end_type = lm_type.type;
          break;
        }
      }

      // AINFO << "right lm intersection start: (" <<
      // right_lm->line_points[right_min_st_idx.front()].x << ", " <<
      // right_lm->line_points[right_min_st_idx.front()].y << "): " <<
      // start_type; AINFO << "right lm intersection end: ("<<
      // right_lm->line_points[right_min_ed_idx.front()].x << ", " <<
      // right_lm->line_points[right_min_ed_idx.front()].y << "): " << end_type;
    }

    // start_type和end_type有一个为实线，不连接拓扑
    if (start_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
        start_type == cem::message::sensor::BevLaneMarkerType::
                          BEV_LMT__DOUBLE_SOLID_SOLID ||
        end_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
        end_type == cem::message::sensor::BevLaneMarkerType::
                        BEV_LMT__DOUBLE_SOLID_SOLID) {
      continue;
    }
    // AINFO << "*************************";

    if (std::find(ego_lane->previous_lane_ids.begin(),
                  ego_lane->previous_lane_ids.end(),
                  guide_lane_id) == ego_lane->previous_lane_ids.end()) {
      ego_lane->previous_lane_ids.push_back(guide_lane_id);
    }

    if (std::find(guide_lane->next_lane_ids.begin(),
                  guide_lane->next_lane_ids.end(),
                  ego_lane->id) == guide_lane->next_lane_ids.end()) {
      guide_lane->next_lane_ids.push_back(ego_lane->id);
    }
  }

  // AINFO << "*******StartEndTopo******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto lane :
  // data_->lane_infos){
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto prev_lane_id : lane.previous_lane_ids){
  //     AINFO << "previous lane id: " << prev_lane_id;
  //   }
  //   for (const auto next_lane_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_lane_id;
  //   }
  // }
  // AINFO << "*************************";
}

void BevMapProcessor::ProcessEndStartTopo(std::vector<int>& target_lane_inds) {
  constexpr double EgoXMinGuideLane =
      -20.0;  // 目标添加guide_lane起始点x的最小值（ego坐标系）
  constexpr double MaxDistLong =
      8.0;  // 目标添加拓扑点的最大距离（中心线，纵向）
  constexpr double MaxDistLat =
      2.0;  // 目标添加拓扑点的最大距离（中心线，横向）
  constexpr double LMDistThreshold =
      0.15;  // 连接vector距离LM的距离阈值，小于这个值就证明越过实线

  if (data_ == nullptr) {
    return;
  }

  // 遍历所有引导车道，仅添加引导车道和自车车道的拓扑关系，end-start只需要在当前车道线挂后继就可以。
  int last_sec = target_lane_inds.size() - 1;
  int end_ind = target_lane_inds[last_sec];
  cem::message::sensor::BevLaneInfo* ego_lane = &data_->lane_infos[end_ind];
  auto& ego_lane_end_point = ego_lane->line_points.back();
  auto& ego_lane_second_last_point =
      ego_lane->line_points.at(ego_lane->line_points.size() - 2);
  Eigen::Vector2d dir_ego_lane, vec_ego_lane;
  Eigen::Vector2d dir_end_start, vec_end_start;

  for (const auto& guide_lane_idx : guide_lane_inds_) {
    // 找到guide_lane实例，并判断是否符合条件。
    if (guide_lane_idx >= data_->lane_infos.size()) {
      continue;
    }
    auto guide_lane = &data_->lane_infos[guide_lane_idx];
    auto guide_lane_id = guide_lane->id;
    if (guide_lane->line_points.empty()) {
      continue;
    }
    // 当前的line_points都是处于自车坐标系，直接使用
    Eigen::Vector3d guide_lane_start_point_vcs(
        guide_lane->line_points.front().x, guide_lane->line_points.front().y,
        0.0);
    if ((guide_lane_start_point_vcs.x() < EgoXMinGuideLane) ||
        (std::find(target_lane_inds.begin(), target_lane_inds.end(),
                   guide_lane_idx) != target_lane_inds.end()) ||
        (!guide_lane->previous_lane_ids.empty())) {
      continue;
    }

    // AINFO << "*******EndStartTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "guide lane id:" <<
    // guide_lane->id; AINFO << "guide start point body: "; AINFO <<
    // guide_lane_start_point_vcs; AINFO << "*************************";

    // guide_lane的起点和ego_lane的终点进行匹配，guide_lane的起点在ego_lane的终点前方一个阈值中，将其挂到ego_lane的后继当中
    auto& guide_lane_start_point = guide_lane->line_points.front();

    vec_ego_lane.x() = ego_lane_end_point.x - ego_lane_second_last_point.x;
    vec_ego_lane.y() = ego_lane_end_point.y - ego_lane_second_last_point.y;
    dir_ego_lane = vec_ego_lane / vec_ego_lane.norm();

    vec_end_start.x() = guide_lane_start_point.x - ego_lane_end_point.x;
    vec_end_start.y() = guide_lane_start_point.y - ego_lane_end_point.y;
    dir_end_start = vec_end_start / vec_end_start.norm();

    double dist_long = dir_ego_lane.dot(vec_end_start);
    double dist_lat = (vec_end_start - dist_long * dir_ego_lane).norm();

    // AINFO << "*******EndStartTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "ego lane id: " <<
    // ego_lane->id << ", guide lane id: " << guide_lane->id; AINFO << "dist
    // longitude:" << dist_long; AINFO << "dist lateral:" << dist_lat; AINFO <<
    // "guide lane start point: (" << guide_lane_start_point.x << ", " <<
    // guide_lane_start_point.y << ")"; AINFO << "ego lane end point: (" <<
    // ego_lane_end_point.x << ", " << ego_lane_end_point.y << ")"; AINFO <<
    // "ego lane second last point: (" << ego_lane_second_last_point.x << ", "
    // << ego_lane_second_last_point.y << ")"; AINFO <<
    // "*************************";

    if (dist_long > MaxDistLong || dist_long < 0 || dist_lat > MaxDistLat) {
      continue;
    }

    // AINFO << "*******EndStartTopo******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp;
    // 生成拓扑连接线段是否与左侧lanemarker相交，找到相交的segment index
    auto seg_start = ego_lane_end_point;
    auto seg_end = guide_lane_start_point;

    double min_left_lm_dist = std::numeric_limits<double>::max();
    std::vector<int> left_min_st_idx, left_min_ed_idx;
    auto left_lm_id = ego_lane->left_lane_marker_id;
    auto left_lm = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [left_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return left_lm_id == lane_marker.id;
        });
    if (left_lm != data_->lanemarkers.end()) {
      if (left_lm->line_points.size() > 2) {
        min_left_lm_dist =
            MinDistSegPolySeg(seg_start, seg_end, left_lm->line_points,
                              left_min_st_idx, left_min_ed_idx);
        // AINFO << "right lm id: " << left_lm->id << ", " << min_left_lm_dist;
      }
    }

    // 生成拓扑连接线段是否与右侧lanemarker相交，找到相交的segment index
    double min_right_lm_dist = std::numeric_limits<double>::max();
    std::vector<int> right_min_st_idx, right_min_ed_idx;
    auto right_lm_id = ego_lane->right_lane_marker_id;
    auto right_lm = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [right_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return right_lm_id == lane_marker.id;
        });
    if (right_lm != data_->lanemarkers.end()) {
      if (right_lm->line_points.size() > 2) {
        min_right_lm_dist =
            MinDistSegPolySeg(seg_start, seg_end, right_lm->line_points,
                              right_min_st_idx, right_min_ed_idx);
        // AINFO << "right lm id: " << right_lm->id << ", " <<
        // min_right_lm_dist;
      }
    }

    // 查看对应相交segment的属性，如果有一个是BEV_LMT__SOLID，直接跳过不添加拓扑
    auto start_type =
        cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
    auto end_type = cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
    if (min_left_lm_dist < LMDistThreshold) {
      for (const auto& lm_type : left_lm->type_segs) {
        if (left_min_st_idx.front() >= lm_type.start_index &&
            left_min_st_idx.front() <= lm_type.end_index) {
          start_type = lm_type.type;
          break;
        }
      }

      for (const auto& lm_type : left_lm->type_segs) {
        if (left_min_ed_idx.front() >= lm_type.start_index &&
            left_min_ed_idx.front() <= lm_type.end_index) {
          end_type = lm_type.type;
          break;
        }
      }

      // AINFO << "left lm intersection start: (" <<
      // left_lm->line_points[left_min_st_idx.front()].x << ", " <<
      // left_lm->line_points[left_min_st_idx.front()].y << "): " << start_type;
      // AINFO << "left lm intersection end: ("<<
      // left_lm->line_points[left_min_ed_idx.front()].x << ", " <<
      // left_lm->line_points[left_min_ed_idx.front()].y << "): " << end_type;
    }

    if (min_right_lm_dist < LMDistThreshold) {
      for (const auto& lm_type : right_lm->type_segs) {
        if (right_min_st_idx.front() >= lm_type.start_index &&
            right_min_st_idx.front() <= lm_type.end_index) {
          start_type = lm_type.type;
          break;
        }
      }

      for (const auto& lm_type : right_lm->type_segs) {
        if (right_min_ed_idx.front() >= lm_type.start_index &&
            right_min_ed_idx.front() <= lm_type.end_index) {
          end_type = lm_type.type;
          break;
        }
      }

      // AINFO << "right lm intersection start: (" <<
      // right_lm->line_points[right_min_st_idx.front()].x << ", " <<
      // right_lm->line_points[right_min_st_idx.front()].y << "): " <<
      // start_type; AINFO << "right lm intersection end: ("<<
      // right_lm->line_points[right_min_ed_idx.front()].x << ", " <<
      // right_lm->line_points[right_min_ed_idx.front()].y << "): " << end_type;
    }

    // start_type和end_type有一个为实线，不连接拓扑
    if (start_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
        start_type == cem::message::sensor::BevLaneMarkerType::
                          BEV_LMT__DOUBLE_SOLID_SOLID ||
        end_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
        end_type == cem::message::sensor::BevLaneMarkerType::
                        BEV_LMT__DOUBLE_SOLID_SOLID) {
      continue;
    }
    // AINFO << "*************************";

    if (std::find(ego_lane->next_lane_ids.begin(),
                  ego_lane->next_lane_ids.end(),
                  guide_lane_id) == ego_lane->next_lane_ids.end()) {
      ego_lane->next_lane_ids.push_back(guide_lane_id);
    }

    if (std::find(guide_lane->previous_lane_ids.begin(),
                  guide_lane->previous_lane_ids.end(),
                  ego_lane->id) == guide_lane->previous_lane_ids.end()) {
      guide_lane->previous_lane_ids.push_back(ego_lane->id);
    }
  }

  // AINFO << "*******EndStartTopo******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto lane :
  // data_->lane_infos){
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto prev_lane_id : lane.previous_lane_ids){
  //     AINFO << "previous lane id: " << prev_lane_id;
  //   }
  //   for (const auto next_lane_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_lane_id;
  //   }
  // }
  // AINFO << "*************************";
}

void BevMapProcessor::ProcessYshapeSplit(std::vector<int>& target_lane_inds) {
  constexpr double EgoXMinGuideLane =
      -20.0;  // 目标添加guide_lane起始点x的最小值（ego坐标系）
  constexpr double MaxDistConfine =
      2.0;  // 目标添加拓扑点的最大距离阈值（中心线，精）
  constexpr double MaxDistSqMarker =
      10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr int PtNumValueStartEnd =
      2;  // 确认切分后首尾所剩line_points数量的最小值
  constexpr int PtNumValueMid =
      3;  // 确认切分后中间后所剩line_points数量的最小值
  constexpr double LMDistThreshold =
      0.15;  // 连接vector距离LM的距离阈值，小于这个值就证明越过实线

  if (data_ == nullptr) {
    return;
  }
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  // AINFO << "*******YshapeSplit******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; AINFO << "Recommend lane number is
  // not 1, but " << sd_guide_lanes.size() << "."; for (const auto &lane :
  // data_->lane_infos) {
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto &prev_id : lane.previous_lane_ids) {
  //     AINFO << "previous lane id: " << prev_id;
  //   }
  //   for (const auto &next_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_id;
  //   }
  // }
  // AINFO << "*************************";

  // 遍历所有引导车道，仅添加引导车道和自车车道的拓扑关系，Y型拓扑主要通过引导车道的起点和ego_lane的所有点进行匹配，找到打断点，需要打断自车车道。
  cem::message::sensor::BevLaneInfo* ego_lane = nullptr;
  cem::message::sensor::BevLaneInfo* prev_ego_lane = nullptr;
  for (const auto& guide_lane_idx : guide_lane_inds_) {
    // 找到guide_lane实例，并判断是否符合条件。
    if (guide_lane_idx >= data_->lane_infos.size()) {
      continue;
    }
    auto guide_lane = &data_->lane_infos[guide_lane_idx];
    auto guide_lane_id = guide_lane->id;
    if (guide_lane->line_points.empty()) {
      continue;
    }
    // 当前的line_points都是处于自车坐标系，直接使用
    Eigen::Vector3d guide_lane_start_point_vcs(
        guide_lane->line_points.front().x, guide_lane->line_points.front().y,
        0.0);
    if ((guide_lane_start_point_vcs.x() < EgoXMinGuideLane) ||
        (std::find(target_lane_inds.begin(), target_lane_inds.end(),
                   guide_lane_idx) != target_lane_inds.end()) ||
        (!guide_lane->previous_lane_ids.empty())) {
      continue;
    }

    // AINFO << "*******YshapeSplit******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "guide lane id:" <<
    // guide_lane->id; AINFO << "guide start point body: "; AINFO <<
    // guide_lane_start_point_vcs; AINFO << "*************************";

    // guide_lane的起点和ego_lane的所有点进行匹配，找到打断点。ego_lane会被打断，
    double distance_pt2seg = std::numeric_limits<double>::max();
    int split_section = -1;
    std::vector<cem::message::common::Point2DF>::iterator split_point;
    auto& guide_lane_start_point = guide_lane->line_points.front();
    for (int sec = 0; sec < target_lane_inds.size(); ++sec) {
      int ind = target_lane_inds[sec];
      ego_lane = &data_->lane_infos[ind];
      for (auto iter = ego_lane->line_points.begin() + 1;
           iter != ego_lane->line_points.end(); ++iter) {
        auto temp_start = *(iter - 1);
        auto temp_end = *iter;
        double tmp_dist = CalculatePoint2SegmentDist(guide_lane_start_point,
                                                     temp_start, temp_end);
        if (distance_pt2seg > tmp_dist) {
          split_point = iter;
          split_section = sec;
          distance_pt2seg = tmp_dist;
        }
      }

      if (sec < target_lane_inds.size() - 1) {
        int ind_st = target_lane_inds[sec];
        int ind_ed = target_lane_inds[sec + 1];
        auto temp_start = data_->lane_infos[ind_st].line_points.back();
        auto temp_end = data_->lane_infos[ind_ed].line_points.front();
        double tmp_dist = CalculatePoint2SegmentDist(guide_lane_start_point,
                                                     temp_start, temp_end);
        if (distance_pt2seg > tmp_dist) {
          split_point = data_->lane_infos[ind_ed].line_points.begin();
          split_section = sec + 1;
          distance_pt2seg = tmp_dist;
        }
      }
    }
    int target_ego_lane_ind = target_lane_inds[split_section];
    ego_lane = &data_->lane_infos[target_ego_lane_ind];
    // AINFO << "*******YshapeSplit******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "target ego lane index: "
    // << target_ego_lane_ind; AINFO << "ego lane id: " << ego_lane->id << ",
    // guide lane id: " << guide_lane->id; AINFO << "split point: (" <<
    // split_point->x << ", " << split_point->y << ")"; AINFO << "guide lane
    // start: (" << guide_lane_start_point.x << ", " << guide_lane_start_point.y
    // << ")"; AINFO << "split section: " << split_section; AINFO << "remaining
    // points: " <<  ego_lane->line_points.end() - split_point; AINFO << "dist:"
    // << distance_pt2seg; AINFO << "*************************";

    if (distance_pt2seg > MaxDistConfine) {
      // AINFO << "TEST: " << distance_square;
      continue;
    }

    // AINFO << "*******YshapeSplit******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp;
    // 拓扑的start点设置规则，这个较为复杂，要分多种情况
    bool get_seg_start = false;
    cem::message::common::Point2DF seg_start;
    // seg_start为前继的终点
    if (split_section == 0 &&
        split_point - ego_lane->line_points.begin() < PtNumValueStartEnd) {
      if (!ego_lane->previous_lane_ids.empty()) {
        auto previous_lane_id = ego_lane->previous_lane_ids.front();
        auto ego_lane_previous = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [previous_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return previous_lane_id == lane.id;
            });
        if (ego_lane_previous != data_->lane_infos.end()) {
          get_seg_start = true;
          seg_start = ego_lane_previous->line_points.back();
        }
      }
      // seg_start为本条车道的终点
    } else if (split_section == target_lane_inds.size() - 1 &&
               ego_lane->line_points.end() - split_point < PtNumValueStartEnd) {
      get_seg_start = true;
      seg_start = ego_lane->line_points.back();
      // seg_start为切分后多条车道的中间断点
    } else if (split_section > 0 &&
               split_point - ego_lane->line_points.begin() <
                   PtNumValueStartEnd) {
      if (!ego_lane->previous_lane_ids.empty()) {
        auto previous_lane_id = ego_lane->previous_lane_ids.front();
        auto ego_lane_previous = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [previous_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return previous_lane_id == lane.id;
            });
        if (ego_lane_previous != data_->lane_infos.end()) {
          get_seg_start = true;
          seg_start = ego_lane_previous->line_points.back();
        }
      }
    } else if (split_section < target_lane_inds.size() - 1 &&
               ego_lane->line_points.end() - split_point < PtNumValueMid) {
      get_seg_start = true;
      seg_start = ego_lane->line_points.back();
    } else {
      get_seg_start = true;
      std::vector<cem::message::common::Point2DF>::iterator
          previous_split_point = split_point - 1;
      seg_start = *previous_split_point;
    }

    if (get_seg_start) {
      auto seg_end = guide_lane_start_point;

      // 生成拓扑连接线段是否与左侧lanemarker相交，找到相交的segment index
      double min_left_lm_dist = std::numeric_limits<double>::max();
      std::vector<int> left_min_st_idx, left_min_ed_idx;
      auto left_lm_id = ego_lane->left_lane_marker_id;
      auto left_lm = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [left_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return left_lm_id == lane_marker.id;
          });
      if (left_lm != data_->lanemarkers.end()) {
        if (left_lm->line_points.size() > 2) {
          min_left_lm_dist =
              MinDistSegPolySeg(seg_start, seg_end, left_lm->line_points,
                                left_min_st_idx, left_min_ed_idx);
          // AINFO << "right lm id: " << left_lm->id << ", " <<
          // min_left_lm_dist;
        }
      }

      // 生成拓扑连接线段是否与右侧lanemarker相交，找到相交的segment index
      double min_right_lm_dist = std::numeric_limits<double>::max();
      std::vector<int> right_min_st_idx, right_min_ed_idx;
      auto right_lm_id = ego_lane->right_lane_marker_id;
      auto right_lm = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [right_lm_id](
              const cem::message::sensor::BevLaneMarker& lane_marker) {
            return right_lm_id == lane_marker.id;
          });
      if (right_lm != data_->lanemarkers.end()) {
        if (right_lm->line_points.size() > 2) {
          min_right_lm_dist =
              MinDistSegPolySeg(seg_start, seg_end, right_lm->line_points,
                                right_min_st_idx, right_min_ed_idx);
          // AINFO << "right lm id: " << right_lm->id << ", " <<
          // min_right_lm_dist;
        }
      }

      // 查看对应相交segment的属性，如果有一个是BEV_LMT__SOLID，直接跳过不添加拓扑
      auto start_type =
          cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
      auto end_type =
          cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
      if (min_left_lm_dist < LMDistThreshold) {
        for (const auto& lm_type : left_lm->type_segs) {
          if (left_min_st_idx.front() >= lm_type.start_index &&
              left_min_st_idx.front() <= lm_type.end_index) {
            start_type = lm_type.type;
            break;
          }
        }

        for (const auto& lm_type : left_lm->type_segs) {
          if (left_min_ed_idx.front() >= lm_type.start_index &&
              left_min_ed_idx.front() <= lm_type.end_index) {
            end_type = lm_type.type;
            break;
          }
        }

        // AINFO << "left lm intersection start: (" <<
        // left_lm->line_points[left_min_st_idx.front()].x << ", " <<
        // left_lm->line_points[left_min_st_idx.front()].y << "): " <<
        // start_type; AINFO << "left lm intersection end: ("<<
        // left_lm->line_points[left_min_ed_idx.front()].x << ", " <<
        // left_lm->line_points[left_min_ed_idx.front()].y << "): " << end_type;
      }

      if (min_right_lm_dist < LMDistThreshold) {
        for (const auto& lm_type : right_lm->type_segs) {
          if (right_min_st_idx.front() >= lm_type.start_index &&
              right_min_st_idx.front() <= lm_type.end_index) {
            start_type = lm_type.type;
            break;
          }
        }

        for (const auto& lm_type : right_lm->type_segs) {
          if (right_min_ed_idx.front() >= lm_type.start_index &&
              right_min_ed_idx.front() <= lm_type.end_index) {
            end_type = lm_type.type;
            break;
          }
        }

        // AINFO << "right lm intersection start: (" <<
        // right_lm->line_points[right_min_st_idx.front()].x << ", " <<
        // right_lm->line_points[right_min_st_idx.front()].y << "): " <<
        // start_type; AINFO << "right lm intersection end: ("<<
        // right_lm->line_points[right_min_ed_idx.front()].x << ", " <<
        // right_lm->line_points[right_min_ed_idx.front()].y << "): " <<
        // end_type;
      }

      // start_type和end_type有一个为实线，不连接拓扑
      if (start_type ==
              cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
          start_type == cem::message::sensor::BevLaneMarkerType::
                            BEV_LMT__DOUBLE_SOLID_SOLID ||
          end_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
          end_type == cem::message::sensor::BevLaneMarkerType::
                          BEV_LMT__DOUBLE_SOLID_SOLID) {
        continue;
      }
    }
    // AINFO << "*************************";

    // 判定跳过的条件:
    // 假设处于起始段，打断点相比于起始点前的点数小于PtNumValueStartEnd，则将guide_lane挂为ego_lane的前继。
    if (split_section == 0 &&
        split_point - ego_lane->line_points.begin() < PtNumValueStartEnd) {
      if (!ego_lane->previous_lane_ids.empty()) {
        guide_lane->previous_lane_ids = ego_lane->previous_lane_ids;
        for (const auto& previous_lane_id : ego_lane->previous_lane_ids) {
          auto ego_lane_previous =
              std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                           [previous_lane_id](
                               const cem::message::sensor::BevLaneInfo& lane) {
                             return previous_lane_id == lane.id;
                           });
          if (ego_lane_previous == data_->lane_infos.end()) {
            continue;
          }
          ego_lane_previous->next_lane_ids.push_back(guide_lane->id);
        }
      }
      continue;
    }

    // 判定跳过的条件:
    // 假设处于终止段，打断点相比于终止点前的点数小于PtNumValueStartEnd，则跳直接将guide_lane挂为后继。
    if (split_section == target_lane_inds.size() - 1 &&
        ego_lane->line_points.end() - split_point < PtNumValueStartEnd) {
      if (!ego_lane->next_lane_ids.empty()) {
        auto next_lane_id = ego_lane->next_lane_ids[0];
        auto ego_lane_next = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return next_lane_id == lane.id;
            });
        if (ego_lane_next != data_->lane_infos.end()) {
          guide_lane->previous_lane_ids = ego_lane_next->previous_lane_ids;
        }
      }

      if (std::find(ego_lane->next_lane_ids.begin(),
                    ego_lane->next_lane_ids.end(),
                    guide_lane_id) == ego_lane->next_lane_ids.end()) {
        ego_lane->next_lane_ids.push_back(guide_lane_id);
      }

      if (std::find(guide_lane->previous_lane_ids.begin(),
                    guide_lane->previous_lane_ids.end(),
                    ego_lane->id) == guide_lane->previous_lane_ids.end()) {
        guide_lane->previous_lane_ids.push_back(ego_lane->id);
      }
      continue;
    }

    // 判定跳过的条件: 假设处于中间段起点或终点
    // 假设与起始点较近，直接将新的支路绑定为前一个sublane的后继，无需打断车道
    if (split_section > 0 &&
        split_point - ego_lane->line_points.begin() < PtNumValueMid) {
      if (!ego_lane->previous_lane_ids.empty()) {
        guide_lane->previous_lane_ids = ego_lane->previous_lane_ids;
        for (const auto& previous_lane_id : ego_lane->previous_lane_ids) {
          auto ego_lane_previous =
              std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                           [previous_lane_id](
                               const cem::message::sensor::BevLaneInfo& lane) {
                             return previous_lane_id == lane.id;
                           });
          if (ego_lane_previous == data_->lane_infos.end()) {
            continue;
          }
          ego_lane_previous->next_lane_ids.push_back(guide_lane->id);
        }
      }
      guide_lane->split_topo_extend =
          cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN;
      continue;
    }
    // 假设与终止点较近，直接将新的支路绑定为前一个sublane的后继，无需打断车道
    if (split_section < target_lane_inds.size() - 1 &&
        ego_lane->line_points.end() - split_point < PtNumValueMid) {
      if (!ego_lane->next_lane_ids.empty()) {
        auto next_lane_id = ego_lane->next_lane_ids[0];
        auto ego_lane_next = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return next_lane_id == lane.id;
            });
        if (ego_lane_next != data_->lane_infos.end()) {
          guide_lane->previous_lane_ids = ego_lane_next->previous_lane_ids;
        }
      }

      if (std::find(ego_lane->next_lane_ids.begin(),
                    ego_lane->next_lane_ids.end(),
                    guide_lane_id) == ego_lane->next_lane_ids.end()) {
        ego_lane->next_lane_ids.push_back(guide_lane_id);
      }

      if (std::find(guide_lane->previous_lane_ids.begin(),
                    guide_lane->previous_lane_ids.end(),
                    ego_lane->id) == guide_lane->previous_lane_ids.end()) {
        guide_lane->previous_lane_ids.push_back(ego_lane->id);
      }
      guide_lane->split_topo_extend =
          cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN;
      continue;
    }

    uint64_t new_id = 101;
    for (uint64_t i = 1; i < 100UL; ++i) {
      auto tmp =
          std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                       [i](const cem::message::sensor::BevLaneInfo& lane) {
                         return lane.id == i;
                       });
      if (tmp == data_->lane_infos.end()) {
        new_id = i;
        break;
      }
    }
    auto splited_lane = *ego_lane;
    splited_lane.line_points.clear();
    splited_lane.line_points.insert(splited_lane.line_points.end(), split_point,
                                    ego_lane->line_points.end());
    splited_lane.number_of_points = splited_lane.line_points.size();
    splited_lane.id = new_id;
    splited_lane.position =
        static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);

    ego_lane->line_points.erase(split_point, ego_lane->line_points.end());
    ego_lane->number_of_points = ego_lane->line_points.size();
    ego_lane->next_lane_ids.clear();
    ego_lane->next_lane_ids.push_back(new_id);
    ego_lane->next_lane_ids.push_back(guide_lane_id);

    guide_lane->previous_lane_ids.push_back(ego_lane->id);

    // // 主道和分叉的向量，后续可以删除，暂时保留
    // float x_split = guide_lane->line_points.back().x - split_point->x;
    // float y_split = guide_lane->line_points.back().y - split_point->y;
    // float x_main  = splited_lane.line_points.back().x - split_point->x;
    // float y_main  = splited_lane.line_points.back().y - split_point->y;

    // AINFO << "ego lane id: " << ego_lane->id << ", "
    //       << ego_lane->line_points.size();
    // AINFO << "split lane id: " << splited_lane.id << ","
    //       << splited_lane.line_points.size();
    // if (x_main * y_split - x_split * y_main < 0) {
    //   guide_lane->split_topo_extend =
    //       cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
    // } else {
    //   guide_lane->split_topo_extend =
    //       cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
    // }
    guide_lane->split_topo_extend =
        cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN;
    splited_lane.previous_lane_ids.clear();
    splited_lane.previous_lane_ids.push_back(ego_lane->id);

    // 假如原始ego_lane存在后继，修改后继lane的前继，将其改为打断后的
    for (const auto& next_lane_id : splited_lane.next_lane_ids) {
      auto ego_lane_next = std::find_if(
          data_->lane_infos.begin(), data_->lane_infos.end(),
          [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
            return next_lane_id == lane.id;
          });
      if (ego_lane_next == data_->lane_infos.end()) {
        continue;
      }
      std::replace(ego_lane_next->previous_lane_ids.begin(),
                   ego_lane_next->previous_lane_ids.end(), ego_lane->id,
                   splited_lane.id);
    }

    // AINFO << "*******YshapeSplit******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; for (const auto & lane:
    // data_->lane_infos){
    //   AINFO << "current lane id: " << lane.id;
    //   for (const auto & prev_id: lane.previous_lane_ids){
    //     AINFO << "previous lane id:" << prev_id;
    //   }
    //   for (const auto & next_id: lane.next_lane_ids){
    //     AINFO << "next lane id:" << next_id;
    //   }
    // }
    // AINFO << "*************************";

    auto id = ego_lane->left_lane_marker_id;
    auto lane_line = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return id == lane_marker.id;
        });
    if (lane_line != data_->lanemarkers.end()) {
      if (!lane_line->line_points.empty()) {
        auto left_lane_marker_first = *lane_line;
        auto left_lane_marker_second = *lane_line;
        left_lane_marker_second.line_points.clear();

        double distance_square = std::numeric_limits<double>::max();
        std::vector<cem::message::common::Point2DF>::iterator
            marker_split_point;
        for (auto iter = left_lane_marker_first.line_points.begin();
             iter != left_lane_marker_first.line_points.end(); ++iter) {
          double tmp_square =
              (split_point->x - iter->x) * (split_point->x - iter->x) +
              (split_point->y - iter->y) * (split_point->y - iter->y);
          if (distance_square > tmp_square) {
            marker_split_point = iter;
            distance_square = tmp_square;
          }
        }

        if (distance_square < MaxDistSqMarker) {
          left_lane_marker_second.line_points.insert(
              left_lane_marker_second.line_points.end(), marker_split_point,
              left_lane_marker_first.line_points.end());
          left_lane_marker_second.number_of_points =
              left_lane_marker_second.line_points.size();
          left_lane_marker_second.id = generate_lanemarker_id();
          splited_lane.left_lane_marker_id = left_lane_marker_second.id;
          data_->lanemarkers.push_back(std::move(left_lane_marker_second));

          left_lane_marker_first.line_points.erase(
              marker_split_point, left_lane_marker_first.line_points.end());
          left_lane_marker_first.number_of_points =
              left_lane_marker_first.line_points.size();
          left_lane_marker_first.id = generate_lanemarker_id();
          ego_lane->left_lane_marker_id = left_lane_marker_first.id;
          data_->lanemarkers.push_back(std::move(left_lane_marker_first));
        }
      }
    }

    id = ego_lane->right_lane_marker_id;
    lane_line = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return id == lane_marker.id;
        });
    if (lane_line != data_->lanemarkers.end()) {
      if (!lane_line->line_points.empty()) {
        auto right_lane_marker_first = *lane_line;
        auto right_lane_marker_second = *lane_line;
        right_lane_marker_second.line_points.clear();

        double distance_square = std::numeric_limits<double>::max();
        std::vector<cem::message::common::Point2DF>::iterator
            marker_split_point;
        for (auto iter = right_lane_marker_first.line_points.begin();
             iter != right_lane_marker_first.line_points.end(); ++iter) {
          double tmp_square =
              (split_point->x - iter->x) * (split_point->x - iter->x) +
              (split_point->y - iter->y) * (split_point->y - iter->y);
          if (distance_square > tmp_square) {
            marker_split_point = iter;
            distance_square = tmp_square;
          }
        }

        if (distance_square < MaxDistSqMarker) {
          right_lane_marker_second.line_points.insert(
              right_lane_marker_second.line_points.end(), marker_split_point,
              right_lane_marker_first.line_points.end());
          right_lane_marker_second.number_of_points =
              right_lane_marker_second.line_points.size();
          right_lane_marker_second.id = generate_lanemarker_id();
          splited_lane.right_lane_marker_id = right_lane_marker_second.id;
          data_->lanemarkers.push_back(std::move(right_lane_marker_second));

          right_lane_marker_first.line_points.erase(
              marker_split_point, right_lane_marker_first.line_points.end());
          right_lane_marker_first.number_of_points =
              right_lane_marker_first.line_points.size();
          right_lane_marker_first.id = generate_lanemarker_id();
          ego_lane->right_lane_marker_id = right_lane_marker_first.id;
          data_->lanemarkers.push_back(std::move(right_lane_marker_first));
        }
      }
    }

    data_->lanemarkers_num = data_->lanemarkers.size();
    data_->lane_infos.push_back(std::move(splited_lane));

    target_lane_inds.insert(target_lane_inds.begin() + (split_section + 1),
                            data_->lane_infos.size() - 1);
  }

  // AINFO << "*******YshapeSplit******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (int idx : target_lane_inds){
  //   ego_lane = &data_->lane_infos[idx];
  //   AINFO << "ego lane id: " << ego_lane->id;
  //   AINFO << "ego lane start: (" << ego_lane->line_points.front().x << ", "
  //   << ego_lane->line_points.front().y << ")"; AINFO << "ego lane end: (" <<
  //   ego_lane->line_points.back().x << ", " << ego_lane->line_points.back().y
  //   << ")";
  // }
  // AINFO << "*************************";

  // AINFO << "*******YshapeSplit******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto lane :
  // data_->lane_infos){
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto prev_lane_id : lane.previous_lane_ids){
  //     AINFO << "previous lane id: " << prev_lane_id;
  //   }
  //   for (const auto next_lane_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_lane_id;
  //   }
  // }
  // AINFO << "*************************";
}

void BevMapProcessor::ProcessYshapeMerge(std::vector<int>& target_lane_inds) {
  constexpr double EgoXMinGuideLane =
      -20.0;  // 目标添加guide_lane起始点x的最小值（ego坐标系）
  constexpr double MaxDistConfine =
      2.0;  // 目标添加拓扑点的最大距离阈值（中心线，精）
  constexpr double MaxDistSqMarker =
      10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr int PtNumValueStartEnd =
      2;  // 确认切分后首尾所剩line_points数量的最小值
  constexpr int PtNumValueMid =
      3;  // 确认切分后中间后所剩line_points数量的最小值
  constexpr double LMDistThreshold =
      0.15;  // 连接vector距离LM的距离阈值，小于这个值就证明越过实线

  if (data_ == nullptr) {
    return;
  }
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  // AINFO << "*******YshapeMerge******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; AINFO << "Recommend lane number is
  // not 1, but " << sd_guide_lanes.size() << "."; for (const auto &lane :
  // data_->lane_infos) {
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto &prev_id : lane.previous_lane_ids) {
  //     AINFO << "previous lane id: " << prev_id;
  //   }
  //   for (const auto &next_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_id;
  //   }
  // }
  // AINFO << "*************************";

  // 遍历所有引导车道，仅添加引导车道和自车车道的拓扑关系，Y型拓扑主要通过引导车道的起点和ego_lane的所有点进行匹配，找到打断点，需要打断自车车道。
  cem::message::sensor::BevLaneInfo* ego_lane = nullptr;
  cem::message::sensor::BevLaneInfo* next_ego_lane = nullptr;
  for (const auto& guide_lane_idx : guide_lane_inds_) {
    // 找到guide_lane实例，并判断是否符合条件。
    if (guide_lane_idx >= data_->lane_infos.size()) {
      continue;
    }
    auto guide_lane = &data_->lane_infos[guide_lane_idx];
    auto guide_lane_id = guide_lane->id;
    if (guide_lane->line_points.empty()) {
      continue;
    }
    // 当前的line_points都是处于自车坐标系，直接使用
    Eigen::Vector3d guide_lane_end_point_vcs(guide_lane->line_points.back().x,
                                             guide_lane->line_points.back().y,
                                             0.0);
    if ((guide_lane_end_point_vcs.x() < EgoXMinGuideLane) ||
        (std::find(target_lane_inds.begin(), target_lane_inds.end(),
                   guide_lane_idx) != target_lane_inds.end()) ||
        (!guide_lane->next_lane_ids.empty())) {
      continue;
    }

    if ((!guide_lane->line_points.back().scatter_visibility)) {
      continue;
    }

    // AINFO << "*******YshapeMerge******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "guide lane id: " <<
    // guide_lane->id; AINFO << "guide start point body: "; AINFO <<
    // guide_lane_end_point_vcs; AINFO << "************************";

    // guide_lane的起点和ego_lane的所有点进行匹配，找到打断点。ego_lane会被打断，
    double distance_pt2seg = std::numeric_limits<double>::max();
    int merge_section = -1;
    std::vector<cem::message::common::Point2DF>::iterator merge_point;
    auto& guide_lane_end_point = guide_lane->line_points.back();
    for (int sec = 0; sec < target_lane_inds.size(); ++sec) {
      int ind = target_lane_inds[sec];
      ego_lane = &data_->lane_infos[ind];
      for (auto iter = ego_lane->line_points.begin() + 1;
           iter != ego_lane->line_points.end(); ++iter) {
        auto temp_start = *(iter - 1);
        auto temp_end = *iter;
        double tmp_dist = CalculatePoint2SegmentDist(guide_lane_end_point,
                                                     temp_start, temp_end);
        if (distance_pt2seg > tmp_dist) {
          merge_point = iter;
          merge_section = sec;
          distance_pt2seg = tmp_dist;
        }
      }

      if (sec < target_lane_inds.size() - 1) {
        int ind_st = target_lane_inds[sec];
        int ind_ed = target_lane_inds[sec + 1];
        auto temp_start = data_->lane_infos[ind_st].line_points.back();
        auto temp_end = data_->lane_infos[ind_ed].line_points.front();
        double tmp_dist = CalculatePoint2SegmentDist(guide_lane_end_point,
                                                     temp_start, temp_end);
        if (distance_pt2seg > tmp_dist) {
          merge_point = data_->lane_infos[ind_ed].line_points.begin();
          merge_section = sec + 1;
          distance_pt2seg = tmp_dist;
        }
      }
    }
    int target_ego_lane_ind = target_lane_inds[merge_section];
    ego_lane = &data_->lane_infos[target_ego_lane_ind];
    // AINFO << "*******YshapeMerge******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; AINFO << "target ego lane index: "
    // << target_ego_lane_ind; AINFO << "ego lane id: " << ego_lane->id << ",
    // guide lane id: " << guide_lane->id; AINFO << "merge section: " <<
    // merge_section; AINFO << "remaining points: " <<
    // ego_lane->line_points.end() - merge_point; AINFO << "merge point: (" <<
    // merge_point->x << ", " << merge_point->y << ")"; AINFO << "dist:" <<
    // distance_pt2seg; AINFO << "************************";

    if (distance_pt2seg > MaxDistConfine) {
      // AINFO << "TEST: " << distance_square;
      continue;
    }

    if (distance_pt2seg <= MaxDistConfine && !merge_point->scatter_visibility) {
      continue;
    }

    // AINFO << "*******YshapeMerge******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp;
    // 拓扑的end点设置规则，这个较为复杂，要分多种情况
    bool get_seg_end = false;
    cem::message::common::Point2DF seg_end;
    // seg_end为本条车道线的起点
    if (merge_section == 0 &&
        merge_point - ego_lane->line_points.begin() < PtNumValueStartEnd) {
      get_seg_end = true;
      seg_end = ego_lane->line_points.front();
      // seg_start为后继车道的起点
    } else if (merge_section == target_lane_inds.size() - 1 &&
               ego_lane->line_points.end() - merge_point < PtNumValueStartEnd) {
      if (!ego_lane->next_lane_ids.empty()) {
        auto next_lane_id = ego_lane->next_lane_ids.front();
        auto ego_lane_next = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return next_lane_id == lane.id;
            });
        if (ego_lane_next != data_->lane_infos.end()) {
          get_seg_end = true;
          seg_end = ego_lane_next->line_points.back();
        }
      }
      // seg_start为切分后多条车道的中间断点
    } else if (merge_section > 0 &&
               merge_point - ego_lane->line_points.begin() < PtNumValueMid) {
      get_seg_end = true;
      seg_end = ego_lane->line_points.front();
    } else if (merge_section < target_lane_inds.size() - 1 &&
               ego_lane->line_points.end() - merge_point < PtNumValueMid) {
      if (!ego_lane->next_lane_ids.empty()) {
        auto next_lane_id = ego_lane->next_lane_ids.front();
        auto ego_lane_next = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return next_lane_id == lane.id;
            });
        if (ego_lane_next != data_->lane_infos.end()) {
          get_seg_end = true;
          seg_end = ego_lane_next->line_points.back();
        }
      }
    } else {
      get_seg_end = true;
      seg_end = *merge_point;
    }

    if (get_seg_end) {
      auto seg_start = guide_lane_end_point;

      // 生成拓扑连接线段是否与左侧lanemarker相交，找到相交的segment index
      double min_left_lm_dist = std::numeric_limits<double>::max();
      std::vector<int> left_min_st_idx, left_min_ed_idx;
      auto left_lm_id = ego_lane->left_lane_marker_id;
      auto left_lm = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [left_lm_id](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return left_lm_id == lane_marker.id;
          });
      if (left_lm != data_->lanemarkers.end()) {
        if (left_lm->line_points.size() > 2) {
          min_left_lm_dist =
              MinDistSegPolySeg(seg_start, seg_end, left_lm->line_points,
                                left_min_st_idx, left_min_ed_idx);
          // AINFO << "right lm id: " << left_lm->id << ", " <<
          // min_left_lm_dist;
        }
      }

      // 生成拓扑连接线段是否与右侧lanemarker相交，找到相交的segment index
      double min_right_lm_dist = std::numeric_limits<double>::max();
      std::vector<int> right_min_st_idx, right_min_ed_idx;
      auto right_lm_id = ego_lane->right_lane_marker_id;
      auto right_lm = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [right_lm_id](
              const cem::message::sensor::BevLaneMarker& lane_marker) {
            return right_lm_id == lane_marker.id;
          });
      if (right_lm != data_->lanemarkers.end()) {
        if (right_lm->line_points.size() > 2) {
          min_right_lm_dist =
              MinDistSegPolySeg(seg_start, seg_end, right_lm->line_points,
                                right_min_st_idx, right_min_ed_idx);
          // AINFO << "right lm id: " << right_lm->id << ", " <<
          // min_right_lm_dist;
        }
      }

      // 查看对应相交segment的属性，如果有一个是BEV_LMT__SOLID，直接跳过不添加拓扑
      auto start_type =
          cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
      auto end_type =
          cem::message::sensor::BevLaneMarkerType::BEV_LMT__UNDECIDED;
      if (min_left_lm_dist < LMDistThreshold) {
        for (const auto& lm_type : left_lm->type_segs) {
          if (left_min_st_idx.front() >= lm_type.start_index &&
              left_min_st_idx.front() <= lm_type.end_index) {
            start_type = lm_type.type;
            break;
          }
        }

        for (const auto& lm_type : left_lm->type_segs) {
          if (left_min_ed_idx.front() >= lm_type.start_index &&
              left_min_ed_idx.front() <= lm_type.end_index) {
            end_type = lm_type.type;
            break;
          }
        }

        // AINFO << "left lm intersection start: (" <<
        // left_lm->line_points[left_min_st_idx.front()].x << ", " <<
        // left_lm->line_points[left_min_st_idx.front()].y << "): " <<
        // start_type; AINFO << "left lm intersection end: ("<<
        // left_lm->line_points[left_min_ed_idx.front()].x << ", " <<
        // left_lm->line_points[left_min_ed_idx.front()].y << "): " << end_type;
      }

      if (min_right_lm_dist < LMDistThreshold) {
        for (const auto& lm_type : right_lm->type_segs) {
          if (right_min_st_idx.front() >= lm_type.start_index &&
              right_min_st_idx.front() <= lm_type.end_index) {
            start_type = lm_type.type;
            break;
          }
        }

        for (const auto& lm_type : right_lm->type_segs) {
          if (right_min_ed_idx.front() >= lm_type.start_index &&
              right_min_ed_idx.front() <= lm_type.end_index) {
            end_type = lm_type.type;
            break;
          }
        }

        // AINFO << "right lm intersection start: (" <<
        // right_lm->line_points[right_min_st_idx.front()].x << ", " <<
        // right_lm->line_points[right_min_st_idx.front()].y << "): " <<
        // start_type; AINFO << "right lm intersection end: ("<<
        // right_lm->line_points[right_min_ed_idx.front()].x << ", " <<
        // right_lm->line_points[right_min_ed_idx.front()].y << "): " <<
        // end_type;
      }

      // start_type和end_type有一个为实线，不连接拓扑
      if (start_type ==
              cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
          start_type == cem::message::sensor::BevLaneMarkerType::
                            BEV_LMT__DOUBLE_SOLID_SOLID ||
          end_type == cem::message::sensor::BevLaneMarkerType::BEV_LMT__SOLID ||
          end_type == cem::message::sensor::BevLaneMarkerType::
                          BEV_LMT__DOUBLE_SOLID_SOLID) {
        continue;
      }
    }
    // AINFO << "*************************";

    // 判定跳过的条件:
    // 假设处于起始段，打断点相比于起始点前的点数小于PtNumValueStartEnd，将其挂在为ego_lane的前继点。
    if (merge_section == 0 &&
        merge_point - ego_lane->line_points.begin() < PtNumValueStartEnd) {
      if (!ego_lane->previous_lane_ids.empty()) {
        auto previous_lane_id = ego_lane->previous_lane_ids[0];
        auto ego_lane_previous = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [previous_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return previous_lane_id == lane.id;
            });
        if (ego_lane_previous != data_->lane_infos.end()) {
          guide_lane->next_lane_ids = ego_lane_previous->next_lane_ids;
        }
      }

      if (std::find(ego_lane->previous_lane_ids.begin(),
                    ego_lane->previous_lane_ids.end(),
                    guide_lane_id) == ego_lane->previous_lane_ids.end()) {
        ego_lane->previous_lane_ids.push_back(guide_lane_id);
      }

      if (std::find(guide_lane->next_lane_ids.begin(),
                    guide_lane->next_lane_ids.end(),
                    ego_lane->id) == guide_lane->next_lane_ids.end()) {
        guide_lane->next_lane_ids.push_back(ego_lane->id);
      }
      continue;
    }

    // 判定跳过的条件:
    // 假设处于终止段，打断点相比于终止点前的点数小于PtNumValueStartEnd，将其挂载为ego_lane后继的前继点。
    if (merge_section == target_lane_inds.size() - 1 &&
        ego_lane->line_points.end() - merge_point < PtNumValueStartEnd) {
      if (!ego_lane->next_lane_ids.empty()) {
        guide_lane->next_lane_ids = ego_lane->next_lane_ids;
        for (const auto& next_lane_id : ego_lane->next_lane_ids) {
          auto ego_lane_next = std::find_if(
              data_->lane_infos.begin(), data_->lane_infos.end(),
              [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
                return next_lane_id == lane.id;
              });
          if (ego_lane_next == data_->lane_infos.end()) {
            continue;
          }
          ego_lane_next->previous_lane_ids.push_back(guide_lane->id);
        }
      }
      continue;
    }

    // 判定跳过的条件: 假设处于中间段起点或终点
    // 假设与起始点较近，直接将新的支路绑定为前一个sublane的后继，无需打断车道
    if (merge_section > 0 &&
        merge_point - ego_lane->line_points.begin() < PtNumValueMid) {
      if (!ego_lane->previous_lane_ids.empty()) {
        auto previous_lane_id = ego_lane->previous_lane_ids[0];
        auto ego_lane_previous = std::find_if(
            data_->lane_infos.begin(), data_->lane_infos.end(),
            [previous_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
              return previous_lane_id == lane.id;
            });
        if (ego_lane_previous != data_->lane_infos.end()) {
          guide_lane->next_lane_ids = ego_lane_previous->next_lane_ids;
        }
      }

      if (std::find(ego_lane->previous_lane_ids.begin(),
                    ego_lane->previous_lane_ids.end(),
                    guide_lane_id) == ego_lane->previous_lane_ids.end()) {
        ego_lane->previous_lane_ids.push_back(guide_lane_id);
      }

      if (std::find(guide_lane->next_lane_ids.begin(),
                    guide_lane->next_lane_ids.end(),
                    ego_lane->id) == guide_lane->next_lane_ids.end()) {
        guide_lane->next_lane_ids.push_back(ego_lane->id);
      }
      guide_lane->merge_topo_extend =
          cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN;
      continue;
    }
    // 假设与终止点较近，直接将新的支路绑定为前一个sublane的后继，无需打断车道
    if (merge_section < target_lane_inds.size() - 1 &&
        ego_lane->line_points.end() - merge_point < PtNumValueMid) {
      if (!ego_lane->next_lane_ids.empty()) {
        guide_lane->next_lane_ids = ego_lane->next_lane_ids;
        for (const auto& next_lane_id : ego_lane->next_lane_ids) {
          auto ego_lane_next = std::find_if(
              data_->lane_infos.begin(), data_->lane_infos.end(),
              [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
                return next_lane_id == lane.id;
              });
          if (ego_lane_next == data_->lane_infos.end()) {
            continue;
          }
          ego_lane_next->previous_lane_ids.push_back(guide_lane->id);
        }
      }
      guide_lane->merge_topo_extend =
          cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN;
      continue;
    }

    uint64_t new_id = 101;
    for (uint64_t i = 1; i < 100UL; ++i) {
      auto tmp =
          std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                       [i](const cem::message::sensor::BevLaneInfo& lane) {
                         return lane.id == i;
                       });
      if (tmp == data_->lane_infos.end()) {
        new_id = i;
        break;
      }
    }
    auto merged_lane = *ego_lane;
    merged_lane.line_points.clear();
    merged_lane.line_points.insert(merged_lane.line_points.end(), merge_point,
                                   ego_lane->line_points.end());
    merged_lane.number_of_points = merged_lane.line_points.size();
    merged_lane.id = new_id;
    merged_lane.position =
        static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);

    ego_lane->line_points.erase(merge_point, ego_lane->line_points.end());
    ego_lane->number_of_points = ego_lane->line_points.size();
    ego_lane->next_lane_ids.clear();
    ego_lane->next_lane_ids.push_back(new_id);

    guide_lane->next_lane_ids.push_back(merged_lane.id);
    guide_lane->merge_topo_extend =
        cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN;

    merged_lane.previous_lane_ids.clear();
    merged_lane.previous_lane_ids.push_back(ego_lane->id);
    merged_lane.previous_lane_ids.push_back(guide_lane->id);

    // 假如原始ego_lane存在后继，修改后继lane的前继，将其改为打断后的
    for (const auto& next_lane_id : merged_lane.next_lane_ids) {
      auto ego_lane_next = std::find_if(
          data_->lane_infos.begin(), data_->lane_infos.end(),
          [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
            return next_lane_id == lane.id;
          });
      if (ego_lane_next == data_->lane_infos.end()) {
        continue;
      }
      std::replace(ego_lane_next->previous_lane_ids.begin(),
                   ego_lane_next->previous_lane_ids.end(), ego_lane->id,
                   merged_lane.id);
    }

    // AINFO << "*******YshapeMerge******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
    // data_->header.timestamp; for (const auto & lane:
    // data_->lane_infos){
    //   AINFO << "current lane id: " << lane.id;
    //   for (const auto & prev_id: lane.previous_lane_ids){
    //     AINFO << "previous lane id:" << prev_id;
    //   }
    //   for (const auto & next_id: lane.next_lane_ids){
    //     AINFO << "next lane id:" << next_id;
    //   }
    // }
    // AINFO << "************************";

    auto id = ego_lane->left_lane_marker_id;
    auto lane_line = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return id == lane_marker.id;
        });
    if (lane_line != data_->lanemarkers.end()) {
      if (!lane_line->line_points.empty()) {
        auto left_lane_marker_first = *lane_line;
        auto left_lane_marker_second = *lane_line;
        left_lane_marker_second.line_points.clear();

        double distance_square = std::numeric_limits<double>::max();
        std::vector<cem::message::common::Point2DF>::iterator
            marker_merge_point;
        for (auto iter = left_lane_marker_first.line_points.begin();
             iter != left_lane_marker_first.line_points.end(); ++iter) {
          double tmp_square =
              (merge_point->x - iter->x) * (merge_point->x - iter->x) +
              (merge_point->y - iter->y) * (merge_point->y - iter->y);
          if (distance_square > tmp_square) {
            marker_merge_point = iter;
            distance_square = tmp_square;
          }
        }

        if (distance_square < MaxDistSqMarker) {
          left_lane_marker_second.line_points.insert(
              left_lane_marker_second.line_points.end(), marker_merge_point,
              left_lane_marker_first.line_points.end());
          left_lane_marker_second.number_of_points =
              left_lane_marker_second.line_points.size();
          left_lane_marker_second.id = generate_lanemarker_id();
          merged_lane.left_lane_marker_id = left_lane_marker_second.id;
          data_->lanemarkers.push_back(std::move(left_lane_marker_second));

          left_lane_marker_first.line_points.erase(
              marker_merge_point, left_lane_marker_first.line_points.end());
          left_lane_marker_first.number_of_points =
              left_lane_marker_first.line_points.size();
          left_lane_marker_first.id = generate_lanemarker_id();
          ego_lane->left_lane_marker_id = left_lane_marker_first.id;
          data_->lanemarkers.push_back(std::move(left_lane_marker_first));
        }
      }
    }

    id = ego_lane->right_lane_marker_id;
    lane_line = std::find_if(
        data_->lanemarkers.begin(), data_->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
          return id == lane_marker.id;
        });
    if (lane_line != data_->lanemarkers.end()) {
      if (!lane_line->line_points.empty()) {
        auto right_lane_marker_first = *lane_line;
        auto right_lane_marker_second = *lane_line;
        right_lane_marker_second.line_points.clear();

        double distance_square = std::numeric_limits<double>::max();
        std::vector<cem::message::common::Point2DF>::iterator
            marker_merge_point;
        for (auto iter = right_lane_marker_first.line_points.begin();
             iter != right_lane_marker_first.line_points.end(); ++iter) {
          double tmp_square =
              (merge_point->x - iter->x) * (merge_point->x - iter->x) +
              (merge_point->y - iter->y) * (merge_point->y - iter->y);
          if (distance_square > tmp_square) {
            marker_merge_point = iter;
            distance_square = tmp_square;
          }
        }

        if (distance_square < MaxDistSqMarker) {
          right_lane_marker_second.line_points.insert(
              right_lane_marker_second.line_points.end(), marker_merge_point,
              right_lane_marker_first.line_points.end());
          right_lane_marker_second.number_of_points =
              right_lane_marker_second.line_points.size();
          right_lane_marker_second.id = generate_lanemarker_id();
          merged_lane.right_lane_marker_id = right_lane_marker_second.id;
          data_->lanemarkers.push_back(std::move(right_lane_marker_second));

          right_lane_marker_first.line_points.erase(
              marker_merge_point, right_lane_marker_first.line_points.end());
          right_lane_marker_first.number_of_points =
              right_lane_marker_first.line_points.size();
          right_lane_marker_first.id = generate_lanemarker_id();
          ego_lane->right_lane_marker_id = right_lane_marker_first.id;
          data_->lanemarkers.push_back(std::move(right_lane_marker_first));
        }
      }
    }

    data_->lanemarkers_num = data_->lanemarkers.size();
    data_->lane_infos.push_back(std::move(merged_lane));

    target_lane_inds.insert(target_lane_inds.begin() + (merge_section + 1),
                            data_->lane_infos.size() - 1);
  }

  // AINFO << "*******YshapeMerge******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (int idx : target_lane_inds){
  //   ego_lane = &data_->lane_infos[idx];
  //   AINFO << "ego lane id: " << ego_lane->id;
  //   AINFO << "ego lane start: (" << ego_lane->line_points.front().x << ", "
  //   << ego_lane->line_points.front().y << ")"; AINFO << "ego lane end: (" <<
  //   ego_lane->line_points.back().x << ", " << ego_lane->line_points.back().y
  //   << ")";
  // }
  // AINFO << "************************";

  // AINFO << "*******YshapeMerge******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) <<
  // data_->header.timestamp; for (const auto lane :
  // data_->lane_infos){
  //   AINFO << "lane id: " << lane.id;
  //   for (const auto prev_lane_id : lane.previous_lane_ids){
  //     AINFO << "previous lane id: " << prev_lane_id;
  //   }
  //   for (const auto next_lane_id : lane.next_lane_ids){
  //     AINFO << "next lane id: " << next_lane_id;
  //   }
  // }
  // AINFO << "************************";
}

void BevMapProcessor::FindNeighbourLanes() {
  constexpr double EgoXMinGuideLane =
      -20.0;  // 目标添加临近车道纵向最接近原点x的最小值（ego坐标系）
  constexpr double EgoXmaxGuideLane =
      20.0;  // 目标添加临近车道纵向最接近原点x的最大值（ego坐标系）

  int left_neighbour_lane_idx = -1;
  int right_neighbour_lane_idx = -1;
  double min_positive_val = std::numeric_limits<double>::max();
  double max_negative_val = std::numeric_limits<double>::lowest();
  auto ego_lane_start =
      data_->lane_infos[ego_lane_inds_.front()].line_points.front();
  auto ego_lane_end =
      data_->lane_infos[ego_lane_inds_.back()].line_points.back();
  Eigen::Vector2d dir_ego_lane, ego_lane_mid;
  dir_ego_lane.x() = ego_lane_end.x - ego_lane_start.x;
  dir_ego_lane.y() = ego_lane_end.y - ego_lane_start.y;
  dir_ego_lane = dir_ego_lane / dir_ego_lane.norm();
  ego_lane_mid.x() = (ego_lane_start.x + ego_lane_end.x) / 2;
  ego_lane_mid.y() = (ego_lane_start.y + ego_lane_end.y) / 2;

  // 遍历guide_lane，找到左邻和右邻
  for (const auto& guide_lane_idx : guide_lane_inds_) {
    if (guide_lane_idx >= data_->lane_infos.size()) {
      continue;
    }

    auto guide_lane = &data_->lane_infos[guide_lane_idx];
    auto guide_lane_id = guide_lane->id;

    if (guide_lane->line_points.size() < 4) {
      continue;
    }

    // 遍历ego_lane_inds_，确认guide_lane与ego_lane没有拓扑关系
    bool ego_topo_flag = false;
    for (const auto& ego_lane_idx : ego_lane_inds_) {
      auto ego_lane = &data_->lane_infos[ego_lane_idx];

      auto iter = std::find(ego_lane->previous_lane_ids.begin(),
                            ego_lane->previous_lane_ids.end(), guide_lane_id);
      if (iter != ego_lane->previous_lane_ids.end()) {
        ego_topo_flag = true;
        break;
      }

      iter = std::find(ego_lane->next_lane_ids.begin(),
                       ego_lane->next_lane_ids.end(), guide_lane_id);
      if (iter != ego_lane->next_lane_ids.end()) {
        ego_topo_flag = true;
        break;
      }
    }

    if (ego_topo_flag) {
      continue;
    }

    // 根据guide_lane的中点与ego_lane中点y方向distance的值计算左右车道关系
    Eigen::Vector2d guide_lane_mid;
    guide_lane_mid.x() =
        (guide_lane->line_points.front().x + guide_lane->line_points.back().x) /
        2;
    guide_lane_mid.y() =
        (guide_lane->line_points.front().y + guide_lane->line_points.back().y) /
        2;

    // x方向基于guide_lane的start和end的x值设置阈值，不在窗口内的对应lane进行过滤
    if (guide_lane->line_points.front().x > EgoXmaxGuideLane ||
        guide_lane->line_points.back().x < EgoXMinGuideLane) {
      continue;
    }

    auto dist_vec = guide_lane_mid - ego_lane_mid;
    double dist_lat_sign = copysign(
        1.0, dir_ego_lane.x() * dist_vec.y() - dir_ego_lane.y() * dist_vec.x());
    double dist_lat_val =
        dist_lat_sign *
        (dist_vec - dist_vec.dot(dir_ego_lane) * dir_ego_lane).norm();
    if (dist_lat_val > 0 && dist_lat_val < min_positive_val) {
      min_positive_val = dist_lat_val;
      left_neighbour_lane_idx = guide_lane_idx;
    }

    if (dist_lat_val < 0 && dist_lat_val > max_negative_val) {
      max_negative_val = dist_lat_val;
      right_neighbour_lane_idx = guide_lane_idx;
    }
  }

  if (left_neighbour_lane_idx >= 0) {
    neighbour_lane_inds_map_[min_positive_val].push_back(
        left_neighbour_lane_idx);
  }

  if (right_neighbour_lane_idx >= 0) {
    neighbour_lane_inds_map_[max_negative_val].push_back(
        right_neighbour_lane_idx);
  }
}

void BevMapProcessor::SetCrossParam(bool cross_road_status, double cross_road_distance, bool is_turn_right) {
  cross_road_status_   = cross_road_status;
  cross_road_distance_ = cross_road_distance;
  is_turn_right_       = is_turn_right;
}

void BevMapProcessor::SetAdc2Junction(double adc_to_junction) {
  adc_to_junction_ = adc_to_junction;
}

void BevMapProcessor::SetRightTurnToLane(uint64_t id) {
  if (lane_map_.count(id) == 0) {
    return;
  }
  auto bev_lane = lane_map_.at(id);
  bev_lane->navi_action = cem::message::sensor::BevAction::RIGHT_TURN;
  bev_lane->bev_turn_type = cem::message::sensor::BevAction::RIGHT_TURN;
}

bool BevMapProcessor::BreakLaneSplit(uint64_t target_lane_id, uint64_t topo_lane_id, std::shared_ptr<cem::message::env_model::RoutingMap> ld_map, const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos, LdMapProcessor::MergeSplitType match_type, LdMapProcessor::MergeSplitType target_lane_type, LdMapProcessor::MergeSplitType topo_lane_type, double distance_to_ego, json &topo_debug_info){
  constexpr double MaxDistSqMarker = 10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MoveDistanceThreshold = 8.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinEndThreshold = 20.0;  // 目标断点到终点的距离阈值
  constexpr double MinDistThreshold = 4.0;  // 目标添加拓扑点的最小距离（车道标志线）
  
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  bool break_success_flag = false;
  // 根据target_lane_id
  auto target_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [target_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id == lane.id; });

  auto topo_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                [topo_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return topo_lane_id == lane.id; });

  if (target_lane == data_->lane_infos.end()){
    return break_success_flag;
  }

  if (topo_lane == data_->lane_infos.end()){
    return break_success_flag;
  }

  if (target_lane->line_points.size() < 4) {
    return break_success_flag;
  }

  // std::vector<cem::message::common::Point2DF>::iterator split_point;
  // bool get_split_point = false; 
  // // 打断备选规则1：通过地图查询的拓扑点距离打断
  // for (auto iter = target_lane->line_points.begin();
  //           iter != target_lane->line_points.end(); ++iter) {
  //   auto temp_point = *iter;
  //   // 确保拓扑点符号与distance_to_ego一致
  //   if (temp_point.x * distance_to_ego > 0) {
  //     double temp_dist = copysign(1.0, distance_to_ego) * std::sqrt(std::pow(temp_point.x, 2) + std::pow(temp_point.y, 2));
  //     if (temp_dist > distance_to_ego) {
  //       get_split_point = true;
  //       split_point = iter;
  //       break;
  //     }
  //   }
  // }

  // if (get_split_point) {
  //   AINFO << "*******SplitPointResult******";
  //   AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  //   AINFO << "sequence number: " << data_->header.cycle_counter;
  //   AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
  //   AINFO << "*******************************";
  // }

  std::vector<cem::message::common::Point2DF>::iterator split_point1;
  double distance_pt2seg = std::numeric_limits<double>::max();
  // 打断备选规则2：找到距离最近点，向前挪动8m（暂定），如果打断点数不够就找最远的点
  for (auto iter = target_lane->line_points.begin() + 1;
            iter != target_lane->line_points.end(); ++iter) {
    // 求与输入的车道line_segment悬空车道起点的最近点
    auto temp_start = *(iter - 1);
    auto temp_end = *iter;
    double tmp_dist = CalculatePoint2SegmentDist(topo_lane->line_points.front(), temp_start, temp_end);
    if (distance_pt2seg > tmp_dist) {
      split_point1 = iter;
      distance_pt2seg = tmp_dist;
    }
  }

  // AINFO << "distance_pt2seg: " << distance_pt2seg;
  if (distance_pt2seg > MinDistThreshold) {
    topo_debug_info["sub lane start point constaint"] = false;
    return break_success_flag;
  }

  // 校验结果split_point1是否可以分出足够数量的行点
  int break_lane1_pts_num = split_point1 - target_lane->line_points.begin();
  int break_lane2_pts_num = target_lane->line_points.end() - split_point1;
  //  AINFO << "split_point1 constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_success_flag;
  }

  // 基于当前的split_point前移8m
  std::vector<cem::message::common::Point2DF>::iterator split_point = split_point1;
  for (auto iter = split_point1;
            iter != target_lane->line_points.begin() + 2; --iter) {
    // 求与输入的车道line_segment与代表悬空车道的line是否相交，找到第一个相交片段
    auto temp_point = *iter;
    double tmp_dist = CalculatePoint2PointDist(*split_point1, temp_point);
    split_point = iter;
    if (tmp_dist > MoveDistanceThreshold) {
      break;
    }
  }

  // AINFO << "*******SplitPointResultModify******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  // AINFO << "sequence number: " << data_->header.cycle_counter;
  // AINFO << "split point: (" << split_point1->x << ", " << split_point1->y << ")";
  // AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
  // AINFO << "*******************************";

  // if (!get_split_point){
  //   return break_success_flag;
  // }

  // 匹配拓扑点和实际拓扑点纵向距离限制条件，根据量产阈值设计为40m
  if(!CheckTopoPointLongitudeDistanceConstraint(*split_point, distance_to_ego, ld_map, match_infos)) {
    topo_debug_info["topo point longitude constraint"] = false;
    return break_success_flag;
  }

  // 校验结果split_point是否可以分出足够数量的行点
  break_lane1_pts_num = split_point - target_lane->line_points.begin();
  break_lane2_pts_num = target_lane->line_points.end() - split_point;
  // AINFO << "split_point constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_success_flag;
  }

  // 打断产生的距离超过阈值约束
  double end_point_val = 0;
  for (auto iter = split_point;
            iter < target_lane->line_points.end() - 1; ++iter) {
    auto temp_point1 = *iter;
    auto temp_point2 = *(iter+1);
    end_point_val += CalculatePoint2PointDist(temp_point1, temp_point2);
  }
  if (end_point_val < MinEndThreshold){
    // AINFO << "break_point-end constraint: " << end_point_val;
    topo_debug_info["break_point-end constraint"] = false;
    return break_success_flag;
  }

  // 校验结果split_point是否出现回折
  auto temp_lane_1_start = topo_lane->line_points.front();
  auto main_lane_end = *(split_point - 1);
  auto main_lane_second_end = *(split_point - 2);
  Eigen::Vector2d main_lane_vec, topo_vec;
  main_lane_vec.x() = main_lane_end.x - main_lane_second_end.x;
  main_lane_vec.y() = main_lane_end.y - main_lane_second_end.y;
  topo_vec.x() = temp_lane_1_start.x - main_lane_end.x;
  topo_vec.y() = temp_lane_1_start.y - main_lane_end.y;
  double start_end_val = topo_vec.dot(main_lane_vec) / main_lane_vec.norm();
# ifdef AddTopoDebug
  AINFO << "main_lane_vec: " << main_lane_vec;
  AINFO << "topo_vec: " << topo_vec;
  AINFO << "start_end_val: " << start_end_val;
# endif
  
  if (start_end_val < MinBackThreshold){
    topo_debug_info["turn back constraint"] = false;
    return break_success_flag;
  }

  // 实际拓扑lane的位置和地图匹配位置校验
  topo_debug_info["split position constraint"] = true;
  ptrdiff_t offset = split_point - target_lane->line_points.begin();
  auto &origin_lane_geos = *(target_lane->geos);
  auto split_point_geos = origin_lane_geos.begin() + offset;
  auto main_lane_geos = std::make_shared<std::vector<Eigen::Vector2f>>(origin_lane_geos.begin(), split_point_geos);
  auto temp_lane1_geos = std::make_shared<std::vector<Eigen::Vector2f>>(split_point_geos, origin_lane_geos.end());
  auto topo_lane_geos = topo_lane->geos;
  if (!CheckSplitLanePositionConstraint(match_type, target_lane_type, topo_lane_type, *temp_lane1_geos, *topo_lane_geos)){
    topo_debug_info["split position constraint"] = false;
    return break_success_flag;
  }

  // 打断车道中心线，改变中心线拓扑关系
  uint64_t new_id = 101;
  for (uint64_t i = 1; i < 100UL; ++i) {
    auto tmp =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                      [i](const cem::message::sensor::BevLaneInfo& lane) {
                        return lane.id == i;
                      });
    if (tmp == data_->lane_infos.end()) {
      new_id = i;
      break;
    }
  }
  auto breaked_lane = *target_lane;
  breaked_lane.line_points.clear();
  breaked_lane.line_points.insert(breaked_lane.line_points.end(), split_point,
                                  target_lane->line_points.end());
  breaked_lane.number_of_points = breaked_lane.line_points.size();
  breaked_lane.id = new_id;

  target_lane->line_points.erase(split_point, target_lane->line_points.end());
  target_lane->number_of_points = target_lane->line_points.size();
  target_lane->geos = main_lane_geos;
  target_lane->indexed_geos.BuildIndex(target_lane->geos);
  target_lane->next_lane_ids.clear();
  target_lane->next_lane_ids.push_back(new_id);

  breaked_lane.previous_lane_ids.clear();
  breaked_lane.previous_lane_ids.push_back(target_lane->id);
  breaked_lane.geos = temp_lane1_geos;
  breaked_lane.indexed_geos.BuildIndex(target_lane->geos);

  // AINFO << "breaked line start x: " << breaked_lane.line_points.front().x;
  if (breaked_lane.line_points.front().x < 0) {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
  } 
  else {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
  }

  // 假如原始target_lane存在后继，修改后继lane的前继，将其改为打断后的
  for (const auto& next_lane_id : breaked_lane.next_lane_ids) {
    auto target_lane_next = std::find_if(
        data_->lane_infos.begin(), data_->lane_infos.end(),
        [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
          return next_lane_id == lane.id;
        });
    if (target_lane_next == data_->lane_infos.end()) {
      continue;
    }
    std::replace(target_lane_next->previous_lane_ids.begin(),
                 target_lane_next->previous_lane_ids.end(), target_lane->id,
                 breaked_lane.id);
  }

  // 打断 lane_marker
  // 打断左边lane_marker
  auto id = target_lane->left_lane_marker_id;
  auto lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto left_lane_marker_first = *lane_line;
      auto left_lane_marker_second = *lane_line;
      left_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_split_point;
      for (auto iter = left_lane_marker_first.line_points.begin();
            iter != left_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (split_point->x - iter->x) * (split_point->x - iter->x) +
            (split_point->y - iter->y) * (split_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_split_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        left_lane_marker_second.line_points.insert(
            left_lane_marker_second.line_points.end(), marker_split_point,
            left_lane_marker_first.line_points.end());
        left_lane_marker_second.number_of_points =
            left_lane_marker_second.line_points.size();
        left_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.left_lane_marker_id = left_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_second));

        left_lane_marker_first.line_points.erase(
            marker_split_point, left_lane_marker_first.line_points.end());
        left_lane_marker_first.number_of_points =
            left_lane_marker_first.line_points.size();
        left_lane_marker_first.id = generate_lanemarker_id();
        target_lane->left_lane_marker_id = left_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_first));
      }
    }
  }

  // 打断左边右侧lane_marker
  id = target_lane->right_lane_marker_id;
  lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto right_lane_marker_first = *lane_line;
      auto right_lane_marker_second = *lane_line;
      right_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_split_point;
      for (auto iter = right_lane_marker_first.line_points.begin();
            iter != right_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (split_point->x - iter->x) * (split_point->x - iter->x) +
            (split_point->y - iter->y) * (split_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_split_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        right_lane_marker_second.line_points.insert(
            right_lane_marker_second.line_points.end(), marker_split_point,
            right_lane_marker_first.line_points.end());
        right_lane_marker_second.number_of_points =
            right_lane_marker_second.line_points.size();
        right_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.right_lane_marker_id = right_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_second));

        right_lane_marker_first.line_points.erase(
            marker_split_point, right_lane_marker_first.line_points.end());
        right_lane_marker_first.number_of_points =
            right_lane_marker_first.line_points.size();
        right_lane_marker_first.id = generate_lanemarker_id();
        target_lane->right_lane_marker_id = right_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_first));
      }
    }
  }

  data_->lanemarkers_num = data_->lanemarkers.size();
  data_->lane_infos.push_back(std::move(breaked_lane));
  break_success_flag = true;

  return break_success_flag;
}

bool BevMapProcessor::BreakLaneMerge(uint64_t target_lane_id, uint64_t topo_lane_id, std::shared_ptr<cem::message::env_model::RoutingMap> ld_map, const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos, LdMapProcessor::MergeSplitType match_type, LdMapProcessor::MergeSplitType target_lane_type, LdMapProcessor::MergeSplitType topo_lane_type, double distance_to_ego, json &topo_debug_info){
  constexpr double MaxDistSqMarker = 10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MoveDistanceThreshold = 8.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinEndThreshold = 12.0;  // 目标断点到终点的距离阈值
  
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  bool break_success_flag = false;
  // 根据target_lane_id
  auto target_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                  [target_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id == lane.id; });

  auto topo_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                [topo_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return topo_lane_id == lane.id; });

  if (target_lane == data_->lane_infos.end()){
    return break_success_flag;
  }

  if (topo_lane == data_->lane_infos.end()){
    return break_success_flag;
  }

  if (target_lane->line_points.size() < 4) {
    return break_success_flag;
  }

  // std::vector<cem::message::common::Point2DF>::iterator split_point;
  // bool get_split_point = false; 
  // // 打断备选规则1：通过地图查询的拓扑点距离打断
  // for (auto iter = target_lane->line_points.begin();
  //           iter != target_lane->line_points.end(); ++iter) {
  //   auto temp_point = *iter;
  //   // 确保拓扑点符号与distance_to_ego一致
  //   if (temp_point.x * distance_to_ego > 0) {
  //     double temp_dist = copysign(1.0, distance_to_ego) * std::sqrt(std::pow(temp_point.x, 2) + std::pow(temp_point.y, 2));
  //     if (temp_dist > distance_to_ego) {
  //       get_split_point = true;
  //       split_point = iter;
  //       break;
  //     }
  //   }
  // }

  // if (get_split_point) {
  //   AINFO << "*******SplitPointResult******";
  //   AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  //   AINFO << "sequence number: " << data_->header.cycle_counter;
  //   AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
  //   AINFO << "*******************************";
  // }

  std::vector<cem::message::common::Point2DF>::iterator merge_point1;
  double distance_pt2seg = std::numeric_limits<double>::max();
  // 打断备选规则2：找到距离最近点，向前挪动8m（暂定），如果打断点数不够就找最远的点
  for (auto iter = target_lane->line_points.begin() + 1;
            iter != target_lane->line_points.end(); ++iter) {
    // 求与输入的车道line_segment悬空车道起点的最近点
    auto temp_start = *(iter - 1);
    auto temp_end = *iter;
    double tmp_dist = CalculatePoint2SegmentDist(topo_lane->line_points.back(), temp_start, temp_end);
    if (distance_pt2seg > tmp_dist) {
      merge_point1 = iter;
      distance_pt2seg = tmp_dist;
    }
  }

  // 校验结果split_point1是否可以分出足够数量的行点
  int break_lane1_pts_num = merge_point1 - target_lane->line_points.begin();
  int break_lane2_pts_num = target_lane->line_points.end() - merge_point1;
  // AINFO << "split_point1 constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_success_flag;
  }

  // 基于当前的split_point前移8m
  std::vector<cem::message::common::Point2DF>::iterator merge_point = merge_point1;
  for (auto iter = merge_point1;
            iter != target_lane->line_points.end() - 2; ++iter) {
    auto temp_point = *iter;
    double tmp_dist = CalculatePoint2PointDist(*merge_point1, temp_point);
    merge_point = iter;
    if (tmp_dist > MoveDistanceThreshold) {
      break;
    }
  }

  // AINFO << "*******MergePointResultModify******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  // AINFO << "sequence number: " << data_->header.cycle_counter;
  // AINFO << "merge point: (" << merge_point1->x << ", " << merge_point1->y << ")";
  // AINFO << "merge point: (" << merge_point->x << ", " << merge_point->y << ")";
  // AINFO << "*******************************";

  // if (!get_split_point){
  //   return break_success_flag;
  // }

  // 校验结果merge_point是否可以分出足够数量的行点
  break_lane1_pts_num = merge_point - target_lane->line_points.begin();
  break_lane2_pts_num = target_lane->line_points.end() - merge_point;
  // AINFO << "merge_point constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_success_flag;
  }

  // 打断产生的距离超过阈值约束
  double end_point_val = 0;
  for (auto iter = merge_point;
            iter < target_lane->line_points.end() - 1; ++iter) {
    auto temp_point1 = *iter;
    auto temp_point2 = *(iter+1);
    end_point_val += CalculatePoint2PointDist(temp_point1, temp_point2);
  }
  if (end_point_val < MinEndThreshold){
    // AINFO << "break lane: " << target_lane->id << "; topo lane: " << topo_lane->id;
    // AINFO << "break_point-end constraint: " << end_point_val;
    topo_debug_info["break_point-end constraint"] = false;
    return break_success_flag;
  }

  // 匹配拓扑点和实际拓扑点纵向距离限制条件，根据量产阈值设计为40m
  if(!CheckTopoPointLongitudeDistanceConstraint(*merge_point, distance_to_ego, ld_map, match_infos)) {
    topo_debug_info["topo point longitude constraint"] = false;
    return break_success_flag;
  }

  // 校验结果merge_point是否出现回折
  auto temp_lane_1_end = topo_lane->line_points.front();
  auto main_lane_start = *(merge_point);
  auto main_lane_second_start = *(merge_point + 1);
  Eigen::Vector2d main_lane_vec, topo_vec;
  main_lane_vec.x() = main_lane_second_start.x - main_lane_start.x;
  main_lane_vec.y() = main_lane_second_start.y - main_lane_start.y;
  topo_vec.x() = main_lane_start.x - temp_lane_1_end.x;
  topo_vec.y() = main_lane_start.y - temp_lane_1_end.y;
  double end_start_val = topo_vec.dot(main_lane_vec) / main_lane_vec.norm();
# ifdef AddTopoDebug
  AINFO << "main_lane_vec: " << main_lane_vec;
  AINFO << "topo_vec: " << topo_vec;
  // AINFO << "start_end_val: " << start_end_val;
# endif
  
  if (end_start_val < MinBackThreshold){
    topo_debug_info["turn back constraint"] = false;
    return break_success_flag;
  }

  // 实际拓扑lane的位置和地图匹配位置校验
  topo_debug_info["merge position constraint"] = true;
  ptrdiff_t offset = merge_point - target_lane->line_points.begin();
  auto &origin_lane_geos = *(target_lane->geos);
  auto merge_point_geos = origin_lane_geos.begin() + offset;
  auto temp_lane1_geos = std::make_shared<std::vector<Eigen::Vector2f>>(origin_lane_geos.begin(), merge_point_geos);
  auto main_lane_geos = std::make_shared<std::vector<Eigen::Vector2f>>(merge_point_geos, origin_lane_geos.end());
  auto topo_lane_geos = topo_lane->geos;
  if (!CheckMergeLanePositionConstraint(match_type, target_lane_type, topo_lane_type, *temp_lane1_geos, *topo_lane_geos)){
    topo_debug_info["merge position constraint"] = false;
    return break_success_flag;
  }

  // 打断车道中心线，改变中心线拓扑关系
  uint64_t new_id = 101;
  for (uint64_t i = 1; i < 100UL; ++i) {
    auto tmp =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                      [i](const cem::message::sensor::BevLaneInfo& lane) {
                        return lane.id == i;
                      });
    if (tmp == data_->lane_infos.end()) {
      new_id = i;
      break;
    }
  }
  auto breaked_lane = *target_lane;
  breaked_lane.line_points.clear();
  breaked_lane.line_points.insert(breaked_lane.line_points.end(), merge_point,
                                  target_lane->line_points.end());
  breaked_lane.number_of_points = breaked_lane.line_points.size();
  breaked_lane.id = new_id;

  target_lane->line_points.erase(merge_point, target_lane->line_points.end());
  target_lane->number_of_points = target_lane->line_points.size();
  target_lane->geos = temp_lane1_geos;
  target_lane->indexed_geos.BuildIndex(target_lane->geos);
  target_lane->next_lane_ids.clear();
  target_lane->next_lane_ids.push_back(new_id);

  breaked_lane.previous_lane_ids.clear();
  breaked_lane.previous_lane_ids.push_back(target_lane->id);
  breaked_lane.geos = main_lane_geos;
  breaked_lane.indexed_geos.BuildIndex(target_lane->geos);

  if (breaked_lane.line_points.front().x < 0) {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
  } 
  else {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
  }

  // 假如原始target_lane存在后继，修改后继lane的前继，将其改为打断后的
  for (const auto& next_lane_id : breaked_lane.next_lane_ids) {
    auto target_lane_next = std::find_if(
        data_->lane_infos.begin(), data_->lane_infos.end(),
        [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
          return next_lane_id == lane.id;
        });
    if (target_lane_next == data_->lane_infos.end()) {
      continue;
    }
    std::replace(target_lane_next->previous_lane_ids.begin(),
                 target_lane_next->previous_lane_ids.end(), target_lane->id,
                 breaked_lane.id);
  }

  // 打断 lane_marker
  // 打断左边lane_marker
  auto id = target_lane->left_lane_marker_id;
  auto lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto left_lane_marker_first = *lane_line;
      auto left_lane_marker_second = *lane_line;
      left_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_merge_point;
      for (auto iter = left_lane_marker_first.line_points.begin();
            iter != left_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (merge_point->x - iter->x) * (merge_point->x - iter->x) +
            (merge_point->y - iter->y) * (merge_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_merge_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        left_lane_marker_second.line_points.insert(
            left_lane_marker_second.line_points.end(), marker_merge_point,
            left_lane_marker_first.line_points.end());
        left_lane_marker_second.number_of_points =
            left_lane_marker_second.line_points.size();
        left_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.left_lane_marker_id = left_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_second));

        left_lane_marker_first.line_points.erase(
            marker_merge_point, left_lane_marker_first.line_points.end());
        left_lane_marker_first.number_of_points =
            left_lane_marker_first.line_points.size();
        left_lane_marker_first.id = generate_lanemarker_id();
        target_lane->left_lane_marker_id = left_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_first));
      }
    }
  }

  // 打断左边右侧lane_marker
  id = target_lane->right_lane_marker_id;
  lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto right_lane_marker_first = *lane_line;
      auto right_lane_marker_second = *lane_line;
      right_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_merge_point;
      for (auto iter = right_lane_marker_first.line_points.begin();
            iter != right_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (merge_point->x - iter->x) * (merge_point->x - iter->x) +
            (merge_point->y - iter->y) * (merge_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_merge_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        right_lane_marker_second.line_points.insert(
            right_lane_marker_second.line_points.end(), marker_merge_point,
            right_lane_marker_first.line_points.end());
        right_lane_marker_second.number_of_points =
            right_lane_marker_second.line_points.size();
        right_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.right_lane_marker_id = right_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_second));

        right_lane_marker_first.line_points.erase(
            marker_merge_point, right_lane_marker_first.line_points.end());
        right_lane_marker_first.number_of_points =
            right_lane_marker_first.line_points.size();
        right_lane_marker_first.id = generate_lanemarker_id();
        target_lane->right_lane_marker_id = right_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_first));
      }
    }
  }

  data_->lanemarkers_num = data_->lanemarkers.size();
  data_->lane_infos.push_back(std::move(breaked_lane));
  break_success_flag = true; 

  return break_success_flag;
}

std::optional<BevMapProcessor::BreakLaneResult> BevMapProcessor::AddSplitTopoByMapInfo(std::optional<const BevSplitInfo*> split_info_result,
                                            cem::message::sensor::BevLaneInfo* main_lane,
                                            const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                            const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                                            const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                                            const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                                            json &topo_debug_info) {
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinTopoEndThreshold = 20.0;  // 目标添加拓扑车道线起点回折最小距离

  std::optional<BreakLaneResult> break_lane_ids;
  
  topo_debug_info["add split topo"] = false;
  topo_debug_info["query id"] = main_lane->id;
  json target_sub_lane_array = json::array();

  if (main_lane -> line_points.size() < 2) {
    topo_debug_info["target_sub_lane_result"] = target_sub_lane_array;
    return break_lane_ids;
  }

  if (!split_info_result.has_value()) {
    return break_lane_ids;
  }

  const BevSplitInfo* split_info = split_info_result.value();
  LdMapProcessor::MergeSplitType topo_type = split_info->match_type;
  topo_debug_info["topo_point type"] = static_cast<int>(topo_type);
  topo_debug_info["mainlane id"] = split_info->bev_lane_from;
  topo_debug_info["distance_to_ego"] = split_info->distance_to_ego;

# ifdef AddTopoDebug
  AINFO << "*******SplitTopoQuery******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "main lane id: " << main_lane->id
        << "; start lane id: " << split_info->bev_lane_from
        << "; distance_to_ego: " << split_info->distance_to_ego
        << "; Topo point type: " << static_cast<int>(topo_type);

  std::stringstream lane_id_str;
  for (const auto& lane_id : split_info->map_lanes_from){
    lane_id_str << lane_id << ", ";
  }
  AINFO << "main_ld_map id: " << lane_id_str.str();
  
  for (const auto& pair: split_info->bev_target_to) {
    AINFO << "key1: " << pair.first 
          << "; end lane id: " << pair.second.bev_lane_to
          << "; Topo lane type: " << static_cast<int>(pair.second.type);

    std::stringstream lane_id_str;
    for (const auto& lane_id : pair.second.map_lanes_to){
      lane_id_str << lane_id << ", ";
    }
    AINFO << "sub_ld_map id: " << lane_id_str.str();
  }
  AINFO << "************************";
# endif

  for (const auto& pair: split_info->bev_target_to) {
    json sub_lane_info;
    sub_lane_info["id"] = pair.second.bev_lane_to;
    sub_lane_info["topo_lane type"] = static_cast<int>(pair.second.type);
    target_sub_lane_array.push_back(sub_lane_info);
  }
  topo_debug_info["target_sub_lane_result"] = target_sub_lane_array;

  // 计算目标打断线到end的距离
  double dist_ego2End = GetEgoToEndLength(main_lane);

  // bev_target_to: 当前匹配得到的后继，目前规则只处理一分二的情况，后期可扩展到一分三等情况，暂时不更改target_lane有后继的情况
  if (split_info->bev_target_to.size() == 2){
    auto main_lane_id = main_lane->id;
    // 主车道与匹配结果横向距离约束
    topo_debug_info["mainlane match constraint"] = true;
    topo_debug_info["turn back constraint"] = true;

    auto target_lane_id1 = split_info->bev_target_to[0].second.bev_lane_to;
    auto target_lane_id2 = split_info->bev_target_to[1].second.bev_lane_to;

    auto target_lane_type1 = split_info->bev_target_to[0].second.type;
    auto target_lane_type2 = split_info->bev_target_to[1].second.type;

    auto target_topo_distance = split_info->distance_to_ego;

    auto mainlane_ld_match = split_info->map_lanes_from;
    auto target_lane1_ld_match = split_info->bev_target_to[0].second.map_lanes_to;
    auto target_lane2_ld_match = split_info->bev_target_to[1].second.map_lanes_to;

    // 不需要打断添加拓扑规则
    if (main_lane->next_lane_ids.empty() && main_lane_id != target_lane_id1 && main_lane_id != target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane1_ld_match, false, true)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      // 找到target_lane1，target_lane2
      auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });

      if (temp_lane_1 != data_->lane_infos.end() && temp_lane_2 != data_->lane_infos.end() &&
          temp_lane_1->line_points.size() >= 2 && temp_lane_1->line_points.size() >= 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckSplitLaneRelationConstraint(*main_lane, {*temp_lane_1, *temp_lane_2}, false, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, true, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        /// target_lane2
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_2), ld_map, target_lane2_ld_match, mainlane_ld_match, true, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 匹配拓扑点和实际拓扑点纵向距离限制条件，根据量产阈值设计为40m
        if(!CheckTopoPointLongitudeDistanceConstraint(main_lane->line_points.back(), target_topo_distance, ld_map, split_match_infos)) {
          topo_debug_info["topo point longitude constraint"] = false;
          return break_lane_ids;
        }

        // 判断是否存在回折，存在回折不添加拓扑
        auto temp_lane_1_start = temp_lane_1->line_points.front();
        auto temp_lane_2_start = temp_lane_2->line_points.front();
        auto main_lane_end = main_lane->line_points.back();
        auto main_lane_second_end = main_lane->line_points[main_lane->line_points.size() - 2];
        Eigen::Vector2d main_lane_vec, topo_vec1, topo_vec2;
        main_lane_vec.x() = main_lane_end.x - main_lane_second_end.x;
        main_lane_vec.y() = main_lane_end.y - main_lane_second_end.y;
        topo_vec1.x() = temp_lane_1_start.x - main_lane_end.x;
        topo_vec1.y() = temp_lane_1_start.y - main_lane_end.y;
        topo_vec2.x() = temp_lane_2_start.x - main_lane_end.x;
        topo_vec2.y() = temp_lane_2_start.y - main_lane_end.y;
        double start_end_val1 = topo_vec1.dot(main_lane_vec) / main_lane_vec.norm();
        double start_end_val2 = topo_vec2.dot(main_lane_vec) / main_lane_vec.norm();
        
        if (start_end_val1 < MinBackThreshold || start_end_val2 < MinBackThreshold){
          // AINFO << "sub start 1 to end: " << start_end_val1;
          // AINFO << "sub start 2 to end: " << start_end_val2;
          topo_debug_info["turn back constraint"] = false;
          return break_lane_ids;
        }

        // 实际拓扑lane的位置和地图匹配位置校验
        topo_debug_info["split position constraint"] = true;
        auto temp_lane1_geos = temp_lane_1->geos;
        auto temp_lane2_geos = temp_lane_2->geos;
        if (!CheckSplitLanePositionConstraint(topo_type, target_lane_type1, target_lane_type2, *temp_lane1_geos, *temp_lane2_geos)){
          topo_debug_info["split position constraint"] = false;
          return break_lane_ids;
        }

        // 添加main_lane的后继
        auto iter1 = std::find(main_lane->next_lane_ids.begin(), main_lane->next_lane_ids.end(), target_lane_id1);

        if (iter1 == main_lane->next_lane_ids.end()) {
          main_lane->next_lane_ids.push_back(target_lane_id1);
          // AINFO << "sublane1: " << target_lane_id1;
        }

        auto iter2 = std::find(main_lane->next_lane_ids.begin(), main_lane->next_lane_ids.end(), target_lane_id2);

        if (iter2 == main_lane->next_lane_ids.end()) {
          main_lane->next_lane_ids.push_back(target_lane_id2);
          // AINFO << "sublane2: " << target_lane_id2;
        }
        
        // 添加previous_lane的前继
        iter1 = std::find(temp_lane_1->previous_lane_ids.begin(),
                          temp_lane_1->previous_lane_ids.end(), main_lane_id);

        iter2 = std::find(temp_lane_2->previous_lane_ids.begin(),
                          temp_lane_2->previous_lane_ids.end(), main_lane_id);

        if (iter1 == temp_lane_1->previous_lane_ids.end()) {
            temp_lane_1->previous_lane_ids.push_back(main_lane_id);
        }

        if (iter2 == temp_lane_2->previous_lane_ids.end()) {
            temp_lane_2->previous_lane_ids.push_back(main_lane_id);
        }

        main_lane->is_build_split_marker_by_map   = true;
        temp_lane_1->is_build_split_marker_by_map = true;
        temp_lane_2->is_build_split_marker_by_map = true;

        // 打上merge_split标签
        if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitLeft &&
            target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
        }
        else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitRight &&
                  target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
          temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        }
        else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitLeft &&
          target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
        }
        else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitRight &&
          target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
        }
        else {
          main_lane->is_build_split_marker_by_map   = false;
          temp_lane_1->is_build_split_marker_by_map = false;
          temp_lane_2->is_build_split_marker_by_map = false;
        }

        topo_debug_info["add split topo"] = true;
        main_lane->is_build_split_by_map   = true;
        temp_lane_1->is_build_split_by_map = true;
        temp_lane_2->is_build_split_by_map = true;
      }
    }
    // target_lane_id1需要打断
    else if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id == target_lane_id1 && main_lane_id != target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane1_ld_match, true, true)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }
      
      // 找到target_lane1，target_lane2
      auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
      if (temp_lane_1 != data_->lane_infos.end() && temp_lane_2 != data_->lane_infos.end() && temp_lane_2->line_points.size() > 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckSplitLaneRelationConstraint(*main_lane, {*temp_lane_2}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane2
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_2), ld_map, target_lane2_ld_match, mainlane_ld_match, true, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneSplit(target_lane_id1, target_lane_id2, ld_map, split_match_infos, topo_type, target_lane_type1, target_lane_type2, target_topo_distance, topo_debug_info);
        if (break_result){
          auto main_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });
          if (main_lane == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto temp_lane_1 = &data_->lane_infos.back();
          target_lane_id1 = temp_lane_1->id;

          auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
          if (temp_lane_2 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          // 添加main_lane的后继
          auto iter2 = std::find(main_lane->next_lane_ids.begin(), main_lane->next_lane_ids.end(), target_lane_id2);

          if (iter2 == main_lane->next_lane_ids.end()) {
            main_lane->next_lane_ids.push_back(target_lane_id2);
          }

          // 添加非打断产生支路的前继
          iter2 = std::find(temp_lane_2->previous_lane_ids.begin(),
                            temp_lane_2->previous_lane_ids.end(), main_lane_id);

          if (iter2 == temp_lane_2->previous_lane_ids.end()) {
              temp_lane_2->previous_lane_ids.push_back(main_lane_id);
          }

          main_lane->is_build_split_marker_by_map   = true;
          temp_lane_1->is_build_split_marker_by_map = true;
          temp_lane_2->is_build_split_marker_by_map = true;

          // 打上merge_split标签
          if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitLeft &&
              target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          }
          else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitRight &&
                  target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
            temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitLeft &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitRight &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
          else {
            main_lane->is_build_split_marker_by_map   = false;
            temp_lane_1->is_build_split_marker_by_map = false;
            temp_lane_2->is_build_split_marker_by_map = false;
          }

          // AINFO << "*******BreakLaneResult******";
          // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
          // AINFO << "start end points: (" << main_lane->line_points.front().x << ", " << main_lane->line_points.front().y << "); (" << main_lane->line_points.back().x << ", " << main_lane->line_points.back().y << ")";
          // AINFO << "************************";
          BreakLaneResult break_lane_result = {main_lane_id, target_lane_id2, target_lane_id1, static_cast<int>(main_lane->split_topo_extend), static_cast<int>(temp_lane_2->split_topo_extend), static_cast<int>(temp_lane_1->split_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add split topo"] = true;
          main_lane->is_build_split_by_map   = true;
          temp_lane_1->is_build_split_by_map = true;
          temp_lane_2->is_build_split_by_map = true;
        }
      }
    }

    // target_lane_id2需要打断
    else if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id != target_lane_id1 && main_lane_id == target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane2_ld_match, true, true)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      // 找到target_lane1，target_lane2
      auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
      if (temp_lane_1 != data_->lane_infos.end() && temp_lane_2 != data_->lane_infos.end() && temp_lane_1->line_points.size() > 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckSplitLaneRelationConstraint(*main_lane, {*temp_lane_1}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, true, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneSplit(target_lane_id2, target_lane_id1, ld_map, split_match_infos, topo_type, target_lane_type2, target_lane_type1, target_topo_distance, topo_debug_info);
        if (break_result){
          auto main_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });
          if (main_lane == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
          if (temp_lane_1 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto temp_lane_2 = &data_->lane_infos.back();
          target_lane_id2 = temp_lane_2->id;

          // 添加main_lane的后继
          auto iter1 = std::find(main_lane->next_lane_ids.begin(),
                                main_lane->next_lane_ids.end(), target_lane_id1);

          if (iter1 == main_lane->next_lane_ids.end()) {
            main_lane->next_lane_ids.push_back(target_lane_id1);
          }

          // 添加非打断产生支路的前继
          iter1 = std::find(temp_lane_1->previous_lane_ids.begin(),
                            temp_lane_1->previous_lane_ids.end(), main_lane_id);

          if (iter1 == temp_lane_1->previous_lane_ids.end()) {
              temp_lane_1->previous_lane_ids.push_back(main_lane_id);
          }

          main_lane->is_build_split_marker_by_map   = true;
          temp_lane_1->is_build_split_marker_by_map = true;
          temp_lane_2->is_build_split_marker_by_map = true;

          // 打上merge_split标签
          if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitLeft &&
              target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          }
          else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitRight &&
                  target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
            temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitLeft &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitRight &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          }
          else {
            main_lane->is_build_split_marker_by_map   = false;
            temp_lane_1->is_build_split_marker_by_map = false;
            temp_lane_2->is_build_split_marker_by_map = false;
          }

          // AINFO << "*******BreakLaneResult******";
          // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
          // AINFO << "start end points: (" << main_lane->line_points.front().x << ", " << main_lane->line_points.front().y << "); (" << main_lane->line_points.back().x << ", " << main_lane->line_points.back().y << ")";
          // AINFO << "************************";
          BreakLaneResult break_lane_result = {main_lane_id, target_lane_id1, target_lane_id2, static_cast<int>(main_lane->split_topo_extend), static_cast<int>(temp_lane_1->split_topo_extend), static_cast<int>(temp_lane_2->split_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add split topo"] = true;
          main_lane->is_build_split_by_map   = true;
          temp_lane_1->is_build_split_by_map = true;
          temp_lane_2->is_build_split_by_map = true;
        }
      }
    }
  }
  // 这个分支处理只有一个搜索结果，需要打断后
  else if (split_info->bev_target_to.size() == 1) {
    auto main_lane_id = main_lane->id;
    // 自车车道与匹配结果横向距离约束
    topo_debug_info["mainlane match constraint"] = true;
    topo_debug_info["turn back constraint"] = true;

    auto target_lane_id1 = split_info->bev_target_to[0].second.bev_lane_to;
    auto target_lane_type1 = split_info->bev_target_to[0].second.type;
    auto target_topo_distance = split_info->distance_to_ego;

    auto mainlane_ld_match = split_info->map_lanes_from;
    auto target_lane1_ld_match = split_info->bev_target_to[0].second.map_lanes_to;

    // 处理有一个后继且后继为connect情况，需要打断自车车道
    if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id != target_lane_id1 && target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward) {
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, {}, false, true)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });

      if (temp_lane_1 != data_->lane_infos.end() && temp_lane_1->line_points.size() > 2) {
        // 匹配车道是符合拓扑关系约束
        if (!CheckSplitLaneRelationConstraint(*main_lane, {*temp_lane_1}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, true, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        /// main_lane
        if(!CheckSublaneDifferenceConstraint(main_lane, ld_map, {}, mainlane_ld_match, true, false)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneSplit(main_lane_id, target_lane_id1, ld_map, connect_match_infos, topo_type, topo_type, target_lane_type1, target_topo_distance, topo_debug_info);
        if (break_result){
          auto main_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });
          if (main_lane == data_->lane_infos.end()) {
            return break_lane_ids;
          }
          auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
          if (temp_lane_1 == data_->lane_infos.end()) {;
            return break_lane_ids;
          }

          auto temp_lane_2 = &data_->lane_infos.back();
          auto target_lane_id2 = temp_lane_2->id;

          // 添加main_lane的后继
          auto iter1 = std::find(main_lane->next_lane_ids.begin(), main_lane->next_lane_ids.end(), target_lane_id1);

          if (iter1 == main_lane->next_lane_ids.end()) {
            main_lane->next_lane_ids.push_back(target_lane_id1);
          }

          // 添加非打断产生支路的前继
          iter1 = std::find(temp_lane_1->previous_lane_ids.begin(),
                            temp_lane_1->previous_lane_ids.end(), main_lane_id);

          if (iter1 == temp_lane_1->previous_lane_ids.end()) {
              temp_lane_1->previous_lane_ids.push_back(main_lane_id);
          }

          // 添加标签: 判断支路的终点在左边还是右边，通过向量的cross判定的
          Eigen::Vector2d vec_main_lane, vec_split_lane;
          vec_main_lane.x() = temp_lane_1->line_points.front().x - main_lane->line_points.front().x;
          vec_main_lane.y() = temp_lane_1->line_points.front().y - main_lane->line_points.front().y;
          vec_split_lane.x() = temp_lane_2->line_points.front().x - main_lane->line_points.front().x;
          vec_split_lane.y() = temp_lane_2->line_points.front().y - main_lane->line_points.front().y;
          double cross_val = vector2dCrossProduct(vec_main_lane, vec_split_lane);

          main_lane->is_build_split_marker_by_map   = true;
          temp_lane_1->is_build_split_marker_by_map = true;
          temp_lane_2->is_build_split_marker_by_map = true;

          if (cross_val < 0) {
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          } 
          else if (cross_val > 0) {
            temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          }
          else {
            main_lane->is_build_split_marker_by_map   = false;
            temp_lane_1->is_build_split_marker_by_map = false;
            temp_lane_2->is_build_split_marker_by_map = false;
          }

          // // 打上merge_split标签
          // target_lane_type2 = topo_type
          // if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitLeft &&
          //     target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
          //   temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          // }
          // else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kSplitRight &&
          //         target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
          //   temp_lane_1->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          // }
          // else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitLeft &&
          //   target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          //   temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
          // }
          // else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kSplitRight &&
          //   target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          //   temp_lane_2->split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
          // }

          BreakLaneResult break_lane_result = {main_lane_id, target_lane_id1, target_lane_id2, static_cast<int>(main_lane->split_topo_extend), static_cast<int>(temp_lane_1->split_topo_extend), static_cast<int>(temp_lane_2->split_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add split topo"] = true;
          main_lane->is_build_split_by_map   = true;
          temp_lane_1->is_build_split_by_map = true;
          temp_lane_2->is_build_split_by_map = true;
        }
      }
    }
  }

  return break_lane_ids;
  
}

std::optional<BevMapProcessor::BreakLaneResult> BevMapProcessor::AddMergeTopoByMapInfo(std::optional<const BevMergeInfo*> merge_info_result,
                                            cem::message::sensor::BevLaneInfo* target_lane1,
                                            const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                            const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                                            const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                                            const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                                            json &topo_debug_info) {
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinTopoEndThreshold = 20.0;  // 目标添加拓扑车道线终点到当前position距离

  std::optional<BreakLaneResult> break_lane_ids;

  topo_debug_info["add merge topo"] = false;
  topo_debug_info["query id"] = target_lane1->id;
  json target_sub_lane_array = json::array();

  if (target_lane1 -> line_points.size() < 2) {
    topo_debug_info["target_sub_lane_result"] = target_sub_lane_array;
    return break_lane_ids;
  }

  if (!merge_info_result.has_value()) {
    return break_lane_ids;
  }

  const BevMergeInfo* merge_info = merge_info_result.value();
  LdMapProcessor::MergeSplitType topo_type = merge_info->match_type;
  topo_debug_info["topo_point type"] = static_cast<int>(topo_type);
  topo_debug_info["mainlane id"] = merge_info->bev_lane_to;
  topo_debug_info["distance_to_ego"] = merge_info->distance_to_ego;

# ifdef AddTopoDebug
  AINFO << "*******MergeTopoQuery******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "ego lane id: " << target_lane1->id
        << "; end lane id: " << merge_info->bev_lane_to
        << "; distance_to_ego: " << merge_info->distance_to_ego
        << "; Topo point type: " << static_cast<int>(topo_type);

  std::stringstream lane_id_str;
  for (const auto& lane_id : merge_info->map_lanes_from){
    lane_id_str << lane_id << ", ";
  }
  AINFO << "main_ld_map id: " << lane_id_str.str();
  
  for (const auto& pair: merge_info->bev_lanes_from) {
    AINFO << "key1: " << pair.first 
          << "; start lane id: " << pair.second.bev_lane_from
          << "; Topo lane type: " << static_cast<int>(pair.second.type);

    std::stringstream lane_id_str;
    for (const auto& lane_id : pair.second.map_lanes_to){
      lane_id_str << lane_id << ", ";
    }
    AINFO << "sub_ld_map id: " << lane_id_str.str();
  }
  AINFO << "************************";
# endif

  for (const auto& pair: merge_info->bev_lanes_from) {
    json sub_lane_info;
    sub_lane_info["id"] = pair.second.bev_lane_from;
    sub_lane_info["topo_lane type"] = static_cast<int>(pair.second.type);
    target_sub_lane_array.push_back(sub_lane_info);
  }
  topo_debug_info["target_sub_lane_result"] = target_sub_lane_array;

  // 假设target_lane1->id不在匹配的merge_info->bev_lanes_from中
  if (merge_info->bev_lanes_from.count(target_lane1->id) == 0) {
    topo_debug_info["target_lane1 match"] = false;
    return break_lane_ids;
  }

  auto main_lane_id = merge_info->bev_lane_to;
  auto main_lane_iter = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                    [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });
  if (main_lane_iter == data_->lane_infos.end()){
    topo_debug_info["mainlane match constraint"] = false;
    return break_lane_ids;
  }
  auto main_lane = &(*main_lane_iter);

  // 计算目标打断线到end的距离
  double dist_ego2End = GetEgoToEndLength(main_lane);

  // bev_lanes_from: 当前匹配得到的前继，目前规则只处理二合一的情况，后期可扩展到三合一等情况，暂时不更改target_lane有后继的情况
  if (merge_info->bev_lanes_from.size() == 2){
    // 主车道与匹配结果横向距离约束
    topo_debug_info["mainlane match constraint"] = true;
    topo_debug_info["turn back constraint"] = true;

    auto target_lane_id1 = target_lane1->id;
    auto target_lane_type1 = merge_info->bev_lanes_from.at(target_lane_id1).type;
    auto target_lane1_ld_match = merge_info->bev_lanes_from.at(target_lane_id1).map_lanes_from;

    uint64_t target_lane_id2 = 0;
    auto target_lane_type2 = LdMapProcessor::MergeSplitType::kConnectForward;
    std::vector<uint64_t> target_lane2_ld_match;
    // 找到target_lane2相关信息
    for (const auto& bev_pair : merge_info->bev_lanes_from) {
        if (bev_pair.second.bev_lane_from != target_lane_id1) {
            target_lane_id2 = bev_pair.second.bev_lane_from;
            target_lane_type2 = bev_pair.second.type;
            target_lane2_ld_match = bev_pair.second.map_lanes_from;
            break;
        }
    }

    auto mainlane_ld_match = merge_info->map_lanes_to;

    auto target_topo_distance = merge_info->distance_to_ego;
    // 不需要打断添加拓扑规则
    if (target_lane1->next_lane_ids.empty() && main_lane_id != target_lane_id1 && main_lane_id != target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane1_ld_match, false, false)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      // 找到target_lane2
      auto temp_lane_1 = target_lane1;
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });

      if (temp_lane_2 != data_->lane_infos.end() &&
          temp_lane_1->line_points.size() >= 2 && temp_lane_1->line_points.size() >= 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckMergeLaneRelationConstraint(*main_lane, {*temp_lane_1, *temp_lane_2}, false, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, false, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        /// target_lane2
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_2), ld_map, target_lane2_ld_match, mainlane_ld_match, false, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 匹配拓扑点和实际拓扑点纵向距离限制条件，根据量产阈值设计为40m
        if(!CheckTopoPointLongitudeDistanceConstraint(temp_lane_1->line_points.back(), target_topo_distance, ld_map, merge_match_infos)) {
          topo_debug_info["topo point longitude constraint"] = false;
          return break_lane_ids;
        }
        
        // 判断是否存在回折，存在回折不添加拓扑
        auto temp_lane_1_end = temp_lane_1->line_points.back();
        auto temp_lane_2_end = temp_lane_2->line_points.back();
        auto main_lane_start = main_lane->line_points.front();
        auto main_lane_second_start = main_lane->line_points[1];
        Eigen::Vector2d main_lane_vec, topo_vec1, topo_vec2;
        main_lane_vec.x() = main_lane_second_start.x - main_lane_start.x;
        main_lane_vec.y() = main_lane_second_start.y - main_lane_start.y;
        topo_vec1.x() = main_lane_start.x - temp_lane_1_end.x;
        topo_vec1.y() = main_lane_start.y - temp_lane_1_end.y;
        topo_vec2.x() = main_lane_start.x - temp_lane_2_end.x;
        topo_vec2.y() = main_lane_start.y - temp_lane_2_end.y;
        double end_start_val1 = topo_vec1.dot(main_lane_vec) / main_lane_vec.norm();
        double end_start_val2 = topo_vec2.dot(main_lane_vec) / main_lane_vec.norm();
        
        if (end_start_val1 < MinBackThreshold || end_start_val2 < MinBackThreshold){
          // AINFO << "sub start 1 to end: " << start_end_val1;
          // AINFO << "sub start 2 to end: " << start_end_val2;
          topo_debug_info["turn back constraint"] = false;
          return break_lane_ids;
        }

        // 实际拓扑lane的位置和地图匹配位置校验
        topo_debug_info["merge position constraint"] = true;
        auto temp_lane1_geos = temp_lane_1->geos;
        auto temp_lane2_geos = temp_lane_2->geos;
        if (!CheckMergeLanePositionConstraint(topo_type, target_lane_type1, target_lane_type2, *temp_lane1_geos, *temp_lane2_geos)){
          topo_debug_info["merge position constraint"] = false;
          return break_lane_ids;
        }

        // 添加main_lane的前继
        auto iter1 = std::find(main_lane->previous_lane_ids.begin(), main_lane->previous_lane_ids.end(), target_lane_id1);

        if (iter1 == main_lane->previous_lane_ids.end()) {
          main_lane->previous_lane_ids.push_back(target_lane_id1);
          // AINFO << "sublane1: " << target_lane_id1;
        }

        auto iter2 = std::find(main_lane->previous_lane_ids.begin(), main_lane->previous_lane_ids.end(), target_lane_id2);

        if (iter2 == main_lane->previous_lane_ids.end()) {
          main_lane->previous_lane_ids.push_back(target_lane_id2);
          // AINFO << "sublane2: " << target_lane_id2;
        }
        
        // 添加previous_lane的前继
        iter1 = std::find(temp_lane_1->next_lane_ids.begin(),
                          temp_lane_1->next_lane_ids.end(), main_lane_id);

        iter2 = std::find(temp_lane_2->next_lane_ids.begin(),
                          temp_lane_2->next_lane_ids.end(), main_lane_id);

        if (iter1 == temp_lane_1->next_lane_ids.end()) {
            temp_lane_1->next_lane_ids.push_back(main_lane_id);
        }

        if (iter2 == temp_lane_2->next_lane_ids.end()) {
            temp_lane_2->next_lane_ids.push_back(main_lane_id);
        }

        // 创建拓扑标志
        main_lane->is_build_merge_marker_by_map   = true;
        temp_lane_1->is_build_merge_marker_by_map = true;
        temp_lane_2->is_build_merge_marker_by_map = true;

        // 打上merge_split标签
        if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeLeft &&
            target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
        }
        else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeRight &&
                  target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
          temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
        }
        else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeLeft &&
          target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
        }
        else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeRight &&
          target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
          temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
        }
        else {
          main_lane->is_build_merge_marker_by_map   = false;
          temp_lane_1->is_build_merge_marker_by_map = false;
          temp_lane_2->is_build_merge_marker_by_map = false;
        }

        topo_debug_info["add merge topo"] = true;
      }
    }
    // target_lane_id1需要打断
    else if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id == target_lane_id1 && main_lane_id != target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane1_ld_match, true, false)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }
      
      // 找到target_lane1，target_lane2
      auto temp_lane_1 = target_lane1;
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
      if (temp_lane_2 != data_->lane_infos.end() && temp_lane_2->line_points.size() > 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckMergeLaneRelationConstraint(*main_lane, {*temp_lane_2}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane2
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_2), ld_map, target_lane2_ld_match, mainlane_ld_match, false, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneMerge(target_lane_id1, target_lane_id2, ld_map, merge_match_infos, topo_type, target_lane_type1, target_lane_type2, target_topo_distance, topo_debug_info);
        if (break_result){
          auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
          if (temp_lane_1 == data_->lane_infos.end()) {;
            return break_lane_ids;
          }

          auto main_lane = &data_->lane_infos.back();
          main_lane_id = main_lane->id;
          auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
          if (temp_lane_2 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          // 添加main_lane的前继
          auto iter2 = std::find(main_lane->previous_lane_ids.begin(), main_lane->previous_lane_ids.end(), target_lane_id2);

          if (iter2 == main_lane->previous_lane_ids.end()) {
            main_lane->previous_lane_ids.push_back(target_lane_id2);
          }

          // 添加非打断产生支路的后继
          iter2 = std::find(temp_lane_2->next_lane_ids.begin(),
                            temp_lane_2->next_lane_ids.end(), main_lane_id);

          if (iter2 == temp_lane_2->next_lane_ids.end()) {
              temp_lane_2->next_lane_ids.push_back(main_lane_id);
          }

          // 创建拓扑标志
          main_lane->is_build_merge_marker_by_map   = true;
          temp_lane_1->is_build_merge_marker_by_map = true;
          temp_lane_2->is_build_merge_marker_by_map = true;

          // 打上merge_split标签
          if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeLeft &&
              target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }
          else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeRight &&
                    target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeLeft &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeRight &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
          else {
            main_lane->is_build_merge   = false;
            temp_lane_1->is_build_merge = false;
            temp_lane_2->is_build_merge = false;
          }

          // AINFO << "*******BreakLaneResult******";
          // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
          // AINFO << "start end points: (" << main_lane->line_points.front().x << ", " << main_lane->line_points.front().y << "); (" << main_lane->line_points.back().x << ", " << main_lane->line_points.back().y << ")";
          // AINFO << "************************";
          BreakLaneResult break_lane_result = {target_lane_id1, target_lane_id2, main_lane_id, static_cast<int>(temp_lane_1->merge_topo_extend), static_cast<int>(temp_lane_2->merge_topo_extend), static_cast<int>(main_lane->merge_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add merge topo"] = true;
        }
      }
    }

    // target_lane_id2需要打断
    else if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id != target_lane_id1 && main_lane_id == target_lane_id2){
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, target_lane2_ld_match, true, false)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      // 找到target_lane2
      auto temp_lane_1 = target_lane1;
      auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
      if (temp_lane_2 != data_->lane_infos.end() && temp_lane_1->line_points.size() > 2){
        // 匹配车道是符合拓扑关系约束
        if (!CheckMergeLaneRelationConstraint(*main_lane, {*temp_lane_1}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, false, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneMerge(target_lane_id2, target_lane_id1, ld_map, merge_match_infos, topo_type, target_lane_type2, target_lane_type1, target_topo_distance, topo_debug_info);
        if (break_result){
          auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
          if (temp_lane_1 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });
          if (temp_lane_2 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto main_lane = &data_->lane_infos.back();
          main_lane_id = main_lane->id;

          // 添加main_lane的前继
          auto iter1 = std::find(main_lane->previous_lane_ids.begin(),
                                main_lane->previous_lane_ids.end(), target_lane_id1);

          if (iter1 == main_lane->previous_lane_ids.end()) {
            main_lane->previous_lane_ids.push_back(target_lane_id1);
          }

          // 添加非打断产生支路的后继
          iter1 = std::find(temp_lane_1->next_lane_ids.begin(),
                            temp_lane_1->next_lane_ids.end(), main_lane_id);

          if (iter1 == temp_lane_1->next_lane_ids.end()) {
              temp_lane_1->next_lane_ids.push_back(main_lane_id);
          }

          // 创建拓扑标志
          main_lane->is_build_merge_marker_by_map   = true; 
          temp_lane_1->is_build_merge_marker_by_map = true;
          temp_lane_2->is_build_merge_marker_by_map = true;

          // 打上merge_split标签
          if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeLeft &&
              target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }
          else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeRight &&
                    target_lane_type2 == LdMapProcessor::MergeSplitType::kConnectForward) {
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeLeft &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }
          else if (target_lane_type2 == LdMapProcessor::MergeSplitType::kMergeRight &&
            target_lane_type1 == LdMapProcessor::MergeSplitType::kConnectForward){
            temp_lane_2->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
          else {
            main_lane->is_build_merge_marker_by_map   = false;
            temp_lane_1->is_build_merge_marker_by_map = false;
            temp_lane_2->is_build_merge_marker_by_map = false;
          }

          // AINFO << "*******BreakLaneResult******";
          // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
          // AINFO << "start end points: (" << main_lane->line_points.front().x << ", " << main_lane->line_points.front().y << "); (" << main_lane->line_points.back().x << ", " << main_lane->line_points.back().y << ")";
          // AINFO << "************************";
          BreakLaneResult break_lane_result = {target_lane_id2, target_lane_id1, main_lane_id, static_cast<int>(temp_lane_2->merge_topo_extend), static_cast<int>(temp_lane_1->merge_topo_extend), static_cast<int>(main_lane->merge_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add merge topo"] = true;
        }
      }
    }
  }
  // 这个分支处理只有一个搜索结果，需要打断后添加拓扑
  else if (target_lane1->next_lane_ids.empty() && merge_info->bev_lanes_from.size() == 1) {
    auto main_lane_id = main_lane->id;
    // 自车车道与匹配结果横向距离约束
    topo_debug_info["mainlane match constraint"] = true;
    topo_debug_info["turn back constraint"] = true;

    auto target_lane_id1 = target_lane1->id;
    auto target_lane_type1 = merge_info->bev_lanes_from.at(target_lane_id1).type;
    auto target_topo_distance = merge_info->distance_to_ego;

    auto mainlane_ld_match = merge_info->map_lanes_to;
    auto target_lane1_ld_match = merge_info->bev_lanes_from.at(target_lane_id1).map_lanes_from;

    // 处理有一个后继且后继为merge情况，需要自车车道打断其他车道
    if ((dist_ego2End - target_topo_distance > MinTopoEndThreshold) && main_lane_id != target_lane_id1 && (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeLeft || target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeRight)) {
      if (!CheckMainlaneDifferenceConstraint(main_lane, ld_map, mainlane_ld_match, {}, false, false)) {
        topo_debug_info["mainlane match constraint"] = false;
        return break_lane_ids;
      }

      auto temp_lane_1 = target_lane1;
      if (temp_lane_1->line_points.size() > 2) {
        // 匹配车道是符合拓扑关系约束
        if (!CheckMergeLaneRelationConstraint(*main_lane, {*temp_lane_1}, true, topo_debug_info)) {
          return break_lane_ids;
        }

        // 悬空线与地图匹配约束
        /// target_lane1
        if(!CheckSublaneDifferenceConstraint(&(*temp_lane_1), ld_map, target_lane1_ld_match, mainlane_ld_match, false, true)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        /// main_lane
        if(!CheckSublaneDifferenceConstraint(main_lane, ld_map, {}, mainlane_ld_match, false, false)) {
          topo_debug_info["sublane match constraint"] = false;
          return break_lane_ids;
        }

        // 调用函数，打断temp_lane_1，ego_lane变成当前ego_lane和他的后继，返回代表是否成功打断的flag变量
        bool break_result;
        break_result = BreakLaneMerge(main_lane_id, target_lane_id1, ld_map, merge_match_infos, topo_type, LdMapProcessor::MergeSplitType::kConnectForward, target_lane_type1, target_topo_distance, topo_debug_info);
        if (break_result){
          auto temp_lane_1 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id1](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id1 == lane.id; });
          if (temp_lane_1 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto target_lane_id2 = main_lane_id;
          auto temp_lane_2 = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                                      [target_lane_id2](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id2 == lane.id; });

          if (temp_lane_2 == data_->lane_infos.end()) {
            return break_lane_ids;
          }

          auto main_lane = &data_->lane_infos.back();
          main_lane_id = main_lane->id;

          // 添加main_lane的前继
          auto iter1 = std::find(main_lane->previous_lane_ids.begin(), main_lane->previous_lane_ids.end(), target_lane_id1);

          if (iter1 == main_lane->previous_lane_ids.end()) {
            main_lane->previous_lane_ids.push_back(target_lane_id1);
          }

          // 添加非打断产生支路的后继
          iter1 = std::find(temp_lane_1->next_lane_ids.begin(),
                            temp_lane_1->next_lane_ids.end(), main_lane_id);

          if (iter1 == temp_lane_1->next_lane_ids.end()) {
              temp_lane_1->next_lane_ids.push_back(main_lane_id);
          }

          // 创建拓扑标志
          main_lane->is_build_merge_marker_by_map   = true;
          temp_lane_1->is_build_merge_marker_by_map = true;
          temp_lane_2->is_build_merge_marker_by_map = true;

          // 打上merge_split标签
          if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeLeft) {
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
          }
          else if (target_lane_type1 == LdMapProcessor::MergeSplitType::kMergeRight) {
            temp_lane_1->merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
          }
          else {
            main_lane->is_build_merge_marker_by_map   = false;
            temp_lane_1->is_build_merge_marker_by_map = false;
            temp_lane_2->is_build_merge_marker_by_map = false;
          }

          BreakLaneResult break_lane_result = {target_lane_id2, target_lane_id1, main_lane_id, static_cast<int>(temp_lane_2->merge_topo_extend), static_cast<int>(temp_lane_1->merge_topo_extend), static_cast<int>(main_lane->merge_topo_extend)};
          break_lane_ids = break_lane_result;
          topo_debug_info["add merge topo"] = true;
        }
      }
    }
  }

  return break_lane_ids;
}

bool BevMapProcessor::UpadateLaneIDMemory(std::set<uint64_t> current_lane_ids, std::pair<std::set<uint64_t>, int> &lane_id_memory_result) {
  if (current_lane_ids == lane_id_memory_result.first){
    lane_id_memory_result.second += 1;
    return false;
  } 
  else {
    lane_id_memory_result.first = current_lane_ids;
    lane_id_memory_result.second = 1;
    return true;
  }
};

void BevMapProcessor::UpadateSplitMemory(bool update_lane_id_flag) {
  split_memory_type_.clear();
  if (update_lane_id_flag) {
    // 清理unordered_map重新记录
    split_memory_result_.clear();
    for (const auto & lane : data_->lane_infos){
      if (lane.next_lane_ids.size() == 2){
        BevMapProcessor::TopoRecord temp_topo_record;
        temp_topo_record.main_lane_id = lane.id;
        temp_topo_record.sub_lane_ids.insert(lane.next_lane_ids.begin(), lane.next_lane_ids.end());
        temp_topo_record.record_number = 1;
        split_memory_result_[lane.id] = temp_topo_record; 
        for (const auto & sub_lane_id : temp_topo_record.sub_lane_ids) {
          auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
          if (sub_lane == data_->lane_infos.end()){
            continue;
          }
          split_memory_type_[sub_lane_id] = sub_lane->split_topo_extend;
        }
      }
    }
  }
  else {
    std::unordered_map<uint64_t, BevMapProcessor::TopoRecord> current_split_result;
    for (const auto & lane : data_->lane_infos){
      if (lane.next_lane_ids.size() == 2){
        BevMapProcessor::TopoRecord temp_topo_record;
        temp_topo_record.main_lane_id = lane.id;
        temp_topo_record.sub_lane_ids.insert(lane.next_lane_ids.begin(), lane.next_lane_ids.end());
        auto iter = split_memory_result_.find(lane.id);
        if (iter != split_memory_result_.end() && iter->second.sub_lane_ids == temp_topo_record.sub_lane_ids) {
          temp_topo_record.record_number = iter->second.record_number + 1;
        }
        else{
          temp_topo_record.record_number = 1;
        }
        current_split_result[lane.id] = temp_topo_record;

        for (const auto & sub_lane_id : temp_topo_record.sub_lane_ids) {
          auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
          if (sub_lane == data_->lane_infos.end()){
            continue;
          }
          split_memory_type_[sub_lane_id] = sub_lane->split_topo_extend;
        }
      }
    }
    split_memory_result_ = current_split_result;
  }
}

void BevMapProcessor::UpadateMergeMemory(bool update_lane_id_flag) {
  merge_memory_type_.clear();
  if (update_lane_id_flag) {
    // 清理unordered_map重新记录
    merge_memory_result_.clear();
    for (const auto & lane : data_->lane_infos){
      if (lane.previous_lane_ids.size() == 2){
        BevMapProcessor::TopoRecord temp_topo_record;
        temp_topo_record.main_lane_id = lane.id;
        temp_topo_record.sub_lane_ids.insert(lane.previous_lane_ids.begin(), lane.previous_lane_ids.end());
        temp_topo_record.record_number = 1;
        merge_memory_result_[lane.id] = temp_topo_record; 
        for (const auto & sub_lane_id : temp_topo_record.sub_lane_ids) {
          auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
          if (sub_lane == data_->lane_infos.end()){
            continue;
          }
          merge_memory_type_[sub_lane_id] = sub_lane->merge_topo_extend;
        }
      }
    }
  }
  else {
    std::unordered_map<uint64_t, BevMapProcessor::TopoRecord> current_merge_result;
    for (const auto & lane : data_->lane_infos){
      if (lane.previous_lane_ids.size() == 2){
        BevMapProcessor::TopoRecord temp_topo_record;
        temp_topo_record.main_lane_id = lane.id;
        temp_topo_record.sub_lane_ids.insert(lane.previous_lane_ids.begin(), lane.previous_lane_ids.end());
        auto iter = merge_memory_result_.find(lane.id);
        if (iter != merge_memory_result_.end() && iter->second.sub_lane_ids == temp_topo_record.sub_lane_ids) {
          temp_topo_record.record_number = iter->second.record_number + 1;
        }
        else{
          temp_topo_record.record_number = 1;
        }
        current_merge_result[lane.id] = temp_topo_record;

        for (const auto & sub_lane_id : temp_topo_record.sub_lane_ids) {
          auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
          if (sub_lane == data_->lane_infos.end()){
            continue;
          }
          merge_memory_type_[sub_lane_id] = sub_lane->merge_topo_extend;
        }
      }
    }
    merge_memory_result_ = current_merge_result;
  }
}

void BevMapProcessor::UpadateBreakMemory(bool update_lane_id_flag, std::unordered_map<uint64_t, BevMapProcessor::BreakTopoRecord> & memory_break_result, std::optional<BevMapProcessor::BreakLaneResult> break_lane_result) {
  if (update_lane_id_flag) {
    // 清理unordered_map重新记录
    memory_break_result.clear();
    if (break_lane_result.has_value()) {
      BevMapProcessor::BreakTopoRecord current_topo_record;
      current_topo_record.break_lane_id = break_lane_result.value().break_lane_id;
      current_topo_record.other_lane_id = break_lane_result.value().other_lane_id;
      current_topo_record.break_lane_type = break_lane_result.value().break_lane_type;
      current_topo_record.other_lane_type = break_lane_result.value().other_lane_type;
      current_topo_record.add_lane_type = break_lane_result.value().add_lane_type;
      current_topo_record.record_number = 1;
      memory_break_result[current_topo_record.break_lane_id] = current_topo_record;
    }
  }
  else {
    std::unordered_map<uint64_t,  BevMapProcessor::BreakTopoRecord> current_break_result;
    if (break_lane_result.has_value()) {
      BevMapProcessor::BreakTopoRecord current_topo_record;
      current_topo_record.break_lane_id = break_lane_result.value().break_lane_id;
      current_topo_record.other_lane_id = break_lane_result.value().other_lane_id;
      current_topo_record.break_lane_type = break_lane_result.value().break_lane_type;
      current_topo_record.other_lane_type = break_lane_result.value().other_lane_type;
      current_topo_record.add_lane_type = break_lane_result.value().add_lane_type;
      auto iter = memory_break_result.find(current_topo_record.break_lane_id);
      if (iter != memory_break_result.end() && iter->second.other_lane_id == current_topo_record.other_lane_id) {
        current_topo_record.record_number = iter->second.record_number + 1;
      }
      else{
        current_topo_record.record_number = 1;
      }
      current_break_result[current_topo_record.break_lane_id] = current_topo_record;
    }
    memory_break_result = current_break_result;
  }
}

void BevMapProcessor::AddSplitByHistory(uint64_t target_lane_id, json &topo_debug_info) {
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离

  // 根据输入id查询有没有对应的拓扑
  BevMapProcessor::TopoRecord target_record;
  target_record.record_number = 0;
  for (const auto& pair : split_memory_result_) {
    const BevMapProcessor::TopoRecord& topo_record = pair.second;
    if (target_lane_id == topo_record.main_lane_id){
      target_record = topo_record;
      break;
    }

    if (topo_record.sub_lane_ids.count(target_lane_id)){
      target_record = topo_record;
      break;
    }
  }

  auto record_number =  target_record.record_number;
   // 记录数量 < 3，不添加拓扑
  if (record_number < 3) {
    return;
  }
  auto main_lane_id = target_record.main_lane_id;
  auto sub_lane_ids = target_record.sub_lane_ids;

  auto main_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });

  if (main_lane == data_->lane_infos.end()) {
    return;
  }

  if (!main_lane->next_lane_ids.empty()) {
    return;
  }

  // 记录数量 >= 3，使用记忆拓扑
  std::vector <std::vector<cem::message::sensor::BevLaneInfo>::iterator> sub_lanes;
  for (const auto & sub_lane_id : sub_lane_ids){
    auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
    if (sub_lane == data_->lane_infos.end()) {
      continue;
    }

    if (split_memory_type_.find(sub_lane_id) == split_memory_type_.end()) {
      continue;
    }

    sub_lanes.push_back(sub_lane);
  }

  if (sub_lanes.size() != sub_lane_ids.size()){
    return;
  }

  if (main_lane->line_points.size() < 2) {
    return;
  }

  // 判断是否存在回折，存在回折不添加拓扑
  for (const auto & sub_lane : sub_lanes) {
    auto sub_lane_start = sub_lane->line_points.front();
    auto main_lane_end = main_lane->line_points.back();
    auto main_lane_second_end = main_lane->line_points[main_lane->line_points.size() - 2];
    Eigen::Vector2d main_lane_vec, topo_vec1;
    main_lane_vec.x() = main_lane_end.x - main_lane_second_end.x;
    main_lane_vec.y() = main_lane_end.y - main_lane_second_end.y;
    topo_vec1.x() = sub_lane_start.x - main_lane_end.x;
    topo_vec1.y() = sub_lane_start.y - main_lane_end.y;
    double start_end_val1 = topo_vec1.dot(main_lane_vec) / main_lane_vec.norm();
    
    if (start_end_val1 < MinBackThreshold){
      // AINFO << "sub start 1 to end: " << start_end_val1;
      topo_debug_info["turn back constraint"] = false;
      return;
    }
  }

  // 添加main_lane 后继
  main_lane->next_lane_ids.insert(main_lane->next_lane_ids.begin(), sub_lane_ids.begin(), sub_lane_ids.end());
  main_lane->is_build_split_by_map   = true;

  for (const auto & sub_lane : sub_lanes){
    auto it = std::find(sub_lane->previous_lane_ids.begin(), sub_lane->previous_lane_ids.end(), main_lane_id);
    if (it != sub_lane->previous_lane_ids.end()) {
      continue;
    }
    sub_lane->previous_lane_ids.push_back(main_lane_id);
    sub_lane->split_topo_extend =  split_memory_type_.at(sub_lane->id);
    sub_lane->is_build_split_by_map = true;
  }

  topo_debug_info["add memory split topo"] = true;
}

std::optional<BevMapProcessor::BreakLaneResult> BevMapProcessor::AddSplitBreakByHistory(uint64_t input_lane_id, json &topo_debug_info) {
  constexpr double MaxDistSqMarker = 10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MoveDistanceThreshold = 8.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinEndThreshold = 20.0;  // 目标断点到终点的距离阈值
  
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  std::optional<BreakLaneResult> break_lane_ids;

  // 根据输入id查询有没有对应的拓扑
  BevMapProcessor::BreakTopoRecord target_record;
  target_record.record_number = 0;
  for (const auto& pair : split_memory_break_result_) {
    const BevMapProcessor::BreakTopoRecord& topo_record = pair.second;
    if (input_lane_id == topo_record.break_lane_id || input_lane_id == topo_record.other_lane_id){
      target_record = topo_record;
      break;
    }
  }

  auto record_number =  target_record.record_number;
   // 记录数量 < 3，不添加拓扑
  if (record_number < 3) {
    return break_lane_ids;
  }

  auto target_lane_id = target_record.break_lane_id;
  auto topo_lane_id = target_record.other_lane_id;

  auto target_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [target_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id == lane.id; });

  if (target_lane == data_->lane_infos.end()) {
    return break_lane_ids;
  }

  if (!target_lane->next_lane_ids.empty()) {
    return break_lane_ids;
  }

  auto topo_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                              [topo_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return topo_lane_id == lane.id; });

  if (topo_lane == data_->lane_infos.end()){
    return break_lane_ids;
  }

  if (target_lane->line_points.size() < 4) {
    return break_lane_ids;
  }

  std::vector<cem::message::common::Point2DF>::iterator split_point1;
  double distance_pt2seg = std::numeric_limits<double>::max();
  // 打断备选规则2：找到距离最近点，向前挪动8m（暂定），如果打断点数不够就找最远的点
  for (auto iter = target_lane->line_points.begin() + 1;
            iter != target_lane->line_points.end(); ++iter) {
    // 求与输入的车道line_segment悬空车道起点的最近点
    auto temp_start = *(iter - 1);
    auto temp_end = *iter;
    double tmp_dist = CalculatePoint2SegmentDist(topo_lane->line_points.front(), temp_start, temp_end);
    if (distance_pt2seg > tmp_dist) {
      split_point1 = iter;
      distance_pt2seg = tmp_dist;
    }
  }

  // 校验结果split_point1是否可以分出足够数量的行点
  int break_lane1_pts_num = split_point1 - target_lane->line_points.begin();
  int break_lane2_pts_num = target_lane->line_points.end() - split_point1;
  //  AINFO << "split_point1 constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_lane_ids;
  }

  // 基于当前的split_point前移8m
  std::vector<cem::message::common::Point2DF>::iterator split_point = split_point1;
  for (auto iter = split_point1;
            iter != target_lane->line_points.begin() + 2; --iter) {
    // 求与输入的车道line_segment与代表悬空车道的line是否相交，找到第一个相交片段
    auto temp_point = *iter;
    double tmp_dist = CalculatePoint2PointDist(*split_point1, temp_point);
    split_point = iter;
    if (tmp_dist > MoveDistanceThreshold) {
      break;
    }
  }

  // AINFO << "*******SplitPointResultModify******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  // AINFO << "sequence number: " << data_->header.cycle_counter;
  // AINFO << "split point: (" << split_point1->x << ", " << split_point1->y << ")";
  // AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
  // AINFO << "*******************************";

  // if (!get_split_point){
  //   return break_success_flag;
  // }

  // 校验结果split_point是否可以分出足够数量的行点
  break_lane1_pts_num = split_point - target_lane->line_points.begin();
  break_lane2_pts_num = target_lane->line_points.end() - split_point;
  // AINFO << "split_point constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_lane_ids;
  }

  // 打断产生的距离超过阈值约束
  double end_point_val = 0;
  for (auto iter = split_point;
            iter < target_lane->line_points.end() - 1; ++iter) {
    auto temp_point1 = *iter;
    auto temp_point2 = *(iter+1);
    end_point_val += CalculatePoint2PointDist(temp_point1, temp_point2);
  }
  if (end_point_val < MinEndThreshold){
    // AINFO << "break_point-end constraint: " << end_point_val;
    topo_debug_info["break_point-end constraint"] = false;
    return break_lane_ids;
  }

  // 校验结果split_point是否出现回折
  auto temp_lane_1_start = topo_lane->line_points.front();
  auto main_lane_end = *(split_point - 1);
  auto main_lane_second_end = *(split_point - 2);
  Eigen::Vector2d main_lane_vec, topo_vec;
  main_lane_vec.x() = main_lane_end.x - main_lane_second_end.x;
  main_lane_vec.y() = main_lane_end.y - main_lane_second_end.y;
  topo_vec.x() = temp_lane_1_start.x - main_lane_end.x;
  topo_vec.y() = temp_lane_1_start.y - main_lane_end.y;
  double start_end_val = topo_vec.dot(main_lane_vec) / main_lane_vec.norm();
# ifdef AddTopoDebug
  AINFO << "main_lane_vec: " << main_lane_vec;
  AINFO << "topo_vec: " << topo_vec;
  AINFO << "start_end_val: " << start_end_val;
# endif
  
  if (start_end_val < MinBackThreshold){
    topo_debug_info["memory turn back constraint"] = false;
    return break_lane_ids;
  }

  // 实际拓扑lane的位置和地图匹配位置校验
  topo_debug_info["memory split position constraint"] = true;
  ptrdiff_t offset = split_point - target_lane->line_points.begin();
  auto &origin_lane_geos = *(target_lane->geos);
  auto split_point_geos = origin_lane_geos.begin() + offset;
  auto main_lane_geos = std::make_shared<std::vector<Eigen::Vector2f>>(origin_lane_geos.begin(), split_point_geos);
  auto temp_lane1_geos = std::make_shared<std::vector<Eigen::Vector2f>>(split_point_geos, origin_lane_geos.end());
  SplitTopoExtendType temp_lane1_type = static_cast<SplitTopoExtendType>(target_record.add_lane_type);
  SplitTopoExtendType topo_lane_type = static_cast<SplitTopoExtendType>(target_record.other_lane_type);

  auto topo_lane_geos = topo_lane->geos;
  if (!CheckMemorySplitLanePositionConstraint(temp_lane1_type, topo_lane_type, *temp_lane1_geos, *topo_lane_geos)){
    topo_debug_info["memory split position constraint"] = false;
    return break_lane_ids;
  }

  // 打断车道中心线，改变中心线拓扑关系
  uint64_t new_id = 101;
  for (uint64_t i = 1; i < 100UL; ++i) {
    auto tmp =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                      [i](const cem::message::sensor::BevLaneInfo& lane) {
                        return lane.id == i;
                      });
    if (tmp == data_->lane_infos.end()) {
      new_id = i;
      break;
    }
  }
  auto breaked_lane = *target_lane;
  breaked_lane.line_points.clear();
  breaked_lane.line_points.insert(breaked_lane.line_points.end(), split_point,
                                  target_lane->line_points.end());
  breaked_lane.number_of_points = breaked_lane.line_points.size();
  breaked_lane.id = new_id;

  target_lane->line_points.erase(split_point, target_lane->line_points.end());
  target_lane->number_of_points = target_lane->line_points.size();
  target_lane->geos = main_lane_geos;
  target_lane->indexed_geos.BuildIndex(target_lane->geos);
  target_lane->next_lane_ids.clear();
  target_lane->next_lane_ids.push_back(new_id);
  target_lane->next_lane_ids.push_back(topo_lane->id);

  breaked_lane.previous_lane_ids.clear();
  breaked_lane.previous_lane_ids.push_back(target_lane->id);
  topo_lane->previous_lane_ids.push_back(target_lane->id);

  breaked_lane.geos = temp_lane1_geos;
  breaked_lane.indexed_geos.BuildIndex(target_lane->geos);

  breaked_lane.is_build_split_marker_by_map = true;
  target_lane->is_build_split_marker_by_map = true;
  topo_lane->is_build_split_marker_by_map = true;  

  breaked_lane.split_topo_extend = temp_lane1_type;
  target_lane->split_topo_extend = static_cast<SplitTopoExtendType>(target_record.break_lane_type);
  topo_lane->split_topo_extend = topo_lane_type;

  if (target_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE &&
      topo_lane->split_topo_extend == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE) {
    breaked_lane.is_build_split_marker_by_map = false;
    target_lane->is_build_split_marker_by_map = false;
    topo_lane->is_build_split_marker_by_map = false;
  }  

  // AINFO << "breaked line start x: " << breaked_lane.line_points.front().x;
  if (breaked_lane.line_points.front().x < 0) {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
  } 
  else {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
  }

  // 假如原始target_lane存在后继，修改后继lane的前继，将其改为打断后的
  for (const auto& next_lane_id : breaked_lane.next_lane_ids) {
    auto target_lane_next = std::find_if(
        data_->lane_infos.begin(), data_->lane_infos.end(),
        [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
          return next_lane_id == lane.id;
        });
    if (target_lane_next == data_->lane_infos.end()) {
      continue;
    }
    std::replace(target_lane_next->previous_lane_ids.begin(),
                 target_lane_next->previous_lane_ids.end(), target_lane->id,
                 breaked_lane.id);
  }

  // 打断 lane_marker
  // 打断左边lane_marker
  auto id = target_lane->left_lane_marker_id;
  auto lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto left_lane_marker_first = *lane_line;
      auto left_lane_marker_second = *lane_line;
      left_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_split_point;
      for (auto iter = left_lane_marker_first.line_points.begin();
            iter != left_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (split_point->x - iter->x) * (split_point->x - iter->x) +
            (split_point->y - iter->y) * (split_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_split_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        left_lane_marker_second.line_points.insert(
            left_lane_marker_second.line_points.end(), marker_split_point,
            left_lane_marker_first.line_points.end());
        left_lane_marker_second.number_of_points =
            left_lane_marker_second.line_points.size();
        left_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.left_lane_marker_id = left_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_second));

        left_lane_marker_first.line_points.erase(
            marker_split_point, left_lane_marker_first.line_points.end());
        left_lane_marker_first.number_of_points =
            left_lane_marker_first.line_points.size();
        left_lane_marker_first.id = generate_lanemarker_id();
        target_lane->left_lane_marker_id = left_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_first));
      }
    }
  }

  // 打断左边右侧lane_marker
  id = target_lane->right_lane_marker_id;
  lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto right_lane_marker_first = *lane_line;
      auto right_lane_marker_second = *lane_line;
      right_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_split_point;
      for (auto iter = right_lane_marker_first.line_points.begin();
            iter != right_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (split_point->x - iter->x) * (split_point->x - iter->x) +
            (split_point->y - iter->y) * (split_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_split_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        right_lane_marker_second.line_points.insert(
            right_lane_marker_second.line_points.end(), marker_split_point,
            right_lane_marker_first.line_points.end());
        right_lane_marker_second.number_of_points =
            right_lane_marker_second.line_points.size();
        right_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.right_lane_marker_id = right_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_second));

        right_lane_marker_first.line_points.erase(
            marker_split_point, right_lane_marker_first.line_points.end());
        right_lane_marker_first.number_of_points =
            right_lane_marker_first.line_points.size();
        right_lane_marker_first.id = generate_lanemarker_id();
        target_lane->right_lane_marker_id = right_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_first));
      }
    }
  }

  target_lane->is_build_split_by_map   = true;
  topo_lane->is_build_split_by_map   = true;
  breaked_lane.is_build_split_by_map   = true;

  data_->lanemarkers_num = data_->lanemarkers.size();
  data_->lane_infos.push_back(std::move(breaked_lane));

  BreakLaneResult break_lane_result = {target_lane->id, topo_lane->id, breaked_lane.id, static_cast<int>(target_lane->split_topo_extend), static_cast<int>(topo_lane->split_topo_extend), static_cast<int>(breaked_lane.split_topo_extend)};
  break_lane_ids = break_lane_result;
  topo_debug_info["add memory split topo"] = true;
  return break_lane_ids;
}

void BevMapProcessor::AddMergeByHistory(uint64_t target_lane_id, json &topo_debug_info) {
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离

  // 根据输入id查询有没有对应的拓扑
  BevMapProcessor::TopoRecord target_record;
  target_record.record_number = 0;
  for (const auto& pair : merge_memory_result_) {
    const BevMapProcessor::TopoRecord& topo_record = pair.second;
    if (target_lane_id == topo_record.main_lane_id){
      target_record = topo_record;
      break;
    }

    if (topo_record.sub_lane_ids.count(target_lane_id)){
      target_record = topo_record;
      break;
    }
  }

  auto record_number =  target_record.record_number;
   // 记录数量 < 3，不添加拓扑
  if (record_number < 3) {
    return;
  }
  auto main_lane_id = target_record.main_lane_id;
  auto sub_lane_ids = target_record.sub_lane_ids;

  auto main_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [main_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return main_lane_id == lane.id; });

  if (main_lane == data_->lane_infos.end()) {
    return;
  }

  if (!main_lane->previous_lane_ids.empty()) {
    return;
  }

  // 记录数量 >= 3，使用记忆拓扑
  std::vector <std::vector<cem::message::sensor::BevLaneInfo>::iterator> sub_lanes;
  for (const auto & sub_lane_id : sub_lane_ids){
    auto sub_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [sub_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return sub_lane_id == lane.id; });
    if (sub_lane == data_->lane_infos.end()) {
      continue;
    }

    if (merge_memory_type_.find(sub_lane_id) == merge_memory_type_.end()) {
      continue;
    }

    sub_lanes.push_back(sub_lane);
  }

  if (sub_lanes.size() != sub_lane_ids.size()){
    return;
  }

   if (main_lane->line_points.size() < 2) {
    return;
  }

  // 判断是否存在回折，存在回折不添加拓扑
  for (const auto & sub_lane : sub_lanes) {
    auto sub_lane_end = sub_lane->line_points.back();
    auto main_lane_start = main_lane->line_points.front();
    auto main_lane_second_start = main_lane->line_points[1];
    Eigen::Vector2d main_lane_vec, topo_vec1;
    main_lane_vec.x() = main_lane_second_start.x - main_lane_start.x;
    main_lane_vec.y() = main_lane_second_start.y - main_lane_start.y;
    topo_vec1.x() = main_lane_start.x - sub_lane_end.x;
    topo_vec1.y() = main_lane_start.y - sub_lane_end.y;
    double end_start_val1 = topo_vec1.dot(main_lane_vec) / main_lane_vec.norm();
    
    if (end_start_val1 < MinBackThreshold){
      // AINFO << "sub start 1 to end: " << end_start_val1;
      topo_debug_info["turn back constraint"] = false;
      return;
    }
  }

  // 添加main_lane 后继
  main_lane->previous_lane_ids.insert(main_lane->previous_lane_ids.begin(), sub_lane_ids.begin(), sub_lane_ids.end());

  for (const auto & sub_lane : sub_lanes){
    auto it = std::find(sub_lane->next_lane_ids.begin(), sub_lane->next_lane_ids.end(), main_lane_id);
    if (it != sub_lane->next_lane_ids.end()) {
      continue;
    }

    sub_lane->next_lane_ids.push_back(main_lane_id);
    sub_lane->merge_topo_extend = merge_memory_type_.at(sub_lane->id);
  }

  topo_debug_info["add memory merge topo"] = true;
}

std::optional<BevMapProcessor::BreakLaneResult> BevMapProcessor::AddMergeBreakByHistory(uint64_t input_lane_id, json &topo_debug_info) {
  constexpr double MaxDistSqMarker = 10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MoveDistanceThreshold = 8.0;  // 目标添加拓扑点的最小距离（车道标志线）
  constexpr double MinBackThreshold = -5.0;  // 目标添加拓扑车道线起点回折最小距离
  constexpr double MinEndThreshold = 12.0;  // 目标断点到终点的距离阈值
  
  // 定义子函数，用于生成新的车道标记ID
  auto generate_lanemarker_id = [&]() {
    uint32_t new_id = 101;
    for (uint32_t i = 1; i < 100UL; ++i) {
      auto tmp = std::find_if(
          data_->lanemarkers.begin(), data_->lanemarkers.end(),
          [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return lane_marker.id == i;
          });
      if (tmp == data_->lanemarkers.end()) {
        new_id = i;
        break;
      }
    }
    return new_id;
  };

  std::optional<BreakLaneResult> break_lane_ids;

  // 根据输入id查询有没有对应的拓扑
  BevMapProcessor::BreakTopoRecord target_record;
  target_record.record_number = 0;
  for (const auto& pair : merge_memory_break_result_) {
    const BevMapProcessor::BreakTopoRecord& topo_record = pair.second;
    if (input_lane_id == topo_record.break_lane_id || input_lane_id == topo_record.other_lane_id){
      target_record = topo_record;
      break;
    }
  }

  auto record_number =  target_record.record_number;
   // 记录数量 < 3，不添加拓扑
  if (record_number < 3) {
    return break_lane_ids;
  }

  auto target_lane_id = target_record.break_lane_id;
  auto topo_lane_id = target_record.other_lane_id;

  auto target_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                        [target_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id == lane.id; });

  if (target_lane == data_->lane_infos.end()) {
    return break_lane_ids;
  }

  if (!target_lane->next_lane_ids.empty()) {
    return break_lane_ids;
  }

  auto topo_lane = std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                              [topo_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return topo_lane_id == lane.id; });

  if (topo_lane == data_->lane_infos.end()){
    return break_lane_ids;
  }

  if (target_lane->line_points.size() < 4) {
    return break_lane_ids;
  }

  std::vector<cem::message::common::Point2DF>::iterator merge_point1;
  double distance_pt2seg = std::numeric_limits<double>::max();
  // 打断备选规则2：找到距离最近点，向前挪动8m（暂定），如果打断点数不够就找最远的点
  for (auto iter = target_lane->line_points.begin() + 1;
            iter != target_lane->line_points.end(); ++iter) {
    // 求与输入的车道line_segment悬空车道起点的最近点
    auto temp_start = *(iter - 1);
    auto temp_end = *iter;
    double tmp_dist = CalculatePoint2SegmentDist(topo_lane->line_points.back(), temp_start, temp_end);
    if (distance_pt2seg > tmp_dist) {
      merge_point1 = iter;
      distance_pt2seg = tmp_dist;
    }
  }

  // 校验结果split_point1是否可以分出足够数量的行点
  int break_lane1_pts_num = merge_point1 - target_lane->line_points.begin();
  int break_lane2_pts_num = target_lane->line_points.end() - merge_point1;
  // AINFO << "split_point1 constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_lane_ids;
  }

  // 基于当前的split_point前移8m
  std::vector<cem::message::common::Point2DF>::iterator merge_point = merge_point1;
  for (auto iter = merge_point1;
            iter != target_lane->line_points.end() - 2; ++iter) {
    auto temp_point = *iter;
    double tmp_dist = CalculatePoint2PointDist(*merge_point1, temp_point);
    merge_point = iter;
    if (tmp_dist > MoveDistanceThreshold) {
      break;
    }
  }

  // AINFO << "*******MergePointResultModify******";
  // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  // AINFO << "sequence number: " << data_->header.cycle_counter;
  // AINFO << "merge point: (" << merge_point1->x << ", " << merge_point1->y << ")";
  // AINFO << "merge point: (" << merge_point->x << ", " << merge_point->y << ")";
  // AINFO << "*******************************";

  // if (!get_split_point){
  //   return break_success_flag;
  // }

  // 校验结果split_point是否可以分出足够数量的行点
  break_lane1_pts_num = merge_point - target_lane->line_points.begin();
  break_lane2_pts_num = target_lane->line_points.end() - merge_point;
  // AINFO << "split_point constraint: " << break_lane1_pts_num << ", " << break_lane2_pts_num;
  if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
    return break_lane_ids;
  }

  // 打断产生的距离超过阈值约束
  double end_point_val = 0;
  for (auto iter = merge_point;
            iter < target_lane->line_points.end() - 1; ++iter) {
    auto temp_point1 = *iter;
    auto temp_point2 = *(iter+1);
    end_point_val += CalculatePoint2PointDist(temp_point1, temp_point2);
  }
  if (end_point_val < MinEndThreshold){
    // AINFO << "break_point-end constraint: " << end_point_val;
    topo_debug_info["break_point-end constraint"] = false;
    return break_lane_ids;
  }

  // 校验结果merge_point是否出现回折
  auto temp_lane_1_end = topo_lane->line_points.front();
  auto main_lane_start = *(merge_point);
  auto main_lane_second_start = *(merge_point + 1);
  Eigen::Vector2d main_lane_vec, topo_vec;
  main_lane_vec.x() = main_lane_second_start.x - main_lane_start.x;
  main_lane_vec.y() = main_lane_second_start.y - main_lane_start.y;
  topo_vec.x() = main_lane_start.x - temp_lane_1_end.x;
  topo_vec.y() = main_lane_start.y - temp_lane_1_end.y;
  double end_start_val = topo_vec.dot(main_lane_vec) / main_lane_vec.norm();
# ifdef AddTopoDebug
  AINFO << "main_lane_vec: " << main_lane_vec;
  AINFO << "topo_vec: " << topo_vec;
  // AINFO << "start_end_val: " << start_end_val;
# endif
  
  if (end_start_val < MinBackThreshold){
    topo_debug_info["turn back constraint"] = false;
    return break_lane_ids;
  }

  // 实际拓扑lane的位置和地图匹配位置校验
  topo_debug_info["merge position constraint"] = true;
  ptrdiff_t offset = merge_point - target_lane->line_points.begin();
  auto &origin_lane_geos = *(target_lane->geos);
  auto merge_point_geos = origin_lane_geos.begin() + offset;
  auto temp_lane1_geos = std::make_shared<std::vector<Eigen::Vector2f>>(origin_lane_geos.begin(), merge_point_geos);
  auto main_lane_geos = std::make_shared<std::vector<Eigen::Vector2f>>(merge_point_geos, origin_lane_geos.end());
  auto topo_lane_geos = topo_lane->geos;
  MergeTopoExtendType temp_lane1_type = static_cast<MergeTopoExtendType>(target_record.break_lane_type);
  MergeTopoExtendType topo_lane_type = static_cast<MergeTopoExtendType>(target_record.other_lane_type);

  if (!CheckMemoryMergeLanePositionConstraint(temp_lane1_type, topo_lane_type, *temp_lane1_geos, *topo_lane_geos)){
    topo_debug_info["merge position constraint"] = false;
    return break_lane_ids;
  }

  // 打断车道中心线，改变中心线拓扑关系
  uint64_t new_id = 101;
  for (uint64_t i = 1; i < 100UL; ++i) {
    auto tmp =
        std::find_if(data_->lane_infos.begin(), data_->lane_infos.end(),
                      [i](const cem::message::sensor::BevLaneInfo& lane) {
                        return lane.id == i;
                      });
    if (tmp == data_->lane_infos.end()) {
      new_id = i;
      break;
    }
  }
  auto breaked_lane = *target_lane;
  breaked_lane.line_points.clear();
  breaked_lane.line_points.insert(breaked_lane.line_points.end(), merge_point,
                                  target_lane->line_points.end());
  breaked_lane.number_of_points = breaked_lane.line_points.size();
  breaked_lane.id = new_id;

  target_lane->line_points.erase(merge_point, target_lane->line_points.end());
  target_lane->number_of_points = target_lane->line_points.size();
  target_lane->geos = temp_lane1_geos;
  target_lane->indexed_geos.BuildIndex(target_lane->geos);
  target_lane->next_lane_ids.clear();
  target_lane->next_lane_ids.push_back(new_id);
  topo_lane->next_lane_ids.push_back(new_id);

  breaked_lane.previous_lane_ids.clear();
  breaked_lane.previous_lane_ids.push_back(target_lane->id);
  breaked_lane.previous_lane_ids.push_back(topo_lane->id);

  breaked_lane.geos = main_lane_geos;
  breaked_lane.indexed_geos.BuildIndex(target_lane->geos);
  
  breaked_lane.is_build_merge_marker_by_map = true;
  target_lane->is_build_merge_marker_by_map = true;
  topo_lane->is_build_merge_marker_by_map = true;

  breaked_lane.merge_topo_extend = static_cast<MergeTopoExtendType>(target_record.add_lane_type);
  target_lane->merge_topo_extend = temp_lane1_type;
  topo_lane->merge_topo_extend = topo_lane_type;

  if (target_lane->merge_topo_extend == MergeTopoExtendType::TOPOLOGY_MERGE_NONE &&
      topo_lane->merge_topo_extend == MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
    breaked_lane.is_build_merge_marker_by_map = false;
    target_lane->is_build_merge_marker_by_map = false;
    topo_lane->is_build_merge_marker_by_map = false;
  }

  if (breaked_lane.line_points.front().x < 0) {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
  } 
  else {
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
    target_lane->position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO);
  }

  // 假如原始target_lane存在后继，修改后继lane的前继，将其改为打断后的
  for (const auto& next_lane_id : breaked_lane.next_lane_ids) {
    auto target_lane_next = std::find_if(
        data_->lane_infos.begin(), data_->lane_infos.end(),
        [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
          return next_lane_id == lane.id;
        });
    if (target_lane_next == data_->lane_infos.end()) {
      continue;
    }
    std::replace(target_lane_next->previous_lane_ids.begin(),
                 target_lane_next->previous_lane_ids.end(), target_lane->id,
                 breaked_lane.id);
  }

  // 打断 lane_marker
  // 打断左边lane_marker
  auto id = target_lane->left_lane_marker_id;
  auto lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto left_lane_marker_first = *lane_line;
      auto left_lane_marker_second = *lane_line;
      left_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_merge_point;
      for (auto iter = left_lane_marker_first.line_points.begin();
            iter != left_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (merge_point->x - iter->x) * (merge_point->x - iter->x) +
            (merge_point->y - iter->y) * (merge_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_merge_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        left_lane_marker_second.line_points.insert(
            left_lane_marker_second.line_points.end(), marker_merge_point,
            left_lane_marker_first.line_points.end());
        left_lane_marker_second.number_of_points =
            left_lane_marker_second.line_points.size();
        left_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.left_lane_marker_id = left_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_second));

        left_lane_marker_first.line_points.erase(
            marker_merge_point, left_lane_marker_first.line_points.end());
        left_lane_marker_first.number_of_points =
            left_lane_marker_first.line_points.size();
        left_lane_marker_first.id = generate_lanemarker_id();
        target_lane->left_lane_marker_id = left_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(left_lane_marker_first));
      }
    }
  }

  // 打断左边右侧lane_marker
  id = target_lane->right_lane_marker_id;
  lane_line = std::find_if(
      data_->lanemarkers.begin(), data_->lanemarkers.end(),
      [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
        return id == lane_marker.id;
      });
  if (lane_line != data_->lanemarkers.end()) {
    if (!lane_line->line_points.empty()) {
      auto right_lane_marker_first = *lane_line;
      auto right_lane_marker_second = *lane_line;
      right_lane_marker_second.line_points.clear();

      double distance_square = std::numeric_limits<double>::max();
      std::vector<cem::message::common::Point2DF>::iterator
          marker_merge_point;
      for (auto iter = right_lane_marker_first.line_points.begin();
            iter != right_lane_marker_first.line_points.end(); ++iter) {
        double tmp_square =
            (merge_point->x - iter->x) * (merge_point->x - iter->x) +
            (merge_point->y - iter->y) * (merge_point->y - iter->y);
        if (distance_square > tmp_square) {
          marker_merge_point = iter;
          distance_square = tmp_square;
        }
      }

      if (distance_square < MaxDistSqMarker) {
        right_lane_marker_second.line_points.insert(
            right_lane_marker_second.line_points.end(), marker_merge_point,
            right_lane_marker_first.line_points.end());
        right_lane_marker_second.number_of_points =
            right_lane_marker_second.line_points.size();
        right_lane_marker_second.id = generate_lanemarker_id();
        breaked_lane.right_lane_marker_id = right_lane_marker_second.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_second));

        right_lane_marker_first.line_points.erase(
            marker_merge_point, right_lane_marker_first.line_points.end());
        right_lane_marker_first.number_of_points =
            right_lane_marker_first.line_points.size();
        right_lane_marker_first.id = generate_lanemarker_id();
        target_lane->right_lane_marker_id = right_lane_marker_first.id;
        data_->lanemarkers.push_back(std::move(right_lane_marker_first));
      }
    }
  }

  data_->lanemarkers_num = data_->lanemarkers.size();
  data_->lane_infos.push_back(std::move(breaked_lane));

  BreakLaneResult break_lane_result = {target_lane->id, topo_lane->id, breaked_lane.id, static_cast<int>(target_lane->merge_topo_extend), static_cast<int>(topo_lane->merge_topo_extend), static_cast<int>(breaked_lane.merge_topo_extend)};
  break_lane_ids = break_lane_result;
  topo_debug_info["add memory merge topo"] = true;
  return break_lane_ids;
}

bool BevMapProcessor::CheckMainlaneDifferenceConstraint(cem::message::sensor::BevLaneInfo* main_lane,
                                                        const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                                        const std::vector<uint64_t>& main_ld_match,
                                                        const std::vector<uint64_t>& sub_ld_match,
                                                        bool select_multi_ld,
                                                        bool is_split_flag) {
  constexpr double DistDiffThreshold = 1.0;  // 感知的mainlane和其匹配地图lanegroup的距离阈值
  constexpr double DistMatchThreshold = 0.5;  // topo_dist_to_ego匹配的阈值

  // 措施：使用接口提供的main_ld匹配信息
  if (main_ld_match.empty()){
    return false;
  }

  std::vector<uint64_t> match_ids;
  if (is_split_flag) {
    if (select_multi_ld) {
      if (sub_ld_match.empty()){
        return false;
      }
      match_ids = {main_ld_match.back(), sub_ld_match.front()};
    }
    else {
      match_ids = {main_ld_match.back()};
    }
  } 
  else {
    if (select_multi_ld) {
      match_ids = {sub_ld_match.back(), main_ld_match.front()};
    }
    else {
      match_ids = {main_ld_match.front()};
    }
  }
  
# ifdef AddTopoDebug
  AINFO << "*******MatchMainLane******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "BEV lane id: " << main_lane->id;
  std::stringstream lane_id_str1;
  for (const auto& temp_ld_lane_id : match_ids) {
    lane_id_str1 << temp_ld_lane_id << ", ";
  }
  AINFO << "lane start and end: " << lane_id_str1.str();
  AINFO << "**************************";
# endif

  std::vector<Eigen::Vector2f> ld_geos;
  for (const auto & temp_ld_lane_id : match_ids) {
    auto temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });

    if (temp_ld_lane == ld_map->lanes.end()) {
      continue;
    }
    
    for (const auto& point : temp_ld_lane->points) {
        Eigen::Vector2f eigenPoint(static_cast<float>(point.x), static_cast<float>(point.y));
        ld_geos.push_back(eigenPoint);
    }
  }

  std::pair<float, float> match_diff_pair = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
  std::vector<Eigen::Vector2f> mainlane_pts = *(main_lane->geos);
  bool match_distance_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(ld_geos, mainlane_pts, match_diff_pair);

  if (!match_distance_valid) {
    return false;
  } 
  else {
    // AINFO << "match_diff: " << match_diff_pair.first;
    if (match_diff_pair.first > DistDiffThreshold) {
      return false;
    }
    else{
      return true;
    }
  }

}

bool BevMapProcessor::CheckSublaneDifferenceConstraint(cem::message::sensor::BevLaneInfo* sub_lane,
                                                const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                                const std::vector<uint64_t>& sub_ld_match, 
                                                const std::vector<uint64_t>& main_ld_match, 
                                                bool is_split_flag, 
                                                bool is_match_flag) {
  constexpr double DistMatchMeanDiffThreshold = 2.0;  // 感知匹配上的sublane和其匹配地图lanegroup的平均距离阈值
  constexpr double DistNoMatchMeanDiffThreshold = 4.0;  // 感知的mainlane和其匹配地图lanegroup的平均距离阈值
  constexpr double DistMatchMaxDiffThreshold = 3.5;  // 感知匹配上的sublane和其匹配地图lanegroup的最大距离阈值
  constexpr double DistNoMatchMaxDiffThreshold = 5.5;  // 感知的mainlane和其匹配地图lanegroup的最大距离阈值
  constexpr double LDLengthThreshold = 20.0; // ld选取的长度阈值，如果不足需要向后取一个section的lane

  // 措施：使用接口提供的sub_ld匹配信息
  if (is_match_flag) {
    if (sub_ld_match.empty()){
      return false;
    }

    std::vector<cem::message::env_model::LaneInfo>::iterator temp_ld_lane;
    uint64_t temp_ld_lane_id;
    std::vector<std::vector<cem::message::env_model::LaneInfo>::iterator> match_lanes;
    if (is_split_flag) {
      temp_ld_lane_id = sub_ld_match.front();
      temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                  [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });
      if (temp_ld_lane == ld_map->lanes.end()) {
        return false;
      }

      match_lanes = {temp_ld_lane};
      if (sub_ld_match.size() >= 2 && temp_ld_lane->length < LDLengthThreshold && temp_ld_lane->next_lane_ids.size() == 1) {
        temp_ld_lane_id = temp_ld_lane->next_lane_ids.front();
        temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                  [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });
        if (temp_ld_lane != ld_map->lanes.end()) {
          match_lanes.push_back(temp_ld_lane);
        }
      }
    }
    else {
      temp_ld_lane_id = sub_ld_match.back();
      temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                  [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });
      if (temp_ld_lane == ld_map->lanes.end()) {
        return false;
      }

      match_lanes = {temp_ld_lane};
      if (sub_ld_match.size() >= 2 && temp_ld_lane->length < LDLengthThreshold && temp_ld_lane->previous_lane_ids.size() == 1) {
        temp_ld_lane_id = temp_ld_lane->previous_lane_ids.front();
        temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                  [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });
        if (temp_ld_lane != ld_map->lanes.end()) {
          match_lanes.insert(match_lanes.begin(), temp_ld_lane);;
        }
      }
    }

  # ifdef AddTopoDebug
    AINFO << "*******MatchSubLane******";
    AINFO << "sequence number: " << data_->header.cycle_counter;
    std::stringstream lane_id_str1;
    for (const auto& ld_lane : match_lanes) {
      lane_id_str1 << ld_lane->id << ", ";
    }
    AINFO << "lane start and end: " << lane_id_str1.str();
    AINFO << "**************************";
  # endif

    std::vector<Eigen::Vector2f> ld_geos;
    for (const auto & temp_ld_lane : match_lanes) {
      for (const auto& point : temp_ld_lane->points) {
          Eigen::Vector2f eigenPoint(static_cast<float>(point.x), static_cast<float>(point.y));
          ld_geos.push_back(eigenPoint);
      }
    }

    std::pair<float, float> match_diff_pair = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    std::vector<Eigen::Vector2f> sublane_pts = *(sub_lane->geos);
    bool match_distance_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(ld_geos, sublane_pts, match_diff_pair);

    if (!match_distance_valid) {
      return false;
    } 
    else {
      // AINFO << "match_diff: " << match_diff_pair.first << ", " << match_diff_pair.second;
      if (match_diff_pair.first <= DistMatchMeanDiffThreshold && match_diff_pair.second <= DistMatchMaxDiffThreshold) {
        return true;
      }
      else{
        return false;
      }
    }
  }
  else {
    if (main_ld_match.empty()){
      return false;
    }

    uint64_t main_ld_id;
    if (is_split_flag) {
      main_ld_id = main_ld_match.back();
    }
    else {
      main_ld_id = main_ld_match.front();
    }

    auto main_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                      [main_ld_id](const cem::message::env_model::LaneInfo &lane) { return main_ld_id == lane.id; });
    if (main_ld_lane == ld_map->lanes.end()) {
      return false;
    }

    std::vector<uint64_t> temp_ld_lane_ids;
    if (is_split_flag) {
      temp_ld_lane_ids = main_ld_lane->next_lane_ids;
    }
    else {
      temp_ld_lane_ids = main_ld_lane->previous_lane_ids;
    }

  # ifdef AddTopoDebug
    AINFO << "*******MatchSubLane******";
    AINFO << "sequence number: " << data_->header.cycle_counter;
    std::stringstream lane_id_str1;
    for (const auto& temp_ld_lane_id : temp_ld_lane_ids) {
      lane_id_str1 << temp_ld_lane_id << ", ";
    }
    AINFO << "lane start and end: " << lane_id_str1.str();
    AINFO << "**************************";
  # endif
    for (const auto & temp_ld_lane_id : temp_ld_lane_ids) {
      std::vector<Eigen::Vector2f> ld_geos;

      auto temp_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [temp_ld_lane_id](const cem::message::env_model::LaneInfo &lane) { return temp_ld_lane_id == lane.id; });

      if (temp_ld_lane == ld_map->lanes.end()) {
        continue;
      }
      
      for (const auto& point : temp_ld_lane->points) {
          Eigen::Vector2f eigenPoint(static_cast<float>(point.x), static_cast<float>(point.y));
          ld_geos.push_back(eigenPoint);
      }

      std::pair<float, float> match_diff_pair = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
      std::vector<Eigen::Vector2f> sublane_pts = *(sub_lane->geos);
      bool match_distance_valid = cem::fusion::LaneGeometry::CalculateAverageDistanceBetweenTwoLines(ld_geos, sublane_pts, match_diff_pair);

      if (!match_distance_valid) {
        continue;
      } 
      // AINFO << "match_diff: " << match_diff_pair.first;
      if (match_diff_pair.first <= DistNoMatchMeanDiffThreshold && match_diff_pair.second <= DistNoMatchMaxDiffThreshold) {
        return true;
      }
    }
  }
  return false;
}

bool BevMapProcessor::CheckSplitLanePositionConstraint (LdMapProcessor::MergeSplitType match_type,
                                         LdMapProcessor::MergeSplitType lane1_type,
                                         LdMapProcessor::MergeSplitType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos) {
  // 打印input：查看是否正确
# ifdef AddTopoDebug
  AINFO << "*******LanePositionInput******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "match_type: " << static_cast<int>(match_type) << "; lane1_type: " << static_cast<int>(lane1_type) << "; lane2_type: " << static_cast<int>(lane2_type);
  AINFO << "lane1 start end points: " << lane1_geos.front() << "; " << lane1_geos.back();
  AINFO << "lane2 start end points: " << lane2_geos.front() << "; " << lane2_geos.back();
  AINFO << "************************";
# endif
  
  // 边缘情况：match_type = connect，不满足条件
  if (match_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    return false;
  }

  // 边缘情况：lane1_type = connect and lane2_type = connect，不满足条件
  if (lane1_type == LdMapProcessor::MergeSplitType::kConnectForward &&
      lane2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    return false;
  }
  
  if (lane1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    if (lane2_type == LdMapProcessor::MergeSplitType::kSplitLeft) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }

    else if (lane2_type == LdMapProcessor::MergeSplitType::kSplitRight) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }
  } 
  
  else if (lane2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    if (lane1_type == LdMapProcessor::MergeSplitType::kSplitLeft) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }

    else if (lane1_type == LdMapProcessor::MergeSplitType::kSplitRight) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }
  } 
  
  return false;
}

bool BevMapProcessor::CheckMemorySplitLanePositionConstraint (SplitTopoExtendType lane1_type,
                                         SplitTopoExtendType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos) {
  // 打印input：查看是否正确
# ifdef AddTopoDebug
  AINFO << "*******LanePositionInput******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "match_type: " << static_cast<int>(match_type) << "; lane1_type: " << static_cast<int>(lane1_type) << "; lane2_type: " << static_cast<int>(lane2_type);
  AINFO << "lane1 start end points: " << lane1_geos.front() << "; " << lane1_geos.back();
  AINFO << "lane2 start end points: " << lane2_geos.front() << "; " << lane2_geos.back();
  AINFO << "************************";
# endif

  // 边缘情况：lane1_type = connect and lane2_type = connect，不满足条件
  if ((lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE || lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN) &&
      (lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE || lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN)) {
    return false;
  }
  
  if (lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE || lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN) {
    if (lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }

    else if (lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }
  } 
  
  else if ((lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_NONE || lane2_type == SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN)) {
    if (lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }

    else if (lane1_type == SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }
  } 
  
  return false;
}

bool BevMapProcessor::CheckMergeLanePositionConstraint (LdMapProcessor::MergeSplitType match_type,
                                         LdMapProcessor::MergeSplitType lane1_type,
                                         LdMapProcessor::MergeSplitType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos) {
  // 打印input：查看是否正确
# ifdef AddTopoDebug
  AINFO << "*******LanePositionInput******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "match_type: " << static_cast<int>(match_type) << "; lane1_type: " << static_cast<int>(lane1_type) << "; lane2_type: " << static_cast<int>(lane2_type);
  AINFO << "lane1 start end points: " << lane1_geos.front() << "; " << lane1_geos.back();
  AINFO << "lane2 start end points: " << lane2_geos.front() << "; " << lane2_geos.back();
  AINFO << "************************";
# endif
  
  // 边缘情况：match_type = connect，不满足条件
  if (match_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    return false;
  }

  // 边缘情况：lane1_type = connect and lane2_type = connect，不满足条件
  if (lane1_type == LdMapProcessor::MergeSplitType::kConnectForward &&
      lane2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    return false;
  }
  
  if (lane1_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    if (lane2_type == LdMapProcessor::MergeSplitType::kMergeRight) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }

    else if (lane2_type == LdMapProcessor::MergeSplitType::kMergeLeft) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }
  } 
  
  else if (lane2_type == LdMapProcessor::MergeSplitType::kConnectForward) {
    if (lane1_type == LdMapProcessor::MergeSplitType::kMergeRight) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }

    else if (lane1_type == LdMapProcessor::MergeSplitType::kMergeLeft) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }
  } 
  
  return false;
}

bool BevMapProcessor::CheckMemoryMergeLanePositionConstraint (MergeTopoExtendType lane1_type,
                                         MergeTopoExtendType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos) {
  // 打印input：查看是否正确
# ifdef AddTopoDebug
  AINFO << "*******LanePositionInput******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "match_type: " << static_cast<int>(match_type) << "; lane1_type: " << static_cast<int>(lane1_type) << "; lane2_type: " << static_cast<int>(lane2_type);
  AINFO << "lane1 start end points: " << lane1_geos.front() << "; " << lane1_geos.back();
  AINFO << "lane2 start end points: " << lane2_geos.front() << "; " << lane2_geos.back();
  AINFO << "************************";
# endif

  // 边缘情况：lane1_type = connect and lane2_type = connect，不满足条件
  if ((lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_NONE || lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN) &&
      (lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_NONE || lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN)) {
    return false;
  }
  
  if (lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_NONE || lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN) {
    if (lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_LEFT) {
      return LaneGeometry::JudgeIsLeft(lane1_geos, lane2_geos);
    }

    else if (lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }
  } 
  
  else if ((lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_NONE || lane2_type == MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN)) {
    if (lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_LEFT) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }

    else if (lane1_type == MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT) {
      return LaneGeometry::JudgeIsLeft(lane2_geos, lane1_geos);
    }
  } 
  
  return false;
}

std::optional<const BevSplitInfo*> BevMapProcessor::GetSplitInfoForTopoAdd(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter, cem::message::sensor::BevLaneInfo* main_lane, const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map) {
  auto candidates = LaneSplitGetter(main_lane->id);
  if (candidates.empty()) {
    return std::nullopt;
  }

# ifdef AddTopoDebug
  AINFO << "*******OriginSplitInfo******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  for (const auto* info : candidates) {
    AINFO << "bev_lane_from: " << info->bev_lane_from;
    AINFO << "distance_to_ego: " << info->distance_to_ego;
    AINFO << "match_type: " << info->match_type;
    std::stringstream lane_id_str;
      for (const auto& lane_id : info->map_lanes_from){
        lane_id_str << lane_id << ", ";
      }
    AINFO << "main_ld_map id: " << lane_id_str.str();
    for (const auto &bev_target_to : info->bev_target_to) {
      AINFO << "bev_lane_to: " << bev_target_to.second.bev_lane_to 
            << "; bev_lane_type: " << bev_target_to.second.type;
      std::stringstream lane_id_str;
      for (const auto& lane_id : bev_target_to.second.map_lanes_to){
        lane_id_str << lane_id << ", ";
      }
      AINFO << "sub_ld_map id: " << lane_id_str.str();
    }
  }
  AINFO << "************************";
# endif

  // 初始化最小绝对值和对应元素（默认首个元素）
  const BevSplitInfo* min_info = candidates[0];
  float min_abs_dist = std::abs(min_info->distance_to_ego);

  // 遍历所有元素，寻找绝对值最小的distance_to_ego
  for (const auto* info : candidates) {
    if (info == nullptr) continue;  // 跳过无效指针

    float current_abs_dist = std::abs(info->distance_to_ego);
    if (current_abs_dist < min_abs_dist) {
        // 更新最小值和对应元素
        min_abs_dist = current_abs_dist;
        min_info = info;
    }
  }

  if (min_info == nullptr) {
    return std::nullopt;
  }

  static std::unique_ptr<BevSplitInfo> dst_info;
  dst_info = std::make_unique<BevSplitInfo>(*min_info); // 深拷贝源数据到堆

  std::vector<std::pair<uint64_t, BevSplitTarget>> deduped_targets;
  std::set<uint64_t> seen_bev_lane_to;
  if (dst_info->map_lanes_from.empty()) {
    return std::nullopt;
  }
  uint64_t ld_main_id = dst_info->map_lanes_from.back();

  // 实现去重匹配结果与地图关系的校验
  for (const auto& target_pair : dst_info->bev_target_to) {
      const auto& bev_target = target_pair.second;
      uint64_t current_bev_lane_to = bev_target.bev_lane_to;
      if (bev_target.map_lanes_to.empty()) continue;

      uint64_t ld_sub_id = bev_target.map_lanes_to.front();
      auto topo_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [ld_sub_id](const cem::message::env_model::LaneInfo &lane) { return ld_sub_id == lane.id; }); 
      if (topo_ld_lane == ld_map->lanes.end()) continue;

      auto it = std::find(topo_ld_lane->previous_lane_ids.begin(), topo_ld_lane->previous_lane_ids.end(), ld_main_id);
      if (it == topo_ld_lane->previous_lane_ids.end()) continue;

      if (!CheckLDSplitTopoType(bev_target.type, topo_ld_lane->split_topology)) continue;

      if (seen_bev_lane_to.count(current_bev_lane_to) > 0) continue;

      deduped_targets.push_back(target_pair);
      seen_bev_lane_to.insert(current_bev_lane_to);
  }

  // 替换去去掉（修改堆上的对象）
  dst_info->bev_target_to = std::move(deduped_targets);

# ifdef AddTopoDebug
  AINFO << "*******ModifiedSplitInfo******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "bev_lane_from: " << dst_info->bev_lane_from;
  AINFO << "distance_to_ego: " << dst_info->distance_to_ego;
  AINFO << "match_type: " << dst_info->match_type;
  std::stringstream lane_id_str;
    for (const auto& lane_id : dst_info->map_lanes_from){
      lane_id_str << lane_id << ", ";
    }
  AINFO << "main_ld_map id: " << lane_id_str.str();
  for (const auto &bev_target_to : dst_info->bev_target_to) {
    AINFO << "bev_lane_to: " << bev_target_to.second.bev_lane_to 
          << "; bev_lane_type: " << bev_target_to.second.type;
    std::stringstream lane_id_str;
    for (const auto& lane_id : bev_target_to.second.map_lanes_to){
      lane_id_str << lane_id << ", ";
    }
    AINFO << "sub_ld_map id: " << lane_id_str.str();
  }
  AINFO << "************************";
# endif

  // 返回const智能指针（调用方无法修改指针指向，也无法修改对象内容）
  return dst_info.get();

}

std::optional<const BevMergeInfo*> BevMapProcessor::GetMergeInfoForTopoAdd (const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter, cem::message::sensor::BevLaneInfo* main_lane, const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map){
  auto candidates = LaneMergeGetter(main_lane->id, false);
  if (candidates.empty()) {
    return std::nullopt;
  }

# ifdef AddMergeDebug
  AINFO << "*******OriginMergeInfo******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  for (const auto* info : candidates) {
    AINFO << "bev_lane_from: " << info->bev_lane_to;
    AINFO << "distance_to_ego: " << info->distance_to_ego;
    AINFO << "match_type: " << info->match_type;
    std::stringstream lane_id_str;
      for (const auto& lane_id : info->map_lanes_to){
        lane_id_str << lane_id << ", ";
      }
    AINFO << "main_ld_map id: " << lane_id_str.str();
    for (const auto &bev_target_from : info->bev_lanes_from) {
      AINFO << "bev_lane_to: " << bev_target_from.second.bev_lane_from 
            << "; bev_lane_type: " << bev_target_from.second.type;
      std::stringstream lane_id_str;
      for (const auto& lane_id : bev_target_from.second.map_lanes_from){
        lane_id_str << lane_id << ", ";
      }
      AINFO << "sub_ld_map id: " << lane_id_str.str();
    }
  }
  AINFO << "************************";
# endif

  // 初始化最小绝对值和对应元素（默认首个元素）
  const BevMergeInfo* min_info = candidates[0];
  float min_abs_dist = std::abs(min_info->distance_to_ego);

  // 遍历所有元素，寻找绝对值最小的distance_to_ego
  for (const auto* info : candidates) {
    if (info == nullptr) continue;  // 跳过无效指针

    float current_abs_dist = std::abs(info->distance_to_ego);
    if (current_abs_dist < min_abs_dist) {
        // 更新最小值和对应元素
        min_abs_dist = current_abs_dist;
        min_info = info;
    }
  }

  if (min_info == nullptr) {
    return std::nullopt;
  }

  static std::unique_ptr<BevMergeInfo> dst_info;
  dst_info = std::make_unique<BevMergeInfo>(*min_info); // 深拷贝源数据到堆

  std::unordered_map<uint64_t, BevMergeSource> deduped_targets;
  std::set<uint64_t> seen_bev_lane_to;
  if (dst_info->map_lanes_to.empty()) {
    return std::nullopt;
  }
  uint64_t ld_main_id = dst_info->map_lanes_to.front();

  // 实现去重匹配结果与地图关系的校验
  for (const auto& target_pair : dst_info->bev_lanes_from) {
      const auto& bev_target = target_pair.second;
      uint64_t current_bev_lane_to = bev_target.bev_lane_from;
      if (bev_target.map_lanes_from.empty()) continue;

      uint64_t ld_sub_id = bev_target.map_lanes_from.back();
      auto topo_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [ld_sub_id](const cem::message::env_model::LaneInfo &lane) { return ld_sub_id == lane.id; }); 
      if (topo_ld_lane == ld_map->lanes.end()) continue;

      auto it = std::find(topo_ld_lane->next_lane_ids.begin(), topo_ld_lane->next_lane_ids.end(), ld_main_id);
      if (it == topo_ld_lane->next_lane_ids.end()) continue;

      if (!CheckLDMergeTopoType(bev_target.type, topo_ld_lane->merge_topology)) continue;

      if (seen_bev_lane_to.count(current_bev_lane_to) > 0) continue;

      deduped_targets[target_pair.first] = target_pair.second;
      seen_bev_lane_to.insert(current_bev_lane_to);
  }

  // 替换去去掉（修改堆上的对象）
  dst_info->bev_lanes_from = std::move(deduped_targets);

# ifdef AddTopoDebug
  AINFO << "*******ModifiedMergeInfo******";
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "bev_lane_from: " << dst_info->bev_lane_to;
  AINFO << "distance_to_ego: " << dst_info->distance_to_ego;
  AINFO << "match_type: " << dst_info->match_type;
  std::stringstream lane_id_str;
    for (const auto& lane_id : dst_info->map_lanes_to){
      lane_id_str << lane_id << ", ";
    }
  AINFO << "main_ld_map id: " << lane_id_str.str();
  for (const auto &bev_target_from : dst_info->bev_lanes_from) {
    AINFO << "bev_lane_to: " << bev_target_from.second.bev_lane_from 
          << "; bev_lane_type: " << bev_target_from.second.type;
    std::stringstream lane_id_str;
    for (const auto& lane_id : bev_target_from.second.map_lanes_from){
      lane_id_str << lane_id << ", ";
    }
    AINFO << "sub_ld_map id: " << lane_id_str.str();
  }
  AINFO << "************************";
# endif

  // 返回const智能指针（调用方无法修改指针指向，也无法修改对象内容）
  return dst_info.get();
}

bool BevMapProcessor::CheckTopoPointLongitudeDistanceConstraint (cem::message::common::Point2DF topo_point,
                                               double topo_dist_to_ego,
                                               std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
                                               const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos) {
  constexpr double DistMatchThreshold = 0.5;  // topo_dist_to_ego匹配的阈值
  constexpr double TopoPointMaxXDiffThreshold = 40.0;  // topo_dist_to_ego匹配的阈值

   // 使用split_info_或merge_info_的拓扑点匹配结果
  if (match_infos.empty()){
    return false;
  }

  double current_min_diff = DistMatchThreshold + 1e-6;
  uint64_t best_match_from = 0;
  uint64_t best_match_to = 0;

  for (const auto& info : match_infos) {
    uint64_t current_lane_from = info.second.lane_from;

    // 强制筛选：只处理target_to.size() >= 1的MergeSplitInfo
    if (info.second.target_to.size() < 1) {
        continue;
    }

    const auto& target_pair = *info.second.target_to.begin();  // 取哈希表首个元素（唯一元素）
    const LdMapProcessor::MergeSplitTarget& target = target_pair.second;
    double current_dist = target.distance_to_ego;
    double abs_diff = std::abs(current_dist - topo_dist_to_ego);
    if (abs_diff <= DistMatchThreshold && abs_diff < current_min_diff) {
      current_min_diff = abs_diff;
      best_match_from = current_lane_from;
      best_match_to = target.lane_to;
    }
  }
  if (best_match_from == 0 || best_match_to == 0) {
    return false;
  }
  auto topo_ld_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [best_match_from](const cem::message::env_model::LaneInfo &lane) { return best_match_from == lane.id; });
  if (topo_ld_lane == ld_map->lanes.end()) {
    return false;
  }

  auto ld_topo_point = topo_ld_lane->points.back();
  auto topo_point_x_diff = std::abs(ld_topo_point.x - topo_point.x);

# ifdef AddTopoDebug
  AINFO << "*******TopoPointLongitude******";
  AINFO << "timestamp: " << std::fixed << std::setprecision(3) << data_->header.timestamp;
  AINFO << "sequence number: " << data_->header.cycle_counter;
  AINFO << "topo_ld_lane: " << topo_ld_lane->id;
  AINFO << "ld_topo_point: (" << ld_topo_point.x << ", " << ld_topo_point.y << ")";
  AINFO << "topo_point: (" << topo_point.x << ", " << topo_point.y << ")";
  AINFO << "topo_point_x_diff: " << topo_point_x_diff << "; " << topo_dist_to_ego;
  AINFO << "************************";
# endif

  if(topo_point_x_diff <= TopoPointMaxXDiffThreshold) {
    return true;
  }
  else {
    return false;
  }
}

bool BevMapProcessor::CheckEgoLaneEmergencyConstraint(const navigation::EmergencyLaneInfo& sd_emergency_lane_info) {
  constexpr double SDLongThreshold = 0.5;
  constexpr float RoadEdgeDistThreshold = 1.0;
  constexpr double DashPercentThreshold = 0.2;
  // 内部函数：通过纵向信息判断Emergency_lane可行性
  auto SDHasEMLane =
      [](const BevLaneInfo* bev_lane_info,
         const std::vector<std::pair<double, double>>& sd_ranges) -> bool {
    if (!bev_lane_info && !bev_lane_info->geos->empty()) {
      return false;
    }
    float start_x = bev_lane_info->geos->front().x();
    float end_x = bev_lane_info->geos->back().x();
    float intersection_length = 0;
    for (auto& range_tmp : sd_ranges) {
      if (range_tmp.first < end_x && range_tmp.second > start_x) {
        if (range_tmp.first < start_x) {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - start_x);
          } else {
            intersection_length += fabs(end_x - start_x);
          }
        } else {
          if (range_tmp.second < end_x) {
            intersection_length += fabs(range_tmp.second - range_tmp.first);
          } else {
            intersection_length += fabs(end_x - range_tmp.first);
          }
        }
      }
    }
    if (fabs(end_x - start_x) > 5) {
      float rate = intersection_length / fabs(end_x - start_x);
      if (rate > SDLongThreshold) {
        return true;
      }
    }
    return false;
  };

  // if (sd_emergency_lane_info.left.exists) {
  //   AINFO << "Left emergency lane exists in merged ranges:";
  //   for (const auto &range : sd_emergency_lane_info.left.ranges) {
  //     AINFO << "  [" << range.first << ", " << range.second << "]";
  //   }
  //   AINFO << "lane num: " << sd_emergency_lane_info.left.lane_num;
  // } else {
  //   AINFO << "No left emergency lane found.";
  // }

  // if (sd_emergency_lane_info.right.exists) {
  //   AINFO << "Right emergency lane exists in merged ranges:";
  //   for (const auto &range : sd_emergency_lane_info.right.ranges) {
  //     AINFO << "  [" << range.first << ", " << range.second << "]";
  //   }
  //   AINFO << "lane num: " << sd_emergency_lane_info.right.lane_num;
  // } else {
  //   AINFO << "No right emergency lane found.";
  // }

  /// 遍历所有车道，找出ego_lane，根据约束条件校验其是否满足
  for (const auto& lane : data_->lane_infos) {
    if (lane.position == 0 &&  // ego_lane
        lane.left_lane_marker_id != 0 &&
        lane.right_lane_marker_id != 0) {
      /// 右侧应急车道纵向重叠超过50%条件
      if (sd_emergency_lane_info.right.exists && SDHasEMLane(&lane, sd_emergency_lane_info.right.ranges)) {
        bool valid_flag = true;
        auto left_lane_boundary = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(lane.left_lane_marker_id);
        auto right_lane_boundary =INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(lane.right_lane_marker_id);
        /// 左侧lane_marker添加虚线占比约束条件
        if (left_lane_boundary) {
          int dash_line_count = 0;
          int all_index_count = 0;
          for (auto& type_tmp : left_lane_boundary->type_segs) {
            all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
            if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED ||
                type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
              dash_line_count += fabs(type_tmp.end_index - type_tmp.start_index);
            }
          }
          if (all_index_count != 0 &&
              (float)dash_line_count / (float)all_index_count > DashPercentThreshold) {
            valid_flag = false;
          }
        }
        /// 右侧lane_marker添加虚线占比约束条件
        if (right_lane_boundary) {
          float dist2rightRoadEdge = std::numeric_limits<float>::max();
          for (auto& road_edge : data_->edges) {
            std::pair<float, float> gap_right_boundary_pair = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            bool gap_right_boundary_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(*road_edge.geos, *right_lane_boundary->geos, gap_right_boundary_pair);
            if (gap_right_boundary_valid &&
                gap_right_boundary_pair.first != std::numeric_limits<float>::max() &&
                gap_right_boundary_pair.first < dist2rightRoadEdge) {
              dist2rightRoadEdge = gap_right_boundary_pair.first;
            }
          }
          if (dist2rightRoadEdge > RoadEdgeDistThreshold) {
            valid_flag = false;
          }
        }
        return valid_flag;
      }

      /// 左侧应急车道纵向重叠超过50%条件
      if (sd_emergency_lane_info.left.exists && SDHasEMLane(&lane, sd_emergency_lane_info.left.ranges)) {
        bool valid_flag = true;
        auto left_lane_boundary = INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(lane.left_lane_marker_id);
        auto right_lane_boundary =INTERNAL_PARAMS.raw_bev_data.GetLaneBoundaryById(lane.right_lane_marker_id);
        /// 右侧lane_marker添加虚线占比约束条件
        if (right_lane_boundary) {
          int dash_line_count = 0;
          int all_index_count = 0;
          for (auto& type_tmp : right_lane_boundary->type_segs) {
            all_index_count += fabs(type_tmp.end_index - type_tmp.start_index);
            if (type_tmp.type == BevLaneMarkerType::BEV_LMT__DASHED ||
                type_tmp.type == BevLaneMarkerType::BEV_LMT__INVALID) {
              dash_line_count += fabs(type_tmp.end_index - type_tmp.start_index);
            }
          }
          if (all_index_count != 0 &&
              (float)dash_line_count / (float)all_index_count > DashPercentThreshold) {
            valid_flag = false;
          }
        }
        /// 左侧lane_marker添加虚线占比约束条件
        if (left_lane_boundary) {
          float dist2leftRoadEdge = std::numeric_limits<float>::max();
          for (auto& road_edge : data_->edges) {
            std::pair<float, float> gap_left_boundary_pair = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            bool gap_left_boundary_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(*road_edge.geos, *left_lane_boundary->geos, gap_left_boundary_pair);
            if (gap_left_boundary_valid &&
                gap_left_boundary_pair.first != std::numeric_limits<double>::max() &&
                gap_left_boundary_pair.first < dist2leftRoadEdge) {
              dist2leftRoadEdge = gap_left_boundary_pair.first;
            }
          }
          if (dist2leftRoadEdge > RoadEdgeDistThreshold) {
            valid_flag = false;
          }
        }
        return valid_flag;
      }
    }
  }
  return false;
}

bool BevMapProcessor::CheckSplitLaneRelationConstraint(const cem::message::sensor::BevLaneInfo& main_lane, 
                                        const std::vector<cem::message::sensor::BevLaneInfo>& sub_lanes, 
                                        bool break_lane_flag, 
                                        json &topo_debug_info) {
  if (!break_lane_flag) {
    std::set<uint64_t> sub_lane_id_set;
    // 查询sub_lanes的前继，要求其前继是空，或者是main_lane
    for (const auto& sub_lane : sub_lanes) {
      sub_lane_id_set.insert(sub_lane.id);
      if (!sub_lane.previous_lane_ids.empty()) {
        for (const auto & prev_lane_id : sub_lane.previous_lane_ids) {
          if (prev_lane_id != main_lane.id) {
            topo_debug_info["isolated target lane"] = false;
            return false;
          }
        }
      }
    }

    // 查询main_lane的后继，要求main_lane后继为空，或者在sub_lane_id_set中
    if (!main_lane.next_lane_ids.empty()) {
      for (const auto & next_lane_id : main_lane.next_lane_ids) {
        if (sub_lane_id_set.count(next_lane_id) == 0) {
          topo_debug_info["isolated target lane"] = false;
          return false;
        }
      }
    }
  }
  else{
    // 查询sub_lanes的前继，要求其前继是空
    for (const auto& sub_lane : sub_lanes) {
      if (!sub_lane.previous_lane_ids.empty()) {
        topo_debug_info["isolated target lane"] = false;
        return false;
      }
    }
  }
  return true;
}

bool BevMapProcessor::CheckMergeLaneRelationConstraint(const cem::message::sensor::BevLaneInfo& main_lane, 
                                        const std::vector<cem::message::sensor::BevLaneInfo>& sub_lanes, 
                                        bool break_lane_flag, 
                                        json &topo_debug_info) {
  if (!break_lane_flag) {
    std::set<uint64_t> sub_lane_id_set;
    // 查询sub_lanes的后继，要求其后继是空，或者是main_lane
    for (const auto& sub_lane : sub_lanes) {
      sub_lane_id_set.insert(sub_lane.id);
      if (!sub_lane.next_lane_ids.empty()) {
        for (const auto & next_lane_id : sub_lane.next_lane_ids) {
          if (next_lane_id != main_lane.id) {
            return false;
          }
        }
      }
    }

    // 查询main_lane的后继，要求main_lane后继为空，或者在sub_lane_id_set中
    if (!main_lane.previous_lane_ids.empty()) {
      for (const auto & prev_lane_id : main_lane.previous_lane_ids) {
        if (sub_lane_id_set.count(prev_lane_id) == 0) {
          topo_debug_info["isolated target lane"] = false;
          return false;
        }
      }
    }
  }
  else{
    // 查询sub_lanes的前继，要求其前继是空
    for (const auto& sub_lane : sub_lanes) {
      if (!sub_lane.next_lane_ids.empty()) {
        topo_debug_info["isolated target lane"] = false;
        return false;
      }
    }
  }
  return true;
}

double BevMapProcessor::GetEgoToEndLength (cem::message::sensor::BevLaneInfo* main_lane) {
  int ego_pt_idx = 0;
  double dist_ego2end = 0;

  if (main_lane->line_points.size() < 2) {
    return 0;
  }

  if (main_lane->line_points.back().x <= 0) {
    dist_ego2end = -std::sqrt(std::pow(main_lane->line_points.back().x, 2) + std::pow(main_lane->line_points.back().y, 2));
    return dist_ego2end;
  }
  else if (main_lane->line_points.front().x > 0) {
    dist_ego2end = std::sqrt(std::pow(main_lane->line_points.front().x, 2) + std::pow(main_lane->line_points.front().y, 2));
  }
  else {
    for (int i = 0; i < main_lane->line_points.size()-1; ++i) {
      if (main_lane->line_points[i+1].x > 0) {
        ego_pt_idx = i;
        dist_ego2end = -std::sqrt(std::pow(main_lane->line_points[ego_pt_idx].x, 2) + std::pow(main_lane->line_points[ego_pt_idx].y, 2));
        break;
      }
    }
  }

  // AINFO << "main_lane_id: " << main_lane->id;
  // AINFO << "index: " << ego_pt_idx
  //       << "; start pt: (" <<  main_lane->line_points[ego_pt_idx].x << ", " << main_lane->line_points[ego_pt_idx].y << "); ";
  // AINFO << "dist_ego2end start: " << dist_ego2end;

  for (int i = ego_pt_idx; i < main_lane->line_points.size()-1; ++i) {
    auto point1 = main_lane->line_points[i];
    auto point2 = main_lane->line_points[i+1];
    dist_ego2end += std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
  }

  // AINFO << "dist_ego2end end: " << dist_ego2end;
  return dist_ego2end;
}

bool BevMapProcessor::CheckLDSplitTopoType(LdMapProcessor::MergeSplitType topo_type, cem::message::env_model::SplitTopology ld_topo_type) {
  if (topo_type == LdMapProcessor::MergeSplitType::kConnectForward &&
      (ld_topo_type == cem::message::env_model::SplitTopology ::TOPOLOGY_SPLIT_NONE ||
       ld_topo_type == cem::message::env_model::SplitTopology ::TOPOLOGY_SPLIT_UNKNOWN
      )) {
    return true;
  }
  else if (topo_type == LdMapProcessor::MergeSplitType::kSplitLeft &&
      ld_topo_type == cem::message::env_model::SplitTopology ::TOPOLOGY_SPLIT_LEFT) {
    return true;
  }
  else if (topo_type == LdMapProcessor::MergeSplitType::kSplitRight &&
      ld_topo_type == cem::message::env_model::SplitTopology ::TOPOLOGY_SPLIT_RIGHT) {
    return true;
  }
  return false;
}

bool BevMapProcessor::CheckLDMergeTopoType(LdMapProcessor::MergeSplitType topo_type, cem::message::env_model::MergeTopology ld_topo_type) {
  if (topo_type == LdMapProcessor::MergeSplitType::kConnectForward &&
      (ld_topo_type == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_NONE ||
       ld_topo_type == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_UNKNOWN || 
       ld_topo_type == cem::message::env_model::MergeTopology::TOPOLOGY_TO_BE_MERGED
      )) {
    return true;
  }
  else if (topo_type == LdMapProcessor::MergeSplitType::kMergeLeft &&
      ld_topo_type == cem::message::env_model::MergeTopology ::TOPOLOGY_MERGE_LEFT) {
    return true;
  }
  else if (topo_type == LdMapProcessor::MergeSplitType::kMergeRight &&
      ld_topo_type == cem::message::env_model::MergeTopology ::TOPOLOGY_MERGE_RIGHT) {
    return true;
  }
  return false;
}

void BevMapProcessor::FindUTurnLDLane(std::vector<uint64_t> ld_match_ids,
                                      const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                      std::vector<cem::message::env_model::LaneInfo>& u_turn_lanes) {
  for (const auto& ld_match_id : ld_match_ids) {
    auto ld_match_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [ld_match_id](const cem::message::env_model::LaneInfo &lane) { return ld_match_id == lane.id; });

    if (ld_match_lane == ld_map->lanes.end()) {
      continue;
    }

    bool has_uturn = false;
    bool has_other = false;
    for (const auto& ld_next_id : ld_match_lane->next_lane_ids) {
      auto ld_next_lane = std::find_if(ld_map->lanes.begin(), ld_map->lanes.end(),
                                    [ld_next_id](const cem::message::env_model::LaneInfo &lane) { return ld_next_id == lane.id; });
      if (ld_next_lane == ld_map->lanes.end()) {
        continue;
      }

      if (ld_next_lane->turn_type == cem::message::env_model::TurnType::U_TURN) {
        has_uturn = true;
      }
      else if (ld_next_lane->turn_type != cem::message::env_model::TurnType::NO_TURN) {
        has_other = true;
        break;
      }
    }

    if (has_uturn && !has_other) {
      u_turn_lanes.push_back(*ld_match_lane);
    }
  }
}

double BevMapProcessor::CalculateCurvature(const cem::message::sensor::BevLaneInfo& lane, size_t idx) {
  if (idx < 1 || idx >= lane.line_points.size() - 1) {
      return 0.0;
  }

  // 取连续三点
  const auto& p_prev = lane.line_points[idx - 1];
  const auto& p_curr = lane.line_points[idx];
  const auto& p_next = lane.line_points[idx + 1];

  // 计算分子（叉积，符号决定弯曲方向）
  double numerator = 2.0 * ((p_next.x - p_curr.x) * (p_curr.y - p_prev.y) - 
                            (p_curr.x - p_prev.x) * (p_next.y - p_curr.y));

  // 计算分母（几何归一化项：三点间距的乘积开平方）
  double d1_sq = std::pow(p_curr.x - p_prev.x, 2) + std::pow(p_curr.y - p_prev.y, 2);
  double d2_sq = std::pow(p_next.x - p_curr.x, 2) + std::pow(p_next.y - p_curr.y, 2);
  double d3_sq = std::pow(p_next.x - p_prev.x, 2) + std::pow(p_next.y - p_prev.y, 2);
  double denominator = std::sqrt(d1_sq * d2_sq * d3_sq);

  // 避免除零（三点共线时分母为0，曲率为0）
  if (denominator < 1e-12) {
      return 0.0;
  }

  return numerator / denominator;
}

bool BevMapProcessor::CheckLeftBending(const cem::message::sensor::BevLaneInfo& lane, json &topo_debug_info) {
  constexpr double eps = 1e-6;
  constexpr double LeftCurvatureThreshold = -0.05;
  constexpr double LeftCountThreshold = 0.6;
  constexpr int CurvatureMinNum = 5;

  // 点集数量不足，直接返回false
  if (lane.line_points.size() < 3) {
      return false;
  }

  // 统计曲率信息
  std::vector<double> curvatures;
  size_t left_count = 0;    // 向左弯曲点数量（k < -eps_）
  size_t right_count = 0;   // 向右弯曲点数量（k > eps_）
  size_t collinear_count = 0; // 近直线点数量（|k| ≤ eps_）

  // 遍历所有中间点计算曲率
  for (size_t i = 1; i < lane.line_points.size() - 1; ++i) {
      double k = CalculateCurvature(lane, i);
      curvatures.push_back(k);

      if (k < -eps) {
        left_count++;
      } else if (k > eps) {
        right_count++;
      } else {
        collinear_count++;
      }
  }

  // 计算统计指标
  size_t total_valid = left_count + right_count;
  double avg_curvature = 0.0;
  if (!curvatures.empty()) {
      avg_curvature = std::accumulate(curvatures.begin(), curvatures.end(), 0.0) / curvatures.size();
  }

  // 判定规则
  bool is_left = false;
  if (total_valid >= CurvatureMinNum) {
    is_left = (avg_curvature < LeftCurvatureThreshold) && (static_cast<double>(left_count) / total_valid >= LeftCountThreshold);
  }
  else {
    is_left = (avg_curvature < LeftCurvatureThreshold);
  }

  // 构造统计信息字符串
  json stats;
  stats["curvatures num"] =  curvatures.size();
  stats["left curve num"] =  left_count;
  stats["right curve num"] =  right_count;
  stats["straight num"] =  collinear_count;
  stats["left percentage"] = static_cast<double>(left_count) / total_valid;
  stats["avg_curvature"] =  avg_curvature;
  stats["is_left"] =  is_left;
  stats["lane id"] =  lane.id;
  topo_debug_info["left bending check"] = stats;

  // AINFO << stats.dump();
  return is_left;
}

float BevMapProcessor::CalculateCenterBoundaryDistanceFromMap(const cem::message::env_model::LaneInfo& lane, 
                                               const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map) {
  constexpr float MidToBoundaryDist = 0.5;
  std::pair<float, float> mid_to_boundary_dist = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};

  // 计算车道中心线的lane_geos
  std::vector<Eigen::Vector2f> lane_geos;
  for (const auto & point : lane.points){
    Eigen::Vector2f point_geo;
    point_geo.x() = point.x;
    point_geo.y() = point.y;
    lane_geos.emplace_back(point_geo);
  }

  // 计算中心线到右边lane_marker的距离
  std::vector<Eigen::Vector2f> boundary_geos;
  for (const auto & right_lane_boundary_id : lane.right_lane_boundary_ids){
    auto right_lane_boundary = std::find_if(ld_map->lane_boundaries.begin(), ld_map->lane_boundaries.end(),
                                    [right_lane_boundary_id](const cem::message::env_model::LaneBoundaryInfo &lane_boundary) { return right_lane_boundary_id == lane_boundary.id; });
    if (right_lane_boundary == ld_map->lane_boundaries.end()) continue;
    const auto& lane_boundary_points = right_lane_boundary->points;
    if (lane_boundary_points.empty()) continue;

    float lane_min_x{0}, boundary_min_x{0};
    lane_min_x = lane_boundary_points.front().x;
    if (!boundary_geos.empty()) {
      boundary_min_x = boundary_geos.front().x();
    }

    std::vector<Eigen::Vector2f> lane_boundary_geos;
    lane_boundary_geos.reserve(lane_boundary_points.size()); // 预留空间，避免扩容
    for (const auto& p : lane_boundary_points) {
        lane_boundary_geos.emplace_back(p.x, p.y);
    }

    if (!boundary_geos.empty() && lane_min_x < boundary_min_x) {
        boundary_geos.insert(boundary_geos.begin(),
                             lane_boundary_geos.begin(), lane_boundary_geos.end());
    } else {
        boundary_geos.insert(boundary_geos.end(),
                             lane_boundary_geos.begin(), lane_boundary_geos.end());
    }
  }

  bool match_distance_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(lane_geos, boundary_geos, mid_to_boundary_dist);
  if (match_distance_valid && mid_to_boundary_dist.first != std::numeric_limits<float>::max()) {
    return mid_to_boundary_dist.first;
  }

  // 计算中心线到左边lane_marker的距离
  boundary_geos.clear();
  mid_to_boundary_dist = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
  for (const auto & left_lane_boundary_id : lane.left_lane_boundary_ids){
    auto left_lane_boundary = std::find_if(ld_map->lane_boundaries.begin(), ld_map->lane_boundaries.end(),
                                    [left_lane_boundary_id](const cem::message::env_model::LaneBoundaryInfo &lane_boundary) { return left_lane_boundary_id == lane_boundary.id; });
    if (left_lane_boundary == ld_map->lane_boundaries.end()) continue;
    const auto& lane_boundary_points = left_lane_boundary->points;
    if (lane_boundary_points.empty()) continue;

    float lane_min_x{0}, boundary_min_x{0};
    lane_min_x = lane_boundary_points.front().x;
    if (!boundary_geos.empty()) {
      boundary_min_x = boundary_geos.front().x();
    }

    std::vector<Eigen::Vector2f> lane_boundary_geos;
    lane_boundary_geos.reserve(lane_boundary_points.size()); // 预留空间，避免扩容
    for (const auto& p : lane_boundary_points) {
        lane_boundary_geos.emplace_back(p.x, p.y);
    }

    if (!boundary_geos.empty() && lane_min_x < boundary_min_x) {
        boundary_geos.insert(boundary_geos.begin(),
                             lane_boundary_geos.begin(), lane_boundary_geos.end());
    } else {
        boundary_geos.insert(boundary_geos.end(),
                             lane_boundary_geos.begin(), lane_boundary_geos.end());
    }
  }
  match_distance_valid = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(lane_geos, boundary_geos, mid_to_boundary_dist);
  if (match_distance_valid && mid_to_boundary_dist.first != std::numeric_limits<float>::max()) {
    return mid_to_boundary_dist.first;
  }
  return MidToBoundaryDist;
}

}  // namespace fusion
}  // namespace cem