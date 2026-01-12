#include "lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.h"
// #include "lib/perception_and_ld_map_fusion/data_fusion/road_slice.h"
#include "lib/perception_and_ld_map_fusion/common/ld_lane_common_process.h"

namespace cem {
namespace fusion {

using cem::message::env_model::LaneInfo;
using cem::message::env_model::MergeTopology;
using cem::message::env_model::Point;
using cem::message::env_model::SectionInfo;
using cem::message::env_model::SplitTopology;

LdMapProcessor::LdMapProcessor() {}

LdMapProcessor::~LdMapProcessor() {}

void LdMapProcessor::Path::PrintPathInfo() {
  std::string info = "";
  info += "lanes: [";
  for (const auto& lane : lanes) {
    info += "(";
    info += std::to_string(lane.first);
    info += ", ";
    info += std::to_string(lane.second);
    info += ") ";
  }
  info += "]";
  info += " start_ele_id: ";
  info += std::to_string(start_ele_id);
  info += " end_ele_id: ";
  info += std::to_string(end_ele_id);
  info += " status: ";
  info += std::to_string(static_cast<int>(status));
  info += " start_section_key: ";
  info += std::to_string(static_cast<int>(start_section_key));
  info += " is_start: ";
  info += std::to_string(static_cast<int>(is_start));
  info += " from_start_section: ";
  info += std::to_string(static_cast<int>(from_start_section));
  info += " change_point: [";
  for (const auto& point : change_point) {
    info += "(";
    info += std::to_string(point.first);
    info += ", ";
    info += std::to_string(static_cast<int>(point.second));
    info += ") ";
  }
  info += "]";
  // AINFO << info;
}

void LdMapProcessor::Proc() {
  ClearCache();
  EstimateMainDirection();
  ConstructLaneTopology();
  JudgeIsOnFreewayOrNot();
  SearchUnreliableRoad();
  return;
}

bool LdMapProcessor::RotateAndTranslate(const Eigen::Isometry3d& T) {
  for (auto& lane : data_->lanes) {
    for (auto& point : lane.points) {
      Eigen::Vector3d point_src(point.x, point.y, point.z);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
      point.z = point_dst.z();
    }
  }

  for (auto& boundary : data_->lane_boundaries) {
    for (auto& point : boundary.points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
  }

  for (auto& boundary : data_->road_boundaries) {
    for (auto& point : boundary.points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
  }

  for (auto& stopline : data_->stop_lines) {
    for (auto& point : stopline.points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
  }

  for (auto& junction : data_->junctions) {
    for (auto& point : junction.points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
  }

  for (auto& cross_walk : data_->cross_walks) {
    for (auto& point : cross_walk.points) {
      Eigen::Vector3d point_src(point.x, point.y, 0);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
    }
  }

  data_->route.navi_start.s_offset += T.translation().x();
  for (auto& section : data_->route.sections) {
    for (auto& point : section.points) {
      Eigen::Vector3d point_src(point.x, point.y, point.z);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
      point.z = point_dst.z();
    }
  }

  for (auto& subpath : data_->route.subpaths) {
    for (auto& section : subpath.sections) {
      for (auto& point : section.points) {
        Eigen::Vector3d point_src(point.x, point.y, point.z);
        auto point_dst = T * point_src;
        point.x = point_dst.x();
        point.y = point_dst.y();
        point.z = point_dst.z();
      }
    }
  }

  for (auto& traffic_light : data_->traffic_lights) {
    auto& center_point = traffic_light.center_position;
    Eigen::Vector3d center_src(center_point.x, center_point.y, center_point.z);
    auto center_dst = T * center_src;
    center_point.x = center_dst.x();
    center_point.y = center_dst.y();
    center_point.z = center_dst.z();

    for (auto& point : traffic_light.bounding_box_geometry) {
      Eigen::Vector3d point_src(point.x, point.y, point.z);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
      point.z = point_dst.z();
    }
  }

  for (auto &exp_trajectory : data_->exp_trajectories) {
    for (auto &point : exp_trajectory.points) {
      Eigen::Vector3d point_src(point.x, point.y, point.z);
      auto point_dst = T * point_src;
      point.x = point_dst.x();
      point.y = point_dst.y();
      point.z = point_dst.z();
    }
  }

  return true;
}

void LdMapProcessor::EstimateMainDirection() {
  auto current_section_id = data_->route.navi_start.section_id;
  auto current_section =
      std::find_if(data_->route.sections.begin(), data_->route.sections.end(),
                   [current_section_id](const SectionInfo& section) {
                     return section.id == current_section_id;
                   });

  LaneInfo* reference_lane = nullptr;
  std::vector<Point>::iterator reference_point;

  if (current_section == data_->route.sections.end()) {
    AWARN << "Cannot find current section in ld map.";
  } else {
    for (auto iter = current_section->lane_ids.begin();
         iter != current_section->lane_ids.end(); ++iter) {
      auto lane = std::find_if(
          data_->lanes.begin(), data_->lanes.end(),
          [iter](const LaneInfo& lane) { return *iter == lane.id; });
      if (lane == data_->lanes.end()) {
        continue;
      }
      if (lane->split_topology != SplitTopology::TOPOLOGY_SPLIT_NONE) {
        continue;
      }
      if (lane->merge_topology != MergeTopology::TOPOLOGY_MERGE_NONE) {
        continue;
      }
      if ((lane->type != LaneType::LANE_NORMAL) &&
          (lane->type != LaneType::LANE_RAMP)) {
        continue;
      }

      if (lane->points.size() < 4) {
        continue;
      }
      if ((lane->points.front().x > 20) || (lane->points.back().x < -20)) {
        continue;
      }

      if (!reference_lane) {
        reference_lane = &(*lane);
        reference_point = std::find_if(
            reference_lane->points.begin(), reference_lane->points.end(),
            [](const Point& pt) { return pt.x > 0; });
        if (reference_point > reference_lane->points.end() - 4) {
          reference_point = reference_lane->points.end() - 4;
        }
      } else {
        auto point_tmp = std::find_if(reference_lane->points.begin(),
                                      reference_lane->points.end(),
                                      [](const Point& pt) { return pt.x > 0; });
        if (point_tmp > reference_lane->points.end() - 4) {
          point_tmp = reference_lane->points.end() - 4;
        }
        if (std::abs(point_tmp->y) < std::abs(reference_point->y)) {
          reference_lane = &(*lane);
          reference_point = point_tmp;
        }
      }
    }
  }

  if (!reference_lane) {
    main_direction_.x() = 1.0;
    main_direction_.y() = 0.0;
  } else {
    main_direction_.x() = (reference_point + 3)->x - reference_point->x;
    main_direction_.y() = (reference_point + 3)->y - reference_point->y;
    main_direction_.normalize();
  }
}

void LdMapProcessor::ConstructLaneTopology() {
  CreateLaneInfoIndex();
  ConstructLaneSequence();
  ExtractLaneGroupsAndSearchEgoLane();
  ExtractEgoLanePoints();
  SearchRoutePath();
}

void LdMapProcessor::CreateLaneInfoIndex() {
  for (auto& lane : data_->lanes) {
    lane.previous_lane_ids.clear();
    lane_map_[lane.id].is_classified = false;
    lane_map_[lane.id].ptr = &lane;
  }

  for (auto& section : data_->route.sections) {
    section_map_.emplace(section.id, &section);
  }

  for (auto& [id, lane] : lane_map_) {
    for (auto next_id : lane.ptr->next_lane_ids) {
      if (lane_map_.count(next_id) != 0) {
        lane_map_.at(next_id).ptr->previous_lane_ids.push_back(lane.ptr->id);
      }
    }
  }
}

void LdMapProcessor::ConstructLaneSequence() {
  std::unordered_map<uint64_t, double> distance_from_lane_end_point_to_ego;

  auto start_section_id = data_->route.navi_start.section_id;
  auto total_distance = -data_->route.navi_start.s_offset;
  auto iter_start = std::find_if(
      data_->route.sections.begin(), data_->route.sections.end(),
      [start_section_id](const cem::message::env_model::SectionInfo& section) {
        return section.id == start_section_id;
      });
  if (iter_start != data_->route.sections.end()) {
    for (auto iter = iter_start; iter != data_->route.sections.end(); ++iter) {
      total_distance += iter->length;
      for (auto id : iter->lane_ids) {
        distance_from_lane_end_point_to_ego.emplace(id, total_distance);
      }
    }

    total_distance = -data_->route.navi_start.s_offset;
    for (auto iter = iter_start - 1; iter >= data_->route.sections.begin();
         --iter) {
      for (auto id : iter->lane_ids) {
        distance_from_lane_end_point_to_ego.emplace(id, total_distance);
      }
      total_distance -= iter->length;
    }
  }

  for (auto& lane : lane_map_) {
    if (lane.second.is_classified) {
      continue;
    }
    std::vector<uint64_t> current_group;
    current_group.push_back(lane.first);
    lane.second.is_classified = true;

    auto current_lane = lane.second.ptr;
    while (true) {
      if (current_lane->previous_lane_ids.size() == 0) {
        break;
      } else if (current_lane->previous_lane_ids.size() > 1) {
        for (auto id : current_lane->previous_lane_ids) {
          if (lane_map_.count(id) > 0) {
            auto& lane_tmp = lane_map_.at(id);

            if (lane_tmp.ptr->merge_topology ==
                    cem::message::env_model::MergeTopology::
                        TOPOLOGY_MERGE_LEFT ||
                lane_tmp.ptr->merge_topology ==
                    cem::message::env_model::MergeTopology::
                        TOPOLOGY_MERGE_RIGHT) {
              MergeSplitInfo* merge_info = nullptr;
              if (merge_info_.count(id) == 0) {
                merge_info = &merge_info_[id];
                merge_info->lane_from = id;
              } else {
                merge_info = &merge_info_.at(id);
              }
              MergeSplitTarget& merge_target =
                  merge_info->target_to[current_lane->id];
              merge_target.lane_to = current_lane->id;
              if (lane_tmp.ptr->merge_topology ==
                  cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_LEFT) {
                merge_target.type = MergeSplitType::kMergeLeft;
              } else if (lane_tmp.ptr->merge_topology ==
                         cem::message::env_model::MergeTopology::
                             TOPOLOGY_MERGE_RIGHT) {
                merge_target.type = MergeSplitType::kMergeRight;
              }
              merge_target.distance_to_ego =
                  distance_from_lane_end_point_to_ego.count(
                      merge_info->lane_from) > 0
                      ? distance_from_lane_end_point_to_ego.at(
                            merge_info->lane_from)
                      : 0;
            } else {
              MergeSplitInfo* merge_info = nullptr;
              if (connect_info_.count(id) == 0) {
                merge_info = &connect_info_[id];
                merge_info->lane_from = id;
              } else {
                merge_info = &connect_info_.at(id);
              }
              MergeSplitTarget& merge_target =
                  merge_info->target_to[current_lane->id];
              merge_target.lane_to = current_lane->id;
              merge_target.type = MergeSplitType::kConnectForward;
              merge_target.distance_to_ego =
                  distance_from_lane_end_point_to_ego.count(
                      merge_info->lane_from) > 0
                      ? distance_from_lane_end_point_to_ego.at(
                            merge_info->lane_from)
                      : 0;
            }
          }
        }
        break;
      }
      auto possible_id = current_lane->previous_lane_ids.front();
      if (lane_map_.count(possible_id) == 0) {
        break;
      }
      auto previous_lane = &lane_map_.at(possible_id);
      if (previous_lane->ptr->next_lane_ids.size() != 1) {
        break;
      }
      if (previous_lane->is_classified) {
        break;
      } else {
        current_group.insert(current_group.begin(), previous_lane->ptr->id);
        previous_lane->is_classified = true;
        current_lane = previous_lane->ptr;
      }
    }

    current_lane = lane.second.ptr;
    while (true) {
      if (current_lane->next_lane_ids.size() == 0) {
        break;
      } else if (current_lane->next_lane_ids.size() > 1) {
        for (auto id : current_lane->next_lane_ids) {
          if (lane_map_.count(id) > 0) {
            auto& lane_tmp = lane_map_.at(id);

            if (lane_tmp.ptr->split_topology ==
                    cem::message::env_model::SplitTopology::
                        TOPOLOGY_SPLIT_LEFT ||
                lane_tmp.ptr->split_topology ==
                    cem::message::env_model::SplitTopology::
                        TOPOLOGY_SPLIT_RIGHT) {
              MergeSplitInfo* split_info = nullptr;
              if (split_info_.count(current_lane->id) == 0) {
                split_info = &split_info_[current_lane->id];
                split_info->lane_from = current_lane->id;
              } else {
                split_info = &split_info_.at(current_lane->id);
              }
              MergeSplitTarget& split_target = split_info->target_to[id];
              split_target.lane_to = id;
              if (lane_tmp.ptr->split_topology ==
                  cem::message::env_model::SplitTopology::TOPOLOGY_SPLIT_LEFT) {
                split_target.type = MergeSplitType::kSplitLeft;
              } else if (lane_tmp.ptr->split_topology ==
                         cem::message::env_model::SplitTopology::
                             TOPOLOGY_SPLIT_RIGHT) {
                split_target.type = MergeSplitType::kSplitRight;
              }
              split_target.distance_to_ego =
                  distance_from_lane_end_point_to_ego.count(
                      split_info->lane_from) > 0
                      ? distance_from_lane_end_point_to_ego.at(
                            split_info->lane_from)
                      : 0;
            } else {
              MergeSplitInfo* split_info = nullptr;
              if (connect_info_.count(current_lane->id) == 0) {
                split_info = &connect_info_[current_lane->id];
                split_info->lane_from = current_lane->id;
              } else {
                split_info = &connect_info_.at(current_lane->id);
              }
              MergeSplitTarget& split_target = split_info->target_to[id];
              split_target.lane_to = id;
              split_target.type = MergeSplitType::kConnectForward;
              split_target.distance_to_ego =
                  distance_from_lane_end_point_to_ego.count(
                      split_info->lane_from) > 0
                      ? distance_from_lane_end_point_to_ego.at(
                            split_info->lane_from)
                      : 0;
            }
          }
        }
        break;
      }
      auto possible_id = current_lane->next_lane_ids.front();
      if (lane_map_.count(possible_id) == 0) {
        break;
      }
      auto next_lane = &lane_map_.at(possible_id);
      if (next_lane->ptr->previous_lane_ids.size() != 1) {
        break;
      }
      if (next_lane->is_classified) {
        break;
      } else {
        current_group.insert(current_group.end(), next_lane->ptr->id);
        next_lane->is_classified = true;
        current_lane = next_lane->ptr;
      }
    }

    uint32_t tracked_group_id = 0;
    for (auto id : current_group) {
      if (lane_group_id_tracked_.count(id) > 0) {
        tracked_group_id = lane_group_id_tracked_.at(id).lane_group_id;
        break;
      }
    }
    if (tracked_group_id == 0) {
      tracked_group_id = lane_group_count_;
      lane_group_count_ =
          (lane_group_count_ + 1U == 0 ? 1 : lane_group_count_ + 1U);
    }

    lanes_.emplace(tracked_group_id, std::move(current_group));

    for (auto id : lanes_.at(tracked_group_id)) {
      lane_id_to_group_id_.emplace(id, tracked_group_id);
      auto& tracked_info = lane_group_id_tracked_[id];
      tracked_info.is_exist = true;
      tracked_info.lane_group_id = tracked_group_id;
    }
  }

  auto add_group_info_into =
      [&](std::unordered_map<uint64_t, MergeSplitInfo>& connect_info) {
        for (auto iter = connect_info.begin(); iter != connect_info.end();) {
          if ((lane_id_to_group_id_.count(iter->second.lane_from) == 0)) {
            iter = connect_info.erase(iter);
            continue;
          }
          iter->second.group_from =
              lane_id_to_group_id_.at(iter->second.lane_from);
          for (auto iter_target = iter->second.target_to.begin();
               iter_target != iter->second.target_to.end();) {
            if (lane_id_to_group_id_.count(iter_target->second.lane_to) == 0) {
              iter_target = iter->second.target_to.erase(iter_target);
              continue;
            }
            iter_target->second.group_to =
                lane_id_to_group_id_.at(iter_target->second.lane_to);
            iter_target++;
          }
          if (iter->second.target_to.empty()) {
            iter = connect_info.erase(iter);
          } else {
            iter++;
          }
        }
      };
  add_group_info_into(merge_info_);
  add_group_info_into(split_info_);
  add_group_info_into(connect_info_);
  
  // gmy_todo
  // std::unordered_map<uint32_t, std::vector<uint64_t> lane_groups_eles; // first: group id, second: eles_id from back to front
  // for (auto& section: data_->route.sections) {
  //   for (auto id : section.lane_ids) {
  //     auto group_id = lane_id_to_group_id_.at(id);
  //     lane_groups_eles[group_id].push_back(id);
  //   }
  // }
}

void LdMapProcessor::ExtractLaneGroupsAndSearchEgoLane() {
  auto front_section_id = data_->route.navi_start.section_id;
  if (section_map_.count(front_section_id) == 0) {
    return;
  }

  SectionInfo* front_section = section_map_.at(front_section_id);
  double distance_to_back =
      front_section->length - data_->route.navi_start.s_offset;

  SectionInfo* previous_of_front_section = nullptr;
  for (auto previous_section_id : front_section->predecessor_section_id_list) {
    if (section_map_.count(previous_section_id) > 0) {
      previous_of_front_section = section_map_.at(previous_section_id);
      break;
    }
  }

  std::unordered_map<uint64_t, uint64_t> front_section_to_previous_connect_info;
  if (previous_of_front_section != nullptr) {
    for (auto id : previous_of_front_section->lane_ids) {
      if (lane_id_to_group_id_.count(id) == 0) {
        continue;
      }
      if (lane_map_.count(id) == 0) {
        continue;
      }
      auto lane = lane_map_.at(id).ptr;
      if (lane->points.empty()) {
        continue;
      }
      for (auto next_id : lane->next_lane_ids) {
        if (lane_map_.count(next_id) > 0) {
          front_section_to_previous_connect_info[next_id] = id;
          break;
        }
      }
      auto group_id = lane_id_to_group_id_.at(id);
      lane_groups_details_.emplace(group_id, LaneGroup(group_id));
      lane_groups_details_.at(group_id).eles.emplace_back(id);
      lane_groups_details_.at(group_id).update_status =
          LaneGroup::UpdateStatus::kNotUpdate;
    }
  }
  auto cur_section = front_section;
  while (true) {
    for (auto id : cur_section->lane_ids) {
      if (lane_id_to_group_id_.count(id) == 0) {
        continue;
      }
      auto group_id = lane_id_to_group_id_.at(id);
      if (lane_groups_details_.count(group_id) == 0) {
        lane_groups_details_.emplace(group_id, LaneGroup(group_id));

        auto& lane_group = lane_groups_details_.at(group_id);
        lane_group.eles.emplace_back(id);
        lane_group.length = distance_to_back;
        if (front_section_to_previous_connect_info.count(id) > 0) {
          lane_group.predecessor_ele =
              front_section_to_previous_connect_info.at(id);
        }
      } else {
        auto& lane_group = lane_groups_details_.at(group_id);
        lane_group.eles.emplace_back(id);
        lane_group.update_status = LaneGroup::UpdateStatus::kUpdated;
        lane_group.length = distance_to_back;
      }
    }
    for (auto& [group_id, group] : lane_groups_details_) {
      if (group.update_status == LaneGroup::UpdateStatus::kUpdated) {
        group.update_status = LaneGroup::UpdateStatus::kNotUpdate;
      } else if (group.update_status == LaneGroup::UpdateStatus::kNotUpdate) {
        if (!group.eles.empty()) {
          if (lane_map_.count(group.eles.back()) == 0) {
            continue;
          }
          auto tmp = lane_map_.at(group.eles.back());
          for (auto id : cur_section->lane_ids) {
            if (std::find(tmp.ptr->next_lane_ids.begin(),
                          tmp.ptr->next_lane_ids.end(),
                          id) != tmp.ptr->next_lane_ids.end()) {
              group.successor_ele = id;
              break;
            }
          }
        }
        group.update_status = LaneGroup::UpdateStatus::kFinished;
      }
    }

    // AINFO << distance_to_back << ", section_id: " << cur_section->id;
    if (distance_to_back > 150.0) {
      break;
    }
    if (cur_section->successor_section_id_list.empty()) {
      break;
    }
    bool found_successor_section = false;
    for (auto id : cur_section->successor_section_id_list) {
      if (section_map_.count(id) == 0) {
        continue;
      }
      cur_section = section_map_.at(id);
      distance_to_back += cur_section->length;
      found_successor_section = true;
      break;
    }
    if (!found_successor_section) {
      break;
    }
  }

  double min_abs_y = std::numeric_limits<double>::max();
  ego_lane_points_.emplace_back(Eigen::Vector2d(0, 0));
  for (auto id : front_section->lane_ids) {
    if (lane_map_.count(id) == 0) {
      continue;
    }

    // 找到group_id
    if (lane_id_to_group_id_.count(id) == 0) {
      continue;
    }
    auto group_id = lane_id_to_group_id_.at(id);

    // 找到车道序列
    if (lane_groups_details_.count(group_id) == 0) {
      continue;
    }
    auto& lane_group = lane_groups_details_.at(group_id);

    // auto info = "group_id: " + std::to_string(group_id) + " - ";
    // for (auto ele : lane_group.eles) {
    //   info += std::to_string(ele);
    //   info += ", ";
    // }
    // AINFO << info;
    Eigen::Vector2d point_first;
    Eigen::Vector2d point_second;
    int visted_point_cnt = 0;
    for (auto lane_id : lane_group.eles) {
      // AINFO << "lane_id: " << lane_id;
      if (lane_map_.count(lane_id) == 0) {
        break;
      }
      bool is_stop = false;
      auto tmp = lane_map_.at(lane_id);
      for (auto pt : tmp.ptr->points) {
        point_first = point_second;
        point_second.x() = pt.x;
        point_second.y() = pt.y;
        visted_point_cnt++;
        if (pt.x > 0) {
          is_stop = true;
          break;
        }
      }
      if (is_stop) {
        break;
      }
    }
    // AINFO << "first: " << point_first;
    // AINFO << "second: " << point_second;
    if (visted_point_cnt == 0) {
      continue;
    } else if (visted_point_cnt == 1) {
      if ((lane_map_.count(lane_group.predecessor_ele) == 0) ||
          (lane_group.predecessor_ele == 0)) {
        continue;
      }
      auto predecessor_ele = lane_map_.at(lane_group.predecessor_ele).ptr;
      bool found_nagative_pt = false;
      point_first = point_second;
      for (auto iter = predecessor_ele->points.rbegin();
           iter != predecessor_ele->points.rend(); ++iter) {
        point_second = point_first;
        point_first.x() = iter->x;
        point_first.y() = iter->y;
        if (iter->x <= 0) {
          found_nagative_pt = true;
          break;
        }
      }
      if (!found_nagative_pt) {
        continue;
      }
    } else if (point_second.x() < 0) {
      if ((lane_map_.count(lane_group.successor_ele) == 0) ||
          (lane_group.successor_ele == 0)) {
        continue;
      }
      auto successor_ele = lane_map_.at(lane_group.successor_ele).ptr;
      bool found_positive_pt = false;
      for (auto iter = successor_ele->points.begin();
           iter != successor_ele->points.end(); ++iter) {
        point_first = point_second;
        point_second.x() = iter->x;
        point_second.y() = iter->y;
        if (iter->x > 0) {
          found_positive_pt = true;
          break;
        }
      }
      if (!found_positive_pt) {
        continue;
      }
    }

    // AINFO << "first: " << point_first;
    // AINFO << "second: " << point_second;

    Eigen::Vector2d direction = point_second - point_first;
    double t = -point_first.x() / direction.x();
    double y_intersection = point_first.y() + t * direction.y();
    // AINFO << "INTERSECTION: " << y_intersection;
    if (std::abs(y_intersection) < min_abs_y) {
      min_abs_y = std::abs(y_intersection);
      ego_lane_group_id_ = group_id;
      ego_lane_points_.front().y() = y_intersection;
    }
    // AINFO << "<<<<<<";
  }

  if (min_abs_y > 1.9) {
    ego_lane_group_id_ = 0;
  }
}

void LdMapProcessor::ExtractEgoLanePoints() {
  if (ego_lane_group_id_ == 0) {
    return;
  }
  if (lane_groups_details_.count(ego_lane_group_id_) == 0) {
    ego_lane_group_id_ = 0;
    return;
  }

  // 第一个点x大于0的话先加进去
  {
    auto ego_lane_group = lane_groups_details_.at(ego_lane_group_id_);
    if (ego_lane_group.eles.empty()) {
      ego_lane_group_id_ = 0;
      return;
    }
    if (lane_map_.count(ego_lane_group.eles.front()) == 0) {
      ego_lane_group_id_ = 0;
      return;
    }
    auto lane = lane_map_.at(ego_lane_group.eles.front()).ptr;
    if (lane->points.empty()) {
      ego_lane_group_id_ = 0;
      return;
    }
    if (lane->points.front().x > 0) {
      Eigen::Vector2d pt(lane->points.front().x, lane->points.front().y);
      ego_lane_points_.push_back(std::move(pt));
    }
  }

  auto cur_group_id = ego_lane_group_id_;
  while (true) {
    auto& ego_lane_group = lane_groups_details_.at(cur_group_id);
    if (ego_lane_group.eles.empty()) {
      break;
    }
    for (auto ele : ego_lane_group.eles) {
      if (lane_map_.count(ele) == 0) {
        continue;
      }
      auto lane = lane_map_.at(ele).ptr;
      for (auto iter = lane->points.begin() + 1; iter < lane->points.end();
           ++iter) {
        Eigen::Vector2d pt(iter->x, iter->y);
        if (pt.x() < 0) {
          continue;
        }
        ego_lane_points_.push_back(std::move(pt));
      }
      ego_lane_candidate_eles_.push_back(ele);
    }
    ego_lane_candidate_groups_.emplace_back(cur_group_id);

    if (ego_lane_group.length > 50) {
      break;
    } else {
      if (connect_info_.count(ego_lane_group.eles.back()) == 0) {
        break;
      }
      auto& targets_to = connect_info_.at(ego_lane_group.eles.back()).target_to;
      int valid_target_cnt = 0;
      for (auto& target : targets_to) {
        if (lane_groups_details_.count(target.second.group_to) > 0) {
          cur_group_id = target.second.group_to;
          valid_target_cnt++;
        }
      }
      if (valid_target_cnt != 1) {
        break;
      }
    }
  }

  // AINFO << GetEgoLaneGroupDebugInfo();
  // AINFO << GetEgoLanePointsDebugInfo();
}

void LdMapProcessor::SearchRoutePath() {
  auto start_section_id = data_->route.navi_start.section_id;
  auto total_distance = -data_->route.navi_start.s_offset;
  auto iter_start = std::find_if(
      data_->route.sections.begin(), data_->route.sections.end(),
      [start_section_id](const cem::message::env_model::SectionInfo& section) {
        return section.id == start_section_id;
      });
  if (iter_start == data_->route.sections.end()) {
    return;
  }

  struct CellInfo {
    uint32_t id;
    uint64_t previous_section_key;
    std::unordered_set<uint32_t> relative_previous_cell_id;
    std::unordered_set<uint32_t> lane_ids;
    std::unordered_map<uint32_t, uint64_t>
        lane_ele_id;  // lane_group_id, lane_element_id
    double total_distance;
    double section_length;
    uint32_t left_cell;
    uint32_t right_cell;
    MergeSplitInfo merge_split_info;
    bool is_end = true;

    void PrintInfo() {
      std::string info = "";
      info = "id: " + std::to_string(id) +
             " previous_section_key: " + std::to_string(previous_section_key) +
             +" relative_previous_cell_id: {";
      for (const auto& id : relative_previous_cell_id) {
        info += std::to_string(id);
        info += " ";
      }
      info += "}";
      info += " lane_ids: {";
      for (const auto& id : lane_ids) {
        info += std::to_string(id);
        info += " ";
      }
      info += "}";
      info += " lane_ele_id: {";
      for (const auto& pair : lane_ele_id) {
        info += " lane_group_id: ";
        info += std::to_string(pair.first);
        info += ", lane_element_id: ";
        info += std::to_string(pair.second);
        info += " ";
      }
      info += "}";
      info += " total_distance: ";
      info += std::to_string(total_distance);
      info += " section_length: ";
      info += std::to_string(section_length);
      info += " left_cell: ";
      info += std::to_string(left_cell);
      info += " right_cell: ";
      info += std::to_string(right_cell);
      // AINFO << " merge_split_info: " << merge_split_info.info;
      // AINFO << info;
    }
  };
  struct SectionCell {
    uint64_t section_id;
    std::map<uint32_t, CellInfo> cells;
  };
  std::unordered_map<int, SectionCell> recommend_cells;

  // std::vector<uint64_t> section_sequence = std::vector<uint64_t>(1, 0);

  // AINFO << __LINE__;
  int section_cnt = 0;
  for (auto iter = iter_start; iter != data_->route.sections.end(); ++iter) {
    total_distance += iter->length;
    // AINFO << "-----------" << iter->id << "--------------";
    auto& section_cells = recommend_cells[++section_cnt];
    section_cells.section_id = iter->id;

    for (size_t i = 0; i < iter->lane_ids.size();) {
      if (lane_id_to_group_id_.count(iter->lane_ids.at(i)) == 0 ||
          lane_map_.count(iter->lane_ids.at(i)) == 0) {
        ++i;
        continue;
      }
      std::unordered_set<uint64_t> previous_lane_of_i;
      for (auto& id :
           lane_map_.at(iter->lane_ids.at(i)).ptr->previous_lane_ids) {
        previous_lane_of_i.insert(id);
      }
      size_t j = i + 1;
      while (true) {
        if (j >= iter->lane_ids.size()) {
          j = iter->lane_ids.size();
          break;
        }
        if (lane_id_to_group_id_.count(iter->lane_ids.at(j)) == 0 ||
            lane_map_.count(iter->lane_ids.at(j)) == 0) {
          break;
        }
        bool from_same_node = false;
        for (auto& id :
             lane_map_.at(iter->lane_ids.at(j)).ptr->previous_lane_ids) {
          if (previous_lane_of_i.count(id) != 0) {
            from_same_node = true;
            break;
          }
        }
        if (from_same_node) {
          j = j + 1;
        } else {
          break;
        }
      }
      CellInfo cell;
      cell.id = static_cast<uint32_t>(i + 1);
      cell.previous_section_key = section_cnt - 1;
      cell.total_distance = total_distance;
      cell.section_length = iter->length;
      for (size_t k = i; k < j; ++k) {
        if (lane_id_to_group_id_.count(iter->lane_ids.at(k)) == 0) {
          continue;
        }
        auto lane_id = lane_id_to_group_id_.at(iter->lane_ids.at(k));
        cell.lane_ids.emplace(lane_id);
        cell.lane_ele_id.emplace(lane_id, iter->lane_ids.at(k));

        if (lane_map_.count(iter->lane_ids.at(k)) > 0 &&
            recommend_cells.count(cell.previous_section_key) > 0) {
          auto& previous_section_cells =
              recommend_cells.at(cell.previous_section_key);
          for (auto& id :
               lane_map_.at(iter->lane_ids.at(k)).ptr->previous_lane_ids) {
            for (auto& previous_cell : previous_section_cells.cells) {
              for (auto& [group_id, ele_id] :
                   previous_cell.second.lane_ele_id) {
                if (id == ele_id) {
                  cell.relative_previous_cell_id.emplace(previous_cell.first);
                  previous_cell.second.is_end = false;
                  break;
                }
              }
            }
          }
        }
        cell.is_end = true;
      }
      section_cells.cells.emplace(static_cast<uint32_t>(i + 1),
                                  std::move(cell));  // 不存key为0,方便后面使用
      i = j;
    }
    // section_sequence.push_back(iter->id);

    int cnt_valid_cell = 0;
    for (auto iter_cell = section_cells.cells.begin();
         iter_cell != section_cells.cells.end(); ++iter_cell) {
      if (iter_cell != section_cells.cells.begin()) {
        iter_cell->second.left_cell = std::prev(iter_cell)->first;
      } else {
        iter_cell->second.left_cell = 0;
      }
      if (std::next(iter_cell) != section_cells.cells.end()) {
        iter_cell->second.right_cell = std::next(iter_cell)->first;
      } else {
        iter_cell->second.right_cell = 0;
      }
      if (!iter_cell->second.relative_previous_cell_id.empty()) {
        cnt_valid_cell++;
      }
    }
    // for (auto& cell : section_cells.cells) {
    //   cell.second.PrintInfo();
    //   AINFO << "<<<<<<>>>>>>";
    // }
    if (cnt_valid_cell <= 1 && iter > iter_start) {
      break;
    }
  }
  // AINFO << __LINE__;

  std::set<uint32_t> relative_cell_from_next_section;
  double left_threshold_keep_neighbor = 0;
  double right_threshold_keep_neighbor = 0;
  for (auto i = section_cnt; i > 0; --i) {
    // AINFO << __LINE__;
    auto& section_cells = recommend_cells.at(i);
    if (relative_cell_from_next_section.empty()) {
      // todo:
      // AINFO << __LINE__;
      if (section_cells.cells.size() > 0) {
        left_threshold_keep_neighbor =
            section_cells.cells.begin()->second.total_distance -
            section_cells.cells.begin()->second.section_length;
        right_threshold_keep_neighbor = left_threshold_keep_neighbor;
      }

    } else {
      // AINFO << __LINE__;
      std::set<uint32_t> relative_cell_to_previous_section;
      for (auto id : relative_cell_from_next_section) {
        if (!section_cells.cells.at(id).relative_previous_cell_id.empty()) {
          relative_cell_to_previous_section.emplace(id);
        }
      }
      // AINFO << __LINE__;
      if (!relative_cell_to_previous_section.empty()) {
        // AINFO << __LINE__;
        for (auto iter = section_cells.cells.begin();
             iter != section_cells.cells.end();) {
          if (relative_cell_to_previous_section.count(iter->second.id) != 0) {
            // 保留
          } else {
            if (*relative_cell_to_previous_section.begin() !=
                    iter->second.right_cell &&
                *std::prev(relative_cell_to_previous_section.end()) !=
                    iter->second.left_cell) {
              iter = section_cells.cells.erase(iter);
              continue;
            }
            if (*std::prev(relative_cell_to_previous_section.end()) ==
                iter->second.left_cell) {
              if (iter->second.is_end) {
                right_threshold_keep_neighbor = iter->second.total_distance;
              }
              if (right_threshold_keep_neighbor - iter->second.total_distance <
                  200) {
                iter = section_cells.cells.erase(iter);
                continue;
              }
            }
            if (*relative_cell_to_previous_section.begin() ==
                iter->second.right_cell) {
              if (iter->second.is_end) {
                left_threshold_keep_neighbor = iter->second.total_distance;
              }
              if (left_threshold_keep_neighbor - iter->second.total_distance <
                  200) {
                iter = section_cells.cells.erase(iter);
                continue;
              }
            }
          }
          ++iter;
        }
        // AINFO << __LINE__;
      } else {
        auto left_cell_iter =
            section_cells.cells.find(*relative_cell_from_next_section.begin());
        uint32_t left_cell_id = left_cell_iter->first;
        // AINFO << __LINE__;
        while (left_cell_iter != section_cells.cells.begin()) {
          left_cell_iter = std::prev(left_cell_iter);
          if (!left_cell_iter->second.relative_previous_cell_id.empty()) {
            left_cell_id = left_cell_iter->first;
            break;
          }
        }
        // AINFO << __LINE__;
        auto right_cell_iter = section_cells.cells.find(
            *std::prev(relative_cell_from_next_section.end()));
        uint32_t right_cell_id = right_cell_iter->first;
        // AINFO << __LINE__;
        while (std::next(right_cell_iter) != section_cells.cells.end()) {
          right_cell_iter = std::next(right_cell_iter);
          if (!right_cell_iter->second.relative_previous_cell_id.empty()) {
            right_cell_id = right_cell_iter->first;
            break;
          }
        }
        // AINFO << __LINE__;
        for (auto iter = section_cells.cells.begin();
             iter != section_cells.cells.end();) {
          if (iter->first < left_cell_id || iter->first > right_cell_id) {
            iter = section_cells.cells.erase(iter);
          } else {
            ++iter;
          }
        }
        // AINFO << __LINE__;
      }
    }
    // AINFO << __LINE__;
    relative_cell_from_next_section.clear();
    for (auto& cell : section_cells.cells) {
      for (auto id : cell.second.relative_previous_cell_id) {
        if (relative_cell_from_next_section.count(id) == 0) {
          relative_cell_from_next_section.emplace(id);
        }
      }
    }
    // std::string tmp = "------section:" + std::to_string(section_sequence.at(i));
    // AINFO << tmp << ", left_keep_dist:" << left_threshold_keep_neighbor
    //       << ", right_keep_dist:" << right_threshold_keep_neighbor;
    // for (auto& cell : section_cells) {
    //   cell.second.PrintInfo();
    // }
  }

  if (recommend_cells.size() > 1) {
    std::unordered_set<uint64_t> relative_ids;
    auto& second_section = recommend_cells.at(2);
    auto& first_section = recommend_cells.at(1);
    for (auto& cell : second_section.cells) {
      for (auto& [group_id, lane_ele_id] : cell.second.lane_ele_id) {
        relative_ids.emplace(lane_ele_id);
      }
    }
    for (auto& cell : first_section.cells) {
      for (auto iter = cell.second.lane_ele_id.begin();
           iter != cell.second.lane_ele_id.end();) {
        if (lane_map_.count(iter->second) == 0) {
          cell.second.lane_ids.erase(iter->first);
          iter = cell.second.lane_ele_id.erase(iter);
        } else {
          auto& lane = lane_map_.at(iter->second).ptr;
          bool keep_lane_element = false;
          for (auto id : lane->next_lane_ids) {
            if (relative_ids.count(id) > 0) {
              keep_lane_element = true;
              break;
            }
          }
          if (!keep_lane_element) {
            cell.second.lane_ids.erase(iter->first);
            iter = cell.second.lane_ele_id.erase(iter);
          } else {
            ++iter;
          }
        }
      }
    }
  }

  // AINFO << __LINE__;

  for (auto i = 1; i <= section_cnt; ++i) {
    for (auto& cell : recommend_cells.at(i).cells) {
      if (i > 1) {
        if (cell.second.relative_previous_cell_id.size() != 0) {
          continue;
        }
      }
      for (auto& lane_ele : cell.second.lane_ele_id) {
        Path path;
        path.lane_eles_id.emplace(lane_ele.second);
        path.start_ele_id = lane_ele.second;
        path.end_ele_id = lane_ele.second;
        std::pair<uint32_t, double> lane;
        lane.first = lane_ele.first;
        lane.second = cell.second.total_distance;
        path.lanes.push_back(std::move(lane));
        path.status = PathStatus::kNotUpdated;
        path.start_section_key = i;
        if (i == 1) {
          path.is_start = true;
          path.from_start_section = true;
        } else {
          path.is_start = false;
          path.from_start_section = false;
        }
        recommend_paths_.emplace_back(std::move(path));
      }
    }
    if (recommend_cells.at(i).cells.empty()) {
      break;
    } else {
      if (recommend_cells.at(i).cells.begin()->second.total_distance > 150) {
        break;
      }
    }
  }
  // AINFO << "--------- FIRST SECTION: ----------";
  // for (auto& path : recommend_paths_) {
  //   path.PrintPathInfo();
  // }
  // AINFO << "--------- FIRST SECTION: ----------";

  // AINFO << __LINE__;
  /*
  for (size_t i = 2; i <= section_cnt; ++i) {
    // AINFO << "section_id:" << section_sequence.at(i) << ", "
    //       << section_sequence.size();
    auto& cur_section = recommend_cells.at(i);
    std::unordered_map<uint32_t, uint64_t> lane_in_cur_section;
    double length = 0;
    for (auto& [cell_id, cell] : cur_section.cells) {
      for (auto& [lane_id, lane_ele_id] : cell.lane_ele_id) {
        lane_in_cur_section.emplace(lane_id, lane_ele_id);
      }
      if (length < 1e-6) {
        length = cell.total_distance;
      }
    }

    std::vector<Path> paths_not_classified;
    for (auto iter = recommend_paths_.begin();
         iter != recommend_paths_.end();) {
      if (i == iter->start_section_key) {
        iter->is_start = true;
        ++iter;
        continue;
      }
      if (lane_in_cur_section.count(iter->lanes.back().first) > 0) {
        iter->end_ele_id = lane_in_cur_section.at(iter->lanes.back().first);
        iter->lane_eles_id.emplace(iter->end_ele_id);
        iter->lanes.back().second = length;
        iter->status = PathStatus::kUpdated;
        ++iter;
      } else {
        if (iter->is_start) {
          paths_not_classified.emplace_back(std::move(*iter));
          iter = recommend_paths_.erase(iter);
        } else {
          ++iter;
        }
      }
    }

    // AINFO << "1111-------";
    // for (auto& path : paths_not_classified) {
    //   path.PrintPathInfo();
    // }
    // AINFO << "1111-------";

    for (auto& path : paths_not_classified) {
      // std::vector<MergeSplitTarget*>tmp;
      bool go_to_end = true;
      if (connect_info_.count(path.end_ele_id) > 0) {
        go_to_end = false;
        auto& targets = connect_info_.at(path.end_ele_id).target_to;
        std::vector<Path> path_tmp;
        for (auto& target : targets) {
          if (lane_in_cur_section.count(target.second.group_to) > 0) {
            Path tmp = path;
            tmp.lanes.emplace_back(
                std::pair<uint32_t, double>(target.second.group_to, length));
            tmp.end_ele_id = target.second.lane_to;
            tmp.lane_eles_id.emplace(tmp.end_ele_id);
            tmp.status = PathStatus::kUpdated;
            path_tmp.emplace_back(std::move(tmp));
          }
        }
        if (path_tmp.size() > 1) {
          for (auto& path : path_tmp) {
            path.change_point.push_back(std::pair<uint32_t, MergeSplitType>(
                (path.lanes.begin() + path.lanes.size() - 2)->first,
                MergeSplitType::kConnectForward));
          }
        }
        // for (auto& path : path_tmp) {
        //   path.PrintPathInfo();
        //   // recommend_paths_.emplace(path.first, std::move(path.second));
        // }
        recommend_paths_.insert(recommend_paths_.end(),
                                std::make_move_iterator(path_tmp.begin()),
                                std::make_move_iterator(path_tmp.end()));
        // AINFO << "--------- UPDATE SECTION1: ---------- (--"
        //       << section_sequence.at(i);
        // for (auto& path : recommend_paths_) {
        //   path.PrintPathInfo();
        // }
        // AINFO << "--------- UPDATE SECTION1: ---------- (--"
        //       << section_sequence.at(i);
      }

      if (split_info_.count(path.end_ele_id) > 0) {
        go_to_end = false;
        auto& targets = split_info_.at(path.end_ele_id).target_to;
        std::vector<Path> path_tmp;
        for (auto& target : targets) {
          if (lane_in_cur_section.count(target.second.group_to) > 0) {
            Path tmp = path;
            tmp.change_point.push_back(std::pair<uint32_t, MergeSplitType>(
                tmp.lanes.back().first, target.second.type));
            tmp.lanes.emplace_back(
                std::pair<uint32_t, double>(target.second.group_to, length));
            tmp.end_ele_id = target.second.lane_to;
            tmp.lane_eles_id.emplace(tmp.end_ele_id);
            tmp.status = PathStatus::kUpdated;
            path_tmp.emplace_back(std::move(tmp));
          }
        }
        recommend_paths_.insert(recommend_paths_.end(),
                                std::make_move_iterator(path_tmp.begin()),
                                std::make_move_iterator(path_tmp.end()));
      }

      if (merge_info_.count(path.end_ele_id) > 0) {
        go_to_end = false;
        auto& targets = merge_info_.at(path.end_ele_id).target_to;
        std::vector<Path> path_tmp;
        for (auto& target : targets) {
          if (lane_in_cur_section.count(target.second.group_to) > 0) {
            Path tmp = path;
            tmp.lanes.emplace_back(
                std::pair<uint32_t, double>(target.second.group_to, length));
            tmp.end_ele_id = target.second.lane_to;
            tmp.lane_eles_id.emplace(tmp.end_ele_id);
            tmp.status = PathStatus::kUpdated;
            path_tmp.emplace_back(std::move(tmp));
          }
        }
        recommend_paths_.insert(recommend_paths_.end(),
                                std::make_move_iterator(path_tmp.begin()),
                                std::make_move_iterator(path_tmp.end()));
      }
      if (go_to_end) {
        recommend_paths_.emplace_back(std::move(path));
      }
    }
    std::unordered_map<uint32_t, std::vector<size_t>> path_from_same_start;
    for (size_t i = 0; i < recommend_paths_.size(); ++i) {
      auto& path = recommend_paths_.at(i);
      if (path.status == PathStatus::kNotUpdated) {
        if (path_from_same_start.count(path.lanes.front().first) == 0) {
          path_from_same_start[path.lanes.front().first].push_back(i);
        } else {
          auto& cur_group = path_from_same_start.at(path.lanes.front().first);
          auto iter_insert = std::lower_bound(
              cur_group.begin(), cur_group.end(), i,
              [&](uint32_t a, uint32_t b) {
                return recommend_paths_.at(a).lanes.back().second >
                       recommend_paths_.at(b).lanes.back().second;
              });
          cur_group.insert(iter_insert, i);
        }
      } else {
        path.status = PathStatus::kNotUpdated;
      }
    }
    std::set<size_t> remove_ids;
    for (auto& [id, paths] : path_from_same_start) {
      if (paths.size() <= 1) {
        continue;
      }
      double max_length =
          recommend_paths_.at(paths.front()).lanes.back().second;
      auto iter_remove = paths.end();
      for (auto iter = paths.begin() + 1; iter != paths.end(); ++iter) {
        if (max_length - recommend_paths_.at(*iter).lanes.back().second > 1) {
          iter_remove = iter;
        }
      }
      for (auto iter = iter_remove; iter != paths.end(); ++iter) {
        if (remove_ids.count(*iter) == 0) {
          remove_ids.emplace(*iter);
        }
      }
    }
    // for (auto iter = remove_ids.rbegin(); iter != remove_ids.rend(); ++iter) {
    //   recommend_paths_.erase(recommend_paths_.begin() + *iter);
    // }
    // AINFO << "--------- UPDATE SECTION: ---------- (--"
    //       << section_sequence.at(i);
    // for (auto& path : recommend_paths_) {
    //   path.PrintPathInfo();
    // }
  }

  // for (auto& path : recommend_paths_) {
  //   path.PrintPathInfo();
  // }
  // AINFO << "---------------";

  std::unordered_set<uint32_t> lane_keeped;
  for (auto iter_first = recommend_paths_.begin();
       iter_first + 1 < recommend_paths_.end();) {
    // if
    bool erase_first = false;
    for (auto iter_second = iter_first + 1;
         iter_second != recommend_paths_.end();) {
      double same_length = 0;
      // iter_first->PrintPathInfo();
      for (size_t i = 0; true; i++) {
        if ((iter_first->lanes.size() <= i) ||
            (iter_second->lanes.size() <= i)) {
          break;
        }
        if (iter_first->lanes.at(i).first == iter_second->lanes.at(i).first) {
          same_length = iter_first->lanes.at(i).second;
          if (same_length > 150) {
            break;
          }
        } else {
          break;
        }
      }
      // AINFO << same_length;
      // iter_second->PrintPathInfo();
      if (same_length > 150) {
        if (iter_first->lanes.back().second >=
            iter_second->lanes.back().second) {
          iter_second = recommend_paths_.erase(iter_second);
        } else {
          iter_first = recommend_paths_.erase(iter_first);
          erase_first = true;
          break;
        }
      } else {
        ++iter_second;
      }
    }
    // AINFO << "<<<<<<<<<>>>>>>>>>";
    if (!erase_first) {
      ++iter_first;
    }
  }
  */

  // for (auto& path : recommend_paths_) {
  //   path.PrintPathInfo();
  // }
  // AINFO << "---------------------------";
}

/*
void LdMapProcessor::SearchRoutePath() {
  std::unordered_map<uint32_t, Path> paths_through_start_section;
  auto start_section_id = data_->route.navi_start.section_id;
  auto total_distance = -data_->route.navi_start.s_offset;
  auto iter_start = std::find_if(
      data_->route.sections.begin(), data_->route.sections.end(),
      [start_section_id](const cem::message::env_model::SectionInfo&
section) { return section.id == start_section_id;
      });
  if (iter_start == data_->route.sections.end()) {
    return;
  }
  total_distance += iter_start->length;
  for (auto id : iter_start->lane_ids) {
    if (lane_id_to_group_id_.count(id) == 0) {
      continue;
    }
    auto lane_group_id = lane_id_to_group_id_.at(id);
    Path tmp;
    tmp.start_ele_id = id;
    tmp.end_ele_id = id;
    tmp.lane = lane_group_id;
    tmp.status = PathStatus::kNotUpdated;
    tmp.distance = total_distance;
    // tmp.next_path = nullptr;
    paths_through_start_section.emplace(lane_group_id, std::move(tmp));
  }
  if (paths_through_start_section.empty()) {
    return;
  }

  auto update_connect_info = [&](Path& current_path, uint64_t
current_ele_id, std::unordered_map<uint32_t, Path>& paths_relative) { if
(connect_info_.count(current_ele_id) > 0) { auto previous_lane =
connect_info_.at(current_ele_id).group_from; if
(paths_relative.count(previous_lane) > 0) {
        paths_relative.at(previous_lane).status =
PathStatus::kConnectedForward;
        paths_relative.at(previous_lane).next_path.push_back(&current_path);
      }
    } else {
      if (lane_map_.count(current_ele_id) == 0) {
        return;
      }
      auto& cur_ele = lane_map_.at(current_ele_id);
      for (auto id : cur_ele.ptr->previous_lane_ids) {
        if (lane_id_to_group_id_.count(id) == 0) {
          continue;
        }
        auto relative_path_id = lane_id_to_group_id_.at(id);
        if (paths_relative.count(relative_path_id) == 0) {
          continue;
        }
        if (connect_info_.count(id) == 0) {
          continue;
        }
        if (connect_info_.at(id).group_to != current_path.lane) {
          continue;
        }
        paths_relative.at(relative_path_id).status =
            PathStatus::kConnectedForward;
        paths_relative.at(relative_path_id).next_path.push_back(&current_path);
      }
    }
  };
  auto update_merge_info = [&](Path& current_path, uint64_t current_ele_id,
                               std::unordered_map<uint32_t, Path>&
                                   paths_relative) {
    if (lane_map_.count(current_ele_id) == 0) {
      return;
    }
    auto& cur_ele = lane_map_.at(current_ele_id);
    for (auto id : cur_ele.ptr->previous_lane_ids) {
      if (lane_id_to_group_id_.count(id) == 0) {
        continue;
      }
      auto relative_path_id = lane_id_to_group_id_.at(id);
      if (paths_relative.count(relative_path_id) == 0) {
        continue;
      }
      if (merge_info_.count(id) == 0) {
        continue;
      }
      if (merge_info_.at(id).group_to != current_path.lane) {
        continue;
      }
      paths_relative.at(relative_path_id).status =
          PathStatus::kConnectedForward;
      paths_relative.at(relative_path_id).merge_path.push_back(&current_path);
    }
  };
  auto update_split_info = [&](Path& current_path, uint64_t current_ele_id,
                               std::unordered_map<uint32_t, Path>&
                                   paths_relative) {
    if (split_info_.count(current_ele_id) > 0) {
      auto previous_lane = split_info_.at(current_ele_id).group_from;
      if (paths_relative.count(previous_lane) > 0) {
        paths_relative.at(previous_lane).status =
PathStatus::kConnectedForward;
        paths_relative.at(previous_lane).split_path.push_back(&current_path);
      }
    }
  };

  std::unordered_map<uint32_t, Path> paths_forward;
  std::vector<uint32_t> paths_forward_ids;
  for (auto iter = iter_start + 1; iter != data_->route.sections.end();
       ++iter) {
    for (auto id : iter->lane_ids) {
      if (lane_id_to_group_id_.count(id) == 0 || lane_map_.count(id) == 0) {
        continue;
      }
      auto lane_group_id = lane_id_to_group_id_.at(id);
      if (paths_through_start_section.count(lane_group_id) > 0) {
        paths_through_start_section.at(lane_group_id).end_ele_id = id;
        paths_through_start_section.at(lane_group_id).status = kUpdated;
        paths_through_start_section.at(lane_group_id).distance +=
iter->length; } else { if (paths_forward.count(lane_group_id) == 0) { Path
tmp; tmp.start_ele_id = id; tmp.end_ele_id = id; tmp.lane = lane_group_id;
          tmp.status = PathStatus::kUpdated;
          tmp.distance = iter->length;
          // tmp.next_path = nullptr;
          update_connect_info(tmp, id, paths_forward);
          update_connect_info(tmp, id, paths_through_start_section);
          update_split_info(tmp, id, paths_forward);
          update_split_info(tmp, id, paths_through_start_section);
          update_merge_info(tmp, id, paths_forward);
          update_merge_info(tmp, id, paths_through_start_section);
          paths_forward.emplace(lane_group_id, std::move(tmp));
          paths_forward_ids.push_back(lane_group_id);
        } else {
          auto& tmp = paths_forward.at(lane_group_id);
          tmp.end_ele_id = id;
          tmp.distance += iter->length;
          tmp.status = PathStatus::kUpdated;
        }
      }
    }

    for (auto path_id_iter = paths_forward_ids.rbegin();
         path_id_iter != paths_forward_ids.rend(); ++path_id_iter) {
      if (paths_forward.count(*path_id_iter) == 0) {
        continue;
      }
      auto& path = paths_forward.at(*path_id_iter);
      for (auto iter_next = path.next_path.begin();
           iter_next != path.next_path.end();) {
        if ((*iter_next)->status == PathStatus::kNotUpdated) {
          iter_next = path.next_path.erase(iter_next);
        } else {
          iter_next++;
        }
      }
      for (auto iter_next = path.split_path.begin();
           iter_next != path.split_path.end();) {
        if ((*iter_next)->status == PathStatus::kNotUpdated) {
          iter_next = path.split_path.erase(iter_next);
        } else {
          iter_next++;
        }
      }
      for (auto iter_next = path.merge_path.begin();
           iter_next != path.merge_path.end();) {
        if ((*iter_next)->status == PathStatus::kNotUpdated) {
          iter_next = path.merge_path.erase(iter_next);
        } else {
          iter_next++;
        }
      }
      if (path.status != PathStatus::kUpdated) {
        if (path.next_path.empty() && path.split_path.empty() &&
            path.merge_path.empty()) {
          path.status = PathStatus::kNotUpdated;
        }
      }
    }
    int count_keep_recommend = 0;
    for (auto iter_through_start_section =
paths_through_start_section.begin(); iter_through_start_section !=
paths_through_start_section.begin();
         ++iter_through_start_section) {
      if (iter_through_start_section->second.status ==
          PathStatus::kConnectedForward) {
        for (auto iter_next =
                 iter_through_start_section->second.next_path.begin();
             iter_next !=
iter_through_start_section->second.next_path.end();) { if
((*iter_next)->status == PathStatus::kNotUpdated) { iter_next =
                iter_through_start_section->second.next_path.erase(iter_next);
          } else {
            iter_next++;
          }
        }
        for (auto iter_next =
                 iter_through_start_section->second.split_path.begin();
             iter_next !=
             iter_through_start_section->second.split_path.end();) {
          if ((*iter_next)->status == PathStatus::kNotUpdated) {
            iter_next =
                iter_through_start_section->second.split_path.erase(iter_next);
          } else {
            iter_next++;
          }
        }
        for (auto iter_next =
                 iter_through_start_section->second.merge_path.begin();
             iter_next !=
             iter_through_start_section->second.merge_path.end();) {
          if ((*iter_next)->status == PathStatus::kNotUpdated) {
            iter_next =
                iter_through_start_section->second.merge_path.erase(iter_next);
          } else {
            iter_next++;
          }
        }
        if (iter_through_start_section->second.next_path.empty() &&
            iter_through_start_section->second.split_path.empty() &&
            iter_through_start_section->second.merge_path.empty()) {
          iter_through_start_section->second.status =
PathStatus::kNotUpdated;
        }

        if (iter_through_start_section->second.status ==
            PathStatus::kNotUpdated) {
          count_keep_recommend += 1;
        }
      }
    }
    if (paths_through_start_section.size() - count_keep_recommend <= 1) {
      break;
    }

    for (auto iter_path_forward = paths_forward.begin();
         iter_path_forward != paths_forward.end();) {
      if (iter_path_forward->second.status == PathStatus::kNotUpdated) {
        iter_path_forward = paths_forward.erase(iter_path_forward);
      } else {
        iter_path_forward++;
      }
    }
  }
  for (auto& path : paths_through_start_section) {
    AINFO << path.second.distance;
    AINFO << path.first;
  }
}
  */

void LdMapProcessor::ClearCache() {
  lane_map_.clear();
  section_map_.clear();
  lane_id_to_group_id_.clear();
  lanes_.clear();
  for (auto iter = lane_group_id_tracked_.begin();
       iter != lane_group_id_tracked_.end();) {
    if (iter->second.is_exist) {
      iter->second.is_exist = false;
      ++iter;
    } else {
      iter = lane_group_id_tracked_.erase(iter);
    }
  }
  merge_info_.clear();
  split_info_.clear();
  connect_info_.clear();
  recommend_paths_.clear();
  lane_groups_details_.clear();
  ego_lane_group_id_ = 0;
  ego_lane_points_.clear();
  ego_lane_candidate_groups_.clear();
  ego_lane_candidate_eles_.clear();
}

void LdMapProcessor::JudgeIsOnFreewayOrNot() {
  if (data_->map_info.map_provider == MapProvider::Baidu) {
    function_mode_ = FunctionMode::kHNOA;
    is_on_freeway_ = true;
  } else {
    function_mode_ = FunctionMode::kMNOA;
    double search_distance = 200.0;
    if (!data_->is_on_highway) {
      search_distance = 800;
    }
    // AINFO << "search distance: " << search_distance;
    bool found_junction = false;
    auto iter_junction = data_->route.sections.end();
    bool found_freeway = false;
    auto start_section_id = data_->route.navi_start.section_id;
    auto start_section =
        std::find_if(data_->route.sections.begin(), data_->route.sections.end(),
                     [start_section_id](
                         const cem::message::env_model::SectionInfo& section) {
                       return section.id == start_section_id;
                     });
    if (start_section == data_->route.sections.end()) {
      return;
    }
    auto total_distance = data_->route.navi_start.s_offset;
    for (auto iter = start_section; iter != data_->route.sections.end();
         ++iter) {
      for (auto id : iter->lane_ids) {
        if (lane_map_.count(id) == 0) {
          continue;
        }
        auto lane = lane_map_.at(id).ptr;
        if ((lane->junction_id != 0) ||
            (lane->type == LaneType::LANE_LEFT_WAIT) ||
            (lane->type == LaneType::LANE_VIRTUAL_COMMON) ||
            (lane->type == LaneType::LANE_VIRTUAL_JUNCTION) ||
            (lane->type == LaneType::LANE_U_TURN_LANE) ||
            (lane->type == LaneType::LANE_RIGHT_TURN_LANE) ||
            (lane->type == LaneType::LANE_RIGHT_TURN_AREA) ||
            (lane->type == LaneType::LANE_U_TURN_AREA) ||
            (lane->type == LaneType::LANE_NO_TURN_AREA) ||
            (lane->type == LaneType::LANE_VIRTUAL_CONNECTED_LANE) ||
            (lane->type == LaneType::LANE_LEFT_TURN_LANE) ||
            (lane->type == LaneType::LANE_NO_TURN_AREA) ||
            (lane->type == LaneType::LANE_NO_TURN_AREA) ||
            (lane->type == LaneType::LANE_NO_TURN_AREA) ||
            (lane->turn_type == TurnType::U_TURN) ||
            (lane->turn_type == TurnType::STRAIGHT_AND_U_TURN) ||
            (lane->turn_type == TurnType::LEFT_TURN_AND_U_TURN) ||
            (lane->turn_type == TurnType::RIGHT_TURN_AND_U_TURN) ||
            (lane->turn_type == TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN) ||
            (lane->turn_type == TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN) ||
            (lane->turn_type ==
             TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN)) {
          found_junction = true;
          iter_junction = iter;
        }
      }

      if ((iter->road_class ==
           cem::message::env_model::RoadClass::EXPRESSWAY) ||
          (iter->road_class ==
           cem::message::env_model::RoadClass::URBAN_EXPRESSWAY)) {
        found_freeway = true;
      }

      total_distance += iter->length;
      if ((total_distance > search_distance) || found_junction) {
        break;
      }
    }

    // AINFO << "found junction: " << found_junction;
    // AINFO << "found freeway: " << found_freeway;
    if (data_->is_on_highway) {
      // 如果当前帧地图标志位是高速

      // 先判断junction所在的section是不是通往城区
      double search_distance = 0;
      double is_junction_to_city_scenario = true;
      for (auto iter = iter_junction + 1; iter < data_->route.sections.end();
           ++iter) {
        search_distance += iter->length;
        if ((iter->road_class ==
             cem::message::env_model::RoadClass::EXPRESSWAY) ||
            (iter->road_class ==
             cem::message::env_model::RoadClass::URBAN_EXPRESSWAY)) {
          is_junction_to_city_scenario = false;
          break;
        }
        if (search_distance > 200.0) {
          break;
        }
      }

      if (found_junction && is_junction_to_city_scenario) {
        // 1.junction通往城区时，如果靠近路口200m那就切为城区。
        is_on_freeway_ = false;
      } else {
        // 2. 自车在高速高架路上，就保持高速标志
        is_on_freeway_ = true;
      }
    } else {
      // 当前帧地图标志位是城区
      if ((!found_junction) && found_freeway) {
        // 1. 如果这一帧自车前方800m有高速高架路并且范围内没有路口
        is_on_freeway_ = true;
      } else {
        // 2. 否则的话
        is_on_freeway_ = false;
      }
    }
  }
}

void LdMapProcessor::SearchUnreliableRoad() {
  distance_to_unrealiable_road_ = 2000.0;
  auto search_distance = -data_->sd_route.navi_start.s_offset;

  std::unordered_map<uint64_t, cem::message::env_model::SDSectionInfo*>
      section_map;
  for (auto& section : data_->sd_route.mpp_sections) {
    section_map.emplace(section.id, &section);
  }

  auto cur_section_id = data_->sd_route.navi_start.section_id;
  while (true) {
    if (section_map.count(cur_section_id) == 0) {
      return;
    }

    auto cur_section = section_map.at(cur_section_id);
    if (cur_section->road_class >= SDRoadClass::SD_TOWNSHIP_ROAD) {
      distance_to_unrealiable_road_ =
          search_distance < 0.0 ? 0.0 : search_distance;
      break;
    }

    std::vector<uint64_t> next_section;
    for (auto id : cur_section->successor_section_id_list) {
      if (section_map.count(id) > 0) {
        next_section.emplace_back(id);
      }
    }
    if (next_section.size() != 1) {
      break;
    } else {
      cur_section_id = next_section.front();
    }
    search_distance += cur_section->length;
    if (search_distance > 2000) {
      break;
    }
  }
}

}  // namespace fusion
}  // namespace cem
