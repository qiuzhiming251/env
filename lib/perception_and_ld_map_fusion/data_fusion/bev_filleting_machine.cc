#include "lib/perception_and_ld_map_fusion/data_fusion/bev_filleting_machine.h"

namespace cem {
namespace fusion {

const double BevFilletingMachine::RELIABLE_RANGE_DISTANCE_THRESHOLD;

BevFilletingMachine::BevFilletingMachine(double slice_thickness)
    : FilletingMachine<cem::message::sensor::BevMapInfo>(slice_thickness) {
  bev_slice_thickness_ = 1;
  bev_slice_totallength_ = 90;
  lane_position_ = RoadSlice::kUncertain;
}

BevFilletingMachine::~BevFilletingMachine() {}

void BevFilletingMachine::Slice() {
  sampling_points_valid_.clear();
  bev_lanes_in_reliable_range_.clear();

  // auto& bev_lane_info = raw_data_->lane_infos;
  bev_slice_knife_direction_ = this->GetSliceKnifeDirection();
  bev_slice_start_point_ = this->GetSliceStartPoint();
  bev_slice_thickness_ = this->GetSliceThickness();
  bev_slice_totallength_ = this->GetSliceTotalLength();
  // AINFO << "bev_slice_knife_direction_: " << bev_slice_knife_direction_.x()
  //       << ", " << bev_slice_knife_direction_.y();
  Eigen::Vector2d bev_slice_knife_dirveh(bev_slice_knife_direction_.y(),
                                         -bev_slice_knife_direction_.x());

  size_t sampling_size =
      std::floor(bev_slice_totallength_ / bev_slice_thickness_) + 1;

  std::unordered_map<uint64_t, const cem::message::sensor::BevLaneMarker*>
      bev_lane_markers;
  for (auto& lane_marker : raw_data_->lanemarkers) {
    bev_lane_markers.emplace(lane_marker.id, &lane_marker);
  }

  for (int k = 0; (k * bev_slice_thickness_) <= bev_slice_totallength_; k++) {
    bev_slice_new_point_ = bev_slice_start_point_ +
                           k * bev_slice_thickness_ * bev_slice_knife_dirveh;
    // AINFO << "bev iteration " << k << ": (" << bev_slice_new_point_(0) << ","
    //       << bev_slice_new_point_(1) << ")";

    std::unique_ptr<RoadSlice> road_slice = std::make_unique<RoadSlice>(
        bev_slice_new_point_, bev_slice_knife_direction_);

    bool road_edge_once = false;

    for (auto& lane_info : raw_data_->lane_infos) {
      if (sampling_points_valid_.count(lane_info.id) == 0) {
        sampling_points_valid_.emplace(
            lane_info.id, std::move(std::vector<bool>(sampling_size, false)));
      }
      const auto& lane_points = lane_info.line_points;
      if (lane_points.size() < 3) {
        continue;
      }

      auto nearest_pts = this->FindNearestPts<Point2DF>(
          lane_points, bev_slice_knife_direction_, bev_slice_new_point_, true);
      // AINFO << "bev nearest_pt1: " << nearest_pts.first.x() << ", "
      //       << nearest_pts.first.y();
      // AINFO << "bev nearest_pt2: " << nearest_pts.second.x() << ", "
      //       << nearest_pts.second.y();
      if ((nearest_pts.first != Eigen::Vector2d::Zero()) &&
          (nearest_pts.second != Eigen::Vector2d::Zero())) {
        Eigen::Vector2d intsect_pt = this->FindIntersection(
            nearest_pts.first, nearest_pts.second, bev_slice_new_point_,
            bev_slice_knife_direction_);
        // AINFO << "bev intsect_pt: " << intsect_pt.x() << ", " <<
        // intsect_pt.y();

        auto left_line_id = lane_info.left_lane_marker_id;
        auto right_line_id = lane_info.right_lane_marker_id;
        auto left_line_type = cem::message::env_model::LineType::UNKNOWN;
        auto right_line_type = cem::message::env_model::LineType::UNKNOWN;

        int point_idx = -1;
        for (auto& lane_pt : lane_points) {
          point_idx++;
          if ((lane_pt.x == nearest_pts.first.x()) &&
              (lane_pt.y == nearest_pts.first.y())) {
            // AINFO << "index: " << point_idx <<": "<<  lane_pt.x() << "," <<
            // lane_pt.y();
            break;
          }
        }

        // auto it_left =
        //     std::find_if(bev_lane_marker.begin(), bev_lane_marker.end(),
        //                  [left_line_id](const BevLaneMarker& bev_lane_marker)
        //                  {
        //                    return bev_lane_marker.id == left_line_id;
        //                  });
        // if (it_left != bev_lane_marker.end()) {
        if (bev_lane_markers.count(left_line_id) > 0) {
          auto it_left = bev_lane_markers.at(left_line_id);
          for (auto& left_type : it_left->type_segs) {
            if (it_left->type_segs.size() == 1) {
              auto it = LineTypeToRoutingMap.find(left_type.type);
              if (it != LineTypeToRoutingMap.end()) {
                left_line_type = it->second;
              } else {
                left_line_type = cem::message::env_model::LineType::UNKNOWN;
              }
            } else {
              if ((left_type.start_index <= point_idx) &&
                  (left_type.end_index >= point_idx)) {
                auto it = LineTypeToRoutingMap.find(left_type.type);
                if (it != LineTypeToRoutingMap.end()) {
                  left_line_type = it->second;
                  // AINFO << "left_type start_index: " <<
                  // left_type.start_index; AINFO << "left_type end_index: " <<
                  // left_type.end_index; AINFO << "left_type type: " <<
                  // left_type.type;
                } else {
                  left_line_type = cem::message::env_model::LineType::UNKNOWN;
                }
                break;
              }
            }
          }
        } else {
          left_line_type = cem::message::env_model::LineType::UNKNOWN;
        }

        // auto it_right =
        //     std::find_if(bev_lane_marker.begin(), bev_lane_marker.end(),
        //                  [right_line_id](const BevLaneMarker&
        //                  bev_lane_marker) {
        //                    return bev_lane_marker.id == right_line_id;
        //                  });
        // if (it_right != bev_lane_marker.end()) {
        if (bev_lane_markers.count(right_line_id) > 0) {
          auto it_right = bev_lane_markers.at(right_line_id);
          for (auto& right_type : it_right->type_segs) {
            if (it_right->type_segs.size() == 1) {
              auto it = LineTypeToRoutingMap.find(right_type.type);
              if (it != LineTypeToRoutingMap.end()) {
                right_line_type = it->second;
              } else {
                right_line_type = cem::message::env_model::LineType::UNKNOWN;
              }
            } else {
              if ((right_type.start_index <= point_idx) &&
                  (right_type.end_index >= point_idx)) {
                auto it = LineTypeToRoutingMap.find(right_type.type);
                if (it != LineTypeToRoutingMap.end()) {
                  right_line_type = it->second;
                  // AINFO << "right_type start_index: " <<
                  // right_type.start_index; AINFO << "right_type end_index: "
                  // << right_type.end_index; AINFO << "right_type type: " <<
                  // right_type.type;
                } else {
                  right_line_type = cem::message::env_model::LineType::UNKNOWN;
                }
                break;
              }
            }
          }
        } else {
          right_line_type = cem::message::env_model::LineType::UNKNOWN;
        }

        // road boudary
        if (!road_edge_once) {
          edge_map_ = SliceRoadEdege();
          road_edge_once = true;
        }
        JudgeLanePosition(lane_info, intsect_pt);

        sampling_points_valid_.at(lane_info.id).at(k) = true;
        if ((bev_lanes_in_reliable_range_.count(lane_info.id) == 0) &&
            (bev_slice_new_point_.norm() <
             BevFilletingMachine::RELIABLE_RANGE_DISTANCE_THRESHOLD)) {
          bev_lanes_in_reliable_range_.emplace(lane_info.id);
        }
        road_slice->FillCell(intsect_pt, lane_info.id, left_line_type,
                             right_line_type, lane_position_);
      } else {
        // AINFO << "bev no intsect_pt";
      }
    }
    // AINFO << road_slice->GenerateSlicePrintInfo();
    road_slices_[k] = std::move(road_slice);
  }

  // // clear too long but isolate lane sampling points
  // double start_pt_position = bev_slice_start_point_.norm();
  // if (bev_slice_start_point_.x() < 0) {
  //   start_pt_position = -start_pt_position;
  // }
  // for (int k = std::floor(bev_slice_totallength_ / bev_slice_thickness_);
  //      k >= 0; --k) {
  //   if (!road_slices_.at(k)->ClearIsolatedPoint()) {
  //     break;
  //   }
  //   if (k * bev_slice_thickness_ + start_pt_position < 50) {
  //     break;
  //   }
  // }
}

void BevFilletingMachine::JudgeLanePosition(const BevLaneInfo& lane_info,
                                            const Eigen::Vector2d& intsect_pt) {
  // AINFO << "bev lane intsect_pt: " << intsect_pt.x() << "," <<
  // intsect_pt.y();
  lane_position_ = RoadSlice::kUncertain;
  if (edge_map_.count(BEV_REP__LEFT) > 0) {
    if ((lane_info.left_lane_id == 0) && (lane_info.right_lane_id != 0)) {
      // AINFO << "bev edge intsect_pt: " << edge_map_[BEV_REP__LEFT].x() << ","
      //       << edge_map_[BEV_REP__LEFT].y();
      double dis_left = DistanceEuclidean(intsect_pt, edge_map_[BEV_REP__LEFT]);
      if (dis_left < 3.0) {
        lane_position_ = RoadSlice::kCloseLeftRoadEdge;
      } else {
        lane_position_ = RoadSlice::kUncertain;
      }
    }
  } else {
    lane_position_ = RoadSlice::kUncertain;
  }

  if (edge_map_.count(BEV_REP__RIGHT) > 0) {
    if ((lane_info.left_lane_id != 0) && (lane_info.right_lane_id == 0)) {
      double dis_right =
          DistanceEuclidean(intsect_pt, edge_map_[BEV_REP__RIGHT]);
      if (dis_right < 3.0) {
        lane_position_ = RoadSlice::kCloseRightRoadEdge;
      } else {
        lane_position_ = RoadSlice::kUncertain;
      }
    }
  } else {
    lane_position_ = RoadSlice::kUncertain;
  }

  // AINFO << "bev lane_position_: " << lane_position_;
}

std::map<BevRoadEdgePosition, Eigen::Vector2d>
BevFilletingMachine::SliceRoadEdege() {
  auto& bev_lane_edges = raw_data_->edges;
  std::map<BevRoadEdgePosition, Eigen::Vector2d> edge_map;
  for (auto& lane_edge : bev_lane_edges) {
    const auto& edge_points = lane_edge.line_points;
    if ((edge_points.size() < 3) ||
        (lane_edge.road_edeg_pos == BevRoadEdgePosition::BEV_REP__UNKNOWN)) {
      continue;
    }

    auto nearest_pts = this->FindNearestPts<Point2DF>(
        edge_points, bev_slice_knife_direction_, bev_slice_new_point_, true);
    if ((nearest_pts.first != Eigen::Vector2d::Zero()) &&
        (nearest_pts.second != Eigen::Vector2d::Zero())) {
      Eigen::Vector2d intsect_edge_pt = this->FindIntersection(
          nearest_pts.first, nearest_pts.second, bev_slice_new_point_,
          bev_slice_knife_direction_);
      // AINFO << "bev edge intsect_pt: " << intsect_edge_pt.x() << ", "
      //       << intsect_edge_pt.y();

      edge_map[lane_edge.road_edeg_pos] = intsect_edge_pt;
    } else {
      // AINFO << "bev no edge intsect_pt";
    }
  }

  // for (const auto& edge : edge_map) {
  //   AINFO << "edge position: " << edge.first << ", edge pt: " << edge.second;
  // }
  return edge_map;
}

double BevFilletingMachine::DistanceEuclidean(const Eigen::Vector2d& P1,
                                              const Eigen::Vector2d& P2) {
  double dx = P1.x() - P2.x();
  double dy = P1.y() - P2.y();
  double dis = std::sqrt(dx * dx + dy * dy);
  // AINFO << "DistanceEuclidean: " << dis;
  return dis;
}

bool BevFilletingMachine::IsLaneAatTheLeftofLaneB(uint64_t a, uint64_t b,
                                                  bool& valid) {
  int count_a_at_the_left = 0;
  int count_a_at_the_right = 0;
  for (auto& slice : road_slices_) {
    if (slice.second->IsLaneAatTheLeftofLaneB(a, b)) {
      count_a_at_the_left++;
    }
    if (slice.second->IsLaneAatTheRightofLaneB(a, b)) {
      count_a_at_the_right++;
    }
  }
  if ((count_a_at_the_left == 0) && (count_a_at_the_right == 0)) {
    valid = false;
  } else {
    valid = true;
  }
  if (count_a_at_the_left > count_a_at_the_right) {
    return true;
  } else {
    return false;
  }
}

bool BevFilletingMachine::IsLaneInReliableRange(uint64_t id) {
  return bev_lanes_in_reliable_range_.count(id) > 0;
};

}  // namespace fusion
}  // namespace cem
