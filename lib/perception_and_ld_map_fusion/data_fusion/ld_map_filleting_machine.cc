#include "lib/perception_and_ld_map_fusion/data_fusion/ld_map_filleting_machine.h"

#include "lib/perception_and_ld_map_fusion/common/ld_lane_common_process.h"

namespace cem {
namespace fusion {

LdMapFilletingMachine::LdMapFilletingMachine(double slice_thickness)
    : FilletingMachine<cem::message::env_model::RoutingMap>(slice_thickness) {
  ldmap_slice_thickness_ = 1;
  ldmap_slice_totallength_ = 90;
  lane_position_ = RoadSlice::kUncertain;
}

LdMapFilletingMachine::~LdMapFilletingMachine() {}

void LdMapFilletingMachine::Slice() {
  // auto& ldmap_lane_info = raw_data_->lanes;
  // auto& ldmap_lane_marker = raw_data_->lane_boundaries;
  ldmap_slice_knife_direction_ = this->GetSliceKnifeDirection();
  ldmap_slice_start_point_ = this->GetSliceStartPoint();
  ldmap_slice_thickness_ = this->GetSliceThickness();
  ldmap_slice_totallength_ = this->GetSliceTotalLength();
  // AINFO << "ldmap_slice_knife_direction_: " <<
  // ldmap_slice_knife_direction_.x()
  //       << ", " << ldmap_slice_knife_direction_.y();
  Eigen::Vector2d ldmap_slice_knife_dirveh(ldmap_slice_knife_direction_.y(),
                                           -ldmap_slice_knife_direction_.x());

  std::unordered_map<uint64_t, const cem::message::env_model::LaneBoundaryInfo*>
      ldmap_lane_markers;
  for (auto& lane_marker : raw_data_->lane_boundaries) {
    ldmap_lane_markers.emplace(lane_marker.id, &lane_marker);
  }

  for (int k = 0; (k * ldmap_slice_thickness_) <= ldmap_slice_totallength_;
       k++) {
    Eigen::Vector2d ldmap_slice_new_point_ =
        ldmap_slice_start_point_ +
        k * ldmap_slice_thickness_ * ldmap_slice_knife_dirveh;
    // AINFO << "ldmap iteration " << k << ": (" << ldmap_slice_new_point_(0)
    //       << ", " << ldmap_slice_new_point_(1) << ")";

    std::unique_ptr<RoadSlice> road_slice = std::make_unique<RoadSlice>(
        ldmap_slice_new_point_, ldmap_slice_knife_direction_);

    for (auto& lane_info : raw_data_->lanes) {
      const auto& lane_points = lane_info.points;
      if (lane_points.size() < 3) {
        continue;
      }
      // if (!IsLdMapLaneValid(lane_info)) {
      //   continue;
      // }
      auto nearest_pts =
          this->FindNearestPts<Point>(lane_points, ldmap_slice_knife_direction_,
                                      ldmap_slice_new_point_, false);
      // AINFO << "ldmap nearest_pt1: " << nearest_pts.first.x() << ", " <<
      // nearest_pts.first.y(); AINFO << "ldmap nearest_pt2: " <<
      // nearest_pts.second.x() << ", " << nearest_pts.second.y();
      if ((nearest_pts.first != Eigen::Vector2d::Zero()) &&
          (nearest_pts.second != Eigen::Vector2d::Zero())) {
        Eigen::Vector2d intsect_pt = this->FindIntersection(
            nearest_pts.first, nearest_pts.second, ldmap_slice_new_point_,
            ldmap_slice_knife_direction_);
        // AINFO << "ldmap intsect_pt: " << intsect_pt.x() << ", "
        //       << intsect_pt.y();

        // AINFO << "ldmap left_lanemarker_id size(): " <<
        // lane_info.left_lane_boundary_ids.size(); AINFO << "ldmap
        // right_lanemarker_id size(): " <<
        // lane_info.right_lane_boundary_ids.size();

        // left and right lane marker id
        auto left_line_type = cem::message::env_model::LineType::UNKNOWN;
        auto right_line_type = cem::message::env_model::LineType::UNKNOWN;

        if ((lane_info.left_lane_boundary_ids.size() == 1) &&
            (lane_info.right_lane_boundary_ids.size() == 1)) {
          auto left_line_id = lane_info.left_lane_boundary_ids.at(0);
          // auto it_left = std::find_if(
          //     ldmap_lane_marker.begin(), ldmap_lane_marker.end(),
          //     [left_line_id](const LaneBoundaryInfo& ldmap_lane_marker) {
          //       return ldmap_lane_marker.id == left_line_id;
          //     });
          // if (it_left != ldmap_lane_marker.end()) {
          if (ldmap_lane_markers.count(left_line_id) > 0) {
            left_line_type = ldmap_lane_markers.at(left_line_id)->line_type;
          } else {
            left_line_type = cem::message::env_model::LineType::UNKNOWN;
          }
        } else {
          // not the only left line
        }

        if ((lane_info.right_lane_boundary_ids.size() == 1) &&
            (lane_info.right_lane_boundary_ids.size() == 1)) {
          auto right_line_id = lane_info.right_lane_boundary_ids.at(0);
          // auto it_right = std::find_if(
          //     ldmap_lane_marker.begin(), ldmap_lane_marker.end(),
          //     [right_line_id](const LaneBoundaryInfo& ldmap_lane_marker) {
          //       return ldmap_lane_marker.id == right_line_id;
          //     });
          // if (it_right != ldmap_lane_marker.end()) {
          if (ldmap_lane_markers.count(right_line_id) > 0) {
            right_line_type = ldmap_lane_markers.at(right_line_id)->line_type;
          } else {
            right_line_type = cem::message::env_model::LineType::UNKNOWN;
          }
        } else {
          // not the only left line
        }

        // road boundary
        JudgeLanePosition(lane_info, intsect_pt);

        road_slice->FillCell(intsect_pt, lane_info.id, left_line_type,
                             right_line_type, lane_position_);
      } else {
        // AINFO << "ldmap no intsect_pt";
      }
    }
    // AINFO << road_slice->GenerateSlicePrintInfo();
    road_slices_[k] = std::move(road_slice);
  }
}

void LdMapFilletingMachine::JudgeLanePosition(
    const LaneInfo& lane_info, const Eigen::Vector2d& intsect_pt) {
  lane_position_ = RoadSlice::kUncertain;
  if ((lane_info.left_lane_id == 0) && (lane_info.right_lane_id != 0)) {
    lane_position_ = RoadSlice::kCloseLeftRoadEdge;
  } else if ((lane_info.left_lane_id != 0) && (lane_info.right_lane_id == 0)) {
    lane_position_ = RoadSlice::kCloseRightRoadEdge;
  } else if ((lane_info.left_lane_id == 0) && (lane_info.right_lane_id == 0)) {
    lane_position_ = RoadSlice::kUniqueLane;
  } else {
    lane_position_ = RoadSlice::kUncertain;
  }
  // AINFO << "ld_map lane_position_: " << lane_position_;
}

}  // namespace fusion
}  // namespace cem
