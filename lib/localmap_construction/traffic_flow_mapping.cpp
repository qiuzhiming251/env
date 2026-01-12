#include <optional>
#include "traffic_flow_mapping.h"

namespace cem {
namespace fusion {

namespace {
constexpr uint64_t kMaxBevTrackID = 100;
constexpr int kMaxConsecutiveInversible = 15;
constexpr size_t kMinValidPointsNumber = 20;
constexpr size_t kMinSplitRegionPointsNumber = 6;
constexpr size_t kMinAssociatedPointsNumber = 10;
constexpr size_t kCompensationPointsNumber = 16;
constexpr double kDistanceRangeCalculation = 500.0;
constexpr double kTrackingLossTimeThreshold = 5.0;
constexpr double kXDirectionAssociationThreshold = 5.0;
constexpr double kYDirectionAssociationThreshold = 1.5;
constexpr double kYDirectionUnassociationThreshold = 2.0;
constexpr double kXDirectionBevLaneConstraint = 20;
constexpr double kMaxObservationRange = 100;
constexpr uint64_t kMinMappedLaneID = 900;
constexpr uint64_t kMaxMappedLaneID = 1000;
} // namespace

TrafficFlowMapping::TrafficFlowMapping() {
    basic_mapping_lane_id_ = kMinMappedLaneID;
}

TrafficFlowMapping::~TrafficFlowMapping() {

}

void TrafficFlowMapping::Process(const EnvInfo& env_info) {
    // if (env_info.v2_dist_to_ramp > kDistanceRangeCalculation || !PreProcessor()) {
    //     return;
    // }

    if (!PreProcessor()) {
        return;
    }

    TrafficFlowTracking();

    if (!CalculateLocalToEgo()) {
        return;
    }

    SetValidTrafficFlow();

    UpdateEgoPoints();

    // FusionInLocalMap();

    CompensationTopologySucessor();
}

void TrafficFlowMapping::CompensationTopologySucessor() {
    // association between traffic flow and bev lane
    for (auto& traffic_flow : traffic_flow_) {
        traffic_flow.second.associated_bev_lane_index.clear();
    }

    for (auto& traffic_flow : traffic_flow_) {
        if (!traffic_flow.second.is_valid) {
            continue;
        }
        for (const auto& bev_lane : bev_map_ptr_->lane_infos) {
            if (!IsOverlapping(traffic_flow.second, bev_lane)) {
                continue;
            }
            AssociateBevLaneInNormalRegion(traffic_flow.second, bev_lane);
        }
    }

    std::unordered_map<uint64_t, std::set<uint32_t>> bev_lane_with_flow_index;
    for (const auto& traffic_flow : traffic_flow_) {
        for (const auto associated_lane_id : traffic_flow.second.associated_bev_lane_index) {
            bev_lane_with_flow_index[associated_lane_id].insert(traffic_flow.first);
        }
    }

    UpdateLength();

    std::unordered_map<uint32_t, uint64_t> selected_traffic_flow_index;
    for (const auto& lane_with_flow_pair : bev_lane_with_flow_index) {
        if (lane_with_flow_pair.second.empty()) {
            continue;
        }

        auto associated_lane = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
            [find_id = lane_with_flow_pair.first](BevLaneInfo &lane) { return lane.id == find_id; });

        if (associated_lane == bev_map_ptr_->lane_infos.end()) {
            continue;
        }

        if (!RequireCompensationForSucessors(*associated_lane)) {
            continue;
        }

        for (const auto flow_id : lane_with_flow_pair.second) {
            if (traffic_flow_[flow_id].ego_points.empty()) {
                continue;
            }
            if (traffic_flow_[flow_id].ego_points.back().x() > associated_lane->line_points.back().x) {
                selected_traffic_flow_index[flow_id] = lane_with_flow_pair.first;
                break;
            }
        }
    }

    for (const auto& flow_with_lane_pair : selected_traffic_flow_index) {
        auto associated_lane = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
            [find_id = flow_with_lane_pair.second](BevLaneInfo &lane) { return lane.id == find_id; });
        if (associated_lane == bev_map_ptr_->lane_infos.end()) {
            continue;
        }

        BevLaneInfo fused_lane;
        for (const auto next_lane_id : associated_lane->next_lane_ids) {
            auto next_lane = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
                [find_id = next_lane_id](BevLaneInfo &lane) { return lane.id == find_id; });
            if (next_lane == bev_map_ptr_->lane_infos.end()) {
                fused_lane.id = next_lane_id;
                break;
            }
        }

        fused_lane.is_flow_map = true;
        fused_lane.line_points.emplace_back(associated_lane->line_points.back());
        fused_lane.geos->emplace_back(fused_lane.line_points.front().x, fused_lane.line_points.front().y);
        double start_x = fused_lane.line_points.back().x;
        Eigen::Vector2d velocity_direction = traffic_flow_[flow_with_lane_pair.first].velocity.head(2);
        velocity_direction.normalize();
        for (size_t i = 0; i < traffic_flow_[flow_with_lane_pair.first].ego_points.size(); i++) {
            if (traffic_flow_[flow_with_lane_pair.first].ego_points[i].x() > start_x) {
                cem::message::common::Point2DF lane_point;
                for (size_t j = 1; j < kCompensationPointsNumber; j++) {
                    lane_point.x = fused_lane.line_points.front().x + j * velocity_direction.x();
                    lane_point.y = fused_lane.line_points.front().y + j * velocity_direction.y();
                    fused_lane.line_points.emplace_back(lane_point);
                    fused_lane.geos->emplace_back(lane_point.x, lane_point.y);
                }
                // lane_point.x = traffic_flow_[flow_with_lane_pair.first].ego_points[i].x();
                // lane_point.y = traffic_flow_[flow_with_lane_pair.first].ego_points[i].y();
                // fused_lane.line_points.emplace_back(lane_point);
                break;
            }
        }

        if (fused_lane.line_points.size() < 2) {
            continue;
        }

        fused_lane.previous_lane_ids.push_back(flow_with_lane_pair.second);
        bev_map_ptr_->lane_infos.emplace_back(fused_lane);
    }
}

bool TrafficFlowMapping::RequireCompensationForSucessors(const BevLaneInfo& bev_lane) {
    if (bev_lane.next_lane_ids.size() != 2) {
        return false;
    }

    bool is_split_lane = false;
    size_t successor_number = 0;
    for (auto next_lane_id : bev_lane.next_lane_ids) {
        auto next_lane = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
            [find_id = next_lane_id](BevLaneInfo &lane) { return lane.id == find_id; });
        if (next_lane != bev_map_ptr_->lane_infos.end()) {
            successor_number++;
        }

        if (next_lane->split_topo_extend != SplitTopoExtendType::TOPOLOGY_SPLIT_NONE) {
            is_split_lane = true;
        }

        if (next_lane->connect_type != BevLaneConnectType::NORMAL) {
            is_split_lane = true;
        }
    }

    if (successor_number == 1 && is_split_lane) {
        return true;
    }

    return false;
}

void TrafficFlowMapping::UpdateLength() {
    for (auto& traffic_flow : traffic_flow_) {
        if (!traffic_flow.second.is_valid || traffic_flow.second.ego_points.size() < 2) {
            continue;
        }
        double length = 0;
        for (size_t i = 0; i < traffic_flow.second.ego_points.size() - 1; i++) {
            length += (traffic_flow.second.ego_points[i + 1].head(2) - traffic_flow.second.ego_points[i].head(2)).norm();
        }
        traffic_flow.second.length = length;
    }
}

void TrafficFlowMapping::AssociateBevLaneInNormalRegion(ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane) {
    size_t starting_point_id = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < trajectory.ego_points.size(); i++) {
        if (trajectory.ego_points[i].x() < bev_lane.line_points.front().x - kXDirectionAssociationThreshold) {
            continue;
        }
        starting_point_id = i;
        break;
    }

    if (starting_point_id == std::numeric_limits<size_t>::max()) {
        return;
    }

    std::pair<size_t, size_t> associated_bev_region_id_range = std::make_pair(0, 0);

    bool is_prev_associated = false;
    for (size_t i = starting_point_id; i < trajectory.ego_points.size(); i++) {
        bool is_associated = false;
        Eigen::Vector2d trajectory_point(trajectory.ego_points[i].x(), trajectory.ego_points[i].y());
        for (size_t j = 0; j < bev_lane.line_points.size() - 1; j++) {
            if (trajectory_point.x() > bev_lane.line_points[j].x && trajectory_point.x() <= bev_lane.line_points[j + 1].x) {
                Eigen::Vector2d pt1(bev_lane.line_points[j].x, bev_lane.line_points[j].y);
                Eigen::Vector2d pt1_to_pt2(bev_lane.line_points[j + 1].x - pt1.x(),
                                           bev_lane.line_points[j + 1].y - pt1.y());
                Eigen::Vector2d pt1_to_trajectory_point = trajectory_point - pt1;

                double distance_to_lane = std::abs((pt1_to_trajectory_point.x() * pt1_to_pt2.y() - pt1_to_trajectory_point.y() * pt1_to_pt2.x())) / pt1_to_pt2.norm();
                if (distance_to_lane <= kYDirectionAssociationThreshold) {
                    is_associated = true;
                } else {
                    is_associated = false;
                }
                // find association region
                if (is_associated && !is_prev_associated) {
                    associated_bev_region_id_range.first = j;
                }
                if (is_associated && is_prev_associated) {
                    associated_bev_region_id_range.second = j;
                }
                is_prev_associated = is_associated;
                break;
            }
        }

        if (associated_bev_region_id_range.second - associated_bev_region_id_range.first >= kMinAssociatedPointsNumber) {
            trajectory.associated_bev_lane_index.push_back(bev_lane.id);
            trajectory.associated_bev_region_id_range[bev_lane.id] = associated_bev_region_id_range;
            return;
        }
    }
}

bool TrafficFlowMapping::IsOverlapping(const ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane) {
    if (bev_lane.line_points.size() < kMinValidPointsNumber) {
        return false;
    }

    if (bev_lane.line_points.front().x > kXDirectionBevLaneConstraint) {
        return false;
    }

    if (trajectory.ego_points.front().x() > bev_lane.line_points.back().x ||
        trajectory.ego_points.back().x() < bev_lane.line_points.front().x) {
        return false;
    }

    return true;
}

void TrafficFlowMapping::FusionInLocalMap() {
    // association between traffic flow and bev lane
    for (auto& traffic_flow : traffic_flow_) {
        if (!traffic_flow.second.is_valid) {
            continue;
        }
        for (const auto& bev_lane : bev_map_ptr_->lane_infos) {
            AssociateBevLaneInSplitRegion(traffic_flow.second, bev_lane);
        }
    }

    std::unordered_map<uint64_t, std::set<uint32_t>> bev_lane_with_flow_index;
    for (const auto& traffic_flow : traffic_flow_) {
        for (const auto associated_lane_id : traffic_flow.second.associated_bev_lane_index) {
            bev_lane_with_flow_index[associated_lane_id].insert(traffic_flow.first);
        }
    }

    UpdateLength();

    // for (const auto& flow_index : bev_lane_with_flow_index) {
    //     AINFO << "flow_index.first(lane id):  " << flow_index.first;
    //     for (auto flow_id : flow_index.second) {
    //         AINFO << "flow_id:  " << flow_id;
    //     }
    // }

    // select the max length of traffic flows
    std::unordered_map<uint32_t, uint64_t> selected_traffic_flow_index;
    for (const auto& flow_index : bev_lane_with_flow_index) {
        auto associated_lane = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
            [find_id = flow_index.first](BevLaneInfo &lane) { return lane.id == find_id; });

        if (associated_lane != bev_map_ptr_->lane_infos.end()) {
            if (!associated_lane->previous_lane_ids.empty() || associated_lane->next_lane_ids.size() > 1) {
                continue;
            }
        }

        if (flow_index.second.empty()) {
            continue;
        }

        double max_length = std::numeric_limits<double>::min();
        uint32_t traffic_flow_id = 0;
        for (const auto flow_id : flow_index.second) {
            if (traffic_flow_[flow_id].length > max_length) {
                traffic_flow_id = flow_id;
                max_length = traffic_flow_[flow_id].length;
            }
        }
        selected_traffic_flow_index[traffic_flow_id] = flow_index.first;
    }

    for (const auto& selected_traffic_flow_id : selected_traffic_flow_index) {
        BevLaneInfo fused_lane;
        fused_lane.is_flow_map = true;
        const auto& trajectory = traffic_flow_[selected_traffic_flow_id.first];
        fused_lane.id = trajectory.tracked_lane_id;
        size_t begin_id = 0;
        if (trajectory.split_id_in_trajectory.count(selected_traffic_flow_id.second) &&
            trajectory.split_id_in_trajectory.at(selected_traffic_flow_id.second) < trajectory.points.size()) {
            begin_id = trajectory.split_id_in_trajectory.at(selected_traffic_flow_id.second);
        }

        for (size_t i = begin_id; i < trajectory.points.size(); i++) {
            Eigen::Vector3d ego_point = T_ego_local_bev_ * trajectory.points[i];
            cem::message::common::Point2DF lane_point;
            lane_point.x = ego_point.x();
            lane_point.y = ego_point.y();
            fused_lane.line_points.emplace_back(lane_point);
        }

        ConstructTopology(selected_traffic_flow_id, trajectory, fused_lane);
    }
}

void TrafficFlowMapping::UpdateEgoPoints() {
    for (auto& traffic_flow : traffic_flow_) {
        if (!traffic_flow.second.is_valid) {
            continue;
        }
        traffic_flow.second.ego_points.resize(traffic_flow.second.points.size());
        for (size_t i = 0; i < traffic_flow.second.points.size(); i++) {
            traffic_flow.second.ego_points.at(i) = T_ego_local_bev_ * traffic_flow.second.points.at(i);
        }
    }
}

bool TrafficFlowMapping::CalculateLocalToEgo() {
    SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_ptr_);
    if (!bev_map_ptr_) {
        return false;
    }

    LocalizationPtr bev_pose_ptr{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_ptr_->header.timestamp, 0.05, bev_pose_ptr);
    if (!bev_pose_ptr || prev_bev_timestamp_ == bev_map_ptr_->header.timestamp) {
        prev_bev_timestamp_ = bev_map_ptr_->header.timestamp;
        return false;
    }

    Eigen::Isometry3d T_local_ego_bev;
    T_local_ego_bev.translation() = Eigen::Vector3d(bev_pose_ptr->posne_dr.at(0),
                                                    bev_pose_ptr->posne_dr.at(1),
                                                                              0);

    T_local_ego_bev.linear() = Eigen::AngleAxisd(bev_pose_ptr->attitude_dr * M_PI / 180.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T_ego_local_bev_ = T_local_ego_bev.inverse();

    return true;
}

void TrafficFlowMapping::TrafficFlowTracking() {
    for (auto& traffic_flow : traffic_flow_) {
        const auto fused_obstacle_iterator = std::find_if(fusion_object_ptr_->fused_obstacles.begin(),
            fusion_object_ptr_->fused_obstacles.end(),
            [find_id = traffic_flow.first] (const FusionObj& fusion_obj) { return fusion_obj.id == find_id; });

        if (fused_obstacle_iterator != fusion_object_ptr_->fused_obstacles.end()) {
            traffic_flow.second.Update(true, *fused_obstacle_iterator);
        } else {
            FusionObj init_object;
            traffic_flow.second.Update(false, init_object);
        }
    }

    for (const auto& fused_obstacle : fusion_object_ptr_->fused_obstacles) {
        Eigen::Vector3d ego_point = T_ego_local_ * fused_obstacle.position;
        if (ego_point.head(2).norm() >= kMaxObservationRange) {
            continue;
        }
        if (!traffic_flow_.count(fused_obstacle.id)) {
            basic_mapping_lane_id_++;
            traffic_flow_[fused_obstacle.id].AddTrack(fused_obstacle, basic_mapping_lane_id_);
            if (basic_mapping_lane_id_ >= kMaxMappedLaneID) {
                basic_mapping_lane_id_ = kMinMappedLaneID;
            }
        }
    }

    for (auto it = traffic_flow_.begin(); it != traffic_flow_.end();) {
        if (ShouldErase(it->second)) {
            it = traffic_flow_.erase(it);
        } else {
            ++it;
        }
    }
}

void TrafficFlowMapping::ConstructTopology(const std::pair<uint32_t, uint64_t>& selected_traffic_flow_id,
    const ObjectTrajectory& trajectory, BevLaneInfo& fused_lane) {
    if (!trajectory.split_bev_region_id_range.count(selected_traffic_flow_id.second)) {
        return;
    }
    auto segmented_lane_1st = std::find_if(bev_map_ptr_->lane_infos.begin(), bev_map_ptr_->lane_infos.end(),
                                [find_id = selected_traffic_flow_id.second] (const BevLaneInfo &lane) { return lane.id == find_id; });

    if (segmented_lane_1st == bev_map_ptr_->lane_infos.end()) {
        return;
    }

    size_t segment_id = trajectory.split_bev_region_id_range.at(selected_traffic_flow_id.second).first;
    if (segment_id >= segmented_lane_1st->line_points.size()) {
        return;
    }

    auto segmented_lane_2nd = *segmented_lane_1st;

    segmented_lane_2nd.line_points.assign(segmented_lane_1st->line_points.begin() + segment_id, segmented_lane_1st->line_points.end());
    segmented_lane_2nd.number_of_points = segmented_lane_2nd.line_points.size();
    segmented_lane_1st->line_points.erase(segmented_lane_1st->line_points.begin() + segment_id, segmented_lane_1st->line_points.end());
    segmented_lane_1st->number_of_points = segmented_lane_1st->line_points.size();

    std::optional<uint64_t> lane_id_2nd = SearchUniqueID(bev_map_ptr_->lane_infos);

    if (!lane_id_2nd.has_value()) {
        return;
    }

    segmented_lane_2nd.id = lane_id_2nd.value();
    segmented_lane_2nd.previous_lane_ids.push_back(segmented_lane_1st->id);

    segmented_lane_1st->next_lane_ids.push_back(segmented_lane_2nd.id);
    segmented_lane_1st->next_lane_ids.push_back(fused_lane.id);

    fused_lane.previous_lane_ids.push_back(segmented_lane_1st->id);

    bev_map_ptr_->lane_infos.emplace_back(fused_lane);
    bev_map_ptr_->lane_infos.emplace_back(segmented_lane_2nd);
}

void TrafficFlowMapping::AssociateBevLaneInSplitRegion(ObjectTrajectory& trajectory, const BevLaneInfo& bev_lane) {
    if (!IsOverlapping(trajectory, bev_lane)) {
        return;
    }

    size_t starting_point_id = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < trajectory.ego_points.size(); i++) {
        if (trajectory.ego_points[i].x() < bev_lane.line_points.front().x - kXDirectionAssociationThreshold) {
            continue;
        }
        starting_point_id = i;
        break;
    }

    if (starting_point_id == std::numeric_limits<size_t>::max()) {
        return;
    }

    size_t split_id_in_trajectory = 0;
    std::pair<size_t, size_t> associated_bev_region_id_range = std::make_pair(0, 0);
    std::pair<size_t, size_t> split_bev_region_id_range = std::make_pair(0, 0);

    bool is_prev_associated = false;
    bool is_prev_split_region = false;
    for (size_t i = starting_point_id; i < trajectory.ego_points.size(); i++) {
        bool is_associated = false;
        bool is_split_region = false;
        Eigen::Vector2d trajectory_point(trajectory.ego_points[i].x(), trajectory.ego_points[i].y());
        for (size_t j = 0; j < bev_lane.line_points.size() - 1; j++) {
            if (trajectory_point.x() > bev_lane.line_points[j].x && trajectory_point.x() <= bev_lane.line_points[j + 1].x) {
                Eigen::Vector2d pt1(bev_lane.line_points[j].x, bev_lane.line_points[j].y);
                Eigen::Vector2d pt1_to_pt2(bev_lane.line_points[j + 1].x - pt1.x(),
                                           bev_lane.line_points[j + 1].y - pt1.y());
                Eigen::Vector2d pt1_to_trajectory_point = trajectory_point - pt1;

                double distance_to_lane = std::abs((pt1_to_trajectory_point.x() * pt1_to_pt2.y() - pt1_to_trajectory_point.y() * pt1_to_pt2.x())) / pt1_to_pt2.norm();
                if (distance_to_lane <= kYDirectionAssociationThreshold) {
                    is_associated = true;
                } else {
                    is_associated = false;
                }
                if (distance_to_lane > kYDirectionUnassociationThreshold) {
                    is_split_region = true;
                } else {
                    is_split_region = false;
                }
                // find association region
                if (is_associated && !is_prev_associated) {
                    associated_bev_region_id_range.first = j;
                }
                if (is_associated && is_prev_associated) {
                    associated_bev_region_id_range.second = j;
                }
                is_prev_associated = is_associated;
                // find split region
                if (is_split_region && !is_prev_split_region) {
                    split_bev_region_id_range.first = j;
                    split_id_in_trajectory = i;
                }
                if (is_split_region && is_prev_split_region) {
                    split_bev_region_id_range.second = j;
                }
                is_prev_split_region = is_split_region;
                break;
            }
        }

        bool is_associated_split_lane = IsAssociatedSplitLane(associated_bev_region_id_range, split_bev_region_id_range);

        if (is_associated_split_lane) {
            trajectory.associated_bev_lane_index.push_back(bev_lane.id);
            trajectory.associated_bev_region_id_range[bev_lane.id] = associated_bev_region_id_range;
            trajectory.split_bev_region_id_range[bev_lane.id] = split_bev_region_id_range;
            trajectory.split_id_in_trajectory[bev_lane.id] = split_id_in_trajectory;
            return;
        }
    }
}

bool TrafficFlowMapping::IsAssociatedSplitLane(const std::pair<size_t, size_t>& associated_region_id_range,
        const std::pair<size_t, size_t>& split_region_id_range) {
    if (associated_region_id_range.second == 0) {
        return false;
    }
    if (associated_region_id_range.second <= associated_region_id_range.first ||
        split_region_id_range.second <= split_region_id_range.first) {
        return false;
    }
    if (associated_region_id_range.second >= split_region_id_range.first) {
        return false;
    }
    if (associated_region_id_range.second - associated_region_id_range.first >= kMinAssociatedPointsNumber &&
        split_region_id_range.second - split_region_id_range.first >= kMinSplitRegionPointsNumber) {
        return true;
    }
    return false;
}

void TrafficFlowMapping::SetValidTrafficFlow() {
    for (auto& traffic_flow : traffic_flow_) {
        if (traffic_flow.second.points.size() > kMinValidPointsNumber) {
            traffic_flow.second.is_valid = true;
        } else {
            traffic_flow.second.is_valid = false;
        }
    }
}

template <typename T>
std::optional<uint64_t> TrafficFlowMapping::SearchUniqueID(T& markers) {
    std::set<uint64_t> lane_index;
    for (const auto& marker : markers) {
        lane_index.insert(marker.id);
    }

    uint64_t init_flow_lane_id = 0;
    uint64_t find_count = 0;
    bool is_found_unique_id = false;
    while (find_count < kMaxBevTrackID) {
        find_count++;
        init_flow_lane_id++;
        if (!lane_index.count(init_flow_lane_id)) {
            is_found_unique_id = true;
            break;
        }
    }

    if (is_found_unique_id) {
        return std::optional<uint64_t>{init_flow_lane_id};
    } else {
        return std::nullopt;
    }
}

bool TrafficFlowMapping::ShouldErase(const ObjectTrajectory& object_trajectory) {
    Eigen::Vector3d ego_position = T_local_ego_.inverse() * object_trajectory.points.back();
    if (ego_position.x() < kXAxisRange) {
        return true;
    }
    if (object_trajectory.consecutive_invisible_count > kMaxConsecutiveInversible) {
        return true;
    }
    auto current_time = apollo::cyber::Time::Now().ToSecond();
    if ((current_time - object_trajectory.timestamp) > kTrackingLossTimeThreshold) {
        return true;
    }
    return false;
}

bool TrafficFlowMapping::PreProcessor() {
    FusObjInfoPtr fusion_obj_ptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(fusion_obj_ptr);

    if (!fusion_obj_ptr) {
        return false;
    }

    LocalizationPtr measurement_pose = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(
        fusion_obj_ptr->header.timestamp, 0.05, measurement_pose);

    if (!measurement_pose) {
        return false;
    }

    if (fusion_object_ptr_ && fusion_object_ptr_->header.timestamp == fusion_obj_ptr->header.timestamp) {
        return false;
    }

    fusion_object_ptr_ = std::make_shared<FusObjInfo>(*fusion_obj_ptr);

    T_local_ego_.translation() = Eigen::Vector3d(measurement_pose->posne_dr.at(0),
                                                 measurement_pose->posne_dr.at(1),
                                                 0);

    T_local_ego_.linear() = Eigen::AngleAxisd(measurement_pose->attitude_dr * M_PI / 180.0,
                                              Eigen::Vector3d::UnitZ()).toRotationMatrix();

    T_ego_local_ = T_local_ego_.inverse();

    for (size_t i = 0; i < fusion_object_ptr_->fused_obstacles.size(); i++) {
        fusion_object_ptr_->fused_obstacles[i].position = T_local_ego_ *
                fusion_object_ptr_->fused_obstacles[i].position;
    }

    return true;
}

}  // namespace fusion
}  // namespace cem