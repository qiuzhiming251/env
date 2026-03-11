#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include "base/sensor_data_manager.h"
#include "common/utility.h"
#include "common/CommonDataType.h"
#include <base/params_manager/params_manager.h>
#include <base/params_manager/params_defination/internal_params.h>
#include "localmap_construction/lane_topology_processor.h"
#include "lib/localmap_construction/lane_mapping.h"

namespace cem{
namespace fusion{

// struct AABB {
//   float min_x = std::numeric_limits<float>::max();
//   float max_x = std::numeric_limits<float>::lowest();
//   float min_y = std::numeric_limits<float>::max();
//   float max_y = std::numeric_limits<float>::lowest();

//   AABB() = default;
//   AABB(const cem::message::common::Point2DF& p) {
//     min_x = p.x;
//     max_x = p.x;
//     min_y = p.y;
//     max_y = p.y;
//   };
//   void Reset() {
//     min_x = std::numeric_limits<float>::max();
//     max_x = std::numeric_limits<float>::lowest();
//     min_y = std::numeric_limits<float>::max();
//     max_y = std::numeric_limits<float>::lowest();
//   };

//   void ImportPoint(const cem::message::common::Point2DF& p) {
//     min_x = std::min(min_x, p.x);
//     max_x = std::max(max_x, p.x);
//     min_y = std::min(min_y, p.y);
//     max_y = std::max(max_y, p.y);
//   };

//   // 计算与另一个 AABB 的最小距离
//   float DistanceTo(const AABB& other) const {
//     float dx =
//         std::max(0.0F, std::max(other.min_x - max_x, min_x - other.max_x));
//     float dy =
//         std::max(0.0F, std::max(other.min_y - max_y, min_y - other.max_y));
//     return dx * dx + dy * dy;
//   };
// };

// struct RoadEdgeInfo {
//   cem::message::sensor::BevLaneMarker edge;
//   AABB total_bbox;
//   std::vector<AABB> segs_bbox;
// };

// typedef struct line_id_vec_s
// {
//     uint32_t line_id = 0;
//     Eigen::Vector2f pts;
//     Eigen::Vector2f pts2;
//     bool is_valid = true;
//     bool is_front = false;
// }line_id_vec;

// typedef struct line_link_pair_info_s
// {
//     line_id_vec first_left_laneMarker;
//     line_id_vec first_lane;
//     line_id_vec first_right_laneMarker;
//     line_id_vec second_left_laneMarker;
//     line_id_vec second_lane;
//     line_id_vec second_right_laneMarker;
// }line_link_pair_info;

// typedef struct lane_id_flag_s
// {
//     uint32_t lane_id = 0;
//     uint32_t left_lanemarker_id = 0;
//     uint32_t right_lanemarker_id = 0;
//     bool is_visited = false;
// }lane_id_flag;

// typedef struct one_interpoltion_group_s
// {
//     uint32_t group_id = 0;
//     uint32_t current_lane_id = 0;
//     std::vector<lane_id_flag> next_lane_ids;
//     std::vector<lane_id_flag> prev_lane_ids;
// }one_interpoltion_group;

class DataManager
{
 private:
  // struct RightTurnSdTrajectory {
  //   bool valid = false;
  //   double timestamp = 0.0;
  //   std::vector<Eigen::Vector2f> trajectory;
  // };

  // struct FreeSpaceRay {
  //   uint32_t angle;
  //   Eigen::Vector2d direction;
  //   Eigen::Vector2d end_point;
  //   std::vector<Eigen::Vector2d> points;
  //   uint32_t from_seq;
  //   uint32_t from_group;
  // };

public:
    DataManager();
    ~DataManager();

    // void Process();
    // void Init();
    // DetectBevMap GetGlobalBevMapPtr();
    // DetectRoutingMap GetGlobalRoutingMapPtr();
    // void SetRightTurnTrajectory(double timestamp,
    //                             const std::vector<Eigen::Vector2f>& trajectory);
    // void GetCrossRoadFlag(bool flag) { cross_road_status_ = flag; };
    // void GetCrossRoadDistance(double distance) { cross_road_distance_ = distance; };
    // const std::vector<cem::message::sensor::BevLaneMarker>&
    // GetLidarClusterRoadEdges() const {
    //   return lidar_cluster_roadedges_;
    // };

    static Eigen::Isometry3d FindTransformTemp(const double &timestamp);

    static Eigen::Isometry3d FindTransform(const double &timestamp);

    static BevMapInfoPtr BevMapWorld2Body(const BevMapInfoPtr &bev_map_input);

private:
    // ////接收传感器消息
    // void InitConfigs();

    // void InitOdomSubscriber();
    // void InitBevMapSubscriber();
    // void InitRoutingMapSubscriber();
    // void InitLidarRoadEdgeSubscriber();

    // ////BEV感知的预处理
    // void BevMapInfoProcess();

    // bool GenerateBezierPoints(
    //     std::vector<SamplingPoint> & control_points,
    //     std::vector<Point2DF> &res_points);
    // bool GetBezierVector(
    //     const std::vector<Point2DF> &line_points,
    //     const bool & is_front,
    //     line_id_vec &liv);
    // void ProcessYshapeSplit();
    // void MergeSplitInterpolation();

    // // Add bev map to navigation container
    // void AddBevToNavigationContainer();

    // ///LD地图的预处理
    // void LDMapInfoProcess();

    // void BevTransToWorld(const std::optional<Eigen::Isometry3d>& T_local_ego);
    // void RoutingMapTransToWorld();

    // static Eigen::Isometry3d CalcRotateTranslateMatrix(LocalizationConstPtr last_loc, LocalizationConstPtr cur_loc);

    // inline std::optional<Eigen::Isometry3d> FindTransform(const double& timestamp);
    // inline Eigen::Isometry3d FindRealTransform(const double& timestamp);

    // void TransformLidarRoadEdgeToBevTime();
    // void FuseLidarRoadEdgeIntoBevMap();
    // void AddDataIntoLidarRoadEdgeGroup(uint32_t sequence_number);
    // bool IsCurveCloseEnough(const RoadEdgeInfo& edge1_info,
    //                         const RoadEdgeInfo& edge2_info);
    // void FilterBevRoadEdge(
    //     const cem::message::sensor::BevLaneMarker& freespace);
    // // Calculate the curve length when the number of points is greater than 1.
    // void CalculateLaneLength(BevLaneInfo &lane);

    // void ConstructRightTurnTrajectory();

    // bool FittingCenterLine();

    // // assign attributes to lane marker points
    // void AssignAttribute2LaneMarkerPts();

    // RoutingMapPtr input_routing_map_ptr_ {nullptr};
    // RoutingMapPtr output_routing_map_ptr_ {nullptr};
    // BevMapInfoPtr input_bev_map_ptr_ {nullptr};
    // BevMapInfoPtr output_bev_map_ptr_ {nullptr};
    // DetectBevMap detect_bev_map_;
    // DetectRoutingMap detect_routing_map_;
    // LaneBuilderParams lane_builder_params_;
    // SDRouteInfoPtr sd_route_Info_ptr_ {nullptr};
    // LidarRoadEdgeInfoPtr lidar_road_edge_ptr_{nullptr};
    // ProcessLanesTopo lane_topology_processor_;
    // std::map<uint32_t, std::map<uint32_t, std::vector<RoadEdgeInfo>>>
    //     hist_lidar_roadedge_group_;

    // bool used_lane_track_ = true;
    // bool cross_road_status_ = false;
    // double cross_road_distance_ = -1.0;
    // static const double DIST_THRESHOLD;

    // std::optional<Eigen::Isometry3d> T_local_ego_;
    // std::unordered_map<uint32_t, FreeSpaceRay> lidar_freespace_;
    // RightTurnSdTrajectory sd_right_turn_trajectory_;
    // cem::message::sensor::BevLaneMarker freespace_polyline_;
    // std::vector<cem::message::sensor::BevLaneMarker> lidar_cluster_roadedges_;

    // std::shared_ptr<cem::fusion::LaneTracker> lane_tracker_ptr_;
};

}
}


#endif
