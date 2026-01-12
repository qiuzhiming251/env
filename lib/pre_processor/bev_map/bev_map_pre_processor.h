#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>

#include "base/sensor_data_manager.h"
#include "common/CommonDataType.h"
#include "common/utility.h"
#include "localmap_construction/lane_topology_processor.h"
#include "lib/base/landmark_buffer.h"

namespace cem {
namespace fusion {

struct AABB {
  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();

  AABB() = default;
  AABB(const cem::message::common::Point2DF &p) {
    min_x = p.x;
    max_x = p.x;
    min_y = p.y;
    max_y = p.y;
  };
  void Reset() {
    min_x = std::numeric_limits<float>::max();
    max_x = std::numeric_limits<float>::lowest();
    min_y = std::numeric_limits<float>::max();
    max_y = std::numeric_limits<float>::lowest();
  };

  void ImportPoint(const cem::message::common::Point2DF &p) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  };

  // 计算与另一个 AABB 的最小距离
  float DistanceTo(const AABB &other) const {
    float dx = std::max(0.0F, std::max(other.min_x - max_x, min_x - other.max_x));
    float dy = std::max(0.0F, std::max(other.min_y - max_y, min_y - other.max_y));
    return dx * dx + dy * dy;
  };
};

struct RoadEdgeInfo {
  cem::message::sensor::BevLaneMarker edge;
  AABB                                total_bbox;
  std::vector<AABB>                   segs_bbox;
};

typedef struct line_id_vec_s {
  uint32_t        line_id = 0;
  Eigen::Vector2f pts;
  Eigen::Vector2f pts2;
  bool            is_valid = true;
  bool            is_front = false;
} line_id_vec;

typedef struct line_link_pair_info_s {
  line_id_vec first_left_laneMarker;
  line_id_vec first_lane;
  line_id_vec first_right_laneMarker;
  line_id_vec second_left_laneMarker;
  line_id_vec second_lane;
  line_id_vec second_right_laneMarker;
} line_link_pair_info;

typedef struct lane_id_flag_s {
  uint32_t lane_id             = 0;
  uint32_t left_lanemarker_id  = 0;
  uint32_t right_lanemarker_id = 0;
  bool     is_visited          = false;
} lane_id_flag;

typedef struct one_interpoltion_group_s {
  uint32_t                  group_id        = 0;
  uint32_t                  current_lane_id = 0;
  std::vector<lane_id_flag> next_lane_ids;
  std::vector<lane_id_flag> prev_lane_ids;
} one_interpoltion_group;

class BevMapPreProcessor {
 private:
  struct RightTurnSdTrajectory {
    bool                         valid     = false;
    double                       timestamp = 0.0;
    std::vector<Eigen::Vector2f> trajectory;
  };

  struct FreeSpaceRay {
    uint32_t                     angle;
    Eigen::Vector2d              direction;
    Eigen::Vector2d              end_point;
    std::vector<Eigen::Vector2d> points;
    uint32_t                     from_seq;
    uint32_t                     from_group;
  };

 public:
  BevMapPreProcessor(std::shared_ptr<LandmarkBuffer> buffer);
  ~BevMapPreProcessor();
  void             Process(BevMapInfoPtr bev_map = nullptr);
  void             Init();
  DetectBevMap     GetGlobalBevMapPtr();
  void             SetRightTurnTrajectory(double timestamp, const std::vector<Eigen::Vector2f> &trajectory);
  const std::vector<cem::message::sensor::BevLaneMarker> &GetLidarClusterRoadEdges() const { return lidar_cluster_roadedges_; };
  void             SetTraverseCrossWalkLaneInfo(std::vector<traverseCrossWalkLane>& crosswalk_lane_list);

 private:
  ////接收传感器消息
  void InitConfigs();
  void InitBevMapSubscriber();
  void InitLidarRoadEdgeSubscriber();

  bool GenerateBezierPoints(std::vector<SamplingPoint> &control_points, std::vector<Point2DF> &res_points);
  bool GetBezierVector(const std::vector<Point2DF> &line_points, const bool &is_front, line_id_vec &liv);

  // 获取SDMap中应急车道信息
  navigation::EmergencyLaneInfo GetEmergencyLaneInfo();

  // 根据bev_track标签、路沿和SD地图信息提取应急车道
  void GetEmergencyLaneIds(const navigation::EmergencyLaneInfo &sd_emergency_lane_info, 
                           std::set<uint64_t>& current_bev_emergency_laneid,
                           std::vector<int>& target_emergency_left_idx, 
                           std::vector<int>& target_emergency_right_idx);

  void ProcessYshapeSplit(std::vector<int>& target_lane_inds);
  void ProcessYshapeMerge(std::vector<int>& target_lane_inds);
  void ProcessEndStartTopo(std::vector<int>& target_lane_inds);
  void ProcessStartEndTopo(std::vector<int>& target_lane_inds);
  void CheckSplitTopo(const std::set<uint64_t>& emergency_lane_set, const std::set<uint64_t>& habor_lane_set);
  void CheckMergeTopo(const std::set<uint64_t>& emergency_lane_set, const std::set<uint64_t>& habor_lane_set);
  void FindNeighbourLanes();
  void GetFilteredShortLaneIds(std::vector<int>& left_most_idx, 
                               std::vector<int>& right_most_idx);
  void MergeSplitInterpolation();
  void CompensateLanePoints();

  // Add bev map to navigation container
  void AddBevToNavigationContainer();

  void BevTransToWorld(const std::optional<Eigen::Isometry3d> &T_local_ego);

  static Eigen::Isometry3d CalcRotateTranslateMatrix(LocalizationConstPtr last_loc, LocalizationConstPtr cur_loc);

  inline std::optional<Eigen::Isometry3d> FindTransform(const double &timestamp);
  inline Eigen::Isometry3d                FindRealTransform(const double &timestamp);

  void TransformLidarRoadEdgeToBevTime();
  void FuseLidarRoadEdgeIntoBevMap();
  void AddDataIntoLidarRoadEdgeGroup(uint32_t sequence_number);
  bool IsCurveCloseEnough(const RoadEdgeInfo &edge1_info, const RoadEdgeInfo &edge2_info);
  void FilterBevRoadEdge(const cem::message::sensor::BevLaneMarker &freespace);

  // Calculate the curve length when the number of points is greater than 1.
  void CalculateLaneLength(BevLaneInfo &lane);

  void ConstructRightTurnTrajectory();

  bool FittingCenterLine();

  // assign attributes to lane marker points
  void AssignAttribute2LaneMarkerPts();

  void FillNearLaneId();

  bool CalculateAverageDistanceBetweenTwoLines(std::vector<Eigen::Vector2f>& line_1, std::vector<Eigen::Vector2f>& line_2, std::pair<float, float> &distance);

  BevMapInfoPtr                                                     input_bev_map_ptr_{nullptr};
  DetectBevMap                                                      detect_bev_map_;
  LaneBuilderParams                                                 lane_builder_params_;
  LidarRoadEdgeInfoPtr                                              lidar_road_edge_ptr_{nullptr};

  std::map<uint32_t, std::map<uint32_t, std::vector<RoadEdgeInfo>>> hist_lidar_roadedge_group_;

  static const double DIST_THRESHOLD;

  std::optional<Eigen::Isometry3d>                 T_local_ego_;
  std::unordered_map<uint32_t, FreeSpaceRay>       lidar_freespace_;
  RightTurnSdTrajectory                            sd_right_turn_trajectory_;
  cem::message::sensor::BevLaneMarker              freespace_polyline_;
  std::vector<cem::message::sensor::BevLaneMarker> lidar_cluster_roadedges_;

  std::shared_ptr<LandmarkBuffer> landmark_buffer_;
  std::vector<traverseCrossWalkLane> crosswalk_lane_list_;
};
}  // namespace fusion
}  // namespace cem

#endif

