/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-05-14
 * @copyright Copyright (c) 2025, BYD
 */

#ifndef MODULES_PERCEPTION_ENV_SRC_LIB_LOCALMAP_CONSTRUCTION_CROSSROAD_CONSTRUCTION_LANELINE_CLUSTER_H_
#define MODULES_PERCEPTION_ENV_SRC_LIB_LOCALMAP_CONSTRUCTION_CROSSROAD_CONSTRUCTION_LANELINE_CLUSTER_H_
#include "base/params_manager/params_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "cross_common.h"
#include <Eigen/Core>
#include "cross_data_manager.h"
#include <common/utils/lane_geometry.h>
#include "lib/common/utility.h"
#include <cstdint>
#include <unordered_map>
#include <variant>
#include "lib/common/log_custom.h"
#ifdef _MSC_VER
#define FORCE_INLINE __forceinline
#elif defined(__GNUC__)
#define FORCE_INLINE inline __attribute__((__always_inline__))
#elif defined(__CLANG__)
#if __has_attribute(__always_inline__)
#define FORCE_INLINE inline __attribute__((__always_inline__))
#else
#define FORCE_INLINE inline
#endif
#else
#define FORCE_INLINE inline
#endif
using namespace byd::common::math;
using namespace cem::message::sensor;

namespace cem {
namespace fusion{
class CrossDataManager;

enum {
  pointType_UNDO,
  pointType_NOISE,   // 噪声
  pointType_BORDER,  // 边界点
  pointType_CORE     // 核心点
};

enum {
  keyVal_THETA = 0,       // 向量夹角，正反向会区分，范围0-pi
  keyVal_THETA_HALF = 5,  // 直线夹角，正反向不区分，范围0-pi/2
  keyVal_X = 1,
  keyVal_Y = 2,
  keyVal_CENTER = 3,
  keyVal_ALL = 4,
  keyVal_LATERAL = 6,  // 计算两条线之间的横向距离
  keyVal_CLUSTER = 7,  // 计算两个cluster之间的距离，依据line之间的距离
  keyVal_DISCRETECOMPS = 8,  // 计算两个离散要素(只支持人行横道和停止线)的距离
  keyVal_MIN_X_Y = 9,
};
enum {
  LANE_GROUP_OTHER = 0,       // 向量夹角，正反向会区分，范围0-pi
  LANE_GROUP_SD = 1,  // 直线夹角，正反向不区分，范围0-pi/2
  LANE_GROUP_EGO = 2,
  LANE_GROUP_EDAGE = 3,
  LANE_GROUP_LANE = 4,
};
class Particle {
 public:
  float x = 0.0f;
  float y = 0.0f;
  float angle = 0.0f;
  float end_angle = 0.0f;
  std::vector<float> xn;           // 多维数据
  int cluster = 0;                 // 第几个簇
  int pointType = pointType_UNDO;  // 1 noise 2 border 3 core
  int groupType = LANE_GROUP_OTHER;  // 1 noise 2 border 3 core
  int pts = 0;                     // points in MinPts
  int corePointID = -1;            // 核心点的标号
  std::vector<int> corepts;        //临近core计数包含自己
  int visited = 0;
  uint64_t lane_id;
  int obj_idx_ = 0;  // 关联物体的ID。在junction判断中，为交点的id
  int cluster_idx = 0;  // 用来聚类logical cluster时记录idx
  // std::shared_ptr<BevLaneInfo> sourcePtr = nullptr;  // 指向原始的车道线
  std::variant<std::shared_ptr<BevLaneInfo>, std::shared_ptr<BevLaneMarker> >sourcePtr;
  BevLaneDirection direction = BevLaneDirection::DIRECTION_UNKNOWN;
  Particle() {}
};
// 定义一个结构体来RoadBoundary 
struct RoadBoundary {
 bool has_left_edge{false};
 bool has_left_yellow_lanemarker{false};
 bool has_right_edge{false};
 bool has_last_right_edge{false};
 int left_edge_id{-1};
 double min_left_dis2SD_box_cross{1000.f};
 double min_right_dis2SD_box_cross{1000.f};
 int right_edge_id{-1};
 int last_right_edge_id{-1};

 void Reset() {
   has_left_edge = false;
   has_right_edge = false;
   has_last_right_edge = false;

   left_edge_id = -1;
   right_edge_id = -1;
   last_right_edge_id = -1;
   min_left_dis2SD_box_cross = 1000.f;
   min_right_dis2SD_box_cross = 1000.f;


 };

};
class LaneLineCluster {
 public:
  LaneLineCluster() {}
  ~LaneLineCluster() {}
  void Init();
  byd::common::math::Vec2d TransformPointVec(const byd::common::math::Vec2d &point,
    const Eigen::Isometry3d &rotate_translate_matrix) {
    byd::common::math::Vec2d point_out;
    Eigen::Vector3d point_before_dr(point.x(), point.y(), 0);
    Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
    point_out.set_x(point_after_dr[0]);
    point_out.set_y(point_after_dr[1]);
    return point_out;
}
  bool PrecessProcess(std::shared_ptr<CrossDataManager> data_manager_ptr);
  void SetLaneLines2Particles();
  // auto GetParticles(){return particles_;};
  float Distance(const Particle& p1, const Particle& p2){return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));};
  float CalcuAngleVector2Vector(const Particle &a, const Particle &b)
  {
    float angle = fabs(a.angle - b.angle);
    if (angle > 180.0f) {
        angle = 360.0f - angle;
    }
    return angle;
  };
FORCE_INLINE float CalcuMinXYDistance(const Particle &a, const Particle &b) 
{
  const float threshold = 20.f;
  Eigen::Vector2f a_first_pt;
  Eigen::Vector2f a_end_pt;
  Eigen::Vector2f b_first_pt;
  Eigen::Vector2f b_end_pt;
  if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(a.sourcePtr)){
    auto &lanePtr = std::get<std::shared_ptr<BevLaneInfo>>(a.sourcePtr);
    if(!lanePtr || lanePtr->line_points.size() < 1) return FLT_MAX;
    a_first_pt.x() = lanePtr->line_points.front().x;
    a_first_pt.y() = lanePtr->line_points.front().y;
    a_end_pt.x() = lanePtr->line_points.back().x;
    a_end_pt.y() = lanePtr->line_points.back().y;
  }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(a.sourcePtr)){
    auto &edagePtr = std::get<std::shared_ptr<BevLaneMarker>>(a.sourcePtr);
    if(!edagePtr || edagePtr->line_points.size() < 1) return FLT_MAX;
    a_first_pt.x() = edagePtr->line_points.front().x;
    a_first_pt.y() = edagePtr->line_points.front().y;
    a_end_pt.x() = edagePtr->line_points.back().x;
    a_end_pt.y() = edagePtr->line_points.back().y;
  }

  auto ab_frist_x = fabs(a.x - b.x);
  auto ab_frist_y = fabs(a.y - b.y);
  return (ab_frist_x > threshold || ab_frist_y > threshold ) ? FLT_MAX :ab_frist_x;
 };
  void DoDbScan(std::vector<Particle> &data, int keyValType,
                const float &Eps,const int &MinPts);
  void GetCluster(std::vector<Particle> &data,std::vector<std::vector<Particle>> &clusters);//转换td::vector<Particle> 到std::vector<std::vector<Particle>>
  void DoCluster(std::vector<Particle> &data, int keyValType,//输出改写data的cluster
                const float &Eps,const int &MinPts,
                std::vector<std::vector<Particle>> &clusters);//输出二维度 第二维度内部在执行cluster
  void MultiStepCluster(std::vector<std::vector<Particle>> &all_clusters);
  void PostProcess(std::vector<std::vector<Particle>> &all_clusters);
  void SortTargetLanes(std::vector<uint64_t> &laneIds);
  void SortTargetLaneIds(std::vector<uint64_t> &laneIds);
  void SortEgoLaneIds(std::vector<uint64_t> &laneIds);
  void SortTopLanes(std::vector<TopLane> &top_lanes);
  std::vector<uint64_t> GetTargetClusteredLaneIds(){return target_clustered_lane_ids_;};
  std::vector<uint64_t> GetTargetForwardLaneIds(){return target_forward_lane_ids_;};
  std::vector<TopLane> GetTopLanes(){return top_lanes_;};
  std::vector<TopLane> GetEgoTopLanes(){return ego_lanes_;};
  std::vector<uint64_t> GetEgoClusteredLaneIds(){return ego_clustered_lane_ids_;};
  std::vector<uint64_t> GetEgoClusteredEdgeIds(){return ego_clustered_edge_ids_;};
  std::vector<TopLane> GetEgoTopLanesFromMap();
  std::vector<TopLane>  GetAverageTargetTopLanesFromMap();
  void AddEgoLanesToMap(const std::vector<TopLane> &egoLanes, std::unordered_map<uint64_t,TopLane> &map);//聚类结果添加到 ego_lanesMap_
  void AddTargetLanesToMap( std::vector<TopLane> &targetLanes, std::unordered_map<uint64_t,std::vector<TopLane>>& map);//聚类结果添加到 target_lanesMap_go_lanesMap_
  int  GetSDOpeningLaneNum(){return SDOpeningLaneNum_;};
  bool GetEgoLeftRoadBoundary(const std::vector<uint64_t>& ids,RoadBoundary& road_boundary);
  bool GetTargetLeftRoadBoundary(const std::vector<uint64_t>& ids,RoadBoundary& road_boundary);
  bool GetEgoRightRoadBoundary(const std::vector<uint64_t>& ids,RoadBoundary& road_boundary);
  bool GetTargetRightRoadBoundary(const std::vector<uint64_t>& ids,RoadBoundary& road_boundary);
  bool GetTargetLastRightRoadBoundary(const std::vector<uint64_t>& ids,RoadBoundary& road_boundary);
  void UpdateTargetRoadBoundary();
  void UpdateTargetYellowLMBoundary();
  void UpdateTargetLaneWidth();
  void UpdateEgoRoadBoundary();
  void RemoveTargetLaneWithBoundary();
  void RemoveEgoLaneWithBoundary();
  void GenernateAnchorsWithFirstLeftBoundaryAndSdLaneNum(const RoadBoundary& road_boundary,uint64_t lane_num,std::vector<TopLane>& anchors);
  void GenernateAnchorsWithFirstRightBoundaryAndSdLaneNum(const RoadBoundary& road_boundary,uint64_t lane_num,std::vector<TopLane>& anchors);
  bool JudgePointOnRightofRoadBounday(const RoadBoundary& road_boundary,const Vector2f& point);
  void UpdateSDParticleWithCenter(std::vector<std::vector<Particle>> &all_clusters);
  void GetEgoLanesFromIds(const std::vector<uint64_t>& ids,std::vector<TopLane>& top_lanes);
  void TurnMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes ,std::vector<TopLane>& anchors);
  void StraightMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes ,std::vector<TopLane>& anchors);
  void HMMergeTopLanesWithAnchors(std::vector<TopLane>& top_lanes ,std::vector<TopLane>& anchors);
  float CalculateVerticalDistance(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, const Eigen::Vector2f& query_point);
  void RemvoveEgoLanesOutDistance(); 
  void RemvoveEgoIdsConflictWithNavAvtion(); 
  void RemvoveEgoLanesOutGuidance();
  void GetTargetLaneNumFromLaneGroup(uint64_t &lane_num);
  bool IsTargetLanesDirectionUnreliable(const std::vector<uint64_t> &sorted_lane_ids); 
  bool IsRoadWidthVaild(const RoadBoundary& road_boundary);
  bool IsRightEdgeOutOfRoad(const RoadBoundary& road_boundary);
  void GetTopLanesFromObs( std::vector<uint64_t> &ids,std::vector<TopLane> & toplanes);
  std::vector<Particle> particles_;
  float lane_width_ = 3.25f;  // 默认车道宽度
  std::vector<uint64_t> target_clustered_lane_ids_;
  std::vector<uint64_t> target_forward_lane_ids_;
  RoadBoundary ego_road_boundary_;
  RoadBoundary target_road_boundary_;
  RoadBoundary target_yellowLM_boundary_;
  std::vector<TopLane> top_lanes_;
  std::vector<TopLane> ego_lanes_;//对外输出
  std::unordered_map<uint64_t,TopLane> ego_lanesMap_;
  std::vector<TopLane> anchors_;
  std::vector<uint64_t> ego_clustered_lane_ids_;
  std::vector<uint64_t> ego_clustered_edge_ids_;
  std::vector<BevLaneInfo> laneLines_;
  std::vector<BevLaneMarker> edages_;
  const double M_PI_ = 3.14159265358979323846;
  std::vector<byd::common::math::Vec2d> sd_opening_pts_;
  byd::common::math::Vec2d sd_box_corss_pt_{1000.f,1000.f};
  byd::common::math::Vec2d box_center_pt_{1000.f,1000.f};
  std::shared_ptr<CrossDataManager> data_manager_ptr_{nullptr};
  int SDOpeningLaneNum_{-1};
  cem::message::env_model::SDDirectionType section_direction_{ cem::message::env_model::SDDirectionType::UNKNOWN };
  double targetAngel_ = std::numeric_limits<double>::max();
  std::list<std::pair<double , std::vector<uint64_t>>> laneGuidanceMap_;
  // TurnType turnType_ = TurnType::OTHER_UNKNOWN;
  BevAction  currentAction_ = BevAction::UNKNOWN;
  BevMapInfo processed_bev_map_info_; 
  std::unordered_map<uint64_t,std::vector<TopLane>> his_top_lanes_;
  
  
  int target_index_{1000};
  int ego_index_{1000};
  bool target_group_has_reverse_{false};
  Vec2d road_center_{0.0f, 0.0f};  // 用于SDSectionPoints的中心点
  bool  road_center_valid_{false}; // 是否有效
  double road_angel_ = std::numeric_limits<double>::max();
  bool road_angel_valid_ = false;

  uint64_t target_lane_num_{0};
  std::vector<std::vector<Eigen::Vector2d>> yellow_lanemarkers_pts_{};
};
}
}

#endif