/**
 * @file cross_data_manager.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 2.1
 * @date 2025-04-09
 *
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 *
 */
#ifndef CROSS_DATA_MANAGER_H
#define CROSS_DATA_MANAGER_H
#include "base/params_manager/params_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include "cross_common.h"
#include "laneline_cluster.h"
#include "lane_line_refiner.h"
#include "edge_merger.h"
#include "occ_processor.h"
// #define PPM_UPDATE 1
#include "draw_point.h"

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>  // 核心FLANN功能
static constexpr double pi = 3.14159265358979323846;
namespace cem {
namespace fusion {
///
class LaneLineCluster;//前向声明
class LaneLineRefiner;//前向声明
class OccProcessor;//前向声明
class EdgeMerger;//前向声明

class CrossDataManager {
 public:
  CrossDataManager();
  struct SdSectionPoints {
    int size{0};
    std::vector<Vec2d> points;
    std::vector<Vec2d> top_points;
    std::vector<Vec2d> sd_end_points;
    std::vector<int> points_lane_num;
    std::vector<uint64_t> lane_groupid;
    std::vector< cem::message::env_model::SDDirectionType> section_direction{ cem::message::env_model::SDDirectionType::UNKNOWN };
    int current_lane_num{-1};
    uint64_t targetLaneGorupId_{0};
    cem::message::env_model::SDDirectionType current_sd_dirction_{cem::message::env_model::SDDirectionType::UNKNOWN};

    Vec2d start_point;
    Vec2d main_vector;
    Vec2d cross_point{DBL_MAX,DBL_MAX};
    Vec2d end_point;
    Vec2d top_vector;
    double top_cross2sd_left_dis{0.0};
    double top_cross2sd_right_dis{0.0};
    int end_point_index{0};
    int start_sd_point_index{0};
    int end_sd_point_index{0};
    bool has_junction{false};
    bool has_intersection_zone{false};
    bool is_valid{false};
    bool straight_angle_large{false};
    double top_angle() const {
      return (top_vector - end_point).Angle();
    };
    void Clear() {
      size = 0;
      points.clear();
      top_points.clear();
      points_lane_num.clear();
      section_direction.clear();
      sd_end_points.clear();
      lane_groupid.clear();
      start_point = Vec2d();
      cross_point = Vec2d(DBL_MAX,DBL_MAX);
      end_point = Vec2d();
      top_vector = Vec2d();
      top_cross2sd_left_dis = 0.0;
      top_cross2sd_right_dis = 0.0;
      end_point_index = 0;
      start_sd_point_index = 0;
      end_sd_point_index = 0;
      has_junction = false;
      has_intersection_zone = false;
      is_valid = false;
      straight_angle_large = false;
      current_lane_num = -1;
    };
  };

  void DataUpdate(const RoutingMapPtr &routing_map_raw, const RoutingMapPtr &routing_map_ptr, const BevMapInfoConstPtr &bev_map_raw,
                  const BevMapInfoPtr &bev_map, const LocalizationPtr &loc, const Eigen::Isometry3d &Twb,
                  const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr, bool on_highway);
  void Init();
  void Reset();

  const RoutingMapPtr& routing_map_raw() { return routing_map_raw_; }
  const RoutingMapPtr& routing_map() { return routing_map_; }
  const BevMapInfoConstPtr& bev_map_raw() { return bev_map_raw_; }
  BevMapInfoPtr& bev_map() { return bev_map_; }
  const LocalizationPtr& currect_loc() { return currect_loc_; }
  const LocalizationPtr& last_loc() { return last_loc_; }
  const BevLaneInfo* GetLaneById(uint64_t id) {
    return raw_lane_unmap_.find(id) == raw_lane_unmap_.end() ? nullptr : raw_lane_unmap_[id];
  };
  const BevLaneMarker* GetLaneMarkerById(uint64_t id) {
    return raw_lanemarker_unmap_.find(id) == raw_lanemarker_unmap_.end() ? nullptr : raw_lanemarker_unmap_[id];
  };
  // bool IsLaneNormal(uint64_t id, double length);
  bool has_ego_lane() { return has_ego_lane_; }
  bool is_egolane_normal() { return is_egolane_normal_; }
  bool is_egolane_low() { return is_egolane_low_; }
  bool is_egolane_death() { return is_egolane_death_; }
  bool is_egolane_disappear(){return is_egolane_disappear_;}
  bool has_bev_stop_lane() { return has_bev_stop_lane_; }
  bool has_bev_junction() { return has_bev_junction_; }
  bool has_bev_cross_walk() { return has_bev_cross_walk_; }
  double cross_road_dis(){return cross_road_dis_;}
  const Opening& GetOpening(){return opening_; }
  const Opening& SetOpening(const Opening& opening){return opening_ = opening; }
  const SdSectionPoints& GetSdSections() { return sd_section_points_; }
  const BevLaneInfo* EgoLane() { return &ego_lane_; };
  const Eigen::Isometry3d& Twb() { return Twb_; }
  const Eigen::Isometry3d& Tbw() { return Tbw_; }
  float GetLocDeltaHeading();
  MainLane& GetMainLane() { return main_lane_; };
  TopLane& GetTopLane() { return top_lane_; };
  std::vector<TopLane>& GetTopLanes() { return top_lanes_; };
  std::vector<TopLane>& GetEgoLanes() { return ego_lanes_; };
  BevAction turn_type() { return ego_turn_type_; }
  std::vector<uint64_t>& target_clustered_lane_ids() { return target_clustered_lane_ids_; }
  std::vector<uint64_t>& ego_clustered_lane_ids() { return ego_clustered_lane_ids_; }
  const std::vector<uint64_t>& GetGuidanceLance(){ return sd_guide_lane_ids_;}
  uint64_t GetNewLandId();
  void UpdateCrossState(CrossState state){cross_state_ = state;}
  CrossState GetCrossState(){return cross_state_;}

  VirtualLineSource GetVirtualLineSource() { return virtual_line_source_; }
  void SetVirtualLineSource(VirtualLineSource source) { virtual_line_source_ = source; }

  bool GetIsInCrossroad() { return is_in_cross_road_; }
  void SetIsInCrossroad(bool is_in_cross_road) { is_in_cross_road_ = is_in_cross_road; }

  uint64_t GetLDEgoLaneId() { return ld_ego_lane_id_; }
  void     SetLDEgoLaneId(uint64_t ld_ego_lane_id) { ld_ego_lane_id_ = ld_ego_lane_id; }

  double FindNearstLeftBoundaryDis(const Vec2d& point);
  double FindNearstRightBoundaryDis(const Vec2d& point);
  const EgoRoadLanes& GetEgoRoadLanes(){return ego_road_lanes_;};
  bool is_find_closetlane_bytopo_success(){return is_find_closetlane_bytopo_success_;}
  void SetIsConnectByTopo(bool input){is_find_closetlane_bytopo_success_ = input;}

  const auto &GetLeftCrossEdges() { return left_edges_; }
  const auto &GetRightCrossEdges() { return right_edges_; }
  const std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& GetOccEdges(){
    return edgesPtr_;
  }
  const OccSdEdgesInfo& GetOccSdEdgeInfo(){
    return occ_sd_edges_info_;
  }
  void DealBoundary(const std::vector<Vec2d> &points);
  bool HasVirtualLane(){return has_is_virtual_;};
  const cem::message::env_model::LaneInfo *FindRoutingMapRawLaneById(uint64_t id);
  const cem::message::env_model::LaneInfo *FindRoutingMapLaneById(uint64_t id);
  bool is_T_cross_road_flag(){return is_T_cross_road_flag_;}
  bool SdHasEepTrajEndLaneid(uint64_t input_id);
  double GetBevLaneStartHeading(const BevLaneInfo& lane, double length);
  double GetBevLaneEndHeading(const BevLaneInfo& lane, double length);
  bool first_junction_lane_is_virtual() { return first_junction_lane_is_virtual_; };

 private:
 void DealEgoLane();
 void DealTCrossRoad(const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr);
 void DealEgoRoad();
 void DealIsInCrossRoad();
 void UpdateSdSectionPoints();
 void FindCrossRoadFlag();
 bool FindStopLine(double& dis);
 bool FindCrossWalk(double& dis);
 bool FindJunction(double& dis);
 Vec2d TransformPointVec(const Vec2d &point,
  const Eigen::Isometry3d &rotate_translate_matrix);
  void DealCrossBoundary();
  // DealCrossOccBoundary函数
  void DealCrossOccBoundary();
  std::pair<double,double> FindTopCrossPointDis(const std::vector<Point2DF>& bev_cross_points,Vec2d A, Vec2d B);

  void TransformDr2Ego(SdSectionPoints *sd_section_ego, const SdSectionPoints &sd_section_dr);
  BevAction TurnType2Action(cem::message::env_model::TurnType turn_type);
  bool FindVirtualLane();
  bool GetSdEdgePoint(std::vector<Vec2d> &sd_points);
  bool GetSdOccEdges(std::vector<SdCrossOccEdgePoint> &sd_edge_points, double &max_main_left_x, const std::vector<Vec2d> &sd_points);
  bool GetOccEdges(std::vector<OccEdge>& occ_edges);
  bool DealOccEdges(std::vector<OccEdge>& occ_edges);
  bool GetOpening(std::vector<OccEdge>& occ_edges);
  bool CrossRoadVirtualLaneCheck();
  RoutingMapPtr routing_map_raw_{nullptr};
  RoutingMapPtr routing_map_{nullptr};
  BevMapInfoConstPtr bev_map_raw_{nullptr};
  BevMapInfoPtr bev_map_{nullptr};
  LocalizationPtr currect_loc_{nullptr};
  LocalizationPtr last_loc_{nullptr};
  public:
  std::shared_ptr<LaneLineCluster> lane_cluster_{nullptr};
  std::shared_ptr<LaneLineRefiner> lane_refiner_{nullptr};
  std::shared_ptr<OccProcessor> occ_processor_{nullptr};
  std::shared_ptr<EdgeMerger> edge_merger_{nullptr};
  // 自车到dr转换
  Eigen::Isometry3d Twb_;
  // dr到自车转换
  Eigen::Isometry3d Tbw_;
  std::unordered_map<uint64_t,const BevLaneInfo*> raw_lane_unmap_;
  std::unordered_map<uint64_t,const BevLaneInfo*> bev_lane_unmap_;
  std::unordered_map<uint64_t,const BevLaneMarker*> raw_lanemarker_unmap_;
  std::unordered_map<uint64_t,const cem::message::env_model::LaneInfo*> routing_lane_unmap_;

  SdSectionPoints sd_section_points_{};
  SdSectionPoints dr_sd_section_points_{};
  MainLane main_lane_;
  std::vector<TopLane> top_lanes_;
  std::vector<TopLane> ego_lanes_;
  TopLane top_lane_;
  Opening opening_;
  BevAction ego_turn_type_ = BevAction::STRAIGHT;
  cem::message::sensor::BevLaneInfo ego_lane_{};
  cem::message::sensor::BevLaneInfo dr_ego_lane_{};
  uint64_t ego_lane_id_{0};
  uint64_t ld_ego_lane_id_{0};
  CrossState cross_state_{CrossState::INIT};
  EgoRoadLanes ego_road_lanes_{};
  // 0:pre 1:obs 2:SD 3:LD 4. exp_trajectary
  VirtualLineSource virtual_line_source_{VirtualLineSource::PRE};
  double cross_road_dis_{1000.0};
  bool has_ego_lane_{false};
  bool is_egolane_normal_{false};
  bool is_egolane_low_{false};
  bool is_egolane_death_{false};
  bool is_in_cross_road_{false};
  bool has_bev_stop_lane_{false};
  bool has_bev_junction_{false};
  bool has_bev_cross_walk_{false};
  bool is_egolane_disappear_{false};
  bool has_bev_ego_navi_action_{false};
  bool using_junction_action_{false};
  bool is_find_closetlane_bytopo_success_ = false;
  bool is_boundary_left_{false};
  bool has_is_virtual_{false};
  bool is_T_cross_road_flag_ = false;
  bool first_junction_lane_is_virtual_ = false;
  Vec2d bev_cut_start_point_;
  Vec2d bev_cut_start_right_point_;
  cem::message::sensor::BevLaneInfo bev_ego_lane_t_;
  std::vector<uint64_t> target_clustered_lane_ids_{};
  std::vector<uint64_t> ego_clustered_lane_ids_{};
  std::vector<uint64_t> sd_guide_lane_ids_{};
  
  std::vector<CrossEdge> left_edges_{};
  std::vector<CrossEdge> right_edges_{};
  OccSdEdgesInfo occ_sd_edges_info_{};
  std::vector<SdCrossOccEdgePoint> sd_edge_points_;

  public:
  std::shared_ptr<CrossDataManager> cross_data_manager_ptr{nullptr};
  std::shared_ptr<ppm::PPMImage> ppmImage{nullptr};
  std::shared_ptr<cv::flann::Index> left_boundary_kdtree_{nullptr};
  std::shared_ptr<cv::flann::Index> right_boundary_kdtree_{nullptr};
  std::vector<std::pair<uint64_t, std::shared_ptr<cv::flann::Index>>> boundary_kdtrees_;
  std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>> edgesPtr_;
  const auto &GetOccProcessor() { return occ_processor_; }
  bool on_highway_{false};

};

}  // namespace fusion
}  // namespace cem
#endif  // CROSS_DATA_MANAGER_H