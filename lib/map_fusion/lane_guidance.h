#ifndef LANE_GUIDANCE_H_
#define LANE_GUIDANCE_H_

#include <unordered_map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include <message/env_model/navigation/navigation.h>
#include "base/sensor_data_manager.h"
#include "common/fitter/bezier_points.h"
#include "lib/map_fusion/graph.h"
#include "lib/map_fusion/road_divider.h"
#include "lib/message/env_model/stop_line/stop_line.h"
#include "lib/sd_navigation/SDMapTopoExtract.h"
#include "lib/common/CommonDataType.h"

namespace cem {
namespace fusion {
using cem::fusion::navigation::LaneDetail;
using cem::fusion::navigation::MergeDetail;
using cem::fusion::navigation::SDMapTopologyExtractor;
class LaneGuidance {
 public:
  LaneGuidance();

  void Process(const RoutingMapPtr routing_map_ptr, 
         const cem::fusion::SdJunctionInfoCityPtr junctions_ptr, 
         DetectBevMap &aligned_map,
         const std::vector<cem::message::env_model::StopLine> &stop_lines,
         std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result);

  std::shared_ptr<BevMapInfo> GetMapInfo() { return last_map_info_ptr_; };

  Eigen::Isometry3d GetTwb() { return T_local_ego_; };

 protected:
  struct CutPoint {
    uint64_t              lane_id;
    bool                  is_stop_line = false;
    std::vector<uint64_t> previous_lane_ids;
    std::vector<uint64_t> next_lane_ids;
    std::vector<uint64_t> merge_lane_ids;
    Eigen::Vector3d       dr_point  = Eigen::Vector3d::Zero();
    Eigen::Vector3d       ego_point = Eigen::Vector3d::Zero();
  };

  // Calculate the arc length of lanes
  void CalculateLengthOfLanes(std::vector<BevLaneInfo> &lane_infos);
  void CalculateLengthOfLanes(const std::vector<BevLaneInfo>::iterator &lane_begin_iter, 
                              const std::vector<BevLaneInfo>::iterator &lane_end_iter);

  // Get the foot of perpendicular from point pt to the line
  // segment formed by points begin and end
  Eigen::Vector2d GetFootPoint(const cem::message::common::Point2DF &pt, const cem::message::common::Point2DF &begin,
                               const cem::message::common::Point2DF &end);

  // Calculate the length of section based on one of the lane
  void CalculateLengthOfSection(std::vector<BevMapSectionInfo> &sections);

  // Calculate the section id and s offset of the ego vehicle for HNOA
  // void CalculateNaviStart(BevMapInfo &bev_map, std::vector<std::vector<LaneGuidance::CutPoint>> &multi_layer_cut_points);

  // Calculate the section id and s offset of the ego vehicle for CNOA
  void CalculateNaviStart(BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections);

  // Init pose member variable of dr
  void InitPose();

  // Integrate SD speed limit, road class and is_on_highway
  void SDStateFusion(BevMapInfo &bev_map, const RoutingMapPtr routing_map_ptr);

  // Assign emergency lane through SD
  void SetBevEmergencyLane(BevMapInfo &bev_map, const std::pair<uint64_t, uint64_t> &fusion_emergency_lane_id);

  // Add lane markers to section by lane marker ids
  void AddLaneMarkersToSection(const BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections,
                               std::vector<std::set<int>> lanemarker_ids, Graph<uint64_t> &lanemarker_graph);

  // Calculate the division line of the section
  void CalculateCutLine(BevMapSectionInfo &section, const LaneGuidance::CutPoint &lane_cut_point,
                        std::pair<double, double> &line_cofficients);

  bool IsLeftLane(std::vector<Eigen::Vector2f>& l1_geos, std::vector<Eigen::Vector2f>& l2_geos);

  // Calculate the relative position relationship of lanes within a section
  void CaculateLanePosition(BevMapSectionInfo &section, std::vector<BevLaneMarker> &edges, Graph<uint64_t> &lane_graph);

  // Reset the position of lanes within a section
  void ResetLanePosition(BevMapSectionInfo &section, std::vector<BevLaneMarker> &edges);

  void CutLaneMarkers(const std::pair<double, double> &line_cofficients, std::vector<BevLaneMarker> &lanemarkers,
                      BevMapSectionInfo &section_1st, BevMapSectionInfo &section_2nd, Graph<uint64_t> &lanemarkers_graph);

  void CutLanes(const std::pair<double, double> &line_cofficients, const std::vector<LaneGuidance::CutPoint> &cut_points,
                BevMapInfo &bev_map, uint64_t cur_road_id, BevMapSectionInfo &section_1st, BevMapSectionInfo &section_2nd,
                Graph<uint64_t> &lanes_graph, Graph<uint64_t> &lanemarkers_graph);

  void LaneGuidanceByRouteLanes(BevMapInfo &bev_map, Graph<uint64_t> &lane_graph,
                                std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                uint64_t &total_section_id);

  void InsertMergeSplitPoints(BevMapInfo &bev_map);

  bool IsLaneInPreviousSection(const BevLaneInfo &lane_infos, const cem::message::common::Point2DF &cut_point,
                               const std::pair<double, double> &line_cofficients);

  bool IsEgoInPreviousSection(const cem::message::common::Point2DF &cut_point, const std::pair<double, double> &line_cofficients);

  void RemoveRepeatPoint(BevMapInfo &bevMap);

  void CutLaneMarkerWithTypeSeg(BevMapInfo &bevMap);

  void CutSectionWidthTypeSeg(BevMapInfo &bevMap);

  void CutSectionWithTypeSeg(BevMapInfo &bev_map);

  void CutSectionWithTypeSegImplement(std::vector<BevMapSectionInfo> &sections, std::set<uint32_t> &laneMarkerIds);

  void BoundLaneMarkerWithLane(BevMapInfo &bev_map);

  void BoundSectionLaneMarkerWithLane(BevMapInfo &bev_map);

  void BoundSectionLaneMarkerWithLaneImplement(std::vector<BevMapSectionInfo> &sections, const BevMapInfo &bev_map);

  void AssignLaneMarkerColor(BevMapInfo &bev_map);

  void AssignSectionLaneMarkerColor(BevMapInfo &bev_map);

  void AssignSectionLaneMarkerColorImplement(std::vector<BevMapSectionInfo> &sections);

  template <typename T>
  bool HaveIntersection(const std::vector<T> &vec1, const std::vector<T> &vec2);

  void RemoveNonExistentSuccessor(BevMapInfo &bev_map);

  void RemoveNonExistentPredecessor(BevMapInfo &bev_map);

  std::vector<std::vector<LaneGuidance::CutPoint>> FindMultiLayerCutPoints(const BevMapInfo &bev_map,
    const std::vector<BevLaneInfo>::iterator &lane_begin_iter, const std::vector<BevLaneInfo>::iterator &lane_end_iter);

  void AssignSectionIDToLane(std::vector<BevMapSectionInfo>& sections);

  void RemoveSectionRepeatPoint(std::vector<BevMapSectionInfo>& sections);

  void RemoveLanemarkerRepeatPoint(BevMapInfo &bev_map);

  void RemoveSectionLanemarkerRepeatPoint(std::vector<BevMapSectionInfo>& sections);

  void RemoveAbnormalLane(std::vector<BevMapSectionInfo>& sections);

  void ReverseBackwardLane(std::vector<BevMapSectionInfo>& sections);

  void CutLaneInfoOutLaneMarker(BevMapInfo &bev_map);

  void LaneAssociationWithJunction(BevMapInfo &bev_map);

  // If the recommended lane's action is unknown, then initialize it to straight
  void InitRecommendedLaneAction(BevMapInfo &bev_map, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                 const RoutingMapPtr routing_map_ptr, const SdJunctionInfoCityPtr junctions_ptr);

  // 右转专用道判定函数
  void IsRightTurnOnly(const SdJunctionInfoCityPtr &junctions_ptr, const RoutingMapPtr &routing_map_raw);

  void InitLaneTurnType(BevMapInfo &bev_map, const cem::fusion::SdJunctionInfoCityPtr junctions_ptr);

  // Calculate distance to junction or Hypothetical junction
  void DistanceToJunction(BevMapInfo &bev_map);

  // Calculate stopline, crosswalk, junction coordinates in the ego frame
  void GetMarkerPositionEgoFrame(BevMapInfo &bev_map);

  // Find the vector position of the section and lane containing the sucessor of
  // the last element of the recommended lane
  std::optional<std::pair<std::size_t, size_t>> FindMaximumSectionAndLanePosition(uint64_t id, BevMapInfo &bev_map,
                                                                                  Graph<uint64_t> &lane_graph);

  // Add the virtual successor lane to next section
  void AddSuccessorLaneToSection(BevLaneInfo &lane_1st, BevLaneInfo &lane_2nd, BevMapSectionInfo &section, Graph<uint64_t> &lane_graph);

  // Calculate the lane segment in front of the ego vehicle
  std::optional<std::pair<uint64_t, uint64_t>> ValidLaneSegment(const BevLaneInfo &lane);

  // Calculate road span by road id
  void RoadSpan(const std::vector<BevLaneInfo> &lane_infos, std::vector<std::pair<uint64_t, uint64_t>> &roads_span);

  // Trace far recommended lane for lane guidance
  void TraceFarRecommendLane(BevMapInfo &bev_map, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result);

  // Special lane assignment for harbor stop and emergency lane
  void SpecialLaneAssignment(BevMapInfo &bev_map);

  // Remove emergency lane topo
  void RemoveSpecialLaneTopo(BevMapInfo &bev_map, const RoutingMapPtr routing_map_ptr,
      std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result);

  // Check and correct section merge topology
  void CheckSectionMergeTopo(BevMapInfo &bev_map);

  // Debug log function
  void DebugLog();

  bool                                           is_highway_                = false;
  double                                         distance_to_junction_      = std::numeric_limits<double>::max();
  bool                                           has_virtual_junction_lane_ = false;
  std::string                                    route_id_{0};
  bool                                           is_right_turn_only_{false};
  uint64_t                                       next_junction_id_{0};
  BevAction                                      recommend_action_ = BevAction::UNKNOWN;
  Eigen::Isometry3d                              T_local_ego_;
  Eigen::Isometry3d                              T_ego_local_;
  std::vector<cem::message::env_model::StopLine> stop_lines_;

  BevMapInfoPtr          last_map_info_ptr_ = nullptr;
  SDMapTopologyExtractor topology_extractor_;
  RoadDivider road_divider_;
};

}  // namespace fusion
}  // namespace cem

#endif  // LANE_GUIDANCE_H_
