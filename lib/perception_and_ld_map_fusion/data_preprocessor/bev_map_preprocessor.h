#pragma once

#include "lib/localmap_construction/diversion_mapping.h"
#include "lib/localmap_construction/lane_mapping.h"
#include "lib/message/sensor/camera/bev_lane/bev_lane.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/data_preprocessor.h"
#include "localmap_construction/lane_topology_processor.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/match_maker.h"


namespace cem {
namespace fusion {

struct BevSplitTarget {
  uint64_t bev_lane_to;
  std::vector<uint64_t> map_lanes_to;
  LdMapProcessor::MergeSplitType type;
};
struct BevMergeSource {
  uint64_t bev_lane_from;
  std::vector<uint64_t> map_lanes_from;
  LdMapProcessor::MergeSplitType type;
};
struct BevSplitInfo {
  uint64_t bev_lane_from;
  std::vector<uint64_t> map_lanes_from;
  std::vector<std::pair<uint64_t, BevSplitTarget>> bev_target_to;
  double distance_to_ego;
  LdMapProcessor::MergeSplitType match_type;
};
struct BevMergeInfo {
  std::unordered_map<uint64_t, BevMergeSource> bev_lanes_from;
  uint64_t bev_lane_to; // 没有匹配上是0
  std::vector<uint64_t> map_lanes_to;
  double distance_to_ego;
  LdMapProcessor::MergeSplitType match_type;
};
struct FarTopoInfo {
  uint64_t bev_id;
  uint32_t group_from;
  uint32_t group_to;
  double distance_to_ego;
  LdMapProcessor::MergeSplitType type;
};
class BevMapProcessor
    : public DataPreprocessor<cem::message::sensor::BevMapInfo> {
 public:
  struct TopoRecord {
    int record_number;
    uint64_t main_lane_id;
    std::set<uint64_t> sub_lane_ids;
  };

  struct BreakTopoRecord {
    int record_number;
    uint64_t break_lane_id;
    uint64_t other_lane_id;
    int break_lane_type;
    int other_lane_type;
    int add_lane_type;
  };

  struct BreakLaneResult {
    uint64_t break_lane_id;
    uint64_t other_lane_id;
    uint64_t add_lane_id;
    int break_lane_type;
    int other_lane_type;
    int add_lane_type;
};

 public:
  BevMapProcessor();
  ~BevMapProcessor();
  bool RotateAndTranslate(const Eigen::Isometry3d& T_meas2tgt) override;
  void EsitmateSliceParameters(const Eigen::Vector2d& road_direction,
                               Eigen::Vector2d& slice_start_point,
                               double& slice_length);
  void SetCrossParam(bool cross_road_status, double cross_road_distance, bool is_turn_right);
  void SetAdc2Junction(double adc_to_junction);
  void SetRightTurnToLane(uint64_t id);
  inline cem::message::sensor::BevLaneInfo* GetLane(uint64_t id) {
    if (lane_map_.count(id) == 0) {
      return nullptr;
    } else {
      return lane_map_.at(id);
    }
  }

  inline std::vector<traverseCrossWalkLane>& GetTraverseCrossWalkLaneInfo() {
    crosswalk_lane_list_.clear();
    lane_topology_processor_.GetTraverseCrossWalkLaneInfo(crosswalk_lane_list_);
    return crosswalk_lane_list_;
  };

  void TopoProcessor(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                     const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                     const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                     const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                     const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                     const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                     const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                     const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                     const bool& low_precision_flag,
                     std::string& debug_infos);

 private:
  void Proc() override;

  void FillNearLaneId();

  void ConstructLanesIndex();

  void AddLaneTopology(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                       const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                       const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                       const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                       const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                       const bool& low_precision_flag,
                       std::string& debug_infos);

  void DeleteLaneTopology(const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter,
                       const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                       const std::function<LaneType(uint64_t)>& LaneTypeGetter,
                       const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                       const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                       const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                       const bool& low_precision_flag,
                       std::string& debug_infos);

  navigation::EmergencyLaneInfo GetEmergencyLaneInfo();

  void GetFilteredShortLaneIds(std::vector<int>& left_most_idx,
                               std::vector<int>& right_most_idx);

  // 根据bev_track标签、路沿和SD地图信息提取应急车道
  void GetEmergencyLaneIds(
      const navigation::EmergencyLaneInfo& sd_emergency_lane_info,
      std::set<uint64_t>& current_bev_emergency_laneid,
      std::vector<int>& target_emergency_left_idx,
      std::vector<int>& target_emergency_right_idx);

  // 根据路沿和SD地图信息提取港湾/临停区车道
  void GetHarborLaneIds(const navigation::HarborStopInfo& sd_habor_lane_info,
                        std::set<uint64_t>& current_bev_habor_laneid);

  double calculateOverlapDistance(const navigation::HarborStopInfo& harbor,
                                  const std::vector<Eigen::Vector2f> geos);

  double CalculateCurvedLaneWidth2(const std::vector<Eigen::Vector2f>& lane1,
                                   const std::vector<Eigen::Vector2f>& lane2);

  void ProcessStartEndTopo(std::vector<int>& target_lane_inds);

  void ProcessEndStartTopo(std::vector<int>& target_lane_inds);

  void ProcessYshapeSplit(std::vector<int>& target_lane_inds);

  void ProcessYshapeMerge(std::vector<int>& target_lane_inds);

  bool BreakLaneSplit(uint64_t target_lane_id, uint64_t topo_lane_id, std::shared_ptr<cem::message::env_model::RoutingMap> ld_map, const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos, LdMapProcessor::MergeSplitType match_type, LdMapProcessor::MergeSplitType target_lane_type, LdMapProcessor::MergeSplitType topo_lane_type, double distance_to_ego, json &debug_info);

  bool BreakLaneMerge(uint64_t target_lane_id, uint64_t topo_lane_id, std::shared_ptr<cem::message::env_model::RoutingMap> ld_map, const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos, LdMapProcessor::MergeSplitType match_type, LdMapProcessor::MergeSplitType target_lane_type, LdMapProcessor::MergeSplitType topo_lane_type, double distance_to_ego, json &debug_info);

  void FindNeighbourLanes();

  void FindUTurnLDLane(std::vector<uint64_t> ld_match_ids,
                       const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                       std::vector<cem::message::env_model::LaneInfo>& u_turn_lanes);

  std::optional<Eigen::Isometry3d> FindTransform(const double& timestamp);

  std::optional<const BevSplitInfo*> GetSplitInfoForTopoAdd (const std::function<std::vector<const BevSplitInfo*>(uint64_t)>& LaneSplitGetter, 
                                                             cem::message::sensor::BevLaneInfo* main_lane,
                                                             const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map);

  std::optional<const BevMergeInfo*> GetMergeInfoForTopoAdd (const std::function<std::vector<const BevMergeInfo*>(uint64_t, bool)>& LaneMergeGetter,
                                                             cem::message::sensor::BevLaneInfo* main_lane,
                                                             const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map);

  double GetEgoToEndLength (cem::message::sensor::BevLaneInfo* main_lane);

  std::optional<BreakLaneResult> AddSplitTopoByMapInfo (std::optional<const BevSplitInfo*> split_info_result, 
                              cem::message::sensor::BevLaneInfo* main_lane,
                              const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                              const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                              const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& split_match_infos,
                              const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                              json &topo_debug_info);

  std::optional<BreakLaneResult> AddMergeTopoByMapInfo (std::optional<const BevMergeInfo*> merge_info_result, 
                              cem::message::sensor::BevLaneInfo* main_lane,
                              const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                              const std::map<uint64_t, std::vector<uint64_t>>& bev_ld_match,
                              const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& merge_match_infos,
                              const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& connect_match_infos,
                              json &topo_debug_info);

  bool CheckMainlaneDifferenceConstraint(cem::message::sensor::BevLaneInfo* main_lane,
                                                        const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                                        const std::vector<uint64_t>& main_ld_match,
                                                        const std::vector<uint64_t>& sub_ld_match,
                                                        bool select_multi_ld,
                                                        bool is_split_flag);

  bool CheckSublaneDifferenceConstraint(cem::message::sensor::BevLaneInfo* sub_lane,
                                                const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map,
                                                const std::vector<uint64_t>& sub_ld_match, 
                                                const std::vector<uint64_t>& main_ld_match, 
                                                bool is_split_flag, 
                                                bool is_match_flag);

  bool CheckTopoPointLongitudeDistanceConstraint (cem::message::common::Point2DF topo_point,
                                               double topo_dist_to_ego,
                                               std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
                                               const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& match_infos);

  bool CheckSplitLanePositionConstraint (LdMapProcessor::MergeSplitType match_type,
                                         LdMapProcessor::MergeSplitType lane1_type,
                                         LdMapProcessor::MergeSplitType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos);

  bool CheckMemorySplitLanePositionConstraint (SplitTopoExtendType lane1_type,
                                         SplitTopoExtendType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos);

  bool CheckMergeLanePositionConstraint (LdMapProcessor::MergeSplitType match_type,
                                         LdMapProcessor::MergeSplitType lane1_type,
                                         LdMapProcessor::MergeSplitType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos);

  bool CheckMemoryMergeLanePositionConstraint (MergeTopoExtendType lane1_type,
                                         MergeTopoExtendType lane2_type,
                                         const std::vector<Eigen::Vector2f> &lane1_geos, 
                                         const std::vector<Eigen::Vector2f> &lane2_geos);

  bool CheckEgoLaneEmergencyConstraint(const navigation::EmergencyLaneInfo& sd_emergency_lane_info);

  bool CheckSplitLaneRelationConstraint(const cem::message::sensor::BevLaneInfo& main_lane, 
                                        const std::vector<cem::message::sensor::BevLaneInfo>& sub_lanes, 
                                        bool break_lane_flag, 
                                        json &topo_debug_info);

  bool CheckMergeLaneRelationConstraint(const cem::message::sensor::BevLaneInfo& main_lane, 
                                        const std::vector<cem::message::sensor::BevLaneInfo>& sub_lanes, 
                                        bool break_lane_flag, 
                                        json &topo_debug_info);

  bool CheckLDSplitTopoType(LdMapProcessor::MergeSplitType topo_type, cem::message::env_model::SplitTopology ld_topo_type);

  bool CheckLDMergeTopoType(LdMapProcessor::MergeSplitType topo_type, cem::message::env_model::MergeTopology ld_topo_type);

  bool CheckLeftBending(const cem::message::sensor::BevLaneInfo& lane, json &topo_debug_info);

  double CalculateCurvature(const cem::message::sensor::BevLaneInfo& lane, size_t idx);

  float CalculateCenterBoundaryDistanceFromMap(const cem::message::env_model::LaneInfo& lane, 
                                               const std::shared_ptr<cem::message::env_model::RoutingMap>& ld_map);

  bool UpadateLaneIDMemory(std::set<uint64_t> current_lane_ids, std::pair<std::set<uint64_t>, int> &lane_id_memory_result);

  void AddSplitByHistory(uint64_t target_lane_id, json &topo_debug_info);

  std::optional<BreakLaneResult> AddSplitBreakByHistory(uint64_t input_lane_id, json &topo_debug_info);

  void AddMergeByHistory(uint64_t target_lane_id, json &topo_debug_info);

  std::optional<BreakLaneResult> AddMergeBreakByHistory(uint64_t input_lane_id, json &topo_debug_info);

  void UpadateSplitMemory(bool update_lane_id_flag);

  void UpadateMergeMemory(bool update_lane_id_flag);

  void UpadateBreakMemory(bool update_lane_id_flag, std::unordered_map<uint64_t, BevMapProcessor::BreakTopoRecord> &memory_break_result, std::optional<BreakLaneResult> break_lane_result);

  void ConfirmEmergencyLane(const std::function<LaneType(uint64_t)> &LaneTypeGetter, std::string &debug_infos, bool EgoIsEmergency);

  void RemoveDuplicateLaneIds();

  void RemoveNonExistentSuccessor();

  void RemoveNonExistentPredecessor();

 public:
  std::shared_ptr<cem::fusion::LaneTracker> lane_tracker_ptr_;

 private:
  bool cross_road_status_ = false;
  double cross_road_distance_ = -1.0;
  bool is_turn_right_ = false;
  double adc_to_junction_ = -1.0;
  uint64_t prev_ego_lane_id = 0;
  uint64_t prev_ego_lane_trans_id = 0;

  std::unordered_map<uint64_t, cem::message::sensor::BevLaneInfo*> lane_map_;
  std::vector<int> ego_lane_inds_;
  std::set<uint64_t> filtered_lane_ids_;
  std::vector<int> guide_lane_inds_;
  std::unordered_map<double, std::vector<int>> neighbour_lane_inds_map_;
  std::optional<Eigen::Isometry3d> T_local_ego_;

  std::shared_ptr<cem::fusion::DiversionTracker> diversion_ptr_;
  ProcessLanesTopo lane_topology_processor_;
  std::vector<traverseCrossWalkLane> crosswalk_lane_list_;

  std::pair<std::set<uint64_t>, int> lane_id_memory_result_ = {std::set<uint64_t>(), 0};
  std::pair<std::set<uint64_t>, int> lane_id_memory_before_split_ = {std::set<uint64_t>(), 0};
  std::pair<std::set<uint64_t>, int> lane_id_memory_before_merge_ = {std::set<uint64_t>(), 0};
  std::unordered_map<uint64_t, BevMapProcessor::TopoRecord> split_memory_result_;
  std::unordered_map<uint64_t, BevMapProcessor::BreakTopoRecord> split_memory_break_result_;
  std::unordered_map<uint64_t, SplitTopoExtendType> split_memory_type_;
  std::unordered_map<uint64_t, BevMapProcessor::TopoRecord> merge_memory_result_;
  std::unordered_map<uint64_t, BevMapProcessor::BreakTopoRecord> merge_memory_break_result_;
  std::unordered_map<uint64_t, MergeTopoExtendType> merge_memory_type_;
};

}  // namespace fusion
}  // namespace cem
