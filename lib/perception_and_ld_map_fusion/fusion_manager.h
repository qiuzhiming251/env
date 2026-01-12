#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>

#include "lib/message/internal_message.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/bev_filleting_machine.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/ld_map_filleting_machine.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/match_maker.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/multi_source_data_mixer.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/bev_map_preprocessor.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/build_lane_topology.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/ld_map_preprocessor.h"
#include "message/env_model/routing_map/routing_map.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/lane_type_checker.h"
// #include "lib/perception_and_ld_map_fusion/visualization/visualizer.h"
#include "lib/map_fusion/map_fusion.h"
#include "message/env_model/navigation/navigation.h"

namespace cem {
namespace fusion {

class FusionManager {

 private:
  std::unique_ptr<BevMapProcessor> bev_map_preprocessor_ = nullptr;
  std::unique_ptr<LaneTypeChecker> bev_lane_type_checker_ = nullptr;
  std::unique_ptr<LdMapProcessor> ld_map_preprocessor_ = nullptr;
  std::unique_ptr<BevFilletingMachine> bev_slicer_ = nullptr;
  std::unique_ptr<LdMapFilletingMachine> ldmap_slicer_ = nullptr;
  std::unique_ptr<MatchMaker> match_maker_ = nullptr;
  // std::unique_ptr<Visualizer> visualizer_ = nullptr;
  std::unique_ptr<MultiSourceDataMixer> data_fusion_controller_ = nullptr;
  std::unique_ptr<MapFusion>            map_fusion_             = nullptr;
  RoutingMapPtr ld_map_ = nullptr;
  BevMapInfoPtr bev_map_ = nullptr;
  std::vector<std::pair<uint64_t, std::pair<int, int>>>
      recommend_driving_lanes_;
  std::string debug_info_ = "";
  std::vector<BevSplitInfo> bev_split_info_;
  std::vector<BevMergeInfo> bev_merge_info_;
  std::vector<FarTopoInfo> far_merge_info_;
  double ego_lane_lat_error_between_ld_and_bev_;
  uint64_t estimated_bev_ego_lane_id_;
  double ego_lane_compare_length_;
  double ld_ego_lane_c0_;
  double bev_ego_lane_c0_;

  void RecommendDrivingLanes();
  void PrintRecommendDrivingLanes();
  void PrintLaneTopoInfo();
  void DebugLaneTopoInfo();
  void MappingMergeSplitInfoFromLdMapToBevMap();
  void GetBevSplitConnectInfo(uint32_t input_group_id, BevSplitInfo& lane_topo_info);
  void GetBevMergeConnectInfo(uint32_t to_group_id, BevMergeInfo& bev_merge_info);
  void BuildFarTopoInfo(
    uint64_t bev_id, 
    uint32_t group_from, 
    const LdMapProcessor::MergeSplitTarget& target_to);
  void BuildFarTopoForward(uint64_t bev_id, uint32_t input_group_id, std::set<uint32_t>& visited_groups);
  std::optional<Eigen::Isometry3d> FindTransform(const double& timestamp);
  void EstimateEgoLaneErrorBetweenLdMapAndBevMap();
  double EstimateEgoLaneCompareDistance();
  bool FitPolynomial(const std::vector<Eigen::Vector2d>& line,
                     Eigen::Vector4d& coefficients_out);

 public:
  void Init();
  void Process();
  void DataFusion();
  void SetCrossRoadInfo(bool cross_road_status, double cross_road_distance, bool is_turn_right);
  void SetAdc2Junction(double adc_to_junction);

  const std::vector<MatchMaker::LaneMatchPair>& GetMatchers() const {
    return match_maker_->GetMatchers();
  };
  LaneType GetLaneTypeByBevId(uint64_t bev_id) const;
  void GetSplitInfoByBevId (uint64_t bev_id);
  void GetMergeInfoByBevId(uint64_t bev_id, std::vector<const LdMapProcessor::MergeSplitInfo*> merge_connect);
  void CheckChangeTopoByBevId(uint64_t bev_id);
  // New:
  std::vector<const BevSplitInfo*> GetBevLaneSplitInfo(uint64_t id_from) const;
  std::vector<const BevMergeInfo*> GetBevLaneMergeInfo(uint64_t search_id, bool search_lane_to = false) const;
  std::vector<const FarTopoInfo*> GetFarMergeInfo(uint64_t bev_id, double min_distance=0, double max_distance=500) const;

  const std::vector<std::pair<uint64_t, std::pair<int, int>>>&
  GetRecommendPath() const {
    return recommend_driving_lanes_;
  };
  const std::string& DebugInfo() const { return debug_info_; };

  const bool IsOnFreeway() const {
    return ld_map_preprocessor_->IsOnFreeWay();
  };

  [[nodiscard]] BevMapInfoPtr GetBevMapInfo() const { return bev_map_; }

  [[nodiscard]] RoutingMapPtr GetLdMapInfo() const { return ld_map_; }

  inline const cem::message::env_model::SectionInfo* const GetLdSection(
      uint64_t id) const {
    return ld_map_preprocessor_->GetSection(id);
  }

  inline std::vector<traverseCrossWalkLane>& GetTraverseCrossWalkLaneInfo() {
    return bev_map_preprocessor_->GetTraverseCrossWalkLaneInfo();
  };

  inline double GetDistanceToUnreliableRoad() const {
    return ld_map_preprocessor_->GetDistanceToUnreliableRoad();
  };

  inline const LdMapProcessor::FunctionMode GetFunctionMode() const {
    return ld_map_preprocessor_->GetFunctionMode();
  };

  inline void GetEgoLaneMatchResult(GeometryMatchResult& result) const {
    result.is_matched = ego_lane_lat_error_between_ld_and_bev_ <= 0.4;
    result.left_offset_average = ego_lane_lat_error_between_ld_and_bev_;
    result.left_offset_max = ego_lane_compare_length_;
    result.ld_line_c0 = ld_ego_lane_c0_;
    result.bev_line_c0 = bev_ego_lane_c0_;
  }

  inline bool GetEgoLaneMatchInfo(
      uint64_t& bev_ego_lane_id_out,
      std::vector<uint64_t>& ld_ego_lane_ids_out) const {
    if (ego_lane_lat_error_between_ld_and_bev_ > 0.4) {
      return false;
    }
    bev_ego_lane_id_out = estimated_bev_ego_lane_id_;
    ld_ego_lane_ids_out = ld_map_preprocessor_->GetEgoLanes();
    return true;
  };
};

}  // namespace fusion
}  // namespace cem
