#pragma once

#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/perception_and_ld_map_fusion/data_preprocessor/data_preprocessor.h"

namespace cem {
namespace fusion {

class LdMapProcessor
    : public DataPreprocessor<cem::message::env_model::RoutingMap> {
  public:
  enum MergeSplitType {
    kConnectForward = 0,
    kMergeLeft = 1,
    kMergeRight = 2,
    kSplitLeft = 3,
    kSplitRight = 4
  };
  struct MergeSplitTarget {
    uint64_t lane_to;
    uint32_t group_to;
    double distance_to_ego;
    MergeSplitType type;
  };
  struct MergeSplitInfo {
    uint64_t lane_from;
    uint32_t group_from;
    std::unordered_map<uint64_t, MergeSplitTarget> target_to;
  };

  enum class FunctionMode {
    kMNOA = 0,
    kHNOA = 1,
  };

 private:
  enum PathStatus {
    kNotUpdated = 0,
    kUpdated = 1,
    kLost = 2,
    kFinished = 3,
    kConnectedForward = 4,
    kMergeForward = 5,
    kSplitForward = 6,
  };
  struct MapLaneInfo {
    bool is_classified = false;
    cem::message::env_model::LaneInfo* ptr = nullptr;
  };
  struct TrackedLaneIdInfo {
    uint32_t lane_group_id;
    bool is_exist;
  };
  struct LaneGroup {
    enum UpdateStatus { kUpdated = 0, kNotUpdate = 1, kFinished = 2 };
    uint32_t group_id;
    std::vector<uint64_t> eles;
    uint64_t predecessor_ele;
    uint64_t successor_ele;
    UpdateStatus update_status;
    double length;

    LaneGroup(uint32_t id) {
      group_id = id;
      eles.clear();
      predecessor_ele = 0;
      successor_ele = 0;
      update_status = UpdateStatus::kUpdated;
      length = 0.0;
    };
  };

 public:
  struct Path {
    std::vector<std::pair<uint32_t, double>> lanes;  // lane_id, distance
    std::unordered_set<uint64_t> lane_eles_id;
    uint64_t start_ele_id;
    uint64_t end_ele_id;
    PathStatus status;
    std::vector<std::pair<uint32_t, MergeSplitType>> change_point;
    uint64_t start_section_key;
    bool is_start = false;
    bool from_start_section = false;

    void PrintPathInfo();
  };

 public:
  LdMapProcessor();
  ~LdMapProcessor();
  bool RotateAndTranslate(const Eigen::Isometry3d& T_meas2tgt) override;
  bool IsOnFreeWay() { return is_on_freeway_; };
  void ClearCache();
  inline const Eigen::Vector2d& GetMainDirection() const {
    return main_direction_;
  };
  inline const std::unordered_map<uint64_t, uint32_t>& GetLaneGroup() const {
    return lane_id_to_group_id_;
  }
  inline const cem::message::env_model::LaneInfo* const GetLane(
      uint64_t id) const {
    if (lane_map_.count(id) == 0) {
      return nullptr;
    } else {
      return lane_map_.at(id).ptr;
    }
  }
  inline const cem::message::env_model::SectionInfo* const GetSection(
      uint64_t id) const {
    if (section_map_.count(id) == 0) {
      return nullptr;
    } else {
      return section_map_.at(id);
    }
  }
  inline const std::vector<Path>& GetRecommendPath() const {
    return recommend_paths_;
  }
  inline const std::unordered_map<uint64_t, MergeSplitInfo>& GetMergeInfo() const {
    return merge_info_;
  }
  inline const std::unordered_map<uint64_t, MergeSplitInfo>& GetSplitInfo() const {
    return split_info_;
  }
  inline const std::unordered_map<uint64_t, MergeSplitInfo>& GetConnectInfo() const {
    return connect_info_;
  }
  inline double GetDistanceToUnreliableRoad() const {
    return distance_to_unrealiable_road_;
  };

  inline const FunctionMode GetFunctionMode() { return function_mode_; };

  inline const std::vector<uint32_t>& GetEgoLaneGroups() const {
    return ego_lane_candidate_groups_;
  };

  inline const std::vector<uint64_t>& GetEgoLanes() const {
    return ego_lane_candidate_eles_;
  };

  inline const std::vector<Eigen::Vector2d>& GetEgoLanePoints() const {
    return ego_lane_points_;
  };
  inline const std::string GetEgoLaneElementsDebugInfo() const {
    std::string text_id = "Ld ego Lane id: ";
    for (auto id : ego_lane_candidate_eles_) {
      text_id += std::to_string(id) + ", ";
    }
    return text_id;
  };

  inline const std::string GetEgoLaneGroupDebugInfo() const {
    std::string text_id = "Ld ego Lane groups: ";
    for (auto id : ego_lane_candidate_groups_) {
      text_id += std::to_string(id) + ", ";
    }
    return text_id;
  };

  inline const std::string GetEgoLanePointsDebugInfo() const {
    std::string text_pt = "Ld ego Lane Points: ";
    for (auto pt : ego_lane_points_) {
      text_pt +=
          "(" + std::to_string(pt.x()) + ", " + std::to_string(pt.y()) + "), ";
    }
    return text_pt;
  };

 private:
  void Proc() override;
  void EstimateMainDirection();
  void ConstructLaneTopology();
  void CreateLaneInfoIndex();
  void ConstructLaneSequence();
  void ExtractLaneGroupsAndSearchEgoLane();
  void ExtractEgoLanePoints();
  void SearchRoutePath();
  void JudgeIsOnFreewayOrNot();
  void SearchUnreliableRoad();

 private:
  Eigen::Vector2d main_direction_;
  uint32_t lane_group_count_ = 1U;

  std::unordered_map<uint64_t, MapLaneInfo> lane_map_;
  std::unordered_map<uint64_t, SectionInfo*> section_map_;
  std::unordered_map<uint64_t, uint32_t> lane_id_to_group_id_;
  std::unordered_map<uint32_t, std::vector<uint64_t>> lanes_;
  std::unordered_map<uint64_t, TrackedLaneIdInfo> lane_group_id_tracked_;
  std::unordered_map<uint64_t, MergeSplitInfo>
      merge_info_;  // key: id_merge_from
  std::unordered_map<uint64_t, MergeSplitInfo>
      split_info_;  // key: id_split_from
  std::unordered_map<uint64_t, MergeSplitInfo> connect_info_;
  std::vector<Path> recommend_paths_;
  bool is_on_freeway_ = false;
  FunctionMode function_mode_ = FunctionMode::kMNOA;
  double distance_to_unrealiable_road_ = 2000.0;
  std::unordered_map<uint32_t, LaneGroup> lane_groups_details_;
  uint32_t ego_lane_group_id_;
  std::vector<Eigen::Vector2d> ego_lane_points_;
  std::vector<uint32_t> ego_lane_candidate_groups_;
  std::vector<uint64_t> ego_lane_candidate_eles_;
};

}  // namespace fusion
}  // namespace cem
