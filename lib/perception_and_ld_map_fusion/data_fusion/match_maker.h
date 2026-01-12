#pragma once

#include "lib/perception_and_ld_map_fusion/data_fusion/road_slice.h"

namespace cem {
namespace fusion {

class MatchMaker {
 public:
  struct LaneMatchPair {
    uint64_t bev_id;
    uint32_t ld_map_group_id;
    std::vector<uint64_t> map_id;
    double probability;
  };
  struct MatchingDetail {
    uint64_t bev_id = 0;
    uint64_t ldmap_id = 0;
    size_t bev_start_point_idx = 0;  // from behind to front
    size_t bev_end_point_idx = 0;    // from behind to front
    std::unordered_map<int, double>
        point_probability;  // for each matched point
  };

 private:
  std::vector<LaneMatchPair> matchers_;
  std::unordered_map<uint64_t, std::vector<std::unique_ptr<MatchingDetail>>>
      matching_details_;
  std::unordered_map<uint64_t, std::vector<LaneMatchPair>> bev_rough_matchers_;
  std::unordered_map<uint64_t, std::vector<LaneMatchPair>> bev_rest_matchers_;
  static constexpr double POINT_MATCHED_PROBABILITY_THRESHOLD = 0.7;
  static constexpr double LANE_MATCHED_PROBABILITY_THRESHOLD = 0.7;
  // static constexpr double MATCHED_PROBABILITY_THREAD = 0.3;

 public:
  MatchMaker();
  ~MatchMaker();
  void Associate(
      std::unordered_map<int, std::unique_ptr<RoadSlice>>& bev_map_slices,
      std::unordered_map<int, std::unique_ptr<RoadSlice>>& ld_map_slices,
      const std::unordered_map<uint64_t, uint32_t>& ld_map_group,
      const std::unordered_map<uint64_t, std::vector<bool>>&
          bev_sampling_points_status);
  void ClearMatchers();
  void PrintMatchers() const;
  void PrintMatchingDetails() const;
  const std::vector<LaneMatchPair>& GetMatchers() const { return matchers_; };
  const std::unordered_map<uint64_t,
                           std::vector<std::unique_ptr<MatchingDetail>>>&
  GetMatchDetails() const {
    return matching_details_;
  };
  const std::unordered_map<uint64_t, std::vector<LaneMatchPair>>&
  GetRoughMatching() const {
    return bev_rough_matchers_;
  };
  const std::unordered_map<uint64_t, std::vector<LaneMatchPair>>&
  GetRestMatching() const {
    return bev_rest_matchers_;
  };

 private:
  void SliceMatching(cem::fusion::RoadSlice& bev_map_slice,
                     cem::fusion::RoadSlice& ld_map_slice);
};

}  // namespace fusion
}  // namespace cem
