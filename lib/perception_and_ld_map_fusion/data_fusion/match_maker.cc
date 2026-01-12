#include "lib/perception_and_ld_map_fusion/data_fusion/match_maker.h"

#include "middleware/middleware_util.h"

#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
extern bool print_debug_info;
#endif

namespace cem {
namespace fusion {

constexpr double MatchMaker::POINT_MATCHED_PROBABILITY_THRESHOLD;
constexpr double MatchMaker::LANE_MATCHED_PROBABILITY_THRESHOLD;

MatchMaker::MatchMaker() {}

MatchMaker::~MatchMaker() {}

void MatchMaker::Associate(
    std::unordered_map<int, std::unique_ptr<RoadSlice>>& bev_map_slices,
    std::unordered_map<int, std::unique_ptr<RoadSlice>>& ld_map_slices,
    const std::unordered_map<uint64_t, uint32_t>& ld_map_group,
    const std::unordered_map<uint64_t, std::vector<bool>>&
        bev_sampling_points_status) {
  ClearMatchers();

  std::unordered_map<std::pair<uint64_t, uint64_t>, MatchingDetail*, PairHash>
      matching_details_lookup_table;
  // Iterate over bev slices, match each bev-slice with the corresponding
  // ldmap-slice on the ld map slices, solidifying the best-matched sliding
  // window with bev-slice
  std::vector<int> sorted_slice_ids;
  std::unordered_map<uint64_t, std::pair<int, int>> bev_slice_range;
  for (auto iter = bev_map_slices.begin(); iter != bev_map_slices.end();
       ++iter) {
    auto it = std::lower_bound(sorted_slice_ids.begin(), sorted_slice_ids.end(),
                               iter->first);
    sorted_slice_ids.insert(it, iter->first);
  }
  for (auto slice_id : sorted_slice_ids) {
    if (ld_map_slices.count(slice_id) != 1) {
      continue;
    }

    // shift bev slice to find best matching with routing map slice
    auto& bev_map_slice = (*bev_map_slices.at(slice_id));
    auto& ld_map_slice = (*ld_map_slices.at(slice_id));
    // AINFO << "\n";
    if ((bev_map_slice.GetLaneSize() != ld_map_slice.GetLaneSize()) &&
        ((!bev_map_slice.HasRoadedgeFeature()) ||
         (!ld_map_slice.HasRoadedgeFeature()))) {
      continue;
    }
    SliceMatching(bev_map_slice, ld_map_slice);

    auto bev_lanes = bev_map_slice.GetLanes();
    for (auto id : bev_lanes) {
      if (bev_slice_range.count(id) == 0) {
        bev_slice_range[id].first = slice_id;
        bev_slice_range[id].second = slice_id;
      } else {
        bev_slice_range.at(id).second = slice_id;
      }
    }

    auto matchers = bev_map_slice & ld_map_slice;
#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    if (print_debug_info) {
      AINFO << "----------------- SLICE ID - " << slice_id
            << "-----------------";
      AINFO << bev_map_slice.GenerateSlicePrintInfo();
      AINFO << ld_map_slice.GenerateSlicePrintInfo();
    }
#endif
    for (auto& match_pair : matchers) {
#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
      if (print_debug_info) {
        AINFO << "slice id: " << slice_id;
        AINFO << "bev - ld:" << match_pair.first.first << " - "
              << match_pair.first.second << ", " << match_pair.second;
      }
#endif
      if (match_pair.second < MatchMaker::POINT_MATCHED_PROBABILITY_THRESHOLD) {
        continue;
      }
      auto bev_id = match_pair.first.first;
      auto ldmap_id = match_pair.first.second;
      auto pair_hash = std::pair<uint64_t, uint64_t>(bev_id, ldmap_id);
      if (matching_details_.count(bev_id) == 0) {
        matching_details_[bev_id].clear();
        matching_details_[bev_id].push_back(
            std::move(std::make_unique<MatchingDetail>()));
        auto matching_detail = matching_details_.at(bev_id).back().get();
        matching_details_lookup_table[pair_hash] = matching_detail;
        matching_detail->bev_id = bev_id;
        matching_detail->ldmap_id = ldmap_id;
        matching_detail->bev_start_point_idx = slice_id;
        matching_detail->bev_end_point_idx = slice_id + 1;
        matching_detail->point_probability[slice_id] = match_pair.second;

      } else {
        if (matching_details_lookup_table.count(pair_hash) == 0) {
          matching_details_[bev_id].push_back(
              std::move(std::make_unique<MatchingDetail>()));
          auto matching_detail = matching_details_.at(bev_id).back().get();
          matching_detail->bev_id = bev_id;
          matching_detail->ldmap_id = ldmap_id;
          matching_detail->bev_start_point_idx = slice_id;
          matching_detail->bev_end_point_idx = slice_id + 1;
          matching_detail->point_probability[slice_id] = match_pair.second;
          matching_details_lookup_table[pair_hash] = matching_detail;
        } else {
          auto matching_detail = matching_details_lookup_table.at(pair_hash);
          matching_detail->bev_end_point_idx = slice_id + 1;
          matching_detail->point_probability.emplace(slice_id,
                                                     match_pair.second);
        }
      }
    }
  }
  // for (auto iter = matching_details_.begin();
  //      iter != matching_details_.end();) {
  //   if (iter->second.empty() || bev_slice_range.count(iter->first) == 0) {
  //     iter = matching_details_.erase(iter);
  //     continue;
  //   }
  //   // AINFO << "bev start end:" << bev_slice_range.at(iter->first).first <<
  //   ",
  //   // "
  //   //       << bev_slice_range.at(iter->first).second;
  //   // std::string ldmap_ids_string = "(";
  //   // for (auto& matcher : iter->second) {
  //   //   ldmap_ids_string += std::to_string(matcher->ldmap_id);
  //   //   ldmap_ids_string += " ";
  //   // }
  //   // ldmap_ids_string.pop_back();
  //   // ldmap_ids_string += ")";
  //   // AINFO << "bev-ldmap: " << iter->first << "-" << ldmap_ids_string;
  //   // AINFO << iter->second.back()->bev_end_point_idx;
  //   if (bev_slice_range.at(iter->first).second >
  //       iter->second.back()->bev_end_point_idx + 10) {
  //     iter = matching_details_.erase(iter);
  //     continue;
  //   }
  //   ++iter;
  // }
  for (auto iter = matching_details_.begin();
       iter != matching_details_.end();) {
    //   AINFO << "bev id: " << iter->first;
    //   std::string tmp = "";
    //   for (auto& detail : iter->second) {
    //     tmp += std::to_string(detail->ldmap_id);
    //     tmp += ", ";
    //   }
    //   AINFO << "map_id: " << tmp;
    // }
    // AINFO << "bev id: " << iter->first;
    auto& lane_sampling_points_status =
        bev_sampling_points_status.at(iter->first);
    auto& bev_lane_match = (*iter);
    struct MatchedLdMapInfo {
      int matched_point_num;
      std::vector<uint64_t> lane_elements_id;
      uint32_t lane_group_id;
      uint32_t start_bev_slice_id;
      uint32_t end_bev_slice_id;
    };
    // std::unordered_map<uint32_t, std::pair<int, std::vector<uint64_t>>>
    std::unordered_map<uint32_t, MatchedLdMapInfo> matched_statistic;
    size_t bev_start_point_idx = std::numeric_limits<size_t>::max();
    size_t bev_end_point_idx = 0;
    for (auto& map_lane_matched : bev_lane_match.second) {
      if (ld_map_group.count(map_lane_matched->ldmap_id) == 0) {
        AERROR << "LdMap lane id not in its topology map. id:"
               << map_lane_matched->ldmap_id;
        continue;
      }
      auto ld_map_lane_virtual_id = ld_map_group.at(map_lane_matched->ldmap_id);
      if (matched_statistic.count(ld_map_lane_virtual_id) == 0) {
        auto& statistic_info = matched_statistic[ld_map_lane_virtual_id];
        statistic_info.matched_point_num =
            map_lane_matched->point_probability.size();
        statistic_info.lane_elements_id =
            std::vector<uint64_t>(1, map_lane_matched->ldmap_id);
        statistic_info.lane_group_id = ld_map_lane_virtual_id;
        statistic_info.start_bev_slice_id =
            map_lane_matched->bev_start_point_idx;
        statistic_info.end_bev_slice_id = map_lane_matched->bev_end_point_idx;
      } else {
        auto& statistic_info = matched_statistic[ld_map_lane_virtual_id];
        statistic_info.matched_point_num +=
            map_lane_matched->point_probability.size();
        statistic_info.lane_elements_id.push_back(map_lane_matched->ldmap_id);
        statistic_info.end_bev_slice_id = map_lane_matched->bev_end_point_idx;
      }
    }

    // bool bev_lane_not_matched = true;
    // std::vector<LaneMatchPair> mix_matchers;
    for (auto& [matched_ld_id, matched_ldmap_info] : matched_statistic) {
      int total_point_num =
          std::count(lane_sampling_points_status.begin() +
                         matched_ldmap_info.start_bev_slice_id,
                     lane_sampling_points_status.begin() +
                         matched_ldmap_info.end_bev_slice_id,
                     true);
      LaneMatchPair matcher;
      matcher.bev_id = bev_lane_match.first;
      matcher.ld_map_group_id = matched_ld_id;

      auto& matched_ldmap_segs = matched_ldmap_info.lane_elements_id;
      for (auto ldmap_lane : matched_ldmap_segs) {
        matcher.map_id.push_back(ldmap_lane);
      }
      if (total_point_num < 5) {
        matcher.probability = 0;
      } else {
        matcher.probability =
            double(matched_ldmap_info.matched_point_num) / total_point_num;
      }

#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
      if (print_debug_info) {
        AINFO << matched_ldmap_info.matched_point_num << ", "
              << total_point_num;
        AINFO << "bev:" << matcher.bev_id;
        std::string map_id = "";
        for (auto id : matcher.map_id) {
          map_id += std::to_string(id);
          map_id += ", ";
        }
        AINFO << "ldmap:" << map_id;
        AINFO << "prob:" << matcher.probability;
        AINFO << "begin bev slice: " << matched_ldmap_info.start_bev_slice_id
              << "end bev slice: " << matched_ldmap_info.end_bev_slice_id;
      }
#endif
        if (matcher.probability >
            MatchMaker::LANE_MATCHED_PROBABILITY_THRESHOLD) {
          matchers_.push_back(std::move(matcher));
        } else {
          for (auto id : matched_ldmap_segs) {
            auto iter_find = std::find_if(
                bev_lane_match.second.begin(), bev_lane_match.second.end(),
                [id](const std::unique_ptr<MatchingDetail>& detail) {
                  return detail->ldmap_id == id;
                });
            if (iter_find != bev_lane_match.second.end()) {
              bev_lane_match.second.erase(iter_find);
            }
          }
        }
    }

    if (bev_lane_match.second.empty()) {
      iter = matching_details_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void MatchMaker::ClearMatchers() {
  matchers_.clear();
  matching_details_.clear();
  bev_rough_matchers_.clear();
  bev_rest_matchers_.clear();
}

void MatchMaker::PrintMatchers() const {
  for (auto& matcher : matchers_) {
    std::string ldmap_ids_string = "(";
    for (auto ldmap_lane_id : matcher.map_id) {
      ldmap_ids_string += std::to_string(ldmap_lane_id);
      ldmap_ids_string += " ";
    }
    ldmap_ids_string.pop_back();
    ldmap_ids_string += ")";
    AINFO << "ld_virtual_id: " << matcher.ld_map_group_id
          << ", bev-ldmap: " << matcher.bev_id << "-" << ldmap_ids_string
          << ", prob:" << matcher.probability;
  }
}

void MatchMaker::PrintMatchingDetails() const {
  for (auto& match_detail : matching_details_) {
    std::string ldmap_ids_string = "(";
    for (auto& matcher : match_detail.second) {
      ldmap_ids_string += std::to_string(matcher->ldmap_id);
      ldmap_ids_string += " ";
    }
    ldmap_ids_string.pop_back();
    ldmap_ids_string += ")";
    AINFO << "bev-ldmap: " << match_detail.first << "-" << ldmap_ids_string;
  }
}

void MatchMaker::SliceMatching(cem::fusion::RoadSlice& bev_map_slice,
                               cem::fusion::RoadSlice& ld_map_slice) {
  double match_probability = bev_map_slice * ld_map_slice;
  int shift_step = 0;
  while (true) {
    if (!bev_map_slice.LeftShift()) {
      break;
    }
    double tmp = bev_map_slice * ld_map_slice;

    // #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    //     if (print_debug_info) {
    //       AINFO << "SHITF STEP: " << bev_map_slice.GetShiftStep();
    //       AINFO << "PROB: " << tmp;
    //     }
    // #endif
    if (tmp > match_probability) {
      match_probability = tmp;
      shift_step = bev_map_slice.GetShiftStep();
    }
  }
  bev_map_slice.ResetShift();
  while (true) {
    if (!bev_map_slice.RightShift()) {
      break;
    }
    double tmp = bev_map_slice * ld_map_slice;
    // #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    //     if (print_debug_info) {
    //       AINFO << "SHITF STEP: " << bev_map_slice.GetShiftStep();
    //       AINFO << "PROB: " << tmp;
    //     }
    // #endif
    if (tmp > match_probability) {
      match_probability = tmp;
      shift_step = bev_map_slice.GetShiftStep();
    }
  }
  bev_map_slice.SetShiftStep(shift_step);
  bev_map_slice.StablizeShift();
  // AINFO << "shift_step: " << shift_step;
}

}  // namespace fusion
}  // namespace cem
