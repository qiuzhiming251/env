#include "SDMapTopoExtract.h"
#include <log_custom.h>
#include <algorithm>
#include <limits>
#include <map>
#include <fmt/format.h>
#include "base/params_manager/params_manager.h"
#include "lib/sd_navigation/GeometryLaneMatcher.h"
#include "lib/sd_navigation/routing_map_debug.h"

namespace cem {
namespace fusion {
namespace navigation {

SDMapTopologyExtractor::SDMapTopologyExtractor() {
  merge_rear_distance_  = 0.0;
  merge_front_distance_ = 500.0;
}

void SDMapTopologyExtractor::GetMergeTopologiesFromMap(std::vector<MergeDetail> &merge_details) {
  merge_details.clear();
  const bool use_ld = is_on_highway_;
  SD_MERGE_LOG << fmt::format("[Topo] Start extracting merges via {} map", use_ld ? "LD" : "SD");
  if (use_ld) {
    GetMergeTopologiesFromLDMap(merge_details);
  } else {
    GetMergeTopologiesFromSDMap(merge_details);
  }
}

void SDMapTopologyExtractor::SetBevLaneMergeTopoUseMap(const std::vector<MergeDetail> &sd_merge_details, BevMapInfo &bev_map) {
  const bool use_ld = is_on_highway_;
  SD_MERGE_LOG << fmt::format("[Topo] Apply merges to BEV via {} map (input {} items)", use_ld ? "LD" : "SD", sd_merge_details.size());

  std::vector<uint64_t> alive_bev_ids;
  alive_bev_ids.reserve(bev_map.lane_infos.size());
  for (const auto &l : bev_map.lane_infos)
    alive_bev_ids.push_back(l.id);
  ResetHysteresisOnSceneChange(alive_bev_ids);

  if (is_on_highway_) {
    SetBevLaneMergeTopoUseLDMap(sd_merge_details, bev_map);
  } else {
    SetBevLaneMergeTopoUseSDMap(sd_merge_details, bev_map);
  }
}
bool SDMapTopologyExtractor::IsInLowPrecisionZone(const double merge_dist) {
    bool ret = false;
    MapEventPtr map_event_ptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr);
    if(nullptr != map_event_ptr)
    {
      bool has_low_precision_zone = false;
      double min_start_offset = 100000.0;
      double max_end_offset = -100000.0;
      for(const auto& oneOddInfo : map_event_ptr->odd_info)
      {
            if (LOW_PRECISION == oneOddInfo.value)
            {
                //Get the min distance of the low precision.
                has_low_precision_zone = true;
                if (min_start_offset > oneOddInfo.start_offset)
                {
                    min_start_offset = oneOddInfo.start_offset;
                }
                if (max_end_offset < oneOddInfo.end_offset)
                {
                    max_end_offset = oneOddInfo.end_offset;
                }
            }
      }
      if(true == has_low_precision_zone)
      {
        if(merge_dist > min_start_offset && merge_dist < max_end_offset)
        {
           ret = true;
        }
      }
    }
    return ret;
}
void SDMapTopologyExtractor::SetBevLaneMergeTopoNew(const RoutingMapPtr routing_map_ptr, BevMapInfo &bevMap) {
    
    std::unordered_set<uint64_t> alive_bev_ids;
    std::optional<Eigen::Isometry3d> T_local_ego_cur = GetTransform(bevMap.header.timestamp);
    if(T_local_ego_cur == std::nullopt) {
        return;
    }
    std::unordered_map<uint64_t, BevLaneInfo *> bev_lane_map;
    bev_lane_map.reserve(bevMap.lane_infos.size());
    for (auto &lane : bevMap.lane_infos) {
         alive_bev_ids.insert(lane.id);
         bev_lane_map[lane.id] = &lane;
    }
    for (auto it = bev_merge_trackers_.begin(); it != bev_merge_trackers_.end();) {
      if (!alive_bev_ids.count(it->first) || it->second.last_dist < 0)
      {
        it = bev_merge_trackers_.erase(it);
      }
      else
        ++it;
    }

    auto bev_matched_merge_infos = INTERNAL_PARAMS.geos_match_merge_data.GetMatchMergeInfo();
    CheckBevMergeTopo(routing_map_ptr, bevMap, bev_matched_merge_infos);
    for(auto &bev_lane : bevMap.lane_infos) {
          if(!bev_lane.geos || bev_lane.geos->size() < 3 || bev_lane.merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE || !bev_lane.next_lane_ids.empty()) {
             continue;
          }
          if(bev_matched_merge_infos.find(bev_lane.id) != bev_matched_merge_infos.end()) {
                    auto ld_merge = bev_matched_merge_infos.at(bev_lane.id);
                    if(ld_merge.distance_to_ego > 0 && !IsInLowPrecisionZone(ld_merge.distance_to_ego) && (ld_merge.type == 1 || ld_merge.type == 2)) {
                        MergeTrack merge_tmp;
                        merge_tmp.stable = (ld_merge.type == 1)? MergeTopoExtendType::TOPOLOGY_MERGE_LEFT : MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT;
                        if(bev_merge_trackers_.find(bev_lane.id) == bev_merge_trackers_.end()) {
                          merge_tmp.hits = 1;
                          merge_tmp.last_dist = ld_merge.distance_to_ego;
                          merge_tmp.last_T_ego = T_local_ego_cur.value();
                          for(auto& pt : *bev_lane.geos)
                              merge_tmp.last_geos->push_back(pt);
                          bev_merge_trackers_.insert({bev_lane.id, merge_tmp});
                        }
                        else if(merge_tmp.stable == bev_merge_trackers_[bev_lane.id].stable) {
                          bev_merge_trackers_[bev_lane.id].hits += 1 ;
                          bev_merge_trackers_[bev_lane.id].last_dist = ld_merge.distance_to_ego;
                          bev_merge_trackers_[bev_lane.id].last_T_ego = T_local_ego_cur.value();
                          for(auto& pt : *bev_lane.geos)
                              bev_merge_trackers_[bev_lane.id].last_geos->push_back(pt);
                        }
                        bev_lane.merge_info_extend.merge_valid  = 1;
                        bev_lane.merge_topo_extend = bev_merge_trackers_[bev_lane.id].stable;
                        bev_lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_LD;
                        bev_lane.merge_info_extend.dis_to_merge = bev_merge_trackers_[bev_lane.id].last_dist;
                    }
          }
          else {
                      //感知lane未匹配,Merge匹配超过10帧，一直保持
                      if(bev_merge_trackers_.find(bev_lane.id) != bev_merge_trackers_.end() && bev_merge_trackers_[bev_lane.id].hits >= 10 && bev_merge_trackers_[bev_lane.id].last_dist > 0) {
                         double dist =  LaneGeometry::GetDistanceBetweenLines(*bev_merge_trackers_[bev_lane.id].last_geos, *bev_lane.geos);
                         double os = std::max(bev_merge_trackers_[bev_lane.id].last_geos->front().x(),bev_lane.geos->front().x());
                         double oe = std::min(bev_merge_trackers_[bev_lane.id].last_geos->back().x(),bev_lane.geos->back().x());
                         double len = bev_merge_trackers_[bev_lane.id].last_geos->back().x() - bev_merge_trackers_[bev_lane.id].last_geos->front().x();
                        if(dist < 1.0 && (oe - os) > 0.3 * len) {
                            bev_lane.merge_info_extend.merge_valid  = 1;
                            bev_lane.merge_topo_extend = bev_merge_trackers_[bev_lane.id].stable;
                            bev_lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_LD;

                            double delta_dist = bev_merge_trackers_[bev_lane.id].last_T_ego(0,3) - T_local_ego_cur.value()(0,3);
                            bev_lane.merge_info_extend.dis_to_merge = bev_merge_trackers_[bev_lane.id].last_dist - delta_dist;
                            bev_merge_trackers_[bev_lane.id].last_dist = bev_lane.merge_info_extend.dis_to_merge;
                            bev_merge_trackers_[bev_lane.id].last_T_ego = T_local_ego_cur.value();
                        }
                    }
                    else {
                        bev_lane.merge_info_extend.merge_valid  = 0;
                        bev_lane.merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
                        bev_lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_UNKNOWN;
                        bev_lane.merge_info_extend.dis_to_merge = 0.0;
                    }
          }
          if(bev_lane.merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE && (bev_lane.merge_info_extend.dis_to_merge < 0.0 || bev_lane.merge_info_extend.dis_to_merge > 500.0)) {
              bev_lane.merge_info_extend.merge_valid  = 0;
              bev_lane.merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
              bev_lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_UNKNOWN;
              bev_lane.merge_info_extend.dis_to_merge = 0.0;
          }
    }
}
void SDMapTopologyExtractor::CheckBevMergeTopo(const RoutingMapPtr routing_map_ptr, BevMapInfo &bevMap, std::map<uint64_t, MergeTopoInfo>& bev_matched_merge_infos) {
    if(!routing_map_ptr) {
       return;
    }
    auto& match_infos = INTERNAL_PARAMS.ld_match_info_data.GetMatchInfo();
    std::map<uint64_t, LaneInfo*> ld_lanes_map;
	  std::map<uint64_t, BevLaneInfo*> bev_lanes_map;
    for(auto& ld_lane : routing_map_ptr->lanes) {
	     ld_lanes_map.insert({ld_lane.id, &ld_lane});
	  }
	  for(auto& bev_lane : bevMap.lane_infos) {
	     bev_lanes_map.insert({bev_lane.id, &bev_lane});
	  }
    for(auto it = bev_matched_merge_infos.begin(); it != bev_matched_merge_infos.end();) {
       if(bev_lanes_map.find(it->first) != bev_lanes_map.end() && bev_lanes_map[it->first]->position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
          bool match_valid = true;
           for(auto ld_id : match_infos.at(it->first)) {
		           if(ld_lanes_map.find(ld_id) != ld_lanes_map.end() && (ld_lanes_map[ld_id]->type == cem::message::env_model::LaneType::LANE_HARBOR_STOP || 
                  ld_lanes_map[ld_id]->type == cem::message::env_model::LaneType::LANE_BUS_NORMAL)) {
                   match_valid = false;
               }
		       }
           if(!match_valid) {
               it = bev_matched_merge_infos.erase(it);
              //  AINFO<<"erase ego lane merge:"<<it->first;
           }
           else{
               ++it;
           }
       }
       else if(bev_lanes_map.find(it->first) != bev_lanes_map.end() && bev_lanes_map[it->first]->previous_lane_ids.size() == 2 && (
               bev_lanes_map.find(bev_lanes_map[it->first]->previous_lane_ids[0]) != bev_lanes_map.end() || 
               bev_lanes_map.find(bev_lanes_map[it->first]->previous_lane_ids[1]) != bev_lanes_map.end())) {
               it = bev_matched_merge_infos.erase(it);
              //  AINFO<<"erase merge main lane merge:"<<it->first;
       }
       else {
          ++it;
       }
    }
}
std::optional<Eigen::Isometry3d> SDMapTopologyExtractor::GetTransform(const double& timestamp) {
  LocalizationPtr odom_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05,
                                                      odom_ptr);

  if (odom_ptr == nullptr) {
    return std::nullopt;
  }

  Eigen::Isometry3d T_local_ego = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd R_local_ego((odom_ptr->attitude_dr) * M_PI / 180.0,
                                Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d t(odom_ptr->posne_dr.at(0), odom_ptr->posne_dr.at(1), 0.0);
  T_local_ego.rotate(R_local_ego);
  T_local_ego.pretranslate(t);

  return T_local_ego;
}
void SDMapTopologyExtractor::GetMergeTopologiesFromSDMap(std::vector<MergeDetail> &merge_details) {
  merge_details.clear();
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    SD_MERGE_LOG << fmt::format("sd_route is null");
    return;
  }
  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    SD_MERGE_LOG << fmt::format("mpp_sections is empty");
    return;
  }

  // 计算每个 section 的全局 s 起点
  std::vector<double> section_start_s(mpp_sections.size(), 0.0);
  double              cumulative_s = 0.0;
  for (size_t i = 0; i < mpp_sections.size(); ++i) {
    section_start_s[i] = cumulative_s;
    cumulative_s += mpp_sections[i].length;
  }

  // 获取自车位置
  uint64_t current_section_id  = sd_route->navi_start.section_id;
  size_t   current_section_idx = static_cast<size_t>(-1);
  for (size_t i = 0; i < mpp_sections.size(); ++i) {
    if (mpp_sections[i].id == current_section_id) {
      current_section_idx = i;
      break;
    }
  }
  if (current_section_idx == static_cast<size_t>(-1)) {
    SD_MERGE_LOG << fmt::format("current section not found");
    return;
  }
  double current_s_offset = sd_route->navi_start.s_offset;
  double current_global_s = section_start_s[current_section_idx] + current_s_offset;

  // SD_MERGE_LOG << fmt::format("current_global_s = {}", current_global_s);
  // SD_COARSE_MATCH_TYPE2_LOG << fmt::format("current_global_s = {}", current_global_s);

  // 计算扫描范围
  double min_s = current_global_s + merge_rear_distance_;
  double max_s = current_global_s + merge_front_distance_;

  struct LaneLoc {
    const LaneInfo *lane{nullptr};
    size_t          sec_idx{static_cast<size_t>(-1)};
    uint64_t        section_id{0};
    uint64_t        lanegroup_id{0};
    int             lane_index{-1};
    bool            valid() const { return lane && sec_idx != static_cast<size_t>(-1); }
  };

  auto find_lane_loc = [&](uint64_t lane_id) -> LaneLoc {
    for (size_t si = 0; si < mpp_sections.size(); ++si) {
      const auto &sec      = mpp_sections[si];
      const auto *sec_full = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(sec.id);
      if (!sec_full)
        continue;
      for (const auto &lg_idx : sec_full->lane_group_idx) {
        const auto *lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
        if (!lg)
          continue;
        for (size_t li = 0; li < lg->lane_info.size(); ++li) {
          const auto &ln = lg->lane_info[li];
          if (ln.id == lane_id) {
            LaneLoc loc;
            loc.lane         = &ln;
            loc.sec_idx      = si;
            loc.section_id   = sec.id;
            loc.lanegroup_id = lg_idx.id;
            loc.lane_index   = static_cast<int>(li);
            return loc;
          }
        }
      }
    }
    return LaneLoc{};
  };

  // SD_MERGE_LOG << fmt::format("scanning range [{}, {}]", min_s, max_s);

  // 临时存储所有 merge 拓扑
  std::vector<MergeDetail> temp_merge_details;

  // 遍历所有 section
  for (size_t sec_idx = 0; sec_idx < mpp_sections.size(); ++sec_idx) {
    const auto &section                = mpp_sections[sec_idx];
    double      section_start_global_s = section_start_s[sec_idx];
    double      section_end_global_s   = section_start_global_s + section.length;

    if (section_end_global_s < min_s || section_start_global_s > max_s) {
      // SD_MERGE_LOG << fmt::format("Section out of range, skipped.");
      continue;
    }

    // SD_COARSE_MATCH_TYPE2_LOG << fmt::format("Processing section ID: {}, s_range: [{}, {}]", section.id,
    //                                    section_start_global_s - current_global_s, section_end_global_s - current_global_s);

    // 遍历 section 内的 lane group
    for (const auto &lg_idx : section.lane_group_idx) {
      double lg_start_global_s = section_start_global_s + lg_idx.start_range_offset;
      double lg_end_global_s   = section_start_global_s + lg_idx.end_range_offset;

      if (lg_end_global_s < min_s || lg_start_global_s > max_s) {
        // SD_MERGE_LOG << fmt::format("    LaneGroup out of range, skipped.");
        continue;
      }

      // SD_COARSE_MATCH_TYPE2_LOG << fmt::format("  LaneGroup ID: {}, s_range: [{}, {}]", lg_idx.id, lg_start_global_s - current_global_s,
      //                                    lg_end_global_s - current_global_s);

      const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
      if (!lane_group) {
        SD_MERGE_LOG << fmt::format("    LaneGroup not found.");
        continue;
      }

      // 遍历 lane group 内的车道
      for (size_t lane_idx = 0; lane_idx < lane_group->lane_info.size(); ++lane_idx) {
        const auto &lane = lane_group->lane_info[lane_idx];
        if (lane.merge_topology != MergeTopology::TOPOLOGY_MERGE_NONE) {
          double merge_start_s = lg_start_global_s - current_global_s;
          double merge_end_s   = lg_end_global_s - current_global_s;

          if (merge_end_s < merge_rear_distance_ || merge_start_s > merge_front_distance_) {
            // SD_MERGE_LOG << fmt::format("    Merge out of range: range=[{}, {}]", merge_start_s, merge_end_s);
            continue;
          }

          // 选择第一个后继车道作为目标车道
          if (!lane.next_lane_ids.empty()) {
            uint64_t    dst_id = lane.next_lane_ids.front();  // 只选择第一个后继车道
            MergeDetail detail;
            detail.dst_lane         = {dst_id, section.id, lg_idx.id, -1};  // 目标车道只有一个
            detail.merge_lane_range = {merge_start_s, merge_end_s};
            detail.type             = lane.merge_topology;
            detail.merge_distance   = merge_end_s;
            detail.src_lanes.push_back({lane.id, section.id, lg_idx.id, static_cast<int>(lane_idx)});  // 源车道只有一个
            temp_merge_details.push_back(detail);
          }
        }
      }
    }
  }

  // 合并连续的 merge 拓扑，按 lane_id 和 lanegroup_id 聚合
  std::map<std::pair<uint64_t, uint64_t>, MergeDetail> merged_details;
  for (const auto &detail : temp_merge_details) {
    uint64_t lane_id      = detail.src_lanes[0].lane_id;
    uint64_t lanegroup_id = detail.src_lanes[0].lanegroup_id;
    auto     key          = std::make_pair(lane_id, lanegroup_id);
    auto     it           = merged_details.find(key);
    if (it != merged_details.end()) {
      // 如果已存在相同 lane_id 和 lanegroup_id 的 merge，合并 range
      it->second.merge_lane_range.first  = std::min(it->second.merge_lane_range.first, detail.merge_lane_range.first);
      it->second.merge_lane_range.second = std::max(it->second.merge_lane_range.second, detail.merge_lane_range.second);
      it->second.merge_distance          = it->second.merge_lane_range.second;
      // SD_MERGE_LOG << fmt::format("Merged into lane_id={}, lanegroup_id={}, new range=[{}, {}]", lane_id, lanegroup_id,
      //                             it->second.merge_lane_range.first, it->second.merge_lane_range.second);

    } else {
      // 如果不存在，添加新的 merge 拓扑
      merged_details[key] = detail;
      // SD_MERGE_LOG << fmt::format("New merge added: lane_id={}, lanegroup_id={}, range=[{}, {}]", lane_id, lanegroup_id,
      //                             detail.merge_lane_range.first, detail.merge_lane_range.second);
    }
  }

  auto backtrack_keep_topo_start_sd = [&](uint64_t lane_id) -> std::pair<double, LaneDetail> {
    // 起点：merge 源车道
    LaneLoc cur = find_lane_loc(lane_id);
    if (!cur.valid()) {
      SD_MERGE_LOG << fmt::format("[SD] backtrack: src lane {} not found on route", lane_id);
      return {0.0, LaneDetail{lane_id, 0, 0, -1}};
    }

    std::unordered_set<uint64_t> visited;
    visited.reserve(64);

    double     keep_start_global = section_start_s[cur.sec_idx];
    LaneDetail tail{cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};

    while (cur.valid() && !visited.count(cur.lane->id)) {
      visited.insert(cur.lane->id);

      const double cur_start = section_start_s[cur.sec_idx];

      uint64_t    chosen_pred_id = 0;
      const auto &prevs          = cur.lane->previous_lane_ids;
      if (prevs.size() == 1) {
        chosen_pred_id = prevs.front();
      } else if (prevs.size() == 2) {
        const auto *p0       = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(prevs[0]);
        const auto *p1       = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(prevs[1]);
        const auto  is_uturn = [](const LaneInfo *ln) {
          return (ln && ln->type == LaneType::LANE_U_TURN_LANE);
        };
        const bool u0 = is_uturn(p0);
        const bool u1 = is_uturn(p1);
        if (u0 ^ u1) {
          chosen_pred_id = u0 ? prevs[1] : prevs[0];
          SD_MERGE_LOG << fmt::format("[SD] prev_count=2 with one U-turn, bypass U-turn: keep-topo continue with pred {} (lane {})",
                                      chosen_pred_id, cur.lane->id);
        }
      }

      if (chosen_pred_id == 0) {
        keep_start_global = cur_start;
        tail              = {cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};
        SD_MERGE_LOG << fmt::format("[SD] keep-topo stop by cond(1): lane {} prev_count={}", cur.lane->id, prevs.size());
        break;
      }

      LaneLoc pred = find_lane_loc(chosen_pred_id);
      if (!pred.valid()) {
        keep_start_global = cur_start;
        tail              = {cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};
        SD_MERGE_LOG << fmt::format("[SD] keep-topo stop by cond(1b): pred {} not on route", chosen_pred_id);
        break;
      }

      const double pred_start = section_start_s[pred.sec_idx];

      // (1c) 方向校验：pred 必须更靠自车
      if (pred_start > cur_start) {
        keep_start_global = cur_start;
        tail              = {cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};
        SD_MERGE_LOG << fmt::format("[SD] keep-topo stop by cond(1c): pred_start({:.2f}) >= cur_start({:.2f})", pred_start, cur_start);
        break;
      }

      // 条件 (2)：前继的后继数量 != 1 在当前车道起点处停止
      if (pred.lane->next_lane_ids.size() != 1) {
        keep_start_global = cur_start;
        tail              = {cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};
        SD_MERGE_LOG << fmt::format("[SD] keep-topo stop by cond(2): pred {} succ_count={}", pred.lane->id,
                                    pred.lane->next_lane_ids.size());
        break;
      }

      // 在自车脚下停止
      if (pred_start <= current_global_s) {
        keep_start_global = current_global_s;
        tail              = {pred.lane->id, pred.section_id, pred.lanegroup_id, pred.lane_index};
        SD_MERGE_LOG << fmt::format("[SD] keep-topo stop by cond(3): pred_start({:.2f}) <= ego_s({:.2f})", pred_start, current_global_s);
        break;
      }
      cur               = pred;
      keep_start_global = pred_start;
      tail              = {cur.lane->id, cur.section_id, cur.lanegroup_id, cur.lane_index};
    }

    double keep_start_rel = keep_start_global - current_global_s;
    if (keep_start_rel < 0.0)
      keep_start_rel = 0.0;
    return {keep_start_rel, tail};
  };

  for (auto &kv : merged_details) {
    auto md = kv.second;
    if (md.merge_distance < 0.0 || md.merge_distance > 500.0)
      continue;
    const uint64_t src_lane_id = md.src_lanes[0].lane_id;

    auto [keep_start_rel, tail_lane] = backtrack_keep_topo_start_sd(src_lane_id);

    double keep_end_rel = md.merge_lane_range.second;
    if (keep_start_rel > keep_end_rel)
      keep_start_rel = keep_end_rel;

    md.merge_lane_keeptopo_range = {keep_start_rel, keep_end_rel};
    md.keeptopo_tail_lane        = tail_lane;

    SD_MERGE_LOG << fmt::format("[SD] merge lane {} sec {}: keep-topo range=[{:.2f}, {:.2f}]m, tail(lane={}, sec={}, lg={}, idx={})",
                                src_lane_id, md.src_lanes[0].section_id, md.merge_lane_keeptopo_range.first,
                                md.merge_lane_keeptopo_range.second, md.keeptopo_tail_lane.lane_id, md.keeptopo_tail_lane.section_id,
                                md.keeptopo_tail_lane.lanegroup_id, md.keeptopo_tail_lane.lane_index);

    merge_details.push_back(std::move(md));
  }
}

void SDMapTopologyExtractor::GetMergeTopologiesFromLDMap(std::vector<MergeDetail> &merge_details) {
  merge_details.clear();

  auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route) {
    SD_MERGE_LOG << "LD route is null";
    return;
  }
  if (ld_route->sections.empty()) {
    SD_MERGE_LOG << "LD route sections is empty";
    return;
  }

  const double current_global_s = GetEgoGlobalS(*ld_route);

  const double min_s = current_global_s - merge_rear_distance_;
  const double max_s = current_global_s + merge_front_distance_;

  // 遍历主路径 sections
  for (const auto &section : ld_route->sections) {
    const double section_start_global_s = GetSectionStartS(section, *ld_route);
    const double section_end_global_s   = section_start_global_s + section.length;

    if (section_end_global_s < min_s || section_start_global_s > max_s) {
      continue;
    }

    for (const auto &pair : INTERNAL_PARAMS.ld_map_data.GetLDLanes()) {
      const auto *lane = pair.second;
      if (!lane || lane->section_id != section.id)
        continue;

      if (lane->merge_topology == MergeTopology::TOPOLOGY_MERGE_NONE)
        continue;

      const double merge_start_s = section_start_global_s - current_global_s;
      const double merge_end_s   = section_end_global_s - current_global_s;
      if (merge_end_s < 0.0 || merge_end_s > 500.0)
        continue;

      MergeDetail detail;
      uint64_t    dst_lane_id = lane->next_lane_ids.empty() ? lane->id : lane->next_lane_ids.front();

      // LD 没有 lanegroup 概念，这里置 0/-1 作为占位
      detail.dst_lane         = {dst_lane_id, section.id, /*lanegroup_id*/ 0, /*lane_index*/ -1};
      detail.merge_lane_range = {merge_start_s, merge_end_s};
      detail.type             = lane->merge_topology;
      detail.merge_distance   = merge_end_s;

      // 源车道
      detail.src_lanes.clear();
      detail.src_lanes.push_back({lane->id, section.id, /*lanegroup_id*/ 0, /*lane_index*/ -1});

      {
        auto [keep_start_rel, tail_lane] = LD_BacktrackKeepTopoStart(lane->id);
        double keep_end_rel              = merge_end_s;
        if (keep_start_rel > merge_end_s) {
          SD_MERGE_LOG << fmt::format("[LD] keep_start_rel {:.3f} > merge_end_s {:.3f}, clamp.", keep_start_rel, merge_end_s);
          keep_start_rel = merge_end_s;
        }
        detail.merge_lane_keeptopo_range = {keep_start_rel, keep_end_rel};
        detail.keeptopo_tail_lane        = tail_lane;

        // SD_MERGE_LOG << fmt::format("[LD] merge lane {} sec {}: keep-topo range = [{:.2f}, {:.2f}] m, tail(lane={}, sec={})", lane->id,
        //                             section.id, keep_start_rel, keep_end_rel, detail.keeptopo_tail_lane.lane_id,
        //                             detail.keeptopo_tail_lane.section_id);
      }

      merge_details.push_back(detail);
    }
  }
}

void SDMapTopologyExtractor::SetBevLaneMergeTopoUseSDMap(const std::vector<MergeDetail> &sd_merge_details, BevMapInfo &bev_map) {
  bool        is_on_highway = true;
  MapEventPtr map_event_ptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr);
  if (map_event_ptr && map_event_ptr->road_type == RT_CITYWAY) {
    SD_MERGE_LOG << fmt::format("Map_event_topic logic: On city way.");
    is_on_highway = false;
    // return;
  }

  RoutingMapPtr routing_map_raw{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(routing_map_raw);
  if (routing_map_raw && !routing_map_raw->is_on_highway) {
    is_on_highway = false;
    SD_MERGE_LOG << fmt::format("RoutingMap_topic logic: On city way.");
    // return;
  }

  auto &bev_lanes = bev_map.lane_infos;
  if (sd_merge_details.empty()) {
    SD_MERGE_LOG << fmt::format("No merge topologies available.");
    return;
  }

  // 选择车前最近的 MergeDetail
  const MergeDetail *closest_merge = nullptr;
  double             min_distance  = std::numeric_limits<double>::max();
  for (const auto &merge : sd_merge_details) {
    double merge_distance = merge.merge_distance;
    if (merge_distance < 0)
      continue;
    if (merge.merge_distance < min_distance) {
      min_distance  = merge.merge_distance;
      closest_merge = &merge;
    }
  }
  if (!closest_merge) {
    SD_MERGE_LOG << fmt::format("No valid merge topologies found.");
    return;
  }

  if (closest_merge->src_lanes.empty()) {
    SD_MERGE_LOG << fmt::format("Empty src_lanes in closest_merge.");
    return;
  }

  // 检查 src_lane 的车道类型，排除 LANE_HARBOR_STOP
  uint64_t    src_lane_id = closest_merge->src_lanes[0].lane_id;
  const auto *src_lane    = INTERNAL_PARAMS.sd_map_data.GetSDLaneInfoById(src_lane_id);
  if (src_lane && src_lane->type == LaneType::LANE_HARBOR_STOP) {
    SD_MERGE_LOG << fmt::format("Source lane ID: {} is LANE_HARBOR_STOP, skipping merge assignment.", src_lane_id);
    return;
  }

  // 构建感知车道线映射
  std::unordered_map<uint64_t, BevLaneInfo *> bev_lane_map;
  for (auto &lane : bev_lanes) {
    bev_lane_map[lane.id] = &lane;
  }

  //从bev预处理子模块获取排序好的当前section车道列表
  std::vector<uint64_t> sorted_bev_lane_ids = INTERNAL_PARAMS.navigation_info_data.get_bev_root_section_ids();
  SD_MERGE_LOG << fmt::format("Sorted BEV lane IDs (no next lanes): {}", fmt::join(sorted_bev_lane_ids, ", "));

  SD_MERGE_LOG << fmt::format("Useing SDMap merge_source");
  // 获取 lanegroup 的车道数和 lane_index
  uint64_t    lanegroup_id = closest_merge->keeptopo_tail_lane.lanegroup_id;
  const auto *lane_group   = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lanegroup_id);
  if (!lane_group) {
    SD_MERGE_LOG << fmt::format("LaneGroup not found for ID: {}", lanegroup_id);
    return;
  }

  // 过滤掉 LANE_DIVERSION && LANE_NON_MOTOR 类型的车道线
  std::vector<const LaneInfo *> normal_lanes;
  for (const auto &lane_info : lane_group->lane_info) {
    if (lane_info.type != LaneType::LANE_DIVERSION && lane_info.type != LaneType::LANE_NON_MOTOR) {
      normal_lanes.push_back(&lane_info);
    }
  }
  int sd_normal_lane_count = normal_lanes.size();

  // 找到 merge 源车道线在非导流区车道线中的索引
  int sd_normal_lane_index = -1;
  for (size_t i = 0; i < normal_lanes.size(); ++i) {
    if (normal_lanes[i]->id == closest_merge->keeptopo_tail_lane.lane_id) {
      sd_normal_lane_index = i;
      break;
    }
  }
  if (sd_normal_lane_index == -1) {
    SD_MERGE_LOG << fmt::format("Merge source lane ID: {} not found in normal lanes.", closest_merge->src_lanes[0].lane_id);
    return;
  }

  int bev_lane_count = sorted_bev_lane_ids.size();
  if (bev_lane_count == 0) {
    SD_MERGE_LOG << fmt::format("No valid candidate BEV lanes available.");
    return;
  }

  // 城区场景下，限定 sd_normal_lane_count == bev_lane_count
  if (!is_on_highway && sd_normal_lane_count != bev_lane_count) {
    SD_MERGE_LOG << fmt::format("City logic: lane count mismatch (SD:{} vs BEV:{}), skipping merge assignment.", sd_normal_lane_count,
                                bev_lane_count);
    return;
  }

  SD_MERGE_LOG << fmt::format("sd_lane_count: {}, sd_normal_lane_index: {}, bev_lane_count: {}", sd_normal_lane_count, sd_normal_lane_index,
                              bev_lane_count);

  //目前需要接触城区merge车道变侧的限制，后续看效果
  // if (!is_on_highway && (sd_normal_lane_index != sd_normal_lane_count - 1) && (sd_normal_lane_index != 0)) {
  //   SD_MERGE_LOG << fmt::format("On cityway, but merge line in SDMap is not side.");
  //   return;
  // }

  if (is_on_highway && sd_normal_lane_count != bev_lane_count) {
    SD_MERGE_LOG << fmt::format("Highway logic:: lane count mismatch (SD:{} vs BEV:{}), skipping merge assignment.", sd_normal_lane_count,
                                bev_lane_count);
    return;
  }
  // if (is_on_highway && (sd_normal_lane_count - bev_lane_count >= 2)) {
  //   SD_MERGE_LOG << fmt::format("Highway logic: highway lane count mismatch (SD:{} vs BEV:{})", sd_normal_lane_count, bev_lane_count);
  //   return;
  // }

  // 计算匹配的感知车道线位置
  int bev_position = -1;
  if ((sd_normal_lane_count == bev_lane_count)) {
    bev_position = sd_normal_lane_index;
  } else {
    if (closest_merge->type == MergeTopology::TOPOLOGY_MERGE_LEFT) {
      int sd_position_from_right = sd_normal_lane_count - 1 - sd_normal_lane_index;
      bev_position               = bev_lane_count - 1 - sd_position_from_right;
    } else if (closest_merge->type == MergeTopology::TOPOLOGY_MERGE_RIGHT) {
      bev_position = sd_normal_lane_index;
    }
  }

  // 验证并赋值拓扑信息
  if (bev_position >= 0 && bev_position < bev_lane_count) {
    uint64_t bev_id = sorted_bev_lane_ids[bev_position];
    auto     iter   = bev_lane_map.find(bev_id);
    if (iter != bev_lane_map.end()) {
      BevLaneInfo *lane = iter->second;
      if (lane && lane->geos && !lane->geos->empty()) {
        if (lane->merge_topo_extend == MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
          double bev_start_x = lane->geos->front().x();
          double bev_end_x   = lane->geos->back().x();
          double merge_start = closest_merge->merge_lane_keeptopo_range.first;
          double merge_end   = closest_merge->merge_lane_keeptopo_range.second;

          // 计算重叠长度
          double       overlap_start     = std::max(bev_start_x, merge_start);
          double       overlap_end       = std::min(bev_end_x, merge_end);
          double       overlap_length    = overlap_end - overlap_start;
          const double overlap_threshold = 0.2 * (bev_end_x - bev_start_x);

          if (overlap_length > overlap_threshold) {
            lane->merge_topo_extend              = static_cast<MergeTopoExtendType>(closest_merge->type);
            lane->merge_info_extend.merge_valid  = 1;
            lane->merge_info_extend.merge_source = MergeSourceExtend::MERGE_SD;
            lane->merge_info_extend.dis_to_merge = closest_merge->merge_distance;

            SD_MERGE_LOG << fmt::format("SDMAP Assigned merge to BEV lane ID: {}, Type: {}, Distance: {}, Overlap: {:.3f}m", bev_id,
                                        StrMergeTopology(closest_merge->type), lane->merge_info_extend.dis_to_merge, overlap_length);
          } else {
            SD_MERGE_LOG << fmt::format(
                "BEV lane ID: {} overlap with merge range ({:.3f}m) below threshold ({:.1f}m), skipping assignment.", bev_id,
                overlap_length, overlap_threshold);
          }
        } else {
          SD_MERGE_LOG << fmt::format("BEV lane ID: {} already has merge topology, skipping SDMap assignment.", bev_id);
        }
      }
    }
  }
}

void SDMapTopologyExtractor::SetMergeHysteresisParams(int promote, int demote, double first_promote_min_dist) {
  promote_hits_needed_    = std::max(1, promote);
  demote_miss_allowed_    = std::max(1, demote);
  first_promote_min_dist_ = std::max(0.0, first_promote_min_dist);
}

void SDMapTopologyExtractor::ResetHysteresisOnSceneChange(const std::vector<uint64_t> &current_bev_ids) {
  // 轻量收敛：只保留当前帧仍存在的 BEV id，避免 tracker 无限增长
  std::unordered_set<uint64_t> alive(current_bev_ids.begin(), current_bev_ids.end());
  for (auto it = bev_merge_trackers_.begin(); it != bev_merge_trackers_.end();) {
    if (!alive.count(it->first))
      it = bev_merge_trackers_.erase(it);
    else
      ++it;
  }
}

void SDMapTopologyExtractor::ReflectStableToBevLane(uint64_t bev_id, cem::message::sensor::BevLaneInfo *lane, const MergeTrack &trk,
                                                    MergeSourceExtend src) {
  if (!lane)
    return;

  double bev_start_x = lane->geos->front().x(), bev_end_x = lane->geos->back().x();
  if (bev_start_x > bev_end_x)
    std::swap(bev_start_x, bev_end_x);
  double bev_len = bev_end_x - bev_start_x;
  if ((bev_id == 1 || bev_id == 2) && (bev_len < 30.0)) {
    SD_MERGE_LOG << fmt::format("bev_id is {} and bev_len is {}, skipping LD assignment.", bev_id, bev_len);
    return;
  }

  if (trk.stable != MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
    lane->merge_topo_extend              = trk.stable;
    lane->merge_info_extend.merge_valid  = 1;
    lane->merge_info_extend.merge_source = src;
    lane->merge_info_extend.dis_to_merge = std::isfinite(trk.last_dist) ? trk.last_dist : 0.0;
  } else {
    lane->merge_topo_extend              = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
    lane->merge_info_extend.merge_valid  = 0;
    lane->merge_info_extend.merge_source = MergeSourceExtend::MERGE_UNKNOWN;
    lane->merge_info_extend.dis_to_merge = 0.0;
  }
}

void SDMapTopologyExtractor::SetBevLaneMergeTopoUseLDMap(const std::vector<MergeDetail> &ld_merge_details, BevMapInfo &bev_map) {
  auto &bev_lanes = bev_map.lane_infos;
  if (ld_merge_details.empty()) {
    SD_MERGE_LOG << "No merge topologies available (LD).";
    return;
  }
  if (bev_lanes.empty()) {
    SD_MERGE_LOG << "No BEV lanes available.";
    return;
  }

  const MergeDetail *closest_merge = nullptr;
  double             min_distance  = std::numeric_limits<double>::infinity();
  for (const auto &md : ld_merge_details) {
    if (md.merge_distance < 0)
      continue;  // 在自车后
    if (md.merge_distance < min_distance) {
      min_distance  = md.merge_distance;
      closest_merge = &md;
    }
  }
  if (!closest_merge) {
    SD_MERGE_LOG << "[LD] No forward merge details, skip.";
    return;
  }

  // src_lanes 可能为空：LD 版不强退，尝试用 dst_lane 退化映射
  uint64_t src_lane_id = 0;
  if (!closest_merge->src_lanes.empty()) {
    src_lane_id = closest_merge->src_lanes[0].lane_id;
  } else {
    src_lane_id = closest_merge->dst_lane.lane_id;
    SD_MERGE_LOG << fmt::format("LD: closest_merge->src_lanes empty, fallback use dst_lane as source index, lane_id={}", src_lane_id);
  }

  // 排除港湾停靠
  if (const auto *src_lane_ld = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(src_lane_id)) {
    if (src_lane_ld->type == LaneType::LANE_HARBOR_STOP) {
      SD_MERGE_LOG << fmt::format("Source lane ID: {} is LANE_HARBOR_STOP, skipping merge assignment (LD).", src_lane_id);
      return;
    }
  }

  std::vector<uint64_t> raw_bev_lane_ids;
  raw_bev_lane_ids.reserve(bev_lanes.size());

  std::unordered_map<uint64_t, BevLaneInfo *> bev_lane_map;
  bev_lane_map.reserve(bev_lanes.size());
  for (auto &lane : bev_lanes) {
    bev_lane_map[lane.id] = &lane;
    raw_bev_lane_ids.push_back(lane.id);
  }

  auto on_miss_update_last = [&]() {
    if (last_matched_bev_id_ == 0)
      return;
    auto it = bev_lane_map.find(last_matched_bev_id_);
    if (it == bev_lane_map.end())
      return;

    auto &trk  = bev_merge_trackers_[last_matched_bev_id_];
    trk.misses = std::min(trk.misses + 1, 1000);
    if (trk.stable != MergeTopoExtendType::TOPOLOGY_MERGE_NONE && trk.misses > demote_miss_allowed_) {
      trk.stable = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
      SD_MERGE_LOG << fmt::format("[HYS] bev={} DEMOTE by consecutive fail (misses>{})", last_matched_bev_id_, demote_miss_allowed_);
    }
    ReflectStableToBevLane(last_matched_bev_id_, it->second, trk, MergeSourceExtend::MERGE_LD);

  };

  std::vector<LineSort>                                                       line_sorts;
  std::map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> laneinfo_geo_map;

  for (auto &lane : bev_lanes) {
    if (lane.geos && !lane.geos->empty() && IsValidLaneForMergeAssignment(lane)) {
      // 与 SD 一致：过滤掉有 previous_lane_ids 的（仅保留“无上游”的主线）
      if (lane.previous_lane_ids.empty()) {
        std::vector<double> geo_x_vec, geo_y_vec;
        geo_x_vec.reserve(lane.geos->size());
        geo_y_vec.reserve(lane.geos->size());
        for (auto &pt : *lane.geos) {
          geo_x_vec.push_back(pt.x());
          geo_y_vec.push_back(pt.y());
        }
        if (geo_x_vec.size() < 5) {
          laneinfo_geo_map.insert({lane.id, {lane.geos, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 1)}});
        } else {
          laneinfo_geo_map.insert({lane.id, {lane.geos, LaneGeometry::PolynomialFitting(geo_x_vec, geo_y_vec, 3)}});
        }
        line_sorts.emplace_back(lane.id, false);
      } else {
        SD_MERGE_LOG << fmt::format("Lane ID: {} has pre_lane_ids, excluded from sorting.", lane.id);
      }
    }
  }

  if (line_sorts.size() >= 16) {
    SD_MERGE_LOG << fmt::format("line_sorts size: {} > 16.", line_sorts.size());
    AWARN << "Input lane size too much, exit from SetBevLaneMergeTopoUseLDMap";
    return;
  }
  // 左右排序（与 SD 一致：用 JudgeIsLeft 基于拟合/几何判定）
  std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
    auto &fit1 = laneinfo_geo_map[l1.id].second;
    auto &fit2 = laneinfo_geo_map[l2.id].second;
    auto &geo1 = laneinfo_geo_map[l1.id].first;
    auto &geo2 = laneinfo_geo_map[l2.id].first;
    if (!geo1 || !geo2 || geo1->empty() || geo2->empty()) {
      SD_MERGE_LOG << fmt::format("Error: Lane {} or Lane {} has no valid geometry.", l1.id, l2.id);
      return false;
    }
    return LaneGeometry::JudgeIsLeft(*geo1, *geo2, fit1, fit2);
  });

  std::vector<uint64_t> sorted_bev_lane_ids;
  sorted_bev_lane_ids.reserve(line_sorts.size());
  for (const auto &sort : line_sorts)
    sorted_bev_lane_ids.push_back(sort.id);
  SD_MERGE_LOG << fmt::format("Sorted BEV lane IDs (no next lanes): {}", fmt::join(sorted_bev_lane_ids, ", "));

  if (sorted_bev_lane_ids.empty()) {
    SD_MERGE_LOG << "No candidate BEV lanes after sorting.";
    return;
  }

  std::vector<Eigen::Vector2d> ld_target_pts;
  bool                         has_target = BuildLDTargetPolylineFromMerge(*closest_merge, ld_target_pts);
  if (!has_target || ld_target_pts.size() < 3) {
    SD_MERGE_LOG << "Build LD target polyline failed (or too short).";
    on_miss_update_last();
    return;
  }

  auto bev_points_getter = [&](uint64_t bev_id, std::vector<Eigen::Vector2d> &out) -> bool {
    auto it = bev_lane_map.find(bev_id);
    if (it == bev_lane_map.end())
      return false;
    BevLaneInfo *ln = it->second;
    if (!ln || !ln->geos || ln->geos->empty())
      return false;
    out.clear();
    out.reserve(ln->geos->size());
    for (const auto &p : *ln->geos)
      out.emplace_back(p.x(), p.y());
    return (out.size() >= 3);
  };

  uint64_t                            best_bev_id = 0;
  double                              best_score  = 0.0;
  cem::fusion::navigation::MatchStats best_stats;
  const double                        kSearchEps            = 0.5;
  const double                        kDefaultMeanLatThresh = 0.65;
  const double                        kHarborMeanLatThresh  = 1.0;
  const double                        kMinInlierRatio       = 0.5;
  double                              kMeanLatThresh        = kDefaultMeanLatThresh;

  // if (const auto *src_lane_ld = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(src_lane_id)) {
  //   if (src_lane_ld->type == LaneType::LANE_HARBOR_STOP) {
  //     SD_MERGE_LOG << fmt::format("Source lane ID: {} is LANE_HARBOR_STOP, relax kMeanLatThresh: 0.45 -> 1.0", src_lane_id);
  //     kMeanLatThresh = kHarborMeanLatThresh;
  //   }
  // }

  bool geom_hit   = cem::fusion::navigation::GeometryLaneMatcher::BestMatchBevByGeometry(raw_bev_lane_ids, bev_points_getter, ld_target_pts,
                                                                                         kSearchEps, kMeanLatThresh, kMinInlierRatio,
                                                                                         best_bev_id, best_score, &best_stats);
  bool overlap_ok = false;
  if (geom_hit && best_bev_id != 0) {
    auto itB = bev_lane_map.find(best_bev_id);
    if (itB != bev_lane_map.end() && itB->second && itB->second->geos && !itB->second->geos->empty()) {
      double bx0 = itB->second->geos->front().x();
      double bx1 = itB->second->geos->back().x();
      if (bx0 > bx1)
        std::swap(bx0, bx1);
      double mx0 = closest_merge->merge_lane_keeptopo_range.first;
      double mx1 = closest_merge->merge_lane_keeptopo_range.second;
      if (mx0 > mx1)
        std::swap(mx0, mx1);
      double       os = std::max(bx0, mx0), oe = std::min(bx1, mx1);
      double       overlap_len = oe - os;
      const double thr         = 0.3 * (bx1 - bx0);
      overlap_ok               = (overlap_len > thr);
      if (!overlap_ok) {
        SD_MERGE_LOG << fmt::format("best_bev={} overlap {:.2f} < thr {:.2f}, treat as miss.", best_bev_id, overlap_len, thr);
        // on_miss_update_last();
        // return;
      }
    }
  }

  if (geom_hit && overlap_ok && best_bev_id != 0) {
    auto itB = bev_lane_map.find(best_bev_id);
    if (itB == bev_lane_map.end() || !itB->second) {
      SD_MERGE_LOG << fmt::format("best_bev {} invalid pointer. keep-by-history.", best_bev_id);
      on_miss_update_last();
      return;
    }

    auto &trk     = bev_merge_trackers_[best_bev_id];
    trk.last_dist = closest_merge->merge_distance;

    const auto observed_stable = static_cast<MergeTopoExtendType>(closest_merge->type);

    if (trk.stable == MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
      // 首见抑制：距离过近不提升
      if (closest_merge->merge_distance < first_promote_min_dist_) {
        SD_MERGE_LOG << fmt::format("[HYS] bev={} first-merge suppressed by dist<{:.0f}m (obs={:.1f})", best_bev_id,
                                    first_promote_min_dist_, closest_merge->merge_distance);
        trk.hits   = 0;
        trk.misses = 0;
      } else {
        trk.hits   = std::min(trk.hits + 1, 1000);
        trk.misses = 0;
        if (trk.hits >= promote_hits_needed_) {
          trk.stable = observed_stable;
          SD_MERGE_LOG << fmt::format("[HYS] bev={} PROMOTE to {} (hits={})", best_bev_id, StrMergeTopoExtendType(trk.stable), trk.hits);
          trk.hits = 0;
        }
      }
    } else {
      trk.misses = 0;
      trk.hits   = std::min(trk.hits + 1, 1000);
      trk.stable = observed_stable;
    }

    // reflect_to_lane(best_bev_id);
    ReflectStableToBevLane(best_bev_id, itB->second, trk, MergeSourceExtend::MERGE_LD);

    // 若命中对象与上一帧不同，对上一帧进行一次 miss+1 并回写
    if (last_matched_bev_id_ != 0 && last_matched_bev_id_ != best_bev_id) {
      auto &prev  = bev_merge_trackers_[last_matched_bev_id_];
      prev.misses = std::min(prev.misses + 1, 1000);
      if (prev.stable != MergeTopoExtendType::TOPOLOGY_MERGE_NONE && prev.misses > demote_miss_allowed_) {
        SD_MERGE_LOG << fmt::format("[HYS] bev={} DEMOTE to NONE by switch (misses>{})", last_matched_bev_id_, demote_miss_allowed_);
        prev.stable = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        prev.hits   = 0;
        prev.misses = 0;
      }
      // reflect_to_lane(last_matched_bev_id_);
      ReflectStableToBevLane(last_matched_bev_id_, itB->second, trk, MergeSourceExtend::MERGE_LD);
    }

    last_matched_bev_id_ = best_bev_id;

    SD_MERGE_LOG << fmt::format("LDMAP Geometry-Assigned (HYS) → bev={} | type={} | dist={:.2f}m | mean_lat={:.3f} | inlier={:.2f}",
                                best_bev_id, StrMergeTopology(closest_merge->type), closest_merge->merge_distance, best_score,
                                best_stats.inlier_ratio);
    return;
  }

  SD_MERGE_LOG << "Geometry match MISS or invalid best; keep-by-history.";
  on_miss_update_last();
}

bool SDMapTopologyExtractor::BuildLDTargetPolylineFromMerge(const MergeDetail &md, std::vector<Eigen::Vector2d> &out_pts) const {
  out_pts.clear();
  const uint64_t tail_id = md.keeptopo_tail_lane.lane_id;
  const uint64_t src_id  = md.src_lanes.empty() ? md.dst_lane.lane_id : md.src_lanes[0].lane_id;
  if (tail_id == 0 || src_id == 0)
    return false;

  std::unordered_set<uint64_t> visited;
  visited.reserve(128);

  uint64_t             cur     = tail_id;
  int                  hops    = 0;
  static constexpr int kMaxHop = 256;

  while (cur && !visited.count(cur) && hops < kMaxHop) {
    visited.insert(cur);
    ++hops;

    const auto *ln = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(cur);
    if (!ln) {
      SD_MERGE_LOG << fmt::format("[LD] Build polyline: lane {} not found, stop.", cur);
      break;
    }

    if (!ln->points.empty()) {
      // out_pts.insert(out_pts.end(), ln->points.begin(), ln->points.end());
      out_pts.reserve(out_pts.size() + ln->points.size());
      for (const auto &p : ln->points) {
        out_pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y));
      }
    } else {
      SD_MERGE_LOG << fmt::format("[LD] Build polyline: lane {} has no center geometry.", cur);
    }

    if (cur == src_id) {
      break;
    }

    if (ln->next_lane_ids.size() != 1) {
      SD_MERGE_LOG << fmt::format("[LD] Build polyline stop: next_count={} at lane {}", ln->next_lane_ids.size(), cur);
      break;
    }
    cur = ln->next_lane_ids.front();
  }

  if (out_pts.size() < 6) {
    SD_MERGE_LOG << fmt::format("[LD] Build polyline too short: {} pts.", out_pts.size());
    return false;
  }

  if (out_pts.size() > 1000) {
    std::vector<Eigen::Vector2d> tmp;
    tmp.reserve(out_pts.size() / 2 + 1);
    for (size_t i = 0; i < out_pts.size(); i += 2)
      tmp.push_back(out_pts[i]);
    out_pts.swap(tmp);
  }
  return true;
}

void SDMapTopologyExtractor::SetBevLaneMergeTopoUseLDMapIndexBased(const MergeDetail                                 &closest_merge,
                                                                   const std::vector<uint64_t>                       &sorted_bev_lane_ids,
                                                                   const std::unordered_map<uint64_t, BevLaneInfo *> &bev_lane_map) {
  const uint64_t ld_tail_sec_id  = closest_merge.keeptopo_tail_lane.section_id;
  const auto    *ld_tail_sec     = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(ld_tail_sec_id);
  const uint64_t ld_tail_lane_id = closest_merge.keeptopo_tail_lane.lane_id;

  auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route || ld_route->sections.empty()) {
    SD_MERGE_LOG << "[LD] route missing in SetBevLaneMergeTopoUseLDMapIndexBased";
    return;
  }
  const uint64_t ego_sec = ld_route->navi_start.section_id;
  if (ego_sec != ld_tail_sec_id) {
    SD_MERGE_LOG << "[LD] ld_tail_sec_id is not equal to ego_sec";
    return;
  }

  // 过滤 LANE_DIVERSION
  std::vector<uint64_t> ld_normal_lane_ids;
  int                   ld_normal_lane_index = -1;
  if (ld_tail_sec) {
    for (uint64_t lid : ld_tail_sec->lane_ids) {
      const auto *ln = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lid);
      if (!ln)
        continue;
      if (ln->type == LaneType::LANE_DIVERSION)
        continue;
      ld_normal_lane_ids.push_back(lid);
    }
    for (size_t i = 0; i < ld_normal_lane_ids.size(); ++i) {
      if (ld_normal_lane_ids[i] == ld_tail_lane_id) {
        ld_normal_lane_index = static_cast<int>(i);
        break;
      }
    }
  }

  const int ld_n  = static_cast<int>(ld_normal_lane_ids.size());
  const int bev_n = static_cast<int>(sorted_bev_lane_ids.size());

  // ===== 新增：车道数不一致则直接跳过索引兜底 =====
  if (ld_n != bev_n) {
    SD_MERGE_LOG << fmt::format("Index fallback skipped: lane count mismatch (LD:{} vs BEV:{})", ld_n, bev_n);
    return;
  }

  if (bev_n == 0 || ld_normal_lane_index < 0) {
    SD_MERGE_LOG << "Index fallback aborted: bev_n==0 or cannot locate LD index.";
    return;
  }

  int bev_pos = -1;
  if (ld_n == bev_n) {
    bev_pos = ld_normal_lane_index;
  } else {
    if (closest_merge.type == MergeTopology::TOPOLOGY_MERGE_LEFT) {
      int ld_from_right = ld_n - 1 - ld_normal_lane_index;
      bev_pos           = bev_n - 1 - ld_from_right;
    } else if (closest_merge.type == MergeTopology::TOPOLOGY_MERGE_RIGHT) {
      bev_pos = ld_normal_lane_index;
    } else {
      // 比例兜底
      if (ld_n <= 1 || bev_n <= 1)
        bev_pos = std::min(std::max(ld_normal_lane_index, 0), bev_n - 1);
      else {
        double cont = double(ld_normal_lane_index) * double(bev_n - 1) / double(ld_n - 1);
        bev_pos     = (int)std::round(cont);
        bev_pos     = std::min(std::max(bev_pos, 0), bev_n - 1);
      }
    }
  }
  bev_pos = std::min(std::max(bev_pos, 0), bev_n - 1);

  uint64_t bev_id = sorted_bev_lane_ids[bev_pos];
  auto     it     = bev_lane_map.find(bev_id);
  if (it == bev_lane_map.end() || !it->second || !it->second->geos || it->second->geos->empty()) {
    SD_MERGE_LOG << "Index fallback: BEV lane invalid.";
    return;
  }

  BevLaneInfo *lane = it->second;
  if (lane->merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
    SD_MERGE_LOG << fmt::format("Index fallback: BEV {} already set, skip.", bev_id);
    return;
  }

  double bev_start_x = lane->geos->front().x(), bev_end_x = lane->geos->back().x();
  if (bev_start_x > bev_end_x)
    std::swap(bev_start_x, bev_end_x);
  double merge_start = closest_merge.merge_lane_keeptopo_range.first;
  double merge_end   = closest_merge.merge_lane_keeptopo_range.second;
  if (merge_start > merge_end)
    std::swap(merge_start, merge_end);

  double       overlap_start     = std::max(bev_start_x, merge_start);
  double       overlap_end       = std::min(bev_end_x, merge_end);
  double       overlap_len       = overlap_end - overlap_start;
  const double overlap_threshold = 0.5 * (bev_end_x - bev_start_x);

  if (overlap_len > overlap_threshold) {
    lane->merge_topo_extend              = static_cast<MergeTopoExtendType>(closest_merge.type);
    lane->merge_info_extend.merge_valid  = 1;
    lane->merge_info_extend.merge_source = MergeSourceExtend::MERGE_LD;
    lane->merge_info_extend.dis_to_merge = closest_merge.merge_distance;

    SD_MERGE_LOG << fmt::format("Index-Assigned merge → BEV {} | type={} | dist={:.2f}m | overlap={:.2f}m", bev_id,
                                StrMergeTopology(closest_merge.type), lane->merge_info_extend.dis_to_merge, overlap_len);
  } else {
    SD_MERGE_LOG << fmt::format("Index fallback overlap {:.2f} below thr {:.2f}, skip.", overlap_len, overlap_threshold);
  }
}

// 辅助函数：判断车道线是否为短线
bool SDMapTopologyExtractor::IsShortMessLane(const BevLaneInfo &lane) {
  if (lane.geos && lane.geos->size() > 1) {
    if (lane.previous_lane_ids.empty() && lane.next_lane_ids.empty()) {
      double start_x = lane.geos->front().x();
      double length  = fabs(lane.geos->back().x() - start_x);
      if (start_x > 80 && length < 30) {
        return true;
      }
    }
  }
  return false;
}

// 判断车道线是否有效（非短线且无后继）
bool SDMapTopologyExtractor::IsValidLaneForMergeAssignment(const BevLaneInfo &lane) {
  if (IsShortMessLane(lane)) {
    return false;  // 短线无效
  }
  // 检查是否有后继车道
  // for (const auto &next_id : lane.next_lane_ids) {
  //   auto next_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(next_id);
  //   if (next_lane) {
  //     return false;  // 有后继，无效
  //   }
  // }
  return true;  // 无后继，有效
}

void SDMapTopologyExtractor::PrintMergeDetails(const std::vector<MergeDetail> &merge_details) {
  if (merge_details.empty()) {
    SD_MERGE_LOG << "[Merge] (0 items)";
    return;
  }

  SD_MERGE_LOG << fmt::format("[Merge] {} item(s):", merge_details.size());

  auto fmt_lane = [](const LaneDetail &ld) {
    return fmt::format("section_id={},lane_id={},lanegroup_id={}, lane_index={}", ld.section_id, ld.lane_id, ld.lanegroup_id,
                       ld.lane_index);
  };

  for (size_t i = 0; i < merge_details.size(); ++i) {
    const auto &md = merge_details[i];

    SD_MERGE_LOG << fmt::format(
        " [Merge][{}] type={}, merge_distance={:.3f}, "
        "range=[{:.3f}, {:.3f}]",
        i, StrMergeTopology(md.type), md.merge_distance, md.merge_lane_range.first, md.merge_lane_range.second);

    SD_MERGE_LOG << fmt::format(" [Merge][{}] keep-topo range=[{:.3f}, {:.3f}]", i, md.merge_lane_keeptopo_range.first,
                                md.merge_lane_keeptopo_range.second);

    SD_MERGE_LOG << fmt::format(" [Merge][{}] Dst: {}", i, fmt_lane(md.dst_lane));

    if (md.src_lanes.empty()) {
      SD_MERGE_LOG << fmt::format(" [Merge][{}] Src: (empty)", i);
    } else {
      std::vector<std::string> src_strs;
      src_strs.reserve(md.src_lanes.size());
      for (const auto &s : md.src_lanes)
        src_strs.push_back(fmt_lane(s));
      SD_MERGE_LOG << fmt::format(" [Merge][{}] Src: {}", i, fmt::join(src_strs, " | "));
    }

    SD_MERGE_LOG << fmt::format(" [Merge][{}] keep-topo tail: {}", i, fmt_lane(md.keeptopo_tail_lane));
  }
}

uint64_t SDMapTopologyExtractor::GetCurrentLaneGroupId() const {
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  if (!sd_route) {
    AWARN << "[SDMapTopologyExtractor] sd_route is null";
    // SD_COARSE_MATCH_TYPE2_LOG << "sd_route is null";
    return 0;
  }
  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    AWARN << "[SDMapTopologyExtractor] mpp_sections is empty";
    // SD_COARSE_MATCH_TYPE2_LOG << "mpp_sections is empty";
    return 0;
  }
  uint64_t current_section_id = sd_route->navi_start.section_id;
  // SD_COARSE_MATCH_TYPE2_LOG << "current_section_id: " << current_section_id;
  const SDSectionInfo *current_section = nullptr;
  for (const auto &section : mpp_sections) {
    if (section.id == current_section_id) {
      current_section = &section;
      break;
    }
  }
  if (!current_section) {
    AWARN << "[SDMapTopologyExtractor] current section not found";
    return 0;
  }
  double current_s_offset = sd_route->navi_start.s_offset;
  // SD_COARSE_MATCH_TYPE2_LOG << "current_s_offset: " << current_s_offset;
  const double            kTol                   = 1.0;  // 1m 公差，按需调整
  const SDLaneGroupIndex *current_lane_group_idx = nullptr;
  for (const auto &lg_idx : current_section->lane_group_idx) {
    if (current_s_offset >= lg_idx.start_range_offset && current_s_offset < lg_idx.end_range_offset) {
      current_lane_group_idx = &lg_idx;
      break;
    }
  }

  if (!current_lane_group_idx) {
    const SDLaneGroupIndex *first_lg = nullptr;
    const SDLaneGroupIndex *last_lg  = nullptr;
    for (const auto &lg : current_section->lane_group_idx) {
      if (!first_lg || lg.start_range_offset < first_lg->start_range_offset)
        first_lg = &lg;
      if (!last_lg || lg.end_range_offset > last_lg->end_range_offset)
        last_lg = &lg;
    }

    if (current_s_offset <= (first_lg ? first_lg->start_range_offset - kTol : 0.0)) {
      current_lane_group_idx = first_lg;
      SD_COARSE_MATCH_TYPE2_LOG << "[SDMapTopologyExtractor] s before first lanegroup, clamp to FIRST id=" << (first_lg ? first_lg->id : 0);
    } else if (current_s_offset >= (current_section->length - kTol) || (last_lg && current_s_offset >= last_lg->end_range_offset - kTol)) {
      current_lane_group_idx = last_lg;
      SD_COARSE_MATCH_TYPE2_LOG << "[SDMapTopologyExtractor] s beyond section length, clamp to LAST lanegroup id="
                                << (last_lg ? last_lg->id : 0);
    } else {
      double                  best    = std::numeric_limits<double>::infinity();
      const SDLaneGroupIndex *nearest = nullptr;
      for (const auto &lg : current_section->lane_group_idx) {
        double d = 0.0;
        if (current_s_offset < lg.start_range_offset)
          d = lg.start_range_offset - current_s_offset;
        else
          d = current_s_offset - lg.end_range_offset;
        if (d < best) {
          best    = d;
          nearest = &lg;
        }
      }
      current_lane_group_idx = nearest;
      SD_COARSE_MATCH_TYPE2_LOG << "[SDMapTopologyExtractor] s in gap, pick NEAREST lanegroup id=" << (nearest ? nearest->id : 0);
    }
  }

  if (!current_lane_group_idx) {
    AWARN << "[SDMapTopologyExtractor] current lane group still not found (unexpected)";
    return 0;
  }
  uint64_t lane_group_id = current_lane_group_idx->id;
  // SD_COARSE_MATCH_TYPE2_LOG << "Exiting GetCurrentLaneGroupId with lane_group_id: " << lane_group_id;
  return lane_group_id;
}

std::vector<VirtualSection> SDMapTopologyExtractor::ExtractVirtualSections(const JunctionInfoCity &junction_info) {
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() == nullptr) {
    SD_ENV_INFO_LOG << fmt::format("SDRouteInfo is null, exiting");
    return {};
  }
  const auto &route_info   = *INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
  double      ego_global_s = GetEgoGlobalS(route_info);

  // SD_ENV_INFO_LOG << fmt::format("{}", route_info);

  std::set<uint64_t> mpp_lg_ids;
  std::set<uint64_t> mpp_section_ids;
  for (const auto &section : route_info.mpp_sections) {
    mpp_section_ids.insert(section.id);
    for (const auto &lg_idx : section.lane_group_idx) {
      mpp_lg_ids.insert(lg_idx.id);
    }
  }

  struct LgInfo {
    const SDLaneGroupInfo *lg;
    double                 start_s;
    double                 end_s;
    std::vector<int>       target_indices;
  };

  auto is_non_navi_type = [](LaneType t) {
    switch (t) {
      case LaneType::LANE_LEFT_WAIT:
      case LaneType::LANE_RIGHT_TURN_AREA:
      case LaneType::LANE_U_TURN_AREA:
      case LaneType::LANE_NO_TURN_AREA:
      case LaneType::LANE_VIRTUAL_COMMON:
      case LaneType::LANE_VIRTUAL_JUNCTION:
        return true;
      default:
        return false;
    }
  };
  auto is_pure_non_navi_lg = [&](const SDLaneGroupInfo *lg) {
    if (!lg || lg->lane_info.empty())
      return true;
    return std::all_of(lg->lane_info.begin(), lg->lane_info.end(), [&](const LaneInfo &ln) { return is_non_navi_type(ln.type); });
  };

  std::vector<LgInfo> lg_chain;

  // auto is_motor = [](const LaneInfo &ln) {
  //   return ln.type != LaneType::LANE_NON_MOTOR;
  // };
  auto is_motor = [](const LaneInfo &ln) {
    return (ln.type != LaneType::LANE_NON_MOTOR) && (ln.type != LaneType::LANE_DIVERSION);
  };
  auto motor_count = [&](const SDLaneGroupInfo *lg) -> int {
    if (!lg)
      return 0;
    int c = 0;
    for (const auto &ln : lg->lane_info)
      if (is_motor(ln))
        ++c;
    return c;
  };
  auto left_motor_idx = [&](const SDLaneGroupInfo *lg) -> int {
    if (!lg)
      return -1;
    for (int i = 0; i < static_cast<int>(lg->lane_info.size()); ++i)
      if (is_motor(lg->lane_info[i]))
        return i;
    return -1;
  };
  auto right_motor_idx = [&](const SDLaneGroupInfo *lg) -> int {
    if (!lg)
      return -1;
    for (int i = static_cast<int>(lg->lane_info.size()) - 1; i >= 0; --i)
      if (is_motor(lg->lane_info[i]))
        return i;
    return -1;
  };
  auto map_to_motor_indices = [&](const SDLaneGroupInfo *lg, const std::vector<int> &raw_indices) {
    std::vector<int> map_actual_to_motor;
    map_actual_to_motor.resize(lg ? lg->lane_info.size() : 0, -1);
    int k = 0;
    if (lg) {
      for (int i = 0; i < static_cast<int>(lg->lane_info.size()); ++i) {
        if (is_motor(lg->lane_info[i])) {
          map_actual_to_motor[i] = k++;
        }
      }
    }
    std::vector<int> out;
    out.reserve(raw_indices.size());
    for (int idx : raw_indices) {
      if (idx >= 0 && idx < static_cast<int>(map_actual_to_motor.size())) {
        int m = map_actual_to_motor[idx];
        if (m >= 0)
          out.push_back(m);  // 过滤掉非机动车索引
      }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  };

  auto is_one_to_one_connection = [&](const SDLaneGroupInfo *prev_lg, const SDLaneGroupInfo *curr_lg) -> bool {
    if (!prev_lg || !curr_lg)
      return false;

    const int prev_motor_num = motor_count(prev_lg);
    const int curr_motor_num = motor_count(curr_lg);

    if (prev_motor_num != curr_motor_num)
      return false;

    for (int i = 0; i < static_cast<int>(prev_lg->lane_info.size()); ++i) {
      if (!is_motor(prev_lg->lane_info[i]))
        continue;

      int motor_successors = 0;
      for (uint64_t next_id : prev_lg->lane_info[i].next_lane_ids) {
        for (int j = 0; j < static_cast<int>(curr_lg->lane_info.size()); ++j) {
          if (curr_lg->lane_info[j].id == next_id && is_motor(curr_lg->lane_info[j])) {
            motor_successors++;
            break;
          }
        }
      }

      if (motor_successors != 1)
        return false;
    }

    for (int i = 0; i < static_cast<int>(curr_lg->lane_info.size()); ++i) {
      if (!is_motor(curr_lg->lane_info[i]))
        continue;

      int motor_predecessors = 0;
      for (uint64_t prev_id : curr_lg->lane_info[i].previous_lane_ids) {
        for (int j = 0; j < static_cast<int>(prev_lg->lane_info.size()); ++j) {
          if (prev_lg->lane_info[j].id == prev_id && is_motor(prev_lg->lane_info[j])) {
            motor_predecessors++;
            break;
          }
        }
      }

      if (motor_predecessors != 1)
        return false;
    }

    return true;
  };

  constexpr double kMinEffectiveLgLen  = 1.0;  //后续根据数据可调，暂定1m
  uint64_t         junction_section_id = junction_info.junction_id;
  auto             junction_section    = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(junction_section_id);
  if (!junction_section) {
    SD_ENV_INFO_LOG << fmt::format("Junction section {} not found", junction_section_id);
    return {};
  }
  if (junction_section->lane_group_idx.empty()) {
    SD_ENV_INFO_LOG << fmt::format("Junction section {} not has no lane groups.", junction_section_id);
    for (uint64_t junction_pre_id : junction_section->predecessor_section_id_list) {
      if (mpp_section_ids.count(junction_pre_id)) {
        junction_section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(junction_pre_id);
        if (!junction_section->lane_group_idx.empty()) {
          SD_ENV_INFO_LOG << fmt::format("Seek Junction Pre_section {}.", junction_pre_id);
        } else {
          SD_ENV_INFO_LOG << fmt::format("Pre_section  {} not has no lane groups.", junction_pre_id);
        }
      }
    }
  }

  // 找到最后一个有效的 lane group
  const SDLaneGroupInfo *current_lg = nullptr;
  SDLaneGroupIndex       last_index;
  for (auto it = junction_section->lane_group_idx.rbegin(); it != junction_section->lane_group_idx.rend(); ++it) {
    const auto *lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(it->id);
    if (!lg || lg->lane_info.empty())
      continue;
    bool pure_non_navi      = is_pure_non_navi_lg(lg);
    bool too_short_non_navi = pure_non_navi && (lg->length < kMinEffectiveLgLen);
    if (!pure_non_navi && !too_short_non_navi) {
      current_lg = lg;
      last_index = *it;
      break;
    }
  }
  if (!current_lg) {
    SD_ENV_INFO_LOG << fmt::format("No valid lane group found in junction section {}.", junction_section_id);
    return {};
  }

  // 计算 junction 最后一个 lg 的 global end s，使用 end_range_offset
  double junction_start_global_s = GetSectionStartS(*junction_section, route_info);
  double current_end_global_s    = junction_start_global_s + last_index.end_range_offset;
  double current_end_s           = current_end_global_s - ego_global_s;
  double current_start_s = current_end_s - current_lg->length;  // 注意：如果有 gap，这里 start_s 到 end_s 的长度仍为 lg->length

  LgInfo entry{current_lg, current_start_s, current_end_s, junction_info.main_lane_indexs};
  lg_chain.push_back(entry);

  double current_start_global_s = current_end_global_s - current_lg->length;

  // 从 junction 向前遍历到 ego
  while (current_lg && current_start_global_s > ego_global_s) {
    const SDLaneGroupInfo *prev_lg = nullptr;
    for (uint64_t prev_lg_id : current_lg->predecessor_lane_group_ids) {
      if (mpp_lg_ids.count(prev_lg_id)) {
        const auto *lg = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(prev_lg_id);
        if (!lg || lg->lane_info.empty())
          continue;
        bool pure_non_navi      = is_pure_non_navi_lg(lg);
        bool too_short_non_navi = pure_non_navi && (lg->length < kMinEffectiveLgLen);
        if (!pure_non_navi && !too_short_non_navi) {
          prev_lg = lg;
          break;
        }
      }
    }

    if (!prev_lg) {
      break;
    }

    double prev_end_global_s   = current_start_global_s;
    double prev_start_global_s = prev_end_global_s - prev_lg->length;
    if (prev_start_global_s < ego_global_s) {
      prev_start_global_s = ego_global_s;
    }

    double prev_start_s = prev_start_global_s - ego_global_s;
    double prev_end_s   = prev_end_global_s - ego_global_s;

    std::vector<int> prev_target_indices = TraceTargetLaneIndices(*prev_lg, *current_lg, lg_chain.back().target_indices);

    entry = {prev_lg, prev_start_s, prev_end_s, prev_target_indices};
    lg_chain.push_back(entry);

    current_lg             = prev_lg;
    current_start_global_s = prev_start_global_s;
  }

  // Reverse to order from ego to junction
  std::reverse(lg_chain.begin(), lg_chain.end());

  // Group into virtual sections
  std::vector<VirtualSection> virtual_sections;
  if (lg_chain.empty()) {
    return virtual_sections;
  }

  VirtualSection current_section;
  current_section.id = lg_chain[0].lg->id;
  // current_section.lane_num            = lg_chain[0].lg->lane_num;
  current_section.lane_num            = static_cast<uint32_t>(motor_count(lg_chain[0].lg));
  current_section.start_s             = lg_chain[0].start_s;
  current_section.end_s               = lg_chain[0].end_s;
  current_section.target_lane_indices = lg_chain[0].target_indices;
  current_section.left_lane_change    = 0;
  current_section.right_lane_change   = 0;
  current_section.mid_lane_change     = 0;
  SD_ENV_INFO_LOG << fmt::format("lg_chain[0].lg->id {}.", lg_chain[0].lg->id);

  for (size_t i = 1; i < lg_chain.size(); ++i) {
    const auto &prev_lg = lg_chain[i - 1].lg;
    const auto &curr_lg = lg_chain[i].lg;
    SD_ENV_INFO_LOG << fmt::format("curr_lg {}.", curr_lg->id);
    // int         prev_num = prev_lg->lane_info.size();
    // int         curr_num = curr_lg->lane_info.size();
    const int prev_motor_num = motor_count(prev_lg);
    const int curr_motor_num = motor_count(curr_lg);
    const int diff           = curr_motor_num - prev_motor_num;

    if (curr_motor_num == current_section.lane_num && is_one_to_one_connection(prev_lg, curr_lg)) {
      // Extend the section
      current_section.end_s = lg_chain[i].end_s;
      current_section.id    = lg_chain[i].lg->id;
      // current_section.target_lane_indices = lg_chain[i].target_indices;
      current_section.target_lane_indices = map_to_motor_indices(curr_lg, lg_chain[i].target_indices);
    } else {
      int left_delta  = 0;
      int right_delta = 0;
      // 左侧减少
      {
        int l_idx_curr = left_motor_idx(curr_lg);
        if (l_idx_curr >= 0) {
          int left_pre_cnt = static_cast<int>(curr_lg->lane_info[l_idx_curr].previous_lane_ids.size());
          if (left_pre_cnt > 1)
            left_delta = -(left_pre_cnt - 1);
        }
      }

      // 左侧增加
      {
        int l_idx_curr = left_motor_idx(curr_lg);
        int l_idx_prev = left_motor_idx(prev_lg);
        if (l_idx_curr >= 0 && l_idx_prev >= 0) {
          int left_pre_cnt   = static_cast<int>(curr_lg->lane_info[l_idx_curr].previous_lane_ids.size());
          int prev_left_succ = static_cast<int>(prev_lg->lane_info[l_idx_prev].next_lane_ids.size());
          if (left_pre_cnt == 1 && prev_left_succ > 1)
            left_delta = (prev_left_succ - 1);
        }
      }

      // 右侧减少
      {
        int r_idx_curr = right_motor_idx(curr_lg);
        if (r_idx_curr >= 0) {
          int right_pre_cnt = static_cast<int>(curr_lg->lane_info[r_idx_curr].previous_lane_ids.size());
          if (right_pre_cnt > 1)
            right_delta = -(right_pre_cnt - 1);
        }
      }

      // 右侧增加
      {
        int r_idx_curr = right_motor_idx(curr_lg);
        int r_idx_prev = right_motor_idx(prev_lg);
        if (r_idx_curr >= 0 && r_idx_prev >= 0) {
          int right_pre_cnt   = static_cast<int>(curr_lg->lane_info[r_idx_curr].previous_lane_ids.size());
          int prev_right_succ = static_cast<int>(prev_lg->lane_info[r_idx_prev].next_lane_ids.size());
          if (right_pre_cnt == 1 && prev_right_succ > 1)
            right_delta = (prev_right_succ - 1);
        }
      }

      // ----中间变化----
      int mid_delta = diff - left_delta - right_delta;
      if (left_delta == 0 && right_delta == 0 && mid_delta != 0) {
        mid_delta = diff;
      }

      current_section.left_lane_change  = left_delta;
      current_section.right_lane_change = right_delta;
      current_section.mid_lane_change   = mid_delta;
      virtual_sections.push_back(current_section);

      current_section.id = curr_lg->id;
      // current_section.lane_num            = curr_lg->lane_num;
      current_section.lane_num = static_cast<uint32_t>(curr_motor_num);
      current_section.start_s  = lg_chain[i].start_s;
      current_section.end_s    = lg_chain[i].end_s;
      // current_section.target_lane_indices = lg_chain[i].target_indices;
      current_section.target_lane_indices = map_to_motor_indices(curr_lg, lg_chain[i].target_indices);
      current_section.left_lane_change    = 0;
      current_section.right_lane_change   = 0;
      current_section.mid_lane_change     = 0;
    }
  }
  virtual_sections.push_back(current_section);

  return virtual_sections;
}

double SDMapTopologyExtractor::GetEgoGlobalS(const SDRouteInfo &route_info) const {
  double   ego_global_s   = 0.0;
  uint64_t ego_section_id = route_info.navi_start.section_id;
  double   ego_s_offset   = route_info.navi_start.s_offset;
  for (const auto &sec : route_info.mpp_sections) {
    if (sec.id == ego_section_id) {
      ego_global_s += ego_s_offset;
      break;
    }
    ego_global_s += sec.length;
  }
  return ego_global_s;
}

double SDMapTopologyExtractor::GetSectionStartS(const SDSectionInfo &section, const SDRouteInfo &route_info) const {
  double current_s = 0.0;
  for (const auto &sec : route_info.mpp_sections) {
    if (sec.id == section.id) {
      return current_s;
    }
    current_s += sec.length;
  }
  return current_s;
}

std::vector<int> SDMapTopologyExtractor::TraceTargetLaneIndices(const SDLaneGroupInfo &prev_lg, const SDLaneGroupInfo &current_lg,
                                                                const std::vector<int> &target_indices) {
  std::vector<int> prev_target_indices;
  for (int target_idx : target_indices) {
    if (target_idx >= 0 && target_idx < current_lg.lane_info.size()) {
      const LaneInfo &target_lane = current_lg.lane_info[target_idx];
      for (const auto &prev_lane_id : target_lane.previous_lane_ids) {
        for (int i = 0; i < prev_lg.lane_info.size(); ++i) {
          if (prev_lg.lane_info[i].id == prev_lane_id) {
            prev_target_indices.push_back(i);
            break;
          }
        }
      }
    }
  }
  std::sort(prev_target_indices.begin(), prev_target_indices.end());
  prev_target_indices.erase(std::unique(prev_target_indices.begin(), prev_target_indices.end()), prev_target_indices.end());
  return prev_target_indices;
}

int SDMapTopologyExtractor::CountNonEmergencyLanesLD(uint64_t section_id) const {
  int cnt = 0;
  for (const auto &kv : INTERNAL_PARAMS.ld_map_data.GetLDLanes()) {
    const auto *lane = kv.second;
    if (lane && lane->section_id == section_id && lane->type != LaneType::LANE_EMERGENCY) {
      ++cnt;
    }
  }
  return cnt;
}

double SDMapTopologyExtractor::GetEgoGlobalS(const cem::message::env_model::RouteInfo &ld_route) const {
  double         s       = 0.0;
  const uint64_t ego_sec = ld_route.navi_start.section_id;
  const double   ego_off = ld_route.navi_start.s_offset;
  for (const auto &sec : ld_route.sections) {
    if (sec.id == ego_sec) {
      s += ego_off;
      break;
    }
    s += sec.length;
  }
  return s;
}

double SDMapTopologyExtractor::GetSectionStartS(const cem::message::env_model::SectionInfo &sec_in,
                                                const cem::message::env_model::RouteInfo   &ld_route) const {
  double s = 0.0;
  for (const auto &sec : ld_route.sections) {
    if (sec.id == sec_in.id)
      return s;
    s += sec.length;
  }
  return s;
}

std::pair<double, LaneDetail> SDMapTopologyExtractor::LD_BacktrackKeepTopoStart(uint64_t lane_id_merge) const {
  using cem::message::env_model::RouteInfo;
  using cem::message::env_model::SectionInfo;

  auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route || ld_route->sections.empty()) {
    SD_MERGE_LOG << "[LD] route missing in LD_BacktrackKeepTopoStart";
    const auto *ln = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id_merge);
    LaneDetail  tail{ln ? ln->id : lane_id_merge, ln ? ln->section_id : 0, 0, -1};
    return {0.0, tail};
  }
  const double ego_s = GetEgoGlobalS(*ld_route);

  std::unordered_map<uint64_t, double> sec_start_map;
  sec_start_map.reserve(ld_route->sections.size());
  {
    double acc = 0.0;
    for (const auto &s : ld_route->sections) {
      sec_start_map.emplace(s.id, acc);
      acc += s.length;
    }
  }
  auto sec_start_by_id = [&](uint64_t sec_id) -> double {
    auto it = sec_start_map.find(sec_id);
    return (it == sec_start_map.end()) ? std::numeric_limits<double>::infinity() : it->second;
  };

  const auto *cur = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id_merge);
  if (!cur) {
    SD_MERGE_LOG << fmt::format("[LD] lane {} not found in LD_BacktrackKeepTopoStart", lane_id_merge);
    return {0.0, LaneDetail{lane_id_merge, 0, 0, -1}};
  }

  std::unordered_set<uint64_t> visited;
  visited.reserve(32);

  // 若 merge 所在 section 不在主路径，直接在自车处停止（或按需改为当前段起点）
  double cur_sec_start = sec_start_by_id(cur->section_id);
  if (!std::isfinite(cur_sec_start)) {
    SD_MERGE_LOG << fmt::format("[LD] merge lane {} section {} not on route, stop at ego.", cur->id, cur->section_id);
    return {0.0, LaneDetail{cur->id, cur->section_id, 0, -1}};
  }

  double     keep_start_global = cur_sec_start;
  LaneDetail tail{cur->id, cur->section_id, 0, -1};

  // 保证 cur_sec_start 单调不增（往自车方向变小）
  while (cur && !visited.count(cur->id)) {
    visited.insert(cur->id);

    cur_sec_start = sec_start_by_id(cur->section_id);
    if (!std::isfinite(cur_sec_start)) {
      keep_start_global = ego_s;
      tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
      SD_MERGE_LOG << "[LD] current section not on route, stop at ego.";
      break;
    }

    // SD_MERGE_LOG << fmt::format("[LD] cur_sec_start={:.2f}", cur_sec_start);

    // 条件 (1)：当前车道前继数量 != 1  在当前车道起点处停止
    if (cur->previous_lane_ids.size() != 1) {
      keep_start_global = cur_sec_start;
      tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
      SD_MERGE_LOG << fmt::format("[LD] keep-topo stop by cond(1): lane {} prev_count={}", cur->id, cur->previous_lane_ids.size());
      break;
    }

    // 唯一前继
    const uint64_t pred_id = cur->previous_lane_ids.front();
    const auto    *pred    = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(pred_id);
    if (!pred) {
      keep_start_global = cur_sec_start;
      tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
      SD_MERGE_LOG << fmt::format("[LD] keep-topo stop by cond(1b): pred {} not found", pred_id);
      break;
    }

    // —— 方向校验：只允许向自车方向回溯（pred.start < cur.start），否则视为拓扑变化 —— //
    double pred_sec_start = sec_start_by_id(pred->section_id);
    // SD_MERGE_LOG << fmt::format("[LD] pred_sec_start {}, pred->points.size {}", pred_sec_start, pred->points.size());
    if (!std::isfinite(pred_sec_start) && !pred->points.empty()) {
      SD_MERGE_LOG << fmt::format("[LD] pred {} not in mainpaht, using first points as pred_sec_start", pred_id);
      pred_sec_start = pred->points[0].x;
    }
    if (!std::isfinite(pred_sec_start) && pred_sec_start >= cur_sec_start) {
      keep_start_global = cur_sec_start;
      tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
      SD_MERGE_LOG << fmt::format("[LD] keep-topo stop by cond(1c): pred_sec_start({:.2f}) valid and >= cur_sec_start({:.2f})",
                                  pred_sec_start, cur_sec_start);
      break;
    }

    // 条件 (2)：前继的后继数量 != 1 在当前车道起点处停止
    if (pred->next_lane_ids.size() != 1) {
      keep_start_global = cur_sec_start;
      tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
      SD_MERGE_LOG << fmt::format("[LD] keep-topo stop by cond(2): pred {} succ_count={}", pred->id, pred->next_lane_ids.size());
      break;
    }

    // 条件 (3)：前继 section 的起点已经在自车后/脚下（<= ego_s） 停在“自车脚下”
    if (pred_sec_start <= ego_s) {
      keep_start_global = ego_s;
      tail              = LaneDetail{pred->id, pred->section_id, 0, -1};
      SD_MERGE_LOG << fmt::format("[LD] keep-topo stop by cond(3): pred_sec_start={:.2f} <= ego_s={:.2f}", pred_sec_start, ego_s);
      break;
    }

    // 继续回溯：把“前继”作为新的当前车道（保证单调减小）
    cur               = pred;
    keep_start_global = pred_sec_start;
    tail              = LaneDetail{cur->id, cur->section_id, 0, -1};
  }

  double keep_start_rel = keep_start_global - ego_s;
  if (keep_start_rel < 0.0)
    keep_start_rel = 0.0;
  return {keep_start_rel, tail};
}

std::vector<std::vector<uint64_t>> SDMapTopologyExtractor::ExtractNonNaviLanesFromJunction(const JunctionInfo *junction_info_ptr) const {
  std::vector<std::vector<uint64_t>> result;

  if (!junction_info_ptr) {
    SD_COARSE_MATCH_TYPE2_LOG << "[ExtractNonNaviLanesFromJunction] junction_info_ptr is null";
    return result;
  }

  auto ld_route = INTERNAL_PARAMS.ld_map_data.GetLDRouteInfoPtr();
  if (!ld_route) {
    SD_COARSE_MATCH_TYPE2_LOG << "[ExtractNonNaviLanesFromJunction] LD route info is null";
    return result;
  }

  const double ego_global_s = GetEgoGlobalS(*ld_route);

  std::unordered_map<uint64_t, double> sec_start_map;
  sec_start_map.reserve(ld_route->sections.size());
  {
    double acc = 0.0;
    for (const auto &s : ld_route->sections) {
      sec_start_map.emplace(s.id, acc);
      acc += s.length;
    }
  }
  auto sec_start_by_id = [&](uint64_t sec_id) -> double {
    auto it = sec_start_map.find(sec_id);
    return (it == sec_start_map.end()) ? std::numeric_limits<double>::infinity() : it->second;
  };

  const uint64_t junction_id      = junction_info_ptr->junction_id;
  const auto    *junction_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(junction_id);
  if (!junction_section) {
    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Junction section {} not found", junction_id);
    return result;
  }

  double junction_section_start_s = sec_start_by_id(junction_id);
  double junction_section_end_s   = junction_section_start_s + junction_section->length;

  std::vector<const cem::message::env_model::SectionInfo *> non_navi_sections;
  for (const auto &succ_id : junction_section->successor_section_id_list) {
    const auto *succ_section = INTERNAL_PARAMS.ld_map_data.GetLDSectionInfoById(succ_id);
    if (succ_section && !succ_section->is_mpp_section) {
      non_navi_sections.push_back(succ_section);
    }
  }

  for (const auto *section : non_navi_sections) {
    std::vector<uint64_t> section_laneids;

    for (uint64_t lane_id : section->lane_ids) {
      const auto *lane = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(lane_id);
      if (lane && lane->type != cem::message::env_model::LaneType::LANE_EMERGENCY) {
        section_laneids.push_back(lane_id);
      }
    }

    std::vector<std::vector<uint64_t>> backtrack_result;
    for (uint64_t lane_id : section_laneids) {
      std::vector<uint64_t>        lane_chain;
      std::unordered_set<uint64_t> visited;
      visited.reserve(64);

      uint64_t    current_lane_id = lane_id;
      const auto *current_lane    = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(current_lane_id);
      if (!current_lane)
        continue;

      double current_lane_start_s = junction_section_end_s;

      while (current_lane && !visited.count(current_lane_id)) {
        visited.insert(current_lane_id);
        lane_chain.push_back(current_lane_id);

        if (current_lane->previous_lane_ids.size() != 1) {
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Stop backtracking: lane {} prev_count={}", current_lane_id,
                                      current_lane->previous_lane_ids.size());
          break;
        }

        uint64_t    prev_lane_id = current_lane->previous_lane_ids.front();
        const auto *prev_lane    = INTERNAL_PARAMS.ld_map_data.GetLDLaneInfoById(prev_lane_id);
        if (!prev_lane) {
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Previous lane {} not found", prev_lane_id);
          break;
        }

        double prev_section_start_s = sec_start_by_id(prev_lane->section_id);
        double prev_lane_start_s    = prev_section_start_s;

        double distance_to_jun = junction_section_end_s - prev_lane_start_s;
        if (distance_to_jun > 300.0) {
          lane_chain.push_back(prev_lane_id);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Stop backtracking: reached 300m limit");
          break;
        } else if (prev_lane_start_s <= ego_global_s) {
          lane_chain.push_back(prev_lane_id);
          SD_COARSE_MATCH_TYPE2_LOG << fmt::format("[ExtractNonNaviLanesFromJunction] Stop backtracking: reached ego position");
          break;
        }

        current_lane_id      = prev_lane_id;
        current_lane         = prev_lane;
        current_lane_start_s = prev_lane_start_s;
      }

      if (!lane_chain.empty()) {
        backtrack_result.push_back(std::move(lane_chain));
      }
    }

    for (auto &lane_chain : backtrack_result) {
      if (!lane_chain.empty()) {
        result.push_back(std::move(lane_chain));
      }
    }
  }

  return result;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem