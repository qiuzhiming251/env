#include "RoadSelector.h"
#include <algorithm>
#include <cmath>
#include "base/params_manager/params_manager.h"
#include "lib/sd_navigation/routing_map_debug.h"

namespace cem {
namespace fusion {
namespace navigation {

RoadSelector::RoadSelector() {}

std::vector<uint64_t> RoadSelector::SelectOptRoad(const std::vector<JunctionInfoCity>      &junctions_info_city,
                                                  const std::set<uint64_t>                 &bev_ego_lane_related,
                                                  const std::vector<uint64_t>              &history_guide_laneids,
                                                  const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                  const BevRouteInfo &complete_section_info, fmt::memory_buffer &info_buffer,
                                                  bool &isSplitRoadSelectWorked) {
  isSplitRoadSelectWorked = false;
  bev_ego_lane_related_   = bev_ego_lane_related;
  history_guide_laneids_  = history_guide_laneids;
  SD_COARSE_MATCH_TYPE2_LOG << "[SelectOptRoad]: bev_sections_in：";
  SD_COARSE_MATCH_TYPE2_LOG << fmt::format("{}", bev_sections_in);

  // use sd point to select road
  int                               road_split_num     = 0;
  const std::vector<SeparatorInfo> &section_separators = complete_section_info.separators;
  for (const auto &junction : junctions_info_city) {
    if ((junction.offset > -100) && (junction.offset < 200)) {
      if (junction.junction_type_city == JunctionTypeCity::RoadSplit ||
          (junction.junction_type_city == JunctionTypeCity::UTurn && junction.junction_action != JunctionAction::UTurn)) {
        road_split_num++;
      }
    }
  }
  if (bev_sections_in.size() > 1)
    isSplitRoadSelectWorked = true;
  if (road_split_num >= 2) {
    std::vector<uint64_t> road_selected_sd                                       = SelectRoadWithSdPts(bev_sections_in, section_separators);
    INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().select_road_junction = {888888};
    if (use_select_road_with_sdpts_) {
      SD_FINE_MATCH_LOG << "[SelectOptRoad]: select_road with SD pts. ";
      SD_COARSE_MATCH_TYPE2_LOG << "[SelectOptRoad]: select_road with SD pts. ";
      INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().select_road_junction = {99999};
      return road_selected_sd;
    }
  }

  if (bev_sections_in.empty()) {
    fmt::format_to(info_buffer, " bev_sec_size=0");
    SD_COARSE_MATCH_TYPE2_LOG << "bev_sections_in is empty";
    return {};
  }
  if (bev_sections_in.size() == 1) {
    fmt::format_to(info_buffer, " bev_sec_size=1");
    SD_COARSE_MATCH_TYPE2_LOG << "Only one section, return first section: ";
    return bev_sections_in[0];
  }

  // 筛选视觉范围内、未通过的 RoadSplit 路口   // 记录最近的已通过RoadSplit路口
  std::vector<JunctionInfoCity> nearby_junctions;
  JunctionInfoCity              latest_passed_split;
  bool                          has_passed_split   = false;
  bool                          process_split_flag = false;
  for (const auto &junction : junctions_info_city) {
    if (junction.junction_type_city == JunctionTypeCity::RoadSplit) {
      if (junction.offset <= 200.0 &&
          (junction.junction_state_city == JunctionStateCity::UNREACHED || junction.junction_state_city == JunctionStateCity::PASSING)) {
        nearby_junctions.push_back(junction);
      } else if (junction.junction_state_city == JunctionStateCity::PASSED) {
        // 如果有多个已通过的split，记录最近的一个
        if (!has_passed_split || junction.offset > latest_passed_split.offset) {
          latest_passed_split = junction;
          has_passed_split    = true;
        }
      }
    }
  }

  //判断自车是否在两个split中间
  if ((!nearby_junctions.empty()) && has_passed_split) {
    double dis = nearby_junctions[0].offset - latest_passed_split.offset;
    SD_COARSE_MATCH_TYPE2_LOG << "latest_passed_split.offset: " << latest_passed_split.offset;
    if (dis < 200 && latest_passed_split.offset > -30.0) {
      SD_COARSE_MATCH_TYPE2_LOG << "2 close split ,adjust roadselect offset to -30m";
      process_split_flag = false;
    } else {
      process_split_flag = true;
    }
  } else {
    process_split_flag = true;
  }

  SD_COARSE_MATCH_TYPE2_LOG << "lprocess_split_flag: " << process_split_flag;

  if ((!nearby_junctions.empty()) && (process_split_flag)) {
    std::sort(nearby_junctions.begin(), nearby_junctions.end(),
              [](const JunctionInfoCity &a, const JunctionInfoCity &b) { return a.offset < b.offset; });
    const JunctionInfoCity &selected_junction = nearby_junctions.front();
    const auto             &junction_type     = selected_junction.junction_type_city;
    const auto             &split_merge_dir   = selected_junction.split_merge_direction;
    std::string             dir_str           = StrDirectionSplitMerge(split_merge_dir);
    std::string             type_str          = StrJunctionTypeCity(static_cast<JunctionTypeCity>(selected_junction.junction_type_city));
    SD_COARSE_MATCH_TYPE2_LOG << "junction_type: " << type_str;
    SD_COARSE_MATCH_TYPE2_LOG << "split_merge_direction: " << dir_str;
    SD_COARSE_MATCH_TYPE2_LOG << "Selected nearest junction: id=" << selected_junction.junction_id << ", type=" << type_str
                              << ", split_merge_direction=" << dir_str << ", offset=" << selected_junction.offset;
    SD_COARSE_MATCH_TYPE2_LOG << "junctions_info_city details:";

    SD_COARSE_MATCH_TYPE2_LOG << fmt::format("{}", selected_junction);
    INTERNAL_PARAMS.navigation_info_data.navi_debug_infos().select_road_junction = {selected_junction.junction_id};
    int selected_index                                                           = 0;

    switch (junction_type) {
      case JunctionTypeCity::RoadSplit: {
        // 使用代价函数选择最优section
        std::vector<int> ego_section_indexs = {};  /// 自车所在的section index
        for (unsigned int i = 0; i < bev_sections_in.size(); i++) {
          for (unsigned int j = 0; j < bev_sections_in[i].size(); j++) {
            if (bev_ego_lane_related_.find(bev_sections_in[i][j]) != bev_ego_lane_related_.end()) {
              ego_section_indexs.emplace_back(i);
              break;
            }
          }
        }

        // 使用形点选路结果作为主要权重
        std::vector<uint64_t> road_selected_sd = SelectRoadWithSdPts(bev_sections_in, section_separators);

        // 使用代价函数选择最优section
        double min_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < bev_sections_in.size(); ++i) {
          double total_cost =
              CalculateCombinedSectionCost(static_cast<int>(i), bev_sections_in, ego_section_indexs, road_selected_sd, split_merge_dir);

          if (total_cost < min_cost) {
            min_cost       = total_cost;
            selected_index = static_cast<int>(i);
          }
        }

        SD_COARSE_MATCH_TYPE2_LOG << " [RoadSelect] final selected index: " << selected_index << " with cost: " << min_cost;

        isSplitRoadSelectWorked = true;
        break;
      }
      default:
        selected_index = SelectMainRoadSectionNoNavi(bev_sections_in, section_separators);
        if (selected_index == -1) {
          /*后续改为返回空值*/
          selected_index = 0;
        }

        break;
    }
    fmt::format_to(info_buffer, "  selected_idx:{}-{}", selected_index, bev_sections_in);
    SD_COARSE_MATCH_TYPE2_LOG << "Final selected index: " << selected_index;
    return bev_sections_in[selected_index];
  } else {
    // 没有符合条件的路口，走无导航选主路逻辑
    SD_COARSE_MATCH_TYPE2_LOG << "No suitable RoadMerge or RoadSplit junction within "
                                 "200m, selecting main road section";
    int selected_section_index = SelectMainRoadSectionNoNavi(bev_sections_in, section_separators);
    fmt::format_to(info_buffer, "  no_navi_idx:{}", selected_section_index);
    if (bev_sections_in.size() > 1)
      isSplitRoadSelectWorked = true;
    SD_COARSE_MATCH_TYPE2_LOG << "No navigation, selected_section_index: " << selected_section_index;
    return selected_section_index == -1 ? std::vector<uint64_t>{} : bev_sections_in[selected_section_index];
  }
}

std::vector<uint64_t> RoadSelector::HoldOptRoad(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                const BevRouteInfo &complete_section_info, const std::vector<uint64_t> &road_selected) {
  const std::vector<SeparatorInfo> &section_separators = complete_section_info.separators;

  if (!section_separators.empty() && !road_selected.empty() && bev_sections_in.size() > 1) {
    auto it = std::find_if(bev_sections_in.begin(), bev_sections_in.end(), [&](const auto &sec) { return sec == road_selected; });
    if (it == bev_sections_in.end()) {
      hist_road_infos_.clear();
      return road_selected;
    }
    const int                    kMinStableCount = 3;
    static std::vector<uint64_t> road_confirmed  = road_selected;
    std::vector<uint64_t>        hold_road_selected;
    uint64_t                     curr_sep_idx = 0;
    bool                         curr_is_left = true;
    bool                         found        = false;
    for (uint64_t sep_idx = 0; sep_idx < section_separators.size(); ++sep_idx) {
      if (sep_idx < bev_sections_in.size()) {
        // 路沿左侧对应 bev_sections_in[sep_idx]
        if (bev_sections_in.at(sep_idx) == road_selected) {
          curr_sep_idx = sep_idx;
          curr_is_left = true;
          found        = true;
          break;
        }

        // 路沿右侧对应 bev_sections_in[sep_idx + 1]
        if (bev_sections_in.at(sep_idx + 1) == road_selected) {
          curr_sep_idx = sep_idx;
          curr_is_left = false;
          found        = true;
          break;
        }
      }
    }
    // 未找到: 异常帧直接透传并清空历史
    if (!found) {
      hist_road_infos_.clear();
      hold_road_selected = road_selected;
      return hold_road_selected;
    }
    const auto        &curr_sep_ids = section_separators.at(curr_sep_idx).edge_ids;
    HistSelectRoadInfo curr{curr_sep_ids, curr_is_left, road_selected};
    hist_road_infos_.insert(hist_road_infos_.begin(), curr);
    if (hist_road_infos_.size() > kMinStableCount)
      hist_road_infos_.pop_back();

    if (hist_road_infos_.size() < kMinStableCount) {
      road_confirmed = road_selected;
    }

    for (size_t i = 0; i < hist_road_infos_.size(); ++i) {
      const auto &hist_road_info = hist_road_infos_[i];
      SD_FINE_MATCH_LOG << "frame[" << i << "] side= " << (hist_road_info.is_left ? 'L' : 'R')
                        << " separator_ids= " << fmt::format("{}", hist_road_info.separator_ids)
                        << " road= " << fmt::format("{}", hist_road_info.road);
    }

    bool hist_stable = (hist_road_infos_.size() == kMinStableCount) && hist_road_infos_.at(0) == hist_road_infos_.at(1) &&
                       hist_road_infos_.at(1) == hist_road_infos_.at(2);

    // SD_FINE_MATCH_LOG << "hist_stable: " << hist_stable;
    if (hist_stable) {
      road_confirmed = road_selected;
    }
    hold_road_selected = road_confirmed;
    // std::ostringstream hold_os_section;
    // for (auto &section : hold_road_selected) {
    //   hold_os_section << section << ",";
    // }
    // SD_FINE_MATCH_LOG << "[Hold BevSection]" << hold_os_section.str();

    return hold_road_selected;
  } else {
    hist_road_infos_.clear();
  }

  return road_selected;
}

std::vector<uint64_t> RoadSelector::SelectRoadWithSdPts(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                        const std::vector<SeparatorInfo>         &section_separators) {
  use_select_road_with_sdpts_ = false;
  if (bev_sections_in.empty()) {
    // SD_FINE_MATCH_LOG << "bev_sections_in is empty";
    return {};
  }

  SD_FINE_MATCH_LOG << "raw bev section";
  for (size_t i = 0; i < bev_sections_in.size(); ++i) {
    for (size_t j = 0; j < bev_sections_in[i].size(); ++j) {
      SD_FINE_MATCH_LOG << bev_sections_in[i][j] << ", ";
    }
    SD_FINE_MATCH_LOG << "-------";
  }

  if (bev_sections_in.size() == 1) {
    SD_FINE_MATCH_LOG << "Only one section, return first section: ";
    SD_FINE_MATCH_LOG << "new bev section";
    for (const auto &bev_section : bev_sections_in[0]) {
      SD_FINE_MATCH_LOG << bev_section;
    }
    return bev_sections_in[0];
  }
  auto sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();

  if (!sd_route) {
    SD_FINE_MATCH_LOG << "sd_route is null";
    return {};
  }

  const auto &mpp_sections = sd_route->mpp_sections;
  if (mpp_sections.empty()) {
    SD_FINE_MATCH_LOG << "mpp_sections is empty";
    return {};
  }

  Eigen::Vector2f              start_pt{-1000.0f, -1000.0f};
  std::vector<Eigen::Vector2f> start_points;

  for (const auto &separator : section_separators) {
    for (size_t i = 0; i < separator.edge_points.size(); i++) {
      if ((separator.types.at(i) == LineSortType::RoadBoundary) || (separator.types.at(i) == LineSortType::VirtualEdge)) {
        start_points.emplace_back(separator.edge_points.at(i).first);
      }
    }
  }

  if (start_points.size() > 0) {
    start_pt = *(std::min_element(start_points.begin(), start_points.end(),
                                  [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); }));
    SD_FINE_MATCH_LOG << "start_point: " << start_pt.x() << ", " << start_pt.y();
  }

  bool use_edge_points = (start_pt.x() == -1000.0f) ? false : true;

  std::vector<const SDSectionInfo *> sd_mapp_sections;
  const SDSectionInfo               *ego_section    = nullptr;
  uint64_t                           ego_section_id = sd_route->navi_start.section_id;
  ego_section                                       = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(ego_section_id);
  int ego_section_idx                               = -1;

  if ((ego_section != nullptr) && (ego_section->is_mpp_section)) {
    // SD_FINE_MATCH_LOG << "ego section.id: " << ego_section->id;
    for (size_t i = 0; i < mpp_sections.size(); i++) {
      if (mpp_sections[i].id == ego_section->id) {
        ego_section_idx = static_cast<int>(i);
        break;
      }
    }
  }

  if ((ego_section_id == 0) || (ego_section == nullptr) || (ego_section_idx == -1)) {
    return {};
  }

  double ego_s_offset = sd_route->navi_start.s_offset;
  double ego_s_global = 0.0;
  for (int i = 0; i < ego_section_idx; ++i) {
    ego_s_global += mpp_sections[i].length;
  }
  ego_s_global += ego_s_offset;
  constexpr double kEdgePointOffs = -50.0;
  double           min_s          = (use_edge_points) ? start_pt.x() - kEdgePointOffs : -100;
  double           max_s          = (use_edge_points) ? start_pt.x() + 200 : 200;
  double           current_s      = 0.0;
  for (size_t i = 0; i < mpp_sections.size(); ++i) {
    const auto &section         = mpp_sections[i];
    double      section_start_s = current_s;
    double      section_end_s   = current_s + section.length;

    if (section_end_s < ego_s_global + min_s || section_start_s > ego_s_global + max_s) {
      current_s += section.length;
      continue;
    }
    if (section.is_mpp_section) {
      sd_mapp_sections.emplace_back(&section);
      // SD_FINE_MATCH_LOG << "section_id: " << section.id;
    }
    current_s += section.length;
  }

  std::vector<Eigen::Vector2f> all_sd_points;
  for (const auto &sd_section : sd_mapp_sections) {
    const auto &section_pts = *sd_section->points;
    // SD_FINE_MATCH_LOG << "---sd_section id---: " << sd_section->id;
    for (const auto &pt : section_pts) {
      if (std::find(all_sd_points.begin(), all_sd_points.end(), pt) != all_sd_points.end()) {
        continue;
      }
      if (use_edge_points && (pt.x() < start_pt.x() - kEdgePointOffs)) {
        continue;
      }
      all_sd_points.emplace_back(pt);
    }
  }

  if (all_sd_points.size() < 2) {
    return {};
  }

  // for (const auto &pt : all_sd_points) {
  //   SD_FINE_MATCH_LOG << "SD Point: (" << pt.x() << ", " << pt.y() << ")\n";
  // }
  float  min_cost   = std::numeric_limits<double>::max();
  size_t nearest_id = -1;
  for (size_t i = 0; i < bev_sections_in.size(); ++i) {
    for (const auto &lane_id : bev_sections_in[i]) {
      auto bev_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(lane_id);
      if ((bev_lane) && (bev_lane->line_points.size() > 2)) {
        std::vector<Eigen::Vector2f> bev_eigen_points;
        float                        bev_lane_front = bev_lane->line_points.back().x;
        float start_overlap = (std::fabs(start_pt.x() - bev_lane_front) < 20.0) ? (bev_lane_front - 30.0) : start_pt.x();
        for (const auto &bev_point : bev_lane->line_points) {
          if (use_edge_points && (bev_point.x < start_overlap)) {
            continue;
          }
          if (bev_point.x < all_sd_points.front().x() || (bev_point.x > all_sd_points.back().x())) {
            continue;
          }
          // SD_FINE_MATCH_LOG << "Bev Point: (" << bev_point.x << ", "
          //                   << bev_point.y << ")\n";
          bev_eigen_points.emplace_back(bev_point.x, bev_point.y);
        }
        std::vector<Eigen::Vector2f> sparse_bev_points{};
        if (bev_eigen_points.size() > 20) {
          sparse_bev_points = SparsePoints(bev_eigen_points, 8);
        } else {
          sparse_bev_points = bev_eigen_points;
        }
        // for (const auto &pt : sparse_bev_points) {
        //   SD_FINE_MATCH_LOG << "Sparse Point: (" << pt.x() << ", " << pt.y()
        //                     << ")\n";
        // }
        if ((sparse_bev_points.size() < 3) || (all_sd_points.size() < 3)) {
          continue;
        }
        float cost = CalcPointSetSimilarity(sparse_bev_points, all_sd_points);
        SD_FINE_MATCH_LOG << "sec-lane-cost: " << i << ", " << bev_lane->id << ", " << cost;
        if (cost < min_cost) {
          min_cost   = cost;
          nearest_id = i;
        }
      }
    }
  }
  SD_FINE_MATCH_LOG << "min_cost: " << min_cost;
  SD_FINE_MATCH_LOG << "nearest_id: " << nearest_id;
  if (nearest_id == -1) {
    return {};
  }

  SD_FINE_MATCH_LOG << "new bev section";
  for (const auto &bev_section : bev_sections_in[nearest_id]) {
    SD_FINE_MATCH_LOG << bev_section;
  }
  use_select_road_with_sdpts_ = true;
  return bev_sections_in[nearest_id];
}

float RoadSelector::CalcPointSetSimilarity(std::vector<Eigen::Vector2f> &bev_points, std::vector<Eigen::Vector2f> &sd_points) {
  float sum_distance = 0.0f;
  float sum_angle    = 0.0f;
  float avg_distance = std::numeric_limits<float>::max();
  float avg_angle    = std::numeric_limits<float>::max();
  float avg_cost     = std::numeric_limits<float>::max();
  int   count        = 0;
  for (size_t i = 0; i < bev_points.size(); i++) {
    float min_distance = std::numeric_limits<float>::max();
    float min_angle    = std::numeric_limits<float>::max();
    for (size_t j = 0; j < sd_points.size() - 1; j++) {
      Eigen::Vector2f seg_start  = sd_points.at(j);
      Eigen::Vector2f seg_end    = sd_points.at(j + 1);
      Eigen::Vector2f sd_seg_vec = seg_end - seg_start;
      // calc distance
      float seg_length_sq = sd_seg_vec.squaredNorm();
      if (seg_length_sq == 0.0f) {
        continue;
      }
      float t                    = ((bev_points.at(i) - seg_start).dot(sd_seg_vec)) / seg_length_sq;
      t                          = std::max(0.0f, std::min(1.0f, t));
      Eigen::Vector2f proj_point = seg_start + t * sd_seg_vec;
      float           distance   = (bev_points.at(i) - proj_point).norm();
      float           angle      = 0.0f;
      // calc angle
      Eigen::Vector2f bev_seg_vec;
      if (i == bev_points.size() - 1) {
        bev_seg_vec = bev_points.at(i) - bev_points.at(i - 1);
      } else {
        bev_seg_vec = bev_points.at(i + 1) - bev_points.at(i);
      }
      float dot_product  = sd_seg_vec.dot(bev_seg_vec);
      float norm_sd_vec  = sd_seg_vec.norm();
      float norm_bev_vec = bev_seg_vec.norm();
      if (norm_sd_vec != 0.0f && norm_bev_vec != 0.0f) {
        float cos_theta = dot_product / (norm_sd_vec * norm_bev_vec);
        cos_theta       = std::max(std::min(cos_theta, 1.0f), -1.0f);
        // SD_FINE_MATCH_LOG << "cos_theta: " << cos_theta;
        angle = acos(cos_theta);
      } else {
        angle = 0.0f;
      }
      // SD_FINE_MATCH_LOG << "angle: " << angle * 360 / M_PI;
      if (distance < min_distance) {
        min_distance = distance;
        min_angle    = angle;
      }
    }
    sum_distance += min_distance;
    sum_angle += min_angle;
    count++;
  }

  // calc cost
  if (count > 0) {
    avg_distance          = sum_distance / count;
    avg_angle             = sum_angle / count;
    float weight_distance = 0.9;
    float weight_angle    = 0.1;
    avg_cost              = weight_distance * avg_distance + weight_angle * avg_angle;  // now not use angle info

    // SD_FINE_MATCH_LOG << "avg_distance: " << avg_distance
    //                   << " avg_angle: " << avg_angle * 360 / M_PI;
  }
  return avg_distance;
}

std::vector<Eigen::Vector2f> RoadSelector::SparsePoints(const std::vector<Eigen::Vector2f> &points, size_t n) {
  if (points.empty() || n == 0 || n == 1) {
    return {};
  }
  size_t                       total_size = points.size();
  std::vector<Eigen::Vector2f> sparse_points;
  if (total_size >= n) {
    size_t step = (total_size - 1) / (n - 1);
    for (size_t i = 0; i < n; ++i) {
      size_t index = i * step;
      if (index < total_size) {
        sparse_points.emplace_back(points[index]);
      }
    }
  } else {
    sparse_points = points;
  }
  return sparse_points;
}

size_t RoadSelector::SelectSectionWithMostLanes(const std::vector<std::vector<uint64_t>> &bev_sections_in) {
  size_t selected_index = 0;
  size_t max_lane_count = 0;
  for (size_t i = 0; i < bev_sections_in.size(); ++i) {
    size_t lane_count = bev_sections_in[i].size();
    if (lane_count > max_lane_count) {
      max_lane_count = lane_count;
      selected_index = i;
    }
  }
  SD_COARSE_MATCH_TYPE2_LOG << "Selected index with most lanes: " << selected_index << ", max_lane_count: " << max_lane_count;
  return selected_index;
}

/// 无导航动作参考时选主路
int RoadSelector::SelectMainRoadSectionNoNavi(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                              const std::vector<SeparatorInfo>         &section_separators) {
  if (bev_sections_in.empty()) {
    SD_COARSE_MATCH_TYPE2_LOG << "bev_sections_in is empty";
    return -1;
  }

  if (bev_sections_in.size() == 1) {
    SD_COARSE_MATCH_TYPE2_LOG << "Only one section, return index 0";
    return 0;
  }

  int              selected_section_index = -1;
  std::vector<int> ego_section_indexs     = {};  /// 自车所在的section index
  for (unsigned int i = 0; i < bev_sections_in.size(); i++) {
    for (unsigned int j = 0; j < bev_sections_in[i].size(); j++) {
      if (bev_ego_lane_related_.find(bev_sections_in[i][j]) != bev_ego_lane_related_.end()) {
        ego_section_indexs.emplace_back(i);
        break;
      }
    }
  }

  SD_COARSE_MATCH_TYPE2_LOG << "ego_section_indexs: " << fmt::format("[{}]", fmt::join(ego_section_indexs, ", "));

  std::vector<uint64_t>      dummy_road_selected_sd = {};
  std::vector<SeparatorInfo> dummy_separators       = {};
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()) {
    const auto &sd_route = INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr();
    if (sd_route && !sd_route->mpp_sections.empty()) {
      dummy_road_selected_sd = SelectRoadWithSdPts(bev_sections_in, section_separators);
    }
  }

  // 使用代价函数选择最优section
  std::vector<int>    dummy_ego_indexs = ego_section_indexs;
  DirectionSplitMerge dummy_direction  = DirectionSplitMerge::Unknown;

  double min_cost       = std::numeric_limits<double>::max();
  int    selected_index = 0;

  for (size_t i = 0; i < bev_sections_in.size(); ++i) {
    double total_cost =
        CalculateCombinedSectionCost(static_cast<int>(i), bev_sections_in, dummy_ego_indexs, dummy_road_selected_sd, dummy_direction);

    if (total_cost < min_cost) {
      min_cost       = total_cost;
      selected_index = static_cast<int>(i);
    }
  }

  SD_COARSE_MATCH_TYPE2_LOG << " [SelectMainRoadSectionNoNavi] final selected index: " << selected_index << " with cost: " << min_cost;
  return selected_index;
}

/**
 * //RoadSplit场景中 判断是否是主路到辅路场景
 * @param junction 城市 junction 信息，包含 junction 类型、后续道路类别等
 * @return bool 如果道路分割主路到匝道，则返回 true，否则返回 false
 */
SplitPassType RoadSelector::RoadSplitTypeJudge(const JunctionInfoCity &junction) {
  SplitPassType passType = SplitPassType::Unknown;

  if (junction.junction_type_city != JunctionTypeCity::RoadSplit) {
    return passType;
  }

  if (junction.succ_road_class.size() == 2) {
    if ((junction.split_merge_direction == DirectionSplitMerge::Left && junction.succ_road_class.front() == RoadMainType::RoadMain) ||
        (junction.split_merge_direction == DirectionSplitMerge::Right && junction.succ_road_class.back() == RoadMainType::RoadMain)) {
      SD_COARSE_MATCH_TYPE2_LOG << "[RoadSplitTypeJudge]: MainToMain";
      return SplitPassType::MainToMain;
    } else {
      SD_COARSE_MATCH_TYPE2_LOG << "[RoadSplitTypeJudge]: MainToRamp";
      return SplitPassType::MainToRamp;
    }
  } else if (junction.succ_road_class.size() == 3) {
    if (junction.split_merge_direction == DirectionSplitMerge::Straight && junction.succ_road_class[1] == RoadMainType::RoadMain) {
      SD_COARSE_MATCH_TYPE2_LOG << "[RoadSplitTypeJudge]: MainToMain";
      return SplitPassType::MainToMain;
    } else {
      SD_COARSE_MATCH_TYPE2_LOG << "[RoadSplitTypeJudge]: MainToRamp";
      return SplitPassType::MainToRamp;
    } /*后续添加多个后继的非直行场景*/
  } else {
    return passType;
  }
}

/**
 * //分叉口直行 选路
 * @param bev_sections_in
 * @return size_t 选路结果
 */
std::vector<std::vector<uint64_t>> RoadSelector::SelectSectionInStraight(const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                                         const JunctionInfoCity                   &junction) {
  std::vector<std::vector<uint64_t>> sections     = bev_sections_in;
  SplitPassType                      PassType     = RoadSplitTypeJudge(junction);
  int                                ramp_lanenum = GetLanenumOfRamp(junction, PassType);

  if (bev_sections_in.size() >= 2) { /* 判断最后的section是否是辅路，后续改为辅路车道数，待充分验证后打开
            */
    // if ((sections[sections.size() - 1].size() == 1) || (sections[sections.size() - 1].size() == ramp_lanenum))
    // // if (sections[sections.size() - 1].size() == 1)
    // {
    sections.pop_back();
    SD_COARSE_MATCH_TYPE2_LOG << "SelectSectionInStraight: sections.pop_back()";
  }
  /*后续添加多段section的处理*/

  return sections;
}
/**
 * //辅路车道数-->子路径车道数
 * @param  junction 路口
 * @param  PassType 路口通过类型
 * @return 辅路车道数
 */
int RoadSelector::GetLanenumOfRamp(const JunctionInfoCity &junction, SplitPassType &PassType) {
  int lanenum = 0;
  if ((junction.main_road_lane_nums > 0) && (junction.target_road_lane_nums > 0)) {
    if (PassType == SplitPassType::MainToMain) {
      lanenum = junction.main_road_lane_nums - junction.target_road_lane_nums;
    } else if (PassType == SplitPassType::MainToRamp) {
      lanenum = junction.main_road_lane_nums - junction.target_road_lane_nums;
    }
  } else {
  }

  if (lanenum < 0) {
    lanenum = 0;
  }
  SD_COARSE_MATCH_TYPE2_LOG << " GetLanenumOfRamp:  " << lanenum;

  return lanenum;
}

double RoadSelector::CalculateCombinedSectionCost(int section_index, const std::vector<std::vector<uint64_t>> &bev_sections_in,
                                                  const std::vector<int> &ego_section_indexs, const std::vector<uint64_t> &road_selected_sd,
                                                  const DirectionSplitMerge &split_merge_dir) {
  // 计算基础代价（考虑自车位置和车道数差异等）
  double cost                = 0.0;
  double ego_in_section_cost = 0.0;
  double lane_diff_cost      = 0.0;
  double history_lane_cost   = 0.0;
  // double single_lane_cost    = 0.0;

  const double kEgoInSectionCost        = 0.0;
  const double kEgoNotInSectionCostHigh = 5.0;
  const double kEgoNotInSectionCostLow  = 1.0;
  const double kLaneDiffThreshold       = 2.0;
  const double kLaneDiffCostFactor      = 1.0;
  const double kHistoryLaneCostFactor   = 2.0;
  // const double kSingleLaneCost          = 2.0;

  // 获取自车所在section的车道数
  uint64_t main_road_lane_num = 0;
  if (INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr() && INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->navi_start.section_id != 0) {
    const auto *ego_section =
        INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(INTERNAL_PARAMS.sd_map_data.GetSDRouteInfoPtr()->navi_start.section_id);
    if (ego_section != nullptr) {
      main_road_lane_num = ego_section->lane_num;
    }
  }

  // 获取当前section的车道数
  uint64_t current_section_lane_num = bev_sections_in[section_index].size();

  // 如果section_index跟属于自车section，给予较低的代价
  bool is_ego_in_section = std::find(ego_section_indexs.begin(), ego_section_indexs.end(), section_index) != ego_section_indexs.end();
  if (is_ego_in_section) {
    ego_in_section_cost += kEgoInSectionCost;
  } else {
    ego_in_section_cost += kEgoNotInSectionCostHigh;  // 自车不在该section中，增加代价
  }

  // 考虑车道数差异
  if (main_road_lane_num > 0) {
    int lane_diff = std::abs(static_cast<int>(current_section_lane_num) - static_cast<int>(main_road_lane_num));
    SD_COARSE_MATCH_TYPE2_LOG << " [CalculateCombinedSectionCost] current_section_lane_num " << current_section_lane_num
                              << " main_road_lane_num: " << main_road_lane_num << " lane_diff: " << lane_diff;
    // 只有车道数差异大于2时才计入cost
    if (lane_diff > kLaneDiffThreshold) {
      lane_diff_cost += lane_diff * kLaneDiffCostFactor;
    }
  }

  // 考虑是否存在历史推荐车道
  int history_lane_count = 0;
  for (const auto &lane_id : bev_sections_in[section_index]) {
    if (std::find(history_guide_laneids_.begin(), history_guide_laneids_.end(), lane_id) != history_guide_laneids_.end()) {
      history_lane_count++;
    }
  }
  history_lane_cost -= history_lane_count * kHistoryLaneCostFactor;

  // // 考虑当前section车道数
  // if (current_section_lane_num == 1) {
  //   single_lane_cost += kSingleLaneCost;  // 单车道section增加代价
  // }

  // 基础总cost代价
  double base_cost = ego_in_section_cost + lane_diff_cost + history_lane_cost;

  // 形点选路权重：如果当前section是形点选路结果，则降低代价
  double sd_weight            = 0.0;
  int    selected_by_sd_index = -1;
  if (!road_selected_sd.empty()) {
    for (size_t i = 0; i < bev_sections_in.size(); ++i) {
      if (bev_sections_in[i] == road_selected_sd) {
        selected_by_sd_index = static_cast<int>(i);
        break;
      }
    }
  }
  if (selected_by_sd_index != -1 && section_index == selected_by_sd_index) {
    sd_weight = -10.0;  // 显著降低代价，优先选择形点选路结果
  }

  // split_merge_dir权重：作为次要参考
  double direction_weight = 0.0;
  if (split_merge_dir == DirectionSplitMerge::Left && section_index == 0) {
    direction_weight = -2.0;  // 稍微降低代价
  } else if (split_merge_dir == DirectionSplitMerge::Right && section_index == static_cast<int>(bev_sections_in.size() - 1)) {
    direction_weight = -2.0;  // 稍微降低代价
  } else if (split_merge_dir == DirectionSplitMerge::Straight) {
    // 对于直行，选择中间的section（如果存在）
    if (bev_sections_in.size() == 3 && section_index == 1) {
      direction_weight = -2.0;
    } else if (bev_sections_in.size() == 2 && section_index == 0) {
      direction_weight = -2.0;
    }
  }

  // 总代价 = 基础代价 + 形点权重 + 方向权重
  double total_cost = base_cost + sd_weight + direction_weight;

  SD_COARSE_MATCH_TYPE2_LOG << " [CalculateCombinedSectionCost] section " << section_index << " base_cost: " << base_cost
                            << " sd_weight: " << sd_weight << " direction_weight: " << direction_weight << " total_cost: " << total_cost;

  return total_cost;
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem
