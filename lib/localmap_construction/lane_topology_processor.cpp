#include "lane_topology_processor.h"
#include "common/utils/GeoMathUtil.h"
#include "common/utils/lane_geometry.h"
// #define ENABLE_DEBUG_LANE_TOPOLOGY 1
// #define ENABLE_DEBUG_LANE_TOPOLOGY2 1

namespace cem {
namespace fusion {

constexpr uint64_t INVALID_LANE_ID = static_cast<uint64_t>(-1);
constexpr bool DEBUG_INFO = false;

// 计算点到线段的最小距离
float PointToSegmentDistance(const Eigen::Vector2f& p, const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    Eigen::Vector2f ab = b - a;
    float ab2 = ab.squaredNorm();
    if (ab2 == 0) return (p - a).norm();
    float t = ((p - a).dot(ab)) / ab2;
    t = std::max(0.0f, std::min(1.0f, t));
    Eigen::Vector2f proj = a + t * ab;
    return (p - proj).norm();
}

// 计算线段到线段的最小距离
float SegmentToSegmentDistance(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
                              const Eigen::Vector2f& q1, const Eigen::Vector2f& q2) {
    // 取两段端点分别对另一段求距离，取最小值
    float d1 = PointToSegmentDistance(p1, q1, q2);
    float d2 = PointToSegmentDistance(p2, q1, q2);
    float d3 = PointToSegmentDistance(q1, p1, p2);
    float d4 = PointToSegmentDistance(q2, p1, p2);
    return std::min({d1, d2, d3, d4});
}

// 判断线段(p1, p2)是否经过多边形polygon的附近
bool IsSegmentNearPolygon(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
                         const std::vector<Eigen::Vector2f>& polygon, float threshold) {
    int n = polygon.size();
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2f& q1 = polygon[i];
        const Eigen::Vector2f& q2 = polygon[(i+1)%n]; // 闭合
        float dist = SegmentToSegmentDistance(p1, p2, q1, q2);
        if (dist <= threshold) {
            return true;
        }
    }
    return false;
}

// 判断线段(p1, p2)是否经过任意一个polygon的附近
bool IsSegmentNearAnyPolygon(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
                             const std::map<uint64_t, std::vector<Eigen::Vector2f>>& cross_walk_geos,
                             float threshold) {
    for (const auto& kv : cross_walk_geos) {
        if (IsSegmentNearPolygon(p1, p2, kv.second, threshold)) {
            return true;
        }
    }
    return false;
}

ProcessLanesTopo::ProcessLanesTopo() {}

void ProcessLanesTopo::GetEgoLaneId(const std::vector<cem::message::sensor::BevLaneInfo> &vct_bev_lane_info) {
  auto ego_it = std::find_if(vct_bev_lane_info.begin(), vct_bev_lane_info.end(), [](const auto &lane) {
    return lane.position == static_cast<int>(cem::message::sensor::BevLanePosition::LANE_LOC_EGO);
  });

  ego_lane_id_ = (ego_it != vct_bev_lane_info.end()) ? ego_it->id : INVALID_LANE_ID;

#ifdef ENABLE_DEBUG_LANE_TOPOLOGY
  AINFO << "Ego lane ID: " << ego_lane_id_;
#endif
}

void ProcessLanesTopo::SetTopoExtendInfo(std::vector<BevLaneInfo> &vct_bev_lane_info) {
  bev_lane_ids_set_.clear();

  for (auto &lane : vct_bev_lane_info) {
    if (!lane.line_points.empty()) {
      bev_lane_ids_set_[lane.id] = &lane;
    }
  }

  GetEgoLaneId(vct_bev_lane_info);
  if (ego_lane_id_ == INVALID_LANE_ID) {
    AINFO << "No ego lane found";
    return;
  }

  split_topo_list_.clear();
  merge_topo_list_.clear();

  for (auto &lane : vct_bev_lane_info) {
    if (lane.next_lane_ids.size() > 1) {
      std::vector<uint32_t> valid_next_ids;
      for (auto id : lane.next_lane_ids) {
        if (bev_lane_ids_set_.count(id)) valid_next_ids.push_back(id);
      }
      if (valid_next_ids.size() > 1) {
        split_topo_list_[lane.id] = std::move(valid_next_ids);
      }
    }

    if (lane.previous_lane_ids.size() > 1) {
      std::vector<uint32_t> valid_prev_ids;
      for (auto id : lane.previous_lane_ids) {
        if (bev_lane_ids_set_.count(id)) valid_prev_ids.push_back(id);
      }
      if (valid_prev_ids.size() > 1) {
        merge_topo_list_[lane.id] = std::move(valid_prev_ids);
      }
    }
  }

  // merge info assignment
  for (auto &lane : vct_bev_lane_info) {
    if (lane.merge_topo_extend != MergeTopoExtendType::TOPOLOGY_MERGE_NONE) {
      continue;
    }
    switch (lane.connect_type) {
      case BevLaneConnectType::NORMAL:
        lane.merge_topo_extend              = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
        lane.merge_info_extend.merge_valid  = 0;
        lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_BEV;
        lane.merge_info_extend.dis_to_merge = 0.0;
        break;
      case BevLaneConnectType::MERGE:
        lane.merge_topo_extend = MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN;
        break;
      default:
        break;
    }
  }

  // split info assignment
  for (auto &lane : vct_bev_lane_info) {
    if (lane.split_topo_extend != SplitTopoExtendType::TOPOLOGY_SPLIT_NONE) {
      continue;
    }
    switch (lane.connect_type) {
      case BevLaneConnectType::NORMAL:
        lane.split_topo_extend              = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
        break;
      case BevLaneConnectType::SPLIT:
        lane.split_topo_extend = SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN;
        break;
      default:
        break;
    }
  }

#ifdef ENABLE_DEBUG_LANE_TOPOLOGY2
  AINFO << "split_topo_list_ size: " << split_topo_list_.size();
  for (const auto &entry : split_topo_list_) {
    AINFO << "  Node ID: " << entry.first << " split lane IDs size: " << entry.second.size();
    std::string tmp = "";
    for (const auto &lane_id : entry.second) {
      tmp += std::to_string(lane_id);
      tmp += ", ";
    }
    AINFO << "    Split Lane IDs: " << tmp;
  }

  AINFO << "merge_topo_list_ size: " << merge_topo_list_.size();
  for (const auto &entry : merge_topo_list_) {
    AINFO << "  Node ID: " << entry.first << " merge lane IDs size: " << entry.second.size();
    std::string tmp = "";
    for (const auto &lane_id : entry.second) {
      tmp += std::to_string(lane_id);
      tmp += ", ";
    }
    AINFO << "    Merge Lane IDs: " << tmp;
  }
#endif

    // case1: 二分三，处理多个split有共用lane
    //hxg UpdateSplitTopoList();
    // case2: 三分二，处理多个merge有共用lane
    //hxg UpdateMergeTopoList();

  // 处理 split 拓扑
  for (const auto &[lane_id, next_ids] : split_topo_list_) {
    if (next_ids.size() != 2) continue;

    auto it1 = bev_lane_ids_set_.find(next_ids[0]);
    auto it2 = bev_lane_ids_set_.find(next_ids[1]);
    if (it1 == bev_lane_ids_set_.end() || it2 == bev_lane_ids_set_.end()) continue;

    auto &lane1 = *it1->second;
    auto &lane2 = *it2->second;

    auto geos1 = LinePointsVector2f(lane1.line_points);
    auto geos2 = LinePointsVector2f(lane2.line_points);
    bool is_left = LaneGeometry::JudgeIsLeft(geos1, geos2);

    if (lane1.connect_type == BevLaneConnectType::SPLIT) {
      lane1.split_topo_extend = is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT
                                        : SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT;
    } else if (lane2.connect_type == BevLaneConnectType::SPLIT) {
      lane2.split_topo_extend = is_left ? SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT
                                        : SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT;
    }
  }

  // 处理 merge 拓扑
  for (const auto &[lane_id, prev_ids] : merge_topo_list_) {
    if (prev_ids.size() != 2) continue;

    auto it1 = bev_lane_ids_set_.find(prev_ids[0]);
    auto it2 = bev_lane_ids_set_.find(prev_ids[1]);
    if (it1 == bev_lane_ids_set_.end() || it2 == bev_lane_ids_set_.end()) continue;

    auto geos1 = LinePointsVector2f(it1->second->line_points);
    auto geos2 = LinePointsVector2f(it2->second->line_points);
    bool is_left = LaneGeometry::JudgeIsLeft(geos1, geos2);

    auto apply_merge_info = [&](BevLaneInfo &lane, bool left, float dis) {
    //   if (dis > 40) {
    //     lane.merge_topo_extend              = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
    //     lane.merge_info_extend.merge_valid  = 0;
    //     lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_BEV;
    //     lane.merge_info_extend.dis_to_merge = 0.0;
    //     return;
    //   }
      lane.merge_topo_extend              = left ? MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT
                                                 : MergeTopoExtendType::TOPOLOGY_MERGE_LEFT;
      lane.merge_info_extend.merge_valid  = 1;
      lane.merge_info_extend.merge_source = MergeSourceExtend::MERGE_BEV;
      lane.merge_info_extend.dis_to_merge = dis;
    };

    if (it1->second->connect_type == BevLaneConnectType::MERGE) {
      apply_merge_info(*it1->second, is_left, geos1.back().x());
    } else if (it2->second->connect_type == BevLaneConnectType::MERGE) {
      apply_merge_info(*it2->second, !is_left, geos2.back().x());
    }
  }
#ifdef ENABLE_DEBUG_LANE_TOPOLOGY
  for (auto &lane_tmp : vct_bev_lane_info) {
    if (lane_tmp.next_lane_ids.size() >= 1) {
      AINFO << "After lane id is: " << lane_tmp.id << " next_lane_id size: " << lane_tmp.next_lane_ids.size();
      std::string tmp = "";
      for (const auto &next_id : lane_tmp.next_lane_ids) {
        tmp += std::to_string(next_id);
        tmp += ", ";
      }
      AINFO << "  Next Lane IDs: " << tmp;
    }
    if (lane_tmp.previous_lane_ids.size() >= 1) {
      AINFO << "After lane id is: " << lane_tmp.id << " previous_lane_ids size: " << lane_tmp.previous_lane_ids.size();
      std::string tmp = "";
      for (const auto &prev_id : lane_tmp.previous_lane_ids) {
        tmp += std::to_string(prev_id);
        tmp += ", ";
      }
      AINFO << "  Previous Lane IDs: " << tmp;
    }
  }
#endif

#ifdef ENABLE_DEBUG_LANE_TOPOLOGY
  AINFO << "split_topo_list_ size: " << split_topo_list_.size();
  for (const auto &entry : split_topo_list_) {
    AINFO << "  Node ID: " << entry.first << " split lane IDs size: " << entry.second.size();
    std::string tmp = "";
    for (const auto &lane_id : entry.second) {
      tmp += std::to_string(lane_id);
      tmp += ", ";
    }
    AINFO << "    Split Lane IDs: " << tmp;
  }

  AINFO << "merge_topo_list_ size: " << merge_topo_list_.size();
  for (const auto &entry : merge_topo_list_) {
    AINFO << "  Node ID: " << entry.first << " merge lane IDs size: " << entry.second.size();
    std::string tmp = "";
    for (const auto &lane_id : entry.second) {
      tmp += std::to_string(lane_id);
      tmp += ", ";
    }
    AINFO << "    Merge Lane IDs: " << tmp;
  }
#endif
}

void ProcessLanesTopo::IsTraverseCrossWalk(
    std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>>& lane_geos,
    std::map<uint64_t, std::vector<Eigen::Vector2f>>& cross_walk_geos,
    const std::vector<BevLaneInfo*>& far_break_bev_lane,
    const std::vector<BevLaneInfo*>& near_break_bev_lane,
    std::set<uint64_t>& traver_ids)
{
    traver_ids.clear();

    using LanePair = std::tuple<uint64_t, uint64_t, uint64_t>;  // far_id, near_id, crosswalk_id
    std::vector<LanePair> crossing_lane_pairs;

    std::pair<uint64_t, uint64_t> ego_crosswalk_id{};

    // 预排序 crosswalk Y 坐标，提升效率
    std::map<uint64_t, std::pair<float, float>> crosswalk_y_bounds;
    std::map<uint64_t, std::pair<float, float>> crosswalk_x_bounds;
    for (const auto& [id, pts] : cross_walk_geos) {
        auto minmax_x = std::minmax_element(pts.begin(), pts.end(),
            [](const auto& a, const auto& b) { return a.x() < b.x(); });
        auto minmax_y = std::minmax_element(pts.begin(), pts.end(),
            [](const auto& a, const auto& b) { return a.y() < b.y(); });

        crosswalk_x_bounds[id] = {minmax_x.first->x(), minmax_x.second->x()};
        crosswalk_y_bounds[id] = {minmax_y.first->y(), minmax_y.second->y()};
    }

    auto IsLaneCrossing = [&](const Eigen::Vector2f& start_pt, const Eigen::Vector2f& end_pt,
                              float cw_x_min, float cw_x_max, float cw_y_min, float cw_y_max) -> bool {
        return (end_pt.x() >= cw_x_max && start_pt.x() <= cw_x_min) &&
               (start_pt.y() >= cw_y_min && start_pt.y() <= cw_y_max) &&
               (end_pt.y()   >= cw_y_min && end_pt.y()   <= cw_y_max);
    };

    // ① 判断 ego 车道是否在某个 crosswalk 上
    if (lane_geos.count(ego_lane_id_)) {
        const auto& ego_pts = lane_geos[ego_lane_id_];
        const auto& start_pt = ego_pts.front();
        const auto& end_pt = ego_pts.back();

        for (const auto& [id, _] : cross_walk_geos) {
            const auto& [x_min, x_max] = crosswalk_x_bounds[id];
            const auto& [y_min, y_max] = crosswalk_y_bounds[id];

            if (end_pt.x() >= x_max && start_pt.x() <= x_min &&
                y_min < 0 && y_max > 0) {
                ego_crosswalk_id = {ego_lane_id_, id};
                break;
            }
        }
    }

    // ② 遍历所有 (far, near) 配对，判断是否一起穿过某个 crosswalk
    for (const auto* f_lane : far_break_bev_lane) {
        for (const auto* n_lane : near_break_bev_lane) {
            if (!lane_geos.count(f_lane->id) || !lane_geos.count(n_lane->id)) continue;

            const auto& f_start = lane_geos[f_lane->id].front();
            const auto& n_end = lane_geos[n_lane->id].back();

            for (const auto& [cw_id, _] : cross_walk_geos) {
                const auto& [x_min, x_max] = crosswalk_x_bounds[cw_id];
                const auto& [y_min, y_max] = crosswalk_y_bounds[cw_id];

                if (IsLaneCrossing(f_start, n_end, x_min, x_max, y_min, y_max)) {
                    crossing_lane_pairs.emplace_back(f_lane->id, n_lane->id, cw_id);

                    if (f_lane->id == ego_lane_id_ || n_lane->id == ego_lane_id_) {
                        ego_crosswalk_id = {ego_lane_id_, cw_id};
                    }
                }
            }
        }
    }

    // ③ 只记录穿越 ego 所在 crosswalk 的车道对
    if (ego_crosswalk_id.first && ego_crosswalk_id.second) {
        for (const auto& [far_id, near_id, cw_id] : crossing_lane_pairs) {
            if (cw_id == ego_crosswalk_id.second) {
                traver_ids.insert(far_id);
                traver_ids.insert(near_id);
            }
        }
    }
}

void ProcessLanesTopo::SortBevLaneInfo(std::vector<cem::message::sensor::BevLaneInfo *> &bev_lane_infos_in,
                                       std::vector<LineSort> &line_sorts)
{
    line_sorts.clear();

    std::map<uint32_t, std::pair<Vec2fVector, LaneGeometry::PolynomialFitting>> laneinfo_geo_map;

    for (auto *bev_lane_info : bev_lane_infos_in) {
        if (!bev_lane_info || !bev_lane_info->geos || bev_lane_info->geos->empty()) continue;

        auto geos = bev_lane_info->geos;
        std::vector<double> geo_x, geo_y;
        for (const auto &pt : *geos) {
            geo_x.push_back(pt.x());
            geo_y.push_back(pt.y());
        }

        // 尝试将前向车道合并
        if (!geo_x.empty() && geo_x.front() > 80 && std::fabs(geo_x.front() - geo_x.back()) < 60) {
            std::vector<uint64_t> sorted_prev_ids(bev_lane_info->previous_lane_ids.begin(),bev_lane_info->previous_lane_ids.end());
            std::sort(sorted_prev_ids.begin(), sorted_prev_ids.end(), [](uint64_t a, uint64_t b) {
                auto lane_a = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(a);
                auto lane_b = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(b);
                if (lane_a && lane_b && lane_a->geos && lane_b->geos &&
                    !lane_a->geos->empty() && !lane_b->geos->empty()) {
                    double len_a = std::fabs(lane_a->geos->front().x() - lane_a->geos->back().x());
                    double len_b = std::fabs(lane_b->geos->front().x() - lane_b->geos->back().x());
                    return len_a < len_b;
                }
                return false;
            });

            if (!sorted_prev_ids.empty()) {
                auto *prev_lane = INTERNAL_PARAMS.raw_bev_data.GetLaneInfoById(sorted_prev_ids.back());
                if (prev_lane && prev_lane->geos && !prev_lane->geos->empty()) {
                    auto merged_geos = std::make_shared<std::vector<Eigen::Vector2f>>();
                    merged_geos->insert(merged_geos->end(), prev_lane->geos->begin(), prev_lane->geos->end());
                    merged_geos->insert(merged_geos->end(), geos->begin(), geos->end());
                    geos = merged_geos;

                    for (const auto &pt : *prev_lane->geos) {
                        geo_x.push_back(pt.x());
                        geo_y.push_back(pt.y());
                    }
                }
            }
        }

        if (geo_x.size() < 5 || (geo_x.front() > 100 && std::fabs(geo_x.back() - geo_x.front()) < 30)) {
            laneinfo_geo_map[bev_lane_info->id] = {geos, LaneGeometry::PolynomialFitting(geo_x, geo_y, 1)};
        } else {
            laneinfo_geo_map[bev_lane_info->id] = {geos, LaneGeometry::PolynomialFitting(geo_x, geo_y, 3)};
        }

        line_sorts.emplace_back(bev_lane_info->id, false); // false = 非road edge
    }

    if (laneinfo_geo_map.empty()) return;

    std::sort(line_sorts.begin(), line_sorts.end(), [&](const auto &l1, const auto &l2) {
        const auto &entry1 = laneinfo_geo_map[l1.id];
        const auto &entry2 = laneinfo_geo_map[l2.id];
        return LaneGeometry::JudgeIsLeft(*entry1.first, *entry2.first, entry1.second, entry2.second);
    });
}

std::vector<Eigen::Vector2f> ProcessLanesTopo::LinePointsVector2f(const std::vector<Point2DF> &lm_points) {
  std::vector<Eigen::Vector2f> geos;
  geos.reserve(lm_points.size());
  for (const auto &pt : lm_points) {
    geos.emplace_back(pt.x, pt.y);
  }
  return geos;
}

void ProcessLanesTopo::CutLaneTail(BevLaneInfo *lane, const float &overlap) {
    if (lane->line_points.size() < 2 || overlap <= 0.0f) {
        return;  // 无法剪裁
    }

    auto  original_points = lane->line_points;
    float tail_x          = lane->line_points.back().x;
    float head_x          = lane->line_points.front().x;
    bool  is_increasing   = (tail_x > head_x);

    // 1. 找到满足 x 差异 > overlap 且保留至少 2 个点的位置
    int new_size = lane->line_points.size();
    while (new_size > 2) {
        float new_tail_x = lane->line_points[new_size - 1].x;
        float delta      = tail_x - new_tail_x;

        // 检查方向是否符合预期
        if ((is_increasing && delta < 0) || (!is_increasing && delta > 0)) {
            lane->line_points = original_points; // 异常，恢复
            return;
        }

        if ((is_increasing && delta > overlap) || (!is_increasing && -delta > overlap)) {
            lane->line_points.resize(new_size);
            return; // 剪裁完成
        }
        --new_size;
    }

    // 2. 尝试修改尾点
    if (lane->line_points.size() >= 2) {
        float new_tail_x = tail_x + (is_increasing ? -overlap : overlap);
        lane->line_points.back().x = new_tail_x;

        float delta = tail_x - new_tail_x;
        if ((is_increasing && delta > overlap) || (!is_increasing && -delta > overlap)) {
            return; // 成功修改尾点
        }
    }

    // 3. 无法满足，恢复
    lane->line_points = original_points;
}

void ProcessLanesTopo::SetBrokenTopoInfo(BevMapInfoPtr &bevMapPtrInput) {
  if (!bevMapPtrInput) {
    return;
  }

  auto IsTurningRight = [&](const BevLaneInfo &lane) -> bool {
    const auto &pts = lane.line_points;
    if (pts.size() < 2) {
      return false;  // 点数不足
    }

    // Step 1: 从末尾截取最多10个点
    int                          start_idx = std::max(0, static_cast<int>(pts.size()) - 10);
    std::vector<Point2DF> local_pts;
    local_pts.reserve(10);

    for (int i = start_idx; i < pts.size(); ++i) {
      // 只保留在±20m范围内的点
      if (std::abs(pts[i].x) <= 20.0f && std::abs(pts[i].y) <= 20.0f) {
        local_pts.push_back(pts[i]);
      }
    }

    // Step 2: 检查有效点数
    if (local_pts.size() < 4) {
      return false;  // 有效点数不足
    }

    // Step 3: 选取首点、中点、尾点
    int mid_idx = local_pts.size() / 2;  // 中间点索引

    const Eigen::Vector2f &p_start{local_pts[0].x, local_pts[0].y};
    const Eigen::Vector2f &p_mid{local_pts[mid_idx].x, local_pts[mid_idx].y};
    const Eigen::Vector2f &p_end{local_pts.back().x, local_pts.back().y};

    // Step 4: 构建两个矢量
    Eigen::Vector2f v1 = p_mid - p_start;  // 矢量1: 首点 -> 中点
    Eigen::Vector2f v2 = p_end - p_mid;    // 矢量2: 中点 -> 尾点

    // Step 5: 计算向量长度，防止除零
    float norm1 = v1.norm();
    float norm2 = v2.norm();
    if (norm1 < 1e-6f || norm2 < 1e-6f) {
      return false;  // 向量太短，无法判断
    }

    // Step 6: 计算叉积判断转向
    // 2D叉积: cross(v1, v2) = v1.x * v2.y - v1.y * v2.x
    float cross_product = v1.x() * v2.y() - v1.y() * v2.x();

    // 叉积为正表示v2相对于v1是逆时针旋转（左转）
    // 叉积为负表示v2相对于v1是顺时针旋转（右转）
    // 叉积绝对值 = |v1||v2|sinθ

    // Step 7: 计算夹角sin值
    float sin_theta = cross_product / (norm1 * norm2);

    // Step 8: 判断是否右转超过5度
    const float sin_threshold = 0.1f;  // 6度的sin值
    // 右转条件: 
    return sin_theta < -sin_threshold;
  };

  //右转状态位
  if (cross_road_status_){
    cross_turn_right_ = false;
    // right_next_lane_ids_.clear();
  } else {
    if (!cross_turn_right_) {
      auto ego_itr = std::find_if(bevMapPtrInput->lane_infos.begin(), bevMapPtrInput->lane_infos.end(),
                                  [&](const BevLaneInfo &lane) { return lane.id == ego_lane_id_; });
      if (ego_itr != bevMapPtrInput->lane_infos.end() && IsTurningRight(*ego_itr)) {
        cross_turn_right_ = true;
        if (DEBUG_INFO)
          AINFO << "[IsTurningRight] ego_lane_id_ = " << ego_lane_id_;
      }
    }
    else{
      if (ego_lane_id_ != last_ego_lane_id_ && !right_next_lane_ids_.count(ego_lane_id_) && ego_lane_id_ < 10000) {
        AINFO <<"ego_lane_id_ = "<< ego_lane_id_ << " last_ego_lane_id_ = " << last_ego_lane_id_;
        cross_turn_right_ = false;
        right_next_lane_ids_.clear();
      }
    }
  }
  if (!cross_turn_right_ || last_ego_lane_id_ == 0)
    last_ego_lane_id_ = ego_lane_id_;

  std::unordered_set<BevLaneInfo *>                          far_set;
  std::unordered_set<BevLaneInfo *>                          near_set;
  std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>> lane_geos;
  std::unordered_map<uint64_t, bool>                         is_valid_map;

  auto get_lane_geos = [&](const BevLaneInfo &lane) -> const std::vector<Eigen::Vector2f> & {
    auto iter = lane_geos.find(lane.id);
    if (iter != lane_geos.end()) {
      return iter->second;
    }

    // 新建几何
    lane_geos[lane.id] = LinePointsVector2f(lane.line_points);
    return lane_geos[lane.id];
  };

  auto check_valid_if_needed = [&](const BevLaneInfo &lane) -> bool {
    auto iter = is_valid_map.find(lane.id);
    if (iter != is_valid_map.end()) {
      return iter->second;
    }

    bool is_valid =
        lane.id && lane.line_points.size() >= 3 &&
        lane.direction !=
            cem::message::sensor::BevLaneDirection::DIRECTION_BACKWARD;  //(lane.next_lane_ids.empty() || lane.previous_lane_ids.empty()) &&

    if (is_valid) {
      if (!cross_turn_right_ && lane.line_points.front().x > (cross_road_distance_ - 20) && cross_road_status_ && cross_road_distance_ > -1) {
        if (DEBUG_INFO)
          AINFO << "cross_status_true";
        is_valid = false;
      }
    }
    if (DEBUG_INFO)
      AINFO << "lane id: " << lane.id << " front pnt:" << lane.line_points.front().x;

    if (!is_valid && DEBUG_INFO) {
      AWARN << "lane id: " << lane.id << " is invalid.lane length: " << lane.line_points.size()
            << " lane direction: " << static_cast<int>(lane.direction) << " next lane size: " << lane.next_lane_ids.size()
            << " previous lane size: " << lane.previous_lane_ids.size();
      for(const auto&next: lane.next_lane_ids) {
        AWARN << "next lane id: " << next;
      }
      for(const auto&pre: lane.previous_lane_ids) {
        AWARN << "previous lane id: " << pre;
      }
    }

    is_valid_map[lane.id] = is_valid;
    return is_valid;
  };

  auto CleanRedundantEdgesAndUpdateInfo = [&]() {  // 根据原有 lane_infos 的关联关系添加边
    for (auto &lane : bevMapPtrInput->lane_infos) {
      lane.next_lane_ids.clear();
      lane.previous_lane_ids.clear();
    }
    // 去除 A->C 如果已存在 A->B->C
    // 去除 A->D 如果已存在 A->B->C->D
    for (auto &[from, tos] : topo_map_) {
      std::unordered_set<uint64_t> indirect_reachable;

      for (auto to : tos) {
        if (topo_map_.count(to)) {
          for (auto child : topo_map_[to]) {
            indirect_reachable.insert(child);

            if (topo_map_.count(child)) {
              for (auto grandchild : topo_map_[child]) {
                indirect_reachable.insert(grandchild);
              }
            }
          }
        }
      }

      // 删除直接连接到间接可达节点的边
      std::unordered_set<uint64_t> filtered;
      for (auto to : tos) {
        if (indirect_reachable.count(to) == 0) {
          filtered.insert(to);
        }
      }
      tos = std::vector<uint64_t>(filtered.begin(), filtered.end());
    }

    for (const auto &[from, tos] : topo_map_) {
      auto f_it = std::find_if(bevMapPtrInput->lane_infos.begin(), bevMapPtrInput->lane_infos.end(),
                               [from](const BevLaneInfo &l) { return l.id == from; });

      for (auto to : tos) {
        auto t_it = std::find_if(bevMapPtrInput->lane_infos.begin(), bevMapPtrInput->lane_infos.end(),
                                 [to](const BevLaneInfo &l) { return l.id == to; });

        if (f_it != bevMapPtrInput->lane_infos.end()) {
          f_it->next_lane_ids.emplace_back(to);
        }
        if (t_it != bevMapPtrInput->lane_infos.end()) {
          t_it->previous_lane_ids.emplace_back(from);
        }
      }
    }

    if (DEBUG_INFO) {
      AINFO << "AFTER";
      PrintAllPaths();
    }
  };

  if (DEBUG_INFO)
    AINFO << "cross_road_status_:" << cross_road_status_ << " cross_road_distance_:" << cross_road_distance_
          << ",adc2junction:" << adc_to_junction_ << ",turn right:" << cross_turn_right_;

  topo_map_.clear();
  topo_add_nodes_.clear();
  for (auto &lane : bevMapPtrInput->lane_infos) {
    for (auto to : lane.next_lane_ids) {  // 这里用的是原数据来源
      AddEdge(lane.id, to);
    }
  }
  if (DEBUG_INFO) {
    AINFO << "BEFORE";
    PrintAllPaths();
  }

  std::map<uint64_t, std::vector<Eigen::Vector2f>> cross_walk_geos;
  for (auto &lane_marker : bevMapPtrInput->crosswalks) {
    auto cwgeos = LinePointsVector2f(lane_marker.line_points);
    if (!cwgeos.empty()) {
      std::sort(cwgeos.begin(), cwgeos.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });
      cross_walk_geos.insert({lane_marker.id, cwgeos});
    }
  }
  for (auto &lane_marker : bevMapPtrInput->junctions) {
    auto cwgeos = LinePointsVector2f(lane_marker.line_points);
    if (!cwgeos.empty()) {
      std::sort(cwgeos.begin(), cwgeos.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });
      cross_walk_geos.insert({lane_marker.id, cwgeos});
    }
  }

  auto IsTurningLane = [&](const BevLaneInfo &laneB) -> bool {
    const auto &pts = laneB.line_points;
    if (pts.size() < 2) {
      return false;  // 点数不足
    }

    //仅限制右拐
    if (pts[pts.size() - 1].y - pts[0].y > 0)
      return false;

    // 向量1: 最末尾两点
    Eigen::Vector2f v1(pts[pts.size() - 1].x - pts[pts.size() - 2].x, pts[pts.size() - 1].y - pts[pts.size() - 2].y);

    // 向量2: 倒数第10点 → 倒数第9点（或首两点）
    Eigen::Vector2f v2;
    if (pts.size() > 10) {
      v2 = Eigen::Vector2f(pts[pts.size() - 10].x - pts[pts.size() - 11].x, pts[pts.size() - 10].y - pts[pts.size() - 11].y);
    } else {
      v2 = Eigen::Vector2f(pts[1].x - pts[0].x, pts[1].y - pts[0].y);
    }

    // 长度检查，防止除零
    float norm1 = v1.norm();
    float norm2 = v2.norm();
    if (norm1 < 1e-6f || norm2 < 1e-6f) {
      return false;  // 无法判断
    }

    // cos(theta) = (v1·v2) / (|v1||v2|)
    float cos_theta = v1.dot(v2) / (norm1 * norm2);

    // cos(20°) ≈ 0.9396926
    const float cos_threshold = 0.98;

    return cos_theta < cos_threshold;  // true 表示转弯
  };

  uint64_t id_far = 34,id_near = 88;
  for (auto &laneA : bevMapPtrInput->lane_infos) {
    //检查右转后继有没有pass的
    if (cross_turn_right_) {
      if (right_next_lane_ids_.count(laneA.id) && (laneA.line_points.empty() || laneA.line_points[0].x < -5)) {
        AINFO <<"erase right_next_lane_ids_"<< laneA.id;
        right_next_lane_ids_.erase(laneA.id);
      }
    }

    if (!check_valid_if_needed(laneA)) {
      continue;
    }

    const auto &geoA = get_lane_geos(laneA);
    float startA_x = geoA.front().x();

    for (auto &laneB : bevMapPtrInput->lane_infos) {
      if (laneA.id == laneB.id)
        continue;
       if (DEBUG_INFO&&laneA.id == id_far && laneB.id == id_near)AINFO<<"@@@@@@:";

      if (!check_valid_if_needed(laneB) || IsTurningLane(laneB) || (!laneB.next_lane_ids.empty()))
        continue;

      const auto &geoB   = get_lane_geos(laneB);
      float       overlap = geoB.back().x() - startA_x;
       if (DEBUG_INFO&&laneA.id == id_far && laneB.id == id_near)AINFO<<"@@@@@@overlap:"<<overlap;

       if (overlap < 7.0f && overlap > -50.0f) {
         float dist = LaneGeometry::GetYDistanceBetweenLines(geoB, geoA);
         if (DEBUG_INFO && laneA.id == id_far && laneB.id == id_near)
           AINFO << "@@@@@@dist:" << dist<<" cross_turn_right_:"<<cross_turn_right_<<" direct_dist:"<<(geoA.front() - geoB.back()).norm();

         // 满足拓扑条件，建立拓扑关系
         if ((!cross_turn_right_ && dist < 3.0f) ||
             (cross_turn_right_ && (geoA.front() - geoB.back()).norm() < 20.f && geoB.back().x() < 15.f && 
             (laneB.id == ego_lane_id_||right_next_lane_ids_.count(laneA.id)))) {
           if (!cross_turn_right_ && adc_to_junction_ < 50) // && IsSegmentNearAnyPolygon(geoB.back(), geoA.front(), cross_walk_geos, 5.0f)
           {
             // 线段ab经过至少一个多边形的附近
             AINFO << "id_far id:" << laneA.id << " id_near id:" << laneB.id << " cross walk";
             continue;
           }
           if (DEBUG_INFO && laneA.id == id_far && laneB.id == id_near)
             AINFO << "@@@@@@:" << geoB.back() << " to:" << geoA.front();

           if (overlap > 0.0f) {
             overlap += dist;  //隔得越远剪裁得越多
             if (DEBUG_INFO && laneA.id == id_far && laneB.id == id_near)
               AINFO << "@@@@@@overlap:" << overlap;
             CutLaneTail(&laneB, overlap);
           }
           if (DEBUG_INFO && laneA.id == id_far && laneB.id == id_near)
             AINFO << "@@@@@@dist:" << dist;
           if (cross_turn_right_) {
             right_next_lane_ids_.insert(laneA.id);
           }
           AddEdge(laneB.id, laneA.id);
           far_set.insert(&laneA);
           near_set.insert(&laneB);
         }
       }
    }
  }

  std::vector<BevLaneInfo *> near_break_bev_lane(near_set.begin(), near_set.end());
  std::vector<BevLaneInfo *> far_break_bev_lane(far_set.begin(), far_set.end());

  // 获取横穿斑马线的车道ID:判断是否存在一对车道（一个远处车道 + 一个近处车道）共同穿过了同一个人行横道区域，并记录它们的 ID 到 traver_ids 中。
  std::set<uint64_t> traver_ids;
  IsTraverseCrossWalk(lane_geos, cross_walk_geos, far_break_bev_lane, near_break_bev_lane, traver_ids);

  if (far_break_bev_lane.empty() || near_break_bev_lane.empty()) {
    // AINFO << " EMPTY!far_break_bev_lane.size():" << far_break_bev_lane.size()
    //       << " near_break_bev_lane.size():" << near_break_bev_lane.size();
    CleanRedundantEdgesAndUpdateInfo();
    return;
  }

  if (DEBUG_INFO) {
    AINFO << "time:" << std::setprecision(15) << bevMapPtrInput->header.timestamp;
    AINFO << "far_break_bev_lane";
    for (const auto &line_sort : far_break_bev_lane) {
      AINFO << "  [id = " << line_sort->id<< "]";
    }
    AINFO << "near_break_bev_lane";
    for (const auto &line_sort : near_break_bev_lane) {
      AINFO << "  [id = " << line_sort->id  << "]";
    }
  }

  //同步到 BevLaneInfo 的前后继
  CleanRedundantEdgesAndUpdateInfo();

  // ----------------------------- 补充拓扑中穿过斑马线的虚拟车道  --------------------------------------------
  traverse_crosswalk_lane_.clear();
  if (!cross_road_status_) {
    std::unordered_set<uint64_t> far_break_ids;
    for (auto *f_line : far_break_bev_lane) {
      far_break_ids.insert(f_line->id);
    }

    for (const auto &lane_info : bevMapPtrInput->lane_infos) {
      uint64_t lane_id = lane_info.id;

      // 是断开的远端车道并且已被遍历过
      if (far_break_ids.count(lane_id) && traver_ids.count(lane_id) && !lane_info.previous_lane_ids.empty()) {
        for (uint64_t prev_id : lane_info.previous_lane_ids) {
          traverse_crosswalk_lane_.emplace_back(
              traverseCrossWalkLane{.far_lane_id = lane_id, .near_lane_id = prev_id, .cross_walk_id = lane_id, .traverse = true});
        }
      }
    }
  }
}

}  // namespace fusion
}  // namespace cem
