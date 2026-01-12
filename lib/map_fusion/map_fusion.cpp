#include "map_fusion.h"

namespace cem {
namespace fusion {

constexpr bool DEBUG_INFO = false;

MapFusion::MapFusion():new_id_(0)
{

}

void MapFusion::Process(const BevMapInfoPtr &bev_map_ptr, RoutingMapPtr &routing_map_ptr)
{
  if (bev_map_ptr == nullptr || bev_ego_lane_id_ == 0 || ld_ego_lane_ids_.empty() || routing_map_ptr == nullptr) {
    if (DEBUG_INFO)
      AINFO << "input ptr is null";
    return;
  }

  BevMapInfo bev_map = *bev_map_ptr;

  // 将bev_map arrows 写入 routing_map中
  UpdateRoutingMapArrows(bev_map_ptr, routing_map_ptr);
  if (DEBUG_INFO) {
    AINFO << "sequence_num:" << bev_map.header.cycle_counter;
    AINFO << "bev ego lane id:" << bev_ego_lane_id_;
    AINFO << "ld ego lane ids:";
    for (auto &it : ld_ego_lane_ids_) {
      AINFO << it;
    }
  }

  // 对于bev_lane marker linepoints染色
  AssignColor2LaneMarkerPts(bev_map);

  // unknown颜色预处理逻辑，复用之前设计逻辑。
  AssignLaneMarkerColor(bev_map);

  UseBevLmAttrTypeInsteadOfHdmap(bev_map, routing_map_ptr);
  // bev_ego_lane_id_ = 0;
  // ld_ego_lane_ids_.clear();
}

void MapFusion::PrintBevLaneMarker(const BevLaneMarker& marker) {
    if (marker.number_of_points<1)
    {
        return;
    }
    // if(marker.id != 77)return;
    AINFO << "BevLaneMarker {";
    AINFO << "  id: " << marker.id << "";
    AINFO << "  number_of_points: " << static_cast<int>(marker.number_of_points) << "";
    AINFO << "  position: " << static_cast<int>(marker.position) << "";
    AINFO << "  type: " << marker.type << "";
    AINFO << "  color: " << static_cast<int>(marker.color) << "";
    AINFO << "  conf: " << marker.conf << "";
    AINFO << "  is_virtual: " << std::boolalpha << marker.is_virtual << "";
    AINFO << "  road_edeg_pos: " << static_cast<int>(marker.road_edeg_pos) << "";

    AINFO << "  line_points (" << marker.line_points.size() << "):";
    AINFO << "  from: (" << marker.line_points.front().x << ", " << marker.line_points.front().y << ")";
    AINFO << "  to: (" << marker.line_points.back().x << ", " << marker.line_points.back().y << ")";

    AINFO << "  type_segs (" << marker.type_segs.size() << "):";
    for (const auto& seg : marker.type_segs) {
        AINFO << "    { type: " << static_cast<int>(seg.type)
                  << ", start_offset: " << seg.start_offset
                  << ", end_offset: " << seg.end_offset
                  << ", start_index: " << seg.start_index
                  << ", end_index: " << seg.end_index << " }";
    }
    AINFO << "}";
}
void MapFusion::PrintLaneBoundaryInfo(const LaneBoundaryInfo& info) {
  if (info.points.size() < 1) {
    return;
  }

  AINFO << "LaneBoundaryInfo:" ;
  AINFO << "  id        : " << info.id ;
  AINFO << "  line_type : " << static_cast<int>(info.line_type) ;
  // AINFO << "  points (" << info.points.size() << "):" ;

  AINFO << "  from: (" << info.points.front().x << ", " << info.points.front().y << ")";
  AINFO << "  to: (" << info.points.back().x << ", " << info.points.back().y << ")";
}

void MapFusion::UseBevLmAttrTypeInsteadOfHdmap(const BevMapInfo &bev_map, RoutingMapPtr routing_map_ptr) {
  new_id_ = 0;

  auto ego_itr = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
                              [this](const BevLaneInfo &lane) {  // 捕获this
                                return lane.id == this->bev_ego_lane_id_;
                              });
  if (ego_itr == bev_map.lane_infos.end()) {
    if (DEBUG_INFO)AINFO << "ego not found:"<< bev_ego_lane_id_;
    return;
  }
  std::vector<BevLaneInfo> ego_lanes;
  ego_lanes.push_back(*ego_itr);
  int cnt = 3;
  auto lane_itr = ego_itr;
  while (cnt > 0) {
    if (lane_itr->next_lane_ids.size() == 1) {
      auto id = lane_itr->next_lane_ids[0];
      lane_itr =
          std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(), [&](const BevLaneInfo &lane) { return lane.id == id; });
      if (lane_itr == bev_map.lane_infos.end())
        break;
      ego_lanes.push_back(*lane_itr);
      if (DEBUG_INFO)
        AINFO << "bev ego lane id:" << lane_itr->id;
    }
    else {
      break;
    }
    cnt--;
  }

  for (const auto bev_ego : ego_lanes) {
    // 左侧处理
    UpdateLaneBoundaryBySide(bev_ego.left_lane_marker_id, bev_map.lanemarkers, routing_map_ptr->lane_boundaries, routing_map_ptr->lanes,
                             LaneSide::LEFT);

    // 右侧处理
    UpdateLaneBoundaryBySide(bev_ego.right_lane_marker_id, bev_map.lanemarkers, routing_map_ptr->lane_boundaries, routing_map_ptr->lanes,
                             LaneSide::RIGHT);
  }
}

void MapFusion::UpdateLaneBoundaryBySide(uint32_t bev_lane_marker_id,
                               const std::vector<BevLaneMarker> &bev_lane_markers,
                               std::vector<LaneBoundaryInfo> &hdmap_lane_boundaries,
                               std::vector<LaneInfo> &hdmap_lanes,
                               const LaneSide &side)
{
  // 查找 bev marker
  auto bev_itr = std::find_if(
      bev_lane_markers.begin(), bev_lane_markers.end(),
      [bev_lane_marker_id](const BevLaneMarker &m) { return m.id == bev_lane_marker_id; });

  if (bev_itr == bev_lane_markers.end() || bev_itr->is_virtual) {
    if (DEBUG_INFO)AERROR << "Bev marker id = " << bev_lane_marker_id << " not found or is virtual.";
    return;
  }
  const BevLaneMarker *bev_marker = &(*bev_itr);

  // V1 版本，直接对于整个bev_lane相matching的ld lane进行染色
  //假设：bev_lane 的 color_seg 长度为 1， 即一个颜色段对应一个车道线，matching车道线不变色
  LaneMarkerColor lane_color = LaneMarkerColor::LMC_UNKNOWN;
  if (bev_marker->color_segs.size() < 1) {
    AERROR << "Bev marker id " << bev_marker->id << " has no color segment.";
  } else if (bev_marker->color_segs.size() == 1){
     switch (bev_marker->color_segs[0].color)
    {
      case BevLaneMarkerColor::BEV_LMC__WHITE:
          lane_color = LaneMarkerColor::LMC_WHITE;
          break;
      case BevLaneMarkerColor::BEV_LMC__YELLOW:
          lane_color = LaneMarkerColor::LMC_YELLOW;
          break;
      case BevLaneMarkerColor::BEV_LMC__UNKNOWN:
          lane_color = LaneMarkerColor::LMC_UNKNOWN; 
          break;
      case BevLaneMarkerColor::BEV_LMC__BLUE:
          lane_color = LaneMarkerColor::LMC_BLUE;
          break;
      case BevLaneMarkerColor::BEV_LMC__GREEN:
          lane_color = LaneMarkerColor::LMC_GREEN;
          break;
      case BevLaneMarkerColor::BEV_LMC__RED:
          lane_color = LaneMarkerColor::LMC_RED;
          break;
      default:
          lane_color = LaneMarkerColor::LMC_UNKNOWN;
          break;
    }
  } else {
    // AERROR << "Bev marker id = " << bev_marker->id << " has more than one color segment.";
    switch (bev_marker->color_segs[0].color)
    {
      case BevLaneMarkerColor::BEV_LMC__WHITE:
          lane_color = LaneMarkerColor::LMC_WHITE;
          break;
      case BevLaneMarkerColor::BEV_LMC__YELLOW:
          lane_color = LaneMarkerColor::LMC_YELLOW;
          break;
      case BevLaneMarkerColor::BEV_LMC__UNKNOWN:
          lane_color = LaneMarkerColor::LMC_UNKNOWN; 
          break;
      case BevLaneMarkerColor::BEV_LMC__BLUE:
          lane_color = LaneMarkerColor::LMC_BLUE;
          break;
      case BevLaneMarkerColor::BEV_LMC__GREEN:
          lane_color = LaneMarkerColor::LMC_GREEN;
          break;
      case BevLaneMarkerColor::BEV_LMC__RED:
          lane_color = LaneMarkerColor::LMC_RED;
          break;
      default:
          lane_color = LaneMarkerColor::LMC_UNKNOWN;
          break;
    }
  }

  std::vector<LaneInfo> ld_lanes;
  for (const auto ld_id : ld_ego_lane_ids_) {
    auto lane_itr = std::find_if(hdmap_lanes.begin(), hdmap_lanes.end(),
                                [ld_id](const LaneInfo &m) { return m.id == ld_id; });

    if (lane_itr != hdmap_lanes.end() && !lane_itr->is_virtual) {
      ld_lanes.push_back(*lane_itr);
    }
  }

  for (const auto &route_lane : ld_lanes) {
    //检查是否其中一个是virtual
    bool has_virtual = false;
    for (auto map_id : route_lane.left_lane_boundary_ids) {
      auto map_lane = std::find_if(hdmap_lane_boundaries.begin(), hdmap_lane_boundaries.end(),
                                   [map_id](const LaneBoundaryInfo &b) { return b.id == map_id; });
      if (map_lane == hdmap_lane_boundaries.end()) {
        continue;
      }
      if (map_lane->line_type == LineType::VIRTUAL_LANE) {
        has_virtual = true;
        break;
      }
    }
    if (has_virtual) {
      continue;
    }
    for (auto map_id : route_lane.right_lane_boundary_ids) {
      auto map_lane = std::find_if(hdmap_lane_boundaries.begin(), hdmap_lane_boundaries.end(),
                                   [map_id](const LaneBoundaryInfo &b) { return b.id == map_id; });
      if (map_lane == hdmap_lane_boundaries.end()) {
        continue;
      }
      if (map_lane->line_type == LineType::VIRTUAL_LANE) {
        has_virtual = true;
        break;
      }
    }
    if (has_virtual) {
      continue;
    }

    const auto &bd_ids = (side == LaneSide::LEFT) ? route_lane.left_lane_boundary_ids : route_lane.right_lane_boundary_ids;

    if (bd_ids.empty()) {
      AERROR << "Route lane has no " << (side == LaneSide::LEFT ? "left" : "right") << " boundary. lane_id = " << route_lane.id;
      continue;
    }
    //遍历LDmap对应车道一侧的landmarker
    for (const auto &route_bd_id : bd_ids) {
      auto hdmap_itr = std::find_if(
          hdmap_lane_boundaries.begin(), hdmap_lane_boundaries.end(),
          [route_bd_id](const LaneBoundaryInfo &b) { return b.id == route_bd_id; });

      if (hdmap_itr == hdmap_lane_boundaries.end()) {
        // AERROR << "HD map marker id = " << route_bd_id << " not found.";
        continue;
      }

      std::vector<std::unique_ptr<LaneBoundaryInfo>> updated_markers;
      if (DEBUG_INFO) {
        AINFO << "BEFORE!BEV:";
        PrintBevLaneMarker(*bev_marker);
        AINFO << "BEFORE!hdmap:";
        PrintLaneBoundaryInfo(*hdmap_itr);
      }
      // 1v1
      bool success = false;
      try {
        success = ReplaceHdmapAttrSegWithBevAttrSeg(bev_marker, &(*hdmap_itr), updated_markers);
      } catch (...) {
        AERROR << "Unknown exception in ReplaceHdmapAttrSegWithBevAttrSeg"
               << " bev_marker id=" << bev_marker->id << " hdmap_itr id=" << hdmap_itr->id;
        continue;
      }

      if (success && !updated_markers.empty() && updated_markers[0] != nullptr) {  //对应要复原id
        updated_markers[0]->id = hdmap_itr->id;
        *hdmap_itr = *updated_markers[0];  //TODO：这句看是否放在循环外面
      }

      hdmap_itr->line_color = lane_color;
      if (updated_markers.empty()) {
        // AERROR << "No updated markers from bev marker id = " << bev_marker->id;
        continue;
      }

      if (updated_markers.size() > 1) {
        //ld_lanes里只有ID，要去hdmap_lanes里找对应的lane
        auto lane_itr =
            std::find_if(hdmap_lanes.begin(), hdmap_lanes.end(), [&route_lane](const LaneInfo &l) { return l.id == route_lane.id; });

        if (lane_itr == hdmap_lanes.end()) {
          AERROR << "Route lane id " << route_lane.id << " not found in hdmap.";
          continue;
        }

        // bool debug = false;

        auto ori_id = updated_markers[0]->id;
        for (size_t j = 1; j < updated_markers.size(); ++j) {
          auto &updated_marker = updated_markers[j];
          // if (j == 1) {
          //   AINFO << "@@@@@@Before@@@@@@@" << lane_itr->id;
          //   debug = true;
          //   for (size_t i = 0; i < hdmap_lane_boundaries.size(); i++) {
          //     PrintLaneBoundaryInfo(hdmap_lane_boundaries[i]);
          //   }
          // }

          hdmap_lane_boundaries.push_back(*updated_marker);
          if (side == LaneSide::LEFT) {
            lane_itr->left_lane_boundary_ids.push_back(updated_marker->id);
            //相邻车道的车道线拓扑
            auto left_lane_itr = std::find_if(hdmap_lanes.begin(), hdmap_lanes.end(),
                                              [&lane_itr](const LaneInfo &l) { return l.id == lane_itr->left_lane_id; });
            if (left_lane_itr != hdmap_lanes.end()) {
              auto ori_lanemarker_itr =
                  std::find_if(left_lane_itr->right_lane_boundary_ids.begin(), left_lane_itr->right_lane_boundary_ids.end(),
                               [ori_id](const auto &id) { return id == ori_id; });
              if (ori_lanemarker_itr != left_lane_itr->right_lane_boundary_ids.end()) {
                left_lane_itr->right_lane_boundary_ids.push_back(updated_marker->id);
              }
            }
          } else {
            lane_itr->right_lane_boundary_ids.push_back(updated_marker->id);
            //相邻车道的车道线拓扑
            auto right_lane_itr = std::find_if(hdmap_lanes.begin(), hdmap_lanes.end(),
                                               [&lane_itr](const LaneInfo &l) { return l.id == lane_itr->right_lane_id; });
            if (right_lane_itr != hdmap_lanes.end()) {
              auto ori_lanemarker_itr =
                  std::find_if(right_lane_itr->left_lane_boundary_ids.begin(), right_lane_itr->left_lane_boundary_ids.end(),
                               [ori_id](const auto &id) { return id == ori_id; });
              if (ori_lanemarker_itr != right_lane_itr->left_lane_boundary_ids.end()) {
                right_lane_itr->left_lane_boundary_ids.push_back(updated_marker->id);
              }
            }
          }
        }

        // if (debug == true) {
        //   AINFO << "@@@@@@After@@@@@@@" << lane_itr->id;
        //   for (size_t i = 0; i < hdmap_lane_boundaries.size(); i++) {
        //     PrintLaneBoundaryInfo(hdmap_lane_boundaries[i]);
        //   }
        // }
      }

      if (DEBUG_INFO) {
        AINFO << "AFTER!hdmap:";
        for (const auto &updated_marker : updated_markers) {
          PrintLaneBoundaryInfo(*updated_marker);
        }
      }
    }
  }
}

bool MapFusion::FindCutPoint(const std::vector<Point2D>& route_points,
                             const cem::message::common::Point2DF& position,
                             int &best_index,
                             Point2D& point_out) {
  if (route_points.size() < 2) return false;

  // 1. 找最近点
  double min_dist2 = std::numeric_limits<double>::max();
  int nearest_index = -1;
  for (size_t i = 0; i < route_points.size(); ++i) {
    double dx = route_points[i].x - position.x;
    double dy = route_points[i].y - position.y;
    double dist2 = dx*dx + dy*dy;
    if (dist2 < min_dist2) {
      min_dist2 = dist2;
      nearest_index = (int)i;
    }
  }

  // 默认输出最近点
  best_index = nearest_index;
  point_out  = { (float)route_points[nearest_index].x,
                 (float)route_points[nearest_index].y };

  auto project_on_segment = [&](int i, int j,
                                Point2D& proj,
                                double& dist2) -> bool {
    Point2D a = route_points[i], b = route_points[j];
    double vx = b.x - a.x, vy = b.y - a.y;
    double wx = position.x - a.x, wy = position.y - a.y;
    double c1 = vx * wx + vy * wy;
    double c2 = vx * vx + vy * vy;
    if (c2 < 1e-12) return false;
    double t = c1 / c2;
    if (t < 0.0 || t > 1.0) return false;
    proj = { (float)(a.x + t * vx), (float)(a.y + t * vy) };
    double dx = position.x - proj.x, dy = position.y - proj.y;
    dist2 = dx*dx + dy*dy;
    return true;
  };

  double best_d2 = min_dist2;
  Point2D proj;

  bool project_valid = best_d2 < 10*10;
  // 2. 看前一段
  if (nearest_index > 0) {
    double d2;
    if (project_on_segment(nearest_index-1, nearest_index, proj, d2) && d2 < best_d2) {
      best_index = nearest_index - 1; // 左端点索引
      point_out  = proj;
      best_d2    = d2;
      project_valid = true;
    }
  }

  // 3. 看后一段
  if (nearest_index+1 < (int)route_points.size()) {
    double d2;
    if (project_on_segment(nearest_index, nearest_index+1, proj, d2) && d2 < best_d2) {
      best_index = nearest_index; // 左端点索引
      point_out  = proj;
      best_d2    = d2;
      project_valid = true;
    }
  }
  if (DEBUG_INFO)
    AINFO << "best_d2:" << best_d2;
  return best_d2 < 10*10 && project_valid;
}

bool MapFusion::ReplaceHdmapAttrSegWithBevAttrSeg(const BevLaneMarker* bev_lm,const LaneBoundaryInfo* route_lm,std::vector<std::unique_ptr<LaneBoundaryInfo>> &route_landmarkers) {
  if (!bev_lm || !route_lm || route_lm->points.size() < 2 || bev_lm->line_points.size() < 2)
    return false;
  if (route_lm->line_type == LineType::VIRTUAL_LANE) {
    if (DEBUG_INFO)
      AWARN << "route_lm line_type is VIRTUAL_LANE";
    return false;
  }

  //根据线型合并
  auto bev_segs_raw = bev_lm->type_segs;
  if (bev_segs_raw.empty()) {
    AERROR << "bev_segs_raw is empty";
    return false;
  }
  auto last_bev_type = bev_segs_raw[0];
  for (size_t i = 1; i < bev_segs_raw.size(); ++i) {
    if (bev_segs_raw[i].type == last_bev_type.type) {
      bev_segs_raw[i - 1].end_index = bev_segs_raw[i].end_index;
      bev_segs_raw.erase(bev_segs_raw.begin() + i);
      --i;
    } else {
      last_bev_type = bev_segs_raw[i];
    }
  }
  if (bev_segs_raw[0].start_index != 0) {
    AWARN << "bev lanemaker id" << bev_lm->id << "'s start_index Not 0: " << bev_segs_raw[0].start_index;
  }

  if (bev_segs_raw[0].start_index >= bev_lm->line_points.size()) {
    AERROR << "start_index out of bounds in bev_lm->line_points";
    return false;
  }
  int last_cut_point = 0;
  Point2D point_out      = route_lm->points[0];
  //1.BEV起点切到HDMap，保留HDMap更早的landmarker
  if (FindCutPoint(route_lm->points, bev_lm->line_points[bev_segs_raw[0].start_index], last_cut_point, point_out)) {
    auto route_lm_new = std::make_unique<LaneBoundaryInfo>();
    route_lm_new->id               = ++new_id_;
    int safe_last_cut = std::min<int>(std::max(last_cut_point, 0), route_lm->points.size() - 1);
    route_lm_new->points.insert(route_lm_new->points.end(), route_lm->points.begin(),
                                route_lm->points.begin() + safe_last_cut + 1);
    route_lm_new->points.emplace_back(point_out);
    route_lm_new->line_type = route_lm->line_type;//TODO:需要传入前继
    if (DEBUG_INFO)
      AINFO << " id:" << route_lm_new->id << " last_cut_point:" << last_cut_point
            << " line type:" << static_cast<int>(route_lm_new->line_type) << " from:" << route_lm_new->points[0].x << ","
            << route_lm_new->points[0].y << " to:" << route_lm_new->points[route_lm_new->points.size() - 1].x << ","
            << route_lm_new->points[route_lm_new->points.size() - 1].y;
    route_lm_new->line_color = route_lm->line_color;
    route_landmarkers.emplace_back(std::move(route_lm_new));
    if (DEBUG_INFO)AINFO << "cut point:"<< last_cut_point;
  }

  //2.中途BEV切到HDMap，保留对应的landmarker
  int cut_point, bev_type = 0; 
  bool cut_point_exist = false;
  bool tail_exist = false;
  for (size_t i = 0; i < bev_segs_raw.size(); i++)
  {
    bev_type = static_cast<int>(bev_segs_raw[i].type);
    if (bev_segs_raw[i].end_index >= bev_lm->line_points.size()) {
      if (DEBUG_INFO)AERROR << "end_index out of bounds in bev_lm->line_points";
      continue;
    }
    cut_point_exist = FindCutPoint(route_lm->points, bev_lm->line_points[bev_segs_raw[i].end_index], cut_point, point_out);

    if (DEBUG_INFO)
      AINFO << "bev point:" << bev_lm->line_points[bev_segs_raw[i].end_index].x << "," << bev_lm->line_points[bev_segs_raw[i].end_index].y
            << "cut route from:" << route_lm->points[0].x << "," << route_lm->points[0].y
            << "to:" << route_lm->points[route_lm->points.size() - 1].x << "," << route_lm->points[route_lm->points.size() - 1].y
            << "cut point_out:" << point_out.x << "," << point_out.y
            << " route size:" << route_lm->points.size();

    if (!cut_point_exist) {
      //bev切过了route的末尾
      if (cut_point == (route_lm->points.size() - 1)) {
        tail_exist = true;
        if (DEBUG_INFO)
          AINFO << "bev lanemaker id" << bev_lm->id << "covers route, seg:" << i << "bev_type:" << bev_type
                << "route_type:" << static_cast<int>(route_lm->line_type);
        break;
      }
      //bev还没切切到route
      else if (cut_point == 0) {
        if (DEBUG_INFO)
          AINFO << "bev lanemaker id" << bev_lm->id << "didnt reach route, seg:" << i << "bev_type:" << bev_type
                << "route_type:" << static_cast<int>(route_lm->line_type);
        continue;
      } else {
        if (DEBUG_INFO)
          AINFO << "bev lanemaker id" << bev_lm->id << "'exists in route, seg:" << i << "bev_type:" << bev_type
                << "route_type:" << static_cast<int>(route_lm->line_type);
        return false;
      }
    }

    if (cut_point + 1 > route_lm->points.size() || cut_point < last_cut_point) {
      if (cut_point > 0)
        AERROR << "cut_point:" << cut_point << "route_lm->points.size():" << route_lm->points.size() << "last_cut_point:" << last_cut_point;
      continue;
    }
    auto route_lm_new = std::make_unique<LaneBoundaryInfo>();
    route_lm_new->id = ++new_id_; //原来的landmarker不连续，直接加i
    int safe_cut      = std::min<int>(std::max(cut_point, 0), route_lm->points.size() - 1);
    if (!route_landmarkers.empty() && route_landmarkers.back() != nullptr && !route_landmarkers.back()->points.empty()) {
      int safe_last_cut = std::min<int>(std::max(last_cut_point, 0), route_lm->points.size() - 1);
      if (safe_last_cut + 1 <= safe_cut) {
        route_lm_new->points.emplace_back(route_landmarkers.back()->points.back());
        route_lm_new->points.insert(route_lm_new->points.end(), route_lm->points.begin() + safe_last_cut + 1,
                                    route_lm->points.begin() + safe_cut + 1);
      } else if (safe_last_cut == safe_cut) {
        route_lm_new->points.emplace_back(route_landmarkers.back()->points.back());
        route_lm_new->points.emplace_back(point_out);
      } else {
        if (DEBUG_INFO)
          AINFO << "bev lanemaker id" << bev_lm->id << "'exists in route, seg:" << i << "bev_type:" << bev_type
                << "route_type:" << static_cast<int>(route_lm->line_type);
        return false;
      }
    } else {
      route_lm_new->points.insert(route_lm_new->points.end(), route_lm->points.begin() ,
                                  route_lm->points.begin() + safe_cut + 1);
    }
    route_lm_new->points.emplace_back(point_out);

    switch (bev_type)
    {
    case BevLaneMarkerType::BEV_LMT__DASHED:
        route_lm_new->line_type = LineType::DASHED;
        break;
    case BevLaneMarkerType::BEV_LMT__SOLID:
        route_lm_new->line_type = LineType::SOLID;
        break;
    case BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_DASHED:
        route_lm_new->line_type = LineType::DASHED_DASHED;
        break;
    case BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_SOLID:
        route_lm_new->line_type = LineType::SOLID_SOLID;
        break;
    case BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_SOLID:
        route_lm_new->line_type = LineType::DASHED_SOLID;
        break;
    case BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_DASHED:
        route_lm_new->line_type = LineType::SOLID_DASHED;
        break;
    
    default:
        route_lm_new->line_type = route_lm->line_type;
        break;
    }
    route_lm_new->line_color = route_lm->line_color;
    if (DEBUG_INFO)
      AINFO << " id:" << route_lm_new->id << "cut point:" << cut_point << " last_cut_point:" << last_cut_point
            << " line type:" << static_cast<int>(route_lm_new->line_type) << " from:" << route_lm_new->points[0].x << ","
            << route_lm_new->points[0].y << " to:" << route_lm_new->points[route_lm_new->points.size() - 1].x << ","
            << route_lm_new->points[route_lm_new->points.size() - 1].y;
    last_cut_point = cut_point;
    route_landmarkers.emplace_back(std::move(route_lm_new));
  }
  //3.HDMap末尾被BEV切剩的landmarker
  if (last_cut_point >= route_lm->points.size()) {
    AERROR << "last_cut_point exceeds route_lm->points.size()";
    return false;
  }
  if (last_cut_point >= 0 && last_cut_point < route_lm->points.size() - 1) {
    auto route_lm_new = std::make_unique<LaneBoundaryInfo>();
    route_lm_new->id = ++new_id_; //原来的landmarker不连续，直接加i
    if (!route_landmarkers.empty()) {
      route_lm_new->points.emplace_back(route_landmarkers.back()->points.back());
      route_lm_new->points.insert(route_lm_new->points.end(), route_lm->points.begin() + last_cut_point + 1, route_lm->points.end());
    } else {
      route_lm_new->points.insert(route_lm_new->points.end(), route_lm->points.begin() + last_cut_point, route_lm->points.end());
    }

    if (tail_exist && bev_type >= 0) {
      switch (bev_type) {
        case BevLaneMarkerType::BEV_LMT__DASHED:
          route_lm_new->line_type = LineType::DASHED;
          break;
        case BevLaneMarkerType::BEV_LMT__SOLID:
          route_lm_new->line_type = LineType::SOLID;
          break;
        case BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_DASHED:
          route_lm_new->line_type = LineType::DASHED_DASHED;
          break;
        case BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_SOLID:
          route_lm_new->line_type = LineType::SOLID_SOLID;
          break;
        case BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_SOLID:
          route_lm_new->line_type = LineType::DASHED_SOLID;
          break;
        case BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_DASHED:
          route_lm_new->line_type = LineType::SOLID_DASHED;
          break;

        default:
          route_lm_new->line_type = route_lm->line_type;
          break;
      }
    } else {
      route_lm_new->line_type = route_lm->line_type;
    }
    route_lm_new->line_color = route_lm->line_color;
    if (DEBUG_INFO)
      AINFO << " id:" << route_lm_new->id << "cut point:" << last_cut_point << " line type:" << static_cast<int>(route_lm_new->line_type)
            << " from:" << route_lm_new->points[0].x << "," << route_lm_new->points[0].y
            << " to:" << route_lm_new->points[route_lm_new->points.size() - 1].x << ","
            << route_lm_new->points[route_lm_new->points.size() - 1].y;

    route_landmarkers.emplace_back(std::move(route_lm_new));
  }

  //根据线型合并
  if (route_landmarkers.empty())return false;  
  auto last_lm_type = route_landmarkers[0]->line_type;
  for (size_t i = 1; i < route_landmarkers.size(); ++i) {
    if (route_landmarkers[i]->line_type == last_lm_type) {
      route_landmarkers[i-1]->points.insert(route_landmarkers[i-1]->points.end(),
                                          route_landmarkers[i]->points.begin(), route_landmarkers[i]->points.end());
      route_landmarkers.erase(route_landmarkers.begin() + i);
      --i; // Adjust index after erasure
    } else {
      last_lm_type = route_landmarkers[i]->line_type;
    }
  }

  //去重
  for (auto &lane : route_landmarkers) {
    for (size_t i = 1; i < lane->points.size(); i++) {
      if (fabs(lane->points[i].x - lane->points[i - 1].x) < 0.01 &&
          fabs(lane->points[i].y - lane->points[i - 1].y) < 0.01) {
        lane->points.erase(lane->points.begin() + i);
        i--;
      }
    }
  }
  return true;
}

void MapFusion::UpdateRoutingMapArrows(const BevMapInfoPtr &bev_map_ptr, RoutingMapPtr &routing_map_ptr) {
  if (bev_map_ptr == nullptr || routing_map_ptr == nullptr) {
    return;
  }

  routing_map_ptr->arrows.clear();
  for (const auto &bev_arrow : bev_map_ptr->arrows) {
    Arrows routing_map_arrow;

    std::vector<Point2D> r_map_points;
    for (const auto &pt : bev_arrow.line_points) {
      Point2D point;
      point.x = pt.x;
      point.y = pt.y;
      r_map_points.emplace_back(point);
    }

    routing_map_arrow.id     = bev_arrow.id;
    routing_map_arrow.type   = bev_arrow.type;
    routing_map_arrow.points = r_map_points;
    routing_map_ptr->arrows.emplace_back(routing_map_arrow);
  }
}

void MapFusion::AssignLaneMarkerColor(BevMapInfo &bev_map) {
  for (auto &lanemarker : bev_map.lanemarkers) {
    auto points = lanemarker.line_points;
    std::vector<int> cut_idx{0};
    if (points.size() < 2) {
      continue;
    }

    for (int pt_idx = 0; pt_idx < points.size() - 1; pt_idx++) {
      auto pre_pt  = points[pt_idx];
      auto curr_pt = points[pt_idx + 1];
      if (pre_pt.color != curr_pt.color && curr_pt.color != PointColor::BEV_LMC__UNKNOWN && pre_pt.color != PointColor::BEV_LMC__UNKNOWN) {
        cut_idx.emplace_back(pt_idx);
        cut_idx.emplace_back(pt_idx + 1);
      }
    }
    int end_idx = static_cast<int>(points.size() - 1);
    cut_idx.emplace_back(end_idx);


    std::vector<BevLmColorSeg> color_segs;
    if (cut_idx.size() < 3) {
      BevLmColorSeg seg;
      if (points.front().color == PointColor::BEV_LMC__UNKNOWN) {
        seg.color = static_cast<BevLaneMarkerColor>(PointColor::BEV_LMC__WHITE);
      } else {
        seg.color = static_cast<BevLaneMarkerColor>(points.front().color);
      }
      seg.start_index = 0;
      seg.end_index   = end_idx;
      color_segs.emplace_back(seg);
      lanemarker.color_segs.clear();
      lanemarker.color_segs = color_segs;
      continue;
    }
  
    lanemarker.color_segs.clear();
    for (int idx = 0; idx <= cut_idx.size() - 2; idx += 2) {
      BevLmColorSeg seg;
      seg.color       = static_cast<BevLaneMarkerColor>(points[cut_idx[idx]].color);
      seg.start_index = cut_idx[idx];
      seg.end_index   = cut_idx[idx + 1];
      lanemarker.color_segs.emplace_back(seg);
    }
  }
}

void MapFusion::AssignColor2LaneMarkerPts(BevMapInfo &bev_map) {
  for (auto &LaneMarker : bev_map.lanemarkers) {
    int len = LaneMarker.line_points.size();
    for (int ptsIndx = 0; ptsIndx < len; ptsIndx++) {
      BevLaneMarkerColor color = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
      for (auto &colorSeg : LaneMarker.color_segs) {
        if (colorSeg.start_index <= ptsIndx && ptsIndx <= colorSeg.end_index) {
          color = (colorSeg.color);
          break;
        }
      }
      LaneMarker.line_points[ptsIndx].color = static_cast<cem::message::common::PointColor>(color);
    }
  }

  // 修改LaneMarker的颜色
  for (auto &lanemarker : bev_map.lanemarkers) {
    auto& points = lanemarker.line_points;
    if (points.size() < 2) {
      continue;
    }

    for (size_t pt_idx = 0; pt_idx < points.size() - 1; pt_idx++) {
      if (points[pt_idx + 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
        points[pt_idx + 1].color = points[pt_idx].color;
      }
    }

    for (size_t pt_idx = points.size() - 1; pt_idx > 0; pt_idx--) {
      if (points[pt_idx - 1].color == PointColor::BEV_LMC__UNKNOWN && points[pt_idx].color != PointColor::BEV_LMC__UNKNOWN) {
        points[pt_idx - 1].color = points[pt_idx].color;
      }
    }
  }

  // AINFO << "*******PtsColorAfter******"; 
  // AINFO << "sequence number: " << bev_map.header.cycle_counter;
  // for (auto &lanemarker : bev_map.lanemarkers){
  //   AINFO << "lanemarker ID: " << lanemarker.id;
  //   std::stringstream pt_color_str;
  //   for (const auto& pt : lanemarker.line_points){
  //     pt_color_str << static_cast<int>(pt.color) << ", ";
  //   }
  //   AINFO << "pt_color_str: " << pt_color_str.str();
  // }
  // AINFO << "*************************";
  return;
}

}  // namespace fusion
}  // namespace cem
