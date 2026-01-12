#include "routing_map_pre_processor.h"
#include <time.h>
#include <limits>
#include <Eigen/Dense>
#include "common/utility.h"
#include "common/utils/GeoMathUtil.h"
namespace cem {
namespace fusion {
const float kEmergencyFilterMaxWidth = 5.0;
RoutingMapPreProcessor::RoutingMapPreProcessor() {}

RoutingMapPreProcessor::~RoutingMapPreProcessor() {}

void RoutingMapPreProcessor::Process(const DetectBevMap &bev_map,
                                     RoutingMapPtr ld_map) {
  if (ld_map == nullptr) {
    SensorDataManager::Instance()->GetLatestSensorFrame(input_routing_map_ptr_);
  } else {
    input_routing_map_ptr_ = ld_map;
  }

  if (!input_routing_map_ptr_) {
    INTERNAL_PARAMS.sd_map_data.ClearContainer();
    INTERNAL_PARAMS.ld_map_data.ClearContainer();
    sd_route_Info_ptr_.reset();
    output_routing_map_ptr_.reset();
    return;
  }
  sd_route_Info_ptr_  = std::make_shared<SDRouteInfo>();
  *sd_route_Info_ptr_ = input_routing_map_ptr_->sd_route;

  output_routing_map_ptr_ = std::make_shared<RoutingMap>(*input_routing_map_ptr_);

  LDMapInfoProcess();

  if (bev_map.bev_map_ptr == nullptr) {
    return;
  }

  BevReplaceMapEdge(bev_map);
}

void RoutingMapPreProcessor::InsertLaneCenter() {
  std::vector<LaneInfo> lanesVec = output_routing_map_ptr_->lanes;
  int                   laneNum  = lanesVec.size();
  for (int laneId = 0; laneId < laneNum; laneId++) {
    LaneInfo           lane      = lanesVec[laneId];
    std::vector<Point> pointsVec = lane.points;
    for (auto iter = pointsVec.begin(); iter != pointsVec.end();) {
      if (iter == pointsVec.begin()) {
        ++iter;
        continue;
      }
      const auto last_point = *(iter - 1);
      double     dist       = static_cast<double>(CalculatePoint2PointDist(last_point, *iter));
      if (dist > SAMPING_INTERVAL * 1.5 && dist < MAX_SAMPING_INTERVAL) {
        int    samping_num     = std::floor(dist / SAMPING_INTERVAL);
        double scale_x         = (iter->x - last_point.x) / dist;
        double scale_y         = (iter->y - last_point.y) / dist;
        double scale_curvature = (iter->curvature - last_point.curvature) / dist;
        for (int i = 1; i <= samping_num; ++i) {
          static Point insert_point;
          double       add_x         = i * SAMPING_INTERVAL * scale_x;
          double       add_y         = i * SAMPING_INTERVAL * scale_y;
          double       add_curvature = i * SAMPING_INTERVAL * scale_curvature;
          insert_point.x             = (last_point.x + add_x);
          insert_point.y             = (last_point.y + add_y);
          insert_point.curvature     = (last_point.curvature + add_curvature);
          iter                       = pointsVec.insert(iter, insert_point);
          ++iter;
        }
      }
      ++iter;
    }
    lane.points.clear();
    lane.points = pointsVec;
    if (pointsVec.empty()) {
      continue;
    }
    output_routing_map_ptr_->lanes[laneId].points = lane.points;
  }
}

void RoutingMapPreProcessor::InsertLaneBoundary() {
  if (output_routing_map_ptr_ == nullptr) {
    return;
  }
  std::vector<LaneBoundaryInfo> lanesBoundaries = output_routing_map_ptr_->lane_boundaries;
  int                           laneNum         = lanesBoundaries.size();
  for (int laneIdx = 0; laneIdx < laneNum; laneIdx++) {
    LaneBoundaryInfo     lane      = lanesBoundaries[laneIdx];
    std::vector<Point2D> pointsVec = lane.points;
    for (auto iter = pointsVec.begin(); iter != pointsVec.end();) {
      if (iter == pointsVec.begin()) {
        ++iter;
        continue;
      }
      const auto last_point = *(iter - 1);
      double     dist       = static_cast<double>(CalculatePoint2PointDist(last_point, *iter));
      if (dist > SAMPING_INTERVAL * 1.5 && dist < MAX_SAMPING_INTERVAL) {
        int    samping_num = std::floor(dist / SAMPING_INTERVAL);
        double scale_x     = (iter->x - last_point.x) / dist;
        double scale_y     = (iter->y - last_point.y) / dist;
        for (int i = 1; i <= samping_num; ++i) {
          static Point2D insert_point;
          double         add_x = i * SAMPING_INTERVAL * scale_x;
          double         add_y = i * SAMPING_INTERVAL * scale_y;
          insert_point.x       = (last_point.x + add_x);
          insert_point.y       = (last_point.y + add_y);
          iter                 = pointsVec.insert(iter, insert_point);
          ++iter;
        }
      }
      ++iter;
    }
    lane.points.clear();
    lane.points = pointsVec;
    if (pointsVec.empty()) {
      continue;
    }
    output_routing_map_ptr_->lane_boundaries[laneIdx].points = lane.points;
  }
}

void RoutingMapPreProcessor::InsertLanePoint() {
  InsertLaneCenter();
  InsertLaneBoundary();
}

inline auto CalculateLineToLaneDist = [](std::vector<Eigen::Vector2d> &sourcePts, std::vector<Eigen::Vector2d> &targetPts) {
  int             count    = 1;
  int             outIdx   = 0;
  double          meanDist = 0.0;
  Eigen::Vector2d foot;
  for (auto &point : sourcePts) {
    double dist = 0.0;
    bool   flag =
        GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(point, targetPts, 0, targetPts.size() - 1, false, foot, dist, outIdx, 0.3);
    if (flag && dist < kEmergencyFilterMaxWidth) {
      meanDist += dist;
      count++;
    }
  }
  meanDist /= count;
  return meanDist;
};

#if 1
void RoutingMapPreProcessor::BoundBoundaryWithLane(const DetectBevMap &bevMap) {
  const BevMapInfoPtr     &bevMapPtr = bevMap.bev_map_ptr;
  const Eigen::Isometry3d &Twb       = bevMap.Twb;
  const Eigen::Isometry3d  Tbw       = Twb.inverse();

  std::unordered_map<uint64_t, LaneInfo>              laneIdToUpdatedLane;
  std::unordered_map<uint64_t, std::vector<uint64_t>> laneIdToNext;

  for (const auto &lane : output_routing_map_ptr_->lanes) {
    laneIdToNext[lane.id] = lane.next_lane_ids;
  }

  // Step 1: 匹配每条 edge 到最邻近 lane
  for (const auto &edge : bevMapPtr->edges) {
    if (edge.line_points.size() < 4 && edge.id > 100)
      continue;

    const Eigen::Vector3d edgeFirstPt(edge.line_points[0].x, edge.line_points[0].y, 0.0);
    const Eigen::Vector3d edgeFirstPtLoc = Tbw * edgeFirstPt;

    std::vector<Eigen::Vector2d> edgePts;
    edgePts.reserve(edge.line_points.size());
    for (const auto &pt : edge.line_points) {
      edgePts.emplace_back(pt.x, pt.y);
    }

    double   minDist       = std::numeric_limits<double>::max();
    uint64_t matchedLaneId = 0;

    for (const auto &lane : output_routing_map_ptr_->lanes) {
      if (lane.points.empty() || lane.type == LaneType::LANE_VIRTUAL_JUNCTION)
        continue;

      std::vector<Eigen::Vector2d> lanePts;
      lanePts.reserve(lane.points.size());
      for (const auto &pt : lane.points) {
        lanePts.emplace_back(pt.x, pt.y);
      }

      double dist = CalculateLineToLaneDist(lanePts, edgePts);
      if (dist > kEmergencyFilterMaxWidth || dist < 0.001)
        continue;

      if (dist < minDist) {
        minDist       = dist;
        matchedLaneId = lane.id;
      }
    }

    if (matchedLaneId == 0)
      continue;

    // Step 2: 根据方向判断左/右边界
    for (auto &lane : output_routing_map_ptr_->lanes) {
      if (lane.id != matchedLaneId || lane.points.empty())
        continue;

      Eigen::Vector3d laneFirstPt(lane.points[0].x, lane.points[0].y, 0.0);
      Eigen::Vector3d laneFirstPtLoc = Tbw * laneFirstPt;

      if (edgeFirstPtLoc.y() > laneFirstPtLoc.y()) {
        lane.left_road_boundary_ids = {edge.id};
        lane.right_road_boundary_ids.clear();
      } else {
        lane.right_road_boundary_ids = {edge.id};
        lane.left_road_boundary_ids.clear();
      }

      laneIdToUpdatedLane[lane.id] = lane;
      break;
    }
  }

  // Step 3: 为未匹配 lane 填充边界信息（通过 next_lane_ids 继承）
  for (auto &lane : output_routing_map_ptr_->lanes) {
    if (!lane.left_road_boundary_ids.empty() || !lane.right_road_boundary_ids.empty())
      continue;

    for (const auto &entry : laneIdToUpdatedLane) {
      const auto &fromLane = entry.second;
      if (std::find(fromLane.next_lane_ids.begin(), fromLane.next_lane_ids.end(), lane.id) != fromLane.next_lane_ids.end()) {
        lane.left_road_boundary_ids  = fromLane.left_road_boundary_ids;
        lane.right_road_boundary_ids = fromLane.right_road_boundary_ids;
        break;
      }
    }
  }
}
#else
//add by qiuzhiming
void RoutingMapPreProcessor::BoundBoundaryWithLane(const DetectBevMap &bevMap) {
  const auto              &bevMapPtr = bevMap.bevMapPtr;
  const Eigen::Isometry3d &Twb       = bevMap.Twb;
  const Eigen::Isometry3d  Tbw       = Twb.inverse();

  // 存储已绑定边界的车道ID
  std::unordered_map<uint64_t, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> laneBoundaryMap;

  // 遍历每条边缘线并绑定到最近的车道
  for (const auto &edge : bevMapPtr->edges) {
    if (edge.line_points.empty()) {
      continue;
    }

    Eigen::Vector3d edgeFirstPtLocal = Tbw * Eigen::Vector3d(edge.line_points[0].x, edge.line_points[0].y, 0.0);
    uint64_t        nearestLaneId    = 0;
    bool            isLeftBoundary   = false;
    if (!FindNearestLane(edge, edgeFirstPtLocal, Tbw, nearestLaneId, isLeftBoundary)) {
      continue;
    }

    auto &boundaryIds = isLeftBoundary ? laneBoundaryMap[nearestLaneId].first : laneBoundaryMap[nearestLaneId].second;
    boundaryIds.clear();
    boundaryIds.emplace_back(edge.id);
  }
  PropagateBoundaries(laneBoundaryMap);
}

bool RoutingMapPreProcessor::FindNearestLane(const BevEdge &edge, const Eigen::Vector3d &edgeFirstPtLocal, const Eigen::Isometry3d &Tbw,
                                             uint64_t &nearestLaneId, bool &isLeftBoundary) {

  double minDistance = std::numeric_limits<double>::max();
  nearestLaneId      = 0;

  // 遍历车道，找到最近的车道ID
  for (const auto &lane : output_routing_map_ptr_->lanes) {
    if (lane.points.empty() || lane.type == LaneType::LANE_VIRTUAL_JUNCTION) {
      continue;
    }

    double distance = CalculateLineToLaneDist(edge.line_points, lane.points);
    if (distance < minDistance && distance > 0.001 && distance < kEmergencyFilterMaxWidth) {
      minDistance   = distance;
      nearestLaneId = lane.id;
    }
  }

  if (nearestLaneId != 0) {
    for (const auto &lane : output_routing_map_ptr_->lanes) {
      if (lane.id == nearestLaneId) {
        Eigen::Vector3d laneFirstPtLocal = Tbw * Eigen::Vector3d(lane.points[0].x, lane.points[0].y, 0.0);
        isLeftBoundary                   = (edgeFirstPtLocal.y() > laneFirstPtLocal.y());
        break;
      }
    }
  }

  return nearestLaneId != 0;
}

void RoutingMapPreProcessor::PropagateBoundaries(
    const std::unordered_map<uint64_t, std::pair<std::vector<uint64_t>, std::vector<uint64_t>>> &laneBoundaryMap) {

  for (auto &lane : output_routing_map_ptr_->lanes) {
    if (laneBoundaryMap.find(lane.id) == laneBoundaryMap.end()) {
      // 如果当前车道还没有边界信息，尝试从前驱车道继承
      for (const auto &prevLaneId : lane.previous_lane_ids) {
        if (laneBoundaryMap.find(prevLaneId) != laneBoundaryMap.end()) {
          lane.left_road_boundary_ids  = laneBoundaryMap.at(prevLaneId).first;
          lane.right_road_boundary_ids = laneBoundaryMap.at(prevLaneId).second;
          break;
        }
      }
    } else {
      // 如果已经有边界信息，直接更新车道边界字段
      lane.left_road_boundary_ids  = laneBoundaryMap.at(lane.id).first;
      lane.right_road_boundary_ids = laneBoundaryMap.at(lane.id).second;
    }
  }
}
#endif

void RoutingMapPreProcessor::BevReplaceMapEdge(const DetectBevMap &bev_map) {
  output_routing_map_ptr_->road_boundaries.clear();
  for (auto edge : bev_map.bev_map_ptr->edges) {
    auto            &edgePoints = edge.line_points;
    RoadBoundaryInfo boundary;
    for (auto &point : edgePoints) {
      Point2D mapPoint;
      mapPoint.x = static_cast<double>(point.x);
      mapPoint.y = static_cast<double>(point.y);
      boundary.points.emplace_back(mapPoint);
    }
    boundary.id = edge.id;
    //由于二者定义基本不一样，数值透传
    if (edge.type == 5) {
      boundary.boundary_type = BoundaryType::VIRTUAL;  // 5 -> 12
    } else if (edge.type >= 0 && edge.type <= 4) {
      boundary.boundary_type = static_cast<BoundaryType>(edge.type);  // 0-4 保持原样
    } else {
      boundary.boundary_type = BoundaryType::UNKNOWN_BOUNDARY;  // 默认值
    }
    output_routing_map_ptr_->road_boundaries.emplace_back(boundary);
  }
}

RoutingMapPtr RoutingMapPreProcessor::GetRoutingMap() {
  return output_routing_map_ptr_;
}

void RoutingMapPreProcessor::LDMapInfoProcess() {
  auto &raw_sd_data = INTERNAL_PARAMS.sd_map_data;
  raw_sd_data.ClearContainer();

  raw_sd_data.SetOwnerRoutingMap(input_routing_map_ptr_);
  raw_sd_data.SetSDRouteInfoPtr(sd_route_Info_ptr_);
  if (input_routing_map_ptr_) {
    for (auto &sd_lane_group_tmp : input_routing_map_ptr_->sd_lane_groups) {
      raw_sd_data.AddSDLaneGroupInfoById(sd_lane_group_tmp.id, &sd_lane_group_tmp);
      for (auto &sd_lane_tmp : sd_lane_group_tmp.lane_info) {
        raw_sd_data.AddSDLaneInfoById(sd_lane_tmp.id, &sd_lane_tmp);
      }
    }
  }
  if (sd_route_Info_ptr_) {
    for (auto &section_info_tmp : sd_route_Info_ptr_->mpp_sections) {
      section_info_tmp.is_mpp_section = true;
      raw_sd_data.AddSDSectionInfoById(section_info_tmp.id, &section_info_tmp);
    }
    for (auto &subpath_tmp : sd_route_Info_ptr_->subpaths) {
      for (auto &section_info_tmp : subpath_tmp.sections) {
        section_info_tmp.is_mpp_section = false;
        raw_sd_data.AddSDSectionInfoById(section_info_tmp.id, &section_info_tmp);
      }
    }
  }

  raw_sd_data.BuildLaneCache();

  auto &raw_ld_data = INTERNAL_PARAMS.ld_map_data;
  raw_ld_data.InitFromRoutingMap(input_routing_map_ptr_);

  RoutingMapTransToWorld();
}

template <typename T>
#if defined __aarch64__
void        update_vector2d(std::vector<T> &points, const double *twd_4x4) {
         float64x2_t v1   = vld1q_f64(twd_4x4);
         float64x2_t v2   = vld1q_f64(twd_4x4 + 2 * 2);
         float64x2_t vadd = {twd_4x4[3], twd_4x4[7]};

         for (int i = 0; i < points.size(); ++i) {
           double     *_data  = &points[i].x;
           float64x2_t vp     = vld1q_f64(_data);
           float64x2_t vp1    = vmulq_f64(vp, v1);
           float64x2_t vp2    = vmulq_f64(vp, v2);
           float64x2_t vp_out = vpaddq_f64(vp1, vp2);
           vp_out             = vaddq_f64(vp_out, vadd);
           vst1q_f64(_data, vp_out);
  }
}
#else
void update_vector2d(std::vector<T> &points, const double *twd_4x4) {
  for (int i = 0; i < points.size(); ++i) {
    double px   = twd_4x4[0] * points[i].x + twd_4x4[1] * points[i].y + twd_4x4[3];
    double py   = twd_4x4[4] * points[i].x + twd_4x4[5] * points[i].y + twd_4x4[7];
    points[i].x = px;
    points[i].y = py;
  }
}
#endif

void RoutingMapPreProcessor::RoutingMapTransToWorld() {
  if (input_routing_map_ptr_ == nullptr) {
    return;
  }
  double timestamp = input_routing_map_ptr_->header.timestamp;
  auto   Twb       = FindTransform(timestamp);
  if (Twb == std::nullopt) {
    AERROR << "LD map can not find the odometry";
    return;
  }
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> row_mat = Twb.value().matrix();
  const double                                *twd_4x4 = row_mat.data();

  output_routing_map_ptr_ = std::make_shared<RoutingMap>(*input_routing_map_ptr_);
  for (auto &lane : output_routing_map_ptr_->lanes) {
    update_vector2d(lane.points, twd_4x4);
  }

  for (auto &boundary : output_routing_map_ptr_->lane_boundaries) {
    update_vector2d(boundary.points, twd_4x4);
  }

  for (auto &stopLane : output_routing_map_ptr_->stop_lines) {
    update_vector2d(stopLane.points, twd_4x4);
  }

  for (auto &junction : output_routing_map_ptr_->junctions) {
    update_vector2d(junction.points, twd_4x4);
  }

  for (auto &crossWalk : output_routing_map_ptr_->cross_walks) {
    update_vector2d(crossWalk.points, twd_4x4);
  }

  for (auto &roadBoundary : output_routing_map_ptr_->road_boundaries) {
    update_vector2d(roadBoundary.points, twd_4x4);
  }

  // add by zhiming
  for (auto &section : output_routing_map_ptr_->route.sections) {
    update_vector2d(section.points, twd_4x4);
  }

  for (auto &trajectory : output_routing_map_ptr_->exp_trajectories) {
    update_vector2d(trajectory.points, twd_4x4);
  }
  // AINFO<<"routing map output timestmap
  // "<<output_routing_map_ptr_->header.timestamp;
}

std::optional<Eigen::Isometry3d> RoutingMapPreProcessor::FindTransform(const double &timestamp) {
  LocalizationPtr odom_t0_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, odom_t0_ptr);

  if (odom_t0_ptr == nullptr) {
    return std::nullopt;
  }

  Eigen::Isometry3d Twb_0 = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd Rwb_0((odom_t0_ptr->attitude_dr) * M_PI / 180.0, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d   trans_0(odom_t0_ptr->posne_dr.at(0), odom_t0_ptr->posne_dr.at(1), 0.0);
  Twb_0.rotate(Rwb_0);
  Twb_0.pretranslate(trans_0);

  return Twb_0;
}

}  // namespace fusion
}  // namespace cem
