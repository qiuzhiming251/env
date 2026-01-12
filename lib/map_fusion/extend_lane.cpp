#include "extend_lane.h"
#if defined __aarch64__
#include <arm_neon.h>
#endif
namespace cem{
namespace fusion{
namespace extendLane{

#if defined __aarch64__
static inline __attribute__((always_inline)) float64x2_t swap_and_neg_one_of_two_f64x2(float64x2_t va, uint64x2_t mask) {
  float64x2_t ret = vreinterpretq_f64_u64(veorq_u64(mask, vreinterpretq_u64_f64(va)));
  return vextq_f64(ret, ret, 1);
}

static inline __attribute__((always_inline)) int line_cross_polyines(const std::vector<Eigen::Vector3d> &polyline, float64x2_t vp1,
                                                                     float64x2_t vp2) {
  uint64x2_t  mask    = {0, 0x8000000000000000LL};
  float64x2_t vp_diff = vp2 - vp1;
  vp_diff             = swap_and_neg_one_of_two_f64x2(vp_diff, mask);
  float64x2_t vq1     = {polyline[0].x(), polyline[0].y()};
  // float64x2_t vq1 = vld1q_f64(pt); pt += 2;

  float64x2_t vd1 = vmulq_f64(vp_diff, vq1 - vp1);
  for (int i = 1; i < polyline.size(); ++i) {
    float64x2_t vq2 = {polyline[i].x(), polyline[i].y()};
    // float64x2_t vq2 = vld1q_f64(pt); pt += 2;
    float64x2_t vq_diff = vq2 - vq1;
    vq_diff             = swap_and_neg_one_of_two_f64x2(vq_diff, mask);

    float64x2_t vd2         = vmulq_f64(vp_diff, vq2 - vp1);
    float64x2_t vd3         = vmulq_f64(vq_diff, vp1 - vq1);
    float64x2_t vd4         = vmulq_f64(vq_diff, vp2 - vq1);
    float64x2_t vd13        = vpaddq_f64(vd1, vd3);
    float64x2_t vd24        = vpaddq_f64(vd2, vd4);
    float64x2_t mul_d12_d34 = vmulq_f64(vd13, vd24);
    uint64x2_t  mm          = vcltq_f64(mul_d12_d34, (float64x2_t){0.0, 0.0});  //vcltz_f64
    int         ret_value   = vaddvq_s64(vreinterpretq_s64_u64(mm));

    if (ret_value == -2) {
      return i;
    }
    vq1 = vq2;
    vd1 = vd2;
  }
  return 0;
}

static inline __attribute__((always_inline)) int pointInPolygon_neon(const Eigen::Vector3d                             &pt,
                                                                     const std::vector<Point2DF> &polygon) {
  int             inside = 0;
  register double tx     = pt.x();
  register double ty     = pt.y();

  register double poly1_x = polygon[polygon.size() - 1].x;
  register double poly1_y = polygon[polygon.size() - 1].y;
  for (size_t i = 0; i < polygon.size(); i++) {
    register double poly2_x = polygon[i].x;
    register double poly2_y = polygon[i].y;

    uint64x1_t k1 = vcgt_f64((float64x1_t){poly1_y}, (float64x1_t){ty});
    uint64x1_t k2 = vcgt_f64((float64x1_t){poly2_y}, (float64x1_t){ty});
    if (k1[0] != k2[0]) {
      double t = (ty - poly1_y) * (poly1_x - poly2_x) / (poly1_y - poly2_y) + poly1_x;
      if (tx < t)
        inside = ~inside;
    }
    poly1_x = poly2_x;
    poly1_y = poly2_y;
  }
  return inside;
}
#endif

inline bool IsLineSegmentsCrossPolygon(const std::vector<Eigen::Vector3d>                &polyline,  // 散点线段
                                       const std::vector<Point2DF> &polygon, bool &beginFiltered,
                                       int insideThresh = 10) {
  if (polygon.size() < 2 || polyline.size() < 2)
    return false;

  //1. 检查 polyline 最新一段是否与 polygon 边界点相近
  const double curb_dist_sqr = 0.5 * 0.5;  // 干涉阈值
  for (size_t i = 0; i < polyline.size(); i++) {
    auto   globalPt = polyline[i];
    double dx_back  = polygon.back().x - globalPt.x();
    double dy_back  = polygon.back().y - globalPt.y();
    double dx_front = polygon.front().x - globalPt.x();
    double dy_front = polygon.front().y - globalPt.y();
    if ((dx_back * dx_back + dy_back * dy_back < curb_dist_sqr) || (dx_front * dx_front + dy_front * dy_front < curb_dist_sqr)) {
      if (!beginFiltered) {
        beginFiltered = true;  // 如果有超过一个点在多边形内部，则记录为预备截断点
      }
      return true;
    }
  }

  // 判断 polyline 是否穿过 polygon（即线段与 polygon 边缘相交）
  auto is_segment_intersect = [](const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q1,
                                 const Eigen::Vector2d &q2) -> bool {
    auto cross = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
      return a.x() * b.y() - a.y() * b.x();
    };

    auto d1 = cross(q2 - q1, p1 - q1);
    auto d2 = cross(q2 - q1, p2 - q1);
    auto d3 = cross(p2 - p1, q1 - p1);
    auto d4 = cross(p2 - p1, q2 - p1);
    return (d1 * d2 < 0) && (d3 * d4 < 0);
  };
  // 2.检查 polyline 任意一段是否与 polygon 边相交
  size_t N = polygon.size();
  for (size_t j = 0; j < N; ++j) {
    Eigen::Vector2d p1(polygon[j].x, polygon[j].y);
    Eigen::Vector2d p2(polygon[(j + 1) % N].x, polygon[(j + 1) % N].y);  // 自动闭合
#if defined __aarch64__
    int ret = line_cross_polyines(polyline, (float64x2_t){polygon[j].x, polygon[j].y},
                                  (float64x2_t){polygon[(j + 1) % N].x, polygon[(j + 1) % N].y});
    if (ret) {
      beginFiltered = true;
      return true;
    }

#else
    for (size_t i = 0; i < polyline.size() - 1; ++i) {
      Eigen::Vector2d a(polyline[i].x(), polyline[i].y());
      Eigen::Vector2d b(polyline[i + 1].x(), polyline[i + 1].y());
      if (is_segment_intersect(a, b, p1, p2)) {
        if (!beginFiltered)
          beginFiltered = true;
        return true;
      }
    }
#endif
  }

  // 3.点数统计判断：有一定比例在 polygon 内部
  auto is_point_inside = [&](const Eigen::Vector3d &pt) -> bool {
    int    cross_count = 0;
    size_t n           = polygon.size();
    for (size_t i = 0; i < n; ++i) {
      const auto &p1 = polygon[i];
      const auto &p2 = polygon[(i + 1) % n];
      if (p1.y == p2.y)
        continue;
      if (pt[1] < std::min(p1.y, p2.y) || pt[1] >= std::max(p1.y, p2.y))
        continue;
      double x_intersect = (pt[1] - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
      if (x_intersect > pt[0])
        ++cross_count;
    }
    return (cross_count % 2 == 1);
  };
  int inside_count = 0;
  for (const auto &pt : polyline) {

#if defined __aarch64__
    int ret = pointInPolygon_neon(pt, polygon);
    if (ret) {
      ++inside_count;
      beginFiltered = true;  // 如果有超过一个点在多边形内部，则记录为预备截断点
      if (inside_count > insideThresh) {
        return true;
      }
    }
#else
    if (is_point_inside(pt)) {
      ++inside_count;
      if (!beginFiltered) {
        beginFiltered = true;  // 如果有超过一个点在多边形内部，则记录为预备截断点
      }
      if (inside_count > insideThresh) {
        return true;
      }
    }
#endif
  }

  return false;
}

inline bool Cross(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
  auto det = [](double x0, double y0, double x1, double y1) {
    return x0 * y1 - y0 * x1;
  };

  auto dir1 = det(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
  auto dir2 = det(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
  auto dir3 = det(x4 - x3, y4 - y3, x1 - x3, y1 - y3);
  auto dir4 = det(x4 - x3, y4 - y3, x2 - x3, y2 - y3);

  return (dir1 * dir2 < 0) && (dir3 * dir4 < 0);
}

inline bool IsSegmentsIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
  return Cross(x1, y1, x2, y2, x3, y3, x4, y4);
}

constexpr float kEmergencyFilterMaxWidth = 10.0;
ExtendLane::ExtendLane(): crosswalk_tracker_ptr_(std::make_shared<cem::fusion::CrossWalkTracker>()) 
{

}
ExtendLane::~ExtendLane()
{

}
double crossProduct(const Point2DF &pt1,
	const Point2DF &pt2,
	const Point2DF &pt3)
{
	return (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
}
void ExtendLane::Process(DetectBevMap &detectBevMap)
{
    T_local_ego_ = detectBevMap.Twb;
    T_ego_local_ = T_local_ego_.inverse();
	if(detectBevMap.bev_map_ptr == nullptr){
		return;
	}
	BevMapInfo &bevMap = *detectBevMap.bev_map_ptr;
	//AINFO<<"befor is virtual";
    FindCrossMinValue(bevMap);

    BoundLaneMarkerWithLane(bevMap);

    IsVirtualLine(bevMap);

    JudgeIsEmergency(bevMap);

    ExtendLaneInfo(bevMap);

    // Extend lane lines
    CurveFittingLaneInfo(bevMap);
    
    SmoothLineInfo(bevMap);

    SmoothLineMarkers(bevMap);

	//CutLaneInfoOutLaneMarker(bevMap);

    CutLaneInfoOutCrossRoad(bevMap);

    DeleteDuplicateLaneInfo(bevMap);

    ExtendLaneMarkerwithLane(bevMap);
}

void ExtendLane::ExtendLaneMarkerwithLane(BevMapInfo &bev_map) {
  auto extend_points = [&](uint32_t lane_marker_id,std::vector<Point2DF>& line_points) {
    if (lane_marker_id == 0) {
      return;
    }
    auto lm_itr = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(), [&](const BevLaneMarker &lm) {return lm.id == lane_marker_id;});
    if (lm_itr != bev_map.lanemarkers.end()) {
      if (m_emergencyLaneMarkerIds.find(lm_itr->id) != m_emergencyLaneMarkerIds.end() ||
          m_virtualLineIdxs.find(lm_itr->id) != m_virtualLineIdxs.end() || lm_itr->line_points.size() < 3) {
        return;
      }

      double dist = 0;
      for (size_t i = 0; i < line_points.size(); ++i) {
        auto            lane_pnt_loc = T_ego_local_ * Eigen::Vector3d{line_points[i].x, line_points[i].y, 0.0};
        Eigen::Vector3d back_pt{lm_itr->line_points.back().x, lm_itr->line_points.back().y, 0.0};
        Eigen::Vector3d back_pt_loc = T_ego_local_ * back_pt;

        if ((back_pt_loc.x() + 1.0) < lane_pnt_loc.x()) {
          if (dist == 0) {
            dist = back_pt_loc.y() - lane_pnt_loc.y();
          }
          Eigen::Vector3d extend_pt = {lane_pnt_loc.x(), lane_pnt_loc.y(), 0.0};

          // 基于相对左右判断方向
          extend_pt.y() += dist;

          Eigen::Vector3d                global_pt = T_local_ego_ * extend_pt;
          Point2DF output_pt;
          output_pt.x            = global_pt.x();
          output_pt.y            = global_pt.y();
          output_pt.mse          = NAN;
          output_pt.point_source = line_points[i].point_source;
          lm_itr->line_points.emplace_back(output_pt);
        }
      }
    }
  };

  for (auto &lane : bev_map.lane_infos) {
    if (!lane.next_lane_ids.empty() || (!lane.line_points.empty() && static_cast<int>(lane.line_points.back().point_source) < 3) ||
        lane.line_points.empty() || lane.is_blocked) {
      continue;
    }

    extend_points(lane.left_lane_marker_id, lane.line_points);
    extend_points(lane.right_lane_marker_id, lane.line_points);
  }
}

void ExtendLane::DeleteDuplicateLaneInfo(BevMapInfo &bev_map) {
    for (auto &lane : bev_map.lane_infos) {
        for (size_t i = 1; i < lane.line_points.size(); i++)
        {
          if (static_cast<int>(lane.line_points[i].point_source) <= 2)
            continue;

          if (fabs(lane.line_points[i].x - lane.line_points[i - 1].x) < 0.01 &&
              fabs(lane.line_points[i].y - lane.line_points[i - 1].y) < 0.01) {
            lane.line_points.erase(lane.line_points.begin() + i);
            i--;
          }
        }        
    }
}

void ExtendLane::BoundLaneMarkerWithLane(BevMapInfo &bev_map) {
  for (auto &lane : bev_map.lane_infos) {
    uint32_t leftLaneMarkerId;
    leftLaneMarkerId = lane.left_lane_marker_id;
    uint32_t rightLaneMarkerId;
    rightLaneMarkerId = lane.right_lane_marker_id;
    std::set<uint64_t> leftLaneIds;
    std::set<uint64_t> rightLaneIds;
    for (auto laneMarker : bev_map.lanemarkers) {
      int remainder = laneMarker.id % static_cast<uint32_t>(1e3);
      if (remainder == leftLaneMarkerId) {
        leftLaneIds.insert(laneMarker.id);
      }
      if (remainder == rightLaneMarkerId) {
        rightLaneIds.insert(laneMarker.id);
      }
    }
    for (auto &section : bev_map.route.sections) {
      for (auto &laneMarker : section.lanemarkers) {
        int remainder = laneMarker.id % static_cast<uint32_t>(1e3);
        if (remainder == leftLaneMarkerId) {
          leftLaneIds.insert(laneMarker.id);
        }
        if (remainder == rightLaneMarkerId) {
          rightLaneIds.insert(laneMarker.id);
        }
      }
    }
    lane.left_lane_boundary_ids.clear();
    for (auto idx : leftLaneIds) {
      lane.left_lane_boundary_ids.emplace_back(idx);
    }
    lane.right_lane_boundary_ids.clear();
    for (auto idx : rightLaneIds) {
      lane.right_lane_boundary_ids.emplace_back(idx);
    }
  }
}

void ExtendLane::FindCrossMinValue(BevMapInfo &bev_map)
{
    m_corssMinValue = std::numeric_limits<double>::max();

    // 通用 Lambda：找出 vector<BevLaneMarker> 中最小 x 坐标点
    auto update_min_x = [&](const std::vector<BevLaneMarker> &lanes) {
        for (const auto &lane : lanes) {
            for (const auto &point : lane.line_points) {
                Eigen::Vector3d pt_global{point.x, point.y, 0.0};
                Eigen::Vector3d pt_ego = T_ego_local_ * pt_global;
                if (pt_ego.x() < m_corssMinValue) {
                    m_corssMinValue = pt_ego.x();
                }
            }
        }
    };

    // 分别处理 stop_lines, junctions, crosswalks
    update_min_x(bev_map.stop_lines);
    update_min_x(bev_map.junctions);
    update_min_x(bev_map.crosswalks);
}

void ExtendLane::CutLaneInfoOutCrossRoad(BevMapInfo &bev_map) {
  //更新crosswalk
  // AINFO << "time:" << std::setprecision(15) << bev_map.header.timestamp;
  // crosswalk_tracker_ptr_->PrintInputMarkers(bev_map.crosswalks);
  // crosswalk_tracker_ptr_->PushUpdate(bev_map.crosswalks,T_ego_local_);
  // auto stable_crosswalks  = crosswalk_tracker_ptr_->GetConfirmedCrosswalks();
  // crosswalk_tracker_ptr_->PrintTrackedCrosswalks();
  // crosswalk_tracker_ptr_->PrintInputMarkers(stable_crosswalks);

  auto filter_lane_points = [&](std::vector<Point2DF> &points, bool debug = false) {
    if (points.empty() || static_cast<int>(points.back().point_source) < 3) {
      return;
    }
    if (points.size() > 150)
    {
      return;
    }    

    //如果延长线段完全未经路口，不删减
    bool have_been_crossed = false;
    std::set<int> filteredIdxs;
    const int     inside_count_thresh = 10;
    bool begin_filtered = false;
    std::vector<Eigen::Vector3d> pointsGlobalSeg;
    int last_filtered_idx = 0;
    bool cut_by_edge = false;
    for (int i = 0; i < points.size(); ++i) {
      if (static_cast<int>(points[i].point_source) <= 2)
        continue;

      if (last_filtered_idx == 0)
        last_filtered_idx = i;

      if (pointsGlobalSeg.empty()) {
        int infer_cnt = std::min(4, static_cast<int>(i) + 1);
        for (int j = infer_cnt - 1; j >= 0; --j) {
          int idx = i - j;
          // 保险一点, idx >= 0 且 idx < points.size()
          if (idx >= 0 && idx < points.size()) {
            Eigen::Vector3d insertPt(points[idx].x, points[idx].y, 0.0);
            pointsGlobalSeg.emplace_back(insertPt);
          }
        }
      }
      Eigen::Vector3d pointGlobal{points[i].x, points[i].y, 0.0};
      pointsGlobalSeg.emplace_back(pointGlobal);

      //&bev_map.crosswalks/&stable_crosswalks
      std::vector<const std::vector<BevLaneMarker> *> region_groups = {&bev_map.junctions, &bev_map.stop_lines, &bev_map.crosswalks ,&bev_map.edges};
      for (size_t i = 0; i < region_groups.size(); ++i) {
        const auto *regions = region_groups[i];
        for (const auto &marker : *regions) {
          //过滤激光路沿
          if(i == 3 && marker.id > 100){
            continue;
          }
          if (IsLineSegmentsCrossPolygon(pointsGlobalSeg, marker.line_points, begin_filtered)) {
            have_been_crossed = true;
            if (i == 3)
              cut_by_edge = true;
            break;
          } else if (begin_filtered) {
            break;
          }

          if (debug && marker.id == 108) {
            AINFO << " begin_filtered: " << begin_filtered << " have_been_crossed: " << have_been_crossed;
            AINFO << "marker. begin_point: " << marker.line_points[0].x << " " << marker.line_points[0].y;
            AINFO << "marker. end_point: " << marker.line_points[marker.line_points.size() - 1].x << " "
                  << marker.line_points[marker.line_points.size() - 1].y;
            AINFO << "pointGlobal begin: " << pointsGlobalSeg[0].x() << " " << pointsGlobalSeg[0].y();
            AINFO << "pointGlobal end: " << pointsGlobalSeg[pointsGlobalSeg.size() - 1].x() << " "
                  << pointsGlobalSeg[pointsGlobalSeg.size() - 1].y();
          }
        }
        if (have_been_crossed || begin_filtered)
          break;
      }

      if (begin_filtered)
      {
        filteredIdxs.insert(i);
      }
    }

    if(!have_been_crossed){
      return;
    }

    if (cut_by_edge && last_filtered_idx > 0) {
      points.erase(points.begin()+last_filtered_idx, points.end());
    } else {
      std::vector<Point2DF> filteredPoints;
      for (int i = 0; i < points.size(); ++i) {
        if (filteredIdxs.find(i) == filteredIdxs.end()) {
          filteredPoints.emplace_back(points[i]);
        }
      }

      points = std::move(filteredPoints);
    }
  };

  for (auto &lane : bev_map.lane_infos) {
    filter_lane_points(lane.line_points);
  }
}

void ExtendLane::IsVirtualLine(const BevMapInfo &bev_map)
{
	//AINFO<<"m_virtualLineIdx size "<<m_virtualLineIdxs.size();
    m_virtualLineIdxs.clear();
    for (auto lanemarker : bev_map.lanemarkers)
    {
        auto points = lanemarker.line_points;
        if (points.empty())
        {
            continue;
        }
        if (points.back().type == PointType::BEV_LMT__INVALID)
        {
            m_virtualLineIdxs.insert(lanemarker.id);
        }
    }
}

void ExtendLane::JudgeIsEmergency(BevMapInfo &bev_map)
{
    m_emergencyLaneIds.clear();
    m_emergencyLaneMarkerIds.clear();
    for (const auto &lane : bev_map.lane_infos)
    {
        if (lane.lane_type == cem::message::sensor::BevLaneType::LANE_TYPE_EMERGENCY)
        {
            m_emergencyLaneIds.insert(lane.id);
            for (auto leftLaneMarkerId : lane.left_lane_boundary_ids)
            {
                m_emergencyLaneMarkerIds.insert(leftLaneMarkerId);
            }
            for (auto rightLaneMarkerId : lane.right_lane_boundary_ids)
            {
                m_emergencyLaneMarkerIds.insert(rightLaneMarkerId);
            }
        }
    }
}


double ExtendLane::CalculateLineToLaneDist(std::vector<Eigen::Vector3d> &sourcePts, std::vector<Eigen::Vector3d> &targetPoints,bool debug = false)
{
    if(debug)
    {
      AINFO << "sourcePts size:" << sourcePts.size() <<"from " << sourcePts[0].transpose() << " to " << sourcePts[sourcePts.size() - 1].transpose();
      AINFO << " targetPoints size:" << targetPoints.size() <<"from "<< targetPoints[0].transpose() << " to " << targetPoints[targetPoints.size() - 1].transpose();
    }
    if (sourcePts.empty() || targetPoints.empty())
    {
        return -1;
    }
    int count = 1;
    int outIdx = 0;
    double meanDist = 0.0;
    auto len = targetPoints.size() - 1;
    std::vector<Eigen::Vector2f> tagetPts;
    tagetPts.resize(len + 1);
    std::transform(targetPoints.begin(), targetPoints.end(), tagetPts.begin(),
                   [](const Eigen::Vector3d &pt) { return Eigen::Vector2f(pt.x(), pt.y()); });
    Eigen::Vector2f foot;
    for (auto pt : sourcePts)
    {
        Eigen::Vector2f srcPoint;
        srcPoint(0) = pt(0);
        srcPoint(1) = pt(1);
        double dist = std::numeric_limits<double>::max();
        // 计算srcPoint 在targetPts中的索引 二分法查找第一个大于srcPoint(0)的索引
        int left = 0;
        int right = len;
        while (left < right)
        {
            int mid = left + (right - left) / 2;
            if (targetPoints[mid](0) < pt(0))
            {
                left = mid + 1;
            }
            else
            {
                right = mid;
            }
        }
        auto startIndex = (left - 10) > 0 ? (left - 10) : 0; // 附近10个点
        auto endIndex = (left + 10) < len ? (left + 10) : len;
        bool flag = GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2f>(srcPoint, tagetPts, startIndex, endIndex, false, foot, dist, outIdx, 0.3f);
        if (debug){
          AINFO << "flag: " << flag << " dist: " << dist << " foot: " << foot.transpose() << " outIdx: " << outIdx;
        }
        if (flag && dist < kEmergencyFilterMaxWidth)
        {
            meanDist += dist;
            count++;
        }
    }
    if (debug) {
      AINFO << "count: " << count;
    }

    meanDist /= count;
    return meanDist;
}

void FilterTailOutliersFromHeadInplace(std::vector<Point2DF>& points, int max_segments = 10, double cos_thresh = 0.94) {
  int n = points.size();
  if (n < 3) return;  // 至少三个点才能计算两个方向段

  int tail_len = std::min(max_segments + 1, n);
  int start_idx = n - tail_len;

  Eigen::Vector2f last_dir;
  bool has_last_dir = false;

  for (int i = start_idx + 1; i < points.size(); ) {
    Eigen::Vector2f dir(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    if (dir.norm() < 1e-4) {
      // 太短，直接删
      points.erase(points.begin() + i);
      continue;
    }
    dir.normalize();

    if (!has_last_dir) {
      last_dir = dir;
      has_last_dir = true;
      ++i;
    } else {
      double cos_theta = dir.dot(last_dir);
      if (cos_theta < cos_thresh) {
        // 当前点为异常点，移除
        points.erase(points.begin() + i);
        continue;  // 不递增 i
      } else {
        last_dir = dir;
        ++i;
      }
    }
  }
}

void ExtendLane::ExtendLaneInfo(BevMapInfo &bev_map) {
  Eigen::Isometry3d Tbw = T_local_ego_.inverse();

  // 第一步：提前收集所有 lane 的最长 lanemarker 的本地坐标点
  std::unordered_map<uint64_t, std::vector<Point2DF>> lane_to_extend_points;
  bool debug = bev_map.header.timestamp > 1753516579.6 && bev_map.header.timestamp < 1753516579.8;
  uint64_t debug_id = 97;

  for (auto &lane : bev_map.lane_infos) {
    if (m_emergencyLaneIds.find(lane.id) != m_emergencyLaneIds.end() || !lane.next_lane_ids.empty() || lane.line_points.empty() ||
        lane.is_blocked)
      continue;
    FilterTailOutliersFromHeadInplace(lane.line_points);
    if (lane.line_points.size() < 5) {
      continue;
    }

    Eigen::Vector3d frontLinePt{lane.line_points.front().x, lane.line_points.front().y, 0.0};
    Eigen::Vector3d backLinePt{lane.line_points.back().x, lane.line_points.back().y, 0.0};
    double          lineLength = (backLinePt - frontLinePt).squaredNorm();
    if (lineLength < 25.0) {
      continue;
    }
    Eigen::Vector3d laneBackPointLoc = Tbw * backLinePt;

    // 找出与当前 lane 的同道、左一、右一 lane 的 lanemarker（ego&相邻车道）
    std::vector<uint64_t> id_vector = {lane.left_lane_marker_id, lane.right_lane_marker_id};
    //对比egolane&相邻车道尾点曲率变化筛选
    Eigen::Vector3d lanefrontPointLoc = Tbw * Eigen::Vector3d(lane.line_points[lane.line_points.size()-5].x, lane.line_points[lane.line_points.size()-5].y, 0.0);
    double ego_delta_y_loc = (laneBackPointLoc[1] - lanefrontPointLoc[1]) * 10 / 5;

    auto insert_marker_ids = [&](const uint64_t lane_id) {
      if (lane_id == 0)
        return;
      auto itr = std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(), [lane_id](const auto &m) { return m.id == lane_id; });
      if (itr == bev_map.lane_infos.end())
        return;
      if (itr->line_points.size() < 5)
        return;
      
      //算尾点曲率变化
      Eigen::Vector3d lane2frontPointLoc = Tbw * Eigen::Vector3d(itr->line_points[itr->line_points.size() - 5].x,
                                                                itr->line_points[itr->line_points.size() - 5].y, 0.0);
      Eigen::Vector3d lane2backPointLoc = Tbw * Eigen::Vector3d(itr->line_points.back().x,
                                                                itr->line_points.back().y, 0.0);
      double ego_delta_y_loc2 = (lane2backPointLoc[1] - lane2frontPointLoc[1]) * 10 / 5;
      if (fabs(ego_delta_y_loc2- ego_delta_y_loc)>1.5)
      {
        // AINFO <<"@@@@@time:"<<std::setprecision(15)<<bev_map.header.timestamp;
        // AINFO << "tgt id:" << itr->id << ",srs id:" << lane.id;
        return;
      }      

      if (itr != bev_map.lane_infos.end()) {
        if (itr->left_lane_marker_id > 0) {
          uint64_t lm_id = itr->left_lane_marker_id;
          auto     lm    = std::find_if(id_vector.begin(), id_vector.end(), [lm_id](const uint64_t &lmid) { return lmid == lm_id; });

          if (lm == id_vector.end())
            id_vector.push_back(itr->left_lane_marker_id);
        }
        if (itr->right_lane_marker_id > 0){
          uint64_t lm_id = itr->right_lane_marker_id;
          auto     lm    = std::find_if(id_vector.begin(), id_vector.end(), [lm_id](const uint64_t &lmid) { return lmid == lm_id; });

          if (lm == id_vector.end())
            id_vector.push_back(itr->right_lane_marker_id);
        }
      }
    };
    insert_marker_ids(lane.left_lane_id);
    insert_marker_ids(lane.right_lane_id);

    uint64_t ref_marker_id = 0;
    double max_x = std::numeric_limits<double>::lowest();
    for (uint64_t target_id : id_vector) {
      auto it = std::find_if(bev_map.lanemarkers.begin(), bev_map.lanemarkers.end(),
                             [target_id](const BevLaneMarker &marker) { return marker.id == target_id; });

      if (it == bev_map.lanemarkers.end())
        continue;

      const auto &marker = *it;
      if (marker.line_points.empty())
        continue;
      if (m_virtualLineIdxs.count(marker.id)) {
        AINFO << "marker id: " << marker.id << " is virtual line";
        continue;
      }

      Eigen::Vector3d back_pt{marker.line_points.back().x, marker.line_points.back().y, 0.0};
      Eigen::Vector3d back_pt_local = Tbw * back_pt;

      if (back_pt_local.x() > max_x) {
        max_x = back_pt_local.x();
        ref_marker_id = marker.id;
      }
      //足够长，提前退出
      if (back_pt_local.x() > 100)
      {
        break;
      }      
    }
    if (debug && lane.id == debug_id) {
      AINFO << "ref_marker_id: " << ref_marker_id << " max_x: " << max_x;
      for (const auto id : id_vector) {
        AINFO << "id_set: " << id;
      }
      AINFO << "lane.left_lane_id: " << lane.left_lane_id << " lane.right_lane_id: " << lane.right_lane_id;
    }

    if (ref_marker_id == 0 || laneBackPointLoc.x() > max_x) continue;

    // 提取 marker 点
    for (const auto &marker : bev_map.lanemarkers) {
      if (marker.id != ref_marker_id) continue;
      std::vector<Point2DF> pt_vec;
      for (const auto &pt : marker.line_points) {
        pt_vec.emplace_back(pt);
      }
      FilterTailOutliersFromHeadInplace(pt_vec);
      lane_to_extend_points[lane.id] = pt_vec;
      break;
    }
  }

  // 第二步：延长 lane
  for (auto &lane : bev_map.lane_infos) {
    if (!lane_to_extend_points.count(lane.id)) continue;
    if (lane.line_points.empty()) continue;

    auto& longest_pts = lane_to_extend_points[lane.id];
    Eigen::Vector3d back_pt{lane.line_points.back().x, lane.line_points.back().y, 0.0};
    Eigen::Vector3d back_pt_local = Tbw * back_pt;

    if(debug && lane.id == debug_id){
      AINFO << "marker local points: " << longest_pts.size();
      AINFO << "from :" << longest_pts.front().x << " " << longest_pts.front().y;
      AINFO << "to :" << longest_pts.back().x << " " << longest_pts.back().y;
      AINFO << "back_pt_local: " << back_pt_local.x() << " " << back_pt_local.y();
    }

    double dist = 0;

    std::vector<Point2DF> new_points;
    if (longest_pts.size() < 2)
      continue;  // 至少要有两个点
    for (size_t i = 0; i + 1 < longest_pts.size(); ++i) {
      Eigen::Vector3d longest_pt = Tbw * Eigen::Vector3d{longest_pts[i].x, longest_pts[i].y, 0.0};
      if ((back_pt_local.x() + 1.0) < longest_pt.x()) {
        if (dist == 0) {
          dist = back_pt_local.y() - longest_pt.y();  
        }
        Eigen::Vector3d extend_pt = {longest_pt.x(), longest_pt.y(), 0.0};

        // 基于相对左右判断方向
        extend_pt.y() += dist;
        if (debug && lane.id == debug_id) {
          AINFO << "infer point local:" << extend_pt.transpose();
          AINFO << " dist:" << dist;
        }

        Eigen::Vector3d global_pt = T_local_ego_ * extend_pt;
        Point2DF output_pt;
        output_pt.x = global_pt.x();
        output_pt.y = global_pt.y();
        output_pt.mse = NAN;
        output_pt.point_source = cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANEMARKER;
        new_points.emplace_back(output_pt);

        if (debug && lane.id == debug_id) {
          AINFO << "add global points: " << output_pt.x << " " << output_pt.y;
        }
      }
    }
    
    if (lane.line_points.size() + new_points.size() > 1e6) {
      AERROR << "Too many points to insert, skipping lane " << lane.id;
      continue;
    }
    lane.line_points.insert(lane.line_points.end(), new_points.begin(), new_points.end());
  }
}

void ExtendLane::FitAndExtendLanePoints(const std::vector<Eigen::Vector3d> &pointsVec, double maxX,
                                        std::vector<Point2DF>   &target_points,
                                        std::unordered_map<uint64_t, Eigen::VectorXd> &coeff_map, uint64_t id, bool extend_to_cross) {
  int n = pointsVec.size();
  if (n < 3) {
    return;
  }

  // // 原来的拟合代码仍保留，可用于其他用途
  // Eigen::VectorXd coeff(3);
  // Eigen::MatrixXd A(n, 3);
  // Eigen::VectorXd b(n);
  // for (int i = 0; i < n; ++i) {
  //   double x = pointsVec[i].x();
  //   A(i, 0) = x * x;
  //   A(i, 1) = x;
  //   A(i, 2) = 1.0;
  //   b(i) = pointsVec[i].y();
  // }
  // coeff = A.colPivHouseholderQr().solve(b);
  // coeff_map[id] = coeff;
  // ==== 使用局部方向向量延伸替代拟合延伸 ====
  const int use_k_tail_points = std::min(5, n - 1);  // 使用尾部最多5段线
  Eigen::Vector3d weighted_dir = Eigen::Vector3d::Zero();
  double total_weight = 0.0;

  for (int i = 0; i < use_k_tail_points; ++i) {
    int idx1 = n - 2 - i;
    int idx2 = n - 1 - i;
    if (idx1 < 0 || idx2 < 0) break;
    Eigen::Vector3d dir = pointsVec[idx2] - pointsVec[idx1];
    double weight = static_cast<double>(use_k_tail_points - i);  // 越靠后权重越大
    weighted_dir += weight * dir;
    total_weight += weight;
  }

  if (fabs(total_weight) < 1e-6 || weighted_dir.norm() < 1e-6) {
    return; // 防止除以0或方向太小
  }

  weighted_dir.normalize();  // 单位方向向量

  // 延伸点
  const double extension_step = 1.0;
  Eigen::Vector3d current = pointsVec.back();
  Point2DF pt;
  pt.point_source =
      extend_to_cross ? cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANEMARKER
                      : cem::message::common::PointSource::SECTION_EXTENTED_FROM_FIT;
  int count = 0;
  const int count_max = extend_to_cross ? 20 : 10;
  while ((current.x() < maxX)&&count < count_max) {
    count++;
    current += extension_step * weighted_dir;
    Eigen::Vector3d globalPt = T_local_ego_ * current;

    pt.x   = globalPt.x();
    pt.y   = globalPt.y();
    pt.mse = NAN;
    target_points.emplace_back(pt);
  }
}

void ExtendLane::CurveFittingLaneInfo(BevMapInfo &bev_map)
{
    m_laneInfoCoeff.clear();
    Eigen::Isometry3d Tbw = T_local_ego_.inverse();
    double maxValue = std::numeric_limits<double>::lowest();
    uint32_t maxIdx = -1;
    bool debug = bev_map.header.timestamp > 1750505799 && bev_map.header.timestamp < 1750505801;

    for (const auto &lane : bev_map.lane_infos) {
        if (m_emergencyLaneIds.count(lane.id)) continue;
        if (lane.line_points.empty()) continue;

        Eigen::Vector3d backPt{lane.line_points.back().x, lane.line_points.back().y, 0.0};
        double x = (T_ego_local_ * backPt).x();
        if (x > maxValue) {
            maxValue = x;
            maxIdx = lane.id;
        }
    }
    bool extend_cross = false;
    if ((maxValue < m_corssMinValue) && (m_corssMinValue < 100.0)) {
      maxValue = m_corssMinValue;
      maxIdx = 0;
      extend_cross = true;
    }
    for (auto &lane : bev_map.lane_infos) {
      if ((m_virtualLineIdxs.count(lane.left_lane_marker_id) && m_virtualLineIdxs.count(lane.right_lane_marker_id)) ||
          lane.next_lane_ids.size() > 0 || lane.id == maxIdx || lane.is_blocked) {
        continue;
      }

        if (lane.line_points.size() < 3) continue;

        std::vector<Eigen::Vector3d> pointsVec;
        Eigen::Vector3d backPt{lane.line_points.back().x, lane.line_points.back().y, 0.0};
        for (const auto &pt : lane.line_points) {
            Eigen::Vector3d global{pt.x, pt.y, 0.0};
            if ((backPt - global).norm() > 50.0) continue;
            pointsVec.emplace_back(T_ego_local_ * global);
        }

        if (pointsVec.size() < 5) continue;
        double len = (pointsVec.back() - pointsVec.front()).norm();
        if (len < 5.0) continue;

        FitAndExtendLanePoints(pointsVec, maxValue, lane.line_points, m_laneInfoCoeff, lane.id,extend_cross);
        if (debug && lane.id == 90) {
          AINFO << "marker local points: " << lane.line_points.size();
          AINFO << "from:" << lane.line_points.front().x << " " << lane.line_points.front().y;
          AINFO << "to:" << lane.line_points.back().x << " " << lane.line_points.back().y;
        }

        //被相邻lane截断
        auto cut_by_lane = [&](const uint64_t lane_id,std::vector<Point2DF>   &target_points) {
          if (lane_id == 0)
            return;
          auto itr =
              std::find_if(bev_map.lane_infos.begin(), bev_map.lane_infos.end(), [lane_id](const auto &m) { return m.id == lane_id; });
          if (itr == bev_map.lane_infos.end() || itr->line_points.size() < 5)
            return;

          //取FIT点若有相交截断
          const float near_threshold = 1.5;
          auto filter_Itr = target_points.end();
          for (size_t lane_idx = 1; lane_idx < target_points.size(); ++lane_idx) {
            if (static_cast<int>(target_points[lane_idx].point_source) <= 3)//只操作FIT点
              continue;

            for (size_t lane_itr_idx = 0; lane_itr_idx < itr->line_points.size(); lane_itr_idx++)
            {
              if (static_cast<int>(itr->line_points[lane_itr_idx].point_source) > 3)  //只参考LandMarker以上的点
                continue;

              //判断是否相交
              if (fabs(itr->line_points[lane_itr_idx].x - target_points[lane_idx].x)<near_threshold &&
                  fabs(itr->line_points[lane_itr_idx].y - target_points[lane_idx].y)<near_threshold )
              {
                filter_Itr = target_points.begin() + lane_idx;
                break;
              }
            }

            if (filter_Itr != target_points.end())
              break;
          }

          if (filter_Itr != target_points.end())
            target_points.erase(filter_Itr, target_points.end());     
        };
        cut_by_lane(lane.left_lane_id, lane.line_points);
        cut_by_lane(lane.right_lane_id, lane.line_points);
    }    
}

void ExtendLane::SmoothLineInfo(BevMapInfo &bev_map)
{
    for (auto &lane : bev_map.lane_infos)
    {
        auto &points = lane.line_points;
        size_t cut_idx = 0;
        for (size_t idx = 0; idx < points.size(); idx++)
        {
            if (static_cast<int>(points[idx].point_source) > 2)
            {
                cut_idx = idx;
                break;
            }
        }
        if (cut_idx < 1)
        {
            continue;
        }
        double deltaY = points[cut_idx].y - points[cut_idx - 1].y;
        double deltaX = points[cut_idx].x - points[cut_idx - 1].x;
        for (size_t idx = cut_idx; idx < points.size(); idx++)
        {
            points[idx].y -= deltaY;
            points[idx].x -= deltaX;
        }
        points.erase(points.begin() + cut_idx);
    }

}

void ExtendLane::SmoothLineMarkers(BevMapInfo &bev_map)
{
    for (auto &lane : bev_map.lanemarkers)
    {
        auto &points = lane.line_points;
        size_t cutIdx{0};
        for (size_t idx = 0; idx < points.size(); idx++)
        {
            if (static_cast<int>(points[idx].point_source) > 2)
            {
                cutIdx = idx;
                break;
            }
        }
        if (cutIdx < 1)
        {
            continue;
        }
        double deltaX = points[cutIdx].x - points[cutIdx - 1].x;
        double deltaY = points[cutIdx].y - points[cutIdx - 1].y;
        for (size_t idx = cutIdx; idx < points.size(); idx++)
        {
            points[idx].y -= deltaY;
            points[idx].x -= deltaX;
        }
    }
}

}
}
}
