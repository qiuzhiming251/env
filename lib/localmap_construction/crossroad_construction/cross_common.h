/**
 * @file cross_common.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 2.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 *
 */
#ifndef CROSSCOMMON_H
#define CROSSCOMMON_H
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
#include "common/utility.h"
#include <cfloat>
#include <iostream>
#include <string>
#include <type_traits>
#include <stdexcept>

#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"

namespace cem {
namespace fusion {
namespace {
constexpr double cross_epslion = 1e-8;
//
constexpr double kConnectStartDis = 15.0;
constexpr int kFitPointsSize = 25;
constexpr uint64_t kPreLaneId = 101;
constexpr uint64_t kPreLeftLanemarkerId = 66661;
constexpr uint64_t kPreRightLanemarkerId = 66662;
constexpr uint64_t kConnectLaneId = 106;
constexpr uint64_t kConnectLeftLanemarkerId = 88881;
constexpr uint64_t kConnectRightLanemarkerId = 88882;
constexpr bool kStraightUsingLdPoints = false;
constexpr bool kTurnUsingLdPoints = true;
constexpr bool kAroundUsingLdPoints = true;
constexpr bool kStraightUsingExpTrajPoints = false;
constexpr bool kTurnUsingExpTrajPoints = true;
constexpr bool kAroundUsingExpTrajPoints = false;
constexpr bool KUpdateStraightVirtualIncrossroad = false;
constexpr bool KUpdateTurnVirtualIncrossroad = false;
constexpr bool KEnableFindClosetLaneByTopo = false;
constexpr bool KEnableFindClosetLaneByLatErr = true;
constexpr bool virtual_using_only_ld_traj = false;
constexpr bool is_enable_mnoa = true;
constexpr float kThresholdToSDIntersection = 80.0f;
using cem::message::common::Point2DF;
using byd::common::math::Vec2d;
using cem::message::env_model::Curve;

inline int Cross_Compare(const double x, const double y) {
  const double result = x - y;
  if (result < -cross_epslion) {
    return -1;
  }
  if (result > cross_epslion) {
    return 1;
  }
  return 0;
}

bool HasSameTurnType(cem::message::env_model::TurnType turn_type, cem::message::sensor::BevAction ego_action) {
  bool out = false;
  switch (ego_action) {
    case BevAction::STRAIGHT:
      if (turn_type == TurnType::NO_TURN) {
        out = true;
      }
      break;
    case BevAction::LEFT_TURN:
      if (turn_type == TurnType::LEFT_TURN) {
        out = true;
      }
      break;
    case BevAction::RIGHT_TURN:
      if (turn_type == TurnType::RIGHT_TURN) {
        out = true;
      }
      break;
    case BevAction::U_TURN:
      if (turn_type == TurnType::U_TURN) {
        out = true;
      }
      break;
    default:
      break;
  }
  return out;
}

enum class CrossState { INIT = 0, PREDICTION = 1, CONNECTION = 2, END = 3 };
// 0:pre 1:obs 2:SD 3:LD 4. exp_trajectary
enum class VirtualLineSource { PRE = 0, OBS = 1, SD = 2, LD = 3, EXP = 4 };

// 三个作用：1.存储预推点，顶点的角度 2.选车道时进行判断 3.连接时直接使用 
// 4.连接线生成的时候is_virtual=true，要使用points和连接点进行拼接，is_virtual=false不使用points拼接
struct MainLane {
  uint64_t id = 0;
  uint64_t lane_id = 0;
  std::vector<Vec2d> points;
  std::vector<Vec2d> left_points;
  std::vector<Vec2d> right_points;
  Vec2d top_points;
  Vec2d left_top_points;
  Vec2d right_top_points;
  double angle = std::numeric_limits<double>::max();
  cem::message::sensor::BevAction navi_action = cem::message::sensor::BevAction::STRAIGHT;
  bool valid = false;
  bool is_virtual = false;
  void Reset() {
    id = 0;
    lane_id = 0;
    points.clear();
    left_points.clear();
    right_points.clear();
    top_points = Vec2d();
    left_top_points = Vec2d();
    right_top_points = Vec2d();
    angle = std::numeric_limits<double>::max();
    cem::message::sensor::BevAction navi_action = cem::message::sensor::BevAction::STRAIGHT;
    valid = false;
    is_virtual = false;
  };
};

struct TopLane {
  uint64_t id = 10001;
  uint64_t lane_id = 10001;
  bool is_merged{false};
  std::vector<Vec2d> points;
  std::vector<Vec2d> left_points;
  std::vector<Vec2d> right_points;
  cem::message::sensor::BevAction navi_action = 
                      cem::message::sensor::BevAction::UNKNOWN;
  cem::message::sensor::BevTurnType turn_type = 
                      cem::message::sensor::BevTurnType::NO_TURN;
  cem::message::sensor::BevLaneDirection direction =
                      cem::message::sensor::BevLaneDirection::DIRECTION_FORWARD;
  bool is_connect_lane{false};
  bool is_ego{false};
  bool is_target{false};
  bool valid = false;
  
  void Reset() {
    id = 0;
    lane_id = 0;
    points.clear();
    left_points.clear();
    right_points.clear();
    valid = false;
    is_connect_lane = false;
    is_ego = false;
  };
  TopLane& operator=(const TopLane& other) {
    if (this != &other) {  // 防止自赋值
        id = other.id;
        lane_id = other.lane_id;
        is_merged = other.is_merged;
        points = other.points;            // vector::operator= 自动深拷贝
        left_points = other.left_points;
        right_points = other.right_points;
        navi_action = other.navi_action;
        turn_type = other.turn_type;
        direction = other.direction;
        is_connect_lane = other.is_connect_lane;
        is_ego = other.is_ego;
        is_target = other.is_target;
        valid = other.valid;
    }
    return *this;
}

};

struct EgoRoadLanes {
  uint64_t                        ego_road_id     = 10001;
  uint64_t                        ego_lane_id     = 10001;
  cem::message::sensor::BevAction geo_navi_action = cem::message::sensor::BevAction::UNKNOWN;
  std::vector<uint64_t>           lane_ids;
  std::vector<uint64_t>           same_turn_lane_ids;
  int                             left_same_turn_size{0};
  int                             right_same_turn_size{0};
  bool                            is_one_lane{true};
  void                            Clear() {
    ego_road_id     = 10001;
    ego_lane_id     = 10001;
    geo_navi_action = cem::message::sensor::BevAction::UNKNOWN;
    lane_ids.clear();
    same_turn_lane_ids.clear();
    left_same_turn_size  = 0;
    right_same_turn_size = 0;
    is_one_lane          = true;
  }
};

struct Opening {
  int    lane_size{0};
  double dis{0.0};
  double heading{0.0};
  double left_y{0.0};
  double right_y{0.0};
  bool   has_left_edge{false};
  bool   has_right_edge{false};
  bool   is_valid{false};
  bool   has_heading{false};
  bool   has_top_lane{false};
  
  std::vector<uint64_t>    lane_ids{};
  Vec2d sd_cross_top_point{};
  // 给出所有路口内的lane，从左到右排序
  std::vector<TopLane> top_lanes{};

  void Clear() {
    lane_size = 0;
    dis       = 0.0;
    heading   = 0.0;
    left_y    = 0.0;
    right_y   = 0.0;
    lane_ids.clear();
    sd_cross_top_point = Vec2d();
    is_valid           = false;
    has_left_edge      = false;
    has_right_edge     = false;
    has_heading        = false;
    has_top_lane       = false;
    top_lanes.clear();
  }
};

struct CrossEdge{
  uint64_t id{0};
  std::vector<Vec2d> points{};
  void Clear(){
    id = 0;
    points.clear();
  }
};

struct OccEdge{
  uint64_t occ_edge_id{0};
  double max_left_dis = 0.0;
  double min_right_dis = 0.0;
  std::vector<cv::Point2f> occ_points;
  std::vector<int>  sd_indexs;
  std::vector<double> left_distances;
  std::vector<double> right_distances;
};
struct OccEdgePoint {
  cv::Point2f occ_point;
  uint64_t occ_edge_id{0};
  int occ_right_index{0};
  int occ_left_index{0};
  double left_distance = 0.0;
  double right_distance = 0.0;
};

struct SdCrossOccEdgePoint {
  Vec2d sd_point;
  double heading{0.0};
  double left_distance{100.0};
  double right_distance{-100.0};
  Vec2d right_point;
  std::vector<OccEdgePoint> occ_points;
  bool is_same_heading{true};
};

struct OccSdEdgesInfo {
  std::vector<SdCrossOccEdgePoint> sd_edge_points;
  int last_is_diff_heading_index = 0;
  int end_point_sd_index = 0;
  double top_left_min_distance = 0.0;
  double top_right_min_distance = 0.0;
  double max_main_left_x = 0.0;
  bool is_valid{false};
  void Clear(){
    sd_edge_points.clear();
    last_is_diff_heading_index = 0;
    end_point_sd_index = 0;
    top_left_min_distance = 0.0;
    top_right_min_distance = 0.0;
    max_main_left_x = 0.0;
    is_valid = false;
  }
};

struct OccOpening {
  int occ_index = 0;
  double min_lanewidth = -1.0;
  double min_left_distance = 1001.0;
  double min_right_distance = -1001.0;
  double mid_distance = 0.0;
  bool left_valid = false;
  bool right_valid = false;
};

struct CrossRoadCenterLine{
  uint64_t id{0};
  double heading{0.0};
  int index{0};
  cem::message::env_model::Point point;
};

struct CrossRoadCenterBackupLane{
  uint64_t id{0};
  uint64_t next_id{0};
  double distance{100.0};
  double heading{0.0};
  double next_lane_heading{0.0};
  double lat_err{0.0};
  int index{0};
  int next_lane_index{0};
  cem::message::env_model::LaneInfo lane;
  std::vector<Point2DF> points;
};

struct CrossRoadCenterSelect{
  int bev_egolane_index{0};
  int ld_egolane_index{0};
  int before_crossroad_bev_lane_size{0};
  int before_crossroad_ld_lane_size{0};
  int after_crossroad_bev_lane_size{0};
  int after_crossroad_ld_lane_size{0};
  double before_bev_lane_heading{0.0};
  double before_ld_lane_heading{0.0};
  double after_bev_lane_heading{0.0};
  double after_ld_lane_heading{0.0};
  std::vector<CrossRoadCenterLine> before_bev_lanes;
  std::vector<CrossRoadCenterLine> before_ld_lanes;
  std::vector<CrossRoadCenterLine> after_bev_lanes;
  std::vector<CrossRoadCenterLine> after_ld_lanes;
  std::vector<CrossRoadCenterBackupLane> ld_backup_lanes;
};

inline bool IsZero(const double x) { return x < cross_epslion && x > -cross_epslion; }

inline int ComparedToZero(const double x) { return Cross_Compare(x, cross_epslion); }

bool IsBoundingBoxOverlap(const Vec2d &A, const Vec2d &B, const Vec2d &C, const Vec2d &D) {
  return (max(A.x(), B.x()) >= min(C.x(), D.x()) - cross_epslion && max(C.x(), D.x()) >= min(A.x(), B.x()) - cross_epslion &&
          max(A.y(), B.y()) >= min(C.y(), D.y()) - cross_epslion && max(C.y(), D.y()) >= min(A.y(), B.y()) - cross_epslion);
}

double CrossProduct(const Vec2d& O, const Vec2d& A, const Vec2d& B) {
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

bool IsStraddleTestPass(const Vec2d &A, const Vec2d &B, const Vec2d &C, const Vec2d &D) {
  double cp1 = CrossProduct(A, B, C);
  double cp2 = CrossProduct(A, B, D);
  double cp3 = CrossProduct(C, D, A);
  double cp4 = CrossProduct(C, D, B);

  // 处理浮点精度问题
  if (std::fabs(cp1) < cross_epslion)
    cp1 = 0;
  if (std::fabs(cp2) < cross_epslion)
    cp2 = 0;
  if (std::fabs(cp3) < cross_epslion)
    cp3 = 0;
  if (std::fabs(cp4) < cross_epslion)
    cp4 = 0;

  return (cp1 * cp2 <= cross_epslion) && (cp3 * cp4 <= cross_epslion);
}

bool LineSegmentsIntersect(const Vec2d &A, const Vec2d &B, const Vec2d &C, const Vec2d &D, Vec2d &intersect) {
  // 快速排斥实验
  if (!IsBoundingBoxOverlap(A, B, C, D))
    return false;

  // 跨立实验
  if (!IsStraddleTestPass(A, B, C, D))
    return false;

  // 计算参数方程
  double denominator = (B.x() - A.x()) * (D.y() - C.y()) - (B.y() - A.y()) * (D.x() - C.x());
  if (fabs(denominator) < cross_epslion) {  // 平行或共线
    return false;                           // 可根据需求扩展共线处理
  }

  double t = ((C.x() - A.x()) * (D.y() - C.y()) - (C.y() - A.y()) * (D.x() - C.x())) / denominator;
  double s = ((C.x() - A.x()) * (B.y() - A.y()) - (C.y() - A.y()) * (B.x() - A.x())) / denominator;

  // 验证参数范围
  if (t < -cross_epslion || t > 1.0 + cross_epslion || s < -cross_epslion || s > 1.0 + cross_epslion)
    return false;

  // 计算交点坐标
  intersect.set_x(A.x() + t * (B.x() - A.x()));
  intersect.set_y(A.y() + t * (B.y() - A.y()));
  return true;
}

std::vector<Vec2d> FindIntersectionPoints(const std::vector<Point2DF> &poly, Vec2d A, Vec2d B) {
  std::vector<Vec2d> result;

  int n = poly.size();
  for (int i = 0; i < n; ++i) {
    Vec2d C(poly[i].x, poly[i].y);
    Vec2d D(poly[(i + 1) % n].x, poly[(i + 1) % n].y);
    Vec2d P;
    if (LineSegmentsIntersect(A, B, C, D, P)) {  // 调用上述相交检测函数
      result.push_back(P);
    }
  }
  return result;
}

/// 求pt与向量p0p1的距离.
template <typename T, int ROWS>
T PointToVectorDist(const Eigen::Matrix<T, ROWS, 1>& p0, const Eigen::Matrix<T, ROWS, 1>& p1,
                    const Eigen::Matrix<T, ROWS, 1>& pt) {
  Eigen::Matrix<T, ROWS, 1> p0_p1 = p1 - p0;
  p0_p1.normalize();
  Eigen::Matrix<T, ROWS, 1> pt_p0 = p0 - pt;
  Eigen::Matrix<T, ROWS, 1> d = pt_p0 - (pt_p0.transpose() * p0_p1) * p0_p1;
  return d.norm();
}

float PointToVectorDist(const Vec2d &p0, const Vec2d &p1, const cv::Point2f &pt) {
  Eigen::Matrix<float, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  p0_p1.normalize();
  Eigen::Matrix<float, 2, 1> pt_p0(p0.x() - pt.x, p0.y() - pt.y);
  Eigen::Matrix<float, 2, 1> d = pt_p0 - (pt_p0.transpose() * p0_p1) * p0_p1;
  return d.norm();
}

float PointToVectorDist(const Vec2d &p0, const Vec2d &p1, const Point2DF &pt) {
  Eigen::Matrix<float, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  p0_p1.normalize();
  Eigen::Matrix<float, 2, 1> pt_p0(p0.x() - pt.x, p0.y() - pt.y);
  Eigen::Matrix<float, 2, 1> d = pt_p0 - (pt_p0.transpose() * p0_p1) * p0_p1;
  return d.norm();
}

template <typename T, typename M>
float PointToVectorDist(const T& p0, const T& p1, const M& pt) {
  Eigen::Matrix<float, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  p0_p1.normalize();
  Eigen::Matrix<float, 2, 1> pt_p0(p0.x() - pt.x(), p0.y() - pt.y());
  Eigen::Matrix<float, 2, 1> d = pt_p0 - (pt_p0.transpose() * p0_p1) * p0_p1;
  return d.norm();
}

template <typename T, typename M>
double PointToLineDis(const T& line, const M& pt) {
  if (line.size() < 2) {
    return -1.0;
  }
  auto it = std::find_if(line.begin() + 1, line.end(), [&](const auto& a) { return a.x() > pt.x(); });
  if (it == line.end()) {
    return -1.0;
  }
  return PointToVectorDist(*(it - 1), *it, pt);
}

/// 判断pt在向量p0p1的左侧还是右侧.
/// 返回值：
///  < 0：pt在p0p1左侧；
///  > 0：pt在p0p1右侧；
///  = 0：pt在p0p1所在直线上.
// template <typename T>
// T PointInVectorSide(const Eigen::Matrix<T, 2, 1>& p0, const Eigen::Matrix<T, 2, 1>& p1,
//                     const Eigen::Matrix<T, 2, 1>& pt) {
//   Eigen::Matrix<T, 2, 1> p0_p1 = p1 - p0;
//   Eigen::Matrix<T, 2, 1> p0_pt = pt - p0;

//   T cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
//   return cross;
// }

// template <typename T, typename M>
// double PointInVectorSide(const T& p0, const T& p1, const M& pt) {
//   Eigen::Matrix<double, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
//   Eigen::Matrix<double, 2, 1> p0_pt(pt.x() - p0.x(), pt.y() - p0.y());

//   double cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
//   return cross;
// }

/// 判断pt在向量p0p1的左侧还是右侧.
/// 返回值：
///  < 0：pt在p0p1左侧；
///  > 0：pt在p0p1右侧；
///  = 0：pt在p0p1所在直线上.
template <typename T, typename M>
double PointInVectorSide(const T &p0, const T &p1, const M &pt) {
  Eigen::Matrix<double, 2, 1> p0_p1(p1.x - p0.x, p1.y - p0.y);
  Eigen::Matrix<double, 2, 1> p0_pt(pt.x - p0.x, pt.y - p0.y);
  double                      cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

double PointInVectorSide(const Vec2d &p0, const Vec2d &p1, const Vec2d &pt) {
  Eigen::Matrix<double, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  Eigen::Matrix<double, 2, 1> p0_pt(pt.x() - p0.x(), pt.y() - p0.y());
  double                      cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

template <typename T>
double PointInVectorSide(const Vec2d &p0, const Vec2d &p1, const T &pt) {
  Eigen::Matrix<double, 2, 1> p0_p1(p1.x() - p0.x(), p1.y() - p0.y());
  Eigen::Matrix<double, 2, 1> p0_pt(pt.x - p0.x(), pt.y - p0.y());
  double                      cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

double PointInVectorSide(const Point2DF &p0, const Point2DF &p1, const Vec2d &pt) {
  Eigen::Matrix<double, 2, 1> p0_p1(p1.x - p0.x, p1.y - p0.y);
  Eigen::Matrix<double, 2, 1> p0_pt(pt.x() - p0.x, pt.y() - p0.y);
  double                      cross = p0_pt.x() * p0_p1.y() - p0_p1.x() * p0_pt.y();
  return cross;
}

template <typename T, typename M>
bool PointInLineRight(const T& line, const M& pt) {
  if (line.size() < 2) {
    return false;
  }
  auto it = std::lower_bound(line.begin(), line.end() - 2, pt.x(), [&](const auto& a, double b) { return a.x() < b; });
  if (it == line.end()) {
    return false;
  }
  return PointInVectorSide(*it, *(it + 1), pt) > 0;
}

byd::common::math::Vec2d InterpolatePoint(const byd::common::math::Vec2d& start, const byd::common::math::Vec2d& end,
                                          double length, double s) {
  byd::common::math::Vec2d new_point;
  double weight = s / length;
  double x = (1 - weight) * start.x() + weight * end.x();
  double y = (1 - weight) * start.y() + weight * end.y();
  new_point.set_x(x);
  new_point.set_y(y);
  return new_point;
}

std::vector<byd::common::math::Vec2d> InterpolateVec2dPoints(const std::vector<byd::common::math::Vec2d>& raw_points,
                                                             double delta_s) {
  if (raw_points.size() <= 2) {
    return raw_points;
  }
  std::vector<byd::common::math::Vec2d> new_points;
  new_points.push_back(raw_points.front());
  double sum_length = 0;
  for (int i = 0; i < raw_points.size() - 1; i++) {
    sum_length += raw_points.at(i).DistanceTo(raw_points.at(i + 1));
  }
  if (sum_length <= delta_s) {
    new_points.push_back(raw_points.back());
    return new_points;
  }
  byd::common::math::Vec2d now_point;
  byd::common::math::Vec2d end_point;
  double total_length = 0;
  for (int i = 0; i < raw_points.size() - 1; i++) {
    now_point = raw_points.at(i);
    end_point = raw_points.at(i + 1);
    const double point_distance = now_point.DistanceTo(end_point);
    total_length += point_distance;
    if (Cross_Compare(total_length, delta_s) <= 0) {
      continue;
    }
    byd::common::math::Vec2d interpolate_point;
    double s = delta_s;
    /*
    **     |<-------total_length-------->|
    **     |<----------s-------->|
    **                |<-point_distance->|
    **-----*----------*----------.-------*---------
    **     ^       now_point     ^   end_point
    **     |<------delta_s------>|
    **                    interpolate_point
    */
    while (s < total_length) {
      interpolate_point = InterpolatePoint(now_point, end_point, point_distance, point_distance - (total_length - s));
      new_points.push_back(interpolate_point);
      s += delta_s;
    }
    total_length = interpolate_point.DistanceTo(end_point);
  }
  return new_points;
}

void FitLaneLinePoint(const std::vector<byd::common::math::Vec2d>& pts, std::vector<double>* c) {
  int n = static_cast<int>(pts.size());
  Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
  Eigen::VectorXd b(n);
  for (int i = 0; i < n; ++i) {
    double xi = pts[i].x();
    double yi = pts[i].y();
    A(i, 0) = 1.0;
    A(i, 1) = xi;
    A(i, 2) = xi * xi;
    A(i, 3) = xi * xi * xi;
    b[i] = yi;
  }
  // x = (A.transpose() * A).inverse() * A.transpose() * b;
  Eigen::VectorXd coeffs = A.householderQr().solve(b);
  c->emplace_back(coeffs[0]);
  c->emplace_back(coeffs[1]);
  c->emplace_back(coeffs[2]);
  c->emplace_back(coeffs[3]);
}

inline std::vector<Vec2d> TranslateSegmentAlongAxisStart(Vec2d axis, const std::vector<Vec2d> &seg1, double distance) {
  const Vec2d        translation = axis * distance;
  std::vector<Vec2d> out_segs;
  out_segs.reserve(seg1.size());
  for (const auto &seg : seg1) {
    out_segs.emplace_back(seg + translation);
  }
  return out_segs;
}

inline void TranslateSegmentAlongAxisStart(Vec2d axis, std::vector<Point2DF> &seg1, double distance) {
  const Vec2d translation = axis * distance;
  for (auto &seg : seg1) {
    Vec2d vec = Vec2d(seg.x, seg.y) + translation;
    seg.x     = vec.x();
    seg.y     = vec.y();
  }
}

class FirstOrderLowerPassFilter {
 public:
  FirstOrderLowerPassFilter()  = default;
  ~FirstOrderLowerPassFilter() = default;
  double Filter(double x_insert) {
    double output = 0.0;
    if (is_first_step_) {
      output         = x_insert;
      is_first_step_ = false;
    } else {
      output = filter_coefficient_ * x_insert + (1 - filter_coefficient_) * last_output_;
    }
    last_output_ = output;
    return output;
  }
  double Filter(const double x_insert, const double delta) {
    double output = 0.0;
    if (is_first_step_) {
      output         = x_insert;
      is_first_step_ = false;
    } else {
      output = filter_coefficient_ * x_insert + (1 - filter_coefficient_) * (last_output_ - delta);
    }
    last_output_ = output;
    return output;
  }
  void   SetCoefficient(double filter_coefficient) { filter_coefficient_ = filter_coefficient; }
  void   ReSet() { is_first_step_ = true; }
  double Last() { return last_output_; }

 private:
  double last_output_{0.0};
  double filter_coefficient_{0.0};
  bool   is_first_step_{true};
};

// 计算两条直线（线段延长线）的交点
std::pair<bool, Vec2d> LineIntersection(Vec2d A, Vec2d B, Vec2d C, Vec2d D) {
  // 计算分母（叉积）
  double denom = (B.x() - A.x()) * (D.y() - C.y()) - (B.y() - A.y()) * (D.x() - C.x());

  // 处理平行或重合的情况
  if (std::fabs(denom) < cross_epslion) {
    // 检查是否重合（点C是否在AB直线上）
    double cross = (C.x() - A.x()) * (B.y() - A.y()) - (C.y() - A.y()) * (B.x() - A.x());
    if (std::fabs(cross) < cross_epslion) {
      // 重合时返回第一个点（可自定义）
      return {false, Vec2d()};
    }
    return {false, Vec2d()};  // 平行不相交
  }

  // 计算参数 t（AB上的位置）
  double t = ((C.x() - A.x()) * (D.y() - C.y()) - (C.y() - A.y()) * (D.x() - C.x())) / denom;

  // 计算交点坐标
  double x = A.x() + t * (B.x() - A.x());
  double y = A.y() + t * (B.y() - A.y());

  return {true, Vec2d(x, y)};
}



float GetDistPointLane(const Eigen::Vector2f &point_a, const Eigen::Vector2f &point_b, const Eigen::Vector2f &point_c) {
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = point_c - point_b;
  Eigen::Vector2f BA = point_a - point_b;

  if (abs(BC.norm()) < 0.0001) {
    return abs(BA.y());
  }

  float dist_proj = BA.dot(BC) / BC.norm();
  // A到BC的垂心为P
  Eigen::Vector2f BP = dist_proj * BC.normalized();
  Eigen::Vector2f AP = (point_b - point_a) + BP;
  return AP.norm();
}

float GetDistPointLane(const Vec2d &a, const Vec2d &b, const Vec2d &c) {
  Eigen::Vector2f point_a(a.x(), a.y()), point_b(b.x(), b.y()), point_c(c.x(), c.y());
  return GetDistPointLane(point_a, point_b, point_c);
}

float GetDistPointLane(const Vec2d &a, const Point2DF &b, const Point2DF &c) {
  Eigen::Vector2f point_a(a.x(), a.y()), point_b(b.x, b.y), point_c(c.x, c.y);
  return GetDistPointLane(point_a, point_b, point_c);
}

float GetDistPointLane(const Point &a, const Point &b, const Point &c) {
  Eigen::Vector2f point_a(a.x, a.y), point_b(b.x, b.y), point_c(c.x, c.y);
  return GetDistPointLane(point_a, point_b, point_c);
}

// 点A到BC的距离，且投影在线段BC上
bool GetDistPointLane(const Eigen::Vector2f &point_a, const Eigen::Vector2f &point_b, const Eigen::Vector2f &point_c, float &dis) {
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = point_c - point_b;
  Eigen::Vector2f BA = point_a - point_b;

  float t = BA.dot(BC) / BC.squaredNorm();
  if (abs(BC.norm()) < 0.0001 || t < -cross_epslion || t > 1.0f + cross_epslion) {
    return false;
  }

  float dist_proj = BA.dot(BC) / BC.norm();
  // A到BC的垂心为P
  Eigen::Vector2f BP = dist_proj * BC.normalized();
  Eigen::Vector2f AP = (point_b - point_a) + BP;
  dis                = AP.norm();
  return true;
}

bool GetDistPointLane(const Vec2d &a, const Vec2d &b, const Vec2d &c, float &dis) {
  Eigen::Vector2f point_a(a.x(), a.y()), point_b(b.x(), b.y()), point_c(c.x(), c.y());
  return GetDistPointLane(point_a, point_b, point_c, dis);
}

// 点A到BC的投影点，且投影在线段BC上
bool GetFootPoint(const Eigen::Vector2f &point_a, const Eigen::Vector2f &point_b, const Eigen::Vector2f &point_c,
                  Eigen::Vector2f &point_p) {
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = point_c - point_b;
  Eigen::Vector2f BA = point_a - point_b;

  float t = BA.dot(BC) / BC.squaredNorm();
  if (abs(BC.norm()) < 0.0001 || t < -cross_epslion || t > 1.0f + cross_epslion) {
    return false;
  }
  // A到BC的垂心为P
  point_p = point_b + t * BC;
  return true;
}

bool GetFootPoint(const Vec2d &a, const Vec2d &b, const Vec2d &c, Vec2d &p) {
  Eigen::Vector2f point_a(a.x(), a.y()), point_b(b.x(), b.y()), point_c(c.x(), c.y());
  Eigen::Vector2f p_t;
  bool            val = GetFootPoint(point_a, point_b, point_c, p_t);
  if (val) {
    p.set_x(p_t.x());
    p.set_y(p_t.y());
  }
  // FLOG_CROSS << " GetFootPoint val: " << val;
  return val;
}
// 点A到BC的投影点，判断投影点是否在线段上
template <typename T, typename M>
bool IsFootPoint(const T &point_a, const M &point_b, const M &point_c) {
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC(point_c.x - point_b.x, point_c.y - point_b.y);
  Eigen::Vector2f BA(point_a.x - point_b.x, point_a.y - point_b.y);

  float t = BA.dot(BC) / BC.squaredNorm();
  if (abs(BC.norm()) < 0.0001 || t < -cross_epslion || t > 1.0f + cross_epslion) {
    return false;
  }
  return true;
}

float GetDistBetweenTwoLaneOverlap(const std::vector<Eigen::Vector2f> &main, const std::vector<Eigen::Vector2f> &other) {
  std::vector<float> dis_vec;
  if(main.size() < 2 || other.size() <2  ){
    return 100.0;
  }
  for (int i = 0; i < static_cast<int> (other.size()); i++) {
    for (int j = 0; j < static_cast<int>(main.size()) - 1;) {
      float dis{0.0};
      if((j+1) >= main.size()){
        XLOG << " main.size() BREAK"  <<  main.size();
        break;
      }
      if (GetDistPointLane(other[i], main[j], main[j + 1], dis)) {
        dis_vec.emplace_back(dis);
        break;
      } else {
        j++;
      }
    }
  }
  if (dis_vec.empty()) {
    return 100.0;
  }
  float sum_dis{0.0f};
  for (auto dis : dis_vec) {
    // AINFO << " fml " << " GetDistBetweenTwoLaneOverlap_dis: " << dis;
    sum_dis += dis;
  }
  return sum_dis / dis_vec.size();
}

#define ENUM_CLASS_STRING(Enum, Value) case Enum::Value: return #Value
// 一个通用的枚举转字符串模板函数
template <typename T>
constexpr auto EnumToString(T value) -> typename std::enable_if<std::is_enum<T>::value, std::string>::type {
  // 此处需要为每个特定的枚举类型进行特化
  // 编译器会提示你如果没有为使用的枚举类型提供特化版本
  static_assert(std::is_enum<T>::value, "T must be an enum type");
  return "Unknown Enum Value";
}
// 2. 特化 cem::message::env_model::TurnType 枚举的 FromString 函数
template <>
std::string EnumToString<cem::message::env_model::TurnType>(cem::message::env_model::TurnType value) {
  switch (value) {
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, NO_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, LEFT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_LEFT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, LEFT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, LEFT_TURN_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_LEFT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::env_model::TurnType, STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN);
    default:
      return "OTHER_UNKNOWN";
  }
}

// 3. 特化BevTurnType
template <>
std::string EnumToString<cem::message::sensor::BevTurnType>(cem::message::sensor::BevTurnType value) {
  switch (value) {
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, NO_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, LEFT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_LEFT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, LEFT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, LEFT_TURN_AND_RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_LEFT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevTurnType, STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN);
    default:
      return "OTHER_UNKNOWN";
  }
}

// 4.特化BevAction
template <>
std::string EnumToString<cem::message::sensor::BevAction>(cem::message::sensor::BevAction value) {
  switch (value) {
    ENUM_CLASS_STRING(cem::message::sensor::BevAction, STRAIGHT);
    ENUM_CLASS_STRING(cem::message::sensor::BevAction, LEFT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevAction, RIGHT_TURN);
    ENUM_CLASS_STRING(cem::message::sensor::BevAction, U_TURN);
    default:
      return "UNKNOWN";
  }
}
// 5.特化CrossState
template <>
std::string EnumToString<CrossState>(CrossState value) {
  switch (value) {
    ENUM_CLASS_STRING(CrossState, INIT);
    ENUM_CLASS_STRING(CrossState, PREDICTION);
    ENUM_CLASS_STRING(CrossState, CONNECTION);
    ENUM_CLASS_STRING(CrossState, END);
    default:
      return "UNKNOWN";
  }
}
// 6.特化VirtualLineSource
template <>
std::string EnumToString<VirtualLineSource>(VirtualLineSource value) {
  switch (value) {
    ENUM_CLASS_STRING(VirtualLineSource, PRE);
    ENUM_CLASS_STRING(VirtualLineSource, LD);
    ENUM_CLASS_STRING(VirtualLineSource, OBS);
    ENUM_CLASS_STRING(VirtualLineSource, SD);
    ENUM_CLASS_STRING(VirtualLineSource, EXP);
    default:
      return "UNKNOWN";
  }
}
// 7.特化JunctionAction
template <>
std::string EnumToString<cem::fusion::navigation::JunctionAction>(cem::fusion::navigation::JunctionAction value) {
  switch (value) {
    ENUM_CLASS_STRING(cem::fusion::navigation::JunctionAction, GoStraight);
    ENUM_CLASS_STRING(cem::fusion::navigation::JunctionAction, TurnLeft);
    ENUM_CLASS_STRING(cem::fusion::navigation::JunctionAction, TurnRight);
    ENUM_CLASS_STRING(cem::fusion::navigation::JunctionAction, UTurn);
    default:
      return "UNKNOWN";
  }
}


}  // namespace
}  // namespace fusion
}  // namespace cem
#endif  // CROSSCOMMON_H