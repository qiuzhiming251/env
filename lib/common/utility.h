#ifndef UTILITY_H_
#define UTILITY_H_

// c++
#include <unordered_map>
#include <queue>
#include <deque>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include <algorithm>
#include <list>
#include <fstream>

// middleware
#include "middleware/middleware_util.h"

// third party
#include <Eigen/Dense>
#include "nlohmann/json.hpp"

// internal: message
#include "message/internal_message.h"

// internal: common
#include "optimizer/cubic_polynomial_optimizer/cubic_polynomial_optimizer.h"
#include "cpp/cpp_headers.h"
#include "modules/common/math/vec2d.h"
#include "cyber/common/macros.h"
#include "design_pattern/factory_method/factory_method.h"
#include "dst/dst_evidence.h"
#include "filter/base_filter.h"
#include "filter/slide_average_filter/slide_average_filter.h"
#include "fitter/least_square_fitter.h"
#include "fitter/bspline_segment.h"
#include "fitter/least_squares_solver.h"
#include "fitter/least_squares_solver_plus.h"
#include "graph/hungarian_optimizer.h"
#include "cyber/common/log.h"
#include "math/math.h"
#include "time/time.h"
#include "enum.h"

namespace cem {
namespace fusion {

// frequently-used namespaces
using json = nlohmann::json;

template <typename T>
inline T ApproximateDistFromPoint2Curve(T point_x, T point_y,
                                        const Curve &curve)
{
    double k =
        curve.c1 + 2 * curve.c2 * point_x + 3 * curve.c3 * point_x * point_x;

    T y0 = CalculateY(curve.c0, curve.c1, curve.c2, curve.c3, point_x);
    return static_cast<T>((point_y - y0) / sqrt(1 + k * k));
}

template <typename T>
float CalculatePoint2PointDist(const T &point1, const T &point2)
{
    return std::sqrt(std::pow(point1.x - point2.x, 2) +
                     std::pow(point1.y - point2.y, 2));
}

template <typename T>
float CalculatePoint2SegmentDist(const T &point1, const T &line_p1, const T &line_p2)
{
    float len_seg = CalculatePoint2PointDist(line_p1, line_p2);
    if (len_seg == 0) { // 线段长度为0，即线段为点
        return CalculatePoint2PointDist(point1, line_p1);
    }
    float r = ((point1.x - line_p1.x) * (line_p2.x - line_p1.x) + (point1.y - line_p1.y) * (line_p2.y - line_p1.y)) / pow(len_seg, 2);
    if (r <= 0) { // 垂足在line p1处
        return CalculatePoint2PointDist(point1, line_p1);
    } else if (r >= 1) { // 垂足在line p2处
        return CalculatePoint2PointDist(point1, line_p2);
    } else { // 垂足在线段上
        T foot;
        foot.x = line_p1.x + r * (line_p2.x - line_p1.x); 
        foot.y = line_p1.y + r * (line_p2.y - line_p1.y);
        return CalculatePoint2PointDist(point1, foot);
    }
}

Eigen::Isometry3d CalcRotateTranslateMatrix(double hist_heading, double hist_x,
                                            double hist_y, double cur_heading,
                                            double cur_x, double cur_y);

Eigen::Isometry3d CalcRotateTranslateMatrix(LocalizationConstPtr last_loc,
                                            LocalizationConstPtr cur_loc);

template <typename T>
void TransformPoint(T *point, const Eigen::Isometry3d &rotate_translate_matrix)
{
    Eigen::Vector3d point_before_dr(point->x, point->y, 0);
    Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
    point->x = point_after_dr[0];
    point->y = point_after_dr[1];
}

template <typename T>
T TransformPoint(const T &point, const Eigen::Isometry3d &rotate_translate_matrix) {
  T point_out = point;
  Eigen::Vector3d point_before_dr(point.x, point.y, 0);
  Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
  point_out.x = point_after_dr[0];
  point_out.y = point_after_dr[1];
  return point_out;
}

void Sampling(GeometryLine *geometry, int16_t idx_start, int16_t idx_end);

void Sampling(GeometryLine *geometry, int16_t idx_start, int16_t idx_end,
              float start_offset);

bool CompareCurvesFromLeft(const Curve &lhs_curve, const Curve &rhs_curve);

Eigen::VectorXd AdjustConnectedSegmentsStates(
    const Eigen::VectorXd &states_ref,
    const Eigen::VectorXd &states_to_be_adjusted, float connected_point_x);

void AdjustConnectedSegmentsStates(const Curve &ref_curve,
                                   Curve *curve_to_be_adjusted,
                                   float connected_point_x);

double DistancePointToSegment(const cem::fusion::Point2DF &point,
                              const cem::fusion::Point2DF &seg_start,
                              const cem::fusion::Point2DF &seg_end);

// 计算两条线段之间的最小距离
float LineSegmentsDistance(const cem::fusion::Point2DF &p1, const cem::fusion::Point2DF &p2, const cem::fusion::Point2DF &q1,
                           const cem::fusion::Point2DF &q2);

// 计算两条线段之间的最小距离
bool GetLineIntersection(const cem::fusion::Point2DF &p1, const cem::fusion::Point2DF &p2, const cem::fusion::Point2DF &q1,
                         const cem::fusion::Point2DF &q2, cem::fusion::Point2DF &intersection);


inline double vector2dCrossProduct(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
    return vec1.x() * vec2.y() - vec1.y() * vec2.x();
}

std::tuple<bool, float, float> AreCurvesTooClose(const Curve &first_curve,
                                                 const Curve &second_curve,
                                                 float thresh = 1.0f);

float CalcCurvesGap(const Curve &first_curve, const Curve &second_curve);

template <typename T>
void AdjustConnectedSegmentsOffset(T *segs, const std::string &keyword)
{
    bool last_seg_is_valid = false;
    for (auto iter = segs->begin(); iter != segs->end(); ++iter)
    {
        if (iter->start_offset <= iter->end_offset)
        {
            last_seg_is_valid = false;
            continue;
        }
        else
        {
            if (iter != segs->begin() && last_seg_is_valid)
            {
                auto prev_seg = std::prev(iter, 1);
                if (keyword == "rear")
                    iter->end_offset = prev_seg->start_offset;
                else
                    iter->start_offset = prev_seg->end_offset;
            }
            last_seg_is_valid = true;
        }
    }
}

bool RelativePoseBasedOnEgoMotion(LocalizationConstPtr last_loc,
                                  LocalizationConstPtr cur_loc, float &delta_x,
                                  float &delta_y, float &delta_heading);

void PredictCurveBasedOnEgoMotion(Curve *curve, LocalizationConstPtr last_loc,
                                  LocalizationConstPtr cur_loc);

bool ThirdOrderBezierFit(std::vector<SamplingPoint> &points);

bool ThirdOrderBezierFitUniform(const float &point_interval, std::vector<SamplingPoint> &points);

inline bool CheckSegLen(const Curve &curve, float thresh)
{
    return (curve.lon_dist_end > curve.lon_dist_start + thresh);
}

inline double RadiusOfCurvature(double point_x, const Curve &curve)
{
    double first_derivative =
        curve.c1 + 2 * curve.c2 * point_x + 3 * curve.c3 * point_x * point_x;
    double second_derivative = 2 * curve.c2 + 6 * curve.c3 * point_x;
    double radius = std::pow((1 + first_derivative * first_derivative), 1.5) /
                    second_derivative;
    return radius;
}

template <typename T>
double CrossProduct(const T& p1, const T& p2, const T& p3)
{
    // 平面两向量p（p1指向p2）和q（p1指向p3）的叉乘是法向量n
    // n = p x q = (0, 0, px*qy-py*qx)
    // 返回n的z分量：若nz=0则三点共线，若nz>0则三点逆时针排列，若nz<0则三点顺时针排列
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

template <typename T>
bool IsOnSegment(const T& p1, const T& p2, const T& p)
{
    return (p.x <= std::max(p1.x, p2.x) && p.x >= std::min(p1.x, p2.x) &&
            p.y <= std::max(p1.y, p2.y) && p.y >= std::min(p1.y, p2.y));
}

template <typename T>
bool IsSegmentsIntersectVectorMethod(const T& p1, const T& p2, const T& q1, const T& q2, T& intersection)
{
    Eigen::Vector2d vec_seg1(p2.x - p1.x, p2.y - p1.y);
    Eigen::Vector2d vec_seg2(q2.x - q1.x, q2.y - q1.y);
    Eigen::Vector2d st_seg1(p1.x, p1.y);
    Eigen::Vector2d st_seg2(q1.x, q1.y);
    Eigen::Vector2d delta_vec1, delta_vec2; 
    double t, u, cross_prd1, cross_prd2;

    cross_prd1 = vector2dCrossProduct(vec_seg1, vec_seg2);
    cross_prd2 = vector2dCrossProduct(vec_seg2, vec_seg1);
    delta_vec1 = st_seg2 - st_seg1;
    delta_vec2 = st_seg1 - st_seg2;

    if (std::abs(cross_prd1) > 1e-9)
    {
        t = vector2dCrossProduct(delta_vec1, vec_seg2) / cross_prd1;
        u = vector2dCrossProduct(delta_vec2, vec_seg1) / cross_prd2;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1){
            intersection.x = st_seg1.x() + t * vec_seg1.x();
            intersection.y = st_seg1.y() + t * vec_seg1.y();
            return true;
        } else {
            return false;
        }
        
    } else {
        return false; 
    }

}

template <typename T>
bool IsSegmentsIntersect(const T& p1, const T& p2, const T& q1, const T& q2, T& intersection)
{
    double d1 = CrossProduct(q1, q2, p1);
    double d2 = CrossProduct(q1, q2, p2);
    double d3 = CrossProduct(p1, p2, q1);
    double d4 = CrossProduct(p1, p2, q2);

    if (std::abs(d1) < 1e-9 && std::abs(d2) < 1e-9 && std::abs(d3) < 1e-9 && std::abs(d4) < 1e-9)
    {
        if (IsOnSegment(p1, p2, q1) || IsOnSegment(p1, p2, q2) || IsOnSegment(q1, q2, p1) || IsOnSegment(q1, q2, p2))
        {
            intersection = p1;
            return true;
        }
        return false;
    }

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
    {
        intersection.x = (p1.x * d2 - p2.x * d1) / (d2 - d1);
        intersection.y = (p1.y * d2 - p2.y * d1) / (d2 - d1);
        return true;
    }

    return false;
}

template <typename T>
bool IsPointInPolygon(const T& p, const std::vector<T>& polygon) {
    if (polygon.size() < 2)
    {
        return false;
    }

    int intersections = 0;
    for (size_t i = 0; i < polygon.size() - 1; i++)
    {
        // overlap with range of y
        if ((polygon[i].y() > p.y()) != (polygon[i + 1].y() > p.y()))
        {
            double x_intersect = (p.y() - polygon[i].y()) * (polygon[i + 1].x() - polygon[i].x()) / (polygon[i + 1].y() - polygon[i].y()) + polygon[i].x();
            if (x_intersect > p.x())
            {
                intersections++;
            }
        }
    }

    // If the number of intersections is odd, then the point is inside the polygon.
    return (intersections % 2 == 1);
}

template <typename T>
bool IsSegmentLineIntersect(const T& p1, const T& p2, const T& l1, const T& l2, T& intersection)
{
    Eigen::Vector2d vec_seg1(p2.x - p1.x, p2.y - p1.y);
    Eigen::Vector2d vec_line2(l2.x - l1.x, l2.y - l1.y);
    Eigen::Vector2d st_seg1(p1.x, p1.y);
    Eigen::Vector2d st_line2(l1.x, l1.y);
    Eigen::Vector2d delta_vec; 
    double u, cross_prd;

    cross_prd = vector2dCrossProduct(vec_line2, vec_seg1);
    delta_vec = st_line2 - st_seg1;

    if (std::abs(cross_prd) > 1e-9)
    {
        u = vector2dCrossProduct(vec_line2, delta_vec) / cross_prd;
        if (u >= 0 && u <= 1){
            intersection.x = st_seg1.x() + u * vec_seg1.x();
            intersection.y = st_seg1.y() + u * vec_seg1.y();
            return true;
        } else {
            return false;
        }
        
    } else {
        return false; 
    }

}

template <typename T>
bool IsLinePolygonIntersect(const T& l1, const T& l2, const std::vector<T>& polygon, std::vector<T>& intersect_seg_st,std::vector<T>& intersect_seg_ed, std::vector<T>& intersections)
{
    bool is_intersect = false;
    if (polygon.empty()){
        return is_intersect; 
    }
    for (size_t i = 0; i < polygon.size() - 1; i++){
        T intersection;
        if (IsSegmentLineIntersect(polygon[i], polygon[i + 1], l1, l2, intersection)){
            intersections.push_back(intersection);
            intersect_seg_st.push_back(polygon[i]);
            intersect_seg_ed.push_back(polygon[i+1]);
            is_intersect = true;
        }
    }
    return is_intersect; 
}

template <typename T>
double MinDistSegPolySeg(const T& seg1, const T& seg2, const std::vector<T>& polycurve_seg, std::vector<int>& min_st_idx, std::vector<int>& min_ed_idx)
{
    int min_st, min_ed;
    double min_dist = std::numeric_limits<double>::max();
    T intersection;
    if (polycurve_seg.size() < 2){
        return min_dist; 
    }
    for (int i = 0; i < polycurve_seg.size() - 1; i++){
        if (IsSegmentsIntersect(polycurve_seg[i], polycurve_seg[i+1], seg1, seg2, intersection)){
            min_st_idx.push_back(i);
            min_ed_idx.push_back(i+1);
            return 0;
        }

        double dist_start1 = CalculatePoint2SegmentDist(seg1, polycurve_seg[i], polycurve_seg[i+1]);
        double dist_end1 = CalculatePoint2SegmentDist(seg2, polycurve_seg[i], polycurve_seg[i+1]);
        double dist_start2 = CalculatePoint2SegmentDist(polycurve_seg[i], seg1, seg2);
        double dist_end2 = CalculatePoint2SegmentDist(polycurve_seg[i+1], seg1, seg2);

        double seg_dist = std::min({dist_start1, dist_end1, dist_start2, dist_end2});
        if (seg_dist < min_dist){
            min_dist = seg_dist;
            min_st = i;
            min_ed = i + 1;
        }
    }
    min_st_idx.push_back(min_st);
    min_ed_idx.push_back(min_ed);
    return min_dist; 
}

template <typename T>
std::optional<typename T::const_iterator> FindTheElement(const T &container, uint64_t id) {
  auto it = std::find_if(container.begin(), container.end(), [&id](const auto &ele) { return ele.id == id; });
  if (it != container.end()) {
    return std::make_optional(it);
  }
  return std::nullopt;
}

} // namespace fusion
} // namespace cem

#endif
