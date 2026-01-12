#include "utility.h"

namespace cem {
namespace fusion {

Eigen::Isometry3d CalcRotateTranslateMatrix(LocalizationConstPtr last_loc,
                                            LocalizationConstPtr cur_loc)
{
    Eigen::Isometry3d rotate_translate_matrix = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd last_global_V(last_loc->attitude_dr * M_PI / 180,
                                    Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d last_global_T(last_loc->posne_dr.at(0),
                                  last_loc->posne_dr.at(1), 0);

    Eigen::AngleAxisd cur_global_V(cur_loc->attitude_dr * M_PI / 180,
                                   Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d cur_global_T(cur_loc->posne_dr.at(0),
                                 cur_loc->posne_dr.at(1), 0);

    Eigen::Matrix3d rotate = Eigen::Matrix3d::Identity();
    rotate = cur_global_V.inverse() * last_global_V;

    Eigen::Vector3d translate =
        cur_global_V.inverse() * (last_global_T - cur_global_T);

    rotate_translate_matrix.rotate(rotate);
    rotate_translate_matrix.pretranslate(translate);

    return rotate_translate_matrix;
}

Eigen::Isometry3d CalcRotateTranslateMatrix(double hist_heading, double hist_x,
                                            double hist_y, double cur_heading,
                                            double cur_x, double cur_y) {
  Eigen::Isometry3d rotate_translate_matrix = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd last_global_V(hist_heading, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d last_global_T(hist_x, hist_y, 0);

  Eigen::AngleAxisd cur_global_V(cur_heading, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d cur_global_T(cur_x, cur_y, 0);

  Eigen::Matrix3d rotate = Eigen::Matrix3d::Identity();
  rotate = cur_global_V.inverse() * last_global_V;

  Eigen::Vector3d translate =
      cur_global_V.inverse() * (last_global_T - cur_global_T);

  rotate_translate_matrix.rotate(rotate);
  rotate_translate_matrix.pretranslate(translate);

  return rotate_translate_matrix;
}

void Sampling(GeometryLine *geometry, int16_t idx_start, int16_t idx_end)
{
    for (int16_t i = idx_start; i < idx_end; ++i)
    {
        geometry->sampling_points.emplace_back(std::move(SamplingPoint()));
        auto &trafficroad_point = geometry->sampling_points.back();
        trafficroad_point.x = static_cast<float>(i);
        trafficroad_point.y = CalculateY(geometry->curve.c0, geometry->curve.c1,
                                         geometry->curve.c2, geometry->curve.c3,
                                         trafficroad_point.x);
    }
}

void Sampling(GeometryLine *geometry, int16_t idx_start, int16_t idx_end,
              float start_offset)
{
    if (idx_start <= idx_end)
    {
        for (int16_t i = idx_start; i <= idx_end; ++i)
        {
            geometry->sampling_points.emplace_back(std::move(SamplingPoint()));
            auto &trafficroad_point = geometry->sampling_points.back();
            trafficroad_point.x = static_cast<float>(i);
            trafficroad_point.y = CalculateY(
                geometry->curve.c0, geometry->curve.c1, geometry->curve.c2,
                geometry->curve.c3, trafficroad_point.x);
            if (i == idx_start)
            {
                trafficroad_point.offset = start_offset;
            }
            else
            {
                const auto &last_point =
                    geometry->sampling_points.at(i - idx_start - 1);
                trafficroad_point.offset =
                    last_point.offset +
                    CalculatePoint2PointDist(trafficroad_point, last_point);
            }
        }
    }
    else
    {
        for (int16_t i = idx_start; i >= idx_end; --i)
        {
            geometry->sampling_points.emplace_back(std::move(SamplingPoint()));
            auto &trafficroad_point = geometry->sampling_points.back();
            trafficroad_point.x = static_cast<float>(i);
            trafficroad_point.y = CalculateY(
                geometry->curve.c0, geometry->curve.c1, geometry->curve.c2,
                geometry->curve.c3, trafficroad_point.x);
            if (i == idx_start)
            {
                trafficroad_point.offset = start_offset;
            }
            else
            {
                const auto &last_point =
                    geometry->sampling_points.at(idx_start - i - 1);
                trafficroad_point.offset =
                    last_point.offset -
                    CalculatePoint2PointDist(trafficroad_point, last_point);
            }
        }
        std::reverse(geometry->sampling_points.begin(),
                     geometry->sampling_points.end());
    }
}

/**
 * @brief To ensure C0 and C1 of connected segments are consistent after
 *        kalman prediction and update procedure. Refer to docs/scheme/version.
 *
 * @param states_ref Kalman states (coefficients here) of reference segment.
 * @param states_to_be_adjusted Kalman states (coefficients here) of
 *        segment to be adjusted.
 * @param connected_point_x Longitudinal coordinate of the connection point.
 * @return Eigen::Vector4f Adjusted states.
 */
Eigen::VectorXd AdjustConnectedSegmentsStates(
    const Eigen::VectorXd &states_ref,
    const Eigen::VectorXd &states_to_be_adjusted, float connected_point_x)
{
    Eigen::VectorXd adjusted_states = states_to_be_adjusted;

    adjusted_states(0) =
        states_ref(0) - states_ref(2) * std::pow(connected_point_x, 2) -
        2 * states_ref(3) * std::pow(connected_point_x, 3) +
        states_to_be_adjusted(2) * std::pow(connected_point_x, 2) +
        2 * states_to_be_adjusted(3) * std::pow(connected_point_x, 3);

    adjusted_states(1) =
        states_ref(1) + 2 * states_ref(2) * connected_point_x +
        3 * states_ref(3) * std::pow(connected_point_x, 2) -
        2 * states_to_be_adjusted(2) * connected_point_x -
        3 * states_to_be_adjusted(3) * std::pow(connected_point_x, 2);

    return adjusted_states;
}

void AdjustConnectedSegmentsStates(const Curve &curve_ref,
                                   Curve *curve_to_be_adjusted,
                                   float connected_point_x)
{
    curve_to_be_adjusted->c0 =
        curve_ref.c0 - curve_ref.c2 * std::pow(connected_point_x, 2) -
        2 * curve_ref.c3 * std::pow(connected_point_x, 3) +
        curve_to_be_adjusted->c2 * std::pow(connected_point_x, 2) +
        2 * curve_to_be_adjusted->c3 * std::pow(connected_point_x, 3);

    curve_to_be_adjusted->c1 =
        curve_ref.c1 + 2 * curve_ref.c2 * connected_point_x +
        3 * curve_ref.c3 * std::pow(connected_point_x, 2) -
        2 * curve_to_be_adjusted->c2 * connected_point_x -
        3 * curve_to_be_adjusted->c3 * std::pow(connected_point_x, 2);
}

std::tuple<bool, float, float> AreCurvesTooClose(const Curve &first_curve,
                                                 const Curve &second_curve,
                                                 float thresh)
{
    float farther_start =
        std::max(first_curve.lon_dist_start, second_curve.lon_dist_start);
    float closer_end =
        std::min(first_curve.lon_dist_end, second_curve.lon_dist_end);
    float anchor_point_x = 0.5f * (farther_start + closer_end);
    float anchor_point_first_y =
        CalculateY(first_curve.c0, first_curve.c1, first_curve.c2,
                   first_curve.c3, anchor_point_x);
    float anchor_point_second_y =
        CalculateY(second_curve.c0, second_curve.c1, second_curve.c2,
                   second_curve.c3, anchor_point_x);
    bool curves_are_too_close =
        (std::fabs(anchor_point_first_y - anchor_point_second_y) < thresh);
    return {curves_are_too_close, anchor_point_first_y, anchor_point_second_y};
}

float CalcCurvesGap(const Curve &first_curve, const Curve &second_curve)
{
    float farther_start =
        std::max(first_curve.lon_dist_start, second_curve.lon_dist_start);
    float left_heading_cos =
        std::cos(std::atan(first_curve.c1 + 2 * first_curve.c2 * farther_start +
                           3 * first_curve.c3 * farther_start * farther_start));
    float right_heading_cos = std::cos(
        std::atan(second_curve.c1 + 2 * second_curve.c2 * farther_start +
                  3 * second_curve.c3 * farther_start * farther_start));

    float first_curve_anchor_point_y =
        CalculateY(first_curve.c0, first_curve.c1, first_curve.c2,
                   first_curve.c3, farther_start);
    float second_curve_anchor_point_y =
        CalculateY(second_curve.c0, second_curve.c1, second_curve.c2,
                   second_curve.c3, farther_start);

    return std::fabs(
        0.5f * (left_heading_cos + right_heading_cos) *
        (first_curve_anchor_point_y - second_curve_anchor_point_y));
}

bool CompareCurvesFromLeft(const Curve &lhs_curve, const Curve &rhs_curve)
{
    float anchor_x =
        std::max(lhs_curve.lon_dist_start, rhs_curve.lon_dist_start);
    float lhs_curve_y = CalculateY(lhs_curve.c0, lhs_curve.c1, lhs_curve.c2,
                                   lhs_curve.c3, anchor_x);
    float rhs_curve_y = CalculateY(rhs_curve.c0, rhs_curve.c1, rhs_curve.c2,
                                   rhs_curve.c3, anchor_x);
    return lhs_curve_y > rhs_curve_y;
}

bool RelativePoseBasedOnEgoMotion(LocalizationConstPtr last_loc,
                                  LocalizationConstPtr cur_loc, float &delta_x,
                                  float &delta_y, float &delta_heading)
{
    if (last_loc == nullptr || cur_loc == nullptr) return false;

    float delta_x_global = cur_loc->posne_dr.at(0) - last_loc->posne_dr.at(0);
    float delta_y_global = cur_loc->posne_dr.at(1) - last_loc->posne_dr.at(1);
    float cos = std::cos(last_loc->attitude_dr * M_PI / 180);//// 上一次定位的航向角余弦值
    float sin = std::sin(last_loc->attitude_dr * M_PI / 180);

    delta_x = cos * delta_x_global + sin * delta_y_global;
    delta_y = -sin * delta_x_global + cos * delta_y_global;
    delta_heading = std::atan(std::tan(cur_loc->attitude_dr * M_PI / 180 -
                                       last_loc->attitude_dr * M_PI / 180));

    return true;
}

void PredictCurveBasedOnEgoMotion(Curve *curve, LocalizationConstPtr last_loc,
                                  LocalizationConstPtr cur_loc)
{
    if (last_loc == nullptr || cur_loc == nullptr) return;

    float delta_x_local, delta_y_local, delta_heading;

    // float delta_x_global = cur_loc->posne_dr.at(0) -
    // last_loc->posne_dr.at(0); float delta_y_global = cur_loc->posne_dr.at(1)
    // - last_loc->posne_dr.at(1);

    // float cos = std::cos(last_loc->attitude_dr * M_PI / 180);
    // float sin = std::sin(last_loc->attitude_dr * M_PI / 180);

    // delta_x_local = cos * delta_x_global + sin * delta_y_global;
    // delta_y_local = -sin * delta_x_global + cos * delta_y_global;
    // delta_heading = std::atan(std::tan(cur_loc->attitude_dr * M_PI / 180 -
    //                                    last_loc->attitude_dr * M_PI / 180));
    if (!RelativePoseBasedOnEgoMotion(last_loc, cur_loc, delta_x_local,
                                      delta_y_local, delta_heading))
    {
        return;
    }
    if (std::fabs(delta_y_local) > 1.0f)
    {
        AERROR << "loc jump";
        return;
    }

    float new_c0 = curve->c0 + curve->c1 * delta_x_local +
                   curve->c2 * std::pow(delta_x_local, 2) +
                   curve->c3 * std::pow(delta_x_local, 3) - delta_y_local;
    float new_c1 =
        std::tan(std::atan(curve->c1 + 2 * curve->c2 * delta_x_local +
                           3 * curve->c3 * std::pow(delta_x_local, 2)) -
                 delta_heading);
    float new_c2 = curve->c2 + 3 * curve->c3 * delta_x_local;
    float new_c3 = curve->c3;

    curve->c0 = new_c0;
    curve->c1 = new_c1;
    curve->c2 = new_c2;
    curve->c3 = new_c3;
    curve->lon_dist_start = curve->lon_dist_start - delta_x_local;
    curve->lon_dist_end = curve->lon_dist_end - delta_x_local;
}

bool ThirdOrderBezierFit(std::vector<SamplingPoint> &points)
{
    if (points.size() != 4)
    {
        AERROR << "Points size is error to fit Bezier";
        return false;
    }

    auto p0 = points[0];
    auto p1 = points[1];
    auto p2 = points[2];
    auto p3 = points[3];
    float step = 0.05;
    size_t bezier_points_num = static_cast<size_t>(1.0 / step) + 1;
    points.clear();
    points.reserve(bezier_points_num);
    SamplingPoint bezier_point;
    for (size_t i = 0; i < bezier_points_num; ++i)
    {
        float t = i * step;
        bezier_point.x = p0.x * pow(1 - t, 3) + 3 * p1.x * t * pow(1 - t, 2) +
                         3 * p2.x * t * t * (1 - t) + p3.x * t * t * t;
        bezier_point.y = p0.y * pow(1 - t, 3) + 3 * p1.y * t * pow(1 - t, 2) +
                         3 * p2.y * t * t * (1 - t) + p3.y * t * t * t;
        points.emplace_back(bezier_point);
    }

    return true;
}
bool ThirdOrderBezierFitUniform(
    const float &point_interval,
    std::vector<SamplingPoint> &points)
{
    if (points.size() != 4)
    {
        AERROR << "Points size is error to fit Bezier";
        return false;
    }

    auto p0 = points[0];
    auto p1 = points[1];
    auto p2 = points[2];
    auto p3 = points[3];
    float step = 0.05;
    size_t bezier_points_num = static_cast<size_t>(1.0 / step) + 1;
    points.clear();
    points.reserve(bezier_points_num);
    SamplingPoint bezier_point;
    for (size_t i = 0; i < bezier_points_num; ++i)
    {
        float t = i * step;
        bezier_point.x = p0.x * pow(1 - t, 3) + 3 * p1.x * t * pow(1 - t, 2) +
                         3 * p2.x * t * t * (1 - t) + p3.x * t * t * t;
        bezier_point.y = p0.y * pow(1 - t, 3) + 3 * p1.y * t * pow(1 - t, 2) +
                         3 * p2.y * t * t * (1 - t) + p3.y * t * t * t;
        points.emplace_back(bezier_point);
    }
    std::vector<SamplingPoint> uniform_points;
    if(points.size() > 0)
    {
        int idx = 0;
        float INTERVAL = point_interval;
        SamplingPoint last_point;
        size_t last_idx = (points.size() - 1);
        while(idx < points.size())
        {
            if(idx == 0 || idx == last_idx)
            {//first or last
                uniform_points.emplace_back(points[idx]);
                last_point = points[idx];
                idx++;
                continue;
            }
            Eigen::Vector2f delta;
            delta(0) = points[idx].x - last_point.x;
            delta(1) = points[idx].y - last_point.y;
            if(delta.norm() > INTERVAL)
            {//interploting
                int n = 1;
                float STEP = INTERVAL/2.0;
                Eigen::Vector2f unit_delta_vec;
                unit_delta_vec = delta/delta.norm();
                while(n*STEP < (delta.norm()-INTERVAL/4.0))
                {
                    SamplingPoint new_pts;
                    new_pts.x = last_point.x + unit_delta_vec(0)*n*STEP;
                    new_pts.y = last_point.y + unit_delta_vec(1)*n*STEP;
                    uniform_points.emplace_back(new_pts);
                    n++;
                }
                uniform_points.emplace_back(points[idx]);
                last_point = points[idx];
            }
            else if(delta.norm() >= INTERVAL/2.0 && delta.norm() <= INTERVAL)
            {
                if(idx == (last_idx - 2))
                {
                    Eigen::Vector2f last_delta;
                    last_delta(0) = points[last_idx].x - points[idx].x;
                    last_delta(1) = points[last_idx].y - points[idx].y;
                    if(last_delta.norm() <= INTERVAL)
                    {//skip the last_idx - 2
                        idx = idx + 2;
                        continue;
                    }
                }
                uniform_points.emplace_back(points[idx]);
                last_point = points[idx];
            }
            idx++;
        }
    }
    std::vector<SamplingPoint>().swap(points);
    points.swap(uniform_points);

    return true;
}

// 计算点到线段的最短距离
double DistancePointToSegment(const cem::fusion::Point2DF &point,
                              const cem::fusion::Point2DF &seg_start,  // a
                              const cem::fusion::Point2DF &seg_end) {  // b
  float ABx = seg_end.x - seg_start.x;
  float ABy = seg_end.y - seg_start.y;
  float APx = point.x - seg_start.x;
  float APy = point.y - seg_start.y;
  float BPx = point.x - seg_end.x;
  float BPy = point.y - seg_end.y;

  // 计算投影系数t
  float dot_product = (APx * ABx + APy * ABy) / (ABx * ABx + ABy * ABy);

  // 计算最短距离
  if (dot_product < 0.0F) {
    // 投影点在A点之前
    return APx * APx + APy * APy;
  } else if (dot_product > 1.0F) {
    // 投影点在B点之后
    return BPx * BPx + BPy * BPy;
  } else {
    // 投影点在AB线段之间
    float projX = seg_start.x + dot_product * ABx;
    float projY = seg_start.y + dot_product * ABy;
    float dx    = projX - point.x;
    float dy    = projY - point.y;
    return dx * dx + dy * dy;
  }
}

// 计算两条线段之间的最小距离
float LineSegmentsDistance(const cem::fusion::Point2DF &p1, const cem::fusion::Point2DF &p2, const cem::fusion::Point2DF &q1,
                                  const cem::fusion::Point2DF &q2) {
  // 判断两条线段是否相交
  float cross1 = CrossProduct(p1, p2, q1);
  float cross2 = CrossProduct(p1, p2, q2);
  float cross3 = CrossProduct(q1, q2, p1);
  float cross4 = CrossProduct(q1, q2, p2);

  // 判断是否相交
  if (cross1 * cross2 < 0.0F && cross3 * cross4 < 0.0F) {
    return 0.0F;  // 如果相交，返回0
  }

  // 如果不相交，计算四个端点到另一条线段的距离
  float dist1 = DistancePointToSegment(p1, q1, q2);
  float dist2 = DistancePointToSegment(p2, q1, q2);
  float dist3 = DistancePointToSegment(q1, p1, p2);
  float dist4 = DistancePointToSegment(q2, p1, p2);

  // 返回最小距离
  return std::min({dist1, dist2, dist3, dist4});
}
// 计算两条线段之间的最小距离
bool GetLineIntersection(const cem::fusion::Point2DF &p1, const cem::fusion::Point2DF &p2, const cem::fusion::Point2DF &q1,
                                const cem::fusion::Point2DF &q2, cem::fusion::Point2DF &intersection) {
  // 判断两条线段是否相交
  float cross1 = CrossProduct(p1, p2, q1);
  float cross2 = CrossProduct(p1, p2, q2);
  float cross3 = CrossProduct(q1, q2, p1);
  float cross4 = CrossProduct(q1, q2, p2);

  // 判断是否相交
  if (cross1 * cross2 < 0.0 && cross3 * cross4 < 0.0) {
    // 相交：计算交点
    Eigen::Vector2f r(p2.x - p1.x, p2.y - p1.y);
    Eigen::Vector2f s(q2.x - q1.x, q2.y - q1.y);
    Eigen::Vector2f qp(q1.x - p1.x, q1.y - p1.y);

    float rxs = r.x() * s.y() - r.y() * s.x();
    if (std::abs(rxs) < 1e-8)
      return false;  // 平行，不应发生

    float t = (qp.x() * s.y() - qp.y() * s.x()) / rxs;

    intersection.x = p1.x + t * r.x();
    intersection.y = p1.y + t * r.y();

    return true;
  }
  return false;
}

} // namespace fusion
} // namespace cem
