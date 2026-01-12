#ifndef _BEZIER_POINTS_H_
#define _BEZIER_POINTS_H_

#include <iostream>
#include <Eigen/Dense>
#include "modules/common/math/vec2d.h"

namespace cem {
namespace fusion {
namespace {
template <typename T>
bool ThirdOrderBezier(const float &point_interval, std::vector<T> &points)
{
    if (points.size() != 4)
    {
        // AINFO << "Points size is error to fit Bezier";
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
    T bezier_point;
    for (size_t i = 0; i < bezier_points_num; ++i)
    {
        float t = i * step;
        bezier_point.x = p0.x * pow(1 - t, 3) + 3 * p1.x * t * pow(1 - t, 2) +
                         3 * p2.x * t * t * (1 - t) + p3.x * t * t * t;
        bezier_point.y = p0.y * pow(1 - t, 3) + 3 * p1.y * t * pow(1 - t, 2) +
                         3 * p2.y * t * t * (1 - t) + p3.y * t * t * t;
        points.emplace_back(bezier_point);
    }
    std::vector<T> uniform_points;
    if (points.size() > 0)
    {
        int idx = 0;
        float INTERVAL = point_interval;
        T last_point;
        size_t last_idx = (points.size() - 1);
        while (idx < points.size())
        {
            if (idx == 0 || idx == last_idx)
            { // first or last
                uniform_points.emplace_back(points[idx]);
                last_point = points[idx];
                idx++;
                continue;
            }
            Eigen::Vector2f delta;
            delta(0) = points[idx].x - last_point.x;
            delta(1) = points[idx].y - last_point.y;
            if (delta.norm() > INTERVAL)
            { // interploting
                int n = 1;
                float STEP = INTERVAL / 2.0;
                Eigen::Vector2f unit_delta_vec;
                unit_delta_vec = delta / delta.norm();
                while (n * STEP < (delta.norm() - INTERVAL / 4.0))
                {
                    T new_pts;
                    new_pts.x = last_point.x + unit_delta_vec(0) * n * STEP;
                    new_pts.y = last_point.y + unit_delta_vec(1) * n * STEP;
                    uniform_points.emplace_back(new_pts);
                    n++;
                }
                uniform_points.emplace_back(points[idx]);
                last_point = points[idx];
            }
            else if (delta.norm() >= INTERVAL / 2.0 && delta.norm() <= INTERVAL)
            {
                if (idx == (last_idx - 2))
                {
                    Eigen::Vector2f last_delta;
                    last_delta(0) = points[last_idx].x - points[idx].x;
                    last_delta(1) = points[last_idx].y - points[idx].y;
                    if (last_delta.norm() <= INTERVAL)
                    { // skip the last_idx - 2
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
    std::vector<T>().swap(points);
    points.swap(uniform_points);

    return true;
}

template <>
bool ThirdOrderBezier<byd::common::math::Vec2d>(const float &point_interval,
                                                std::vector<byd::common::math::Vec2d> &points) {
  if (points.size() != 4) {
    // AINFO << "Points size is error to fit Bezier";
    return false;
  }

  auto p0 = points[0];
  auto p1 = points[1];
  auto p2 = points[2];
  auto p3 = points[3];

  for (int i=1; i<points.size();i++)
  {
    if (fabs(points[i].x() - points[0].x())>500 || fabs(points[i].y() - points[0].y())>500)
    {
      return false;
    }
  }

  float step = 0.05;
  size_t bezier_points_num = static_cast<size_t>(1.0 / step) + 1;
  points.clear();
  points.reserve(bezier_points_num);
  byd::common::math::Vec2d bezier_point;
  for (size_t i = 0; i < bezier_points_num; ++i) {
    float t = i * step;
    bezier_point.set_x(p0.x() * pow(1 - t, 3) + 3 * p1.x() * t * pow(1 - t, 2) + 3 * p2.x() * t * t * (1 - t) +
                       p3.x() * t * t * t);
    bezier_point.set_y(p0.y() * pow(1 - t, 3) + 3 * p1.y() * t * pow(1 - t, 2) + 3 * p2.y() * t * t * (1 - t) +
                       p3.y() * t * t * t);
    points.emplace_back(bezier_point);
  }
  std::vector<byd::common::math::Vec2d> uniform_points;
  if (points.size() > 0) {
    int idx = 0;
    float INTERVAL = point_interval;
    byd::common::math::Vec2d last_point;
    size_t last_idx = (points.size() - 1);
    while (idx < points.size()) {
      if (idx == 0 || idx == last_idx) {  // first or last
        uniform_points.emplace_back(points[idx]);
        last_point = points[idx];
        idx++;
        continue;
      }
      Eigen::Vector2f delta;
      delta(0) = points[idx].x() - last_point.x();
      delta(1) = points[idx].y() - last_point.y();
      if (delta.norm() > INTERVAL) {  // interploting
        int n = 1;
        float STEP = INTERVAL / 2.0;
        Eigen::Vector2f unit_delta_vec;
        unit_delta_vec = delta / delta.norm();
        while (n * STEP < (delta.norm() - INTERVAL / 4.0)) {
          byd::common::math::Vec2d new_pts;
          new_pts.set_x(last_point.x() + unit_delta_vec(0) * n * STEP);
          new_pts.set_y(last_point.y() + unit_delta_vec(1) * n * STEP);
          uniform_points.emplace_back(new_pts);
          n++;
        }
        uniform_points.emplace_back(points[idx]);
        last_point = points[idx];
      } else if (delta.norm() >= INTERVAL / 2.0 && delta.norm() <= INTERVAL) {
        if (idx == (last_idx - 2)) {
          Eigen::Vector2f last_delta;
          last_delta(0) = points[last_idx].x() - points[idx].x();
          last_delta(1) = points[last_idx].y() - points[idx].y();
          if (last_delta.norm() <= INTERVAL) {  // skip the last_idx - 2
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
  std::vector<byd::common::math::Vec2d>().swap(points);
  points.swap(uniform_points);

  return true;
}

template <typename T>
bool InsertBezierPoints(std::vector<T> &bottom_points, std::vector<T> &top_points, std::vector<T> &control_points, int &bot_index, int &top_index)
{
    if (bottom_points.size() < 2 || top_points.size() < 2)
    {
        return false;
    }

    float control_point = 5.0;

    T p0, p1, p2, p3;
    bot_index = -1;
    top_index = -1;

    p0 = bottom_points[bottom_points.size() - 2];
    p1 = bottom_points.back();

    bot_index = bottom_points.size() - 2;

    p2 = top_points.front();
    p3 = top_points[1];

    if (top_points.size() == 2)
    {
        top_index = 0;
    }
    else
    {
        top_index = 1;
    }

    if (bot_index < 0 || top_index < 0)
    {
        return false;
    }
    control_points.clear();
    control_points.emplace_back(p0);
    control_points.emplace_back(p1);
    control_points.emplace_back(p2);
    control_points.emplace_back(p3);

    const float interval = 2.0;
    if (ThirdOrderBezier(interval, control_points))
    {
        return true;
        // bottom_points.erase(vec.begin() + bot_index + 1,bottom_points.end());//remove points after p0;
        // top_points.erase(top_points.begin(),top_points.begin() + top_index+1);
        // top_points.insert(top_points.begin(),control_points.begin()+1,control_points.end());
    }
    else
    {
        control_points.clear();
        return false;
    }

    return false;
}
}
} // namespace fusion
} // namespace cem

#endif
