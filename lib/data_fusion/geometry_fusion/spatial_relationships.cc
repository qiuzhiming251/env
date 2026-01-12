
#include "spatial_relationships.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <vector>

namespace geometry {
namespace spatial_relationships {

double PointDotProduct(const cem::message::env_model::SamplingPoint &a,
                       const cem::message::env_model::SamplingPoint &b) {
  return a.x * b.x + a.y * b.y;
}




Point ProjectionPointPointToCurve(const cem::message::env_model::SamplingPoint &pt,
                                  const cem::message::env_model::GeometryLine &curve) {
  assert(!curve.sampling_points.empty());
  // adapt data type and call geometry operations in geometry_2d.h

  int index;
  Eigen::Vector2f proj_pt;
  //TO DO
  // map_interface::GeometryXY::DistanceXY2CurveLineSegWithIndex(geo_pt, geo_curve,
  //                                                             &index, &proj_pt);
  //返回点在曲线上的投影点和所在曲线的索引

  double angle = KdeRegression::CalcCurveDirection(curve, index);
  cem::message::env_model::SamplingPoint ret;
  ret.x = proj_pt.x();
  ret.y = proj_pt.y();
  ret.heading = angle;
  return ret;
}

bool IsPointOnCurve(const cem::message::env_model::SamplingPoint &pt,
                    const cem::message::env_model::GeometryLine &curve, double tolerance) {
  const auto &curve_points = curve.sampling_points;
  return std::any_of(curve_points.begin(), curve_points.end(),
                     [&](const cem::message::env_model::SamplingPoint &curve_pt) {
                       return std::sqrt((curve_pt.x -pt.x)* (curve_pt.x-pt.x) + (curve_pt.y-pt.y)* (curve_pt.y - pt.y)) < tolerance;
                     });
}


int IndexOfPointOnCurve(const cem::message::env_model::SamplingPoint &pt,
                        const cem::message::env_model::GeometryLine &curve,
                        double tolerance) {
  const auto &curve_points = curve.sampling_points;
  for (size_t i = 0U; i < curve_points.size(); ++i) {
    const auto &curve_pt = curve_points[i];

    if (std::sqrt((curve_pt.x -pt.x)* (curve_pt.x-pt.x) + (curve_pt.y-pt.y)* (curve_pt.y - pt.y)) < tolerance) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

bool Point2LineDistance(
      const cem::message::env_model::SamplingPoint &pt,
      const cem::message::env_model::GeometryLine &curve,
      float &distance)
{
    if(!curve.is_curve_valid){
      return false;
    }
    
    distance = std::abs(curve.curve.c1*pt.x -1.f + curve.curve.c0)/
                  std::sqrt(curve.curve.c0*curve.curve.c0 + curve.curve.c1*curve.curve.c1);
    return true;
}

}  // namespace spatial_relationships
}  // namespace geometry
