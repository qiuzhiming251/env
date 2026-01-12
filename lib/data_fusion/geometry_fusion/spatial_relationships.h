#pragma once
#include <string>
#include <vector>
#include "message/env_model/traffic_road/geometry_line.h"
#include <Eigen/Core>
#include "modules/perception/env/src/lib/localmap_construction/crossroad_construction/kde_regression.h"
// #include "spatial_distances.h"

namespace geometry {
namespace spatial_relationships {
using Point = cem::message::env_model::SamplingPoint;

enum class ConnectionStructure {
  kNotConnected = 0,
  kOverlappedSameDirection = 1,
  kOverlappedReverseDirection = 2,
  kHeadHeadConnected = 3,
  kTailTailConnected = 4,
  kHead1Tail2Connected = 5,
  kHead2Tail1Connected = 6,
};

double PointDotProduct(const cem::message::env_model::SamplingPoint &a,
                       const cem::message::env_model::SamplingPoint &b);


cem::message::env_model::SamplingPoint ProjectionPointPointToCurve(
    const cem::message::env_model::SamplingPoint &pt, const cem::message::env_model::GeometryLine &curve);

bool IsPointOnCurve(const cem::message::env_model::SamplingPoint &pt,
                    const cem::message::env_model::GeometryLine &curve, double tolerance);

int IndexOfPointOnCurve(const cem::message::env_model::SamplingPoint &pt,
                        const cem::message::env_model::GeometryLine &curve,
                        double tolerance);


static bool Point2LineDistance(
      const cem::message::env_model::SamplingPoint &pt,
      const cem::message::env_model::GeometryLine &curve,
      float &distance);

}  // namespace spatial_relationships
}  // namespace geometry
