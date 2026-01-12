

#pragma once

#include <Eigen/Core>
#include <utility>
#include <vector>
#include "lib/data_fusion/geometry_fusion/spatial_relationships.h"
#include <cstdint> // for uint32_t
#include <numeric>
#include <cmath>
#include "lib/common/log_custom.h"

namespace geometry {

using Point = Eigen::Vector2f;
using DirectedPoint = Eigen::Vector3f;
using WeightedPoint = std::pair<Point, float>;
// directed point with weight
struct WeightedDPoint {
  cem::message::env_model::SamplingPoint pt{};
  double theta{};
  double weight{};

  WeightedDPoint(cem::message::env_model::SamplingPoint pt, double theta, double weight)
      : pt(std::move(pt)), theta(theta), weight(weight) {}
      WeightedDPoint& operator = (WeightedDPoint &&other){
        if(this != &other){
          pt = other.pt;
          theta = other.theta;
          weight = other.weight;
        }
        return *this;
      }
};

struct WeightedCurve {
  cem::message::env_model::GeometryLine curve;
  double weight{0.0F};
  // 是否可以作为拟合起始迭代点
  bool start_end_candidate{false};

  WeightedCurve( cem::message::env_model::GeometryLine in_curve, float in_weight,
                bool in_start_candidate = false)
      : curve(std::move(in_curve)),
        weight(in_weight),
        start_end_candidate(in_start_candidate)
  {}

  WeightedCurve( cem::message::env_model::GeometryLine &&in_curve, float in_weight,
                bool in_start_candidate = false)
      : curve(in_curve),
        weight(in_weight),
        start_end_candidate(in_start_candidate)
  {}
  WeightedCurve(){};
//复制构造函数
  WeightedCurve& operator = (const WeightedCurve &other){
    if(this != &other){
      curve = other.curve;
      weight = other.weight;
      start_end_candidate = other.start_end_candidate;
    }
    return *this;
  };
};


struct {
  bool operator()(const WeightedCurve &ele1, const WeightedCurve &ele2) {
        return static_cast<double>(ele1.start_end_candidate) * ele1.weight <
               static_cast<double>(ele2.start_end_candidate) * ele2.weight;
  }
} CompareWeight;
// struct {
//   bool operator()(const WeightedCurve &ele1, const WeightedCurve &ele2) {
//         return static_cast<double>(ele1.start_end_candidate) * ele1.weight <
//                static_cast<double>(ele2.start_end_candidate) * ele2.weight;
//   }
// } CompareWeight;

static bool IsTwoPointsEqual(
    const Point &pt1, const Point &pt2, double tolerance) {
  if (std::abs(pt1.x()- pt2.x()) > tolerance) { return false; }
  if (std::abs(pt1.y() - pt2.y()) > tolerance) { return false; }
  return true;
}

class OBB {
 public:
  OBB(const DirectedPoint &st, float length, float width);

  bool IsContainPoint(const Point &pt) const;

  const DirectedPoint &GetStartPoint() const;

 private:
  DirectedPoint st_;
  float length_;
  float width_;
  float half_length_;
  float half_width_;
  float max_axis_diff_;

  float x_max_;
  float x_min_;
  float y_max_;
  float y_min_;

  Point center_;
  Eigen::Matrix2f rotation_matrix_;
};

class AABB {
 public:
  AABB(const DirectedPoint &st, float forward_len, float side_len);

  bool IsContainPoint(const Point &pt) const;

  const DirectedPoint &GetStartPoint() const;

 private:
  DirectedPoint st_;

  float x_max_{0.0f};
  float x_min_{0.0f};
  float y_max_{0.0f};
  float y_min_{0.0f};
};

class KdeRegression {
 public:
  // static bool SmoothingWithSpecifiedStart(
  //     const std::vector<WeightedCurve> &raw_curves,
  //     const double &max_error, const double &resolution,
  //     const Eigen::Vector3f &start_pt,
  //      cem::message::env_model::GeometryLine &fitted_curve);

  static bool Smoothing(
      const std::vector<WeightedCurve> &raw_curves,
      const double &max_error, const double &resolution,
       cem::message::env_model::GeometryLine &fitted_curve);

  static double CalcCurveDirection(const cem::message::env_model::GeometryLine &curve, int i);
 private:
  static void AdaptInputs(
      const std::vector<WeightedCurve> &raw_curves,
      std::vector<WeightedPoint> &points, std::vector<DirectedPoint> &seeds);

  static bool FindIterationStartPoint(const std::vector<WeightedPoint> &points,
                                      const std::vector<DirectedPoint> &seeds,
                                      const double &max_error,
                                      DirectedPoint &start);

  static Point FindIterationEndPoint(
      const std::vector<WeightedCurve> &curves, double max_error);

  static DirectedPoint FindIterationStartPoint(
      const std::vector<WeightedCurve> &curves, double max_error);

  static bool CurveFindSuccessor(
      const std::vector<WeightedCurve> &curves, double max_error,
      WeightedDPoint &seed, std::vector<bool> &visit_status);

  static bool CurveFindPredecessor(
      const std::vector<WeightedCurve> &curves, double max_error,
      WeightedDPoint &seed, std::vector<bool> &visit_status);

  static bool PerformKernelSmoothing(
      const DirectedPoint &start, const Point &end,
      const std::vector<WeightedPoint> &raw_points,
      float max_error,
      std::vector<Point> &smoothed_points);

  static bool IsRectangleClean(const OBB &rect,
                               const std::vector<WeightedPoint> &raw_points);

  static DirectedPoint CalcCenterPoint(const std::vector<DirectedPoint> &pts);
  inline static double CalcAngelAbs(const double &angle1, const double &angle2);
  inline static double AngleAverage(const std::vector<double> &angles);
  
};
template<typename T, typename f>
std::vector<T> VectorFilter(const std::vector<T>& vec, f predicate) {
  std::vector<T> result;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(result), predicate);
  return result;
}
}  // namespace geometry
