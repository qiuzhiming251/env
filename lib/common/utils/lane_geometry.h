#pragma once

#include "common/utility.h"
#include "lib/common/utils/GeoMathUtil.h"

namespace cem {
namespace fusion {
namespace LaneGeometry {
class PolynomialFitting;
typedef std::vector<Eigen::Vector2f> Line;

//////////////////////////////LANE TOPO/////////////////////////////////////
bool JudgeIsLeft(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);

bool JudgeIsLeft(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2, const PolynomialFitting &l1_fit,
                 const PolynomialFitting &l2_fit);

bool JudgeIsLeft(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line);

bool JudgeIsLeftUseRawPt(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);
int  JudgeIsLeftUseRawPtFilter(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);
bool JudgeIsLeft_navigation(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line);

double CalculateLineOverlap(const Eigen::Vector2f &track_start, const Eigen::Vector2f &track_end, const Eigen::Vector2f &object_start,
                            const Eigen::Vector2f &object_end);

double CalcLinesOverlap(const Line &l0, const Line &l1, int method);

int pnpoly(std::vector<Eigen::Vector2d> &poly, const Eigen::Vector2d &pt);
///////////////////////////////DISTANCE////////////////////////////////////
double CalculateDistance(const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2);

double DistPointToLineSegment(const Eigen::Vector2f &pt, const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2);

double DistPointToLineSegmentHasOverLap(const Eigen::Vector2f &pt, const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2);

Eigen::Vector2f PointProjectToLineSegment(const Eigen::Vector2f &lpt0, const Eigen::Vector2f &lpt1, const Eigen::Vector2f &pt);

double GetDistToLine(const Eigen::Vector2f &pt, const Line &line, bool need_extent);

double GetYDistanceBetweenLines(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);

double GetDistanceBetweenLines(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);

double GetDistanceBetweenLinesThin(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);

double GetProjDistanceBetweenLinesAverage(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2);

double InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x);

double GetDistanceBetweenLinesOverX(const std::vector<Eigen::Vector2f> &line1, const std::vector<Eigen::Vector2f> &line2);

/////////////////////////////////OTHERS/////////////////////////////////////
double GetLineLength(const Line &line);

void InsertPointsWithLine(const std::vector<Eigen::Vector2f> &origin_pts, std::vector<Eigen::Vector2f> &output_pts, const double gap);

bool IsBoundaryCrossLine(const std::vector<Eigen::Vector2f> &line, const std::vector<Eigen::Vector2f> &boundary);

double GetThreePointAngleCos(const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2, const Eigen::Vector2f &pt3);

double CalculateCurvatureFrom3Points(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

bool SegmentIsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4);

void IsIntersectForTwoLines(const Line &l1, const Line &l2, std::vector<int> &l1_intersect_idxs);

bool CalculateAverageDistanceBetweenTwoLines(std::vector<Eigen::Vector2f>& line_1, std::vector<Eigen::Vector2f>& line_2, std::pair<float, float> &distance);

double CalLinesSimilar(LaneGeometry::Line& l1, LaneGeometry::Line& l2);
class PolynomialFitting {
 public:
  PolynomialFitting() = default;

  explicit PolynomialFitting(std::vector<double> x_vec, std::vector<double> y_vec, const int order)
      : x_vec_(std::move(x_vec)), y_vec_(std::move(y_vec)) {
    Eigen::VectorXd xvals = Eigen::Map<Eigen::VectorXd>(&x_vec_[0], x_vec_.size());
    Eigen::VectorXd yvals = Eigen::Map<Eigen::VectorXd>(&y_vec_[0], y_vec_.size());

    if (xvals.size() != yvals.size()) {
      //std::cout << "xvals.size() != yvals.size(), PolynomialFitting fail!" << std::endl;
    }
    if (order > xvals.size() + 1) {
      //std::cout << "Too few points, PolynomialFitting fail!" << std::endl;
    }

    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto            Q      = A.householderQr();
    Eigen::VectorXd coeffs = Q.solve(yvals);
    for (long i = 0; i < coeffs.size(); i++) {
      coeffs_.push_back(coeffs[i]);
    }
  }

  ~PolynomialFitting() = default;

  const std::vector<double> &coeffs() const { return coeffs_; }

  /// @brief filter coefficients: coeffs_ = last_coeffs * w + coeffs_(1-w);
  /// @param last_coeffs poly distance can not more than 1.0m
  /// @param w the weight for filter
  /// @return true if input poly is near enough
  bool PolyFilter(const std::vector<double> &last_coeffs, const double &w) {
    if (coeffs_.size() != last_coeffs.size() || w > 1.0 || w < 0.0) {
      return false;
    }
    const auto error = 1.0;
    if (std::abs(GetValue(coeffs_, 0) - GetValue(last_coeffs, 0)) > error) {
      return false;
    }
    if (std::abs(GetValue(coeffs_, 10) - GetValue(last_coeffs, 10)) > error) {
      return false;
    }
    for (size_t i = 0; i < coeffs_.size(); i++) {
      coeffs_[i] = coeffs_[i] * (1 - w) + last_coeffs[i] * w;
    }
    return true;
  }

  double GetValue(const std::vector<double> &coeffs, const double &x) const {
    double y = 0.0;
    for (size_t i = 0; i < coeffs.size(); i++) {
      y += coeffs[i] * pow(x, double(i));
    }
    return y;
  }

  double GetValue(const double &x) const {
    double y = 0.0;
    for (size_t i = 0; i < coeffs_.size(); i++) {
      y += coeffs_[i] * pow(x, double(i));
    }
    return y;
  }

 private:
  std::vector<double> x_vec_;
  std::vector<double> y_vec_;
  std::vector<double> coeffs_;
};
}  // namespace LaneGeometry
}  // namespace fusion
}  // namespace cem
