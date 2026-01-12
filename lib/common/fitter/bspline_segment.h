#ifndef _BSPLINE_SEGMENT_H_
#define _BSPLINE_SEGMENT_H_

#include <deque>
#include <iostream>
#include <set>
#include <cfloat>
#include <Eigen/Dense>
#include "message/internal_message.h"
#include "cyber/common/log.h"
#include "least_square_fitter.h"
#include "least_squares_solver.h"
#include "least_squares_solver_plus.h"
#include "common/utility.h"

namespace cem {
namespace fusion {

using namespace std;
using namespace Eigen;

typedef cem::message::common::Point2DF Point2DF;

int judgePointInPnPoly(const std::vector<Point2DF> &vertices, const float &x, const float &y);

void computeRectangleWithVector(
    const Eigen::Vector2f & first,
    const Eigen::Vector2f & second,
    const float & width,
    const float & length,
    std::vector<Point2DF> &vertices);

bool FitByBSpline(
    const std::vector<Eigen::Vector2f> & raw_points, 
    const int &cubic,
    std::vector<Eigen::Vector2f> &fitted_point_list);

void GetSegsByBSplineAndRectangle(
    const std::vector<Point2DF> &seg, 
    std::vector<std::vector<Point2DF>>& seg_groups);

void FitRawCurveLineWithCoeffs(
    const std::vector<Point2DF> &raw_line_points, 
    const bool & removed_invalid,
    std::vector<Curve> & fit_curves,
    std::vector<std::vector<Point2DF> > & bev_seg_groups);

} // namespace fusion
} // namespace cem

#endif
