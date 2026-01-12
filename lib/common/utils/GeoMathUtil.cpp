
#include "GeoMathUtil.h"

#include <iostream>
#include <cmath>
//#include "GeographicLib/Geodesic.hpp"
//#include "GeographicLib/GeodesicLine.hpp"
//#include "GeographicLib/Constants.hpp"
//#include <GeographicLib/TransverseMercator.hpp>


using namespace cem::fusion;

double GeoMathUtil::PI = 3.14159265358979;
double GeoMathUtil::zero_range = 0.15;  //向量叉乘结果浮动范围判断是否平行
double GeoMathUtil::dist_range = 0.03;  //判断端点是否重合的标准距离


// bool GeoMathUtil::get_mapping_foot(const Eigen::Vector2d& in,  const std::vector<Eigen::Vector2d>& vctvdGeo,
//                                 Eigen::Vector2d& outPt, double& outDist, int& outIndex) {
//   if(vctvdGeo.empty()) {
//     return false;
//   }
//   std::vector<std::vector<int>> vctIndexOut = {{0,(int)vctvdGeo.size() - 1}};
//   outDist = std::numeric_limits<double>::max();
//   for (auto index_tmp : vctIndexOut) {
//     Eigen::Vector2d foot_pt;
//     double foot_dist = 0;
//     int foot_index_in_geo = 0;
//     if (hasMappingPointOnLine(in, vctvdGeo, index_tmp[0], index_tmp[1], false, foot_pt, foot_dist, foot_index_in_geo)) {
//       if (foot_dist < outDist) {
//         outDist = foot_dist;
//         outPt = foot_pt;
//         outIndex = foot_index_in_geo;
//       }
//     }
//   }
//   if (outDist == std::numeric_limits<double>::max()) {
//     outDist = 0;
//     return false;
//   } else {
//     outIndex = (outIndex <= 0) ? 1 : outIndex;
//     return true;
//   }
// }

/**
 * 比较double数值大小
 * @param a
 * @param b
 * @return
 */
// int GeoMathUtil::compareDouble(const double& num1, const double& num2, double standard) {
//   if (fabs(num1 - num2) <= standard)
//     return 0;
//   if (num1 > num2)
//     return 1;
//   else
//     return -1;
// }

//double GeoMathUtil::GetWgsPointDist(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2) {
//    try {
//        double s12 = 0;
//        GeographicLib::Geodesic::WGS84().Inverse(pt1.y(), pt1.x(), pt2.y(), pt2.x(), s12);
//        return s12;
//    } catch (const std::exception& e) { return 0; }
//}
//
//double GeoMathUtil::GetWgsPointHeading(const Eigen::Vector2d& pt_start, const Eigen::Vector2d& pt_end) {
//    try {
//        double azi1 = 0;
//        double azi2 = 0;
//        GeographicLib::Geodesic::WGS84().Inverse(pt_start.y(), pt_start.y(), pt_end.y(), pt_end.x(), azi1, azi2);
//        if (azi1 < 0) {
//            azi1 += 360;
//        }
//        return azi1;
//    } catch (const std::exception& e) { return 0; }
//}
//
//Eigen::Vector2d GeoMathUtil::WgstoTM(const Eigen::Vector2d& pt_wgs, const double& lon) {
//    Eigen::Vector2d res;
//    GeographicLib::TransverseMercator::UTM().Forward(lon, pt_wgs.y(), pt_wgs.x(), res.x(), res.y());
//    return res;
//}
//
//Eigen::Vector2d GeoMathUtil::TMtoWgs(const Eigen::Vector2d& pt_tm, const double& lon) {
//    Eigen::Vector2d res;
//    GeographicLib::TransverseMercator::UTM().Reverse(lon, pt_tm.x(), pt_tm.y(), res.y(), res.x());
//    return res;
//}

RectRange::RectRange(Eigen::Vector2d center, float range) : center(center), range(range) {
  lt = center;
  lt.x() -= range;
  lt.y() += range;

  rb = center;
  rb.x() += range;
  rb.y() -= range;
}

void RectRange::setRange(const Eigen::Vector2d& p_center, float p_range) {
  this->center = p_center;
  this->range = p_range;

  lt = center;
  lt.x() -= range;
  lt.y() += range;

  rb = center;
  rb.x() += range;
  rb.y() -= range;
}

bool RectRange::isInRange(const Eigen::Vector2d& in) const {
  return (in.x() > lt.x() && in.x() < rb.x() && in.y() < lt.y() && in.y() > rb.y());
}