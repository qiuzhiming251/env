#include "lane_geometry.h"
#include "common/utility.h"

namespace cem {
namespace fusion {
namespace LaneGeometry {
constexpr double min_lanewidth_th = 2.0;

///////////////////////////////TOPO///////////////////////////////////////
bool JudgeIsLeft(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {
  if (l1.size() < 2 || l2.size() < 2) {
    return false;
  }
  double start_x = std::max(l1.front().x(), l2.front().x());
  double end_x   = std::min(l1.back().x(), l2.back().x());

  std::vector<double> x_vec, y_vec;
  for (auto &pt : l1) {
    x_vec.push_back(pt.x());
    y_vec.push_back(pt.y());
  }
  PolynomialFitting fit1(x_vec, y_vec, 3);
  double            start_y = fit1.GetValue(start_x);
  double            end_y   = fit1.GetValue(end_x);

  std::vector<double> x_vec2, y_vec2;
  for (auto &pt : l2) {
    x_vec2.push_back(pt.x());
    y_vec2.push_back(pt.y());
  }
  PolynomialFitting fit2(x_vec2, y_vec2, 3);
  double            start_y_2        = fit2.GetValue(start_x);
  double            end_y_2          = fit2.GetValue(end_x);
  double            delta_end        = std::fabs(end_y - end_y_2);
  double            delta_start      = std::fabs(start_y - start_y_2);
  Eigen::Vector3d   line_direct_unit = Eigen::Vector3d(end_x - start_x, end_y - start_y, 0).normalized();
  if (end_x <= start_x) {
    line_direct_unit = Eigen::Vector3d(l1.back().x() - l1.front().x(), l1.back().y() - l1.front().y(), 0).normalized();
  }

  std::vector<double> sample_x_vec = {
      start_x, 0.5 * start_x + 0.5 * end_x, 0.3 * start_x + 0.7 * end_x, 0.2 * start_x + 0.8 * end_x, 0.1 * start_x + 0.9 * end_x, end_x};
  if (delta_end < delta_start) {
    sample_x_vec = {
        start_x, 0.5 * start_x + 0.5 * end_x, 0.7 * start_x + 0.3 * end_x, 0.8 * start_x + 0.2 * end_x, 0.9 * start_x + 0.1 * end_x, end_x};
  }
  int left_cnt  = 0;
  int right_cnt = 0;
  for (size_t i = 0; i < sample_x_vec.size(); i++) {
    const double          sample_x                  = sample_x_vec.at(i);
    double                sample_y1                 = fit1.GetValue(sample_x);
    double                sample_y2                 = fit2.GetValue(sample_x);
    const Eigen::Vector3d vertical_line_direct_unit = Eigen::Vector3d(0, sample_y2 - sample_y1, 0).normalized();
    const Eigen::Vector3d res                       = line_direct_unit.cross(vertical_line_direct_unit);
    double                crossproduct              = res.z();
    if (crossproduct < 0)
      left_cnt++;
    else
      right_cnt++;
  }
  return left_cnt > right_cnt;
}

bool JudgeIsLeft(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2, const PolynomialFitting &l1_fit,
                 const PolynomialFitting &l2_fit) {
  if (l1.size() < 2 || l2.size() < 2) {
    return false;
  }
  double start_x = std::max(l1.front().x(), l2.front().x());
  double end_x   = std::min(l1.back().x(), l2.back().x());

  double start_y = l1_fit.GetValue(start_x);
  double end_y   = l1_fit.GetValue(end_x);

  double start_y_2 = l2_fit.GetValue(start_x);
  double end_y_2   = l2_fit.GetValue(end_x);

  double          delta_end        = std::fabs(end_y - end_y_2);
  double          delta_start      = std::fabs(start_y - start_y_2);
  Eigen::Vector3d line_direct_unit = Eigen::Vector3d(end_x - start_x, end_y - start_y, 0).normalized();
  if (end_x <= start_x) {
    line_direct_unit = Eigen::Vector3d(l1.back().x() - l1.front().x(), l1.back().y() - l1.front().y(), 0).normalized();
  }

  std::vector<double> sample_x_vec = {
      start_x, 0.5 * start_x + 0.5 * end_x, 0.3 * start_x + 0.7 * end_x, 0.2 * start_x + 0.8 * end_x, 0.1 * start_x + 0.9 * end_x, end_x};
  if (delta_end < delta_start) {
    sample_x_vec = {
        start_x, 0.5 * start_x + 0.5 * end_x, 0.7 * start_x + 0.3 * end_x, 0.8 * start_x + 0.2 * end_x, 0.9 * start_x + 0.1 * end_x, end_x};
  }
  int left_cnt  = 0;
  int right_cnt = 0;
  for (size_t i = 0; i < sample_x_vec.size(); i++) {
    const double          sample_x                  = sample_x_vec.at(i);
    double                sample_y1                 = l1_fit.GetValue(sample_x);
    double                sample_y2                 = l2_fit.GetValue(sample_x);
    const Eigen::Vector3d vertical_line_direct_unit = Eigen::Vector3d(0, sample_y2 - sample_y1, 0).normalized();
    const Eigen::Vector3d res                       = line_direct_unit.cross(vertical_line_direct_unit);
    double                crossproduct              = res.z();
    if (crossproduct < 0)
      left_cnt++;
    else
      right_cnt++;
  }
  return left_cnt > right_cnt;
}

bool JudgeIsLeft(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line) {

  if (line.size() < 2) {
    return false;
  }
  int left_cnt  = 0;
  int right_cnt = 0;
  for (unsigned i = 1; i < line.size(); i++) {
    const Eigen::Vector3d line_direct_unit = Eigen::Vector3d(line[i].x() - line[i - 1].x(), line[i].y() - line[i - 1].y(), 0).normalized();
    const Eigen::Vector3d pt_direct_unit   = Eigen::Vector3d(pt.x() - line[i - 1].x(), pt.y() - line[i - 1].y(), 0).normalized();
    const Eigen::Vector3d res              = line_direct_unit.cross(pt_direct_unit);
    double                crossproduct     = res.z();
    if (crossproduct > 0) {
      left_cnt++;
    } else {
      right_cnt++;
    }
  }
  return left_cnt > right_cnt;
}

bool JudgeIsLeft_navigation(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line) {
  if (line.empty())
    return 0;

  int   bestIdx = 0;
  float bestDx  = std::numeric_limits<float>::infinity();

  for (int i = 0; i < static_cast<int>(line.size()); ++i) {
    float dx = std::fabs(line[i].x() - pt.x());
    if (dx < bestDx) {
      bestDx  = dx;
      bestIdx = i;
    }
  }

  const float dy = pt.y() - line[bestIdx].y();
  return dy > 0.0f;
}

int JudgeIsLeftFilter(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line) {  //距离近不比较

  if (line.size() < 2) {
    return -1;
  }
  int left_cnt  = 0;
  int right_cnt = 0;
  for (unsigned i = 1; i < line.size(); i++) {
    const Eigen::Vector3d line_direct  = Eigen::Vector3d(line[i].x() - line[i - 1].x(), line[i].y() - line[i - 1].y(), 0);
    const Eigen::Vector3d pt_direct    = Eigen::Vector3d(pt.x() - line[i - 1].x(), pt.y() - line[i - 1].y(), 0);
    const Eigen::Vector3d res          = line_direct.cross(pt_direct);
    double                crossproduct = res.z();
    auto                  distance     = res.norm() / line_direct.norm();
    if (distance < 1.0 || distance > 10.0) {
      continue;
    }
    if (crossproduct > 0) {
      left_cnt++;
    } else {
      right_cnt++;
    }
  }

  if (left_cnt > right_cnt) {
    return 1;
  } else if (left_cnt < right_cnt) {
    return 0;
  } else {
    return -1;
  }
}

bool JudgeIsLeftUseRawPt(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {

  if (l1.size() < 2 || l2.size() < 2) {
    return false;
  }
  int left_cnt  = 0;
  int right_cnt = 0;

  for (auto &l1_pt : l1) {
    if (l2.size() == 2 && fabs(l1_pt.x() - l2.front().x()) < 1e-6 && fabs(l1_pt.y() - l2.front().y()) < 1e-6) {
      continue;
    }
    if (JudgeIsLeft(l1_pt, l2)) {
      left_cnt++;
    } else {
      right_cnt++;
    }
  }
  return left_cnt > right_cnt;
}

int JudgeIsLeftUseRawPtFilter(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {

  if (l1.size() < 2 || l2.size() < 2) {
    return false;
  }
  int left_cnt  = 0;
  int right_cnt = 0;
  for (auto &l1_pt : l1) {
    if (l2.size() == 2 && fabs(l1_pt.x() - l2.front().x()) < 1e-6 && fabs(l1_pt.y() - l2.front().y()) < 1e-6) {
      continue;
    }
    auto ret = JudgeIsLeftFilter(l1_pt, l2);
    if (1 == ret) {
      left_cnt++;
    } else if (0 == ret) {
      right_cnt++;
    }
  }
  if (0 != left_cnt || 0 != right_cnt) {
    return left_cnt > right_cnt;
  } else {
    return -1;
  }
}

double CalculateLineOverlap(const Eigen::Vector2f &track_start, const Eigen::Vector2f &track_end, const Eigen::Vector2f &object_start,
                            const Eigen::Vector2f &object_end) {
  double          overlap = 0.0;
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Eigen::Vector2f p3;
  Eigen::Vector2f p4;
  double          track_length  = (track_start - track_end).norm();
  double          object_length = (object_start - object_end).norm();
  double          length;
  if (track_length > object_length) {
    p1     = track_start;
    p2     = track_end;
    p3     = object_start;
    p4     = object_end;
    length = track_length;
  } else {
    p1     = object_start;
    p2     = object_end;
    p3     = track_start;
    p4     = track_end;
    length = object_length;
  }
  if (p2.x() == p3.x() || p2.y() == p3.y()) {
    p3.x() += 0.1;
    p3.y() += 0.1;
  }
  if (p2.x() == p4.x() || p2.y() == p4.y()) {
    p4.x() += 0.1;
    p4.y() += 0.1;
  }
  // 计算两个点（向量）的点积
  auto DotProduct = [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) -> double {
    return a.x() * b.x() + a.y() * b.y();
  };
  // 计算向量的模长
  auto Magnitude = [&DotProduct](const Eigen::Vector2f &vec) -> double {
    return std::sqrt(DotProduct(vec, vec));
  };

  // 创建向量
  Eigen::Vector2f vec_p1p2(p1.x() - p2.x(), p1.y() - p2.y());
  Eigen::Vector2f vec_p2p3(p3.x() - p2.x(), p3.y() - p2.y());
  Eigen::Vector2f vec_p2p4(p4.x() - p2.x(), p4.y() - p2.y());

  double dot23 = DotProduct(vec_p2p3, vec_p1p2);
  double dot24 = DotProduct(vec_p2p4, vec_p1p2);

  double magnitude = Magnitude(vec_p1p2);
  // 计算投影长度
  double projectedLength23 = dot23 / magnitude;
  double projectedLength24 = dot24 / magnitude;
  // 类型1
  if (projectedLength23 > length && length != 0) {
    if (projectedLength24 > length) {
      overlap = 0.0;
    }
    if (projectedLength24 > 0 && projectedLength24 < length) {
      overlap = (length - projectedLength24) / length;
    }
    if (projectedLength24 < 0) {
      overlap = 1;
    }
  }
  // 类型2
  if ((projectedLength23 > 0 && projectedLength23 < length) && length != 0) {
    if (projectedLength24 > 0 && projectedLength24 < length) {
      overlap = (projectedLength23 - projectedLength24) / length;
    }
    if (projectedLength24 < 0) {
      overlap = projectedLength23 / length;
    }
  }
  if (projectedLength23 < 0) {
    overlap = 0;
  }
  return overlap;
}

double CalcLinesOverlap(const Line &l0, const Line &l1, int method) {
  if (l0.size() < 2 || l1.size() < 2) {
    return 0;
  }
  auto         dis_1     = CalculateDistance(l0.front(), l0.back());
  auto         dis_2     = CalculateDistance(l1.front(), l1.back());
  const double threshold = 1e-8;
  if (dis_1 < threshold || dis_2 < threshold) {
    return 0.0;
  }

  Eigen::Vector2f start_project_pt = PointProjectToLineSegment(l0.front(), l0.back(), l1.front());
  Eigen::Vector2f end_project_pt   = PointProjectToLineSegment(l0.front(), l0.back(), l1.back());
  double          overlap0         = CalculateDistance(start_project_pt, end_project_pt) / CalculateDistance(l0.front(), l0.back());
  if (method == 1) {
    return overlap0;
  }

  double overlap1 = CalculateDistance(start_project_pt, end_project_pt) / CalculateDistance(l1.front(), l1.back());
  if (method == 2) {
    return (overlap0 + overlap1) / 2;
  }
  return std::max(overlap0, overlap1);
}

//// pnpoly判断点是否在多边形内
int pnpoly(std::vector<Eigen::Vector2d> &poly, const Eigen::Vector2d &pt) {
  if (poly.size() < 3) {
    return 0;
  }
  size_t i, j, c = 0;
  for (i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y() > pt.y()) != (poly[j].y() > pt.y())) &&
        (pt.x() < (poly[j].x() - poly[i].x()) * (pt.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()))
      c = !c;
  }
  return c;
}

///////////////////////////////DISTANCE////////////////////////////////////
double CalculateDistance(const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2) {
  return std::sqrt((pt1(0) - pt2(0)) * (pt1(0) - pt2(0)) + (pt1(1) - pt2(1)) * (pt1(1) - pt2(1)));
}

double DistPointToLineSegment(const Eigen::Vector2f &pt, const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2) {
  double a = CalculateDistance(pt1, pt2);
  double b = CalculateDistance(pt1, pt);
  double c = CalculateDistance(pt2, pt);
  if (a == (b + c))
    return 0.0;
  if (a <= 0.001)
    return std::min(b, c);
  if (c * c >= a * a + b * b)
    return b;
  if (b * b >= a * a + c * c)
    return c;
  double circum = 0.5 * (a + b + c);
  double area   = std::sqrt(circum * (circum - a) * (circum - b) * (circum - c));
  return 2 * area / a;
}

double DistPointToLineSegmentHasOverLap(const Eigen::Vector2f &pt, const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2) {
  double a = CalculateDistance(pt1, pt2);
  double b = CalculateDistance(pt1, pt);
  double c = CalculateDistance(pt2, pt);
  if (a == (b + c))
    return 0.0;
  if (a <= 0.001)
    return std::min(b, c);
  if (c * c > a * a + b * b)
    return -1;
  if (b * b > a * a + c * c)
    return -1;
  double circum = 0.5 * (a + b + c);
  double area   = std::sqrt(circum * (circum - a) * (circum - b) * (circum - c));
  return 2 * area / a;
}

Eigen::Vector2f PointProjectToLineSegment(const Eigen::Vector2f &lpt0, const Eigen::Vector2f &lpt1, const Eigen::Vector2f &pt) {
  double a = CalculateDistance(lpt0, lpt1);
  double b = CalculateDistance(lpt0, pt);
  double c = CalculateDistance(lpt1, pt);

  if (c * c >= a * a + b * b)
    return lpt0;
  if (b * b >= a * a + c * c)
    return lpt1;

  double          project_distance = ((pt.x() - lpt0.x()) * (lpt1.x() - lpt0.x()) + (pt.y() - lpt0.y()) * (lpt1.y() - lpt0.y())) / a;
  double          dx               = (project_distance / a) * (lpt1.x() - lpt0.x()) + lpt0.x();
  double          dy               = (project_distance / a) * (lpt1.y() - lpt0.y()) + lpt0.y();
  Eigen::Vector2f project_pt       = Eigen::Vector2f(dx, dy);
  return project_pt;
}

double GetDistToLine(const Eigen::Vector2f &pt, const Line &line, bool need_extent) {
  double      min_dist = 999999.0;
  const auto &pts      = line;
  if (pts.empty())
    return min_dist;
  if (pts.size() == 1)
    return (pt - pts.front()).norm();
  std::vector<Eigen::Vector2f> pts_extent;
  const double                 extent_length = 10;  // m
  if (need_extent) {
    const auto &s    = pts.at(1);
    const auto &e    = pts.at(0);
    const auto  unit = (e - s) / (e - s).norm();
    pts_extent.emplace_back(extent_length * unit + pts.at(0));
  }
  pts_extent.insert(pts_extent.end(), pts.begin(), pts.end());
  if (need_extent) {
    const auto &s    = pts.at(pts.size() - 2);
    const auto &e    = pts.at(pts.size() - 1);
    const auto  unit = (e - s) / (e - s).norm();
    pts_extent.emplace_back(extent_length * unit + pts.back());
  }

  for (size_t i = 0; i + 1 < pts_extent.size(); ++i) {
    double dis = DistPointToLineSegment(pt, pts_extent.at(i), pts_extent.at(i + 1));
    min_dist   = std::min(min_dist, dis);
  }
  return min_dist;
}

double GetYDistanceBetweenLines(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {
  if (l1.size() < 2 || l2.size() < 2)
    return 9999.0;

  size_t max_pts = 10;

  // Step 1: 取末尾/开头的点
  std::vector<Eigen::Vector2f> l1_fit_pts(l1.end() - std::min(max_pts, l1.size()), l1.end());
  std::vector<Eigen::Vector2f> l2_fit_pts(l2.begin(), l2.begin() + std::min(max_pts, l2.size()));

  // Step 2: 计算 y 差
  double dy1 = std::abs(l1_fit_pts.back().y() - l1_fit_pts.front().y());
  double dy2 = std::abs(l2_fit_pts.back().y() - l2_fit_pts.front().y());

  // Step 3: 拟合函数
  auto FitLine = [](const std::vector<Eigen::Vector2f> &pts, double &a, double &b) -> bool {
    int n = pts.size();
    if (n < 2)
      return false;

    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (const auto &pt : pts) {
      sum_x += pt.x();
      sum_y += pt.y();
      sum_xx += pt.x() * pt.x();
      sum_xy += pt.x() * pt.y();
    }

    double denom = n * sum_xx - sum_x * sum_x;
    if (std::fabs(denom) < 1e-6)
      return false;

    a = (n * sum_xy - sum_x * sum_y) / denom;
    b = (sum_y - a * sum_x) / n;
    return true;
  };

  double a = 0, b = 0;

  if (dy1 < dy2) {
    // 用 l1 拟合，测量 l2 起始点的垂直距离
    if (!FitLine(l1_fit_pts, a, b))
      return 9999.0;

    const Eigen::Vector2f &pt   = l2_fit_pts.front();
    double                 dist = std::abs(a * pt.x() - pt.y() + b) / std::sqrt(a * a + 1.0);
    return dist;
  } else {
    // 用 l2 拟合，测量 l1 末尾点的垂直距离
    if (!FitLine(l2_fit_pts, a, b))
      return 9999.0;

    const Eigen::Vector2f &pt   = l1_fit_pts.back();
    double                 dist = std::abs(a * pt.x() - pt.y() + b) / std::sqrt(a * a + 1.0);
    return dist;
  }
}

double GetDistanceBetweenLines(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {
  if (l1.size() < 2 || l2.size() < 2) {
    return 9999.0;
  }
  double distance = 9999.0;
  for (size_t i = 0; i < l1.size(); ++i) {
    int    index = -1;
    double dist  = 9999.0;
    for (size_t j = 0; j < l2.size(); ++j) {
      double tmp = CalculateDistance(l1[i], l2[j]);
      if (tmp < dist) {
        dist  = tmp;
        index = j;
      }
    }
    if (index >= 0 && index < (int)(l2.size())) {
      if (index > 0) {
        double dist1 = DistPointToLineSegment(l1[i], l2[index], l2[index - 1]);
        dist         = std::min(dist, dist1);
      }
      if (index < (int)(l2.size()) - 1) {
        double dist2 = DistPointToLineSegment(l1[i], l2[index], l2[index + 1]);
        dist         = std::min(dist, dist2);
      }
    }

    if (dist < distance) {
      distance = dist;
    }
  }
  return distance;
}

double GetDistanceBetweenLinesThin(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {
  if (l1.size() < 2 || l2.size() < 2) {
    return 9999.0;
  }
  double distance = 9999.0;
  for (size_t i = 0; i < l1.size(); ++i) {
    if (l1.size() > 20) {
      if (i % 5 != 0) {
        continue;
      }
    }
    int    index = -1;
    double dist  = 9999.0;
    for (size_t j = 0; j < l2.size(); ++j) {
      double tmp = CalculateDistance(l1[i], l2[j]);
      if (tmp < dist) {
        dist  = tmp;
        index = j;
      }
    }
    if (index >= 0 && index < (int)(l2.size())) {
      if (index > 0) {
        double dist1 = DistPointToLineSegment(l1[i], l2[index], l2[index - 1]);
        dist         = std::min(dist, dist1);
      }
      if (index < (int)(l2.size()) - 1) {
        double dist2 = DistPointToLineSegment(l1[i], l2[index], l2[index + 1]);
        dist         = std::min(dist, dist2);
      }
    }

    if (dist < distance) {
      distance = dist;
    }
  }
  return distance;
}

double InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x) {
  if (geos.empty())
    return std::numeric_limits<double>::quiet_NaN();
  if (x < geos.front().x() || x > geos.back().x())
    return std::numeric_limits<double>::quiet_NaN();

  for (size_t i = 0; i < geos.size() - 1; ++i) {
    if (geos[i].x() <= x && x <= geos[i + 1].x()) {
      double t = (x - geos[i].x()) / (geos[i + 1].x() - geos[i].x());
      return geos[i].y() + t * (geos[i + 1].y() - geos[i].y());
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double GetDistanceBetweenLinesOverX(const std::vector<Eigen::Vector2f> &line1, const std::vector<Eigen::Vector2f> &line2) {
  if (line1.size() < 2 || line2.size() < 2) {
    return 0.0;
  }

  double min_x1 = std::numeric_limits<double>::max(), max_x1 = std::numeric_limits<double>::lowest();
  for (const auto &pt : line1) {
    min_x1 = std::min(min_x1, static_cast<double>(pt.x()));
    max_x1 = std::max(max_x1, static_cast<double>(pt.x()));
  }
  double min_x2 = std::numeric_limits<double>::max(), max_x2 = std::numeric_limits<double>::lowest();
  for (const auto &pt : line2) {
    min_x2 = std::min(min_x2, static_cast<double>(pt.x()));
    max_x2 = std::max(max_x2, static_cast<double>(pt.x()));
  }

  double overlap_min = std::max(min_x1, min_x2);
  double overlap_max = std::min(max_x1, max_x2);

  if (overlap_min >= overlap_max) {
    return -1;
  }

  const int num_samples   = 10;
  double    step          = (overlap_max - overlap_min) / (num_samples - 1);
  double    sum_dist      = 0.0;
  int       valid_samples = 0;

  for (int i = 0; i < num_samples; ++i) {
    double x  = overlap_min + i * step;
    double y1 = InterpolateYAtX(line1, x);
    double y2 = InterpolateYAtX(line2, x);
    if (!std::isnan(y1) && !std::isnan(y2)) {
      sum_dist += std::abs(y1 - y2);
      valid_samples++;
    }
  }

  return (valid_samples > 0) ? (sum_dist / valid_samples) : 0.0;
}

double GetProjDistanceBetweenLinesAverage(const std::vector<Eigen::Vector2f> &l1, const std::vector<Eigen::Vector2f> &l2) {
  if (l1.size() < 2 || l2.size() < 2) {
    return -1;
  }
  std::vector<double> distance_list = {};
  for (size_t i = 0; i < l1.size(); ++i) {
    if (l1.size() > 20) {
      if (i % 5 != 0) {
        continue;
      }
    }
    int    index = -1;
    double dist  = 9999.0;
    for (size_t j = 0; j < l2.size(); ++j) {
      double tmp = CalculateDistance(l1[i], l2[j]);
      if (tmp < dist) {
        dist  = tmp;
        index = j;
      }
    }
    double proj_dist = 9999;
    if (index >= 0 && index < (int)(l2.size())) {
      if (index > 0) {
        double dist1 = DistPointToLineSegment(l1[i], l2[index], l2[index - 1]);
        if (dist1 != -1) {
          proj_dist = std::min(proj_dist, dist1);
        }
      }
      if (index < (int)(l2.size()) - 1) {
        double dist2 = DistPointToLineSegment(l1[i], l2[index], l2[index + 1]);
        if (dist2 != -1) {
          proj_dist = std::min(proj_dist, dist2);
        }
      }
    }
    if (proj_dist != 9999) {
      distance_list.push_back(proj_dist);
    }
  }
  if (!distance_list.empty()) {
    double sum = std::accumulate(distance_list.begin(), distance_list.end(), 0.0);
    return sum / distance_list.size();
  } else {
    return -1;
  }
}

/////////////////////////////////OTHERS/////////////////////////////////////
double GetLineLength(const Line &line) {
  double res_len = 0.0;
  if (line.empty()) {
    return res_len;
  }
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    res_len += (line.at(i + 1) - line.at(i)).norm();
  }
  return res_len;
}

void InsertPointsWithLine(const std::vector<Eigen::Vector2f> &origin_pts, std::vector<Eigen::Vector2f> &output_pts, const double gap) {
  output_pts.reserve(origin_pts.size());
  if (origin_pts.size() > 1) {
    for (size_t i = 1; i < origin_pts.size(); i++) {
      output_pts.emplace_back(origin_pts[i - 1]);
      double dis = (origin_pts[i - 1] - origin_pts[i]).norm();
      if (dis > gap) {
        double   x_diff = origin_pts[i].x() - origin_pts[i - 1].x();
        double   y_diff = origin_pts[i].y() - origin_pts[i - 1].y();
        uint32_t count  = dis / gap;

        double xt = x_diff * gap / dis;
        double yt = y_diff * gap / dis;

        for (uint32_t j = 0; j < count; j++) {
          double x = xt * (j + 1) + origin_pts[i - 1].x();
          double y = yt * (j + 1) + origin_pts[i - 1].y();
          output_pts.emplace_back(Eigen::Vector2f(x, y));
        }
      }
    }
    output_pts.emplace_back(origin_pts.back());
  }
}

bool IsBoundaryCrossLine(const std::vector<Eigen::Vector2f> &line, const std::vector<Eigen::Vector2f> &boundary) {
  if (boundary.empty() || line.size() < 2) {
    return false;
  } else {
    Eigen::Vector2f line_direct = (line.back() - line.front()).normalized();
    int             count1      = 0;
    int             count2      = 0;
    for (size_t i = 0; i < boundary.size(); ++i) {
      const Eigen::Vector2f boundary_direct_front = (boundary.at(i) - line.front()).normalized();
      const Eigen::Vector2f boundary_direct_back  = (boundary.at(i) - line.back()).normalized();
      if (std::acos(boundary_direct_front.dot(line_direct)) / M_PI * 180 < 90 &&
          std::acos(boundary_direct_back.dot(line_direct)) / M_PI * 180 > 90) {
        if (line_direct.x() * boundary_direct_front.y() > line_direct.y() * boundary_direct_front.x()) {
          ++count1;
        } else {
          ++count2;
        }
      }
      if (count1 > 0 && count2 > 0) {
        return true;
      }
    }
    return false;
  }
}

double GetThreePointAngleCos(const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2, const Eigen::Vector2f &pt3) {
  const Eigen::Vector2f vec1 = (pt1 - pt2).normalized();
  const Eigen::Vector2f vec2 = (pt3 - pt2).normalized();
  return vec1.dot(vec2);
}

double CalculateCurvatureFrom3Points(const Eigen::Vector2f &p0, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
  double x0 = p0.x();
  double y0 = p0.y();
  double x1 = p1.x();
  double y1 = p1.y();
  double x2 = p2.x();
  double y2 = p2.y();

  double a = 2 * (x1 - x0);
  double b = 2 * (y1 - y0);
  double d = 2 * (x2 - x1);
  double e = 2 * (y2 - y1);

  double c = x1 * x1 + y1 * y1 - x0 * x0 - y0 * y0;
  double f = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;

  double g = b * d - e * a;

  if (std::abs(g) <= 1e-6) {
    return 1e-4;
  }
  double x = (b * f - e * c) / g;
  double y = (d * c - a * f) / g;

  double radius    = hypot(x - x1, y - y1);
  double curvature = 1e-4;
  if (!std::isnan(radius) && !std::isinf(radius) /*&& radius > 5.0*/) {
    curvature = 1.0 / radius;
  }

  // sign
  double value = (x2 - x1) * (y0 - y1) - (x0 - x1) * (y2 - y1);
  return std::copysign(curvature, value);
}

bool SegmentIsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &p3, const Eigen::Vector2f &p4) {
  auto CrossProduct = [](const Eigen::Vector2f &vec1, const Eigen::Vector2f &vec2) {
    return vec1.x() * vec2.y() - vec1.y() * vec2.x();
  };

  if ((p1.x() > p2.x() ? p1.x() : p2.x()) < (p3.x() < p4.x() ? p3.x() : p4.x()) ||
      (p1.x() < p2.x() ? p1.x() : p2.x()) > (p3.x() > p4.x() ? p3.x() : p4.x()) ||
      (p1.y() > p2.y() ? p1.y() : p2.y()) < (p3.y() < p4.y() ? p3.y() : p4.y()) ||
      (p1.y() < p2.y() ? p1.y() : p2.y()) > (p3.y() > p4.y() ? p3.y() : p4.y())) {
    return false;
  }

  Eigen::Vector2f vec21 = p2 - p1;
  Eigen::Vector2f vec43 = p4 - p3;
  double          res1  = CrossProduct(vec21, (p4 - p1));
  double          res2  = CrossProduct(vec21, (p3 - p1));
  double          res3  = CrossProduct(vec43, (p2 - p3));
  double          res4  = CrossProduct(vec43, (p1 - p3));
  if (res1 * res2 <= 0.0 && res3 * res4 <= 0.0) {
    return true;
  }
  return false;
}

void IsIntersectForTwoLines(const Line &l1, const Line &l2, std::vector<int> &l1_intersect_idxs) {
  const auto &points1 = l1;
  const auto &points2 = l2;
  if (points1.size() < 2 || points2.size() < 2) {
    return;
  }
  l1_intersect_idxs.clear();
  for (size_t i = 0; i + 1 < points1.size(); ++i) {
    for (size_t j = 0; j + 1 < points2.size(); ++j) {
      bool res = SegmentIsIntersect(points1.at(i), points1.at(i + 1), points2.at(j), points2.at(j + 1));
      if (res) {
        l1_intersect_idxs.push_back(i);
      }
    }
  }
}

bool CalculateAverageDistanceBetweenTwoLines(std::vector<Eigen::Vector2f>& line_1, std::vector<Eigen::Vector2f>& line_2, std::pair<float, float> &distance) {
    uint32_t cnt = 0;
    distance = {0.0, 0.0};

    float mean_dist  = 0.0, max_dist = 0.0;
    if(line_1.empty() || line_2.empty())
    {
        return false;
    }

    for(auto& pt: line_1)
    {
        int outIndex = 0;
        double dist = 0.0;
        Eigen::Vector2f foot;
        bool flag = GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2f>(pt, line_2,  0, line_2.size()-1, false, foot, dist, outIndex, 0.3);
        if(!flag) continue;
        if(dist > max_dist)
        {
            max_dist = dist;
        }
        mean_dist += dist;
        cnt++;
    }
    if(cnt > line_1.size()){
        cnt = line_1.size();
    }

    if(cnt >= 5)
    {
        mean_dist /= cnt;
    }else{
        return false;
    }

    distance = {mean_dist, max_dist};
    return true;
}

// bool DrecretePointsOptWithQsqpFixStartAndEnd(const std::vector<Eigen::Vector2f>& origin_pts,
//                                              std::vector<Eigen::Vector2f>& output_pts) {
//   using OsqpSolver = static_opt::math::OsqpSolver;
//   using OSQPConfig = static_opt::math::OsqpConfig;
//   OSQPConfig osqp_config_;
//   if (origin_pts.empty()) {
//     // VLOG(0) << "curver fit empty points!";
//     return false;
//   }
//   output_pts.clear();
//   // set bound
//   const double bound = 1.0;
//   const auto& origin_pts_size = origin_pts.size();
//   std::vector<double> bounds(origin_pts_size, bound);
//   bounds.front() = 0;
//   bounds.back() = 0;
//
//   // copy origin_pts into ref_points
//   std::vector<std::pair<double, double>> ref_points;
//   ref_points.reserve(origin_pts_size);
//
//   for (size_t i = 0; i < origin_pts_size; i++) {
//     ref_points.push_back(std::make_pair(origin_pts.at(i).x(), origin_pts.at(i).y()));
//   }
//   double weight_fem_pos_deviation = osqp_config_.weight_fem_pos_deviation_bev;
//   double weight_path_length = osqp_config_.weight_path_length_bev;
//   double weight_ref_deviation = osqp_config_.weight_ref_deviation_bev;
//   OsqpSolver solver;
//   solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
//   solver.set_weight_path_length(weight_path_length);
//   solver.set_weight_ref_deviation(weight_ref_deviation);
//
//   solver.set_max_iter(osqp_config_.max_iter);
//   solver.set_time_limit(osqp_config_.time_limit);
//   solver.set_verbose(osqp_config_.verbose);
//   solver.set_scaled_termination(osqp_config_.scaled_termination);
//   solver.set_warm_start(osqp_config_.max_iter);
//
//   solver.set_ref_points(ref_points);
//   solver.set_bounds_around_refs(bounds);
//
//   if (!solver.Solve()) {
//     // VLOG(0) << "osqp solve failed!!!" << std::endl;
//     output_pts = origin_pts;
//     return false;
//   }
//
//   output_pts.reserve(solver.opt_x().size());
//
//   for (size_t i = 0; i < solver.opt_x().size(); ++i) {
//     output_pts.emplace_back(solver.opt_x().at(i), solver.opt_y().at(i));
//   }
//   return true;
// }
  double CalLinesSimilar(LaneGeometry::Line& l1, LaneGeometry::Line& l2)
  {
      if(l1.size() < 3 || l2.size() < 3) {
          return 99999;
      }
      double average_distance = 0;
      std::vector<Eigen::Vector2f> l1_sample_pt = {l1.front(), l1[l1.size()/2], l1.back()};
      for (size_t i = 0; i < l1_sample_pt.size(); ++i) {
            int index = -1;
            double dist = 9999.0;
            for (size_t j = 0; j < l2.size(); ++j) {
                double tmp = LaneGeometry::CalculateDistance(l1[i], l2[j]);
                if (tmp < dist) {
                    dist = tmp;
                    index = j;
                }
            }
            if (index >= 0 && index < (int) (l2.size())) {
                  if (index > 0) {
                      double dist1 = LaneGeometry::DistPointToLineSegment(l1[i], l2[index], l2[index - 1]);
                      dist = std::min(dist, dist1);
                  }
                  if (index < (int) (l2.size()) - 1) {
                      double dist2 = LaneGeometry::DistPointToLineSegment(l1[i], l2[index], l2[index + 1]);
                      dist = std::min(dist, dist2);
                  }
            }
            average_distance += dist;
        }
        return average_distance/3;               
  }

}  // namespace LaneGeometry
}  // namespace fusion
}  // namespace cem
