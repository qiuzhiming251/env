#include "GeometryLaneMatcher.h"

namespace cem {
namespace fusion {
namespace navigation {

static void DebugStats(const std::vector<Eigen::Vector2d> &pts, std::string_view name) {
  if (pts.empty())
    return;

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double sum_x = 0.0, sum_y = 0.0;

  for (const auto &p : pts) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
    sum_x += p.x();
    sum_y += p.y();
  }
  const double mean_x = sum_x / pts.size();
  const double mean_y = sum_y / pts.size();

  SD_MERGE_LOG << fmt::format(
      "[{}] bbox_x=[{:.2f},{:.2f}] bbox_y=[{:.2f},{:.2f}] mean=({:.2f},{:.2f}) first=({:.2f},{:.2f}) last=({:.2f},{:.2f})", name, min_x,
      max_x, min_y, max_y, mean_x, mean_y, pts.front().x(), pts.front().y(), pts.back().x(), pts.back().y());
}

static void DebugPointsXY(const std::vector<Eigen::Vector2d> &pts, std::string_view name, size_t max_print = 10, size_t step = 1) {
  if (pts.empty())
    return;

  fmt::memory_buffer info_x, info_y;
  fmt::format_to(info_x, "Size:{} {}_x += [", pts.size(), name);
  fmt::format_to(info_y, "Size:{} {}_y += [", pts.size(), name);

  size_t printed = 0;
  for (size_t i = 0; i < pts.size() && printed < max_print; i += step, ++printed) {
    const auto &p = pts[i];
    fmt::format_to(info_x, "{:5.1f}{}", p.x(), (printed + step < std::min(pts.size(), max_print * step) ? "," : ""));
    fmt::format_to(info_y, "{:5.1f}{}", p.y(), (printed + step < std::min(pts.size(), max_print * step) ? "," : ""));
  }
  fmt::format_to(info_x, "];");
  fmt::format_to(info_y, "];");

  SD_MERGE_LOG << std::string_view(info_x.data(), info_x.size());
  SD_MERGE_LOG << std::string_view(info_y.data(), info_y.size());
}

MatchStats GeometryLaneMatcher::CalcMeanLateral(const std::vector<Eigen::Vector2d> &bev_pts, const std::vector<Eigen::Vector2d> &ld_pts,
                                                double search_eps, bool best_drop) {
  MatchStats st;
  if (bev_pts.size() < 3 || ld_pts.size() < 3) {
    SD_MERGE_LOG << fmt::format("CalcMeanLateral: bev_size={} ld_size={} → not enough points", bev_pts.size(), ld_pts.size());
    return st;
  }

  //   DebugStats(bev_pts, "BEV");
  //   DebugStats(ld_pts, "LD ");
  //   DebugPointsXY(bev_pts, "BEV", 10, /*step=*/std::max<size_t>(1, bev_pts.size() / 80));
  //   DebugPointsXY(ld_pts, "LD ", 10, /*step=*/std::max<size_t>(1, ld_pts.size() / 80));

  double sum   = 0.0;
  double maxv  = 0.0;
  int    count = 0;

  Eigen::Vector2d foot;
  double          dist    = 0.0;
  int             out_idx = 0;

  const int ib = 0;
  const int ie = static_cast<int>(ld_pts.size()) - 1;

  for (const auto &p : bev_pts) {
    bool ok =
        cem::fusion::GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2d>(p, ld_pts, ib, ie, best_drop, foot, dist, out_idx, search_eps);
    if (ok) {
      sum += std::abs(dist);
      maxv = std::max(maxv, std::abs(dist));
      count += 1;
    }
  }

  st.inlier_cnt   = count;
  st.max_lat      = maxv;
  st.mean_lat     = (count > 0) ? (sum / static_cast<double>(count)) : std::numeric_limits<double>::infinity();
  st.inlier_ratio = (bev_pts.empty() ? 0.0 : (static_cast<double>(count) / static_cast<double>(bev_pts.size())));
  SD_MERGE_LOG << fmt::format("CalcMeanLateral: inlier_cnt={} inlier_ratio={:.2f} mean_lat={:.3f} max_lat={:.3f}", st.inlier_cnt,
                              st.inlier_ratio, st.mean_lat, st.max_lat);
  return st;
}

bool GeometryLaneMatcher::BestMatchBevByGeometry(const std::vector<uint64_t> &raw_bev_lane_ids, const BevGetter &bev_points_getter,
                                                 const std::vector<Eigen::Vector2d> &ld_target_pts, double search_eps,
                                                 double max_mean_thresh, double min_inlier_ratio, uint64_t &best_bev_id_out,
                                                 double &best_score_out, MatchStats *best_stats_out) {
  best_bev_id_out = 0;
  best_score_out  = std::numeric_limits<double>::infinity();
  MatchStats best_stats;

  for (auto bev_id : raw_bev_lane_ids) {
    std::vector<Eigen::Vector2d> bev_pts;
    if (!bev_points_getter || !bev_points_getter(bev_id, bev_pts) || bev_pts.size() < 3)
      continue;

    auto st = CalcMeanLateral(bev_pts, ld_target_pts, search_eps, false);
    if (st.inlier_cnt <= 0)
      continue;
    if (st.inlier_ratio < min_inlier_ratio || st.inlier_cnt < 10)
      continue;
    if (st.mean_lat > max_mean_thresh)
      continue;

    if (st.mean_lat < best_score_out) {
      best_score_out  = st.mean_lat;
      best_bev_id_out = bev_id;
      best_stats      = st;
    }
  }

  if (best_stats_out)
    *best_stats_out = best_stats;
  return (best_bev_id_out != 0);
}

}  // namespace navigation
}  // namespace fusion
}  // namespace cem
