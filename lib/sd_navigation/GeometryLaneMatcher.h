#ifndef GEOMETRYLANEMATCHER_H
#define GEOMETRYLANEMATCHER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "Eigen/src/Core/Matrix.h"

#include "lib/common/log_custom.h"
#include "lib/common/utils/GeoMathUtil.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "lib/message/sensor/camera/bev_lane/bev_lane.h"
#include "modules/perception/env/src/lib/common/utility.h"

namespace cem {
namespace fusion {
namespace navigation {

struct MatchStats {
  int    inlier_cnt   = 0;
  double mean_lat     = 0.0;
  double max_lat      = 0.0;
  double inlier_ratio = 0.0;
};

class GeometryLaneMatcher {
 public:
  static MatchStats CalcMeanLateral(const std::vector<Eigen::Vector2d> &bev_pts, const std::vector<Eigen::Vector2d> &ld_pts,
                                    double search_eps, bool best_drop = false);

  using BevGetter = std::function<bool(uint64_t, std::vector<Eigen::Vector2d> &)>;

  static bool BestMatchBevByGeometry(const std::vector<uint64_t> &raw_bev_lane_ids, const BevGetter &bev_points_getter,
                                     const std::vector<Eigen::Vector2d> &ld_target_pts, double search_eps, double max_mean_thresh,
                                     double min_inlier_ratio, uint64_t &best_bev_id_out, double &best_score_out,
                                     MatchStats *best_stats_out = nullptr);
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // GEOMETRYLANEMATCHER_H
