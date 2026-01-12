#pragma once

// #define PRINT_BEV_LDMAP_MATCHING_INFO

#include <stdint.h>

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include "lib/message/env_model/routing_map/routing_map.h"

namespace cem {
namespace fusion {

struct PairHash {
  template <typename T1, typename T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    return std::hash<T1>()(pair.first) ^ (std::hash<T2>()(pair.second) << 1);
  }
};

class RoadSlice {
 public:
  enum LanePosition {
    kUncertain = 0,
    kCloseLeftRoadEdge = 1,
    kCloseRightRoadEdge = 2,
    kUniqueLane = 3,
  };

 private:
  struct LaneLineType {
    cem::message::env_model::LineType left_line_type;
    cem::message::env_model::LineType right_line_type;
  };
  struct Cell {
    uint64_t lane_id;
    LaneLineType laneline_types;
    LanePosition lane_position;
    double probability;
    Eigen::Vector2d origin_point;
  };

 private:
  Eigen::Vector2d slice_origin_point_;
  Eigen::Vector2d slice_direction_;
  std::vector<std::unique_ptr<Cell>> cells_;
  std::unordered_map<uint64_t, Eigen::Vector2d> origin_points_;
  int left_first_cell_id_;
  int left_last_cell_id_;
  int right_last_cell_id_;
  int shift_step_;
  bool left_road_edge_feature_involved_;
  bool right_road_edge_feature_involved_;

 public:
  RoadSlice(Eigen::Vector2d& slice_origin_point,
            Eigen::Vector2d& slice_direction);
  ~RoadSlice();
  // void FilterLanePosition();
  bool LeftShift();
  bool RightShift();
  void ResetShift();
  void StablizeShift();
  void SetShiftStep(int step);
  const int GetShiftStep() const { return shift_step_; };
  const std::vector<std::unique_ptr<Cell>>& GetCells() const { return cells_; };
  void FillCell(Eigen::Vector2d& point, uint64_t lane_id,
                cem::message::env_model::LineType left_line_type,
                cem::message::env_model::LineType right_line_type,
                LanePosition lane_position);
  bool ClearIsolatedPoint();
  double operator*(const RoadSlice& other);
  std::unordered_map<std::pair<uint64_t, uint64_t>, double, PairHash> operator&(
      const RoadSlice& other);
  std::string GenerateSlicePrintInfo() const;
  bool IsLaneAatTheLeftofLaneB(uint64_t a, uint64_t b);
  bool IsLaneAatTheRightofLaneB(uint64_t a, uint64_t b);
  inline bool GetPoint(uint64_t lane_id, Eigen::Vector2d& point) {
    if (origin_points_.count(lane_id) == 0) {
      return false;
    } else {
      point = origin_points_.at(lane_id);
      return true;
    }
  }
  inline std::unordered_set<uint64_t> GetLanes() {
    std::unordered_set<uint64_t> lanes;
    for (auto point : origin_points_) {
      lanes.emplace(point.first);
    }
    return lanes;
  };
  inline const int GetLaneSize() const { return origin_points_.size(); };
  bool HasRoadedgeFeature();

 private:
  static constexpr double SLICE_DEFAULT_LENGTH = 64.0;
  static constexpr double CELL_WIDTH = 0.3;
  static constexpr double SHIFT_LIMIT = 1.0;

  static constexpr double MEAN_NORMAL_DISTRIBUTE = 0.0;
  static constexpr double STDDEV_NORMAL_DISTRIBUTE = CELL_WIDTH;
  static constexpr double PROBILITY_RESOLUSION =
      0.1 * RoadSlice::STDDEV_NORMAL_DISTRIBUTE;

  static constexpr double N_TIMES = 3.0;
  static constexpr double SIGMA_N_TIMES = N_TIMES * STDDEV_NORMAL_DISTRIBUTE;
  static constexpr int CELL_SEARCH_RADIUS = 5;

  static bool is_gen_normal_distribute_;
  static std::vector<double> normal_distribute_probability_;

 public:
  static void GenNormalDistribute();
  static int GetProbilityLookUpTabelIndex(double mean, double value);
};

}  // namespace fusion
}  // namespace cem
