#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/utility.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/road_slice.h"

namespace cem {
namespace fusion {

template <typename DataType>
class FilletingMachine {
 private:
  double slice_thickness_;
  Eigen::Vector2d slicing_knife_direction_;
  Eigen::Vector2d slice_start_point_;
  double slice_total_length_;

 protected:
  std::shared_ptr<const DataType> raw_data_ = nullptr;
  std::unordered_map<int, std::unique_ptr<RoadSlice>> road_slices_;

 public:
  FilletingMachine(double slice_thickness = 1.0);
  ~FilletingMachine();
  void SetSliceKnifeDirection(Eigen::Vector2d& direction);
  void SetSliceStartPoint(Eigen::Vector2d& start_point);
  void SetSliceTotalLength(double length);
  void SetData(std::shared_ptr<const DataType> input_data);
  const Eigen::Vector2d& GetSliceKnifeDirection();
  const Eigen::Vector2d& GetSliceStartPoint();
  const double& GetSliceThickness();
  const double& GetSliceTotalLength();
  std::unordered_map<int, std::unique_ptr<RoadSlice>>& GetSlices();

  virtual void Process();

  template <typename PointType>
  inline std::pair<Eigen::Vector2d, Eigen::Vector2d> FindNearestPts(
      const std::vector<PointType>& lane_pts, const Eigen::Vector2d& D1,
      const Eigen::Vector2d& P1, bool check_point_distance = false);

  Eigen::Vector2d FindIntersection(const Eigen::Vector2d& P1,
                                   const Eigen::Vector2d& P2,
                                   const Eigen::Vector2d& S,
                                   const Eigen::Vector2d& D);
  template<typename PointType>
  inline double DistanceToLine(const Eigen::Vector2d& D1, const Eigen::Vector2d& P1,
                        const PointType& pt);
  template<typename PointType>
  inline double DistanceToLineWithoutNorm(const Eigen::Vector2d& D1, const Eigen::Vector2d& P1,
                                          const PointType& pt);

  template<typename PointType>
  int PointPosition(const Eigen::Vector2d& D1, const Eigen::Vector2d& P1,
                    const PointType& pt);

  inline bool GetSamplingPoint(uint64_t lane_id, uint32_t slice_id,
                               Eigen::Vector2d& point) {
    if (road_slices_.count(slice_id) == 0) {
      return false;
    } else {
      auto& slice = road_slices_.at(slice_id);
      if (slice->GetPoint(lane_id, point)) {
        return true;
      } else {
        return false;
      }
    }
  };

 private:
  virtual void Slice() = 0;
  std::vector<double> lane_pts_distance_;
};

template <typename DataType>
FilletingMachine<DataType>::FilletingMachine(double slice_thickness)
    : slice_thickness_(slice_thickness) {
  RoadSlice::GenNormalDistribute();
  lane_pts_distance_.resize(10000);
}

template <typename DataType>
FilletingMachine<DataType>::~FilletingMachine() {}

template <typename DataType>
void FilletingMachine<DataType>::SetSliceKnifeDirection(
    Eigen::Vector2d& direction) {
  slicing_knife_direction_ = direction;
  slicing_knife_direction_.normalize();
}

template <typename DataType>
void FilletingMachine<DataType>::SetSliceStartPoint(
    Eigen::Vector2d& start_point) {
  slice_start_point_ = start_point;
}

template <typename DataType>
void FilletingMachine<DataType>::SetSliceTotalLength(double length) {
  slice_total_length_ = length;
}

template <typename DataType>
void FilletingMachine<DataType>::SetData(
    std::shared_ptr<const DataType> input_data) {
  raw_data_ = input_data;
}

template <typename DataType>
const Eigen::Vector2d& FilletingMachine<DataType>::GetSliceKnifeDirection() {
  return slicing_knife_direction_;
}

template <typename DataType>
const Eigen::Vector2d& FilletingMachine<DataType>::GetSliceStartPoint() {
  return slice_start_point_;
}

template <typename DataType>
const double& FilletingMachine<DataType>::GetSliceThickness() {
  return slice_thickness_;
}

template <typename DataType>
const double& FilletingMachine<DataType>::GetSliceTotalLength() {
  return slice_total_length_;
}

template <typename DataType>
std::unordered_map<int, std::unique_ptr<RoadSlice>>&
FilletingMachine<DataType>::GetSlices() {
  return road_slices_;
}

template <typename DataType>
void FilletingMachine<DataType>::Process() {
  road_slices_.clear();
  Slice();
}

template <typename DataType>
template <typename PointType>
inline std::pair<Eigen::Vector2d, Eigen::Vector2d>
FilletingMachine<DataType>::FindNearestPts(
    const std::vector<PointType>& lane_pts, const Eigen::Vector2d& D1,
    const Eigen::Vector2d& P1, bool check_point_distance) {
  double first_min_distance = std::numeric_limits<double>::infinity();
  double second_min_distance = std::numeric_limits<double>::infinity();
  int first_index = -1, second_index = -1;
  int pts_num = lane_pts.size();
  if (pts_num > lane_pts_distance_.size()) {
    lane_pts_distance_.resize(pts_num);
  }
  for (int i = 0; i < pts_num; i++) {
    lane_pts_distance_[i] = 
        this->DistanceToLineWithoutNorm(D1, P1, lane_pts[i]);
    if (lane_pts_distance_[i] < first_min_distance) {
      second_min_distance = first_min_distance;
      first_min_distance = lane_pts_distance_[i];
      second_index = first_index;
      first_index = i;
    } else if (lane_pts_distance_[i] < second_min_distance) {
      second_min_distance = lane_pts_distance_[i];
      second_index = i;
    }
  }

  // fine two side points
  int side1 = this->PointPosition(D1, P1, lane_pts[first_index]);
  if (side1 == this->PointPosition(D1, P1, lane_pts[second_index])) {
    second_min_distance = std::numeric_limits<double>::infinity();
    second_index = -1;
    for (int i = 0; i < pts_num; i++) {
      if (i != first_index) {
        if (lane_pts_distance_[i] < second_min_distance) {
          if (this->PointPosition(D1, P1, lane_pts[i]) !=
              side1) {
            second_min_distance = lane_pts_distance_[i];
            second_index = i;
          }
        }
      }
    }
    if (second_index == -1) {
      return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }
  }

  if (check_point_distance) {
    if ((this->DistanceToLine(D1, P1, lane_pts[first_index]) > 5) ||
        (this->DistanceToLine(D1, P1, lane_pts[second_index]) > 5)) {
      return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
    }
  }

  return std::make_pair(Eigen::Vector2d(lane_pts[first_index].x,
                                        lane_pts[first_index].y),
                        Eigen::Vector2d(lane_pts[second_index].x,
                                        lane_pts[second_index].y));
}

template <typename DataType>
Eigen::Vector2d FilletingMachine<DataType>::FindIntersection(
    const Eigen::Vector2d& P1, const Eigen::Vector2d& P2,
    const Eigen::Vector2d& S, const Eigen::Vector2d& D) {
  Eigen::Vector2d P1P2 = P2 - P1;
  // calculate coefficient matrix
  Eigen::Matrix2d A;
  A << P1P2(0), -D(0), P1P2(1), -D(1);
  // calculate constant matrix
  Eigen::Vector2d b;
  b << S(0) - P1(0), S(1) - P1(1);

  Eigen::Vector2d solution = A.colPivHouseholderQr().solve(b);
  // check if valid
  if (A.determinant() != 0) {
    double t = solution(0);
    double s = solution(1);
    Eigen::Vector2d intersection = P1 + t * P1P2;
    return intersection;
  } else {
    // line parallel or overlap
    return Eigen::Vector2d::Zero();
  }
}

// distance of point to line
template <typename DataType>
template <typename PointType>
inline double FilletingMachine<DataType>::DistanceToLineWithoutNorm(const Eigen::Vector2d& D1,
                                                  const Eigen::Vector2d& P1,
                                                  const PointType& pt) {
  return std::abs((pt.x - P1.x()) * D1.y() - (pt.y - P1.y()) * D1.x());
}
// distance of point to line
template <typename DataType>
template <typename PointType>
inline double FilletingMachine<DataType>::DistanceToLine(const Eigen::Vector2d& D1,
                                                  const Eigen::Vector2d& P1,
                                                  const PointType& pt) {
  return std::abs((pt.x - P1.x()) * D1.y() - (pt.y - P1.y()) * D1.x()) / D1.norm();
}

// posiion of point and line
template <typename DataType>
template <typename PointType>
int FilletingMachine<DataType>::PointPosition(const Eigen::Vector2d& D1,
                                              const Eigen::Vector2d& P1,
                                              const PointType& pt) {
  double cross_product =
      (pt.x - P1.x()) * D1.y() - (pt.y - P1.y()) * D1.x();
  return cross_product > 0 ? 1 : (cross_product < 0 ? -1 : 0);
}

}  // namespace fusion
}  // namespace cem
