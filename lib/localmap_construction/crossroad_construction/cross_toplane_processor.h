/**
 * @file cross_toplane_processor.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 2.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 *
 */
#ifndef CROSSTOPLANE_PROCESSOR_H
#define CROSSTOPLANE_PROCESSOR_H

#include "cross_data_manager.h"
#include "lib/message/internal_message.h"

namespace cem {
namespace fusion {

///
class CrossToplaneProcessor {
 public:

  CrossToplaneProcessor();
  void DataUpdate();
  void Init(const std::shared_ptr<CrossDataManager>& data_manager);
  void Reset();
  /**
   * @brief   Get The CrossSectionId
   * 主要用来求取路口后的车道的大概方向，用于区分左右转和直行的车道线的处理
   *
   * @author fanminglei (fan.minglei@byd.com)
   * @date 2025-04-10
   */
  void GetCrossSectionId();

  void Preprocessing();

  bool FindClosetLane();
  bool FindClosetLaneByLatErr(const std::vector<cem::message::common::Point2DF>& points);
  bool FindClosetLane(const BevLaneInfo &virtual_lane);

  uint64_t ConnectLaneId() { return connect_lane_id_; }
  bool UpdateClosetLane();
  bool UpdateClosetLane(const BevLaneInfo &connect_lane);

  int his_connect_top_lane_index() { return his_connect_top_lane_index_; }

 private:
  bool GetPointsHeading(const std::vector<cem::fusion::Point2DF>& points, double& line_heading,
                        double& average_point_err, double& interior_points_percent);
  double GetPointsY(const std::vector<cem::fusion::Point2DF>& points,double input_x,double heading);
  void GenerateTopLanePoints(const byd::common::math::Vec2d& point, double heading, double half_width,
                             bool is_connect_lane, uint64_t lane_id = 0);

  void CalculateRoadHeading();

  bool GetAverageHeading(const std::vector<std::tuple<double, double, double>>& heading_infos, double& average_heading);
  bool FindClosetLaneByTopo(const BevLaneInfo &lane_input);
  byd::common::math::Vec2d TransformPointVec(const byd::common::math::Vec2d &point, const Eigen::Isometry3d &rotate_translate_matrix);
  Opening opening_;
  std::shared_ptr<CrossDataManager> data_manager_{nullptr};
  FirstOrderLowerPassFilter average_heading_filter_;
  uint64_t cross_section_id_{0};
  std::vector<uint64_t> top_lane_ids_;
  std::map<uint32_t, double> lane_id_dis_;
  std::vector<uint64_t> top_edge_ids_;
  uint64_t connect_lane_id_{0};
  std::vector<cem::fusion::Point2DF> connect_lane_points_;
  double his_average_heading_{0.0};
  TopLane his_connect_top_lane_;
  int his_connect_top_lane_index_{0};
};

}  // namespace fusion
}  // namespace cem
#endif  // CROSSTOPLANE_PROCESSOR_H