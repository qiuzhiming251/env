/**
 * @file road_connector.h
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
#ifndef ROAD_CONNECTOR_H
#define ROAD_CONNECTOR_H

#include "cross_data_manager.h"

namespace cem {
namespace fusion {

///
class RoadConnector {
 public:
  RoadConnector();
  void Init(const std::shared_ptr<CrossDataManager>& data_manager);
  void Reset();
  bool GenerateConnectLane();
  bool GenerateConnectLane(const BevLaneInfo &his_lane);
  bool UpdateConnectLane();
  bool UpdateMainLane();
  void SetDrGeoLane(const BevLaneInfo &dr_ego_lane) { dr_ego_lane_ = dr_ego_lane; }

  BevLaneInfo& connect_lane() { return connect_lane_; }
  void  SetConnectLane(const BevLaneInfo& connect_lane ) {  connect_lane_ = connect_lane; }
  BevLaneMarker& connect_left_lanemarker() { return connect_left_lanemarker_; }
  BevLaneMarker& connect_right_lanemarker() { return connect_right_lanemarker_; }

 private:
  void NewMainLane(double c0, double c1, const std::vector<cem::message::common::Point2DF>& ego_points,
                   double half_width, uint64_t lane_id, double control_dis = 15.0);
  void UpdateConnectMainLane(const cem::message::sensor::BevLaneInfo& lane);
  bool FitterEgelane(Curve &ego_curve);
  cem::message::sensor::BevLaneInfo *FindBevLaneById(uint64_t id);
  std::shared_ptr<CrossDataManager> data_manager_{nullptr};
  BevLaneInfo dr_ego_lane_;
  BevLaneInfo connect_lane_;
  BevLaneMarker connect_left_lanemarker_;
  BevLaneMarker connect_right_lanemarker_;
};
}  // namespace fusion
}  // namespace cem
#endif  // ROAD_CONNECTOR_H