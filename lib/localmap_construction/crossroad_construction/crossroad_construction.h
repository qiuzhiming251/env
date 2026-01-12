/**
 * @file crossroad_construction.h
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

#ifndef CROSSROAD_CONSTRUCTION_H
#define CROSSROAD_CONSTRUCTION_H

#include <memory>

#include "common/utility.h"
#include "common/nlohmann/json.hpp"
#include "cross_toplane_processor.h"
#include "lib/message/env_model/routing_map/routing_map.h"
#include "road_connector.h"
#include "virtual_egoroad_processor.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"

namespace cem {
namespace fusion {

/// construct virtual lines to go through the cross section
class CrossRoadConstruction {
  const char* StateToString(CrossState cross_state);

  public:
  CrossRoadConstruction();
  ~CrossRoadConstruction();
  void Init();
  void ReInitCrossInfo();
  void SetGeometryMatchInfo(GeometryMatchInfo* geometry_match_info){
    geometry_match_info_ = geometry_match_info;
  }
  void SetEnvStatus(std::shared_ptr<byd::msg::orin::routing_map::EnvStatus> env_status){
    env_status_ = env_status;
  }
  bool IsCrossRoad();
  double CrossRoadDis();
  void Process(BevMapInfoPtr& bev_map, const Eigen::Isometry3d& Twb, 
                  BevMapInfoConstPtr &raw_bev_map, const SdJunctionInfoCityPtr 
                  &junctions_ptr,RoutingMapPtr routing_map_ptr,
                  bool on_highway);
  bool IsTurnRight() { return is_right_turn_only_; }

 private:
  bool DataUpdate(const BevMapInfoPtr &bev_map, const Eigen::Isometry3d &Twb, RoutingMapPtr routing_map_ptr,
                  BevMapInfoConstPtr &raw_bev_map, const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr, bool on_highway);
  void Reset();
  void InitAction();
  bool Crossdecision();
  bool IsLaneDeath(const BevLaneInfo& lane, float end_offset);
  void SetCrossroadConstructionDebugInfo(BevMapInfoPtr &GlobalBevMapOutPut);
  bool SelectBestTopLane( std::vector<TopLane>& candiates, const std::vector<TopLane>& ego_lanes,std::vector<byd::common::math::Vec2d> currentTrace,TopLane& ret);

  bool CrossStateToEnd();
  bool CrossStatePreToEnd();

  bool CrossStateConnctToEnd();

  void PredictionAction();

  void OutPutVirtualLane();

  void ConnectionAction();

  void EndAction();

  void UpdateEgoLane();

  void GenerateCrossVirtualLane();

  void RemoveVirtualOverLap();

  void IsRightTurnOnly(const SdJunctionInfoCityPtr &junctions_ptr);
  bool HasVirtualLane();
  void CutVirtualLane();

  RoutingMapPtr routing_map_raw_{nullptr};
  BevMapInfoConstPtr bev_map_raw_{nullptr};
  BevMapInfoPtr bev_map_{nullptr};
  LocalizationPtr currect_loc_{nullptr};
  std::shared_ptr<CrossDataManager> data_manager_{nullptr};
  std::shared_ptr<CrossToplaneProcessor> cross_toplane_processor_;
  std::shared_ptr<RoadConnector> road_connector_;
  std::shared_ptr<VirtualEgoRoadProcessor> virtual_egoroad_;
  CrossState cross_state_{CrossState::INIT};
  GeometryMatchInfo* geometry_match_info_{nullptr};
  uint32_t end_count_{0};
  float stop_line_dis_{0.0};
  bool is_right_turn_only_{false};
  std::string route_id_{0};
  uint64_t next_junction_id_{0};
  cem::fusion::navigation::JunctionTypeCity current_junction_type_{0};
  bool is_cross_road_out_{false};
  std::shared_ptr<byd::msg::orin::routing_map::EnvStatus> env_status_{};
  int last_dnp_status_{-1};
  bool noa2icc{false};
};

}  // namespace fusion
}  // namespace cem
#endif  // CROSSROAD_CONSTRUCTION_H