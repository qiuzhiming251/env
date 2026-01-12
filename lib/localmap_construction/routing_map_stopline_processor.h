/**
 * @file routing_map_stopline_processor.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 1.0
 * @date 2025-10-22
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

#ifndef ROUTINGmAP_STOPLINE_PROCESSOR_H
#define ROUTINGmAP_STOPLINE_PROCESSOR_H

#include "lib/message/internal_message.h"

#include "lib/perception_and_ld_map_fusion/data_fusion/geometry_match_info.h"

namespace cem {
namespace fusion {
class RoutingMapStopLineProcessor {
 public:
  RoutingMapStopLineProcessor();
  ~RoutingMapStopLineProcessor();
  void Init();
  void Reset();
  void Process(BevMapInfoPtr &bev_map_raw, const Eigen::Isometry3d &Twb, RoutingMapPtr routing_map_ptr,
               const GeometryMatchInfo &geometry_match_info);

 private:
  bool FindBevStopline(BevMapInfoPtr &bev_map_raw);
  bool FindTrafficLightCrossroadLane(RoutingMapPtr routing_map_ptr,const GeometryMatchInfo& geometry_match_info);
  bool AdjustLDLane(RoutingMapPtr routing_map_ptr);
  void ResampleLDEgoLanePoints();
  // 自车到dr转换
  Eigen::Isometry3d Twb_;
  // dr到自车转换
  Eigen::Isometry3d Tbw_;
  // 当前帧存储的自车在ld地图下的lane的指针
  cem::message::env_model::LaneInfo *            ld_ego_lane_{nullptr};
  // 存储的历史记忆的需要修改的egolane和cross_lane实体
  cem::message::env_model::LaneInfo              ego_lane_;
  std::vector<cem::message::env_model::LaneInfo> cross_lanes_;
  cem::message::env_model::Point                 dr_cut_point_;
  std::unordered_map<uint64_t, cem::message::env_model::LaneInfo *> routing_lane_unmap_;
  float bev_stopline_dis_{100.0f};
  uint64_t not_adjust_lane_id_{0};
};
}  // namespace fusion
}  // namespace cem
#endif  // ROUTINGmAP_STOPLINE_PROCESSOR_H