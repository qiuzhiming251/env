/**
 * @file crossroad_lane_selector.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief
 * @version 1.0
 * @date 2025-09-18
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
#ifndef CROSSROAD_SELECTOR_H
#define CROSSROAD_SELECTOR_H

#include "cross_data_manager.h"

namespace cem {
namespace fusion {

///
class CrossRoadSelector {
 public:
  CrossRoadSelector();
  void Init(const std::shared_ptr<CrossDataManager> &data_manager) { data_manager_ = data_manager; }
  bool Process(const cem::message::env_model::LaneInfo *lane, std::vector<Point2DF> &ld_lane_points);

 private:
  bool FindRoutingMapVirtualLane(const cem::message::env_model::LaneInfo *lane, uint64_t &egolane_id);
  void AddLaneLength(uint64_t next_id,double max_add_length, std::vector<Point2DF> &ld_lane_points);

  std::shared_ptr<CrossDataManager> data_manager_{nullptr};
};

}  // namespace fusion
}  // namespace cem
#endif  // CROSSROAD_SELECTOR_H
