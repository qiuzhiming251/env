#pragma once

#include "lib/message/env_model/routing_map/routing_map.h"
namespace cem {
namespace fusion {

inline bool IsLdMapLaneValid(const cem::message::env_model::LaneInfo &lane) {
  if ((lane.split_topology !=
        cem::message::env_model::SplitTopology::TOPOLOGY_SPLIT_NONE) ||
      (lane.merge_topology !=
        cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_NONE) ||
      (lane.junction_id != 0)) {
    return false;
  }
  return true;
}

} // namespace fusion
} // namespace cem