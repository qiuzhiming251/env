#ifndef TRAFFIC_LANE_H_
#define TRAFFIC_LANE_H_

#include <vector>

#include "message/common/header.h"
#include "lane.h"

namespace cem {
namespace message {
namespace env_model {

struct TrafficLane
{
    cem::message::common::Header header;
    uint32_t lanes_num = 0;
    std::vector<Lane> lanes = {};
    uint32_t desired_lane_id = 0;
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
