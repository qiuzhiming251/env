#ifndef TRAFFIC_ROAD_EDGE_H_
#define TRAFFIC_ROAD_EDGE_H_

#include <vector>

#include "message/common/header.h"
#include "message/common/cem_header.h"
#include "road_edge.h"

namespace cem {
namespace message {
namespace env_model {

struct TrafficRoadEdge
{
    cem::message::common::Header header;
    cem::message::common::CemHeader cem_header;
    uint32_t number_of_road_edge = 0;
    std::vector<RoadEdge> road_edges_list = {};                  // perception
    std::vector<RoadEdge> hdm_road_edges = {};                   // hdmap
    std::vector<RoadEdge> perception_hdm_fusion_road_edges = {}; // fusion
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
