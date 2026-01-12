#ifndef ROAD_EDGE_H_
#define ROAD_EDGE_H_

#include <vector>

#include "geometry_line.h"

namespace cem {
namespace message {
namespace env_model {

enum TrafficRoadEdgePosition
{
    TRE_REP__UNKNOWN = 0,
    TRE_REP__LEFT = 1,
    TRE_REP__RIGHT = 2,
    TRE_REP__OTHERS = 3
};

enum TrafficRoadEdgeType
{
    TRE_RET__UNKNOWN = 0,
    TRE_RET__FLAT = 1,
    TRE_RET__CURB = 2,
    TRE_RET__ELEVATED = 3,
    TRE_RET__CONE_AND_POLE = 4,
    TRE_RET__PARKED_CARS = 5
};

struct RoadEdgeGeoSeg
{
    uint32_t id = 0;
    uint32_t contributing_sensors = 0;
    uint32_t type = 0;
    float height_above_ground = 0.0f;
    bool is_start_out_of_view = false;
    bool is_end_out_of_view = false;
    GeometryLine geometry;
};

struct RoadEdge
{
    uint32_t id = 0;
    uint32_t position = 0;
    uint32_t linked_lane_segs_ids_num = 0;
    std::vector<uint32_t> linked_lane_segs_ids = {};
    uint32_t road_edge_geo_segs_num = 0;
    std::vector<RoadEdgeGeoSeg> road_edge_geo_segs = {};
    float start_offset = 0.0f;
    float end_offset = 0.0f;
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
