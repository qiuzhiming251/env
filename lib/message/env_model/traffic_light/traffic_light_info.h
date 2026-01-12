#ifndef TRAFFIC_LIGHT_INFO_H_
#define TRAFFIC_LIGHT_INFO_H_

#include <vector>

#include "message/common/header.h"
#include "message/common/cem_header.h"
#include "traffic_light.h"

namespace cem {
namespace message {
namespace env_model {

struct TrafficLightInfo
{
    cem::message::common::Header header;
    cem::message::common::CemHeader cem_header;
    uint32_t tla_objs_num = 0;
    std::vector<TrafficLight> tla_objs = {};
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
