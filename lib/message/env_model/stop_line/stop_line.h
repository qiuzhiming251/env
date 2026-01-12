#ifndef STOP_LINE_H_
#define STOP_LINE_H_

#include <vector>
#include "message/common/cem_header.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"


namespace cem {
namespace message {
namespace env_model {

struct StopLine
{
    uint32_t id = 0;
    uint8_t number_of_points = 0;
    uint8_t position = 0;
    uint8_t type = 0;
    uint8_t color = 0;
    float conf = 0.0f;
    bool is_virtual = false;
    bool is_associated_bev = false;
    std::vector<cem::message::common::Point2DF> ego_line_points = {};
    std::vector<cem::message::common::Point2DF> dr_line_points = {};
    double distance_to_stopline{std::numeric_limits<double>::lowest()};
};
  


} // namespace env_model
} // namespace message
} // namespace cem

#endif