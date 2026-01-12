#ifndef VEHICLE_SIGNAL_H_
#define VEHICLE_SIGNAL_H_

#include "message/common/header.h"

namespace cem {
namespace message {
namespace sensor {

struct VehicleSignal
{
    cem::message::common::Header header;
    float steering_wheel_angle = 0.0f; // degree
    float dynamic_yaw_rate = 0.0f;     // rad·s-1
    uint32_t stand_still = 0;
    float average_speed = 0.0f; // m·s-1
    uint32_t left_direction_light = 0;
    uint32_t right_direction_light = 0;
    float lat_accel = 0.0f; // m·s-2
    float lon_accel = 0.0f; // m·s-2
    uint32_t gear_info = 0; // p:1 r:2 n:3 f:4~11 reserve:12~15
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
