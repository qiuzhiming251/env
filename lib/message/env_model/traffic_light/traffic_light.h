#ifndef TRAFFIC_LIGHT_H_
#define TRAFFIC_LIGHT_H_

#include <vector>

#include "message/common/header.h"
#include "message/common/cem_header.h"

namespace cem {
namespace message {
namespace env_model {

enum TlaColor { //红绿灯颜色
  COLOR_NO_DISPLAY = 0,
  GREEN = 1,
  GREEN_TO_YELLOW = 2,
  YELLOW = 3,
  YELLOW_TO_RED = 4,
  RED = 5,
  RED_TO_GREEN = 6
};

enum TlaType { //红绿灯类型
  TYPE_NO_DISPLAY = 0,
  ROUND = 1,
  ARROW = 2,
  TURN_AROUND = 3,
  RESERVED = 4
};

struct TrafficLight 
{
    uint32_t tla_id = 0;
    double tla_distance_x = 0.0;
    double tla_distance_y = 0.0;
    double tla_distance_z = 0.0;
    double tla_position_confidence = 0.0;
    uint32_t left_tla_color = 0;
    uint32_t left_tla_type = 0;
    uint32_t straight_tla_color = 0;
    uint32_t straight_tla_type = 0;
    uint32_t right_tla_color = 0;
    uint32_t right_tla_type = 0;
    uint32_t new_left_tla_second = 0;
    uint32_t new_straight_tla_second = 0;
    uint32_t new_right_tla_Second = 0;
    double tla_reserved_1 = 0.0;
    double tla_reserved_2 = 0.0;
    double tla_reserved_3 = 0.0;
    double tla_reserved_4 = 0.0;	
    double tla_reserved_5 = 0.0;
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
