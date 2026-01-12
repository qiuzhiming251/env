#ifndef SPEED_LIMIT_H_
#define SPEED_LIMIT_H_

#include <vector>
#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

enum NormalSpeedStatus{
  NORMAL = 0x0,
  UNKNOWN_LIMIT_SPEED = 0x1,
  UNLIMITED = 0x2,
  SPL_CANCELLED = 0x3,
  NO_DISPLAY = 0x4
};

enum SpeedLimitMode{
  passive_mode = 0x0,
  fusion_mode = 0x1,
  map_mode = 0x2,
  vision_mode = 0x3,
  hdmap_mode = 0x4
};

struct ConditionLimit {
  float condition_limit_speed_value = 0;
  uint32_t condition_type = 0;
  double condition_distance = 0;
};

struct FusionSpeedInfo {
  float speed_value = 0.0;
  NormalSpeedStatus normal_speed_status = NORMAL;
  SpeedLimitMode speed_limit_mode = passive_mode;
  std::vector<ConditionLimit> condition_limits = {};
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif