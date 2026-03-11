#ifndef TOP_STATE_H_
#define TOP_STATE_H_

#include <cstdint>
#include <vector>

#include "message/common/header.h"
#include "message/common/geometry.h"

namespace cem {
namespace message {
namespace sensor {

enum class OddType {
  CITYWAY_MODE = 0,
  HIGHWAY_MODE = 1,
};

enum class StateType {
  DEFAULT      = 0,
  ADAS         = 1,
  OTHER        = 2,
};

struct TopState {
  cem::message::common::Header header;
  OddType odd_type = OddType::CITYWAY_MODE;
  StateType state_type = StateType::DEFAULT;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif