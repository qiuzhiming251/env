#ifndef TWIST_H_
#define TWIST_H_

#include <vector>

#include "message/common/geometry.h"

namespace cem {
namespace message {
namespace sensor {

struct Twist
{
    cem::message::common::Vector3DF linear;
    std::vector<float> linear_covariance = {};
    cem::message::common::Vector3DF angular;
    std::vector<float> angular_covariance = {};
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
