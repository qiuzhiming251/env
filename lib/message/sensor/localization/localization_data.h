#ifndef LOCALIZATION_DATA_H_
#define LOCALIZATION_DATA_H_

#include <vector>

#include "message/common/header.h"
#include "message/common/geometry.h"
#include "twist.h"

namespace cem {
namespace message {
namespace sensor {

struct LocalizationData
{
    cem::message::common::Header header;
    uint32_t milisecond = 0;
    std::vector<char> utc_str = {};
    double unix_time = 0.0d;
    std::vector<double> pos_ll = {};
    double height = 0.0f;
    std::vector<double> posne = {};
    std::vector<double> posne_dr = {};
    double attitude_dr = 0.0f;
    std::vector<double> velocity = {};
    std::vector<double> attitude = {};
    std::vector<double> groysocpe = {};
    std::vector<double> accelerate = {};
    uint32_t status = 0;
    uint32_t checksum = 0;
    cem::message::common::QuaternionF orientation;
    cem::message::sensor::Twist twist;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
