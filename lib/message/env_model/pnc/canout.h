#ifndef CANOUT_H
#define CANOUT_H

#include <vector>

#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {
struct CAN1
{
    cem::message::common::Header header;
    uint32_t DNP_Stats_S = 0;

};
} // namespace env_model
} // namespace message
} // namespace cem

#endif