#ifndef DRIVERS_EVENT_
#define DRIVERS_EVENT_

#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

struct DriversEvent
{
    cem::message::common::Header header;
    uint64_t function_support_status = 0; //bit42=1表示相机故障(0x40000000000)
};

} // namespace env_model
} // namespace message
} // namespace cem

#endif
