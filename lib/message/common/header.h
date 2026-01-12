#ifndef HEADER_H_
#define HEADER_H_

#include <stdint.h>

namespace cem {
namespace message {
namespace common {

struct Header
{
    double timestamp = 0.0d;      // s, timestamp of raw message
    double recv_timestamp = 0.0d; // s, timestamp when message was received
    uint32_t sequence_num = 0;
    uint32_t cycle_counter = 0;
    uint32_t status = 0;
};

} // namespace common
} // namespace message
} // namespace cem

#endif
