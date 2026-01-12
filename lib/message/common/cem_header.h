#ifndef CEM_HEADER_H_
#define CEM_HEADER_H_

#include <vector>

namespace cem {
namespace message {
namespace common {

struct CemHeader
{
    uint64_t timestamp = 0;
    uint16_t cycle_counter = 0;
    uint64_t status = 0;
};

} // namespace common
} // namespace message
} // namespace cem

#endif
