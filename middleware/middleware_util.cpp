#include "middleware_util.h"

namespace cem {
namespace fusion {

#ifdef MW_TYPE_DDS
ClockNode::ClockNode() : MW_NODE_BASE("fusion_lane_clock")
{
    clock = this->get_clock();
}
#endif

} // namespace fusion
} // namespace cem
