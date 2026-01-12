#ifndef PLANNING_RESULT_DATA_H_
#define PLANNING_RESULT_DATA_H_

#include <vector>

#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

struct PLanningResultData
{
    enum LaneChangeState {
        Keeping = 0,
        PrepareLeft = 1,
        PrepareRight = 2,
        ChangeLeft = 3,
        ChangeRight = 4,
        ChangeCancel = 5,
        KeepingInit = 6,
        ChangeInvalid = 7,
};
    cem::message::common::Header header;
    LaneChangeState lc_state = Keeping;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
