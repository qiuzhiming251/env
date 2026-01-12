#ifndef PLAN_FUNC_STATE_DATA_H_
#define PLAN_FUNC_STATE_DATA_H_

#include <vector>

#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

struct FuncState {
    uint32_t is_noa_actv_flg;
};
struct PLanFuncState
{
    cem::message::common::Header header;
    FuncState func_state;
};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
