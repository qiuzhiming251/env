#ifndef NAVIGATIONPARAMS_H
#define NAVIGATIONPARAMS_H

#include <vector>
#include <set>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include "lib/message/env_model/routing_map/routing_map.h"



namespace cem {
namespace fusion {
namespace navigation {
namespace NavigationConfig {

  extern double max_distance           ;     // 路口起效最大距离参数
  extern double fine_matching_threshold;     // 精匹配阈值参数
  extern double single_guide_lane_distance;  // 单边推荐的生效距离
  extern unsigned int  oppo_lane_keep_cnt_threshold;  // 对向车道的过滤保持帧数
};

}
}
}

#endif //NAVIGATIONPARAMS_H
