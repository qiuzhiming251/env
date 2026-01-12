#include "NavigationParams.h"

namespace cem {
namespace fusion {
namespace navigation {
namespace NavigationConfig {

double max_distance            = 700.0;  // 路口起效最大距离参数
double fine_matching_threshold = 5.0;  // 精匹配阈值参数
double single_guide_lane_distance    = 400.0;  // 单边推荐的生效距离
unsigned int oppo_lane_keep_cnt_threshold  = 5; // 对向车道的过滤保持帧数
};
}
}
}