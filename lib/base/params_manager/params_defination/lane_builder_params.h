#ifndef LANE_BUILDER_PARAMS_H_
#define LANE_BUILDER_PARAMS_H_

#include <cstddef>

namespace cem {
namespace fusion {

struct LaneBuilderParams
{
    float lane_min_width = 2.5f;
    float lane_max_width = 5.0f;
    float lane_default_width = 3.25f;
    size_t lane_width_filter_buffer_size = 20;
    size_t max_lanes_num = 13;
    size_t max_lanemarkers_num = 14;
    size_t lane_model_change_thresh = 20;
    bool enable_gen_virtual_lanemarkers = false;
    float re_correct_trigger_thresh = 0.3f;
    bool enable_check_virtual_lanemarkers = true;
    float Bev_delay_angle = 0.3f;//单位度数
};

} // namespace fusion
} // namespace cem

#endif
