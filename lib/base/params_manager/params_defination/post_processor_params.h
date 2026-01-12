#ifndef POST_PROCESSOR_PARAMS_H_
#define POST_PROCESSOR_PARAMS_H_

namespace cem {
namespace fusion {

struct PostProcessorParams
{
    bool tsrm_enable = false;
    bool tsrm_priority_higher_fvcm = false;
    float forward_hdmap_curvature_dist_thresh = 80.0f;
    bool enable_forward_hdmap_curvature_forever = true;
    int tsrm_sleep_frame = 0;
    bool enable_sd_spd_lm = false;
    bool enable_bev_shape_fuse = false;
};

} // namespace fusion
} // namespace cem

#endif
