#ifndef NRP_PARAMS_H_
#define NRP_PARAMS_H_

#include <string>

namespace cem {
namespace fusion {

struct NrpParams
{
    std::string nrp_mode = "Fusion"; // Fusion, Map or Switch
    float switch_from_vision_to_hdmap_dist_thresh = 500.0f;
    float switch_from_hdmap_to_vision_dist_thresh = 500.0f;
    float enter_split_merge_dist_thresh = 50.0f;
    float exit_split_merge_dist_thresh = 50.0f;
    float ramp_end_area_dist_thresh = 50.0f;
    float enter_urban_dist_thresh = 200.0f;
    float exit_urban_dist_thresh = 200.0f;
    bool pre_center_line_opt_ego_lanemarker = true;
    bool re_enable = false;
    bool re_points_viz = false;
    float re_diff_dis = 0.3;
    bool enable_hdm_loc_check = true;
    bool enable_nrp_switch = true;
    float ld_map_activation_dist_thresh = 2000.0f;
    float ld_map_deactivation_dist_thresh = 1.0f;
};

} // namespace fusion
} // namespace cem

#endif
