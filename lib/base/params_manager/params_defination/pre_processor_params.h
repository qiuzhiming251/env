#ifndef PRE_PROCESSOR_PARAMS_H_
#define PRE_PROCESSOR_PARAMS_H_

namespace cem {
namespace fusion {

struct PreProcessorParams
{
    bool enable_fvcm_comp = false;
    bool enable_fvcm_re_protection = false;
    bool enable_fvcm_re_protection_outside_ramp = false;
    bool enable_vision_pre_fusion_re_protection = false;
    bool enable_vision_pre_fusion_in_rainy = false;
    bool enable_vision_pre_fusion_within_hdmap = false;
    bool enable_hdmap_comp = false;
    float hdm_virtual_point_weight = 1.0f;
    bool enable_extend_virtual_seg = false;
    bool enable_gtr_fix = true;
    float collision_prediction_time = 1.0f;
    bool enable_gtr_choose_opposing_lane = true;
    bool enable_collision_detection = true;
    bool enable_bev_evaluate = true;
    bool enable_limit_bev_evaluate_speed = true;
    bool enable_collision_detection_rviz = false;
    bool enable_opposite_lanemarker_filter = true;
    bool enable_re_constraint_correction = true;
    bool enable_reduce_hdmap_weight = true;
    bool enable_nrp_downgrade_by_evaluate = true;
    bool enable_hdmap_high_priority_seg_enable = true;
    bool enable_hdmap_curvature_diff_cost_enable = true;
    bool enable_hdmap_double_re_constraint = true;
    bool enable_hdm_lane_discrete_point_smooth = true;
    bool enable_hdm_lane_vitrual_erase_and_generator = true;
    bool enable_hdm_lane_fit = true;
    bool enable_replace_bad_measurement_with_good = true;
    bool enable_remove_bev_faraway_points = false;
    bool enable_join_bev_merge_lanemarkers = true;
    bool enable_double_roadedge_constrain = true;
    bool enable_extend_lanemarker_length_for_re = true;
    bool enable_downgrade_on_ramp = false;
    bool enable_highway_measurement_replacement = true;
    bool enable_bev_lanemarker_width = true;
    bool is_ola_lanemarker_width_available = false;
    bool enable_pole_roadedge_constraint_correction = true;
    bool enable_bev_lane_topology = true;
    bool enable_bev_lane_topology_rviz = false;
    bool enable_mean_nrp_downgrade_work_without_re_constraint = true;
    bool enable_hdmap_eval_recovery_optimize_on_ramp = true;
    bool enable_map_checker_downgrade = true;
    bool enable_large_hdmap_radius_re_constraint = true;
    float large_hdmap_radius_thresh = 250.0f;
};

} // namespace fusion
} // namespace cem

#endif
