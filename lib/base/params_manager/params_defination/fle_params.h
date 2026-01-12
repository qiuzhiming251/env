#ifndef FLE_PARAMS_H_
#define FLE_PARAMS_H_

#include <cstddef>

namespace cem {
namespace fusion {

struct FLEParams
{
    float trajectory_length_threshold = 20.0f;
    float lon_dist_end_threshold = 20.0f;
    float weight_of_lane_markers = 6.0f;
    float weight_coeff_k_of_lane_markers = -0.099f;
    float weight_coeff_b_of_lane_markers = 1.198f;
    float weight_coeff_max_of_lane_markers = 1.0f;
    float weight_coeff_min_of_lane_markers = 0.01f;
    float weight_x_min_lane_markers = 2.0f;
    float weight_x_max_of_lane_markers = 12.0f;
    float weight_of_road_edges = 8.0f;
    float weight_of_gtr = 4.0f;
    float weight_of_ego_trajectory = 4.0f;
    float weight_coeff_b_of_measuration = 100.0f;
    float weight_coeff_k_of_measuration = -0.25f;
    float weight_coeff_a_of_prediction = 14400.0f;
    float weight_coeff_b_of_prediction = 60.0f;
    uint8_t recent_curves_size = 1;
    uint8_t curves_windows_num = 1;
    float edge_weight_threshold = 4.0;
    float meas_proporation_threshold = 0.6;
    float last_ref_curve_filter_param = 0.5;
};

} // namespace fusion
} // namespace cem

#endif
