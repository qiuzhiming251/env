#ifndef MASTER_PARAMS_H_
#define MASTER_PARAMS_H_

#include <cstddef>
#include <string>

namespace cem {
namespace fusion {

struct MasterParams
{
    std::string fusion_method = "SequentialFusion";
    std::string lane_fusion_method = "CentralFusion";
    size_t segments_num = 2;                    // 2 or 4
    size_t coef_filter_segs_num = 1;             
    std::string used_localization = "Asensing"; // Saic, Asensing or Daisch
    struct FusionSensorsParams
    {
        bool enable_fvcm = true;
        bool enable_around_view = true;
        bool enable_hdmap = true;
        bool enable_traffic_flow = true;
        bool enable_road_edge = true;
        bool enable_further_lane_extension = true;
        bool enable_measurement_evaluate = false;
        bool enable_bev = true;
        bool enable_bev_motion_nonhost = true;
        bool enable_vision_pre_fusion = true;
    } fusion_sensor_params_;

    struct SceneParams
    {
        bool intersection_enable_stopline = true;
        float enter_intersection_dx_end_threshold = 15.0;
        float stopline_compensate_dis_max = 20.0;
        float thorough_exit_crossroad_dx_start_threshold = 3;
        float coming_exit_crossroad_dx_start_threshold = 10;
        float dx_startHysteresisThreshold = 2;
        float dx_endHysteresisThreshold = 5;
    } scene_params_;
};

} // namespace fusion
} // namespace cem

#endif
