#ifndef DATA_FUSION_PARAMS_H_
#define DATA_FUSION_PARAMS_H_

#include <cstddef>
#include <string>

namespace cem {
namespace fusion {

struct DataFusionParams
{
    struct MotionFusionParams
    {
        struct KalmanMotionFusionParams
        {
            std::string filter_method = "CustomKalmanFilter";
            bool enable_weighted_mean_update = true;
        } kalman_motion_fusion_params_;
    } motion_fusion_params_;

    struct TrackAttributeFusionParams
    {
        struct ColorFusionParams
        {
            /* data */
        } color_fusion_params_;

        struct ExistenceFusionParams
        {
            /* data */
        } existence_fusion_params_;

        struct TypeFusionParams
        {
            bool enable_type_fusion = true;
            size_t type_mismatch_thresh = 3;
            bool enable_cone_protection = true;
            size_t cone_risk_thresh = 3;
            bool enable_forward_vision_type = true;
        } type_fusion_params_;
    } track_attribute_fusion_params_;

    struct TrackerParams
    {
        struct PbfTrackerParams
        {
            std::string motion_fusion_method = "KalmanMotionFusion";
            std::string color_fusion_method = "DstColorFusion";
            std::string existence_fusion_method = "DstExistenceFusion";
            std::string type_fusion_method = "DstTypeFusion";
        } pbf_tracker_params_;
    } tracker_params_;
};

} // namespace fusion
} // namespace cem

#endif
