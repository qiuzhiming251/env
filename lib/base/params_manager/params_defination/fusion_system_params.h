#ifndef FUSION_SYSTEM_PARAMS_H_
#define FUSION_SYSTEM_PARAMS_H_

#include <string>

namespace cem {
namespace fusion {

struct FusionSystemParams
{
    struct SequentialFusionParams
    {
        std::string data_association_method = "NNAssociation";
        std::string track_method = "PbfTracker";
        float new_track_existence_thresh = 0.1f;
        float new_track_start_thresh = 5.0f;
        float new_track_length_thresh = 20.0f;
        float track_merge_dist_thresh = 0.3f;
    } sequential_fusion_params_;

    struct CentralFusionParams
    {
        bool enable_central_fusion = false;
    } central_fusion_params_;
};

} // namespace fusion
} // namespace cem

#endif
