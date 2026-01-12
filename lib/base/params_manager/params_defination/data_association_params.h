#ifndef DATA_ASSOCIATION_PARAMS_H_
#define DATA_ASSOCIATION_PARAMS_H_

#include <string>

namespace cem {
namespace fusion {

struct DataAssociationParams
{
    struct NNAssociationParams
    {
        float basic_weight = 0.01f;
        float match_dist_thresh = 0.3f;
        std::string dist_direction = "normal"; // normal or lateral
    } nn_association_params_;
};

} // namespace fusion
} // namespace cem

#endif
