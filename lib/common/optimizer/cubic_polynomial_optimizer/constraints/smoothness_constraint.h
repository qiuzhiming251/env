#ifndef SMOOTHNESS_CONSTRAINT_H_
#define SMOOTHNESS_CONSTRAINT_H_

#include "common/math/math.h"
#include "message/internal_message.h"
#include "constraints_helper.h"

namespace cem {
namespace fusion {

struct SmoothnessConstraint
{
    SmoothnessConstraint(const Curve &curve, float weight, size_t seg_idx,
                         float first_seg_end_x, float seg_len, bool is_on_ramp)
        : curve_(curve),
          weight_(weight),
          seg_idx_(seg_idx),
          first_seg_end_x_(first_seg_end_x),
          seg_len_(seg_len)
    {
        float c2_scale_weight_min = is_on_ramp ? 1000.0f : 4000.0f;
        c2_scale_weight_ = 1.0f;

        if (std::fabs(curve_.c0) < 6.0f)
        {
            c2_scale_weight_ = 1.0f + 0.4f / (std::fabs(curve_.c2) + 1.0E-9);
            c2_scale_weight_ = std::max(c2_scale_weight_, c2_scale_weight_min);
        }
    }

    template <typename T>
    bool operator()(T const *const *parameter_blocks, T *residual) const
    {
        T c0, c1, c2, c3;
        CalcSegCoefficients(&c0, &c1, &c2, &c3, parameter_blocks, seg_idx_,
                            first_seg_end_x_, seg_len_);

        residual[0] = T(weight_) * (c0 - T(curve_.c0));
        residual[1] = T(weight_ * 5) * (c1 - T(curve_.c1));
        residual[2] = T(weight_ * c2_scale_weight_) * (c2 - T(curve_.c2));
        residual[3] = T(weight_) * (c3 - T(curve_.c3));

        return true;
    }


    static ceres::CostFunction *Create(const Curve &curve, float weight,
                                       size_t seg_idx, float first_seg_end_x,
                                       float seg_len, size_t segs_num,
                                       bool is_on_ramp)
    {
        auto *cost_function =
            new ceres::DynamicAutoDiffCostFunction<SmoothnessConstraint, 5>(
                new SmoothnessConstraint(curve, weight, seg_idx,
                                         first_seg_end_x, seg_len, is_on_ramp));
        cost_function->AddParameterBlock(2 * segs_num + 2);
        cost_function->SetNumResiduals(4);

        return cost_function;
    }

    Curve curve_;
    float weight_;
    float c2_scale_weight_;
    int seg_idx_;
    float first_seg_end_x_;
    float seg_len_;
};

} // namespace fusion
} // namespace cem

#endif
