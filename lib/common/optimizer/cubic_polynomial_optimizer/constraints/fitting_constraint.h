#ifndef FITTING_CONSTRAINT_H_
#define FITTING_CONSTRAINT_H_

#include "common/math/math.h"
#include "message/internal_message.h"
#include "constraints_helper.h"

namespace cem {
namespace fusion {

struct FittingConstraint
{
    FittingConstraint(const std::vector<SamplingPoint> &sampling_points,
                      float weight, size_t seg_idx, float first_seg_end_x,
                      float seg_len)
        : sampling_points_(&sampling_points),
          weight_(weight),
          seg_idx_(seg_idx),
          first_seg_end_x_(first_seg_end_x),
          seg_len_(seg_len)
    {
    }

    template <typename T>
    bool operator()(T const *const *parameter_blocks, T *residual) const
    {
        T c0, c1, c2, c3;
        CalcSegCoefficients(&c0, &c1, &c2, &c3, parameter_blocks, seg_idx_,
                            first_seg_end_x_, seg_len_);

        for (size_t i = 0; i < sampling_points_->size(); ++i)
        {
            const auto &point = sampling_points_->at(i);
            T dy = T(point.y) - CalculateY(c0, c1, c2, c3, T(point.x));
            residual[i] = dy * T(weight_) * T(point.weight);
        }

        return true;
    }

    static ceres::CostFunction *Create(
        const std::vector<SamplingPoint> &sampling_points, float weight,
        size_t seg_idx, float first_seg_end_x, float seg_len, size_t segs_num)
    {
        if (sampling_points.empty()) return nullptr;

        auto *cost_function =
            new ceres::DynamicAutoDiffCostFunction<FittingConstraint, 5>(
                new FittingConstraint(sampling_points, weight, seg_idx,
                                      first_seg_end_x, seg_len));
        cost_function->AddParameterBlock(2 * segs_num + 2);
        cost_function->SetNumResiduals(sampling_points.size());

        return cost_function;
    }

    const std::vector<SamplingPoint> *sampling_points_;
    const float weight_;
    const size_t seg_idx_;
    const float first_seg_end_x_;
    const float seg_len_;
};

} // namespace fusion
} // namespace cem

#endif
