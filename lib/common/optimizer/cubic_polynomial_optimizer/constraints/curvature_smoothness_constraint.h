#ifndef CURVATURE_SMOOTHNESS_CONSTRAINT_H_
#define CURVATURE_SMOOTHNESS_CONSTRAINT_H_

#include "constraints_helper.h"

namespace cem {
namespace fusion {

struct CurvatureSmoothnessConstraint
{
    CurvatureSmoothnessConstraint(float weight, size_t segs_num,
                                  float first_seg_end_x, float seg_len)
        : weight_(weight),
          segs_num_(segs_num),
          first_seg_end_x_(first_seg_end_x),
          seg_len_(seg_len)
    {
    }

    template <typename T>
    bool operator()(T const *const *parameter_blocks, T *residual) const
    {
        auto curve_params = parameter_blocks[0];

        T c0 = curve_params[0];
        T c1 = curve_params[1];
        for (size_t i = 1; i < segs_num_; ++i)
        {
            T last_c0 = c0;
            T last_c1 = c1;
            T last_c2 = curve_params[i * 2];
            T last_c3 = curve_params[i * 2 + 1];

            auto connected_point_x = first_seg_end_x_ + seg_len_ * (i - 1);

            T c2 = curve_params[i * 2 + 2];
            T c3 = curve_params[i * 2 + 3];

            T last_curvature = last_c2 + last_c3 * T(connected_point_x);
            T cur_curvature = c2 + c3 * T(connected_point_x);

            residual[i - 1] = T(weight_) * (last_curvature - cur_curvature);
        }

        return true;
    }

    static ceres::CostFunction *Create(float weight, float first_seg_end_x,
                                       float seg_len, size_t segs_num)
    {
        auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
            CurvatureSmoothnessConstraint, 5>(new CurvatureSmoothnessConstraint(
            weight, segs_num, first_seg_end_x, seg_len));
        cost_function->AddParameterBlock(2 * segs_num + 2);
        cost_function->SetNumResiduals(segs_num - 1);

        return cost_function;
    }

    float weight_;
    size_t segs_num_;
    float first_seg_end_x_;
    float seg_len_;
};

} // namespace fusion
} // namespace cem

#endif
