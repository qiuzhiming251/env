#ifndef DIFFERENCE_CONSTRAINT_H_
#define DIFFERENCE_CONSTRAINT_H_

#include <functional>
#include "message/internal_message.h"
#include "constraints_helper.h"

namespace cem {
namespace fusion {

struct DifferenceConstraint
{
    typedef std::function<bool(LocalizationConstPtr, LocalizationConstPtr,
                               float &, float &, float &)>
        RelativePoseFuncObjType;

    DifferenceConstraint(const Curve &curve, float weight, size_t seg_idx,
                         float first_seg_end_x, float seg_len, bool is_on_ramp,
                         RelativePoseFuncObjType calc_relative_pose,
                         LocalizationPtr last_frame_position,
                         LocalizationPtr cur_frame_position)
        : curve_(curve),
          weight_(weight),
          seg_idx_(seg_idx),
          first_seg_end_x_(first_seg_end_x),
          seg_len_(seg_len)
    {
        float c2_scale_weight_min = is_on_ramp ? 100.0f : 400.0f;
        c2_scale_weight_ = 1;

        if (std::fabs(curve_.c0) < 6.0f)
        {
            c2_scale_weight_ = 1.0f + 0.4f / (std::fabs(curve_.c2) + 1.0E-9);
            c2_scale_weight_ = std::max(c2_scale_weight_, c2_scale_weight_min);
        }
        relative_pose_valid_ =
            calc_relative_pose(last_frame_position, cur_frame_position,
                               delta_odo_x_, delta_odo_y_, delta_odo_heading_);
        if (std::fabs(delta_odo_x_) < 1.0E-2)
        {
            relative_pose_valid_ = false;
        }
    }

    template <typename T>
    bool operator()(T const *const *parameter_blocks, T *residual) const
    {
        T c0, c1, c2, c3;
        CalcSegCoefficients(&c0, &c1, &c2, &c3, parameter_blocks, seg_idx_,
                            first_seg_end_x_, seg_len_);

        T diff_c1 =
            (c0 - T(curve_.c0) + T(delta_odo_y_)) / T(delta_odo_x_) +
            T(curve_.c2 * delta_odo_x_) +
            T(2 * curve_.c3 * delta_odo_x_ * delta_odo_x_ - delta_odo_heading_);
        T diff_c2 =
            (c1 - T(curve_.c1) + T(delta_odo_heading_)) / T(2 * delta_odo_x_) +
            T(1.5 * curve_.c3 * delta_odo_x_);
        T diff_c3 = (c2 - T(curve_.c2)) / T(delta_odo_x_);

        residual[0] = T(weight_ * 5) * (c1 - diff_c1);
        residual[1] = T(weight_ * c2_scale_weight_) * (c2 - diff_c2);
        residual[2] = T(weight_) * (c3 - diff_c3);

        if (!relative_pose_valid_)
        {
            residual[0] = T(0);
            residual[1] = T(0);
            residual[2] = T(0);
        }

        return true;
    }

    static ceres::CostFunction *Create(
        const Curve &curve, float weight, size_t seg_idx, float first_seg_end_x,
        float seg_len, size_t segs_num, bool is_on_ramp,
        RelativePoseFuncObjType calc_relative_pose,
        LocalizationPtr last_frame_position, LocalizationPtr cur_frame_position)
    {
        auto *cost_function =
            new ceres::DynamicAutoDiffCostFunction<DifferenceConstraint, 5>(
                new DifferenceConstraint(
                    curve, weight, seg_idx, first_seg_end_x, seg_len,
                    is_on_ramp, calc_relative_pose, last_frame_position,
                    cur_frame_position));
        cost_function->AddParameterBlock(2 * segs_num + 2);
        cost_function->SetNumResiduals(3);

        return cost_function;
    }

    Curve curve_;
    float weight_;
    float c2_scale_weight_;
    size_t seg_idx_;
    float first_seg_end_x_;
    float seg_len_;
    float delta_odo_x_;
    float delta_odo_y_;
    float delta_odo_heading_;
    bool relative_pose_valid_;
};

} // namespace fusion
} // namespace cem

#endif
