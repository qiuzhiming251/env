#ifndef ROADEDGE_LANEMARKER_COLLISION_CONSTRAINT_H_
#define ROADEDGE_LANEMARKER_COLLISION_CONSTRAINT_H_

#include "common/math/math.h"
#include "message/internal_message.h"
#include "constraints_helper.h"

namespace cem {
namespace fusion {

struct RoadEdgeLaneMarkerCollisionConstraint
{
    RoadEdgeLaneMarkerCollisionConstraint(
        const std::vector<std::pair<int, SamplingPoint>> &re_points,
        float weight, size_t seg_idx, float first_seg_end_x, float seg_len,
        float collision_thresh)
        : re_points_(re_points),
          weight_(weight),
          seg_idx_(seg_idx),
          first_seg_end_x_(first_seg_end_x),
          seg_len_(seg_len),
          collision_thresh_(collision_thresh)
    {
    }

    template <typename T>
    bool operator()(T const *const *parameter_blocks, T *residual) const
    {
        T c0, c1, c2, c3;
        CalcSegCoefficients(&c0, &c1, &c2, &c3, parameter_blocks, seg_idx_,
                            first_seg_end_x_, seg_len_);

        size_t unsafe_re_points_num = 0;
        for (size_t i = 0; i < re_points_.size(); ++i)
        {
            T px = T(re_points_[i].second.x);
            T py = T(re_points_[i].second.y);
            T dy = py - CalculateY(c0, c1, c2, c3, px);

            T sign = T(re_points_[i].first) > T(0) ? T(1) : T(-1);
            residual[i] = dy * sign;

            if (residual[i] > T(collision_thresh_))
            {
                residual[i] = T(0.0);
            }
            else
            {
                residual[i] -= T(collision_thresh_);
                residual[i] *= T(3.0);
                ++unsafe_re_points_num;
            }
            residual[i] *=
                (T(weight_) * T(10.0) * (ceres::exp(residual[i]) - T(1)));

        }

        if (unsafe_re_points_num > 0 && seg_idx_ == 2)
        {
            // LaneFusionDiagnostic::Instance()->Update(
            //     DiagnosticLevel::DEBUG, DTC::RE_CONSTRAIN_HDM_MARKER);
        }

        return true;
    }


    static ceres::CostFunction *Create(
        const std::vector<std::pair<int, SamplingPoint>> &re_points,
        float weight, size_t seg_idx, float first_seg_end_x, float seg_len,
        size_t segs_num, float collision_thresh)
    {
        if (re_points.empty()) return nullptr;

        auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
            RoadEdgeLaneMarkerCollisionConstraint, 5>(
            new RoadEdgeLaneMarkerCollisionConstraint(
                re_points, weight, seg_idx, first_seg_end_x, seg_len,
                collision_thresh));
        cost_function->AddParameterBlock(2 * segs_num + 2);
        cost_function->SetNumResiduals(re_points.size());

        return cost_function;
    }

    const std::vector<std::pair<int, SamplingPoint>> re_points_;
    float weight_;
    
    const size_t seg_idx_;
    const float first_seg_end_x_;
    const float seg_len_;
    const float collision_thresh_;
};

} // namespace fusion
} // namespace cem

#endif
