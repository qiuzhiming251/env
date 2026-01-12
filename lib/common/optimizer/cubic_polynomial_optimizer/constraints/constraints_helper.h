#ifndef CONSTRAINTS_HELPER_H_
#define CONSTRAINTS_HELPER_H_

#include <cstddef>
#include <cmath>
#include <ceres/ceres.h>
namespace cem {
namespace fusion {

template <typename T>
void CalcSegCoefficients(T *c0, T *c1, T *c2, T *c3,
                         T const *const *parameter_blocks, size_t seg_idx,
                         float first_seg_end_x, float seg_len)
{
    // all coefficients are stored at parameter_blocks[0] which is the only one
    // parameter block
    auto curve_params = parameter_blocks[0];

    *c0 = curve_params[0];
    *c1 = curve_params[1];

    for (size_t i = 1; i <= seg_idx; ++i)
    {
        T last_seg_c0 = *c0;
        T last_seg_c1 = *c1;
        T last_seg_c2 = curve_params[i * 2];
        T last_seg_c3 = curve_params[i * 2 + 1];

        T c2 = curve_params[i * 2 + 2];
        T c3 = curve_params[i * 2 + 3];

        auto connected_point_x = first_seg_end_x + seg_len * (i - 1);
        T pow_x_2 = T(std::pow(connected_point_x, 2));
        T pow_x_3 = T(std::pow(connected_point_x, 3));

        *c0 = last_seg_c0 - last_seg_c2 * pow_x_2 -
              T(2) * last_seg_c3 * pow_x_3 + c2 * pow_x_2 + T(2) * c3 * pow_x_3;
        *c1 = last_seg_c1 + T(2) * last_seg_c2 * T(connected_point_x) +
              T(3) * last_seg_c3 * pow_x_2 - T(2) * c2 * T(connected_point_x) -
              T(3) * c3 * pow_x_2; //zph use at connect point y0=y1, y0'=y1'
    }

    *c2 = curve_params[seg_idx * 2 + 2];
    *c3 = curve_params[seg_idx * 2 + 3];
}

} // namespace fusion
} // namespace cem

#endif
