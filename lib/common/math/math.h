#ifndef MATH_H_
#define MATH_H_

#include <tuple>
#include <algorithm>
#include <numeric>
#include <cmath>

#include "message/internal_message.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/vec2d.h"
namespace cem {
namespace fusion {
using byd::common::math::LineSegment2d;
using byd::common::math::Vec2d;

template <typename T>
inline T CalculateY(const Curve& curve, T x)
{
    return curve.c0 + curve.c1 * x + curve.c2 * x * x + curve.c3 * x * x * x;
}

template <typename T>
inline T CalculateY(T c0, T c1, T c2, T c3, T x)
{
    return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
}

template <typename T>
inline T Min3(T num1, T num2, T num3)
{
    return std::min(std::min(num1, num2), num3);
}

template <typename T>
inline T Max3(T num1, T num2, T num3)
{
    return std::max(std::max(num1, num2), num3);
}

template <typename T>
inline T Middle3(T num1, T num2, T num3)
{
    return Max3(std::min(num1, num2), std::min(num1, num3),
                std::min(num2, num3));
}

template <typename Iterator, typename T>
void CalculateSTD(Iterator first, Iterator last, T* mean, T* std)
{
    auto values_num = last - first;
    if (values_num < 2) return;

    T sum = std::accumulate(first, last, 0.0);
    *mean = sum / values_num;

    T accum = 0.0;
    std::for_each(first, last, [&](const T value) {
        accum += (value - *mean) * (value - *mean);
    });

    *std = std::sqrt(accum / (values_num - 1));
}

std::tuple<float, float, float> CalculateCurvesOverlapDist(
    const Curve& first_curve, const Curve& second_curve);

template <typename T>
bool CalculateWeightedMEAN(const std::vector<T>& raw_values, T* weighted_mean)
{
    *weighted_mean = 0.0;
    size_t removed_values_num = 0;
    if (raw_values.size() < removed_values_num + 2) return false;

    T raw_sum = std::accumulate(raw_values.begin(), raw_values.end(), 0.0);
    T raw_mean = raw_sum / raw_values.size();
    std::vector<T> filtered_values = raw_values;
    std::sort(filtered_values.begin(), filtered_values.end(),
              [&](const T a, const T b) -> bool {
                  return std::fabs(a - raw_mean) < std::fabs(b - raw_mean);
              });
    filtered_values.resize(raw_values.size() - removed_values_num);

    T filtered_sum =
        std::accumulate(filtered_values.begin(), filtered_values.end(), 0.0);
    T filtered_mean = filtered_sum / filtered_values.size();

    T filtered_deviation_square_sum = 0.0;
    std::vector<T> filtered_deviation_squares;
    filtered_deviation_squares.clear();
    filtered_deviation_squares.reserve(filtered_values.size());
    std::for_each(
        filtered_values.begin(), filtered_values.end(), [&](const T value) {
            filtered_deviation_squares.emplace_back((value - filtered_mean) *
                                                    (value - filtered_mean));
            filtered_deviation_square_sum += filtered_deviation_squares.back();
        });

    T weights_sum = 0.0;
    std::vector<T> weights;
    weights.clear();


    if (filtered_deviation_square_sum < 1E-30)
    {
        weights.resize(filtered_values.size(), 1.0 / filtered_values.size());
        weights_sum = 1.0;
    }
    else
    {
        weights.reserve(filtered_values.size());
        std::for_each(
            filtered_deviation_squares.begin(),
            filtered_deviation_squares.end(), [&](const T deviation_square) {
                weights.emplace_back(
                    1 -
                    std::pow(deviation_square / filtered_deviation_square_sum,
                             4));
                weights_sum += weights.back();
            });
    }

    for (size_t i = 0; i < filtered_values.size(); ++i)
    {
        *weighted_mean += filtered_values.at(i) * weights.at(i) / weights_sum;
    }

    return true;
}

class MultiLineSegment {
   public:
    MultiLineSegment() = default;
    void Init(const std::vector<Vec2d> &points);

    bool GetProjection(const Vec2d &point, double *accumulate_s, double *lateral, double *min_distance, int *index_min = nullptr,
                       double radius1d = -1.0, int index_center = -1, std::string *str_info = nullptr) const;

    [[nodiscard]] const std::vector<Vec2d> &GetPathPoints() const { return path_points_; }

    [[nodiscard]] double Length() const { return length_; }

    [[nodiscard]] std::pair<Vec2d, double> GetPointAtDistance(double dis_spec) const;

   private:
    int                        num_points_   = 0;
    int                        num_segments_ = 0;
    double                     length_       = 0.0;
    std::vector<Vec2d>         path_points_;
    std::vector<double>        accumulated_s_;
    std::vector<LineSegment2d> segments_;
};

} // namespace fusion
} // namespace cem

#endif
