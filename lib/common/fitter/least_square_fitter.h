#ifndef LEAST_SQUARE_FITTER_H_
#define LEAST_SQUARE_FITTER_H_

#include <Eigen/Dense>

#include "message/internal_message.h"
#include <deque>
#include "cyber/common/log.h"

namespace cem {
namespace fusion {

std::pair<bool, Eigen::VectorXd> FitterLeastSquareMethod(
    const GeometryLine &geometry, int rear_fitted_range, int front_fitted_range,
    uint8_t orders);

Eigen::VectorXd FitterLeastSquareMethod(GeometryLine *geometry, uint8_t orders);

Eigen::VectorXd FitterLeastSquareMethod(const std::deque<SamplingPoint> &points,
                                        uint8_t orders);

bool FitterLeastSquareMethod(const std::vector<SamplingPoint> &points, uint8_t orders, Curve &curve);

bool FitterLeastSquareMethod(const std::vector<cem::fusion::Point2DF> &points, uint8_t orders, Curve &curve);

template <typename T>
Eigen::VectorXd FitterLeastSquareMethod(T first, T end, uint8_t orders)
{
    Eigen::VectorXd result;
    int64_t count = end - first;

    if (count < orders + 1 || orders < 1) return result;

    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < count) raw_data_x.resize(count);
    if (raw_data_y.size() < count) raw_data_y.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        raw_data_x[i] = (first + i)->x;
        raw_data_y[i] = (first + i)->y;
    }


    Eigen::Map<Eigen::VectorXd> sample_x(raw_data_x.data(), count);
    Eigen::Map<Eigen::VectorXd> sample_y(raw_data_y.data(), count);

    Eigen::MatrixXd vandermonde_mtx(count, orders + 1);

    vandermonde_mtx.col(0) = Eigen::VectorXd::Constant(count, 1, 1);
    for (size_t i = 1; i < orders + 1; ++i)
    {
        vandermonde_mtx.col(i) =
            vandermonde_mtx.col(i - 1).array() * sample_x.array();
    }

    result = vandermonde_mtx.householderQr().solve(sample_y);

    return result;
}

void FitterLeastSquareBasedLaneContinuityMethod(
    GeometryLine *geometry, const GeometryLine &base_geometry,
    double connected_point_x);

void FitterLeastSquareBasedLaneContinuityMethod(
    GeometryLine *far_geometry, GeometryLine *near_geometry,
    const std::deque<SamplingPoint> &points, size_t connect_point_idx);

bool IsLeastSquareFitValid(const Eigen::VectorXd &Y, const Eigen::MatrixXd &A,
                           const Eigen::VectorXd &X, float mse_thresh);

} // namespace fusion
} // namespace cem

#endif
