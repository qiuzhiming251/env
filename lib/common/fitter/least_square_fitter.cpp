#include "least_square_fitter.h"

namespace cem {
namespace fusion {


std::pair<bool, Eigen::VectorXd> FitterLeastSquareMethod(
    const GeometryLine &geometry, int rear_fitted_range, int front_fitted_range,
    uint8_t orders)
{
    Eigen::VectorXd result;


    if (rear_fitted_range > 0 || front_fitted_range < 0 ||
        geometry.sampling_points_num < orders + 1 || orders < 1)
    {

        return {false, result};
    }


    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < geometry.sampling_points.size())
        raw_data_x.resize(geometry.sampling_points.size());
    if (raw_data_y.size() < geometry.sampling_points.size())
        raw_data_y.resize(geometry.sampling_points.size());

    uint16_t count = 0;
    for (size_t i = 0; i < geometry.sampling_points.size(); ++i)
    {
        if (geometry.sampling_points.at(i).x >= rear_fitted_range &&
            geometry.sampling_points.at(i).x <= front_fitted_range)
        {
            raw_data_x[count] = geometry.sampling_points.at(i).x;
            raw_data_y[count] = geometry.sampling_points.at(i).y;
            ++count;
        }
    }
    if (count < orders + 1)
    {

        return {false, result};
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
    return {true, result};
}


Eigen::VectorXd FitterLeastSquareMethod(GeometryLine *geometry, uint8_t orders)
{
    Eigen::VectorXd result;
    result.setZero(orders + 1, 1);
    auto count = geometry->sampling_points.size();


    if (count < orders + 1 || orders < 1)
    {

        geometry->is_curve_valid = false;
        geometry->is_sampling_point_valid = false;
        return result;
    }

    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < count) raw_data_x.resize(count);
    if (raw_data_y.size() < count) raw_data_y.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        raw_data_x[i] = geometry->sampling_points.at(i).x;
        raw_data_y[i] = geometry->sampling_points.at(i).y;
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

    float mse_thresh = 0.5;

    if (IsLeastSquareFitValid(sample_y, vandermonde_mtx, result, mse_thresh))
    {
        geometry->is_curve_valid = true;
        geometry->curve.c0 = result(0);
        geometry->curve.c1 = result(1);
        geometry->curve.c2 = result(2);
        geometry->curve.c3 = result(3);
    }
    else
    {
        geometry->is_curve_valid = false;
    }

    return result;
}

Eigen::VectorXd FitterLeastSquareMethod(const std::deque<SamplingPoint> &deque,
                                        uint8_t orders)
{
    Eigen::VectorXd result;
    auto count = deque.size();

    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < count) raw_data_x.resize(count);
    if (raw_data_y.size() < count) raw_data_y.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        raw_data_x[i] = deque[i].x;
        raw_data_y[i] = deque[i].y;
    }


    Eigen::Map<Eigen::VectorXd> sample_x(raw_data_x.data(), count);
    Eigen::Map<Eigen::VectorXd> sample_y(raw_data_y.data(), count);


    Eigen::MatrixXd vandermonde_mtx(count, orders + 1);


    vandermonde_mtx.col(0) = Eigen::VectorXd::Constant(count, 1, 1);
    for (size_t i = 0; i < orders + 1; ++i)
    {
        vandermonde_mtx.col(i) =
            vandermonde_mtx.col(i - 1).array() * sample_x.array();
    }


    result = vandermonde_mtx.householderQr().solve(sample_y);

    return result;
}

bool FitterLeastSquareMethod(const std::vector<SamplingPoint> &points,
                             uint8_t orders, Curve &curve)
{
    Eigen::VectorXd result;
    uint16_t count = points.size();


    if (count < orders + 1 || orders < 1)
    {
        AERROR << "Insufficient sampling points to fit a " << orders
               << " order polynomial, please check it!";
        return false;
    }

    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < count) raw_data_x.resize(count);
    if (raw_data_y.size() < count) raw_data_y.resize(count);

    for (size_t i = 0; i < count; ++i)
    {
        raw_data_x[i] = points[i].x;
        raw_data_y[i] = points[i].y;
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

    float mse_thresh = 0.5;
    if (IsLeastSquareFitValid(sample_y, vandermonde_mtx, result, mse_thresh))
    {
        curve.c0 = result(0);
        curve.c1 = result(1);
        curve.c2 = result(2);
        curve.c3 = result(3);
        curve.lon_dist_start = *(std::min_element(
            std::begin(raw_data_x), std::begin(raw_data_x) + count));
        curve.lon_dist_end = *(std::max_element(
            std::begin(raw_data_x), std::begin(raw_data_x) + count));

        return true;
    }
    else
    {
        return false;
    }
}

bool FitterLeastSquareMethod(const std::vector<cem::fusion::Point2DF> &points, uint8_t orders, Curve &curve) {
  Eigen::VectorXd result;
  uint16_t count = points.size();

  if (count < orders + 1 || orders < 1) {
    AERROR << "Insufficient sampling points to fit a " << orders << " order polynomial, please check it!";
    return false;
  }

  static std::vector<double> raw_data_x;
  static std::vector<double> raw_data_y;
  if (raw_data_x.size() < count) raw_data_x.resize(count);
  if (raw_data_y.size() < count) raw_data_y.resize(count);

  for (size_t i = 0; i < count; ++i) {
    raw_data_x[i] = points[i].x;
    raw_data_y[i] = points[i].y;
  }

  Eigen::Map<Eigen::VectorXd> sample_x(raw_data_x.data(), count);
  Eigen::Map<Eigen::VectorXd> sample_y(raw_data_y.data(), count);

  Eigen::MatrixXd vandermonde_mtx(count, orders + 1);

  vandermonde_mtx.col(0) = Eigen::VectorXd::Constant(count, 1, 1);
  for (size_t i = 1; i < orders + 1; ++i) {
    vandermonde_mtx.col(i) = vandermonde_mtx.col(i - 1).array() * sample_x.array();
  }

  result = vandermonde_mtx.householderQr().solve(sample_y);

  float mse_thresh = 0.5;
  if (IsLeastSquareFitValid(sample_y, vandermonde_mtx, result, mse_thresh)) {
    curve.c0 = result(0);
    curve.c1 = result(1);
    curve.c2 = result.size() < 3 ? 0.0 : result(2);
    curve.c3 = result.size() < 4 ? 0.0 : result(3);

    curve.lon_dist_start = *(std::min_element(std::begin(raw_data_x), std::begin(raw_data_x) + count));
    curve.lon_dist_end = *(std::max_element(std::begin(raw_data_x), std::begin(raw_data_x) + count));

    return true;
  } else {
    return false;
  }
}

void FitterLeastSquareBasedLaneContinuityMethod(
    GeometryLine *geometry, const GeometryLine &base_geometry,
    double connected_point_x)
{
    if (geometry->sampling_points.empty())
    {
        geometry->is_curve_valid = false;
        return;
    }
    if (base_geometry.sampling_points.empty() || !base_geometry.is_curve_valid)
    {
        FitterLeastSquareMethod(geometry, 3);
        return;
    }
    Eigen::VectorXd base_param(6);
    base_param << base_geometry.curve.c0, base_geometry.curve.c1,
        base_geometry.curve.c2, base_geometry.curve.c3, base_geometry.curve.c2,
        base_geometry.curve.c3;

    Eigen::VectorXd result;
    auto count = geometry->sampling_points.size();
    auto base_count = base_geometry.sampling_points.size();
    auto total_count = count + base_count;

    static std::vector<double> raw_data_x;
    static std::vector<double> raw_data_y;
    if (raw_data_x.size() < total_count) raw_data_x.resize(total_count);
    if (raw_data_y.size() < total_count) raw_data_y.resize(total_count);

    for (size_t i = 0; i < base_count; ++i)
    {
        raw_data_x[i] = base_geometry.sampling_points.at(i).x;
        raw_data_y[i] = base_geometry.sampling_points.at(i).y;
    }
    for (size_t i = 0; i < count; ++i)
    {
        auto idx = i + base_count;
        raw_data_x[idx] = geometry->sampling_points.at(i).x;
        raw_data_y[idx] = geometry->sampling_points.at(i).y;
    }


    Eigen::Map<Eigen::VectorXd> sample_x(raw_data_x.data(), total_count);
    Eigen::Map<Eigen::VectorXd> sample_y(raw_data_y.data(), total_count);

    Eigen::MatrixXd vandermonde_mtx = Eigen::MatrixXd::Zero(total_count, 6);


    vandermonde_mtx.col(0) = Eigen::VectorXd::Constant(total_count, 1, 1);
    for (size_t i = 1; i < 4; ++i)
    {
        vandermonde_mtx.col(i) =
            vandermonde_mtx.col(i - 1).array() * sample_x.array();
    }


    for (size_t i = base_count; i < total_count; ++i)
    {
        double x = sample_x(i);
        vandermonde_mtx(i, 2) =
            -std::pow(connected_point_x, 2) + 2 * connected_point_x * x;
        vandermonde_mtx(i, 3) = 3 * std::pow(connected_point_x, 2) * x -
                                2 * std::pow(connected_point_x, 3);
        vandermonde_mtx(i, 4) =
            std::pow(connected_point_x, 2) - 2 * connected_point_x * x + x * x;
        vandermonde_mtx(i, 5) = 2 * std::pow(connected_point_x, 3) -
                                3 * std::pow(connected_point_x, 2) * x +
                                std::pow(x, 3);
    }

    float weight = 1 * vandermonde_mtx.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    result =
        (vandermonde_mtx.transpose() * vandermonde_mtx + weight * I).inverse() *
        (weight * base_param + vandermonde_mtx.transpose() * sample_y);

    float mse_thresh = 0.5;
    if (IsLeastSquareFitValid(sample_y, vandermonde_mtx, result, mse_thresh))
    {
        geometry->is_curve_valid = true;

        geometry->curve.c0 = result(0) -
                             result(2) * std::pow(connected_point_x, 2) -
                             2 * result(3) * std::pow(connected_point_x, 3) +
                             result(4) * std::pow(connected_point_x, 2) +
                             2 * result(5) * std::pow(connected_point_x, 3);

        geometry->curve.c1 = result(1) + 2 * result(2) * connected_point_x +
                             3 * result(3) * std::pow(connected_point_x, 2) -
                             2 * result(4) * connected_point_x -
                             3 * result(5) * std::pow(connected_point_x, 2);

        geometry->curve.c2 = result(4);
        geometry->curve.c3 = result(5);
        geometry->curve.lon_dist_start = geometry->sampling_points.at(0).x;
        geometry->curve.lon_dist_end =
            geometry->sampling_points.at(count - 1).x;
    }
    else
    {
        geometry->is_curve_valid = false;
    }
}

void FitterLeastSquareBasedLaneContinuityMethod(
    GeometryLine *far_geometry, GeometryLine *near_geometry,
    const std::deque<SamplingPoint> &points, size_t connect_point_idx)
{
    far_geometry->is_curve_valid = false;
    near_geometry->is_curve_valid = false;

    static size_t min_fit_points_size = 4;
    if (points.size() < min_fit_points_size) return;

    if (connect_point_idx < min_fit_points_size)
    {
        auto fit_result =
            FitterLeastSquareMethod(points.begin(), points.end(), 3);
        if (fit_result.rows() != 0)
        {
            near_geometry->is_curve_valid = true;
            near_geometry->curve.c0 = fit_result(0);
            near_geometry->curve.c1 = fit_result(1);
            near_geometry->curve.c2 = fit_result(2);
            near_geometry->curve.c3 = fit_result(3);
            near_geometry->curve.lon_dist_start = points.front().x;
            near_geometry->curve.lon_dist_end = std::min(points.back().x, 0.0f);
        }
        return;
    }

    float connected_point_x = points[connect_point_idx].x;
    Eigen::VectorXd result;
    std::vector<double> raw_data_x(points.size());
    std::vector<double> raw_data_y(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        raw_data_x[i] = points[i].x;
        raw_data_y[i] = points[i].y;
    }


    Eigen::Map<Eigen::VectorXd> sample_x(raw_data_x.data(), points.size());
    Eigen::Map<Eigen::VectorXd> sample_y(raw_data_y.data(), points.size());

    Eigen::MatrixXd vandermonde_mtx = Eigen::MatrixXd::Zero(points.size(), 6);

    vandermonde_mtx.col(0) = Eigen::VectorXd::Constant(points.size(), 1, 1);
    for (size_t i = 1; i < 4; ++i)
    {
        vandermonde_mtx.col(i) =
            vandermonde_mtx.col(i - 1).array() * sample_x.array();
    }

    for (size_t i = connect_point_idx; i < points.size(); ++i)
    {
        double x = sample_x(i);
        vandermonde_mtx(i, 2) =
            -std::pow(connected_point_x, 2) + 2 * connected_point_x * x;
        vandermonde_mtx(i, 3) = 3 * std::pow(connected_point_x, 2) * x -
                                2 * std::pow(connected_point_x, 3);
        vandermonde_mtx(i, 4) =
            std::pow(connected_point_x, 2) - 2 * connected_point_x * x + x * x;
        vandermonde_mtx(i, 5) = 2 * std::pow(connected_point_x, 3) -
                                3 * std::pow(connected_point_x, 2) * x +
                                std::pow(x, 3);
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    result = vandermonde_mtx.householderQr().solve(sample_y);

    float mse_thresh = 0.5;
    if (IsLeastSquareFitValid(sample_y, vandermonde_mtx, result, mse_thresh))
    {
        near_geometry->is_curve_valid = true;
        near_geometry->curve.c0 =
            result(0) - result(2) * std::pow(connected_point_x, 2) -
            2 * result(3) * std::pow(connected_point_x, 3) +
            result(4) * std::pow(connected_point_x, 2) +
            2 * result(5) * std::pow(connected_point_x, 3);
        near_geometry->curve.c1 =
            result(1) + 2 * result(2) * connected_point_x +
            3 * result(3) * std::pow(connected_point_x, 2) -
            2 * result(4) * connected_point_x -
            3 * result(5) * std::pow(connected_point_x, 2);
        near_geometry->curve.c2 = result(4);
        near_geometry->curve.c3 = result(5);
        near_geometry->curve.lon_dist_start = connected_point_x;
        near_geometry->curve.lon_dist_end = points.back().x;

        far_geometry->is_curve_valid = true;
        far_geometry->curve.c0 = result(0);
        far_geometry->curve.c1 = result(1);
        far_geometry->curve.c2 = result(2);
        far_geometry->curve.c3 = result(3);
        far_geometry->curve.lon_dist_start = points.front().x;
        far_geometry->curve.lon_dist_end = connected_point_x;
    }
    else
    {
        far_geometry->is_curve_valid = false;
        near_geometry->is_curve_valid = false;
    }
}

bool IsLeastSquareFitValid(const Eigen::VectorXd &Y, const Eigen::MatrixXd &A,
                           const Eigen::VectorXd &X, float mse_thresh)
{
    if (A.rows() != Y.rows() || A.cols() != X.rows()) return false;

    for (size_t i = 0; i < X.rows(); ++i)
    {
        if (std::isnan(X(i))) return false;
    }

    Eigen::VectorXd error = Y - A * X;
    double sum_squared_error = error.transpose() * error;
    double mse = sum_squared_error / error.rows();

    if (mse > mse_thresh * mse_thresh) return false;

    return true;
}

} // namespace fusion
} // namespace cem
