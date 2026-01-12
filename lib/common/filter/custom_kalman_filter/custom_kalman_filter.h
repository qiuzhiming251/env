#ifndef CUSTOM_KALMAN_FILTER_H_
#define CUSTOM_KALMAN_FILTER_H_

#include "common/filter/kalman_filter/kalman_filter.h"

namespace cem {
namespace fusion {

class CustomKalmanFilter : public KalmanFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    CustomKalmanFilter() = default;
    ~CustomKalmanFilter() = default;

    bool Predict(const Eigen::MatrixXd &transform_matrix,
                 const Eigen::MatrixXd &env_uncertainty_matrix) override;

    bool Correct(const Eigen::VectorXd &cur_observation,
                 const Eigen::MatrixXd &cur_observation_uncertainty) override;

    std::string Name() const override { return "CustomKalmanFilter"; }
};

} // namespace fusion
} // namespace cem

#endif
