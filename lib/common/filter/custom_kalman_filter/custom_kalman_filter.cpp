#include "custom_kalman_filter.h"

namespace cem {
namespace fusion {


bool CustomKalmanFilter::Predict(const Eigen::MatrixXd &transform_matrix,
                                 const Eigen::MatrixXd &env_uncertainty_matrix)
{
    if (!init_)
    {

        return false;
    }
    if (transform_matrix.rows() != states_num_)
    {

        return false;
    }
    if (transform_matrix.cols() != states_num_)
    {

        return false;
    }
    if (env_uncertainty_matrix.rows() != states_num_)
    {

        return false;
    }
    if (env_uncertainty_matrix.cols() != states_num_)
    {

        return false;
    }
    transform_matrix_ = transform_matrix;
    env_uncertainty_ = env_uncertainty_matrix;

    global_uncertainty_ = transform_matrix_ * global_uncertainty_ *
                              transform_matrix_.transpose() +
                          env_uncertainty_;

    return true;
}

bool CustomKalmanFilter::Correct(
    const Eigen::VectorXd &cur_observation,
    const Eigen::MatrixXd &cur_observation_uncertainty)
{
    if (!init_)
    {

        return false;
    }
    if (cur_observation.rows() != states_num_)
    {

        return false;
    }
    if (cur_observation_uncertainty.rows() != states_num_)
    {

        return false;
    }
    if (cur_observation_uncertainty.cols() != states_num_)
    {

        return false;
    }

    cur_observation_ = cur_observation;
    cur_observation_uncertainty_ = cur_observation_uncertainty;


    kalman_gain_ = global_uncertainty_ * c_matrix_.transpose() *
                   (c_matrix_ * global_uncertainty_ * c_matrix_.transpose() +
                    cur_observation_uncertainty_)
                       .inverse();

    global_states_ =
        global_states_ +
        kalman_gain_ * (cur_observation_ - c_matrix_ * global_states_);

    Eigen::MatrixXd tmp_identity;
    tmp_identity.setIdentity(states_num_, states_num_);
    global_uncertainty_ =
        (tmp_identity - kalman_gain_ * c_matrix_) * global_uncertainty_;

    return true;
}

FUSION_REGISTER_FILTER(CustomKalmanFilter)

} // namespace fusion
} // namespace cem
