#include "kalman_filter.h"

namespace cem {
namespace fusion {


bool KalmanFilter::Init(const Eigen::VectorXd &initial_belief_states,
                        const Eigen::MatrixXd &initial_uncertainty)
{
    if (initial_uncertainty.rows() != initial_uncertainty.cols())
    {
        return false;
    }
    states_num_ = static_cast<int>(initial_uncertainty.rows());

    if (states_num_ <= 0)
    {
        return false;
    }

    if (states_num_ != initial_belief_states.rows())
    {
        return false;
    }

    global_states_ = initial_belief_states;
    global_uncertainty_ = initial_uncertainty;
    prior_global_states_ = global_states_;

    transform_matrix_.setIdentity(states_num_, states_num_);
    cur_observation_.setZero(states_num_, 1);
    cur_observation_uncertainty_.setIdentity(states_num_, states_num_);

    c_matrix_.setIdentity(states_num_, states_num_);
    env_uncertainty_.setZero(states_num_, states_num_);

    gain_break_down_.setZero(states_num_, 1);
    value_break_down_.setZero(states_num_, 1);

    kalman_gain_.setZero(states_num_, states_num_);
    init_ = true;
    return true;
}


bool KalmanFilter::Predict(const Eigen::MatrixXd &transform_matrix,
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
    global_states_ = transform_matrix_ * global_states_;
    global_uncertainty_ = transform_matrix_ * global_uncertainty_ *
                              transform_matrix_.transpose() +
                          env_uncertainty_;
    return true;
}

bool KalmanFilter::Correct(const Eigen::VectorXd &cur_observation,
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
        (tmp_identity - kalman_gain_ * c_matrix_) * global_uncertainty_ *
            (tmp_identity - kalman_gain_ * c_matrix_).transpose() +
        kalman_gain_ * cur_observation_uncertainty_ * kalman_gain_.transpose();
    return true;
}

bool KalmanFilter::SetControlMatrix(const Eigen::MatrixXd &control_matrix)
{
    if (!init_)
    {

        return false;
    }
    if (control_matrix.rows() != states_num_ ||
        control_matrix.cols() != states_num_)
    {

        return false;
    }
    c_matrix_ = control_matrix;
    return true;
}

bool KalmanFilter::SetGainBreakdownThresh(const std::vector<bool> &break_down,
                                          const float threshold)
{
    if (static_cast<int>(break_down.size()) != states_num_)
    {
        return false;
    }
    for (int i = 0; i < states_num_; i++)
    {
        if (break_down[i])
        {
            gain_break_down_(i) = 1;
        }
    }
    gain_break_down_threshold_ = threshold;
    return true;
}

bool KalmanFilter::SetValueBreakdownThresh(const std::vector<bool> &break_down,
                                           const float threshold)
{
    if (static_cast<int>(break_down.size()) != states_num_)
    {
        return false;
    }
    for (int i = 0; i < states_num_; i++)
    {
        if (break_down[i])
        {
            value_break_down_(i) = 1;
        }
    }
    value_break_down_threshold_ = threshold;
    return true;
}
void KalmanFilter::CorrectionBreakdown()
{
    Eigen::VectorXd states_gain = global_states_ - prior_global_states_;
    Eigen::VectorXd breakdown_diff = states_gain.cwiseProduct(gain_break_down_);
    global_states_ -= breakdown_diff;
    if (breakdown_diff.norm() > gain_break_down_threshold_)
    {
        breakdown_diff.normalize();
        breakdown_diff *= gain_break_down_threshold_;
    }
    global_states_ += breakdown_diff;

    Eigen::VectorXd temp;
    temp.setOnes(states_num_, 1);
    if ((global_states_.cwiseProduct(value_break_down_)).norm() <
        value_break_down_threshold_)
    {
        global_states_ = global_states_.cwiseProduct(temp - value_break_down_);
    }
    prior_global_states_ = global_states_;
}

bool KalmanFilter::DeCorrelation(int x, int y, int x_len, int y_len)
{
    if (x >= states_num_ || y >= states_num_ || x + x_len >= states_num_ ||
        y + y_len >= states_num_)
    {
        return false;
    }
    for (int i = 0; i < x_len; i++)
    {
        for (int j = 0; j < y_len; j++)
        {
            global_uncertainty_(x + i, y + j) = 0;
        }
    }
    return true;
}

FUSION_REGISTER_FILTER(KalmanFilter)

} // namespace fusion
} // namespace cem
