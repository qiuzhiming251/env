#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "common/filter/base_filter.h"

namespace cem {
namespace fusion {

class KalmanFilter : public BaseFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    KalmanFilter() = default;
    ~KalmanFilter() = default;

    bool Init(const Eigen::VectorXd &global_states,
              const Eigen::MatrixXd &global_uncertainty) override;

    bool Predict(const Eigen::MatrixXd &transform_matrix,
                 const Eigen::MatrixXd &env_uncertainty_matrix) override;


    bool Correct(const Eigen::VectorXd &cur_observation,
                 const Eigen::MatrixXd &cur_observation_uncertainty) override;


    bool SetControlMatrix(const Eigen::MatrixXd &control_matrix) override;


    inline Eigen::VectorXd GetStates() const override { return global_states_; }

    inline void SetStates(const Eigen::VectorXd &states) override
    {
        global_states_ = states;
    }


    inline Eigen::MatrixXd GetUncertainty() const override
    {
        return global_uncertainty_;
    }

    bool DeCorrelation(int x, int y, int x_len, int y_len) override;
    void CorrectionBreakdown() override;
    bool SetGainBreakdownThresh(const std::vector<bool> &break_down,
                                const float threshold = 2.0f) override;
    bool SetValueBreakdownThresh(const std::vector<bool> &break_down,
                                 const float threshold = 0.05f) override;

    std::string Name() const override { return "KalmanFilter"; }

protected:
    Eigen::VectorXd prior_global_states_; 
    Eigen::VectorXd gain_break_down_;     
    Eigen::VectorXd value_break_down_;    
    Eigen::MatrixXd kalman_gain_;         

    float value_break_down_threshold_ = 999.0f;
    float gain_break_down_threshold_ = 0.0f;
};

} // namespace fusion
} // namespace cem

#endif
