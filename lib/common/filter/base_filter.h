#ifndef BASE_FILTER_H_
#define BASE_FILTER_H_

#include "common/utility.h"

namespace cem {
namespace fusion {

// @brief base filter inference
class BaseFilter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    // @brief constructor
    BaseFilter() = default;

    // @brief destructor
    virtual ~BaseFilter() = default;

    /**
     * @brief Initialize filter.
     *
     * @param global_states A vector contains system states.
     * @param global_uncertainty A covariance matrix which indicate the
     *        uncertainty of each system state.
     * @return bool Whether initialization is successful?
     */
    virtual bool Init(const Eigen::VectorXd &global_states,
                      const Eigen::MatrixXd &global_uncertainty) = 0;

    /**
     * @brief Predict the current state and uncertainty of system.
     *
     * @param transform_matrix Transform the state from the pre moment to
     *        current moment.
     * @param env_uncertainty_matrix The uncertainty brought by the
     *        environment when predict.
     * @return bool Whether prediction is successful?
     */
    virtual bool Predict(const Eigen::MatrixXd &transform_matrix,
                         const Eigen::MatrixXd &env_uncertainty_matrix) = 0;

    // @brief use the current observation to correct the predict
    // @params[IN] cur_observation: the observation in current time
    // @params[IN] cur_observation_uncertainty: the uncertainty of
    //             the observation
    virtual bool Correct(
        const Eigen::VectorXd &cur_observation,
        const Eigen::MatrixXd &cur_observation_uncertainty) = 0;

    // @brief set the control matrix
    virtual bool SetControlMatrix(const Eigen::MatrixXd &c_matrix) = 0;

    // @brief get the system states
    virtual Eigen::VectorXd GetStates() const = 0;

    virtual void SetStates(const Eigen::VectorXd &states) = 0;

    // @brief get the belief uncertainty
    virtual Eigen::MatrixXd GetUncertainty() const = 0;

    virtual bool DeCorrelation(int x, int y, int x_len, int y_len) = 0;
    virtual void CorrectionBreakdown() = 0;
    virtual bool SetGainBreakdownThresh(const std::vector<bool> &break_down,
                                        const float threshold = 2.0f) = 0;
    virtual bool SetValueBreakdownThresh(const std::vector<bool> &break_down,
                                         const float threshold = 0.05f) = 0;

    // @brief get the name of the filter
    virtual std::string Name() const = 0;

protected:
    // @brief whether the filter has been init
    bool init_ = false;

    // @brief the number of the system states
    int states_num_ = 0;

    Eigen::MatrixXd transform_matrix_;            // F
    Eigen::VectorXd global_states_;               // X
    Eigen::MatrixXd global_uncertainty_;          // P
    Eigen::MatrixXd env_uncertainty_;             // Q
    Eigen::MatrixXd cur_observation_;             // Z
    Eigen::MatrixXd cur_observation_uncertainty_; // R
    Eigen::MatrixXd c_matrix_;                    // H
};

typedef std::unique_ptr<BaseFilter> BaseFilterPtr;
typedef std::unique_ptr<const BaseFilter> BaseFilterConstPtr;

PERCEPTION_REGISTER_REGISTERER(BaseFilter);
#define FUSION_REGISTER_FILTER(name) PERCEPTION_REGISTER_CLASS(BaseFilter, name)

} // namespace fusion
} // namespace cem

#endif
