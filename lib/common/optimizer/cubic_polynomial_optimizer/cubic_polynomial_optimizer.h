#ifndef CUBIC_POLYNOMIAL_OPTIMIZER_H_
#define CUBIC_POLYNOMIAL_OPTIMIZER_H_

#include "constraints/constraints_headers.h"

namespace cem {
namespace fusion {

class CubicPolynomialOptimizer
{
private:
    struct ProblemFastRemoveOptions : public ceres::Problem::Options
    {
        ProblemFastRemoveOptions() { enable_fast_removal = true; }
    };

    bool is_inited_ = false;
    double* curve_params_ = nullptr;
    size_t params_num_ = 0;
    std::vector<double*> parameter_blocks_;
    ceres::Problem problem_;
    ceres::Solver::Options solver_options_;
    ceres::Solver::Summary solve_summary_;

public:
    CubicPolynomialOptimizer();
    CubicPolynomialOptimizer(size_t params_num);
    ~CubicPolynomialOptimizer() { Reset(); }

public:
    template <typename ConstraintType, typename... Args>
    void AddConstraint(Args&&... args)
    {
        if (!is_inited_) return;

        auto cost_function =
            ConstraintType::Create(std::forward<Args>(args)...);
        if (cost_function != nullptr)
        {
            problem_.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1f),
                                      parameter_blocks_);
        }
    }

    void Init(size_t params_num);

    bool Solve();

    inline ceres::Solver::Options* MutableSolverOptions()
    {
        return &solver_options_;
    }

    inline ceres::Solver::Summary& GetSolveSummary() { return solve_summary_; }

    inline const double* GetOptimizationResult() { return curve_params_; }

private:
    void Reset();
};

} // namespace fusion
} // namespace cem

#endif
