#include "cubic_polynomial_optimizer.h"

namespace cem {
namespace fusion {

CubicPolynomialOptimizer::CubicPolynomialOptimizer()
    : problem_(ProblemFastRemoveOptions())
{
    solver_options_.linear_solver_type = ceres::DENSE_QR;
    solver_options_.max_num_iterations = 20;
    solver_options_.function_tolerance = 0.05d;
    solver_options_.minimizer_progress_to_stdout = false;
}

CubicPolynomialOptimizer::CubicPolynomialOptimizer(size_t params_num)
    : CubicPolynomialOptimizer()
{
    Init(params_num);
}

void CubicPolynomialOptimizer::Init(size_t params_num)
{
    Reset();
    params_num_ = params_num;

    curve_params_ = new double[params_num_]();

    parameter_blocks_.emplace_back(curve_params_);
    is_inited_ = true;
}

bool CubicPolynomialOptimizer::Solve()
{
    if (is_inited_)
    {
        ceres::Solve(solver_options_, &problem_, &solve_summary_);

        if ((solve_summary_.num_unsuccessful_steps > 0 &&
             solve_summary_.num_successful_steps == 0) ||
            solve_summary_.final_cost == solve_summary_.initial_cost)
        {
            return false;
        }

        for (size_t i = 0; i < params_num_; ++i)
        {
            if (std::isnan(curve_params_[i])) return false;
        }

        return true;
    }
    return false;
}

void CubicPolynomialOptimizer::Reset()
{
    for (auto parameter_block : parameter_blocks_)
    {
        if (problem_.HasParameterBlock(parameter_block))
            problem_.RemoveParameterBlock(parameter_block);
    }

    parameter_blocks_.clear();

    if (curve_params_ != nullptr)
    {
        delete[] curve_params_;
        curve_params_ = nullptr;
    }

    params_num_ = 0;
    is_inited_ = false;
}

} // namespace fusion
} // namespace cem
