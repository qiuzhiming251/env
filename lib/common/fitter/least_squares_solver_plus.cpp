#include "least_squares_solver_plus.h"
namespace cem {
namespace fusion {
LeastSquaresSolverPlus::LeastSquaresSolverPlus()
    : LeastSquaresSolver(),
      number_of_rows_A2_(0),
      regularized_parameter_U2_(0.0)
{
}

void LeastSquaresSolverPlus::InitializeSolver(const std::int32_t num_of_rows,
                                              const std::int32_t num_of_cols,
                                              const std::int32_t num_of_rows_A2)
{
    number_of_rows_ = num_of_rows;
    number_of_cols_ = num_of_cols;
    number_of_rows_A2_ = num_of_rows_A2;
    A_.setZero(num_of_rows, num_of_cols);
    b_.setZero(num_of_rows, 1);
    x_.setZero(num_of_cols, 1);
    x_last_.setZero(num_of_cols, 1);
    U_.setIdentity(num_of_cols, num_of_cols);
    W_.setIdentity(num_of_rows, 1);

    A2_.setZero(num_of_rows_A2, num_of_cols);
    b2_.setZero(num_of_rows_A2, 1);
    U2_.setIdentity(num_of_cols, num_of_cols);
    W2_.setZero(num_of_rows_A2, 1);

    initialization_done_ = true;
}

void LeastSquaresSolverPlus::AddElementToA2(const std::int32_t row_index,
                                            const std::int32_t col_index,
                                            const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_A2_) ||
        (col_index >= number_of_cols_))
    {

    }
    else
    {
        A2_(row_index, col_index) = value;
    }

    return;
}

void LeastSquaresSolverPlus::AddElementToB2(const std::int32_t row_index,
                                            const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_A2_))
    {

    }
    else
    {
        b2_(row_index) = value;
    }

    return;
}

void LeastSquaresSolverPlus::AddElementToW2(const std::int32_t row_index,
                                            const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_A2_))
    {

    }
    else
    {
        W2_(row_index) = value;
    }

    return;
}

void LeastSquaresSolverPlus::CalculateX()
{

    if (number_of_rows_A2_ > 0)
    {
        float u2_param = float(number_of_rows_) / float(number_of_rows_A2_);
        U2_ = U2_ * u2_param; 
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A2t = A2_.transpose();
    for (int i = 0; i < number_of_rows_A2_; ++i)
    {
        A2t.col(i) *= W2_(i);
    }
    auto A2tA2 = A2t * A2_;
    auto A2tb2 = A2t * b2_;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> At = A_.transpose();
    for (int i = 0; i < number_of_rows_; ++i)
    {
        At.col(i) *= W_(i);
    }

    auto AtA = At * A_ + U2_ * A2tA2 + U_;
    auto Atb = At * b_ + U2_ * A2tb2 + U_ * x_last_;

    x_ = AtA.llt().solve(Atb);

    auto cost_gtr = A_ * x_ - b_;
    auto cost_re = A2_ * x_ - b2_;
    auto cost_last = x_ - x_last_;



    calculation_done_ = true;
};
} // namespace fusion
} // namespace cem