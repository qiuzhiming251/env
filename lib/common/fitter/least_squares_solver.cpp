#include "least_squares_solver.h"
namespace cem {
namespace fusion {
LeastSquaresSolver::LeastSquaresSolver()
    : number_of_rows_(0),
      number_of_cols_(0),
      initialization_done_(false),
      calculation_done_(false),
      regularized_parameter_(0.0)
{
}

void LeastSquaresSolver::InitializeSolver(const std::int32_t num_of_rows,
                                          const std::int32_t num_of_cols)
{
    number_of_rows_ = num_of_rows;
    number_of_cols_ = num_of_cols;
    A_.setZero(num_of_rows, num_of_cols);
    b_.setZero(num_of_rows, 1);
    W_.setOnes(num_of_rows, 1);
    x_.setZero(num_of_cols, 1);
    x_last_.setZero(num_of_cols, 1);
    U_.setZero(num_of_cols, num_of_cols);
    initialization_done_ = true;
}

void LeastSquaresSolver::AddElementToA(const std::int32_t row_index,
                                       const std::int32_t col_index,
                                       const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_) ||
        (col_index >= number_of_cols_))
    {

    }
    else
    {
        A_(row_index, col_index) = value;
    }

    return;
}

void LeastSquaresSolver::AddElementToB(const std::int32_t row_index,
                                       const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_))
    {

    }
    else
    {
        b_(row_index) = value;
    }

    return;
}

void LeastSquaresSolver::AddElementToW(const std::int32_t row_index,
                                       const float value)
{
    if ((initialization_done_ == false) || (row_index >= number_of_rows_))
    {

    }
    else
    {
        W_(row_index) = value;
    }

    return;
}

void LeastSquaresSolver::AddElementToXLast(const std::int32_t col_index,
                                           const float value)
{
    if ((initialization_done_ == false) || (col_index >= number_of_cols_))
    {

    }
    else
    {
        x_last_(col_index) = value;
    }

    return;
}

void LeastSquaresSolver::AddElementToU(const std::int32_t col_index,
                                       const float value)
{
    if ((initialization_done_ == false) || (col_index >= number_of_cols_))
    {

    }
    else
    {
        U_(col_index, col_index) = value;
    }

    return;
}

float LeastSquaresSolver::GetX(const std::int32_t index)
{
    if ((calculation_done_ == false) || (index >= number_of_cols_))
    {
        return 0.0;

    }
    return x_(index);
}

void LeastSquaresSolver::CalculateX()
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> At = A_.transpose();
    for (int i = 0; i < number_of_rows_; ++i)
    {
        At.col(i) *= W_(i);
    }

    auto AtA = At * A_ + U_;
    auto Atb = At * b_ + U_ * x_last_;


    x_ = AtA.llt().solve(Atb);


    calculation_done_ = true;
};
} // namespace fusion
} // namespace cem
