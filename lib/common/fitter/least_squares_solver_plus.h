#ifndef LEAST_SQUARES_SOLVER_PLUS_
#define LEAST_SQUARES_SOLVER_PLUS_

#include "least_squares_solver.h"
namespace cem {
namespace fusion {
class LeastSquaresSolverPlus : public LeastSquaresSolver
{
public:
    LeastSquaresSolverPlus();
    ~LeastSquaresSolverPlus() = default;

    void InitializeSolver(const std::int32_t num_of_rows,
                          const std::int32_t num_of_cols,
                          const std::int32_t num_of_rows_A2);
    void CalculateX();

    void AddElementToA2(const std::int32_t row_index,
                        const std::int32_t col_index, const float value);
    void AddElementToB2(const std::int32_t row_index, const float value);
    void AddElementToW2(const std::int32_t row_col_index, const float value);

private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A2_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b2_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> U2_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> W2_;

    int32_t number_of_rows_A2_;
    float regularized_parameter_U2_;
};
} // namespace fusion
} // namespace cem
#endif