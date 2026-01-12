#ifndef LEAST_SQUARES_SOLVER_
#define LEAST_SQUARES_SOLVER_

#include <cstdint>
#include <Eigen/Dense>
#include <iostream>
namespace cem {
namespace fusion {

class LeastSquaresSolver
{
public:

    LeastSquaresSolver();
    void AddElementToA(const std::int32_t row_index,
                       const std::int32_t col_index, const float value);
    void AddElementToB(const std::int32_t row_index, const float value);
    void AddElementToW(const std::int32_t row_index, const float value);
    void AddElementToXLast(const std::int32_t row_index, const float value);
    void AddElementToU(const std::int32_t col_index, const float value);
    float GetX(const std::int32_t index);
    inline void SetRegularizedParam(const float value)
    {
        regularized_parameter_ = value;
        U_ = U_ * regularized_parameter_;
    }
    inline std::int32_t GetNumberOfRows() const { return number_of_rows_; }
    inline std::int32_t GetNumberOfCols() const { return number_of_cols_; }


    void InitializeSolver(const std::int32_t number_of_rows,
                          const std::int32_t number_of_cols);
    virtual void CalculateX();
    virtual ~LeastSquaresSolver() = default;

protected:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> U_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> W_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x_last_;


    int32_t number_of_rows_; 
    int32_t number_of_cols_; 
    bool initialization_done_;
    bool calculation_done_;

    float regularized_parameter_;

private:
};
} // namespace fusion
} // namespace cem
#endif