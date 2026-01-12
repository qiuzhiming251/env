#ifndef HUNGARIAN_OPTIMIZER_H_
#define HUNGARIAN_OPTIMIZER_H_

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "secure_matrix.h"

namespace cem {
namespace fusion {

template <typename T>
class HungarianOptimizer
{
    static const int kHungarianOptimizerRowNotFound = -1;
    static const int kHungarianOptimizerColNotFound = -2;

public:
    HungarianOptimizer();
    explicit HungarianOptimizer(const int max_optimization_size);
    ~HungarianOptimizer() {}

    SecureMat<T>* costs() { return &costs_; }

    T* costs(const size_t row, const size_t col) { return &(costs_(row, col)); }

    void Maximize(std::vector<std::pair<size_t, size_t>>* assignments);
    void Minimize(std::vector<std::pair<size_t, size_t>>* assignments);

    void OptimizationInit();
    void OptimizationClear();

    void PrintMatrix();

private:
    enum class Mark
    {
        NONE,
        PRIME,
        STAR
    };

    void FindAssignments(std::vector<std::pair<size_t, size_t>>* assignments);

    bool IsStarred(const size_t row, const size_t col) const
    {
        return marks_(row, col) == Mark::STAR;
    }

    void Star(const size_t row, const size_t col)
    {
        marks_(row, col) = Mark::STAR;
        ++stars_in_col_[col];
    }

    void Unstar(const size_t row, const size_t col)
    {
        marks_(row, col) = Mark::NONE;
        --stars_in_col_[col];
    }

    int FindStarInRow(const size_t row) const;

    int FindStarInCol(const size_t col) const;

    bool IsPrimed(const size_t row, const size_t col) const
    {
        return marks_(row, col) == Mark::PRIME;
    }

    void Prime(const size_t row, const size_t col)
    {
        marks_(row, col) = Mark::PRIME;
    }

    int FindPrimeInRow(const size_t row) const;

    void ClearPrimes();

    bool ColContainsStar(const size_t col) const
    {
        return stars_in_col_[col] > 0;
    }

    bool RowCovered(const size_t row) const { return rows_covered_[row]; }


    void CoverRow(const size_t row) { rows_covered_[row] = true; }


    void UncoverRow(const size_t row) { rows_covered_[row] = false; }


    bool ColCovered(const size_t col) const { return cols_covered_[col]; }


    void CoverCol(const size_t col) { cols_covered_[col] = true; }


    void UncoverCol(const size_t col) { cols_covered_[col] = false; }


    void ClearCovers();


    T FindSmallestUncovered();


    bool FindZero(size_t* zero_row, size_t* zero_col);


    void DoMunkres();

    void CheckStar();


    void ReduceRows();


    void StarZeroes();


    void CoverStarredZeroes();


    void PrimeZeroes();



    void MakeAugmentingPath();


    void AugmentPath();


    int max_optimization_size_ = 1000;


    bool optimization_initialized_ = false;


    size_t matrix_size_ = 0;


    SecureMat<T> costs_;


    T max_cost_{0};


    std::vector<bool> rows_covered_;
    std::vector<bool> cols_covered_;


    SecureMat<Mark> marks_;


    std::vector<int> stars_in_col_;


    std::vector<std::pair<size_t, size_t>> assignments_;


    int zero_col_ = 0;
    int zero_row_ = 0;


    size_t width_ = 0;
    size_t height_ = 0;


    std::function<void()> fn_state_ = nullptr;

    std::vector<size_t> uncov_col_;
    std::vector<size_t> uncov_row_;
}; 

template <typename T>
HungarianOptimizer<T>::HungarianOptimizer() : HungarianOptimizer(1000)
{
}

template <typename T>
HungarianOptimizer<T>::HungarianOptimizer(const int max_optimization_size)
    : max_optimization_size_(max_optimization_size)
{
    costs_.Reserve(max_optimization_size, max_optimization_size);
    stars_in_col_.reserve(max_optimization_size);
    rows_covered_.reserve(max_optimization_size);
    cols_covered_.reserve(max_optimization_size);
    assignments_.reserve(max_optimization_size);
    uncov_row_.reserve(max_optimization_size);
    uncov_col_.reserve(max_optimization_size);
}


template <typename T>
void HungarianOptimizer<T>::Maximize(
    std::vector<std::pair<size_t, size_t>>* assignments)
{
    OptimizationInit();

    for (size_t row = 0; row < height_; ++row)
    {
        for (size_t col = 0; col < width_; ++col)
        {
            costs_(row, col) = max_cost_ - costs_(row, col);
        }
    }
    Minimize(assignments);
}


template <typename T>
void HungarianOptimizer<T>::Minimize(
    std::vector<std::pair<size_t, size_t>>* assignments)
{
    OptimizationInit();
    DoMunkres();
    FindAssignments(assignments);
    OptimizationClear();
}

template <typename T>
void HungarianOptimizer<T>::OptimizationInit()
{
    if (optimization_initialized_)
    {
        return;
    }
    width_ = costs_.width();
    if (width_ > 0)
    {
        height_ = costs_.height();
    }
    else
    {
        height_ = 0;
    }

    matrix_size_ = std::max(height_, width_);
    max_cost_ = 0;


    costs_.Resize(matrix_size_, matrix_size_);
    for (size_t row = 0; row < matrix_size_; ++row)
    {
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            if ((row >= height_) || (col >= width_))
            {
                costs_(row, col) = 0;
            }
            else
            {
                max_cost_ = std::max(max_cost_, costs_(row, col));
            }
        }
    }


    marks_.Resize(matrix_size_, matrix_size_);
    for (size_t row = 0; row < matrix_size_; ++row)
    {
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            marks_(row, col) = Mark::NONE;
        }
    }

    stars_in_col_.assign(matrix_size_, 0);

    rows_covered_.assign(matrix_size_, false);
    cols_covered_.assign(matrix_size_, false);

    assignments_.resize(matrix_size_ * 2);

    optimization_initialized_ = true;
}

template <typename T>
void HungarianOptimizer<T>::OptimizationClear()
{
    optimization_initialized_ = false;
}


template <typename T>
void HungarianOptimizer<T>::FindAssignments(
    std::vector<std::pair<size_t, size_t>>* assignments)
{
    assignments->clear();
    for (size_t row = 0; row < height_; ++row)
    {
        for (size_t col = 0; col < width_; ++col)
        {
            if (IsStarred(row, col))
            {
                assignments->push_back(std::make_pair(row, col));
                break;
            }
        }
    }
}


template <typename T>
int HungarianOptimizer<T>::FindStarInRow(const size_t row) const
{
    for (size_t col = 0; col < matrix_size_; ++col)
    {
        if (IsStarred(row, col))
        {
            return static_cast<int>(col);
        }
    }

    return kHungarianOptimizerColNotFound;
}


template <typename T>
int HungarianOptimizer<T>::FindStarInCol(const size_t col) const
{
    if (!ColContainsStar(col))
    {
        return kHungarianOptimizerRowNotFound;
    }

    for (size_t row = 0; row < matrix_size_; ++row)
    {
        if (IsStarred(row, col))
        {
            return static_cast<int>(row);
        }
    }


    return kHungarianOptimizerRowNotFound;
}


template <typename T>
int HungarianOptimizer<T>::FindPrimeInRow(const size_t row) const
{
    for (size_t col = 0; col < matrix_size_; ++col)
    {
        if (IsPrimed(row, col))
        {
            return static_cast<int>(col);
        }
    }

    return kHungarianOptimizerColNotFound;
}


template <typename T>
void HungarianOptimizer<T>::ClearPrimes()
{
    for (size_t row = 0; row < matrix_size_; ++row)
    {
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            if (IsPrimed(row, col))
            {
                marks_(row, col) = Mark::NONE;
            }
        }
    }
}


template <typename T>
void HungarianOptimizer<T>::ClearCovers()
{
    for (size_t x = 0; x < matrix_size_; x++)
    {
        UncoverRow(x);
        UncoverCol(x);
    }
}


template <typename T>
T HungarianOptimizer<T>::FindSmallestUncovered()
{
    T minval = std::numeric_limits<T>::max();
    uncov_col_.clear();
    uncov_row_.clear();

    for (size_t i = 0; i < matrix_size_; ++i)
    {
        if (!RowCovered(i))
        {
            uncov_row_.push_back(i);
        }
        if (!ColCovered(i))
        {
            uncov_col_.push_back(i);
        }
    }

    for (size_t row = 0; row < uncov_row_.size(); ++row)
    {
        for (size_t col = 0; col < uncov_col_.size(); ++col)
        {
            minval = std::min(minval, costs_(uncov_row_[row], uncov_col_[col]));
        }
    }

    return minval;
}


template <typename T>
bool HungarianOptimizer<T>::FindZero(size_t* zero_row, size_t* zero_col)
{
    uncov_col_.clear();
    uncov_row_.clear();

    for (size_t i = 0; i < matrix_size_; ++i)
    {
        if (!RowCovered(i))
        {
            uncov_row_.push_back(i);
        }
        if (!ColCovered(i))
        {
            uncov_col_.push_back(i);
        }
    }
    if (uncov_row_.empty() || uncov_col_.empty())
    {
        return false;
    }

    for (size_t i = 0; i < uncov_row_.size(); ++i)
    {
        for (size_t j = 0; j < uncov_col_.size(); ++j)
        {
            if (costs_(uncov_row_[i], uncov_col_[j]) == 0)
            {
                *zero_row = uncov_row_[i];
                *zero_col = uncov_col_[j];
                return true;
            }
        }
    }
    return false;
}


template <typename T>
void HungarianOptimizer<T>::PrintMatrix()
{
    for (size_t row = 0; row < matrix_size_; ++row)
    {
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            printf("%g ", costs_(row, col));

            if (IsStarred(row, col))
            {
                printf("*");
            }

            if (IsPrimed(row, col))
            {
                printf("'");
            }
        }
        printf("\n");
    }
}


template <typename T>
void HungarianOptimizer<T>::DoMunkres()
{
    int max_num_iter = 1000;
    int num_iter = 0;
    fn_state_ = std::bind(&HungarianOptimizer::ReduceRows, this);
    while (fn_state_ != nullptr && num_iter < max_num_iter)
    {
        fn_state_();
        ++num_iter;
    }
    if (num_iter >= max_num_iter)
    {
        CheckStar();
    }
}

template <typename T>
void HungarianOptimizer<T>::CheckStar()
{
    for (size_t row = 0; row < height_; ++row)
    {
        int star_col = -1;
        bool is_single = true;
        for (size_t col = 0; col < width_; ++col)
        {
            if (IsStarred(row, col))
            {
                if (star_col == -1)
                {
                    star_col = col;
                }
                else
                {
                    is_single = false;
                    break;
                }
            }
        }
        if (!is_single)
        {
            for (size_t col = 0; col < width_; ++col)
            {
                Unstar(row, col);
            }
        }
    }
}


template <typename T>
void HungarianOptimizer<T>::ReduceRows()
{
    for (size_t row = 0; row < matrix_size_; ++row)
    {
        T min_cost = costs_(row, 0);
        for (size_t col = 1; col < matrix_size_; ++col)
        {
            min_cost = std::min(min_cost, costs_(row, col));
        }
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            costs_(row, col) -= min_cost;
        }
    }
    fn_state_ = std::bind(&HungarianOptimizer::StarZeroes, this);
}


template <typename T>
void HungarianOptimizer<T>::StarZeroes()
{

    for (size_t row = 0; row < matrix_size_; ++row)
    {
        if (RowCovered(row))
        {
            continue;
        }
        for (size_t col = 0; col < matrix_size_; ++col)
        {
            if (ColCovered(col))
            {
                continue;
            }
            if (costs_(row, col) == 0)
            {
                Star(row, col);
                CoverRow(row);
                CoverCol(col);
                break;
            }
        }
    }
    ClearCovers();
    fn_state_ = std::bind(&HungarianOptimizer::CoverStarredZeroes, this);
}


template <typename T>
void HungarianOptimizer<T>::CoverStarredZeroes()
{
    size_t num_covered = 0;

    for (size_t col = 0; col < matrix_size_; ++col)
    {
        if (ColContainsStar(col))
        {
            CoverCol(col);
            num_covered++;
        }
    }

    if (num_covered >= matrix_size_)
    {
        fn_state_ = nullptr;
        return;
    }
    fn_state_ = std::bind(&HungarianOptimizer::PrimeZeroes, this);
}


template <typename T>
void HungarianOptimizer<T>::PrimeZeroes()
{

    for (;;)
    {
        size_t zero_row = 0;
        size_t zero_col = 0;
        if (!FindZero(&zero_row, &zero_col))
        {

            fn_state_ = std::bind(&HungarianOptimizer::AugmentPath, this);
            return;
        }

        Prime(zero_row, zero_col);
        int star_col = FindStarInRow(zero_row);

        if (star_col != kHungarianOptimizerColNotFound)
        {
            CoverRow(zero_row);
            UncoverCol(star_col);
        }
        else
        {
            std::pair<size_t, size_t> first_assignment =
                std::make_pair(zero_row, zero_col);
            assignments_[0] = first_assignment;
            fn_state_ =
                std::bind(&HungarianOptimizer::MakeAugmentingPath, this);
            return;
        }
    }
}


template <typename T>
void HungarianOptimizer<T>::MakeAugmentingPath()
{
    bool done = false;
    size_t count = 0;



    while (!done)
    {
 
        int row = FindStarInCol(assignments_[count].second);

        if (row != kHungarianOptimizerRowNotFound)
        {
            count++;
            assignments_[count].first = row;
            assignments_[count].second = assignments_[count - 1].second;
        }
        else
        {
            done = true;
        }

        if (!done)
        {
            int col = FindPrimeInRow(assignments_[count].first);
            count++;
            assignments_[count].first = assignments_[count - 1].first;
            assignments_[count].second = col;
        }
    }


    for (size_t i = 0; i <= count; ++i)
    {
        size_t row = assignments_[i].first;
        size_t col = assignments_[i].second;

        if (IsStarred(row, col))
        {
            Unstar(row, col);
        }
        else
        {
            Star(row, col);
        }
    }

    ClearCovers();
    ClearPrimes();
    fn_state_ = std::bind(&HungarianOptimizer::CoverStarredZeroes, this);
}


template <typename T>
void HungarianOptimizer<T>::AugmentPath()
{
    T minval = FindSmallestUncovered();

    for (size_t row = 0; row < matrix_size_; ++row)
    {
        if (RowCovered(row))
        {
            for (size_t c = 0; c < matrix_size_; ++c)
            {
                costs_(row, c) += minval;
            }
        }
    }
    for (size_t col = 0; col < matrix_size_; ++col)
    {
        if (!ColCovered(col))
        {
            for (size_t r = 0; r < matrix_size_; ++r)
            {
                costs_(r, col) -= minval;
            }
        }
    }
    fn_state_ = std::bind(&HungarianOptimizer::PrimeZeroes, this);
}

} // namespace fusion
} // namespace cem

#endif
