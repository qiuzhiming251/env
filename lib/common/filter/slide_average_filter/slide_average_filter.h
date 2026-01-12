#ifndef SLIDE_AVERAGE_FILTER_H_
#define SLIDE_AVERAGE_FILTER_H_

#include "common/utility.h"

namespace cem {
namespace fusion {

template <typename T>
class SlideAverageFilter
{
private:
    std::queue<T> data_buffer_;
    size_t buffer_size_ = 20;
    T buffer_sum_ = 0;
    T average_data_ = 0;

public:
    SlideAverageFilter(const size_t &init_buffer_size, const T &init_data);
    SlideAverageFilter() = delete;
    ~SlideAverageFilter() = default;

    void AddNewData(const T &new_data);
    T GetAverageData(const T &new_data);
    inline T GetAverageData() { return average_data_; }
    inline T GetLastPushedData()
    {
        return data_buffer_.empty() ? 0 : data_buffer_.back();
    }
    inline void SetQueueSize(const size_t &new_size)
    {
        buffer_size_ = new_size;
    }
    void ResetDataBuffer(const T &reset_data);
};

template <typename T>
SlideAverageFilter<T>::SlideAverageFilter(const size_t &init_buffer_size,
                                          const T &init_data)
    : buffer_size_(init_buffer_size),
      average_data_(init_data)
{
    for (size_t i = 0; i < buffer_size_; ++i) data_buffer_.push(init_data);

    buffer_sum_ = init_data * init_buffer_size;
}

template <typename T>
void SlideAverageFilter<T>::AddNewData(const T &new_data)
{
    buffer_sum_ += new_data;

    if (data_buffer_.size() >= buffer_size_)
    {
        buffer_sum_ -= data_buffer_.front();
        data_buffer_.pop();
        data_buffer_.push(new_data);
        average_data_ = buffer_sum_ / buffer_size_;
    }
    else
    {
        data_buffer_.push(new_data);
        average_data_ = buffer_sum_ / data_buffer_.size();
    }
}

template <typename T>
T SlideAverageFilter<T>::GetAverageData(const T &new_data)
{
    AddNewData(new_data);
    return average_data_;
}

template <typename T>
void SlideAverageFilter<T>::ResetDataBuffer(const T &reset_data)
{
    for (size_t i = 0; i < buffer_size_; ++i)
    {
        data_buffer_.pop();
        data_buffer_.push(reset_data);
    }

    buffer_sum_ = reset_data * buffer_size_;
    average_data_ = reset_data;
}

} // namespace fusion
} // namespace cem

#endif
