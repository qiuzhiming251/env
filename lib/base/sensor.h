#ifndef SENSOR_H_
#define SENSOR_H_

#include <vector>
#include <mutex>
#include <math.h>
// #include "common/log/log.h"
#include "common/time/time.h"
#include "glog/logging.h"
#include "sensor_meta.h"

namespace cem {
namespace fusion {

template <typename MSG_PTR_TYPE>
class Sensor
{
public:
    Sensor() = default;
    Sensor(size_t number) : max_cached_frame_num_(number){};

    // query frames whose timestamp is in range
    // (_latest_fused_time_stamp, time_stamp]
    void QueryLatestFrames(double timestamp, std::vector<MSG_PTR_TYPE> *frames);

    // query latest frame whose timestamp is in range
    // (_latest_fused_time_stamp, time_stamp]
    MSG_PTR_TYPE QueryLatestFrame(double timestamp, double epsilon_s);

    MSG_PTR_TYPE QueryLatestFrame();

    void AddFrame(const MSG_PTR_TYPE &frame_ptr);

    void ClearFrames();

    void CheckMsgBlocking();

    inline void SetMaxCachedFrameNumber(size_t number)
    {
        max_cached_frame_num_ = number;
    }

    inline void SetLatestQueryTimestamp(double latest_query_timestamp)
    {
        latest_query_timestamp_ = latest_query_timestamp;
    }

private:
    // FRIEND_TEST(SensorTest, test);
 template <typename T, typename = void>
 struct HasHeaderTimestamp : std::false_type {};
 template <typename T>
 struct HasHeaderTimestamp<T, std::void_t<decltype(std::declval<T>()->header.timestamp)>> : std::true_type {};

 template <typename T, typename = void>
 struct HasHeaderMeasurementTimestamp : std::false_type {};
 template <typename T>
 struct HasHeaderMeasurementTimestamp<
     T, std::void_t<decltype(std::declval<T>()->has_header()), decltype(std::declval<T>()->header().measurement_timestamp())>>
     : std::true_type {};

 template <typename T, typename = void>
 struct HasHeaderRecvTimestamp : std::false_type {};
 template <typename T>
 struct HasHeaderRecvTimestamp<T, std::void_t<decltype(std::declval<T>()->header.recv_timestamp)>> : std::true_type {};

 template <typename T, typename = void>
 struct HasHeaderPublishTimestamp : std::false_type {};
 template <typename T>
 struct HasHeaderPublishTimestamp<
     T, std::void_t<decltype(std::declval<T>()->has_header()), decltype(std::declval<T>()->header().publish_timestamp())>>
     : std::true_type {};

 double GetTimestamp(const MSG_PTR_TYPE &frame_ptr) const {
   if constexpr (HasHeaderTimestamp<MSG_PTR_TYPE>::value) {
     return frame_ptr->header.timestamp;
   } else if constexpr (HasHeaderMeasurementTimestamp<MSG_PTR_TYPE>::value) {
     return frame_ptr->header().measurement_timestamp();
   } else {
     static_assert(HasHeaderTimestamp<MSG_PTR_TYPE>::value || HasHeaderMeasurementTimestamp<MSG_PTR_TYPE>::value,
                   "MSG_PTR_TYPE must have either header.timestamp or header().measurement_timestamp()");
     return 0.0;
   }
 }

 double GetRecvTimestamp(const MSG_PTR_TYPE &frame_ptr) const {
   if constexpr (HasHeaderRecvTimestamp<MSG_PTR_TYPE>::value) {
     return frame_ptr->header.recv_timestamp;
   } else if constexpr (HasHeaderPublishTimestamp<MSG_PTR_TYPE>::value) {
     return frame_ptr->header().publish_timestamp();
   } else {
     static_assert(HasHeaderRecvTimestamp<MSG_PTR_TYPE>::value || HasHeaderPublishTimestamp<MSG_PTR_TYPE>::value,
                   "MSG_PTR_TYPE must have either header.recv_timestamp or header().publish_timestamp()");
     return 0.0;
   }
 }

 std::mutex               data_mutex_{};
 double                   latest_query_timestamp_ = 0.0d;
 std::deque<MSG_PTR_TYPE> frames_;
 size_t                   max_cached_frame_num_ = 10;    //zph max input 10 frames keep 500ms data
 double                   max_msg_delay_period_ = 1.5d;  // 1.5s
};

template <typename MSG_PTR_TYPE>
void Sensor<MSG_PTR_TYPE>::QueryLatestFrames(double timestamp,
                                             std::vector<MSG_PTR_TYPE> *frames)
{
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    if (frames == nullptr)
    {
        AERROR << "frames are not available";
        return;
    }

    frames->clear();
    for (size_t i = 0; i < frames_.size(); ++i)
    {
        if (GetTimestamp(frames_[i]) > latest_query_timestamp_ && GetTimestamp(frames_[i]) <= timestamp) {
     frames->push_back(frames_[i]);
        }
    }
    latest_query_timestamp_ = timestamp;
}

template <typename MSG_PTR_TYPE>
MSG_PTR_TYPE Sensor<MSG_PTR_TYPE>::QueryLatestFrame(double timestamp,
                                                    double epsilon_s)
{
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    CheckMsgBlocking();

    MSG_PTR_TYPE latest_frame = nullptr;

    for (size_t i = 0; i < frames_.size(); ++i)
    {
      if (GetTimestamp(frames_[i]) <= timestamp) {
        latest_frame = frames_[i];
        latest_query_timestamp_ = GetTimestamp(frames_[i]);
      } else {
        if (latest_frame == nullptr) {
          if (GetTimestamp(frames_[i]) - timestamp <= epsilon_s) {
            latest_frame = frames_[i];
            latest_query_timestamp_ = GetTimestamp(frames_[i]);
          }
        } else {
          if (fabs(GetTimestamp(frames_[i]) - timestamp) <
              fabs(latest_query_timestamp_ - timestamp)) {
            latest_frame = frames_[i];
            latest_query_timestamp_ = GetTimestamp(frames_[i]);
          }
        }

        break;
      }
    }

    return latest_frame;
}

template <typename MSG_PTR_TYPE>
MSG_PTR_TYPE Sensor<MSG_PTR_TYPE>::QueryLatestFrame()
{
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    CheckMsgBlocking();

    return frames_.empty() ? nullptr : frames_.back();
}

template <typename MSG_PTR_TYPE>
void Sensor<MSG_PTR_TYPE>::AddFrame(const MSG_PTR_TYPE &frame_ptr)
{
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    if (frames_.size() == max_cached_frame_num_)
    {
        frames_.pop_front();
    }
    frames_.emplace_back(frame_ptr);
}

template <typename MSG_PTR_TYPE>
void Sensor<MSG_PTR_TYPE>::ClearFrames()
{
    std::lock_guard<std::mutex> data_lock(data_mutex_);

    frames_.clear();
}

template <typename MSG_PTR_TYPE>
void Sensor<MSG_PTR_TYPE>::CheckMsgBlocking()
{
    if (!frames_.empty() && std::fabs(GetMwTimeNowSec() - GetRecvTimestamp(frames_.back())) > max_msg_delay_period_) {
        frames_.clear();
    }
}

} // namespace fusion
} // namespace cem

#endif
