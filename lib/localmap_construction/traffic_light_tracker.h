/**
 * @file traffic_light_tracker.h
 * @author fanminglei (fan.minglei@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-11
 * 
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 * 
 */
#ifndef TRAFFIC_LIGHT_TRACKER_H_
#define TRAFFIC_LIGHT_TRACKER_H_

#include <cstdint>
#include <unordered_map>

#include <cyber/common/log.h>

#include "Eigen/Dense"
#include "lib/message/internal_message.h"
#include "lib/message/sensor/vision/tsrmobject.h"
#include "lib/common/log_custom.h"

namespace cem {
namespace fusion {

using cem::message::sensor::TrfObjectInfo;
constexpr double kInitMinLoopTime = 0.05;
constexpr double epslion = 1e-8;
inline int Compare(const double x, const double y) {
  const double result = x - y;
  if (result < -epslion) {
    return -1;
  }
  if (result > epslion) {
    return 1;
  }
  return 0;
}

inline int Compare2Zero(const double x) {
  if (x < -epslion) {
    return -1;
  }
  if (x > epslion) {
    return 1;
  }
  return 0;
}

template <typename T>
class DebounceModule {
 public:
  DebounceModule() = default;
  DebounceModule(double rise_time, double main_loop_time);
  ~DebounceModule() = default;
  void ResetTime(double rise_time, double main_loop_time);
  void Reset();
  T DealDebounce(T input);

  void SetLastStatus(T input) { in_pre_ = input; }

  T LastStatus() const { return in_pre_; }

 private:
  T in_pre_{};
  double rise_time_val_ = 0.0;
  double rise_time_limit_{kInitMinLoopTime};
  double main_loop_time_{kInitMinLoopTime};
};
template <typename T>
DebounceModule<T>::DebounceModule(const double rise_time, const double main_loop_time)
    : rise_time_limit_(rise_time), main_loop_time_(main_loop_time) {}

template <typename T>
void DebounceModule<T>::ResetTime(const double rise_time, const double main_loop_time) {
  rise_time_limit_ = rise_time;
  main_loop_time_ = main_loop_time;
}

template <typename T>
void DebounceModule<T>::Reset() {
  in_pre_ = T{};
  rise_time_val_ = 0.0;
}

template <typename T>
T DebounceModule<T>::DealDebounce(T input) {
  if (input != in_pre_) {
    if (Compare(rise_time_val_, rise_time_limit_) == -1) {
      rise_time_val_ += main_loop_time_;
    } else {
      rise_time_val_ = 0.0;
      in_pre_ = input;
    }
  } else {
    rise_time_val_ = 0.0;
    in_pre_ = input;
  }
  return in_pre_;
}

class TrafficLightTracker {
 public:
  TrafficLightTracker();

  ~TrafficLightTracker();

  bool Init(const std::shared_ptr<TrfObjectInfo>& trf_obj_ptr_);
  void UpdateWithDetectedObject(const TrfObjectInfo& trf_obj);

  inline bool IsDie() { return lost_age_ > 18; }
  inline int LostAge() { return lost_age_; }
  void UpdateWithoutDetectedObject();
  inline std::shared_ptr<TrfObjectInfo> GetConstTrackedObject() const { return trf_obj_; }
  inline std::shared_ptr<TrfObjectInfo>& GetTrackedObject() { return trf_obj_; }

 private:
  std::shared_ptr<TrfObjectInfo> trf_obj_{nullptr};
  DebounceModule<TrafficLightDirectionType> direction_debounce_{0.2, kInitMinLoopTime};
  DebounceModule<TrafficLightShapeType> shape_debounce_{0.3, kInitMinLoopTime};
  int lost_age_{0};
};

using TrafficLightTrackerPtr = std::shared_ptr<TrafficLightTracker>;
using TrafficLightTrackerConstPtr = std::shared_ptr<const TrafficLightTracker>;

}  // namespace fusion
}  // namespace cem

#endif  // TRAFFIC_LIGHT_TRACKER_H_
