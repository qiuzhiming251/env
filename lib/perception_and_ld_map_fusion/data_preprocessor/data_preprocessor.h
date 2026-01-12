#pragma once

#include <typeinfo>

#include "common/utils/GeoMathUtil.h"
#include "lib/base/sensor_data_manager.h"
#include "lib/common/utility.h"
#include "lib/message/internal_message.h"
#include "lib/perception_and_ld_map_fusion/data_fusion/road_slice.h"

namespace cem {
namespace fusion {

template <typename DataType>
class DataPreprocessor {
 private:
  int lost_count_ = 0;
  double timestamp_last_fetch_ = 0.0;
  LocalizationPtr data_pose_ = nullptr;

 protected:
  double timestamp_ = 0.0;  // sec
  std::shared_ptr<DataType> data_ = nullptr;

 public:
  DataPreprocessor();
  ~DataPreprocessor();
  bool FeedLatestFrame();
  virtual bool Process(double target_timstamp = 0.0);
  virtual bool RotateAndTranslate(const Eigen::Isometry3d& T_meas2tgt) = 0;
  std::shared_ptr<DataType> GetData();
  const LocalizationConstPtr GetDataPose();
  double GetTimestamp();

 private:
  virtual bool TimeSync(double target_timestamp);
  virtual void Proc() = 0;
};

template <typename DataType>
DataPreprocessor<DataType>::DataPreprocessor() {}

template <typename DataType>
DataPreprocessor<DataType>::~DataPreprocessor() {}

template <typename DataType>
bool DataPreprocessor<DataType>::Process(double target_timestamp) {
  // if (!FeedLatestFrame()) {
  //   return false;
  // }
  if (TimeSync(target_timestamp)) {
    Proc();
    return true;
  }
  return false;
}

template <typename DataType>
bool DataPreprocessor<DataType>::FeedLatestFrame() {
  data_ = nullptr;
  std::shared_ptr<DataType> data_tmp = nullptr;
  if (!SensorDataManager::Instance()->GetLatestSensorFrame(data_tmp)) {
    AERROR << "Cannot fetch latest frame. " << typeid(data_tmp).name();
    return false;
  }
  if (!data_tmp) {
    AERROR << "SensorDataManager fetch null pointer."
           << typeid(data_tmp).name();
    return false;
  }

  if (data_tmp->header.timestamp < 1e-3) {
    AERROR << "input data timestamp invalid:. receive timestamp value: "
           << data_tmp->header.timestamp;
    return false;
  }
  if (fabs(timestamp_last_fetch_ - data_tmp->header.timestamp) < 1e-3) {
    // todo: process for the same frame
    // lost_count_++;
    // if (lost_count_ > 10) {
    //   AERROR << "Cannot fetch newer msg during long time. "
    //          << typeid(data_tmp).name();
    //   data_ = nullptr;
    //   return false;
    // }
    // return true;
    AWARN << "Fetch the same frame with last frame";
  } else {
    timestamp_last_fetch_ = data_tmp->header.timestamp;
  }

  if (data_tmp->header.timestamp - timestamp_ > 0.2) {
    AWARN << "message is not continuous. " << typeid(data_tmp).name();
  }

  // lost_count_ = 0;
  timestamp_ = data_tmp->header.timestamp;
  data_ = std::make_shared<DataType>(*data_tmp);
  return true;
}

template <typename DataType>
bool DataPreprocessor<DataType>::TimeSync(double target_timestamp) {
  if (target_timestamp < 1e-5) {
    target_timestamp = GetMwTimeNowSec();
  }
  LocalizationPtr measurement_pose = nullptr;

  int retry_count = 0;
  while (retry_count < 3) {
    if (retry_count > 0) {
      auto wakeup_time =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
      std::this_thread::sleep_until(wakeup_time);  // sleep 5ms
    }
    if (SensorDataManager::Instance()->GetLatestSensorFrame(target_timestamp,
                                                            0.05, data_pose_) &&
        SensorDataManager::Instance()->GetLatestSensorFrame(timestamp_, 0.05,
                                                            measurement_pose)) {
      break;
    }
    retry_count++;
  }
  if (retry_count == 3) {
    AERROR << "Cannot fetch lastest loclization frame.";
    data_pose_ = nullptr;
    return false;
  }

  Eigen::Isometry3d T = CalcRotateTranslateMatrix(measurement_pose, data_pose_);

  if (RotateAndTranslate(T)) {
    timestamp_ = target_timestamp;
    data_->header.timestamp = timestamp_;
  }
  return true;
}

template <typename DataType>
std::shared_ptr<DataType> DataPreprocessor<DataType>::GetData() {
  return data_;
}

template <typename DataType>
double DataPreprocessor<DataType>::GetTimestamp() {
  return timestamp_;
}

template <typename DataType>
const LocalizationConstPtr DataPreprocessor<DataType>::GetDataPose() {
  return data_pose_;
};

}  // namespace fusion
}  // namespace cem
