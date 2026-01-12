#include "lib/localmap_construction/traffic_light_tracker.h"
#include <cstdint>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cyber/common/log.h>
#include <message/sensor/vision/tsrmobject.h>

#include "Eigen/src/Core/ArrayWrapper.h"
#include "base/sensor_data_manager.h"
#include "lib/common/utility.h"

namespace cem {
namespace fusion {

TrafficLightTracker::TrafficLightTracker() {}
TrafficLightTracker::~TrafficLightTracker() {}

bool TrafficLightTracker::Init(const std::shared_ptr<TrfObjectInfo>& trf_obj_ptr_) {
  if (trf_obj_ptr_ == nullptr) {
    return false;
  }
  trf_obj_ = trf_obj_ptr_;
  direction_debounce_.SetLastStatus(trf_obj_->attributes.traffic_light_direction);
  shape_debounce_.SetLastStatus(trf_obj_->attributes.traffic_light_shape);
  return true;
}

void TrafficLightTracker::UpdateWithDetectedObject(const TrfObjectInfo& trf_obj) {
  
    // 处理id
  trf_obj_->id = trf_obj.id;

  // 处理position(丢失过多帧后重新赋值position)
  double pose_k = lost_age_ > 3 ? 1.0 : 0.5;
  trf_obj_->position.x = trf_obj_->position.x * (1.0 - pose_k) + trf_obj.position.x * pose_k;
  trf_obj_->position.y = trf_obj_->position.y * (1.0 - pose_k) + trf_obj.position.y * pose_k;
  trf_obj_->position_std = trf_obj.position_std;
  trf_obj_->publish_time = trf_obj.publish_time;

  // 处理颜色
  trf_obj_->attributes.traffic_light_color = trf_obj.attributes.traffic_light_color;

  auto orin_type  = trf_obj_->attributes.traffic_light_direction;
  auto orin_shape = trf_obj_->attributes.traffic_light_shape;
  // 处理朝向
  trf_obj_->attributes.traffic_light_direction = direction_debounce_.DealDebounce(trf_obj.attributes.traffic_light_direction);
  // 处理shape
  trf_obj_->attributes.traffic_light_shape = shape_debounce_.DealDebounce(trf_obj.attributes.traffic_light_shape);
  FLOG_TLIGHT << "trj_obj_direction:" << trf_obj.attributes.traffic_light_direction << "  orin_direction:" << orin_type
              << "  new_direction:" << trf_obj_->attributes.traffic_light_direction << "  trj_obj_shape:" << trf_obj.attributes.traffic_light_shape << "  orin_shape:" << orin_shape
              << "  new_shape:" << trf_obj_->attributes.traffic_light_shape;

  // 处理倒计时num--TODO
  trf_obj_->attributes.traffic_light_num = trf_obj.attributes.traffic_light_num;

  //
  trf_obj_->attributes.traffic_light_flashing = trf_obj.attributes.traffic_light_flashing;

  lost_age_ = 0;
}

void TrafficLightTracker::UpdateWithoutDetectedObject() {
  if (trf_obj_->position.x < 0.0) {
    lost_age_ += 30;
  } else {
    lost_age_++;
  }
}
}  // namespace fusion
}  // namespace cem
