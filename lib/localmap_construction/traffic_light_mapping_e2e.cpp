#include "lib/localmap_construction/traffic_light_mapping_e2e.h"
#include <log_custom.h>
#include <cyber/time/clock.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <new>
#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cyber/common/log.h>
#include <localmap_construction/cloud_config_traffic_light.h>
#include <message/env_model/traffic_light/traffic_light_info.h>
#include <message/internal_message.h>
#include <message/sensor/vision/tsrmobject.h>
#include <perception_and_ld_map_fusion/data_fusion/geometry_match_info.h>
#include "fmt/core.h"
#include "fmt/ranges.h"
#include "lib/message/env_model/routing_map/routing_map.h"

#include "Eigen/src/Core/ArrayWrapper.h"
#include "Eigen/src/Core/Matrix.h"
#include "base/sensor_data_manager.h"
#include "fmt/format.h"
#include "lib/common/utility.h"
#include "magic_enum/magic_enum.hpp"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/base/params_manager/params_manager.h"
#include "modules/perception/env/src/lib/pre_processor/data_manager/data_manager.h"

namespace cem::fusion::e2e {

using byd::common::math::LineSegment2d;
using byd::common::math::Vec2d;
using ::byd::msg::orin::e2e_map::NaviMainAction;

constexpr int max_stay_frames_with_info = 5;
constexpr int max_stay_frames_no_info   = 3;

inline LineSegment2d TranslateSegmentAlongAxisStart(const LineSegment2d &axis, const LineSegment2d &seg1, double distance) {
  const Vec2d translation = axis.unit_direction() * -distance;
  return {seg1.start() + translation, seg1.end() + translation};
}

LightStatus LightColorConvert(const TrafficLightColorType &color_type, bool is_blinking = false) {
  switch (color_type) {
    case cem::message::sensor::TLC_RED_FLASHING:
    case message::sensor::TLC_RED: {
      return message::env_model::LightStatus::RED_LIGHT;
    }
    case message::sensor::TLC_GREEN: {
      return is_blinking ? message::env_model::LightStatus::GREEN_BLINKING : message::env_model::LightStatus::GREEN_LIGHT;
    }
    case message::sensor::TLC_YELLOW: {
      return is_blinking ? message::env_model::LightStatus::YELLOW_BLINKING : message::env_model::LightStatus::YELLOW_LIGHT;
    }
    case cem::message::sensor::TLC_GREEN_FLASHING:
      return message::env_model::LightStatus::GREEN_BLINKING;
    case cem::message::sensor::TLC_YELLOW_FLASHING:
      return message::env_model::LightStatus::YELLOW_BLINKING;
    case cem::message::sensor::TLC_BLURRING_MODE:
      return message::env_model::LightStatus::BLURRING_MODE;
    case cem::message::sensor::TLC_BLOCK_FAILED:
      return message::env_model::LightStatus::BLOCK_FAIL;
    case cem::message::sensor::TLC_NOT_MATCH:
      return message::env_model::LightStatus::CLOUD_NOT_MATCH;
    case cem::message::sensor::TLC_NONE_LIGHT:
      return message::env_model::LightStatus::NONE_LIGHT;
    default: {
      return message::env_model::LightStatus::UNKNOWN_LIGHT;
    }
  }
}

using cem::message::sensor::TrfObjectInfo;

class TrafficLightMapping::IDPool {
 public:
  IDPool(int range = 1024) : range_(range) { Reset(); };
  ~IDPool() = default;
  void Reset() {
    id_map_.clear();
    for (int i = 1; i <= range_; ++i) {
      id_map_[i] = i;
    }
  };
  uint32_t RentID() {
    CheckPoolNotEmpty();
    auto iter = id_map_.begin();
    auto id   = iter->second;
    id_map_.erase(iter);
    return id;
  }

  void TakeBackID(uint32_t id) {
    if (id > range_) {
      AWARN << "Cannot take back an ID (" << id << ") is out of range (" << range_ << ").";
      return;
    }
    if (id_map_.count(id) > 0) {
      AWARN << "Cannot take back an id not rented before.";
      return;
    }
    id_map_[id] = id;
  };

 private:
  bool CheckPoolNotEmpty() {
    if (id_map_.empty()) {
      AERROR << __FILE__ << ": ID Pool is empty! Need bigger buffer.";
      throw std::runtime_error("ID Pool is empty! Need bigger buffer.");
    }
    return true;
  };
  std::unordered_map<uint32_t, uint32_t> id_map_;
  uint32_t                               range_;
};

TrafficLightMapping::TrafficLightMapping() : id_pool_(new IDPool()), timestamp_(0), lost_count_(0) {}

TrafficLightMapping::~TrafficLightMapping() {
  if (id_pool_) {
    delete id_pool_;
  }
}

void TrafficLightMapping::StayPrevTrafficLightSet() {
  auto SaveObj = [&](const TrafficLight &light) {
    if (IsValid(light)) {
      deal_per_traffic_light_objects_.insert({light.traffic_obj_info->id, *light.traffic_obj_info});
    }
  };
  SaveObj(traffic_lights_.left);
  SaveObj(traffic_lights_.u_turn);
  SaveObj(traffic_lights_.straight);
  SaveObj(traffic_lights_.right);
}

void TrafficLightMapping::StayPreviousState(const TrafficLight &previous_light, TrafficLight &traffic_light) {
  if (previous_light.is_valid && previous_light.stay_prev_counter < max_stay_frames_no_info) {
    traffic_light = previous_light;
    traffic_light.stay_prev_counter++;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
  }
}

void TrafficLightMapping::Process(const RoutingMapPtr &routing_map_input, const RoutingMapPtr &routing_map_output,
                                  const BevMapInfoPtr &bev_map, const std::vector<cem::message::env_model::StopLine> &stopline) {

  std::string                  cloud_debug_info;
  std::optional<TrafficLights> cloud_traffic_light = cem::fusion::claud_traffic::GetCloudTrafficLights(bev_map, cloud_debug_info);
  CLOUD_TRAFFIC_LOG << "cloud_traffic_light______________________end_______";

  FLOG_TLIGHT << "------------------start----------------";
  double timestamp     = GetMwTimeNowSec();
  info_traffic_lights_ = "";
  traffic_light_e2e_   = std::make_shared<cem::message::env_model::TrafficLightsE2EInfo>();
  FeedLatestFrame();

  LocalMapping(timestamp);

  GetRoutingTrafficLight(routing_map_input);

  DealPerTrafficLight(bev_map, routing_map_input);

  {
    Reset();
    // sd_navigation_city_ = sd_navigation_city;
    TimeSync();
    routing_map_ptr_ = routing_map_input;
    FillPerceptionTrafficLights();
    traffic_light_e2e_->header.timestamp = timestamp_;
    SetTrafficLightColor();
    bool valid_transform = TransformPostionOfObjs(timestamp);
    FilterLightsByPosition();
    for (const auto &obj : perception_traffic_lights_) {
      deal_per_traffic_light_objects_.insert({obj.id, obj});
    }

    /********** 路口面绑灯和过滤灯的逻辑 **********/
    // 01 获取停止线
    GetStopLineMsg(stopline, stop_line_msg_);
    // 02 路口面绑定红绿灯
    BindLightsByGroundMarkings(stopline, bev_map);
    // 03 路口面绑到停止线上
    BindGroundMarkingsAndStopLine(bev_map);
    // 04 过滤下个路口的灯
    if (perception_traffic_light_info_) {
      lights_info_this_junction_ = std::make_shared<PercepTrfInfo>(*perception_traffic_light_info_);
    }
    FilterLightsNextJunction();
    // 05 大小路口判断
    int junction_size = JudgeJunctionSize(bev_map);
    if (junction_size == 1) {
      is_virtual_junction_ = false;
    } else if (junction_size == 2) {
      is_virtual_junction_ = true;
    }
    TRAFFIC_LOG << "res junction_size: " << junction_size << ", res is_virtual_junction_: " << is_virtual_junction_;

    // 路口面绑停止线成功，直接选定绑定成功的灯和70m以内的灯为有效灯，过滤其他灯
    if (valid_transform && !perception_traffic_lights_.empty() && perception_traffic_light_info_) {
      SetLightsTurn(straight_shapes_, LightTurn::STRAIGHT);
      SetLightsTurn(left_shapes_, LightTurn::LEFT);
      SetLightsTurn(right_shapes_, LightTurn::RIGHT);
      SetLightsTurn(uturn_shapes_, LightTurn::UTURN);

      SetTrafficLights(LightTurn::STRAIGHT, straight_shapes_, traffic_lights_previous_.straight, traffic_lights_.straight);
      SetTrafficLights(LightTurn::LEFT, left_shapes_, traffic_lights_previous_.left, traffic_lights_.left);
      SetTrafficLights(LightTurn::UTURN, uturn_shapes_, traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
      SetTrafficLights(LightTurn::RIGHT, right_shapes_, traffic_lights_previous_.right, traffic_lights_.right);

      for (const auto &obj : perception_traffic_lights_) {
        // deal_per_traffic_light_objects_.insert({obj.id, obj});
        std::vector<std::string> turn_strs;
        for (const auto turn : obj.turn) {
          turn_strs.emplace_back(magic_enum::enum_name(turn));
        }
        TRAFFIC_REC_LOG << fmt::format("final_obj_id:{}  shape:{} color:{} flash:{:d}  turn:{} num:{}", obj.id,
                                       magic_enum::enum_name(obj.attributes.traffic_light_shape),
                                       magic_enum::enum_name(obj.attributes.traffic_light_color), obj.attributes.traffic_light_flashing,
                                       turn_strs, obj.attributes.traffic_light_num);
      }
    } else if (traffic_lights_previous_.straight.is_valid || traffic_lights_previous_.left.is_valid ||
               traffic_lights_previous_.u_turn.is_valid || traffic_lights_previous_.right.is_valid) {
      TRAFFIC_REC_LOG << fmt::format("stay_prev_state.");
      StayPreviousState(traffic_lights_previous_.straight, traffic_lights_.straight);
      StayPreviousState(traffic_lights_previous_.left, traffic_lights_.left);
      StayPreviousState(traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
      StayPreviousState(traffic_lights_previous_.right, traffic_lights_.right);
      StayPrevTrafficLightSet();
    }
    TRAFFIC_REC_LOG << fmt::format("prev_dis_to_junction:{:.2f} valid_transform:{:d} ptr:{} perception_traffic_lights_size:{}",
                                   prev_action_dis_, valid_transform, fmt::ptr(perception_traffic_light_info_),
                                   perception_traffic_lights_.size());
    IsLightBlockFailed(traffic_lights_previous_.left, traffic_lights_.left);
    IsLightBlockFailed(traffic_lights_previous_.u_turn, traffic_lights_.u_turn);
    IsLightBlockFailed(traffic_lights_previous_.right, traffic_lights_.right);
    IsLightBlockFailed(traffic_lights_previous_.straight, traffic_lights_.straight);
    traffic_lights_previous_ = traffic_lights_;
    TRAFFIC_REC_LOG << fmt::format("traffic_straight {}", traffic_lights_.straight);
    TRAFFIC_REC_LOG << fmt::format("traffic_left     {}", traffic_lights_.left);
    TRAFFIC_REC_LOG << fmt::format("traffic_uturn    {}", traffic_lights_.u_turn);
    TRAFFIC_REC_LOG << fmt::format("traffic_right    {}", traffic_lights_.right);
  }

  // IsEgoDedicatedRight();

  // SetTrafficLight(routing_map_input, routing_map_output);

  if (traffic_light_e2e_) {
    SetSrTrafficLight(traffic_light_e2e_->traffic_lights);
  }

  // BindingTrafficLightToBev(bev_map);

  info_traffic_lights_.clear();
  if (cloud_traffic_light) {
    info_traffic_lights_ =
        fmt::format(" cloud_junc_id:{} type:{} ", cloud_traffic_light->junction_tail_id, cloud_traffic_light->junction_cloud_type);
    traffic_lights_ = ChooseBestTrafficLights(*cloud_traffic_light, traffic_lights_);
  }
  TRAFFIC_LOG << fmt::format("traffic_straight {}", traffic_lights_.straight);
  TRAFFIC_LOG << fmt::format("traffic_left     {}", traffic_lights_.left);
  TRAFFIC_LOG << fmt::format("traffic_uturn    {}", traffic_lights_.u_turn);
  TRAFFIC_LOG << fmt::format("traffic_right    {}", traffic_lights_.right);

  std::string bind_lights_str = "";
  if (lights_info_this_junction_) {
    for (auto light : lights_info_this_junction_->objects) {
      bind_lights_str = bind_lights_str + std::to_string(light.id) + ",";
    }
  }
  info_traffic_lights_ += fmt::format(" keep_lights:{} ", bind_lights_str);

  SetLightStatus_E2E(stopline, bev_map);
  SetTrafficLightInfo();
  if (!cloud_traffic_light || (cloud_traffic_light && cloud_traffic_light->junction_cloud_type == 2)) {
    TRAFFIC_LOG << "is_target_turn:1";
    traffic_light_e2e_->traffic_status.clear();
    info_traffic_lights_ += " E2E_TIN ";
    SetLightStatus_E2E_TIN(stopline, bev_map);
  }

  // TrafficLightNumCorrection();

  SetTrafficLightInfo();
  info_traffic_lights_ += "cloud_info:[" + cloud_debug_info + "] ";

  last_tracker_time_ = timestamp;
}

TrafficLights TrafficLightMapping::ChooseBestTrafficLights(const TrafficLights &cloud_lights, const TrafficLights &default_lights) {
  TrafficLights traffic_lights{default_lights};
  switch (cloud_lights.junction_cloud_type) {
    case 1:
      return cloud_lights;
    case 2:
      traffic_lights.distance_to_stopline = cloud_lights.distance_to_stopline;
      return traffic_lights;
    case 3:
      return traffic_lights;
    default:
      return cloud_lights;
  }
  return traffic_lights;
}

void TrafficLightMapping::SetTrafficLightInfo() {
  auto AppendLightInfo = [&](const TrafficLight &light, const char *prefix) {
    if (light.is_valid) {
      info_traffic_lights_ +=
          fmt::format(" {}:{}-{}-{};", prefix, light.traffic_obj_info ? light.traffic_obj_info->id : 0, light.color, light.traffic_reason);
    }
  };
  info_traffic_lights_ += " TRF";

  AppendLightInfo(traffic_lights_.left, "L");
  AppendLightInfo(traffic_lights_.right, "R");
  AppendLightInfo(traffic_lights_.u_turn, "U");
  AppendLightInfo(traffic_lights_.straight, "S");
}

bool TrafficLightMapping::IsLightBlockFailed(const TrafficLight &light_prev, TrafficLight &light) const {
  light.invalid_counter = light_prev.invalid_counter;
  bool is_prev_valid    = light_prev.color == message::sensor::TLC_RED || light_prev.color == message::sensor::TLC_GREEN ||
                       light_prev.color == message::sensor::TLC_YELLOW || light_prev.color == message::sensor::TLC_YELLOW_FLASHING ||
                       light_prev.color == message::sensor::TLC_GREEN_FLASHING || light_prev.color == message::sensor::TLC_RED_FLASHING ||
                       light_prev.color == message::sensor::TLC_BLURRING_MODE;
  bool is_current_invalid =
      light.stay_prev_counter > 0 ||
      (light.color != message::sensor::TLC_RED && light.color != message::sensor::TLC_GREEN && light.color != message::sensor::TLC_YELLOW &&
       light.color != message::sensor::TLC_YELLOW_FLASHING && light.color != message::sensor::TLC_GREEN_FLASHING &&
       light.color != message::sensor::TLC_RED_FLASHING && light.color != message::sensor::TLC_BLURRING_MODE);
  if (is_current_invalid && prev_action_dis_ > -20 && prev_action_dis_ < 80.0) {
    if (is_prev_valid || light.invalid_counter > 0) {
      light.invalid_counter++;
    }
    if (light.invalid_counter >= 1) {
      light.color             = message::sensor::TLC_BLOCK_FAILED;
      light.is_valid          = true;
      light.stay_prev_counter = 0;
      light.traffic_light_num = 1000;
      light.traffic_obj_info.reset();
      TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check sucessful turn:{}  oringin_counter:{}  invalid_counter:{}  dis:{:.2f}",
                                     magic_enum::enum_name(light.turn_type), light_prev.invalid_counter, light.invalid_counter,
                                     prev_action_dis_);
      return true;
    }
    TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check failed counter_less_3. turn:{} oringin_counter:{}   invalid_counter:{}  dis:{:.2f}",
                                   magic_enum::enum_name(light.turn_type), light_prev.invalid_counter, light.invalid_counter,
                                   prev_action_dis_);
    return false;
  }
  TRAFFIC_REC_LOG << fmt::format("BLOCK_FAILED_check failed not in 50m->ego_junction turn:{}    dis:{:.2f}  current_invalid:{:d}",
                                 magic_enum::enum_name(light.turn_type), distance_to_junction_prev_, !is_current_invalid);
  light.invalid_counter = 0;
  return false;
}

void TrafficLightMapping::TrafficLightNumCorrection() {
  SDTrafficLightPtr sd_traffic_light_ptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(sd_traffic_light_ptr);
  if (!sd_traffic_light_ptr || !traffic_light_e2e_) {
    return;
  }
  TrafficLightStatus *target_status_ptr{nullptr};
  for (auto &status : traffic_light_e2e_->traffic_status) {
    if (status.is_navi_light) {
      target_status_ptr = &status;
      break;
    }
  }
  if (target_status_ptr == nullptr) {
    return;
  }
  TrafficLightStatus &target_status = *target_status_ptr;
  if (target_status.light_status == LightStatus::NONE_LIGHT || target_status.light_status == LightStatus::UNKNOWN_LIGHT ||
      target_status.traffic_light_num == 4294967295 || target_status.traffic_light_num == 1000) {
    TRAFFIC_REC_LOG << fmt::format("target_status:{}  num:{}", target_status.light_status, target_status.traffic_light_num);
    return;
  }
  if (!sd_traffic_light_ptr->has_header() || !sd_traffic_light_ptr->header().has_measurement_timestamp() ||
      !sd_traffic_light_ptr->has_end_time() || sd_traffic_light_ptr->end_time() < 100 || !sd_traffic_light_ptr->has_traffic_light_dist()) {
    return;
  }
  double sd_num_doulbe = static_cast<double>(sd_traffic_light_ptr->end_time()) - sd_traffic_light_ptr->header().measurement_timestamp();
  if (sd_num_doulbe < 0 || sd_num_doulbe > 1000.0) {
    return;
  }
  std::size_t sd_num   = static_cast<std::size_t>(sd_num_doulbe);
  bool        is_big   = std::fabs(target_status.traffic_light_num - sd_num) > 5;
  std::size_t orin_num = target_status.traffic_light_num;
  if (is_big) {
    target_status.traffic_light_num = -1;
  }
  TRAFFIC_REC_LOG << fmt::format("sd_measurement_time:{:.2f}  end_time:{}  sd_dis:{}  sd_num:{}  target_num:{},{}  is_close:{:d}",
                                 sd_traffic_light_ptr->header().measurement_timestamp(), sd_traffic_light_ptr->end_time(),
                                 sd_traffic_light_ptr->traffic_light_dist(), sd_num, orin_num, target_status.traffic_light_num, !is_big);
}

cem::message::env_model::TurnType E2EActionConvertTurnType(NaviMainAction navi_main_action) {
  switch (navi_main_action) {
    case NaviMainAction::NMA_TURN_LEFT:
    case NaviMainAction::NMA_SLIGHT_LEFT:
    case NaviMainAction::NMA_TURN_HARD_LEFT:
      return cem::message::env_model::TurnType::LEFT_TURN;
    case NaviMainAction::NMA_TURN_RIGHT:
    // case NaviMainAction::NMA_ENTRY_RING:
    // case NaviMainAction::NMA_SLIGHT_RIGHT:
    case NaviMainAction::NMA_TURN_HARD_RIGHT:
    case NaviMainAction::NMA_RIGHT_UTURN:
      return cem::message::env_model::TurnType::RIGHT_TURN;
    case NaviMainAction::NMA_LEFT_UTURN:
      return cem::message::env_model::TurnType::U_TURN;
    default:
      return cem::message::env_model::TurnType::NO_TURN;
  }
}

namespace TurnFlags {
using Type = std::uint32_t;  // 定义掩码的类型

constexpr Type STRAIGHT = 1 << 0;  // 1 (0b0001)
constexpr Type LEFT     = 1 << 1;  // 2 (0b0010)
constexpr Type RIGHT    = 1 << 2;  // 4 (0b0100)
constexpr Type U_TURN   = 1 << 3;  // 8 (0b1000)

// 为 OTHER_UNKNOWN 定义一个特殊值
constexpr Type UNKNOWN = 1 << 8;
}  // namespace TurnFlags

// 3. 核心转换函数
// 输入: TurnType 枚举值
// 输出: uint32_t 类型的位掩码
TurnFlags::Type ToBitMask(::byd::msg::orin::e2e_map::TurnType input_turn) {
  switch (input_turn) {
    case ::byd::msg::orin::e2e_map::TurnType::NO_TURN:
      return 0;
    case ::byd::msg::orin::e2e_map::TurnType::LEFT_TURN:
      return TurnFlags::LEFT;
    case ::byd::msg::orin::e2e_map::TurnType::RIGHT_TURN:
      return TurnFlags::RIGHT;
    case ::byd::msg::orin::e2e_map::TurnType::U_TURN:
      return TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_LEFT_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::LEFT;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_RIGHT_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::RIGHT;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_U_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::LEFT_TURN_AND_U_TURN:
      return TurnFlags::LEFT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::RIGHT_TURN_AND_U_TURN:
      return TurnFlags::RIGHT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::LEFT | TurnFlags::RIGHT;
    case ::byd::msg::orin::e2e_map::TurnType::LEFT_TURN_AND_RIGHT_TURN:
      return TurnFlags::LEFT | TurnFlags::RIGHT;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::LEFT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::RIGHT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN:
      return TurnFlags::LEFT | TurnFlags::RIGHT | TurnFlags::U_TURN;
    case ::byd::msg::orin::e2e_map::TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN:
      return TurnFlags::STRAIGHT | TurnFlags::LEFT | TurnFlags::RIGHT | TurnFlags::U_TURN;

    case ::byd::msg::orin::e2e_map::TurnType::OTHER_UNKNOWN:
    default:
      return TurnFlags::UNKNOWN;
  }
}

std::string RecommendInfo::DebugString() const {
  fmt::memory_buffer buf;
  fmt::format_to(buf, "dis:[{:.2f},{:.2f}]; is_virtual:{:d} ", dis_start, dis_end, is_virtual);
  if (recommend_lanes_info_ptr != nullptr) {
    fmt::format_to(buf, " id:{} start_dis:{:.2f};", recommend_lanes_info_ptr->id(), recommend_lanes_info_ptr->dis());
  }
  if (!recommend_lane_vec.empty()) {
    fmt::format_to(buf, " [");
  }
  for (const auto *val : recommend_lane_vec) {
    if (val == nullptr) {
      continue;
    }
    fmt::format_to(buf, " seq_id:{} lane_type:{}  turn:{} plan_turn:{};", val->lane_seq(), magic_enum::enum_name(val->lane_type()),
                   magic_enum::enum_name(val->turn_type()), magic_enum::enum_name(val->plan_turn_type()));
  }
  if (!recommend_lane_vec.empty()) {
    fmt::format_to(buf, " ]");
  }

  if (!junctions_vec.empty()) {
    fmt::format_to(buf, " [");
  }
  for (const auto *val : junctions_vec) {
    if (val == nullptr) {
      continue;
    }
    fmt::format_to(buf, "jun_dis:{:.2f};", val->dis());
  }
  if (!junctions_vec.empty()) {
    fmt::format_to(buf, "]");
  }

  if (!actions_vec.empty()) {
    fmt::format_to(buf, " [");
  }
  for (const auto &val : actions_vec) {
    fmt::format_to(buf, " action:{} ass_act:{} dis:{:.2f};", magic_enum::enum_name(val.main_action()),
                   magic_enum::enum_name(val.assistant_action()), val.action_dis());
  }
  if (!actions_vec.empty()) {
    fmt::format_to(buf, " ]");
  }

  return fmt::to_string(buf);
}

void TrafficLightMapping::GetActionBaseRecommendLanes(const E2EMapRawPtr &e2e_map) {
  recommend_infos_.clear();
  if (!e2e_map || !e2e_map->has_route_info() || !e2e_map->route_info().has_mpp_info()) {
    return;
  }
  const auto &mpp_info_tmp    = e2e_map->route_info().mpp_info();
  const auto &recommend_lanes = mpp_info_tmp.recommend_lanes();
  if (recommend_lanes.empty()) {
    TRAFFIC_LOG << "no_recommend_lanes.";
    return;
  }

  std::map<uint64_t, const ::byd::msg::orin::e2e_map::LaneGroupInfo *> lane_group_info_map;
  for (const auto &lane_group : mpp_info_tmp.lane_group_info()) {
    lane_group_info_map.insert({lane_group.id(), &lane_group});
  }

  constexpr double min_offset_t = 0.1;
  for (const auto &recom : recommend_lanes) {
    if (recom.dis() > 400) {
      break;
    }
    RecommendInfo recommend_info_t;
    recommend_info_t.dis_start = recom.dis();
    recommend_info_t.dis_end   = recom.dis() + recom.length();

    recommend_info_t.recommend_lanes_info_ptr = &recom;

    auto it_lane_group = lane_group_info_map.find(recom.id());
    if (it_lane_group == lane_group_info_map.end()) {
      TRAFFIC_LOG << fmt::format("no_find_recommend_lane_group:{}", recom.id());
      continue;
    }
    recommend_info_t.lane_group_ptr = it_lane_group->second;
    const auto &lane_info_t         = it_lane_group->second->lane_info();
    for (const auto &seq_id : recom.recommend_lane_seq()) {
      auto it_lane = std::find_if(lane_info_t.begin(), lane_info_t.end(),
                                  [&](const ::byd::msg::orin::e2e_map::LaneInfo &lane_t) { return lane_t.lane_seq() == seq_id; });
      if (it_lane == lane_info_t.end()) {
        TRAFFIC_LOG << fmt::format("no_find_lane_seq_id:{} in_lane_group:{}", seq_id, recom.id());
        continue;
      }
      recommend_info_t.recommend_lane_vec.emplace_back(&(*it_lane));
    }
    if (!recommend_infos_.empty()) {
      double prev_end  = recommend_infos_.back().dis_end;
      double new_start = recommend_info_t.dis_start;
      if (new_start - prev_end > min_offset_t) {
        RecommendInfo recommend_info_insert;
        recommend_info_insert.dis_start  = prev_end;
        recommend_info_insert.dis_end    = new_start;
        recommend_info_insert.is_virtual = true;

        recommend_info_insert.recommend_lane_vec = recommend_infos_.back().recommend_lane_vec;

        recommend_infos_.push_back(recommend_info_insert);
      }
    }
    recommend_infos_.push_back(recommend_info_t);
  }

  for (const auto &jun : mpp_info_tmp.reminder_junction_info()) {
    if (jun.has_type() && jun.type() != ::byd::msg::orin::e2e_map::ReminderSceneType::RST_JUNCTION) {
      continue;
    }
    for (auto &val : recommend_infos_) {
      if (jun.dis() >= val.dis_start && jun.dis() < val.dis_end) {
        val.junctions_vec.push_back(&jun);
      }
    }
  }
  for (const auto &action : actions_prev_) {
    for (auto &val : recommend_infos_) {
      if (action.action_dis() >= val.dis_start && action.action_dis() < val.dis_end) {
        val.actions_vec.push_back(action);
      }
    }
  }

  for (const auto &val : recommend_infos_) {
    TRAFFIC_LOG << val.DebugString();
  }
}

std::vector<RecommendInfo> TrafficLightMapping::GetActionSpecial(double event_dis, double offset) {
  std::vector<RecommendInfo> res;
  for (auto &recom_val : recommend_infos_) {
    if (event_dis - offset > recom_val.dis_start && event_dis - offset < recom_val.dis_end) {
      res.push_back(recom_val);
      continue;
    }
    if (event_dis > recom_val.dis_start && event_dis < recom_val.dis_end) {
      res.push_back(recom_val);
      continue;
    }
    if (event_dis + offset > recom_val.dis_start && event_dis + offset < recom_val.dis_end) {
      res.push_back(recom_val);
      continue;
    }
  }
  return res;
}

void TrafficLightMapping::SetActionBlur(::byd::msg::orin::e2e_map::NaviActionInfo &next_event) {
  if (next_event.main_action() == NaviMainAction::NMA_NONE || next_event.main_action() == NaviMainAction::NMA_ENTRY_RING ||
      next_event.main_action() == NaviMainAction::NMA_LEAVE_RING || next_event.main_action() == NaviMainAction::NMA_TURN_LEFT ||
      next_event.main_action() == NaviMainAction::NMA_TURN_RIGHT || next_event.main_action() == NaviMainAction::NMA_TURN_HARD_LEFT ||
      next_event.main_action() == NaviMainAction::NMA_LEFT_UTURN || next_event.main_action() == NaviMainAction::NMA_RIGHT_UTURN) {
    return;
  }

  auto SetResTurn = [](const RecommendInfo &sec, ::byd::msg::orin::e2e_map::NaviActionInfo &next_event) {
    for (const auto *lane : sec.recommend_lane_vec) {
      if (lane == nullptr) {
        continue;
      }
      TurnFlags::Type turn_flag = ToBitMask(lane->plan_turn_type());
      if ((turn_flag & TurnFlags::LEFT) != 0U) {
        if (next_event.main_action() == NaviMainAction::NMA_SLIGHT_RIGHT) {
          next_event.set_main_action(NaviMainAction::NMA_CONTINUE);
        } else {
          next_event.set_main_action(NaviMainAction::NMA_TURN_LEFT);
        }
        TRAFFIC_LOG << fmt::format("change_action_left:") << sec.DebugString();
        return true;
      }
      if ((turn_flag & TurnFlags::RIGHT) != 0U) {
        if (next_event.main_action() == NaviMainAction::NMA_SLIGHT_LEFT) {
          next_event.set_main_action(NaviMainAction::NMA_CONTINUE);
        } else {
          next_event.set_main_action(NaviMainAction::NMA_TURN_RIGHT);
        }
        TRAFFIC_LOG << fmt::format("change_action_right:") << sec.DebugString();
        return true;
      }
      if ((turn_flag & TurnFlags::U_TURN) != 0U) {
        next_event.set_main_action(NaviMainAction::NMA_LEFT_UTURN);
        TRAFFIC_LOG << fmt::format("change_action_uturn:") << sec.DebugString();
        return true;
      }
      if (lane->lane_type() == ::byd::msg::orin::e2e_map::LaneType::LANE_RIGHT_TURN_LANE) {
        next_event.set_main_action(NaviMainAction::NMA_TURN_RIGHT);
        TRAFFIC_LOG << fmt::format("right_lane_change_action_right:") << sec.DebugString();
        return true;
      }
      next_event.set_main_action(NaviMainAction::NMA_CONTINUE);
      TRAFFIC_LOG << fmt::format("change_action_continue:") << sec.DebugString();
      return true;
    }
    return false;
  };

  if (next_event.main_action() == NaviMainAction::NMA_CONTINUE) {
    auto sections = GetActionSpecial(next_event.action_dis(), 5.0);
    if (sections.empty()) {
      TRAFFIC_LOG << fmt::format("can't find the dis:{:.2f} around continue_action.", next_event.action_dis());
      return;
    }
    if (sections.size() == 1 && sections.at(0).is_virtual && sections.front().recommend_lane_vec.size() == 1) {
      auto type_front = sections.front().recommend_lane_vec.front()->plan_turn_type();
      if (type_front == ::byd::msg::orin::e2e_map::TurnType::LEFT_TURN) {
        next_event.set_main_action(NaviMainAction::NMA_TURN_LEFT);
        TRAFFIC_LOG << fmt::format("action_continue__right_lane_change_action_left:") << sections.front().DebugString();
      } else if (type_front == ::byd::msg::orin::e2e_map::TurnType::RIGHT_TURN) {
        next_event.set_main_action(NaviMainAction::NMA_TURN_RIGHT);
        TRAFFIC_LOG << fmt::format("action_continue__right_lane_change_action_right:") << sections.front().DebugString();
      }
    }
    return;
  }

  for (std::size_t idx = 0; idx <= 4; idx++) {
    auto sections = GetActionSpecial(next_event.action_dis(), static_cast<double>(idx) * 5.0);
    if (sections.empty() && idx == 0) {
      TRAFFIC_LOG << fmt::format("can't find the dis:{:.2f}", next_event.action_dis());
    }
    if (sections.size() == 1 && sections.at(0).is_virtual) {
      TRAFFIC_LOG << "dis_in_virtual_section.";
      if (SetResTurn(sections.front(), next_event)) {
        return;
      }
    }
    TRAFFIC_LOG << "dis_in_section_multi.";
    for (const auto &sec : sections) {
      if (SetResTurn(sec, next_event)) {
        return;
      }
    }
  }
}

std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> TrafficLightMapping::GetActionEvent(const E2EMapRawPtr &e2e_map) {
  auto ActionIsValid = [](const byd::msg::orin::e2e_map::NaviActionInfo &event) {
    if (event.has_main_action() && event.main_action() == byd::msg::orin::e2e_map::NMA_NONE) {
      return false;
    }
    if (event.has_assistant_action() &&
        (event.assistant_action() == byd::msg::orin::e2e_map::NAA_VIA1 || event.assistant_action() == byd::msg::orin::e2e_map::NAA_VIA2 ||
         event.assistant_action() == byd::msg::orin::e2e_map::NAA_VIA3 || event.assistant_action() == byd::msg::orin::e2e_map::NAA_VIA4)) {
      return false;
    }
    if (!event.has_main_action() || !event.has_assistant_action() || !event.has_action_dis()) {
      return false;
    }
    return true;
  };

  google::protobuf::RepeatedPtrField<byd::msg::orin::e2e_map::NaviActionInfo> actions_all;

  std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> res_event  = std::nullopt;
  std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> prev_event = std::nullopt;
  std::optional<::byd::msg::orin::e2e_map::NaviActionInfo> next_event = std::nullopt;

  uint64_t e2e_header_seq = 0;

  if (e2e_map == nullptr || e2e_map->navi_action().empty()) {
    actions_prev_.Clear();
  } else {
    e2e_header_seq       = e2e_map->header().sequence_num();
    auto actions_current = e2e_map->navi_action();
    actions_prev_.erase(std::remove_if(actions_prev_.begin(), actions_prev_.end(),
                                       [&](const byd::msg::orin::e2e_map::NaviActionInfo &rhs) { return !ActionIsValid(rhs); }),
                        actions_prev_.end());
    actions_current.erase(std::remove_if(actions_current.begin(), actions_current.end(),
                                         [&](const byd::msg::orin::e2e_map::NaviActionInfo &rhs) { return !ActionIsValid(rhs); }),
                          actions_current.end());
    if (actions_current.empty()) {
      return std::nullopt;
    }
    next_event = actions_current[0];

    double dis_offset{0.0};
    int    idx_target = -1;
    for (int idx = 0; idx < actions_prev_.size(); idx++) {
      if (actions_prev_[idx].main_action() != next_event->main_action() ||
          actions_prev_[idx].assistant_action() != next_event->assistant_action()) {
        continue;
      }
      double dis_off_t = next_event->action_dis() - actions_prev_[idx].action_dis();
      TRAFFIC_LOG << fmt::format("dis_off_t:{:.2f}  ", dis_off_t);
      if (dis_off_t > -40.0 && dis_off_t < 10) {
        dis_offset = dis_off_t;
        idx_target = idx;
        break;
      }
    }

    TRAFFIC_LOG << fmt::format("dis_offset:{:.2f}  idx_target:{}", dis_offset, idx_target);
    for (int idx = std::max(0, idx_target - 1); idx < actions_prev_.size() && idx < idx_target; idx++) {
      double dis_set = actions_prev_[idx].action_dis() + dis_offset;
      if (dis_set < -40.0) {
        continue;
      }
      auto *ptr = actions_all.Add();
      *ptr      = actions_prev_[idx];
      ptr->set_action_dis(dis_set);
      prev_event = std::make_optional(*ptr);
      TRAFFIC_LOG << fmt::format("insert_main_action:{}  ass_action:{}  dis:{:.2f}", ptr->main_action(), ptr->assistant_action(),
                                 ptr->action_dis());
    }
    actions_all.Add(actions_current.begin(), actions_current.end());
    for (const auto &act : actions_current) {
      if (act.action_dis() > 500) {
        continue;
      }
      TRAFFIC_LOG << fmt::format("current_main_action:{}  ass_action:{}  dis:{:.2f}", magic_enum::enum_name(act.main_action()),
                                 magic_enum::enum_name(act.assistant_action()), act.action_dis());
    }
    for (const auto &act : actions_all) {
      if (act.action_dis() > 500) {
        continue;
      }
      TRAFFIC_LOG << fmt::format("all_current_main_action:{}  ass_action:{}  dis:{:.2f}", act.main_action(), act.assistant_action(),
                                 act.action_dis());
    }
  }
  actions_prev_ = actions_all;
  for (int idx = 0; idx + 1 < actions_all.size();) {
    if (actions_all[idx].main_action() == byd::msg::orin::e2e_map::NMA_CONTINUE &&
        (actions_all[idx + 1].main_action() == byd::msg::orin::e2e_map::NMA_TURN_LEFT ||
         actions_all[idx + 1].main_action() == byd::msg::orin::e2e_map::NMA_TURN_RIGHT ||
         actions_all[idx + 1].main_action() == byd::msg::orin::e2e_map::NMA_LEFT_UTURN ||
         actions_all[idx + 1].main_action() == byd::msg::orin::e2e_map::NMA_CONTINUE) &&
        std::fabs(actions_all[idx + 1].action_dis() - actions_all[idx].action_dis()) < 40) {
      actions_all.erase(actions_all.begin() + idx);  // idx stay prev value
    } else {
      ++idx;
    }
  }
  if (prev_event) {
    TRAFFIC_LOG << fmt::format("raw_e2e_header:{}  prev_event_action:{}  ass_action:{}  dis:{:.2f}", e2e_header_seq,
                               prev_event->main_action(), prev_event->assistant_action(), prev_event->action_dis());
  }
  if (next_event) {
    TRAFFIC_LOG << fmt::format("raw_e2e_header:{}  next_event_action:{}  ass_action:{}  dis:{:.2f}", e2e_header_seq,
                               next_event->main_action(), next_event->assistant_action(), next_event->action_dis());
  }
  prev_event = std::nullopt;
  next_event = std::nullopt;
  for (int idx = 0; idx < actions_all.size(); idx++) {
    if (actions_all[idx].action_dis() > 0) {
      next_event = actions_all[idx];
      if (idx > 0) {
        prev_event = actions_all[idx - 1];
      }
      break;
    }
  }
  if (!prev_event && !next_event && !actions_all.empty()) {
    prev_event = actions_all[actions_all.size() - 1];
  }
  if (prev_event) {
    info_traffic_lights_ +=
        fmt::format(" prev_action:{} dis:{:.0f} ", magic_enum::enum_name(prev_event->main_action()), prev_event->action_dis());
    TRAFFIC_LOG << fmt::format("current_e2e_header:{}  prev_event_action:{}  ass_action:{}  dis:{:.2f}", e2e_header_seq,
                               prev_event->main_action(), prev_event->assistant_action(), prev_event->action_dis());
  }
  if (next_event) {
    info_traffic_lights_ +=
        fmt::format(" next_action:{} dis:{:.0f} ", magic_enum::enum_name(next_event->main_action()), next_event->action_dis());
    TRAFFIC_LOG << fmt::format("current_e2e_header:{}  next_event_action:{}  ass_action:{}  dis:{:.2f}", e2e_header_seq,
                               next_event->main_action(), next_event->assistant_action(), next_event->action_dis());
  }
  info_traffic_lights_ += fmt::format("");

  if (!prev_event && !next_event) {
    return res_event;
  }
  if (!prev_event && next_event) {
    return next_event;
  }
  if (prev_event && !next_event) {
    return prev_event;
  }
  constexpr double opt_dis = 35.0;
  if (std::fabs(next_event->action_dis()) < opt_dis) {
    return next_event;
  }
  if (std::fabs(prev_event->action_dis()) < opt_dis) {
    return prev_event;
  }
  if (next_event->action_dis() < 130) {
    TRAFFIC_LOG << fmt::format("prev_next:next");
    return next_event;
  }
  TRAFFIC_LOG << fmt::format("prev_next:prev");
  return prev_event;
}

void TrafficLightMapping::GetIntersectionZone(const BevMapInfoPtr &bev_map) {
  if (bev_map == nullptr) {
    return;
  }
  std::unordered_map<int, int> delete_map;
  for (size_t i = 0; i < bev_map->intersection_zone.size(); i++) {
    auto &zone = bev_map->intersection_zone[i];
    if (zone.line_points.size() < 3) {
      continue;
    }

    std::vector<Vec2d> vec_points;
    for (auto p : zone.line_points) {
      vec_points.emplace_back(p.x, p.y);
    }
    byd::common::math::Polygon2d polygon_tmp = byd::common::math::Polygon2d{vec_points};
    if (polygon_tmp.num_points() == 0) {
      AERROR << "polygon_tmp.num_points():" << polygon_tmp.num_points();
      continue;
    }

    // 交汇区在正前方，就认为是自车会通过的，保存
    if (polygon_tmp.max_x() < 0.0) {
      delete_map[zone.id] = 1;
      continue;
    }
    if (polygon_tmp.min_y() < 0.0 && polygon_tmp.max_y() > 0.0 && polygon_tmp.min_x() < 200.0) {
      if (intersection_zone_map_.count(zone.id) > 0) {
        // 更新消息
        auto &found_zone       = intersection_zone_map_[zone.id];
        found_zone.timestamp_  = bev_map->header.timestamp;
        found_zone.max_length_ = (double(std::abs(polygon_tmp.max_x() - polygon_tmp.min_x())) * 0.2 + found_zone.max_length_ * 0.8);
        found_zone.polygon_    = polygon_tmp;
      } else {
        // 新增交汇区域
        IntersectionZoneMsg msg;
        msg.id_                         = zone.id;
        msg.timestamp_                  = bev_map->header.timestamp;
        msg.max_length_                 = std::abs(polygon_tmp.max_x() - polygon_tmp.min_x());
        msg.polygon_                    = polygon_tmp;
        intersection_zone_map_[zone.id] = msg;
      }
    }
  }

  // 管理生命周期
  for (auto it = intersection_zone_map_.begin(); it != intersection_zone_map_.end();) {
    if (bev_map->header.timestamp - it->second.timestamp_ > 2.0 || delete_map.count(it->second.id_)) {
      it = intersection_zone_map_.erase(it);
    } else {
      ++it;
    }
  }
  TRAFFIC_LOG << "intersection_zone_map_.size():" << intersection_zone_map_.size();
}
// edit by liumianli
void TrafficLightMapping::BindLightsByGroundMarkings(const std::vector<cem::message::env_model::StopLine> &stopline,
                                                     const BevMapInfoPtr                                  &bev_map) {
  if (bev_map == nullptr || perception_traffic_light_info_ == nullptr) {
    return;
  }
  // 通过路口面，召回可以与此路口绑定的红绿灯目标
  double now_time = bev_map->header.timestamp;
  TRAFFIC_LOG << "bev_map time:" << std::to_string(bev_map->header.timestamp);
  // 解析路口面
  GetIntersectionZone(bev_map);

  // 将历史最新转到当前坐标系
  TransformIntersection(now_time);

  // 遍历接收到的红绿灯目标，根据距离，将灯绑定到此路口面
  for (auto &[id, zone] : intersection_zone_map_) {
    TRAFFIC_LOG << "将灯绑定到路口面,zone id:" << id;
    auto &bind_lights = zone.bind_lights_map_;
    bind_lights.clear();
    // 小路口面，30m以内的灯都可以绑定
    double dist_threshold = std::max(zone.max_length_ * 1.2, 30.0);
    // 遍历红绿灯，判断是否和交汇区域绑定
    for (auto &light : perception_traffic_light_info_->objects) {
      Vec2d light_pos{light.position.x, light.position.y};
      // 灯的位置相比路口面更靠近自车超过13m，则不绑定此灯与路口面
      if ((zone.polygon_.min_x() - 16.0) > light_pos.x() || bind_lights.count(light.id) > 0) {
        continue;
      }

      double dist = DBL_MAX;
      // 当前帧丢失路口面，使用转换到当前帧的路口面
      if (now_time > zone.timestamp_ + 0.05) {
        if (zone.polygon_trans_.num_points() > 2 && now_time - zone.timestamp_trans_ < 0.05) {
          dist = zone.polygon_trans_.DistanceTo(light_pos);
        }
      } else {
        dist = zone.polygon_.DistanceTo(light_pos);
      }

      if (zone.lights_dis_map_.count(light.id) > 0) {
        zone.lights_dis_map_[light.id] = (dist * 0.2 + zone.lights_dis_map_[light.id] * 0.8);
      } else {
        zone.lights_dis_map_[light.id] = dist;
      }
      TRAFFIC_LOG << "bind light.id:" << light.id << ", dist:" << zone.lights_dis_map_[light.id] << ", thres:" << dist_threshold;
      if (zone.lights_dis_map_[light.id] < dist_threshold) {
        bind_lights[light.id] = zone.lights_dis_map_[light.id];
      }
    }
  }
  for (auto &[id, zone] : intersection_zone_map_) {
    TRAFFIC_LOG << "intersection_zone ID:" << id << ", max_length_" << zone.max_length_;
  }
}

// edit by liu.mianli 获取停止线相关的信息
void TrafficLightMapping::GetStopLineMsg(const std::vector<cem::message::env_model::StopLine> &stopline, StopLineEnv &stop_line_msg_) {
  if (!stopline.empty()) {
    stop_line_msg_.line_t_.reserve(stopline.front().dr_line_points.size());
    // auto Twb = DataManager::FindTransform(bev_map->header.timestamp);
    for (const auto &point : stopline.front().dr_line_points) {
      stop_line_msg_.line_t_.emplace_back<Point2D>({point.x, point.y});
      // TransformPoint(&stop_line_msg_.line_t_.back(), Twb);
    }
    for (const auto &point : stopline.front().ego_line_points) {
      stop_line_msg_.line_ego_.emplace_back<Point2D>({point.x, point.y});
      if (point.x > stop_line_msg_.line_max_x_) {
        stop_line_msg_.line_max_x_ = point.x;
      }
    }
  }
  // 取第一个停止线
  if (stop_line_msg_.line_ego_.size() >= 2) {
    Vec2d start{stop_line_msg_.line_ego_[0].x, stop_line_msg_.line_ego_[0].y};
    Vec2d end{stop_line_msg_.line_ego_[1].x, stop_line_msg_.line_ego_[1].y};
    stopline_ego_e2e_ = std::make_optional<LineSegment2d>(start, end);
    TRAFFIC_LOG << "stopline_ego_e2e_ x:" << start.x() << " y:" << start.y();
  }
}

void TrafficLightMapping::BindGroundMarkingsAndStopLine(const BevMapInfoPtr &bev_map) {
  if (!bev_map) {
    return;
  }
  const double bind_dist   = 30.0;
  const double unbind_dist = 45.0;

  // 重新计算is_stopline_zone_valid_
  is_stopline_zone_valid_ = false;
  bind_intersection_zone_dist_.clear();
  unbind_intersection_zone_dist_.clear();
  if (stopline_ego_e2e_ && intersection_zone_map_.size() > 0) {
    for (auto [id, zone] : intersection_zone_map_) {
      if ((bev_map->header.timestamp - zone.timestamp_) > 2.0) {
        continue;
      }
      double dist = 0.5 * (bind_dist + unbind_dist);
      if (bev_map->header.timestamp - zone.timestamp_ > 0.05) {
        if ((bev_map->header.timestamp - zone.timestamp_trans_ < 0.05) && zone.polygon_trans_.num_points() > 2) {
          double dist1 = zone.polygon_trans_.DistanceTo(stopline_ego_e2e_->end());
          double dist2 = zone.polygon_trans_.DistanceTo(stopline_ego_e2e_->start());
          dist         = std::min(dist1, dist2);
        }
      } else {
        double dist1 = zone.polygon_.DistanceTo(stopline_ego_e2e_->end());
        double dist2 = zone.polygon_.DistanceTo(stopline_ego_e2e_->start());
        dist         = std::min(dist1, dist2);
      }

      if (dist < bind_dist) {
        bind_intersection_zone_dist_[id] = dist;
        TRAFFIC_LOG << "stop line bind_intersection_zone_ id:" << id;
      } else if (dist > unbind_dist) {
        unbind_intersection_zone_dist_[id] = dist;
        TRAFFIC_LOG << "stop line unbind_intersection_zone_ id:" << id;
      }
    }
  }
  for (auto &[zone_id, dist] : bind_intersection_zone_dist_) {
    auto zone = intersection_zone_map_[zone_id];
    if (zone.bind_lights_map_.size() > 0) {
      is_stopline_zone_valid_ = true;
    }
  }
}

void TrafficLightMapping::FilterLightsNextJunction(std::vector<cem::message::sensor::TrfObjectInfo> &lights, std::string str) {
  std::vector<std::size_t> need_remove_indexes;
  // 有路口面，但是没有绑灯，使用阈值召回一部分灯，避免路口面过小导致漏绑
  constexpr double dis_max_longitude = 40.0;
  for (std::size_t idx = 0; idx != lights.size(); idx++) {
    auto  &light              = lights[idx];
    bool   bind_this_junction = false;
    double max_len_this       = 0.0;
    for (auto &[zone_id, dist] : bind_intersection_zone_dist_) {
      auto &zone = intersection_zone_map_[zone_id];
      if (zone.bind_lights_map_.count(light.id) > 0) {
        max_len_this       = std::max(max_len_this, zone.max_length_);
        bind_this_junction = true;
      }
    }
    double max_len_next       = 0.0;
    bool   bind_next_junction = false;
    for (auto &[zone_id, dist] : unbind_intersection_zone_dist_) {
      auto &zone = intersection_zone_map_[zone_id];
      if (zone.bind_lights_map_.count(light.id) > 0) {
        max_len_next       = std::max(max_len_next, zone.max_length_);
        bind_next_junction = true;
      }
    }
    // 有当前路口面且没有绑此灯，或者此灯属于下个路口面，距离大于40m则删除
    if (!bind_this_junction && (bind_intersection_zone_dist_.size() > 0 || bind_next_junction)) {
      if (stopline_ego_e2e_->DistanceTo(Vec2d{light.position.x, light.position.y}) > dis_max_longitude) {
        TRAFFIC_LOG << str << ", remove light:" << light.id;
        need_remove_indexes.push_back(idx);
      }
    }
    // 绑多个路口面，判断一下
    else if (bind_this_junction && bind_next_junction) {
      if (max_len_next > max_len_this && stopline_ego_e2e_->DistanceTo(Vec2d{light.position.x, light.position.y}) > dis_max_longitude) {
        TRAFFIC_LOG << str << ", remove light:" << light.id;
        need_remove_indexes.push_back(idx);
      } else {
        TRAFFIC_LOG << str << ", bind both, keep light:" << light.id;
      }
    }
  }
  for (auto it = need_remove_indexes.rbegin(); it != need_remove_indexes.rend(); ++it) {
    lights.erase(lights.begin() + static_cast<int64_t>(*it));
  }
  for (auto light : lights) {
    str = str + std::to_string(light.id) + ", ";
  }
  TRAFFIC_LOG << "exist light: " << str;
}

void TrafficLightMapping::FilterLightsNextJunction() {
  if (!stopline_ego_e2e_ || !lights_info_this_junction_) {
    return;
  }
  FilterLightsNextJunction(perception_traffic_lights_, "perception_traffic_lights_ ");
  FilterLightsNextJunction(lights_info_this_junction_->objects, "lights_info_this_junction_->objects ");
}
int TrafficLightMapping::JudgeJunctionSize(const BevMapInfoPtr &bev_map) {
  // 返回路口是大小信息，0：未知；1：小路口；2.大路口；
  int judge = 2;
  if (!bev_map || stop_line_msg_.line_t_.empty()) {
    return judge;
  }
  // 提取大路口面尺寸
  double zone_max_len = -1.0;
  for (auto &[zone_id, dist] : bind_intersection_zone_dist_) {
    if (intersection_zone_map_[zone_id].max_length_ > 40.0) {
      TRAFFIC_LOG << "路口面较大的大路口：" << intersection_zone_map_[zone_id].max_length_;
      zone_max_len = intersection_zone_map_[zone_id].max_length_;
    }
  }

  // 当前和相邻的任意车道中心线穿越停止线，则为小路口
  std::unordered_map<int, int> calculated_line;
  bool                         all_neighbor_cross = true;
  double                       max_ego_points     = std::numeric_limits<double>::lowest();
  for (const auto &lane : bev_map->lane_infos) {
    if (lane.position == (uint32_t)cem::message::sensor::BevLanePosition::LANE_LOC_OTHER) {
      continue;
    }
    // 纠正大路口的误判，可能存在误检路口面。只要有一条邻近中心线没有穿过路口，则为大路口，否则为小路口
    if (zone_max_len > 0.0 && lane.line_points.back().x - stop_line_msg_.line_max_x_ < zone_max_len && (uint32_t)lane.position <= 4) {
      all_neighbor_cross = false;
    }
    for (const auto &point : lane.line_points) {
      if (point.x > max_ego_points) {
        max_ego_points = point.x;
      }
    }
    calculated_line[lane.id] = 1;
    for (const auto &next_id : lane.next_lane_ids) {
      if (calculated_line.count(lane.id)) {
        continue;
      }
      for (const auto &next_lane : bev_map->lane_infos) {
        if (next_lane.id == next_id) {
          for (const auto &point : next_lane.line_points) {
            if (point.x > max_ego_points) {
              max_ego_points = point.x;
            }
          }
          calculated_line[next_id] = 1;
        }
      }
    }
    if (max_ego_points - stop_line_msg_.line_max_x_ > 15.0) {
      judge = 1;
    }
  }
  TRAFFIC_LOG << "max_ego_points: " << max_ego_points << ", stop_line_msg_.line_max_x: " << stop_line_msg_.line_max_x_;
  TRAFFIC_LOG << "max_ego_points - stop_line_msg_.line_max_x_: " << max_ego_points - stop_line_msg_.line_max_x_;

  if (zone_max_len > 0.0) {  // 若有大路口面，check一下输出
    if (all_neighbor_cross) {
      judge = 1;
    } else {
      judge = 2;
    }
    return judge;
  } else if (judge == 1) {  // 若根据中心线已经判断出是小路口，则返回
    return judge;
  }

  // 以上都不满足，计算断开的中心线之间的距离，若距离小就是小路口面
  double min_other_lane_points = std::numeric_limits<double>::infinity();
  for (const auto &lane : bev_map->lane_infos) {
    if (lane.position != (uint32_t)cem::message::sensor::BevLanePosition::LANE_LOC_OTHER || std::abs(lane.line_points.front().y) > 10.0 ||
        lane.line_points.front().x - stop_line_msg_.line_max_x_ < 0.0 || lane.line_points.back().x - stop_line_msg_.line_max_x_ < 50.0) {
      continue;
    }
    // 与自车平行
    double k = std::abs((lane.line_points.back().y - lane.line_points.front().y)) /
               (std::abs(lane.line_points.back().x - lane.line_points.front().x) + 0.01);
    if (k < 0.08) {
      min_other_lane_points = lane.line_points.front().x;
    }
  }
  TRAFFIC_LOG << "min_other_lane_points: " << min_other_lane_points << ", stop_line_msg_.line_max_x: " << stop_line_msg_.line_max_x_;
  TRAFFIC_LOG << "min_other_lane_points - stop_line_msg_.line_max_x_: " << min_other_lane_points - stop_line_msg_.line_max_x_;
  // other与停止线距离近就是小路口，否则就是大路口
  double dist_threshold = stop_line_msg_.line_max_x_ > 40.0 ? 35 : 30;
  if (min_other_lane_points != std::numeric_limits<double>::infinity()) {
    // 如果断开的other距离较大，则为大路口，否则为小路口
    if ((min_other_lane_points - stop_line_msg_.line_max_x_) < dist_threshold) {
      judge = 1;
    } else {
      judge = 2;
    }
  } else {
    judge = 2;
  }

  return judge;
}

::byd::msg::orin::e2e_map::NaviMainAction LightDir2MainAction(::byd::msg::orin::e2e_map::TrafficLightInfo_LightDir light_dir) {
  switch (light_dir) {
    case ::byd::msg::orin::e2e_map::TrafficLightInfo_LightDir::TrafficLightInfo_LightDir_STRAIGHT:
      return ::byd::msg::orin::e2e_map::NaviMainAction::NMA_STRAIGHT;
    case ::byd::msg::orin::e2e_map::TrafficLightInfo_LightDir::TrafficLightInfo_LightDir_TURN_LEFT:
      return ::byd::msg::orin::e2e_map::NaviMainAction::NMA_TURN_LEFT;
    case ::byd::msg::orin::e2e_map::TrafficLightInfo_LightDir::TrafficLightInfo_LightDir_TURN_RIGHT:
      return ::byd::msg::orin::e2e_map::NaviMainAction::NMA_TURN_RIGHT;
    case ::byd::msg::orin::e2e_map::TrafficLightInfo_LightDir::TrafficLightInfo_LightDir_U_TURN:
      return ::byd::msg::orin::e2e_map::NaviMainAction::NMA_LEFT_UTURN;
    default:
      TRAFFIC_LOG << "default_straight.";
      return ::byd::msg::orin::e2e_map::NaviMainAction::NMA_STRAIGHT;
  };
}

void TrafficLightMapping::SetLightStatus_E2E(const std::vector<cem::message::env_model::StopLine> &stopline, const BevMapInfoPtr &bev_map) {
  info_traffic_lights_ += fmt::format(" prev_dis_to_junction:{:.2f} ", prev_action_dis_);
  prev_action_dis_ = std::numeric_limits<double>::infinity();
  E2EMapRawPtr e2e_map{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(e2e_map);
  if (traffic_light_e2e_ == nullptr || e2e_map == nullptr) {
    e2e_navi_action_previous_.set_main_action(NaviMainAction::NMA_UNKNOWN);
    e2e_navi_action_previous_.set_action_dis(std::numeric_limits<double>::infinity());
    TRAFFIC_LOG << "e2e_bind_traffic_light_bev  "
                << fmt::format("traffic_light_e2e_:{}  e2e_map:{}", fmt::ptr(traffic_light_e2e_), fmt::ptr(e2e_map));
    info_traffic_lights_ += fmt::format(" traffic_light_e2e_:{}  e2e_map:{}", fmt::ptr(traffic_light_e2e_), fmt::ptr(e2e_map));
    return;
  }
  // if (deal_per_traffic_light_objects_.empty()) {
  //   e2e_navi_action_previous_.set_main_action(NaviMainAction::NMA_UNKNOWN);
  //   e2e_navi_action_previous_.set_action_dis(std::numeric_limits<double>::infinity());
  //   TRAFFIC_LOG << "e2e_bind_traffic_light_bev  " << fmt::format("deal_per_traffic_light_objects_ is empty.");
  //   return;
  // }
  const auto &GetSpecTrafficLight = [&](TurnType turn_type) -> std::optional<TrafficLight> {
    const auto &light_t = traffic_lights_.GetLightState(turn_type);
    if (light_t.traffic_obj_info) {
      return light_t;
    }
    return std::nullopt;
  };

  next_event_ = GetActionEvent(e2e_map);

  GetActionBaseRecommendLanes(e2e_map);

  bool is_action_block_failed{false};
  if (next_event_) {
    info_traffic_lights_ +=
        fmt::format(" choosed_event:{} dis:{:.0f} ", magic_enum::enum_name(next_event_->main_action()), next_event_->action_dis());
    SetActionBlur(next_event_.value());
    auto        turn_type    = E2EActionConvertTurnType(next_event_->main_action());
    const auto &light_action = traffic_lights_.GetLightState(turn_type);
    if (light_action.is_valid && light_action.color == message::sensor::TLC_BLOCK_FAILED) {
      is_action_block_failed = true;
    }
  } else {
    info_traffic_lights_ += " no_event";
  }
  info_traffic_lights_ += fmt::format(" e2e_counter:{}", e2e_map->header().sequence_num());
  if (perception_traffic_light_info_) {
    info_traffic_lights_ += fmt::format(" vis_counter:{}", perception_traffic_light_info_->header.cycle_counter);
  }

  TRAFFIC_LOG << fmt::format("e2e_map_counter:{}", e2e_map->header().sequence_num());
  info_traffic_lights_ +=
      fmt::format(" input_stopline_size:{} bev_counter:{} ", stopline.size(), bev_map ? bev_map->header.cycle_counter : 0);

  bool is_stopline_valid = is_stopline_zone_valid_;  // 路口面绑灯有效则有效
  info_traffic_lights_ += fmt::format("  zone_valid:{:d} ", is_stopline_zone_valid_);

  double min_dis = GetJunctionDis();
  double dis_po  = std::numeric_limits<double>::infinity();

  std::optional<TrafficLight> traffic_light_opt = std::nullopt;

  std::vector<TurnType> turn_type_vec{TurnType::NO_TURN, TurnType::LEFT_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};

  std::vector<std::pair<TurnType, double>> turn_type_target;
  for (TurnType turn_type_t : turn_type_vec) {
    traffic_light_opt = GetSpecTrafficLight(turn_type_t);
    if (!traffic_light_opt) {
      continue;
    }
    Eigen::Vector2d traffic_pos{traffic_light_opt->traffic_obj_info->position.x, traffic_light_opt->traffic_obj_info->position.y};
    // traffic_light_dis = traffic_pos.norm();
    if (stopline_ego_e2e_) {
      dis_po = stopline_ego_e2e_->DistanceTo(Vec2d{traffic_pos.x(), traffic_pos.y()});
      TRAFFIC_LOG << fmt::format("dis_po:{:.2f}  turn_type:{}", dis_po, turn_type_t);
      if (dis_po < min_dis) {
        is_stopline_valid = true;
        turn_type_target.emplace_back(turn_type_t, dis_po);
      }
    }
  }
  TurnType target_turn_type = TurnType::OTHER_UNKNOWN;

  std::sort(turn_type_target.begin(), turn_type_target.end(),
            [](std::pair<TurnType, double> lhs, std::pair<TurnType, double> rhs) { return lhs.second < rhs.second; });

  double dis_action          = next_event_ ? next_event_->action_dis() : std::numeric_limits<double>::infinity();
  bool   is_virtual_stopline = false;
  if (stop_line_msg_.line_t_.empty() && dis_action > 20.0 && dis_action < 200 && bev_map) {
    auto Twb = DataManager::FindTransform(bev_map->header.timestamp);
    stop_line_msg_.line_t_.emplace_back<Point2D>({dis_action, -5.0});
    TransformPoint(&stop_line_msg_.line_t_.back(), Twb);
    TRAFFIC_LOG << fmt::format("virtual_stop_line:{:.2f},{:.2f}", stop_line_msg_.line_t_.back().x, stop_line_msg_.line_t_.back().y);
    stop_line_msg_.line_t_.emplace_back<Point2D>({dis_action, 5.0});
    TransformPoint(&stop_line_msg_.line_t_.back(), Twb);
    TRAFFIC_LOG << fmt::format("virtual_stop_line:{:.2f},{:.2f}", stop_line_msg_.line_t_.back().x, stop_line_msg_.line_t_.back().y);
    is_virtual_stopline = true;
  }
  TRAFFIC_LOG << fmt::format(
      "dis_po:{:.2f} min_dis:{:.2f} virtual_jun:{:d} is_stopline_valid:{:d}  is_virtual_stopline:{:d} stopline_ego_size:{} dr_size:{}",
      dis_po, min_dis, is_virtual_junction_, is_stopline_valid, is_virtual_stopline, stop_line_msg_.line_ego_.size(),
      stop_line_msg_.line_t_.size());

  auto SetTrafficLight = [&](TurnType target_turn_type) {
    std::vector<TurnType> turn_type_vec{TurnType::NO_TURN, TurnType::LEFT_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};
    for (TurnType turn_type_t : turn_type_vec) {
      auto       &traffic_light_status = traffic_light_e2e_->traffic_status.emplace_back();
      const auto &light                = traffic_lights_.GetLightState(turn_type_t);
      traffic_light_status.turn_type   = turn_type_t;
      if (!light.is_valid) {
        continue;
      }
      traffic_light_status.is_navi_light            = turn_type_t == target_turn_type;
      traffic_light_status.light_status             = LightColorConvert(light.color, light.traffic_light_flashing);
      traffic_light_status.traffic_light_num        = light.traffic_light_num;
      traffic_light_status.stop_line_pts            = stop_line_msg_.line_t_;
      traffic_light_status.stopline_is_virtual      = is_virtual_stopline;
      traffic_light_status.distance_to_stopline     = traffic_lights_.distance_to_stopline;
      traffic_light_status.traffic_match_cloud_file = traffic_lights_.traffic_match_cloud_file;
      TRAFFIC_LOG << fmt::format("e2e_bind_traffic_light_bev  light_status:{}", traffic_light_status);
    }
  };

  double traffic_light_dis = std::numeric_limits<double>::infinity();

  double dis_threshold = min_dis;
  bool   is_event_valid{false};
  if (next_event_) {
    TurnType action_turn_type = E2EActionConvertTurnType(next_event_->main_action());
    TRAFFIC_LOG << fmt::format("event_action:{} dis:{:.2f}", magic_enum::enum_name(next_event_->main_action()), next_event_->action_dis())
                << "  action_turn_type:" << static_cast<int>(action_turn_type);
    std::vector<TurnType> turn_vec{TurnType::LEFT_TURN, TurnType::NO_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};
    for (const auto &turn_t : turn_vec) {
      traffic_light_opt = GetSpecTrafficLight(turn_t);
      if (traffic_light_opt && traffic_light_opt->traffic_obj_info) {
        Eigen::Vector2d traffic_pos{traffic_light_opt->traffic_obj_info->position.x, traffic_light_opt->traffic_obj_info->position.y};
        traffic_light_dis = traffic_pos.norm();
        is_event_valid |= std::fabs(next_event_->action_dis() - traffic_light_dis) < dis_threshold;
        TRAFFIC_LOG << fmt::format(
            "action_dis:{:.2f}  traffic_light_dis:{:.2f} obj:{} pos:{:.2f}-{:.2f} color:{}", next_event_ ? next_event_->action_dis() : -1.0,
            traffic_light_dis, traffic_light_opt->traffic_obj_info->id, traffic_light_opt->traffic_obj_info->position.x,
            traffic_light_opt->traffic_obj_info->position.y, traffic_light_opt->traffic_obj_info->attributes.traffic_light_color);
        if (is_event_valid) {
          break;
        }
      }
    }
  }
  if (next_event_) {
    is_event_valid |= std::fabs(next_event_->action_dis() - stopline_ego_e2e_->start().Length()) < dis_threshold;
    is_event_valid |= std::fabs(next_event_->action_dis() - stopline_ego_e2e_->end().Length()) < dis_threshold;
  }

  info_traffic_lights_ += fmt::format(
      "  dis_obj:{:.2f} dis_stop:{:.2f} virtual_jun:{:d}  valid-event:{:d}  stopline:{:d} virutal:{:d} block_failed:{:d} ",
      traffic_light_dis, dis_po, is_virtual_junction_, is_event_valid, is_stopline_valid, is_virtual_stopline, is_action_block_failed);
  bool is_settled{false};
  prev_action_dis_ = next_event_->action_dis();
  if (next_event_ &&
      ((std::fabs(next_event_->action_dis() - traffic_light_dis) < dis_threshold && (is_stopline_valid || is_virtual_stopline)) ||
       (is_action_block_failed && !stop_line_msg_.line_t_.empty()))) {
    auto turn_type = E2EActionConvertTurnType(next_event_->main_action());
    TRAFFIC_LOG << fmt::format(
        "e2e_bind_traffic_light_bev  target_turn:{}  is_event_valid:{:d} is_stopline_valid:{:d}  is_virtual_stopline:{:d}  "
        "is_action_block_failed:{:d}",
        magic_enum::enum_name(turn_type), is_event_valid, is_stopline_valid, is_virtual_stopline, is_action_block_failed);
    info_traffic_lights_ += fmt::format(" event_0 turn:{} ", turn_type);
    is_settled = true;
    SetTrafficLight(turn_type);
  }
  if (!is_settled && is_stopline_valid) {
    if (next_event_) {
      if (!is_event_valid && std::fabs(next_event_->action_dis()) > 60) {
        next_event_->set_main_action(::byd::msg::orin::e2e_map::NaviMainAction::NMA_CONTINUE);
      }
      target_turn_type = E2EActionConvertTurnType(next_event_->main_action());
    } else {
      target_turn_type = TurnType::NO_TURN;
    }
    TRAFFIC_LOG << fmt::format(
        "e2e_bind_traffic_light_bev  target_turn:{}  is_event_valid:{:d} is_stopline_valid:{:d}  is_virtual_stopline:{:d}  "
        "is_action_block_failed:{:d}",
        magic_enum::enum_name(target_turn_type), is_event_valid, is_stopline_valid, is_virtual_stopline, is_action_block_failed);
    info_traffic_lights_ += fmt::format(" no_event turn:{} ", target_turn_type);
    SetTrafficLight(target_turn_type);
  } else if (e2e_navi_action_previous_.main_action() != NaviMainAction::NMA_UNKNOWN) {
    auto turn_type = E2EActionConvertTurnType(e2e_navi_action_previous_.main_action());
    TRAFFIC_LOG << "e2e_bind_traffic_light_bev  " << fmt::format("previous_turn:{}", magic_enum::enum_name(turn_type));
    info_traffic_lights_ += " prev_event ";
    SetTrafficLight(turn_type);
  } else {
    e2e_navi_action_previous_.set_main_action(NaviMainAction::NMA_UNKNOWN);
    e2e_navi_action_previous_.set_action_dis(std::numeric_limits<double>::infinity());
    info_traffic_lights_ += " default ";
    double action_dis = 0.0;
    if (next_event_) {
      action_dis = next_event_->action_dis();
    }
    TRAFFIC_LOG << "e2e_bind_traffic_light_bev  is default  "
                << fmt::format("prev_action:{} action_dis:{:.2f}  calc_dis:{:.2f}", e2e_navi_action_previous_.main_action(), action_dis,
                               traffic_light_dis);
  }
  if (bev_map) {
    const auto &position       = traffic_light_opt && traffic_light_opt->traffic_obj_info ? traffic_light_opt->traffic_obj_info->position
                                                                                          : cem::message::common::Point3DD(0, 0, 0);
    traffic_light_e2e_->pose.x = position.x;
    traffic_light_e2e_->pose.y = position.y;
    traffic_light_e2e_->pose.z = position.z;
    auto Twb                   = DataManager::FindTransform(bev_map->header.timestamp);
    TransformPoint(&traffic_light_e2e_->pose, Twb);
    traffic_light_e2e_->pose.z = position.z;
    TRAFFIC_LOG << fmt::format("traffic_light_e2e_position_info  obj_id:{}  raw_pos:{:.2f},{:.2f}  dr_pos:{:.2f}  dr_pos:{:.2f}",
                               deal_per_traffic_light_objects_.empty() ? 0 : deal_per_traffic_light_objects_.begin()->first, position.x,
                               position.y, traffic_light_e2e_->pose.x, traffic_light_e2e_->pose.y);
  }
}

void TrafficLightMapping::ProcessFlashing(const TrafficLight &traffic_light, TrafficLight &rhs) {
  if (!traffic_light.is_valid) {
    return;
  }
  // if (traffic_light.color == TrafficLightColorType::TLC_BLOCK_FAILED) {
  //   rhs.color = TrafficLightColorType::TLC_BLOCK_FAILED;
  //   return;
  // }
  if (traffic_light.color == TrafficLightColorType::TLC_RED_FLASHING &&
      (rhs.color == TrafficLightColorType::TLC_RED || rhs.color == TrafficLightColorType::TLC_NONE_LIGHT ||
       rhs.color == TrafficLightColorType::TLC_UNKNOWN)) {
    rhs.color = TrafficLightColorType::TLC_RED_FLASHING;
  } else if (traffic_light.color == TrafficLightColorType::TLC_GREEN_FLASHING &&
             (rhs.color == TrafficLightColorType::TLC_GREEN || rhs.color == TrafficLightColorType::TLC_NONE_LIGHT ||
              rhs.color == TrafficLightColorType::TLC_UNKNOWN)) {
    rhs.color = TrafficLightColorType::TLC_GREEN_FLASHING;
  } else if (traffic_light.color == TrafficLightColorType::TLC_YELLOW_FLASHING &&
             (rhs.color == TrafficLightColorType::TLC_YELLOW || rhs.color == TrafficLightColorType::TLC_NONE_LIGHT ||
              rhs.color == TrafficLightColorType::TLC_UNKNOWN)) {
    rhs.color = TrafficLightColorType::TLC_YELLOW_FLASHING;
  }
};

cem::message::sensor::TrafficLightColorType ColorConvert_TIN(const byd::msg::perception::TrafficLightInfo rhs, bool is_flash) {
  is_flash = false;
  switch (rhs) {
    case byd::msg::perception::TrafficLightInfo::RED_LIGHT:
      if (is_flash) {
        return cem::message::sensor::TrafficLightColorType::TLC_RED_FLASHING;
      }
      return cem::message::sensor::TrafficLightColorType::TLC_RED;
    case byd::msg::perception::TrafficLightInfo::GREEN_LIGHT:
      if (is_flash) {
        return cem::message::sensor::TrafficLightColorType::TLC_GREEN_FLASHING;
      }
      return cem::message::sensor::TrafficLightColorType::TLC_GREEN;
    case byd::msg::perception::TrafficLightInfo::YELLOW_LIGHT:
      if (is_flash) {
        return cem::message::sensor::TrafficLightColorType::TLC_YELLOW_FLASHING;
      }
      return cem::message::sensor::TrafficLightColorType::TLC_YELLOW;
    case byd::msg::perception::TrafficLightInfo::NONE_LIGHT:
    case byd::msg::perception::TrafficLightInfo::OTHER_LIGHT:
      return cem::message::sensor::TrafficLightColorType::TLC_NONE_LIGHT;
    default: {
      return cem::message::sensor::TrafficLightColorType::TLC_UNKNOWN;
    }
  }
  return cem::message::sensor::TrafficLightColorType::TLC_UNKNOWN;
};

void TrafficLightMapping::IsLightBlockFailed_TIN(const TrafficLight &light_prev, TrafficLight &light) {
  if (prev_action_dis_ < -30 || prev_action_dis_ > 80.0) {
    return;
  }
  if (light.turn_type != TurnType::RIGHT_TURN) {
    if (light_prev.color == TrafficLightColorType::TLC_UNKNOWN || light_prev.color == TrafficLightColorType::TLC_NONE_LIGHT ||
        light_prev.color == TrafficLightColorType::TLC_OTHER) {
      return;
    }
    if (light.color != TrafficLightColorType::TLC_UNKNOWN && light.color != TrafficLightColorType::TLC_NONE_LIGHT &&
        light.color != TrafficLightColorType::TLC_OTHER) {
      return;
    }
    light.color = TrafficLightColorType::TLC_BLOCK_FAILED;
    return;
  }
  bool is_curr_invalid =
      (light.color == TrafficLightColorType::TLC_GREEN && light.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) ||
      (light.color == TrafficLightColorType::TLC_GREEN_FLASHING &&
       light.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) ||
      light.color == TrafficLightColorType::TLC_UNKNOWN || light.color == TrafficLightColorType::TLC_NONE_LIGHT ||
      light.color == TrafficLightColorType::TLC_OTHER;
  bool is_prev_invalid = (light_prev.color == TrafficLightColorType::TLC_GREEN &&
                          light_prev.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) ||
                         (light_prev.color == TrafficLightColorType::TLC_GREEN_FLASHING &&
                          light_prev.traffic_reason == byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ) ||
                         light_prev.color == TrafficLightColorType::TLC_UNKNOWN ||
                         light_prev.color == TrafficLightColorType::TLC_NONE_LIGHT || light_prev.color == TrafficLightColorType::TLC_OTHER;
  TRAFFIC_LOG << fmt::format("light_corlor:{}  reason:{}  prev_color:{} reason:{} is_cur_valid:{:d} prev_invalid:{:d}", light.color,
                             light.traffic_reason, light_prev.color, light_prev.traffic_reason, is_curr_invalid, is_prev_invalid);
  if (!is_prev_invalid && is_curr_invalid) {
    light.color = TrafficLightColorType::TLC_BLOCK_FAILED;
    return;
  }
}

void TrafficLightMapping::LightColorFilter(const std::deque<TrafficLights> &traffic_lights_raw, TrafficLight &curr_lights) {
  const std::size_t max_light_size = 5;
  if (!curr_lights.is_valid || traffic_lights_raw.size() < max_light_size) {
    return;
  }
  if (curr_lights.color != message::sensor::TLC_GREEN && curr_lights.color != message::sensor::TLC_GREEN_FLASHING) {
    return;
  }
  std::size_t green_counter       = 0;
  std::size_t green_flash_counter = 0;

  std::vector<std::size_t> vec_counter;
  TrafficLight             traffic_light_temp{};
  for (std::size_t idx = 0; idx < traffic_lights_raw.size() && idx < max_light_size; idx++) {
    if (traffic_lights_raw[idx].straight.turn_type == curr_lights.turn_type) {
      traffic_light_temp = traffic_lights_raw[idx].straight;
    } else if (traffic_lights_raw[idx].left.turn_type == curr_lights.turn_type) {
      traffic_light_temp = traffic_lights_raw[idx].left;
    } else if (traffic_lights_raw[idx].u_turn.turn_type == curr_lights.turn_type) {
      traffic_light_temp = traffic_lights_raw[idx].u_turn;
    } else if (traffic_lights_raw[idx].right.turn_type == curr_lights.turn_type) {
      traffic_light_temp = traffic_lights_raw[idx].right;
    }
    if (traffic_light_temp.color == message::sensor::TLC_GREEN) {
      green_counter++;
      vec_counter.push_back(0);
    } else if (traffic_light_temp.color == message::sensor::TLC_GREEN_FLASHING) {
      green_flash_counter++;
      vec_counter.push_back(1);
    } else {
      break;
    }
  }
  if (green_counter + green_flash_counter < 3) {
    curr_lights.color = message::sensor::TLC_GREEN;
  } else if (green_counter > green_flash_counter) {
    curr_lights.color = message::sensor::TLC_GREEN;
  } else if (green_counter < green_flash_counter) {
    curr_lights.color = message::sensor::TLC_GREEN_FLASHING;
  }
  std::string temp_info;

  temp_info = fmt::format(" counter_green:{};turn:{} ", vec_counter, static_cast<int>(curr_lights.turn_type));
  info_traffic_lights_ += temp_info;
  TRAFFIC_LOG << temp_info;
}

void TrafficLightDequeStay(std::deque<TrafficLights> &traffic_lights_deque) {
  const std::size_t max_deque_size = 30;
  if (traffic_lights_deque.size() > max_deque_size) {
    traffic_lights_deque.pop_back();
  }
}

bool ColorIsEqual(const cem::message::sensor::TrafficLightColorType &lhs, const cem::message::sensor::TrafficLightColorType &rhs) {
  auto NormalizeColor = [](const cem::message::sensor::TrafficLightColorType &color) {
    switch (color) {
      case TrafficLightColorType::TLC_GREEN_FLASHING:
        return TrafficLightColorType::TLC_GREEN;
      case TrafficLightColorType::TLC_YELLOW_FLASHING:
        return TrafficLightColorType::TLC_YELLOW;
      case TrafficLightColorType::TLC_RED_FLASHING:
        return TrafficLightColorType::TLC_RED;
      default:
        return color;
    }
  };

  return NormalizeColor(lhs) == NormalizeColor(rhs);
}

void TrafficLightMapping::SetLightStatus_E2E_TIN(const std::vector<cem::message::env_model::StopLine> &stopline,
                                                 const BevMapInfoPtr                                  &bev_map) {
  static TrafficLights             traffic_lights_prev;
  static std::deque<TrafficLights> traffic_lights_raw;
  TrafficLightDequeStay(traffic_lights_raw);

  E2EMapRawPtr e2e_map{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(e2e_map);

  if (traffic_light_e2e_ == nullptr || tin_e2e_result_ == nullptr) {
    e2e_navi_action_previous_.set_main_action(NaviMainAction::NMA_UNKNOWN);
    e2e_navi_action_previous_.set_action_dis(std::numeric_limits<double>::infinity());
    TRAFFIC_LOG << "e2e_bind_traffic_light_bev  "
                << fmt::format("traffic_light_e2e_:{}  tin_e2e_result_:{}  e2e_map:{}", fmt::ptr(traffic_light_e2e_),
                               fmt::ptr(tin_e2e_result_), fmt::ptr(e2e_map));
    return;
  }
  std::vector<Point2D> stop_line_t;
  if (!stopline.empty()) {
    stop_line_t.reserve(stopline.front().dr_line_points.size());
    // auto Twb = DataManager::FindTransform(bev_map->header.timestamp);
    for (const auto &point : stopline.front().dr_line_points) {
      stop_line_t.emplace_back<Point2D>({point.x, point.y});
      // TransformPoint(&stop_line_t.back(), Twb);
    }
  }

  double dis_action          = next_event_ ? next_event_->action_dis() : std::numeric_limits<double>::infinity();
  bool   is_virtual_stopline = false;
  if (stop_line_t.empty() && dis_action > 20.0 && dis_action < 200 && bev_map) {
    auto Twb = DataManager::FindTransform(bev_map->header.timestamp);
    stop_line_t.emplace_back<Point2D>({dis_action, -5.0});
    TransformPoint(&stop_line_t.back(), Twb);
    TRAFFIC_LOG << fmt::format("virtual_stop_line:{:.2f},{:.2f}", stop_line_t.back().x, stop_line_t.back().y);
    stop_line_t.emplace_back<Point2D>({dis_action, 5.0});
    TransformPoint(&stop_line_t.back(), Twb);
    TRAFFIC_LOG << fmt::format("virtual_stop_line:{:.2f},{:.2f}", stop_line_t.back().x, stop_line_t.back().y);
    is_virtual_stopline = true;
  }

  TrafficLights traffic_light_e2e_0 = traffic_lights_;

  traffic_lights_ = TrafficLights();

  constexpr uint max_light_num = 900;

  TRAFFIC_LOG << fmt::format("tin_counter:{}", tin_e2e_result_->header().sequence_num());
  info_traffic_lights_ += fmt::format(" counter TIN:{} ", tin_e2e_result_->header().sequence_num());
  if (perception_traffic_light_info_) {
    info_traffic_lights_ += fmt::format(" VIS:{} ", perception_traffic_light_info_->header.cycle_counter);
  }
  FindFinalTINResult(tin_e2e_result_, lights_info_this_junction_, is_virtual_stopline);

  info_traffic_lights_ += fmt::format(" dis_to_junc:{:.2f}", distance_to_junction_prev_);
  for (const auto &tin_light : tin_e2e_result_->e2e_light_states_filtered()) {
    TRAFFIC_LOG << fmt::format("direction:{} color:{} flash:{:d}  counter:{} prev_action_dis_:{:.2f}",
                               magic_enum::enum_name(tin_light.direction()), magic_enum::enum_name(tin_light.trafficlight_info()),
                               tin_light.trafficlight_flashing(), tin_light.trafficlight_countdown(), prev_action_dis_);
    if (tin_light.has_direction() && tin_light.direction() == byd::msg::perception::LEFT_TURN && tin_light.has_trafficlight_info()) {
      traffic_lights_.left.is_valid          = true;
      traffic_lights_.left.color             = final_tin.left.color;
      traffic_lights_.left.traffic_light_num = 4294967295;  //tin_light.trafficlight_countdown();
      if (traffic_light_e2e_0.left.is_valid && traffic_light_e2e_0.left.traffic_light_num < max_light_num &&
          ColorIsEqual(traffic_light_e2e_0.left.color, traffic_lights_.left.color)) {
        traffic_lights_.left.traffic_light_num = traffic_light_e2e_0.left.traffic_light_num;
      }
    }
    if (tin_light.has_direction() && tin_light.direction() == byd::msg::perception::LEFT_U_TURN && tin_light.has_trafficlight_info()) {
      traffic_lights_.u_turn.is_valid          = true;
      traffic_lights_.u_turn.color             = final_tin.u_turn.color;
      traffic_lights_.u_turn.traffic_light_num = 4294967295;  //tin_light.trafficlight_countdown();
      if (traffic_light_e2e_0.u_turn.is_valid && traffic_light_e2e_0.u_turn.traffic_light_num < max_light_num &&
          ColorIsEqual(traffic_light_e2e_0.left.color, traffic_lights_.left.color)) {
        traffic_lights_.u_turn.traffic_light_num = traffic_light_e2e_0.u_turn.traffic_light_num;
      }
    }
    if (tin_light.has_direction() && tin_light.direction() == byd::msg::perception::GO_STRAIGHT && tin_light.has_trafficlight_info()) {
      traffic_lights_.straight.is_valid          = true;
      traffic_lights_.straight.color             = final_tin.straight.color;
      traffic_lights_.straight.traffic_light_num = 4294967295;  // tin_light.trafficlight_countdown();
      if (traffic_light_e2e_0.straight.is_valid && traffic_light_e2e_0.straight.traffic_light_num < max_light_num &&
          ColorIsEqual(traffic_light_e2e_0.left.color, traffic_lights_.left.color)) {
        traffic_lights_.straight.traffic_light_num = traffic_light_e2e_0.straight.traffic_light_num;
      }
    }
    if (tin_light.has_direction() && tin_light.direction() == byd::msg::perception::RIGHT_TURN && tin_light.has_trafficlight_info()) {
      traffic_lights_.right.is_valid = true;
      traffic_lights_.right.color    = final_tin.right.color;
      if (final_tin.right.color == cem::message::sensor::TrafficLightColorType::TLC_UNKNOWN ||
          final_tin.right.color == cem::message::sensor::TrafficLightColorType::TLC_NONE_LIGHT) {
        traffic_lights_.right.color          = cem::message::sensor::TrafficLightColorType::TLC_GREEN;
        traffic_lights_.right.traffic_reason = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
      }
      traffic_lights_.right.traffic_light_num = 4294967295;  //tin_light.trafficlight_countdown();
      if (traffic_light_e2e_0.right.is_valid && traffic_light_e2e_0.right.traffic_light_num < max_light_num &&
          ColorIsEqual(traffic_light_e2e_0.left.color, traffic_lights_.left.color)) {
        traffic_lights_.right.traffic_light_num = traffic_light_e2e_0.right.traffic_light_num;
      }
    }
  }

  auto SetTrafficLight = [&](TurnType target_turn_type) {
    std::vector<TurnType> turn_type_vec{TurnType::NO_TURN, TurnType::LEFT_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};
    for (TurnType turn_type_t : turn_type_vec) {
      auto       &traffic_light_status = traffic_light_e2e_->traffic_status.emplace_back();
      const auto &light                = traffic_lights_.GetLightState(turn_type_t);
      traffic_light_status.turn_type   = turn_type_t;
      if (!light.is_valid) {
        continue;
      }
      traffic_light_status.is_navi_light            = turn_type_t == target_turn_type;
      traffic_light_status.light_status             = LightColorConvert(light.color, light.traffic_light_flashing);
      traffic_light_status.traffic_light_num        = light.traffic_light_num;
      traffic_light_status.stop_line_pts            = stop_line_t;
      traffic_light_status.stopline_is_virtual      = is_virtual_stopline;
      traffic_light_status.distance_to_stopline     = traffic_light_e2e_0.distance_to_stopline;
      traffic_light_status.traffic_match_cloud_file = traffic_lights_.traffic_match_cloud_file;
      if (!stopline_ego_e2e_ && !is_virtual_stopline) {
        traffic_light_status.light_status = message::env_model::LightStatus::NONE_LIGHT;
      }
      TRAFFIC_LOG << fmt::format("e2e_bind_traffic_light_bev  light_status:{}", traffic_light_status);
    }
  };

  traffic_lights_raw.push_front(traffic_lights_);
  LightColorFilter(traffic_lights_raw, traffic_lights_.u_turn);
  LightColorFilter(traffic_lights_raw, traffic_lights_.left);
  LightColorFilter(traffic_lights_raw, traffic_lights_.straight);
  LightColorFilter(traffic_lights_raw, traffic_lights_.right);
  traffic_lights_prev = traffic_lights_;

  auto turn_type = cem::message::env_model::TurnType::NO_TURN;
  if (next_event_) {
    turn_type = E2EActionConvertTurnType(next_event_->main_action());
  }
  TRAFFIC_LOG << "e2e_bind_traffic_light_bev  " << fmt::format("target_turn:{}", magic_enum::enum_name(turn_type));
  SetTrafficLight(turn_type);
  if (tin_e2e_result_->has_header() && tin_e2e_result_->header().has_sequence_num()) {
    fmt::memory_buffer buf;
    fmt::format_to(buf, " e2e_traffic_info:{}", tin_e2e_result_->header().sequence_num());
    // info_traffic_lights_ += fmt::to_string(buf);
    TRAFFIC_LOG << info_traffic_lights_;
  }
}

bool TrafficLightMapping::IsTrafficLightValid(const TrfObjectInfo &obj) {
  return stopline_ego_e2e_ && stopline_ego_e2e_->DistanceTo(Vec2d{obj.position.x, obj.position.y}) < GetJunctionDis();
}

bool TrafficLightMapping::IsTrafficLightBindZone(const TrfObjectInfo &obj) {
  if (bind_intersection_zone_dist_.size() == 0) {
    return false;
  }
  for (auto &[zone_id, dist] : bind_intersection_zone_dist_) {
    if (intersection_zone_map_[zone_id].bind_lights_map_.count(obj.id) > 0) {
      return true;
    }
  }
  return false;
}

TurnType TrafficLightMapping::ConvertTINTurnType(byd::msg::perception::DriveDirection direction) {
  switch (direction) {
    case byd::msg::perception::DriveDirection::LEFT_TURN:
      return TurnType::LEFT_TURN;
    case byd::msg::perception::DriveDirection::RIGHT_TURN:
      return TurnType::RIGHT_TURN;
    case byd::msg::perception::DriveDirection::GO_STRAIGHT:
      return TurnType::NO_TURN;
    case byd::msg::perception::DriveDirection::LEFT_U_TURN:
      return TurnType::U_TURN;
    default:
      return TurnType::OTHER_UNKNOWN;
  };
};

void TrafficLightMapping::GetMatchLights(const PercepTrfInfoPtr                   &lights_info_this_junction,
                                         const std::vector<TrafficLightShapeType> &light_shape, TinLight &tin_light,
                                         bool is_virtual_stopline) {
  if (!tin_light.is_valid) {
    return;
  }
  TRAFFIC_LOG << "bind zone size:" << bind_intersection_zone_dist_.size();

  for (const auto &obj : lights_info_this_junction->objects) {
    auto it = std::find(light_shape.begin(), light_shape.end(), obj.attributes.traffic_light_shape);
    if (it == light_shape.end()) {
      TRAFFIC_LOG << "no_shape.";
      continue;
    }
    bool is_traffic_light_valid = IsTrafficLightValid(obj);
    bool is_traffic_light_bind  = IsTrafficLightBindZone(obj);
    TRAFFIC_LOG << "obj->id:" << obj.id << ", dist valid:" << is_traffic_light_valid << ", bind zone valid:" << is_traffic_light_bind;
    // 当没有路口面时，使用大小路口判断
    if ((is_traffic_light_valid) || is_traffic_light_bind || is_virtual_stopline || (!stopline_ego_e2e_)) {
      tin_light.match_objs.push_back(obj);
      continue;
    }
    TRAFFIC_LOG << fmt::format(
        "bev_valid:{:d} virtual_valid:{:d} obj:{}  dis:{:.2f}  ass_jun_dis:{:.2f} ", is_traffic_light_valid, is_virtual_stopline, obj.id,
        stopline_ego_e2e_ ? stopline_ego_e2e_->DistanceTo(Vec2d{obj.position.x, obj.position.y}) : 400.0, GetJunctionDis());
  }
};

void TrafficLightMapping::FilterByChronology(TinLight &tin_light, std::string &binding_info) {
  tin_light.final_matched_objs.clear();
  for (const auto &pair : tin_light.match_light_records) {
    int              obj_id        = pair.first;
    std::vector<int> light_records = pair.second;
    bool             is_valid_obj  = true;
    // 检查是否存在连续两个4
    for (size_t i = 1; i < light_records.size(); ++i) {
      if (light_records[i] == 4 && light_records[i - 1] == 4) {
        is_valid_obj = false;
        break;
      }
    }
    // 使用TRAFFIC_LOG打印每个灯箱ID及其对应的整数列表
    std::string records_str = fmt::format("TIN LightBox ID: {}, records:", obj_id);
    for (int record : light_records) {
      records_str += fmt::format(" {}", record);
    }
    TRAFFIC_LOG << records_str;
    if (!is_valid_obj)
      continue;
    auto valid_pair = tin_light.match_light_valid_records.find(obj_id);
    if (valid_pair != tin_light.match_light_valid_records.end()) {
      const std::vector<int> &valid_records = valid_pair->second;
      if (!valid_records.empty()) {
        int whole_one_count   = std::count(valid_records.begin(), valid_records.end(), 1);
        int whole_zero_count  = std::count(valid_records.begin(), valid_records.end(), 0);
        int whole_valid_count = whole_one_count + whole_zero_count;
        int one_count = std::count(valid_records.end() - std::min(20, static_cast<int>(valid_records.size())), valid_records.end(), 1);
        int zero_count = std::count(valid_records.end() - std::min(20, static_cast<int>(valid_records.size())), valid_records.end(), 0);
        int valid_count = one_count + zero_count;
        if (valid_count > 0) {
          float valid_score = static_cast<float>(one_count) / valid_count;
          float whole_valid_score = static_cast<float>(whole_one_count) / whole_valid_count;
          TRAFFIC_LOG << fmt::format("The Valid Light ID is: {}, with a score of: {:.2f}", obj_id, valid_score);
          if (valid_score > 0.7 && valid_count > 2) {
            auto it = tin_light.match_obj_records.find(obj_id);
            if (it != tin_light.match_obj_records.end() && !it->second.empty()) {
              auto &obj = it->second.back();
              tin_light.final_matched_objs.push_back(obj);
              TRAFFIC_LOG << fmt::format("The Filter Light ID is: {}, with a score of: {:.2f}, flash:{}", obj.id, valid_score,
                                         obj.attributes.traffic_light_flashing);
            }
          }
          tin_light.final_matched_scores[obj_id] = {whole_valid_score, whole_valid_count};
        }
      } else {
        tin_light.final_matched_scores[obj_id] = {0.0, 0};
      }
    } else {
      tin_light.final_matched_scores[obj_id] = {0.0, 0};
    }
    // 如果都不满足，认为是TIN混乱导致，送入全部目标
    // if (tin_light.final_matched_objs.size() == 0){
    //   tin_light.final_matched_objs = tin_light.match_objs
    // }
  }
  // 打印过阈值前所有目标的分数；
  for (auto it = tin_light.final_matched_scores.begin(); it != tin_light.final_matched_scores.end(); ++it) {
    int obj_id = it->first;
    float score = it->second.first;
    binding_info += fmt::format("{}:{:.2f}", obj_id, score);
    if (std::next(it) != tin_light.final_matched_scores.end()) {
        binding_info += ",";
    }
  }
  binding_info += "]";
}

bool TrafficLightMapping::BindingTrafficlightToTIN(TinLight &tin_light) {
  if (tin_light.final_matched_objs.empty()) {
    return false;
  }

  std::vector<std::pair<float, cem::message::sensor::TrfObjectInfo>> sorted_objs;
  for (const auto &obj : tin_light.final_matched_objs) {
    auto it = tin_light.final_matched_scores.find(obj.id);
    if (it != tin_light.final_matched_scores.end()) {
      sorted_objs.emplace_back(it->second.first, obj);
    }
  }
  std::sort(sorted_objs.begin(), sorted_objs.end(),
            [](const std::pair<float, cem::message::sensor::TrfObjectInfo> &a,
               const std::pair<float, cem::message::sensor::TrfObjectInfo> &b) { return a.first > b.first; });
  if (sorted_objs.empty()) {
    return false;
  }

  tin_light.final_matched_obj = sorted_objs.front().second;
  float max_score             = sorted_objs.front().first;

  // 兜底遮挡后目标ID出现分裂的情况
  if (tin_light.final_matched_obj.attributes.traffic_light_color == TLC_UNKNOWN) {
    for (size_t i = 1; i < sorted_objs.size(); i++) {
      const auto &scored_obj  = sorted_objs[i].second;
      float       obj_score   = sorted_objs[i].first;
      float       score_ratio = obj_score / max_score;
      if (scored_obj.attributes.traffic_light_color != TLC_UNKNOWN && score_ratio > 0.9 &&
          scored_obj.attributes.traffic_light_shape == tin_light.final_matched_obj.attributes.traffic_light_shape) {
        tin_light.final_matched_obj = scored_obj;
        max_score                   = obj_score;
        break;
      }
    }
  }

  TRAFFIC_LOG << fmt::format("The highest scoring traffic light ID is: {}, with a score of: {:.2f}, flash:{}, color : {}, status:{}",
                             tin_light.final_matched_obj.id, max_score, tin_light.final_matched_obj.attributes.traffic_light_flashing,
                             tin_light.final_matched_obj.attributes.traffic_light_color, tin_light.final_matched_obj.track_status);
  return true;
}

void TrafficLightMapping::LightColorToTin(TinLight &tin_light) {
  const auto &obj = tin_light.final_matched_obj;
  if (obj.attributes.traffic_light_flashing) {
    if (obj.attributes.traffic_light_color == TrafficLightColorType::TLC_RED) {
      tin_light.color = TrafficLightColorType::TLC_RED_FLASHING;
      TRAFFIC_LOG << fmt::format("TIN Traffic light is RED and FLASHING. Set tin_light.color to TLC_RED_FLASHING.");
    } else if (obj.attributes.traffic_light_color == TrafficLightColorType::TLC_YELLOW) {
      tin_light.color = TrafficLightColorType::TLC_YELLOW_FLASHING;
      TRAFFIC_LOG << fmt::format("TIN Traffic light is YELLOW and FLASHING. Set tin_light.color to TLC_YELLOW_FLASHING.");
    } else if (obj.attributes.traffic_light_color == TrafficLightColorType::TLC_GREEN) {
      tin_light.color = TrafficLightColorType::TLC_GREEN_FLASHING;
      TRAFFIC_LOG << fmt::format("TIN Traffic light is GREEN and FLASHING. Set tin_light.color to TLC_GREEN_FLASHING.");
    }
    return;
  }
  if (obj.track_status == UPDATED) {
    tin_light.color = obj.attributes.traffic_light_color;
    TRAFFIC_LOG << fmt::format("TIN Traffic light is UPDATED. Set tin_light.color to {}.", magic_enum::enum_name(tin_light.color));
  } else if (obj.attributes.is_occluded) {
    tin_light.color = cem::message::sensor::TLC_BLOCK_FAILED;
    TRAFFIC_LOG << fmt::format("TIN Traffic light is OCCLUDED! Set tin_light.color to TLC_BLOCK_FAILED.");
  } else {
    tin_light.color = obj.attributes.traffic_light_color;
    TRAFFIC_LOG << fmt::format("TIN Traffic light is COASTED! Set tin_light.color to {}.",
                               magic_enum::enum_name(obj.attributes.traffic_light_color));
  }
}

void TrafficLightMapping::ClearInvalidObj(TinLight &cur_tin_light) {
  for (auto it = cur_tin_light.match_light_records.begin(); it != cur_tin_light.match_light_records.end();) {
    int               obj_id        = it->first;
    std::vector<int> &light_records = it->second;
    if (light_records.size() >= 2 && light_records[light_records.size() - 1] == 4 && light_records[light_records.size() - 2] == 4) {
      // 分开判断是否存在于其他记录中
      bool exists_in_match_obj_records = cur_tin_light.match_obj_records.find(obj_id) != cur_tin_light.match_obj_records.end();
      bool exists_in_match_light_valid_records =
          cur_tin_light.match_light_valid_records.find(obj_id) != cur_tin_light.match_light_valid_records.end();
      bool exists_in_final_matched_scores = cur_tin_light.final_matched_scores.find(obj_id) != cur_tin_light.final_matched_scores.end();

      // 如果obj_id在相关记录中存在则进行删除
      if (exists_in_match_obj_records) {
        cur_tin_light.match_obj_records.erase(obj_id);
      }
      if (exists_in_match_light_valid_records) {
        cur_tin_light.match_light_valid_records.erase(obj_id);
      }
      if (exists_in_final_matched_scores) {
        cur_tin_light.final_matched_scores.erase(obj_id);
      }
      it = cur_tin_light.match_light_records.erase(it);
    } else {
      ++it;
    }
  }
}

TinLight &TrafficLightMapping::GetTinLight(TurnType direction) {
  switch (direction) {
    case TurnType::LEFT_TURN:
      return final_tin.left;
    case TurnType::U_TURN:
      return final_tin.u_turn;
    case TurnType::NO_TURN:
      return final_tin.straight;
    case TurnType::RIGHT_TURN:
      return final_tin.right;
    default:
      throw std::invalid_argument("Unknown TurnType");  // 处理未知方向
  }
}

void TrafficLightMapping::GetBestLight(const TinLight &tin_light, std::string &info_tmp) {
  TRAFFIC_LOG << fmt::format("Current Turn Type: {}", magic_enum::enum_name(tin_light.turn_type));
  TinLight &cur_tin_light  = GetTinLight(tin_light.turn_type);
  cur_tin_light.is_valid   = tin_light.is_valid;
  cur_tin_light.match_objs = tin_light.match_objs;
  cur_tin_light.color      = tin_light.color;

  if (cur_tin_light.match_objs.size() == 0) {
    TRAFFIC_LOG << "match_objs_size is 0";
  }
  for (const auto &obj : cur_tin_light.match_objs) {
    if (obj.track_status == UPDATED) {
      if (cur_tin_light.color == TrafficLightColorType::TLC_NONE_LIGHT) {
        cur_tin_light.addFrameData(obj.id, obj, 3);
      } else if (obj.attributes.traffic_light_shape == TrafficLightShapeType::TLS_OTHER_SHAPE) {
        cur_tin_light.addFrameData(obj.id, obj, 5);
      } else {
        if (obj.attributes.traffic_light_color == cur_tin_light.color &&
            obj.attributes.traffic_light_direction == TrafficLightDirectionType::TLD_BACK &&
            obj.attributes.traffic_light_shape != TrafficLightShapeType::TLS_OTHER_SHAPE &&
            obj.attributes.traffic_light_shape != TrafficLightShapeType::TLS_UNKNOWN) {
          cur_tin_light.addFrameData(obj.id, obj, 1);
        } else {
          cur_tin_light.addFrameData(obj.id, obj, 0);
        }
      }  
    } else {
      cur_tin_light.addFrameData(obj.id, obj, 2);
    }
  }

  for (const auto &pair : cur_tin_light.match_obj_records) {
    int   obj_id   = pair.first;
    auto  it       = std::find_if(cur_tin_light.match_objs.begin(), cur_tin_light.match_objs.end(),
                                  [obj_id](const TrfObjectInfo &obj) { return obj.id == obj_id; });
    auto &prev_obj = pair.second.back();
    if (it == cur_tin_light.match_objs.end()) {
      cur_tin_light.addFrameData(obj_id, prev_obj, 4);
    }
  }
  std::string binding_info = "[";
  FilterByChronology(cur_tin_light, binding_info);
  bool is_binding = BindingTrafficlightToTIN(cur_tin_light);
  // check match_light_records 最后两帧为4就销毁掉目标ID；
  ClearInvalidObj(cur_tin_light);
  if (is_binding) {
    LightColorToTin(cur_tin_light);
  } else {
    cur_tin_light.color = TrafficLightColorType::TLC_NONE_LIGHT;
  }

  info_tmp += fmt::format(" {}, {}, isbinding:{}, bind_objs:{}, bind_obj:{}; ", 
          magic_enum::enum_name(tin_light.turn_type), magic_enum::enum_name(cur_tin_light.color), is_binding, binding_info, cur_tin_light.final_matched_obj.id);
};

void TrafficLightMapping::GetFinalObj(const TinLightsObj &raw_tin_obj) {
  std::string info_tmp;
  info_tmp = " tin_out:[";
  GetBestLight(raw_tin_obj.u_turn, info_tmp);
  GetBestLight(raw_tin_obj.left, info_tmp);
  GetBestLight(raw_tin_obj.straight, info_tmp);
  GetBestLight(raw_tin_obj.right, info_tmp);

  info_tmp += "] ";
  TRAFFIC_LOG << info_tmp;
  info_traffic_lights_ += info_tmp;
}

void TrafficLightMapping::FindFinalTINResult(E2EResultRawPtr tin_e2e_result, PercepTrfInfoPtr lights_info_this_junction,
                                             bool is_virtual_stopline) {
  if (!tin_e2e_result || !lights_info_this_junction) {
    TRAFFIC_LOG << fmt::format("tin_e2e:{}  perception_ptr:{}", fmt::ptr(tin_e2e_result.get()), fmt::ptr(lights_info_this_junction.get()));
    final_tin.left.color     = TrafficLightColorType::TLC_NONE_LIGHT;
    final_tin.u_turn.color   = TrafficLightColorType::TLC_NONE_LIGHT;
    final_tin.straight.color = TrafficLightColorType::TLC_NONE_LIGHT;
    final_tin.right.color    = TrafficLightColorType::TLC_NONE_LIGHT;
    return;
  }

  auto SetLight = [&](const google::protobuf::RepeatedPtrField<::byd::msg::perception::DirectionLightState> &lights_status,
                      TurnType turn_type, const std::vector<TrafficLightShapeType> &light_shape, TinLight &traffic_ligh) {
    auto shapes_tmp = light_shape;
    if (turn_type != TurnType::RIGHT_TURN) {
      shapes_tmp.push_back(TrafficLightShapeType::TLS_CIRCULAR);
      shapes_tmp.push_back(TrafficLightShapeType::TLS_UNKNOWN);
      shapes_tmp.push_back(TrafficLightShapeType::TLS_OTHER_SHAPE);
    }

    for (const auto &status : lights_status) {
      if (status.has_direction() && ConvertTINTurnType(status.direction()) == turn_type) {
        traffic_ligh.color    = ColorConvert_TIN(status.trafficlight_info(), status.trafficlight_flashing());
        traffic_ligh.is_valid = true;
        GetMatchLights(lights_info_this_junction, shapes_tmp, traffic_ligh, is_virtual_stopline);
        std::vector<uint32_t> obj_id;
        for (const auto &obj : traffic_ligh.match_objs) {
          obj_id.push_back(obj.id);
        }
        TRAFFIC_LOG << fmt::format("tin_e2e_match_id turn_type:{}  ids:{} color:{}", magic_enum::enum_name(turn_type), obj_id,
                                   magic_enum::enum_name(traffic_ligh.color));
        return;
      };
    };
  };

  TinLightsObj raw_traffic_lights_tin;
  TRAFFIC_LOG << fmt::format("-------raw-------is_bev_stopline:{:d}  is_virtual_stopline:{:d}", stopline_ego_e2e_.has_value(),
                             is_virtual_stopline);
  SetLight(tin_e2e_result->e2e_light_states(), TurnType::U_TURN, uturn_shapes_, raw_traffic_lights_tin.u_turn);
  SetLight(tin_e2e_result->e2e_light_states(), TurnType::LEFT_TURN, left_shapes_, raw_traffic_lights_tin.left);
  SetLight(tin_e2e_result->e2e_light_states(), TurnType::NO_TURN, straight_shapes_, raw_traffic_lights_tin.straight);
  SetLight(tin_e2e_result->e2e_light_states(), TurnType::RIGHT_TURN, right_shapes_, raw_traffic_lights_tin.right);

  GetFinalObj(raw_traffic_lights_tin);
  TRAFFIC_LOG << fmt::format("tin_final u_turn_____color:{}", magic_enum::enum_name(final_tin.u_turn.color));
  TRAFFIC_LOG << fmt::format("tin_final left_______color:{}", magic_enum::enum_name(final_tin.left.color));
  TRAFFIC_LOG << fmt::format("tin_final straight___color:{}", magic_enum::enum_name(final_tin.straight.color));
  TRAFFIC_LOG << fmt::format("tin_final right______color:{}", magic_enum::enum_name(final_tin.right.color));
}

void TrafficLightMapping::IsEgoDedicatedRight() {
  /*
  if (sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane()) {
    TRAFFIC_REC_LOG << fmt::format("ego_in_right_dedicated raw_traffic_right    {}", traffic_lights_.right);
    traffic_lights_.right.color                  = message::sensor::TLC_GREEN;
    traffic_lights_.right.traffic_light_flashing = false;
    traffic_lights_.right.is_valid               = true;
  }
  */
}

void TrafficLightMapping::BindingTrafficLightToBev(const BevMapInfoPtr &bev_map) const {
  /*
  if (bev_map == nullptr || (sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane())/) {
    return;
  }
  auto ConvertLightState = [](cem::message::sensor::TrafficLightColorType color_type, bool is_flash) -> BevTrafficLightState {
    switch (color_type) {
      case cem::message::sensor::TrafficLightColorType::TLC_GREEN: {
        return is_flash ? BevTrafficLightState::TL_COLOR_GREEN : BevTrafficLightState::TL_COLOR_GREEN_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_RED: {
        return is_flash ? BevTrafficLightState::TL_COLOR_RED : BevTrafficLightState::TL_COLOR_RED_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_YELLOW: {
        return is_flash ? BevTrafficLightState::TL_COLOR_YELLOW_FLASH : BevTrafficLightState::TL_COLOR_YELLOW;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_GREEN_FLASHING: {
        return BevTrafficLightState::TL_COLOR_GREEN_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_YELLOW_FLASHING: {
        return BevTrafficLightState::TL_COLOR_YELLOW_FLASH;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_BLURRING_MODE: {
        return BevTrafficLightState::TL_COLOR_UNKNOWN;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_BLOCK_FAILED: {
        return BevTrafficLightState::TL_COLOR_BLOCK_FAILED;
      }
      case cem::message::sensor::TrafficLightColorType::TLC_BLOCK_FAILED: {
        return BevTrafficLightState::TL_COLOR_BLOCK_FAILED;
      }
      default: {
        return BevTrafficLightState::TL_COLOR_UNKNOWN;
      }
    }
    return BevTrafficLightState::TL_COLOR_UNKNOWN;
  };
  auto SetLightLanes = [&](std::vector<BevLaneInfo> &lanes_info) {
    for (auto &bev_lane : lanes_info) {
      if (!bev_lane.is_virtual || bev_lane.navi_action == BevAction::UNKNOWN) {
        continue;
      }
      TrafficLight light_t{};
      switch (bev_lane.navi_action) {
        case BevAction::LEFT_TURN: {
          light_t = traffic_lights_.left;
          break;
        }
        case BevAction::RIGHT_TURN: {
          light_t = traffic_lights_.right;
          break;
        }
        case BevAction::STRAIGHT: {
          light_t = traffic_lights_.straight;
          break;
        }
        case BevAction::U_TURN: {
          light_t = traffic_lights_.u_turn;
          break;
        }
        default:
          break;
      }
      if (light_t.is_valid) {
        bev_lane.trafficlight_state    = ConvertLightState(light_t.color, light_t.traffic_light_flashing);
        bev_lane.traffic_light_num     = light_t.traffic_light_num;
        bev_lane.traffic_light_seq_num = light_t.perception_seq_num;
        bev_lane.stay_prev_counter     = light_t.stay_prev_counter;
        bev_lane.traffic_set_reason    = light_t.traffic_reason;
        if (light_t.traffic_obj_info) {
          bev_lane.traffic_light_obj_id = light_t.traffic_obj_info->id;
        }
      }
    }
  };
  SetLightLanes(bev_map->lane_infos);
  for (auto &sec : bev_map->route.sections) {
    SetLightLanes(sec.lane_infos);
  }
  */
}

void TrafficLightMapping::GetRoutingTrafficLight(const RoutingMapPtr &routing_map) {
  routing_traffic_light_maps_.clear();
  if (routing_map == nullptr) {
    FLOG_TLIGHT << "routing_traffic_light_maps_ is empty!!! ";
    return;
  }
  routing_traffic_light_maps_ = routing_map->traffic_lights;
}

void TrafficLightMapping::DealPerTrafficLight(const BevMapInfoPtr &bev_map, const RoutingMapPtr &routing_map) {
  perception_traffic_light_ids_.clear();
  TrafficLights traffic_lights;
  BevMapInfoPtr bev_map_tmp = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(bev_map_tmp);
  if (bev_map_tmp == nullptr || routing_map == nullptr) {
    return;
  }
  const auto &lane_info = bev_map_tmp->lane_infos;
  uint64_t    left_lane_id{0};
  uint64_t    right_lane_id{0};
  float       max_left_y{-10.0f};
  float       min_right_y{10.0f};
  bool        has_lane_info{true};
  for (const auto &lane : lane_info) {
    // FLOG_TLIGHT << " lane id: " << lane.id << " ,line_points: " << lane.line_points.size()
    //      << " ,start x: " << lane.line_points.begin()->x;
    if (lane.line_points.size() < 2 || lane.line_points.begin()->x > 0.0 || lane.line_points.back().x < 0.0) {
      continue;
    }
    auto it =
        std::find_if(lane.line_points.begin(), lane.line_points.end(), [](const cem::fusion::Point2DF &point) { return point.x >= 0.0; });
    if (it == lane.line_points.end()) {
      continue;
    }
    // FLOG_TLIGHT << " line_point_x: " << it->x << " ,y: " << it->y;
    if (it->y > max_left_y) {
      max_left_y   = it->y;
      left_lane_id = lane.id;
    }
    if (it->y < min_right_y) {
      min_right_y   = it->y;
      right_lane_id = lane.id;
    }
  }

  if (left_lane_id == 0 || right_lane_id == 0) {
    has_lane_info = false;
  }
  // FLOG_TLIGHT << " left_lane_id: " << left_lane_id << " ,max_left_y: " << max_left_y << " ,right_lane_id: " << right_lane_id
  //      << " ,min_right_y: " << min_right_y << " ,has_lane_info: " << has_lane_info;
  cem::fusion::Point2DF left_start_point;
  cem::fusion::Point2DF left_end_point;
  cem::fusion::Point2DF right_start_point;
  cem::fusion::Point2DF right_end_point;
  for (const auto &lane : lane_info) {
    if (lane.id == left_lane_id) {
      auto it_start =
          std::find_if(lane.line_points.begin(), lane.line_points.end(), [](const cem::fusion::Point2DF &point) { return point.x >= 0.0; });
      left_start_point = cem::fusion::Point2DF(it_start->x, it_start->y + 7.0);
      auto it_end      = std::find_if(lane.line_points.begin(), lane.line_points.end(),
                                      [](const cem::fusion::Point2DF &point) { return point.x >= 10.0; });
      left_end_point   = it_end == lane.line_points.end() ? lane.line_points.back() : *it_end;
      left_end_point.y += 7.0;
    } else if (lane.id == right_lane_id) {
      auto it_start =
          std::find_if(lane.line_points.begin(), lane.line_points.end(), [](const cem::fusion::Point2DF &point) { return point.x >= 0.0; });
      right_start_point = cem::fusion::Point2DF(it_start->x, it_start->y - 1.0);
      auto it_end       = std::find_if(lane.line_points.begin(), lane.line_points.end(),
                                       [](const cem::fusion::Point2DF &point) { return point.x >= 10.0; });
      right_end_point   = it_end == lane.line_points.end() ? lane.line_points.back() : *it_end;
      right_end_point.y -= 1.0;
    }
  }

  for (const auto &obj : deal_per_traffic_light_objects_) {
    const auto &traffic_light = obj.second;
    if (traffic_light.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK || traffic_light.position.x > 250.0) {
      FLOG_TLIGHT << fmt::format("invalid_need_remove_light  direction_reason id:{}  direction:{} pos_x:{:.2f}", traffic_light.id,
                                 traffic_light.attributes.traffic_light_direction, traffic_light.position.x);
      continue;
    }

    if (FilterTrafficLightShape(traffic_light.attributes.traffic_light_shape)) {
      FLOG_TLIGHT << fmt::format("invalid_need_remove_light  shape_reason id:{}  shape:{}", traffic_light.id,
                                 traffic_light.attributes.traffic_light_shape);
      continue;
    }
    perception_traffic_light_ids_.emplace_back(obj.second.id);
  }

  for (auto id : perception_traffic_light_ids_) {
    FLOG_TLIGHT << " fist filter traffic_light_ids: " << id;
  }
  // 匹配routing_map中红绿灯位置处对应的BEV红绿灯
  std::unordered_set<uint32_t> match_traffic_light_ids;
  if (!routing_traffic_light_maps_.empty()) {
    for (const auto &map_light : routing_traffic_light_maps_) {
      const auto point = map_light.center_position;
      FLOG_TLIGHT << " routing_traffic_light_maps_point_id: " << map_light.id << " ,x: " << point.x << " ,y: " << point.y;
      for (auto id : perception_traffic_light_ids_) {
        const auto &position = deal_per_traffic_light_objects_[id].position;
        if (std::abs(position.x - point.x) < 100.0 && std::abs(position.y - point.y) < 30.0) {
          match_traffic_light_ids.insert(id);
        }
      }
    }
    // 再次遍历，将不匹配但是和感知红绿灯挨着的进行召回
    std::unordered_set<uint32_t> racall_traffic_light_ids;
    for (auto id : match_traffic_light_ids) {
      const auto &map_light_pos = deal_per_traffic_light_objects_[id].position;
      for (auto id : perception_traffic_light_ids_) {
        if (match_traffic_light_ids.count(id) == 1) {
          continue;
        }
        const auto &position = deal_per_traffic_light_objects_[id].position;
        if (std::abs(position.x - map_light_pos.x) < 5.0 && std::abs(position.y - map_light_pos.y) < 15.0 &&
            std::abs(position.z - map_light_pos.z) < 2.0) {
          racall_traffic_light_ids.insert(id);
        }
      }
    }
    if (!racall_traffic_light_ids.empty()) {
      match_traffic_light_ids.insert(racall_traffic_light_ids.begin(), racall_traffic_light_ids.end());
    }
  }

  for (auto id : match_traffic_light_ids) {
    FLOG_TLIGHT << " match_traffic_light_ids: " << id;
  }

  // 提取匹配routing_map的红绿灯
  if (!match_traffic_light_ids.empty()) {
    for (auto id : match_traffic_light_ids) {
      ExtractTrafficLight(deal_per_traffic_light_objects_[id], traffic_lights);
    }
    traffic_lights_                          = traffic_lights;
    traffic_lights_.header                   = routing_map->header;
    traffic_lights_.ld_perception_is_matched = true;

    // 设置红绿灯位置：
    auto &position             = deal_per_traffic_light_objects_[*match_traffic_light_ids.begin()].position;
    traffic_light_e2e_->pose.x = position.x;
    traffic_light_e2e_->pose.y = position.y;
    traffic_light_e2e_->pose.z = position.z;
    FLOG_TLIGHT << " u_turn_color: " << traffic_lights_.u_turn.color << " ,valid: " << traffic_lights_.u_turn.is_valid
                << ",flash:" << traffic_lights_.u_turn.traffic_light_flashing << " , left_color: " << traffic_lights_.left.color
                << " ,valid: " << traffic_lights_.left.is_valid << ",flash:" << traffic_lights_.left.traffic_light_flashing
                << " , straight_color: " << traffic_lights_.straight.color << " ,valid: " << traffic_lights_.straight.is_valid
                << ",flash:" << traffic_lights_.straight.traffic_light_flashing << " ,right_color: " << traffic_lights_.right.color
                << " ,valid: " << traffic_lights_.right.is_valid << ", flash:" << traffic_lights_.right.traffic_light_flashing;
    return;
  }

  // 没有红绿灯或者没有lane直接返回
  if (perception_traffic_light_ids_.empty()) {
    FLOG_TLIGHT << "traffic_light_ids.empty!! ";
    return;
  }

  // 没有lane的时候默认过滤前方左侧10m，右侧5m内的红绿灯
  if (!has_lane_info) {
    left_start_point  = {0, 10.0};
    left_end_point    = {10, 10.0};
    right_start_point = {0, -5.0};
    right_end_point   = {10, -5.0};
  }

  // 根据横向位置过滤红绿灯
  std::unordered_set<uint32_t> bev_traffic_light_ids;
  for (auto id : perception_traffic_light_ids_) {
    const auto &pos = deal_per_traffic_light_objects_[id].position;
    if (PointInVectorSide(left_start_point, left_end_point, pos) > 0 && PointInVectorSide(right_start_point, right_end_point, pos) < 0) {
      bev_traffic_light_ids.insert(id);
    }
  }

  if (bev_traffic_light_ids.empty()) {
    FLOG_TLIGHT << " bev_traffic_light_ids.empty! ";
    return;
  }

  // 再次遍历，进行召回
  std::unordered_set<uint32_t> bev_racall_traffic_light_ids;
  for (auto id : bev_traffic_light_ids) {
    const auto &map_light_pos = deal_per_traffic_light_objects_[id].position;
    for (auto id : perception_traffic_light_ids_) {
      if (bev_traffic_light_ids.count(id) == 1) {
        continue;
      }
      const auto &position = deal_per_traffic_light_objects_[id].position;
      if (std::abs(position.x - map_light_pos.x) < 5.0 && std::abs(position.y - map_light_pos.y) < 15.0 &&
          std::abs(position.z - map_light_pos.z) < 2.0) {
        bev_racall_traffic_light_ids.insert(id);
      }
    }
  }
  if (!bev_racall_traffic_light_ids.empty()) {
    bev_traffic_light_ids.insert(bev_racall_traffic_light_ids.begin(), bev_racall_traffic_light_ids.end());
  }
  for (auto id : bev_traffic_light_ids) {
    FLOG_TLIGHT << " bev_traffic_light_ids: " << id;
  }
  for (auto id : bev_traffic_light_ids) {
    ExtractTrafficLight(deal_per_traffic_light_objects_[id], traffic_lights);
  }
  traffic_lights_        = traffic_lights;
  traffic_lights_.header = routing_map->header;

  // 设置红绿灯位置：
  auto &position             = deal_per_traffic_light_objects_[*bev_traffic_light_ids.begin()].position;
  traffic_light_e2e_->pose.x = position.x;
  traffic_light_e2e_->pose.y = position.y;
  traffic_light_e2e_->pose.z = position.z;
  FLOG_TLIGHT << " turn_color: " << traffic_lights_.u_turn.color << " ,valid: " << traffic_lights_.u_turn.is_valid
              << " , left_color: " << traffic_lights_.left.color << " ,valid: " << traffic_lights_.left.is_valid
              << " , straight_color: " << traffic_lights_.straight.color << " ,valid: " << traffic_lights_.straight.is_valid
              << " ,right_color: " << traffic_lights_.right.color << " ,valid: " << traffic_lights_.right.is_valid;
}

// enum TrafficLightColorType { //红绿灯颜色
//   TLC_UNKNOWN = 0,
//   TLC_RED = 1,
//   TLC_GREEN = 2,
//   TLC_YELLOW = 3,
//   TLC_BLACK = 4,
//   TLC_OTHER = 5
// };
// enum TrafficLightShapeType { //红绿灯形状
//   TLS_UNKNOWN = 0, // 未知
//   TLS_CIRCULAR = 1, //圆形
//   TLS_LEFT_ARROW = 2,//向左箭头
//   TLS_RIGHT_ARROW = 3,//向右箭头
//   TLS_UP_ARROW = 4,//向上箭头
//   TLS_DOWN_ARROW = 5,//向下箭头
//   TLS_BICYCLE = 6, //自行车
//   TLS_PEDESTRIAN = 7, //行人
//   TLS_CLOSE_TO_TRAFFIC = 8, //禁止通行
//   TLS_UP_LEFT_ARROW = 9, //上与左箭头
//   TLS_TURN_ARROUND_ARROW = 10, //掉头箭头
//   TLS_UP_RIGHT_ARROW = 11, //上与右箭头
//   TLS_LEFT_TURN_ARROUND_ARROW = 12, //左与转弯箭头
//   TLS_SLOW_DOWN = 13, //慢
//   TLS_HEART_SHAPE = 14, // 心型图案
//   TLS_NUMBER = 15, //倒计时数字
//   TLS_PROCESS_BAR = 16, //进度条红绿灯
//   TLS_BUS_ONLY_SHAPE = 17, //公交专用
//   TLS_RECTANGLE = 18, //方块形
//   TLS_OTHER_SHAPE = 19 //其它形状
// };

// 0,1,18,19是左转和直行
// 10，12 掉头
// 2，9，12 左转
// 4，9，11 直行
// 3，11 右转
// 过滤 5，6，7，8, 13，14，15, 16, 17

bool TrafficLightMapping::FilterTrafficLightShape(const cem::message::sensor::TrafficLightShapeType &shape) {
  return shape == TrafficLightShapeType::TLS_DOWN_ARROW || shape == TrafficLightShapeType::TLS_BICYCLE ||
         shape == TrafficLightShapeType::TLS_PEDESTRIAN || shape == TrafficLightShapeType::TLS_CLOSE_TO_TRAFFIC ||
         shape == TrafficLightShapeType::TLS_SLOW_DOWN || shape == TrafficLightShapeType::TLS_HEART_SHAPE ||
         shape == TrafficLightShapeType::TLS_NUMBER || shape == TrafficLightShapeType::TLS_PROCESS_BAR ||
         shape == TrafficLightShapeType::TLS_BUS_ONLY_SHAPE || shape == TrafficLightShapeType::TLS_RECTANGLE ||
         shape == TrafficLightShapeType::TLS_DOWN_ARROW;
}

// 根据红绿灯的不同形状提取到不同方向红绿灯
void TrafficLightMapping::ExtractTrafficLight(const cem::message::sensor::TrfObjectInfo &trf_obj, TrafficLights &traffic_lights) {
  FLOG_TLIGHT << "ExtractTrafficLight_id: " << trf_obj.id << ", color: " << trf_obj.attributes.traffic_light_color
              << " , shape: " << trf_obj.attributes.traffic_light_shape << " ,direction: " << trf_obj.attributes.traffic_light_direction
              << " ,flash:" << trf_obj.attributes.traffic_light_flashing;
  if (trf_obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK) {
    return;
  }
  switch (trf_obj.attributes.traffic_light_shape) {
    case TrafficLightShapeType::TLS_UNKNOWN:
    case TrafficLightShapeType::TLS_CIRCULAR:
      TransTrafficLight(trf_obj, traffic_lights.left, false);
      TransTrafficLight(trf_obj, traffic_lights.straight, false);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, false);
      break;
    case TrafficLightShapeType::TLS_LEFT_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.left, true);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, true);
      break;
    case TrafficLightShapeType::TLS_RIGHT_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.right, true);
      break;
    case TrafficLightShapeType::TLS_UP_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.straight, true);
      break;
    case TrafficLightShapeType::TLS_UP_LEFT_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.left, true);
      TransTrafficLight(trf_obj, traffic_lights.straight, true);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, true);
      break;
    case TrafficLightShapeType::TLS_TURN_ARROUND_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.u_turn, true);
      break;
    case TrafficLightShapeType::TLS_UP_RIGHT_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.straight, true);
      TransTrafficLight(trf_obj, traffic_lights.right, true);
      break;
    case TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW:
      TransTrafficLight(trf_obj, traffic_lights.left, true);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, true);
      break;
    case TrafficLightShapeType::TLS_RECTANGLE:
      TransTrafficLight(trf_obj, traffic_lights.left, false);
      TransTrafficLight(trf_obj, traffic_lights.straight, false);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, false);
      break;
    case TrafficLightShapeType::TLS_OTHER_SHAPE:
      TransTrafficLight(trf_obj, traffic_lights.left, false);
      TransTrafficLight(trf_obj, traffic_lights.straight, false);
      TransTrafficLight(trf_obj, traffic_lights.u_turn, false);
      break;
    default:
      AINFO << "error ";
      break;
  }
  if (!traffic_lights.right.is_valid) {
    traffic_lights.right.is_valid = true;
    traffic_lights.right.color    = cem::message::sensor::TrafficLightColorType::TLC_GREEN;
  }
  return;
}

void TrafficLightMapping::TransTrafficLight(const cem::message::sensor::TrfObjectInfo &trf_obj, TrafficLight &traffic_light,
                                            bool is_arrow) {
  if (traffic_light.publish_time - trf_obj.publish_time > 0.005) {
    return;
  }
  if (std::fabs(traffic_light.publish_time - trf_obj.publish_time) <= 0.005 && !is_arrow && traffic_light.is_confirmed) {
    // 非箭头的红绿灯信号不能覆盖箭头红绿灯信号
    return;
  }
  traffic_light.color                  = trf_obj.attributes.traffic_light_color;
  traffic_light.traffic_light_num      = trf_obj.attributes.traffic_light_num;
  traffic_light.is_confirmed           = is_arrow;
  traffic_light.traffic_light_flashing = trf_obj.attributes.traffic_light_flashing;
  traffic_light.is_valid               = true;
  traffic_light.publish_time           = trf_obj.publish_time;
}

double TrafficLightMapping::LaneNearDisToJunction(const LaneInfo &ld_lane) {
  if (ld_lane.points.size() < 2 || deal_per_traffic_light_objects_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const auto     &traffic_light = deal_per_traffic_light_objects_.begin()->second;
  Eigen::Vector2d traffic_pos{traffic_light.position.x, traffic_light.position.y};
  Eigen::Vector2d point_start{ld_lane.points.front().x, ld_lane.points.front().y};
  Eigen::Vector2d point_end{ld_lane.points.back().x, ld_lane.points.back().y};
  double          dis_start = (traffic_pos - point_start).norm();
  double          dis_end   = (traffic_pos - point_end).norm();
  return std::min(dis_start, dis_end);
}

bool HeadValidCheck(const cem::message::common::Header &lhs, const cem::message::common::Header &rhs) {
  bool                     is_traffic_light_valid = true;
  static constexpr double  time_invalid_max       = 1;
  static constexpr uint8_t counter_invalid_max    = 10;
  if ((lhs.timestamp - rhs.timestamp) >= time_invalid_max) {
    TRAFFIC_LOG << fmt::format(
        "time exceed the threshold. current_time:{:.2f}  "
        "traffic_light_time:{:.2f}",
        lhs.timestamp, rhs.timestamp);
    is_traffic_light_valid = false;
  }
  // counter check
  if ((lhs.cycle_counter - rhs.cycle_counter) >= counter_invalid_max) {
    TRAFFIC_LOG << fmt::format(
        "counter exceed the threshold. current_counter:{}  "
        "traffic_light_counter:{}",
        lhs.cycle_counter, rhs.cycle_counter);
    is_traffic_light_valid = false;
  }
  return is_traffic_light_valid;
}

bool TrafficLightMapping::IsPrevLaneHasStopLine(const RoutingMapPtr &routing_map_output, TrafficLightStatus &traffic_light_status) {
  const auto &lanes = routing_map_output->lanes;
  for (const auto &lane_id : traffic_light_status.lane_ids) {
    const auto &it_lane = FindTheElement(lanes, lane_id);
    if (!it_lane.has_value()) {
      continue;
    }
    const auto &lane = **it_lane;
    for (const auto &prev_lane_id : lane.previous_lane_ids) {
      const auto &prev_lane = FindTheElement(lanes, prev_lane_id);
      if (!prev_lane.has_value() || (**prev_lane).traffic_stop_lines.empty()) {
        continue;
      }
      for (const auto &prev_stop_id : (**prev_lane).traffic_stop_lines) {
        const auto &stop_line = FindTheElement(routing_map_output->stop_lines, prev_stop_id);
        if (stop_line.has_value() && !(**stop_line).points.empty()) {
          traffic_light_status.stop_line_pts = (**stop_line).points;
          TRAFFIC_LOG << fmt::format("fint stop line:{1} of the previous lane:{0}", (*prev_lane)->id, traffic_light_status.stop_line_pts);
          return true;
        }
      }
    }
  }
  return false;
}

void TrafficLightMapping::GenerateStopLinePoints(const RoutingMapPtr &routing_map_output) {
  std::vector<TurnType> turn_all{TurnType::NO_TURN, TurnType::LEFT_TURN, TurnType::RIGHT_TURN, TurnType::U_TURN};
  for (auto &traffic_light_status : traffic_light_e2e_->traffic_status) {
    TRAFFIC_LOG << fmt::format("{}", traffic_light_status);
    traffic_light_status.is_navi_light = true;
    turn_all.erase(std::remove_if(turn_all.begin(), turn_all.end(),
                                  [&traffic_light_status](const TurnType &turn) { return traffic_light_status.turn_type == turn; }),
                   turn_all.end());
    if (traffic_light_status.lane_ids.empty()) {
      return;
    }
    if (IsPrevLaneHasStopLine(routing_map_output, traffic_light_status)) {
      continue;
    }
    const auto &boundaries = routing_map_output->lane_boundaries;
    const auto &lanes      = routing_map_output->lanes;

    std::optional<LineSegment2d> segment_t;
    double                       left_dis{std::numeric_limits<double>::infinity()};
    double                       right_dis{std::numeric_limits<double>::lowest()};
    Vec2d                        left_point;
    Vec2d                        right_point;

    for (const auto &lane_id : traffic_light_status.lane_ids) {
      const auto &it_lane = FindTheElement(lanes, lane_id);
      if (!it_lane.has_value() || (**it_lane).points.empty()) {
        continue;
      }
      const auto &lane = **it_lane;

      fmt::memory_buffer point_buf;
      fmt::format_to(point_buf, "lane_id:{}  lane_point:{},{}", lane.id, lane.points.front(), lane.points.back());
      if (!segment_t.has_value()) {
        segment_t = LineSegment2d({Vec2d(lane.points.front().x, lane.points.front().y), Vec2d(lane.points.back().x, lane.points.back().y)});
      }
      auto it_left_bound = FindTheElement(boundaries, lane.left_lane_boundary_ids.front());
      if (it_left_bound.has_value() && (*it_left_bound)->points.size() > 2) {
        fmt::format_to(point_buf, "  id:{} left_bound_point:{}", lane.left_lane_boundary_ids.front(),
                       it_left_bound.value()->points.front());
        Vec2d left_p_t = Vec2d((*it_left_bound)->points.front().x, (*it_left_bound)->points.front().y);
        if (auto dis = segment_t->ProductOntoUnit(left_p_t); dis < left_dis) {
          left_dis   = dis;
          left_point = left_p_t;
        }
      }
      auto it_right_bound = FindTheElement(boundaries, lane.right_lane_boundary_ids.front());
      if (it_right_bound.has_value() && (*it_right_bound)->points.size() > 2) {
        fmt::format_to(point_buf, "  id:{} right_bound_point:{}", lane.right_lane_boundary_ids.front(),
                       it_right_bound.value()->points.front());
        Vec2d right_p_t = Vec2d((*it_right_bound)->points.front().x, (*it_right_bound)->points.front().y);
        if (auto dis = segment_t->ProductOntoUnit(right_p_t); dis > right_dis) {
          right_dis   = dis;
          right_point = right_p_t;
        }
      }
      // TRAFFIC_LOG << std::string_view(point_buf.data(), point_buf.size());
    }
    if (segment_t->length() > 1.0 && left_point.DistanceTo(right_point) > 1.0) {
      static constexpr double stop_line_width = 0.2;
      static constexpr double extend_len      = 6.0;
      LineSegment2d           seg1{left_point, right_point};
      left_point -= seg1.unit_direction() * extend_len;
      right_point += seg1.unit_direction() * extend_len;
      LineSegment2d seg2     = TranslateSegmentAlongAxisStart(*segment_t, {left_point, right_point}, stop_line_width);
      auto         &line_pts = traffic_light_status.stop_line_pts;
      line_pts.clear();
      line_pts.emplace_back<Point2D>({left_point.x(), left_point.y()});
      line_pts.emplace_back<Point2D>({right_point.x(), right_point.y()});
      line_pts.emplace_back<Point2D>({seg2.end().x(), seg2.end().y()});
      line_pts.emplace_back<Point2D>({seg2.start().x(), seg2.start().y()});
      TRAFFIC_LOG << fmt::format("stop_line_points based on bounds:{}", line_pts);
    }
  }
  std::vector<Point2D> points;
  if (!traffic_light_e2e_->traffic_status.empty()) {
    points = traffic_light_e2e_->traffic_status.front().stop_line_pts;
  }
  for (TurnType turn_type : turn_all) {
    auto       &traffic_status       = traffic_light_e2e_->traffic_status.emplace_back();
    const auto &light_status         = traffic_lights_.GetLightState(turn_type);
    traffic_status.turn_type         = turn_type;
    traffic_status.stop_line_pts     = points;
    traffic_status.is_navi_light     = false;
    traffic_status.light_status      = LightColorConvert(light_status.color, light_status.traffic_light_flashing);
    traffic_status.traffic_light_num = light_status.traffic_light_num;
  }
  TRAFFIC_LOG << fmt::format("traffic_e2e_info:{}", traffic_light_e2e_->traffic_status);
}

void TrafficLightMapping::SetE2EInfo(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &lanes_traffic_all) {
  if (lanes_traffic_all.empty()) {
    TRAFFIC_LOG << "lanes_traffic is empty.";
    return;
  }
  std::unordered_set<uint64_t> present_ids;
  for (const auto &lane : lanes_traffic_all) {
    present_ids.insert(lane.id);
  }
  TRAFFIC_LOG << fmt::format("raw_lanes. size:{}", lanes_traffic_all.size());
  for (const LaneInfo &lane : lanes_traffic_all) {
    // TRAFFIC_LOG << fmt::format(
    //     "lane_id:{} prev_lanes:{} left_lanes:{}  right_lanes:{} turn:{} "
    //     "light_status:{}",
    //     lane.id, lane.previous_lane_ids, lane.left_lane_id, lane.right_lane_id, lane.turn_type, lane.light_status);
  }

  // remove the lanes in lanes_traffic_all whose prev-lane has been in lanes_traffic_all.
  // lanes_traffic_all.erase(
  //     std::remove_if(lanes_traffic_all.begin(), lanes_traffic_all.end(),
  //                    [&](const LaneInfo& lane_t) {
  //                      return std::any_of(lane_t.previous_lane_ids.begin(),
  //                                         lane_t.previous_lane_ids.end(),
  //                                         [&](uint64_t prev_id) {
  //                                           return present_ids.count(prev_id) >
  //                                                  0;
  //                                         });
  //                    }),
  //     lanes_traffic_all.end());
  std::sort(lanes_traffic_all.begin(), lanes_traffic_all.end(), [](const LaneInfo &lhs, const LaneInfo &rhs) {
    if (lhs.turn_type != rhs.turn_type) {
      return lhs.turn_type < rhs.turn_type;
    }
    return lhs.light_status < rhs.light_status;
  });
  TRAFFIC_LOG << "filtered_lanes. size:" << lanes_traffic_all.size();
  if (lanes_traffic_all.empty()) {
    return;
  }
  for (const auto &lane : lanes_traffic_all) {
    // TRAFFIC_LOG << fmt::format("lane_id:{} turn:{} light_status:{}", lane.id, magic_enum::enum_name(lane.turn_type),
    //  magic_enum::enum_name(lane.light_status));
  }
  TrafficLightStatus traffic_light_status_t;
  size_t             idx                   = 0;
  traffic_light_status_t.light_status      = lanes_traffic_all[idx].light_status;
  traffic_light_status_t.turn_type         = lanes_traffic_all[idx].turn_type;
  traffic_light_status_t.traffic_light_num = lanes_traffic_all[idx].light_countdown;
  traffic_light_status_t.lane_ids.emplace_back(lanes_traffic_all[idx].id);
  traffic_light_e2e_->traffic_status.push_back(traffic_light_status_t);
  for (idx++; idx < lanes_traffic_all.size(); idx++) {
    if (lanes_traffic_all[idx].light_status == traffic_light_status_t.light_status &&
        lanes_traffic_all[idx].turn_type == traffic_light_status_t.turn_type) {
      traffic_light_e2e_->traffic_status.back().lane_ids.emplace_back(lanes_traffic_all[idx].id);
      continue;
    }
    traffic_light_status_t.light_status      = lanes_traffic_all[idx].light_status;
    traffic_light_status_t.turn_type         = lanes_traffic_all[idx].turn_type;
    traffic_light_status_t.traffic_light_num = lanes_traffic_all[idx].light_countdown;
    traffic_light_status_t.lane_ids.emplace_back(lanes_traffic_all[idx].id);
    traffic_light_e2e_->traffic_status.push_back(traffic_light_status_t);
  }
  GenerateStopLinePoints(routing_map_output);
}

void TrafficLightMapping::ProcessNotRoutingLanes(const RoutingMapPtr &routing_map_output, std::vector<LaneInfo> &traffic_light_lanes) {
  if (traffic_light_lanes.empty()) {
    return;
  }
  auto                        &lanes      = routing_map_output->lanes;
  const auto                  &stop_lines = routing_map_output->stop_lines;
  std::unordered_set<uint64_t> junction_id_set;
  for (const auto &lane_info : traffic_light_lanes) {
    if (lane_info.junction_id != 0) {
      junction_id_set.insert(lane_info.junction_id);
    }
  }
  const auto              &routing_sections       = routing_map_output->route.sections;
  uint64_t                 ego_section_id         = routing_map_output->route.navi_start.section_id;
  static constexpr int64_t max_extend_section_num = 3;
  auto                     ego_section_it         = FindTheElement(routing_sections, ego_section_id);
  if (!ego_section_it.has_value()) {
    return;
  }
  int64_t                      farthest_section_idx = 0;
  std::unordered_set<uint64_t> lanes_light_set;
  for (const auto &lane_info_t : traffic_light_lanes) {
    lanes_light_set.insert(lane_info_t.id);
    auto section_idx = FindTheElement(routing_sections, lane_info_t.section_id);
    if (section_idx.has_value()) {
      if (auto dis = std::distance(routing_sections.begin(), *section_idx); dis > farthest_section_idx) {
        farthest_section_idx = dis;
      }
    }
  }
  int64_t farthest_succeed = farthest_section_idx - std::distance(routing_sections.begin(), *ego_section_it) + max_extend_section_num;
  static constexpr int64_t max_recursive_depth = 40;
  TRAFFIC_LOG << "farthest_succeed:" << farthest_succeed;

  if (farthest_succeed <= 0) {
    return;
  }

  std::function<void(const LaneInfo &, int)> SetNextLaneTrafficInfo;
  SetNextLaneTrafficInfo = [&](const LaneInfo &lane, int recursive_num) -> void {
    if (lanes_light_set.count(lane.id) == 0 &&
        ((lane.junction_id != 0 && junction_id_set.count(lane.junction_id) > 0) || lane.turn_type == TurnType::LEFT_TURN ||
         lane.turn_type == TurnType::RIGHT_TURN || lane.turn_type == TurnType::U_TURN)) {
      // traffic_light_lanes.emplace_back(lane);
      TRAFFIC_LOG << fmt::format(
          "SetNextLaneTrafficInfo lane_id:{}  section_id:{}  junction_id:{}  "
          "turn_type:{} recursive_num:{}",
          lane.id, lane.section_id, lane.junction_id, lane.turn_type, recursive_num);
      SetLaneTrafficInfo(lane, lanes, stop_lines);
    }
    if (recursive_num >= farthest_succeed || recursive_num > max_recursive_depth) {
      return;
    }
    if (lane.next_lane_ids.empty()) {
      return;
    }
    for (auto lane_id : lane.next_lane_ids) {
      auto lane_it = FindTheElement(lanes, lane_id);
      if (!lane_it.has_value()) {
        continue;
      }
      SetNextLaneTrafficInfo(**lane_it, recursive_num + 1);
    }
  };

  for (const auto &lane_id : (*ego_section_it)->lane_ids) {
    auto lane = FindTheElement(lanes, lane_id);
    if (!lane.has_value()) {
      continue;
    }
    SetNextLaneTrafficInfo(**lane, 0);
  }
}

std::tuple<uint64_t, bool, bool> TrafficLightMapping::LaneStopLineInfo(const std::vector<LaneInfo>     &lanes_output,
                                                                       const std::vector<StopLineInfo> &stop_lines, uint64_t id_temp) {
  auto it_lane_temp =
      std::find_if(lanes_output.begin(), lanes_output.end(), [&id_temp](const auto &lane_t) { return id_temp == lane_t.id; });
  if (it_lane_temp == lanes_output.end()) {
    return std::make_tuple(0, false, false);
  }
  bool has_stop_line = !it_lane_temp->traffic_stop_lines.empty() && it_lane_temp->traffic_stop_lines.front() != 0;
  if (!has_stop_line) {
    return std::make_tuple(0, false, false);
  }
  bool stop_line_front{false};
  bool stop_line_back{false};
  if (it_lane_temp->points.size() >= 2) {
    if (auto stop_line = FindTheElement(stop_lines, it_lane_temp->traffic_stop_lines.front());
        stop_line.has_value() && !stop_line.value()->points.empty()) {
      Vec2d point_start(it_lane_temp->points.back().x, it_lane_temp->points.back().y);
      Vec2d point_end(it_lane_temp->points.front().x, it_lane_temp->points.front().y);
      Vec2d stop_point(stop_line.value()->points.front().x, stop_line.value()->points.front().y);
      if (point_start.DistanceTo(stop_point) > point_end.DistanceTo(stop_point)) {
        stop_line_front = true;
      } else {
        stop_line_back = true;
      }
    }
  }
  return std::make_tuple(it_lane_temp->traffic_stop_lines.front(), stop_line_front, stop_line_back);
};

std::optional<LaneInfo> TrafficLightMapping::SetLaneTrafficInfo(const LaneInfo &lane_traffic, std::vector<LaneInfo> &lanes_output,
                                                                const std::vector<StopLineInfo> &stop_lines) {
  cem::message::env_model::TurnType turn_temp{lane_traffic.turn_type};
  if (lane_traffic.type == cem::message::env_model::LaneType::LANE_LEFT_WAIT) {
    if (traffic_lights_.left.is_valid &&
        (traffic_lights_.left.color == message::sensor::TLC_GREEN || traffic_lights_.left.color == message::sensor::TLC_GREEN_FLASHING)) {
      turn_temp = TurnType::LEFT_TURN;
    } else {
      turn_temp = TurnType::NO_TURN;
    }
  }
  const auto &light_status = traffic_lights_.GetLightState(turn_temp);
  if (!light_status.is_valid) {
    TRAFFIC_LOG << "  lane_id:" << lane_traffic.id << "  turn_type:" << static_cast<int>(lane_traffic.turn_type);
    return nullopt;
  }
  auto it_lane_output =
      std::find_if(lanes_output.begin(), lanes_output.end(), [&lane_traffic](const auto &lane_t) { return lane_traffic.id == lane_t.id; });
  if (it_lane_output == lanes_output.end()) {
    TRAFFIC_LOG << fmt::format("not find the lane:{}", lane_traffic.id);
    return nullopt;
  }
  it_lane_output->light_status    = LightColorConvert(light_status.color, light_status.traffic_light_flashing);
  it_lane_output->light_countdown = light_status.traffic_light_num;
  // it_lane_output->traffic_light_obj_id  = light_status.traffic_obj_info ? light_status.traffic_obj_info->id : 0;
  // it_lane_output->traffic_light_seq_num = light_status.perception_seq_num;
  // it_lane_output->traffic_set_reason    = light_status.traffic_reason;
  // it_lane_output->stay_prev_counter     = light_status.stay_prev_counter;

  if (it_lane_output->junction_id == 0) {
    bool need_set_junction_id = true;
    if (!it_lane_output->cross_walks.empty() && it_lane_output->cross_walks.front() != 0) {
      for (uint64_t id_prev : it_lane_output->previous_lane_ids) {
        auto lane_prev_t = FindTheElement(lanes_output, id_prev);
        if (lane_prev_t.has_value() && !lane_prev_t.value()->cross_walks.empty() && lane_prev_t->base()->cross_walks.front() != 0) {
          need_set_junction_id = false;
        }
      }
      if (need_set_junction_id) {
        TRAFFIC_LOG << fmt::format("lane_id:{} has_cross id_be_set 1", it_lane_output->id);
        it_lane_output->junction_id = 1;
      }
    }

    auto [has_stop_line, stop_line_front, stop_line_back] = LaneStopLineInfo(lanes_output, stop_lines, it_lane_output->id);
    if (need_set_junction_id && it_lane_output->junction_id == 0 && has_stop_line != 0U && stop_line_front) {
      TRAFFIC_LOG << fmt::format("lane_id:{} has_stop_front id_be_set 1", it_lane_output->id);
      it_lane_output->junction_id = 1;
    }

    if (need_set_junction_id && it_lane_output->junction_id == 0) {
      for (uint64_t id_prev : it_lane_output->previous_lane_ids) {
        auto [has_stop_line, stop_line_front, stop_line_back] = LaneStopLineInfo(lanes_output, stop_lines, id_prev);
        if ((has_stop_line != 0U) && stop_line_back) {
          TRAFFIC_LOG << fmt::format("prev_lane_id:{} has_stop_back id_be_set 1", it_lane_output->id);
          it_lane_output->junction_id = 1;
          break;
        }
      }
    }

    if (it_lane_output->junction_id == 1) {
      it_lane_output->type       = message::env_model::LaneType::LANE_VIRTUAL_JUNCTION;
      it_lane_output->is_virtual = true;
    }
  }
  TRAFFIC_LOG << "lane_id:" << it_lane_output->id
              << "  is_left_wait:" << static_cast<bool>(lane_traffic.type == cem::message::env_model::LaneType::LANE_LEFT_WAIT)
              << "  light_status:" << magic_enum::enum_name(it_lane_output->light_status) << "  num:" << it_lane_output->light_countdown;
  info_traffic_lights_ += fmt::format("{},{},{:d};", it_lane_output->id, it_lane_output->light_status,
                                      light_status.traffic_light_num > 200 ? 200 : light_status.traffic_light_num);
  return *it_lane_output;
}

void TrafficLightMapping::SetJunctionPoints(const RoutingMapPtr &routing_map, const std::vector<LaneInfo> &lanes_traffic_all) {
  std::vector<Point2D> junction_points;
  std::vector<Point2D> cross_points;
  for (const auto &lane : lanes_traffic_all) {
    if (lane.junction_id <= 1) {
      continue;
    }
    auto it_junction = FindTheElement(routing_map->junctions, lane.junction_id);
    if (it_junction) {
      junction_points = (**it_junction).points;
      if (junction_points.size() >= 4) {
        TRAFFIC_LOG << fmt::format("current_junction_points junction_id:{}  point:{:.2f},{:.2f}", lane.junction_id,
                                   junction_points.front().x, junction_points.front().y);
        break;
      }
    }
    for (auto id_cross : lane.cross_walks) {
      auto it_cross = FindTheElement(routing_map->cross_walks, id_cross);
      if (it_cross) {
        cross_points = (**it_cross).points;
        if (cross_points.size() >= 4) {
          TRAFFIC_LOG << fmt::format("current_junction_points cross_id:{}  point:{:.2f},{:.2f}", id_cross, cross_points.front().x,
                                     cross_points.front().y);
          break;
        }
      }
    }
    if (cross_points.size() >= 4) {
      break;
    }
  }
  if (!junction_points.empty()) {
    previous_junction_points_ = junction_points;
  } else if (!cross_points.empty()) {
    previous_junction_points_ = cross_points;
  } else {
    previous_junction_points_.clear();
  }
}

double TrafficLightMapping::GetPreviousJunctionEgoPassed() {
  constexpr double kInvalidDistance = std::numeric_limits<double>::infinity();
  if (!routing_map_ptr_ || junction_ids_vec_.empty()) {
    return kInvalidDistance;
  }
  const auto &sections  = routing_map_ptr_->route.sections;
  const auto &route_ego = routing_map_ptr_->route.navi_start;
  TRAFFIC_LOG << fmt::format("ego_section_id:{}  s_offset:{:.2f}", route_ego.section_id, route_ego.s_offset);
  auto it_section =
      std::find_if(sections.rbegin(), sections.rend(), [&](const auto &section) { return route_ego.section_id == section.id; });
  if (it_section == sections.rend()) {
    return kInvalidDistance;
  }
  auto it_val = std::find(junction_ids_vec_.rbegin(), junction_ids_vec_.rend(), it_section->id);
  if (it_val != junction_ids_vec_.rend()) {
    info_traffic_lights_ += fmt::format("  ego_pass_junction:{}  dis:{:.2f}  ", *it_val, route_ego.s_offset);
    return route_ego.s_offset;
  }
  double dis = route_ego.s_offset;
  for (++it_section; it_section != sections.rend(); ++it_section) {
    dis += it_section->length;
    if (it_val = std::find(junction_ids_vec_.rbegin(), junction_ids_vec_.rend(), it_section->id); it_val != junction_ids_vec_.rend()) {
      info_traffic_lights_ += fmt::format("  ego_pass_junction:{}  dis:{:.2f}  ", *it_val, dis);
      return dis;
    }
  }
  return kInvalidDistance;
}

void TrafficLightMapping::GetJunctionIdsPool(const RoutingMapPtr &routing_map) {
  const auto &sections           = routing_map->route.sections;
  const auto &lanes              = routing_map->lanes;
  auto        GetJunctionPolygon = [&](uint64_t junction_id) {
    auto it_junction = FindTheElement(routing_map->junctions, junction_id);
    if (it_junction && it_junction.value()->points.size() >= 4) {
      std::vector<Vec2d> vec_points;
      for (const auto &p : it_junction.value()->points) {
        vec_points.emplace_back(p.x, p.y);
      }
      return byd::common::math::Polygon2d{vec_points};
    }
    return byd::common::math::Polygon2d{};
  };
  for (size_t idx = 0; idx < sections.size(); idx++) {
    const auto &section_current = sections.at(idx);
    for (uint64_t lane_id : section_current.lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end()) {
        continue;
      }
      if (it_lane->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION && it_lane->junction_id != 0) {
        TRAFFIC_LOG << fmt::format("find_ld_junction:{}  virtual_lane:{}", it_lane->junction_id, lane_id);
        ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
        break;
      }
      if (idx > 0) {
        bool find_virtual_junction{false};
        for (uint64_t lane_temp : sections.at(idx - 1).lane_ids) {
          auto [stop_line_id, _, stopline_in_back] = LaneStopLineInfo(lanes, routing_map->stop_lines, lane_temp);
          if (stopline_in_back) {
            TRAFFIC_LOG << fmt::format("find_stopline_junction:{}  prev_stop_line:{}  junction_id:{} current_section_id:{}",
                                       it_lane->junction_id, lane_temp, it_lane->junction_id, section_current.id);
            if (it_lane->junction_id != 0) {
              // ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
            } else {
              maybe_junctions_.insert({section_current.id, GetTheElementOfMap(routing_map->stop_lines, stop_line_id)});
            }
            find_virtual_junction = true;
            break;
          }
        }
        if (find_virtual_junction) {
          break;
        }
      } else {
        auto [stop_line_id, stopline_in_front, _] = LaneStopLineInfo(lanes, routing_map->stop_lines, lane_id);
        if (stopline_in_front) {
          TRAFFIC_LOG << fmt::format("find_ld_junction:{}  current_stop_line:{}  junction_id:{} current_section_id:{}",
                                     it_lane->junction_id, lane_id, it_lane->junction_id, section_current.id);
          if (it_lane->junction_id != 0) {
            ldmap_junction_ids_.insert({it_lane->junction_id, GetJunctionPolygon(it_lane->junction_id)});
          } else {
            maybe_junctions_.insert({section_current.id, GetTheElementOfMap(routing_map->stop_lines, stop_line_id)});
          }
          break;
        }
      }
    }
  }
}

double TrafficLightMapping::DistanceToJunction(const byd::common::math::Polygon2d &polygon, const TrafficLight &light) {
  double dis = std::numeric_limits<double>::infinity();
  if (!light.is_valid || !light.traffic_obj_info) {
    return dis;
  }
  if (polygon.points().empty()) {
    return dis;
  }
  Vec2d light_pos{light.traffic_obj_info->position.x, light.traffic_obj_info->position.y};
  return polygon.DistanceTo(light_pos);
}

bool TrafficLightMapping::LaneCrossHasTrafficLight(const LaneInfo &lane_rhs) {
  if (!routing_map_ptr_ || lane_rhs.points.size() < 2) {
    return false;
  }

  bool lane_has_crosswald = !lane_rhs.cross_walks.empty() && lane_rhs.cross_walks.front() != 0;
  if (!lane_has_crosswald) {
    return false;
  }
  bool is_light_block_failed = traffic_lights_.GetLightState(lane_rhs.turn_type).color == message::sensor::TLC_BLOCK_FAILED;

  auto lanes_is_in_junction = [&](const std::vector<uint64_t> &lane_ids) {
    return std::any_of(lane_ids.begin(), lane_ids.end(), [&](uint64_t lane_id_t) {
      auto it_prev_lane_t = FindTheElement(routing_map_ptr_->lanes, lane_id_t);
      if (it_prev_lane_t.has_value()) {
        return (*it_prev_lane_t)->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION ||
               (*it_prev_lane_t)->type == message::env_model::LaneType::LANE_VIRTUAL_COMMON || (*it_prev_lane_t)->junction_id != 0;
      }
      return false;
    });
  };
  bool   prev_lane_is_in_junction = lanes_is_in_junction(lane_rhs.previous_lane_ids);
  bool   succ_lane_is_in_junction = lanes_is_in_junction(lane_rhs.next_lane_ids);
  bool   is_right_dedicated       = false;  //sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane();
  double dis_threshold            = lane_belong_traffic_light_threshold;
  if (is_right_dedicated) {
    dis_threshold = dis_crosswalk_right_dedicated;
  }

  const auto &traffic_light = GetTrafficLightBaseTurnType(lane_rhs.turn_type);
  const auto &polygon       = GetTheElementOfMap(routing_map_ptr_->cross_walks, lane_rhs.cross_walks.front());

  if (polygon.points().size() < 3) {
    return false;
  }
  lane_has_crosswald = polygon.HasOverlap(
      LineSegment2d{{lane_rhs.points.begin()->x, lane_rhs.points.begin()->y}, {lane_rhs.points.rbegin()->x, lane_rhs.points.rbegin()->y}});
  double dis_polygon          = traffic_light ? polygon.DistanceTo(Vec2d{(*traffic_light).position.x, (*traffic_light).position.y})
                                              : std::numeric_limits<double>::infinity();
  bool   is_in_small_junction = !prev_lane_is_in_junction && !succ_lane_is_in_junction && lane_has_crosswald &&
                              (dis_polygon < dis_threshold || is_light_block_failed);
  TRAFFIC_LOG << fmt::format("crosswalk lane_id:{} intersection:{:d}  dis_polygon:{:.2f}  block:{:d}  right_only:{:d} res:{:d} counter:{}",
                             lane_rhs.id, lane_has_crosswald, dis_polygon, is_light_block_failed, is_right_dedicated, is_in_small_junction,
                             routing_map_ptr_->header.cycle_counter);
  return is_in_small_junction;
}

bool TrafficLightMapping::LaneStoplineTrafficLight(uint64_t section_id, const LaneInfo &lane_rhs) {
  auto find_stopline = maybe_junctions_.find(section_id);
  if (find_stopline == maybe_junctions_.end() || find_stopline->second.points().size() < 3 || lane_rhs.points.size() < 2) {
    return false;
  }
  bool is_light_block_failed = traffic_lights_.GetLightState(lane_rhs.turn_type).color == message::sensor::TLC_BLOCK_FAILED;
  bool is_right_dedicated    = false;  //sd_navigation_city_ && sd_navigation_city_->IsEgoInDedicatedRightLane();

  constexpr double extend_lenght = 3.0;

  double dis_threshold = lane_belong_traffic_light_threshold;
  if (is_right_dedicated) {
    dis_threshold = dis_crosswalk_right_dedicated;
  }

  auto  traffic_light = GetTrafficLightBaseTurnType(lane_rhs.turn_type);
  Vec2d start_p{lane_rhs.points.begin()->x, lane_rhs.points.begin()->y};
  Vec2d end_p{lane_rhs.points.rbegin()->x, lane_rhs.points.rbegin()->y};

  LineSegment2d lane_segment_t{start_p, end_p};
  LineSegment2d lineseg_stopline{start_p - lane_segment_t.unit_direction() * extend_lenght,
                                 end_p + lane_segment_t.unit_direction() * extend_lenght};

  bool   lane_through_stopline = find_stopline->second.HasOverlap(lineseg_stopline);
  double dis_polygon = traffic_light ? find_stopline->second.DistanceTo(Vec2d{(*traffic_light).position.x, (*traffic_light).position.y})
                                     : std::numeric_limits<double>::infinity();
  bool   lane_has_stop_line = lane_through_stopline && (is_light_block_failed || dis_polygon < dis_threshold);
  TRAFFIC_LOG << fmt::format("stopline  lane_id:{} intersection:{:d}  dis_poly:{:.2f} block:{:d} right_only:{:d} res:{:d} counter:{:d}",
                             lane_rhs.id, lane_through_stopline, dis_polygon, is_light_block_failed, is_right_dedicated, lane_has_stop_line,
                             routing_map_ptr_->header.cycle_counter);
  return lane_has_stop_line;
}

void TrafficLightMapping::SetTrafficLight(const RoutingMapPtr &routing_map, const RoutingMapPtr &routing_map_output) {
  if (routing_map == nullptr) {
    TRAFFIC_LOG << fmt::format("routing_map is nullptr.routing_map:{:p}  routing_map_out:{:p}", fmt::ptr(routing_map),
                               fmt::ptr(routing_map_output));
    return;
  }

  double distance_to_junction = std::numeric_limits<double>::infinity();
  previous_junction_points_.clear();
  distance_to_junction_ = std::numeric_limits<double>::infinity();

  TRAFFIC_LOG << "section_id:" << routing_map->route.navi_start.section_id << "  offset:" << routing_map->route.navi_start.s_offset;

  auto it_section = std::find_if(routing_map->route.sections.begin(), routing_map->route.sections.end(),
                                 [&routing_map](const auto &section) { return routing_map->route.navi_start.section_id == section.id; });

  auto       &lanes        = routing_map->lanes;
  auto       &lanes_output = routing_map->lanes;
  const auto &stop_lines   = routing_map->stop_lines;

  bool section_is_junction_prev    = false;
  bool section_is_junction_current = false;
  bool stop_count_dis              = false;

  std::vector<LaneInfo> lanes_traffic_all;
  TRAFFIC_LOG << fmt::format(
      "section_id:1  lane_id:2  is_virtual_in_junction:3  lane_has_stop_line:5  dis_to_traffic_light:6 is_Uturn_valid:7  "
      "is_in_small_junction:8  is_associate_traffic_light:9   turn_type:11  is_no_virtual_junction_lane:14");
  GetJunctionIdsPool(routing_map);
  for (; it_section != routing_map->route.sections.end(); it_section++) {
    section_is_junction_current = false;
    for (auto lane_id : it_section->lane_ids) {
      auto it_lane = std::find_if(lanes.begin(), lanes.end(), [&lane_id](const auto &lane_t) { return lane_t.id == lane_id; });
      if (it_lane == lanes.end()) {
        continue;
      }
      double           min_distance_to_traffic_light = LaneNearDisToJunction(*it_lane);
      constexpr double lane_virtual_dis_threshold    = 50.0;
      constexpr double lane_uturn_dis_threshold      = 30.0;
      constexpr double dis_novirtual_light           = 20.0;

      bool is_light_block_failed       = traffic_lights_.GetLightState(it_lane->turn_type).color == message::sensor::TLC_BLOCK_FAILED;
      bool lane_is_virtual_in_junction = it_lane->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION &&
                                         it_lane->junction_id != 0 &&
                                         (min_distance_to_traffic_light < lane_virtual_dis_threshold || is_light_block_failed);
      bool is_in_small_junction = LaneCrossHasTrafficLight(*it_lane);
      bool lane_has_stop_line   = LaneStoplineTrafficLight(it_section->id, *it_lane);

      bool is_no_virtual_junction_lane{false};
      if (ldmap_junction_ids_.count(it_lane->junction_id) > 0) {
        double dis = DistanceToJunction(ldmap_junction_ids_[it_lane->junction_id], traffic_lights_.GetLightState(it_lane->turn_type));
        TRAFFIC_LOG << fmt::format("junction_id:{}  dis:{:.2f}  turn:{}", it_lane->junction_id, dis,
                                   magic_enum::enum_name(it_lane->turn_type));
        if (dis < dis_novirtual_light) {
          is_no_virtual_junction_lane = true;
        }
      }

      bool is_uturn_valid =
          (it_lane->turn_type == TurnType::U_TURN) &&
          ((it_lane->junction_id == 0 && min_distance_to_traffic_light < lane_uturn_dis_threshold) ||
           (it_lane->junction_id != 0 && (min_distance_to_traffic_light < lane_virtual_dis_threshold || is_light_block_failed)));

      if (ldmap_junction_ids_.count(it_lane->junction_id) > 0 || prev_find_small_sections_.count(it_section->id) > 0) {
        TRAFFIC_LOG << "lane_id:" << it_lane->id << " section_id:" << it_lane->section_id << " has_junction:" << it_lane->junction_id;
        section_is_junction_current = true;
      }
      TRAFFIC_LOG << fmt::format("1:{}  2:{}  3:{:d}  5:{:d}  6:{{{:.2f}}} 8:{:d} 10:{:d} 11:{} 14:{:d}", it_section->id, it_lane->id,
                                 lane_is_virtual_in_junction, lane_has_stop_line, min_distance_to_traffic_light, is_uturn_valid,
                                 is_in_small_junction, it_lane->turn_type, is_no_virtual_junction_lane);

      if (!lane_is_virtual_in_junction && !is_in_small_junction && !is_no_virtual_junction_lane && !lane_has_stop_line &&
          it_lane->turn_type != TurnType::RIGHT_TURN && it_lane->turn_type != TurnType::LEFT_TURN &&
          it_lane->turn_type != TurnType::U_TURN) {
        continue;
      }
      auto [it, inserted] = prev_find_small_sections_.insert(it_section->id);
      if (inserted) {
        TRAFFIC_LOG << fmt::format("section_id:{}  is_stored_in_set.", it_section->id);
      }
      section_is_junction_current = true;

      if (it_lane->turn_type == TurnType::U_TURN && !is_uturn_valid && !is_no_virtual_junction_lane) {
        continue;
      }
      if (auto lane_out = SetLaneTrafficInfo(*it_lane, lanes_output, stop_lines); lane_out.has_value()) {
        lanes_traffic_all.emplace_back(*lane_out);
      }
    }
    // when the first junction is found.
    if (std::isinf(distance_to_junction)) {
      distance_to_junction = 0.0;
    }
    double dis_t = distance_to_junction;
    if (section_is_junction_current && !section_is_junction_prev) {
      stop_count_dis = true;
      distance_to_junction -= routing_map->route.navi_start.s_offset;
      TRAFFIC_LOG << "find_junction section:" << it_section->id << "  length:" << it_section->length
                  << "  s_offset:" << routing_map->route.navi_start.s_offset << "  distance_to_junction:" << distance_to_junction
                  << "  raw_dis:" << dis_t;
    } else if (!stop_count_dis) {
      distance_to_junction += it_section->length;
      TRAFFIC_LOG << "no_find_junction section:" << it_section->id << "  length:" << it_section->length
                  << "  distance_to_junction:" << distance_to_junction << "  raw_dis:" << dis_t;
    }
    // check the first junction has been expired .
    if (!section_is_junction_current && section_is_junction_prev) {
      if (junction_ids_vec_.empty() || junction_ids_vec_.back() != it_section->id) {
        TRAFFIC_LOG << "junction_ids_vec_emplace_back_section_id:" << it_section->id;
        junction_ids_vec_.emplace_back(it_section->id);
      }
      break;
    }
    section_is_junction_prev = section_is_junction_current;
  }
  ProcessNotRoutingLanes(routing_map, lanes_traffic_all);
  SetJunctionPoints(routing_map, lanes_traffic_all);

  if (distance_to_junction >= 0.0) {
    distance_to_junction_ = distance_to_junction;
  } else {
    distance_to_junction_ = 0.0;
  }
  TRAFFIC_LOG << fmt::format("routing_map_counter:{}  raw_distance_to_junction:{:.2f}   output_distance_to_junction:{:.2f}",
                             routing_map->header.cycle_counter, distance_to_junction, distance_to_junction_);
  // SetE2EInfo(routing_map, lanes_traffic_all);
}

void TrafficLightMapping::Process(double target_timestamp) {  // TODO: feed target timestamp
  FLOG_TLIGHT << "------------------start----------------";
  FeedLatestFrame();
  TimeSync();
  // Tracking();
  for (auto &obj_pair : perception_traffic_light_objects_) {
    auto &obj = obj_pair.second;
    FLOG_TLIGHT << "***obj_id: " << obj.id << " , position: (" << obj.position.x << ", " << obj.position.y << ", " << obj.position.z << ")"
                << " , l_w_h: (" << obj.length << ", " << obj.width << ", " << obj.height << ")";
    FLOG_TLIGHT << "status: " << obj.track_status << " ,exist_score: " << obj.exist_score << " ,track_age: " << obj.track_age;
    FLOG_TLIGHT << "direction: " << obj.attributes.traffic_light_direction << ", color: " << obj.attributes.traffic_light_color
                << " ,shape: " << obj.attributes.traffic_light_shape << " ,num: " << obj.attributes.traffic_light_num;
  }
  PercepTrfInfoPtr traffic_light_info;
  SensorDataManager::Instance()->GetLatestSensorFrame(traffic_light_info);
  if (traffic_light_info == nullptr) {
    FLOG_TLIGHT << "****************traffic_light_info == nullptr***********";
    return;
  }
  timestamp_ = traffic_light_info->header.timestamp;
  FLOG_TLIGHT << "------------------mid";
  std::unordered_map<uint32_t, cem::message::sensor::TrfObjectInfo> traffic_light_objects;
  for (auto obj : traffic_light_info->objects) {
    if (obj.type != ObjectType::TRAFFIC_LIGHT) {
      continue;
    }
    if (traffic_light_objects.count(obj.id) > 0) {
      AWARN << "Ignore repeated id of vision input traffic light.";
      continue;
    }
    traffic_light_objects[obj.id] = std::move(obj);
  }
  FLOG_TLIGHT << std::fixed << std::setprecision(3) << "timestamp:: " << timestamp_;
  FLOG_TLIGHT << "traffic light number:" << traffic_light_objects.size();
  for (auto &obj_pair : traffic_light_objects) {
    auto &obj = obj_pair.second;
    FLOG_TLIGHT << "***obj_id: " << obj.id << " , position: (" << obj.position.x << ", " << obj.position.y << ", " << obj.position.z << ")"
                << " , l_w_h: (" << obj.length << ", " << obj.width << ", " << obj.height << ")";
    FLOG_TLIGHT << "status: " << obj.track_status << " ,exist_score: " << obj.exist_score << " ,track_age: " << obj.track_age;
    FLOG_TLIGHT << "direction: " << obj.attributes.traffic_light_direction << ", color: " << obj.attributes.traffic_light_color
                << " ,shape: " << obj.attributes.traffic_light_shape << " ,num: " << obj.attributes.traffic_light_num;
  }
}

void TrafficLightMapping::LocalMapping(double timestamp) {
  // if (lost_count_ > 0) {
  //   perception_traffic_light_objects_.clear();
  // }
  FLOG_TLIGHT << " ----original perception_traffic_light_objects---";
  LogTrafficLightObjects(perception_traffic_light_objects_);
  RemoveNearTrafficLight();
  UpdataTracks(timestamp);
  FLOG_TLIGHT << " ----traffic_light_trackers_---";
  LogTrafficLightObjects(traffic_light_trackers_);
  LightAssociationResult association_result;
  Association(association_result);

  // 更新匹配的红绿灯
  UpdateAssignedTracks(association_result);

  // 更新未匹配上的红绿灯
  UpdateUnassignedTracks(association_result);

  // 创建新的红绿灯tracker
  CreateNewTracker(association_result);

  // 过滤掉丢失重复的
  RemoveTrackTrafficLight();
  deal_per_traffic_light_objects_.clear();
  for (const auto &[id, traffic_light_tracker] : traffic_light_trackers_) {
    deal_per_traffic_light_objects_.insert({id, *traffic_light_tracker->GetConstTrackedObject()});
  }
  FLOG_TLIGHT << " ----map deal_per_traffic_light_objects_---";
  LogTrafficLightObjects(deal_per_traffic_light_objects_);
}

void TrafficLightMapping::RemoveNearTrafficLight() {
  std::set<uint32_t> need_remove_ids;
  for (auto it = perception_traffic_light_objects_.begin(); it != perception_traffic_light_objects_.end(); it++) {
    if (need_remove_ids.count(it->first) == 1) {
      continue;
    }
    const auto           &it_pose = it->second.position;
    std::vector<uint32_t> near_obj_ids;
    auto                  it_next = it;
    for (++it_next; it_next != perception_traffic_light_objects_.end(); it_next++) {
      const auto &next_pose = it_next->second.position;
      if (need_remove_ids.count(it_next->first) == 1) {
        continue;
      }
      if (std::abs(it_pose.x - next_pose.x) < 9.0 && std::abs(it_pose.y - next_pose.y) < 1.5) {
        near_obj_ids.emplace_back(it_next->first);
      }
    }
    if (!near_obj_ids.empty()) {
      float    exist_score    = it->second.exist_score;
      uint32_t need_remove_id = it->first;
      for (auto id : near_obj_ids) {
        if (perception_traffic_light_objects_[id].exist_score > exist_score) {
          need_remove_ids.insert(need_remove_id);
          exist_score    = perception_traffic_light_objects_[id].exist_score;
          need_remove_id = id;
        } else {
          need_remove_ids.insert(id);
        }
      }
    }
  }

  for (auto id : need_remove_ids) {
    perception_traffic_light_objects_.erase(id);
  }
}

void TrafficLightMapping::RemoveTrackTrafficLight() {
  std::set<uint32_t> need_remove_ids;
  for (auto it = traffic_light_trackers_.begin(); it != traffic_light_trackers_.end(); it++) {
    if (need_remove_ids.count(it->first) == 1) {
      continue;
    }
    if (it->second->GetConstTrackedObject()->attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK) {
      need_remove_ids.insert(it->first);
      continue;
    }
    const auto           &it_pose = it->second->GetConstTrackedObject()->position;
    std::vector<uint32_t> near_obj_ids;
    auto                  it_next = it;
    for (++it_next; it_next != traffic_light_trackers_.end(); it_next++) {
      const auto &next_pose = it_next->second->GetConstTrackedObject()->position;
      ;
      if (need_remove_ids.count(it_next->first) == 1) {
        continue;
      }
      if (std::abs(it_pose.x - next_pose.x) < 15.0 && std::abs(it_pose.y - next_pose.y) < 1.5) {
        near_obj_ids.emplace_back(it_next->first);
      }
    }
    if (!near_obj_ids.empty()) {
      int      lost_age       = it->second->LostAge();
      uint32_t need_remove_id = it->first;
      for (auto id : near_obj_ids) {
        if (traffic_light_trackers_[id]->LostAge() < lost_age) {
          need_remove_ids.insert(need_remove_id);
          lost_age       = traffic_light_trackers_[id]->LostAge();
          need_remove_id = id;
        } else {
          need_remove_ids.insert(id);
        }
      }
    }
  }

  for (auto id : need_remove_ids) {
    traffic_light_trackers_.erase(id);
  }
  need_remove_ids.clear();
  for (auto it = traffic_light_trackers_.begin(); it != traffic_light_trackers_.end(); it++) {
    if (need_remove_ids.count(it->first) == 1) {
      continue;
    }
    const auto &it_pose = it->second->GetConstTrackedObject()->position;

    if (auto it_exist = std::find_if(traffic_light_trackers_.begin(), traffic_light_trackers_.end(),
                                     [](const auto &val_t) { return std::fabs(val_t.second->GetConstTrackedObject()->position.y) < 10.0; });
        it_exist != traffic_light_trackers_.end() && std::fabs(it_pose.y) > 25) {
      need_remove_ids.insert(it->first);
      FLOG_TLIGHT << fmt::format(
          "need_remove_id_pos lateral_error obj_id:{} pos:[{:.2f},{:.2f}]  is too big.need remove the traffic_light.", it->first, it_pose.x,
          it_pose.y);
      continue;
    }
  }
  for (auto id : need_remove_ids) {
    traffic_light_trackers_.erase(id);
  }

  std::map<uint32_t, double> diff_set;

  constexpr double max_distance_gap         = 50;
  constexpr double max_distance_polygon_gap = 30;
  bool             exist_less_gap           = false;
  need_remove_ids.clear();
  if (previous_junction_points_.size() >= 4) {
    std::vector<Vec2d> points;
    points.reserve(previous_junction_points_.size());
    for (const auto &point : previous_junction_points_) {
      points.emplace_back((Vec2d){point.x, point.y});
    }
    FLOG_TLIGHT << fmt::format("polygon first point:{:.2f}  {:.2f}", points.front().x(), points.front().y());
    byd::common::math::Polygon2d junction_points{points};
    for (const auto &val : traffic_light_trackers_) {
      Vec2d pos_t{val.second->GetConstTrackedObject()->position.x, val.second->GetConstTrackedObject()->position.y};
      diff_set.insert({val.first, junction_points.DistanceTo(pos_t)});
      FLOG_TLIGHT << fmt::format("obj_id:{}  pos:{:.2f},{:.2f}  polygon_dis:{:.2f}", val.first, pos_t.x(), pos_t.y(), diff_set[val.first]);
    }
    for (auto &traffic_light_tracker : traffic_light_trackers_) {
      if (need_remove_ids.count(traffic_light_tracker.first) == 1) {
        continue;
      }
      double dis_t = diff_set[traffic_light_tracker.first];
      if (dis_t > max_distance_polygon_gap) {
        need_remove_ids.insert(traffic_light_tracker.first);
        FLOG_TLIGHT << fmt::format("need_remove_id_pos dis_error traffic_light_id:{}   polygon_diff:{:.2f}", traffic_light_tracker.first,
                                   dis_t);
      }
    }
    for (auto id : need_remove_ids) {
      traffic_light_trackers_.erase(id);
    }

    return;
  }

  need_remove_ids.clear();
  if (distance_to_junction_ > 1.0 && !std::isinf(distance_to_junction_) && !traffic_light_trackers_.empty()) {
    for (const auto &val : traffic_light_trackers_) {
      Eigen::Vector2d pos_t{val.second->GetConstTrackedObject()->position.x, val.second->GetConstTrackedObject()->position.y};
      diff_set.insert({val.first, pos_t.norm() - distance_to_junction_});
      FLOG_TLIGHT << fmt::format("obj_id:{}  pos:{:.2f},{:.2f}  dis:{:.2f}", val.first, pos_t.x(), pos_t.y(), diff_set[val.first]);
      if (!exist_less_gap && std::fabs(diff_set[val.first]) < max_distance_gap) {
        exist_less_gap = true;
      }
    }
  }
  // for (auto &traffic_light_tracker : traffic_light_trackers_) {
  //   if (need_remove_ids.count(traffic_light_tracker.first) == 1) {
  //     continue;
  //   }
  //   const auto &it_pose = traffic_light_tracker.second->GetConstTrackedObject()->position;
  //   if (distance_to_junction_ > 1.0 && !std::isinf(distance_to_junction_) && exist_less_gap) {
  //     Eigen::Vector2d pos{it_pose.x, it_pose.y};
  //     if (std::fabs(diff_set[traffic_light_tracker.first]) > max_distance_gap) {
  //       need_remove_ids.insert(traffic_light_tracker.first);
  //       FLOG_TLIGHT << fmt::format(
  //           "need_remove_id_pos dis_error traffic_light_id:{}  pos:[{:.2f},{:.2f}]  distance is to far. previous_distance_to_junction_:{:.2f}  traffic_light_pos_dis:{:.2f}  "
  //           "diff:{:.2f}",
  //           traffic_light_tracker.first, pos.x(), pos.y(), distance_to_junction_, pos.norm(), pos.norm() - distance_to_junction_);
  //       continue;
  //     }
  //   }
  // }
  // for (auto id : need_remove_ids) {
  //   traffic_light_trackers_.erase(id);
  // }
}

bool TrafficLightMapping::IsValid(const TrafficLight &light) {
  return light.is_valid && light.traffic_obj_info;
};

void TrafficLightMapping::ValidateComparePrevLight(const TrafficLight                                                      &light_previous,
                                                   std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &valid_objs) {
  if (!IsValid(light_previous)) {
    TRAFFIC_REC_LOG << "light_previous_is_invalid.";
    return;
  }

  constexpr double dis_max   = 20.0;
  const auto       DisPoints = [](cem::message::common::Point3DD lhs, cem::message::common::Point3DD rhs) {
    double delta_x = lhs.x - rhs.x;
    double delta_y = lhs.y - rhs.y;
    return std::sqrt(delta_x * delta_x + delta_y * delta_y);
  };

  auto DisTooFar = [DisPoints, dis_max](const TrafficLight &light_previous, const cem::message::sensor::TrfObjectInfo &light_current_obj) {
    if (double dis_temp = DisPoints(light_current_obj.position, light_previous.traffic_obj_info->position); dis_temp > dis_max) {
      TRAFFIC_REC_LOG << fmt::format("prev_obj:{}  pos:{:.2f}-{:.2f}  cur_obj:{}  pos:{:.2f}-{:.2f} dis_temp:{:.2f}  dis_max:{:.2f}",
                                     light_previous.traffic_obj_info->id, light_previous.traffic_obj_info->position.x,
                                     light_previous.traffic_obj_info->position.y, light_current_obj.id, light_current_obj.position.x,
                                     light_current_obj.position.y, dis_temp, dis_max);
      return true;
    }
    return false;
  };
  valid_objs.erase(std::remove_if(valid_objs.begin(), valid_objs.end(),
                                  [&light_previous, DisTooFar](const auto &it) { return DisTooFar(light_previous, *it); }),
                   valid_objs.end());
}

bool TrafficLightMapping::TransformPostionOfObjs(double timestamp) {
  if (!IsValid(traffic_lights_previous_.left) && !IsValid(traffic_lights_previous_.u_turn) && !IsValid(traffic_lights_previous_.straight) &&
      !IsValid(traffic_lights_previous_.right)) {
    return true;
  }
  const auto &T_prev_to_current = GetTransformMatrix(timestamp);
  if (!T_prev_to_current) {
    return false;
  }
  auto PosTrans = [&T_prev_to_current, this](TrafficLight &light) {
    if (!this->IsValid(light)) {
      return;
    }
    Eigen::Vector3d point_src(light.traffic_obj_info->position.x, light.traffic_obj_info->position.y, light.traffic_obj_info->position.z);
    auto            Pos                = *T_prev_to_current * point_src;
    light.traffic_obj_info->position.x = Pos.x();
    light.traffic_obj_info->position.y = Pos.y();
    light.traffic_obj_info->position.z = Pos.z();
  };
  PosTrans(traffic_lights_previous_.left);
  PosTrans(traffic_lights_previous_.u_turn);
  PosTrans(traffic_lights_previous_.straight);
  PosTrans(traffic_lights_previous_.right);
  return true;
}
// edit by liu.mianli
void TrafficLightMapping::TransformIntersection(double timestamp) {
  for (auto &[id, zone] : intersection_zone_map_) {
    if (zone.timestamp_ != timestamp) {
      const auto &T_prev_to_current = GetTransformMatrix(zone.timestamp_, timestamp);
      if (!T_prev_to_current) {
        AERROR << "T_prev_to_current FAILED, time gap:" << timestamp - zone.timestamp_;
        continue;
      }
      std::vector<Vec2d> points_trans(zone.polygon_.num_points());
      for (int i = 0; i < zone.polygon_.num_points(); i++) {
        auto           &p = zone.polygon_.points()[i];
        Eigen::Vector3d point_src(p.x(), p.y(), 1.0);
        auto            pos = *T_prev_to_current * point_src;
        points_trans[i].set_x(pos.x());
        points_trans[i].set_y(pos.y());
      }
      zone.timestamp_trans_ = timestamp;
      zone.polygon_trans_   = byd::common::math::Polygon2d{points_trans};
    }
  }
}

std::optional<Eigen::Isometry3d> TrafficLightMapping::GetTransformMatrix(double timestamp) {
  LocalizationPtr target_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, target_pose);
  LocalizationPtr measurement_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(last_tracker_time_, 0.05, measurement_pose);
  if (!(target_pose && measurement_pose)) {
    traffic_light_trackers_.clear();
    return std::nullopt;
  }

  return CalcRotateTranslateMatrix(measurement_pose, target_pose);
}
std::optional<Eigen::Isometry3d> TrafficLightMapping::GetTransformMatrix(double timestamp_prev, double timestamp_now) {
  LocalizationPtr target_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp_now, 0.05, target_pose);
  LocalizationPtr measurement_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp_prev, 0.05, measurement_pose);
  if (!(target_pose && measurement_pose)) {
    return std::nullopt;
  }
  return CalcRotateTranslateMatrix(measurement_pose, target_pose);
}

void TrafficLightMapping::UpdataTracks(double timestamp) {
  if (traffic_light_trackers_.empty()) {
    return;
  }
  LocalizationPtr target_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05, target_pose);
  LocalizationPtr measurement_pose = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(last_tracker_time_, 0.05, measurement_pose);
  if (!(target_pose && measurement_pose)) {
    traffic_light_trackers_.clear();
    return;
  }

  Eigen::Isometry3d T;
  T = CalcRotateTranslateMatrix(measurement_pose, target_pose);

  for (auto &[track_id, tracker_ptr] : traffic_light_trackers_) {
    auto           &position = tracker_ptr->GetTrackedObject()->position;
    Eigen::Vector3d point_src(position.x, position.y, position.z);
    auto            point_dst = T * point_src;
    position.x                = point_dst.x();
    position.y                = point_dst.y();
    position.z                = point_dst.z();
  }
}

void TrafficLightMapping::Association(LightAssociationResult &association_result) {
  std::set<uint32_t> used_tracks;
  std::set<uint32_t> used_detects;
  for (const auto &[detect_id, detect_obj] : perception_traffic_light_objects_) {
    const auto &position = detect_obj.position;
    // min_err_x, err_y , match_id
    std::pair<double, std::pair<double, uint32_t>> min_err_x{100, {}};
    // min_err_y, err_x, match_id
    std::pair<double, std::pair<double, uint32_t>> min_err_y{100, {}};
    // min_err_dis, match_id
    std::pair<double, uint32_t> min_err_dis{100.0, 0};
    bool                        has_same_id{false};
    for (const auto &[track_id, tracker_ptr] : traffic_light_trackers_) {
      const auto &track_obj = tracker_ptr->GetConstTrackedObject();
      if (used_tracks.count(track_id) == 1) {
        continue;
      }
      const auto &track_pos = track_obj->position;
      double      err_x     = std::abs(track_pos.x - position.x);
      double      err_y     = std::abs(track_pos.y - position.y);
      double      dis       = err_x * err_x + err_y * err_y;
      // 优先匹配相同id的
      if (track_obj->attributes.traffic_light_direction != detect_obj.attributes.traffic_light_direction) {
        continue;
      }
      if (track_id == detect_id) {
        min_err_dis.first  = dis;
        min_err_dis.second = track_id;
        has_same_id        = true;
        break;
      }

      // id不同的求最近的dis
      if (dis < min_err_dis.first) {
        min_err_dis.first  = dis;
        min_err_dis.second = track_id;
      }

      // id不同的求最近的err_x
      if (err_x < min_err_x.first) {
        min_err_x.first         = err_x;
        min_err_x.second.first  = err_y;
        min_err_x.second.second = track_id;
      }

      // id不同的求最近的err_y
      if (err_y < min_err_y.first) {
        min_err_y.first         = err_y;
        min_err_y.second.first  = err_x;
        min_err_y.second.second = track_id;
      }
    }
    if (has_same_id || min_err_dis.first < 5.0) {
      FLOG_TLIGHT << " assignments detect_id: " << detect_id << " ,track id: " << min_err_dis.second
                  << " ,min_err_dis: " << min_err_dis.first << " ,has_same_id: " << has_same_id;
      association_result.assignments.emplace_back(std::make_tuple(detect_id, min_err_dis.second, min_err_dis.first));
      used_tracks.insert(min_err_dis.second);
      used_detects.insert(detect_id);
    } else if (min_err_y.first < 1.0 && min_err_y.second.first < 16.0) {
      FLOG_TLIGHT << " assignments detect_id: " << detect_id << " ,track id: " << min_err_y.second.second
                  << " ,min_err_y: " << min_err_y.first << " , min_err_x:" << min_err_y.second.first << " ,has_same_id: " << has_same_id;
      association_result.assignments.emplace_back(std::make_tuple(detect_id, min_err_y.second.second, min_err_dis.first));
      used_tracks.insert(min_err_y.second.second);
      used_detects.insert(detect_id);
    }
  }

  for (const auto &[id, detect_obj] : perception_traffic_light_objects_) {
    if (used_detects.count(id) != 1) {
      association_result.unsigned_objects.emplace_back(id);
    }
  }

  for (const auto &[track_id, tracker_ptr] : traffic_light_trackers_) {
    if (used_tracks.count(track_id) != 1) {
      association_result.unassigned_tracks.emplace_back(track_id);
    }
  }
}

void TrafficLightMapping::UpdateAssignedTracks(const LightAssociationResult &association_result) {
  if (association_result.assignments.empty()) {
    return;
  }
  for (const auto &assign : association_result.assignments) {
    auto detect_id = std::get<0>(assign);
    auto track_id  = std::get<1>(assign);
    FLOG_TLIGHT << fmt::format("detect_id:{}  track_id:{}", detect_id, track_id);
    traffic_light_trackers_[track_id]->UpdateWithDetectedObject(perception_traffic_light_objects_[detect_id]);
    // id变化时，删除掉原来的
    if (track_id != detect_id) {
      traffic_light_trackers_[detect_id] = traffic_light_trackers_[track_id];
      traffic_light_trackers_.erase(track_id);
    }
  }
}

void TrafficLightMapping::UpdateUnassignedTracks(const LightAssociationResult &association_result) {
  if (association_result.unassigned_tracks.empty()) {
    return;
  }

  for (const auto &unassigned_id : association_result.unassigned_tracks) {
    traffic_light_trackers_[unassigned_id]->UpdateWithoutDetectedObject();
  }

  for (auto it = traffic_light_trackers_.begin(); it != traffic_light_trackers_.end();) {
    if (it->second->IsDie()) {
      it = traffic_light_trackers_.erase(it);  // erase 返回下一个有效的迭代器
    } else {
      ++it;
    }
  }
}

void TrafficLightMapping::CreateNewTracker(const LightAssociationResult &association_result) {
  if (association_result.unsigned_objects.empty()) {
    return;
  }

  for (auto unsigned_id : association_result.unsigned_objects) {
    FLOG_TLIGHT << "CreateNewTracker unsigned_objects id: " << unsigned_id;
    const auto                    &obj         = perception_traffic_light_objects_[unsigned_id];
    std::shared_ptr<TrfObjectInfo> trf_obj_ptr = std::make_shared<TrfObjectInfo>(obj);
    TrafficLightTrackerPtr         traffic_light_tracker_ptr;
    traffic_light_tracker_ptr.reset(new TrafficLightTracker);
    if (traffic_light_tracker_ptr->Init(trf_obj_ptr)) {
      traffic_light_trackers_[unsigned_id] = traffic_light_tracker_ptr;
    }
  }
}

void TrafficLightMapping::LogTrafficLightObjects(const PerTrfObjectInfoMaps &traffic_light_objects) {
  for (auto &obj_pair : traffic_light_objects) {
    auto &obj = obj_pair.second;
    FLOG_TLIGHT << std::fixed << std::setprecision(5) << "obj_id: " << obj.id << " , position: (" << obj.position.x << ", "
                << obj.position.y << ", " << obj.position.z << ")"
                << "  direction: " << obj.attributes.traffic_light_direction << ", color: " << obj.attributes.traffic_light_color
                << " ,shape: " << obj.attributes.traffic_light_shape << " ,num: " << obj.attributes.traffic_light_num
                << "  is_flash:" << obj.attributes.traffic_light_flashing << "  publish_time:" << obj.publish_time;
  }
}

void TrafficLightMapping::LogTrafficLightObjects(const std::unordered_map<uint32_t, TrafficLightTrackerPtr> &traffic_light_objects) {
  for (const auto &[track_id, tracker_ptr] : traffic_light_objects) {
    const auto &obj = tracker_ptr->GetConstTrackedObject();
    FLOG_TLIGHT << std::fixed << std::setprecision(5) << "---TrafficLightTracker obj_id: " << obj->id << " , position: (" << obj->position.x
                << ", " << obj->position.y << ", " << obj->position.z << ")"
                << " ,lost age: " << tracker_ptr->LostAge() << "  direction: " << obj->attributes.traffic_light_direction
                << ", color: " << obj->attributes.traffic_light_color << " ,shape: " << obj->attributes.traffic_light_shape
                << " ,num: " << obj->attributes.traffic_light_num << "  publish_time:" << obj->publish_time;
  }
}

void TrafficLightMapping::SetSrTrafficLight(std::vector<TrafficLightMap> &traffic_lights) {
  traffic_lights.clear();

  for (const auto &[id, obj] : deal_per_traffic_light_objects_) {
    if (obj.position.x < 10) {
      continue;
    }
    if (obj.attributes.traffic_light_color != TrafficLightColorType::TLC_GREEN &&
        obj.attributes.traffic_light_color != TrafficLightColorType::TLC_RED &&
        obj.attributes.traffic_light_color != TrafficLightColorType::TLC_YELLOW) {
      TransSrTrafficLight(obj, traffic_lights.emplace_back(), 55, 19);
      continue;
    }
    TRAFFIC_LOG << fmt::format("sr_id:{}  color:{:d}", obj.id, obj.attributes.traffic_light_color);
    switch (obj.attributes.traffic_light_shape) {
      case TrafficLightShapeType::TLS_CIRCULAR:
      case TrafficLightShapeType::TLS_HEART_SHAPE:
      case TrafficLightShapeType::TLS_RECTANGLE:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 1, 1);
        break;
      case TrafficLightShapeType::TLS_UP_ARROW:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 2, 4);
        break;
      case TrafficLightShapeType::TLS_UP_LEFT_ARROW:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 3, 4);
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 3, 2);
        break;
      case TrafficLightShapeType::TLS_UP_RIGHT_ARROW:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 4, 4);
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 4, 3);
        break;
      case TrafficLightShapeType::TLS_LEFT_ARROW:
      case TrafficLightShapeType::TLS_TURN_ARROUND_ARROW:
      case TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 5, 2);
        break;
      case TrafficLightShapeType::TLS_RIGHT_ARROW:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 6, 3);
        break;
      default:
        TransSrTrafficLight(obj, traffic_lights.emplace_back(), 7, 19);
        break;
    }
  }
}

void TrafficLightMapping::TransSrTrafficLight(const cem::message::sensor::TrfObjectInfo &ori_light,
                                              cem::message::env_model::TrafficLightMap &out_light, uint64_t id, uint32_t shape) {
  out_light.id              = id;
  out_light.center_position = ori_light.position;
  out_light.light_countdown = ori_light.attributes.traffic_light_num;
  out_light.shape           = shape;
  out_light.light_status    = LightColorConvert(ori_light.attributes.traffic_light_color);
}

void TrafficLightMapping::Tracking() {
  // 目标：输出状态稳定的traffic light目标。
  // TODO：获取单帧感知输入并校验关键感知数据。
  // TODO：校验感知输入，如果质量很好，基本没有id、pose、color、type等跳变，不要做tracking。

  // TODO：如果需要tracking，需要先association再tracking
}

void TrafficLightMapping::LaneAssociation() {
  // 目标：给出稳定的红绿灯绑定车道的拓扑关系(需要等下map_fusion的输出，决定绑定高精还是感知lane)
}

void TrafficLightMapping::SetLightsTurn(const std::vector<TrafficLightShapeType> &shapes, LightTurn turn) {
  for (auto &obj : perception_traffic_lights_) {
    auto it = std::find(shapes.begin(), shapes.end(), obj.attributes.traffic_light_shape);
    if (it != shapes.end()) {
      obj.turn.emplace_back(turn);
    }
  }
}

void TrafficLightMapping::SetTrafficLightColor() {
  for (auto &obj : perception_traffic_lights_) {
    if (obj.attributes.traffic_light_flashing) {
      switch (obj.attributes.traffic_light_color) {
        case TrafficLightColorType::TLC_RED:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_RED_FLASHING;
          break;
        case TrafficLightColorType::TLC_GREEN:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_GREEN_FLASHING;
          break;
        case TrafficLightColorType::TLC_YELLOW:
          obj.attributes.traffic_light_color = TrafficLightColorType::TLC_YELLOW_FLASHING;
          break;
        default:
          break;
      };
    }
  }
}

void TrafficLightMapping::FilterLightsByPosition() {
  if (perception_traffic_lights_.empty()) {
    return;
  }
  constexpr double dis_max_lateral   = 30.0;
  constexpr double dis_min_longitude = -10.0;
  constexpr double dis_max_longitude = 200.0;
  std::sort(perception_traffic_lights_.begin(), perception_traffic_lights_.end(), [](const auto &lhs, const auto &rhs) {
    if (std::fabs(lhs.position.x - rhs.position.x) < 1.0) {
      return lhs.position.y < rhs.position.y;
    }
    return lhs.position.x < rhs.position.x;
  });
  std::vector<std::size_t> need_remove_indexes;
  for (std::size_t idx = 0; idx != perception_traffic_lights_.size(); idx++) {
    const auto &obj = perception_traffic_lights_[idx];
    if (std::fabs(obj.position.y) > dis_max_lateral) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_y_reason:{:2f}", obj.id, obj.position.y);
      need_remove_indexes.push_back(idx);
    } else if (obj.position.x > dis_max_longitude) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_x_reason:{:2f}", obj.id, obj.position.x);
      need_remove_indexes.push_back(idx);
    } else if (obj.position.x < dis_min_longitude) {
      TRAFFIC_REC_LOG << fmt::format("obj_id:{}  deleted_x_reason_less_zero:{:2f}", obj.id, obj.position.x);
      need_remove_indexes.push_back(idx);
    }
  }
  for (auto it = need_remove_indexes.rbegin(); it != need_remove_indexes.rend(); ++it) {
    perception_traffic_lights_.erase(perception_traffic_lights_.begin() + static_cast<int64_t>(*it));
  }

  if (perception_traffic_lights_.size() < 2) {
    return;
  }

  constexpr double max_x_gap  = 80.0;
  double           base_x_dis = perception_traffic_lights_.begin()->position.x;
  perception_traffic_lights_.erase(std::remove_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(),
                                                  [&](const cem::message::sensor::TrfObjectInfo &traffic_light) {
                                                    return traffic_light.position.x - base_x_dis > max_x_gap;
                                                  }),
                                   perception_traffic_lights_.end());
}

std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> TrafficLightMapping::SatisfyTrafficLight(
    const std::vector<TrafficLightShapeType> &shapes, const std::vector<TrafficLightColorType> &colors, LightTurn turn) {
  std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> res;
  for (TrafficLightShapeType type_val : shapes) {
    for (TrafficLightColorType color_val : colors) {
      for (auto it_light = perception_traffic_lights_.begin(); it_light != perception_traffic_lights_.end(); ++it_light) {
        const auto &obj = *it_light;
        auto        it  = std::find(obj.turn.begin(), obj.turn.end(), turn);
        if ((it != obj.turn.end() || obj.turn.empty()) && obj.attributes.traffic_light_shape == type_val &&
            obj.attributes.traffic_light_color == color_val) {
          TRAFFIC_REC_LOG << fmt::format("match_obj_find  turn:{:<12} obj.id:{}", magic_enum::enum_name(turn), obj.id);
          res.emplace_back(it_light);
        }
      }
    }
  }

  return res;
}

bool TrafficLightMapping::TrafficLightPositionTrack(std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &objs,
                                                    const TrafficLight &previous_light) {
  if (objs.empty()) {
    return false;
  };
  if (!previous_light.is_valid || !previous_light.traffic_obj_info) {
    return true;
  }
  constexpr double max_dis_between_prev = 25.0;
  objs.erase(std::remove_if(objs.begin(), objs.end(),
                            [&](const std::vector<cem::message::sensor::TrfObjectInfo>::iterator &it) {
                              double delta_x  = it->position.x - previous_light.traffic_obj_info->position.x;
                              double delta_y  = it->position.y - previous_light.traffic_obj_info->position.y;
                              double dis_prev = std::sqrt(delta_x * delta_x + delta_y * delta_y);
                              if (dis_prev > max_dis_between_prev) {
                                GEOMETRY_LOG << fmt::format("obj_id:{}  dis_prev:{:.2f}  ", it->id, dis_prev);
                                return true;
                              }
                              return false;
                            }),
             objs.end());

  return true;
}

cem::message::sensor::TrfObjectInfo TrafficLightMapping::FindOptObj(
    const TrafficLight &previous_light, std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &valid_objs) {
  if (valid_objs.empty()) {
    return {};
  }
  if (valid_objs.front()->attributes.traffic_light_shape != TrafficLightShapeType::TLS_UNKNOWN &&
      valid_objs.front()->attributes.traffic_light_shape != TrafficLightShapeType::TLS_OTHER_SHAPE) {
    ValidateComparePrevLight(previous_light, valid_objs);
  }
  if (valid_objs.size() == 1) {
    return *(valid_objs.front());
  }
  if (previous_light.is_valid && previous_light.traffic_obj_info) {
    const auto              &prev_pos = previous_light.traffic_obj_info->position;
    std::vector<std::size_t> idx_vec;
    constexpr double         gap_dis = 10.0;
    for (std::size_t idx = 0; idx < valid_objs.size(); idx++) {
      const auto &obj     = *(valid_objs[idx]);
      double      delta_x = obj.position.x - prev_pos.x;
      double      delta_y = obj.position.y - prev_pos.y;
      if (std::sqrt(delta_x * delta_x + delta_y * delta_y) < gap_dis) {
        idx_vec.emplace_back(idx);
      }
    }
    if (idx_vec.size() == 1) {
      TRAFFIC_REC_LOG << fmt::format("min_distance_id:{}", valid_objs[idx_vec.front()]->id);
      return *(valid_objs[idx_vec.front()]);
    }
  }
  auto sort_vec = valid_objs;
  std::sort(sort_vec.begin(), sort_vec.end(),
            [](const auto &lhs, const auto &rhs) { return std::fabs(lhs->position.y) < std::fabs(rhs->position.y); });
  if (std::fabs(sort_vec[1]->position.y - sort_vec[0]->position.y) > 3.0) {
    TRAFFIC_REC_LOG << fmt::format("min_lateral_distance_id:{}", valid_objs.front()->id);
    return *sort_vec[0];
  }
  TRAFFIC_REC_LOG << fmt::format("default_id:{}", valid_objs.front()->id);
  return *(valid_objs.front());
};

void TrafficLightMapping::SetTrafficLights(LightTurn turn, const std::vector<TrafficLightShapeType> &shapes,
                                           const TrafficLight &previous_light, TrafficLight &traffic_light) {
  if (perception_traffic_lights_.empty()) {
    return;
  }

  auto SetLightT = [&](const cem::message::sensor::TrfObjectInfo &obj, TrafficLight &traffic_light_t) {
    traffic_light_t.traffic_light_flashing = obj.attributes.traffic_light_flashing;
    traffic_light_t.traffic_light_num      = obj.attributes.traffic_light_num;
    traffic_light_t.color                  = obj.attributes.traffic_light_color;
    traffic_light_t.traffic_obj_info       = std::make_shared<cem::message::sensor::TrfObjectInfo>(obj);
  };
  traffic_light.is_valid           = true;
  traffic_light.perception_seq_num = perception_traffic_light_info_->header.cycle_counter;
  auto shapes_add                  = shapes;
  if (turn != LightTurn::RIGHT) {
    shapes_add.emplace_back(TrafficLightShapeType::TLS_CIRCULAR);
  }
  auto shapes_all = shapes_add;
  shapes_all.emplace_back(TrafficLightShapeType::TLS_UNKNOWN);
  shapes_all.emplace_back(TrafficLightShapeType::TLS_OTHER_SHAPE);

  std::size_t unknown_shape_obj_num =
      std::count_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(), [](const auto &light) {
        return light.attributes.traffic_light_shape == TrafficLightShapeType::TLS_UNKNOWN ||
               light.attributes.traffic_light_shape == TrafficLightShapeType::TLS_OTHER_SHAPE;
      });
  bool is_exist_flash = std::find_if(perception_traffic_lights_.begin(), perception_traffic_lights_.end(),
                                     [](const cem::message::sensor::TrfObjectInfo &light) {
                                       return light.attributes.traffic_light_flashing;
                                     }) != perception_traffic_lights_.end();

  auto valid_objs = SatisfyTrafficLight(
      shapes,
      {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_YELLOW,
       TrafficLightColorType::TLC_GREEN_FLASHING, TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
      turn);

  auto FindOptObj = [](const std::vector<std::vector<cem::message::sensor::TrfObjectInfo>::iterator> &valid_objs,
                       const TrafficLight &previous_light) -> cem::message::sensor::TrfObjectInfo {
    if (valid_objs.size() == 1) {
      return *(valid_objs.front());
    }
    if (previous_light.is_valid && previous_light.traffic_obj_info) {
      const auto              &prev_pos = previous_light.traffic_obj_info->position;
      std::vector<std::size_t> idx_vec;
      constexpr double         gap_dis = 10.0;
      double                   min_dis = std::numeric_limits<double>::infinity();
      size_t                   min_idx = 0;
      for (std::size_t idx = 0; idx < valid_objs.size(); idx++) {
        const auto &obj     = *(valid_objs[idx]);
        double      delta_x = obj.position.x - prev_pos.x;
        double      delta_y = obj.position.y - prev_pos.y;
        double      dis_val = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (dis_val < gap_dis) {
          idx_vec.emplace_back(idx);
        }
        if (min_dis > dis_val) {
          min_dis = dis_val;
          min_idx = idx;
        }
      }
      if (idx_vec.size() == 1) {
        TRAFFIC_REC_LOG << fmt::format("find_min_distance_id:{}", valid_objs[idx_vec.front()]->id);
        return *(valid_objs[idx_vec.front()]);
      }
      if (idx_vec.size() >= 2 && !std::isinf(min_dis)) {
        TRAFFIC_REC_LOG << fmt::format("find_>2_min_distance_id:{}", valid_objs[min_idx]->id);
        return *(valid_objs[min_idx]);
      }
    }
    auto sort_vec = valid_objs;
    std::sort(sort_vec.begin(), sort_vec.end(),
              [](const auto &lhs, const auto &rhs) { return std::fabs(lhs->position.y) < std::fabs(rhs->position.y); });
    if (std::fabs(sort_vec[1]->position.y - sort_vec[0]->position.y) > 3.0) {
      TRAFFIC_REC_LOG << fmt::format("min_lateral_distance_id:{}", valid_objs.front()->id);
      return *sort_vec[0];
    }
    TRAFFIC_REC_LOG << fmt::format("default_id:{}", valid_objs.front()->id);
    return *(valid_objs.front());
  };

  if (!valid_objs.empty()) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_ARROW_LIGHT;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_CIRCULAR},
                                              {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_RED,
                                               TrafficLightColorType::TLC_YELLOW, TrafficLightColorType::TLC_GREEN_FLASHING,
                                               TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
                                              turn);
             turn != LightTurn::RIGHT && !valid_objs.empty()) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_CIRCLE_LIGHT;
  } else if (turn == LightTurn::RIGHT) {
    if (previous_light.is_valid && previous_light.traffic_reason != byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ &&
        traffic_light.perception_seq_num - previous_light.perception_seq_num <= max_stay_frames_with_info) {
      TRAFFIC_REC_LOG << "stay_prev_info turn:" << magic_enum::enum_name(turn);
      traffic_light                = previous_light;
      traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
      traffic_light.stay_prev_counter++;
    } else {
      traffic_light.traffic_light_flashing = false;
      traffic_light.traffic_light_num      = 1000;
      traffic_light.color                  = TLC_GREEN;
      traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
    }
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                              {TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_GREEN_FLASHING}, turn);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_GREEN_FLASHING : message::sensor::TLC_GREEN;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                              {TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_RED_FLASHING}, turn);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_RED_FLASHING : message::sensor::TLC_RED;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                              {TrafficLightColorType::TLC_YELLOW, TrafficLightColorType::TLC_YELLOW_FLASHING}, turn);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.color                  = is_exist_flash ? message::sensor::TLC_YELLOW_FLASHING : message::sensor::TLC_YELLOW;
    traffic_light.traffic_light_flashing = is_exist_flash;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_UNKNOWN_GREEN;
  } else if (valid_objs = SatisfyTrafficLight({TrafficLightShapeType::TLS_UNKNOWN, TrafficLightShapeType::TLS_OTHER_SHAPE},
                                              {TrafficLightColorType::TLC_RED, TrafficLightColorType::TLC_YELLOW,
                                               TrafficLightColorType::TLC_GREEN, TrafficLightColorType::TLC_GREEN_FLASHING,
                                               TrafficLightColorType::TLC_YELLOW_FLASHING, TrafficLightColorType::TLC_RED_FLASHING},
                                              turn);
             !valid_objs.empty() && valid_objs.size() == unknown_shape_obj_num) {
    SetLightT(FindOptObj(valid_objs, previous_light), traffic_light);
    traffic_light.color          = message::sensor::TLC_BLURRING_MODE;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_SHAPE;
  } else if (valid_objs = SatisfyTrafficLight(
                 shapes_all, {TrafficLightColorType::TLC_OTHER, TrafficLightColorType::TLC_BLACK, TrafficLightColorType::TLC_UNKNOWN},
                 turn);
             !valid_objs.empty()) {
    traffic_light.traffic_light_flashing = false;
    traffic_light.traffic_light_num      = 1000;
    traffic_light.color                  = message::sensor::TLC_BLURRING_MODE;
    traffic_light.traffic_reason         = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
  } else if (previous_light.is_valid && previous_light.traffic_reason != byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ &&
             traffic_light.perception_seq_num - previous_light.perception_seq_num <= max_stay_frames_with_info) {
    TRAFFIC_REC_LOG << "stay_prev_info turn:" << magic_enum::enum_name(turn);
    traffic_light                = previous_light;
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::STAY_PREV;
    traffic_light.stay_prev_counter++;
  } else {
    TRAFFIC_REC_LOG << "no find any match info. turn:" << static_cast<int>(turn);
    traffic_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE;
  }
}

void TrafficLightMapping::FillPerceptionTrafficLights() {
  if (!perception_traffic_light_info_) {
    TRAFFIC_REC_LOG << "perception_traffic_light_info_ is nullptr.";
    return;
  }

  // if (fabs(timestamp_ - traffic_light_info->header.timestamp) < 1e-3) {
  //   return;
  // }

  TRAFFIC_REC_LOG << fmt::format("traffic_light_info_cycle_counter:{}  publish_time:{:.3f}  measurement_time:{:.3f} ",
                                 perception_traffic_light_info_->header.cycle_counter,
                                 perception_traffic_light_info_->header.recv_timestamp, perception_traffic_light_info_->header.timestamp);
  timestamp_                    = perception_traffic_light_info_->header.timestamp;
  cycle_counter_perception_obj_ = perception_traffic_light_info_->header.cycle_counter;
  perception_traffic_lights_.clear();
  for (const auto &obj : perception_traffic_light_info_->objects) {
    if (obj.type != ObjectType::TRAFFIC_LIGHT) {
      continue;
    }
    if (obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_BACK &&
        obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_LEFT_BACK &&
        obj.attributes.traffic_light_direction != TrafficLightDirectionType::TLD_RIGHT_BACK) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light  direction_reason id:{}  direction:{}", obj.id,
                                     obj.attributes.traffic_light_direction);
      continue;
    }
    if (std::isnan(obj.position.x) || std::isnan(obj.position.y)) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light nan_reason obj id:{} is pos_nan_value. pos x:{} y:{}  z:{}", obj.id,
                                     obj.position.x, obj.position.y, obj.position.z);
      continue;
    }

    if (FilterTrafficLightShape(obj.attributes.traffic_light_shape)) {
      TRAFFIC_REC_LOG << fmt::format("invalid_need_remove_light  shape_reason id:{}  shape:{}", obj.id, obj.attributes.traffic_light_shape);
      continue;
    }

    TRAFFIC_REC_LOG << fmt::format(
        "normal_traffic_light_id:{}  position:[{:.2f},{:.2f},{:.2f}] shape:{}  color:{}  num:{} is_flash:{:d} direction:{}", obj.id,
        obj.position.x, obj.position.y, obj.position.z, magic_enum::enum_name(obj.attributes.traffic_light_shape),
        magic_enum::enum_name(obj.attributes.traffic_light_color), obj.attributes.traffic_light_num, obj.attributes.traffic_light_flashing,
        magic_enum::enum_name(obj.attributes.traffic_light_direction));

    perception_traffic_lights_.push_back(obj);
    perception_traffic_lights_.back().publish_time = perception_traffic_light_info_->header.timestamp;
  }
  traffic_lights_.header.timestamp      = perception_traffic_light_info_->header.timestamp;
  traffic_lights_.header.recv_timestamp = perception_traffic_light_info_->header.recv_timestamp;
  traffic_lights_.header.cycle_counter  = perception_traffic_light_info_->header.cycle_counter;
}

void TrafficLightMapping::FeedLatestFrame() {
  PercepTrfInfoPtr traffic_light_info;
  SensorDataManager::Instance()->GetLatestSensorFrame(traffic_light_info);
  // 无信息10帧后清除
  if (!traffic_light_info) {
    lost_count_++;
    if (lost_count_ > 10) {
      perception_traffic_light_objects_.clear();
    }
    return;
  }

  // 重复信息10帧后清除
  if (fabs(timestamp_ - traffic_light_info->header.timestamp) < 1e-3) {
    lost_count_++;
    if (lost_count_ > 10) {
      perception_traffic_light_objects_.clear();
    }
    return;
  }

  // TRAFFIC_LOG << fmt::format("traffic_light_info  publish_time:{:.3f}  measurement_time:{:.3f} counter:{}", traffic_light_info->header.recv_timestamp,
  //                            traffic_light_info->header.timestamp, traffic_light_info->header.cycle_counter);
  lost_count_ = 0;
  timestamp_  = traffic_light_info->header.timestamp;
  perception_traffic_light_objects_.clear();
  for (auto obj : traffic_light_info->objects) {
    if (obj.type != ObjectType::TRAFFIC_LIGHT) {
      continue;
    }
    if (perception_traffic_light_objects_.count(obj.id) > 0) {
      AWARN << "Ignore repeated id of vision input traffic light.";
      continue;
    }
    if (std::isnan(obj.position.x) || std::isnan(obj.position.x) || std::isnan(obj.position.x)) {
      FLOG_TLIGHT << fmt::format("obj id:{} is pos_nan_value. pos x:{} y:{}  z:{}", obj.id, obj.position.x, obj.position.y, obj.position.z);
      continue;
    }
    obj.publish_time = traffic_light_info->header.timestamp;

    perception_traffic_light_objects_[obj.id] = std::move(obj);
  }
}

void TrafficLightMapping::TimeSync() {
  E2EResultRawPtr  tin_e2e_latest{nullptr};
  PercepTrfInfoPtr perception_traffic_light_info_latest{nullptr};

  SensorDataManager::Instance()->GetLatestSensorFrame(tin_e2e_latest);
  SensorDataManager::Instance()->GetLatestSensorFrame(perception_traffic_light_info_latest);

  const std::size_t max_size = 3;
  if (tin_e2e_latest) {
    if (tin_e2e_deque_.size() >= max_size) {
      tin_e2e_deque_.pop_back();
    }
    tin_e2e_deque_.push_front(tin_e2e_latest);
    TRAFFIC_LOG << fmt::format("tin_latest_time:{:.3f}", tin_e2e_latest->header().measurement_timestamp());
  }
  if (perception_traffic_light_info_latest) {
    if (perception_traffic_light_info_deque_.size() >= max_size) {
      perception_traffic_light_info_deque_.pop_back();
    }
    perception_traffic_light_info_deque_.push_front(perception_traffic_light_info_latest);
    TRAFFIC_LOG << fmt::format("vis_latest_time:{:.3f}", perception_traffic_light_info_latest->header.timestamp);
  }

  constexpr double time_max = 0.03;
  for (const auto &vision_obj : perception_traffic_light_info_deque_) {
    for (const auto &tin_obj : tin_e2e_deque_) {
      if (!tin_obj->has_header() || !tin_obj->header().has_measurement_timestamp()) {
        continue;
      }
      double time_offset = vision_obj->header.timestamp - tin_obj->header().measurement_timestamp();
      if (std::fabs(time_offset) < time_max) {
        perception_traffic_light_info_ = vision_obj;
        tin_e2e_result_                = tin_obj;
        TRAFFIC_LOG << fmt::format("tin_time:{:.3f}  vis_time:{:.3f}", tin_e2e_result_->header().measurement_timestamp(),
                                   perception_traffic_light_info_->header.timestamp);
        return;
      }
    }
  }
}

}  // namespace cem::fusion::e2e
