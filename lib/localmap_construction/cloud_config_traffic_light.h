/**
 * @copyright Copyright (c) 2025 BYD Corporation. All rights reserved.
 * @copyright
 * @copyright BYD Corporation and its licenses retain all intellectual property
 * @copyright and proprietary rights in and to this software, related documentation
 * @copyright and any modifications thereto. Any use, reproduction, disclosure or
 * @copyright distribution of this software and related documentation without an express
 * @copyright license agreement from BYD Corporation is strictly prohibited.
 * 
 * 
 * @file cloud_config_traffic_light.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-22
 */
#pragma once
#include <cstddef>
#define PLATFORM_C_CLOUD false

#include <sys/types.h>
#include <atomic>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <message/common/geometry.h>
#include <message/env_model/routing_map/routing_map.h>
#include <nlohmann/json.hpp>
#include "lib/message/sensor/vision/tsrmobject.h"
#include "magic_enum/magic_enum.hpp"
#include "nlohmann/json.hpp"
#include "modules/msg/orin_msgs/routing_map.pb.h"
#if (PLATFORM_C_CLOUD)
#include "modules/common/resource_manager_interface/resource_manager_interface.h"
#else
namespace byd::common::resource_manager {
struct FileInfo {
  std::string              module_name;
  std::string              version;
  std::vector<std::string> file_list;
};
using MessagePtr      = std::shared_ptr<FileInfo>;
using StorageCallback = std::function<int(const FileInfo &info)>;
class ResourceManagerInterface {
 public:
  ResourceManagerInterface(const std::string &module_name) {}
  bool       Subscribe(const StorageCallback &cb) { return false; }
  MessagePtr RequestSync(int timeout, int32_t retry_times = 0) { return nullptr; }
  bool       RequestAsync(const StorageCallback &cb, int timeout = 2000, int32_t retry_times = -1) { return false; }
  void       Release() {}
};
}  // namespace byd::common::resource_manager
#endif
#include "modules/perception/env/src/lib/common/log_custom.h"
#include "modules/perception/env/src/lib/message/internal_message.h"

namespace cem::fusion {
using TurnType              = cem::message::env_model::TurnType;
using Point3D               = cem::message::env_model::Point3D;
using TrfObjectInfo         = cem::message::sensor::TrfObjectInfo;
using TrafficLightShapeType = cem::message::sensor::TrafficLightShapeType;
using TrafficLightColorType = cem::message::sensor::TrafficLightColorType;
struct TrafficLight {
  TrafficLight(TurnType turn_type_t = TurnType::OTHER_UNKNOWN) : turn_type(turn_type_t) {}

  std::shared_ptr<cem::message::sensor::TrfObjectInfo> traffic_obj_info{nullptr};
  cem::message::sensor::TrafficLightColorType          color{cem::message::sensor::TrafficLightColorType::TLC_NONE_LIGHT};
  LightStatus                                          light_status;
  uint32_t                                             traffic_light_num{1000};
  bool                                                 traffic_light_flashing{false};
  bool                                                 is_valid{false};
  bool                                                 is_confirmed{false};
  double                                               publish_time{0.0};
  uint64_t                                             perception_seq_num{0};
  std::string                                          traffic_source{0};
  uint32_t                                             stay_prev_counter{0};
  uint32_t                                             invalid_counter{0};
  TurnType                                             turn_type{TurnType::OTHER_UNKNOWN};

  byd::msg::orin::routing_map::LaneInfo::TrafficSetReason traffic_reason{byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE};
  double                                                  yellow_flashing_start_time = 0.0;
};
struct TrafficLights {
  cem::message::common::Header header;

  bool                 ld_perception_is_matched{false};
  TrafficLight         u_turn{TurnType::U_TURN};
  TrafficLight         left{TurnType::LEFT_TURN};
  TrafficLight         straight{TurnType::NO_TURN};
  TrafficLight         right{TurnType::RIGHT_TURN};
  uint                 junction_cloud_type{0};
  double               distance_to_stopline{0.0};
  bool                 traffic_match_cloud_file{false};
  uint64_t             junction_tail_id{0};
  std::vector<Point2D> stop_line;  //E2E-25158新增
  TrafficLight         GetLightState(const TurnType &turn_type) const {
    TrafficLight rev;
    switch (turn_type) {
      case message::env_model::TurnType::NO_TURN: {
        return straight.is_valid ? straight : rev;
      }
      case message::env_model::TurnType::LEFT_TURN: {
        return left.is_valid ? left : rev;
      }
      case message::env_model::TurnType::RIGHT_TURN: {
        return right.is_valid ? right : rev;
      }
      case message::env_model::TurnType::U_TURN: {
        return u_turn.is_valid ? u_turn : rev;
      }
      default: {
        return rev;
      }
    }
  }
};
}  // namespace cem::fusion

namespace fmt {
template <>
struct formatter<cem::fusion::TrafficLight> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::fusion::TrafficLight &rhs, FormatContext &ctx) const {
    return format_to(ctx.out(),
                     "valid:{:d} color:{:-<16}  num:{:-^10} flash:{:d} perception_seq:{:-^10}  obj_id:{:-^10}  reason:{} "
                     "stay_prev_counter:{} invalid_counter:{:d}",
                     rhs.is_valid, magic_enum::enum_name(rhs.color), rhs.traffic_light_num, rhs.traffic_light_flashing,
                     rhs.perception_seq_num, rhs.traffic_obj_info ? rhs.traffic_obj_info->id : 0,
                     magic_enum::enum_name(rhs.traffic_reason).substr(26), rhs.stay_prev_counter, rhs.invalid_counter);
  }
};
}  // namespace fmt

namespace cem::fusion::claud_traffic {
#if defined(CLOUD_LIGHT_LOG) && (CLOUD_LIGHT_LOG)
#define CLOUD_TRAFFIC_LOG LOG_LP
#else
#define CLOUD_TRAFFIC_LOG NULL_LOG
#endif

const double EARTH_RADIUS = 6371000.0;  // 使用WGS84平均半径

// 定义瓦片坐标结构体
struct TileCoord {
  uint32_t x;  // 使用无符号类型确保非负
  uint32_t y;
  uint8_t  z;  // 缩放级别通常范围 0-31，8 位足够

  TileCoord(uint32_t x_val = 0, uint32_t y_val = 0, uint8_t z_val = 0) : x(x_val), y(y_val), z(z_val) {}
};

struct EgoTileId {
  uint64_t tile_id_0{0};
  uint64_t tile_id_1{0};
  uint64_t tile_id_2{0};
  bool     operator<(const EgoTileId &other) const {
    return std::tie(tile_id_0, tile_id_1, tile_id_2) < std::tie(other.tile_id_0, other.tile_id_1, other.tile_id_2);
  }
};

struct BoundingBox {
  double min_lon;
  double min_lat;
  double max_lon;
  double max_lat;
};

struct CoordSystem {
  double lon{0.0};
  double lat{0.0};
  double height{0.0};
  double heading{0.0};
};

struct EgoState {
  enum class Dir { LeftStart, Unknown };
  Dir      ego_dir{Dir::Unknown};
  uint     index{0};
  TurnType turn_type{TurnType::OTHER_UNKNOWN};
};

struct TrafficLightSpecial {
  uint64_t              seq_id{0};
  TrafficLightShapeType light_type{TrafficLightShapeType::TLS_CIRCULAR};
  std::string           light_name;

  std::vector<uint32_t>        lane_seq_id;
  Point3D                      position;
  Point3D                      position_ego;  //相对于自车的距离
  std::optional<TrfObjectInfo> matched_obj_id{std::nullopt};
  double                       lon_factor = 1.0;
  double                       lat_factor = 1.0;
  [[nodiscard]] std::string    DebugString(int indent = 0) const {
    std::stringstream ss_res;
    std::string       indent_str(indent, ' ');

    ss_res << indent_str << "TrafficLightSpecial:\n";
    ss_res << indent_str << "  seq_id: " << seq_id << "\n";
    ss_res << indent_str << "  light_type: " << static_cast<uint>(light_type) << "\n";
    ss_res << indent_str << "  light_name: " << light_name << "\n";

    ss_res << indent_str << "  lane_seq_id: [";
    for (const auto &id : lane_seq_id) {
      ss_res << id << ", ";
    }
    if (!lane_seq_id.empty()) {
      ss_res.seekp(-2, ss_res.cur);
    }
    ss_res << "]\n";

    ss_res << indent_str << "  position: (" << std::fixed << std::setprecision(6) << position.x << ", " << position.y << ", " << position.z
           << ")";

    return ss_res.str();
  }
};

struct TrafficLightLaneInfo {
  uint32_t              seq_id{0};
  std::vector<uint32_t> traffic_light_seq_id;
  uint32_t              arrow_type;
  std::string           lane_name;

  [[nodiscard]] std::string DebugString(int indent = 0) const {
    std::stringstream ss_res;
    std::string       indent_str(indent, ' ');

    ss_res << indent_str << "TrafficLightLaneInfo:\n";
    ss_res << indent_str << "  seq_id: " << seq_id << "\n";

    ss_res << indent_str << "  traffic_light_seq_id: [";
    for (const auto &id : traffic_light_seq_id) {
      ss_res << id << ", ";
    }
    if (!traffic_light_seq_id.empty()) {
      ss_res.seekp(-2, ss_res.cur);
    }
    ss_res << "]\n";

    ss_res << indent_str << "  arrow_type: " << arrow_type << "\n";
    ss_res << indent_str << "  lane_name: " << lane_name << "\n";

    return ss_res.str();
  }
};

using byd::common::resource_manager::FileInfo;
using byd::common::resource_manager::MessagePtr;
using byd::common::resource_manager::ResourceManagerInterface;

class ResourceManageGet {
 public:
  ResourceManageGet();

  void Update();

  [[nodiscard]] bool IsVersionChange() const;

  [[nodiscard]] std::optional<std::string> IsFileExist(const std::string &file_name) const;

  std::vector<std::string> GetFileStr() const {
    std::lock_guard<std::mutex> lock_g(data_lock_);
    if (!message_ptr_active_) {
      return {};
    }
    return message_ptr_active_->file_list;
  }

 private:
  std::shared_ptr<ResourceManagerInterface> InitResource();

  MessagePtr GetFileList();
  void       SubFileList();

  MessagePtr message_ptr_active_;
  MessagePtr message_ptr_latest_;
  MessagePtr message_ptr_prev_;
  uint64_t   counter_run_{0};

  mutable std::mutex data_lock_;

  std::shared_ptr<ResourceManagerInterface> resource_interface_;
};

struct TrafficLightJunction {
  uint64_t    tail_id{0};
  std::string version;
  std::string date;

  bool     flash_yellow_light{false};
  uint32_t junction_type{0};
  uint32_t strategy{0};
  double   distance_to_stopline{0.0};
  bool     safe_takeover{false};

  std::vector<TrafficLightSpecial>  traffic_lights_spec;
  std::vector<TrafficLightLaneInfo> lane_info;

  uint                       action{0};
  CoordSystem                position;
  Point3D                    position_ego;
  uint32_t                   position_lane_seq_id{0};
  std::vector<Point2D>       stop_line;  //透传停止线需要
  std::vector<TrfObjectInfo> matched_obj_ids;

  [[nodiscard]] std::string DebugString(int indent = 0) const {
    std::stringstream ss_res;
    std::string       indent_str(indent, ' ');

    ss_res << indent_str << "TrafficLightJunction:\n";
    ss_res << indent_str << "  tail_id: " << tail_id << "\n";
    ss_res << indent_str << "  version: " << version << "\n";
    ss_res << indent_str << "  date: " << date << "\n";
    ss_res << indent_str << "  flash_yellow_light: " << std::boolalpha << flash_yellow_light << "\n";
    ss_res << indent_str << "  junction_type: " << junction_type << "\n";
    ss_res << indent_str << "  stratege: " << strategy << "\n";
    ss_res << indent_str << "  distance_to_stopline: " << std::fixed << std::setprecision(2) << distance_to_stopline << "\n";
    ss_res << indent_str << "  safe_takeover: " << safe_takeover << "\n";
    ss_res << indent_str << "  action: " << static_cast<int>(action) << "\n";
    ss_res << indent_str << "  position: (lon:" << std::fixed << std::setprecision(6) << position.lon << ", lat:" << position.lat
           << ", angle:" << position.heading << ") lane_seq_id:" << position_lane_seq_id;

    ss_res << indent_str << "  traffic_lights_spec:\n";
    for (const auto &special : traffic_lights_spec) {
      ss_res << special.DebugString(indent + 4) << "\n";
    }

    ss_res << indent_str << "  lane_info:\n";
    for (const auto &info : lane_info) {
      ss_res << info.DebugString(indent + 4) << "\n";
    }

    return ss_res.str();
  }
};

std::optional<nlohmann::json> LoadJsonConfig(const std::string &file_path);

std::optional<TrafficLightJunction> GetTrafficLightSpecialJunction();

MapMatchResultPtr GetLatestMapMatchResult();
PercepTrfInfoPtr  GetLatestPerceptionInfo();

void FindMatchObjsOfObj(const std::vector<TrfObjectInfo> &objs, TrafficLightSpecial &traffic_light);

std::optional<TrafficLights> GetCloudTrafficLights(const BevMapInfoPtr &bev_map, std::string &cloud_debug_info,
                                                   Eigen::Isometry3d *Tbw_ptr = nullptr);

}  // namespace cem::fusion::claud_traffic