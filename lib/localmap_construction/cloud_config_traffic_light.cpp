/**
 * @brief 
 * @param file_path 
 * @return std::optional<nlohmann::json> 
 * 
 * @author lingpeng (ling.peng3@byd.com)
 * @date 2025-08-22
 */
#include "modules/perception/env/src/lib/localmap_construction/cloud_config_traffic_light.h"
#include <resource_manager_interface.h>
#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <cyber/common/log.h>
#include <math/math.h>
#include <message/env_model/routing_map/routing_map.h>
#include "Eigen/src/Core/Matrix.h"
#include "fmt/ranges.h"
#include "magic_enum/magic_enum.hpp"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/perception/env/src/lib/base/sensor_data_manager.h"

namespace cem::fusion::claud_traffic {

ResourceManageGet::ResourceManageGet() {
  CLOUD_TRAFFIC_LOG << "init_resource.";
  InitResource();
  GetFileList();
  SubFileList();
}

std::shared_ptr<byd::common::resource_manager::ResourceManagerInterface> ResourceManageGet::InitResource() {
  resource_interface_ = std::make_shared<ResourceManagerInterface>("perception_env");
  if (resource_interface_ == nullptr) {
    CLOUD_TRAFFIC_LOG << "resource_interface_ init nullptr error";
    return nullptr;
  };
  return resource_interface_;
}

byd::common::resource_manager::MessagePtr ResourceManageGet::GetFileList() {
  if (!resource_interface_) {
    return nullptr;
  }

  auto GetInfo = [&](const byd::common::resource_manager::FileInfo &info) -> int {
    CLOUD_TRAFFIC_LOG << "recv file info:"
                      << "module_name:" << info.module_name << "version:" << info.version << "file:";
    for (const auto &file : info.file_list) {
      CLOUD_TRAFFIC_LOG << file;
    }
    {
      auto file_tmp = std::make_shared<FileInfo>(info);

      std::lock_guard<std::mutex> lock{data_lock_};
      message_ptr_latest_ = file_tmp;
    }
    return 0;
  };

  resource_interface_->RequestAsync(GetInfo);

  return nullptr;
}

void ResourceManageGet::SubFileList() {
  if (!resource_interface_) {
    return;
  }
  resource_interface_->Subscribe([this](const byd::common::resource_manager::FileInfo &info) -> int {
    CLOUD_TRAFFIC_LOG << "recv file info:"
                      << "module_name:" << info.module_name << "version:" << info.version << "file:";
    for (const auto &file : info.file_list) {
      CLOUD_TRAFFIC_LOG << file;
    }
    {
      auto file_tmp = std::make_shared<FileInfo>(info);

      std::lock_guard<std::mutex> lock{data_lock_};
      message_ptr_latest_ = file_tmp;
    }
    return 0;
  });
}

bool ResourceManageGet::IsVersionChange() const {
  MessagePtr current;
  MessagePtr prev;
  {
    std::lock_guard<std::mutex> lock(data_lock_);
    current = message_ptr_active_;
    prev    = message_ptr_prev_;
  }
  return !(current && prev && current->version == prev->version);
}

void ResourceManageGet::Update() {
  std::lock_guard<std::mutex> lock{data_lock_};
  message_ptr_prev_   = message_ptr_active_;
  message_ptr_active_ = message_ptr_latest_;
  CLOUD_TRAFFIC_LOG << "update counter_run_:" << counter_run_++
                    << fmt::format("  prev:{} version:{}  cur:{} version:{}", fmt::ptr(message_ptr_prev_.get()),
                                   message_ptr_prev_ ? message_ptr_prev_->version : "NULL", fmt::ptr(message_ptr_active_.get()),
                                   message_ptr_active_ ? message_ptr_active_->version : "NULL");
  if (message_ptr_active_) {
    for (const auto &file : message_ptr_active_->file_list) {
      CLOUD_TRAFFIC_LOG << "file_path:" << file;
    }
  }
}

std::optional<std::string> ResourceManageGet::IsFileExist(const std::string &file_name) const {
  std::lock_guard<std::mutex> lock{data_lock_};
  if (!message_ptr_active_) {
    return std::nullopt;
  }
  for (const std::string &file_path : message_ptr_active_->file_list) {
    if (file_path.find(file_name) != std::string::npos) {
      return file_path;
    }
  }
  return std::nullopt;
}

std::optional<TileCoord> LongLat2Tile(double longitude, double latitude, uint8_t zoom) {
  if (zoom > 31) {
    CLOUD_TRAFFIC_LOG << fmt::format("Zoom level exceeds maximum 31");
    return std::nullopt;
  }

  const uint32_t n_val  = 1U << zoom;
  const auto     x_tile = static_cast<uint32_t>((longitude + 180.0) / 360.0 * n_val);

  const double lat_rad = latitude * M_PI / 180.0;
  const double y_merc  = std::log(std::tan(lat_rad) + 1.0 / std::cos(lat_rad));
  const auto   y_tile  = static_cast<uint32_t>((1.0 - y_merc / M_PI) / 2.0 * n_val);

  return TileCoord{x_tile, y_tile, zoom};
}

std::optional<uint64_t> EncodeTileId(const TileCoord &tile) {
  constexpr uint64_t max_28bit = (1ULL << 28) - 1;  // 28 位最大值

  if (tile.z > 31) {
    CLOUD_TRAFFIC_LOG << fmt::format("Zoom exceeds 31");
    return std::nullopt;
  }
  if (tile.x > max_28bit || tile.y > max_28bit) {
    CLOUD_TRAFFIC_LOG << fmt::format("x or y exceeds 28-bit range");
    return std::nullopt;
  }

  return (static_cast<uint64_t>(tile.x) << 36) | (static_cast<uint64_t>(tile.y) << 8) | tile.z;
}

TileCoord DecodeTileId(uint64_t code) {
  const uint8_t  zoom  = code & 0xFFULL;
  const uint32_t y_val = (code >> 8) & 0x0FFFFFFFULL;   // 28 位掩码
  const uint32_t x_val = (code >> 36) & 0x0FFFFFFFULL;  // 28 位掩码

  return {x_val, y_val, zoom};
}

/**
 * @brief 根据中心点和半径，计算其地理边界框（Bounding Box）。
 * @param center 中心点的经纬度。
 * @param radius_km 半径（单位：公里）。
 * @return BoundingBox 结构体。
 */
BoundingBox CalculateSearchBBox(const CoordSystem &center, double radius_km) {
  // 这是一个简化的估算，但在大多数场景下足够精确
  // 地球上1度纬度大约等于 111.1 公里
  const double km_per_lat_degree = 111.1;

  // 计算纬度差值
  const double delta_lat = radius_km / km_per_lat_degree;

  // 地球上1度经度的距离取决于纬度, 在赤道处最长 (约111.32km)
  const double km_per_lon_degree_at_equator = 111.32;
  // 计算当前纬度下的1度经度对应的公里数
  const double km_per_lon_degree = km_per_lon_degree_at_equator * std::cos(center.lat * M_PI / 180.0);

  // 计算经度差值
  const double delta_lon = radius_km / km_per_lon_degree;

  return {center.lon - delta_lon, center.lat - delta_lat, center.lon + delta_lon, center.lat + delta_lat};
}

/**
 * @brief 查找一个点附近指定半径内，在三个指定层级下的所有瓦片ID组合。
 *
 * @param center_point 中心点的经纬度。
 * @param radius_km 搜索半径（公里）。
 * @param zooms 一个包含三个缩放级别的向量，例如 {6, 15, 18}。
 * @return 一个包含所有候选 EgoTileId 的 vector，结果已去重。
 */
std::vector<EgoTileId> FindNearbyEgoTileIDs(const CoordSystem &center_point, double radius_km, std::vector<uint8_t> zooms) {
  // 步骤一: 输入验证
  if (zooms.size() != 3) {
    // CLOUD_TRAFFIC_LOG << "Error: Zooms vector must contain exactly 3 levels.";
    return {};  // 返回空 vector
  }

  // 对 zoom 级别进行排序，确保 zooms[2] 是最高精度的级别
  std::sort(zooms.begin(), zooms.end());

  const uint8_t zoom_coarse = zooms[0];  // 最粗糙的级别, e.g., 6
  const uint8_t zoom_mid    = zooms[1];  // 中间级别, e.g., 15
  const uint8_t zoom_fine   = zooms[2];  // 最精细的级别, e.g., 18

  // 使用 std::set 存储结果，可以自动处理重复的 EgoTileId
  std::set<EgoTileId> unique_results;

  // 步骤二: 在最高精度级别 (zoom_fine) 上计算地理边界框
  BoundingBox bbox = CalculateSearchBBox(center_point, radius_km);

  // 步骤三: 将地理边界框转换为最高精度级别的瓦片坐标范围
  auto top_left_coord_opt     = LongLat2Tile(bbox.min_lon, bbox.max_lat, zoom_fine);
  auto bottom_right_coord_opt = LongLat2Tile(bbox.max_lon, bbox.min_lat, zoom_fine);

  if (!top_left_coord_opt || !bottom_right_coord_opt) {
    // CLOUD_TRAFFIC_LOG << "Error: Failed to convert bbox to tile coordinates at finest zoom.";
    return {};
  }

  TileCoord top_left     = *top_left_coord_opt;
  TileCoord bottom_right = *bottom_right_coord_opt;

  // 步骤四: 遍历最高精度级别的所有瓦片
  for (uint32_t x_fine = top_left.x; x_fine <= bottom_right.x; ++x_fine) {
    for (uint32_t y_fine = top_left.y; y_fine <= bottom_right.y; ++y_fine) {
      // 当前我们有了一个高精度瓦片坐标 (x_fine, y_fine, zoom_fine)

      // 步骤五: 计算此高精度瓦片对应的中、低精度父瓦片坐标
      // 核心原理: parent_coord = child_coord / (2 ^ (zoom_child - zoom_parent))

      // 计算中等精度瓦片坐标
      uint32_t zoom_diff_mid = zoom_fine - zoom_mid;
      uint32_t scale_mid     = 1U << zoom_diff_mid;  // 2 ^ zoom_diff
      uint32_t x_mid         = x_fine / scale_mid;
      uint32_t y_mid         = y_fine / scale_mid;

      // 计算粗糙精度瓦片坐标
      uint32_t zoom_diff_coarse = zoom_fine - zoom_coarse;
      uint32_t scale_coarse     = 1U << zoom_diff_coarse;  // 2 ^ zoom_diff
      uint32_t x_coarse         = x_fine / scale_coarse;
      uint32_t y_coarse         = y_fine / scale_coarse;

      // 步骤六: 将三个层级的坐标分别编码为 ID
      auto id_coarse_opt = EncodeTileId({x_coarse, y_coarse, zoom_coarse});
      auto id_mid_opt    = EncodeTileId({x_mid, y_mid, zoom_mid});
      auto id_fine_opt   = EncodeTileId({x_fine, y_fine, zoom_fine});

      // 确保所有ID都成功生成
      if (id_coarse_opt && id_mid_opt && id_fine_opt) {
        // 组合成一个 EgoTileId 并插入到 set 中
        unique_results.insert({*id_coarse_opt, *id_mid_opt, *id_fine_opt});
      }
    }
  }

  // 步骤七: 将 set 中的唯一结果转换为 vector 返回
  return {unique_results.begin(), unique_results.end()};
}

// 将度转换为弧度
double DegToRad(double degrees) {
  return degrees * M_PI / 180.0;
}

/**
 * @brief 将点从坐标系A转换到坐标系B
 * @param p_in_A 在坐标系A下的点
 * @param A 坐标系A的定义
 * @param B 坐标系B的定义
 * @return Point3D 在坐标系B下的点
 */
Point3D TransformPointCorA2CorB(const Point3D &p_in_a, const CoordSystem &system_a, const CoordSystem &system_b) {
  // --- 步骤 1: 将点P从A的局部坐标系转换到全局ENU(东-北-上)坐标系的位移 ---
  double heading_a_rad = DegToRad(system_a.heading);

  // **注意：下面的旋转公式已根据“左正右负”的约定进行了修改**
  // 局部坐标系: x向前, y向左
  // 全局坐标系: ENU (East-North-Up) 东-北-上
  // delta_east  = p_x * sin(h) - p_y * cos(h)
  // delta_north = p_x * cos(h) + p_y * sin(h)
  double delta_east_p  = p_in_a.x * sin(heading_a_rad) - p_in_a.y * cos(heading_a_rad);
  double delta_north_p = p_in_a.x * cos(heading_a_rad) + p_in_a.y * sin(heading_a_rad);

  // --- 步骤 2: 计算坐标系A原点相对于坐标系B原点的全局ENU位移 ---
  double avg_lat_rad   = DegToRad((system_a.lat + system_b.lat) / 2.0);
  double delta_lon_rad = DegToRad(system_a.lon - system_b.lon);
  double delta_lat_rad = DegToRad(system_a.lat - system_b.lat);

  double delta_east_ab  = delta_lon_rad * EARTH_RADIUS * cos(avg_lat_rad);
  double delta_north_ab = delta_lat_rad * EARTH_RADIUS;

  // --- 步骤 3: 计算P点相对于B原点的总的全局ENU位移 ---
  double total_delta_east  = delta_east_p + delta_east_ab;
  double total_delta_north = delta_north_p + delta_north_ab;

  // --- 步骤 4: 将总的全局位移转换到B的局部坐标系 ---
  double heading_b_rad = DegToRad(system_b.heading);

  // **注意：这是步骤1的逆运算，同样遵循“左正右负”约定**
  // local_x = delta_east * sin(h) + delta_north * cos(h)
  // local_y = -delta_east * cos(h) + delta_north * sin(h)
  Point3D p_in_b;
  p_in_b.x = total_delta_east * sin(heading_b_rad) + total_delta_north * cos(heading_b_rad);
  p_in_b.y = -total_delta_east * cos(heading_b_rad) + total_delta_north * sin(heading_b_rad);
  p_in_b.z = p_in_a.z + system_a.height - system_b.height;

  return p_in_b;
}

std::optional<nlohmann::json> LoadJsonConfig(const std::string &file_path) {
  std::ifstream file_stream(file_path);
  if (!file_stream.is_open()) {
    CLOUD_TRAFFIC_LOG << "Failed_to_open:" << file_path;
    return std::nullopt;
  }

  nlohmann::json config;
  try {
    file_stream >> config;
  } catch (const nlohmann::json::parse_error &e) {
    CLOUD_TRAFFIC_LOG << "JSON_parse_error:" << e.what();
    return std::nullopt;
  }

  // 使用配置数据
  // if (config.contains("timeout")) {
  //   int timeout = config["timeout"];
  //   std::cout << "Timeout: " << timeout << "ms\n";
  // }

  return config;
}

MapMatchResultPtr GetLatestMapMatchResult() {
  MapMatchResultPtr map_match_result_ptr(nullptr);
  SensorDataManager::Instance()->GetLatestSensorFrame(map_match_result_ptr);
  if (!map_match_result_ptr || !map_match_result_ptr->has_pose()) {
    return nullptr;
  }
  // CLOUD_TRAFFIC_LOG << map_match_result_ptr->DebugString();
  return map_match_result_ptr;
}

TrafficLightShapeType GetLightShape(uint light_type) {
  TrafficLightShapeType shape{TrafficLightShapeType::TLS_CIRCULAR};
  switch (light_type) {
    case 1:
      shape = TrafficLightShapeType::TLS_LEFT_ARROW;
      break;
    case 2:
      shape = TrafficLightShapeType::TLS_RIGHT_ARROW;
      break;
    case 3:
      shape = TrafficLightShapeType::TLS_TURN_ARROUND_ARROW;
      break;
    case 4:
      shape = TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW;
      break;

    default:
      break;
  }

  return shape;
}

PercepTrfInfoPtr GetLatestPerceptionInfo() {
  PercepTrfInfoPtr perception_traffic_light_info{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(perception_traffic_light_info);
  if (!perception_traffic_light_info) {
    return nullptr;
  }
  // CLOUD_TRAFFIC_LOG << perception_traffic_light_info->DebugString();
  return perception_traffic_light_info;
}

TurnType GetTurnType(uint32_t arrow_type) {
  TurnType turn_type{TurnType::OTHER_UNKNOWN};
  return turn_type;
}

template <typename T>
bool safe_get_json_value(const nlohmann::json &j, const std::string &key, T &out_value) {
  if (!j.contains(key)) {
    CLOUD_TRAFFIC_LOG << "JSON key '" << key << "' is missing.";
    return false;
  }
  try {
    j.at(key).get_to(out_value);
    return true;
  } catch (const nlohmann::json::exception &e) {
    CLOUD_TRAFFIC_LOG << "JSON type mismatch for key '" << key << "'. Error: " << e.what();
    return false;
  }
}

std::optional<nlohmann::json> GetJsonFileTileId(EgoTileId pos_tile_id, const ResourceManageGet &resource_manager) {
  static std::map<uint64_t, std::optional<nlohmann::json>> map_id_json;
  if (resource_manager.IsVersionChange()) {
    map_id_json.clear();
  }

  auto res = map_id_json.find(pos_tile_id.tile_id_0);
  if (res != map_id_json.end()) {
    return res->second;
  }

  std::string tile_id_0 = std::to_string(pos_tile_id.tile_id_0);
  // std::string file_path = "modules/perception/env/conf/CloudTrafficFiles/" + tile_id_0 + ".json";
  std::optional<std::string> file_path = resource_manager.IsFileExist(tile_id_0 + ".json");
  if (!file_path) {
    CLOUD_TRAFFIC_LOG << "no_find_" << tile_id_0 << ".json in cloud.";
    return std::nullopt;
  }

  std::optional<nlohmann::json> val = LoadJsonConfig(file_path.value());
  map_id_json.emplace(pos_tile_id.tile_id_0, val);
  if (!val) {
    CLOUD_TRAFFIC_LOG << "File doesn't exist or is not valid JSON. LoadJsonConfig should handle logging. path:" << *file_path;
  }
  return val;
}

std::optional<TrafficLightJunction> GetTrafficLightSpecialJunction(EgoTileId pos_tile_id, std::vector<uint8_t> zooms,
                                                                   const ResourceManageGet &resource_manager) {
  // --- Step 1: Load the root JSON file ---
  std::optional<nlohmann::json> val = GetJsonFileTileId(pos_tile_id, resource_manager);
  if (!val) {
    CLOUD_TRAFFIC_LOG << fmt::format("pos_tile_id_0:{}  json_is_not_exist.", pos_tile_id.tile_id_0);
    return std::nullopt;
  }

  // --- Step 2: Safely navigate the nested JSON structure ---
  const nlohmann::json *current_node = &(*val);

  std::string zoom_0 = "zoom" + std::to_string(zooms[0]);
  std::string zoom_1 = "zoom" + std::to_string(zooms[1]);
  std::string zoom_2 = "zoom" + std::to_string(zooms[2]);
  // Check for zoom_1
  if (!current_node->is_object() || !current_node->contains(zoom_1)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing '" << zoom_1 << "' level key.";
    return std::nullopt;
  }
  current_node = &(*current_node)[zoom_1];

  // Check for tile_id_1
  std::string tile_id_1 = std::to_string(pos_tile_id.tile_id_1);
  if (!current_node->is_object() || !current_node->contains(tile_id_1)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing tile_id '" << tile_id_1 << "' at zoom6 level.";
    return std::nullopt;
  }
  current_node = &(*current_node)[tile_id_1];

  // Check for zoom_2
  if (!current_node->is_object() || !current_node->contains(zoom_2)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing '" << zoom_2 << "' level key.";
    return std::nullopt;
  }
  current_node = &(*current_node)[zoom_2];

  // Check for tile_id_2
  std::string tile_id_2 = std::to_string(pos_tile_id.tile_id_2);
  if (!current_node->is_object() || !current_node->contains(tile_id_2)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing tile_id '" << tile_id_2 << "' at zoom15 level.";
    return std::nullopt;
  }
  current_node = &(*current_node)[tile_id_2];

  // Check for "configData" array and ensure it's not empty
  if (!current_node->is_object() || !current_node->contains("configData") || !(*current_node)["configData"].is_array() ||
      (*current_node)["configData"].empty()) {
    CLOUD_TRAFFIC_LOG << "JSON is missing or has an empty 'configData' array.";
    return std::nullopt;
  }
  // We are interested in the first element of the configData array
  const auto &junction_json = (*current_node)["configData"][0];
  if (!junction_json.is_object()) {
    CLOUD_TRAFFIC_LOG << "'configData' element is not a valid JSON object.";
    return std::nullopt;
  }

  // --- Step 3: Safely parse the junction data object ---
  TrafficLightJunction jun_t;
  jun_t.tail_id = pos_tile_id.tile_id_2;

  // Use the helper for required fields. If any fails, we abort.
  if (!safe_get_json_value(junction_json, "version", jun_t.version) || !safe_get_json_value(junction_json, "date", jun_t.date) ||
      !safe_get_json_value(junction_json, "action", jun_t.action) || !safe_get_json_value(junction_json, "strategy", jun_t.strategy) ||
      !safe_get_json_value(junction_json, "junction_type", jun_t.junction_type) ||
      !safe_get_json_value(junction_json, "distance_to_stop_line", jun_t.distance_to_stopline)) {
    CLOUD_TRAFFIC_LOG << "A required field is missing or has wrong type in junction config.";
    return std::nullopt;
  }

  // For boolean flags converted from numbers, check existence and type explicitly.
  uint32_t safe_takeover_val = 0;
  if (safe_get_json_value(junction_json, "safe_takeover", safe_takeover_val)) {
    jun_t.safe_takeover = (safe_takeover_val != 0U);
  }

  uint32_t flash_yellow_val = 0;
  if (safe_get_json_value(junction_json, "flashing_yellow_light", flash_yellow_val)) {
    jun_t.flash_yellow_light = (flash_yellow_val != 0U);
  }

  // Safely parse nested 'position' object
  if (junction_json.contains("position") && junction_json.at("position").is_object()) {
    const auto &pos_json = junction_json.at("position");
    if (!safe_get_json_value(pos_json, "lon", jun_t.position.lon) || !safe_get_json_value(pos_json, "lat", jun_t.position.lat) ||
        !safe_get_json_value(pos_json, "angle", jun_t.position.heading) ||
        !safe_get_json_value(pos_json, "lane_seq_id", jun_t.position_lane_seq_id)) {
      CLOUD_TRAFFIC_LOG << "A required field is missing or has wrong type in 'position' object.";
      return std::nullopt;
    }
  } else {
    CLOUD_TRAFFIC_LOG << "Required 'position' object is missing.";
    return std::nullopt;
  }

  // --- Step 4: Safely parse arrays ---

  // Safely parse 'traffic_lights' array
  if (junction_json.contains("traffic_lights") && junction_json.at("traffic_lights").is_array()) {
    for (const auto &val : junction_json.at("traffic_lights")) {
      if (!val.is_object()) {
        continue;  // Skip if element is not an object
      }

      TrafficLightSpecial trf_spe;
      if (!safe_get_json_value(val, "seq_id", trf_spe.seq_id)) {
        continue;
      }

      // Safely parse nested 'lane_seq_id' array
      if (val.contains("lane_seq_id") && val.at("lane_seq_id").is_array()) {
        // Use get_to for direct conversion with type safety
        val.at("lane_seq_id").get_to(trf_spe.lane_seq_id);
      }

      // Safely parse nested 'attribute' object
      if (val.contains("attribute") && val.at("attribute").is_object()) {
        const auto &attr = val.at("attribute");
        uint        light_type_int{0};
        if (safe_get_json_value(attr, "light_type", light_type_int)) {
          trf_spe.light_type = GetLightShape(light_type_int);
        }
        safe_get_json_value(attr, "light_name", trf_spe.light_name);

        if (attr.contains("position") && attr.at("position").is_object()) {
          const auto &light_pos = attr.at("position");
          safe_get_json_value(light_pos, "x", trf_spe.position.x);
          safe_get_json_value(light_pos, "y", trf_spe.position.y);
        }
      }
      jun_t.traffic_lights_spec.emplace_back(trf_spe);
    }
  }

  // Safely parse 'lane_infos' array
  if (junction_json.contains("lane_infos") && junction_json.at("lane_infos").is_array()) {
    for (const auto &val : junction_json.at("lane_infos")) {
      if (!val.is_object()) {
        continue;
      }

      TrafficLightLaneInfo trf_lane;
      if (!safe_get_json_value(val, "seq_id", trf_lane.seq_id)) {
        continue;
      }

      if (val.contains("traffic_light_seq_id") && val.at("traffic_light_seq_id").is_array()) {
        val.at("traffic_light_seq_id").get_to(trf_lane.traffic_light_seq_id);
      }

      if (val.contains("attribute") && val.at("attribute").is_object()) {
        const auto &attr = val.at("attribute");
        safe_get_json_value(attr, "arrow_type", trf_lane.arrow_type);
        safe_get_json_value(attr, "lane_name", trf_lane.lane_name);
      }
      jun_t.lane_info.emplace_back(trf_lane);
    }
  }

  CLOUD_TRAFFIC_LOG << "junc_info:" << jun_t.DebugString();
  return jun_t;
}

// std::optional<TrafficLightJunction> GetTrafficLightSpecialJunction(EgoTileId ego_tile_id) {
//   std::string tile_id_0 = std::to_string(ego_tile_id.tile_id_0);
//   std::string tile_id_1 = std::to_string(ego_tile_id.tile_id_1);
//   std::string tile_id_2 = std::to_string(ego_tile_id.tile_id_2);

//   std::optional<nlohmann::json> val = LoadJsonConfig("record_bag_docker/" + tile_id_0 + ".json");
//   if (!val) {
//     return std::nullopt;
//   }

//   const auto &junction_json = (*val)["zoom6"][tile_id_1]["zoom15"][tile_id_2]["configData"][0];
//   // CLOUD_TRAFFIC_LOG << junction_json.dump();

//   TrafficLightJunction jun_t;
//   jun_t.tail_id       = ego_tile_id.tile_id_2;
//   jun_t.version       = junction_json["version"];
//   jun_t.date          = junction_json["date"];
//   jun_t.action        = junction_json["action"];
//   jun_t.strategy      = junction_json["strategy"];
//   jun_t.safe_takeover = junction_json["safe_takeover"].get<uint32_t>() != 0U;
//   jun_t.junction_type = junction_json["junction_type"];

//   jun_t.distance_to_stopline = junction_json["distance_to_stop_line"];
//   jun_t.flash_yellow_light   = junction_json["flashing_yellow_light"].get<uint32_t>() != 0U;
//   jun_t.position.lon         = junction_json["position"]["lon"];
//   jun_t.position.lat         = junction_json["position"]["lat"];
//   jun_t.position.heading     = junction_json["position"]["angle"];
//   jun_t.position_lane_seq_id = junction_json["position"]["lane_seq_id"];
//   if (!junction_json["traffic_lights"].empty()) {
//     for (const auto &val : junction_json["traffic_lights"]) {
//       TrafficLightSpecial trf_spe;
//       trf_spe.seq_id = val["seq_id"];
//       for (const auto &val_0 : val["lane_seq_id"]) {
//         trf_spe.lane_seq_id.emplace_back(val_0);
//       }
//       trf_spe.light_type = GetLightShape(val["attribute"]["light_type"]);
//       trf_spe.position.x = val["attribute"]["position"]["x"];
//       trf_spe.position.y = val["attribute"]["position"]["y"];
//       trf_spe.light_name = val["attribute"]["light_name"];

//       jun_t.traffic_lights_spec.emplace_back(trf_spe);
//     }
//   }

//   if (!junction_json["lane_infos"].empty()) {
//     for (const auto &val : junction_json["lane_infos"]) {
//       TrafficLightLaneInfo trf_lane;
//       trf_lane.seq_id = val["seq_id"];
//       for (const auto &id_0 : val["traffic_light_seq_id"]) {
//         trf_lane.traffic_light_seq_id.emplace_back(id_0);
//       }
//       trf_lane.arrow_type = val["attribute"]["arrow_type"];
//       trf_lane.lane_name  = val["attribute"]["lane_name"];

//       jun_t.lane_info.emplace_back(trf_lane);
//     }
//   }

//   CLOUD_TRAFFIC_LOG << jun_t.DebugString();

//   return jun_t;
// }
bool IsValidTrafficLight(const TrfObjectInfo &rhs) {
  if (rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_PEDESTRIAN ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_BICYCLE ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_CLOSE_TO_TRAFFIC ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_SLOW_DOWN ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_HEART_SHAPE ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_NUMBER ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_PROCESS_BAR ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_BUS_ONLY_SHAPE ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_DOWN_ARROW ||
      rhs.attributes.traffic_light_shape == TrafficLightShapeType::TLS_RECTANGLE) {
    return false;
  }
  if (rhs.attributes.traffic_light_color == TrafficLightColorType::TLC_BLACK) {
    return false;
  }
  if (rhs.attributes.traffic_light_direction == TrafficLightDirectionType::TLD_LEFT ||
      rhs.attributes.traffic_light_direction == TrafficLightDirectionType::TLD_RIGHT ||
      rhs.attributes.traffic_light_direction == TrafficLightDirectionType::TLD_UNKNOWN ||
      rhs.attributes.traffic_light_direction == TrafficLightDirectionType::TLD_OTHER) {
    return false;
  }
  return true;
}

constexpr double lon_threshold = 10.0;
constexpr double lat_threshold = 8.0;

void FindMatchObjsOfObj(const std::vector<TrfObjectInfo> &objs_filtered, const TrafficLightSpecial &traffic_light,
                        std::vector<TrfObjectInfo> &ids, double lon_thre, double lat_thre) {
  for (const auto &obj_filt : objs_filtered) {
    if (std::fabs(obj_filt.position.x - traffic_light.position_ego.x) > lon_thre) {
      continue;
    }
    if (std::fabs(obj_filt.position.y - traffic_light.position_ego.y) > lat_thre) {
      continue;
    }
    auto res = std::find_if(ids.begin(), ids.end(), [&obj_filt](const TrfObjectInfo &rhs) { return rhs.id == obj_filt.id; });
    if (res == ids.end()) {
      CLOUD_TRAFFIC_LOG << fmt::format("find_insert_filted_id:{} traffic_id:{}", obj_filt.id, traffic_light.seq_id);
      ids.emplace_back(obj_filt);
    }
  }
}

void FillAroundLights(const std::vector<TrfObjectInfo> &objs_filterd, std::vector<TrfObjectInfo> &matched_objs, double lon_thre,
                      double lat_thre) {
  for (std::size_t idx = 0; idx < matched_objs.size(); idx++) {
    const auto &obj_match = matched_objs[idx];
    for (const auto &obj_filt : objs_filterd) {
      auto res =
          std::find_if(matched_objs.begin(), matched_objs.end(), [&obj_filt](const TrfObjectInfo &rhs) { return rhs.id == obj_filt.id; });
      if (res != matched_objs.end()) {
        continue;
      }
      if (std::fabs(obj_filt.position.x - obj_match.position.x) > lon_thre) {
        continue;
      }
      if (std::fabs(obj_filt.position.y - obj_match.position.y) > lat_thre) {
        continue;
      }
      CLOUD_TRAFFIC_LOG << fmt::format("fill_insert_filterd_id:{}  obj_match:{}", obj_filt.id, obj_match.id);
      matched_objs.emplace_back(obj_filt);
      idx = 0;
    }
  }
}

bool TrafficLightMatch(const std::vector<TrfObjectInfo> &obj_filterd, TrafficLightJunction &junc) {
  constexpr double delta = 2.0;
  for (std::size_t lon_idx = 1; lon_idx <= 5; lon_idx++) {
    for (std::size_t lat_idx = 1; lat_idx <= 4; lat_idx++) {
      double lon_thre = static_cast<double>(lon_idx) * delta;
      double lat_thre = static_cast<double>(lat_idx) * delta;
      for (const auto &traffic_light : junc.traffic_lights_spec) {
        FindMatchObjsOfObj(obj_filterd, traffic_light, junc.matched_obj_ids, lon_thre, lat_thre);
      }
      FillAroundLights(obj_filterd, junc.matched_obj_ids, lon_thre, lat_thre);
      if (junc.matched_obj_ids.size() == junc.traffic_lights_spec.size()) {
        std::sort(junc.matched_obj_ids.begin(), junc.matched_obj_ids.end(),
                  [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.position.y > rhs.position.y; });
        CLOUD_TRAFFIC_LOG << fmt::format("matched_lon:{:.2f} lat:{:.2f}", lon_thre, lat_thre);
        return true;
      }
    }
  }
  return false;
}

EgoState InfoEgoState(const BevMapInfoPtr &bev_map, const Point3D &jun_pos, Eigen::Isometry3d *Tbw_ptr) {
  EgoState res;
  if (!bev_map) {
    return res;
  }
  const BevLaneInfo *ego_bev_lane{nullptr};

  // uint64_t ego_section_id = bev_map->route.navi_start.section_id;
  for (const auto &lane : bev_map->lane_infos) {
    if (lane.position == 0) {
      ego_bev_lane = &lane;
    }
  }
  if (ego_bev_lane == nullptr) {
    return res;
  }
  std::vector<Vec2d> points;
  const auto        &lane_points = ego_bev_lane->line_points;
  Eigen::Vector3d    line_point_st;
  for (const auto &point_curr : lane_points) {
    line_point_st.x() = point_curr.x;
    line_point_st.y() = point_curr.y;
    if (Tbw_ptr != nullptr) {
      line_point_st = *Tbw_ptr * line_point_st;
    }
    // line_point_st     = line_point_st;
    points.emplace_back(line_point_st.x(), line_point_st.y());
  }
  CLOUD_TRAFFIC_LOG << fmt::format("bev_ego_lane_id:{} map_left_lane_id:{} right_lane_id:{}", ego_bev_lane->id, ego_bev_lane->left_lane_id,
                                   ego_bev_lane->right_lane_id);
  if (points.empty()) {
    CLOUD_TRAFFIC_LOG << "lane_point_empty.";
    return res;
  }
  MultiLineSegment multi_seg;
  multi_seg.Init(points);
  double proj_s{0.0};
  double proj_l{0.0};
  double min_dis{0.0};
  bool   proj_val = multi_seg.GetProjection(Vec2d{jun_pos.x, jun_pos.y}, &proj_s, &proj_l, &min_dis);
  if (proj_val && proj_s < multi_seg.Length() && proj_s > 0) {
    res.ego_dir = EgoState::Dir::LeftStart;
    res.index   = static_cast<int>(std::fabs(proj_l) / 3.5);
  }
  CLOUD_TRAFFIC_LOG << fmt::format("proj_val:{:d} proj_s:{:.2f}  proj_l:{:.2f}  min_dis:{:.2f}  res_dir:{} res_index:{} len:{:.2f}",
                                   proj_val, proj_s, proj_l, min_dis, magic_enum::enum_name(res.ego_dir), res.index, multi_seg.Length());
  return res;
}

void LaneClassTrafficLight(const EgoState &ego_state, const TrafficLightJunction &junc, TrafficLights &traffic_light) {
  std::optional<TrfObjectInfo> target_obj = std::nullopt;

  bool is_valid_right = false;
  for (const auto &lane : junc.lane_info) {
    if (lane.seq_id != ego_state.index) {
      continue;
    }
    for (const auto &traffic : junc.traffic_lights_spec) {
      for (const auto ass_id : traffic.lane_seq_id) {
        if (ass_id != lane.seq_id) {
          continue;
        }
        TrafficLight res;
        res.traffic_obj_info       = std::make_shared<TrfObjectInfo>(*traffic.matched_obj_id);
        res.traffic_light_num      = res.traffic_obj_info->attributes.traffic_light_num;
        res.color                  = res.traffic_obj_info->attributes.traffic_light_color;
        res.traffic_light_flashing = res.traffic_obj_info->attributes.traffic_light_flashing;
        CLOUD_TRAFFIC_LOG << fmt::format("find_lane_seq_id:{} lane_arrow:{} traffic_id:{} ", lane.seq_id, lane.arrow_type,
                                         traffic.matched_obj_id->id);
        if ((lane.arrow_type & 0b0001U) != 0U) {
          traffic_light.straight           = res;
          traffic_light.straight.is_valid  = true;
          traffic_light.straight.turn_type = TurnType::NO_TURN;
        }
        if ((lane.arrow_type & 0b0010U) != 0U) {
          traffic_light.left           = res;
          traffic_light.left.is_valid  = true;
          traffic_light.left.turn_type = TurnType::LEFT_TURN;
        }
        if ((lane.arrow_type & 0b0100U) != 0U) {
          is_valid_right                = true;
          traffic_light.right           = res;
          traffic_light.right.is_valid  = true;
          traffic_light.right.turn_type = TurnType::RIGHT_TURN;
        }
        if ((lane.arrow_type & 0b1000U) != 0U) {
          traffic_light.u_turn           = res;
          traffic_light.u_turn.is_valid  = true;
          traffic_light.u_turn.turn_type = TurnType::U_TURN;
        }
      }
    }
  }
  if (!is_valid_right) {
    traffic_light.right.is_valid       = true;
    traffic_light.right.color          = TrafficLightColorType::TLC_GREEN;
    traffic_light.right.traffic_reason = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
    traffic_light.right.turn_type      = TurnType::RIGHT_TURN;
  }
}

/**
 * @brief 将经纬度坐标转换为瓦片坐标。
 * @param lon 经度 (-180 到 180)。
 * @param lat 纬度 (-85.0511 到 85.0511)。
 * @param zoom 缩放级别 (0 到 31)。
 * @return 包含瓦片坐标的 std::optional，如果输入无效则为空。
 */
std::optional<uint64_t> GetTileId(CoordSystem ego_pos, uint8_t zoom) {
  const auto mid = LongLat2Tile(ego_pos.lon, ego_pos.lat, zoom);
  if (mid) {
    const auto tile_id = EncodeTileId(*mid);
    if (tile_id) {
      const auto de_id = DecodeTileId(*tile_id);
      CLOUD_TRAFFIC_LOG << fmt::format("tile_id:{} decode_mid:{}  {}  {}", *tile_id, de_id.x, de_id.y, de_id.z)
                        << fmt::format("  mid:{}  {}   zoom:{}", mid->x, mid->y, mid->z);
      return tile_id;
    }
  }
  return std::nullopt;
}

std::vector<uint8_t> GetZoomList(const ResourceManageGet &resource_manager) {
  std::vector<uint8_t> zooms;

  auto file_path = resource_manager.IsFileExist("configer_param.json");
  if (!file_path) {
    CLOUD_TRAFFIC_LOG << "no_para_json_in_cloud";
    return zooms;
  }

  std::optional<nlohmann::json> val = LoadJsonConfig(file_path.value());
  if (!val) {
    CLOUD_TRAFFIC_LOG << "Config file not found";
    return zooms;
  }

  if (!val->contains("zoom_list")) {
    CLOUD_TRAFFIC_LOG << "Missing 'zoom_list' field";
    return zooms;
  }

  auto &zoom_list = (*val)["zoom_list"];
  if (!zoom_list.is_array()) {
    CLOUD_TRAFFIC_LOG << "'zoom_list' is not an array";
    return zooms;
  }

  if (zoom_list.size() != 3) {
    CLOUD_TRAFFIC_LOG << fmt::format("Invalid zoom_list size: {}", zoom_list.size());
    return zooms;
  }

  for (const auto &item : zoom_list) {
    if (!item.is_number()) {
      CLOUD_TRAFFIC_LOG << "Non-integer value in zoom_list";
      return zooms;
    }
    const int value = item.get<int>();
    if (value < 0 || value > 255) {
      CLOUD_TRAFFIC_LOG << fmt::format("Value {} out of uint8_t range", value);
      return zooms;
    }
    zooms.push_back(static_cast<uint8_t>(value));
  }
  CLOUD_TRAFFIC_LOG << fmt::format("zoom_list:{}", zooms);
  if (zooms.size() != 3) {
    zooms.clear();
    return zooms;
  }
  return zooms;
}

std::optional<EgoTileId> GetEgoTileId(CoordSystem ego_pos, std::vector<uint8_t> zooms) {
  if (zooms.empty()) {
    return std::nullopt;
  }

  auto tile_0 = GetTileId(ego_pos, zooms[0]);
  auto tile_1 = GetTileId(ego_pos, zooms[1]);
  auto tile_2 = GetTileId(ego_pos, zooms[2]);

  if (tile_0 && tile_1 && tile_2) {
    return EgoTileId{*tile_0, *tile_1, *tile_2};
  }

  return std::nullopt;
}

void ChangeLightArrow(const TrafficLightSpecial &light_cloud, const PercepTrfInfoPtr &perception_traffic_light_info) {
  if (!light_cloud.matched_obj_id) {
    return;
  }
  auto light_val = std::find_if(perception_traffic_light_info->objects.begin(), perception_traffic_light_info->objects.end(),
                                [light_cloud](const TrfObjectInfo &rhs) { return rhs.id == light_cloud.matched_obj_id->id; });
  if (light_val == perception_traffic_light_info->objects.end()) {
    return;
  }

  // Bit definitions:
  const unsigned int UP_BIT          = 0b00001U;  // bit 0: 直行
  const unsigned int LEFT_BIT        = 0b00010U;  // bit 1: 左转
  const unsigned int RIGHT_BIT       = 0b00100U;  // bit 2: 右转
  const unsigned int LEFT_UTURN_BIT  = 0b01000U;  // bit 3: 左掉头
  const unsigned int RIGHT_UTURN_BIT = 0b10000U;  // bit 4: 右掉头

  const auto &type = light_cloud.light_type;
  CLOUD_TRAFFIC_LOG << fmt::format("change_perception_id:{}  cloud_arrow:{:b}", light_val->id, type);

  // 优先检查组合箭头类型
  // 注意：TLS_LEFT_TURN_ARROUND_ARROW (左与转弯箭头) 对应 左转+左掉头
  if (type == (LEFT_BIT | LEFT_UTURN_BIT)) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW;
  } else if (type == (UP_BIT | RIGHT_BIT)) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_UP_RIGHT_ARROW;
  } else if (type == (UP_BIT | LEFT_BIT)) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_UP_LEFT_ARROW;
  }
  // 检查单个箭头类型
  else if (type == UP_BIT) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_UP_ARROW;
  } else if (type == LEFT_BIT) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_LEFT_ARROW;
  } else if (type == RIGHT_BIT) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_RIGHT_ARROW;
  }
  // 检查掉头类型 (左掉头, 右掉头, 或两者都有)
  else if (type == LEFT_UTURN_BIT || type == RIGHT_UTURN_BIT || type == (LEFT_UTURN_BIT | RIGHT_UTURN_BIT)) {
    light_val->attributes.traffic_light_shape = TrafficLightShapeType::TLS_TURN_ARROUND_ARROW;
  }
}

void SetRedTrafficLight(TrafficLight &traffic_light, TrafficLightColorType color_type, const TrafficLightSpecial &traffic_light_file) {
  traffic_light.is_valid         = true;
  traffic_light.color            = color_type;
  traffic_light.traffic_reason   = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
  traffic_light.traffic_obj_info = std::make_shared<cem::message::sensor::TrfObjectInfo>();

  traffic_light.traffic_obj_info->id         = 998;
  traffic_light.traffic_obj_info->position.x = traffic_light_file.position_ego.x;
  traffic_light.traffic_obj_info->position.y = traffic_light_file.position_ego.y;
}

std::optional<TrafficLights> GetMatchTraffciLight(const CoordSystem &ego_pos, const EgoTileId &ego_tile_id,
                                                  const std::vector<uint8_t> &zooms, const std::vector<TrfObjectInfo> &obj_filterd,
                                                  const BevMapInfoPtr &bev_map, PercepTrfInfoPtr perception_traffic_light_info,
                                                  const ResourceManageGet &resource_manager, Eigen::Isometry3d *Tbw_ptr) {
  std::optional<TrafficLightJunction> junc{std::nullopt};

  static std::map<EgoTileId, std::optional<TrafficLightJunction>> map_tileid_junction;
  if (resource_manager.IsVersionChange()) {
    map_tileid_junction.clear();
  }

  auto res = map_tileid_junction.find(ego_tile_id);
  if (res != map_tileid_junction.end()) {
    junc = res->second;
  } else {
    junc = GetTrafficLightSpecialJunction(ego_tile_id, zooms, resource_manager);
    map_tileid_junction.emplace(ego_tile_id, junc);
  }
  if (!junc || junc->junction_type == 0) {
    CLOUD_TRAFFIC_LOG << "not_find_junc.";
    return std::nullopt;
  }

  junc->matched_obj_ids.clear();
  for (auto &traf : junc->traffic_lights_spec) {
    traf.matched_obj_id = std::nullopt;
  }
  junc->position_ego = Point3D{};

  junc->position_ego = TransformPointCorA2CorB({0.0, 0.0, 0.0}, junc->position, ego_pos);
  CLOUD_TRAFFIC_LOG << fmt::format("ego_localization:{:.4f},{:.4f}  heading:{:.4f} junc_pos:{:.2f},{:.2f}", ego_pos.lon, ego_pos.lat,
                                   ego_pos.heading, junc->position_ego.x, junc->position_ego.y);
  for (auto &traffic_light : junc->traffic_lights_spec) {
    traffic_light.position_ego = TransformPointCorA2CorB(traffic_light.position, junc->position, ego_pos);
    CLOUD_TRAFFIC_LOG << fmt::format(
        "junc_localization:{:.4f},{:.4f} heading:{:.4f} obj_id:{} raw_light_pos:{:.2f},{:.2f}  new_pos:{:.2f},{:.2f}", junc->position.lon,
        junc->position.lat, junc->position.heading, traffic_light.seq_id, traffic_light.position.x, traffic_light.position.y,
        traffic_light.position_ego.x, traffic_light.position_ego.y);
  }

  bool suc = TrafficLightMatch(obj_filterd, *junc);

  std::vector<uint32_t> obj_ids;
  for (std::size_t idx = 0; idx < junc->matched_obj_ids.size(); idx++) {
    const auto &obj = junc->matched_obj_ids[idx];
    if (idx >= 0 && idx < junc->traffic_lights_spec.size()) {
      junc->traffic_lights_spec.at(idx).matched_obj_id = std::make_optional(obj);
    }
    obj_ids.emplace_back(obj.id);
  }

  TrafficLights traffic_lights;
  traffic_lights.junction_cloud_type  = junc->junction_type;
  traffic_lights.distance_to_stopline = junc->distance_to_stopline;
  traffic_lights.junction_tail_id     = junc->tail_id;

  EgoState ego_state = InfoEgoState(bev_map, junc->position_ego, Tbw_ptr);
  CLOUD_TRAFFIC_LOG << fmt::format("cloud_matched:{:d} junc_ids:{}", suc, obj_ids);
  if (ego_state.ego_dir == EgoState::Dir::Unknown) {
    CLOUD_TRAFFIC_LOG << fmt::format("ego_state_infer_failed.");
    return std::nullopt;
  }
  if (ego_state.index > junc->lane_info.size() - 1) {
    CLOUD_TRAFFIC_LOG << fmt::format("ego_num_exceed_lane_num.  ego_idx:{}  lane_num:{}", ego_state.index, junc->lane_info.size());
    return std::nullopt;
  }

  if (!suc) {
    if (!junc->traffic_lights_spec.empty()) {
      SetRedTrafficLight(traffic_lights.left, TrafficLightColorType::TLC_NOT_MATCH, junc->traffic_lights_spec.front());
      SetRedTrafficLight(traffic_lights.u_turn, TrafficLightColorType::TLC_NOT_MATCH, junc->traffic_lights_spec.front());
      SetRedTrafficLight(traffic_lights.straight, TrafficLightColorType::TLC_NOT_MATCH, junc->traffic_lights_spec.front());
      SetRedTrafficLight(traffic_lights.right, TrafficLightColorType::TLC_NOT_MATCH, junc->traffic_lights_spec.front());
      CLOUD_TRAFFIC_LOG << fmt::format("find_ego_not_find_traffic_light.");
      return traffic_lights;
    }
    return std::nullopt;
  }
  switch (junc->junction_type) {
    case 1:
    case 2: {
      LaneClassTrafficLight(ego_state, *junc, traffic_lights);
      break;
    }
    case 3:
      for (const auto &light_cloud : junc->traffic_lights_spec) {
        if (light_cloud.matched_obj_id) {
          ChangeLightArrow(light_cloud, perception_traffic_light_info);
        }
      }
      CLOUD_TRAFFIC_LOG << fmt::format("get_traffic_lights_in_case3. change_light_arrow.");
      return std::nullopt;
    default:
      break;
  }

  return traffic_lights;
}

std::optional<TrafficLights> GetCloudTrafficLights(const BevMapInfoPtr &bev_map, std::string &cloud_debug_info,
                                                   Eigen::Isometry3d *Tbw_ptr) {
  CLOUD_TRAFFIC_LOG << "cloud_traffic_light______________________begin_______";
  static ResourceManageGet resource_manage_get;
  resource_manage_get.Update();
  for (const auto &info : resource_manage_get.GetFileStr()) {
    cloud_debug_info += info + ";";
  }

  MapMatchResultPtr map_match_result_ptr = GetLatestMapMatchResult();
  if (!map_match_result_ptr) {
    CLOUD_TRAFFIC_LOG << "no_find_map_match.";
    return std::nullopt;
  }
  PercepTrfInfoPtr perception_traffic_light_info = GetLatestPerceptionInfo();
  if (!perception_traffic_light_info) {
    CLOUD_TRAFFIC_LOG << "perception_traffic_light_info is nullptr.";
    return std::nullopt;
  }
  if (perception_traffic_light_info->objects.empty()) {
    CLOUD_TRAFFIC_LOG << "no_find_traffic_lights.";
    // return std::nullopt;
  }
  std::vector<TrfObjectInfo> obj_filterd;
  for (const auto &obj : perception_traffic_light_info->objects) {
    CLOUD_TRAFFIC_LOG << fmt::format("obj_id:{}  pos:{:.2f},{:.2f} direction:{} shape:{} color:{}", obj.id, obj.position.x, obj.position.y,
                                     magic_enum::enum_name(obj.attributes.traffic_light_direction),
                                     magic_enum::enum_name(obj.attributes.traffic_light_shape),
                                     magic_enum::enum_name(obj.attributes.traffic_light_color));
    if (IsValidTrafficLight(obj)) {
      obj_filterd.emplace_back(obj);
    }
  }
  CLOUD_TRAFFIC_LOG << fmt::format("Counter_map_match:{}  traffic_light:{} bev:{}", map_match_result_ptr->header().sequence_num(),
                                   perception_traffic_light_info->header.cycle_counter, bev_map ? bev_map->header.cycle_counter : 0);

  CoordSystem ego_pos;
  ego_pos.lon     = map_match_result_ptr->pose().position().lon() * 180.0 / M_PI;
  ego_pos.lat     = map_match_result_ptr->pose().position().lat() * 180.0 / M_PI;
  ego_pos.heading = map_match_result_ptr->pose().heading();

  std::vector<uint8_t> zooms = GetZoomList(resource_manage_get);
  if (zooms.empty()) {
    CLOUD_TRAFFIC_LOG << "no_find_config_para_json.";
    return std::nullopt;
  }
  CLOUD_TRAFFIC_LOG << fmt::format("find_zoom:{}", zooms);

  std::optional<EgoTileId> ego_tile_id = GetEgoTileId(ego_pos, zooms);
  if (!ego_tile_id) {
    return std::nullopt;
  }

  constexpr double       ego_radius     = 0.3;  // km
  std::vector<EgoTileId> ego_tile_ids_0 = FindNearbyEgoTileIDs(ego_pos, ego_radius, zooms);
  for (const auto &ego_tile_id : ego_tile_ids_0) {
    CLOUD_TRAFFIC_LOG << fmt::format("Find_Junction_In_Tile  ego_around_tile_0:{} tile_1:{}  tile_2:{}", ego_tile_id.tile_id_0,
                                     ego_tile_id.tile_id_1, ego_tile_id.tile_id_2);
    auto traffic_res = GetMatchTraffciLight(ego_pos, ego_tile_id, zooms, obj_filterd, bev_map, perception_traffic_light_info,
                                            resource_manage_get, Tbw_ptr);
    if (traffic_res) {
      return traffic_res;
    }
  }
  return std::nullopt;
}

}  // namespace cem::fusion::claud_traffic