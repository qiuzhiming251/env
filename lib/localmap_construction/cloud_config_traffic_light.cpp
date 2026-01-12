/**
 * @brief 
 * @param file_path 
 * @return std::optional<nlohmann::json> 
 * 
 * @author lingpeng (ling.peng3@byd.com)
 * @date 2025-08-22
 */
#include "modules/perception/env/src/lib/localmap_construction/cloud_config_traffic_light.h"

#if (PLATFORM_C_CLOUD)
#include <resource_manager_interface.h>
#endif

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

//是否已经收到云端配置信号
static std::atomic<bool> cloud_signal_received{false};

//接收到云端信号后，后续所有读取函数只会尝试从云端读取配置文件。
inline void SetCloudSignalReceived(bool flag) {
  cloud_signal_received.store(flag, std::memory_order_relaxed);
}
inline bool IsCloudSignalReceived() {
  return cloud_signal_received.load(std::memory_order_relaxed);
}

static std::map<std::string, std::optional<nlohmann::json>> local_zoom_cache;
static std::mutex                                           local_zoom_mtx;

// 清除所有本地zoom配置缓存
static void ClearLocalZoomCache() {
  std::lock_guard<std::mutex> lk(local_zoom_mtx);
  local_zoom_cache.clear();
  CLOUD_TRAFFIC_LOG << "[LOCAL] zoom cache cleared.";
}

static std::map<uint64_t, std::optional<nlohmann::json>> local_tile_cache;
static std::mutex                                        local_tile_mtx;

// 清除所有本地路口JSON缓存
static void ClearAllLocalTileCache() {
  std::lock_guard<std::mutex> lk(local_tile_mtx);
  local_tile_cache.clear();
  CLOUD_TRAFFIC_LOG << "[LOCAL] tile cache cleared";
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
  if (zoom > 31) {  //zoom超过31(2^31)会导致 x、y 超出32位整数范围，直接返回nullopt
    CLOUD_TRAFFIC_LOG << fmt::format("Zoom level exceeds maximum 31");
    return std::nullopt;
  }

  const uint32_t n_val = 1U << zoom;  //n_val = 2^zoom, 表示该zoom下每行/列的瓦片数量
  const auto x_tile = static_cast<uint32_t>((longitude + 180.0) / 360.0 * n_val);  //经度平移再归一化，最后乘以n_val得到列号

  const double lat_rad = latitude * M_PI / 180.0;                                //从度转换为弧度
  const double y_merc  = std::log(std::tan(lat_rad) + 1.0 / std::cos(lat_rad));  //墨卡托投影
  const auto y_tile = static_cast<uint32_t>((1.0 - y_merc / M_PI) / 2.0 * n_val);  //把投影值映射到[0，1]，最后乘以n_val得到行号

  return TileCoord{x_tile, y_tile, zoom};  //返回完整的瓦片坐标
}

std::optional<uint64_t> EncodeTileId(const TileCoord &tile) {
  constexpr uint64_t max_28bit = (1ULL << 28) - 1;  // 28 位最大值，2^28‑1

  if (tile.z > 31) {  //zoom同样不能超过31
    CLOUD_TRAFFIC_LOG << fmt::format("Zoom exceeds 31");
    return std::nullopt;
  }
  if (tile.x > max_28bit || tile.y > max_28bit) {  //x，y坐标不能超过28位最大值
    CLOUD_TRAFFIC_LOG << fmt::format("x or y exceeds 28-bit range");
    return std::nullopt;
  }
  // 64 位编码方式：
  //   bits 63~36 : x (28 位)
  //   bits 35~8  : y (28 位)
  //   bits 7~0   : zoom (8 位)
  return (static_cast<uint64_t>(tile.x) << 36) |  // x 左移 36 位
         (static_cast<uint64_t>(tile.y) << 8) |   // y 左移 8 位
         tile.z;                                  // zoom 放在最低 8 位
}

TileCoord DecodeTileId(uint64_t code) {                 //把 EncodeTileId 得到的 64 bit 整数恢复成 (x, y, zoom)
  const uint8_t  zoom  = code & 0xFFULL;                //低8位，0xFF = 1111 1111
  const uint32_t y_val = (code >> 8) & 0x0FFFFFFFULL;   // 28 位掩码，如果是0xFFFFFFFF则是32位，数有几个1就完了
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
  const double delta_lat = radius_km / km_per_lat_degree;  //表示需要在纬度方向上移动多少度

  // 地球上1度经度的距离取决于纬度, 在赤道处最长 (约111.32km)
  const double km_per_lon_degree_at_equator = 111.32;
  // 计算当前纬度下的1度经度对应的公里数
  const double km_per_lon_degree = km_per_lon_degree_at_equator * std::cos(center.lat * M_PI / 180.0);  //1度经度对应的长度随纬度余弦衰减

  // 计算经度差值
  const double delta_lon = radius_km / km_per_lon_degree;  //表示需要在经度方向上移动多少度

  return {center.lon - delta_lon, center.lat - delta_lat, center.lon + delta_lon, center.lat + delta_lat};  //返回一个经纬度矩形
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
  BoundingBox bbox = CalculateSearchBBox(center_point, radius_km);  //返回一个经纬度矩形，最精细级别怎么理解？和步骤三有关

  // 左下角 (min_lon, min_lat) = (center.lon - Δlon, center.lat - Δlat)
  // 右上角 (max_lon, max_lat) = (center.lon + Δlon, center.lat + Δlat)

  // 步骤三: 将地理边界框转换为最高精度级别的瓦片坐标范围
  auto top_left_coord_opt     = LongLat2Tile(bbox.min_lon, bbox.max_lat, zoom_fine);  //返回瓦片的左上坐标
  auto bottom_right_coord_opt = LongLat2Tile(bbox.max_lon, bbox.min_lat, zoom_fine);  //返回瓦片的右下坐标

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
      uint32_t scale_mid     = 1U << zoom_diff_mid;  // 2 ^ zoom_diff_mid
      uint32_t x_mid         = x_fine / scale_mid;
      uint32_t y_mid         = y_fine / scale_mid;

      // 计算粗糙精度瓦片坐标
      uint32_t zoom_diff_coarse = zoom_fine - zoom_coarse;
      uint32_t scale_coarse     = 1U << zoom_diff_coarse;  // 2 ^ zoom_diff_coarse
      uint32_t x_coarse         = x_fine / scale_coarse;
      uint32_t y_coarse = y_fine / scale_coarse;  //x,y并不是具体地理坐标，而是对应的瓦片网格序号，在这里会变小说明不同层级瓦片数量不同

      // 步骤六: 将三个层级的坐标分别编码为64位编码ID
      auto id_coarse_opt = EncodeTileId({x_coarse, y_coarse, zoom_coarse});
      auto id_mid_opt    = EncodeTileId({x_mid, y_mid, zoom_mid});
      auto id_fine_opt   = EncodeTileId({x_fine, y_fine, zoom_fine});

      // 确保所有ID都成功生成
      if (id_coarse_opt && id_mid_opt && id_fine_opt) {
        // 组合成一个 EgoTileId 并插入到 set 中
        unique_results.insert({*id_coarse_opt, *id_mid_opt, *id_fine_opt});  //EgoTileId是自车在三级瓦片系统中的唯一标识
      }
    }
  }

  // 步骤七: 将 set 中的唯一结果转换为 vector 返回
  return {unique_results.begin(), unique_results.end()};  //去重
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
  // double delta_east_p  = p_in_a.x * sin(heading_a_rad) - p_in_a.y * cos(heading_a_rad);
  // double delta_north_p = p_in_a.x * cos(heading_a_rad) + p_in_a.y * sin(heading_a_rad);
  double delta_east_p  = p_in_a.x * std::sin(heading_a_rad) - p_in_a.y * std::cos(heading_a_rad);
  double delta_north_p = p_in_a.x * std::cos(heading_a_rad) + p_in_a.y * std::sin(heading_a_rad);

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

TrafficLightShapeType GetLightShape(uint light_type) {  //待定
  // Bit definitions:
  const unsigned int UP_BIT          = 0b00001U;  // bit 0: 直行
  const unsigned int LEFT_BIT        = 0b00010U;  // bit 1: 左转
  const unsigned int RIGHT_BIT       = 0b00100U;  // bit 2: 右转
  const unsigned int LEFT_UTURN_BIT  = 0b01000U;  // bit 3: 左掉头
  const unsigned int RIGHT_UTURN_BIT = 0b10000U;  // bit 4: 右掉头
  if (light_type == (LEFT_BIT | LEFT_UTURN_BIT)) {
    return TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW;  // 左转 + 左掉头
  }
  if (light_type == (UP_BIT | RIGHT_BIT)) {
    return TrafficLightShapeType::TLS_UP_RIGHT_ARROW;  // 直行 + 右转
  }
  if (light_type == (UP_BIT | LEFT_BIT)) {
    return TrafficLightShapeType::TLS_UP_LEFT_ARROW;  // 直行 + 左转
  }
  // 单灯
  if (light_type == UP_BIT)
    return TrafficLightShapeType::TLS_UP_ARROW;
  if (light_type == LEFT_BIT)
    return TrafficLightShapeType::TLS_LEFT_ARROW;
  if (light_type == RIGHT_BIT)
    return TrafficLightShapeType::TLS_RIGHT_ARROW;

  if (light_type == LEFT_UTURN_BIT || light_type == RIGHT_UTURN_BIT || light_type == (LEFT_UTURN_BIT | RIGHT_UTURN_BIT))
    return TrafficLightShapeType::TLS_TURN_ARROUND_ARROW;  // 双掉头

  // 未匹配到任何已知形状,视为圆形
  return TrafficLightShapeType::TLS_CIRCULAR;
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

std::optional<nlohmann::json> GetJsonFileTileIdCloud(EgoTileId pos_tile_id, const ResourceManageGet &resource_manager) {
  static std::map<uint64_t, std::optional<nlohmann::json>> map_id_json;
  if (resource_manager.IsVersionChange()) {
    map_id_json.clear();
  }

  auto res = map_id_json.find(pos_tile_id.tile_id_0);  //先在缓存中检查是否读取过该文件
  if (res != map_id_json.end()) {
    return res->second;
  }

  std::string                tile_id_0 = std::to_string(pos_tile_id.tile_id_0);
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

// 读取本地路口JSON
std::optional<nlohmann::json> GetJsonFileTileIdLocal(EgoTileId pos_tile_id) {
  {
    std::lock_guard<std::mutex> lk(local_tile_mtx);
    auto                        it = local_tile_cache.find(pos_tile_id.tile_id_0);
    if (it != local_tile_cache.end())
      return it->second;
  }

  std::string                   tile_id_0 = std::to_string(pos_tile_id.tile_id_0);
  std::string                   file_path = "modules/perception/env/conf/CloudTrafficFiles/" + tile_id_0 + ".json";
  std::optional<nlohmann::json> cfg       = LoadJsonConfig(file_path);

  {
    std::lock_guard<std::mutex> lk(local_tile_mtx);
    local_tile_cache.emplace(pos_tile_id.tile_id_0, cfg);
  }
  if (!cfg)
    CLOUD_TRAFFIC_LOG << "[LOCAL] Failed to parse " << file_path;
  return cfg;
}

std::optional<nlohmann::json> GetJsonFileTileId(const EgoTileId &pos_tile_id, const ResourceManageGet &resource_manager) {
  // 云端
  if (IsCloudSignalReceived()) {
    if (auto cfg = GetJsonFileTileIdCloud(pos_tile_id, resource_manager); cfg) {
      CLOUD_TRAFFIC_LOG << "[CLOUD] Load config from CLOUD for tile_id_0 =" << pos_tile_id.tile_id_0;
      return cfg;
    }
    CLOUD_TRAFFIC_LOG << "[CLOUD] Cloud config not available while cloud signal is active.";
    return std::nullopt;
  }
  // 本地
  if (auto cfg = GetJsonFileTileIdLocal(pos_tile_id); cfg) {
    CLOUD_TRAFFIC_LOG << "Load config from LOCAL for tile_id_0 =" << pos_tile_id.tile_id_0;
    return cfg;
  }
  CLOUD_TRAFFIC_LOG << "Unable to load any config for tile_id_0 =" << pos_tile_id.tile_id_0;
  return std::nullopt;
}

std::optional<TrafficLightJunction> GetTrafficLightSpecialJunction(EgoTileId pos_tile_id, std::vector<uint8_t> zooms,
                                                                   const ResourceManageGet &resource_manager) {
  // --- Step 1: Load the root JSON file ---
  std::optional<nlohmann::json> val = GetJsonFileTileId(pos_tile_id, resource_manager);  //todo:区分本地和云端获取数据
  if (!val) {
    CLOUD_TRAFFIC_LOG << fmt::format("pos_tile_id_0:{}  json_is_not_exist.", pos_tile_id.tile_id_0);
    return std::nullopt;
  }

  // --- Step 2: Safely navigate the nested JSON structure ---
  const nlohmann::json *current_node = &(*val);  //把 JSON 指针指向根对象，准备逐层向下遍历，要“先检查-再下沉”的方式一步一步走进子对象

  std::string zoom_0 = "zoom" + std::to_string(zooms[0]);
  std::string zoom_1 = "zoom" + std::to_string(zooms[1]);
  std::string zoom_2 = "zoom" + std::to_string(zooms[2]);
  // Check for zoom_1
  if (!current_node->is_object() || !current_node->contains(zoom_1)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing '" << zoom_1 << "' level key.";
    return std::nullopt;
  }
  current_node = &(*current_node)[zoom_1];  // 现在指向根下的 zoom_1 对象

  // Check for tile_id_1
  std::string tile_id_1 = std::to_string(pos_tile_id.tile_id_1);
  if (!current_node->is_object() || !current_node->contains(tile_id_1)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing tile_id '" << tile_id_1 << "' at zoom6 level.";
    return std::nullopt;
  }
  current_node = &(*current_node)[tile_id_1];  // 指向 zoom_1 → tile_id_1

  // Check for zoom_2
  if (!current_node->is_object() || !current_node->contains(zoom_2)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing '" << zoom_2 << "' level key.";
    return std::nullopt;
  }
  current_node = &(*current_node)[zoom_2];  // 现在指向 zoom_2 对象

  // Check for tile_id_2
  std::string tile_id_2 = std::to_string(pos_tile_id.tile_id_2);
  if (!current_node->is_object() || !current_node->contains(tile_id_2)) {
    CLOUD_TRAFFIC_LOG << "JSON is missing tile_id '" << tile_id_2 << "' at zoom15 level.";
    return std::nullopt;
  }
  current_node = &(*current_node)[tile_id_2];  // 指向 zoom_2 → tile_id_2

  // Check for "configData" array and ensure it's not empty
  if (!current_node->is_object() || !current_node->contains("configData") || !(*current_node)["configData"].is_array() ||
      (*current_node)["configData"].empty()) {
    CLOUD_TRAFFIC_LOG << "JSON is missing or has an empty 'configData' array.";
    return std::nullopt;
  }
  // We are interested in the first element of the configData array
  const auto &junction_json = (*current_node)["configData"][0];  // 只取tile_id_2的第0项，即第一个交叉口
  if (!junction_json.is_object()) {
    CLOUD_TRAFFIC_LOG << "'configData' element is not a valid JSON object.";
    return std::nullopt;
  }

  // --- Step 3: Safely parse the junction data object ---
  TrafficLightJunction jun_t;
  jun_t.tail_id = pos_tile_id.tile_id_2;  //保存精细瓦片的id

  // Use the helper for required fields. If any fails, we abort.
  if (!safe_get_json_value(junction_json, "version", jun_t.version) || !safe_get_json_value(junction_json, "date", jun_t.date) ||
      !safe_get_json_value(junction_json, "action", jun_t.action) || !safe_get_json_value(junction_json, "strategy", jun_t.strategy) ||
      !safe_get_json_value(junction_json, "junction_type", jun_t.junction_type) ||
      !safe_get_json_value(junction_json, "distance_to_stop_line", jun_t.distance_to_stopline)) {
    CLOUD_TRAFFIC_LOG << "A required field is missing or has wrong type in junction config.";
    return std::nullopt;
  }

  // For boolean flags converted from numbers, check existence and type explicitly.
  uint32_t safe_takeover_val = 0;  // JSON 中的布尔值是 0/1 整数，需要先读成 uint32_t 再转成 bool
  if (safe_get_json_value(junction_json, "safe_takeover", safe_takeover_val)) {  //安全接管
    jun_t.safe_takeover = (safe_takeover_val != 0U);                             //0U表示无符号的0
  }

  uint32_t flash_yellow_val = 0;
  if (safe_get_json_value(junction_json, "flashing_yellow_light", flash_yellow_val)) {  //黄灯闪烁
    jun_t.flash_yellow_light = (flash_yellow_val != 0U);
  }
  //E2E-25158新增：读取配置文件给出的停止线
  constexpr double lane_width = 3.5;  //车道宽度
  if (junction_json.contains("stop_line") && junction_json.at("stop_line").is_array()) {
    const auto &stop_line_arr = junction_json.at("stop_line");
    for (const auto &spt_json : stop_line_arr) {
      if (!spt_json.is_object()) {
        CLOUD_TRAFFIC_LOG << "'stop_line' element is not an object, skipping.";
        continue;
      }
      Point2D spt_l;
      auto    has_x_points = safe_get_json_value(spt_json, "x", spt_l.x);
      auto    has_y_points = safe_get_json_value(spt_json, "y", spt_l.y);
      if (has_x_points && has_y_points) {
        jun_t.stop_line.emplace_back(std::move(spt_l));  //在上面两个都是true的时候才push
      }
    }
    if (jun_t.stop_line.size() >= 2) {
      jun_t.stop_line[0].y +=
          lane_width;  //左端点向左平移, 因为停止线给的自车坐标是从最左车道算的，但当前自车车道不一定，所以现在暴力往左和往右平移
      jun_t.stop_line[1].y -= lane_width;  //右端点向右平移, todo:根据当前自车车道数平移停止线
    } else {
      CLOUD_TRAFFIC_LOG << fmt::format("stop_line size is {}, expected at least 2 points.", jun_t.stop_line.size());
    }
  } else {
    CLOUD_TRAFFIC_LOG << "JSON does not contain a valid 'stop_line' array.";
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
      if (!safe_get_json_value(val, "seq_id", trf_spe.seq_id)) {  //红绿灯唯一的编号，从左到右，序号从0开始
        continue;
      }

      // Safely parse nested 'lane_seq_id' array
      if (val.contains("lane_seq_id") && val.at("lane_seq_id").is_array()) {
        // Use get_to for direct conversion with type safety
        val.at("lane_seq_id").get_to(trf_spe.lane_seq_id);  //灯绑定的车道数组
      }

      // Safely parse nested 'attribute' object
      if (val.contains("attribute") && val.at("attribute").is_object()) {
        const auto &attr = val.at("attribute");  //at是读取json里键的方式，在需要确保键一定存在且不希望意外创建新键时使用
        uint light_type_int{0};
        if (safe_get_json_value(attr, "light_type", light_type_int)) {
          trf_spe.light_type = GetLightShape(light_type_int);  //light_type_int是原始给出的，公交车道和潮汐车道灯也是在这里？
        }
        safe_get_json_value(attr, "light_name", trf_spe.light_name);

        if (attr.contains("position") && attr.at("position").is_object()) {  //该红绿灯的坐标
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
    for (const auto &val : junction_json.at("lane_infos")) {  //遍历 lane_infos 数组中的每一条记录
      if (!val.is_object()) {
        continue;
      }

      TrafficLightLaneInfo trf_lane;
      if (!safe_get_json_value(val, "seq_id", trf_lane.seq_id)) {  //同向车道从左到右，序号也是从0开始
        continue;                                                  //*必须字段
      }

      if (val.contains("traffic_light_seq_id") && val.at("traffic_light_seq_id").is_array()) {
        val.at("traffic_light_seq_id").get_to(trf_lane.traffic_light_seq_id);  // 关联的红绿灯序号数组
      }

      if (val.contains("attribute") && val.at("attribute").is_object()) {
        const auto &attr = val.at("attribute");
        safe_get_json_value(attr, "arrow_type", trf_lane.arrow_type);  //车道转向
        safe_get_json_value(attr, "lane_name", trf_lane.lane_name);    //车道名称
      }
      jun_t.lane_info.emplace_back(trf_lane);
    }
  }

  // CLOUD_TRAFFIC_LOG << "junc_info:" << jun_t.DebugString(); //这里打印出来的light_type是TrafficLightShapeType定义的
  return jun_t;
}

static void LogTrafficLights(const TrafficLights &tl) {
  struct Entry {
    TurnType            tt;
    const TrafficLight *light;
  };
  const std::array<Entry, 4> entries = {{{TurnType::NO_TURN, &tl.straight},
                                         {TurnType::LEFT_TURN, &tl.left},
                                         {TurnType::RIGHT_TURN, &tl.right},
                                         {TurnType::U_TURN, &tl.u_turn}}};

  CLOUD_TRAFFIC_LOG << "===== TrafficLights Summary (junction_type = " << tl.junction_cloud_type << ") =====";

  for (const auto &e : entries) {
    const TrafficLight &l        = *e.light;
    std::string         obj_info = "NONE";
    if (l.traffic_obj_info) {
      const auto &info = *l.traffic_obj_info;
      obj_info         = fmt::format("id={}, pos=({:.2f},{:.2f}), num={}, flashing={}", info.id, info.position.x, info.position.y,
                                     l.traffic_light_num, l.traffic_light_flashing);
    }

    CLOUD_TRAFFIC_LOG << fmt::format("is_valid={}, color={}, light_status={}, turn_type={}, obj_info={}", l.is_valid,
                                     magic_enum::enum_name(l.color), magic_enum::enum_name(l.light_status),
                                     magic_enum::enum_name(l.turn_type), obj_info);
  }

  CLOUD_TRAFFIC_LOG << "===== End of TrafficLights Summary =====";
}

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

static void FilterPerceptionByDistance(TrafficLightJunction &junc) {
  const std::size_t keep_light_size = junc.traffic_lights_spec.size() + 1;

  struct PerceptionToCloud {
    double      dist;
    std::size_t idx;
  };
  std::vector<PerceptionToCloud> light_dists;
  light_dists.reserve(junc.matched_obj_ids.size());

  for (std::size_t pi = 0; pi < junc.matched_obj_ids.size(); ++pi) {
    const auto &perc     = junc.matched_obj_ids[pi];
    double      min_dist = std::numeric_limits<double>::max();
    for (const auto &cloud : junc.traffic_lights_spec) {
      double dx = perc.position.x - cloud.position_ego.x;
      double dy = perc.position.y - cloud.position_ego.y;
      double d  = std::hypot(dx, dy);
      if (d < min_dist) {
        min_dist = d;
      }
    }
    light_dists.push_back({min_dist, pi});
  }

  std::sort(light_dists.begin(), light_dists.end(), [](const PerceptionToCloud &a, const PerceptionToCloud &b) { return a.dist < b.dist; });
  const std::size_t real_keep_num = std::min(light_dists.size(), keep_light_size);

  std::vector<TrfObjectInfo> filtered;
  filtered.reserve(real_keep_num);
  for (std::size_t k = 0; k < real_keep_num; ++k) {
    filtered.emplace_back(junc.matched_obj_ids[light_dists[k].idx]);
  }

  junc.matched_obj_ids.swap(filtered);
}
//辅助工具函数：利用KM算法构造代价矩阵求出云端灯和感知灯的全局最小代价匹配
static std::vector<std::pair<std::size_t, std::size_t>> KM_MinCost(const std::vector<std::vector<double>> &init_cm) {
  const std::size_t                rows = init_cm.size();
  const std::size_t                cols = init_cm[0].size();
  const std::size_t                N    = std::max(rows, cols);  //需要的矩阵尺寸, 必须是方阵
  const double                     INF  = 1e8;                   //用于后面的过滤
  const double                     BIG  = 1e6;                   //用于补充矩阵，不用INF是避免编译报错
  std::vector<std::vector<double>> temp_cm(N,
                                           std::vector<double>(N, BIG));  //构造一个临时矩阵，把矩阵补成N×N的方阵，空的地方用一个极大值补上

  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      temp_cm[i][j] = init_cm[i][j];
    }
  }
  const std::size_t   tmp_array = N + 1;                     //准备 N+1 长度的数组，防止越界
  std::vector<double> u(tmp_array, 0.0), v(tmp_array, 0.0);  //u、v分别是行/列的潜在值
  std::vector<int>    match(tmp_array, -1), way(tmp_array, -1);

  for (std::size_t row = 1; row <= N; ++row) {
    match[0]                    = static_cast<int>(row);  // 把待匹配的行放到第0列
    std::size_t         cur_col = 0;                      // 当前搜索的列, 初始为0
    std::vector<double> minv(tmp_array, INF);             //最小值
    std::vector<char>   used(tmp_array, false);

    do {  //算法循环主体
      used[cur_col]       = true;
      std::size_t cur_row = static_cast<std::size_t>(match[cur_col]);  // 已匹配的行
      double      delta   = INF;                                       //本轮循环需要的调整值
      std::size_t nxt_col = 0;

      for (std::size_t col = 1; col <= N; ++col) {  //遍历所有的列，尝试寻找最小的代价
        if (used[col])
          continue;
        double cur = temp_cm[cur_row - 1][col - 1] - u[cur_row] - v[col];
        if (cur < minv[col]) {
          minv[col] = cur;
          way[col]  = static_cast<int>(cur_col);
        }
        if (minv[col] < delta) {
          delta   = minv[col];
          nxt_col = col;
        }
      }

      // 调整潜在函数，使得至少有一条新的零等价边出现
      for (std::size_t col = 0; col <= N; ++col) {
        if (used[col]) {
          u[match[col]] += delta;
          v[col] -= delta;
        } else {
          minv[col] -= delta;
        }
      }
      cur_col = nxt_col;
    } while (match[cur_col] != -1);  //当循环到一个未匹配的列时，退出循环

    do {  //翻转匹配
      std::size_t nxt_col = static_cast<std::size_t>(way[cur_col]);
      match[cur_col]      = match[nxt_col];
      cur_col             = nxt_col;
    } while (cur_col);
  }

  std::vector<std::pair<std::size_t, std::size_t>> res;  //最后的匹配结果
  for (std::size_t col = 1; col <= cols; ++col) {
    if (match[col] != -1 && static_cast<std::size_t>(match[col]) <= rows) {
      std::size_t cloud_idx     = static_cast<std::size_t>(match[col] - 1);
      std::size_t precetion_idx = col - 1;
      if (temp_cm[cloud_idx][precetion_idx] < INF / 2)  // 过滤掉填充的 INF
        res.emplace_back(cloud_idx, precetion_idx);
    }
  }
  return res;
}

/*
  MatchCloudsToPerception--寻找全局最优解
  1️. 计算每一对云端灯和感知灯的完整代价
  2️. 把代价矩阵交给KM算法求出最小总代价的匹配集合
  3️. 把匹配结果写回 cloud.matched_obj_id
*/
void MatchCloudsToPerception(std::vector<TrfObjectInfo> &matched_objs, TrafficLightJunction &junc, const EgoState &ego_state) {
  if (junc.traffic_lights_spec.empty() || matched_objs.empty()) {
    CLOUD_TRAFFIC_LOG << "cloud or perception light is empty, skip.";
    return;
  }
  constexpr double  SHAPE_PENALTY = 5.0;                              // 形状不匹配的代价, 可调
  constexpr double  COLOR_PENALTY = 8.0;                              // 颜色不准确的代价, 可调
  constexpr double  LANE_PENALTY  = 10.0;                             // 车道不匹配的代价, 可调，但需要一个较大值
  const std::size_t C             = junc.traffic_lights_spec.size();  // 行数C = 云端灯数量
  const std::size_t P             = matched_objs.size();              // 列数P = 感知灯数量

  std::vector<std::vector<double>> cost_matrix(C, std::vector<double>(P, 0.0));  // cm[i][j] = 云端灯i和感知灯j之间的代价
  // 循环求出每盏云端灯和感知灯之间的代价
  for (std::size_t ci = 0; ci < junc.traffic_lights_spec.size(); ++ci) {
    const auto &cloud             = junc.traffic_lights_spec[ci];
    bool        contains_ego_lane = std::any_of(cloud.lane_seq_id.begin(), cloud.lane_seq_id.end(), [&](uint32_t lane_id) {
      return static_cast<int>(lane_id) == ego_state.index;
    });  //判断云端灯是否包含当前车道
    double      lane_penalty      = contains_ego_lane ? 0.0 : LANE_PENALTY;
    for (std::size_t pi = 0; pi < matched_objs.size(); ++pi) {
      const auto &perc = matched_objs[pi];
      double      dx   = perc.position.x - cloud.position_ego.x;
      double      dy   = perc.position.y - cloud.position_ego.y;
      double      dist = std::hypot(dx, dy);

      double shape_penalty = (perc.attributes.traffic_light_shape != cloud.light_type) ? SHAPE_PENALTY : 0.0;
      double color_penalty = (perc.attributes.traffic_light_color == TrafficLightColorType::TLC_UNKNOWN) ? COLOR_PENALTY : 0.0;
      cost_matrix[ci][pi]  = dist + shape_penalty + lane_penalty + color_penalty;

      CLOUD_TRAFFIC_LOG << fmt::format(
          "cloud: '{}' ↔ perception: id={} dist={:.2f} shape_penalty={:.2f} color_penalty={:.2f} → cost={:.2f}", cloud.seq_id, perc.id,
          dist, shape_penalty, color_penalty, cost_matrix[ci][pi]);
    }
  }
  auto matches = KM_MinCost(cost_matrix);  //通过最小代价函数，得到云端灯和感知灯的最优匹配，这里的工具代价函数可以更换
  std::unordered_set<uint32_t> matched_ids;  //记录已经被占用的感知灯id

  for (const auto &light_pair : matches) {
    std::size_t ci        = light_pair.first;   // 云端灯索引
    std::size_t pi        = light_pair.second;  // 感知灯索引
    auto       &cloud     = junc.traffic_lights_spec[ci];
    const auto &percetion = matched_objs[pi];

    cloud.matched_obj_id = std::make_optional(percetion);  //将正确匹配写回结构体
    matched_ids.insert(percetion.id);
    CLOUD_TRAFFIC_LOG << fmt::format(
        "[MATCH_RESULT] cloud seq_id={} name='{}' (shape={}, lane_ids={}) ↔ perception id={} (shape={}) cost={:.2f}", cloud.seq_id,
        cloud.light_name, magic_enum::enum_name(cloud.light_type).data(), cloud.lane_seq_id, percetion.id,
        magic_enum::enum_name(percetion.attributes.traffic_light_shape).data(), cost_matrix[ci][pi]);
  }
  matched_objs.erase(std::remove_if(matched_objs.begin(), matched_objs.end(),
                                    [&](const TrfObjectInfo &obj) { return matched_ids.find(obj.id) == matched_ids.end(); }),
                     matched_objs.end());
}

constexpr double lon_threshold = 10.0;
constexpr double lat_threshold = 8.0;

void FindMatchObjsOfObj(const std::vector<TrfObjectInfo> &objs_filtered, const TrafficLightSpecial &traffic_light,
                        std::vector<TrfObjectInfo> &ids, double lon_thre, double lat_thre,
                        std::unordered_map<uint32_t, std::pair<double, double>> &id_to_factor) {
  for (const auto &obj_filt : objs_filtered) {                                       //遍历过滤后的感知灯
    if (std::fabs(obj_filt.position.x - traffic_light.position_ego.x) > lon_thre) {  //纵向
      continue;
    }
    if (std::fabs(obj_filt.position.y - traffic_light.position_ego.y) > lat_thre) {  //横向
      continue;
    }  //跳过前后方向、左右方向不满足感知灯和云端灯距离阈值要求的灯
    auto res = std::find_if(ids.begin(), ids.end(), [&obj_filt](const TrfObjectInfo &rhs) { return rhs.id == obj_filt.id; });  //查重
    if (res == ids.end()) {  //没在处理过的列表找到对应的id，说明是第一次匹配
      CLOUD_TRAFFIC_LOG << fmt::format("find_insert_filted_id:{} traffic_id:{}, matched in first stage: lon:{:.2f} lat:{:.2f}", obj_filt.id,
                                       traffic_light.seq_id, lon_thre, lat_thre);
      ids.emplace_back(obj_filt);                                                        //存入集合中
      id_to_factor[obj_filt.id] = {traffic_light.lon_factor, traffic_light.lat_factor};  //把阈值放大因子和对应的id写入map
    }
  }
}

void FillAroundLights(const std::vector<TrfObjectInfo> &objs_filterd, std::vector<TrfObjectInfo> &matched_objs, double lon_thre,
                      double lat_thre, std::unordered_map<uint32_t, std::pair<double, double>> &id_to_factor) {
  for (std::size_t idx = 0; idx < matched_objs.size(); idx++) {
    const auto &obj_match  = matched_objs[idx];      //当前已经匹配的红绿灯
    double      factor_lon = 1.0, factor_lat = 1.0;  //从map中读取相应灯的阈值放大因子
    auto        it = id_to_factor.find(obj_match.id);
    if (it != id_to_factor.end()) {
      factor_lon = it->second.first;
      factor_lat = it->second.second;
    }
    double cur_lon_thre = lon_thre * factor_lon;
    double cur_lat_thre = lat_thre * factor_lat;
    for (const auto &obj_filt : objs_filterd) {
      auto res =
          std::find_if(matched_objs.begin(), matched_objs.end(), [&obj_filt](const TrfObjectInfo &rhs) { return rhs.id == obj_filt.id; });
      if (res != matched_objs.end()) {
        continue;  //跳过已经匹配过的感知灯
      }
      if (std::fabs(obj_filt.position.x - obj_match.position.x) > cur_lon_thre) {
        continue;
      }
      if (std::fabs(obj_filt.position.y - obj_match.position.y) > cur_lat_thre) {
        continue;
      }  //跳过前后方向、左右方向不满足感知灯和已匹配灯距离阈值要求的灯
      CLOUD_TRAFFIC_LOG << fmt::format("fill_insert_filterd_id:{}  obj_match:{}, matched in second stage: lon:{:.2f} lat:{:.2f}",
                                       obj_filt.id, obj_match.id, cur_lon_thre, cur_lat_thre);
      matched_objs.emplace_back(obj_filt);  //将新找到的感知灯加入集合
      idx = 0;  //是一种把新加入的灯也立即参与后续搜索的扩散算法。不会陷入死循环是因为前面会检查是否还有未匹配的灯，如果没有了就不会执行到这
    }
  }
}

bool TrafficLightMatch(const std::vector<TrfObjectInfo> &obj_filterd, TrafficLightJunction &junc, const EgoState &ego_state) {
  constexpr double delta = 2.0;
  std::unordered_map<uint32_t, std::pair<double, double>> id_to_factor;  //构建一个局部哈希表，把灯和对应的阈值放大因子存入
  for (std::size_t lon_idx = 1; lon_idx <= 5; lon_idx++) {               //遍历经度阈值，2/4/6/8/10
    for (std::size_t lat_idx = 1; lat_idx <= 4; lat_idx++) {             //遍历纬度阈值，2/4/6/8
      double base_lon_thre = static_cast<double>(lon_idx) * delta;
      double base_lat_thre = static_cast<double>(lat_idx) * delta;
      for (const auto &traffic_light : junc.traffic_lights_spec) {
        double lon_thre = base_lon_thre * traffic_light.lon_factor;
        double lat_thre = base_lat_thre * traffic_light.lat_factor;
        FindMatchObjsOfObj(obj_filterd, traffic_light, junc.matched_obj_ids, lon_thre, lat_thre,
                           id_to_factor);  //对云端红绿灯尝试匹配感知灯
      }
      FillAroundLights(obj_filterd, junc.matched_obj_ids, base_lon_thre, base_lat_thre,
                       id_to_factor);  //对已匹配的红绿灯附近满足阈值距离的灯也尝试进行匹配
    }
  }
  CLOUD_TRAFFIC_LOG << fmt::format("intial_light_ids_size:{}", junc.matched_obj_ids.size());
  //如果在进行匹配之前，找到的感知灯的数量大于云端灯的数量，则进行一次感知灯的过滤，至多保留N+1个灯
  if (junc.matched_obj_ids.size() > junc.traffic_lights_spec.size()) {
    FilterPerceptionByDistance(junc);
    CLOUD_TRAFFIC_LOG << fmt::format("filtered_light_ids_size:{}", junc.matched_obj_ids.size());
  }
  std::sort(junc.matched_obj_ids.begin(), junc.matched_obj_ids.end(),
            [](const TrfObjectInfo &lhs, const TrfObjectInfo &rhs) { return lhs.position.y > rhs.position.y; });  //将感知灯从左到右排序
  MatchCloudsToPerception(junc.matched_obj_ids, junc, ego_state);
  CLOUD_TRAFFIC_LOG << fmt::format("final matched_ids_size:{}", junc.matched_obj_ids.size());
  if (junc.matched_obj_ids.size() == junc.traffic_lights_spec.size()) {  //匹配的感知灯数量 ?= 云端的红绿灯数量
    return true;
  }
  return false;
}

EgoState InfoEgoState(const BevMapInfoPtr &bev_map, const Point3D &jun_pos, const uint32_t &jun_lan_seq_id,
                      Eigen::Isometry3d *Tbw_ptr = nullptr) {
  EgoState res;
  if (!bev_map) {
    CLOUD_TRAFFIC_LOG << "bev_map_empty.";
    return res;
  }
  const BevLaneInfo *ego_bev_lane{nullptr};

  // uint64_t ego_section_id = bev_map->route.navi_start.section_id;
  for (const auto &lane : bev_map->lane_infos) {
    if (lane.position == 0) {  //表示自车当前所在车道
      ego_bev_lane = &lane;
    }
  }
  if (ego_bev_lane == nullptr) {
    CLOUD_TRAFFIC_LOG << "ego_bev_lane_empty.";
    return res;
  }

  std::vector<Vec2d> points;                                   //存放车道中心线的离散点
  const auto        &lane_points = ego_bev_lane->line_points;  //车道中线点的集合
  Eigen::Vector3d    line_point_st;
  for (const auto &point_curr : lane_points) {  //遍历车道中心线
    line_point_st.x() = point_curr.x;
    line_point_st.y() = point_curr.y;
    if (Tbw_ptr != nullptr) {
      line_point_st = *Tbw_ptr * line_point_st;
    }
    points.emplace_back(line_point_st.x(), line_point_st.y());  //把二维点存入vector
  }

  if (points.empty()) {
    CLOUD_TRAFFIC_LOG << "lane_point_empty.";
    return res;
  }
  MultiLineSegment multi_seg;

  multi_seg.Init(points);  //用车道点初始化可投影的折线
  double proj_s{0.0};      //投影点从车道起点开始的累积弧长。可以判断路口在车道前半段还是后半段
  double proj_l{0.0};      //投影点相对车道中心线的横向偏移
  double min_dis{0.0};     //路口点到投影的欧氏距离
  bool   proj_val = multi_seg.GetProjection(Vec2d{jun_pos.x, jun_pos.y}, &proj_s, &proj_l, &min_dis);  //这里假设折线是连续的
  if (proj_val && proj_s < multi_seg.Length() + 5.0 && proj_s > -5.0) {      //投影成功且投影点在车道起点和终点中
    res.ego_dir = EgoState::Dir::LeftStart;                         //标志位，表示有效？
    res.index   = static_cast<int>(proj_l / 3.5) + jun_lan_seq_id;  //表示自车现在处于第几车道
  }
  CLOUD_TRAFFIC_LOG << fmt::format("proj_val:{:d} proj_s:{:.2f}  proj_l:{:.2f}  min_dis:{:.2f}  res_dir:{} res_index:{} len:{:.2f}",
                                   proj_val, proj_s, proj_l, min_dis, magic_enum::enum_name(res.ego_dir), res.index, multi_seg.Length());
  return res;
}

void LaneClassTrafficLight(const EgoState &ego_state, const TrafficLightJunction &junc, TrafficLights &traffic_light) {
  std::optional<TrfObjectInfo> target_obj = std::nullopt;

  bool is_valid_right = false;
  for (const auto &lane : junc.lane_info) {
    if (lane.seq_id != ego_state.index) {  //只处理自车所在车道
      continue;
    }
    for (const auto &traffic : junc.traffic_lights_spec) {  //遍历所有云端灯
      for (const auto ass_id : traffic.lane_seq_id) {
        if (ass_id != lane.seq_id) {  //只处理与当前车道关联的云端灯
          continue;
        }
        TrafficLight res;
        res.traffic_obj_info = std::make_shared<TrfObjectInfo>(*traffic.matched_obj_id);  //解引用后复制进shared_ptr，后面可以安全共享
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
  if (!is_valid_right) {  //右转默认是绿灯
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
  const auto mid = LongLat2Tile(ego_pos.lon, ego_pos.lat, zoom);  //把经纬度转为瓦片坐标，(x,y,zoom)
  if (mid) {
    const auto tile_id = EncodeTileId(*mid);  //把瓦片坐标压缩成64bit编码
    if (tile_id) {
      const auto de_id = DecodeTileId(*tile_id);  //解码，验证编解码前后是否一致
      CLOUD_TRAFFIC_LOG << fmt::format("tile_id:{} decode_mid:{}  {}  {}", *tile_id, de_id.x, de_id.y, de_id.z)
                        << fmt::format("  mid:{}  {}   zoom:{}", mid->x, mid->y, mid->z);
      return tile_id;  //返回瓦片坐标的编码id
    }
  }
  return std::nullopt;  //任意一步失败，如经纬度非法，ZOOM超出范围等均返回nullopt
}

std::optional<nlohmann::json> GetZoomListJsonCloud(const ResourceManageGet &resource_manager) {
  static std::map<std::string, std::optional<nlohmann::json>> cloud_zoom_cache;
  static std::mutex                                           cloud_zoom_mtx;

  // 版本切换时清空缓存
  if (resource_manager.IsVersionChange()) {
    std::lock_guard<std::mutex> lk(cloud_zoom_mtx);
    cloud_zoom_cache.clear();
  }

  // 先查缓存
  {
    std::lock_guard<std::mutex> lk(cloud_zoom_mtx);
    auto                        it = cloud_zoom_cache.find("configer_param.json");
    if (it != cloud_zoom_cache.end())
      return it->second;
  }

  // 判断云端文件是否已经下发
  std::optional<std::string> cloud_path = resource_manager.IsFileExist("configer_param.json");
  if (!cloud_path) {
    CLOUD_TRAFFIC_LOG << "[CLOUD] configer_param.json NOT found in cloud.";
    std::lock_guard<std::mutex> lk(cloud_zoom_mtx);
    cloud_zoom_cache.emplace("configer_param.json", std::nullopt);
    return std::nullopt;
  }

  // 读取并缓存
  std::optional<nlohmann::json> cfg = LoadJsonConfig(*cloud_path);
  {
    std::lock_guard<std::mutex> lk(cloud_zoom_mtx);
    cloud_zoom_cache.emplace("configer_param.json", cfg);
  }
  if (!cfg)
    CLOUD_TRAFFIC_LOG << "[CLOUD] Failed to parse configer_param.json : " << *cloud_path;
  return cfg;
}

std::optional<nlohmann::json> GetZoomListJsonLocal() {
  const std::string filename = "configer_param.json";
  // 缓存查询
  {
    std::lock_guard<std::mutex> lk(local_zoom_mtx);
    auto                        it = local_zoom_cache.find(filename);
    if (it != local_zoom_cache.end())
      return it->second;
  }
  const std::string             file_path = "modules/perception/env/conf/CloudTrafficFiles/" + filename;
  std::optional<nlohmann::json> cfg       = LoadJsonConfig(file_path);
  {
    std::lock_guard<std::mutex> lk(local_zoom_mtx);
    local_zoom_cache.emplace(filename, cfg);
  }
  if (!cfg)
    CLOUD_TRAFFIC_LOG << "[LOCAL] Failed to parse " << file_path;
  return cfg;
}

std::optional<nlohmann::json> GetZoomListJson(const ResourceManageGet &resource_manager) {
  //云端
  if (IsCloudSignalReceived()) {
    if (auto cfg = GetZoomListJsonCloud(resource_manager); cfg) {
      CLOUD_TRAFFIC_LOG << "[CLOUD] Loaded zoom list from cloud.";
      return cfg;
    }
    CLOUD_TRAFFIC_LOG << "[CLOUD] Cloud config not available while cloud signal is active.";
    return std::nullopt;
  }
  //本地
  if (auto cfg = GetZoomListJsonLocal(); cfg) {
    CLOUD_TRAFFIC_LOG << "[LOCAL] Loaded zoom list from local file.";
    return cfg;
  }
  CLOUD_TRAFFIC_LOG << "Unable to load any zoom list.";
  return std::nullopt;
}

std::vector<uint8_t> GetZoomList(const ResourceManageGet &resource_manager) {
  std::vector<uint8_t>          zooms;  //定义空容器，等待数据存入
  std::optional<nlohmann::json> val = GetZoomListJson(resource_manager);
  if (!val)
    return zooms;

  if (!val->contains("zoom_list")) {  //检查是否包含该键，否则表示配置文件不符合预期
    CLOUD_TRAFFIC_LOG << "Missing 'zoom_list' field";
    return zooms;
  }

  auto &zoom_list = (*val)["zoom_list"];
  if (!zoom_list.is_array()) {
    CLOUD_TRAFFIC_LOG << "'zoom_list' is not an array";
    return zooms;
  }

  if (zoom_list.size() != 3) {  //只接受三层瓦片，粗-中-细层
    CLOUD_TRAFFIC_LOG << fmt::format("Invalid zoom_list size: {}", zoom_list.size());
    return zooms;
  }

  for (const auto &item : zoom_list) {
    if (!item.is_number()) {  //检查当前元素是否为数值型
      CLOUD_TRAFFIC_LOG << "Non-integer value in zoom_list";
      return zooms;
    }
    const int value = item.get<int>();  //把JSON元素强制转换为int型
    if (value < 0 || value > 255) {
      CLOUD_TRAFFIC_LOG << fmt::format("Value {} out of uint8_t range", value);
      return zooms;
    }
    zooms.push_back(static_cast<uint8_t>(value));  //再转为uint_8型
  }
  CLOUD_TRAFFIC_LOG << fmt::format("zoom_list:{}", zooms);
  if (zooms.size() != 3) {  //防御性检查
    zooms.clear();
    return zooms;
  }
  return zooms;
}

std::optional<EgoTileId> GetEgoTileId(CoordSystem ego_pos, std::vector<uint8_t> zooms) {
  if (zooms.empty()) {
    return std::nullopt;
  }

  auto tile_0 = GetTileId(ego_pos, zooms[0]);  //粗糙级，用于快速定位到哪个大瓦片文件
  auto tile_1 = GetTileId(ego_pos, zooms[1]);  //中层级，用于在大瓦片内部进一步定位到子目录
  auto tile_2 = GetTileId(ego_pos, zooms[2]);  //精细级，用于精确到具体交叉口的灯配置信息

  if (tile_0 && tile_1 && tile_2) {  //三次计算都必须成功
    return EgoTileId{*tile_0, *tile_1, *tile_2};
  }

  return std::nullopt;
}

void ChangeLightArrow(const TrafficLightSpecial &light_cloud, const PercepTrfInfoPtr &perception_traffic_light_info) {
  if (!light_cloud.matched_obj_id) {
    return;  //确保云端灯匹配到了感知灯
  }
  auto light_val =
      std::find_if(perception_traffic_light_info->objects.begin(), perception_traffic_light_info->objects.end(),
                   [light_cloud](const TrfObjectInfo &rhs) { return rhs.id == light_cloud.matched_obj_id->id; });  //在集合中找到感知灯
  if (light_val == perception_traffic_light_info->objects.end()) {
    return;
  }
  light_val->attributes.traffic_light_shape = light_cloud.light_type;
  CLOUD_TRAFFIC_LOG << fmt::format("change_perception_id:{}  cloud_arrow:{}", light_val->id, light_cloud.light_type);
}

void SetNoneTrafficLight(TrafficLight &traffic_light, TrafficLightColorType color_type) {
  traffic_light.is_valid         = true;
  traffic_light.color            = color_type;
  traffic_light.traffic_reason   = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
  traffic_light.traffic_obj_info = std::make_shared<cem::message::sensor::TrfObjectInfo>();

  traffic_light.traffic_obj_info->id         = 998;
}

static bool IsKeyLightForEgo(const TrafficLightSpecial &cloud_light, const EgoState &ego_state, const TrafficLightJunction &junc) {
  // ego_state.index 已经是 lane_info 的下标，取对应 lane 的 seq_id
  if (ego_state.ego_dir == EgoState::Dir::Unknown || ego_state.index < 0 || ego_state.index >= static_cast<int>(junc.lane_info.size())) {
    CLOUD_TRAFFIC_LOG << fmt::format("invalid ego state, dir={}, idx={}, lane_num={}", static_cast<int>(ego_state.ego_dir), ego_state.index,
                                     junc.lane_info.size());
    return false;
  }
  const auto &ego_lane = junc.lane_info[ego_state.index];
  // cloud_light.lane_seq_id 里存放的是 lane 的 seq_id（uint32_t）
  return std::find(cloud_light.lane_seq_id.begin(), cloud_light.lane_seq_id.end(), ego_lane.seq_id) != cloud_light.lane_seq_id.end();
}

void SetDefaultTrafficLight(TrafficLight &traffic_light, TrafficLightColorType color_type, const TrafficLightSpecial &traffic_light_front) {
  traffic_light.is_valid         = true;
  traffic_light.color            = color_type;
  traffic_light.traffic_reason   = byd::msg::orin::routing_map::LaneInfo::NEW_BLURRING_COLOR;
  traffic_light.traffic_obj_info = std::make_shared<cem::message::sensor::TrfObjectInfo>();

  traffic_light.traffic_obj_info->id         = 998;
  traffic_light.traffic_obj_info->position.x = traffic_light_front.position_ego.x;
  traffic_light.traffic_obj_info->position.y = traffic_light_front.position_ego.y;

  if (traffic_light.turn_type == TurnType::RIGHT_TURN && color_type != TrafficLightColorType::TLC_NOT_MATCH) {
    traffic_light.color = TrafficLightColorType::TLC_GREEN;
  }
}

void FillTrafficLightFromCloud(const TrafficLightSpecial &cloud, TrafficLight &target, TurnType tt) {
  target.traffic_obj_info       = std::make_shared<TrfObjectInfo>(*cloud.matched_obj_id);
  target.traffic_light_num      = target.traffic_obj_info->attributes.traffic_light_num;
  target.color                  = target.traffic_obj_info->attributes.traffic_light_color;
  target.traffic_light_flashing = target.traffic_obj_info->attributes.traffic_light_flashing;
  target.is_valid               = true;
  target.turn_type              = tt;
}

static std::optional<TrafficLights> PartialMatchTrafficLight(const EgoState &ego_state, const TrafficLightJunction &junc,
                                                             const std::vector<TrfObjectInfo> &objs_filtered) {
  TrafficLights traffic_light;

  const auto        &ego_lane        = junc.lane_info[ego_state.index];
  const unsigned int lane_arrow      = ego_lane.arrow_type;  // 0b0001直,0b0010左,0b0100右,0b1000掉头
  const double       nomatch_dis_thr = 100.0;
  auto               get_turn_type   = [&](TurnType tt) -> TrafficLight                 &{
    switch (tt) {
      case TurnType::NO_TURN:
        return traffic_light.straight;
      case TurnType::LEFT_TURN:
        return traffic_light.left;
      case TurnType::RIGHT_TURN:
        return traffic_light.right;
      case TurnType::U_TURN:
        return traffic_light.u_turn;
      default:
        return traffic_light.straight;  //保护，防止编译出错
    }
  };
  auto get_light_shape = [&](const TrafficLightShapeType &shape, TurnType turn) -> bool {
    switch (turn) {
      case TurnType::NO_TURN:
        return shape == TrafficLightShapeType::TLS_UP_ARROW || shape == TrafficLightShapeType::TLS_CIRCULAR;
      case TurnType::LEFT_TURN:
        return shape == TrafficLightShapeType::TLS_LEFT_ARROW || shape == TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW ||
               shape == TrafficLightShapeType::TLS_CIRCULAR;
      case TurnType::RIGHT_TURN:
        return shape == TrafficLightShapeType::TLS_RIGHT_ARROW || shape == TrafficLightShapeType::TLS_UP_RIGHT_ARROW ||
               shape == TrafficLightShapeType::TLS_CIRCULAR;
      case TurnType::U_TURN:
        return shape == TrafficLightShapeType::TLS_TURN_ARROUND_ARROW || shape == TrafficLightShapeType::TLS_LEFT_TURN_ARROUND_ARROW ||
               shape == TrafficLightShapeType::TLS_CIRCULAR;
      default:
        return false;
    }
  };
  bool Keylightfilled = false;
  for (const auto &cloud : junc.traffic_lights_spec) {
    if (!cloud.matched_obj_id)
      continue;  //没匹配到感知灯，跳过
    if (!IsKeyLightForEgo(cloud, ego_state, junc))
      continue;  //不是控制本车道的云端灯，跳过
    const auto &perception       = *cloud.matched_obj_id;
    const auto  perception_shape = perception.attributes.traffic_light_shape;
    if ((lane_arrow & 0b0001U) != 0U) {  //根据 lane_arrow 把匹配的方向和对应灯填进去
      if (get_light_shape(perception_shape, TurnType::NO_TURN)) {
        FillTrafficLightFromCloud(cloud, traffic_light.straight, TurnType::NO_TURN);
        Keylightfilled = true;
      }
    }
    if ((lane_arrow & 0b0010U) != 0U) {
      if (get_light_shape(perception_shape, TurnType::LEFT_TURN)) {
        FillTrafficLightFromCloud(cloud, traffic_light.left, TurnType::LEFT_TURN);
        Keylightfilled = true;
      }
    }
    if ((lane_arrow & 0b0100U) != 0U) {
      if (get_light_shape(perception_shape, TurnType::RIGHT_TURN)) {
        FillTrafficLightFromCloud(cloud, traffic_light.right, TurnType::RIGHT_TURN);
        Keylightfilled = true;
      }
    }
    if ((lane_arrow & 0b1000U) != 0U) {
      if (get_light_shape(perception_shape, TurnType::U_TURN)) {
        FillTrafficLightFromCloud(cloud, traffic_light.u_turn, TurnType::U_TURN);
        Keylightfilled = true;
      }
    }
    break;
  }

  const auto &frontest_cloud_light = junc.traffic_lights_spec.front();  //没有匹配的方向就用默认颜色填充，位置距离列表中第一个云端灯
  double                dist_to_light = std::hypot(frontest_cloud_light.position_ego.x, frontest_cloud_light.position_ego.y);
  std::vector<TurnType> turn_vec{TurnType::LEFT_TURN, TurnType::U_TURN, TurnType::RIGHT_TURN, TurnType::NO_TURN};

  switch (junc.junction_type) {
    case 1:  //车道级红绿灯，在一定距离内，没能给出自车所在车道的红绿灯状态的时候要发LightStatus::CLOUD_LIGHT_NOMATCH
    case 2:    //红绿灯超出视野范围，只要提前刹车就行
    case 3:    //右转圆灯，只需要基于云端灯更改灯的shape，不完全匹配走不走这个逻辑。
    case 5:    //需要配置文件发出虚拟停止线
    case 6:    //前方路口有灯但感知输出较晚，需要提前减速
    case 7:    //当前车道不受灯控
    case 8:    //感知无法检出当前路口的红绿灯
    case 9: {  //存在黄闪灯或者慢字灯
      for (const auto &turn_t : turn_vec) {
        TrafficLight &trf_light = get_turn_type(turn_t);
        if (!Keylightfilled) {  //控制当前自车车道的红绿灯是否匹配
          if (dist_to_light <= nomatch_dis_thr) {
            SetDefaultTrafficLight(trf_light, TrafficLightColorType::TLC_NOT_MATCH, frontest_cloud_light);
            CLOUD_TRAFFIC_LOG << fmt::format("[LIGHT_NOTMATCH] case {}", junc.junction_type);
          }
        } else {
          if (trf_light.turn_type == TurnType::RIGHT_TURN && !trf_light.is_valid) {
            trf_light.is_valid       = true;
            trf_light.color          = TrafficLightColorType::TLC_GREEN;
            trf_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
          }
        }
      }
      break;
    }
    case 4:  //红灯损坏的情况，在红灯灭的时候要给出红灯
    default: {
      for (const auto &turn_t : turn_vec) {
        TrafficLight &trf_light = get_turn_type(turn_t);
        if (!Keylightfilled) {  //控制当前自车车道的红绿灯是否匹配
          SetDefaultTrafficLight(trf_light, TrafficLightColorType::TLC_RED, frontest_cloud_light);
        } else {
          if (trf_light.turn_type == TurnType::RIGHT_TURN && !trf_light.is_valid) {
            trf_light.is_valid       = true;
            trf_light.color          = TrafficLightColorType::TLC_GREEN;
            trf_light.traffic_reason = byd::msg::orin::routing_map::LaneInfo::SET_DEFAULT_OBJ;
          }
        }
      }
    }
  }
  traffic_light.junction_cloud_type = junc.junction_type;
  LogTrafficLights(traffic_light);
  return traffic_light;
}

std::optional<TrafficLights> GetMatchTrafficLight(const CoordSystem &ego_pos, const EgoTileId &ego_tile_id,
                                                  const std::vector<uint8_t> &zooms, const std::vector<TrfObjectInfo> &obj_filterd,
                                                  const BevMapInfoPtr &bev_map, PercepTrfInfoPtr perception_traffic_light_info,
                                                  const ResourceManageGet &resource_manager, std::string &cloud_debug_info,
                                                  Eigen::Isometry3d *Tbw_ptr = nullptr) {
  std::optional<TrafficLightJunction> junc{std::nullopt};

  static std::map<EgoTileId, std::optional<TrafficLightJunction>> map_tileid_junction;
  if (resource_manager.IsVersionChange()) {
    map_tileid_junction.clear();
  }

  auto res = map_tileid_junction.find(ego_tile_id);  //查询缓存，若没缓存就读取JSON
  if (res != map_tileid_junction.end()) {
    junc = res->second;
  } else {
    junc =
        GetTrafficLightSpecialJunction(ego_tile_id, zooms, resource_manager);  // 从JSON中拿云端灯和车道整体信息。老版本和新版本的字段不统一
    cloud_debug_info += fmt::format("cloud_junc_version: {}", junc->version + ";");
    map_tileid_junction.emplace(ego_tile_id, junc);
  }
  if (!junc || junc->junction_type == 0) {
    CLOUD_TRAFFIC_LOG << "not_find_junc.";  //路口有效性的检查
    return std::nullopt;
  }
  junc->matched_obj_ids.clear();
  for (auto &traf : junc->traffic_lights_spec) {
    traf.matched_obj_id = std::nullopt;  //清除上一次红绿灯匹配信息
  }
  junc->position_ego = Point3D{};

  junc->position_ego = TransformPointCorA2CorB({0.0, 0.0, 0.0}, junc->position, ego_pos);  // 将路口原点从路口坐标系 → 车辆局部 ENU 坐标系
  CLOUD_TRAFFIC_LOG << fmt::format("ego_localization:{:.4f},{:.4f}  heading:{:.4f} junc_pos:{:.2f},{:.2f}", ego_pos.lon, ego_pos.lat,
                                   ego_pos.heading, junc->position_ego.x, junc->position_ego.y);
  for (auto &traffic_light : junc->traffic_lights_spec) {
    traffic_light.position_ego =
        TransformPointCorA2CorB(traffic_light.position, junc->position, ego_pos);  //计算出每一个红绿灯相对于自车的坐标
    CLOUD_TRAFFIC_LOG << fmt::format(
        "junc_localization:{:.4f},{:.4f} heading:{:.4f} obj_id:{} raw_light_pos:{:.2f},{:.2f}  new_pos:{:.2f},{:.2f}", junc->position.lon,
        junc->position.lat, junc->position.heading, traffic_light.seq_id, traffic_light.position.x, traffic_light.position.y,
        traffic_light.position_ego.x, traffic_light.position_ego.y);
  }
  //给出自车所在车道的序号和行驶方向。todo:车道中心线可能会出现断续，不稳定 //, Tbw_ptr
  EgoState ego_state = InfoEgoState(bev_map, junc->position_ego, junc->position_lane_seq_id, Tbw_ptr);
  if (ego_state.ego_dir == EgoState::Dir::Unknown) {
    CLOUD_TRAFFIC_LOG << fmt::format("ego_state_infer_failed.");
    return std::nullopt;  //自车方向未知，放弃匹配
  }
  if (ego_state.index > junc->lane_info.size() - 1) {
    CLOUD_TRAFFIC_LOG << fmt::format("ego_num_exceed_lane_num.  ego_idx:{}  lane_num:{}", ego_state.index, junc->lane_info.size());
    return std::nullopt;  //自车所在车道索引越界
  }

  for (auto &traffic_light : junc->traffic_lights_spec) {
    int    lane_gap    = 0;
    double Lane_factor = 1.0;  //放大因子
    if (!traffic_light.lane_seq_id.empty()) {
      for (uint32_t lane_id : traffic_light.lane_seq_id) {
        int cur_gap = std::abs(static_cast<int>(lane_id) - static_cast<int>(ego_state.index));
        lane_gap    = std::max(lane_gap, cur_gap);  //灯所控制车道与现在车道最大的差值
      }
    } else {
      lane_gap = 0;
    }
    double lane_factor = 1.0;  // 同车道不放大
    if (lane_gap == 1) {
      lane_factor = 1.5;  // 相邻车道
    } else if (lane_gap > 1) {
      lane_factor = 2.0;  // 其余车道
    }
    traffic_light.lon_factor = lane_factor;  // 经度阈值放大系数, 纵向
    traffic_light.lat_factor = lane_factor;  // 纬度阈值放大系数, 横向
    CLOUD_TRAFFIC_LOG << fmt::format("cloud_id:{}, lane_gap:{}, lane_factor:{}", traffic_light.seq_id, lane_gap, lane_factor);
  }

  bool suc = TrafficLightMatch(obj_filterd, *junc, ego_state);  // 检查云端的红绿灯是否都找到了最近且满足距离阈值的感知红绿灯。
  cloud_debug_info += fmt::format(" cloud_junc_version: {};", junc->version);
  CLOUD_TRAFFIC_LOG << fmt::format("cloud_matched:{:d} ", suc);
  TrafficLights traffic_lights;
  traffic_lights.junction_cloud_type  = junc->junction_type;         //云端路口类型
  traffic_lights.distance_to_stopline = junc->distance_to_stopline;  //自车到路口停止线的距离
  traffic_lights.junction_tail_id     = junc->tail_id;               //精细瓦片的id
  traffic_lights.stop_line            = junc->stop_line;             //云端下发的停止线的位置

  if (!suc) {                                  //处理匹配失败的情况。
    if (!junc->traffic_lights_spec.empty()) {  //若路口本身有灯，给出距离最近的云端灯的位置信息，然后给自车所在车道的方向指定颜色
      return PartialMatchTrafficLight(ego_state, *junc, obj_filterd);
    }
    return std::nullopt;  //没灯就返回空值
  }
  switch (junc->junction_type) {  //根据路口类型分类。todo终极目标：添加更多路口类型
    case 1:
    case 2:  //case 1/2，车道级红绿灯，多分组路口/红绿灯超出视野范围，需要把灯颜色等信息分配到自车行驶的方向
    case 3: {
      LaneClassTrafficLight(ego_state, *junc, traffic_lights);
      CLOUD_TRAFFIC_LOG << fmt::format("get_traffic_lights_in_case {}.", junc->junction_type);
      break;
    }
    // case 3: {  //特殊路口，只需要让云端灯的箭头形状同步到感知灯，使用perception_traffic_light_info
    //   for (const auto &light_cloud : junc->traffic_lights_spec) {
    //     if (light_cloud.matched_obj_id) {
    //       ChangeLightArrow(light_cloud, perception_traffic_light_info);
    //     }
    //   }
    //   CLOUD_TRAFFIC_LOG << ("get_traffic_lights_in_case3. change_light_arrow.");
    //   return std::nullopt;
    // }
    case 4:    //红灯损坏，即直行为绿灯，红灯的时候灯灭了的情况
    case 5:    //需要给出虚拟停止线
    case 6:    //更新：前方路口有灯但感知输出较晚，需要提前减速
    case 7:    //更新：当前车道不受灯控
    case 8: {  //更新：感知无法检出当前路口的红绿灯
      LaneClassTrafficLight(ego_state, *junc, traffic_lights);
      CLOUD_TRAFFIC_LOG << fmt::format("get_traffic_lights_in_case {}.", junc->junction_type);
      break;
    }
    case 9: {
      if (junc->traffic_lights_spec.empty()) {
        SetNoneTrafficLight(traffic_lights.left, TrafficLightColorType::TLC_NONE_LIGHT);
        SetNoneTrafficLight(traffic_lights.u_turn, TrafficLightColorType::TLC_NONE_LIGHT);
        SetNoneTrafficLight(traffic_lights.straight, TrafficLightColorType::TLC_NONE_LIGHT);
        SetNoneTrafficLight(traffic_lights.right, TrafficLightColorType::TLC_NONE_LIGHT);
        CLOUD_TRAFFIC_LOG << fmt::format("case: {}, this junction is no lights.", junc->junction_type);
      } else {
        LaneClassTrafficLight(ego_state, *junc, traffic_lights);
        CLOUD_TRAFFIC_LOG << fmt::format("get_traffic_lights_in_case {}.", junc->junction_type);
        break;
      }
    }
    default:
      break;
  }
  LogTrafficLights(traffic_lights);
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

  MapMatchResultPtr map_match_result_ptr = GetLatestMapMatchResult();  //获取地图匹配结果，包括自车的经纬度，航向
  if (!map_match_result_ptr) {
    CLOUD_TRAFFIC_LOG << "no_find_map_match.";
    return std::nullopt;
  }
  PercepTrfInfoPtr perception_traffic_light_info = GetLatestPerceptionInfo();  //获取感知给的红绿灯对象
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
    if (IsValidTrafficLight(obj)) {  //对感知给的红绿灯对象进行过滤
      obj_filterd.emplace_back(obj);
    }
  }
  CLOUD_TRAFFIC_LOG << fmt::format("Counter_map_match:{}  traffic_light:{} bev:{}", map_match_result_ptr->header().sequence_num(),
                                   perception_traffic_light_info->header.cycle_counter, bev_map ? bev_map->header.cycle_counter : 0);

  CoordSystem ego_pos;  //声明一个自车全局坐标结构体，经度纬度航向
  ego_pos.lon     = map_match_result_ptr->pose().position().lon() * 180.0 / M_PI;  //经度，转为十进制的度
  ego_pos.lat     = map_match_result_ptr->pose().position().lat() * 180.0 / M_PI;  //纬度
  ego_pos.heading = map_match_result_ptr->pose().heading();                        //航向，保持为弧度

  bool        cloud_file_cur  = resource_manage_get.IsFileExist("configer_param.json").has_value();
  static bool cloud_file_prev = false;
  if (cloud_file_cur != cloud_file_prev) {
    // 状态发生变化, 执行一次切换操作
    SetCloudSignalReceived(cloud_file_cur);
    if (cloud_file_cur) {
      ClearLocalZoomCache();
      ClearAllLocalTileCache();
      CLOUD_TRAFFIC_LOG << "[CLOUD] Switch to cloud mode, local caches cleared.";
    } else {
      CLOUD_TRAFFIC_LOG << "[LOCAL] Switch back to local mode.";
    }
    cloud_file_prev = cloud_file_cur;
  }
  cloud_debug_info +=
      fmt::format("Counter:{} :CloudSignalReceived: {}--cloud_file_cur: {} cloud_file_prev: {}",
                  perception_traffic_light_info->header.cycle_counter, IsCloudSignalReceived(), cloud_file_cur, cloud_file_prev);
  std::vector<uint8_t> zooms = GetZoomList(resource_manage_get);  //读取配置文件里的Zoom列表，三层瓦片, todo:区分本地和云端获取数据
  if (zooms.empty()) {
    CLOUD_TRAFFIC_LOG << "no_find_config_para_json.";
    return std::nullopt;
  }
  CLOUD_TRAFFIC_LOG << fmt::format("find_zoom:{}", zooms);
  std::optional<EgoTileId> ego_tile_id = GetEgoTileId(ego_pos, zooms);  //一次性得到三个瓦片的ID，返回结构体ego_tile_id
  if (!ego_tile_id) {
    return std::nullopt;  //如果任何一级失败了，则返回nullopt
  }

  constexpr double ego_radius = 0.3;                                                         // km
  std::vector<EgoTileId> ego_tile_ids_0 = FindNearbyEgoTileIDs(ego_pos, ego_radius, zooms);  // 计算0.3km半径内，三个层级的候选瓦片集合
  for (const auto &ego_tile_id : ego_tile_ids_0) {  //从粗到细，遍历所有候选瓦片集合，逐个尝试匹配云端交叉口
    CLOUD_TRAFFIC_LOG << fmt::format("Find_Junction_In_Tile  ego_around_tile_0:{} tile_1:{}  tile_2:{}", ego_tile_id.tile_id_0,
                                     ego_tile_id.tile_id_1, ego_tile_id.tile_id_2);
    auto traffic_res = GetMatchTrafficLight(ego_pos, ego_tile_id, zooms, obj_filterd, bev_map, perception_traffic_light_info,
                                            resource_manage_get, cloud_debug_info, Tbw_ptr);
    if (traffic_res) {
      return traffic_res;
    }
  }
  return std::nullopt;
}

}  // namespace cem::fusion::claud_traffic