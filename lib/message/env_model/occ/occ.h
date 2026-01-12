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
 * @file occ.h
 * @author xu.xuefeng1@byd.com
 * @brief
 * @version 0.1
 * @date 2025-07-06
 */
#ifndef OCC_H_
#define OCC_H_

#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include "message/common/geometry.h"
#include "message/common/header.h"
#include "modules/common/math/vec2d.h"

namespace cem {
namespace env_model {
namespace occ { 

using byd::common::math::Vec2d;

enum GeneralDetectorType {
  GOD_TYPE_UNKNOWN = 0,
  STEREO_OCC = 1,
  BEV_FREESPACE = 2
};

enum GeneralObjectType {
  GOD_UNKNOWN = 0, // 未知,网络输出不了情况
  GOD_DYNAMIC = 1, // 动态目标
  GOD_STATIC = 2// 静态目标
};

struct HeightRange {
  double min{0.f}; // 针对悬空目标, 最低高度
  double max{0.f} ;// 针对悬空目标，最高高度
};

struct GodInfo {
  uint64_t id = 0; 
  std::vector< Eigen::Vector2d> shape{} ;
  GeneralObjectType category ;
  HeightRange height_range  ;
  double x{0.f} ; // 最近点？
  double y{0.f} ;
  double z{0.f}; // 对bev freespace不会给高度
  double length{0.f} ;
  double width{0.f} ;
  double height{0.f} ;
  double heading{0.f} ;
  double velocity_x{0.f};
  double velocity_y{0.f} ;
};

struct RLEEntry {
  int32_t value{0}; // 像素值
  int32_t count{0}; // 出现次数
};

struct RLEImage {
  int32_t width{0};
  int32_t height{0};
  std::vector<RLEEntry> rle_units{};
  double pixel_width_resolution{0.f};
  double pixel_height_resolution{0.f};
  double veh_pos_x{0.f}; // bev_occ原点像素坐标x轴(cv)偏移量
  double veh_pos_y{0.f}; // bev_occ原点像素坐标y轴(cv)偏移量
};

struct OCCInfo {
   cem::message::common::Header     header;
   int32_t                          frame_id{0};
   GeneralDetectorType                godtype       = GeneralDetectorType::GOD_TYPE_UNKNOWN ;
   std::vector<GodInfo> objects{}; // 通用障碍物目标列表
   std::vector< Eigen::Vector2d> driving_corridor{}; // 通行边界
   RLEImage occ_mask; // occ mask信息
};
}
}
}  // namespace cem::fusion::occ
#endif
