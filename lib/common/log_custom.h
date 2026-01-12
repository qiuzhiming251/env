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
 * @file log_custom.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-03-18
 */

#ifndef LOG_CUSTOM_H_
#define LOG_CUSTOM_H_

#include "cyber/common/log.h"

// NOLINTBEGIN
class NullLog {
 public:
  template <typename T>
  NullLog &operator<<(const T &rhs) {
    return *this;
  }
};

#define NULL_LOG \
  if (true) {    \
  } else         \
    NullLog()

/**
 * @brief 简单的示例
 * 
 * @author lingpeng (ling.peng3@byd.com)
 * @date 2025-03-21
 // TRAFFIC_LIGHT_LOG 定义在BUILD中， 
 // TRAFFIC_LOG 在工程中可以使用。比如 TRAFFIC_LOG << "counter:" << counter;
 // 其他的都不用改
 */

#if defined(PC_X86_RUN_FLAG) && (PC_X86_RUN_FLAG)
#define LOG_LP AINFO
#else
#define LOG_LP NULL_LOG
#endif

/*
  #if defined(TRAFFIC_LIGHT_LOG) && (TRAFFIC_LIGHT_LOG)
  #define TRAFFIC_LOG AINFO
  #else
  #define TRAFFIC_LOG NULL_LOG
  #endif
 */

#if defined(MAP_CHANGE_LOG) && (MAP_CHANGE_LOG)
#define GEOMETRY_LOG AINFO
#else
#define GEOMETRY_LOG NULL_LOG
#endif

#if defined(FML_TRALHT_LOG) && (FML_TRALHT_LOG)
#define FLOG_TLIGHT AINFO << " -fml-"
#else
#define FLOG_TLIGHT NULL_LOG
#endif

#if defined(SD_COARSE_MATCH) && (SD_COARSE_MATCH)
#define SD_COARSE_MATCH_LOG AINFO
#else
#define SD_COARSE_MATCH_LOG NULL_LOG
#endif

#if defined(SD_COARSE_MATCH_TYPE2) && (SD_COARSE_MATCH_TYPE2)
#define SD_COARSE_MATCH_TYPE2_LOG AINFO
#else
#define SD_COARSE_MATCH_TYPE2_LOG NULL_LOG
#endif

#if defined(SD_BEV_PROCESS) && (SD_BEV_PROCESS)
#define SD_BEV_PROCESS AINFO
#else
#define SD_BEV_PROCESS NULL_LOG
#endif

#if defined(SD_MERGE) && (SD_MERGE)
#define SD_MERGE_LOG AINFO
#else
#define SD_MERGE_LOG NULL_LOG
#endif

#if defined(SD_FINE_MATCH) && (SD_FINE_MATCH)
#define SD_FINE_MATCH_LOG AINFO
#else
#define SD_FINE_MATCH_LOG NULL_LOG
#endif

#if defined(STOP_LINE_MAPPING) && (STOP_LINE_MAPPING)
#define STOP_LINE_MAPPING_LOG AINFO
#else
#define STOP_LINE_MAPPING_LOG NULL_LOG
#endif

#if defined(SD_ENV_INFO_LOG) && (SD_ENV_INFO_LOG)
#define SD_ENV_INFO_LOG AINFO
#else
#define SD_ENV_INFO_LOG NULL_LOG
#endif

#if defined(SD_NAVIGATION_DEBUG) && (SD_NAVIGATION_DEBUG)
#define SD1LOG AINFO
#else
#define SD1LOG NULL_LOG
#endif

#if defined(FML_CROSS_LOG) && (FML_CROSS_LOG)
#define FLOG_CROSS AINFO << " -fml_cross-"
#else
#define FLOG_CROSS NULL_LOG
#endif

#if defined(XXF_LOG) && (XXF_LOG)
#define XLOG AINFO << " -xxf-"
#else
#define XLOG NULL_LOG
#endif

#if defined(XXF_LOG) && (XXF_LOG)
#define PPM_UPDATE 0
#else
#define PPM_UPDATE 0
#endif
// #if defined(XXF_LOG) && (XXF_LOG)
// #define LANE_REFINE_DEBUG 1
// #else
// #define LANE_REFINE_DEBUG 0
// #endif
// #if defined(XXF_LOG) && (XXF_LOG)
// #define EDGE_MERGER_DEBUG 1
// #else
// #define EDGE_MERGER_DEBUG 0
// #endif
//LANE_CLUSTER OCC_DEBUG 同时打开
#if defined(XXF_LOG) && (XXF_LOG)
#define LANE_CLUSTER 0
#else
#define LANE_CLUSTER 0
#endif
#if defined(XXF_LOG) && (XXF_LOG)
#define OCC_DEBUG 0
#else
#define OCC_DEBUG 0
#endif
#if defined(XXF_LOG) && (XXF_LOG)
#define MAP_SMOOTH_DEBUG 1
#else
#define MAP_SMOOTH_DEBUG 0
#endif

// #if defined(XXF_LOG) && (XXF_LOG)
// #define XLOG AINFO << " XLOG:"
// AINFO << "#####################"
// #else
// AINFO << "XXXXXXXXXXXXXXXXXXXXXXXXX"
// #endif



// NOLINTEND
#endif
