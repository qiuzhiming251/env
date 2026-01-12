#ifndef ENVINFOPROCESSOR_H
#define ENVINFOPROCESSOR_H

#include <memory>
#include <vector>
#include <base/params_manager/params_defination/internal_params.h>
#include "SdNavigationHighway.h"
#include "base/params_manager/params_manager.h"
#include "base/sensor_data_manager.h"
#include "message/env_model/routing_map/routing_map.h"
#include "message/env_model/stop_line/stop_line.h"

namespace cem {
namespace fusion {
namespace navigation {
using cem::message::env_model::EnvInfo;
using cem::message::env_model::MapEvent;
using cem::message::env_model::RoadClass;
using cem::message::env_model::RouteInfo;
using cem::message::env_model::RoutingMap;
using cem::message::env_model::SDRouteInfo;
using cem::message::env_model::SDSectionInfo;
using cem::message::env_model::SDSubPath;
using cem::message::env_model::SectionInfo;

class EnvInfoProcessor {
 public:
  EnvInfoProcessor();

  void Process(const std::vector<cem::message::env_model::StopLine> &stop_line_obj);

  // 获取处理后的 EnvInfo
  const EnvInfo *GetEnvInfo() const;

  void SetIsOnHighway(bool is_on_highway);

 private:
  // 检查是否在匝道内
  bool CheckIfInRamp();
  /*
  * @brief 当前是否在隧道内
  * @return 
*/
  bool EgoIsInTunnel();

  // 判断 SDsection 是否为匝道
  bool IsRampSection(const SDSectionInfo &section_info);

  // 判断 LDsection 是否为匝道
  bool IsRampSection(const SectionInfo &ld_section);

  // 计算到子路径的距离
  double CalculateDistanceToSubpath();

  // 计算到车道数量减少的分叉点的距离
  double CalculateDistanceToSplitRouteLaneNumDec();

  // 获取自车的全局 S 坐标
  double GetEgoGlobalS(const SDRouteInfo &sd_route) const;

  // 获取 section 的起始 S 坐标
  double GetSectionStartS(const SDSectionInfo &section, const SDRouteInfo &sd_route) const;

  // 获取车道组中非紧急车道的数量
  int GetLaneNumWithoutEmergency(const SDLaneGroupInfo *lanegroup);

  // 获取 section 中非紧急车道的最小数量
  int GetSDSectionMinLaneNumNOTEmergency(const SDSectionInfo &section_info);

  std::vector<V2RoadClass> CalculateV2RoadClasses() const;

  V2RoadClassType ConvertSDRoadClassToV2RoadClassType(SDRoadClass sd_road_class) const;

  bool UpdateIsSwitchedToLD(const MapEvent &map_event);

  // 存储处理后的 EnvInfo
  std::unique_ptr<EnvInfo> env_info_;

  std::vector<V2TurnInfo> CalculateV2TurnInfo(const std::vector<cem::message::env_model::StopLine> &stop_line_obj);

  void          PrintEnvInfo(const EnvInfo &env_info) const;
  UTurnPosition FindUTurnLanePosition(const std::vector<TurnType> &lane_arrows);

  std::pair<V2TurnInfo::V2TurnType, V2TurnInfo::V2DetailTurnType> GetTurnInfoForRoadSplit(JunctionAction action);

  std::vector<V2TurnInfo> ProcessCityJunctions(const std::vector<cem::message::env_model::StopLine> &stop_line_obj,
                                               const std::vector<navigation::JunctionInfoCity>      &city_junctions);

  std::vector<V2TurnInfo> ProcessHighwayJunctions(const std::vector<cem::message::env_model::StopLine> &stop_line_obj,
                                                  const std::vector<navigation::JunctionInfo>          &highway_junctions);

  inline bool IsHighwayMode() const { return (env_info_ && env_info_->is_on_highway_); }

  double          GetEgoGlobalS(const RouteInfo &ld_route) const;                             // LD 版
  double          GetSectionStartS(const SectionInfo &sec, const RouteInfo &ld_route) const;  // LD 版
  V2RoadClassType ConvertLDRoadClassToV2RoadClassType(RoadClass rc) const;                    // LD 版
  int             CountNonEmergencyLanesLD(uint64_t section_id);

  // 新增成员变量用于跟踪切图标志位
  bool   is_switched_to_LD_{false};                // 切图标志位
  bool   is_set_by_highway_complex_scene_{false};  // 是否由高速复杂场景切图逻辑设置
  double last_remain_dis_{0.0};                    // 上一时刻的 remain_dis
  double accumulated_distance_{0.0};               // 累计行驶距离
  bool   is_accumulated_valid_{false};

  double last_ego_s_on_route_ = std::numeric_limits<double>::quiet_NaN();
  bool   last_in_ramp_        = false;
  bool   last_on_highway_     = false;

  // 常量定义
  static constexpr double DEFAULT_LARGE_DISTANCE          = 10000.0;  // 默认大距离
  static constexpr double LD_MAP_ACTIVATION_DIST_THRESH   = 800.0;    // 进入匝道前切换距离
  static constexpr double LD_MAP_DEACTIVATION_DIST_THRESH = 500.0;    // 驶出匝道后切换距离
  static constexpr double MAX_REMAIN_DIS_JUMP             = 50.0;     // remain_dis 跳变阈值
  static constexpr double MERGE_THRESHOLD                 = 20.0;

  // 高速复杂场景常量
  static constexpr double HIGHWAY_CONSECUTIVE_JUNCTION_MAX_GAP = 250.0;
  static constexpr double HIGHWAY_CUT_WINDOW_BEFORE            = 500.0;
  static constexpr double HIGHWAY_CUT_WINDOW_AFTER             = 100.0;
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // ENVINFOPROCESSOR_H