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
 * @file routing_map_debug.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-10
 */
#ifndef ROUTING_MAP_DEBUG_H_
#define ROUTING_MAP_DEBUG_H_
#include <cmath>
#include <cstddef>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <message/env_model/navigation/navigation.h>

#include <base/params_manager/params_manager.h>
#include "lib/message/env_model/routing_map/routing_map.h"
#include "modules/perception/env/src/lib/message/env_model/routing_map/routing_map.h"
#include "modules/perception/env/src/lib/message/sensor/camera/bev_lane/bev_lane.h"

namespace cem::message::env_model {

/** 
 * @brief 计算两个弧度角度的最小差值（考虑 -π 到 π 的环绕）
 * @param theta1 第一个角度（弧度），范围 [-π, π]
 * @param theta2 第二个角度（弧度），范围 [-π, π]
 * @return 角度差（弧度），范围 [-π, π]
 *
 * @example
 *   angleDiff(M_PI, -M_PI)     -> 0.0      （实际等效角度相同）
 *   angleDiff(0.75*M_PI, -M_PI)-> 0.25*M_PI（最短路径差）
 */
inline double angleDifference(double theta1, double theta2) {
  double diff = theta2 - theta1;
  if (diff > M_PI) {
    // diff -= 2 * M_PI;
  } else if (diff < -M_PI) {
    // diff += 2 * M_PI;
  }
  return diff * (180 / M_PI);
}

inline std::string StrLaneType(const LaneType &lane_type) {
  switch (lane_type) {
    case LaneType::LANE_UNKNOWN:
      return "UNKNOWN";
    case LaneType::LANE_NORMAL:
      return "NORMAL";
    case LaneType::LANE_ACC:
      return "ACC";
    case LaneType::LANE_DEC:
      return "DEC";
    case LaneType::LANE_RAMP:
      return "RAMP";
    case LaneType::LANE_EMERGENCY:
      return "EMERGENCY";
    case LaneType::LANE_ACC_DCC:
      return "ACC_DCC";
    case LaneType::LANE_BUS_NORMAL:
      return "BUS_NORMAL";
    case LaneType::LANE_HOV_NORMAL:
      return "HOV_NORMAL";
    case LaneType::LANE_NON_MOTOR:
      return "NON_MOTOR";
    case LaneType::LANE_LEFT_WAIT:
      return "LEFT_WAIT";
    case LaneType::LANE_VIRTUAL_COMMON:
      return "VIRTUAL_COMMON";
    case LaneType::LANE_VIRTUAL_JUNCTION:
      return "VIRTUAL_JUNCTION";
    case LaneType::LANE_ROUND_ABOUT:
      return "ROUND_ABOUT";
    case LaneType::LANE_REVERSIBLE:
      return "REVERSIBLE";
    case LaneType::LANE_VARIABLE_TURN:
      return "VARIABLE_TURN";
    case LaneType::LANE_HARBOR_STOP:
      return "HARBOR_STOP";
    case LaneType::LANE_ENTRY:
      return "LANE_ENTRY";
    case LaneType::LANE_EXIT:
      return "LANE_EXIT";
    case LaneType::LANE_DIVERSION:
      return "LANE_DIVERSION";
    case LaneType::LANE_U_TURN_LANE:
      return "LANE_U_TURN_LANE";
    case LaneType::LANE_RIGHT_TURN_LANE:
      return "LANE_RIGHT_TURN_LANE";
    case LaneType::LANE_RIGHT_TURN_AREA:
      return "LANE_RIGHT_TURN_AREA";
    case LaneType::LANE_U_TURN_AREA:
      return "LANE_U_TURN_AREA";
    case LaneType::LANE_NO_TURN_AREA:
      return "LANE_NO_TURN_AREA";
    case LaneType::LANE_VIRTUAL_CONNECTED_LANE:
      return "LANE_VIRTUAL_CONNECTED_LANE";
    case LaneType::LANE_PARKING:
      return "LANE_PARKING";
    case LaneType::LANE_TOLLBOOTH:
      return "LANE_TOLLBOOTH";
    case LaneType::LANE_OBSTACLE:
      return "LANE_OBSTACLE";
    case LaneType::LANE_MIXED_TOLL:
      return "LANE_MIXED_TOLL";
    case LaneType::LANE_REVERSIBLE_CONNECTION_LANE:
      return "LANE_REVERSIBLE_CONNECTION_LANE";
    case LaneType::LANE_ENTRANCE_OR_EXIT_LANE:
      return "LANE_ENTRANCE_OR_EXIT_LANE";
    case LaneType::LANE_LEFT_TURN_LANE:
      return "LANE_LEFT_TURN_LANE";
    case LaneType::LANE_CHECKPOINT_LANE:
      return "LANE_CHECKPOINT_LANE";
    case LaneType::LANE_USE_THE_LEFT_TURN_LANE:
      return "LANE_USE_THE_LEFT_TURN_LANE";
    case LaneType::LANE_BRT:
      return "LANE_BRT";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(lane_type));
  }
}

inline std::string StrTurnType(const TurnType &turn_type) {
  switch (turn_type) {
    case TurnType::NO_TURN:
      return "NO_TURN";
    case TurnType::LEFT_TURN:
      return "LEFT_TURN";
    case TurnType::RIGHT_TURN:
      return "RIGHT_TURN";
    case TurnType::U_TURN:
      return "U_TURN";
    case TurnType::STRAIGHT_AND_LEFT_TURN:
      return "STRAIGHT_AND_LEFT";
    case TurnType::STRAIGHT_AND_RIGHT_TURN:
      return "STRAIGHT_AND_RIGHT";
    case TurnType::STRAIGHT_AND_U_TURN:
      return "STRAIGHT_AND_U";
    case TurnType::LEFT_TURN_AND_U_TURN:
      return "LEFT_AND_U";
    case TurnType::RIGHT_TURN_AND_U_TURN:
      return "RIGHT_AND_U";
    case TurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN:
      return "STRAIGHT_LEFT_RIGHT";
    case TurnType::LEFT_TURN_AND_RIGHT_TURN:
      return "LEFT_AND_RIGHT";
    case TurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN:
      return "STRAIGHT_LEFT_U";
    case TurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN:
      return "STRAIGHT_RIGHT_U";
    case TurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN:
      return "LEFT_RIGHT_U";
    case TurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN:
      return "ALL_TURNS";
    case TurnType::OTHER_UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(turn_type));
  }
}

inline std::string StrNoneOddType(const NoneOddType &nodd_type) {
  switch (nodd_type) {
    case NoneOddType::NODD_NONE:
      return "NONE";
    case NoneOddType::NORMAL_NOT_ODD:
      return "NORMAL";
    case NoneOddType::NODD_LOW_PRECISION:
      return "LOW_PRECISION";
    case NoneOddType::NODD_LANE_BOUNDARY_CHANGE:
      return "BOUNDARY_CHANGE";
    case NoneOddType::NODD_LANE_BOUNDARY_LOSS_WIDTH_ABNORMAL:
      return "BOUNDARY_WIDTH_ABNORMAL";
    case NoneOddType::NODD_LANE_NUM_INCREASE:
      return "LANE_NUM_INC";
    case NoneOddType::NODD_LANE_NUM_DECREASE:
      return "LANE_NUM_DEC";
    case NoneOddType::NODD_LANE_TYPE_CHANGE:
      return "LANE_TYPE_CHANGE";
    case NoneOddType::NODD_CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_NOT_DEGRADATION:
      return "CONSTR_OPPOSITE_NO_DEGRADE";
    case NoneOddType::NODD_CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_DEGRADATION:
      return "CONSTR_OPPOSITE_DEGRADE";
    case NoneOddType::NODD_CONSTRUCTION_SAME_DIR_CHANGE_WAY_NOT_DEGRADATION:
      return "CONSTR_SAME_NO_DEGRADE";
    case NoneOddType::NODD_CONSTRUCTION_SAME_DIR_CHANGE_WAY_DEGRADATION:
      return "CONSTR_SAME_DEGRADE";
    case NoneOddType::NODD_CONSTRUCTION_CLOSE_WAY:
      return "CONSTR_CLOSE_WAY";
    case NoneOddType::NODD_CONSTRUCTION_CLOSE_WAY_TO_NOT_HIGHWAY_DEGRADATION:
      return "CONSTR_CLOSE_DEGRADE";
    case NoneOddType::NODD_RAMP_INCREASE:
      return "RAMP_INC";
    case NoneOddType::NODD_TRAFFIC_CONE:
      return "TRAFFIC_CONE";
    case NoneOddType::NODD_WATER_SAFETY_BARRIER:
      return "WATER_BARRIER";
    case NoneOddType::NODD_UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(nodd_type));
  }
}

inline std::string StrLightStatus(const LightStatus &status) {
  switch (status) {
    case LightStatus::NONE_LIGHT:
      return "NONE";
    case LightStatus::GREEN_LIGHT:
      return "GREEN";
    case LightStatus::YELLOW_LIGHT:
      return "YELLOW";
    case LightStatus::RED_LIGHT:
      return "RED";
    case LightStatus::UNKNOWN_LIGHT:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(status));
  }
}

inline std::string StrSplitTopologyExtendType(const cem::message::sensor::SplitTopoExtendType &topology) {
  switch (topology) {
    case cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_NONE:
      return "SPLIT_NONE";
    case cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_LEFT:
      return "SPLIT_LEFT";
    case cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_RIGHT:
      return "SPLIT_RIGHT";
    case cem::message::sensor::SplitTopoExtendType::TOPOLOGY_SPLIT_UNKNOWN:
      return "SPLIT_UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(topology));
  }
}

inline std::string StrSplitTopology(const SplitTopology &topology) {
  switch (topology) {
    case SplitTopology::TOPOLOGY_SPLIT_NONE:
      return "NONE";
    case SplitTopology::TOPOLOGY_SPLIT_LEFT:
      return "LEFT";
    case SplitTopology::TOPOLOGY_SPLIT_RIGHT:
      return "RIGHT";
    case SplitTopology::TOPOLOGY_SPLIT_UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(topology));
  }
}

inline std::string StrSDRoadClass(const SDRoadClass &sd_road_class) {
  switch (sd_road_class) {
    case SDRoadClass::SD_HIGHWAY:
      return "HIGHWAY";
    case SDRoadClass::SD_CITY_FAST_WAY:
      return "CITY_FAST_WAY";
    case SDRoadClass::SD_NATIONAL_ROAD:
      return "NATIONAL";
    case SDRoadClass::SD_PROVINCIAL_ROAD:
      return "PROVINCIAL";
    case SDRoadClass::SD_COUNTY_ROAD:
      return "COUNTY";
    case SDRoadClass::SD_TOWNSHIP_ROAD:
      return "TOWNSHIP";
    case SDRoadClass::SD_OTHER_ROAD:
      return "OTHER";
    case SDRoadClass::SD_LEVEL_9_ROAD:
      return "LEVEL_9";
    case SDRoadClass::SD_FERRY:
      return "FERRY";
    case SDRoadClass::SD_WALY_WAY:
      return "WALKWAY";
    case SDRoadClass::SD_INVALID:
      return "INVALID";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(sd_road_class));
  }
}

inline std::string StrMergeTopologyExtendType(const cem::message::sensor::MergeTopoExtendType &topology) {
  switch (topology) {
    case cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_NONE:
      return "MERGE_NONE";
    case cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_LEFT:
      return "MERGE_LEFT";
    case cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_RIGHT:
      return "MERGE_RIGHT";
    case cem::message::sensor::MergeTopoExtendType::TOPOLOGY_TO_BE_MERGED:
      return "TO_BE_MERGED";
    case cem::message::sensor::MergeTopoExtendType::TOPOLOGY_MERGE_UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(topology));
  }
}

inline std::string StrMergeTopology(const MergeTopology &topology) {
  switch (topology) {
    case MergeTopology::TOPOLOGY_MERGE_NONE:
      return "NONE";
    case MergeTopology::TOPOLOGY_MERGE_LEFT:
      return "LEFT";
    case MergeTopology::TOPOLOGY_MERGE_RIGHT:
      return "RIGHT";
    case MergeTopology::TOPOLOGY_TO_BE_MERGED:
      return "TO_BE_MERGED";
    case MergeTopology::TOPOLOGY_MERGE_UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(topology));
  }
}

inline std::string StrLinkType(uint32_t link_type) {
  static constexpr std::pair<SDLinkTypeMask, const char *> flag_names[] = {
      {SDLinkTypeMask::SDLT_RING, "RING"},
      {SDLinkTypeMask::SDLT_NOATTR, "NOATTR"},
      {SDLinkTypeMask::SDLT_MAINSEPARATE, "MAINSEPARATE"},
      {SDLinkTypeMask::SDLT_JCT, "JCT"},
      {SDLinkTypeMask::SDLT_IC, "IC"},
      {SDLinkTypeMask::SDLT_PARK, "PARK"},
      {SDLinkTypeMask::SDLT_SERVICE, "SERVICE"},
      {SDLinkTypeMask::SDLT_BRIDGE, "BRIDGE"},
      {SDLinkTypeMask::SDLT_WALKSTREET, "WALKSTREET"},
      {SDLinkTypeMask::SDLT_CROSSLINK, "CROSSLINK"},
      {SDLinkTypeMask::SDLT_SIDEROAD, "SIDEROAD"},
      {SDLinkTypeMask::SDLT_RAMP, "RAMP"},
      {SDLinkTypeMask::SDLT_CLOSEDROAD, "CLOSEDROAD"},
      {SDLinkTypeMask::SDLT_UNDEFINEDTRAFFICAREA, "UNDEFINEDTRAFFICAREA"},
      {SDLinkTypeMask::SDLT_POICONNECTION, "POICONNECTION"},
      {SDLinkTypeMask::SDLT_TUNNEL, "TUNNEL"},
      {SDLinkTypeMask::SDLT_RA_FOOTWAY, "RA_FOOTWAY"},
      {SDLinkTypeMask::SDLT_BUS, "BUS"},
      {SDLinkTypeMask::SDLT_RIGHTTURN, "RIGHTTURN"},
      {SDLinkTypeMask::SDLT_SCENICROAD, "SCENICROAD"},
      {SDLinkTypeMask::SDLT_INAREAROAD, "INAREAROAD"},
      {SDLinkTypeMask::SDLT_LEFTTURN, "LEFTTURN"},
      {SDLinkTypeMask::SDLT_UTURN, "UTURN"},
      {SDLinkTypeMask::SDLT_MAINSIDECONNECT, "MAINSIDECONNECT"},
      {SDLinkTypeMask::SDLT_DUMMYLINK, "DUMMYLINK"},
      {SDLinkTypeMask::SDLT_PARKLINK, "PARKLINK"}};

  if (link_type == 0) {
    return "INVALID";
  }

  std::vector<std::string> active_flags;
  for (const auto &[mask, name] : flag_names) {
    if ((link_type & static_cast<uint32_t>(mask)) == static_cast<uint32_t>(mask)) {
      active_flags.emplace_back(name);
    }
  }

  if (active_flags.empty()) {
    return fmt::format("UNDEFINED(0x{:08X} raw:{})", link_type, link_type);
  }
  if (active_flags.size() > 1) {
    return fmt::format("{} raw-link-type:{}", fmt::join(active_flags, "|"), link_type);
  }
  return fmt::format("{} ", fmt::join(active_flags, "|"));
}

inline std::string StrRoadClass(const RoadClass &road_class) {
  switch (road_class) {
    case RoadClass::UNKNOWN:
      return "UNKNOWN";
    case RoadClass::EXPRESSWAY:
      return "EXPRESSWAY";
    case RoadClass::URBAN_EXPRESSWAY:
      return "URBAN_EXPRESSWAY";
    case RoadClass::NATION_ROAD:
      return "NATION";
    case RoadClass::PROVINCE_ROAD:
      return "PROVINCE";
    case RoadClass::COUNTRY_ROAD:
      return "COUNTRY";
    case RoadClass::TOWN_ROAD:
      return "TOWN";
    case RoadClass::SPECIAL_ROAD:
      return "SPECIAL";
    case RoadClass::WALK_ROAD:
      return "WALK";
    case RoadClass::PEOPLE_FERRY:
      return "PEOPLE_FERRY";
    case RoadClass::FERRY:
      return "FERRY";
    case RoadClass::OTHERS:
      return "OTHERS";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(road_class));
  }
}

inline std::string StrV2RoadClassType(const V2RoadClassType &road_class) {
  switch (road_class) {
    case V2RoadClassType::UNKNOWN_ROAD:
      return "UNKNOWN_ROAD";
    case V2RoadClassType::HIGH_WAY_ROAD:
      return "HIGH_WAY_ROAD";
    case V2RoadClassType::EXPRESS_WAY_ROAD:
      return "EXPRESS_WAY_ROAD";
    case V2RoadClassType::NATIOANL_ROAD:
      return "NATIOANL_ROAD";
    case V2RoadClassType::PROVINCIAL_ROAD:
      return "PROVINCIAL_ROAD";
    case V2RoadClassType::MAIN_ROAD:
      return "MAIN_ROAD";
    case V2RoadClassType::SUB_ROAD:
      return "SUB_ROAD";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(road_class));
  }
}

inline std::string StrV2TurnType(const V2TurnInfo::V2TurnType &turn_type) {
  switch (turn_type) {
    case V2TurnInfo::V2TurnType::UNKNOWN:
      return "UNKNOWN";
    case V2TurnInfo::V2TurnType::LEFT:
      return "LEFT";
    case V2TurnInfo::V2TurnType::RIGHT:
      return "RIGHT";
    case V2TurnInfo::V2TurnType::STRAIGHT:
      return "STRAIGHT";
    case V2TurnInfo::V2TurnType::U_TURN_LEFT:
      return "U_TURN_LEFT";
    case V2TurnInfo::V2TurnType::U_TURN_RIGHT:
      return "U_TURN_RIGHT";
    case V2TurnInfo::V2TurnType::MERGE_LEFT:
      return "MERGE_LEFT";
    case V2TurnInfo::V2TurnType::MERGE_RIGHT:
      return "MERGE_RIGHT";
    case V2TurnInfo::V2TurnType::RAMP_LEFT:
      return "RAMP_LEFT";
    case V2TurnInfo::V2TurnType::RAMP_RIGHT:
      return "RAMP_RIGHT";
    case V2TurnInfo::V2TurnType::RAMP_STRAIGHT:
      return "RAMP_STRAIGHT";
    case V2TurnInfo::V2TurnType::RAMP_U_TURN_LEFT:
      return "RAMP_U_TURN_LEFT";
    case V2TurnInfo::V2TurnType::RAMP_U_TURN_RIGHT:
      return "RAMP_U_TURN_RIGHT";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(turn_type));
  }
}

inline std::string StrV2DetailTurnType(const V2TurnInfo::V2DetailTurnType &turn_type) {
  switch (turn_type) {
    case V2TurnInfo::V2DetailTurnType::NONE:
      return "NONE";
    case V2TurnInfo::V2DetailTurnType::TURN_LEFT:
      return "TURN_LEFT";
    case V2TurnInfo::V2DetailTurnType::TURN_RIGHT:
      return "TURN_RIGHT";
    case V2TurnInfo::V2DetailTurnType::SLIGHT_LEFT:
      return "SLIGHT_LEFT";
    case V2TurnInfo::V2DetailTurnType::SLIGHT_RIGHT:
      return "SLIGHT_RIGHT";
    case V2TurnInfo::V2DetailTurnType::TURN_HARD_RIGHT:
      return "TURN_HARD_RIGHT";
    case V2TurnInfo::V2DetailTurnType::UTURN:
      return "UTURN";
    case V2TurnInfo::V2DetailTurnType::CONTINUE:
      return "CONTINUE";
    case V2TurnInfo::V2DetailTurnType::TURN_RIGHT_ONLY:
      return "TURN_RIGHT_ONLY";
    case V2TurnInfo::V2DetailTurnType::UTURN_RIGHT:
      return "UTURN_RIGHT";
    case V2TurnInfo::V2DetailTurnType::LEFT_MERGE:
      return "LEFT_MERGE";
    case V2TurnInfo::V2DetailTurnType::RIGHT_MERGE:
      return "RIGHT_MERGE";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(turn_type));
  }
}

inline std::string StrSDPathType(const SDPathType &path_type) {
  switch (path_type) {
    case SDPathType::NONE:
      return "NONE";
    case SDPathType::SPLIT:
      return "SPLIT";
    case SDPathType::MERGE:
      return "MERGE";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(path_type));
  }
}

inline std::string StrSDDirectionType(const SDDirectionType &direction_type) {
  switch (direction_type) {
    case SDDirectionType::UNKNOWN:
      return "UNKNOWN";
    case SDDirectionType::BIDIRECTIONAL_PASSABLE:
      return "BIDIRECTIONAL";
    case SDDirectionType::FORWARD_ONLY:
      return "FORWARD";
    case SDDirectionType::REVERSE_ONLY:
      return "REVERSE";
    case SDDirectionType::INVALID:
      return "INVALID";

    // 处理未定义或未来新增值
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(direction_type));
  }
}

inline std::string StrV2TrafficFlowType(const V2TrafficFlow::V2TrafficFlowType &traffic_flow_type) {
  switch (traffic_flow_type) {
    case V2TrafficFlow::V2TrafficFlowType::SMOOTH_FLOW:
      return "SMOOTH_FLOW";
    case V2TrafficFlow::V2TrafficFlowType::SLOW_FLOW:
      return "SLOW_FLOW";
    case V2TrafficFlow::V2TrafficFlowType::JAMMED_FLOW:
      return "JAMMED_FLOW";
    case V2TrafficFlow::V2TrafficFlowType::SEVERE_JAMMED_FLOW:
      return "SEVERE_JAMMED_FLOW";
    case V2TrafficFlow::V2TrafficFlowType::UNKNOWN_FLOW:
      return "UNKNOWN_FLOW";
    default:
      return "INVALID";
  }
}

}  // namespace cem::message::env_model

namespace cem::fusion::navigation {
// 枚举类型转换为字符串的函数
inline std::string StrJunctionType(JunctionType type) {
  switch (type) {
    case JunctionType::Unknow:
      return "Unknow";
    case JunctionType::RampInto:
      return "RampInto";
    case JunctionType::ApproachRampInto:
      return "ApproachRampInto";
    case JunctionType::RampMerge:
      return "RampMerge";
    case JunctionType::ApproachRampMerge:
      return "ApproachRampMerge";
    case JunctionType::RampSplitLeft:
      return "RampSplitLeft";
    case JunctionType::RampSplitRight:
      return "RampSplitRight";
    case JunctionType::RampSplitMiddle:
      return "RampSplitMiddle";
    case JunctionType::MainRoadSplitLeft:
      return "MainRoadSplitLeft";
    case JunctionType::MainRoadSplitRight:
      return "MainRoadSplitRight";
    case JunctionType::MainRoadSplitMiddle:
      return "MainRoadSplitMiddle";
    default:
      return "Unknown(" + std::to_string(static_cast<int>(type)) + ")";
  }
}

inline std::string StrJunctionStateCity(const JunctionStateCity &state) {
  switch (state) {
    case JunctionStateCity::PASSING:
      return "PASSING";
    case JunctionStateCity::PASSED:
      return "PASSED";
    case JunctionStateCity::UNREACHED:
      return "UNREACHED";
    case JunctionStateCity::UNKNOWN:
      return "UNKNOWN";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(state));
  }
}

inline std::string StrJunctionTypeCity(const JunctionTypeCity &type) {
  switch (type) {
    case JunctionTypeCity::Unknown:
      return "Unknown";
    case JunctionTypeCity::RoadSplit:
      return "RoadSplit";
    case JunctionTypeCity::RoadMerge:
      return "RoadMerge";
    case JunctionTypeCity::CrossRoad:
      return "CrossRoad";
    case JunctionTypeCity::TJunction:
      return "TJunction";
    case JunctionTypeCity::SmallTJunction:
      return "SmallTJunction";
    case JunctionTypeCity::UTurn:
      return "Uturn";
    case JunctionTypeCity::InvalidJunction:
      return "InvalidJunction";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(type));
  }
}

inline std::string StrJunctionAction(const JunctionAction &action) {
  switch (action) {
    case JunctionAction::Unknown:
      return "Unknown";
    case JunctionAction::TurnLeft:
      return "TurnLeft";
    case JunctionAction::GoStraight:
      return "GoStraight";
    case JunctionAction::TurnRight:
      return "TurnRight";
    case JunctionAction::UTurn:
      return "UTurn";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(action));
  }
}

inline std::string StrDirectionSplitMerge(const DirectionSplitMerge &dir) {
  switch (dir) {
    case DirectionSplitMerge::Unknown:
      return "Unknown";
    case DirectionSplitMerge::Left:
      return "Left";
    case DirectionSplitMerge::Right:
      return "Right";
    case DirectionSplitMerge::Straight:
      return "Straight";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(dir));
  }
}

inline std::string StrRoadMainType(const RoadMainType &type) {
  switch (type) {
    case RoadMainType::RoadRamp:
      return "RoadRamp";
    case RoadMainType::RoadMain:
      return "RoadMain";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(type));
  }
}

inline std::string StrNaviMatchType(const NaviMatchType &match_type) {
  switch (match_type) {
    case NaviMatchType::Unknow:
      return "Unknow";
    case NaviMatchType::Type1:
      return "SmallTJunction";
    case NaviMatchType::Type2:
      return "RoadSplit_Ramp";
    case NaviMatchType::Type3:
      return "CoarseMatch";
    case NaviMatchType::Type4:
      return "FineMatch";
    case NaviMatchType::Type5:
      return "Type5-VirtualGuideLine";
    case NaviMatchType::Type6:
      return "Type6-ALL";
    case NaviMatchType::Type7:
      return "UTurn";
    case NaviMatchType::Type8:
      return "Protect";
    case NaviMatchType::Type9:
      return "RoadSplit_Straight";
    case NaviMatchType::Type10:
      return "GuideLanesWithSd";
    default:
      return fmt::format("UNDEFINED({})", static_cast<int>(match_type));
  }
}

inline std::string StrNaviMatchTypeHighway(const int &match_type) {
  switch (match_type) {
    case 1:
      return "all-push";
    case 2:
      return "ramp-into";
    case 3:
      return "split-left";
    case 4:
      return "split-middle";
    case 5:
      return "split-right";
    case 6:
      return "app-ramp-into";
    case 7:
      return "app-ramp-merge";
    case 8:
      return "ramp-merge";
    case 9:
      return "MainRoadSplitLeft";
    case 10:
      return "MainRoadSplitRight";
    case 11:
      return "MainRoadSplitMiddle";

    default:
      return fmt::format("UNDEFINED({})", match_type);
  }
}

}  // namespace cem::fusion::navigation

namespace fmt {
using cem::fusion::ParamsManager;
using cem::fusion::navigation::BevRouteInfo;
using cem::fusion::navigation::JunctionInfo;
using cem::fusion::navigation::JunctionInfoCity;
using cem::message::env_model::EnvInfo;
using cem::message::env_model::LaneInfo;
using cem::message::env_model::Point;
using cem::message::env_model::RouteInfo;
using cem::message::env_model::SDLaneGroupIndex;
using cem::message::env_model::SDSectionInfo;
using cem::message::env_model::TurnType;
using cem::message::sensor::BevLaneInfo;
using cem::message::sensor::BevLaneMarker;

template <>
struct formatter<SDLaneGroupIndex> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  // format 方法实现
  template <typename FormatContext>
  auto format(const SDLaneGroupIndex &sd_lane_group_index, FormatContext &ctx) const {
    return format_to(ctx.out(), "(id:{} offset:[{:.2f},{:.2f}])", sd_lane_group_index.id, sd_lane_group_index.start_range_offset,
                     sd_lane_group_index.end_range_offset);
  }
};

template <>
struct formatter<LaneInfo> : formatter<std::string_view> {
  // 解析格式说明符（这里支持空说明符）
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) {
    auto it = ctx.begin();
    if (it != ctx.end() && *it != '}') {
      throw format_error("invalid format specifier for LaneInfo");
    }
    return it;
  }

  template <typename FormatContext>
  auto format(const LaneInfo &lane_info, FormatContext &ctx) const {
    fmt::memory_buffer    buffer;
    constexpr const char *indent = "  ";

    // 条件显示辅助函数
    auto vector_cond_field = [&](const auto &vec, const char *name) {
      if (!vec.empty()) {
        fmt::format_to(buffer, "{}{}: {}", indent, name, fmt::join(vec, ", "));
      }
    };

    auto default_cond_field = [&](const auto &val, const char *name) {
      if (val != 0) {
        fmt::format_to(buffer, "{}{}: {}", indent, name, val);
      }
    };

    // 基础字段（始终显示）
    fmt::format_to(buffer, "LaneInfo:");
    fmt::format_to(buffer, "{}id: {}", indent, lane_info.id);
    default_cond_field(lane_info.section_id, "section_id");
    default_cond_field(lane_info.junction_id, "junction_id");
    default_cond_field(lane_info.left_lane_id, "left_lane_id");
    default_cond_field(lane_info.right_lane_id, "right_lane_id");

    // 条件字段
    vector_cond_field(lane_info.previous_lane_ids, "previous_lane_ids");
    vector_cond_field(lane_info.next_lane_ids, "next_lane_ids");
    vector_cond_field(lane_info.left_lane_boundary_ids, "left_lane_boundary_ids");
    vector_cond_field(lane_info.right_lane_boundary_ids, "right_lane_boundary_ids");
    vector_cond_field(lane_info.left_road_boundary_ids, "left_road_boundary_ids");
    vector_cond_field(lane_info.right_road_boundary_ids, "right_road_boundary_ids");

    // 其他固定字段
    fmt::format_to(buffer, "{}is_virtual: {:d}", indent, lane_info.is_virtual);
    fmt::format_to(buffer, "{}type: {}", indent, StrLaneType(lane_info.type));
    fmt::format_to(buffer, "{}length: {:.2f}m", indent, lane_info.length);
    // fmt::format_to(buffer, "{}none_odd_type: {}", indent, StrNoneOddType(lane_info.none_odd_type));
    fmt::format_to(buffer, "{}turn_type: {}", indent, StrTurnType(lane_info.turn_type));
    fmt::format_to(buffer, "{}plan_turn_type: {}", indent, StrTurnType(lane_info.plan_turn_type));
    // fmt::format_to(buffer, "{}light_status: {}", indent, StrLightStatus(lane_info.light_status));
    fmt::format_to(buffer, "{}split_topology: {}", indent, StrSplitTopology(lane_info.split_topology));
    fmt::format_to(buffer, "{}merge_topology: {}", indent, StrMergeTopology(lane_info.merge_topology));
    fmt::format_to(buffer, "{}speed_limit: {:.2f}m/s", indent, lane_info.speed_limit);

    // 特殊处理points字段
    if (!lane_info.points.empty()) {
      fmt::format_to(buffer, "{}points:", indent);
      constexpr const char *sub_indent = "    ";
      for (const auto &p : lane_info.points) {
        fmt::format_to(buffer, "{}{}", sub_indent, fmt::format("{}", p));
      }
    }

    // 其他条件字段
    vector_cond_field(lane_info.cross_walks, "cross_walks");
    vector_cond_field(lane_info.traffic_stop_lines, "traffic_stop_lines");
    fmt::format_to(buffer, "{}light_countdown: {}", indent, lane_info.light_countdown);

    // 输出到上下文（无额外换行）
    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buffer));
  };
};
// 高速路口格式化
#if 1
template <>
struct formatter<JunctionInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  // format 方法实现
  template <typename FormatContext>
  auto format(const JunctionInfo &info, FormatContext &ctx) const {
    fmt::memory_buffer buf;

    // 基础字段
    fmt::format_to(buf, "Junction_Info  junction_id:{}  junction_type:{} offset:{:.2f}  split_num:{} ", info.junction_id,
                   cem::fusion::navigation::StrJunctionType(info.junction_type), info.offset, info.split_num);

    // 车道信息
    fmt::format_to(buf, "main_road_lane_nums:{}  target_road_lane_nums:{} ", info.main_road_lane_nums, info.target_road_lane_nums);

    // 状态标志
    fmt::format_to(buf, "has_passed_flag:{}  has_effected_flag:{}", info.has_passed_flag, info.has_effected_flag);

    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buf));
  }
};

// 为JunctionInfo*指针类型添加格式化支持
template <>
struct formatter<cem::fusion::navigation::JunctionInfo *> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(cem::fusion::navigation::JunctionInfo *ptr, FormatContext &ctx) {
    if (!ptr)
      return format_to(ctx.out(), "nullptr");
    // 委托给对象特化
    return format_to(ctx.out(), "{}", *ptr);
  }
};
#endif

template <>
struct formatter<JunctionInfoCity> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  // format 方法实现
  template <typename FormatContext>
  auto format(const JunctionInfoCity &junction_city_info, FormatContext &ctx) const {
    fmt::memory_buffer buf;

    // 基础字段
    fmt::format_to(buf,
                   "Junction_Info  junction_id:{}  state:{} offset:{:.2f}  junction_type_city:{} action:{} match_arrow:{:d} main_num:{}  "
                   "targe_num:{} ",
                   junction_city_info.junction_id, cem::fusion::navigation::StrJunctionStateCity(junction_city_info.junction_state_city),
                   junction_city_info.offset, cem::fusion::navigation::StrJunctionTypeCity(junction_city_info.junction_type_city),
                   cem::fusion::navigation::StrJunctionAction(junction_city_info.junction_action), junction_city_info.need_match_arrow,
                   junction_city_info.main_road_lane_nums, junction_city_info.target_road_lane_nums);
    fmt::format_to(buf, "envelope_dis:{:.2f}  envelope_size:{}  is_valid:{:d}  is_dedicated_right_turn_lane:{:d}",
                   junction_city_info.distance_to_envelope, junction_city_info.junction_envelope_points.size(), junction_city_info.is_valid,
                   junction_city_info.is_dedicated_right_turn_lane);

    if (junction_city_info.junction_type_city == cem::fusion::navigation::JunctionTypeCity::RoadMerge ||
        junction_city_info.junction_type_city == cem::fusion::navigation::JunctionTypeCity::RoadSplit) {
      fmt::format_to(buf, " split_merge:{}", cem::fusion::navigation::StrDirectionSplitMerge(junction_city_info.split_merge_direction));
    }

    if (!junction_city_info.map_lane_arrows_plan.empty()) {
      fmt::format_to(buf, " turn_type:{{");
      for (size_t i = 0; i < junction_city_info.map_lane_arrows_plan.size(); ++i) {
        if (i != 0) {
          fmt::format_to(buf, ",");
        }
        fmt::format_to(buf, "{}", cem::message::env_model::StrTurnType(junction_city_info.map_lane_arrows_plan[i]));
      }
      fmt::format_to(buf, "}}");
    }
    fmt::format_to(buf, "  main_index:{}", junction_city_info.main_lane_indexs);

    if (!junction_city_info.succ_road_class.empty()) {
      fmt::format_to(buf, " succ_road_class:{{");
      for (size_t i = 0; i < junction_city_info.succ_road_class.size(); ++i) {
        if (i != 0) {
          fmt::format_to(buf, ",");
        }
        fmt::format_to(buf, "{}", cem::fusion::navigation::StrRoadMainType(junction_city_info.succ_road_class[i]));
      }
      fmt::format_to(buf, "}}");
    }

    if (!junction_city_info.prev_road_class.empty()) {
      fmt::format_to(buf, " prev_road_class:{{");
      for (size_t i = 0; i < junction_city_info.prev_road_class.size(); ++i) {
        if (i != 0) {
          fmt::format_to(buf, ",");
        }
        fmt::format_to(buf, "{}", cem::fusion::navigation::StrRoadMainType(junction_city_info.prev_road_class[i]));
      }
      fmt::format_to(buf, "}}");
    }
    fmt::format_to(buf, "  junction_ids:{}  split_direction_ids:{} time:{:.2f}", junction_city_info.junction_ids,
                   junction_city_info.split_direction_ids, junction_city_info.time);

    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buf));
  }
};

template <>
struct formatter<EnvInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const EnvInfo &env_info, FormatContext &ctx) const {
    fmt::memory_buffer buf;

    fmt::format_to(buf, "EnvInfo: {{\n");
    fmt::format_to(buf, "  v2_valid: {},\n", env_info.v2_valid);
    fmt::format_to(buf, "  v2_has_navigation: {},\n", env_info.v2_has_navigation);
    fmt::format_to(buf, "  v2_dist_to_ramp: {:.2f},\n", env_info.v2_dist_to_ramp);
    fmt::format_to(buf, "  v2_dist_to_toll: {:.2f},\n", env_info.v2_dist_to_toll);
    fmt::format_to(buf, "  v2_dist_to_tunnel: {:.2f},\n", env_info.v2_dist_to_tunnel);
    fmt::format_to(buf, "  v2_dist_to_subpath: {:.2f},\n", env_info.v2_dist_to_subpath);
    fmt::format_to(buf, "  v2_dist_to_split_routelanenum_dec: {:.2f},\n", env_info.v2_dist_to_split_routelanenum_dec);
    fmt::format_to(buf, "  is_switched_to_LD_: {},\n", env_info.is_switched_to_LD_);
    fmt::format_to(buf, "  v2_curvatures: [\n");
    for (const auto &curvature : env_info.v2_curvatures) {
      fmt::format_to(buf, "    {{ distance: {:.2f}, curvature: {:.2f} }},\n", curvature.distance, curvature.curvature);
    }
    fmt::format_to(buf, "  ],\n");

    fmt::format_to(buf, "  v2_road_classes: [\n");
    for (const auto &road_class : env_info.v2_road_classes) {
      fmt::format_to(buf, "    {{ start: {:.2f}, end: {:.2f}, road_class: {} }},\n", road_class.start, road_class.end,
                     StrV2RoadClassType(road_class.road_class));
    }
    fmt::format_to(buf, "  ],\n");

    fmt::format_to(buf, "  traffic_flows: [\n");
    for (const auto &traffic_flow : env_info.traffic_flows) {
      fmt::format_to(buf, "    {{ start_s: {:.2f}, end_s: {:.2f}, type: {} }},\n", traffic_flow.start_s, traffic_flow.end_s,
                     cem::message::env_model::StrV2TrafficFlowType(traffic_flow.type));
    }
    fmt::format_to(buf, "  ],\n");

    fmt::format_to(buf, "  v2_turn_info: [\n");
    for (const auto &turn_info : env_info.v2_turn_info) {
      fmt::format_to(buf,
                     "    {{ id: {}, turn_type: {}, detail_turnType: {}, v2_dist: {:.2f}, dist: {:.2f}, before_turn: {{ road_class: {}, "
                     "lane_num: {} }}, "
                     "after_turn: {{ "
                     "road_class: {}, lane_num: {} }} }},\n",
                     turn_info.id, StrV2TurnType(turn_info.turn_type), StrV2DetailTurnType(turn_info.detail_turn_type), turn_info.v2_dist,
                     turn_info.dist, StrV2RoadClassType(turn_info.before_turn.road_class), turn_info.before_turn.lane_num,
                     StrV2RoadClassType(turn_info.after_turn.road_class), turn_info.after_turn.lane_num);
    }
    fmt::format_to(buf, "  ]\n");
    fmt::format_to(buf, "}}");

    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buf));
  }
};

template <>
struct formatter<cem::message::sensor::BevLaneInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::sensor::BevLaneInfo &lane, FormatContext &ctx) const {
    std::string prev_ids  = lane.previous_lane_ids.empty() ? "None" : fmt::format("{}", fmt::join(lane.previous_lane_ids, ","));
    std::string next_ids  = lane.next_lane_ids.empty() ? "None" : fmt::format("{}", fmt::join(lane.next_lane_ids, ","));
    std::string left_ids  = lane.left_lane_id ? std::to_string(lane.left_lane_id) : "None";
    std::string right_ids = lane.right_lane_id ? std::to_string(lane.right_lane_id) : "None";
    return fmt::format_to(
        ctx.out(),
        "  Lane ID: {}, Length: {:.3f}, PointSize: {}, StartPointX: {:.4f}, StartPointY: {:.4f}, EndPointX: {:.4f}, EndPointY: {:.4f}, "
        "Prev IDs: {}, Next IDs: {}, Left IDs: {}, Right IDs: {}, MergeInfoExtend: {}, MergeTop: {}",
        lane.id, lane.length, lane.geos ? lane.geos->size() : 0, lane.geos && !lane.geos->empty() ? lane.geos->front().x() : 0.0,
        lane.geos && !lane.geos->empty() ? lane.geos->front().y() : 0.0, lane.geos && !lane.geos->empty() ? lane.geos->back().x() : 0.0,
        lane.geos && !lane.geos->empty() ? lane.geos->back().y() : 0.0, prev_ids, next_ids, left_ids, right_ids,
        lane.merge_info_extend.dis_to_merge, static_cast<int>(lane.merge_topo_extend));
  }
};
template <>
struct formatter<BevLaneMarker> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const BevLaneMarker &edge, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "  Edge ID: {}, PointSize: {}, StartPointX: {:.4f}, StartPointY: {:.4f}, EndPointX: {:.4f}, EndPointY: {:.4f}", edge.id,
        edge.geos ? edge.geos->size() : 0, edge.geos && !edge.geos->empty() ? edge.geos->front().x() : 0.0,
        edge.geos && !edge.geos->empty() ? edge.geos->front().y() : 0.0, edge.geos && !edge.geos->empty() ? edge.geos->back().x() : 0.0,
        edge.geos && !edge.geos->empty() ? edge.geos->back().y() : 0.0);
  }
};

template <>
struct formatter<cem::fusion::navigation::BevRouteInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::fusion::navigation::BevRouteInfo &route_info, FormatContext &ctx) const {
    auto out = ctx.out();

    out = fmt::format_to(out, "BevRouteInfo:\n");
    out = fmt::format_to(out, "  Filtered Non-Ego Lanes: {}\n", route_info.filtered_non_ego_lanes ? "true" : "false");
    out = fmt::format_to(out, "  Sections size: {}:\n", route_info.sections.size());
    for (const auto &section : route_info.sections) {
      std::string lane_ids_str = fmt::format("{}", fmt::join(section.lane_ids, ","));
      out                      = fmt::format_to(out,
                                                "    Section ID: {}, Length: {:.3f}, StartOffset: {:.3f}, EndOffset: {:.3f}, LaneNum: {}, LaneIDs: [{}], "
                                                                     "LeftEdgeDist: {:.3f}, RightEdgeDist: {:.3f}\n",
                                                section.id, section.length, section.start_range_offset, section.end_range_offset, section.lane_num, lane_ids_str,
                                                section.left_edge_distance, section.right_edge_distance);
    }

    out = fmt::format_to(out, "  Separators size: {}:\n", route_info.separators.size());
    for (size_t i = 0; i < route_info.separators.size(); ++i) {
      const auto &sep = route_info.separators[i];

      size_t count = std::min({sep.edge_ids.size(), sep.types.size(), sep.edge_points.size()});
      out          = fmt::format_to(out, "    Separator Group {}: size = {}\n", i, count);
      for (size_t j = 0; j < count; ++j) {
        const auto &pt_pair = sep.edge_points[j];
        out =
            fmt::format_to(out, "      Edge ID: {}, Type: {}, StartPoint: ({:.4f}, {:.4f}), EndPoint: ({:.4f}, {:.4f})\n", sep.edge_ids[j],
                           static_cast<int>(sep.types[j]), pt_pair.first.x(), pt_pair.first.y(), pt_pair.second.x(), pt_pair.second.y());
      }
    }

    return out;
  }
};

template <>
struct formatter<cem::message::env_model::SDSectionInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::env_model::SDSectionInfo &section, FormatContext &ctx) const {
    fmt::memory_buffer buf;

    std::string pre_str =
        section.predecessor_section_id_list.empty() ? "None" : fmt::format("{}", fmt::join(section.predecessor_section_id_list, "&"));
    std::string suc_str =
        section.successor_section_id_list.empty() ? "None" : fmt::format("{}", fmt::join(section.successor_section_id_list, "&"));
    fmt::format_to(buf,
                   "  [{}] Section ID: {}, Is MPP: {}, Lane Num: {}, Link Type: {}, Road Class: {}, Length: {:.2f}m, Predecessor IDs: {}, "
                   "Successor IDs: {}, Has Points: {}\n",
                   "MPP", section.id, section.is_mpp_section ? "Yes" : "No", section.lane_num,
                   cem::message::env_model::StrLinkType(section.link_type), cem::message::env_model::StrSDRoadClass(section.road_class),
                   section.length, pre_str, suc_str, section.points ? section.points->size() : 0);

    fmt::format_to(buf, "    Lane Groups number in Section: {}\n", section.lane_group_idx.size());
    if (section.lane_group_idx.empty()) {
      fmt::format_to(buf, "      No Lane Groups available.\n");
    } else {
      for (const auto &lg_idx : section.lane_group_idx) {
        fmt::format_to(buf, "      Lane Group ID: {}, Start Offset: {:.2f}, End Offset: {:.2f}\n", lg_idx.id, lg_idx.start_range_offset,
                       lg_idx.end_range_offset);
        const auto *lane_group = INTERNAL_PARAMS.sd_map_data.GetSDLaneGroupInfoById(lg_idx.id);
        if (lane_group) {
          std::string suc_str = lane_group->successor_lane_group_ids.empty()
                                    ? "None"
                                    : fmt::format("{}", fmt::join(lane_group->successor_lane_group_ids, "&"));
          std::string pre_str = lane_group->predecessor_lane_group_ids.empty()
                                    ? "None"
                                    : fmt::format("{}", fmt::join(lane_group->predecessor_lane_group_ids, "&"));

          fmt::format_to(
              buf,
              "        Lane Group Details - Length: {:.2f}m, Lane Num: {}, Successor Lane Group IDs: {}, Predecessor Lane Group IDs: {}\n",
              lane_group->length, lane_group->lane_num, suc_str, pre_str);

          fmt::format_to(buf, "        Lanes in Group: {}\n", lane_group->lane_info.size());
          for (size_t i = 0; i < lane_group->lane_info.size(); ++i) {
            const auto &lane     = lane_group->lane_info[i];
            std::string next_str = lane.next_lane_ids.empty() ? "None" : fmt::format("{}", fmt::join(lane.next_lane_ids, "&"));
            std::string prev_str = lane.previous_lane_ids.empty() ? "None" : fmt::format("{}", fmt::join(lane.previous_lane_ids, "&"));

            fmt::format_to(buf,
                           "          Lane[{}] - ID: {}, Next Lane IDs: {}, Previous Lane IDs: {}, Split Topology: {}, Merge Topology: {}, "
                           "Type: {}, Plan_turn_type: {}\n",
                           i, lane.id, next_str, prev_str, cem::message::env_model::StrSplitTopology(lane.split_topology),
                           cem::message::env_model::StrMergeTopology(lane.merge_topology), cem::message::env_model::StrLaneType(lane.type),
                           StrTurnType(lane.plan_turn_type));
          }
        } else {
          fmt::format_to(buf, "        Lane Group Details: Not Found\n");
        }
      }
    }

    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buf));
  }
};

template <>
struct formatter<cem::message::env_model::SDRouteInfo> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const cem::message::env_model::SDRouteInfo &route_info, FormatContext &ctx) const {
    fmt::memory_buffer buf;

    fmt::format_to(buf, "[SDMAP DATA] ============== SD Map Data Debug Start ==============\n");
    fmt::format_to(buf, "[SDMAP DATA] Navigation Start - Section ID: {}, S-Offset: {:.2f}\n", route_info.navi_start.section_id,
                   route_info.navi_start.s_offset);

    // 主路径段 (MPP)
    fmt::format_to(buf, "[SDNOA DEBUG] Main Path Sections (MPP):\n");
    if (route_info.mpp_sections.empty()) {
      fmt::format_to(buf, "  No MPP sections available.\n");
    } else {
      uint64_t ego_section_id = route_info.navi_start.section_id;
      auto     it             = std::find_if(route_info.mpp_sections.begin(), route_info.mpp_sections.end(),
                                             [ego_section_id](const auto &section) { return section.id == ego_section_id; });
      if (it != route_info.mpp_sections.end()) {
        for (auto section_it = it; section_it != route_info.mpp_sections.end(); ++section_it) {
          fmt::format_to(buf, "{}", *section_it);  // 直接调用 SDSectionInfo 的 formatter
        }
      } else {
        fmt::format_to(buf, "  Ego section ID {} not found in MPP sections.\n", ego_section_id);
      }
    }

    // 子路径 (按需打印)
    /*
    fmt::format_to(buf, "[SDNOA DEBUG] SubPaths:\n");
    if (route_info.subpaths.empty()) {
      fmt::format_to(buf, "  No SubPaths available.\n");
    } else {
      for (const auto &sub_path : route_info.subpaths) {
        fmt::format_to(buf, "[SDNOA DEBUG] SubPath - Enter Section ID: {}, Path Type: {}\n",
                       sub_path.enter_section_id, cem::message::env_model::StrSDPathType(sub_path.path_type));
        for (const auto& section : sub_path.sections) {
          fmt::format_to(buf, "{}", section);  // 直接调用 SDSectionInfo 的 formatter
        }
      }
    }
    */
    fmt::format_to(buf, "[SDNOA DEBUG] ============== SD Map Data Debug End ==============\n");

    return fmt::format_to(ctx.out(), "{}", fmt::to_string(buf));
  }
};

}  // namespace fmt
#endif
