#pragma once

#include <string>
#include <vector>
#include "message/common/header.h"

namespace cem {
namespace message {
namespace env_model {

// 枚举定义
enum OddInfo_Type {
  CONSTRUCTION       = 1,
  REVERSIBLE_LANE    = 2,
  ROUND_ISLAND       = 3,
  FUNC_TEST_QUESTION = 4,
  LANE_IS_TOO_NARROW = 5,
  LANE_IS_TOO_WIDE   = 6,
  NRP_MINUS_ODD      = 7
};

enum ValueType {
  NORMAL_NOT_ODD                                       = 1,
  LOW_PRECISION                                        = 22,
  LANE_BOUNDARY_CHANGE                                 = 100,
  LANE_BOUNDARY_LOSS_WIDTH_ABNORMAL                    = 101,
  LANE_NUM_INCREASE                                    = 102,
  LANE_NUM_DECREASE                                    = 103,
  LANE_TYPE_CHANGE                                     = 104,
  CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_NOT_DEGRADATION = 105,
  CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_DEGRADATION     = 106,
  CONSTRUCTION_SAME_DIR_CHANGE_WAY_NOT_DEGRADATION     = 107,
  CONSTRUCTION_SAME_DIR_CHANGE_WAY_DEGRADATION         = 108,
  CONSTRUCTION_CLOSE_WAY                               = 109,
  CONSTRUCTION_CLOSE_WAY_TO_NOT_HIGHWAY_DEGRADATION    = 110,
  RAMP_INCREASE                                        = 111,
  TRAFFIC_CONE                                         = 112,
  WATER_SAFETY_BARRIER                                 = 113,
  MergeInDeceleration                                  = 200,
  UNKNOWN                                              = 255
};

enum ControlWayPoint {
  NAVI_MISMATCHED_WITH_MAP       = 0,
  NO_MORE_CONTROL_WAY_POINT      = 1,
  MERGE_LEFT_ONTO_THE_MAIN_ROAD  = 2,
  MERGE_RIGHT_ONTO_THE_MAIN_ROAD = 3,
  COMING_LEFT_ONTO_THE_RAMP      = 4,
  COMING_RIGHT_ONTO_THE_RAMP     = 5,
  OTHERS                         = 6
};

enum NaviStatus { NO_NAVAGATION = 0, NAVI_PATH_CALCULATING = 1, NAVIGATING = 2, CRUISE = 3, YAW = 4, REPLANNING = 5 };

enum LocationQuality { UNABLED = 0, ROAD_LEVEL = 1, LANE_LEVEL = 2 };

enum LaneType_EV {
  LT_UNKNOWN         = 0,
  LT_EMERGENCY       = 1,
  LT_ACCLERATION     = 2,
  LT_DECELERATION    = 3,
  LT_ENTRY           = 4,
  LT_ON_RAMP         = 5,
  LT_CONNECTING_RAMP = 6,
  LT_NORMAL          = 7,
  LT_OTHERS          = 8
};

enum RoadType { RT_UNKNOWN = 0, RT_HIGHWAY = 1, RT_CITYWAY = 2 };

enum ReminderWayPoint { NO_MORE_REMINDER_WAY_POINT = 0, TUNNEL = 1, TOLL_BOOTH = 2 };

enum RemainType { TOLL_BOOTH_REMAIN = 0, NAVI_END = 1, NAVI_MAP_MATCHED_END = 2 };

enum NaviMainAction {
  NMA_NONE            = 0,
  NMA_TURN_LEFT       = 1,
  NMA_TURN_RIGHT      = 2,
  NMA_SLIGHT_LEFT     = 3,
  NMA_SLIGHT_RIGHT    = 4,
  NMA_TURN_HARD_LEFT  = 5,
  NMA_TURN_HARD_RIGHT = 6,
  NMA_LEFT_UTURN      = 7,
  NMA_CONTINUE        = 8,
  NMA_MERGE_LEFT      = 9,
  NMA_MERGE_RIGHT     = 10,
  NMA_ENTRY_RING      = 11,
  NMA_LEAVE_RING      = 12,
  NMA_RIGHT_UTURN     = 64,
  NMA_STRAIGHT        = 70,
  NMA_UNKNOWN         = 999
};

enum NaviAssistantAction {
  NAA_NONE                            = 0,
  NAA_FRONT                           = 201,
  NAA_RIGHT_FRONT                     = 202,
  NAA_RIGHT                           = 203,
  NAA_RIGHT_BACK                      = 204,
  NAA_BACK                            = 205,
  NAA_LEFT_BACK                       = 206,
  NAA_LEFT                            = 207,
  NAA_LEFT_FRONT                      = 208,
  NAA_RING                            = 209,
  NAA_RINGOUT                         = 210,
  NAA_LEFT_SIDE                       = 211,
  NAA_RIGHT_SIDE                      = 212,
  NAA_LEFT_SIDE_MAIN                  = 213,
  NAA_BRANCH_LEFT_MAIN                = 214,
  NAA_RIGHT_SIDE_MAIN                 = 215,
  NAA_BRANCH_RIGHT_MAIN               = 216,
  NAA_CENTER_MAIN                     = 217,
  NAA_LEFT_SIDE_IC                    = 218,
  NAA_RIGHT_SIDE_IC                   = 219,
  NAA_BRANCH_LEFT                     = 220,
  NAA_BRANCH_RIGHT                    = 221,
  NAA_BRANCH_CENTER                   = 222,
  NAA_START                           = 223,
  NAA_DEST                            = 224,
  NAA_VIA1                            = 225,
  NAA_VIA2                            = 226,
  NAA_VIA3                            = 227,
  NAA_VIA4                            = 228,
  NAA_INFERRY                         = 229,
  NAA_OUTFERRY                        = 230,
  NAA_TOLLGATE                        = 231,
  NAA_LEFT_SIDE_STRAIGHT_IC           = 232,
  NAA_RIGHT_SIDE_STRAIGHT_IC          = 233,
  NAA_LEFT_SIDE_STRAIGHT              = 234,
  NAA_RIGHT_SIDE_STRAIGHT             = 235,
  NAA_BRANCH_LEFT_STRAIGHT            = 236,
  NAA_BRANCH_CENTER_STRAIGHT          = 237,
  NAA_BRANCH_RIGHT_STRAIGHT           = 238,
  NAA_BRANCH_LEFT_IC                  = 239,
  NAA_BRANCH_CENTER_IC                = 240,
  NAA_BRANCH_RIGHT_IC                 = 241,
  NAA_BRANCH_LEFT_IC_STRAIGHT         = 242,
  NAA_BRANCH_CENTER_IC_STRAIGHT       = 243,
  NAA_BRANCH_RIGHT_IC_STRAIGHT        = 244,
  NAA_STRAIGHT_2BRANCH_LEFT_BASE      = 245,
  NAA_STRAIGHT_2BRANCH_RIGHT_BASE     = 246,
  NAA_STRAIGHT_3BRANCH_LEFT_BASE      = 247,
  NAA_STRAIGHT_3BRANCH_MIDDLE_BASE    = 248,
  NAA_STRAIGHT_3BRANCH_RIGHT_BASE     = 249,
  NAA_LEFT_2BRANCH_LEFT_BASE          = 250,
  NAA_LEFT_2BRANCH_RIGHT_BASE         = 251,
  NAA_LEFT_3BRANCH_LEFT_BASE          = 252,
  NAA_LEFT_3BRANCH_MIDDLE_BASE        = 253,
  NAA_LEFT_3BRANCH_RIGHT_BASE         = 254,
  NAA_RIGHT_2BRANCH_LEFT_BASE         = 255,
  NAA_RIGHT_2BRANCH_RIGHT_BASE        = 256,
  NAA_RIGHT_3BRANCH_LEFT_BASE         = 257,
  NAA_RIGHT_3BRANCH_MIDDLE_BASE       = 258,
  NAA_RIGHT_3BRANCH_RIGHT_BASE        = 259,
  NAA_LEFT_FRONT_2BRANCH_LEFT_BASE    = 260,
  NAA_LEFT_FRONT_2BRANCH_RIGHT_BASE   = 261,
  NAA_RIGHT_FRONT_2BRANCH_LEFT_BASE   = 262,
  NAA_RIGHT_FRONT_2BRANCH_RIGHT_BASE  = 263,
  NAA_BACK_2BRANCH_LEFT_BASE          = 264,
  NAA_BACK_2BRANCH_RIGHT_BASE         = 265,
  NAA_BACK_3BRANCH_LEFT_BASE          = 266,
  NAA_BACK_3BRANCH_MIDDLE_BASE        = 267,
  NAA_BACK_3BRANCH_RIGHT_BASE         = 268,
  NAA_MULTICROSS                      = 269,
  NAA_PARKINGFLOOR                    = 270,
  NAA_LEFT_FRONT_NOT_LEFT             = 271,
  NAA_RIGHT_FRONT_NOT_RIGHT           = 272,
  NAA_LEFT_BACK_NOT_BACK              = 273,
  NAA_LEFT_FRONT_3BRANCH_LEFT_BASE    = 274,
  NAA_LEFT_FRONT_3BRANCH_MIDDLE_BASE  = 275,
  NAA_LEFT_FRONT_3BRANCH_RIGHT_BASE   = 276,
  NAA_RIGHT_FRONT_3BRANCH_LEFT_BASE   = 277,
  NAA_RIGHT_FRONT_3BRANCH_MIDDLE_BASE = 278,
  NAA_RIGHT_FRONT_3BRANCH_RIGHT_BASE  = 279,
  NAA_LEFT_BACK_2BRANCH_LEFT_BASE     = 280,
  NAA_LEFT_BACK_2BRANCH_RIGHT_BASE    = 281,
  NAA_LEFT_BACK_3BRANCH_LEFT_BASE     = 282,
  NAA_LEFT_BACK_3BRANCH_MIDDLE_BASE   = 283,
  NAA_LEFT_BACK_3BRANCH_RIGHT_BASE    = 284,
  NAA_RIGHT_BACK_2BRANCH_LEFT_BASE    = 285,
  NAA_RIGHT_BACK_2BRANCH_RIGHT_BASE   = 286,
  NAA_RIGHT_BACK_3BRANCH_LEFT_BASE    = 287,
  NAA_RIGHT_BACK_3BRANCH_MIDDLE_BASE  = 288,
  NAA_RIGHT_BACK_3BRANCH_RIGHT_BASE   = 289,
  NAA_RING_FRONT                      = 290,
  NAA_RING_RIGHT_FRONT                = 291,
  NAA_RING_RIGHT                      = 292,
  NAA_RING_RIGHT_BACK                 = 293,
  NAA_RING_BACK                       = 294,
  NAA_RING_LEFT_BACK                  = 295,
  NAA_RING_LEFT                       = 296,
  NAA_RING_LEFT_FRONT                 = 297,
  NAA_BACK_RIGHT                      = 300,
  NAA_RIGHT_BACK_NOT_BACK             = 301,
  NAA_NEAR_RIGHT_FRONT                = 302,
  NAA_STRAIGHT_FRONT                  = 303,
  NAA_STRAIGHT_2BRANCH_LEFT           = 304,
  NAA_STRAIGHT_2BRANCH_RIGHT          = 305,
  NAA_STRAIGHT_3BRANCH_LEFT           = 306,
  NAA_STRAIGHT_3BRANCH_RIGHT          = 307,
  NAA_STRAIGHT_3BRANCH_CENTER         = 308,
  NAA_CONSTRUCTION_TOLLGATE           = 309,
  NAA_SOLIDDOT_LINE                   = 400,
  NAA_STRAIGHT                        = 401,
  NAA_UNKNOWN                         = 999
};

// 结构体定义
struct OddInfo {
  uint64_t     section_id   = 0;
  uint64_t     lane_id      = 0;  // 0 if on road level
  OddInfo_Type type         = OddInfo_Type::CONSTRUCTION;
  double       start_offset = 0.0;  // meters
  double       end_offset   = 0.0;  // meters
  ValueType    value;
};

struct SpeedLimitInfo {
  uint32_t speed_limit = 0;    // kph
  double   offset      = 0.0;  // meters
};

struct TrafficLightME {
  uint32_t light_state     = 0;  // 0: 无/未知, 1: 红灯, 2: 绿灯, 3: 黄灯
  uint32_t light_direction = 0;  // 0: 无效值, 1: 左转, 2: 右转, 7: 调头, 8: 直行
  uint64_t end_timestamp   = 0;
};

struct ReminderWayInfo {
  ReminderWayPoint reminder_waypoint_type = ReminderWayPoint::NO_MORE_REMINDER_WAY_POINT;
  double           reminder_waypoint_dis  = 0.0;  // meters
};

struct RemainDistanceInfo {
  RemainType remain_dis_type = RemainType::NAVI_END;
  double     remain_dis      = 0.0;  // meters
};

struct NaviActionInfo {
  NaviMainAction      main_action      = NaviMainAction::NMA_NONE;
  NaviAssistantAction assistant_action = NaviAssistantAction::NAA_NONE;
  double              action_dis       = 0.0;  // meters
};

struct CurvatureInfo {
  double curvature  = 0.0;
  double remain_dis = 0.0;  // meters
};

struct TrafficInfoNotify {
  cem::message::common::Header header;
  uint32_t                     traffic_jam_dist;           // 交通拥堵路段长度，单位：m
  uint32_t                     dist_to_start_traffic_jam;  // 交通拥堵起点与当前位置距离，单位：m
  uint32_t traffic_jam_status;  // 交通拥堵等级：0x00=未知, 0x01=良好, 0x02=不佳, 0x03=拥堵, 0x04=严重拥堵
  uint32_t pass_time;           // 通过时间，单位：s
};

struct MapEvent {
  cem::message::common::Header header;
  ControlWayPoint              control_waypoint_type = ControlWayPoint::NAVI_MISMATCHED_WITH_MAP;  // 修正为使用完整限定名
  double                       control_waypoint_dis  = 0.0;                                        // meters

  std::vector<ReminderWayInfo>    reminder_way_info;  // List of ReminderWayInfo
  std::vector<RemainDistanceInfo> remain_dis_info;    // List of RemainDistanceInfo

  NaviStatus      navi_stat  = NaviStatus::NO_NAVAGATION;
  LocationQuality loc_qulity = LocationQuality::UNABLED;

  std::vector<OddInfo> odd_info;  // List of OddInfo

  LaneType_EV lane_type = LaneType_EV::LT_UNKNOWN;
  RoadType    road_type = RoadType::RT_UNKNOWN;

  std::vector<SpeedLimitInfo> speed_limit_info;  // List of SpeedLimitInfo

  TrafficLightME traffic_light_info;       // Traffic light info
  double         traffic_light_dis = 0.0;  // meters

  std::vector<NaviActionInfo> navi_action;     // List of NaviActionInfo
  std::vector<CurvatureInfo>  curvature_info;  // List of CurvatureInfo
  TrafficInfoNotify           traffic_info;
};

}  // namespace env_model
}  // namespace message
}  // namespace cem
