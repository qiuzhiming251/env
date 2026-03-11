#ifndef E2E_MAP_H_
#define E2E_MAP_H_

#include <vector>

#include "message/common/geometry.h"
#include "message/common/header.h"
#include "Eigen/Dense"


namespace cem {
namespace message {
namespace sensor {


enum class E2eCoordSys 
{
    WGS84 = 0,
    GCJ02 = 1,
    RFU   = 2,
    FLU   = 3
};


//enum defination
enum class E2eNaviStatus {
    NO_NAVAGATION                   = 0,
    NAVI_PATH_CALCULATING           = 1,
    NAVIGATING                      = 2,
    CRUISE                          = 3,
    YAW                             = 4,
    REPLANNING                      = 5
};

enum class E2eNaviMainAction {
    NMA_NONE                        = 0, //无基本导航动作
    NMA_TURN_LEFT                   = 1, // 左转
    NMA_TURN_RIGHT                  = 2, // 右转
    NMA_SLIGHT_LEFT                 = 3, // 向左前方行驶
    NMA_SLIGHT_RIGHT                = 4, // 向右前方行驶
    NMA_TURN_HARD_LEFT              = 5, // 向左后方行驶
    NMA_TURN_HARD_RIGHT             = 6, // 向右后方行驶
    NMA_LEFT_UTURN                  = 7, // 左转调头
    NMA_CONTINUE                    = 8, // 直行
    NMA_MERGE_LEFT                  = 9, // 靠左
    NMA_MERGE_RIGHT                 = 10, // 靠右
    NMA_ENTRY_RING                  = 11, // 进入环岛
    NMA_LEAVE_RING                  = 12, // 离开环岛

    NMA_RIGHT_UTURN                 = 64, // 右转掉头

    NMA_STRAIGHT                    = 70, //顺行
    NMA_UNKNOWN                     = 999 //未定义的动作
};

enum E2eNaviAssistantAction {
    NAA_NONE                            = 0, //无辅助导航动作
    NAA_FRONT                           = 201,//直行
    NAA_RIGHT_FRONT                     = 202,//右前方转弯
    NAA_RIGHT                           = 203,//右转
    NAA_RIGHT_BACK                      = 204,//右后方转弯
    NAA_BACK                            = 205,//掉头
    NAA_LEFT_BACK                       = 206,//左后方转弯
    NAA_LEFT                            = 207,//左转
    NAA_LEFT_FRONT                      = 208,//左前方转弯
    NAA_RING                            = 209,//环岛
    NAA_RINGOUT                         = 210,//环岛出口
    NAA_LEFT_SIDE                       = 211,//普通/JCT/SAPA二分歧 靠左
    NAA_RIGHT_SIDE                      = 212,//普通/JCT/SAPA二分歧 靠右
    NAA_LEFT_SIDE_MAIN                  = 213,//左侧走本线
    NAA_BRANCH_LEFT_MAIN                = 214,//靠最左走本线
    NAA_RIGHT_SIDE_MAIN                 = 215,//右侧走本线
    NAA_BRANCH_RIGHT_MAIN               = 216,//靠最右走本线
    NAA_CENTER_MAIN                     = 217,//中间走本线
    NAA_LEFT_SIDE_IC                    = 218,//IC二分岐左侧走IC
    NAA_RIGHT_SIDE_IC                   = 219,//IC二分岐右侧走IC
    NAA_BRANCH_LEFT                     = 220,//普通三分歧/JCT/SAPA 靠最左
    NAA_BRANCH_RIGHT                    = 221,//普通三分歧/JCT/SAPA 靠最右
    NAA_BRANCH_CENTER                   = 222,//普通三分歧/JCT/SAPA 靠中间
    NAA_START                           = 223,//起点
    NAA_DEST                            = 224,//终点
    NAA_VIA1                            = 225,//第一个途经点
    NAA_VIA2                            = 226,//第二个途经点
    NAA_VIA3                            = 227,//第三个途经点
    NAA_VIA4                            = 228,//第四个途经点
    NAA_INFERRY                         = 229,//进入渡口
    NAA_OUTFERRY                        = 230,//离开渡口
    NAA_TOLLGATE                        = 231,//收费站
    NAA_LEFT_SIDE_STRAIGHT_IC           = 232,//IC二分歧左侧直行走IC
    NAA_RIGHT_SIDE_STRAIGHT_IC          = 233,//IC二分歧右侧直行走IC
    NAA_LEFT_SIDE_STRAIGHT              = 234,//普通/JCT/SAPA二分歧左侧 直行
    NAA_RIGHT_SIDE_STRAIGHT             = 235,//普通/JCT/SAPA二分歧右侧 直行
    NAA_BRANCH_LEFT_STRAIGHT            = 236,//普通/JCT/SAPA三分歧左侧 直行
    NAA_BRANCH_CENTER_STRAIGHT          = 237,//普通/JCT/SAPA三分歧中央 直行
    NAA_BRANCH_RIGHT_STRAIGHT           = 238,//普通/JCT/SAPA三分歧右侧 直行
    NAA_BRANCH_LEFT_IC                  = 239,//IC三分歧左侧走IC
    NAA_BRANCH_CENTER_IC                = 240,//IC三分歧中央走IC
    NAA_BRANCH_RIGHT_IC                 = 241,//IC三分歧右侧走IC
    NAA_BRANCH_LEFT_IC_STRAIGHT         = 242,//IC三分歧左侧直行
    NAA_BRANCH_CENTER_IC_STRAIGHT       = 243,//IC三分歧中间直行
    NAA_BRANCH_RIGHT_IC_STRAIGHT        = 244,//IC三分歧右侧直行
    NAA_STRAIGHT_2BRANCH_LEFT_BASE      = 245,//八方向靠左直行
    NAA_STRAIGHT_2BRANCH_RIGHT_BASE     = 246,//八方向靠右直行
    NAA_STRAIGHT_3BRANCH_LEFT_BASE      = 247,//八方向靠最左侧直行
    NAA_STRAIGHT_3BRANCH_MIDDLE_BASE    = 248,//八方向沿中间直行
    NAA_STRAIGHT_3BRANCH_RIGHT_BASE     = 249,//八方向靠最右侧直行
    NAA_LEFT_2BRANCH_LEFT_BASE          = 250,//八方向左转+随后靠左
    NAA_LEFT_2BRANCH_RIGHT_BASE         = 251,//八方向左转+随后靠右
    NAA_LEFT_3BRANCH_LEFT_BASE          = 252,//八方向左转+随后靠最左
    NAA_LEFT_3BRANCH_MIDDLE_BASE        = 253,//八方向左转+随后沿中间
    NAA_LEFT_3BRANCH_RIGHT_BASE         = 254,//八方向左转+随后靠最右
    NAA_RIGHT_2BRANCH_LEFT_BASE         = 255,//八方向右转+随后靠左
    NAA_RIGHT_2BRANCH_RIGHT_BASE        = 256,// 八方向右转+随后靠右
    NAA_RIGHT_3BRANCH_LEFT_BASE         = 257,//八方向右转+随后靠最左
    NAA_RIGHT_3BRANCH_MIDDLE_BASE       = 258,//八方向右转+随后沿中间
    NAA_RIGHT_3BRANCH_RIGHT_BASE        = 259,//八方向右转+随后靠最右
    NAA_LEFT_FRONT_2BRANCH_LEFT_BASE    = 260,//八方向左前方靠左侧
    NAA_LEFT_FRONT_2BRANCH_RIGHT_BASE   = 261,//八方向左前方靠右侧
    NAA_RIGHT_FRONT_2BRANCH_LEFT_BASE   = 262,//八方向右前方靠左侧
    NAA_RIGHT_FRONT_2BRANCH_RIGHT_BASE  = 263,//八方向右前方靠右侧
    NAA_BACK_2BRANCH_LEFT_BASE          = 264,//八方向掉头+随后靠左
    NAA_BACK_2BRANCH_RIGHT_BASE         = 265,//八方向掉头+随后靠右
    NAA_BACK_3BRANCH_LEFT_BASE          = 266,//八方向掉头+随后靠最左
    NAA_BACK_3BRANCH_MIDDLE_BASE        = 267,//八方向掉头+随后沿中间
    NAA_BACK_3BRANCH_RIGHT_BASE         = 268,//八方向掉头+随后靠最右
    NAA_MULTICROSS                      = 269,//多交叉路口
    NAA_PARKINGFLOOR                    = 270,//地下停车场
    NAA_LEFT_FRONT_NOT_LEFT             = 271,//向左前方行驶，注意不是左转
    NAA_RIGHT_FRONT_NOT_RIGHT           = 272,//向右前方行驶，注意不是右转
    NAA_LEFT_BACK_NOT_BACK              = 273,//向左后方行驶，注意不是掉头
    NAA_LEFT_FRONT_3BRANCH_LEFT_BASE    = 274,//八方向左前方行驶，进入最左侧道路
    NAA_LEFT_FRONT_3BRANCH_MIDDLE_BASE  = 275,//方向左前方行驶，进入中间道路
    NAA_LEFT_FRONT_3BRANCH_RIGHT_BASE   = 276,//八方向左前方行驶，进入最右侧道路
    NAA_RIGHT_FRONT_3BRANCH_LEFT_BASE   = 277,//八方向右前方行驶，进入最左侧道路
    NAA_RIGHT_FRONT_3BRANCH_MIDDLE_BASE = 278,//八方向右前方行驶，进入中间道路
    NAA_RIGHT_FRONT_3BRANCH_RIGHT_BASE  = 279,//八方向右前方行驶，进入最右侧道路
    NAA_LEFT_BACK_2BRANCH_LEFT_BASE     = 280,//八方向左后方行驶，进入左侧道路
    NAA_LEFT_BACK_2BRANCH_RIGHT_BASE    = 281,//八方向左后方行驶，进入右侧道路
    NAA_LEFT_BACK_3BRANCH_LEFT_BASE     = 282,//八方向左后方行驶，进入最左侧道路
    NAA_LEFT_BACK_3BRANCH_MIDDLE_BASE   = 283,//八方向左后方行驶，进入中间道路
    NAA_LEFT_BACK_3BRANCH_RIGHT_BASE    = 284,// 八方向左后方行驶，进入最右侧道路
    NAA_RIGHT_BACK_2BRANCH_LEFT_BASE    = 285,//八方向右后方行驶，进入左侧道路
    NAA_RIGHT_BACK_2BRANCH_RIGHT_BASE   = 286,//八方向右后方行驶，进入右侧道路
    NAA_RIGHT_BACK_3BRANCH_LEFT_BASE    = 287,//八方向右后方行驶，进入最左侧道路
    NAA_RIGHT_BACK_3BRANCH_MIDDLE_BASE  = 288,//八方向右后方行驶，进入中间道路
    NAA_RIGHT_BACK_3BRANCH_RIGHT_BASE   = 289,//八方向右后方行驶，进入最右侧道路
    NAA_RING_FRONT                      = 290,//环岛直行
    NAA_RING_RIGHT_FRONT                = 291,//环岛右前方转弯
    NAA_RING_RIGHT                      = 292,//环岛右转
    NAA_RING_RIGHT_BACK                 = 293,//环岛右后方转弯
    NAA_RING_BACK                       = 294,//环岛掉头
    NAA_RING_LEFT_BACK                  = 295,// 环岛左后方转弯
    NAA_RING_LEFT                       = 296,//环岛左转
    NAA_RING_LEFT_FRONT                 = 297,//环岛左前方转弯
    NAA_BACK_RIGHT                      = 300,//向右掉头
    NAA_RIGHT_BACK_NOT_BACK             = 301,//向右后方行驶，注意不是掉头
    NAA_NEAR_RIGHT_FRONT                = 302,//临近路口靠右前方
    NAA_STRAIGHT_FRONT                  = 303,//辅助直行用于复杂路口放大图
    NAA_STRAIGHT_2BRANCH_LEFT           = 304,//辅助直行二分歧靠左
    NAA_STRAIGHT_2BRANCH_RIGHT          = 305,//辅助直行二分歧靠右
    NAA_STRAIGHT_3BRANCH_LEFT           = 306,//辅助直行三分歧靠左
    NAA_STRAIGHT_3BRANCH_RIGHT          = 307,//辅助直行三分歧靠右
    NAA_STRAIGHT_3BRANCH_CENTER         = 308,//辅助直行三分歧靠中间
    NAA_CONSTRUCTION_TOLLGATE           = 309,//自然地物收费站
    NAA_SOLIDDOT_LINE                   = 400,//特殊标线
    NAA_STRAIGHT                        = 401,//顺行

    NAA_UNKNOWN                         = 999   //未定义的动作
};

enum class E2eRoadClassType {
    RCT_HIGHWAY = 0,            // 高速
    RCT_CITY_FAST_WAY = 1,      // 城市快速路
    RCT_NATIONAL_ROAD = 2,      // 国道
    RCT_PROVINCIAL_ROAD = 3,    // 省道
    RCT_COUNTY_ROAD = 4,        // 县道
    RCT_TOWNSHIP_ROAD = 5,      // 乡镇道路
    RCT_OTHER_ROAD = 6,         // 其他道路
    RCT_LEVEL_9_ROAD = 7,       // 九级路
    RCT_FERRY = 8,              // 轮渡
    RCT_WALK_WAY = 9,           // 行人道路
    RCT_INVALID = 127
};

enum class E2eDirectionType {
    UNKNOWN = 0,        //未知
    BIDIRECTIONAL_PASSABLE = 1, //双向可通行
    FORWARD_ONLY = 2,   //仅正向通行
    REVERSE_ONLY = 3,   //仅反向通行
    INVALID = 4       //无效
};

enum class E2eReminderSceneType {
    RST_NO_MORE_REMINDER_SCENE          = 0,
    RST_TOLL_BOOTH                      = 1, 
    RST_IC_OR_JCT                       = 2,
    RST_TUNNEL                          = 3,
    RST_BRIDGE                          = 4,
    RST_JUNCTION                        = 5
};

enum class E2eLaneType {
    LANE_UNKNOWN = 0,
    LANE_NORMAL = 1,
    LANE_ACC = 2,
    LANE_DEC = 3,
    LANE_RAMP = 4,
    LANE_EMERGENCY = 5,
    LANE_ACC_DCC = 6,
    LANE_BUS_NORMAL = 7,
    LANE_HOV_NORMAL = 8,
    LANE_NON_MOTOR = 9,
    LANE_LEFT_WAIT = 10,
    LANE_VIRTUAL_COMMON = 11,
    LANE_VIRTUAL_JUNCTION = 12,
    LANE_ROUND_ABOUT = 13,
    LANE_REVERSIBLE = 16,
    LANE_VARIABLE_TURN = 17,
    LANE_HARBOR_STOP = 18, // 港湾停靠站
    LANE_ENTRY = 19,  // 入口
    LANE_EXIT = 20, // 出口
    LANE_DIVERSION = 21, // 导流区车道
    LANE_U_TURN_LANE = 22, // 掉头车道
    LANE_RIGHT_TURN_LANE         = 23, // 右转专用车道
    LANE_RIGHT_TURN_AREA         = 24, // 右转等待车道
    LANE_U_TURN_AREA             = 25, // 掉头等待车道
    LANE_NO_TURN_AREA            = 26, // 直行等待车道
    LANE_VIRTUAL_CONNECTED_LANE  = 27 // 虚拟连接车道
};

enum class E2eTurnType {
    NO_TURN = 0,
    LEFT_TURN = 1,
    RIGHT_TURN = 2,
    U_TURN = 3,
    STRAIGHT_AND_LEFT_TURN = 4,
    STRAIGHT_AND_RIGHT_TURN = 5,
    STRAIGHT_AND_U_TURN = 6,
    LEFT_TURN_AND_U_TURN = 7,
    RIGHT_TURN_AND_U_TURN = 8,
    STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN = 9,
    LEFT_TURN_AND_RIGHT_TURN = 10,
    STRAIGHT_AND_LEFT_TURN_AND_U_TURN = 11,
    STRAIGHT_AND_RIGHT_TURN_AND_U_TURN = 12,
    LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN = 13,
    STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN = 14,
    
    OTHER_UNKNOWN = 9999
};

enum class E2eSplitTopology {
    TOPOLOGY_SPLIT_NONE = 0,
    TOPOLOGY_SPLIT_LEFT = 1,  // split to left
    TOPOLOGY_SPLIT_RIGHT = 2,  // split to right
    TOPOLOGY_SPLIT_UNKNOWN = 3
};

enum class E2eMergeTopology {
    TOPOLOGY_MERGE_NONE = 0,
    TOPOLOGY_MERGE_LEFT = 1,   // merge to left, ego is on right of target lane
    TOPOLOGY_MERGE_RIGHT = 2,  // merge to right, ego is on left of target lane
    TOPOLOGY_TO_BE_MERGED = 3,
    TOPOLOGY_MERGE_UNKNOWN = 4
};

struct E2eNaviPosition
{
    uint64_t section_id;
    double s_offset;    //uint: m
};

struct E2eRoadClassInfo
{
    E2eRoadClassType road_class;
    double dis;    
};

struct E2eLaneNumInfo
{
    uint32_t lane_num;
    double dis;    
};

struct E2ePoint
{
    double x;
    double y;
    double z;
    double curvature;    
};

struct E2eSpeedLimitInfo
{
    double speed_limit; //uint: km/h
    double dis;
};

struct E2eFormwayInfo
{
    uint32_t formway; //数值是LinkTypeMask按位或组合的值
    double dis;
};

struct E2eDirectionInfo
{
    E2eDirectionType direction_type; 
    double dis;
};

struct E2eReminderSceneInfo
{
    E2eReminderSceneType type; 
    double dis;
};

struct E2eLaneInfo
{
    uint32_t         lane_seq; //车道序号，从左到右，从1开始
    E2eLaneType      lane_type;
    E2eTurnType      turn_type;
    E2eTurnType      plan_turn_type;
    std::vector<uint32_t> next_lane_seq;
    std::vector<uint32_t> previous_lane_seq;
    E2eSplitTopology split_topology;
    E2eMergeTopology merge_topology;
    uint64_t        id; //问题排查时需要    
};

struct E2eLaneGroupInfo
{
  std::vector<E2eLaneInfo> lane_info;  // 如果有空的lanegroup这里只会填dis
  double dis;
  uint64_t id;  // TBD
  std::vector<uint64_t> next_lane_group;
  std::vector<uint64_t> previous_lane_group;
};

struct E2ePathInfo
{
  std::vector<E2eRoadClassInfo> road_class_info;
  std::vector<E2eLaneNumInfo> lane_num_info;
  std::vector<E2ePoint> shape_points;
  std::vector<E2eSpeedLimitInfo> speed_limit_info;
  std::vector<E2eFormwayInfo> formway_info;
  std::vector<E2eDirectionInfo> direction_info;
  std::vector<E2eReminderSceneInfo> reminder_toll_booth_info;
  std::vector<E2eReminderSceneInfo> reminder_ic_or_jct_info;
  std::vector<E2eReminderSceneInfo> reminder_tunnel_info;
  std::vector<E2eReminderSceneInfo> reminder_bridge_info;
  std::vector<E2eReminderSceneInfo> reminder_junction_info;
  std::vector<E2eLaneGroupInfo> lane_group_info;
};

struct E2eSubPathInfo
{
    enum class E2ePathType {
        NONE = 0,
        SPLIT = 2,
        MERGE = 3
    };
    
    double dis; 
    E2ePathInfo subpath;
    E2ePathType path_type;
    std::vector<uint64_t> enter_lane_seq;
    uint64_t path_id;
};

struct E2EMapInfo
{
    E2eCoordSys coord_sys;
    std::string map_version;
    std::string engine_version;
    std::string sdmap_version;
};

struct E2eNaviActionInfo
{
    E2eNaviMainAction main_action;
    E2eNaviAssistantAction assistant_action;
    double action_dis;      //uint: m
};

struct E2eRemainDistanceInfo
{
    enum class E2eRemainType {
        TOLL_BOOTH                      = 0,
        NAVI_END                        = 1,
        NAVI_MAP_MATCHED_END            = 2
    };
    E2eRemainType remain_dis_type;
    double remain_dis;
};

struct E2eRouteInfo
{
    std::string id;
    E2eNaviPosition navi_start; //from currrent section
    E2ePathInfo mpp_info;
    std::vector<E2eSubPathInfo> subpaths;
};


struct E2eMap
{
    cem::message::common::Header header;
    E2EMapInfo map_info;
    E2eNaviStatus navi_stat;
    std::vector<E2eNaviActionInfo> navi_action;
    std::vector<E2eRemainDistanceInfo> remain_dis_info;
    E2eRouteInfo route_info;

};

} // namespace sensor
} // namespace message
} // namespace cem

#endif
