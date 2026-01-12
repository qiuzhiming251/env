#ifndef ROUTING_MAP_H_
#define ROUTING_MAP_H_

#include <cstdint>
#include <Eigen/Geometry>
#include "message/common/geometry.h"
#include "message/common/header.h"
#include "message/env_model/speed_limit/speed_limit.h"
#include "modules/msg/basic_msgs/geometry.pb.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"

namespace cem {
namespace message {
namespace env_model {
enum PointSource {
  PS_UNKNOWN = 0,
  PS_BEV     = 1,  // 感知形点
  PS_ENV     = 2,  // 环境模型延长的形点
  PS_HDMAP   = 3   // 地图形点
};

enum class CoordSys { WGS84 = 0, GCJ02 = 1, RFU = 2, FLU = 3 };

enum class LaneType {
  LANE_UNKNOWN                    = 0,
  LANE_NORMAL                     = 1,
  LANE_ACC                        = 2,
  LANE_DEC                        = 3,
  LANE_RAMP                       = 4,
  LANE_EMERGENCY                  = 5,
  LANE_ACC_DCC                    = 6,
  LANE_BUS_NORMAL                 = 7,
  LANE_HOV_NORMAL                 = 8,
  LANE_NON_MOTOR                  = 9,
  LANE_LEFT_WAIT                  = 10,
  LANE_VIRTUAL_COMMON             = 11,
  LANE_VIRTUAL_JUNCTION           = 12,
  LANE_ROUND_ABOUT                = 13,
  LANE_REVERSIBLE                 = 16,
  LANE_VARIABLE_TURN              = 17,
  LANE_HARBOR_STOP                = 18,
  LANE_ENTRY                      = 19,  // 入口
  LANE_EXIT                       = 20,  // 出口
  LANE_DIVERSION                  = 21,  // 导流区车道
  LANE_U_TURN_LANE                = 22,  // 掉头车道
  LANE_RIGHT_TURN_LANE            = 23,  // 右转专用车道
  LANE_RIGHT_TURN_AREA            = 24,  // 右转等待车道
  LANE_U_TURN_AREA                = 25,  // 掉头等待车道
  LANE_NO_TURN_AREA               = 26,  // 直行等待车道
  LANE_VIRTUAL_CONNECTED_LANE     = 27,  // 虚拟连接车道
  LANE_PARKING                    = 28,  // 停车车道
  LANE_TOLLBOOTH                  = 29,  // 收费站车道
  LANE_OBSTACLE                   = 30,  // 障碍物
  LANE_MIXED_TOLL                 = 31,  // 混合收费车道
  LANE_REVERSIBLE_CONNECTION_LANE = 32,  //潮汐连接路
  LANE_ENTRANCE_OR_EXIT_LANE      = 33,  //主辅路出入口
  LANE_LEFT_TURN_LANE             = 34,  //左转专用道
  LANE_CHECKPOINT_LANE            = 35,  //检查站车道
  LANE_USE_THE_LEFT_TURN_LANE     = 36,  //借道左转车道
  LANE_BRT                        = 37   //公交专用道
};

enum class NoneOddType {
  NODD_NONE                                                 = 0,
  NORMAL_NOT_ODD                                            = 1,
  NODD_LOW_PRECISION                                        = 22,
  NODD_LANE_BOUNDARY_CHANGE                                 = 100,
  NODD_LANE_BOUNDARY_LOSS_WIDTH_ABNORMAL                    = 101,
  NODD_LANE_NUM_INCREASE                                    = 102,
  NODD_LANE_NUM_DECREASE                                    = 103,
  NODD_LANE_TYPE_CHANGE                                     = 104,
  NODD_CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_NOT_DEGRADATION = 105,
  NODD_CONSTRUCTION_OPPOSITE_DIR_BORROW_WAY_DEGRADATION     = 106,
  NODD_CONSTRUCTION_SAME_DIR_CHANGE_WAY_NOT_DEGRADATION     = 107,
  NODD_CONSTRUCTION_SAME_DIR_CHANGE_WAY_DEGRADATION         = 108,
  NODD_CONSTRUCTION_CLOSE_WAY                               = 109,
  NODD_CONSTRUCTION_CLOSE_WAY_TO_NOT_HIGHWAY_DEGRADATION    = 110,
  NODD_RAMP_INCREASE                                        = 111,
  NODD_TRAFFIC_CONE                                         = 112,
  NODD_WATER_SAFETY_BARRIER                                 = 113,
  NODD_UNKNOWN                                              = 255
};

enum class TurnType {
  NO_TURN                                = 0,
  LEFT_TURN                              = 1,
  RIGHT_TURN                             = 2,
  U_TURN                                 = 3,
  STRAIGHT_AND_LEFT_TURN                 = 4,
  STRAIGHT_AND_RIGHT_TURN                = 5,
  STRAIGHT_AND_U_TURN                    = 6,
  LEFT_TURN_AND_U_TURN                   = 7,
  RIGHT_TURN_AND_U_TURN                  = 8,
  STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN  = 9,
  LEFT_TURN_AND_RIGHT_TURN               = 10,
  STRAIGHT_AND_LEFT_TURN_AND_U_TURN      = 11,
  STRAIGHT_AND_RIGHT_TURN_AND_U_TURN     = 12,
  LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN    = 13,
  STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN = 14,
  OTHER_UNKNOWN                          = 9999
};

enum class LightStatus {
  NONE_LIGHT      = 0,  //无信号灯
  GREEN_LIGHT     = 1,
  YELLOW_LIGHT    = 2,
  RED_LIGHT       = 3,
  UNKNOWN_LIGHT   = 4,  //有信号灯状态未知
  YELLOW_BLINKING = 5,
  FAIL_DETECTION  = 6,
  BLOCK_FAIL      = 7,
  GREEN_BLINKING  = 8,
  BLURRING_MODE   = 9,
  CLOUD_NOT_MATCH = 10,
};

enum class SplitTopology {
  TOPOLOGY_SPLIT_NONE    = 0,
  TOPOLOGY_SPLIT_LEFT    = 1,  // split to left
  TOPOLOGY_SPLIT_RIGHT   = 2,  // split to right
  TOPOLOGY_SPLIT_UNKNOWN = 3
};

enum class MergeTopology {
  TOPOLOGY_MERGE_NONE    = 0,
  TOPOLOGY_MERGE_LEFT    = 1,  // merge to left, ego is on right of target lane
  TOPOLOGY_MERGE_RIGHT   = 2,  // merge to right, ego is on left of target lane
  TOPOLOGY_TO_BE_MERGED  = 3,
  TOPOLOGY_MERGE_UNKNOWN = 4
};
// struct MergeDetail {
//   std::vector<uint64_t> src_ids;         // 前继车道ID列表
//   uint64_t              dst_id;          // 目标车道ID
//   uint64_t              section_id;      // merge所在的section ID
//   uint64_t              lanegroup_id;    // dst_id所在的lanegroup ID
//   double                merge_distance;  // merge点与自车的距离
//   MergeTopology         type;            // merge类型
// };

enum class LineType {
  UNKNOWN       = 0,
  SOLID         = 1,
  DASHED        = 2,
  SOLID_SOLID   = 3,
  DASHED_DASHED = 4,
  SOLID_DASHED  = 5,
  DASHED_SOLID  = 6,
  VIRTUAL_LANE  = 7,
  OTHERS        = 9999
};

enum class LaneMarkerColor { LMC_UNKNOWN = 0, LMC_WHITE = 1, LMC_YELLOW = 2, LMC_BLUE = 3, LMC_GREEN = 4, LMC_RED = 5 };

// enum class BoundaryType { UNKNOWN_BOUNDARY = 0, VIRTUAL = 12 };
enum class BoundaryType {
    UNKNOWN_BOUNDARY  = 0,
    FLAT_BOUNDARY     = 1,
    LOW_BOUNDARY      = 2,
    HIGH_BOUNDARY     = 3,
    FENCE_BOUNDARY    = 4,   
    VIRTUAL           = 12  
};


enum class RoadClass {
  UNKNOWN          = 0,
  EXPRESSWAY       = 1,
  URBAN_EXPRESSWAY = 2,
  NATION_ROAD      = 3,
  PROVINCE_ROAD    = 4,
  COUNTRY_ROAD     = 6,
  TOWN_ROAD        = 7,
  SPECIAL_ROAD     = 8,  // 特服道路
  WALK_ROAD        = 9,
  PEOPLE_FERRY     = 10,  // 人渡
  FERRY            = 11,  // 轮渡
  OTHERS           = 99
};

enum class MapType { UNKOWN_MAP = 0, PERCEPTION_MAP = 1, HD_MAP = 2, BEV_MAP = 3 };

enum class V2RoadClassType {
  UNKNOWN_ROAD     = 0,
  HIGH_WAY_ROAD    = 1,
  EXPRESS_WAY_ROAD = 2,
  NATIOANL_ROAD    = 3,
  PROVINCIAL_ROAD  = 4,
  MAIN_ROAD        = 5,
  SUB_ROAD         = 6
};

enum class MapProvider {
  Baidu = 0,
  Byd = 1,
};

struct MapInfo {
  CoordSys    coord_sys;
  std::string map_version;
  std::string engine_version;
  MapProvider map_provider;
};

struct Point {
  double      x            = 0.0;
  double      y            = 0.0;
  double      z            = 0.0;
  double      mse          = 0.0;
  PointSource point_source = PointSource::PS_UNKNOWN;
  double      curvature    = 0.0;
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
  Point2D() = default;
  Point2D(double x, double y) : x(x), y(y) {}
};

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

enum class RestrictedType : uint8_t {
    RT_NONE               = 0, // 无
    RT_IDAL               = 1, // 潮汐车道
    RT_BRT                = 2, // 公交专用道
    RT_HOV                = 3, // HOV车道
    RT_TAXI               = 4, // 出租车专用道
    RT_OTHER              = 5, // 其他车道
    RT_HVL                = 7, // 危化品车辆专用道
    RT_ADL                = 8, // 救护车专用道
    RT_SHARED_LEFT_TURN   = 9  // 借道左转车道
};


struct RestrictedInfo {          
    RestrictedType restricted_type = RestrictedType::RT_NONE;
    bool is_passable = true;   ///不在使用这个字段
    uint32_t passable_env_state = 0; 
};

struct LightInfo {
    bool prev_has_light_now_no{false};
    bool maybe_exist_light{false};
};

struct LaneInfo {
  uint64_t              id                      = 0;
  uint64_t              section_id              = 0;
  uint64_t              junction_id             = 0;
  uint64_t              left_lane_id            = 0;
  uint64_t              right_lane_id           = 0;
  std::vector<uint64_t> next_lane_ids           = {};
  std::vector<uint64_t> left_lane_boundary_ids  = {};
  std::vector<uint64_t> right_lane_boundary_ids = {};
  std::vector<uint64_t> left_road_boundary_ids  = {};
  std::vector<uint64_t> right_road_boundary_ids = {};
  LaneType              type;
  NoneOddType           none_odd_type;
  TurnType              turn_type;
  SplitTopology         split_topology;
  MergeTopology         merge_topology;
  double                length             = 0.0;
  double                speed_limit        = 0.0;
  bool                  is_virtual         = false;
  std::vector<Point>    points             = {};
  std::vector<uint64_t> cross_walks        = {};
  std::vector<uint64_t> traffic_stop_lines = {};
  std::vector<uint64_t> previous_lane_ids  = {};
  std::vector<uint64_t> exp_trajectory_ids = {};
  TurnType              plan_turn_type     = TurnType::OTHER_UNKNOWN;  // 0319新增
  bool                  cnoa_is_virtual    = false;                    // 0520新增
  uint32_t              lane_seq           = 0;
  bool                  is_acc_adj_lane    = false;                               //是否是加速车道邻车道
  double                merge_start_dis    = std::numeric_limits<double>::max();  //汇流区起点距自车的距离，当is_acc_adj_lane为true时有效
  double                merge_end_dis      = std::numeric_limits<double>::max();  //汇流区终点距自车的距离，当is_acc_adj_lane为true时有效
  LightInfo             light_info;

  LightStatus light_status;
  uint32_t    light_countdown = 0;
  uint32_t    traffic_light_obj_id{0};
  uint32_t    traffic_light_seq_num{0};
  uint32_t    stay_prev_counter{0};
  uint32_t    stopline_angle_flag{0};
  double      distance_to_stopline{0.0};
  bool        traffic_match_cloud_file{false};
  uint64_t    cloud_junction_type{0};
  double      yellow_flashing_start_time{0.0};

  byd::msg::orin::routing_map::LaneInfo::TrafficSetReason traffic_set_reason{byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE};
  RestrictedInfo restricted_info; 
};

struct LaneBoundaryInfo {
  uint64_t             id     = 0;
  std::vector<Point2D> points = {};
  LineType             line_type;
  LaneMarkerColor      line_color;
};

struct RoadBoundaryInfo {
  uint64_t             id     = 0;
  std::vector<Point2D> points = {};
  BoundaryType         boundary_type{BoundaryType::UNKNOWN_BOUNDARY};
};

struct JunctionInfo {
  uint64_t             id     = 0;
  std::vector<Point2D> points = {};
};

struct CrossWalkInfo {
  uint64_t id                 = 0;
  std::vector<Point2D> points = {};
};

struct Arrows {
  uint16_t             id     = 0;
  uint32_t             type   = 0;
  std::vector<Point2D> points = {};
};

struct StopLineInfo {
  uint64_t             id     = 0;
  std::vector<Point2D> points = {};
  uint32_t             type   = 0;  //type: 0 - real stopline, 1 - virtual stopline
};

struct NaviPosition {
  uint64_t section_id = 0;
  double   s_offset   = 0.0;
};

struct SectionInfo {
  uint64_t              id       = 0;
  double                length   = 0.0;  //uint: m
  std::vector<uint64_t> lane_ids = {};
  std::vector<Point>    points   = {};
  RoadClass             road_class;
  uint32_t              link_type                   = 0;
  uint32_t              lane_num                    = 0;
  std::vector<uint64_t> predecessor_section_id_list = {};  // proto新增字段
  std::vector<uint64_t> successor_section_id_list   = {};  // proto新增字段
  bool                  is_mpp_section              = false;
};

// SubPath: 新增的结构体定义
struct SubPath {
  uint64_t                 enter_section_id = 0;
  std::vector<SectionInfo> sections         = {};
};

struct PathExtend {
  uint64_t from_section_idx = 0;  // 此通路的前驱section在route sections中的索引，如果小于0表示从route第一个section之前就分出此通路。
  uint64_t from_section_id = 0;
  uint64_t to_section_idx = 0;    // 此通路的后继section在route sections中的索引
  std::vector<SectionInfo> sections;  // 此通路经过的sections
};

struct RouteInfo {
  uint64_t                 id = 0;
  NaviPosition             navi_start;
  std::vector<SectionInfo> sections = {};
  std::vector<SubPath>     subpaths = {};  // proto新增字段
  std::string              route_id = "";
  std::vector<PathExtend>  extend_sections; //用于表示主路径、即mpp的其他可通行通路sections列表
};

struct TrafficLightMap {
  uint64_t                                    id = 0;
  cem::message::common::Point3DD              center_position;
  std::vector<cem::message::common::Point3DD> bounding_box_geometry = {};
  std::vector<uint64_t>                       laneids               = {};
  uint32_t                                    light_countdown       = 0;
  uint32_t                                    shape                 = 0;
  LightStatus                                 light_status;
};

struct V2RoadClass {
  double          start      = 0.0;
  double          end        = 0.0;
  V2RoadClassType road_class = V2RoadClassType::UNKNOWN_ROAD;
};

struct V2RoadInfo {
  V2RoadClassType road_class = V2RoadClassType::UNKNOWN_ROAD;
  int32_t         lane_num   = 0;
};

struct V2TurnInfo {
  uint64_t id       = 0;
  bool     is_valid = false;

  enum class V2TurnType {
    UNKNOWN           = 0,
    LEFT              = 1,
    RIGHT             = 2,
    STRAIGHT          = 3,
    U_TURN_LEFT       = 4,
    U_TURN_RIGHT      = 5,
    MERGE_LEFT        = 6,
    MERGE_RIGHT       = 7,
    RAMP_LEFT         = 11,
    RAMP_RIGHT        = 12,
    RAMP_STRAIGHT     = 13,
    RAMP_U_TURN_LEFT  = 14,
    RAMP_U_TURN_RIGHT = 15
  };

  V2TurnType turn_type = V2TurnType::UNKNOWN;

  enum class V2DetailTurnType {
    NONE            = 0,
    TURN_LEFT       = 2,
    TURN_RIGHT      = 3,
    SLIGHT_LEFT     = 4,
    SLIGHT_RIGHT    = 5,
    TURN_HARD_LEFT  = 6,
    TURN_HARD_RIGHT = 7,
    UTURN           = 8,
    CONTINUE        = 9,
    TURN_RIGHT_ONLY = 10,
    UTURN_RIGHT     = 19,
    LEFT_MERGE      = 65,
    RIGHT_MERGE     = 66
  };

  V2DetailTurnType detail_turn_type = V2DetailTurnType::NONE;
  double           dist             = 0.0;  // 视觉范围内到停止线距离
  V2RoadInfo       before_turn;
  V2RoadInfo       after_turn;
  double           v2_dist = 0.0;  // V2提供的距离
};

struct V2Curvature {
  double distance  = 0.0;  // 距离
  double curvature = 0.0;  // 曲率
};

struct V2TrafficFlow {
  double start_s = 0.0;
  double end_s   = 0.0;
  enum class V2TrafficFlowType { UNKNOWN_FLOW = 0, SMOOTH_FLOW = 1, SLOW_FLOW = 2, JAMMED_FLOW = 3, SEVERE_JAMMED_FLOW = 4, NO_FLOW = 5 };
  V2TrafficFlowType type = V2TrafficFlowType::UNKNOWN_FLOW;
};

struct NonODDInfo {
  std::string id;
  uint64_t    reason;
  double      dist;
};

struct EnvInfo {
  bool                       v2_valid                          = false;
  bool                       v2_has_navigation                 = false;
  double                     v2_dist_to_ramp                   = 0.0;
  double                     v2_dist_to_toll                   = 0.0;
  double                     v2_dist_to_tunnel                 = 0.0;
  double                     v2_dist_to_subpath                = 0.0;
  double                     v2_dist_to_split_routelanenum_dec = 0.0;
  std::vector<V2RoadClass>   v2_road_classes                   = {};
  std::vector<V2TurnInfo>    v2_turn_info                      = {};
  std::vector<V2TrafficFlow> traffic_flows                     = {};
  std::vector<V2Curvature>   v2_curvatures                     = {};
  std::vector<NonODDInfo>    v2_non_odd_info;
  bool                       is_switched_to_LD_ = false;
  bool                       is_on_highway_     = false;
  std::vector<std::string>   switch_ld_reason;
};
////////////////////////////////////////////////////LD Map////////////////////////////////////////////////////////////////
enum class LDLinkTypeMask : uint32_t {
  LT_NORMAL               = 0,       //  0X00000000, // 普通
  LT_TUNNEL               = 1,        //   0X00000001, // 隧道
  LT_BRIDGE               = 2,         // 0X00000002, // 桥
  LT_TOLLBOOTH            = 4,         //  0X00000004, // 收费站道路
  LT_DEAD_END             = 8,         // 0X00000008, // 断头路
  LT_IC                   = 16,        //  0X00000010, // IC(高速连接普通路的道路)
  LT_JCT                  = 32,        // 0X00000020, // JCT(高速连接高速的道路)
  LT_SAPA                 = 64,        //  0X00000040, // SAPA(服务器、停车区道路)
  LT_WITHIN_INTERSECTION  = 128,       //  0X00000080, // 路口内道路
  LT_AUTHORIZED           = 256,       // 0X00000100, // 授权道路
  LT_TOLLGATE             = 512,       //  0X00000200, // 收费岛道路
  LT_ABANDONED_TOLLGATE   = 1024,      //  0X00000400, // 废弃收费岛道路
  LT_CHECKPOINT           = 2048,      // 0X00000800, // 检查站道路
  LT_ROUNDABOUT           = 4096,      //  0X00001000, // 环岛内道路
  LT_SERRATED             = 8192,      //   0X00002000, // 锯齿道路
  LT_MAIN_ROAD            = 16384,     //   0X00004000, // 主路(main road)
  LT_SIDE_ROAD            = 32768,     // 0X00008000, // 辅路(side road)
  LT_MAINROAD_CONNECTION  = 65536,     // 0X00010000, // 主辅路连接路(mainside connection)
  LT_NO_S_INTERSECTION    = 131072,    // 0X00020000, // 无小路口
  LT_S_INTERSECTION_LEFT  = 262144,    // 0X00040000, // 小路口左侧道路
  LT_S_INTERSECTION_RIGHT = 524288,    //0X00080000, // 小路口右侧道路
  LT_S_INTERSECTION_BOTH  = 1048576,   //0X00100000, // 小路口两侧道路
  LT_INJUNCTION           = 2097152,   //0X00200000, // 路口内道路（编译计算）
  LT_BOOTH_EXIT           = 4194304,   //0X00400000, // 出口收费站
  LT_BOOTH_ENTRANCE       = 8388608,   //0X00800000, // 入口收费站
  LT_BOOTH_EXIT_ENTRANCE  = 16777216  //0X01000000  // 出入口收费站
};
////////////////////////////////////////////////////SD Map////////////////////////////////////////////////////////////////
enum class SDLinkTypeMask : uint32_t {
  SDLT_INVALID              = 0,
  SDLT_RING                 = 1,         //环岛 0X00000001
  SDLT_NOATTR               = 2,         //无属性 0X00000002
  SDLT_MAINSEPARATE         = 4,         //上下行分离 0X00000004
  SDLT_JCT                  = 8,         //JCT 0X00000008
  SDLT_CROSSLINK            = 16,        //交叉点内LINK 0X00000010
  SDLT_IC                   = 32,        //IC 0X00000020
  SDLT_PARK                 = 64,        //停车区 0X00000040
  SDLT_SERVICE              = 128,       //服务区 0X00000080
  SDLT_BRIDGE               = 256,       //桥 0X00000100
  SDLT_WALKSTREET           = 512,       //步行街 0X00000200
  SDLT_SIDEROAD             = 1024,      //辅路 0X00000400
  SDLT_RAMP                 = 2048,      //匝道 0X00000800
  SDLT_CLOSEDROAD           = 4096,      //全封闭道路 0X00001000
  SDLT_UNDEFINEDTRAFFICAREA = 8192,      //未定义交通区域 0X00002000
  SDLT_POICONNECTION        = 16384,     //连接路 0X00004000
  SDLT_TUNNEL               = 32768,     //隧道 0X00008000
  SDLT_RA_FOOTWAY           = 65536,     //步行道 0X00010000
  SDLT_BUS                  = 131072,    //公交专用道 0X00020000
  SDLT_RIGHTTURN            = 262144,    //提前右转 0X00040000
  SDLT_SCENICROAD           = 524288,    //风景线路 0X00080000
  SDLT_INAREAROAD           = 1048576,   //区域内道路 0X00100000
  SDLT_LEFTTURN             = 2097152,   //提前左转 0X00200000
  SDLT_UTURN                = 4194304,   //掉头口 0X00400000
  SDLT_MAINSIDECONNECT      = 8388608,   //主辅路出入口 0X00800000
  SDLT_DUMMYLINK            = 16777216,  //虚拟链接路 0X01000000
  SDLT_PARKLINK             = 33554432   //停车位引导路 0X02000000
};

enum class SDDirectionType {
  UNKNOWN                = 0,  //未知
  BIDIRECTIONAL_PASSABLE = 1,  //双向可通行
  FORWARD_ONLY           = 2,  //仅正向通行
  REVERSE_ONLY           = 3,  //仅反向通行
  INVALID                = 4   //无效
};

enum class SDLinkTypeExtendMask : uint32_t {
  SDLTE_INVALID                = 0,     // = 0X00000000     /**< 0X00,无效值 */
  SDLTE_PARKING_ENUUTRANCE     = 1,     // = 0X00000001,    /*停车场出入口连接路 */
  SDLTE_GAS_STATION_ENUUTRANCE = 2,     // = 0X00000002,    /*通往加油站的连接路 */
  SDLTE_ESCAPE_LANE            = 4,     // = 0X00000004,    /*避险车道 */
  SDLTE_TRUCK_LANE             = 8,     // = 0X00000008,    /*货车专用*/
  SDLTE_NOTROUNDABOUT          = 16,    // = 0X00000010,    /*非标环岛 */
  SDLTE_FRONT_ROAD             = 32,    // = 0X00000020,    /*门前路段*/
  SDLTE_FOOTBRIDGE             = 64,    // = 0X00000040,    /*跨线天桥*/
  SDLTE_TUNNEL                 = 128,   // = 0X00000080,    /*跨线地道 */
  SDLTE_FLYOVER                = 256,   // = 0X00000100,    /*立交桥*/
  SDLTE_TAXILANE               = 512,   // = 0X00000200,    /*出租车专用道*/
  SDLTE_PASSENGERCARLANE       = 1024,  // = 0X00000400,    /*客运车专用道*/
  SDLTE_PANSHANROAD            = 2048   // = 0X00000800,    /*盘山路*/
};
enum class SDRoadClass {
  SD_HIGHWAY         = 0,
  SD_CITY_FAST_WAY   = 1,
  SD_NATIONAL_ROAD   = 2,
  SD_PROVINCIAL_ROAD = 3,
  SD_COUNTY_ROAD     = 4,
  SD_TOWNSHIP_ROAD   = 5,
  SD_OTHER_ROAD      = 6,
  SD_LEVEL_9_ROAD    = 7,
  SD_FERRY           = 8,
  SD_WALY_WAY        = 9,
  SD_INVALID         = 127
};

struct SDLaneGroupIndex {
  uint64_t id                 = 0;
  double   start_range_offset = 0.0;
  double   end_range_offset   = 0.0;
};

struct SDRecommendLGSegment {
  uint32_t              start_offset = 0;
  uint32_t              end_offset   = 0;
  std::vector<uint32_t> lane_seqs    = {};
  uint32_t              lane_num     = 0;
};

struct SDRecommendLaneGroup {
  uint64_t                          lane_group_id  = 0;
  std::vector<SDRecommendLGSegment> recommend_lane = {};
  std::vector<SDRecommendLGSegment> available_lane = {};
};

struct SDSectionInfo {
  uint64_t                                      id                          = 0;
  double                                        length                      = 0;  //uint: m
  uint64_t                                      lane_num                    = 0;
  std::shared_ptr<std::vector<Eigen::Vector2f>> points                      = std::make_shared<std::vector<Eigen::Vector2f>>();
  SDRoadClass                                   road_class                  = SDRoadClass::SD_INVALID;
  std::vector<uint64_t>                         predecessor_section_id_list = {};
  std::vector<uint64_t>                         successor_section_id_list   = {};
  uint32_t                                      link_type                   = 0;  //与SDLinkTypeMask按位与判断
  double                                        speed_limit                 = 0;  // uint: m/s
  bool                                          has_toll_station            = false;
  SDDirectionType                               direction                   = SDDirectionType::UNKNOWN;
  uint32_t                                      reverse_lane_count          = 0;
  double                                        exp_speed                   = 0;
  bool                                          is_mpp_section              = false;
  std::vector<SDLaneGroupIndex>                 lane_group_idx              = {};     // 0319新增成员
  bool                                          has_junction                = false;  // 0319新增成员
  uint32_t                                      link_type_extend            = 0;
};

enum class SDPathType { NONE = 0, SPLIT = 2, MERGE = 3 };

struct SDSubPath {
  uint64_t                   enter_section_id = 0;  // 汇入时为汇入主路的link， 汇出时为汇出主路的link
  std::vector<SDSectionInfo> sections         = {};
  SDPathType                 path_type        = SDPathType::NONE;
};

struct SDRouteInfo {
  std::string                       id = "";
  NaviPosition                      navi_start;  //from currrent section
  std::vector<SDSectionInfo>        mpp_sections        = {};
  std::vector<SDSubPath>            subpaths            = {};
  std::vector<SDRecommendLaneGroup> recommend_lane_list = {};
};

struct SDLaneGroupInfo {
  uint64_t                          id                         = 0;
  double                            length                     = 0.0;
  std::vector<uint64_t>             successor_lane_group_ids   = {};
  std::vector<uint64_t>             predecessor_lane_group_ids = {};
  uint32_t                          lane_num                   = 0;
  std::vector<LaneInfo>             lane_info                  = {};
  std::vector<SDRecommendLGSegment> recommend_lane             = {};
};

struct Trajectory {
  uint64_t              id               = 0;
  uint64_t              lane_id          = 0;
  uint64_t              start_lane_id    = 0;
  uint64_t              end_lane_id      = 0;
  std::vector<uint64_t> relative_lane_id = {};
  std::vector<Point>    points           = {};
};

enum class SpecialType : uint64_t {
  /*default: normal*/
  ST_NORMAL = 0,
  /*LD map low precision */
  ST_LD_MAP_LOW_PRECISION = 1 << 0,
  /*Switch to LD map but no map data */
  ST_LD_MAP_NO_DATA = 1 << 1,
  /*From PnC's downgrading.*/
  ST_PNC_STATE_DOWNGRADE = 1 << 2,
  /*Two lane linking raises exception.*/
  ST_LINKED_EXCEPTION = 1 << 3,
  /*LD map line fitting failed.*/
  ST_MAP_LINE_FITTING_FAILURE = 1 << 4,
  /*RTK location signal is invalid.*/
  ST_RTK_SIGNAL_INVALID_IN_TUNNUL = 1 << 5,
  /*In ramp navigation deviation.*/
  ST_NAVIGATION_DEVIATION_IN_RAMP = 1 << 6
};

enum SensorWarningType {
  NO_WARNING = 0,
  CLOSE_TO_UNRELIABLE_ROAD = 1,
  SWITCH_TO_HD_MAP_FAILURE = 2,
};

struct SensorStatusInfo {
  /*2: NOA, 1: ICC/TJA*/
  uint32_t sensor_status = 0;
  /*For each type in 'SpecialType' use bitwise operation(OR).*/
  uint64_t special_type = 0;
  SensorWarningType warning_type = SensorWarningType::NO_WARNING;
};

struct TrafficLightStatus {
  LightStatus           light_status{LightStatus::NONE_LIGHT};
  TurnType              turn_type;
  std::vector<Point2D>  stop_line_pts     = {};
  std::vector<Point2D>  stop_line_pts_wait = {};
  uint32_t              traffic_light_num = 0;
  std::vector<uint32_t> lane_ids          = {};
  bool                  is_navi_light     = false;
  uint64_t              traffic_obj_id{0};
  uint64_t              perception_sequence{0};
  uint64_t              stay_prev_counter{0};
  uint64_t              invalid_counter{0};
  bool                  stopline_is_virtual = false;
  double                distance_to_stopline{0.0};
  bool                  traffic_match_cloud_file{false};

  byd::msg::orin::routing_map::LaneInfo::TrafficSetReason traffic_reason{byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE};
};

struct RoutingMap {
  cem::message::common::Header  header;
  MapInfo                       map_info;
  MapType                       type;
  std::vector<LaneInfo>         lanes           = {};
  std::vector<LaneBoundaryInfo> lane_boundaries = {};
  std::vector<RoadBoundaryInfo> road_boundaries = {};
  std::vector<StopLineInfo>     stop_lines      = {};
  std::vector<JunctionInfo>     junctions       = {};
  std::vector<CrossWalkInfo>    cross_walks     = {};
  std::vector<Arrows>           arrows          = {};  // bev_map 箭头
  RouteInfo                     route;
  RoadClass                     cur_road_class;
  std::vector<TrafficLightMap>  traffic_lights = {};
  bool                          is_on_highway  = false;
  EnvInfo                       env_info;
  SDRouteInfo                   sd_route;
  std::vector<SDLaneGroupInfo>  sd_lane_groups   = {};  // 0319新增成员
  std::vector<Trajectory>       exp_trajectories = {};
  FusionSpeedInfo               speed_info       = {};
  SensorStatusInfo              sensor_status_info;
  std::string                   debug_infos = "";
  TrafficLightStatus            traffic_light_uturn;
  TrafficLightStatus            traffic_light_left;
  TrafficLightStatus            traffic_light_straight;
  TrafficLightStatus            traffic_light_right;
};

struct TrafficLightsE2EInfo {
  cem::message::common::Header    header;
  std::vector<TrafficLightStatus> traffic_status = {};
  std::vector<TrafficLightMap>    traffic_lights = {};

  Point3D            pose;
  TrafficLightStatus traffic_light_uturn;
  TrafficLightStatus traffic_light_left;
  TrafficLightStatus traffic_light_straight;
  TrafficLightStatus traffic_light_right;
};

}  // namespace env_model
}  // namespace message
}  // namespace cem

#endif
