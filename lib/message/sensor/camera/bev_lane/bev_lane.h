#ifndef BEV_LANE_H_
#define BEV_LANE_H_

#include <cstdint>
#include <vector>
#include <message/sensor/vision/tsrmobject.h>

#include "Eigen/Dense"
#include "lib/common/utils/KDTreeUtil.h"
#include "message/common/geometry.h"
#include "message/common/header.h"
#include "message/env_model/routing_map/routing_map.h"
#include "message/env_model/speed_limit/speed_limit.h"
#include "modules/msg/orin_msgs/routing_map.pb.h"

namespace cem {
namespace message {
namespace sensor {

enum class MapType { UNKOWN_MAP = 0, PERCEPTION_MAP = 1, HD_MAP = 2, BEV_MAP = 3 };

enum BevLaneMarkerType {
  BEV_LMT__UNDECIDED            = 0,
  BEV_LMT__DASHED               = 1,
  BEV_LMT__SOLID                = 2,
  BEV_LMT__DOUBLE_DASHED_DASHED = 3,
  BEV_LMT__DOUBLE_SOLID_SOLID   = 4,
  BEV_LMT__DOUBLE_DASHED_SOLID  = 5,
  BEV_LMT__DOUBLE_SOLID_DASHED  = 6,
  BEV_LMT__LEFT_DIVERSION       = 7,
  BEV_LMT__RIGHT_DIVERSION      = 8,
  BEV_LMT__INVALID              = 9,
  BEV_RMT__CROSSWALK            = 10,
  BEV_RMT__STRAIGHT             = 11,
  BEV_RMT__LEFT                 = 12,
  BEV_RMT__RIGHT                = 13,
  BEV_RMT__TURNING              = 14,
  BEV_RMT__STRAIGHT_LEFT        = 15,
  BEV_RMT__STRAIGHT_RIGHT       = 16,
  BEV_RMT__STRAIGHT_LEFT_RIGHT  = 17,
  BEV_RMT__LEFT_RIGHT           = 18,
  BEV_RMT__STRAIGHT_TURNING     = 19,
  BEV_RMT__LEFT_TURNING         = 20,
  BEV_RMT__IMPORT               = 21,
  BEV_RMT__EXPORT               = 22,
  BEV_RMT__STOPLINE             = 23,
  RM_TYPE_DECELERATION_ZONE     = 24,
  RM_TYPE_DIVERSION_ZONE        = 25,
  RM_TYPE_INTERSECTION_ZONE     = 26,
  BEV_LMT__NUM                  = 27
};

enum BevLaneMarkerColor {
  BEV_LMC__WHITE   = 0,
  BEV_LMC__YELLOW  = 1,
  BEV_LMC__UNKNOWN = 2,
  BEV_LMC__BLUE    = 3,
  BEV_LMC__GREEN   = 4,
  BEV_LMC__RED     = 5
};

enum BevEdgeType { BEV_ET__STOPLANE = 9, BEV_ET__LOWEDGE = 1, BEV_ET__HIGHEDGE = 2 };

enum BevRoadEdgePosition { BEV_REP__UNKNOWN = 0, BEV_REP__LEFT = 1, BEV_REP__RIGHT = 2 };

enum class BevLanePosition {
  LANE_LOC_EGO           = 0,
  LANE_LOC_LEFT_FIRST    = 1,
  LANE_LOC_RIGHT_FIRST   = 2,
  LANE_LOC_LEFT_SECOND   = 3,
  LANE_LOC_RIGHT_SECOND  = 4,
  LANE_LOC_LEFT_THIRD    = 5,
  LANE_LOC_RIGHT_THIRD   = 6,
  LANE_LOC_OTHER         = 7,
  LANE_LOC_LEFT_FOURTH   = 8,
  LANE_LOC_RIGHT_FOURTH  = 9,
  LANE_LOC_LEFT_FIFTH    = 10,
  LANE_LOC_RIGHT_FIFTH   = 11,
  LANE_LOC_LEFT_SIXTH    = 12,
  LANE_LOC_RIGHT_SIXTH   = 13,
  LANE_LOC_LEFT_SEVENTH  = 14,
  LANE_LOC_RIGHT_SEVENTH = 15
};

enum class BevLaneDirection {
  DIRECTION_UNKNOWN   = 0,
  DIRECTION_TWOWAY    = 1,
  DIRECTION_FORWARD   = 2,
  DIRECTION_BACKWARD  = 3,
  DIRECTION_NO_TWOWAY = 4
};

enum class BevLaneType {
  LANE_TYPE_UNKNOWN          = 0,
  LANE_TYPE_MAIN             = 1,
  LANE_TYPE_SIDE             = 2,
  LANE_TYPE_EMERGENCY        = 3,
  LANE_TYPE_OTHER            = 4,
  LANE_TYPE_BLOCKED          = 5,
  LANE_TYPE_EMERGENCY_STOP   = 6,
  LANE_TYPE_VIRTUAL_JUNCTION = 7,
  LANE_TYPE_HARBOR_STOP      = 8,
  LANE_ACC                   = 9,
  LANE_ACC_ADJ               = 10,
  LANE_DEC                   = 11,
  LANE_BRT                   = 12,
  LANE_REVERSIBLE            = 16
};

enum class BevLanePositionToCrossPoint {
  UNKNOWN      = 0,
  TOP          = 1,
  BOTTOM       = 2,
  LEFT         = 3,
  RIGHT        = 4,
  TOP_LEFT     = 5,
  BOTTOM_LEFT  = 6,
  TOP_RIGHT    = 7,
  BOTTOM_RIGHT = 8
};

enum class BevArrowType {
  ARROW_UNKNOWN              = 0,
  ARROW_STRAIGHT             = 1,
  ARROW_LEFT_TURN            = 2,
  ARROW_RIGHT_TURN           = 3,
  ARROW_TURN_AROUND          = 4,
  ARROW_STRAIGHT_LEFT_TURN   = 5,
  ARROW_STRAIGHT_RIGHT_TURN  = 6,
  ARROW_LEFT_RIGHT_TURN      = 7,
  ARROW_STRAIGHT_TURN_AROUND = 8,
  ARROW_LEFT_TURN_AROUND     = 9,
  ARROW_ALL                  = 10
};

enum class BevLaneConnectType { NORMAL = 0, MERGE = 1, SPLIT = 2 };

enum class SplitTopoExtendType {
  TOPOLOGY_SPLIT_NONE    = 0,
  TOPOLOGY_SPLIT_LEFT    = 1,  // split to left
  TOPOLOGY_SPLIT_RIGHT   = 2,  // split to right
  TOPOLOGY_SPLIT_UNKNOWN = 3
};

enum class MergeTopoExtendType {
  TOPOLOGY_MERGE_NONE    = 0,
  TOPOLOGY_MERGE_LEFT    = 1,  // merge to left, ego is on right of target lane
  TOPOLOGY_MERGE_RIGHT   = 2,  // merge to right, ego is on left of target lane
  TOPOLOGY_TO_BE_MERGED  = 3,
  TOPOLOGY_MERGE_UNKNOWN = 4
};

enum class MergeSourceExtend { MERGE_UNKNOWN = 0, MERGE_BEV = 1, MERGE_LD = 2, MERGE_SD = 3 };

struct MergeInfoExtend {
  bool              merge_valid  = 0;
  MergeSourceExtend merge_source = MergeSourceExtend::MERGE_BEV;
  double            dis_to_merge = 0.0;
};

enum class BevLaneIntersectionType { IS_TYPE_INVALID = 0, IS_TYPE_EXIT = 1, IS_TYPE_ENTRANCE = 2 };

enum class BevTrafficLightState {
  TL_COLOR_UNKNOWN       = 0,
  TL_COLOR_RED           = 1,
  TL_COLOR_YELLOW        = 2,
  TL_COLOR_GREEN         = 3,
  TL_COLOR_GREEN_FLASH   = 4,
  TL_COLOR_RED_FLASH     = 5,
  TL_COLOR_YELLOW_FLASH  = 6,
  TL_COLOR_BLURRING_MODE = 7,
  TL_COLOR_BLOCK_FAILED  = 8,
  TLC_CLOUD_NOMATCH      = 9,
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

enum class BevTurnType {
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

enum class BevAction {
  UNKNOWN    = 0,
  LEFT_TURN  = 1,
  STRAIGHT   = 2,
  RIGHT_TURN = 3,
  U_TURN     = 4,
};

struct BevLmTypeSeg {
  BevLaneMarkerType type         = BevLaneMarkerType::BEV_LMT__UNDECIDED;
  float             start_offset = 0.0f;
  float             end_offset   = 0.0f;
  int               start_index  = 0;
  int               end_index    = 0;
};

struct BevLmColorSeg {
  BevLaneMarkerColor color        = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
  float              start_offset = 0.0f;
  float              end_offset   = 0.0f;
  int                start_index  = 0;
  int                end_index    = 0;
};

struct BevLmMergedSeg {
  BevLaneMarkerType  type         = BevLaneMarkerType::BEV_LMT__UNDECIDED;
  BevLaneMarkerColor color        = BevLaneMarkerColor::BEV_LMC__UNKNOWN;
  int                start_index  = 0;
  int                end_index    = 0;
  float              start_offset = 0.0f;
  float              end_offset   = 0.0f;
};

struct BevLaneMarker {
  uint64_t                                      id                = 0;
  uint8_t                                       number_of_points  = 0;
  uint8_t                                       position          = 0;
  uint32_t                                      type              = 0;
  uint8_t                                       color             = 0;
  float                                         conf              = 0.0f;
  bool                                          is_virtual        = false;
  bool                                          is_associated_bev = false;
  BevRoadEdgePosition                           road_edeg_pos     = BevRoadEdgePosition::BEV_REP__UNKNOWN;
  std::vector<cem::message::common::Point2DF>   line_points       = {};
  std::vector<cem::message::common::Point2DF>   line_points_ego   = {}; // 当前junction，crosswalk，stopline填充相应字段
  std::vector<BevLmTypeSeg>                     type_segs         = {};
  std::vector<BevLmColorSeg>                    color_segs        = {};
  std::vector<BevLmMergedSeg>                   merged_segs       = {};
  std::shared_ptr<std::vector<Eigen::Vector2f>> geos              = std::make_shared<std::vector<Eigen::Vector2f>>();
  KDTreeUtil::IndexedGeos                       indexed_geos{};
};

enum class Bev_RestrictedType : uint8_t {
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


struct Bev_RestrictedInfo {
    Bev_RestrictedType restricted_type = Bev_RestrictedType::RT_NONE;
    bool is_passable = false;
    uint32_t passable_env_state = 0;
};

struct BevMapSectionInfo; //提前引用声明，用于定义BevLaneInfo::section
struct BevLaneInfo {
  uint64_t id                   = 0;
  uint64_t section_id           = 0;
  uint64_t left_lane_id         = 0;
  uint64_t right_lane_id        = 0;
  uint64_t road_id              = 0;
  uint64_t junction_id          = 0;
  uint32_t left_lane_marker_id  = 0;
  uint32_t right_lane_marker_id = 0;
  uint32_t number_of_points     = 0;
  uint32_t position             = 0;
  uint32_t connect_score        = 0;
  double   length               = 0;
  double   speed_limit          = 0;  // uint: m/s
  float    width                = 0.0f;
  float    start_dx             = 0;
  float    end_dx               = 0;
  float    conf                 = 0.0f;

  std::vector<uint64_t> left_lane_boundary_ids  = {};
  std::vector<uint64_t> right_lane_boundary_ids = {};
  std::vector<uint64_t> left_road_boundary_ids  = {};
  std::vector<uint64_t> right_road_boundary_ids = {};
  std::vector<uint64_t> previous_lane_ids       = {};
  std::vector<uint64_t> next_lane_ids           = {};
  std::vector<uint64_t> stopline_ids            = {};

  std::vector<cem::message::common::Point2DF>   line_points = {};
  std::shared_ptr<std::vector<Eigen::Vector2f>> geos        = std::make_shared<std::vector<Eigen::Vector2f>>();

  BevLaneDirection            direction               = BevLaneDirection::DIRECTION_UNKNOWN;
  BevLaneType                 lane_type               = BevLaneType::LANE_TYPE_UNKNOWN;
  BevArrowType                arrow_type              = BevArrowType::ARROW_UNKNOWN;
  BevTurnType                 plan_turn_type          = BevTurnType::OTHER_UNKNOWN;
  BevLaneConnectType          connect_type            = BevLaneConnectType::NORMAL;
  BevLaneIntersectionType     intersection_type       = BevLaneIntersectionType::IS_TYPE_INVALID;
  BevLanePositionToCrossPoint position_to_cross_point = BevLanePositionToCrossPoint::UNKNOWN;
  SplitTopoExtendType         split_topo_extend       = SplitTopoExtendType::TOPOLOGY_SPLIT_NONE;
  MergeTopoExtendType         merge_topo_extend       = MergeTopoExtendType::TOPOLOGY_MERGE_NONE;
  MergeInfoExtend             merge_info_extend;
  KDTreeUtil::IndexedGeos     indexed_geos{};

  bool                 is_horizontal_lane = false;
  bool                 is_blocked         = false;
  bool                 is_flow_map        = false;
  bool                 is_virtual         = false;
  bool                 is_junction_lane   = false;
  bool                 is_mounted         = false;
  bool                 is_compensation    = false;
  bool                 is_bev_topo_connected = false;
  bool                 is_topological_connection = false;
  bool                 is_topo_disassembled = false; // split 拓扑是否拆开
  bool                 is_build_split_by_map = false; //是否构建地图匹配的split拓扑，给下游添加拓扑+形点判定使用
  bool                 is_build_split_marker_by_map = false; //是否构建地图匹配的split拓扑标签
  bool                 is_build_merge_marker_by_map = false; //是否构建地图匹配的merge拓扑标签
  bool                 is_build_merge = false; //是否构建与地图无关的merge拓扑
  bool                 is_build_merge_left = false;//是否构建merge拓扑到左边相邻车道,true: left, false: right
  bool                 is_build_split = false; //是否是依赖地图和地图与bev匹配关系构建split拓扑
  bool                 is_build_split_left = false;//构建的split在自车道左边还是右边,true: left, false: right
  BevAction            navi_action        = BevAction::UNKNOWN;
  BevAction            bev_turn_type      = BevAction::UNKNOWN;

  BevTrafficLightState trafficlight_state = BevTrafficLightState::TL_COLOR_UNKNOWN;
  uint32_t             traffic_light_num  = 0;
  uint32_t             traffic_light_obj_id{0};
  uint32_t             traffic_light_seq_num{0};
  std::vector<uint64_t> traffic_stop_lines = {};
  std::vector<uint64_t> traffic_crosswalks = {};
  uint32_t             stay_prev_counter{0};
  uint32_t              stopline_angle_flag{0};
  double                yellow_flashing_start_time{0.0};

  byd::msg::orin::routing_map::LaneInfo::TrafficSetReason traffic_set_reason{byd::msg::orin::routing_map::LaneInfo::UNKNOWN_STATE};

  bool is_default_arrow = false;
  bool is_acc_adj_lane = false; //是否是加速车道邻车道
  double merge_start_dis =  std::numeric_limits<double>::max(); //汇流区起点距自车的距离，当is_acc_adj_lane为true时有效
  double merge_end_dis =  std::numeric_limits<double>::max();  //汇流区终点距自车的距离，当is_acc_adj_lane为true时有效
  Bev_RestrictedInfo restricted_info;
  bool operator<(const BevLaneInfo& other) const {
      if (id != other.id) {
          return id < other.id;
      }
  }
};

struct BevMapNaviPosition {
  uint64_t section_id = 0;
  double   s_offset   = 0.0;
};

struct BevMapSectionInfo {
  uint64_t                                    id          = 0;
  double                                      length      = 0.0;  //uint: m
  std::vector<cem::message::common::Point2DF> points      = {};
  std::vector<BevLaneMarker>                  lanemarkers = {};  // lane
  std::vector<BevLaneInfo>                    lane_infos  = {};  // lane center
  Eigen::Vector3d                             line_func_ego = {0, 0, 0};  // section末端切割线的系数A、B、C，对应在自车坐标系的直线方程Ax+By+C=0
  Eigen::Vector3d                             front_left    = {0, 0, 0};  // section起点切割线的左点
  Eigen::Vector3d                             front_right   = {0, 0, 0};  // section起点切割线的右点
  Eigen::Vector3d                             back_left     = {0, 0, 0};  // section终点切割线的左点
  Eigen::Vector3d                             back_right    = {0, 0, 0};  // section终点切割线的右点
};

struct BevMapSubPathInfo {
  uint64_t                       enter_section_id = 0;
  std::vector<BevMapSectionInfo> sections = {};
};

struct BevMapRouteInfo {
  uint64_t                       id         = 0;
  BevMapNaviPosition             navi_start = {};
  std::vector<BevMapSectionInfo> sections   = {};
  std::vector<BevMapSubPathInfo> subpaths   = {};
};

struct BevMapInfo {
  cem::message::common::Header     header;
  int32_t                          frame_id         = 0;
  uint16_t                         lanemarkers_num  = 0;
  uint16_t                         edge_num         = 0;
  bool                             is_local_pose    = false;
  bool                             is_on_highway    = false;
  MapType                          map_type         = MapType::UNKOWN_MAP;
  RoadClass                        road_class       = RoadClass::UNKNOWN;
  std::vector<BevLaneMarker>       lanemarkers      = {};  // lane
  std::vector<BevLaneMarker>       edges            = {};
  std::vector<BevLaneInfo>         lane_infos       = {};  // lane center
  std::vector<BevLaneMarker>       stop_lines       = {};
  std::vector<BevLaneMarker>       crosswalks       = {};
  std::vector<BevLaneMarker>       arrows           = {};
  std::vector<BevLaneMarker>       junctions        = {};
  std::vector<BevLaneMarker>       diversion_zone   = {};
  BevMapRouteInfo                  route            = {};
  env_model::FusionSpeedInfo       speed_info       = {};
  cem::message::env_model::EnvInfo env_info;
  std::string debug_infos = "";
};

struct LidarRoadEdgeInfo {
  cem::message::common::Header header;
  std::vector<BevLaneMarker>   edges = {};
};

}  // namespace sensor
}  // namespace message
}  // namespace cem

#endif
