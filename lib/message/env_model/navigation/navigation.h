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
 * @file navigation.h
 * @author lingpeng (ling.peng3@byd.com)
 * @brief
 * @version 0.1
 * @date 2025-04-15
 */
#ifndef NAVIGATION_H_LP_
#define NAVIGATION_H_LP_

#include <sys/types.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "lib/message/env_model/routing_map/routing_map.h"
#include "modules/common/math/vec2d.h"

namespace cem::fusion::navigation {
typedef std::shared_ptr<std::vector<Eigen::Vector2f>> Vec2fVector;

using byd::common::math::Vec2d;
using cem::message::env_model::TurnType;

enum class NaviMatchType {
  Unknow = 0,
  Type1  = 1,   // 非直行的 小T型路口
  Type2  = 2,   // 主辅路分叉 RoadSplit
  Type3  = 3,   // 路口 粗匹配
  Type4  = 4,   // 路口 精匹配
  Type5  = 5,   // 路口 虚拟引导线
  Type6  = 6,   // 全部推荐
  Type7  = 7,   // UTurn
  Type8  = 8,   /// protect
  Type9  = 9,   // 主辅路直行
  Type10 = 10,  // GuideLanesWithSd
};

enum class JunctionType {
  Unknow            = 0,
  RampInto          = 1,  ///下匝道
  ApproachRampInto  = 2,  ///途径下匝道
  RampMerge         = 3,  ///匝道汇入
  ApproachRampMerge = 4,  ///途径匝道汇入
  RampSplitLeft     = 5,  ///匝道内分流向左
  RampSplitRight    = 6,  ///匝道内分流向右
  RampSplitMiddle   = 7,  ///匝道内分流向中
  MainRoadSplitLeft     = 8,  ///main road内分流向左
  MainRoadSplitRight    = 9,  ///main road内分流向右
  MainRoadSplitMiddle   = 10,  ///main road内分流向中
  ApproachRampMergeLeft = 11 ///途径匝道左侧汇入
};

struct SDLaneNumInfo {

  uint64_t cur_lg_id                   = 0;
  uint64_t next_lg_id                  = 0;
  uint32_t cur_lane_num                = 0;
  uint32_t next_lane_num               = 0;
  double   next_lg_offset              = 0;  //到达下一lg的距离
  uint64_t next_lane_num_change        = 0;
  double   next_lane_num_change_offset = 0;  //到达下一lane_num_change的lg的距离
};

struct JunctionBevFeature {

  uint64_t     junction_id   = 0;
  JunctionType junction_type = JunctionType::Unknow;

  uint32_t        ego_left_road_edge_id     = 0;
  Eigen::Vector2f ego_left_road_edge_point  = {0, 0};  //道路边界形点的尾点
  double          left_update_offset        = 0;       //左侧道路边界更新时候offset
  uint32_t        ego_right_road_edge_id    = 0;
  Eigen::Vector2f ego_right_road_edge_point = {0, 0};
  double          right_update_offset       = 0;  //右侧道路边界更新时候offset
};

enum class LineSortType {
  LaneLine     = 0,  //中心线
  RoadBoundary = 1,  //路岩
  VirtualEdge  = 2,  //导流区虚拟路岩
  LateralDist  = 3   //大间距
};

struct LineSort {
  LineSort(uint32_t ID, bool IsRoadBoundary) : id(ID), is_road_boundary(IsRoadBoundary) {}

  uint32_t id;
  bool     is_road_boundary;
};

struct LineSortCity {
  uint32_t     id;
  LineSortType type;
  LineSortCity(uint32_t id_val, LineSortType type_val) : id(id_val), type(type_val) {}
};

struct BevSectionInfo {
  uint64_t              id                          = 0;
  double                length                      = 0;
  double                start_range_offset          = 0.0;
  double                end_range_offset            = 0.0;
  uint64_t              lane_num                    = 0;
  std::vector<uint64_t> lane_ids                    = {};
  double                left_edge_distance          = -1.0;
  double                right_edge_distance         = -1.0;
  std::vector<uint64_t> predecessor_section_id_list = {};  //预留
  std::vector<uint64_t> successor_section_id_list   = {};  //预留
};

struct SeparatorInfo {
  std::vector<uint64_t>                                    edge_ids;     // 分割路岩ID列表
  std::vector<LineSortType>                                types;        // 分割路岩类型列表
  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> edge_points;  // 每个分割路岩的起终点
};

struct BevRouteInfo {
  std::vector<BevSectionInfo> sections;
  std::vector<SeparatorInfo>  separators;
  bool                        filtered_non_ego_lanes = false;
};

enum class SplitDirection {
  Straight = 0,
  SplitLeft,
  SplitRight,
};

struct JunctionInfo {
  uint64_t              junction_id     = 0;
  JunctionType          junction_type   = JunctionType::Unknow;  ///地图导航路口类型
  double                offset          = 0;                     ///自车距离路口特征坐标点的纵向距离
  double                diff_offset     = 0;                     ///junction距离next junction距离
  int                   split_num       = 0;                     ///匝道内分流数量
  SplitDirection        split_direction = SplitDirection::Straight;
  bool                  is_left_ahead_of_time{false};  // only for split
  std::vector<uint64_t> junction_ids;                  // junction关联的所有ids

  int main_road_lane_nums   = 0;  ///自车所在路的车道数量（不包含应急车道等非机动车可行驶车道）
  int road_merge_main_road_lane_nums   = 0;  ///主路的车道数量（不包含应急车道等非机动车可行驶车道）
  int target_road_lane_nums = 0;  ///分叉目标道路的车道数量（不包含应急车道等非机动车可行驶车道）

  bool has_passed_flag   = false;  ///自车是否已经经过了此路口
  bool has_effected_flag = false;  ///此路口是否开始作用生效  sd_junction_infos_中非passed junction才可置此标志位为true
  int  main_road_navigation_lane_num =0;//only use in high road split
};

struct Oppo_lanes_to_move {
  int                   keep_count = 0;
  std::vector<uint64_t> oppo_lane_ids;
};

struct Split_lanes_to_move {
  int                   keep_count = 0;
  std::vector<uint64_t> split_lane_ids;
};

/* 施工区域的保持结构 */
struct ConstructionZoneInfo {
  bool pre_ConstructionZone       = false;  //上一帧的施工状态
  bool pre_final_ConstructionZone = false;  //上一帧的施工状态,未使用
  int  check_count                = 0;
  int  keep_count                 = 0;
};


struct LaneSideInfo {
  bool                                   exists = false;  // 该侧是否存在应急车道
  std::vector<std::pair<double, double>> ranges;          // 应急车道的多个区间 [start_s, end_s]
  int                                    lane_num = 0;    // 该区间对应LaneGroup的车道总数
};

struct EmergencyLaneInfo {
  LaneSideInfo left{};   // 左侧应急车道信息
  LaneSideInfo right{};  // 右侧应急车道信息
};

struct HarborStopInfo {
  bool                                   exists = false;         // 是否存在港湾停靠站
  std::vector<std::pair<double, double>> ranges;                 // 港湾停靠站的区间 [start_s, end_s]
  bool                                   is_left_most  = false;  // 最左
  bool                                   is_right_most = false;  // 最右
  bool                                   is_obstacle_harbor = false;  // 港湾车道左侧为路沿
};

struct BusLaneInfo {
  bool                                   exists = false;  // 是否存在公交车道
  std::vector<std::pair<double, double>> ranges;          // 公交车道的区间 [start_s, end_s]
  int                                    index;
  bool                                   is_left_most  = false;  // 最左
  bool                                   is_right_most = false;  // 最右
  bool                                   is_passable = true;  // 是否可通行
  uint32_t                               passable_env_state = 0;  // 是否可通行 0：未知 ；1：可通行； 2：不可通行
};


enum class AccLanePosition {
    UNKNOWN = 0,
    ACC_LEFT_SIDE,   // 加速车道在道路左侧（少见）
    ACC_RIGHT_SIDE   // 加速车道在道路右侧（常见）
};

enum class AccLaneMergeDir {
    ACC_MERGE_UNKNOWN = 0,
    ACC_MERGE_LEFT,   // 加速车道向左侧merge
    ACC_MERGE_RIGHT   // 加速车道向右侧merge
};
// 加速车道信息结构体
struct AccLaneInfo {
  bool                                   exists        = false;  // 是否存在加速车道
  bool                                   is_left_most  = false;  // 是否是车道组最左车道
  bool                                   is_right_most = false;  // 是否是车道组最右车道
  std::vector<std::pair<double, double>> ranges;                 // 加速车道区间 [start_s, end_s]（相对自车）
  int                                    normal_lane_num = 0;    // 该区间对应LaneGroup的普通车道总数
  uint64_t                               ego_normal_lane_id = 0; //加速车道邻车道在自车section的id
  double                                 merge_start_dis    = std::numeric_limits<double>::max();//加速车道起点距离
  double                                 merge_end_dis      = std::numeric_limits<double>::max();//加速车道终点距离
  AccLaneMergeDir                        merge_dir          = AccLaneMergeDir::ACC_MERGE_UNKNOWN;  //merge 方向
};

// 减速车道信息结构体
struct DecLaneInfo {
  bool                                   exists        = false;  // 是否存在减速车道
  bool                                   is_left_most  = false;  // 是否是车道组最左车道
  bool                                   is_right_most = false;  // 是否是车道组最右车道
  std::vector<std::pair<double, double>> ranges;                 // 减速车道区间 [start_s, end_s]（相对自车）
  int                                    normal_lane_num = 0;    // 该区间对应LaneGroup的普通车道总数
};

struct LaneDetail {
  uint64_t lane_id{0};       // 车道ID
  uint64_t section_id{0};    // 所属section ID
  uint64_t lanegroup_id{0};  // 所属lanegroup ID
  int      lane_index{0};    // 在 lane group 中从左往右的索引（从 0 开始）
};

struct MergeDetail {
  std::vector<LaneDetail>           src_lanes;         // 参与合并的车道列表（merge拓扑车道）
  LaneDetail                        dst_lane;          // 合并后的目标车道
  std::pair<double, double>         merge_lane_range;  // merge拓扑影响的车道线范围 [start_s, end_s]
  message::env_model::MergeTopology type;              // merge类型
  double                            merge_distance;
  std::pair<double, double>         merge_lane_keeptopo_range;
  LaneDetail                        keeptopo_tail_lane;  // 回溯停止位置所在的车道（到自车/变化处）
};

enum class JunctionTypeCity {
  Unknown         = 0,
  RoadSplit       = 1,  ///主辅路汇出
  RoadMerge       = 4,  ///辅路汇入主路
  CrossRoad       = 5,  ///十字路口
  TJunction       = 6,  ///T型路口
  SmallTJunction  = 7,  ///T型小路口  路口内有车道线
  InvalidJunction = 8,  ///无效路口
  UTurn           = 9,  // U掉头场景
  RING            = 10, //环岛
};

enum class UTurnPosition { LEFTMOST, RIGHTMOST, UNKNOWN };

enum class JunctionAction {
  Unknown    = 0,
  TurnLeft   = 1,
  GoStraight = 2,
  TurnRight  = 3,
  UTurn      = 4,
};

enum class DirectionSplitMerge {
  Unknown  = 0,
  Left     = 1,
  Right    = 2,
  Straight = 3,
};
/*分岔口通过类型*/
enum class SplitPassType {
  Unknown    = 0,
  MainToMain = 1, /*主路到主路*/
  MainToRamp = 2,
  RampToMain = 3,
  RampToRamp = 4,
};

enum class RoadMainType {
  RoadRamp = 0,
  RoadMain = 1,
};

enum class JunctionStateCity {
  PASSING   = 0,
  PASSED    = 1,
  UNREACHED = 2,
  UNKNOWN   = 3,
};

struct JunctionInfoCity : public JunctionInfo {
  JunctionTypeCity          junction_type_city{JunctionTypeCity::Unknown};        ///地图导航路口类型
  JunctionAction            junction_action{JunctionAction::Unknown};             ///路口导航动作
  DirectionSplitMerge       split_merge_direction{DirectionSplitMerge::Unknown};  ///split_merge方向
  std::vector<TurnType>     map_lane_arrows_plan;                                 ///路口前车道箭头排列 从左向右
  uint64_t                  map_plan_turn_type_lane_group_id;                     ///plan turn type corresponding lane group id;
  std::vector<TurnType>     map_lane_arrows_default;                              ///路口前车道箭头排列 从左向右
  std::vector<int>          main_lane_indexs;                                     ///目标车道的index
  std::vector<RoadMainType> succ_road_class;
  std::vector<RoadMainType> prev_road_class;
  JunctionStateCity         junction_state_city{JunctionStateCity::UNKNOWN};
  std::vector<uint64_t>     junction_ids;  // junction关联的所有id。
  double                    time{0};
  std::vector<Vec2d>        junction_envelope_points;
  double                    distance_to_envelope{std::numeric_limits<double>::infinity()};
  bool                      is_dedicated_right_turn_lane{false};
  double                    next_angle{0.0};
  bool                      is_valid{true};
  bool                      need_match_arrow{true};
  bool                      is_only_Turn{false};  //T or Uturn junction,the first lane is only turn,not go stright
  std::size_t               idx_start{0};
  std::size_t               idx_end{0};

  std::vector<uint64_t> split_direction_ids;  // only for split
};

struct VirtualSection {
  uint64_t         id;                     // 虚拟 section 的唯一标识符
  double           start_s;                // 相对自车起点
  double           end_s;                  // 相对自车终点
  uint32_t         lane_num;               // 虚拟 section 的车道数
  std::vector<int> target_lane_indices;    // 连接到路口目标车道的索引列表
  int              left_lane_change  = 0;  // 左侧车道数变化（增加/减少）
  int              right_lane_change = 0;  // 右侧车道数变化（增加/减少）
  int              mid_lane_change   = 0;
};

struct CutMapZone {
  bool             active{false};        // 是否触发切图
  int              sceneType{0};         // 1: 1->3 split; 2: 连续split; 3: split+十字
  uint64_t         anchorJunctionId{0};  // 触发的锚点路口ID（场景1&2为前一个split；场景3为split）
  JunctionTypeCity anchorType{JunctionTypeCity::Unknown};
  double           anchorOffset{std::numeric_limits<double>::infinity()};  // 锚点路口到自车的offset（m）
  double           winBefore{150.0};                                       // 取图前窗口（m）
  double           winAfter{50.0};                                         // 取图后窗口（m）
};

struct RecommendLane {
  uint64_t lanegroup_id;  // only for sd
  uint64_t section_id;    // only for ld
  double                start_offset = 0.0;
  double                end_offset   = 0.0;
  uint64_t              lane_num;
  std::vector<uint32_t> lane_seqs;
  std::vector<uint64_t> lane_ids;  // only for ld
  friend bool operator==(const RecommendLane& a,
                         const RecommendLane& b) noexcept {
    return a.lanegroup_id == b.lanegroup_id && a.lane_num == b.lane_num &&
           a.start_offset == b.start_offset && a.end_offset == b.end_offset &&
           a.lane_seqs == b.lane_seqs;
  }
};

/*车道重叠率*/
struct TurnLaneOverlap {
  int    current_lane_index;
  int    junction_lane_index;
  double overlap_ratio;
};
/* 路口车道变化状态 */
struct LanesChangeStatus {
  bool                  confirmed           = false;  // 车道变化是否已确认
  int                   current_lane_count  = -1;     // 当前车道数
  int                   junction_lane_count = -1;     // 路口车道数
  int                   confirmation_frames = 0;      // 确认帧数计数
  std::vector<int>      recommended_junction_lanes;   // 路口推荐车道索引
  std::vector<uint64_t> guide_lanes;                  // 最终推荐的bev车道ID
};
/* 相邻路口 */
struct AdjacentJunctionInfo {
  uint64_t         junction_id;                     // 路口ID
  double           offset;                          // 路口偏移量
  JunctionTypeCity junction_type;                   // 路口类型
  bool             is_adjacent_to_dedicated_right;  // 是否与右转专用道相邻
};

struct HistSelectRoadInfo {
  std::vector<uint64_t> separator_ids = {};  // 路沿id
  bool is_left = true;  // true：section在路沿左侧, false：section在路沿右侧
  std::vector<uint64_t> road;  // 实际选出的 road id
  bool operator==(const HistSelectRoadInfo &rhs) const noexcept {
    return separator_ids == rhs.separator_ids && is_left == rhs.is_left;
  }  // 只要「路沿id + 左右侧」相同就认为位置相同
};
struct MergeTopoInfo {
  double distance_to_ego;
  uint32_t type;
};
}  // namespace cem::fusion::navigation
#endif
