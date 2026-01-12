#ifndef COARSEMATCHING_H
#define COARSEMATCHING_H

#include <vector>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include "BevDataProcessor.h"
#include "NavigationParams.h"
#include "SDMapTopoExtract.h"
#include "lib/common/log_custom.h"
#include "lib/message/env_model/routing_map/routing_map.h"

namespace cem {
namespace fusion {
namespace navigation {

class CoarseMatching {
 public:
  CoarseMatching() = default;

  // Unknow = 0,
  // Type1  = 1,  // 非直行的 小T型路口
  // Type2  = 2,  // 主辅路分叉 RoadSplit
  // Type3  = 3,  // 路口 粗匹配
  // Type4  = 4,  // 路口 精匹配
  // Type5  = 5,  // 路口 虚拟引导线
  // Type6  = 6,  // 全部推荐
  // Type7  = 7,  // UTurn
  // Type8  = 8
  std::vector<uint64_t> CoarseMatchingType1(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected);
  std::vector<uint64_t> CoarseMatchingType2(const std::vector<JunctionInfoCity> &junctions_info_city, const JunctionInfoCity &junction,
                                            const std::vector<uint64_t> &road_selected, bool isSplitRoadSelectWorked,
                                            const BevRouteInfo &complete_section_info);
  std::vector<uint64_t> CoarseMatchingType3(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected);
  std::vector<uint64_t> CoarseMatchingType33(const std::vector<JunctionInfoCity> &junctions_info_city, const JunctionInfoCity &junction,
                                             const std::vector<uint64_t> &road_selected, const BevRouteInfo &route_info);
  std::vector<uint64_t> CoarseMatchingType7(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected);

  bool                              IsRoadSplitMain2Ramp(const JunctionInfoCity &junction);
  std::vector<AdjacentJunctionInfo> adjacent_junctions_info_;  //相邻路口信息
  // 获取自车当前所在 LaneGroup 的车道数
  int GetCurrentLaneNum();

  // 获取车道完整几何形状的起点和终点x坐标，避免重复计算相同前继车道
  std::pair<double, double> GetLaneStartEndS(uint64_t lane_id, std::unordered_set<uint64_t> &processed_lanes);
  std::pair<double, double> GetLaneStartEndS2(uint64_t lane_id, std::unordered_set<uint64_t> &processed_lanes);
  /// @brief 收集前继车道
  /// @param start_id
  /// @return
  std::unordered_set<uint64_t> CollectPreLanes(uint64_t start_id) const;

  bool HasCommonAncestorForThreeLanes(const std::vector<uint64_t> &overlap_lanes);

  // 检查两条车道是否共享前继车道
  bool CheckLanesShareAncestor(uint64_t lane1, uint64_t lane2);

  std::string PrintProcessedLanes(const std::unordered_set<uint64_t> &lanes);

  SDMapTopologyExtractor topology_extractor_;
  BevDataProcessor       bev_processor_;

 private:
  // 根据转向动作单边推荐
  std::vector<uint64_t> SelectSingleSideLanes(const std::vector<uint64_t> &road_selected, const JunctionInfoCity &junction);

  // 根据推荐车道数选择车道
  std::vector<uint64_t> SelectLanesByCount(const std::vector<uint64_t> &road_selected, int count, bool from_right);

  std::vector<uint64_t> SelectTargetLanes(const std::vector<uint64_t> &road_selected, const std::vector<int> &main_lane_indexs);

  bool HasMainAndRamp(const std::vector<RoadMainType> &succ_road_class);

  // 根据 LaneNum 设计逐级推荐策略
  void apply_gradual_strategy(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected_in, int LaneNum,
                              bool is_right_turn, std::vector<uint64_t> &guide_lanes);

  //获取主路能够顺到Ramp的车道数
  std::pair<int, int> GetReachableLaneCountToRamp(const JunctionInfoCity &junction);

  // 获取只能通到Ramp的车道数
  int GetExclusiveLaneCountToRamp(const JunctionInfoCity &junction, uint64_t target_section_id, uint64_t bev_size);

  int GetMaxNormalLaneCountToJunction(const JunctionInfoCity &junction);

  // 检查路口是否只有一个左转车道
  bool hasSingleLeftTurnLane(const std::vector<TurnType> &junction_turn_types);

  bool hasNoRightTurnLane(const std::vector<TurnType> &junction_turn_types);
  /**
 * 检查转向类型是否包含右转
 * 
 * @param type 
 * @return true or false 
 */
  bool containsRightTurn(TurnType type);
  // 根据路口动作和转向类型获取推荐的路口车道索引
  std::vector<int> getRecommendedJunctionLanes(const JunctionInfoCity &junction_info);
  /**
 * 基于拓扑与感知匹配后的推荐车道
 * @param junction   路口
 * @param road_selected  选路结果
 * @param route_info  感知路段
 * @param check_flag  校验标志位
 * @return guide_lanes 
 */
  std::vector<uint64_t> GetRecommendedLanes(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                            const BevRouteInfo &route_info, bool &check_flag);

  /**
 * 检查路口箭头类型是否全部相同
 * 
 * @param junction 
 * @return true or false 
 */
  bool allSameTurnType(const JunctionInfoCity &junction);
  /**
 * 计算两个区间的重叠率
 * 
 * @param type 
 * @return true or false 
 */
  double calculateOverlapRatio(double c_start, double c_end, double j_start, double j_end);

  // 处理特殊场景的辅助函数
  void processSpecialCase(int current_lane_count, int junction_lane_count, int first_lane_coverage,
                          const std::vector<int> &recommended_junction_lanes, std::vector<TurnLaneOverlap> &overlaps);
  // 处理右转专用道场景的辅助函数
  void processRightTurnLaneCase(int current_lane_count, int junction_lane_count, const std::vector<int> &recommended_junction_lanes,
                                std::vector<TurnLaneOverlap> &overlaps);

  /**
 * 计算重叠率总和并筛选推荐车道
 * 
 * @param junction 
 * @return true or false 
 */
  std::vector<int> calculateAndSelectLanesByOverlap(int current_lane_count, int junction_lane_count,
                                                    const std::vector<int>      &recommended_junction_lanes,
                                                    const std::vector<TurnType> &junction_turn_types, bool b_right_turn_jct);
  /**
 * 十字路口自车位置的推荐车道计算
 * 
 * @param junction 路口信息结构体
 * @return 符合条件的当前车道索引列表 
 */
  std::vector<uint64_t> CrossLaneRecommendation(const JunctionInfoCity &junction_info, const std::vector<uint64_t> &road_selected,
                                                const BevRouteInfo &route_info);

  std::vector<uint64_t> extractGuideLanes(const std::vector<int> &selected_lanes, const std::vector<uint64_t> &road_selected);

  bool matchJunctionSection(const JunctionInfo &junction_info, const BevRouteInfo &route_info, const std::vector<uint64_t> &road_selected);

  bool confirmLanesChange(int current_lane_count, int junction_lane_count);

  void resetLanesChangeStatus();
  // 从路口section中提取推荐车道
  std::vector<uint64_t> extractGuideLanesFromJunctionSection(const std::vector<int> &recommended_junction_lanes,
                                                             const BevRouteInfo     &route_info);
  bool                  IsAdjacentToDedicatedRight(uint64_t current_junction_id);

  bool HasIntermediateRoadMerge(const std::vector<JunctionInfoCity> &junctions_info_city, const JunctionInfoCity &current_junction);

  //std::string PrintProcessedLanes(const std::unordered_set<uint64_t>& lanes);

  bool IsLaneOrPredecessorsProcessed(uint64_t lane_id, const std::unordered_set<uint64_t> &processed_lanes);

  bool IsInterestedJunction(const JunctionInfoCity &junction);

  std::vector<uint64_t> SelectLanesBasedOnDirection(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected);

  LanesChangeStatus g_laneschange_status;       //路口前车道数变化状态
  uint64_t          g_junction_section_id = 0;  // 路口section的ID
  int               g_matching_frames     = 0;  // 匹配成功的帧数计数
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // COARSEMATCHING_H