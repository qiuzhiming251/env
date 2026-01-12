#ifndef SDNAVIGATIONCITY_H
#define SDNAVIGATIONCITY_H

#include <message/env_model/navigation/navigation.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "NavigationParams.h"
#include "RoadSelector.h"
#include "SdNavigationBase.h"
#include "common/nlohmann/json.hpp"
#include "fmt/format.h"
#include "lib/common/utils/lane_geometry.h"
#include "lib/localmap_construction/stopline_mapping.h"
#include "lib/sd_navigation/BevDataProcessor.h"
#include "lib/sd_navigation/CoarseMatching.h"
#include "lib/sd_navigation/SDMapElementExtract.h"
#include "lib/sd_navigation/SDMapTopoExtract.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"

namespace cem {
namespace fusion {
namespace navigation {

using byd::common::math::Polygon2d;
using byd::common::math::Vec2d;
using json = nlohmann::json;

enum class MainLanePos {
  Unknow        = 0,
  NearLeft      = 1,
  NearRight     = 2,
  NearLeftRight = 3,
};

class SdNavigationCity : public SdNavigationBase {

 public:
  SdNavigationCity() = default;

  virtual void Proc(const std::shared_ptr<RoutingMap> &raw_routing_map, BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut,
                    std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result, std::vector<uint64_t> &sd_guide_lane_ids,
                    AccLaneInfo &sd_acc_lane_info);

  void CalcBevIndex(const std::vector<uint64_t> &road_selected, std::vector<int> &lane_index_left, std::vector<int> &lane_index_right);

  std::string GetInfo() const { return {info_buffer_.data(), info_buffer_.size()}; }

  /*
  * @brief 打印edge信息
*/
  void Edge_log_print(const std::vector<BevLaneMarker> &all_bev_road_edges);
  /*
  * @brief 根据港湾车道信息，去除感知的港湾车道id
  * @param harbor  港湾车道信息
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
  void Erase_Harborlane_Ids(const HarborStopInfo &sd_harbor_stop_info_, std::vector<uint64_t> &road_selected_buslane_filtered,
                            const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list);
  /*
  * @brief 根据线到左边界的距离，过滤最左侧车道
  * @param harbor  港湾车道信息
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
  void Erase_Oppolane_Ids(const HarborStopInfo &sd_harbor_stop_info_, std::vector<uint64_t> &road_selected_buslane_filtered,
                          const std::vector<BevLaneMarker> &all_bev_edges);
  /*
  * @brief 根据港湾车道信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
  double calculateOverlapDistance(const HarborStopInfo &harbor, const std::vector<Eigen::Vector2f> geos);

  /*
  * @brief 根据地图车道列表，获取港湾车道信息
  * @return sd_harbor_stop_info_  港湾车道信息
*/
  void GetBusLaneInfo();
  /*
  * @brief 根据地图车道列表，获取港湾车道信息
  * @return sd_harbor_stop_info_  港湾车道信息
*/
  void GetHarborStopInfo();
  /*
  * @brief 根据地图车道列表，去除感知的公交车道id
  * @param lanetype_list  地图车道列表
  * @param road_selected  感知路段车道id
  * @return road_selected  筛选后的感知路段车道id
*/
  void Erase_Buslane_Ids(std::vector<LaneType> &lanetype_list, std::vector<uint64_t> &road_selected);
  /*
   * @brief 过滤掉公交车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉公交车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterBusLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                      BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 过滤掉公交车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉公交车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterSpecialLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                          BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut);

  /* const std::shared_ptr<RoutingMap> &raw_routing_map, SDLaneNumInfo &lane_num_info
  * @brief 获取当前地图的LaneType列表
  * @param raw_routing_map  SDMap的信息
  * @return 当前地图的LaneType列表
*/
  std::vector<LaneType> GetCurLaneTypeList();
  /*
  * @brief 判断当前是否是双向单车道
  * @param raw_routing_map  SDMap的信息
  * @return 
*/
  bool DetectBidirSingleLane();

  /*
  * @brief 根据range信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
  double calculateOverlapDisinRange(double rangeStart, double rangeEnd, const std::vector<Eigen::Vector2f> geos);
  /*
   * @brief 匹配公交车道类型
   * @param bus_lane_info_   地图公交车道信息
   * @param road_selected_buslane_filtered  车道id
   * @param all_bev_edges  感知边界信息
   * @param GlobalBevMapOutPut  感知信息
   * @return 
  */
  void MatchBusLaneType(const BusLaneInfo &bus_lane_info_, std::vector<uint64_t> &road_selected_buslane_filtered,
                        const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                        BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 更新公交车道类型
   * @param target_buslane_id   公交车道ID
   * @param GlobalBevMapOutPut  感知信息
   * @return 
  */
  void UpdateBusLaneType(uint64_t target_buslane_id, BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 更新港湾车道类型
   * @param road_selected  bev选路后的车道列表
   * @param filtered_lanes_without_harbor  过滤后的车道列表
   * @param GlobalBevMapOutPut  感知信息
   * @return 
  */
  void UpdateHarborLaneType(const std::vector<uint64_t> &road_selected, std::vector<uint64_t> &filtered_lanes_without_harbor,
                            BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 更新对向车道方向
   * @param filtered_lanes_without_harbor  bev选路后的车道列表
   * @param final_lanes  过滤后的车道列表
   * @param GlobalBevMapOutPut  感知信息
   * @return
  */
  void UpdateOppoLaneDir(const std::vector<uint64_t> &filtered_lanes_without_harbor, std::vector<uint64_t> &final_lanes,
                         BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 过滤掉对向车道
   * @param road_selected  bev选路后的车道列表
   * @return 过滤掉对向车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterOppositeLane(const std::shared_ptr<RoutingMap> &raw_routing_map,
                                           std::vector<uint64_t> &filtered_lanes_without_harbor, BevMapInfoConstPtr &raw_bev_map);

  /* harbor 车道能否过滤的校验 */
  bool RemoveHarborlaneCheck(const HarborStopInfo &sd_harbor_stop_info_, std::vector<uint64_t> &road_selected_buslane_filtered);

  /*
   * @brief 过滤掉加速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉加速车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterACCLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                      BevMapInfoConstPtr &raw_bev_map, bool &approaching_merge, JunctionInfoCity &merge_jct,
                                      uint64_t &acc_adj_lane_id);
  //加速车道邻车道
  void SetBevAccAdjLane(BevMapInfo &bev_map, const uint64_t &AccAdjLane_id, JunctionInfoCity &merge_jct);

  /*
   * @brief 判断是否是途径汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
  bool IsApproachMergeJCT(JunctionInfoCity &merge_jct);

  std::vector<Eigen::Vector2f> GetNextRightPoints(const std::shared_ptr<RoutingMap> &routing_map);

  bool IsEgoInDedicatedRightLane() const;

  bool IsPointLeftOfSegment(const Eigen::Vector2f &pt, const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

  bool IsPointLeftOfPolyline(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &polyline);

  double GetSignedDistanceToLine(const Eigen::Vector2f &pt, const std::vector<Eigen::Vector2f> &line, bool need_extent = false);

  Eigen::Vector2f GetTangentDirection(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2);

  Eigen::Vector2f GetNormalDirection(const Eigen::Vector2f &tangent);

  double CalculateCurvedLaneWidth(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2);

  double CalculateCurvedLaneWidth2(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2);

  double CalculateLaneWidth(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2);

  bool IsContainsLastElement(const std::vector<uint64_t> &road_selected_buslane_filtered,
                             const std::vector<uint64_t> &indices_id_to_remove);
  /**
   * @brief 回溯检查当前车道的主前继链中是否在最多5层内存在 split_topo_extend == TOPOLOGY_SPLIT_RIGHT
   * @param start_lane 起始车道指针
   * @return true 如果发现符合条件的前继车道
  */
  bool HasAncestorWithSplitRight(const BevLaneInfo *start_lane) const;

  inline bool              IsCutMapActive() const { return cut_map_zone_.active; }
  inline const CutMapZone &GetCutMapZone() const { return cut_map_zone_; }
  void                     ExtractTargetJunctions(const std::vector<JunctionInfoCity> &junctions_info_city);

 private:
  void Reset();

  void UpdateCutMapFlagByHardScenes();

  void SelectGuideLanes(const std::vector<JunctionInfoCity> &junctions_info_city, const std::vector<uint64_t> &road_selected_in,
                        const std::vector<int> &lane_index_left, const std::vector<int> &lane_index_right,
                        BevMapInfoPtr &GlobalBevMapOutPut, std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                        std::vector<uint64_t> &sd_guide_lane_ids);

  void SelectGuideLanesWithSd(const std::vector<uint64_t>           &bev_ego_root_section_x0,
                              const std::map<double, RecommendLane> &sd_recommend_lane, BevMapInfoPtr &GlobalBevMapOutPut,
                              std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                              std::vector<uint64_t>                                 &sd_guide_lane_ids);

  void JunctionNaviMatch(const JunctionInfoCity &junction_info, const std::vector<uint64_t> &candidate_lanes_in,
                         const std::vector<int> &lane_index_left, const std::vector<int> &lane_index_right,
                         BevMapInfoPtr &GlobalBevMapOutPut, NaviMatchType &match_type, std::vector<uint64_t> &guide_lanes_out);

  /**
   * @brief 执行精匹配，输出推荐的车道 ID 列表
   * @param junctions_info_city SDMap 提取的路口信息
   * @param road_selected 选择的道路 ID 列表
   * @param lane_index_left 车道左侧索引
   * @param lane_index_right 车道右侧索引
   * @return 推荐的车道 ID 列表
   */
  std::vector<uint64_t> PerformFineMatching(const JunctionInfoCity &junction, const std::vector<uint64_t> &road_selected,
                                            const std::vector<int> &lane_index_left, const std::vector<int> &lane_index_right,
                                            BevMapInfoPtr &GlobalBevMapOutPut);

  void JudgeMainLanePos(const JunctionInfoCity &junction, int &size_sum_lane, int &size_target_lane);

  void JunctionsStateProc(BevMapInfoConstPtr &raw_bev_map);

  bool FindCurrentJunction();

  bool JunctionStateCalculate(BevMapInfoConstPtr &raw_bev_map, JunctionInfoCity &junction_current);

  void UpdateJunctionStateQueue();
  void SetJunctionInfo();

  static int IdxInMppSection(uint64_t section_id);

  int IdxInAllMppSection(uint64_t section_id);

  void SetNaviInterfaceAndDebugInfos(BevMapInfoPtr &GlobalBevMapOutPut);

  float CalcLineSmoothness(const BevLaneInfo &line);

  bool IsInterestedJunction(const JunctionInfoCity &junction_info_city);

  bool IsRoadSplitMain2Ramp(const JunctionInfoCity &junction);

  bool IsEffectedJunction(const JunctionInfoCity &junction_info);
  // 检测与右转专用道路口相邻的十字路口或T型路口
  void DetectAdjacentRightTurnJunctions(const std::vector<JunctionInfoCity> &junctions_info_city,
                                        std::vector<AdjacentJunctionInfo>   &adjacent_junctions_info);

  void AssignMergeAttributesToBevLanes(BevMapInfo &bev_map);

  //void SetEgoNormalLane(uint64_t  &ego_normal_lane_id);
  void SetACCLaneInfo(AccLaneInfo &sd_acc_lane_info);

  void SetReversibleLaneType(const std::shared_ptr<RoutingMap> &raw_routing_map, 
                             BevMapInfoConstPtr &raw_bev_map, 
                             BevMapInfoPtr &GlobalBevMapOutPut);

  std::vector<BevLaneInfo *>             bev_candidate_lanes_        = {};
  std::vector<BevLaneMarker *>           bev_candidate_road_edges_   = {};
  std::vector<LineSortCity>              bev_line_sorts_             = {};
  std::vector<int>                       bev_left_road_edge_indexs_  = {};
  std::vector<int>                       bev_right_road_edge_indexs_ = {};
  std::map<uint64_t, int>                last_intersect_bev_laneids_ = {};
  std::vector<std::vector<uint64_t>>     bev_sections_               = {};
  BevRouteInfo                           complete_section_info_;
  std::pair<int, int>                    bev_egoRoadEdgeIndexPair_ = {-1, -1};
  std::set<uint64_t>                     bev_ego_lane_related_     = {};
  std::vector<uint64_t>                  history_guide_laneids_    = {};
  std::vector<int>                       lane_index_left_;
  std::vector<int>                       lane_index_right_;
  std::set<uint64_t>                     road_selected_repeat_;
  std::vector<uint64_t>                  road_selected_candidate_;
  std::map<uint64_t, std::set<uint64_t>> road_selected_map_;              // 候选lane-对应的重复lane
  std::vector<uint64_t>                  road_selected_without_buslane_;  // 过滤公交车道的道路
  std::vector<uint64_t>                  road_selected_without_merge_;    // 过滤merge车道的道路
  CoarseMatching                         coarse_matching_;                // 粗匹配类实例
  std::vector<uint64_t>                  guide_lanes_;                    // 输出推荐车道
  BevDataProcessor                       bev_data_processor_;
  RoadSelector                           road_selector_;
  std::vector<uint64_t>                  bev_ego_root_section_x0_;
  Oppo_lanes_to_move                     pre_oppo_lanes_to_move_;  //上一帧对向车道信息

  SDMapElementExtract             sd_map_element_extract_;
  std::vector<JunctionInfoCity>   junctions_info_city_;     // 提取地图元素
  std::vector<uint64_t>           road_selected_;           // 选择道路
  std::vector<uint64_t>           hold_road_selected_;      // 选择保持道路
  std::deque<JunctionInfoCity>    junction_history_state_;  // 历史状态
  JunctionInfoCity                junction_current_;
  JunctionInfoCity                junction_previous_;
  std::vector<uint64_t>           route_section_ids_;
  MainLanePos                     main_lane_pos_ = MainLanePos::Unknow;
  std::vector<size_t>             junctions_around_;
  fmt::memory_buffer              info_buffer_;
  SDLaneNumInfo                   lane_num_info_;        //地图车道数信息
  HarborStopInfo                  sd_harbor_stop_info_;  // 港湾停靠站信息
  BusLaneInfo                     bus_lane_info_;        // 公交车道信息
  SDMapTopologyExtractor          topology_extractor_;
  bool                            isSplitRoadSelectWorked_;  //split路口是否已经选路
  std::map<double, RecommendLane> sd_recommend_lane_;        // 地图推荐车道信息
  CutMapZone                      cut_map_zone_;
  AccLaneInfo                     sd_acc_lane_info_;
  std::vector<JunctionInfoCity>   target_junction_vct_;

  std::map<JunctionTypeCity, double> junction_type_dis_in_{{JunctionTypeCity::CrossRoad, 3.0},       {JunctionTypeCity::TJunction, 3.0},
                                                           {JunctionTypeCity::RoadSplit, 3.0},       {JunctionTypeCity::RoadMerge, 3.0},
                                                           {JunctionTypeCity::SmallTJunction, 3.0},  {JunctionTypeCity::Unknown, 3.0},
                                                           {JunctionTypeCity::InvalidJunction, 3.0}, {JunctionTypeCity::UTurn, 3.0}};
  std::map<JunctionTypeCity, double> junction_type_dis_out_{{JunctionTypeCity::CrossRoad, 30.0},       {JunctionTypeCity::TJunction, 30.0},
                                                            {JunctionTypeCity::RoadSplit, 12.0},       {JunctionTypeCity::RoadMerge, 20.0},
                                                            {JunctionTypeCity::SmallTJunction, 10.0},  {JunctionTypeCity::Unknown, 20.0},
                                                            {JunctionTypeCity::InvalidJunction, 10.0}, {JunctionTypeCity::UTurn, 10.0}};

  std::map<JunctionAction, cem::message::sensor::BevAction> JunctionAction2Bev_ = {
      {JunctionAction::Unknown, cem::message::sensor::BevAction::UNKNOWN},
      {JunctionAction::TurnLeft, cem::message::sensor::BevAction::LEFT_TURN},
      {JunctionAction::TurnRight, cem::message::sensor::BevAction::RIGHT_TURN},
      {JunctionAction::GoStraight, cem::message::sensor::BevAction::STRAIGHT},
      {JunctionAction::UTurn, cem::message::sensor::BevAction::U_TURN}};
};

}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  // SDNAVIGATIONCITY_H
