#ifndef SDNAVIGATION_H
#define SDNAVIGATION_H

#include <iterator>
#include <map>
#include <base/params_manager/params_defination/internal_params.h>
#include <base/params_manager/params_manager.h>
#include "common/utils/GeoMathUtil.h"
#include "common/utils/lane_geometry.h"
#include "fmt/format.h"
#include "lib/common/log_custom.h"
#include "lib/sd_navigation/BevDataProcessor.h"
#include "lib/sd_navigation/routing_map_debug.h"
#include "sd_navigation/SdNavigationBase.h"
#include "modules/perception/env/src/lib/sd_navigation/SDMapElementExtract.h"
#include "lib/sd_navigation/SDMapTopoExtract.h"

namespace cem {
namespace fusion {
namespace navigation {
#if defined(SD_HIGHJUNCTION_CONVERTE) && (SD_HIGHJUNCTION_CONVERTE)
#define SD_HIGHJUNCTION_CONVERTE_LOG AINFO
#else
#define SD_HIGHJUNCTION_CONVERTE_LOG NULL_LOG  //NULL_LOG
#endif

class SdNavigationHighway : public SdNavigationBase {
 public:
  SdNavigationHighway();

  virtual void Proc(const std::shared_ptr<RoutingMap> &raw_routing_map, BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut,
                    std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result, std::vector<uint64_t> &sd_guide_lane_ids,
                    AccLaneInfo &sd_acc_lane_info);

  [[nodiscard]] const std::map<double, RecommendLane> &GetLdRecommendLane() const { return map_ld_recommend_lane_; }
  // inline bool IsCutMapActive() const { return true; }
  void               SetLDJunctionInfo();
  fmt::memory_buffer ld_info_buffer_;
  std::string        GethighInfo() const { return {ld_info_buffer_.data(), ld_info_buffer_.size()}; }
  inline std::string StrNaviSplitDirectionHighway(const SplitDirection &splitdirection) {
    switch (splitdirection) {
      case SplitDirection::Straight:
        return "Straight";
      case SplitDirection::SplitLeft:
        return "SplitLeft";
      case SplitDirection::SplitRight:
        return "SplitRight";
      default:
        return fmt::format("UNDEFINED({})", splitdirection);
    }
  }
  inline std::string StrNaviJunctionHighway(const int &match_type) {
    switch (match_type) {
      case 0:
        return "Unknow";
      case 1:
        return "RampInto";
      case 2:
        return "ApproachRampInto";
      case 3:
        return "RampMerge";
      case 4:
        return "ApproachRampMerge";
      case 5:
        return "RampSplitLeft";
      case 6:
        return "RampSplitRight";
      case 7:
        return "RampSplitMiddle";
      case 8:
        return "MainRoadSplitLeft";
      case 9:
        return "MainRoadSplitRight";
      case 10:
        return "MainRoadSplitMiddle";
      case 11:
        return "ApproachRampMergeLeft";
      default:
        return fmt::format("UNDEFINED({})", match_type);
    }
  }

 private:
  ///void GetSDJunctionInfos();
  void GetLDJunctionInfos();
  //bool IsRampSection(const SDSectionInfo &section_info);
  bool LD_IsRampSection(const cem::message::env_model::SectionInfo &section_info);

  //bool IsMainRoadSection(const SDSectionInfo &section_info);
  bool LD_IsMainRoadSection(const cem::message::env_model::SectionInfo &section_info);
  ///是否是城区道路
  //bool IsCityRoadSection(const SDSectionInfo &section_info);
  ///USE LD 是否是城区道路
  bool LD_IsCityRoadSection(const cem::message::env_model::SectionInfo &section_info);

  void judge_left_or_rightsection(std::vector<uint64_t> &sort_section_id, const cem::message::env_model::SectionInfo &SectionInfo);

  ///导航事件状态机管理
  void ManageJunctionEvent(int &current_target_junction_index);

  ///bev数据处理
  void BevMapProcess();

  ///获取bevlane的候选
  void GetCandidateBevLanes(const std::vector<BevLaneInfo> &lane_infos_in, std::vector<BevLaneInfo *> &candidate_bev_lanes_out);

  ///将bevlane和bev roadedge按左右相对位置排序
  void SortBevLaneInfoAndRoadEdges(const std::vector<BevLaneInfo *> &bev_lane_infos_in, const std::vector<BevLaneMarker> &road_edges_in,
                                   std::vector<LineSort> &line_sorts, std::vector<int> &left_road_edge_indexs,
                                   std::vector<int> &right_road_edge_indexs);

  ///使用道路边界过滤bevlanes
  void BevLanesFilterByRoadEdges(const std::vector<LineSort> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                 const std::vector<int> &right_road_edge_indexs, std::vector<BevLaneInfo *> &candidateBevLanes);

  ///获取EgoRoadEdges的index
  std::pair<int, int> GetEgoLeftRightRoadEdges(const std::vector<LineSort> &line_sorts, const std::vector<int> &left_road_edge_indexs,
                                               const std::vector<int> &right_road_edge_indexs);

  ///使用roadEdges划分道路section
  void DivideBevSectionsByRoadEdges(const std::vector<BevLaneInfo *> &bev_candidate_lanes_in, std::vector<std::vector<uint64_t>> &sections);

  ///bev blocked车道的过滤
  void FilterBevBlockedLanes(std::vector<BevLaneInfo *> &bev_candidate_lanes_in, std::vector<std::vector<uint64_t>> &sections);

  void AssignBevLaneTypes();

  ///【应急车道】获取SDMap中应急车道信息
  EmergencyLaneInfo GetEmergencyLaneInfo();

  // 获取SDMap中港湾停靠站信息
  void GetHarborStopInfo();

  //获取加速车道信息
  void GetAccLaneInfo(bool approaching_merge);

  //获取减速车道信息
  void GetDecLaneInfo();

  uint64_t FindOriginNormalLaneIdFromAccLane(const cem::message::env_model::LaneInfo *acc_lane, uint64_t ego_section_id);

  uint64_t FindOriginNormalLaneIdFromAccLane2(const cem::message::env_model::LaneInfo *acc_lane, uint64_t ego_section_id,
                                              double max_search_distance /* = 800.0 */);

  ///【应急车道】融合SDMap和感知 获取应急车道id
  void FusionCurrentBevEmergencyLaneid(const std::vector<uint64_t> &target_section, const EmergencyLaneInfo &sd_emergency_lane_info,
                                       const int &selected_section_index, uint64_t &current_left_bev_emergency_laneid,
                                       uint64_t &current_right_bev_emergency_laneid);

  ///选择导航车道
  void SelectBevNavigationLanes(const std::shared_ptr<RoutingMap> &raw_routing_map, int current_target_junction_index,
                                const std::vector<std::vector<uint64_t>> &bev_sections_in, uint64_t &current_right_bev_emergency_laneid,
                                uint64_t                                              &current_left_bev_emergency_laneid,
                                std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                std::vector<uint64_t> &guide_laneids_ref, BevMapInfoPtr &GlobalBevMapOutPut);

  ///主路无导航选路
  int SelectMainRoadSectionNoNavi(const std::vector<std::vector<uint64_t>> &bev_sections_in);

  ///获取sd section中lanegroup的最小的车道数
  //int GetSDSectionMinLaneNumNOTEmergency(const SDSectionInfo &section_info);
  int GetLDSectionMinLaneNumNOTEmergency(const cem::message::env_model::SectionInfo &section_info);

  void SetBevEmergencyLane(BevMapInfo &bev_map, const std::pair<uint64_t, uint64_t> &fusion_emergency_lane_id);

  void HoldGuideLanes(
      const std::vector<std::vector<uint64_t>> &bev_sections_in,
      std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
      std::vector<uint64_t> &guide_laneids_ref,
      BevMapInfoPtr &GlobalBevMapOutPut);
  bool IsLeftGuidePos(std::vector<uint64_t> &bev_sections_out, uint64_t curr_id, uint64_t hist_id);

  //加速车道
  void SetBevAccLane(BevMapInfo &bev_map, const uint64_t &AccLane_id);
  //加速车道邻车道
  void SetBevAccAdjLane(BevMapInfo &bev_map, const uint64_t &AccAdjLane_id, JunctionInfo &merge_jct);

  void SetNaviInterfaceAndDebugInfos(BevMapInfoPtr &GlobalBevMapOutPut);

  bool IsInterestedJunction(const JunctionInfo &junction_info);

  bool IsV2IgnoredJunction(const JunctionInfo &junction_info);

  void JunctionNaviMatch(const JunctionInfo *junction_info_ptr, const std::vector<uint64_t> &candidate_lanes_in, int &match_type,
                         std::vector<uint64_t> &guide_laneids);

  bool IsValidRoadEdge(const BevLaneMarker &edge);

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
                            const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                            uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid);
  /*
  * @brief 根据港湾车道信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
  double calculateOverlapDistance(const HarborStopInfo &harbor, const std::vector<Eigen::Vector2f> geos);
  /*
  * @brief 根据range信息和车道线，判断是否存在重叠
  * @return 重叠距离
*/
  double calculateOverlapDisinRange(double rangeStart, double rangeEnd, const std::vector<Eigen::Vector2f> geos);

  // 检测车道集合中首尾车道的分叉情况
  void DetectSplitLanes(const std::vector<uint64_t> &rsplit_lane_filtered, const std::vector<BevLaneMarker> &all_bev_edges,
                        std::vector<uint64_t> &split_indices_id_to_remove, bool has_junction_app_ramp, bool has_junction_app_merge,
                        uint64_t acc_lane_id);

  // 检查相邻车道是否共享前继车道
  bool CheckLanesShareAncestor(uint64_t lane1, uint64_t lane2);

  /*
   * @brief 收集前继车道
   *@param start_id 
   * @return 
   */
  std::unordered_set<uint64_t> CollectPreLanes(uint64_t start_id) const;

  std::string PrintProcessedLanes(const std::unordered_set<uint64_t> &lanes);
  /*
   * @brief 过滤掉分叉车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉分叉车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterSplitLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                        BevMapInfoConstPtr &raw_bev_map, uint64_t acc_lane_id, BevMapInfoPtr &GlobalBevMapOutPut);
  /*
   * @brief 过滤掉加速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉加速车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterACCLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                      BevMapInfoConstPtr &raw_bev_map, bool &approaching_merge, bool &ego_merge, uint64_t &acc_lane_id,
                                      uint64_t &acc_adj_lane_id);

  /*
   * @brief 过滤掉减速车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉减速车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterDecLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                      BevMapInfoConstPtr &raw_bev_map, BevMapInfoPtr &GlobalBevMapOutPut);

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
                                      BevMapInfoConstPtr &raw_bev_map, uint64_t &right_bev_emergency_laneid,
                                      uint64_t &left_bev_emergency_laneid);
  /*
   * @brief 过滤掉特殊车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev车道
   * @return 过滤掉公交车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterSpecialLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                          BevMapInfoConstPtr &raw_bev_map, std::vector<JunctionInfo *> &target_sd_junctions,
                                          uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid,
                                          EmergencyLaneInfo &emergency_lane_info, BevMapInfoPtr &GlobalBevMapOutPut);

  /*
   * @brief 过滤掉应急车道
   * @param raw_routing_map  SDMap的信息
   * @param road_selected  bev选路后的车道列表
   * @param raw_bev_map  bev
   * @return 过滤掉应急车道的车道 ID 列表
  */
  std::vector<uint64_t> FilterEmergencyLane(const std::shared_ptr<RoutingMap> &raw_routing_map, const std::vector<uint64_t> &road_selected,
                                            BevMapInfoConstPtr &raw_bev_map);
  /*
  * @brief 获取当前地图的LaneType列表
  * @param raw_routing_map  SDMap的信息
  * @return 当前地图的LaneType列表
*/
  std::vector<LaneType> GetCurLaneTypeList(const std::shared_ptr<RoutingMap> &raw_routing_map, SDLaneNumInfo &lane_num_info);
  /*
  * @brief 当前是否在隧道内
  * @return 
*/
  bool EgoIsInTunnel();

  /*
   * @brief 判断是否是途径汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
  bool IsApproachMergeJCT(std::vector<JunctionInfo *> &target_sd_junctions, JunctionInfo &merge_jct);
  /*
   * @brief 判断是否是自车汇入
   * @param target_sd_junctions  路口的信息
   * @return bool
  */
  bool IsRampMergeJCT(std::vector<JunctionInfo *> &target_sd_junctions, JunctionInfo &merge_jct);

  bool                  EgoInMainRoad();
  void                  SetACCLaneInfo(AccLaneInfo &sd_acc_lane_info);
  void                  SetLdRecommendLane();
  void                  SelectGuideLanesWithLd(const std::vector<uint64_t>           &bev_ego_root_section_x0,
                                               const std::unordered_map<double, RecommendLane> &ld_recommend_lane, BevMapInfoPtr &GlobalBevMapOutPut,
                                               std::vector<std::pair<uint64_t, std::pair<int, int>>> &guide_lane_result,
                                               std::vector<uint64_t>                                 &sd_guide_lane_ids);
  std::vector<uint64_t> MergeSectionLaneIdsLeft2Right(const std::vector<std::vector<uint64_t>> &complete_section_info) const;

  std::vector<uint64_t> FindRootsAtX0AndSort(const BevMapInfo &bev_map, const std::vector<uint64_t> &merged_lane_ids) const;
  static double         InterpolateYAtX(const std::vector<Eigen::Vector2f> &geos, double x) noexcept;
  /*
   * @brief 临停车道赋值
   * @return
   */
  void UpdateHarborLaneType(const std::vector<uint64_t> &road_selected, std::vector<uint64_t> &filtered_lanes_without_harbor,
                            BevMapInfoPtr &GlobalBevMapOutPut);

  bool HasCommonAncestor(const LineSort &l1, const LineSort &l2);

  /**
   * @brief 回溯检查当前车道的主前继链中是否在最多5层内存在 split_topo_extend == TOPOLOGY_SPLIT_RIGHT
   * @param start_lane 起始车道指针
   * @return true 如果发现符合条件的前继车道
  */
  bool HasAncestorWithSplitRight(const BevLaneInfo *start_lane) const;

  double CalculateCurvedLaneWidth2(const std::vector<Eigen::Vector2f> &lane1, const std::vector<Eigen::Vector2f> &lane2);

  /* harbor 车道能否过滤的校验 */
  bool RemoveHarborlaneCheck(const HarborStopInfo &sd_harbor_stop_info_, std::vector<uint64_t> &road_selected_buslane_filtered);

  bool OnlyHarborSceneCheck(const HarborStopInfo &sd_harbor_stop_info_, const EmergencyLaneInfo &g_emergency_lane_info_);

  bool LeftConstructionZoneCheck(const std::vector<uint64_t>            &road_selected_buslane_filtered,
                                                    const std::vector<BevLaneMarker> &all_bev_edges, std::vector<LaneType> &lanetype_list,
                                                    uint64_t &right_bev_emergency_laneid, uint64_t &left_bev_emergency_laneid);

  double CalculateSectionCost(int section_index, const std::vector<std::vector<uint64_t>> &bev_sections_in,
                              const std::vector<int> &ego_section_indexs, const std::vector<JunctionInfo *> &target_sd_junctions);

 private:
  /// 自车前方2km的地图导航信息
  std::vector<JunctionInfo> sd_junction_infos_;
  uint64_t                  ego_section_id_     = 0;
  uint64_t                  ego_normal_lane_id_ = 0;

  // 港湾停靠站信息
  HarborStopInfo sd_harbor_stop_info_;
  AccLaneInfo    sd_acc_lane_info_;
  DecLaneInfo    sd_dec_lane_info_;
  EmergencyLaneInfo g_emergency_lane_info_;

  uint64_t           JunctionEvent_last_passed_junction_id_ = 0;
  JunctionBevFeature JunctionEvent_last_junction_bev_feature_;

  std::vector<uint64_t>             history_guide_laneids_      = {};
  std::unordered_map<uint64_t, int> last_intersect_bev_laneids_ = {};

  std::unordered_map<JunctionType, int> JunctionEvent_min_passed_dist_map_ = {
      {JunctionType::RampInto, -200},          {JunctionType::ApproachRampInto, -160}, {JunctionType::RampMerge, -300},
      {JunctionType::ApproachRampMerge, -250}, {JunctionType::RampSplitLeft, -50},     {JunctionType::RampSplitRight, -50},
      {JunctionType::RampSplitMiddle, -50},
      // {JunctionType::MainRoadSplitLeft, -100},
      // {JunctionType::MainRoadSplitRight, -100},
      // {JunctionType::MainRoadSplitMiddle, -100},
      // {JunctionType::ApproachRampMergeLeft, -250}
  };
  std::unordered_map<JunctionType, int> JunctionEvent_max_start_effect_dist_map_ = {
      {JunctionType::RampInto, 2000},         {JunctionType::ApproachRampInto, 500}, {JunctionType::RampMerge, 200},
      {JunctionType::ApproachRampMerge, 200}, {JunctionType::RampSplitLeft, 500},    {JunctionType::RampSplitRight, 500},
      {JunctionType::RampSplitMiddle, 500}};

  std::unordered_map<JunctionType, int> JunctionEvent_max_start_effect_dist_map_less_lane_num_ = {{JunctionType::RampInto, 1000},
                                                                                        {JunctionType::RampSplitLeft, 300},
                                                                                        {JunctionType::RampSplitRight, 300},
                                                                                        {JunctionType::RampSplitMiddle, 300}};

  std::unordered_map<JunctionType, int> JunctionEvent_max_start_effect_dist_map_more_lane_num_ = {
      {JunctionType::RampSplitLeft, 1000},
      {JunctionType::RampSplitRight, 1000},
  };

  inline cem::message::sensor::BevLaneType mapLaneType(cem::message::env_model::LaneType lane_type) const {
    switch (lane_type) {
      case cem::message::env_model::LaneType::LANE_UNKNOWN:
        return cem::message::sensor::BevLaneType::LANE_TYPE_UNKNOWN;
      case cem::message::env_model::LaneType::LANE_NORMAL:
      case cem::message::env_model::LaneType::LANE_ACC:
      case cem::message::env_model::LaneType::LANE_DEC:
      case cem::message::env_model::LaneType::LANE_ACC_DCC:
        return cem::message::sensor::BevLaneType::LANE_TYPE_MAIN;
      case cem::message::env_model::LaneType::LANE_RAMP:
        return cem::message::sensor::BevLaneType::LANE_TYPE_SIDE;
      case cem::message::env_model::LaneType::LANE_EMERGENCY:
        return cem::message::sensor::BevLaneType::LANE_TYPE_EMERGENCY;
      case cem::message::env_model::LaneType::LANE_BUS_NORMAL:
      case cem::message::env_model::LaneType::LANE_HOV_NORMAL:
      case cem::message::env_model::LaneType::LANE_NON_MOTOR:
      case cem::message::env_model::LaneType::LANE_LEFT_WAIT:
      case cem::message::env_model::LaneType::LANE_VIRTUAL_COMMON:
      case cem::message::env_model::LaneType::LANE_VIRTUAL_JUNCTION:
      case cem::message::env_model::LaneType::LANE_ROUND_ABOUT:
      case cem::message::env_model::LaneType::LANE_REVERSIBLE:
      case cem::message::env_model::LaneType::LANE_VARIABLE_TURN:
      case cem::message::env_model::LaneType::LANE_HARBOR_STOP:
        return cem::message::sensor::BevLaneType::LANE_TYPE_OTHER;
      default:
        return cem::message::sensor::BevLaneType::LANE_TYPE_OTHER;
    }
  }

  std::vector<BevLaneInfo *>         bev_candidate_lanes_        = {};
  std::vector<LineSort>              bev_line_sorts_             = {};
  std::vector<int>                   bev_left_road_edge_indexs_  = {};
  std::vector<int>                   bev_right_road_edge_indexs_ = {};
  std::pair<int, int>                bev_egoRoadEdgeIndexPair_   = {-1, -1};
  std::vector<std::vector<uint64_t>> bev_sections_               = {};
  std::set<uint64_t>                 bev_ego_lane_related_       = {};
  BevDataProcessor                   bev_data_processor_;
  Split_lanes_to_move                pre_split_lanes_to_move_;//上一帧split车道信息
  ConstructionZoneInfo               construction_zone_info_;  //施工区域判断的保持

  ////junction 选路生效距离
  std::unordered_map<JunctionType, int> Junction_select_road_dist_map_ = {
      {JunctionType::RampInto, 500},        {JunctionType::RampSplitLeft, 200},    {JunctionType::RampSplitRight, 200},
      {JunctionType::RampSplitMiddle, 200}, {JunctionType::ApproachRampInto, 200}, {JunctionType::ApproachRampMerge, 60},
      {JunctionType::MainRoadSplitLeft, 200}, {JunctionType::MainRoadSplitRight, 200}, {JunctionType::MainRoadSplitMiddle, 200},
  };

  // 应急车道扫描范围
  double rear_distance_  = -100.0;  // 自车后 100m
  double front_distance_ = 300.0;   // 自车前 300m
  // 应急车道扫描范围
  double dec_rear_distance_  = -100.0;  // 自车后 100m
  double dec_front_distance_ = 900.0;   // 自车前 900m
  // 加速车道扫描范围
  double acc_rear_distance_  = -100.0;  // 自车后 100m
  double acc_front_distance_ = 800.0;   // 自车前 800m

  SDMapTopologyExtractor          topology_extractor_;
  double                          merge_rear_distance_  = -100.0;  // 自车后 100m
  double                          merge_front_distance_ = 500.0;   // 自车前 500m
  std::map<double, RecommendLane> map_ld_recommend_lane_;
  std::vector<uint64_t>           bev_ego_root_section_x0_;
  uint64_t                        hist_guide_id_     = 0;
  uint64_t hist_ego_id_ = 0;
  bool active_last_ego_guide_ = false;
  bool active_hist_guide_ = false;
  std::vector<Eigen::Vector2f> ld_merge_points;//only use in merge;
  std::unordered_map<uint64_t, int> navi_lane_num = {};
};
}  // namespace navigation
}  // namespace fusion
}  // namespace cem

#endif  //SDNAVIGATION_H
