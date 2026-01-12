#include "lib/perception_and_ld_map_fusion/fusion_manager.h"
#include "lib/common/function_timer.h"

#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
bool print_debug_info = false;
#endif

namespace cem {
namespace fusion {

static std::unordered_set<uint64_t> GetAllLastLaneByTopology(
    BevLaneInfo* lane,
    std::unique_ptr<cem::fusion::BevMapProcessor>& bev_map_preprocessor,
    std::unique_ptr<cem::fusion::BevFilletingMachine>& bev_slicer) {
  std::unordered_set<uint64_t> next_lanes;
  std::vector<uint64_t> in_range_next_lane_ids;
  for (auto id : lane->next_lane_ids) {
    if (bev_slicer->IsLaneInReliableRange(id)) {
      in_range_next_lane_ids.push_back(id);
    }
  }
  if (in_range_next_lane_ids.empty()) {
    next_lanes.emplace(lane->id);
    return next_lanes;
  } else {
    for (uint64_t id : lane->next_lane_ids) {
      BevLaneInfo* bev_lane = bev_map_preprocessor->GetLane(id);
      if (bev_lane == nullptr) {
        continue;
      }
      std::unordered_set<uint64_t> tmp =
          GetAllLastLaneByTopology(bev_lane, bev_map_preprocessor, bev_slicer);
      next_lanes.insert(tmp.begin(), tmp.end());
    }
    return next_lanes;
  }
};

void FusionManager::Init() {
  bev_map_preprocessor_ = std::make_unique<BevMapProcessor>();
  bev_lane_type_checker_ = std::make_unique<LaneTypeChecker>();
  ld_map_preprocessor_ = std::make_unique<LdMapProcessor>();
  bev_slicer_ = std::make_unique<BevFilletingMachine>();
  ldmap_slicer_ = std::make_unique<LdMapFilletingMachine>();
  match_maker_ = std::make_unique<MatchMaker>();
  // visualizer_ = std::make_unique<Visualizer>();
  data_fusion_controller_ = std::make_unique<MultiSourceDataMixer>();
  data_fusion_controller_->Init(bev_map_preprocessor_.get(),
                                ld_map_preprocessor_.get(), bev_slicer_.get());
  map_fusion_ = std::make_unique<MapFusion>();
}

void FusionManager::Process() {
  // AINFO << "-----------------fusion start-----------------------";
  // auto ts_start = GetMwTimeNowSec();
  debug_info_.clear();
  bev_map_ = nullptr;
  ld_map_ = nullptr;
  ego_lane_lat_error_between_ld_and_bev_ = std::numeric_limits<double>::max();
  estimated_bev_ego_lane_id_ = 0;
  ego_lane_compare_length_ = 50.0;
  bool has_bev_map = bev_map_preprocessor_->FeedLatestFrame();
  bool has_ld_map = ld_map_preprocessor_->FeedLatestFrame();
  if (!has_bev_map) {
    return;
  }
  bev_map_ = bev_map_preprocessor_->GetData();
  ld_map_ = ld_map_preprocessor_->GetData();
  if (has_bev_map && has_ld_map) {
    DataFusion();    
  } else {
    // 没有地图的话，bev也不做时间同步了
    ld_map_preprocessor_->ClearCache();
    // 同时清理合汇流信息和匹配信息
    match_maker_->ClearMatchers();
    bev_split_info_.clear();
    bev_merge_info_.clear();
  }

  auto& match_details = match_maker_->GetMatchDetails();
  auto LaneSplitGetter = [this](uint64_t bev_id) {
        return this->GetBevLaneSplitInfo(bev_id);
  };
  auto LaneMergeGetter = [this](uint64_t bev_id, bool type) {
        return this->GetBevLaneMergeInfo(bev_id, type);
  };

  auto LaneTypeGetter = [this](uint64_t bev_id) {
        return this->GetLaneTypeByBevId(bev_id);
  };

  // AINFO << std::setprecision(16) << "time_now:" << time_now << ", time get: "
  //       << bev_map_preprocessor_->GetDataPose()->header.timestamp;
  std::map<uint64_t, std::vector<uint64_t>> bev_ld_match;
  for (auto& matcher : match_maker_->GetMatchers()) {
    std::vector<uint64_t> map_ids;
    for (auto ldmap_lane_id : matcher.map_id) {
      map_ids.emplace_back(ldmap_lane_id);
    }
    // 检查bev_id是否已存在
    auto it = bev_ld_match.find(matcher.bev_id);
    if (it != bev_ld_match.end()) {
        // 存在：将当前map_ids追加到已有vector末尾
        it->second.insert(it->second.end(), map_ids.begin(), map_ids.end());
    } else {
        // 不存在：插入新的键值对（原逻辑）
        bev_ld_match.insert(std::make_pair(matcher.bev_id, map_ids));
    }
  }
  INTERNAL_PARAMS.ld_match_info_data.ClearContainer();
  INTERNAL_PARAMS.ld_match_info_data.SetMatchInfo(bev_ld_match);
  std::shared_ptr<BuildLaneTopology> build_lane_topology;
  build_lane_topology.reset(new BuildLaneTopology());
  bool low_precision_flag = build_lane_topology->isLDMapLowPrecision();
  auto& split_match_infos = ld_map_preprocessor_->GetSplitInfo();
  auto& merge_match_infos = ld_map_preprocessor_->GetMergeInfo();
  auto& connect_match_infos = ld_map_preprocessor_->GetConnectInfo();
  bev_map_preprocessor_->TopoProcessor(
      LaneSplitGetter, LaneMergeGetter, LaneTypeGetter,
      ld_map_preprocessor_->GetData(), bev_ld_match, split_match_infos,
      merge_match_infos, connect_match_infos, low_precision_flag, debug_info_);
  // 各种拓扑构建入口
  bev_map_ids_info bev_map_ids;
  for (auto& matcher : match_maker_->GetMatchers()) {
    std::vector<uint64_t> map_ids;
    for (auto ldmap_lane_id : matcher.map_id) {
      map_ids.emplace_back(ldmap_lane_id);
    }
    bev_map_ids.local_map_matched_ids.insert(
        std::make_pair(matcher.bev_id, map_ids));
  }
  bev_map_ids.lane_id_to_group_id_ = ld_map_preprocessor_->GetLaneGroup();
  for (size_t idx = 0;
       idx < bev_map_preprocessor_->GetData()->lane_infos.size();
       idx++) {  // 根据感知ego id获取对应匹配地图的ego地图数据。
    auto& bev_lane = bev_map_preprocessor_->GetData()->lane_infos[idx];
    if (bev_lane.position ==
        static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
      auto map_ego_lane = ld_map_preprocessor_->GetLane(bev_lane.id);
      if (map_ego_lane != nullptr) {
        bev_map_ids.bev_eog_id_map_lane_info = map_ego_lane;
      }
      std::vector<const BevSplitInfo*> split_map_info =
          GetBevLaneSplitInfo(bev_lane.id);
      std::vector<const BevMergeInfo*> merge_map_info =
          GetBevLaneMergeInfo(bev_lane.id);
      for (auto& elem : split_map_info) {
        bev_map_ids.ldmap_split_info.emplace_back(*elem);
      }
      for (auto& elem : merge_map_info) {
        bev_map_ids.ldmap_merge_info.emplace_back(*elem);
      }
    }
  }
  INTERNAL_PARAMS.geos_match_merge_data.ClearContainer();
  for(auto& bev_lane : bev_map_preprocessor_->GetData()->lane_infos) {
    auto merge_info = GetFarMergeInfo(bev_lane.id, 0, 500);
    if(!merge_info.empty() && (merge_info[0]->type == 1 || merge_info[0]->type == 2)) {
        MergeTopoInfo merge_topo;
        merge_topo.type = merge_info[0]->type;
        merge_topo.distance_to_ego = merge_info[0]->distance_to_ego;
        INTERNAL_PARAMS.geos_match_merge_data.SetMatchMergeInfo(bev_lane.id, merge_topo);
    }
  }
  if (nullptr != bev_map_preprocessor_) {
    if (nullptr != bev_map_) {  // 保持自车道在lane track丢失.e.g. ORIN-1370824
      std::optional<Eigen::Isometry3d> T_local_ego_ =
          FindTransform(bev_map_->header.timestamp);
      if (nullptr != bev_map_preprocessor_->lane_tracker_ptr_ && T_local_ego_) {
        bev_map_preprocessor_->lane_tracker_ptr_->Update(
            bev_map_->lane_infos, bev_map_->lanemarkers, T_local_ego_.value(),
            false);
        bev_map_->lane_infos =
            bev_map_preprocessor_->lane_tracker_ptr_->getResult(
                bev_map_->lanemarkers);
      }
    }
    if (nullptr != build_lane_topology) {
      build_lane_topology->BuildNewMergeSplitTopo(bev_map_, ld_map_, bev_map_ids);
    }
  }

  // 车道类型更新
  bev_lane_type_checker_->UpdateBevTopologyLaneType(bev_map_, ld_map_, LaneSplitGetter, LaneMergeGetter);

  // RecommendDrivingLanes();

  // AINFO << "-----------------fusion end-----------------------";
}

void FusionManager::DataFusion() {
  double time_now = GetMwTimeNowSec();
  if (!bev_map_preprocessor_->Process(time_now) ||
      !ld_map_preprocessor_->Process(time_now)) {
    return;
  }
  auto bev_map = bev_map_preprocessor_->GetData();
  auto ld_map = ld_map_preprocessor_->GetData();

#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
  AINFO << "bev_map_sequence_id:" << bev_map->header.cycle_counter;
  AINFO << "routing_map_sequence_id:" << ld_map->header.cycle_counter;
#endif

  auto main_road_direction = ld_map_preprocessor_->GetMainDirection();
  Eigen::Vector2d slice_knife_direction(-main_road_direction.y(),
                                        main_road_direction.x());
  Eigen::Vector2d start_point;
  double length;
  bev_map_preprocessor_->EsitmateSliceParameters(main_road_direction,
                                                 start_point, length);
  // AINFO << "slice_knife_direction:" << slice_knife_direction;
  // AINFO << "start point:" << start_point;
  // AINFO << "length:" << length;
  bev_slicer_->SetSliceKnifeDirection(slice_knife_direction);
  bev_slicer_->SetSliceStartPoint(start_point);
  bev_slicer_->SetSliceTotalLength(length);
  bev_slicer_->SetData(bev_map);
  bev_slicer_->Process();

  ldmap_slicer_->SetSliceKnifeDirection(slice_knife_direction);
  ldmap_slicer_->SetSliceStartPoint(start_point);
  ldmap_slicer_->SetSliceTotalLength(length);
  ldmap_slicer_->SetData(ld_map);
  ldmap_slicer_->Process();

  auto& bev_slices = bev_slicer_->GetSlices();
  auto& ldmap_slices = ldmap_slicer_->GetSlices();

  const auto& ld_map_lane_group = ld_map_preprocessor_->GetLaneGroup();
  const auto& bev_map_sampling_points_status =
      bev_slicer_->GetSamplingPointStatus();
  match_maker_->Associate(bev_slices, ldmap_slices, ld_map_lane_group,
                          bev_map_sampling_points_status);
#ifdef PRINT_BEV_LDMAP_MATCHING_INFO
  match_maker_->PrintMatchers();
  match_maker_->PrintMatchingDetails();
#endif
  auto& match_details = match_maker_->GetMatchDetails();

  data_fusion_controller_->Process(match_details,
                                   bev_map_preprocessor_->GetDataPose());
  EstimateEgoLaneErrorBetweenLdMapAndBevMap();

  // data_fusion_controller_->GetFusedMap();

  // visualizer_->Process(bev_map, ld_map);

  // 车道信息接口：地图与感知的匹配信息、地图车道的类型（normal/emergence）、（地图拓扑类型，地图拓扑剩余距离，拓扑车道数量）
  // auto matchers_ = match_maker_->GetMatchers();
  PrintLaneTopoInfo();
  MappingMergeSplitInfoFromLdMapToBevMap();
  #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    uint64_t input_bev_id = 52;
    /* std::string debug_info_1="";
    const auto& split_points = GetBevLaneMergeInfo(input_bev_id, true);
    for (auto& merge_info : split_points) {
      debug_info_1 += "---> Find: From: ";
      for (auto& bev_lanes : merge_info->bev_lanes_from) {
        debug_info_1 += " id: " + std::to_string(bev_lanes.first) +
              " <" + std::to_string(static_cast<int>(bev_lanes.second.type)) + ">";
      }

      debug_info_1 += " ,To: " + std::to_string(merge_info->bev_lane_to);
      debug_info_1 += ", match_type: <" + std::to_string(merge_info->match_type) + ">";
      debug_info_1 += ", dis: " + std::to_string(merge_info->distance_to_ego);
    }
    AINFO << debug_info_1; */
#endif

  //感知线型替换地图线型
  uint64_t              bev_ego_lane_id_out;
  std::vector<uint64_t> ld_ego_lane_ids_out;
  if (GetEgoLaneMatchInfo(bev_ego_lane_id_out, ld_ego_lane_ids_out)) {
    map_fusion_->SetGeometryMatchInfo(bev_ego_lane_id_out, ld_ego_lane_ids_out);
  }
  FunctionTimer::measureTime("map_fusion_", [&]() {map_fusion_->Process(bev_map_, ld_map_);});
}

std::optional<Eigen::Isometry3d> FusionManager::FindTransform(
    const double& timestamp) {
  LocalizationPtr odom_ptr{nullptr};
  SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05,
                                                      odom_ptr);

  if (odom_ptr == nullptr) {
    return std::nullopt;
  }

  Eigen::Isometry3d T_local_ego = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd R_local_ego((odom_ptr->attitude_dr) * M_PI / 180.0,
                                Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d t(odom_ptr->posne_dr.at(0), odom_ptr->posne_dr.at(1), 0.0);
  T_local_ego.rotate(R_local_ego);
  T_local_ego.pretranslate(t);

  return T_local_ego;
}
LaneType FusionManager::GetLaneTypeByBevId(uint64_t bev_id) const {
  std::vector<uint64_t> ld_map_ids;
  LaneType ld_map_lane_type = LaneType::LANE_UNKNOWN;
  const auto& matchers_ = match_maker_->GetMatchers();

  for (const auto& matcher : matchers_) {
    if (matcher.bev_id == bev_id && !matcher.map_id.empty()) {
      ld_map_ids = matcher.map_id;
      for (uint64_t ld_map_id : ld_map_ids) {
        const auto& ld_lane = ld_map_preprocessor_->GetLane(ld_map_id);
        if (ld_lane == nullptr) {
          // AINFO << "NO ld lane";
          continue;
        }
        ld_map_lane_type = ld_lane->type;
        /* # ifdef PRINT_BEV_LDMAP_MATCHING_INFO
          AINFO << "GetLaneTypeByBevId, bev_id:" << bev_id;
          AINFO << "ld:" << ld_map_id;
          AINFO << ", type: " << static_cast<int>(ld_map_lane_type);
        # endif  */
        if (ld_map_lane_type == LaneType::LANE_EMERGENCY || ld_map_lane_type == LaneType::LANE_HARBOR_STOP) {
          return ld_map_lane_type;
        }
      }
    }
  }
  /* # ifdef PRINT_BEV_LDMAP_MATCHING_INFO
  if (ld_map_lane_type == LaneType::LANE_UNKNOWN) {
    AINFO << "GetLaneTypeByBevId Failed, bev_id: " << bev_id;
  }
  # endif */ 
  return ld_map_lane_type;
}

void FusionManager::GetSplitInfoByBevId(uint64_t bev_id) {
  
  const auto& matchers_ = match_maker_->GetMatchers();
  auto& match_details = match_maker_->GetMatchDetails();
  auto& split_info_ = ld_map_preprocessor_->GetSplitInfo();
  auto& connect_info_ = ld_map_preprocessor_->GetConnectInfo();

  // CASE：区分一个bev在一个拓扑点和两个target to匹配上，不能在input_bev一样时，
  // 既有split，又有connect，优先选择split
  // std::vector<std::tuple<uint32_t, uint64_t, LdMapProcessor::MergeSplitType>> split_to_bev_lanes; 
  // 记录target_to :[input_group_id, to_bev_id, type]
    
  for (auto& bev_matcher : matchers_) {
    // CASE： 一个input bev_id 可以找到多个对应的多个group_to,找到这个bev上所有的topo点
    if (bev_matcher.bev_id == bev_id){
      uint32_t input_group_id = bev_matcher.ld_map_group_id;
      // AINFO << "Split:matcher_bev_id: " << bev_matcher.bev_id << ", group_id: " << group_id;
      
      for (const auto& [ele_id, info] : split_info_) {
        uint32_t info_group_from = info.group_from;
        if (info_group_from == input_group_id) {
          // AINFO << "find group id: (" << input_group_id << ")in info";

          // 处理 target_to 中的每个目标，一般一个类型info只有一个target_to匹配
          for (auto iter_target = info.target_to.begin(); 
              iter_target != info.target_to.end(); 
              ++iter_target) {
            const auto& [lane_id, target] = *iter_target;
            uint32_t group_to = target.group_to;
            // AINFO << "group_to:" << group_to;
            if ((target.distance_to_ego < -50) || (target.distance_to_ego > 200)) {
              // AINFO << "out of distance";
              continue;
            }

            BevSplitInfo lane_topo_info;
            lane_topo_info.bev_lane_from = bev_id;
            lane_topo_info.match_type = target.type; // Split的type
            lane_topo_info.distance_to_ego = target.distance_to_ego;
            lane_topo_info.map_lanes_from.insert(
              lane_topo_info.map_lanes_from.end(), 
              bev_matcher.map_id.begin(), 
              bev_matcher.map_id.end()
            );

            // 一个类型内的 一种拓扑类型 对应的 bev
            std::vector<std::pair<uint64_t, size_t>> target_to_bev_location; // [bev_id, bev_end_point_idx]
            BevSplitTarget split_target;

            for (auto& bev_target_matcher : matchers_){
              // target_to 中的group_to找到对应的bev_id
              if (bev_target_matcher.ld_map_group_id == group_to){

                uint64_t target_to_bev_id = bev_target_matcher.bev_id;
                split_target.type = target.type;
                split_target.map_lanes_to.insert(
                  split_target.map_lanes_to.end(), 
                  bev_target_matcher.map_id.begin(), 
                  bev_target_matcher.map_id.end()
                );

                for (const auto& match_detail : match_details) {
                  if (match_detail.first == target_to_bev_id) {
                    const auto& match_detail_vector = match_detail.second;

                    if (!match_detail_vector.empty()) {
                      const auto& first_matching_detail = match_detail_vector.front();
                      if (first_matching_detail->bev_end_point_idx != 0) {
                        target_to_bev_location.push_back(
                          std::make_pair(first_matching_detail->bev_id, 
                                        first_matching_detail->bev_end_point_idx));
                      }
                    }
                  }
                }
              }
            }  

            if (!target_to_bev_location.empty()) {
              split_target.bev_lane_to = target_to_bev_location[0].first;
              // CASE: 一个target_to.group_id 对应多个bev_id,按照逻辑更新split_target.bev_lane_to
              if (target_to_bev_location.size() > 1){
                // 一个group_to 对应bev从近到远排序
                std::sort(target_to_bev_location.begin(), target_to_bev_location.end(),
                          [](const std::pair<uint64_t, size_t>& a, const std::pair<uint64_t, size_t>& b) {
                            return a.second < b.second;
                          });
                for (size_t i = 0; i < target_to_bev_location.size(); ++i) {
                  auto [target_to_bev, bev_end_point_idx] = target_to_bev_location[i];
                  if (target_to_bev == bev_id) {
                    // Use the next element if it exists
                    if (i + 1 < target_to_bev_location.size()) {
                      split_target.bev_lane_to = target_to_bev_location[i + 1].first;
                      // AINFO << "target_to_bev.size() > 1, 1st_bev to self, next_bev: " << split_target.bev_lane_to;
                      break;
                    }
                  }
                }
              }       
              // AINFO << "target_to.group_id matched bev_id:" << split_target.bev_lane_to;

              // lane_topo_info.bev_target_to.emplace(split_target.bev_lane_to, split_target);
              lane_topo_info.bev_target_to.push_back(
                std::make_pair(split_target.bev_lane_to, split_target)
              );
            } /* else {
              AINFO << "target_to_bev_location.empty";
            } */

            // 查找这个拓扑点对应的connect点
            GetBevSplitConnectInfo(input_group_id, lane_topo_info);
            if ( !lane_topo_info.bev_target_to.empty() ) {
              bev_split_info_.push_back(lane_topo_info);
            } /* else {
              AINFO << "Faild to get input_bev : " << bev_id;
            } */
          }
        }
      }
    }
  }
}

void FusionManager::GetBevSplitConnectInfo(   
  uint32_t input_group_id, 
  BevSplitInfo& lane_topo_info) {

  auto& connect_info_ = ld_map_preprocessor_->GetConnectInfo();
  const auto& matchers_ = match_maker_->GetMatchers();
  auto& match_details = match_maker_->GetMatchDetails();
  
  uint64_t bev_id = lane_topo_info.bev_lane_from;
  uint64_t match_split_bev_id  = 0;
  if ( lane_topo_info.bev_target_to.size() > 0 ) {
    for (const auto& entry : lane_topo_info.bev_target_to) {
      match_split_bev_id = entry.first;
      break;
    }
  }
  double split_distance = lane_topo_info.distance_to_ego;

  for (const auto& [ele_id, info] : connect_info_) {
    uint32_t info_group_from = info.group_from;
    if (info_group_from == input_group_id) {
      // AINFO << "find group id: (" << input_group_id << ")in Connect info";

      // 处理 target_to 中的每个目标，一般一个类型info只有一个target_to匹配
      for (auto iter_target = info.target_to.begin(); 
            iter_target != info.target_to.end(); 
            ++iter_target) {
        const auto& [lane_id, target] = *iter_target;
        if ( std::abs(target.distance_to_ego - split_distance) > 1 ) {
          continue;
        }
        uint32_t group_to = target.group_to;
        // AINFO << "C_group_to:" << group_to;
        // 一个类型内的 一种拓扑类型 对应的 bev
        std::vector<std::pair<uint64_t, size_t>> target_to_bev_location; // [bev_id, bev_end_point_idx]
        BevSplitTarget split_target;

        for (auto& bev_target_matcher : matchers_){
          // target_to 中的group_to找到对应的bev_id
          if (bev_target_matcher.ld_map_group_id == group_to){

            uint64_t target_to_bev_id = bev_target_matcher.bev_id;

            if (target_to_bev_id == match_split_bev_id) {
              // AINFO << "find " << input_group_id << " again in connect info, to same bev: " << target_to_bev_id;
              continue;
            }
            split_target.type = target.type;
            split_target.map_lanes_to.insert(
              split_target.map_lanes_to.end(), 
              bev_target_matcher.map_id.begin(), 
              bev_target_matcher.map_id.end()
            );

            for (const auto& match_detail : match_details) {
              if (match_detail.first == target_to_bev_id) {
                const auto& match_detail_vector = match_detail.second;

                if (!match_detail_vector.empty()) {
                  const auto& first_matching_detail = match_detail_vector.front();
                  if (first_matching_detail->bev_end_point_idx != 0) {
                    target_to_bev_location.push_back(
                      std::make_pair(first_matching_detail->bev_id, 
                                      first_matching_detail->bev_end_point_idx));
                  }
                }
              }
            }
          }
        }
        if (!target_to_bev_location.empty()) {

          split_target.bev_lane_to = target_to_bev_location[0].first;
          // CASE: 一个target_to.group_id 对应多个bev_id,按照逻辑更新split_target.bev_lane_to
          if (target_to_bev_location.size() > 1){
            // 一个group_to 对应bev从近到远排序
            std::sort(target_to_bev_location.begin(), target_to_bev_location.end(),
                      [](const std::pair<uint64_t, size_t>& a, const std::pair<uint64_t, size_t>& b) {
                        return a.second < b.second;
                      });
            for (size_t i = 0; i < target_to_bev_location.size(); ++i) {
              auto [target_to_bev, bev_end_point_idx] = target_to_bev_location[i];
              if (target_to_bev == bev_id) {
                // Use the next element if it exists
                if (i + 1 < target_to_bev_location.size()) {
                  split_target.bev_lane_to = target_to_bev_location[i + 1].first;
                  // AINFO << "target_to_bev.size() > 1, 1st_bev to self, next_bev: " << split_target.bev_lane_to;
                  break;
                }
              }
            }
          }
          // AINFO << "target_to.group_id matched bev_id:" << split_target.bev_lane_to;
          // lane_topo_info.bev_target_to.emplace(split_target.bev_lane_to, split_target);
          lane_topo_info.bev_target_to.push_back(
            std::make_pair(split_target.bev_lane_to, split_target)
          );
        }  
      
      }
    }
  }
}

void FusionManager::CheckChangeTopoByBevId(uint64_t bev_id) {
  const auto& matchers_ = match_maker_->GetMatchers();

  std::set<uint32_t> search_group;
  for (auto& bev_matcher : matchers_) {
   if (bev_matcher.bev_id == bev_id){
      uint32_t input_group_id = bev_matcher.ld_map_group_id;

      if (search_group.find(input_group_id) != search_group.end()) {
          // 如果已经处理过，跳过后续处理
          continue;
      }
      search_group.insert(input_group_id);
      // 创建一个新的visited_groups集合，用于防止递归调用中的循环引用
      std::set<uint32_t> visited_groups;
      BuildFarTopoForward(bev_id, input_group_id, visited_groups);
    }
  }
}

void FusionManager::BuildFarTopoInfo(
  uint64_t bev_id, 
  uint32_t group_from,
  const LdMapProcessor::MergeSplitTarget& target_to) {
  
  auto& merge_info_ = ld_map_preprocessor_->GetMergeInfo();
  auto to_group_id = target_to.group_to;
  double merge_distance = target_to.distance_to_ego;
  FarTopoInfo far_merge_info;
  far_merge_info.bev_id = bev_id;
  far_merge_info.group_from = group_from;
  far_merge_info.group_to = to_group_id;
  far_merge_info.distance_to_ego = merge_distance;
  far_merge_info.type = target_to.type;

  #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    AINFO << "---> Fartopo: bev_id: " << bev_id
          << ", from: g" << group_from
          << ", to: g" << to_group_id
          << ", merge_distance: " << merge_distance
          << ", type: " << static_cast<int>(target_to.type);
  #endif

  far_merge_info_.push_back(far_merge_info);
}

void FusionManager::BuildFarTopoForward(uint64_t bev_id, uint32_t input_group_id, std::set<uint32_t>& visited_groups) {
  // 检查是否已经访问过当前group_id，防止无限递归
  if (visited_groups.find(input_group_id) != visited_groups.end()) {
    // AINFO << "<<WORNING>>: visited_groups: " <<  input_group_id;
    return;
  }
  visited_groups.insert(input_group_id);
  
  const auto& matchers_ = match_maker_->GetMatchers();
  auto& split_info_ = ld_map_preprocessor_->GetSplitInfo();
  auto& merge_info_ = ld_map_preprocessor_->GetMergeInfo();
  auto& connect_info_ = ld_map_preprocessor_->GetConnectInfo();
  
  for (const auto& [ele_id, info] : merge_info_) {
    uint32_t info_group_from = info.group_from;
    if (info_group_from == input_group_id) {
      for (auto iter_target = info.target_to.begin();
          iter_target != info.target_to.end();
          ++iter_target) {
        const auto& [lane_id, target] = *iter_target;
        
        bool found = false;
        for (const auto& far_merge_info : far_merge_info_) {
            if (far_merge_info.bev_id == bev_id && far_merge_info.group_from == info_group_from) {
                found = true;
            }
        }
        if (!found) {
          BuildFarTopoInfo(bev_id, info_group_from, target);
        }
        return;
      }
    }
  }

  for (const auto& [ele_id, info] : split_info_) {
    uint32_t info_group_from = info.group_from;
    if (info_group_from == input_group_id) {
      return;
    }
  }

  std::unordered_set<uint32_t> connect_succesor_groups;
  for (const auto& [ele_id, info] : connect_info_) {
    uint32_t info_group_from = info.group_from;
    if (info_group_from == input_group_id) {
      for (auto iter_target = info.target_to.begin();
          iter_target != info.target_to.end();
          ++iter_target) {
        const auto& [lane_id, target] = *iter_target;
        // 存在一个group_from有多个connect后继
        connect_succesor_groups.insert(target.group_to);
      }
    }
  }
  
  // 一个group_from有多个connect后继或者没有connect后继就return结束
  if (connect_succesor_groups.size() == 1) {
    // 取出唯一的元素送入BuildFarTopoForward
    uint32_t group_to = *connect_succesor_groups.begin();
    BuildFarTopoForward(bev_id, group_to, visited_groups);
  } 
  #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    if (connect_succesor_groups.empty()) {
      AINFO << "Failed build Fartopo---Empty succ: bev_id :" << bev_id;
    } else if (connect_succesor_groups.size() > 1) {
      AINFO << "Failed build Fartopo---succ.size = " << connect_succesor_groups.size();
    }
  #endif

  return;
}

void FusionManager::GetBevMergeConnectInfo(
  uint32_t to_group_id, 
  BevMergeInfo& bev_merge_info) {

  auto& merge_info_ = ld_map_preprocessor_->GetMergeInfo();
  const auto& matchers_ = match_maker_->GetMatchers();
  auto& match_details = match_maker_->GetMatchDetails();
  
  double merge_distance = bev_merge_info.distance_to_ego;
  uint64_t merge_to_bev = bev_merge_info.bev_lane_to;
  uint64_t match_merge_bev_id = 0;
  for (auto& bev_lanes : bev_merge_info.bev_lanes_from) {
    match_merge_bev_id = bev_lanes.first;
    break; //默认只有一个connect,这个取输入的bev_id
  }
   
  for (const auto& [ele_id, info] : merge_info_) {
    // 处理 target_to 中的每个目标，一般一个类型info只有一个target_to匹配
    for (auto iter_target = info.target_to.begin(); 
          iter_target != info.target_to.end(); 
          ++iter_target) {
      const auto& [lane_id, target] = *iter_target;
      if (std::abs(target.distance_to_ego - merge_distance) > 1) {
          continue;
      }
      if (target.group_to != to_group_id) {
        continue;
      }
      bev_merge_info.match_type = target.type;
      uint32_t merge_with_group_id = info.group_from;
      
      // 是同一个拓扑点
      std::vector<std::pair<uint64_t, size_t>> target_to_bev_location; // [bev_id, bev_end_point_idx]
      BevMergeSource merge_source;
      for (auto& bev_target_matcher : matchers_){
        // target_to 中的group_to找到对应的bev_id
        if (bev_target_matcher.ld_map_group_id == merge_with_group_id){

          uint64_t target_to_bev_id = bev_target_matcher.bev_id;

          if (target_to_bev_id == match_merge_bev_id) {
            // AINFO << "merge with_group_id: " << merge_with_group_id << "match parallel two merge from lanes: " << match_merge_bev_id;
            continue;
          }
          
          merge_source.map_lanes_from.insert(
            merge_source.map_lanes_from.end(), 
            bev_target_matcher.map_id.begin(), 
            bev_target_matcher.map_id.end()
          );
          
          for (const auto& match_detail : match_details) {
            if (match_detail.first == target_to_bev_id) {
              const auto& match_detail_vector = match_detail.second;

              if (!match_detail_vector.empty()) {
                const auto& first_matching_detail = match_detail_vector.front();
                if (first_matching_detail->bev_end_point_idx != 0) {
                  target_to_bev_location.push_back(
                    std::make_pair(first_matching_detail->bev_id, 
                                    first_matching_detail->bev_end_point_idx));
                }
              }
            }
          }
        }
      }
      if (!target_to_bev_location.empty()) {
        merge_source.type = target.type;
        uint64_t bev_lane_to =target_to_bev_location[0].first;

        if (target_to_bev_location.size() > 1){
          std::sort(target_to_bev_location.begin(), target_to_bev_location.end(),
                      [](const std::pair<uint64_t, size_t>& a, const std::pair<uint64_t, size_t>& b) {
                        return a.second < b.second;
                      });
          for (size_t i = 0; i < target_to_bev_location.size(); ++i) {
            auto [target_to_bev, bev_end_point_idx] = target_to_bev_location[i];
            if (target_to_bev == merge_to_bev) {
              // Use the next element if it exists
              if (i + 1 < target_to_bev_location.size()) {
                bev_lane_to = target_to_bev_location[i + 1].first;
                AINFO << "target_to_bev.size() > 1, merge_from match merge_to_bev, next_bev: " << bev_lane_to;
                break;
              }
            }
          }
        }
        merge_source.bev_lane_from = bev_lane_to;
        bev_merge_info.bev_lanes_from.emplace(bev_lane_to, merge_source);
      }
    }
  }
}

void FusionManager::GetMergeInfoByBevId(
    uint64_t bev_id, 
    std::vector<const LdMapProcessor::MergeSplitInfo*> merge_connect) {
  const auto& matchers_ = match_maker_->GetMatchers();
  auto& match_details = match_maker_->GetMatchDetails();
  
  // 当前input_bev_id 能匹配到的所有group_id
  for (auto& bev_matcher : matchers_) {
    if (bev_matcher.bev_id == bev_id){
      uint32_t input_group_id = bev_matcher.ld_map_group_id;

      for (auto it = merge_connect.begin(); it != merge_connect.end(); ++it) {
        const auto* connect_info_ptr = *it;
        if (connect_info_ptr != nullptr) {
            uint32_t info_group_from = connect_info_ptr->group_from;
            // 在merge_connect里面找到group_from
            if (info_group_from == input_group_id) {
              // AINFO << "Input_bev_id: " << bev_matcher.bev_id 
              //       << ", group_id: " << info_group_from;
              bool new_BevMergeInfo = true;
              //遍历 merge_connect里的target_to里的 group to、distance_to_ego
              for (const auto& target_pair : connect_info_ptr->target_to) {
                const auto& target = target_pair.second;
                uint32_t group_to = target.group_to;

                // AINFO << "merge_connection_info: group_to: " << group_to;
                if ((target.distance_to_ego < -50) || (target.distance_to_ego > 200)) {
                  // AINFO << "out of distance";
                  continue;
                }

                // 先匹配group_to 对应的bev_to,再结合距离，判断是否已存在这个拓扑点
                std::vector<std::pair<uint64_t, size_t>> target_to_bev_location;
                BevMergeInfo bev_merge_info;
                
                for (auto& bev_target_matcher : matchers_){
                  // target_to 中的group_to找到对应的bev_id
                  if (bev_target_matcher.ld_map_group_id == group_to){
                    bev_merge_info.map_lanes_to.insert(
                      bev_merge_info.map_lanes_to.end(), 
                      bev_target_matcher.map_id.begin(), 
                      bev_target_matcher.map_id.end()
                    );

                    uint64_t target_to_bev_id = bev_target_matcher.bev_id;

                    for (const auto& match_detail : match_details) {
                      if (match_detail.first == target_to_bev_id) {
                        const auto& match_detail_vector = match_detail.second;

                        if (!match_detail_vector.empty()) {
                          const auto& first_matching_detail = match_detail_vector.front();
                          if (first_matching_detail->bev_end_point_idx != 0) {
                            target_to_bev_location.push_back(
                              std::make_pair(first_matching_detail->bev_id, 
                                            first_matching_detail->bev_end_point_idx));
                          }
                        }
                      }
                    }
                  }
                } 
                  
                uint64_t bev_lane_to = 0; //target_to_bev_location.empty()，merge_to没有匹配上
                // 更新 bev_lane_to
                if (!target_to_bev_location.empty()) {
                  bev_lane_to = target_to_bev_location[0].first;
                  // AINFO << "target_to_bev_id: " << bev_lane_to << " Merge_info: bev_lane_to!=0" ;
                  if (target_to_bev_location.size() > 1){
                    std::sort(target_to_bev_location.begin(), target_to_bev_location.end(),
                            [](const std::pair<uint64_t, size_t>& a, const std::pair<uint64_t, size_t>& b) {
                              return a.second < b.second;
                            });
                  }
                  for (size_t i = 0; i < target_to_bev_location.size(); ++i) {
                    auto [target_to_bev, bev_end_point_idx] = target_to_bev_location[i];
                    if (target_to_bev == bev_id) {
                      // Use the next element if it exists
                      if (i + 1 < target_to_bev_location.size()) {
                        bev_lane_to = target_to_bev_location[i + 1].first;
                        // AINFO << "target_to_bev.size() > 1, 1st_bev to self, next_bev: " << bev_lane_to;
                        break;
                      }
                    }
                  }
                  // AINFO << "bev_connect_to: " << bev_lane_to;
                } /* else {
                  AINFO << "bev_id: " << bev_id << " Merge_info: bev_lane_to=0" ;
                } */
                
                // 遍历bev_merge_info_中的bev_lane_to，distance_to_ego相等的拓扑点，如果有就跳过。
                // bev_merge_info_ 有可能是空的
                if (!bev_merge_info_.empty()) {
                  for (const auto& merge_info : bev_merge_info_) {
                    // TODO :再加一个from的判断
                    if (merge_info.bev_lane_to == bev_lane_to && 
                        std::abs(merge_info.distance_to_ego - target.distance_to_ego) < 1) {
                        new_BevMergeInfo = false;
                        continue;
                    }
                  }
                }
                
                if (new_BevMergeInfo) {
                  // 建立bev_merge_info 的拓扑点
                  bev_merge_info.bev_lane_to = bev_lane_to;
                  bev_merge_info.distance_to_ego = target.distance_to_ego;
                  bev_merge_info.match_type = LdMapProcessor::MergeSplitType::kConnectForward; //默认先是0

                  BevMergeSource merge_source;
                  merge_source.bev_lane_from = bev_id;
                  merge_source.type = target.type;
                  merge_source.map_lanes_from.insert(
                    merge_source.map_lanes_from.end(), 
                    bev_matcher.map_id.begin(), 
                    bev_matcher.map_id.end()
                  );

                  bev_merge_info.bev_lanes_from.emplace(bev_id, merge_source);
                  
                  GetBevMergeConnectInfo(group_to, bev_merge_info);

                  // 整合bev_merge_info到 merge_info_
                  if (bev_merge_info.bev_lane_to == 0 & bev_merge_info.bev_lanes_from.size() < 2) {
                    // AINFO << bev_id << "Merge faild: bev_lane_to = 0 and bev_lanes_from.size < 2";
                    continue;
                  } else {
                    bev_merge_info_.push_back(bev_merge_info);
                  }
                } else {
                  continue;
                }
              }
            }
          }
      }
    }
  }
}

//New:
std::vector<const BevSplitInfo*> FusionManager::GetBevLaneSplitInfo(
  uint64_t id_from) const {
  std::vector<const cem::fusion::BevSplitInfo*> ret;
  // 检索bev_split_info_,传递符合条件的元素指针放到ret
  for (const auto& split_info : bev_split_info_) {
    if (split_info.bev_lane_from == id_from) {
        ret.push_back(&split_info);
    }
  }
  return ret;  
}

std::vector<const BevMergeInfo*> FusionManager::GetBevLaneMergeInfo(
    uint64_t search_id, bool search_lane_to) const {
  std::vector<const BevMergeInfo*> ret;

  // 检索bev_merge_info_,传递符合条件的元素指针放到ret
  if (!search_lane_to) {
    for (const auto& merge_info : bev_merge_info_) {
      auto& from_lanes = merge_info.bev_lanes_from;
      for (const auto& from_lane : from_lanes) {
        if (from_lane.first == search_id) {
            ret.push_back(&merge_info);
        }
      }
    }
  } else {
    for (const auto& merge_info : bev_merge_info_) {
      if (merge_info.bev_lane_to == search_id) {
        ret.push_back(&merge_info);
      }
    }
  }
  return ret;
}

std::vector<const FarTopoInfo*> FusionManager::GetFarMergeInfo(
  uint64_t bev_id, 
  double min_distance, 
  double max_distance) const {
  const FarTopoInfo* min_info = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  
  for (const auto& far_merge_info : far_merge_info_) {
    if (far_merge_info.bev_id == bev_id && 
        far_merge_info.distance_to_ego < max_distance &&
        far_merge_info.distance_to_ego > min_distance) {

        // 检查是否是更小的距离
        if (far_merge_info.distance_to_ego < min_dist) {
          min_dist = far_merge_info.distance_to_ego;
          min_info = &far_merge_info;
        }
      }
  }
  
  std::vector<const FarTopoInfo*> ret;
  if (min_info != nullptr) {
    ret.push_back(min_info);
    // AINFO << "FarMerge bev: " << min_info->bev_id
    //       << " distance: " << min_info->distance_to_ego
    //       << " group-from: " << min_info->group_from
    //       << " group-to: " << min_info->group_to;
  }
  return ret;
}

void FusionManager::MappingMergeSplitInfoFromLdMapToBevMap() {
  bev_split_info_.clear();
  bev_merge_info_.clear();
  far_merge_info_.clear();
  auto& split_info_ = ld_map_preprocessor_->GetSplitInfo();
  auto& merge_info_ = ld_map_preprocessor_->GetMergeInfo();
  auto& connect_info_ = ld_map_preprocessor_->GetConnectInfo();

  std::vector<const LdMapProcessor::MergeSplitInfo*> merge_connect;
  std::unordered_set<uint32_t> merge_group_to_set;  // 用于快速查找

  // 先收集所有merge_info中的group_to值
  for (const auto& [ele_id, merge_info] : merge_info_) {
    for (const auto& [lane_id, target2] : merge_info.target_to) {
      merge_group_to_set.insert(target2.group_to);
    }
  }

  // 然后查找connect_info中匹配的项
  for (const auto& [ele_id, connect_info] : connect_info_) {
    for (const auto& [lane_id, target] : connect_info.target_to) {
      uint32_t connect_group_to = target.group_to;
      // 检查是否在merge_info的group_to集合中
      if (merge_group_to_set.find(connect_group_to) != merge_group_to_set.end()) {
        merge_connect.push_back(&connect_info);  // 存储指针
      }
    }
  }

  for (auto& bev_lane : bev_map_->lane_infos){
    //以拓扑点添加BevSplitInfo*
    GetSplitInfoByBevId(bev_lane.id);
    GetMergeInfoByBevId(bev_lane.id, merge_connect);
    CheckChangeTopoByBevId(bev_lane.id);
  }
  
  #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
    DebugLaneTopoInfo();
  #endif
}

void FusionManager::DebugLaneTopoInfo(){
  // match_maker_->PrintMatchers();
  AINFO << "---> Merge_info_: ";
  for (auto& merge_info : bev_merge_info_) {
    AINFO << "From: ";
    for (auto& bev_lanes : merge_info.bev_lanes_from) {
      AINFO << "id: " << bev_lanes.first 
            << ", type" << static_cast<int>(bev_lanes.second.type);
      std::string ldmap_ids_string = "(";
      for (auto ldmap_lane_id : bev_lanes.second.map_lanes_from) {
        ldmap_ids_string += std::to_string(ldmap_lane_id);
        ldmap_ids_string += " ";
      }
      AINFO << "map_lanes_to: " << ldmap_ids_string + ")";
    }

    AINFO << "To: " << merge_info.bev_lane_to;
    std::string ldmap_ids_string = "(";
    for (auto ldmap_lane_id : merge_info.map_lanes_to) {
        ldmap_ids_string += std::to_string(ldmap_lane_id);
        ldmap_ids_string += " ";
    }
    AINFO << "map_lanes_to: " << ldmap_ids_string + ")";
    AINFO << "match_type: " << merge_info.match_type;
    AINFO << "distance: " << merge_info.distance_to_ego;
  }

  AINFO << "---> Split_info_: ";
  for (auto& split_info : bev_split_info_) {
    AINFO << "From: " << split_info.bev_lane_from;
    std::string ldmap_ids_string = "(";
    for (auto ldmap_lane_id : split_info.map_lanes_from) {
      ldmap_ids_string += std::to_string(ldmap_lane_id);
      ldmap_ids_string += " ";
    }
    AINFO << "map_lanes_from: " << ldmap_ids_string + ")";
    AINFO << "To: ";
    for (auto& bev_lanes : split_info.bev_target_to) {
      AINFO << "id: " << bev_lanes.first 
            << ", type" << static_cast<int>(bev_lanes.second.type);
      std::string ldmap_ids_string = "(";
      for (auto ldmap_lane_id : bev_lanes.second.map_lanes_to) {
        ldmap_ids_string += std::to_string(ldmap_lane_id);
        ldmap_ids_string += " ";
      }
      AINFO << "map_lanes_to: " << ldmap_ids_string + ")";
    }
    AINFO << "match_type: " << split_info.match_type;
    AINFO << "distance: " << split_info.distance_to_ego;
  }
}

void FusionManager::PrintLaneTopoInfo() {
  // debug_info_.clear();
  auto& split_info_ = ld_map_preprocessor_->GetSplitInfo();
  auto& merge_info_ = ld_map_preprocessor_->GetMergeInfo();
  auto& connect_info_ = ld_map_preprocessor_->GetConnectInfo();
  const auto& matchers_ = match_maker_->GetMatchers();

  // 打印目前的merge_info_ connect_info  split_info_
  auto print_split_merge_info = [&](const std::unordered_map<uint64_t, LdMapProcessor::MergeSplitInfo>& info_map) {
    for (auto iter = info_map.cbegin(); iter != info_map.cend(); ++iter) {;
      // AINFO << "topo_to_lane_id: " << iter->first;
      uint32_t group_from_id = iter->second.group_from;
      const auto& targets = iter->second.target_to;

      for (auto target_iter = targets.cbegin(); target_iter != targets.cend(); ++target_iter) {
        uint32_t group_to_id = target_iter->second.group_to;
        double distance_to_ego = std::round(target_iter->second.distance_to_ego);
        if (distance_to_ego > 500 || distance_to_ego < -50) {
          continue;
        }
        int merge_split_type = static_cast<int>(target_iter->second.type);
        
        #ifdef PRINT_BEV_LDMAP_MATCHING_INFO
          AINFO << "T: " << merge_split_type 
                << ", dis:" << distance_to_ego
                << " g" << group_from_id 
                << " -to- g" << group_to_id;
        #endif
        std::string result = "[ T" + std::to_string(merge_split_type) +
                            ", dis: " + std::to_string(distance_to_ego) + 
                            "g" + std::to_string(group_from_id) +
                            " -to- g" + std::to_string(group_to_id) + "] ";
        debug_info_ += result;
      }
    }
  };
  debug_info_ += "topo_info_: {";
  print_split_merge_info(split_info_);
  // AINFO << "merge_info_";
  print_split_merge_info(merge_info_);
  // AINFO << "connect_info_";
  print_split_merge_info(connect_info_);

  debug_info_ += "}, match_info_: {";
  for (auto& bev_target_matcher : matchers_){
    uint32_t group_to = bev_target_matcher.ld_map_group_id;
    uint64_t target_to_bev_id = bev_target_matcher.bev_id;
    std::string ldmap_ids_string = "(";
    for (auto ldmap_lane_id : bev_target_matcher.map_id) {
      ldmap_ids_string += std::to_string(ldmap_lane_id);
      ldmap_ids_string += " ";
    }
    std::string result = "[ bev" + std::to_string(target_to_bev_id) +
                         " == g" + std::to_string(group_to) + 
                         " ld: " + ldmap_ids_string + ") ];\n";

    debug_info_ += result;
  }
  debug_info_ += "};";

  // 添加DebugLaneTopoInfo中的信息
  debug_info_ += "Merge_info_: ";
  for (auto& merge_info : bev_merge_info_) {
    debug_info_ += "From: ";
    for (auto& bev_lanes : merge_info.bev_lanes_from) {
      debug_info_ += " id: " + std::to_string(bev_lanes.first) +
            " ( " + std::to_string(static_cast<int>(bev_lanes.second.type)) + ") ";
      // std::string ldmap_ids_string = "(";
      // for (auto ldmap_lane_id : bev_lanes.second.map_lanes_from) {
      //   ldmap_ids_string += std::to_string(ldmap_lane_id);
      //   ldmap_ids_string += " ";
      // }
      // debug_info_ += "map_lanes_to: " + ldmap_ids_string + ")\n";
    }

    debug_info_ += " ,To: " + std::to_string(merge_info.bev_lane_to);
    // std::string ldmap_ids_string = "(";
    // for (auto ldmap_lane_id : merge_info.map_lanes_to) {
    //     ldmap_ids_string += std::to_string(ldmap_lane_id);
    //     ldmap_ids_string += " ";
    // }
    // debug_info_ += "map_lanes_to: " + ldmap_ids_string + ")\n";
    debug_info_ += ", match_type: ( " + std::to_string(merge_info.match_type) + " )";
    debug_info_ += ", dis: " + std::to_string(merge_info.distance_to_ego);
  }

  debug_info_ += "Split_info_: ";
  for (auto& split_info : bev_split_info_) {
    debug_info_ += "From: " + std::to_string(split_info.bev_lane_from);
    // std::string ldmap_ids_string = "(";
    // for (auto ldmap_lane_id : split_info.map_lanes_from) {
    //   ldmap_ids_string += std::to_string(ldmap_lane_id);
    //   ldmap_ids_string += " ";
    // }
    // debug_info_ += "map_lanes_from: " + ldmap_ids_string + ")\n";
    debug_info_ += ", To: ";
    for (auto& bev_lanes : split_info.bev_target_to) {
      debug_info_ += "id: " + std::to_string(bev_lanes.first) +
            " ( " + std::to_string(static_cast<int>(bev_lanes.second.type)) + ") " ;
      // std::string ldmap_ids_string = "(";
      // for (auto ldmap_lane_id : bev_lanes.second.map_lanes_to) {
      //   ldmap_ids_string += std::to_string(ldmap_lane_id);
      //   ldmap_ids_string += " ";
      // }
      // debug_info_ += "map_lanes_to: " + ldmap_ids_string + ")\n";
    }
    debug_info_ += ", match_type: " + std::to_string(split_info.match_type);
    debug_info_ += ", dis: " + std::to_string(split_info.distance_to_ego);
  }
}

void FusionManager::RecommendDrivingLanes() {
  recommend_driving_lanes_.clear();
  auto matchers = match_maker_->GetMatchers();
  auto& match_details = match_maker_->GetMatchDetails();
  const auto& recommend_path = ld_map_preprocessor_->GetRecommendPath();
  const auto& ld_lane_group = ld_map_preprocessor_->GetLaneGroup();

  std::unordered_set<uint64_t>
      matched_pairs_from_start;  // 如果有后继的话存的是后继
  std::unordered_set<uint64_t> matched_pairs_from_ego;  // 只存当前段，不管后继
  for (auto& match_detail : match_details) {
    auto bev_lane = bev_map_preprocessor_->GetLane(match_detail.first);
    if (bev_lane == nullptr) {
      continue;
    }
    if (bev_lane->direction == BevLaneDirection::DIRECTION_BACKWARD) {
      continue;
    }

    double min_dist = std::numeric_limits<double>::max();
    for (auto& pt : bev_lane->line_points) {
      if (pt.x * pt.x + pt.y * pt.y < min_dist) {
        min_dist = pt.x * pt.x + pt.y * pt.y;
      }
    }
    if (min_dist > 15 * 15) {
      continue;
    }

    for (auto& path : recommend_path) {
      if (!path.from_start_section) {
        continue;
      }
      bool matched_in_start = false;
      for (auto& segment : match_detail.second) {
        if (ld_lane_group.count(segment->ldmap_id) == 0) {
          continue;
        }
        auto matched_segment_ld_group_id = ld_lane_group.at(segment->ldmap_id);

        for (auto& ld_lane : path.lanes) {
          if (ld_lane.first == matched_segment_ld_group_id) {
            matched_in_start = true;
            break;
          }
          if (ld_lane.second > 30.0) {
            break;
          }
        }
        if (matched_in_start) {
          break;
        }
      }
      if (matched_in_start) {
        matched_pairs_from_ego.emplace(bev_lane->id);
        auto tmp = GetAllLastLaneByTopology(bev_lane, bev_map_preprocessor_,
                                            bev_slicer_);

        for (auto bev_id : tmp) {
          if (match_details.count(bev_id) == 0) {
            continue;
          }
          if (matched_pairs_from_start.count(bev_id) == 0) {
            matched_pairs_from_start.emplace(bev_id);
          }
        }
        break;
      }
    }
  }
  // {
  //   std::string tmp = "lane from start: ";
  //   for (auto i : matched_pairs_from_start) {
  //     tmp += std::to_string(i);
  //     tmp += ", ";
  //   }
  //   AINFO << tmp;
  // }

  std::vector<std::pair<uint64_t, const LdMapProcessor::Path*>>
      matched_pairs_from_last;
  for (auto bev_id : matched_pairs_from_start) {
    std::vector<const LdMapProcessor::Path*> path_tmp;
    for (auto& path : recommend_path) {
      if (!path.from_start_section) {
        continue;
      }

      auto& match_detail = match_details.at(bev_id);
      for (auto iter = match_detail.rbegin(); iter != match_detail.rend();
           ++iter) {
        if (path.lane_eles_id.count((*iter)->ldmap_id) > 0) {
          path_tmp.emplace_back(&path);
          break;
        }
        if ((*iter)->bev_end_point_idx * bev_slicer_->GetSliceThickness() -
                bev_slicer_->GetSliceStartPoint().norm() <
            100) {
          break;
        }
      }
    }
    if (path_tmp.empty()) {
      continue;
    }

    const LdMapProcessor::Path* path_use = nullptr;
    for (auto& path : path_tmp) {
      if (path_use == nullptr) {
        path_use = path;
      } else {
        if (path->lanes.back().second > path_use->lanes.back().second) {
          path_use = path;
        }
      }
    }
    if (path_use != nullptr) {
      matched_pairs_from_last.push_back(
          std::pair<uint64_t, const LdMapProcessor::Path*>(bev_id, path_use));
    }
  }
  // {
  //   std::string tmp = "lane from last: ";
  //   for (auto i : matched_pairs_from_last) {
  //     tmp += std::to_string(i.first);
  //     tmp += ", ";
  //   }
  //   AINFO << tmp;
  // }

  // for (auto iter = matched_pairs_from_last.begin();
  //      iter != matched_pairs_from_last.end();) {
  //   auto bev_lane = bev_map_preprocessor_->GetLane(iter->first);
  //   if (bev_lane == nullptr) {
  //     iter = matched_pairs_from_last.erase(iter);
  //     continue;
  //   }
  //   auto last_lanes =
  //   GetAllLastLaneByTopology(bev_lane,bev_map_preprocessor_); if
  //   (last_lanes.count(bev_lane->id) == 0 && !last_lanes.empty()) {
  //     iter
  //   }
  // }

  if (matched_pairs_from_last.empty()) {
    for (auto& match_detail : match_details) {
      auto bev_lane = bev_map_preprocessor_->GetLane(match_detail.first);
      if (bev_lane == nullptr) {
        continue;
      }
      if (bev_lane->direction == BevLaneDirection::DIRECTION_BACKWARD) {
        continue;
      }
      // if (bev_lane->position !=
      //     static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER)) {
      //   continue;
      // }
      std::vector<const LdMapProcessor::Path*> path_tmp;
      for (auto& path : recommend_path) {
        if (path.from_start_section) {
          continue;
        }
        if (path.lane_eles_id.count(match_detail.second.back()->ldmap_id) > 0) {
          path_tmp.emplace_back(&path);
        }
      }
      if (path_tmp.empty()) {
        continue;
      }

      const LdMapProcessor::Path* path_use = nullptr;
      for (auto& path : path_tmp) {
        if (path_use == nullptr) {
          path_use = path;
        } else {
          if (path->lanes.back().second > path_use->lanes.back().second) {
            path_use = path;
          }
        }
      }

      if (path_use != nullptr) {
        matched_pairs_from_last.push_back(
            std::pair<uint64_t, const LdMapProcessor::Path*>(match_detail.first,
                                                             path_use));
      }
    }
  }
  // std::string tmp = "";
  // for (auto pair : matched_pairs_from_last) {
  //   tmp += std::to_string(pair.first);
  //   tmp += ", ";
  // }
  // AINFO << "LAST: " << tmp;

  if (matched_pairs_from_last.empty()) {
    for (auto bev_id : matched_pairs_from_ego) {
      auto& match_detail = match_details.at(bev_id);

      std::vector<const LdMapProcessor::Path*> path_tmp;
      for (auto& path : recommend_path) {
        if (!path.from_start_section) {
          continue;
        }
        if (path.lane_eles_id.count(match_detail.back()->ldmap_id) > 0) {
          path_tmp.emplace_back(&path);
          break;
        }
      }

      if (path_tmp.empty()) {
        continue;
      }

      const LdMapProcessor::Path* path_use = nullptr;
      for (auto& path : path_tmp) {
        if (path_use == nullptr) {
          path_use = path;
        } else {
          if (path->lanes.back().second > path_use->lanes.back().second) {
            path_use = path;
          }
        }
      }
      if (path_use != nullptr) {
        std::pair<int, int> info;
        info.first = static_cast<int>(path_use->lanes.back().second);
        info.second = -1;
        recommend_driving_lanes_.push_back(
            std::pair<uint64_t, std::pair<int, int>>(bev_id, std::move(info)));
      }
    }
  } else {
    for (auto& path : matched_pairs_from_last) {
      std::pair<int, int> info;
      info.first = static_cast<int>(path.second->lanes.back().second);
      info.second = -1;
      recommend_driving_lanes_.push_back(
          std::pair<uint64_t, std::pair<int, int>>(path.first,
                                                   std::move(info)));
    }
  }

  /*
  if (recommend_driving_lanes_.size() > 1) {
    for (auto iter = recommend_driving_lanes_.begin();
         iter != recommend_driving_lanes_.end();) {
      auto bev_lane = bev_map_preprocessor_->GetLane(iter->first);
      if (bev_lane == nullptr) {
        iter = recommend_driving_lanes_.erase(iter);
        continue;
      }
      bool already_removed = false;
      bool found_next_lane_matched = false;
      for (auto id : bev_lane->next_lane_ids) {
        auto iter_tmp = std::find_if(
            recommend_driving_lanes_.begin(), recommend_driving_lanes_.end(),
            [id](const std::pair<uint64_t, std::pair<int, int>>& path) {
              return id == path.first;
            });
        if (match_details.count(id) > 0) {
          found_next_lane_matched = true;
        }
        if (iter_tmp != recommend_driving_lanes_.end()) {
          already_removed = true;
          iter = recommend_driving_lanes_.erase(iter);
          break;
        }
      }
      if (!already_removed) {
        if (found_next_lane_matched) {
          iter = recommend_driving_lanes_.erase(iter);
        } else {
          ++iter;
        }
      }
    }
  }
  */

  int min_distance = std::numeric_limits<int>::max();
  int count_min_distance = 0;
  for (auto& path : recommend_driving_lanes_) {
    if (path.second.first < min_distance) {
      min_distance = path.second.first;
      count_min_distance = 1;
    } else if (path.second.first == min_distance) {
      count_min_distance += 1;
    }
  }

  bool all_same_distance =
      count_min_distance == recommend_driving_lanes_.size();
  if (!all_same_distance) {
    for (auto& path : recommend_driving_lanes_) {
      if (path.second.first > min_distance) {
        path.second.first = min_distance;
      } else {
        path.second.second = 0;
      }
    }
  }

  /*
  std::unordered_map<uint32_t, std::vector<const LdMapProcessor::Path*>>
      recommend_path_map;
  for (auto& path : recommend_path) {
    recommend_path_map[path.lanes.front().first].push_back(&path);
  }
  std::unordered_map<uint64_t, std::vector<const MatchMaker::LaneMatchPair*>>
      matcher_map;
  for (auto& matcher : matchers) {
    matcher_map[matcher.bev_id].push_back(&matcher);
  }

  // 对匹配多条地图车道的，进行排序
  for (auto& matcher : matcher_map) {
    if (matcher.second.size() > 1) {
      if (match_details.count(matcher.first) == 0) {
        continue;
      }
      std::vector<int> index;
      auto match_detail = match_details.at(matcher.first);
      for (auto& match_pair : matcher.second) {
        auto find_iter = std::find_if(
            match_detail.begin(), match_detail.end(),
            [match_pair](
                std::unique_ptr<cem::fusion::MatchMaker::MatchingDetail>&
                    detail) {
              return match_pair->map_id.front() == detail->ldmap_id;
            });
        index.push_back(find_iter - match_detail.begin());
      }

      // 使用索引进行排序
      std::vector<size_t> indices(matcher.second.size());
      std::iota(indices.begin(), indices.end(), 0);  // 初始化为0,1,2,...

      // 按照index中的值对indices进行排序
      std::sort(indices.begin(), indices.end(),
                [&index](size_t i, size_t j) { return index[i] < index[j]; });

      // 根据排好的indices重新排列matcher.second
      std::vector<const MatchMaker::LaneMatchPair*> sorted_matcher_second(
          matcher.second.size());
      for (size_t i = 0; i < indices.size(); ++i) {
        sorted_matcher_second[i] = std::move(matcher.second[indices[i]]);
      }
      matcher.second = std::move(sorted_matcher_second);
    }
  }

  // 推荐车道选取
  for (auto& matcher : matchers) {
    if (recommend_path_map.count(matcher.ld_map_group_id) == 0) {
      continue;
    }

    int cnt = matcher_map.at(matcher.bev_id).size();
    if (cnt == 1) {
      auto bev_lane = bev_map_preprocessor_->GetLane(matcher.bev_id);
      if (bev_lane == nullptr) {
        continue;
      }
      auto& matched_paths = recommend_path_map.at(matcher.ld_map_group_id);
      auto end_id = matched_paths.front()->end_ele_id;
      auto iter_tmp =
          std::find(matcher.map_id.begin(), matcher.map_id.end(), end_id);
      if (iter_tmp < matcher.map_id.end() - 1) {
        continue;
      }

      bool keep_lane = false;
      double length = 0.0;
      std::vector<uint64_t> keep_id;
      if (bev_lane->next_lane_ids.empty()) {
        keep_lane = true;
        length = matched_paths.front()->lanes.back().second;
        keep_id.push_back(matcher.bev_id);
      } else {
        for (auto id : bev_lane->next_lane_ids) {
          if (matcher_map.count(id) == 0) {
            continue;
          }
          auto last_ld_group_id = matcher_map.at(id).back()->ld_map_group_id;
          auto last_ele_id = matcher_map.at(id).back()->map_id.back();
          for (auto& path : matched_paths) {
            for (auto iter = path->lanes.begin(); iter != path->lanes.end();
                 ++iter) {
              if (iter->first == last_ld_group_id) {
                if (iter < path->lanes.end() - 1) {
                  keep_lane = true;
                  break;
                } else {
                  auto& mapped_ld_lanes = matcher_map.at(id).back();
                  auto iter_tmp =
                      std::find(mapped_ld_lanes->map_id.begin(),
                                mapped_ld_lanes->map_id.end(), last_ele_id);
                  if (iter_tmp < mapped_ld_lanes->map_id.end() - 1) {
                    continue;
                  } else {
                    keep_lane = true;
                    length = path->lanes.back().second;
                    keep_id.push_back(id);
                    break;
                  }
                }
              }
            }
          }
        }

        if (keep_lane) {
          for (auto id : keep_id) {
            std::pair<int, int> info;
            info.first =
                static_cast<int>(matched_paths.front()->lanes.back().second);
            info.second = -1;
            recommend_driving_lanes_.push_back(
                std::pair<uint64_t, std::pair<int, int>>(id,
  std::move(info)));
          }
        }
      }
    } else if (cnt > 1) {
    }

    // std::unordered_map<uint64_t,
    //                    std::vector<cem::fusion::MatchMaker::LaneMatchPair>>
    //     ordered_matchers;
    // for (auto& matcher : matchers) {
    //   if (ordered_matchers.count(matcher.bev_id) == 0) {
    //     ordered_matchers[matcher.bev_id].push_back(matcher);
    //     auto& cur_matcher = ordered_matchers.at(matcher.bev_id).back();
    //     for (auto ld_map_id:cur_matcher.map_id) {
    //       if (ld)
    //     }
    //   }
    // }

    double min_dist = std::numeric_limits<double>::max();
    for (auto& path : recommend_path) {
      if (min_dist > path.lanes.back().second) {
        min_dist = path.lanes.back().second;
      }
    }
    for (auto& path : recommend_path) {
      auto found_iter = std::find_if(
          matchers.begin(), matchers.end(),
          [path](const MatchMaker::LaneMatchPair& matcher) {
            return matcher.ld_map_group_id == path.lanes.front().first;
          });
      if (found_iter == matchers.end()) {
        continue;
      }
      if (path.lanes.back().second - min_dist < 1.0) {
        auto info = std::pair<int, int>(static_cast<int>(min_dist), 0);
        recommend_driving_lanes_.push_back(
            std::pair<uint64_t, std::pair<int, int>>(found_iter->bev_id,
                                                     std::move(info)));
      } else {
        auto info = std::pair<int, int>(min_dist, -1);
        recommend_driving_lanes_.push_back(
            std::pair<uint64_t, std::pair<int, int>>(found_iter->bev_id,
                                                     std::move(info)));
      }
    }
    */
  PrintRecommendDrivingLanes();
}

void FusionManager::PrintRecommendDrivingLanes() {
  debug_info_.clear();
  for (const auto& lane : recommend_driving_lanes_) {
    std::string result = "Lane: " + std::to_string(lane.first) +
                         " -> (Inner: " + std::to_string(lane.second.first) +
                         ", Outer: " + std::to_string(lane.second.second) +
                         "); ";
    debug_info_ += result;
    // AINFO << result;
  }
};



void FusionManager::SetCrossRoadInfo(bool cross_road_status,
                                     double cross_road_distance,bool is_turn_right) {
  bev_map_preprocessor_->SetCrossParam(cross_road_status,cross_road_distance,is_turn_right);
}
void FusionManager::SetAdc2Junction(double adc_to_junction) {
  bev_map_preprocessor_->SetAdc2Junction(adc_to_junction);
}

void FusionManager::EstimateEgoLaneErrorBetweenLdMapAndBevMap() {
  auto& ld_ego_lane = ld_map_preprocessor_->GetEgoLaneGroups();
  auto& ld_ego_lane_points = ld_map_preprocessor_->GetEgoLanePoints();
  if (ld_ego_lane.empty() || ld_ego_lane_points.empty()) {
    debug_info_ += "{Ego_lane_match_info: Cannot found ld ego lane}";
    return;
  }
  auto bev_map = bev_map_preprocessor_->GetData();
  if (bev_map == nullptr) {
    debug_info_ += "{Ego_lane_match_info: Cannot get bev frame}";
    return;
  }

  ego_lane_compare_length_ = EstimateEgoLaneCompareDistance();
  Eigen::Vector4d ld_ego_lane_coeffs;
  if (!FitPolynomial(ld_ego_lane_points, ld_ego_lane_coeffs)) {
    debug_info_ += "{Ego_lane_match_info: Cannot fit ld ego lane}";
    return;
  }
  ld_ego_lane_c0_ = ld_ego_lane_coeffs(0);

  for (auto& lane : bev_map->lane_infos) {
    bool is_valid = false;
    if (lane.line_points.size() < 2) {
      continue;
    }
    auto iter = lane.line_points.begin() + 1;
    for (; iter != lane.line_points.end(); ++iter) {
      if (((iter - 1)->x < 0) && (iter->x >= 0)) {
        is_valid = true;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    double m = (iter->y - (iter - 1)->y) / (iter->x - (iter - 1)->x);
    double y = m * (-(iter - 1)->x) + (iter - 1)->y;

    int point_cnt = 1;
    cem::message::common::Point2DF last_pt(0, y);
    double max_error = std::abs(y - ld_ego_lane_coeffs(0));
    double error_sum = max_error;
    double length = 0;
    for (; iter != lane.line_points.end(); ++iter) {
      auto& pt = *iter;
      if (pt.x > ld_ego_lane_points.back().x()) {
        break;
      }
      auto error_pt =
          std::abs(ld_ego_lane_coeffs(0) + ld_ego_lane_coeffs(1) * pt.x +
                   ld_ego_lane_coeffs(2) * pt.x * pt.x +
                   ld_ego_lane_coeffs(3) * pt.x * pt.x * pt.x - pt.y);
      error_sum += error_pt;
      point_cnt++;
      length += std::sqrt((pt.x - last_pt.x) * (pt.x - last_pt.x) +
                          (pt.y - last_pt.y) * (pt.y - last_pt.y));
      if (length > ego_lane_compare_length_) {
        break;
      }
      if (length < 20) {
        if (error_pt > max_error) {
          max_error = error_pt;
        }
      }
      last_pt = pt;
    }
    if (point_cnt < 3 || length < 15 || max_error > 1) {
      continue;
    }
    double error_avg = error_sum / point_cnt;
    if (error_avg < ego_lane_lat_error_between_ld_and_bev_) {
      ego_lane_lat_error_between_ld_and_bev_ = error_avg;
      estimated_bev_ego_lane_id_ = lane.id;
      bev_ego_lane_c0_ = y;
    }
  }
  debug_info_ += "{Ego_lane_match_info: [" +
                 ld_map_preprocessor_->GetEgoLaneElementsDebugInfo() + "] ";
  debug_info_ +=
      "[Bev Ego lane id: " + std::to_string(estimated_bev_ego_lane_id_) + "] ";
  debug_info_ += "[match_error: " +
                 std::to_string(ego_lane_lat_error_between_ld_and_bev_) + "]} ";
}

bool FusionManager::FitPolynomial(const std::vector<Eigen::Vector2d>& line,
                                  Eigen::Vector4d& coefficients_out) {
  coefficients_out.setZero();

  size_t num_points = line.size();

  double length = 0;
  for (size_t i = 1; i < line.size(); ++i) {
    length += (line.at(i) - line.at(i - 1)).norm();
    if (length > ego_lane_compare_length_) {
      num_points = i;
      break;
    }
  }

  if (num_points < 2) {
    return false;
  }

  // 根据数据点数量确定多项式次数
  int degree = std::min(3, static_cast<int>(num_points - 1));

  // 构造设计矩阵 A
  Eigen::MatrixXd A(num_points, degree + 1);
  for (size_t i = 0; i < num_points; ++i) {
    double x = line[i].x();
    for (int j = 0; j <= degree; ++j) {
      A(i, j) = std::pow(x, j);
    }
  }

  // 构造观测值向量 y
  Eigen::VectorXd y(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    y(i) = line[i].y();
  }

  // 求解最小二乘问题
  Eigen::VectorXd coefficients =
      (A.transpose() * A).ldlt().solve(A.transpose() * y);
  for (size_t i = 0; i < coefficients.size(); ++i) {
    coefficients_out(i) = coefficients(i);
  }
  return true;
}

double FusionManager::EstimateEgoLaneCompareDistance() {
  double ego_lane_compare_length = 50.0;
  auto pose = bev_map_preprocessor_->GetDataPose();
  if (pose == nullptr) {
    // do noting
  } else {
    double vx = 0.0, vy = 0.0;
    if (pose->velocity.size() == 3) {
      vx = pose->velocity.at(0);
      vy = pose->velocity.at(1);
      auto speed = std::sqrt(vx * vx + vy * vy) * 3.6;
      ego_lane_compare_length = speed;
    }
  }
  if (ego_lane_compare_length > 50.0) {
    ego_lane_compare_length = 50.0;
  }
  if (ego_lane_compare_length < 15.0) {
    ego_lane_compare_length = 15.0;
  }
  return ego_lane_compare_length;
}

}  // namespace fusion
}  // namespace cem
