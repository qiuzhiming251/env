#include "crossroad_construction.h"

#include "base/sensor_data_manager.h"
#include "lib/common/fitter/two_segment_bezier_interpolator.h"
#define USE_CLUSTER 0
// #define PPM_UPDATE 1

/// construct virtual lines to go through the cross section
namespace cem {
namespace fusion {

CrossRoadConstruction::CrossRoadConstruction() {}

CrossRoadConstruction::~CrossRoadConstruction() {}

std::shared_ptr<CrossDataManager> CreateCrossDataManager() {
  std::shared_ptr<CrossDataManager> instance(new CrossDataManager());
  instance->Init();
  instance->cross_data_manager_ptr = instance;
  return instance;
}
void CrossRoadConstruction::Init() {
  data_manager_ = CreateCrossDataManager();
  cross_toplane_processor_ = std::make_shared<CrossToplaneProcessor>();
  cross_toplane_processor_->Init(data_manager_);
  virtual_egoroad_ = std::make_shared<VirtualEgoRoadProcessor>();
  virtual_egoroad_->Init(data_manager_);
  road_connector_ = std::make_shared<RoadConnector>();
  road_connector_->Init(data_manager_);
}

bool CrossRoadConstruction::DataUpdate(const BevMapInfoPtr &bev_map, const Eigen::Isometry3d &Twb, 
                                        RoutingMapPtr routing_map_ptr,BevMapInfoConstPtr &raw_bev_map,
                                        const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr,
                                        bool on_highway) {
  // 获取bev_map & bev_map_raw
  if (!bev_map) {
    FLOG_CROSS << "----bev_map nullptr----- ";
    return false;
  }

  CAN1Ptr canoutptr = nullptr;
  SensorDataManager::Instance()->GetLatestSensorFrame(canoutptr);
  if(canoutptr){
    if(!noa2icc){
      noa2icc = (last_dnp_status_ == 6 && canoutptr->DNP_Stats_S ==  4)?true:false;
    }
    XLOG << "===  noa2icc " << noa2icc << " canoutptr->DNP_Stats_S " << canoutptr->DNP_Stats_S;
    last_dnp_status_ = static_cast<int>(canoutptr->DNP_Stats_S);
  }
  // 获取定位信息
  double measure_timestamp = bev_map->header.timestamp;
  SensorDataManager::Instance()->GetLatestSensorFrame(measure_timestamp, 0.05, currect_loc_);
  // 获取routing_map

  SensorDataManager::Instance()->GetLatestSensorFrame(measure_timestamp, 0.05,routing_map_raw_);

  if (!raw_bev_map || !currect_loc_ || !routing_map_raw_) {
    return false;
  }

  // 更新数据
  bev_map_raw_ = raw_bev_map;
  bev_map_     = bev_map;

  // 取出与bev相匹配的LD的egolaneid
  if (geometry_match_info_ != nullptr && geometry_match_info_->GetEgoLanes() != std::nullopt &&
      !geometry_match_info_->GetEgoLanes()->ld_lane_.empty()) {
    auto id = geometry_match_info_->GetEgoLanes()->ld_lane_.front().lane_id;
    XLOG << " LDEgoLaneId: " << id;
    data_manager_->SetLDEgoLaneId(id);
  }

  data_manager_->DataUpdate(routing_map_raw_, routing_map_ptr, bev_map_raw_, bev_map, currect_loc_, Twb, junctions_ptr, on_highway);
  data_manager_->UpdateCrossState(cross_state_);

  if (!data_manager_->first_junction_lane_is_virtual() && cross_state_ == CrossState::INIT) {//必须有图
    // return false;
  }


  // AINFO << "*******IsRightTurnOnly******"; 
  // AINFO << "sequence_number: " << bev_map->header.cycle_counter;
  // AINFO << "before is_right_turn_only_: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_;
  IsRightTurnOnly(junctions_ptr);
  // AINFO << "after is_right_turn_only_: " << is_right_turn_only_ << "; next junction id: " << next_junction_id_;
  // AINFO << "****************************";

  FLOG_CROSS << " is_right_turn_only: " << is_right_turn_only_;
  if(is_right_turn_only_){
    return false;
  }

  FLOG_CROSS << " update end ";
  return true;
}

void CrossRoadConstruction::Reset() {
  cross_state_ = CrossState::INIT;
  end_count_ = 0;
  cross_toplane_processor_->Reset();
  virtual_egoroad_->ReSet();
  road_connector_->Reset();
  data_manager_->Reset();
  is_cross_road_out_ = false;
  last_dnp_status_ = -1;
  noa2icc =false;
}

void CrossRoadConstruction::Process(BevMapInfoPtr &bev_map, const Eigen::Isometry3d &Twb, BevMapInfoConstPtr &raw_bev_map,
    const SdJunctionInfoCityPtr &junctions_ptr,RoutingMapPtr routing_map_ptr,bool on_highway) {
  XLOG << "********cross_road_start******** "
             << ", cross_state: " << EnumToString(cross_state_);
  if (!DataUpdate(bev_map, Twb, routing_map_ptr, raw_bev_map, junctions_ptr, on_highway)) {
    Reset();
    SetCrossroadConstructionDebugInfo(bev_map);
    XLOG << "----map nullptr----";
    return;
  }

  switch (cross_state_) {
    case CrossState::INIT:
    InitAction();
    XLOG <<  "@@@@CrossState::INIT";
    break;
    // case CrossState::PREDICTION:
    // XLOG <<"@@@@CrossState::PREDICTION";
    // PredictionAction();
    // case CrossState::CONNECTION:
    // AINFO<<"@@@@CrossState::CONNECTION";
    // ConnectionAction();
    // break;
    case CrossState::END:
    XLOG <<  "@@@@CrossState::END";
      EndAction();
      break;
    default:
      XLOG << "error ";
      break;
  }
  ///////////////////////////////////ICC场景截断 同时后面不在进行连接动作//////////////////////////////////////////////////////
  { 
      // icc下也退出

      auto hd_map_suppression_reason = static_cast<int>(env_status_->hd_map_suppression_reason());
      auto distance2next_junction = env_status_->distance_to_next_junction();
      bool suppression = (hd_map_suppression_reason == 2 || hd_map_suppression_reason == 3 || 
                  hd_map_suppression_reason == 5 || hd_map_suppression_reason == 6 ||
                  hd_map_suppression_reason == 15 || hd_map_suppression_reason == 21 ||
                  hd_map_suppression_reason == 31 || hd_map_suppression_reason == 32);
                  if(routing_map_ptr  ) XLOG << "/////////////////// TIME " << routing_map_ptr->header.timestamp; 
      XLOG << "suppression " << suppression << "  hd_map_suppression_reason " << hd_map_suppression_reason << " noa2icc " <<  noa2icc << " last_dnp_status_ "  <<  last_dnp_status_ << " distance2next_junction " << distance2next_junction;
        
        if(suppression &&  noa2icc &&  last_dnp_status_ ==4 ){//小路口前后大路口 切图失败+其他远导致降级也会触发这个条件-》后果小路口过不去 但是概率不大  CANOAC2-98784
          PredictionAction();
          CutVirtualLane();
          cross_state_ = CrossState::PREDICTION;//保持预测 防止跳到连接状态 到PREDICTION函数后不会在往下跳转
          OutPutVirtualLane();
          XLOG << "分支 1 ";
        // }else if(last_dnp_status_ !=6 && last_dnp_status_ != -1 ){//一直是ICC
        }else if(last_dnp_status_ == 4 ){//一直是ICC noamap信号不会发出
          // ReInitCrossInfo();
          // AINFO << "!!!!!!!!!!!!!!!CrossStatePreToEnd";
          // cross_state_ = CrossState::END;
          XLOG << "分支 一直ICC";
          // cross_state_ = CrossState::END;//不输出

        }else if(suppression){//增加一个条件 切图失败直接退出
          // cross_state_ = CrossState::END;
          XLOG << "切图失败 退路口逻辑";//不输出
          XLOG << "分支 大路口切图失败";//不输出
        }else{//剩下是小路口 CANOAC2-98784
          switch (cross_state_) {
                case CrossState::PREDICTION:
                XLOG <<"@@@@CrossState::PREDICTION";
                PredictionAction();//更新虚拟线
                //判断是否跳转到接线状态
                if (cross_toplane_processor_->FindClosetLane(virtual_egoroad_->virtual_lane()) &&
                    road_connector_->GenerateConnectLane(virtual_egoroad_->virtual_lane())) {
                  road_connector_->SetDrGeoLane(virtual_egoroad_->GetDrGeoLane());
                  XLOG << "PredictionAction to CONNECTION first!!! ";
                  cross_state_ = CrossState::CONNECTION;
                } else {
                  XLOG << "FindClosetLane faild!!!";
                }
                //输出虚拟线
                OutPutVirtualLane();
                break;
                case CrossState::CONNECTION:
                XLOG <<"@@@@CrossState::CONNECTION";
                ConnectionAction();
                break;
                default:
                  XLOG << "error ";
                  break;
              }
          XLOG << "分支 小路口/或者者大路口切图成功/地图无路口  " ;

        }
  }
  SetCrossroadConstructionDebugInfo(bev_map);
  FLOG_CROSS << std::fixed << std::setprecision(5) << "----cross road end-----bev_map time: " << bev_map->header.timestamp
             << " ,routing_map_time: " << routing_map_raw_->header.timestamp;
}

bool CrossRoadConstruction::IsCrossRoad(){
  // 路口状态下返回true
  if(cross_state_ == CrossState::PREDICTION || cross_state_ == CrossState::CONNECTION){
    return true;
  }
  return false;
}

double CrossRoadConstruction::CrossRoadDis() {
  // 路口状态下返回0
  if (cross_state_ == CrossState::PREDICTION || cross_state_ == CrossState::CONNECTION) {
    return 0.0;
  }
  return data_manager_->cross_road_dis();
}

void CrossRoadConstruction::IsRightTurnOnly(const SdJunctionInfoCityPtr &junctions_ptr) {
  constexpr double EnterJunctionThreshold = 80.0;  // ego_lane 跳到其next添加拓扑的阈值

  if (junctions_ptr == nullptr || routing_map_raw_ == nullptr) {
    return;
  }

  // Initialization upon switching navigation state.
  if (routing_map_raw_->sd_route.id != route_id_) {
    route_id_ = routing_map_raw_->sd_route.id;
    is_right_turn_only_ = false;
    next_junction_id_ = 0;
    return;
  }

  // Judgment before entering a right turn dedicated lane scenario.
  bool first_unreached = false;
  for (size_t i = 0; i < junctions_ptr->size(); i++) {
    // AINFO << junctions_ptr->at(i).junction_id << "  is_dedicated_right_turn_lane:  " << junctions_ptr->at(i).is_dedicated_right_turn_lane
    //       << "  junction_state:  " << (int)junctions_ptr->at(i).junction_state_city
    //       << "  junction_type:   " << (int)junctions_ptr->at(i).junction_type_city
    //       << "  junction_action:   " << (int)junctions_ptr->at(i).junction_action 
    //       << "  junction offset:   " << junctions_ptr->at(i).offset;
    if (is_right_turn_only_) {
      break;
    }

    if (junctions_ptr->at(i).offset > EnterJunctionThreshold) {
      first_unreached = true;
    }

    if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
      if (junctions_ptr->at(i).is_dedicated_right_turn_lane && !is_right_turn_only_ &&
          junctions_ptr->at(i).offset < EnterJunctionThreshold && junctions_ptr->at(i+1).offset > EnterJunctionThreshold &&
          junctions_ptr->at(i).junction_action == navigation::JunctionAction::TurnRight) {
        is_right_turn_only_ = true;
        next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
      }
    }

    if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
      if (!is_right_turn_only_ &&
          junctions_ptr->at(i).offset < EnterJunctionThreshold && junctions_ptr->at(i+1).offset > EnterJunctionThreshold &&
          junctions_ptr->at(i).junction_type_city == navigation::JunctionTypeCity::RING) {
        is_right_turn_only_ = true;
        next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
        current_junction_type_ = junctions_ptr->at(i).junction_type_city;
      }
    }

    if (first_unreached) {
      break;
    }
  }

  // Judgment of exiting a right turn dedicated lane scenario
  for (size_t i = 0; i < junctions_ptr->size(); i++) {
    auto junction = junctions_ptr->at(i);
    if (!is_right_turn_only_) {
      break;
    }

    if (current_junction_type_ == navigation::JunctionTypeCity::RING) {
      if (junction.junction_id == next_junction_id_ && junction.offset < 0 && junction.junction_type_city != navigation::JunctionTypeCity::RING) {
        current_junction_type_ = junction.junction_type_city;
        is_right_turn_only_ = false;
        break;
      }
      else if (junction.junction_id == next_junction_id_ && junction.offset < 0){
        if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
          next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
        }
        break;
      }
    }
    else 
    {
      if (junction.junction_id == next_junction_id_ && junction.offset < 0 && junction.junction_type_city == navigation::JunctionTypeCity::RoadMerge) {
        is_right_turn_only_ = false;
        break;
      }
      else if (junction.junction_id == next_junction_id_ && junction.offset < 0){
        if (junctions_ptr->size() > 1 && i < junctions_ptr->size() - 1) {
          next_junction_id_ = junctions_ptr->at(i + 1).junction_id;
        }
        break;
      }
    }
  }
}

void CrossRoadConstruction::InitAction() {
  // 路口判断
  XLOG << "Crossdecision "  <<  Crossdecision();
  if (Crossdecision()) {
    XLOG << " Not cross road!!!";
    Reset();
    return;
  }
  is_cross_road_out_ = true;

  // 对路口后的车道线进行预处理，主要是先根据距离等条件把路口后的车道线筛选出来
  // 要区分直行和左右转车道线的筛选
  cross_toplane_processor_->Preprocessing();

  // 第一次生成预推轨迹成功后，然后跳转到PREDICTION状态，后面失败后也能使用预推轨迹
  XLOG <<  "virtual_egoroad_->GetVirtualLanes() " << virtual_egoroad_->GetVirtualLanes();
  if (virtual_egoroad_->GetVirtualLanes()) {
    XLOG << " init to PREDICTION";
    cross_state_ = CrossState::PREDICTION;
    // OutPutVirtualLane();
  }
  if(data_manager_){
    data_manager_->Init();//xxf init
  }
}

bool CrossRoadConstruction::IsLaneDeath(const BevLaneInfo &lane, float end_offset) {
  if (lane.line_points.size() < 2) {
    return false;
  }
  const auto &lane_markers = bev_map_raw_->lanemarkers;
  if (lane.line_points.front().x < -10.0 && lane.line_points.back().x < end_offset && lane.line_points.back().x > 1.0 &&
      lane.next_lane_ids.empty()) {
    auto left_it  = std::find_if(lane_markers.begin(), lane_markers.end(),
                                [&lane](const BevLaneMarker &p) { return p.id == lane.left_lane_marker_id; });
    auto right_it = std::find_if(lane_markers.begin(), lane_markers.end(),
                                 [&lane](const BevLaneMarker &p) { return p.id == lane.right_lane_marker_id; });
    if (left_it != lane_markers.end() && right_it != lane_markers.end()) {
      if (left_it->line_points.size() >= 2 && right_it->line_points.size() >= 2) {
        if (left_it->line_points.front().x < -10.0 && left_it->line_points.back().x < end_offset && left_it->line_points.back().x > 0.0 &&
            right_it->line_points.front().x < -10.0 && right_it->line_points.back().x < end_offset &&
            right_it->line_points.back().x > 0.0) {
          return true;
        }
      }
    }
  }
  return false;
}

bool CrossRoadConstruction::Crossdecision() {
  // 查找停止线
  bool has_bev_stop_lane = data_manager_->has_bev_stop_lane();
  bool has_bev_junction = data_manager_->has_bev_junction();
  bool has_bev_cross_walk = data_manager_->has_bev_cross_walk();
  // 判断车道是否消失
  bool  ego_lane_death{false};
  float ego_lane_end_offset{0.0};
  for (const auto &lane : bev_map_->lane_infos) {
    // 0为当前车道
    // |5|3|1|0|2|4|6|
    if (lane.position == 0) {
      //ego_lane_death      = IsLaneDeath(lane, 25.0f);
      if (lane.line_points.empty()) {
        break;
      }
      ego_lane_end_offset = (TransformPoint(lane.line_points.back(), data_manager_->Tbw())).x;
      if (ego_lane_end_offset < 80 && ego_lane_end_offset > -0.1) {
        ego_lane_death = true;
      }
      break;
    }
  }

  // 判断路口juction
  bool has_junction{false};

  const auto &navi_start  = routing_map_raw_->sd_route.navi_start;
  const auto &mpp_section = routing_map_raw_->sd_route.mpp_sections;

  const auto it_navi_seciton =
      std::find_if(mpp_section.begin(), mpp_section.end(),
                   [&navi_start](const cem::message::env_model::SDSectionInfo &p) { return p.id == navi_start.section_id; });
  if (it_navi_seciton != mpp_section.end()) {
    if (it_navi_seciton->has_junction) {
      has_junction = true;
    } else {
      double navi_seciton_length = it_navi_seciton->length - navi_start.s_offset;
      for (auto it = it_navi_seciton + 1; it != mpp_section.end(); it++) {
        // FLOG_CROSS << "seciton_id: " << it->id << " ,has_junction: " << it->has_junction << " ,navi_seciton_length: " << navi_seciton_length;
        navi_seciton_length += it->length;
        if (navi_seciton_length > 120.0) {
          break;
        }
        if (it->has_junction) {
          has_junction = true;
          break;
        }
      }
    }
  } else {
    FLOG_CROSS << "has no navi_seciton!!!";
  }
  //
  bool has_virtual = data_manager_->HasVirtualLane();
  // 有停止线或者路口面后需要添加一个虚拟车道  has bev_ego_lane
  // if (data_manager_->has_ego_lane()) {
  //   FLOG_CROSS << " ,line_points.empty: " << data_manager_->EgoLane()->line_points.empty() << " , ego_lane_death: " << ego_lane_death
  //              << " , next_lane_ids.empty: " << data_manager_->EgoLane()->next_lane_ids.empty();
  //   if (!data_manager_->EgoLane()->line_points.empty()) {
  //     FLOG_CROSS << " ,line_points.back().x: " << data_manager_->EgoLane()->line_points.back().x;
  //   }
  // }
  bool has_bev_ego_lane{false};
  if (data_manager_->has_ego_lane() && !data_manager_->EgoLane()->line_points.empty() && ego_lane_death &&
      data_manager_->EgoLane()->next_lane_ids.empty() && data_manager_->EgoLane()->line_points.back().x < 80.0) {
    has_bev_ego_lane = true;
  }
  has_junction = true;
  bool is_cross_flag = ((has_bev_stop_lane || has_bev_junction || has_bev_cross_walk) &&
                        (has_junction || data_manager_->GetSdSections().has_intersection_zone)) ||
                       data_manager_->is_T_cross_road_flag();
  // 由于T型路口进入的路口模式，默认有egolane
  if(data_manager_->is_T_cross_road_flag()){
    has_bev_ego_lane = true;
  }
  XLOG << "has_bev_ego_lane: " << has_bev_ego_lane << " ,has_stop_lane: " << has_bev_stop_lane
  << " , has_bev_cross_walk: " << has_bev_cross_walk << " , has_bev_junction: " << has_bev_junction
  << " ,has_sd_junction: " << has_junction << " ,has_virtual: " << has_virtual << " ,is_cross_flag: " << is_cross_flag
  << " ,ego_lane_death: " << ego_lane_death << "is_enable_mnoa " <<  is_enable_mnoa;
  // mnoa下路口进入条件不一样
  is_cross_road_out_ = is_enable_mnoa ? (has_virtual || is_cross_flag) : is_cross_flag;
  XLOG << "##########################################" ;
  XLOG << "is_cross_flag  "  << is_cross_flag;
  XLOG << " has_virtual " << has_virtual ;
  XLOG << "is_cross_road_out_  "  << is_cross_road_out_;
  XLOG << "has_bev_ego_lane  "  << has_bev_ego_lane;
  return !(has_bev_ego_lane && is_cross_flag);
}

void CrossRoadConstruction::GenerateCrossVirtualLane() {
  cem::message::sensor::BevLaneInfo cross_lane;
  cross_lane.id = kConnectLaneId;
  // cross_lane.id = data_manager_->GetNewLandId();
  cross_lane.previous_lane_ids.emplace_back(data_manager_->EgoLane()->id);
  cross_lane.is_virtual  = true;
  cross_lane.navi_action = data_manager_->EgoLane()->navi_action;
  bev_map_->lane_infos.emplace_back(cross_lane);
  for (auto &lane : bev_map_->lane_infos) {
    if (lane.position == 0) {
      lane.next_lane_ids.emplace_back(cross_lane.id);
      break;
    }
  }
}

void CrossRoadConstruction::PredictionAction() {
  ///////////////////////////直接跳转///////////////////////////////////////
#if USE_CLUSTER
  auto topLanes = data_manager_->GetTopLanes();
  if (topLanes.size() > 0) {
    XLOG << "USE_CLUSTER  !!!";
    cross_state_ = CrossState::CONNECTION;
  }else{
    XLOG << "FindClosetLane faild!!!";
  }
  //////////////////////////////////////////////////////////////////
#else
  //AINFO << "PredictionAction----- ";
  cross_toplane_processor_->Preprocessing();
  if (CrossStatePreToEnd()) {
    ReInitCrossInfo();
    XLOG << "!!!!!!!!!!!!!!!CrossStatePreToEnd";
    cross_state_ = CrossState::END;
    return;
  }
  XLOG << "@@@@ PredictionAction   "   << cross_toplane_processor_->FindClosetLane(virtual_egoroad_->virtual_lane()) << " " <<  road_connector_->GenerateConnectLane(virtual_egoroad_->virtual_lane()) ;
  virtual_egoroad_->UpdateVirtualLanes();
  // 有路口后车道，并且生成连接线成功后进入CONNECTION状态
  // if (0) {
  if (cross_toplane_processor_->FindClosetLane(virtual_egoroad_->virtual_lane()) &&
      road_connector_->GenerateConnectLane(virtual_egoroad_->virtual_lane())) {
    road_connector_->SetDrGeoLane(virtual_egoroad_->GetDrGeoLane());
    FLOG_CROSS << "PredictionAction to CONNECTION first!!! ";
    cross_state_ = CrossState::CONNECTION;
  } else {
    FLOG_CROSS << "FindClosetLane faild!!!";
  }
#endif

// 将生成的virtual lane的放到bev map中
// OutPutVirtualLane();
}

void CrossRoadConstruction::CutVirtualLane() {
  XLOG << "CutVirtualLane " ;
  //TO DO virtual_egoroad_的点 virtual_egoroad_->virtual_lane().line_points只保留从起点开始的1m范围内 
  auto len = virtual_egoroad_->virtual_lane().line_points.size(); 
  if( len < 2)
  {
    return;
  }
  auto & pts = virtual_egoroad_->virtual_lane().line_points;
  auto start_pt = pts[0];
  double offset = 0.f;
  int idx = 1 ;
  for( ;idx < len; ++idx){
    offset += std::sqrt(pts[idx].x*pts[idx-1].x + pts[idx].y*pts[idx-1].y);
    if(offset > 1.0){
      break;
    }
  }
  pts.resize(idx + 1);
  return;
}

void CrossRoadConstruction::OutPutVirtualLane() {
  virtual_egoroad_->virtual_lane().is_junction_lane = true;
  virtual_egoroad_->virtual_lane().id = kPreLaneId;
  // virtual_egoroad_->virtual_lane().id               = data_manager_->GetNewLandId();
  if (data_manager_->is_egolane_disappear()) {
    virtual_egoroad_->virtual_lane().previous_lane_ids.clear();
  }
  if (!virtual_egoroad_->virtual_lane().previous_lane_ids.empty()) {
    auto id = virtual_egoroad_->virtual_lane().previous_lane_ids.front();
    FLOG_CROSS << " virtual_lane_previous_lane_id: " << id;
    bool has_previous_lane{false};
    for (auto &lane : bev_map_->lane_infos) {
      if (lane.id == id && !lane.line_points.empty()) {
        has_previous_lane = true;
        lane.next_lane_ids.emplace_back(virtual_egoroad_->virtual_lane().id);
        break;
      }
    }
    if (!has_previous_lane) {
      virtual_egoroad_->virtual_lane().previous_lane_ids.clear();
    }
  }

  bev_map_->lane_infos.emplace_back(virtual_egoroad_->virtual_lane());
  // AINFO << "##### virtual_lane emplace";
  if (bev_map_->lane_infos.back().navi_action == BevAction::UNKNOWN) {
    bev_map_->lane_infos.back().navi_action = BevAction::STRAIGHT;
  }

  bev_map_->lanemarkers_num += 2;
  bev_map_->lanemarkers.emplace_back(virtual_egoroad_->virtual_left_lanemarker());
  bev_map_->lanemarkers.emplace_back(virtual_egoroad_->virtual_right_lanemarker());
  FLOG_CROSS << "PredictionAction end virtual_lane: " << virtual_egoroad_->virtual_lane().is_virtual
             << " navi_action:" << int(virtual_egoroad_->virtual_lane().navi_action)
             << " ,point_size: " << virtual_egoroad_->virtual_lane().line_points.size();
  // for (const auto &point : virtual_egoroad_->virtual_lane().line_points) {
  //   auto p = TransformPoint(point, data_manager_->Tbw());
  //   FLOG_CROSS << "before virtual_lane_point_x: " << p.x << " y: " << p.y;
  // }
  RemoveVirtualOverLap();
}

void CrossRoadConstruction::UpdateEgoLane() {
  
}

void CrossRoadConstruction::ConnectionAction() {
  
  
  ///////////////////////////////////////////////////////////////////////////////
#if USE_CLUSTER
    if(!data_manager_){
      return;
    }
     auto top_lanes = data_manager_->GetTopLanes();
     auto ego_lanes = data_manager_->GetEgoLanes();
     std::vector<byd::common::math::Vec2d> trace;
     if (top_lanes.empty() || ego_lanes.empty()) {//top_lanes实时更新 到距离30才有值
       return;
     }
      std::vector<byd::common::math::Vec2d> newCurve;
      auto itEgoLane = std::find_if(ego_lanes.begin(), ego_lanes.end(),
                            [](const auto &lane) { return lane.is_ego; });
      if (itEgoLane == ego_lanes.end()) {
        return;
      }
  
      TopLane best_lane;
      auto select_success = SelectBestTopLane(top_lanes,ego_lanes,trace,best_lane);
      if(!select_success){
        return ;
      }
      XLOG <<  "select_success " <<  select_success;
      XLOG  << "####best id " << best_lane.lane_id << " " <<best_lane.points.back().x() << " "<<  best_lane.points.back().y();
  #if 0
      auto &ppmImage = data_manager_->ppmImage;
      double y_offset = 1.6f;
      for (const auto& toplane : top_lanes) {
          auto color_index = 0;
          color_index = toplane.is_target ? 1 : 0;
          ppmImage->DrawNumber(static_cast<int>(toplane.lane_id), 4.0f, 48.0f - y_offset, color_index);
          y_offset += 8.0f;
        }
      y_offset = 1.6f;      
      for (const auto& toplane : ego_lanes) {
          auto color_index = 1;
          color_index = toplane.is_ego ? 0 : 1;
          ppmImage->DrawNumber(static_cast<int>(toplane.lane_id), 2.0f, 48.0f - y_offset, color_index);
          y_offset += 8.0f;
      }      
      ppmImage->Save();
      AINFO << "####PPM_UPDATE";
  #endif
      newCurve = cem::fusion::interpolator::GenerateStaticSmoothCurve(best_lane.points, itEgoLane->points);
      if (newCurve.empty()) {
        XLOG << "newCurve is empty, return";
        return;
      }
      bev_map_->lane_infos.emplace_back();
      auto &connect_lane = bev_map_->lane_infos.back();
      for (const auto &point : newCurve) {
        cem::message::common::Point2DF drPoint(point.x(), point.y());
        drPoint = TransformPoint(drPoint, data_manager_->Twb());
        connect_lane.line_points.emplace_back(drPoint);
      }
      // connect_lane.id = data_manager_->GetNewLandId();
      connect_lane.id = kConnectLaneId;
      connect_lane.is_junction_lane = true;
      connect_lane.is_virtual = true;
      connect_lane.previous_lane_ids.emplace_back(itEgoLane->lane_id);
      if(best_lane.is_connect_lane){
        connect_lane.next_lane_ids.emplace_back(best_lane.lane_id);
      }
      road_connector_->SetConnectLane(connect_lane);//存连接线
#else 
///////////////////////////////////////////////////////////////////////////////
  cross_toplane_processor_->Preprocessing();
  if (CrossStateConnctToEnd()) {
    ReInitCrossInfo();
    XLOG << "connect state to end----- ";
    cross_state_ = CrossState::END;
    return;
  }
  bool update_main_lane = road_connector_->UpdateMainLane();
  bool update_closet_lane = cross_toplane_processor_->UpdateClosetLane(road_connector_->connect_lane());
  // road_connector_->connect_lane().id = data_manager_->GetNewLandId();
  road_connector_->connect_lane().id = kConnectLaneId;
  road_connector_->connect_lane().is_junction_lane = true;
  // 失败后使用历史的
  FLOG_CROSS << "update_main_lane: " << update_main_lane  << " ,update_closet_lane: " << update_closet_lane;
  if (!update_main_lane || !update_closet_lane) {
    // 将连接车道放到BEVmap中
    // 将生成的connect lane的放到bev map中
    if (data_manager_->is_egolane_disappear()) {
      road_connector_->connect_lane().previous_lane_ids.clear();
    }
    bev_map_->lane_infos.emplace_back(road_connector_->connect_lane());
    if (!road_connector_->connect_lane().previous_lane_ids.empty()) {
      auto id = road_connector_->connect_lane().previous_lane_ids.front();
      for (auto &lane : bev_map_->lane_infos) {
        if (lane.id == id) {
          lane.next_lane_ids.emplace_back(road_connector_->connect_lane().id);
        }
      }
    }
    if (!road_connector_->connect_lane().next_lane_ids.empty()) {
      auto id = road_connector_->connect_lane().next_lane_ids.front();
      FLOG_CROSS << " connect_lane().next_lane_id: " << id;
      for(auto& lane:bev_map_->lane_infos){
        if(lane.id == id){
          lane.previous_lane_ids.emplace_back(road_connector_->connect_lane().id);
        }
      }
    }
    bev_map_->lanemarkers_num += 2;
    bev_map_->lanemarkers.emplace_back(road_connector_->connect_left_lanemarker());
    bev_map_->lanemarkers.emplace_back(road_connector_->connect_right_lanemarker());
    FLOG_CROSS << "connect_lane using history----- is virtual: " << road_connector_->connect_lane().is_virtual
               << " navi_action:" << int(road_connector_->connect_lane().navi_action) << " ,id: " << road_connector_->connect_lane().id;
    RemoveVirtualOverLap();
    return;
  }
  bool update_connect_lane = road_connector_->UpdateConnectLane();
  if (!update_connect_lane) {
    // ReInitCrossInfo();
    // cross_state_ = CrossState::END;
    FLOG_CROSS << "----update_connect_lane faild----- ";
    // return;
  }
  // 将生成的connect lane的放到bev map中
  if (data_manager_->is_egolane_disappear()) {
    road_connector_->connect_lane().previous_lane_ids.clear();
  }
  bev_map_->lane_infos.emplace_back(road_connector_->connect_lane());
  bev_map_->lanemarkers_num += 2;
  bev_map_->lanemarkers.emplace_back(road_connector_->connect_left_lanemarker());
  bev_map_->lanemarkers.emplace_back(road_connector_->connect_right_lanemarker());
  if (!road_connector_->connect_lane().previous_lane_ids.empty()) {
    auto id = road_connector_->connect_lane().previous_lane_ids.front();
    FLOG_CROSS << " connect_lane().next_lane_id: " << id;
    for (auto &lane : bev_map_->lane_infos) {
      if (lane.id == id) {
        lane.next_lane_ids.emplace_back(road_connector_->connect_lane().id);
      }
    }
  }
  if (!road_connector_->connect_lane().next_lane_ids.empty()) {
    auto id = road_connector_->connect_lane().next_lane_ids.front();
    for (auto &lane : bev_map_->lane_infos) {
      if (lane.id == id) {
        lane.previous_lane_ids.emplace_back(road_connector_->connect_lane().id);
      }
    }
  }
  FLOG_CROSS << "----ConnectionAction end----- is_virtual: " << road_connector_->connect_lane().is_virtual
             << " navi_action:" << int(road_connector_->connect_lane().navi_action) << " ,id: " << road_connector_->connect_lane().id;
  RemoveVirtualOverLap();
// 将连接车道放到BEVmap中
#endif
}

void CrossRoadConstruction::EndAction() {
  end_count_++;
  if (end_count_ >= 3) {
    end_count_ = 0;
    cross_state_ = CrossState::INIT;
    last_dnp_status_ = -1;
  }
}

bool CrossRoadConstruction::CrossStateToEnd() {
  // 存在新的ego lane退出
  bool has_cross_road_flag = data_manager_->has_bev_stop_lane() || data_manager_->has_bev_junction() || data_manager_->has_bev_cross_walk();
  if (data_manager_->has_ego_lane()) {
    // 路口中或者laneid变化了，egolane正常,如果是预推就要opening的x小于5或者end_point的x小于5；如果是连接状态，连接线的点小于0
    bool is_crossed_road = false;
    if (cross_state_ == CrossState::PREDICTION) {
      bool sd_valid            = false;
      bool opening_valid       = false;
      bool virtual_lane_passed = false;
      if (data_manager_->GetSdSections().is_valid) {
        sd_valid = data_manager_->GetSdSections().end_point.x() < 1.0;
      }
      if (data_manager_->GetOpening().is_valid) {
        opening_valid = data_manager_->GetOpening().dis < 1.0;
      }
      if (!virtual_egoroad_->virtual_lane().line_points.empty()) {
        bool is_back = true;
        for (const auto &point : virtual_egoroad_->virtual_lane().line_points) {
          const auto &pt = TransformPoint(point, data_manager_->Tbw());
          if (pt.x > -2.0) {
            is_back = false;
            break;
          }
        }
        virtual_lane_passed = is_back;
      } else {
        virtual_lane_passed = true;
      }
      is_crossed_road = sd_valid || opening_valid || virtual_lane_passed;
    } else if (cross_state_ == CrossState::CONNECTION && !road_connector_->connect_lane().line_points.empty()) {
      const auto &pt  = TransformPoint(road_connector_->connect_lane().line_points.back(), data_manager_->Tbw());
      is_crossed_road = pt.x < -2.0;
    }
    bool is_in_cross_has_ego_lane = data_manager_->GetIsInCrossroad() && data_manager_->is_egolane_normal() && is_crossed_road;
    // 路口前如果geolane长度大于停止线20m也退出
    bool is_before_has_ego_lane =
        (has_cross_road_flag && data_manager_->cross_road_dis() + 15.0 < data_manager_->EgoLane()->line_points.back().x &&
         !data_manager_->GetIsInCrossroad()) ||
        data_manager_->EgoLane()->line_points.back().x > 80.0;
    // ego lane有nextlane
    bool has_next_lane = !data_manager_->EgoLane()->next_lane_ids.empty() && cross_state_ == CrossState::PREDICTION;
    FLOG_CROSS << "is_before_has_ego_lane: " << is_before_has_ego_lane << " , is_in_cross_has_ego_lane: " << is_in_cross_has_ego_lane
               << " ,has_cross_road_flag: " << has_cross_road_flag << " ,has_next_lane: " << has_next_lane;
    if (is_in_cross_has_ego_lane || is_before_has_ego_lane || (has_next_lane && !data_manager_->is_T_cross_road_flag())) {
      FLOG_CROSS << "CrossStateToEnd by new egolane id: " << data_manager_->EgoLane()->id;
      if (has_next_lane) {
        FLOG_CROSS << "next_lane_ids id: " << data_manager_->EgoLane()->next_lane_ids.front();
      }
      return true;
    }
  }
  return false;
}

void CrossRoadConstruction::SetCrossroadConstructionDebugInfo(BevMapInfoPtr &GlobalBevMapOutPut)
{
  json crossroad_debug_json;
  json opening_debug_json;
  json lane_groups_json;
  json toplane_json;
  // auto crossroad_debug_info  = INTERNAL_PARAMS.navigation_info_data.crossroad_debug_infos();

  if(!GlobalBevMapOutPut){
    return;
  }

  // opening
  json opening_lane_ids_array               = data_manager_->GetOpening().lane_ids;
  opening_debug_json["lane_size"]           = data_manager_->GetOpening().lane_size;
  opening_debug_json["dis"]                 = data_manager_->GetOpening().dis;
  opening_debug_json["heading"]             = data_manager_->GetOpening().heading;
  opening_debug_json["left_y"]              = data_manager_->GetOpening().left_y;
  opening_debug_json["right_y"]             = data_manager_->GetOpening().right_y;
  opening_debug_json["is_valid"]            = data_manager_->GetOpening().is_valid;
  opening_debug_json["lane_ids"]            = opening_lane_ids_array;
  opening_debug_json["sd_cross_top_point"]  = {{"x", data_manager_->GetOpening().sd_cross_top_point.x()},
                                              {"y", data_manager_->GetOpening().sd_cross_top_point.y()}};

  // lane_group
  json lane_groups_array = json::array();
  json lane_ids_array    = json::array();
  lane_groups_json["group_id"] = "15692120460";
  lane_groups_json["lane_ids"] = lane_ids_array;
  lane_groups_array.push_back(lane_groups_json);

  const auto toplanes = data_manager_->GetTopLanes();
  json toplane_array = json::array();
  for (const auto toplane : toplanes) {
    if(toplane.points.size()==2){
      toplane_json["lane_id"] = toplane.lane_id;
      toplane_json["x1"] =toplane.points[0].x();
      toplane_json["y1"] =toplane.points[0].y();
      toplane_json["x2"] =toplane.points[1].x();
      toplane_json["y2"] =toplane.points[1].y();
      toplane_array.push_back(toplane_json);
    }
  }

  crossroad_debug_json["CrossRoad"]["cross_state_"]          = EnumToString(cross_state_);
  crossroad_debug_json["CrossRoad"]["opening"]               = opening_debug_json;
  crossroad_debug_json["CrossRoad"]["lane_groups"]           = lane_groups_array;
  crossroad_debug_json["CrossRoad"]["toplane_array"]         = toplane_array;
  crossroad_debug_json["CrossRoad"]["is_right_turn_only"]    = is_right_turn_only_;
  crossroad_debug_json["CrossRoad"]["has_bev_stop_lane"]     = data_manager_->has_bev_stop_lane();
  crossroad_debug_json["CrossRoad"]["has_bev_junction"]      = data_manager_->has_bev_junction();
  crossroad_debug_json["CrossRoad"]["has_bev_cross_walk"]    = data_manager_->has_bev_cross_walk();
  crossroad_debug_json["CrossRoad"]["cross_road_dis"]        = data_manager_->cross_road_dis();
  crossroad_debug_json["CrossRoad"]["IsInCrossroad"]         = data_manager_->GetIsInCrossroad();
  crossroad_debug_json["CrossRoad"]["is_topo_connect"]       = data_manager_->is_find_closetlane_bytopo_success();
  crossroad_debug_json["CrossRoad"]["ego_turn_type"]         = EnumToString(data_manager_->turn_type());
  crossroad_debug_json["CrossRoad"]["max_main_left_x"]       = static_cast<int>(data_manager_->GetOccSdEdgeInfo().max_main_left_x);
  crossroad_debug_json["CrossRoad"]["connect_toplane_index"] = cross_toplane_processor_->his_connect_top_lane_index();
  crossroad_debug_json["CrossRoad"]["virtual_line_source"]   = EnumToString(data_manager_->GetVirtualLineSource());
  crossroad_debug_json["CrossRoad"]["first_junction_lane_is_virtual"] = data_manager_->first_junction_lane_is_virtual();
  if (data_manager_->has_ego_lane() && !data_manager_->EgoLane()->line_points.empty()) {
    crossroad_debug_json["CrossRoad"]["line_points.back.x"]   = data_manager_->EgoLane()->line_points.back().x;
    crossroad_debug_json["CrossRoad"]["ego_lane_navi_action"] = EnumToString(data_manager_->EgoLane()->navi_action);
  }

  std::string debug_str = crossroad_debug_json.dump();

  // FLOG_CROSS << "CROSS_ROAD_DEBUG_INFO:" << debug_str;
  GlobalBevMapOutPut->debug_infos += debug_str;
}

bool CrossRoadConstruction::CrossStatePreToEnd(){
  return CrossStateToEnd();
}

bool CrossRoadConstruction::CrossStateConnctToEnd(){
  return CrossStateToEnd();
}

void CrossRoadConstruction::ReInitCrossInfo() { Reset(); }

void CrossRoadConstruction::RemoveVirtualOverLap() {
  auto virtual_lane = std::find_if(bev_map_->lane_infos.begin(), bev_map_->lane_infos.end(),
                                   [](const BevLaneInfo &laneinfo) { return laneinfo.is_virtual == true; });

  if (virtual_lane != bev_map_->lane_infos.end()) {
    if (!virtual_lane->previous_lane_ids.empty()) {
      
      uint64_t pre_lane_id = virtual_lane->previous_lane_ids.at(0);

      auto previous_lane = std::find_if(bev_map_->lane_infos.begin(), bev_map_->lane_infos.end(),
                                        [pre_lane_id](const BevLaneInfo &laneinfo) { return laneinfo.id == pre_lane_id; });
      if (previous_lane != bev_map_->lane_infos.end()) {
        FLOG_CROSS << "!virtual_lane previous_lane != bev_map_->lane_infos.end()";
        if (!previous_lane->line_points.empty() && !virtual_lane->line_points.empty()) {
          // 如果是地图或者经验轨迹的就删除多余的感知车道线，否则删除预推的
          if ((data_manager_->GetVirtualLineSource() == VirtualLineSource::LD || data_manager_->GetVirtualLineSource() == VirtualLineSource::EXP)) {
            FLOG_CROSS << " cut previous_lane_id: " << previous_lane->id;
            auto point      = virtual_lane->line_points.front();
            auto point_body = TransformPoint(point, data_manager_->Tbw());
            int  end_index  = 0;
            for (int i = previous_lane->line_points.size() - 1; i >= 0; i--) {
              if ((TransformPoint(previous_lane->line_points[i], data_manager_->Tbw())).x + 2.0 < point_body.x) {
                end_index = i;
                break;
              }
            }
            // FLOG_CROSS << " cut line_points.size: " << previous_lane->line_points.size() << " ,end_index: " << end_index + 1;
            if (end_index == 0) {
              previous_lane->line_points.clear();
              previous_lane->line_points.emplace_back(
                  TransformPoint(cem::message::common::Point2DF(point_body.x - 0.5, point_body.y), data_manager_->Twb()));
              previous_lane->line_points.emplace_back(
                  TransformPoint(cem::message::common::Point2DF(point_body.x - 1.0, point_body.y), data_manager_->Twb()));
            } else {
              previous_lane->line_points.resize(end_index + 1);
            }
            // FLOG_CROSS << " after cut line_points.size: " << previous_lane->line_points.size();
          } else {
            FLOG_CROSS << " cut virtual_lane";
            auto point      = previous_lane->line_points.back();
            auto point_body = TransformPoint(point, data_manager_->Tbw());
            // FLOG_CROSS << ".........previous lane last point is " << point_body.x << ", y is " << point_body.y;

            for (auto it = virtual_lane->line_points.begin(); it != virtual_lane->line_points.end();) {
              // FLOG_CROSS << "...... previous virtual point x is " << (TransformPoint(*it, data_manager_->Tbw())).x;
              if (virtual_lane->line_points.size() < 3) {
                break;
              }
              if ((TransformPoint(*it, data_manager_->Tbw())).x < (point_body.x + 5)) {
                it = virtual_lane->line_points.erase(it);
              } else {
                it++;
              }
              if (TransformPoint(*it, data_manager_->Tbw()).x > (point_body.x + 6)) {
                break;
              }
            }
          }
        }
        // for (auto it = virtual_lane->line_points.begin(); it != virtual_lane->line_points.end();) {
        //   auto virtual_point = TransformPoint(*it, data_manager_->Tbw());
        //   FLOG_CROSS << "after cut virtual point x is " << virtual_point.x << " ,y: " << virtual_point.y;
        //   it++;
        // }
      }
    }

    if (!virtual_lane->next_lane_ids.empty()) {
      uint64_t next_lane_id = virtual_lane->next_lane_ids.at(0);

      auto next_lane = std::find_if(bev_map_->lane_infos.begin(), bev_map_->lane_infos.end(),
                                    [next_lane_id](const BevLaneInfo &laneinfo) { return laneinfo.id == next_lane_id; });
      if (next_lane != bev_map_->lane_infos.end()) {
        FLOG_CROSS << "virtual_lane next_lane != bev_map_->lane_infos.end() ";
        if (!next_lane->line_points.empty() && !virtual_lane->line_points.empty()) {
          auto point      = next_lane->line_points.front();
          auto point_body = TransformPoint(point, data_manager_->Tbw());
          // FLOG_CROSS <<".........next lane last point is "<<point_body.x<<", y is "<<point_body.y;
          for (auto it = virtual_lane->line_points.begin(); it != virtual_lane->line_points.end();) {
            auto virtual_point = TransformPoint(*it, data_manager_->Tbw());
            // FLOG_CROSS << "next virtual point x is " << virtual_point.x << " ,y: " << virtual_point.y;
            if (virtual_lane->line_points.size() < 2) {
              break;
            }
            if (virtual_point.x > (point_body.x - 2)) {
              it = virtual_lane->line_points.erase(it);
            } else {
              it++;
            }
          }
          if (virtual_lane->line_points.size() < 2) {
            virtual_lane->line_points.clear();
            virtual_lane->line_points.emplace_back(point);
            virtual_lane->line_points.emplace_back(point.x - 0.1, point.y);
          }
          // for (auto it = virtual_lane->line_points.begin(); it != virtual_lane->line_points.end();) {
          //   auto virtual_point = TransformPoint(*it, data_manager_->Tbw());
          //   FLOG_CROSS << "after cut virtual point x is " << virtual_point.x << " ,y: " << virtual_point.y;
          //   it++;
          // }
        }
      }
    }
  }
}

bool CrossRoadConstruction::SelectBestTopLane( std::vector<TopLane>& candiates, const std::vector<TopLane>& ego_lanes,
                                                std::vector<byd::common::math::Vec2d> currentTrace, TopLane& best_lane)
{
  if (candiates.empty() || ego_lanes.empty()) {
    return false;
  }
  //按照导航动作 求ego_lanes对应的车道
  auto bev_action = data_manager_->turn_type();
  bool SelectBestTopLaneStraight = [&]() {
    AINFO <<  "SelectBestTopLaneStraight " ;
    //TODO
    auto it = std::find_if(ego_lanes.begin(), ego_lanes.end(), [](const TopLane& lane) {
      AINFO <<  "#ID ane.is_ego " << lane.lane_id << " " << lane.is_ego;
      return lane.is_ego;
    });
    auto index = std::distance(ego_lanes.begin(), it);
    AINFO <<  "index " << index;
    if (it != candiates.end() && index < candiates.size()) {
      best_lane = candiates.at(index);
      candiates[index].is_target = true;
      return true;
    }else{
      return false;
    }
  }();
  return SelectBestTopLaneStraight;
  
}
}  // namespace fusion
}  // namespace cem