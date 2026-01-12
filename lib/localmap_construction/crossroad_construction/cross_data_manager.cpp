#include "cross_data_manager.h"
#include "hptimer.h"
int  ppm::PPMImage::ppmCount = 0;

namespace cem {
namespace fusion {
CrossDataManager::CrossDataManager() {}
void CrossDataManager::Init() {
  lane_cluster_ = std::make_shared<LaneLineCluster>();
  lane_refiner_ = std::make_shared<LaneLineRefiner>();
  occ_processor_ = std::make_shared<OccProcessor>();
  occ_processor_ ->Init(cross_data_manager_ptr);
  edge_merger_ = std::make_shared<EdgeMerger>();
  edge_merger_->Init();
}
void CrossDataManager::Reset() {
  ego_turn_type_        = BevAction::STRAIGHT;
  cross_state_          = CrossState::INIT;
  sd_section_points_    = {};
  dr_sd_section_points_ = {};
  main_lane_            = {};
  top_lane_             = {};
  ego_lane_             = {};
  dr_ego_lane_          = {};
  virtual_line_source_ = VirtualLineSource::PRE;
  is_in_cross_road_ = false;
  is_egolane_disappear_ = false;
  has_bev_ego_navi_action_ = false;
  using_junction_action_ = false;
  is_T_cross_road_flag_ = true;
  first_junction_lane_is_virtual_ = false;
  cross_road_dis_ = 1000.0;
  opening_.Clear();
  ego_road_lanes_.Clear();
  is_find_closetlane_bytopo_success_ = false;
  is_boundary_left_ = false;
  has_is_virtual_ = false;
  lane_cluster_ = std::make_shared<LaneLineCluster>();
  lane_refiner_ = std::make_shared<LaneLineRefiner>();
  occ_processor_ = std::make_shared<OccProcessor>();
  occ_processor_ ->Init(cross_data_manager_ptr);
  bev_ego_lane_t_ = cem::message::sensor::BevLaneInfo();
  bev_cut_start_point_ = Vec2d();
  bev_cut_start_right_point_ = Vec2d();
}

uint64_t CrossDataManager::GetNewLandId() {
  uint64_t new_id = 101;
  for (uint64_t i = 1; i < 100UL; ++i) {
    auto tmp = std::find_if(bev_map_raw_->lane_infos.begin(), bev_map_raw_->lane_infos.end(),
                            [i](const cem::message::sensor::BevLaneInfo& lane) { return lane.id == i; });
    if (tmp == bev_map_raw_->lane_infos.end()) {
      new_id = i;
      break;
    }
  }
  return new_id;
}

void CrossDataManager::DataUpdate(const RoutingMapPtr &routing_map_raw, const RoutingMapPtr &routing_map_ptr,
                                  const BevMapInfoConstPtr &bev_map_raw, const BevMapInfoPtr &bev_map, const LocalizationPtr &loc,
                                  const Eigen::Isometry3d &Twb, const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr, bool on_highway) {
  routing_map_raw_ = routing_map_raw;
  routing_map_     = routing_map_ptr;
  bev_map_raw_     = bev_map_raw;
  bev_map_         = bev_map;

  last_loc_          = currect_loc_;
  currect_loc_       = loc;
  Twb_               = Twb;
  Tbw_               = Twb.inverse();
  ego_lane_id_       = 0;
  is_egolane_normal_ = false;
  is_egolane_low_    = false;
  is_egolane_death_  = false;
  has_ego_lane_      = false;
  ego_lane_          = {};

  has_bev_stop_lane_  = false;
  has_bev_junction_   = false;
  has_bev_cross_walk_ = false;
  first_junction_lane_is_virtual_ = false;
  on_highway_ = on_highway;
  opening_.Clear();
  raw_lane_unmap_.clear();
  bev_lane_unmap_.clear();
  raw_lanemarker_unmap_.clear();
  routing_lane_unmap_.clear();
  edgesPtr_.clear();
  top_lanes_.clear();

  if (bev_map_raw_ == nullptr || bev_map_ == nullptr || routing_map_ == nullptr) {
    return;
  }

  for (auto &lane : routing_map_->lanes) {
    routing_lane_unmap_[lane.id] = &lane;
  }

  // 新增路口lane虚拟线检查
  // if (!CrossRoadVirtualLaneCheck()) {
  //   return;
  // }
#if PPM_UPDATE
  auto timestamp = bev_map_->header.timestamp;
  int temp_time_s = static_cast<int>(timestamp);
  int temp_time_us = static_cast<int>((timestamp-temp_time_s)*1000);
  XLOG<< "///////BEV MAP TIME/////////// : " << std::to_string( timestamp);
  // ppmImage = std::make_shared<ppm::PPMImage>();
  // ppmImage->DrawNumber(static_cast<int>(temp_time_us), 0.f,  48.0f - 8*2.f  , 2);
  // ppmImage->DrawNumber(static_cast<int>(temp_time_s), 0.f,  48.0f, 2);

#endif
  ///////////////////////////////////////////occ_processor_///////////////////////////////////////////////////
  HPTimer timer;
  // if (occ_processor_) {
  if (0) {

    occ_processor_->GetObserveOccInfo(cross_data_manager_ptr);
    occ_processor_->Process();
    occ_processor_->GetEdageInfoPtr(edgesPtr_);
    // for(auto &edge:edgesPtr_){
    //   XLOG << "edge id: " << edge.first << " size: " << edge.second->size();
    //   // for(auto &point:*edge.second){
    //   //   XLOG << "point: " << point.x << " " << point.y;
    //   // }
    //   }
  }

  ///////////////////////////////////////////occ_processor_///////////////////////////////////////////////////
  
  ///////////////////////////////////////////Edage Merge///////////////////////////////////////////////////
  if(edge_merger_ ){
    // auto bev_map_refined = std::make_shared<BevMapInfo>(*bev_map_);
    edge_merger_->SetCrossDataManager(cross_data_manager_ptr);

    // edge_merger_->Preprocess();
    // edge_merger_->Process();



  }

  ///////////////////////////////////////////Edage Merge///////////////////////////////////////////////////

  
  ///////////////////////////////////////////LineRefine///////////////////////////////////////////////////
  BevLaneMarker plane_lanemarker;
  for(const auto& junction:bev_map_raw_->junctions){
    if(junction.type == 26){
      plane_lanemarker = junction;
      break;
    }
  }
  if(lane_refiner_ &&  bev_map_){
    // auto bev_map_refined = std::make_shared<BevMapInfo>(*bev_map_);

    XLOG << "lane_refiner_ PROCESSED";
    auto bev_map_refined = bev_map_;
    lane_refiner_->SetPlaneLanemarker(plane_lanemarker);
    for ( auto &lane : bev_map_refined->lane_infos) {
       std::transform(lane.line_points.begin(), lane.line_points.end(), lane.line_points.begin(), [this](auto &pt) {
                        Eigen::Vector3d ptVCS(pt.x, pt.y, 0);
                        ptVCS = Tbw_ * ptVCS;
                        cem::message::common::Point2DF ret;
                        {
                          ret.x            = ptVCS[0];
                          ret.y            = ptVCS[1];
                          ret.point_source = pt.point_source;
                          ret.mse = pt.mse;
                          ret.scatter_visibility = pt.scatter_visibility;

                        };
                        return ret;
                      });
    }
    lane_refiner_->SetBevMap(bev_map_refined);//需要修理的laneline 原地输出
    lane_refiner_->SetCrossDateManager(cross_data_manager_ptr);
    // lane_refiner_->Process();
  #if 0
    for ( auto &lane : bev_map_refined->lane_infos) {
      for (const auto &point : lane.line_points) {
        ppmImage->DrawPoint(point.x, point.y, 0);
      }
    }
    for (const auto &point : plane_lanemarker.line_points) {
      ppmImage->DrawPoint(point.x, point.y, 1);
    }
    ppmImage->Save();
    
  #endif
    for ( auto &lane : bev_map_refined->lane_infos) {
       std::transform(lane.line_points.begin(), lane.line_points.end(), lane.line_points.begin(), [this](auto &pt) {
                        Eigen::Vector3d ptDR(pt.x, pt.y, 0);
                        ptDR = Twb_ * ptDR;
                        cem::message::common::Point2DF ret;
                        {
                          ret.x            = ptDR[0];
                          ret.y            = ptDR[1];
                          ret.point_source = pt.point_source;
                          ret.mse = pt.mse;
                          ret.scatter_visibility = pt.scatter_visibility;
                        };
                        return ret;
                      });
    }

  }

  ///////////////////////////////////////////LineRefine///////////////////////////////////////////////////

  for (auto &lane : bev_map_raw_->lane_infos) {
    if (lane.position == 0) {
      ego_lane_id_ = lane.id;
    }
    raw_lane_unmap_[lane.id] = &lane;
  }
  for (auto &lanemarker : bev_map_raw_->lanemarkers) {
    raw_lanemarker_unmap_[lanemarker.id] = &lanemarker;
  }
  for (auto &lane : bev_map_->lane_infos) {
    bev_lane_unmap_[lane.id] = &lane;
  }

  DealTCrossRoad(junctions_ptr);

  DealEgoLane();

  DealIsInCrossRoad();

  has_is_virtual_ = FindVirtualLane();
  FLOG_CROSS << " has_is_virtual: " << has_is_virtual_ << " ,ego_turn_type: " << EnumToString(ego_turn_type_);

  UpdateSdSectionPoints();

  FindCrossRoadFlag();

  DealEgoRoad();

  DealCrossOccBoundary();

  // DealCrossBoundary();

/////////////////////////////////////////////////////cluster///////////////////////////////

  std::vector<std::vector<Particle>> all_clusters;
  std::vector<std::vector<uint64_t>> clustered_lane_ids;
if(lane_cluster_ &&
  cross_data_manager_ptr->GetSdSections().end_point.x() < kThresholdToSDIntersection
  && std::abs(cross_data_manager_ptr->GetSdSections().end_point.x()) > 1e-6f){//导致偶发几帧坐标转换
// if(lane_cluster_){
#if LANE_CLUSTER
  XLOG << "CROSS PT X: "  << cross_data_manager_ptr->GetSdSections().end_point.x();
#endif
auto ret = lane_cluster_->PrecessProcess(cross_data_manager_ptr);
// if(ret){
if(0){
  lane_cluster_->SetLaneLines2Particles();
  lane_cluster_->MultiStepCluster(all_clusters);
  lane_cluster_->PostProcess(all_clusters);
}
#if LANE_CLUSTER
  XLOG << "UPDATE CROSS PT X: "  << cross_data_manager_ptr->GetSdSections().end_point.x();
#endif
  // target_clustered_lane_ids_ = lane_cluster_->GetTargetClusteredLaneIds();
  // ego_clustered_lane_ids_ = lane_cluster_->GetEgoClusteredLaneIds();
  
  top_lanes_ = lane_cluster_->GetTopLanes();
  ego_lanes_.clear();
  ego_lanes_ = lane_cluster_->GetEgoTopLanes();
  //遍历target_clustered_lane_ids_
#if LANE_CLUSTER
  auto y_offset = 1.6f;
  // for (const auto& lane_id : target_clustered_lane_ids_) {
  //     ppmImage->DrawNumber(static_cast<int>(lane_id), 4.0f, 48.0f - y_offset, 0);
  //     y_offset += 8.0f;
  //   }

  y_offset = 1.6f;      
  // for (const auto& toplane : ego_lanes_) {
  //     auto color_index = 1;
  //     color_index = toplane.is_ego ? 0 : 1;
  //     ppmImage->DrawNumber(static_cast<int>(toplane.lane_id), 2.0f, 48.0f - y_offset, color_index);
  //     y_offset += 8.0f;
  // }      
  y_offset = 1.6f;      
  for (const auto& lane_id : sd_guide_lane_ids_) {
      ppmImage->DrawNumber(static_cast<int>(lane_id), -2.0f, 48.0f - y_offset, 0);
      y_offset += 8.0f;
    }      
    int color_index = 2;
    for (auto& subCluster : all_clusters) {
      color_index++;
      for(auto& partical:subCluster){
        bool is_sd_line = partical.groupType == LANE_GROUP_SD;
        bool is_edg = partical.groupType == LANE_GROUP_EDAGE;
        if(std::holds_alternative<std::shared_ptr<BevLaneInfo>>(partical.sourcePtr)){
          auto &lane = std::get<std::shared_ptr<BevLaneInfo>>(partical.sourcePtr);
        if(lane){
          ppmImage->DrawNumber(static_cast<int>(partical.lane_id),  lane->line_points.front().x - 0.8,lane->line_points.front().y , color_index);
          for(auto& pt:lane->line_points){
            if(is_sd_line){
                ppmImage->DrawPoint(pt.x,pt.y,  color_index);
            }else{
                ppmImage->DrawPoint(pt.x,pt.y,  color_index);
              }
            }
          }
        }else if(std::holds_alternative<std::shared_ptr<BevLaneMarker>>(partical.sourcePtr)){//edg数据类型吧一样
          auto &edage = std::get<std::shared_ptr<BevLaneMarker>>(partical.sourcePtr);
          if(edage){
            if(edage->line_points.size() < 2){
              continue;
            }
            ppmImage->DrawNumber(static_cast<int>(partical.lane_id),  edage->line_points.front().x - 1.8,edage->line_points.front().y , color_index%14);
            for(auto& pt:edage->line_points){
                ppmImage->DrawPoint(pt.x,pt.y,1);//最后颜色
              }
            }
      }


      }
    }
  y_offset = 1.6f;
  for (const auto& anchor : lane_cluster_->top_lanes_) {
      for (auto& pt : anchor.points) {
        ppmImage->DrawPoint(pt.x(), pt.y(), 0);
        XLOG << "TOP LANE ID: " << anchor.id << " lane_id " << anchor.lane_id  <<" POINT: " << pt.x() << " " << pt.y();
        if(anchor.points.size()<1){
          XLOG << "ERROR: anchor.points.size<1";
        }
        
      }
      auto color_index = 0;
      color_index = anchor.is_connect_lane ? 1 : 0;
      ppmImage->DrawNumber(static_cast<int>(anchor.id),  anchor.points.front().x() - 0.8,anchor.points.front().y() , color_index);
     
      y_offset += 8.0f;

  }
  for (const auto& anchor : lane_cluster_->ego_lanes_) {
      for (auto& pt : anchor.points) {
        ppmImage->DrawPoint(pt.x(), pt.y(), 1);
    }
  }
  if(sd_section_points_.current_lane_num > -1){
    ppmImage->DrawNumber(static_cast<int>(sd_section_points_.current_lane_num), -4.0f, 48.0f - 1.6f, 0);
  }
  ppmImage->Save();
  XLOG << "!!!!SAVE";
#endif
}
// XLOG << "XXF TIME COST " << timer.ElapsedMilliSeconds();
/////////////////////////////////////////////////////cluster///////////////////////////////
FLOG_CROSS << "top_lanes_size: " << top_lanes_.size();
for (const auto &top : top_lanes_) {
  FLOG_CROSS << "top_is_connect_lane: " << top.is_connect_lane << " ,lane_id: " << top.lane_id;
  for (const auto &pt : top.points) {
    FLOG_CROSS << " x: " << pt.x() << " ,y: " << pt.y();
  }
}
FLOG_CROSS << " DataUpdate end is_in_cross_road: " << is_in_cross_road_;
}

void CrossDataManager::UpdateSdSectionPoints() {
  sd_section_points_.Clear();
  if (routing_map_raw_ == nullptr || bev_map_raw_ == nullptr) {
    FLOG_CROSS << " UpdateSdSectionPoints faild!";
    return;
  }

  ///////////////////////////////////////////cluster///////////////////////////////////////////////////
  // auto junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoCityPtr();
  // if(junctions_ptr){
  //   uint64_t section_id{0};
  //   auto current_junction_id = INTERNAL_PARAMS.navigation_info_data.target_crossroad_junction_id();
  //   for(const auto & junction : *junctions_ptr) {
  //     if (junction.junction_id == current_junction_id) {
  //       auto section_ids = junction.junction_ids; 
  //       if(!section_ids.empty()){
  //         section_id = section_ids.back(); 
  //       }
  //       break;
  //     }
  //   }

  //   std::shared_ptr<std::vector<Eigen::Vector2f>> pointsPtr{nullptr};
  //   if(section_id){
  //     auto section = INTERNAL_PARAMS.sd_map_data.GetSDSectionInfoById(section_id);
  //     if (section) {
  //       pointsPtr = section->points;
  //     }else{
  //       AINFO << "!!!!section null";
  //     }
      
  //   }
  //   if(pointsPtr && pointsPtr->size() > 1){
  //     auto mid_point = ((*pointsPtr).front() + (*pointsPtr).back())/2;
  //     auto qu_point_front = ((*pointsPtr).front() + mid_point)/2;
  //     auto qu_point_back = ((*pointsPtr).back() + mid_point)/2;
  //     sd_section_points_.top_points.emplace_back(qu_point_front.x(), qu_point_front.y());
  //     sd_section_points_.top_points.emplace_back(qu_point_back.x(), qu_point_back.y());
  
  //   }else{
  //     // AINFO << "!!!!pointsPtr null";
  //   }
  // }else{
  //   AINFO << "!!!!junctions_ptr null";
  // }

  ///////////////////////////////////////////cluster///////////////////////////////////////////////////
  // 找出路口面
  BevLaneMarker cross_lanemarker;
  for (const auto &junction : bev_map_raw_->junctions) {
    if (junction.type == 26) {
      cross_lanemarker = junction;
      break;
    }
  }
  // INTERSECTION_ZONE
  const auto &bev_cross_points      = cross_lanemarker.line_points;
  bool        has_intersection_zone = bev_cross_points.size() > 3;
  double      cross_left_point_y    = -1000.0;
  double      cross_right_point_y   = 1000.0;
  double      cross_top_point_x     = -1000.0;
  double      cross_mian_point_x    = 1000.0;
  if (has_intersection_zone) {
    double average_x{0.0};
    double average_y{0.0};
    for (const auto &point : bev_cross_points) {
      average_x += point.x;
      average_y += point.y;
      if (point.y > cross_left_point_y) {
        cross_left_point_y = point.y;
      }
      if (point.y < cross_right_point_y) {
        cross_right_point_y = point.y;
      }
      if (point.x < cross_mian_point_x) {
        cross_mian_point_x = point.x;
      }
      if (point.x > cross_top_point_x) {
        cross_top_point_x = point.x;
      }
    }
    sd_section_points_.cross_point = Vec2d(average_x / bev_cross_points.size(), average_y / bev_cross_points.size());
  }

  const auto &navi_start  = routing_map_raw_->sd_route.navi_start;
  const auto &mpp_section = routing_map_raw_->sd_route.mpp_sections;
  const auto  it_navi_seciton =
      std::find_if(mpp_section.begin(), mpp_section.end(),
                   [&navi_start](const cem::message::env_model::SDSectionInfo &p) { return p.id == navi_start.section_id; });
  if (it_navi_seciton == mpp_section.end()) {
    FLOG_CROSS << " it_navi_seciton == mpp_section.end()!!!";
    return;
  }

  double navi_seciton_length = - navi_start.s_offset;
  for (auto it = it_navi_seciton; it != mpp_section.end(); it++) {
    // FLOG_CROSS << " it_navi_seciton_id: " << it->id << " ,point_size: " << it->points->size();
    if (it->has_junction && !it->points->empty()) {
      sd_section_points_.has_junction = true;
    }
    for (const auto& point : *it->points) {
      // FLOG_CROSS << "sd_section_points x: " << point.x() << " ,y: " << point.y();
      byd::common::math::Vec2d v_p(point.x(), point.y());
      if (sd_section_points_.points.empty()) {
        sd_section_points_.points.emplace_back(v_p);
        sd_section_points_.points_lane_num.emplace_back(it->lane_num);
        if(it->lane_group_idx.size() > 0){
          sd_section_points_.lane_groupid.emplace_back(it->lane_group_idx.back().id);
        }else{//navistart 缺少字段该信息 默认填充
          sd_section_points_.lane_groupid.emplace_back(0);
        }
        sd_section_points_.section_direction.emplace_back(it->direction);
      } else if (sd_section_points_.points.back().DistanceTo(v_p) > 0.5) {
        sd_section_points_.points.emplace_back(v_p);
        if(it->lane_group_idx.size() > 0){
          sd_section_points_.lane_groupid.emplace_back(it->lane_group_idx.back().id);
        }else{//navistart 缺少字段该信息 默认填充
          sd_section_points_.lane_groupid.emplace_back(0);
        }
        sd_section_points_.points_lane_num.emplace_back(it->lane_num);
        sd_section_points_.section_direction.emplace_back(it->direction);
      }
    }
    if (navi_seciton_length > 200.0) {
      break;
    }
    navi_seciton_length += it->length;
  }
  // for (const auto &point : bev_cross_points) {
  //   FLOG_CROSS << "bev_cross_points x: " << point.x << " ,y: " << point.y;
  // }
  if (sd_section_points_.points.size() < 2) {
    sd_section_points_.Clear();
    return;
  }
  FLOG_CROSS << " has_intersection_zone: " << has_intersection_zone << " , is_in_cross_road: " << is_in_cross_road_;
  std::vector<byd::common::math::Vec2d> intersetion_points;
  std::vector<byd::common::math::Vec2d> intersetion_headings;
  std::vector<int> intersection_indexs;
  int laneNum = -1;
  uint64_t targetLaneGorupId  = 0;
  cem::message::env_model::SDDirectionType sdDirection =  cem::message::env_model::SDDirectionType::UNKNOWN;
  int intersetion_index{0};
  int cross_sd_start_index = -1;
  double start_heading = (sd_section_points_.points[1] - sd_section_points_.points[0]).Angle();
  double end_heading = start_heading;
  // 沿sd垂线寻找路口起点和终点参数
  bool has_start_intersection_point = false;
  bool has_end_intersection_point = false;
  byd::common::math::Vec2d start_intersetion_point;
  int start_intersetion_index = 0;
  byd::common::math::Vec2d top_intersetion_point;
  int top_intersetion_index = 0;
  if (has_intersection_zone) {
    // 沿sd垂线寻找路口起点
    for (int i = 0; i < sd_section_points_.points.size() - 1; i++) {
      auto axis        = sd_section_points_.points[i + 1] - sd_section_points_.points[i];
      double length = sd_section_points_.points[i + 1].DistanceTo(sd_section_points_.points[i]);
      auto right_point = Vec2d(sd_section_points_.points[i].x() + axis.y(), sd_section_points_.points[i].y() - axis.x());
      double min_dis = 1000.0;
      int min_bev_cross_points_index = 0;
      bool is_in_left = true;
      for (int j = 0; j < bev_cross_points.size(); j++) {
        if (PointInVectorSide(sd_section_points_.points[i], right_point, bev_cross_points[j]) > 0) {
          is_in_left = false;
          break;
        }
        auto dis = PointToVectorDist(sd_section_points_.points[i], right_point, bev_cross_points[j]);
        if(dis < min_dis){
          min_dis = dis;
          min_bev_cross_points_index = j;
        }
      }
      if (is_in_left && min_dis < length) {
        has_start_intersection_point = true;
        GetFootPoint(Vec2d(bev_cross_points[min_bev_cross_points_index].x, bev_cross_points[min_bev_cross_points_index].y),
                     sd_section_points_.points[i], sd_section_points_.points[i + 1], start_intersetion_point);
        start_intersetion_index = i;
        break;
      }
    }
    if (has_start_intersection_point) {
      FLOG_CROSS << "start_intersetion_index: " << start_intersetion_index << " ,start_intersetion_point_x: " << start_intersetion_point.x()
                 << " ,y: " << start_intersetion_point.y();
    }
    // 沿sd垂线寻找路口终点
    int start_find_top_index = has_start_intersection_point ? start_intersetion_index + 1: 0;
    for (int i = start_find_top_index; i < sd_section_points_.points.size() - 1; i++) {
      auto axis        = sd_section_points_.points[i + 1] - sd_section_points_.points[i];
      double length = sd_section_points_.points[i + 1].DistanceTo(sd_section_points_.points[i]);
      auto right_point = Vec2d(sd_section_points_.points[i].x() + axis.y(), sd_section_points_.points[i].y() - axis.x());
      double max_dis = 0.0;
      int max_bev_cross_points_index = 0;
      bool is_in_left = false;
      for (int j = 0; j < bev_cross_points.size(); j++) {
        if (PointInVectorSide(sd_section_points_.points[i], right_point, bev_cross_points[j]) < 0) {
          auto dis = PointToVectorDist(sd_section_points_.points[i], right_point, bev_cross_points[j]);
          if (dis > max_dis) {
            is_in_left = true;
            max_dis = dis;
            max_bev_cross_points_index = j;
          }
        }
      }
      if (is_in_left && max_dis < length) {
        has_end_intersection_point = true;
        end_heading = (sd_section_points_.points[i + 1] - sd_section_points_.points[i]).Angle();
        GetFootPoint(Vec2d(bev_cross_points[max_bev_cross_points_index].x, bev_cross_points[max_bev_cross_points_index].y),
                     sd_section_points_.points[i], sd_section_points_.points[i + 1], top_intersetion_point);
        top_intersetion_index = i;
        break;
      }
    }
    if (has_end_intersection_point) {
      FLOG_CROSS << "top_intersetion_index: " << top_intersetion_index << " ,top_intersetion_point_x: " << top_intersetion_point.x()
                 << " ,y: " << top_intersetion_point.y();
    }

    for (int i = 1; i < sd_section_points_.points.size(); i++) {
      // FLOG_CROSS << "intersetion_sd_points x: " << sd_section_points_.points[i - 1].x() << " ,y: " << sd_section_points_.points[i - 1].y()
      // << " ,next_x: " << sd_section_points_.points[i].x() << " ,y: " << sd_section_points_.points[i].y();
      auto points = FindIntersectionPoints(bev_cross_points, sd_section_points_.points[i - 1], sd_section_points_.points[i]);
      if (points.empty()) {
        continue;
      }
      laneNum = sd_section_points_.points_lane_num[i];
      targetLaneGorupId = sd_section_points_.lane_groupid[i];
      // XLOG << "targetLaneGorupId" << targetLaneGorupId;
      byd::common::math::Vec2d intersetion_heading = sd_section_points_.points[i];
      sdDirection = sd_section_points_.section_direction[i];
      intersetion_index = i;
      if (cross_sd_start_index < 0) {
        cross_sd_start_index = i - 1;
      }
      for (const auto &point : points) {
        FLOG_CROSS << " intersetion_points x: " << point.x() << " ,y: " << point.y() << ",heading_point_x: " << intersetion_heading.x()
                   << " ,y: " << intersetion_heading.y();
        intersetion_points.emplace_back(point);
        intersetion_headings.emplace_back(intersetion_heading);
        intersection_indexs.emplace_back(i - 1);
      }
    }
    // XLOG << "sd_section_points_.lane_groupid[i] #0 " << targetLaneGorupId;
    FLOG_CROSS << " intersetion_index: " << intersetion_index;
    int reserve_index = intersetion_index + 1;
    for(int i = 0; i < sd_section_points_.points.size();i++){
      if(sd_section_points_.points[i].x() > 150.0 || std::abs(sd_section_points_.points[i].y()) > 50.0){
        if(i > reserve_index){
          reserve_index = i + 1;
        }
        break;
      }
    }
    reserve_index = std::max(reserve_index,top_intersetion_index + 2);
    sd_section_points_.points.resize(reserve_index);

    FLOG_CROSS << " intersetion_points size: " << intersetion_points.size() << " ,sd_section_points_sizeL"
               << sd_section_points_.points.size() << " , start_heading: " << start_heading << " ,end_heading: " << end_heading
               << " ,top_intersetion_index: " << top_intersetion_index;
  }

  // 再次判断ego_turn_type，直行大角度改成左右转
  if (has_intersection_zone && !is_in_cross_road_ && virtual_line_source_ != VirtualLineSource::LD && virtual_line_source_ != VirtualLineSource::EXP && !is_enable_mnoa) {
    double angle = (end_heading - start_heading) * 180 / pi;
    if (angle > 35 || (angle > 30 && dr_sd_section_points_.straight_angle_large)) {
      sd_section_points_.straight_angle_large    = true;
      dr_sd_section_points_.straight_angle_large = true;
      if (ego_turn_type_ == BevAction::STRAIGHT) {
        ego_turn_type_ = BevAction::LEFT_TURN;
      }
    } else if (angle < -35 || (angle < -30 && dr_sd_section_points_.straight_angle_large)) {
      sd_section_points_.straight_angle_large    = true;
      dr_sd_section_points_.straight_angle_large = true;
      if (ego_turn_type_ == BevAction::STRAIGHT) {
        ego_turn_type_ = BevAction::RIGHT_TURN;
      }
    } else {
      sd_section_points_.straight_angle_large    = false;
      dr_sd_section_points_.straight_angle_large = false;
    }
    FLOG_CROSS << "delta_angle is " << angle << " ,ego_turn_type: " << int(ego_turn_type_)
               << " ,straight_angle_large: " << sd_section_points_.straight_angle_large;
  } else {
    sd_section_points_.straight_angle_large = dr_sd_section_points_.straight_angle_large;
  }
  FLOG_CROSS << " ego_turn_type: " << int(ego_turn_type_);
  // 根据intersetion_points点的情况判断怎么得到start，end point
  // 0: 无效,使用历史值; 1：使用两个点; 2. 使用两个极值;  3. 使用最后一个点; 4. 使用一个极值
  int using_intersetion_points = 0;
  if(has_intersection_zone){
    // 分为路口前和路口中
    // 路口前，必须有两个交点左右转的交点的heading也必须匹配，如果交点不是两个则根据路口面极值去更新，如果没有交点则使用历史值
    if(is_in_cross_road_){
      if(!intersetion_points.empty()){
        double sd_end_heading = (intersetion_headings.back() - intersetion_points.back()).Angle();
        auto   end_point      = TransformPointVec(dr_sd_section_points_.end_point, Tbw_);
        auto   top_vector     = TransformPointVec(dr_sd_section_points_.top_vector, Tbw_);
        double target_heading = (top_vector - end_point).Angle();
        FLOG_CROSS << " sd_end_heading: " << sd_end_heading << " ,target_heading: " << target_heading;
        // 最后一个点heading符合就是用，不然使用极值点
        if(std::abs(target_heading - sd_end_heading) < 0.2){
          using_intersetion_points = 3;
        }else{
          using_intersetion_points = 4;
        }
      }
    }else{
      if(intersetion_points.size() == 2){
        double sd_begin_heading = (intersetion_headings.front() - intersetion_points.front()).Angle();
        double sd_end_heading = (intersetion_headings.back() - intersetion_points.back()).Angle();
        FLOG_CROSS << " sd_begin_heading: " << sd_begin_heading << " ,sd_end_heading: " << sd_end_heading;
        // 路口前有两个交点且交点的角度都符合，正常更新所有值
        if(std::abs(sd_begin_heading - start_heading) < 0.2 && std::abs(sd_end_heading - end_heading) < 0.2){
          using_intersetion_points = 1;
        }else{
          using_intersetion_points = 2;
        }
        // using_intersetion_points = 2;
      }else if(!intersetion_points.empty()){
        using_intersetion_points = 2;
      }
    }
  }
  FLOG_CROSS << " using_intersetion_points: " << using_intersetion_points
             << " ,dr_sd_section_points_.has_intersection_zone: " << dr_sd_section_points_.has_intersection_zone;
  if(using_intersetion_points == 1){
    sd_section_points_.main_vector           = intersetion_headings[0];
    sd_section_points_.start_point           = intersetion_points[0];
    sd_section_points_.end_point             = intersetion_points[1];
    sd_section_points_.top_vector            = intersetion_headings[1];
    sd_section_points_.targetLaneGorupId_    = targetLaneGorupId;
    sd_section_points_.current_lane_num      = laneNum;
    sd_section_points_.current_sd_dirction_  = sdDirection;
    sd_section_points_.start_sd_point_index  = cross_sd_start_index;
    dr_sd_section_points_.end_sd_point_index = intersetion_index + 1;
    sd_section_points_.has_intersection_zone = true;
    XLOG << " sd_section_points_.targetLaneGorupId_#2 " << sd_section_points_.targetLaneGorupId_;
    auto [left_dis, right_dis]                = FindTopCrossPointDis(bev_cross_points, intersetion_points[1], intersetion_headings[1]);
    sd_section_points_.top_cross2sd_left_dis  = left_dis;
    sd_section_points_.top_cross2sd_right_dis = right_dis;
    //更新dr_sd_section_points
    dr_sd_section_points_.has_intersection_zone = true;
    dr_sd_section_points_.end_point             = TransformPointVec(sd_section_points_.end_point, Twb_);
    dr_sd_section_points_.start_point           = TransformPointVec(sd_section_points_.start_point, Twb_);
    dr_sd_section_points_.cross_point           = TransformPointVec(sd_section_points_.cross_point, Twb_);
    dr_sd_section_points_.top_vector            = TransformPointVec(sd_section_points_.top_vector, Twb_);
    dr_sd_section_points_.main_vector           = TransformPointVec(sd_section_points_.main_vector, Twb_);
    dr_sd_section_points_.start_sd_point_index  = sd_section_points_.start_sd_point_index;
    dr_sd_section_points_.end_sd_point_index    = sd_section_points_.end_sd_point_index;
    dr_sd_section_points_.current_lane_num      = sd_section_points_.current_lane_num;
    XLOG << " sd_section_points_.targetLaneGorupId_#2 " << sd_section_points_.targetLaneGorupId_;
    dr_sd_section_points_.current_sd_dirction_   = sd_section_points_.current_sd_dirction_;
    dr_sd_section_points_.top_cross2sd_left_dis  = sd_section_points_.top_cross2sd_left_dis;
    dr_sd_section_points_.top_cross2sd_right_dis = sd_section_points_.top_cross2sd_right_dis;
  }else if(using_intersetion_points == 2 && has_start_intersection_point && has_end_intersection_point){
    //
    sd_section_points_.main_vector           = sd_section_points_.points[start_intersetion_index + 1];
    sd_section_points_.start_point           = start_intersetion_point;
    sd_section_points_.end_point             = top_intersetion_point;
    sd_section_points_.top_vector            = sd_section_points_.points[top_intersetion_index + 1];
    sd_section_points_.targetLaneGorupId_    = sd_section_points_.lane_groupid[top_intersetion_index + 1];
    sd_section_points_.current_lane_num      = sd_section_points_.points_lane_num[top_intersetion_index + 1];
    sd_section_points_.current_sd_dirction_  = sd_section_points_.section_direction[top_intersetion_index + 1];
    sd_section_points_.start_sd_point_index  = start_intersetion_index;
    dr_sd_section_points_.end_sd_point_index = top_intersetion_index + 1;
    sd_section_points_.has_intersection_zone = true;
    auto [left_dis, right_dis]                = FindTopCrossPointDis(bev_cross_points, sd_section_points_.end_point, sd_section_points_.top_vector);
    sd_section_points_.top_cross2sd_left_dis  = left_dis;
    sd_section_points_.top_cross2sd_right_dis = right_dis;

  }else if(using_intersetion_points == 3){
    sd_section_points_.end_point             = intersetion_points.back();
    sd_section_points_.top_vector            = intersetion_headings.back();
    sd_section_points_.current_lane_num      = laneNum;
    sd_section_points_.targetLaneGorupId_    = targetLaneGorupId;
    sd_section_points_.current_sd_dirction_  = sdDirection;
    sd_section_points_.start_sd_point_index  = cross_sd_start_index;
    sd_section_points_.end_sd_point_index = intersetion_index + 1;
    sd_section_points_.has_intersection_zone = true;
    if (dr_sd_section_points_.has_intersection_zone) {
      sd_section_points_.start_point = TransformPointVec(dr_sd_section_points_.start_point, Tbw_);
      sd_section_points_.main_vector = TransformPointVec(dr_sd_section_points_.main_vector, Tbw_);
    }
    auto [left_dis, right_dis] = FindTopCrossPointDis(bev_cross_points, intersetion_points.back(), intersetion_headings.back());
    sd_section_points_.top_cross2sd_left_dis  = left_dis;
    sd_section_points_.top_cross2sd_right_dis = right_dis;
  }else if(using_intersetion_points == 4){
    sd_section_points_.end_point             = top_intersetion_point;
    sd_section_points_.top_vector            = sd_section_points_.points[top_intersetion_index + 1];
    sd_section_points_.targetLaneGorupId_    = sd_section_points_.lane_groupid[top_intersetion_index + 1];
    sd_section_points_.current_lane_num      = sd_section_points_.points_lane_num[top_intersetion_index + 1];
    sd_section_points_.current_sd_dirction_  = sd_section_points_.section_direction[top_intersetion_index + 1];
    sd_section_points_.start_sd_point_index  = 0;
    sd_section_points_.end_sd_point_index    = top_intersetion_index;
    sd_section_points_.has_intersection_zone = true;
    if (dr_sd_section_points_.has_intersection_zone) {
      sd_section_points_.start_point = TransformPointVec(dr_sd_section_points_.start_point, Tbw_);
      sd_section_points_.main_vector = TransformPointVec(dr_sd_section_points_.main_vector, Tbw_);
    }
    auto [left_dis, right_dis] = FindTopCrossPointDis(bev_cross_points, intersetion_points.back(), intersetion_headings.back());
    sd_section_points_.top_cross2sd_left_dis  = left_dis;
    sd_section_points_.top_cross2sd_right_dis = right_dis;
  }else{
    if (dr_sd_section_points_.has_intersection_zone && TransformPointVec(dr_sd_section_points_.end_point, Tbw_).x() > 0.0) {
      sd_section_points_.has_intersection_zone = true;
      sd_section_points_.end_point             = TransformPointVec(dr_sd_section_points_.end_point, Tbw_);
      sd_section_points_.start_point           = TransformPointVec(dr_sd_section_points_.start_point, Tbw_);
      sd_section_points_.cross_point           = TransformPointVec(dr_sd_section_points_.cross_point, Tbw_);
      sd_section_points_.top_vector            = TransformPointVec(dr_sd_section_points_.top_vector, Tbw_);
      sd_section_points_.main_vector           = TransformPointVec(dr_sd_section_points_.main_vector, Tbw_);
      sd_section_points_.current_lane_num      = dr_sd_section_points_.current_lane_num;
      sd_section_points_.targetLaneGorupId_    = targetLaneGorupId;
      XLOG << " sd_section_points_.targetLaneGorupId_#3 " << targetLaneGorupId;
      sd_section_points_.start_sd_point_index   = dr_sd_section_points_.start_sd_point_index;
      sd_section_points_.end_sd_point_index     = dr_sd_section_points_.end_sd_point_index;
      sd_section_points_.current_sd_dirction_   = dr_sd_section_points_.current_sd_dirction_;
      sd_section_points_.top_cross2sd_left_dis  = dr_sd_section_points_.top_cross2sd_left_dis;
      sd_section_points_.top_cross2sd_right_dis = dr_sd_section_points_.top_cross2sd_right_dis;
    } else {
      sd_section_points_.points.clear();
      XLOG << " sd_section_points_.targetLaneGorupId_#4 " << targetLaneGorupId;
    }
  }
  FLOG_CROSS << " sd_section_points_.end_point_x: " << sd_section_points_.end_point.x() << " ,y: " << sd_section_points_.end_point.y()
             << " ,top_vector_x: " << sd_section_points_.top_vector.x() << " ,y: " << sd_section_points_.top_vector.y()
             << " , sd_section_points_.top_vector: " << sd_section_points_.top_vector.Angle()
             << " ,start_x: " << sd_section_points_.start_point.x() << " ,lane number: " << sd_section_points_.current_lane_num
             << " ,direction: " << int(sd_section_points_.current_sd_dirction_);
}

// egolane默认延长到停止线或者路口面，所以可以使用最后一个点的位置进行判断
void CrossDataManager::DealIsInCrossRoad() {
  if (!is_in_cross_road_) {
    if (dr_ego_lane_.line_points.empty()) {
      return;
    }
    auto point = TransformPoint(dr_ego_lane_.line_points.back(), Tbw_);
    if (point.x <= 0.1 && (cross_state_ == CrossState::PREDICTION || cross_state_ == CrossState::CONNECTION)) {
      is_in_cross_road_ = true;
    }
  }
  // 判断过路口过程中egolane是否消失了一次，如果消失了就不再
  if(is_in_cross_road_ && !is_egolane_disappear_){
    bool has_ego_lane{false};
    for (auto &lane : bev_map_->lane_infos) {
      if (lane.id == dr_ego_lane_.id) {
        has_ego_lane = true;
      }
    }
    is_egolane_disappear_ = !has_ego_lane;
  }
}

void CrossDataManager::DealTCrossRoad(const cem::fusion::SdJunctionInfoCityPtr &junctions_ptr){
  // 由于T路口进入的，那么就要一直裁剪原来的egolane
  if ((cross_state_ == CrossState::PREDICTION || cross_state_ == CrossState::CONNECTION) && is_T_cross_road_flag_) {
    for (auto &lane : bev_map_->lane_infos) {
      if (lane.id == bev_ego_lane_t_.id) {
        if (lane.line_points.size() < 2) {
          // 需要删除虚拟线的前继
          lane.line_points.clear();
          lane.next_lane_ids.clear();
          return;
        }
        int cut_index = -1;
        FLOG_CROSS << " bev_cut_start_point_x: " << bev_cut_start_point_.x() << " ,y: " << bev_cut_start_point_.y()
                   << " bev_cut_start_right_point_: " << bev_cut_start_right_point_.x() << " ,y: " << bev_cut_start_right_point_.y();
        for (int i = 0; i < lane.line_points.size() - 1; i++) {
          // FLOG_CROSS << " lane.line_points[" << i << "]_x: " << lane.line_points[i].x << " ,y: " << lane.line_points[i].y;
          if (PointInVectorSide(bev_cut_start_point_,bev_cut_start_right_point_,lane.line_points[i]) <= 0) {
            cut_index = i;
            break;
          }
        }
        if(cut_index < 0){
          cut_index = lane.line_points.size();
        }
        FLOG_CROSS << " bev_ego_lane_t_id: " << bev_ego_lane_t_.id << " ,cut_index: " << cut_index
        << " lane.line_points.size: " << lane.line_points.size();
        // 需要删除虚拟线的前继
        if (cut_index == 0) {
          lane.line_points.clear();
          lane.next_lane_ids.clear();
          lane.line_points.emplace_back(bev_cut_start_point_.x(), bev_cut_start_point_.y());
          auto axis = bev_cut_start_right_point_ - bev_cut_start_point_;
          lane.line_points.emplace_back(bev_cut_start_point_.x() + axis.y() * 0.1, bev_cut_start_point_.y() - axis.x() * 0.1);
          FLOG_CROSS << " bev_ego_lane_t not fit!";
          return;
        }
        lane.line_points.resize(cut_index);
        lane.next_lane_ids.clear();
        break;
      }
    }
    return;
  }

  is_T_cross_road_flag_ = false;
  // 寻找80m内的右转T型路口
  if (junctions_ptr == nullptr) {
    return;
  }

  bool has_Tcrossroad = false;
  double sd_junction_offset = 1000.0;
  for (const auto &sd_junction : *junctions_ptr) {
    // FLOG_CROSS << " sd_junction_id: " << sd_junction.junction_id << " ,sd_junction.offset: " << sd_junction.offset
    // << " ,junction_action: " << int(sd_junction.junction_action) << " ,junction_type_city: " << int(sd_junction.junction_type_city);
    if (sd_junction.junction_action == cem::fusion::navigation::JunctionAction::TurnRight &&
        sd_junction.junction_type_city == cem::fusion::navigation::JunctionTypeCity::SmallTJunction && sd_junction.offset < 80.0 &&
        sd_junction.offset > 0.0) {
      // FLOG_CROSS << " sd_junction_id: " << sd_junction.junction_id << " ,sd_junction.offset: " << sd_junction.offset;
      sd_junction_offset = sd_junction.offset;
      has_Tcrossroad = true;
      break;
    }
  }
  if(!has_Tcrossroad){
    FLOG_CROSS << " has no has_Tcrossroad";
    return;
  }
  // 进入路口模式，且在路口中直接返回


  // 寻找有效egolane
  for (auto &lane : bev_map_->lane_infos) {
    FLOG_CROSS << "lane id: " << lane.id << " ,position: " << lane.position << " ,direction: " << int(lane.direction);
    if (lane.position == 0 && lane.direction != BevLaneDirection::DIRECTION_BACKWARD) {
      if (lane.line_points.empty()) {
        continue;
      }
      double lane_end_x = (TransformPoint(lane.line_points.back(), Tbw_)).x;
      if(lane_end_x < sd_junction_offset){
        continue;
      }
      bev_ego_lane_t_ = lane;
      break;
    }
  }
  // 裁剪点求取
  // 如果有经验轨迹或者地图虚拟线取起始点
    // 路口后不再更新
  if (is_in_cross_road_ || bev_ego_lane_t_.line_points.size() < 2) {
    FLOG_CROSS << " bev_ego_lane_t_.line_points.size() < 2";
    return;
  }
  if (routing_map_ == nullptr) {
    FLOG_CROSS << " routing_map_ == nullptr";
    return;
  }

  const auto * ld_ego_lane = FindRoutingMapRawLaneById(ld_ego_lane_id_);
  // 寻找120m内的lane有没有经验轨迹或者is_virtual: true
  constexpr double min_length = 120.0;
  double   length{0.0};
  int      index{0};
  uint64_t ld_lane_id = ld_ego_lane_id_;
  bool has_exp_traj{false};
  bool has_ld_traj{false};
  while (length < min_length && index < 100) {
    const auto *lane = FindRoutingMapRawLaneById(ld_lane_id);
    if (lane == nullptr || lane->is_virtual || lane->points.size() < 2) {
      break;
    }
    if (!lane->exp_trajectory_ids.empty()) {
      has_exp_traj = true;
    }
    for(const auto id:lane->next_lane_ids){
      const auto *next_lane = FindRoutingMapRawLaneById(id);
      if(next_lane == nullptr || next_lane->points.empty()){
        continue;
      }
      if(next_lane->is_virtual){
        has_ld_traj = true;
        break;
      }
    }
    // has_exp_traj = false;
    FLOG_CROSS << " has_ld_traj: " << has_ld_traj << " ,has_exp_traj: " << has_exp_traj;
    if(has_exp_traj || has_ld_traj){
      ld_ego_lane = lane;
      break;
    }

    if (lane->next_lane_ids.size() != 1) {
      break;
    }
    ld_lane_id = lane->next_lane_ids.front();
    double step_length = lane->points.front().x < 0.0 ? lane->points.back().x :lane->length;
    length += step_length;
    index++;
  }

  if (!has_exp_traj && !has_ld_traj) {
    FLOG_CROSS << " has not find exp_traj!!! ";
    return;
  }

  if (ld_ego_lane == nullptr) {
    FLOG_CROSS << " ld_ego_lane == nullptr";
    return;
  }

  bool has_trajectory = false;
  if (has_exp_traj) {
    for (auto exp_traj_id : ld_ego_lane->exp_trajectory_ids) {
      for (const auto &exp_traj : routing_map_->exp_trajectories) {
        if (exp_traj.id == exp_traj_id && SdHasEepTrajEndLaneid(exp_traj.end_lane_id) && exp_traj.points.size() > 1) {
          has_trajectory             = true;
          bev_cut_start_point_       = Vec2d(exp_traj.points.front().x, exp_traj.points.front().y);
          auto next_p                = Vec2d(exp_traj.points[1].x, exp_traj.points[1].y);
          auto axis                  = next_p - bev_cut_start_point_;
          bev_cut_start_right_point_ = Vec2d(bev_cut_start_point_.x() + axis.y(), bev_cut_start_point_.y() - axis.x());
          break;
        }
      }
    }
  } else if (has_ld_traj) {
    for (auto ld_id : ld_ego_lane->next_lane_ids) {
      const auto *next_lane = FindRoutingMapLaneById(ld_id);
      if (next_lane == nullptr || next_lane->points.size() < 2) {
        continue;
      }
      has_trajectory       = true;
      bev_cut_start_point_ = Vec2d(next_lane->points.front().x, next_lane->points.front().y);
      auto next_p          = Vec2d(next_lane->points[1].x, next_lane->points[1].y);
      auto axis            = next_p - bev_cut_start_point_;
      bev_cut_start_right_point_ = Vec2d(bev_cut_start_point_.x() + axis.y(), bev_cut_start_point_.y() - axis.x());
      break;
    }
  }

  if(!has_trajectory){
    FLOG_CROSS << " has no has_exp/ld_trajectory";
    return;
  }
  // 根据裁剪点对egolane就行裁剪
  for (auto &lane : bev_map_->lane_infos) {
    if(lane.id == bev_ego_lane_t_.id){
      if(lane.line_points.size() < 2){
        FLOG_CROSS << " lane.line_points.size() < 2";
        return;
      }
      int cut_index = 0;
      for(int i = 0;i < lane.line_points.size();i++){
        if(PointInVectorSide(bev_cut_start_point_,bev_cut_start_right_point_,lane.line_points[i]) <= 0){
          cut_index = i;
          break;
        }
      }
      if(cut_index == 0){
        FLOG_CROSS << " cut_index == 0";
        return;
      }
      lane.line_points.resize(cut_index);
      lane.next_lane_ids.clear();
      break;
    }
  }
  is_T_cross_road_flag_ = true;
  FLOG_CROSS << " is_T_cross_road_flag: " << is_T_cross_road_flag_;
  return;
}

void CrossDataManager::DealEgoLane(){
  int ego_lane_size{0};
  for (auto &lane : bev_map_->lane_infos) {
    XLOG << "lane id: " << lane.id << " ,position: " << lane.position << " ,direction: " << int(lane.direction);
    if (lane.position == 0 && lane.direction != BevLaneDirection::DIRECTION_BACKWARD) {
      if(lane.line_points.size() < 2){
        continue;
      }
      double lane_end_x = (TransformPoint(lane.line_points.back(), Tbw_)).x;
      double lane_start_x = (TransformPoint(lane.line_points.front(), Tbw_)).x;
      if (lane_end_x < 0.1 || lane_start_x > 0.1) {
        XLOG << " lane_end_x " << lane_end_x  <<  "lane_start_x "  << lane_start_x;
        continue;
      }
      bool has_ego_lane = false;
      // 先检查next lane是否满足作为egolane
      double total_length = lane_end_x;
      XLOG << "lane_end_x "  << lane_end_x;
      int index = 0;
      BevLaneInfo next_lane = lane;
      while (next_lane.next_lane_ids.size() >= 1 && total_length < 80.0 && index < 10) {
        double   next_heading       = GetBevLaneEndHeading(next_lane, 15.0);
        if(std::abs(next_heading) > 10.0){
          next_heading = 0.0;
        }
        uint64_t next_id = next_lane.next_lane_ids.front();
        if (next_lane.next_lane_ids.size() > 1) {
          double min_heading_err = 100.0;
          for (auto next_next_id : next_lane.next_lane_ids) {
            XLOG << "---next_next_id " << next_next_id;
            if (bev_lane_unmap_.find(next_next_id) == bev_lane_unmap_.end()) {
              continue;
            }
            auto *next_next_lane = bev_lane_unmap_[next_next_id];
            if (next_next_lane->line_points.size() < 2) {
              continue;
            }
            double next_next_heading = GetBevLaneEndHeading(*next_next_lane, 15.0);
            if (next_next_heading > 10.0) {
              continue;
            }
            double angle_diff = std::abs(next_next_heading - next_heading);
            if (angle_diff < min_heading_err) {
              min_heading_err = angle_diff;
              next_id         = next_next_id;
            }
          }
        }
        bool has_next_lane = false;
        for (const auto &lane_tp : bev_map_->lane_infos) {
          if (lane_tp.id == next_id && lane_tp.line_points.size() > 1) {
            next_lane = lane_tp;
            total_length += next_lane.length;
            has_next_lane = true;
            break;
          }
        }
        if (!has_next_lane) {
          break;
        }
        index++;
      }
      // 80m内找到了最后，使用nextlane
      XLOG << "total_length "  <<  total_length << "  TransformPoint(lane.line_points.front(), Tbw_)).x " << TransformPoint(lane.line_points.front(), Tbw_).x ;
      if (next_lane.next_lane_ids.empty() && total_length < 80.0) {
        dr_ego_lane_ = next_lane;
        ego_lane_size++;
        has_ego_lane = true;
      }
      // 不满足再使用egolane
      if (!has_ego_lane) {
        if (!lane.line_points.empty() && (TransformPoint(lane.line_points.front(), Tbw_)).x < -1.0) {
          dr_ego_lane_ = lane;
          ego_lane_size++;
        }
      }
    }
  }
  if (ego_lane_size < 1) {
    XLOG << " -------bev_map has no ego lane!!! ";
    return;
  }

  // 将ego_lane转换到自车坐标系
  ego_lane_ = dr_ego_lane_;
  XLOG << " DealEgoLane update!ego_lane id: " << ego_lane_.id << " ,line_points size:" << ego_lane_.line_points.size()
             << " ,ego_turn_type_navi_action: " << EnumToString(ego_lane_.navi_action);
  for (auto &point : ego_lane_.line_points) {
    TransformPoint(&point, Tbw_);
  }

  if (!ego_lane_.line_points.empty()) {
    // for(const auto& point:ego_lane_.line_points){
    //   FLOG_CROSS << " ego lane point_x: " << point.x << " ,y: " << point.y;
    // }
    XLOG << " ego lane id: " << ego_lane_.id << " , x: " << ego_lane_.line_points.back().x
               << " ,y:" << ego_lane_.line_points.back().y << ",length: " <<ego_lane_.length;
  }

  // 进入路口模式后先查找ego_lane_.navi_action，如果有就使用，并且此路口不再使用junction_action;如果没有就使用
  // junction_action，并且此路口不再使用navi_action

  // navi_action
  if (!is_in_cross_road_ && !using_junction_action_) {
    if (ego_lane_.navi_action == BevAction::STRAIGHT || ego_lane_.navi_action == BevAction::LEFT_TURN ||
        ego_lane_.navi_action == BevAction::RIGHT_TURN || ego_lane_.navi_action == BevAction::U_TURN) {
      has_bev_ego_navi_action_ = true;
      ego_turn_type_           = ego_lane_.navi_action;
    }
  }
  // junction_action
  auto junctions_ptr = INTERNAL_PARAMS.navigation_info_data.GetJunctionInfoCityPtr();
  double junctions_offset{DBL_MAX};
  if(junctions_ptr && !is_in_cross_road_ && !has_bev_ego_navi_action_){
    fusion::navigation::JunctionAction  junction_action{fusion::navigation::JunctionAction::Unknown};
    for (const auto &junction : *junctions_ptr) {
      if (junction.offset < junctions_offset && junction.offset < 120.0 && junction.offset > 0.0 &&
          (junction.junction_type_city == fusion::navigation::JunctionTypeCity::CrossRoad ||
           junction.junction_type_city == fusion::navigation::JunctionTypeCity::TJunction ||
           junction.junction_type_city == fusion::navigation::JunctionTypeCity::SmallTJunction ||
           (junction.junction_type_city == fusion::navigation::JunctionTypeCity::UTurn &&
            junction.junction_action == fusion::navigation::JunctionAction::UTurn)) &&
          junction.need_match_arrow) {
        junctions_offset = junction.offset;
        junction_action  = junction.junction_action;
        XLOG << "junctions_offset: " << junctions_offset << " ,junction_action: " << EnumToString(junction_action)
                   << " ,junction_id: " << junction.junction_id;
      }
    }
    if (junction_action != fusion::navigation::JunctionAction::Unknown) {
      switch (junction_action) {
        case fusion::navigation::JunctionAction::GoStraight:
          ego_turn_type_ = BevAction::STRAIGHT;
          using_junction_action_ = true;
          break;
        case fusion::navigation::JunctionAction::TurnLeft:
          ego_turn_type_ = BevAction::LEFT_TURN;
          using_junction_action_ = true;
          break;
        case fusion::navigation::JunctionAction::TurnRight:
          ego_turn_type_ = BevAction::RIGHT_TURN;
          using_junction_action_ = true;
          break;
        case fusion::navigation::JunctionAction::UTurn:
          ego_turn_type_ = BevAction::U_TURN;
          using_junction_action_ = true;
          break;
        default:
          XLOG << "error ";
          break;
      }
    }
  }
  // ego_turn_type_ = BevAction::STRAIGHT;
  XLOG << "has_bev_ego_navi_action: " << has_bev_ego_navi_action_ << " ,using_junction_action: " << using_junction_action_
             << " , ego_turn_type: " << EnumToString(ego_turn_type_);
  // if (ego_lane_.navi_action != BevAction::STRAIGHT && ego_lane_.navi_action != BevAction::LEFT_TURN &&
  //     ego_lane_.navi_action != BevAction::RIGHT_TURN && ego_lane_.navi_action != BevAction::U_TURN) {
  //   ego_turn_type_ =
  //       (ego_turn_type_ == BevAction::RIGHT_TURN || ego_turn_type_ == BevAction::LEFT_TURN || ego_turn_type_ == BevAction::U_TURN)
  //           ? ego_turn_type_
  //           : BevAction::STRAIGHT;
  // } else {
  //   ego_turn_type_ = ego_lane_.navi_action;
  // }
  // ego_turn_type_ = BevAction::RIGHT_TURN;

  // 检查lane
  const auto& lane_points = ego_lane_.line_points;
  if (lane_points.size() < 3) {
    return;
  }
  if (std::abs(ego_lane_.line_points.back().x - ego_lane_.line_points.front().x) > 15.0 && ego_lane_.line_points.back().x > 10.0) {
    is_egolane_normal_ = true;
  }
  has_ego_lane_ = true;
  XLOG << " has_ego_lane: " << has_ego_lane_ << " ,is_egolane_normal: " << is_egolane_normal_;
}

double CrossDataManager::GetBevLaneStartHeading(const BevLaneInfo &lane, double length) {
  if (lane.line_points.size() < 2) {
    return 100.0;
  }
  auto   start_point = TransformPoint(lane.line_points.front(), Tbw_);
  auto   end_point   = TransformPoint(lane.line_points.back(), Tbw_);
  double dis         = std::hypot(start_point.x - end_point.x, start_point.y - end_point.y);
  if (dis < 0.5) {
    return 100.0;
  }
  if (dis < length) {
    return Vec2d(end_point.x - start_point.x, end_point.y - start_point.y).Angle();
  }
  for (int i = 1; i < lane.line_points.size(); i++) {
    auto now_p = TransformPoint(lane.line_points[i], Tbw_);
    if (std::hypot(now_p.x - start_point.x, now_p.y - start_point.y) > length - 1.0) {
      return Vec2d(now_p.x - start_point.x, now_p.y - start_point.y).Angle();
    }
  }
  return Vec2d(end_point.x - start_point.x, end_point.y - start_point.y).Angle();
}

double CrossDataManager::GetBevLaneEndHeading(const BevLaneInfo &lane, double length) {
  if (lane.line_points.size() < 2) {
    return 100.0;
  }
  auto   start_point = TransformPoint(lane.line_points.front(), Tbw_);
  auto   end_point   = TransformPoint(lane.line_points.back(), Tbw_);
  double dis         = std::hypot(start_point.x - end_point.x, start_point.y - end_point.y);
  if (dis < 0.5) {
    return 100.0;
  }
  if (dis < length) {
    return Vec2d(end_point.x - start_point.x, end_point.y - start_point.y).Angle();
  }
  for (int i = lane.line_points.size() - 2; i >= 0; i--) {
    auto now_p = TransformPoint(lane.line_points[i], Tbw_);
    if (std::hypot(now_p.x - end_point.x, now_p.y - end_point.y) > length - 1.0) {
      return Vec2d(end_point.x - now_p.x, end_point.y - now_p.y).Angle();
    }
  }
  return Vec2d(end_point.x - start_point.x, end_point.y - start_point.y).Angle();
}

// bool CrossDataManager::IsLaneNormal(uint64_t id, double length) {
//   if (raw_lane_unmap_.find(id) == raw_lane_unmap_.end()) {
//     return false;
//   }
//   // 检查lane
//   const auto& lane_points = raw_lane_unmap_[id]->line_points;
//   auto left_lane_marker_id = raw_lane_unmap_[id]->left_lane_marker_id;
//   auto right_lane_marker_id = raw_lane_unmap_[id]->right_lane_marker_id;
//   if (lane_points.size() < 3 || lane_points.front().x > 3.0 ||
//       std::abs(lane_points.back().x - lane_points.front().x) < length ||
//       raw_lanemarker_unmap_.find(left_lane_marker_id) == raw_lanemarker_unmap_.end() ||
//       raw_lanemarker_unmap_.find(right_lane_marker_id) == raw_lanemarker_unmap_.end()) {
//     return false;
//   }
//   // 检查lanemarker
//   const auto& left_lanemarker_points = raw_lanemarker_unmap_[left_lane_marker_id]->line_points;
//   const auto& right_lanemarker_points = raw_lanemarker_unmap_[right_lane_marker_id]->line_points;
//   if (left_lanemarker_points.size() < 3 || left_lanemarker_points.front().x > 5.0 ||
//       left_lanemarker_points.back().x < length || right_lanemarker_points.size() < 3 ||
//       right_lanemarker_points.front().x > 5.0 || right_lanemarker_points.back().x < length) {
//     return false;
//   }
//   // 检查heading
//   auto start_p = lane_points.front();
//   auto end_p = lane_points.back();
//   byd::common::math::Vec2d vec(end_p.x - start_p.x,end_p.y - start_p.y);
//   if(std::abs(vec.Angle()) > 0.5){
//     FLOG_CROSS << " ego lane angle: " << vec.Angle();
//     return false;
//   }
//   // to avoid wrong ego lane
//   for (auto it = lane_points.begin();it!=lane_points.end();it++)
//   {
//     if (it->x>0)
//     {
//       if (fabs(it->y)>2)
//       {
//         return false;
//       }
//     }
//   }
//   return true;
// }
void CrossDataManager::FindCrossRoadFlag() {
  double dis{1000.0};
  if (FindStopLine(dis)) {
    FLOG_CROSS << " FindStopLine(dis): " << dis;
    cross_road_dis_ = dis;
  }
  if (FindCrossWalk(dis)) {
    FLOG_CROSS << " FindCrossWalk(dis): " << dis;
    cross_road_dis_ = std::min(cross_road_dis_, dis);
  }
  if (FindJunction(dis)) {
    FLOG_CROSS << " FindJunction(dis): " << dis;
    cross_road_dis_ = std::min(cross_road_dis_, dis);
  }
  FLOG_CROSS << " FindCrossRoadFlag_cross_road_dis: " << cross_road_dis_;
}

bool CrossDataManager::FindStopLine(double &dis) {
  for (const auto &stop_lane : bev_map_raw_->stop_lines) {
    if (stop_lane.line_points.size() < 2) {
      continue;
    }
    if (stop_lane.line_points.front().x < kThresholdToSDIntersection && stop_lane.line_points.front().x > 0) {
      has_bev_stop_lane_ = true;
      dis                = stop_lane.line_points.front().x;
      return true;
    }
  }
  return false;
}

bool CrossDataManager::FindCrossWalk(double& dis) {
  for (const auto &crosswalk : bev_map_raw_->crosswalks) {
    const auto& points = crosswalk.line_points;
    if (points.size() < 4) {
      continue;
    }
    // 求斑马线的方向（一般短边就是斑马线方向）,将方向大于45度的斑马线过滤掉
    int min_index = 0;
    double min_dis_square = DBL_MAX;
    double average_y = points.front().y;
    for (int i = 1; i < points.size(); i++) {
      // FLOG_CROSS << " crosswalk_point_x: " << point.x << " ,y: " << point.y;
      double distance = (points[i].x - points[0].x) * (points[i].x - points[0].x) + (points[i].y - points[0].y) * (points[i].y - points[0].y);
      if (distance < min_dis_square) {
        min_dis_square   = distance;
        min_index = i;
      }
      average_y += points[i].y;
    }
    average_y /= points.size();
    double angle = Vec2d(points[0].x - points[min_index].x,points[0].y - points[min_index].y).Angle();
    if (std::abs(angle) > pi / 2) {
      angle = std::abs(angle) - pi;
    }
    if(std::abs(angle) > pi/4){
      continue;
    }

    double min_dis{1000};
    for (const auto &point : points) {
      // FLOG_CROSS << " crosswalk_point_x: " << point.x << " ,y: " << point.y;
      if (point.x < min_dis && (std::abs(point.y) < 8.0 || std::abs(average_y) < 8.0)) {
        min_dis = point.x;
      }
    }
    if (min_dis < kThresholdToSDIntersection && min_dis > 0) {
      has_bev_cross_walk_ = true;
      dis                 = min_dis;
      return true;
    }
  }
  return false;
}

bool CrossDataManager::FindJunction(double &dis) {
  // for (const auto &junction : bev_map_raw_->junctions) {
  //   if (junction.line_points.size() < 4) {
  //     continue;
  //   }
  //   if (junction.line_points.front().x < kThresholdToSDIntersection && junction.line_points.front().x >0) {
  //     has_bev_junction_ = true;
  //     break;
  //   }
  // }
  if (sd_section_points_.has_intersection_zone && sd_section_points_.start_point.x() < kThresholdToSDIntersection &&
      sd_section_points_.start_point.x() > 0) {
    has_bev_junction_ = true;
    dis               = sd_section_points_.start_point.x();
    return true;
  }
  return false;
}

void CrossDataManager::DealEgoRoad() {
  FLOG_CROSS << " DealEgoRoad";
  if (!has_ego_lane_ || cross_road_dis_ <= 25 || is_in_cross_road_) {
    return;
  }
  auto ego_lane_id  = dr_ego_lane_.id;
  auto ego_road_id  = dr_ego_lane_.road_id;
  auto left_lane_id = ego_lane_id;
  auto next_lane_id = dr_ego_lane_.left_lane_id;
  int  index        = 0;
  while (next_lane_id != 0 && bev_lane_unmap_.count(next_lane_id) > 0 && index < 10) {
    const auto *next_lane = bev_lane_unmap_[next_lane_id];
    if (next_lane->road_id == ego_road_id) {
      left_lane_id = next_lane_id;
      next_lane_id = next_lane->left_lane_id;
    } else {
      break;
    }
    index++;
  }
  index = 0;
  std::vector<uint64_t> ego_road_lane_ids;
  while (left_lane_id != 0 && bev_lane_unmap_.count(left_lane_id) > 0 && index < 10) {
    const auto *next_lane = bev_lane_unmap_[left_lane_id];
    if (next_lane->road_id == ego_road_id) {
      ego_road_lane_ids.emplace_back(left_lane_id);
      left_lane_id = next_lane->right_lane_id;
    } else {
      break;
    }
    index++;
  }
  FLOG_CROSS << " ego_road_lane_ids: " << ego_road_lane_ids.size();
  for (auto id : ego_road_lane_ids) {
    const auto *lane = bev_lane_unmap_[id];
    FLOG_CROSS << "lane id: " << id << " , plan_turn: " << EnumToString(lane->plan_turn_type);
  }

  std::vector<uint64_t> same_turn_lane_ids;
  if (ego_turn_type_ == BevAction::STRAIGHT) {
    for (auto id : ego_road_lane_ids) {
      const auto *lane = bev_lane_unmap_[id];
      if (lane->plan_turn_type == BevTurnType::NO_TURN || lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_RIGHT_TURN || lane->plan_turn_type == BevTurnType::STRAIGHT_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN) {
        same_turn_lane_ids.emplace_back(id);
      }
    }
  } else if (ego_turn_type_ == BevAction::LEFT_TURN) {
    for (auto id : ego_road_lane_ids) {
      const auto *lane = bev_lane_unmap_[id];
      if (lane->plan_turn_type == BevTurnType::LEFT_TURN || lane->plan_turn_type == BevTurnType::LEFT_TURN_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::LEFT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN_AND_U_TURN) {
        same_turn_lane_ids.emplace_back(id);
      }
    }
  } else if (ego_turn_type_ == BevAction::RIGHT_TURN) {
    for (auto id : ego_road_lane_ids) {
      const auto *lane = bev_lane_unmap_[id];
      if (lane->plan_turn_type == BevTurnType::RIGHT_TURN || lane->plan_turn_type == BevTurnType::STRAIGHT_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::RIGHT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::LEFT_TURN_AND_RIGHT_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_RIGHT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN ||
          lane->plan_turn_type == BevTurnType::STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN) {
        same_turn_lane_ids.emplace_back(id);
      }
    }
  }

  int lane_size = same_turn_lane_ids.size();
  int ego_lane_index{0};
  for (int i = 0; i < lane_size; i++) {
    if (same_turn_lane_ids[i] == ego_lane_id) {
      ego_lane_index = i;
      break;
    }
  }
  ego_road_lanes_.ego_road_id          = ego_road_id;
  ego_road_lanes_.ego_lane_id          = ego_lane_id;
  ego_road_lanes_.geo_navi_action      = ego_turn_type_;
  ego_road_lanes_.lane_ids             = ego_road_lane_ids;
  ego_road_lanes_.same_turn_lane_ids   = same_turn_lane_ids;
  ego_road_lanes_.left_same_turn_size  = ego_lane_index;
  ego_road_lanes_.right_same_turn_size = std::max(lane_size - 1 - ego_lane_index, 0);
  ego_road_lanes_.is_one_lane          = lane_size == 1;

  // log
  for (int i = 0; i < lane_size; i++) {
    FLOG_CROSS << "same_turn_lane_id: " << same_turn_lane_ids[i];
  }
  FLOG_CROSS << "same lane_size: " << lane_size << " ,left_same_turn_size: " << ego_road_lanes_.left_same_turn_size
             << " ,right_same_turn_size: " << ego_road_lanes_.right_same_turn_size;
}

float CrossDataManager::GetLocDeltaHeading() {
  return (currect_loc_ == nullptr || last_loc_ == nullptr)
             ? 0.0
             : std::atan(std::tan(currect_loc_->attitude_dr * pi / 180 - last_loc_->attitude_dr * pi / 180));
}

byd::common::math::Vec2d CrossDataManager::TransformPointVec(const byd::common::math::Vec2d &point,
  const Eigen::Isometry3d &rotate_translate_matrix) {
byd::common::math::Vec2d point_out;
Eigen::Vector3d point_before_dr(point.x(), point.y(), 0);
Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
point_out.set_x(point_after_dr[0]);
point_out.set_y(point_after_dr[1]);
return point_out;
}

void CrossDataManager::DealCrossOccBoundary(){
  if (cross_state_ == CrossState::INIT || cross_state_ == CrossState::END) {
    return;
  }
  // 预推状态下，左右转等到路口前30m内再调整
  if (cross_state_ == CrossState::PREDICTION && (ego_turn_type_ == BevAction::LEFT_TURN || ego_turn_type_ == BevAction::RIGHT_TURN) &&
      sd_section_points_.start_point.x() > 30.0) {
    return;
  }
  occ_sd_edges_info_.Clear();
  // 处理sd插值和垂向量
  std::vector<Vec2d> sd_points;
  if(!GetSdEdgePoint(sd_points)){
    FLOG_CROSS << " GetSdEdgePoint faild";
    return;
  }
  // 计算sd与occ在各点的边界
  auto & sd_edge_points  = occ_sd_edges_info_.sd_edge_points;
  double max_main_left_x = sd_section_points_.start_point.x();
  GetSdOccEdges(sd_edge_points, max_main_left_x, sd_points);

  if (ego_turn_type_ == BevAction::LEFT_TURN) {
    occ_sd_edges_info_.max_main_left_x = max_main_left_x;
  }

  // 遍历寻找每一个occ的左右边界 
  std::vector<OccEdge> occ_edges;
  if(!GetOccEdges(occ_edges)){
    FLOG_CROSS << " GetOccEdges faild";
    return;
  }

  // 路沿排序等处理
  if(!DealOccEdges(occ_edges)){
    FLOG_CROSS << " DealOccEdges faild";
    return;
  }
  
  // 右转只有一个右侧occ路沿处理
  if (ego_turn_type_ == BevAction::RIGHT_TURN && occ_edges.back().max_left_dis < 0.0 && occ_edges.size() == 1 &&
      occ_edges.back().occ_points.size() >= 3) {
    occ_sd_edges_info_.top_left_min_distance  = 100.0;
    occ_sd_edges_info_.top_right_min_distance = occ_edges.back().max_left_dis;
    occ_sd_edges_info_.is_valid               = true;
    FLOG_CROSS << " has only one occ_edge RIGHT_TURN";
    return;
  }

  if(occ_edges.size() < 2){
    FLOG_CROSS << " has only one occ_edge!!!";
    return;
  }

  // 寻找要行驶的两个edge
  if(!GetOpening(occ_edges)){
    FLOG_CROSS << " GetOpening faild";
    return;
  }
  // 修改sd的toplane交点
  sd_section_points_.sd_end_points.emplace_back(sd_section_points_.end_point);
  sd_section_points_.sd_end_points.emplace_back(sd_section_points_.top_vector);
  if (occ_sd_edges_info_.is_valid &&
      ((occ_sd_edges_info_.top_left_min_distance > 0.0 && occ_sd_edges_info_.top_right_min_distance > 0.0) ||
       (occ_sd_edges_info_.top_left_min_distance < 0.0 && occ_sd_edges_info_.top_right_min_distance < 0.0))) {
    Vec2d axis = sd_section_points_.top_vector - sd_section_points_.end_point;
    axis.Normalize();
    axis.SelfRotate(pi / 2);
    double move_dis = (occ_sd_edges_info_.top_left_min_distance + occ_sd_edges_info_.top_right_min_distance) / 2;
    FLOG_CROSS << " sd_end_points_move_dis:" << move_dis;
    sd_section_points_.sd_end_points = TranslateSegmentAlongAxisStart(
        axis, sd_section_points_.sd_end_points, move_dis);
  }
  FLOG_CROSS << " occ_sd_edges_info_.is_valid: " << occ_sd_edges_info_.is_valid
             << " ,top_left_min_distance: " << occ_sd_edges_info_.top_left_min_distance
             << " ,top_right_min_distance: " << occ_sd_edges_info_.top_right_min_distance
             << " ,max_main_left_x:" << occ_sd_edges_info_.max_main_left_x;
  // 寻找sd的最近边界
  // for (int i = 0; i < sd_edge_points.size(); i++){
  //   auto& sd_edge_point = sd_edge_points[i];
  //   for(int j = 0;j < sd_edge_point.occ_points.size();j++){
  //     if(sd_edge_point.occ_points[j].right_distance > mid_distance){
  //       if(j == 0){
  //         sd_edge_point.left_distance = sd_edge_point.occ_points[j].right_distance;
  //       }else{
  //         sd_edge_point.left_distance = sd_edge_point.occ_points[j].right_distance;
  //         sd_edge_point.right_distance = sd_edge_point.occ_points[j - 1].left_distance;
  //       }
  //       break;
  //     }
  //   }
  // }
  
  return;
}

bool CrossDataManager::GetSdEdgePoint(std::vector<Vec2d>& sd_points) {
  const auto &sd_section_points = sd_section_points_.points;
  if (edgesPtr_.empty() || sd_section_points.size() <= sd_section_points_.start_sd_point_index) {
    FLOG_CROSS << "has no occ edge or sd section!!! ";
    return false;
  }
  // for (auto [id, points] : edgesPtr_) {
  //   if (points == nullptr) {
  //     continue;
  //   }
  //   FLOG_CROSS << "edge id: " << id << " size: " << points->size();
  //   for (const auto &point : *points) {
  //     FLOG_CROSS << "edge_x: " << point.x << " ,y: " << point.y;
  //   }
  // }
  sd_points   = InterpolateVec2dPoints(sd_section_points, 2.0);
  int                start_index = 0;
  int                end_index   = sd_points.size();
  for (int i = 0; i < sd_points.size(); i++) {
    if (sd_points[i].x() > 0.0 && start_index == 0) {
      start_index = i;
    }
    if (sd_points[i].x() > 100.0 || std::abs(sd_points[i].y()) > 30.0) {
      end_index = i;
      break;
    }
  }
  sd_points.assign(sd_points.begin() + start_index, sd_points.begin() + end_index);
  // 寻找sd的左右边界
  FLOG_CROSS << " after insert sd_points: " << sd_points.size();
  if (sd_points.size() < 2) {
    return false;
  }
  // for (int i = 0; i < sd_points.size(); i++) {
  //   FLOG_CROSS << "sd_points[" << i << "]_x: " << sd_points[i].x() << " ,y: " << sd_points[i].y();
  // }
  auto &sd_edge_points = occ_sd_edges_info_.sd_edge_points;
  sd_edge_points.reserve(sd_points.size());
  for (int i = 0; i < sd_points.size() - 1; i++) {
    if (occ_sd_edges_info_.end_point_sd_index == 0 && sd_section_points_.end_point.DistanceSquareTo(sd_points[i]) < 2) {
      occ_sd_edges_info_.end_point_sd_index = i;
    }
    auto   axis            = sd_points[i + 1] - sd_points[i];
    double angle           = axis.Angle();
    bool   is_same_heading = true;
    if (!sd_edge_points.empty() && std::abs(angle - sd_edge_points.back().heading) > 0.5) {
      is_same_heading = false;
    }
    auto &sd_edge_point           = sd_edge_points.emplace_back();
    sd_edge_point.sd_point        = sd_points[i];
    sd_edge_point.heading         = angle;
    sd_edge_point.is_same_heading = is_same_heading;
    sd_edge_point.right_point     = Vec2d(sd_points[i].x() + axis.y(), sd_points[i].y() - axis.x());
    // FLOG_CROSS << " sd_edge_points[" << i << "] x: " << sd_points[i].x() << " ,y: " << sd_points[i].y() << " , heading: " << angle
    //            << " , is_same_heading: " << is_same_heading;
  }
  FLOG_CROSS << " end_point_sd_index: " << occ_sd_edges_info_.end_point_sd_index;
  return true;
}

bool CrossDataManager::GetSdOccEdges(std::vector<SdCrossOccEdgePoint> &sd_edge_points,double& max_main_left_x, const std::vector<Vec2d>& sd_points){
  // 计算sd与occ在各点的边界
  for (const auto &[occ_id, occ_points] : edgesPtr_) {
    if (occ_points == nullptr || occ_points->size() < 3) {
      continue;
    }
    // 寻找左侧近距离路沿位置
    if ((occ_points->front().x < sd_section_points_.start_point.x() + 20.0 && occ_points->front().y < 4.5 && occ_points->front().y > 0.0) ||
        (occ_points->back().x < sd_section_points_.start_point.x() + 20.0 && occ_points->back().y < 4.5 && occ_points->back().y > 0.0)) {
      for (const auto &point : *occ_points) {
        if (point.x < sd_section_points_.start_point.x() + 20.0 && point.y < 4.5 && point.x > max_main_left_x) {
          max_main_left_x = point.x;
        }
      }
    }

    int sd_point_index = 0;
    for (; sd_point_index < sd_edge_points.size(); sd_point_index++) {
      double start_dis =
          PointToVectorDist(sd_edge_points[sd_point_index].sd_point, sd_edge_points[sd_point_index].right_point, occ_points->front());
      bool   start_is_right = PointInVectorSide(sd_edge_points[sd_point_index].sd_point, sd_edge_points[sd_point_index].right_point,
                                              occ_points->front()) >= 0.0;
      double end_dis =
          PointToVectorDist(sd_edge_points[sd_point_index].sd_point, sd_edge_points[sd_point_index].right_point, occ_points->back());
      bool end_is_right =
          PointInVectorSide(sd_edge_points[sd_point_index].sd_point, sd_edge_points[sd_point_index].right_point, occ_points->back()) >= 0.0;
      // 起点和终点都不在范围内就寻找下一个点
      if (((start_is_right && end_is_right) || (!start_is_right && !end_is_right)) && start_dis > 1.0 && end_dis > 1.0) {
        continue;
      }
      double min_dis   = std::numeric_limits<double>::max();
      double max_dis   = std::numeric_limits<double>::lowest();
      int    occ_min_index = -1;
      int    occ_max_index = -1;
      for (int occ_point_index = 0; occ_point_index < occ_points->size(); occ_point_index++) {
        double dis = PointToVectorDist(sd_edge_points[sd_point_index].sd_point, sd_edge_points[sd_point_index].right_point,
                                       occ_points->at(occ_point_index));
        if (dis > 1.0) {
          continue;
        }
        double lat_dis =
            PointToVectorDist(sd_edge_points[sd_point_index].sd_point, sd_points[sd_point_index + 1], occ_points->at(occ_point_index));
        if (lat_dis < min_dis) {
          min_dis   = lat_dis;
          occ_min_index = occ_point_index;
        }
        if(lat_dis > max_dis){
          max_dis   = lat_dis;
          occ_max_index = occ_point_index;
        }
      }

      if (occ_min_index > -1 && occ_max_index > -1) {
        bool min_is_right =
            PointInVectorSide(sd_edge_points[sd_point_index].sd_point, sd_points[sd_point_index + 1], occ_points->at(occ_min_index)) >= 0.0;
        if (min_is_right) {
          min_dis = -min_dis;
        }
        bool max_is_right =
            PointInVectorSide(sd_edge_points[sd_point_index].sd_point, sd_points[sd_point_index + 1], occ_points->at(occ_max_index)) >= 0.0;
        if (max_is_right) {
          max_dis = -max_dis;
        }
        auto &occ_point = sd_edge_points[sd_point_index].occ_points.emplace_back();
        if (min_dis > max_dis) {
          occ_point.left_distance  = min_dis;
          occ_point.right_distance = max_dis;
          occ_point.occ_left_index  = occ_min_index;
          occ_point.occ_right_index  = occ_max_index;
        } else {
          occ_point.left_distance  = max_dis;
          occ_point.right_distance = min_dis;
          occ_point.occ_left_index  = occ_max_index;
          occ_point.occ_right_index  = occ_min_index;
        }
        occ_point.occ_edge_id = occ_id;
        occ_point.occ_point   = occ_points->at(occ_min_index);
      }
    }
  }
  return true;
}

bool CrossDataManager::GetOccEdges(std::vector<OccEdge> &occ_edges) {
  // 寻找路口的转折点，如果没有就从路口面点往前10m
  auto & sd_edge_points  = occ_sd_edges_info_.sd_edge_points;
  if (sd_edge_points.empty()) {
    return false;
  }
  int last_is_diff_heading = 0;
  for (int i = sd_edge_points.size() - 1; i >= 0; i--) {
    if (!sd_edge_points[i].is_same_heading) {
      last_is_diff_heading = std::min(i + 1, int(sd_edge_points.size() - 1));
      break;
    }
  }
  if (occ_sd_edges_info_.end_point_sd_index == 0) {
    FLOG_CROSS << "end_point_sd_index == 0!";
    return false;
  }
  // 开始点必须是路口转折后的点,取交点往后10m,往前20m
  int start_top_sd_index = std::max(0, occ_sd_edges_info_.end_point_sd_index - 10);
  start_top_sd_index     = std::max(last_is_diff_heading, start_top_sd_index);
  int end_top_sd_index   = std::min(int(sd_edge_points.size()) - 1, occ_sd_edges_info_.end_point_sd_index + 10);

  FLOG_CROSS << " last_is_diff_heading: " << last_is_diff_heading << " ,start_top_sd_index: " << start_top_sd_index
             << " ,end_top_sd_index: " << end_top_sd_index;
  // 遍历寻找每一个occ的左右边界
  for (int i = start_top_sd_index; i < end_top_sd_index; i++) {
    auto &sd_edge_point = sd_edge_points[i];
    // FLOG_CROSS << " sd_edge_pointsindex: " << i << " ,sd_edge_point.occ_points.size: " << sd_edge_point.occ_points.size();
    // if (sd_edge_point.occ_points.size() > 1) {
    //   std::sort(sd_edge_point.occ_points.begin(), sd_edge_point.occ_points.end(),
    //             [&](OccEdgePoint a, OccEdgePoint b) { return Cross_Compare(a.left_distance, b.left_distance) < 0; });
    // }
    if (occ_edges.empty()) {
      for (const auto &point : sd_edge_point.occ_points) {
        FLOG_CROSS << "occdebe_id: " << point.occ_edge_id << " ,left_distance: " << point.left_distance
                   << " ,right_dis: " << point.right_distance;
        auto &occ_edge       = occ_edges.emplace_back();
        occ_edge.occ_edge_id = point.occ_edge_id;
        occ_edge.occ_points.emplace_back(point.occ_point);
        occ_edge.left_distances.emplace_back(point.left_distance);
        occ_edge.right_distances.emplace_back(point.right_distance);
        occ_edge.sd_indexs.emplace_back(i);
      }
    } else {
      for (const auto &point : sd_edge_point.occ_points) {
        FLOG_CROSS << "occedge_id: " << point.occ_edge_id << " ,left_distance: " << point.left_distance
                   << " ,right_dis: " << point.right_distance;
        auto it = std::find_if(occ_edges.begin(), occ_edges.end(), [&](const OccEdge &a) { return a.occ_edge_id == point.occ_edge_id; });
        if (it == occ_edges.end()) {
          auto &occ_edge       = occ_edges.emplace_back();
          occ_edge.occ_edge_id = point.occ_edge_id;
          occ_edge.occ_points.emplace_back(point.occ_point);
          occ_edge.left_distances.emplace_back(point.left_distance);
          occ_edge.right_distances.emplace_back(point.right_distance);
          occ_edge.sd_indexs.emplace_back(i);
        } else {
          it->occ_points.emplace_back(point.occ_point);
          it->left_distances.emplace_back(point.left_distance);
          it->right_distances.emplace_back(point.right_distance);
          it->sd_indexs.emplace_back(i);
        }
      }
    }
  }
  // 遍历将漏掉的同一occ id的再次添加进来
  for (int i = last_is_diff_heading; i < sd_edge_points.size(); i++) {
    auto &sd_edge_point = sd_edge_points[i];
    for (int j = 0; j < sd_edge_point.occ_points.size(); j++) {
      auto id = sd_edge_point.occ_points[j].occ_edge_id;
      auto it = std::find_if(occ_edges.begin(), occ_edges.end(), [id](const OccEdge &occ_edge) { return occ_edge.occ_edge_id == id; });
      if (it != occ_edges.end()) {
        // FLOG_CROSS << "add occedge_id: " << it->occ_edge_id << " ,left_distance: " << sd_edge_point.occ_points[j].left_distance
        //            << " ,right_dis: " << sd_edge_point.occ_points[j].right_distance;
        it->occ_points.emplace_back(sd_edge_point.occ_points[j].occ_point);
        it->left_distances.emplace_back(sd_edge_point.occ_points[j].left_distance);
        it->right_distances.emplace_back(sd_edge_point.occ_points[j].right_distance);
        it->sd_indexs.emplace_back(i);
      }
    }
  }
  return true;
}

bool CrossDataManager::DealOccEdges(std::vector<OccEdge> &occ_edges) {
  // 遍历求出每个occ路沿的distance最大最小值
  for (auto &occ_edge : occ_edges) {
    double max_left  = std::numeric_limits<double>::lowest();
    double min_right = std::numeric_limits<double>::max();
    for (int i = 0; i < occ_edge.left_distances.size(); i++) {
      if (occ_edge.left_distances[i] > max_left) {
        max_left = occ_edge.left_distances[i];
      }
      if (occ_edge.right_distances[i] < min_right) {
        min_right = occ_edge.right_distances[i];
      }
    }
    occ_edge.max_left_dis  = max_left;
    occ_edge.min_right_dis = min_right;
    // FLOG_CROSS << " before occ_edge_id: " << occ_edge.occ_edge_id << " ,size: " << occ_edge.sd_indexs.size() << " ,max_left: " << max_left
    //            << " ,min_right: " << min_right;
  }
  // 将edge按照从右到左排序
  for (int i = 0; i < occ_edges.size(); i++) {
    int min = i;
    for (int j = i + 1; j < occ_edges.size(); j++) {
      if (occ_edges[j].min_right_dis < occ_edges[min].min_right_dis) {
        min = j;
      }
    }
    if (min != i) {
      std::swap(occ_edges[i], occ_edges[min]);
    }
  }
  FLOG_CROSS << " occ_edges: " << occ_edges.size();

  // 只有一个edge就退出
  if (occ_edges.empty() || (occ_edges.size() < 2 && ego_turn_type_ != BevAction::RIGHT_TURN)) {
    FLOG_CROSS << " has only one occ_edge!!!";
    return false;
  }

  // 处理重合的路沿
  if (occ_edges.size() > 1) {
    std::vector<uint64_t> erase_edges;
    for (int i = 0; i < occ_edges.size() - 1; i++) {
      if (occ_edges[i].max_left_dis > occ_edges[i + 1].min_right_dis) {
        occ_edges[i + 1].max_left_dis  = std::max(occ_edges[i].max_left_dis, occ_edges[i + 1].max_left_dis);
        occ_edges[i + 1].min_right_dis = std::min(occ_edges[i].min_right_dis, occ_edges[i + 1].min_right_dis);
        erase_edges.emplace_back(occ_edges[i].occ_edge_id);
      }
    }
    for (auto it = occ_edges.begin(); it != occ_edges.end();) {
      bool has_edge = false;
      for (int i = 0; i < erase_edges.size(); i++) {
        if (erase_edges[i] == it->occ_edge_id) {
          has_edge = true;
        }
      }
      if (has_edge) {
        it = occ_edges.erase(it);
      } else {
        it++;
      }
    }
  }

  FLOG_CROSS << " occ_edges: " << occ_edges.size();
  for (const auto &occ_edge : occ_edges) {
    FLOG_CROSS << " after delete occ_edge_id: " << occ_edge.occ_edge_id << " ,min_right_dis: " << occ_edge.min_right_dis
               << " ,max_left_dis: " << occ_edge.max_left_dis << " ,point_size: " << occ_edge.occ_points.size();
  }

  // for (int i = 0; i < occ_edges.size(); i++) {
  //   for (int j = 0; j < occ_edges[i].sd_indexs.size(); j++) {
  //     FLOG_CROSS << " tocc_edges_id: " << occ_edges[i].occ_edge_id << " ,sd_index: " << occ_edges[i].sd_indexs[j]
  //                << " ,left_distance: " << occ_edges[i].left_distances[j] << " ,right_distance: " << occ_edges[i].right_distances[j];
  //   }
  // }
  return !occ_edges.empty();
}

bool CrossDataManager::GetOpening(std::vector<OccEdge> &occ_edges) {
  double                  mid_distance = 0;
  std::vector<OccOpening> occ_openings;
  for (int i = 0; i < occ_edges.size() - 1; i++) {
    if (occ_edges[i].occ_points.size() < 1) {
      continue;
    }
    double min_lanewidth      = -1.0;
    double min_left_distance  = 1001.0;
    double min_right_distance = -1001.0;
    min_right_distance        = occ_edges[i].max_left_dis;
    min_left_distance         = occ_edges[i + 1].min_right_dis;
    min_lanewidth             = std::abs(min_right_distance - min_left_distance);
    mid_distance              = (min_right_distance + min_left_distance) / 2;
    FLOG_CROSS << " occ_index: " << i << " ,mid_distance: " << mid_distance << " ,min_lanewidth: " << min_lanewidth
               << " ,min_left_distance: " << min_left_distance << " , min_right_distance: " << min_right_distance;
    //&& min_right_distance > sd_section_points_.top_cross2sd_right_dis &&
    // min_left_distance < sd_section_points_.top_cross2sd_left_dis

    // 使用宽度和路口面过滤
    if (min_lanewidth > 5.0) {
      auto &occ_opening              = occ_openings.emplace_back();
      occ_opening.occ_index          = i;
      occ_opening.mid_distance       = mid_distance;
      occ_opening.min_lanewidth      = min_lanewidth;
      occ_opening.min_left_distance  = min_left_distance;
      occ_opening.min_right_distance = min_right_distance;
      occ_opening.left_valid         = occ_edges[i].occ_points.size() < 3 ? false : true;
      occ_opening.right_valid        = occ_edges[i].occ_points.size() < 3 ? false : true;
    }
  }
  FLOG_CROSS << "top_cross2sd_right_dis: " << sd_section_points_.top_cross2sd_right_dis
             << " ,top_cross2sd_left_dis: " << sd_section_points_.top_cross2sd_left_dis;
  // 再进行openings选择
  double road_width       = sd_section_points_.current_lane_num * 3.5;
  double occ_fit_distance = sd_section_points_.top_cross2sd_right_dis +
                            (sd_section_points_.top_cross2sd_left_dis - sd_section_points_.top_cross2sd_right_dis) * 0.33;
  double cross_mid_distance = sd_section_points_.top_cross2sd_right_dis +
                              (sd_section_points_.top_cross2sd_left_dis - sd_section_points_.top_cross2sd_right_dis) * 0.5;
  FLOG_CROSS << " occ_fit_distance: " << occ_fit_distance << " ,cross_mid_distance: " << cross_mid_distance;
  if (occ_openings.empty()) {
    FLOG_CROSS << " occ_openings.empty()";
    return false;
  } else {
    bool has_occ_opening = false;
    for (int i = 0; i < occ_openings.size(); i++) {
      if (occ_openings[i].min_left_distance > 1.0 && occ_openings[i].min_right_distance < -1.0) {
        has_occ_opening = true;
        // 如果宽度不合适，sd还在occ路沿中间，之间返回错误，按照sd去生成预推线；宽度合适了再调整
        if ((road_width < occ_openings[i].min_lanewidth + 3.5 && std::abs(occ_openings[i].min_right_distance) < road_width &&
             road_width + 8.0 > occ_openings[i].min_lanewidth) ||
            (ego_turn_type_ == BevAction::RIGHT_TURN && occ_openings[i].min_right_distance + 5.0 > sd_section_points_.top_cross2sd_right_dis)) {
          occ_sd_edges_info_.top_left_min_distance  = occ_openings[i].min_left_distance;
          occ_sd_edges_info_.top_right_min_distance = occ_openings[i].min_right_distance;
          occ_sd_edges_info_.is_valid               = occ_openings[i].left_valid && occ_openings[i].right_valid;
        }
        break;
      }
    }
    // 如果不能直接找到满足条件的，需要寻找左右侧的是否合适
    if (!has_occ_opening) {
      double min_diff_dis = 1000.0;
      int    fit_index    = -1;
      for (int i = 0; i < occ_openings.size(); i++) {
        double diff = std::abs(occ_openings[i].mid_distance - occ_fit_distance);
        if (diff < min_diff_dis) {
          min_diff_dis = diff;
          fit_index    = i;
        }
      }
      if (fit_index >= 0 && occ_openings[fit_index].mid_distance < cross_mid_distance &&
          (std::abs(occ_openings[fit_index].min_lanewidth - road_width) < 4.0 ||
           std::abs(occ_openings[fit_index].min_lanewidth - 2.0 * road_width) < 4.0) &&
          (occ_openings[fit_index].mid_distance > sd_section_points_.top_cross2sd_right_dis &&
           occ_openings[fit_index].mid_distance < sd_section_points_.top_cross2sd_left_dis)) {
        occ_sd_edges_info_.top_left_min_distance  = occ_openings[fit_index].min_left_distance;
        occ_sd_edges_info_.top_right_min_distance = occ_openings[fit_index].min_right_distance;
        occ_sd_edges_info_.is_valid               = true;
      }
    }
  }
  return true;
}

void CrossDataManager::DealCrossBoundary() {
  if (cross_state_ == CrossState::INIT || cross_state_ == CrossState::END) {
    return;
  }
  const auto &road_edges        = bev_map_->edges;
  const auto &sd_section_points = sd_section_points_.points;
  if (road_edges.empty() || sd_section_points.size() <= sd_section_points_.start_sd_point_index) {
    return;
  }
  if (!is_in_cross_road_) {
    double sd_angle = (sd_section_points.back() - sd_section_points[sd_section_points_.start_sd_point_index]).Angle();
    is_boundary_left_  = ComparedToZero(sd_angle) >= 0;
    FLOG_CROSS << " sd_angle: " << sd_angle << " ,is_left: " << is_boundary_left_;
  }
  FLOG_CROSS << "is_left: " << is_boundary_left_;
  // for (int i = sd_section_points_.start_sd_point_index; i < sd_section_points.size(); i++) {
  //   FLOG_CROSS << " sd_section_points__x: " << sd_section_points[i].x() << " ,y: " << sd_section_points[i].y();
  // }

  left_edges_.clear();
  right_edges_.clear();
  for (const auto &edge : road_edges) {
    // FLOG_CROSS << " **********edge_id: " << edge.id << " ,edge.line_points_size: " << edge.line_points.size();
    bool      edge_is_in_left{false};
    bool      is_success{false};
    CrossEdge cross_edge;
    cross_edge.id = edge.id;
    auto &points  = cross_edge.points;
    points.reserve(edge.line_points.size());
    double edge_length = 0.0;
    for (const auto &point : edge.line_points) {
      auto pt = TransformPoint(point, Tbw_);
      if (pt.x < 0.0) {
        continue;
      }
      // 第一次需要先判断这个边界的其中一个点在哪边
      if (!is_success) {
        if (is_boundary_left_) {
          edge_is_in_left = true;
          for (int i = sd_section_points_.start_sd_point_index + 1; i < sd_section_points.size(); i++) {
            if (PointInVectorSide(sd_section_points[i - 1], sd_section_points[i], pt) > 0) {
              edge_is_in_left = false;
              break;
            }
          }
        } else {
          for (int i = sd_section_points_.start_sd_point_index + 1; i < sd_section_points.size(); i++) {
            if (PointInVectorSide(sd_section_points[i - 1], sd_section_points[i], pt) < 0) {
              edge_is_in_left = true;
              break;
            }
          }
        }
        is_success = true;
      }

      if (edge_is_in_left) {
        // FLOG_CROSS << " left_boundary_points_x: " << pt.x << " ,y: " << pt.y;
        points.emplace_back(pt.x, pt.y);
      } else {
        // FLOG_CROSS << " right_boundary_points_x: " << pt.x << " ,y: " << pt.y;
        points.emplace_back(pt.x, pt.y);
      }
      if(points.size() > 1){
        edge_length += points[points.size() - 2].DistanceTo(points.back());
      }
    }
    if(edge_length < 5.0){
      // FLOG_CROSS << " edge_length: " << edge_length;
      continue;
    }

    if (edge_is_in_left) {
      left_edges_.emplace_back(std::move(cross_edge));
    } else {
      right_edges_.emplace_back(std::move(cross_edge));
    }
    if(edge_length < 5.0){
      // FLOG_CROSS << " edge_length: " << edge_length;
      continue;
    }

    if (edge_is_in_left) {
      left_edges_.emplace_back(std::move(cross_edge));
    } else {
      right_edges_.emplace_back(std::move(cross_edge));
    }
  }
}

void CrossDataManager::DealBoundary(const std::vector<Vec2d> &points) {
  if (cross_state_ == CrossState::INIT || cross_state_ == CrossState::END) {
    return;
  }
  boundary_kdtrees_.clear();
  const auto &road_edges        = bev_map_->edges;
  const auto &sd_section_points = sd_section_points_.points;
  if (road_edges.empty() || sd_section_points.size() < 2) {
    return;
  }
  double                   sd_angle = (sd_section_points.back() - sd_section_points.front()).Angle();
  bool                     is_left  = ComparedToZero(sd_angle) >= 0;
  std::vector<cv::Point2f> left_kdtree_points;
  std::vector<cv::Point2f> right_kdtree_points;
  // for(float i = -5.01;i < 10.01;i += 1.001){
  //   tp_points.emplace_back(i,-2.5f);
  // }
  std::vector<cv::Point2f> kdtree_points;
  for (const auto &edge : road_edges) {
    FLOG_CROSS << " **********edge_id***************: " << edge.id;
    for (const auto &point : edge.line_points) {
      auto pt = TransformPoint(point, Tbw_);
      kdtree_points.emplace_back(pt.x, pt.y);
      // FLOG_CROSS << " pt.x: " << pt.x << " ,y: " << pt.y;
    }
  }

  if (kdtree_points.empty()) {
    FLOG_CROSS << " kdtree_points.empty()";
    return;
  }
  cv::flann::KDTreeIndexParams index_param(1);
  cv::Mat                      dataMat(kdtree_points.size(), 2, CV_32F,kdtree_points.data());
  // for (size_t i = 0; i < kdtree_points.size(); ++i) {
  //   dataMat.at<float>(i, 0) = kdtree_points[i].x;
  //   dataMat.at<float>(i, 1) = kdtree_points[i].y;
  // }

  for (int i = 0; i < dataMat.rows; ++i) {
    float x = dataMat.at<float>(i, 0);
    float y = dataMat.at<float>(i, 1);
    // FLOG_CROSS << "KDTree Point[" << i << "]: (" << x << ", " << y << ")";
  }

  if (!dataMat.isContinuous()) {
    FLOG_CROSS << "dataMat.isContinuous";
  }
  if (dataMat.type() != CV_32F || dataMat.cols != 2) {
    FLOG_CROSS << "dataMat.type err";
}
  std::shared_ptr<cv::flann::Index> kdtree = std::make_shared<cv::flann::Index>(dataMat, index_param);
  if (kdtree == nullptr) {
    FLOG_CROSS << " right_boundary_kdtree_ == nullptr";
    return;
  }

// 在构建KDTree后添加测试代码
  std::vector<int>   indices(1);
  std::vector<float> dists(1);
  for (const auto &point : points) {
    float   query_data[2] = {float(point.x()), float(point.y())};
    cv::Mat queryMat(1, 2, CV_32F, query_data);
    FLOG_CROSS << "Query point: (" << query_data[0] << ", " << query_data[1] << ")";
    // 验证查询矩阵有效性
    if (queryMat.empty() || queryMat.rows != 1) {
      continue;
    }
    kdtree->knnSearch(queryMat, indices, dists, 1);  // 搜索最近邻
    if (indices[0] < 0) {
      FLOG_CROSS << "KDTree build faild!!!";
    } else {
      FLOG_CROSS << "index: " << indices[0] << ", point: (" << kdtree_points[indices[0]].x << ", "
                 << kdtree_points[indices[0]].y << ") dists: " << std::sqrt(dists[0]);
    }
  }
}

bool CrossDataManager::FindVirtualLane() {
  // 非mnoa下不运行
  if (!is_enable_mnoa) {
    return false;
  }
  if (routing_map_raw_ == nullptr) {
    return false;
  }

  const auto *          lane = FindRoutingMapRawLaneById(ld_ego_lane_id_);
  if (lane == nullptr || lane->points.empty() || lane->points.back().x < 0.0 || is_in_cross_road_) {
    return false;
  }
  FLOG_CROSS << " lane->points.back().x: " << lane->points.back().x << " ,id: " << lane->id;
  double lane_length = lane->points.back().x;
  if (lane_length > 80.0) {
    return false;
  }
  auto next_lane_id_size = lane->next_lane_ids.size();
  if (next_lane_id_size == 0) {
    return false;
  }
  bool next_lane_is_virtual = false;
  for (const auto id : lane->next_lane_ids) {
    const auto *next_lane = FindRoutingMapRawLaneById(id);
    if (next_lane == nullptr) {
      continue;
    }
    FLOG_CROSS << " next_laneid: " << next_lane->id << " ,is_virtual: " << next_lane->is_virtual;
    if (next_lane->is_virtual) {
      next_lane_is_virtual = true;
      // ego_turn_type_       = TurnType2Action(next_lane->turn_type);
      break;
    }
  }
  FLOG_CROSS << " next_lane_id_size: " << next_lane_id_size << " ,next_lane_is_virtual: " << next_lane_is_virtual;
  if (next_lane_id_size > 1 && !next_lane_is_virtual) {
    return false;
  }
  if (next_lane_is_virtual) {
    return true;
  }

  int      index          = 0;
  uint64_t lane_id        = lane->next_lane_ids.front();
  bool     has_is_virtual = false;
  while (lane_length < 80.0 && index < 100) {
    index++;
    const auto *lane = FindRoutingMapRawLaneById(lane_id);
    if (lane == nullptr) {
      break;
    }
    lane_length += lane->length;
    if (lane->next_lane_ids.empty() || lane_length > 80.0) {
      break;
    }

    next_lane_is_virtual = false;
    for (const auto id : lane->next_lane_ids) {
      const auto *next_lane = FindRoutingMapRawLaneById(id);
      if (next_lane == nullptr) {
        continue;
      }
      if (next_lane->is_virtual) {
        // ego_turn_type_       = TurnType2Action(next_lane->turn_type);
        next_lane_is_virtual = true;
      }
    }
    if (next_lane_is_virtual) {
      has_is_virtual = true;
      break;
    }
    if (lane->next_lane_ids.size() > 1 && !next_lane_is_virtual) {
      break;
    }
    lane_id = lane->next_lane_ids.front();
  }
  return has_is_virtual;
}

bool CrossDataManager::CrossRoadVirtualLaneCheck() {
  // 只在init下检测
  if(cross_state_ != CrossState::INIT){
    return true;
  }
  // 第一个路口junction，lane没有is_virtual的就返回，不认为是路口
  bool cross_lane_is_virtual = false;
  bool has_section_junction  = false;
  for (const auto &section : routing_map_->route.sections) {
    for (const auto lane_id : section.lane_ids) {
      if (routing_lane_unmap_.find(lane_id) == routing_lane_unmap_.end()) {
        continue;
      }
      const auto *lane = routing_lane_unmap_[lane_id];
      if (lane->type == message::env_model::LaneType::LANE_VIRTUAL_JUNCTION && lane->junction_id != 0) {
        has_section_junction = true;
        if (lane->is_virtual) {
          cross_lane_is_virtual = true;
          break;
        }
      }
    }
    if (has_section_junction) {
      break;
    }
  }
  first_junction_lane_is_virtual_ = cross_lane_is_virtual;
  return first_junction_lane_is_virtual_;
}

const cem::message::env_model::LaneInfo *CrossDataManager::FindRoutingMapRawLaneById(uint64_t id) {
  if (routing_map_raw_ == nullptr) {
    return nullptr;
  }
  for (const auto &lane : routing_map_raw_->lanes) {
    if (lane.id == id) {
      return &lane;
    }
  }
  return nullptr;
}

const cem::message::env_model::LaneInfo *CrossDataManager::FindRoutingMapLaneById(uint64_t id) {
  if (routing_map_ == nullptr) {
    return nullptr;
  }
  for (const auto &lane : routing_map_->lanes) {
    if (lane.id == id) {
      return &lane;
    }
  }
  return nullptr;
}

BevAction CrossDataManager::TurnType2Action(cem::message::env_model::TurnType turn_type) {
  BevAction action = BevAction::STRAIGHT;
  if (turn_type == cem::message::env_model::TurnType::RIGHT_TURN) {
    action = BevAction::RIGHT_TURN;
  } else if (turn_type == cem::message::env_model::TurnType::LEFT_TURN) {
    action = BevAction::LEFT_TURN;
  } else if (turn_type == cem::message::env_model::TurnType::U_TURN) {
    action = BevAction::U_TURN;
  }
  return action;
}

double CrossDataManager::FindNearstLeftBoundaryDis(const Vec2d &point) {
  if (boundary_kdtrees_.empty()) {
    return 100.0;
  }
  double min_dis{100000000.0};
  for (const auto &[id, kd_tree] : boundary_kdtrees_) {
    int                dim = 1;
    std::vector<int>   nearest_index(dim, -1);
    std::vector<float> nearest_dis(dim,FLT_MAX);
    std::vector<float> query_point = std::vector<float>{0.0f, 0.0f};
    if (kd_tree == nullptr) {
      continue;
    }
    kd_tree->knnSearch(query_point, nearest_index, nearest_dis, dim, cv::flann::SearchParams(-1));
    if (nearest_index.size() < 1) {
      continue;
    }
    FLOG_CROSS << " boundary_id: " << id << " ,nearest_dis: " << nearest_dis.front() << " ,nearest_index: " << nearest_index.front();
    if (nearest_dis.front() < min_dis) {
      min_dis = nearest_dis.front();
    }
  }
  return min_dis;
}

bool CrossDataManager::SdHasEepTrajEndLaneid(uint64_t input_id) {
  const auto &route      = routing_map_->route;
  const auto &navi_start = route.navi_start;
  const auto  it_navi_seciton =
      std::find_if(route.sections.begin(), route.sections.end(),
                   [&navi_start](const cem::message::env_model::SectionInfo &p) { return p.id == navi_start.section_id; });
  if (it_navi_seciton == route.sections.end() || route.sections.empty()) {
    return false;
  }
  double navi_seciton_length = it_navi_seciton->length - navi_start.s_offset;
  for (auto it = it_navi_seciton; it != route.sections.end(); it++) {
    for (const auto id : it->lane_ids) {
      if (id == input_id) {
        return true;
      }
    }
    if (navi_seciton_length > 300.0) {
      break;
    }
    navi_seciton_length += it->length;
  }
  return false;
}

double CrossDataManager::FindNearstRightBoundaryDis(const Vec2d &point) {
  if (right_boundary_kdtree_ == nullptr) {
    FLOG_CROSS << " right_boundary_kdtree_ == nullptr";
    return 100.0;
  }
      int k = 5; // 查找最近5个点
    cv::Mat indices(k, 1, CV_32S); // 存储索引
    cv::Mat dists(k, 1, CV_32F);  // 存储距离
  std::vector<float> query_point = std::vector<float>{0.0f, 0.0f};
  right_boundary_kdtree_->knnSearch(query_point, indices, dists, k, cv::flann::SearchParams(-1));
  // if (indices.col < 1) {
  //   return 100.0;
  // }
  FLOG_CROSS << "nearest_dis: " << indices.at<int>(0,0) << " ,nearest_index: " << dists.at<float>(0,0);
  return 100.0;;

  // const auto &road_edges        = bev_map_->edges;
  // if (road_edges.empty()) {
  //   return 100.0;
  // }
  //   const auto &             ego_lian_points = road_edges.front().line_points;
  // std::vector<cv::Point2f> kdtree_points;
  // for (const auto &point : ego_lian_points) {
  //   auto pt = TransformPoint(point, Tbw_);
  //   FLOG_CROSS << " ego_lian_points_x: " << pt.x << " ,y: " << pt.y;
  //   kdtree_points.emplace_back(pt.x, pt.y);
  // }
  // if (kdtree_points.size() < 2) {
  //   return 100.0;
  // }
  // cv::flann::KDTreeIndexParams index_param(1);
  // right_boundary_kdtree_  = std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1), index_param);
  // int                dim = 1;
  // std::vector<int>   nearest_index(dim);
  // std::vector<float> nearest_dis(dim);
  // std::vector<float> query_point = std::vector<float>{static_cast<float>(point.x()), static_cast<float>(point.y())};
  // right_boundary_kdtree_->knnSearch(query_point, nearest_index, nearest_dis, dim, cv::flann::SearchParams(32));
  // if (nearest_index.size() < 1) {
  //   return 100.0;
  // }
  // FLOG_CROSS << " nearest_dis: " << nearest_dis.front() << " ,nearest_index: " << nearest_index.front();
  // return nearest_dis.front();
}

std::pair<double, double> CrossDataManager::FindTopCrossPointDis(const std::vector<Point2DF> &bev_cross_points, Vec2d A, Vec2d B) {
  double left_max_dis  = -1000.0;
  double right_max_dis = 1000.0;
  for (const auto &point : bev_cross_points) {
    double dis = PointToVectorDist(A, B, point);
    if (PointInVectorSide(A, B, point) >= 0.0) {
      if (-dis < right_max_dis) {
        right_max_dis = -dis;
      }
    } else {
      if (dis > left_max_dis) {
        left_max_dis = dis;
      }
    }
  }
  return {left_max_dis, right_max_dis};
}

}  // namespace fusion
}  // namespace cem