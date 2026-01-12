#include "messenger.h"
#include "lib/common/function_timer.h"

namespace cem {
namespace fusion {
Messenger::Messenger() {
    smooth_state_.last_is_local_map = false;
    smooth_state_.is_transitioning = false;
    smooth_state_.transition_frames = 0;
    smooth_state_.last_map_lane = std::make_shared<byd::msg::orin::routing_map::RoutingMap>();
    smooth_state_.last_perception_lane = std::make_shared<byd::msg::orin::routing_map::RoutingMap>();
}

void Messenger::Publish(RoutingMapPtr routingMap, cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info)
{
    AdaptInternal2Output(routingMap, routing_map_, routingMap->is_on_highway, sd_acc_lane_info);
    AINFO << "publish routing Map timestamp " << routingMap->header.timestamp;
}

void Messenger::PulishDiagno()
{
    AdaptInternal2Output(diagno_info_);
}

void Messenger::PulishTrafficLightE2E(const TrafficLightsE2EInfoPtr traffic_lights_e2e)
{
    AdaptInternal2Output(traffic_lights_e2e, traffic_light_e2e_);
}

void Messenger::MapSourceSmoother(bool is_local_map, const MsgRoutingMapPtr routing_map, 
                                 MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info) {
    if (!routing_map || !output_routing_map) {
        return;
    }
    if (routing_map_count_ < 5) {
        routing_map_count_++;
        XLOG << "routing_map_count_ " << routing_map_count_;
        smooth_state_.last_is_local_map = is_local_map;
        return;
    }
    // 检测数据源是否发生变化
    
    bool source_changed = (is_local_map != smooth_state_.last_is_local_map);
    if(bev_map_info){
        XLOG << " is_local_map " << is_local_map << " - " << smooth_state_.last_is_local_map << " source_changed " << source_changed << " " <<std::to_string(bev_map_info->header.timestamp) << " " << bev_map_info->header.sequence_num ;
    }
    
    if (source_changed) {
        smooth_state_.is_transitioning = true;
        smooth_state_.transition_frames = 0;
        
        // // 保存当前状态作为过渡起点
        // if (is_local_map) {
        //     // 地图→感知切换：保存当前地图数据
        //     smooth_state_.last_map_lane = std::make_shared<MsgRoutingMap>(*routing_map);
        // } else {
        //     // 感知→地图切换：保存当前感知数据
        //     smooth_state_.last_perception_lane = std::make_shared<MsgRoutingMap>(*routing_map);
        // }
    }
    
    if (smooth_state_.is_transitioning) {
        // 执行平滑过渡算法
        XLOG << " is_transitioning " << smooth_state_.is_transitioning;
        PerformSmoothTransition(is_local_map, routing_map, output_routing_map,bev_map_info);//todelet  output_routing_map
        smooth_state_.transition_frames++;
        
        if (smooth_state_.transition_frames >= TRANSITION_DURATION) {
            smooth_state_.is_transitioning = false;
        }
    } else {
        // 非过渡状态，直接输出
        *output_routing_map = *routing_map;
    }
    
    smooth_state_.last_is_local_map = is_local_map;

    
    // if (is_local_map) {
    //     smooth_state_.last_perception_lane = std::make_shared<MsgRoutingMap>(*routing_map);
    // } else {
    //     smooth_state_.last_map_lane = std::make_shared<MsgRoutingMap>(*routing_map);
    // }
}

void Messenger::PerformSmoothTransition(bool is_local_map, const MsgRoutingMapPtr current_map, 
                                      MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info) {
    double alpha = static_cast<double>(smooth_state_.transition_frames) / 
                  (TRANSITION_DURATION - 1);
    
    if (is_local_map) {
        // 地图→感知：感知权重从0增加到1
        if (smooth_state_.last_map_lane) {
            XLOG << "SmoothLaneTransition TO BEV1";
            SmoothLaneTransition(smooth_state_.last_map_lane, current_map, alpha, output_routing_map,bev_map_info);
        } else {
            *output_routing_map = *current_map;
        }
    } else {
        // 感知→地图：地图权重从0增加到1
        if (smooth_state_.last_perception_lane) {
            XLOG << "SmoothLaneTransition TO MAP";
            SmoothLaneTransition(smooth_state_.last_perception_lane, current_map, alpha, output_routing_map,bev_map_info);
        } else {
            *output_routing_map = *current_map;
        }
    }
}

void Messenger::SmoothLaneTransition(const MsgRoutingMapPtr source_routing_map, const MsgRoutingMapPtr target_routing_map,
                                   double weight, MsgRoutingMapPtr output_routing_map,BevMapInfoConstPtr bev_map_info) {
    *output_routing_map = *target_routing_map;//全部属性赋值target_routing_map
    auto timestamp = bev_map_info->header.timestamp;
    int temp_time_s = static_cast<int>(timestamp);
    int temp_time_us = static_cast<int>((timestamp-temp_time_s)*1000);
    XLOG<< "///////BEV MAP TIME/////////// : " << std::to_string( timestamp);

    XLOG << "SmoothLaneTransition " ;
    if (!match_pairs_.empty()) {
        LocalizationPtr odom_ptr{nullptr};
        SensorDataManager::Instance()->GetLatestSensorFrame( target_routing_map->header().measurement_timestamp(), 0.05,odom_ptr);
    
        if (odom_ptr == nullptr) {
            return ;
        }
        T_ego_local_ = Eigen::Isometry3d::Identity();
        T_local_ego_ = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd Rwb_0((odom_ptr->attitude_dr)* M_PI / 180.0,
        Eigen::Vector3d(0, 0, 1));
        Eigen::Vector3d trans_0(odom_ptr->posne_dr.at(0),
        odom_ptr->posne_dr.at(1),
        0.0);
        T_local_ego_.rotate(Rwb_0);
        T_local_ego_.pretranslate(trans_0);
        T_ego_local_ = T_local_ego_.inverse();
#if MAP_SMOOTH_DEBUG
        ppm_image_ = std::make_shared<ppm::PPMImage>();
        ppm_image_->DrawNumber(static_cast<int>(temp_time_us), 0.f,  48.0f - 8*2.f  , 2);
        ppm_image_->DrawNumber(static_cast<int>(temp_time_s), 0.f,  48.0f, 2);
#endif
        Points map_points;
        Points bev_points;
        std::vector<std::pair<uint64_t,std::pair<Eigen::Vector3d,Eigen::Vector3d>>> ego_bev_split_points;
        std::vector<std::pair<uint64_t,std::pair<Eigen::Vector3d,Eigen::Vector3d>>> ego_map_split_points;
        const auto &bev_routing_map = is_local_map_? target_routing_map : source_routing_map;
        const auto &map_routing_map = is_local_map_ ? source_routing_map : target_routing_map;
        std::unordered_map<uint64_t, decltype(bev_routing_map->mutable_lanes()->begin())> lane_map;
        for (auto it = bev_routing_map->mutable_lanes()->begin(); it != bev_routing_map->mutable_lanes()->end(); ++it) {
            lane_map[it->id()] = it;
            XLOG << "bev lane id: " << it->id();
        }
        uint64_t ego_lane_id = 0;
        bool     ego_lane_found = false;
        if(bev_map_info){
            for(auto &seecton:bev_map_info->route.sections){
                for(auto &bevLane:seecton.lane_infos){
                    if(bevLane.position == 0){
                        ego_lane_id = bevLane.id;
                        ego_lane_found = true;
                        break;
                    }
                }
                if(ego_lane_found){
                    break;
                }
            }
        }
        XLOG << "found bev ego lane  ID" << ego_lane_id;
        if(!ego_lane_found){
            return;
        }
        auto it_start = lane_map.find(ego_lane_id);
        if(it_start == lane_map.end()){
            return;
        }
        //循环找后继
        int max_count = 5;
        bool empty_points = false;
        while(max_count--){
            auto &points = it_start->second->points();

                            
            for(const auto &pt:points){
                Eigen::Vector3d ptVCS(pt.x(), pt.y(), 0);
                ptVCS = T_ego_local_ * ptVCS;
                XLOG << " BEV " <<ptVCS.x() << " " << ptVCS.y() ;
    #if MAP_SMOOTH_DEBUG
                ppm_image_->DrawPoint(ptVCS.x(), ptVCS.y(), it_start->first%14);
    #endif
                auto *new_point = bev_points.Add();  // 直接使用 Add()
                new_point->set_x(ptVCS[0]);
                new_point->set_y(ptVCS[1]);
            }
            auto len = points.size();
            if( len < 2){
                continue;
            }
            Eigen::Vector3d start_pt(points[0].x(), points[0].y(), 0);
            start_pt = T_ego_local_*start_pt;
            Eigen::Vector3d end_pt(points[len-1].x(), points[len-1].y(), 0);
            end_pt = T_ego_local_*end_pt;
            // XLOG << "BEV ID " << it_start->first << " "<< start_pt.x() << " " << start_pt.y() << " " << end_pt.x() << " " << end_pt.y();
            std::pair<Eigen::Vector3d,Eigen::Vector3d> end_point_pair;
            end_point_pair.first << start_pt.x(), start_pt.y(),0.0f;//全局坐标
            end_point_pair.second << end_pt.x(), end_pt.y(),0.0f;
            ego_bev_split_points.emplace_back(it_start->first, end_point_pair);
            auto  &next_lanes_ids = it_start->second->next_lane_ids();
            if(next_lanes_ids.empty()){
                break;
            }
            auto next_lane_it = lane_map.find(next_lanes_ids[0]);
            Eigen::Vector3d angle(points[points.size()-1].x()-points[0].x(), points[points.size()-1].y()-points[0].y(), 0);
            double angle_degree_1 = std::atan2(angle.y(), angle.x())*180.0/M_PI;
            auto diff = 10000.f;
            for(auto &next_lane_id : next_lanes_ids){
                auto next_lane_it = lane_map.find(next_lane_id);//TODO 选择曲率小的
                if(next_lane_it != lane_map.end()){
                    const auto &next_points = next_lane_it->second->points();
                    if (next_points.size() < 2){//推荐虚拟节点
                        XLOG << "next lane points size < 2 " << next_lane_id;
                        empty_points = true;
                        break;
                    }
                    auto angle_degree_2 = std::atan2(next_points[next_points.size()-1].y()-next_points[0].y(), next_points[next_points.size()-1].x()-next_points[0].x())*180.0/M_PI;
                    if (std::abs(angle_degree_1-angle_degree_2) < diff){
                        diff = std::abs(angle_degree_1-angle_degree_2);
                        XLOG << "diff " << diff << " angle_degree_1 " << angle_degree_1 << " angle_degree_2 " << angle_degree_2 <<" next_lane_id " << next_lane_id;
                        it_start = next_lane_it;
                    }
                }
           }
           if(empty_points){
                XLOG << "empty points " << next_lanes_ids.size();
                break;
           }

        }

        
        {
            double last_x = 0.0;
            double last_y = 0.0;
            bool   last_valid = false;
            bool   large_angle = false;

            for (const  auto &mather : match_pairs_){
                auto ego_it = std::find_if(ego_bev_split_points.begin(), ego_bev_split_points.end(), [&](const auto &lane){
                    return lane.first == mather.bev_id;
                });
                if(ego_it == ego_bev_split_points.end()){
                    XLOG << "NOT EGO BEVLANE";
                    continue;
                }
                for(const auto &ld_seglane : mather.map_id){
                    XLOG << "matcher " << mather.bev_id << "  => " << ld_seglane;
                    auto it = std::find_if(map_routing_map->lanes().begin(), map_routing_map->lanes().end(), [&](const auto &lane){
                        return lane.id()   == ld_seglane;
                    });
                    if(it != map_routing_map->lanes().end()){
                        if((it->split_topology() !=
                                byd::msg::orin::routing_map::LaneInfo_SplitTopology::LaneInfo_SplitTopology_TOPOLOGY_SPLIT_NONE) ||
                            (it->merge_topology() !=
                                byd::msg::orin::routing_map::LaneInfo_MergeTopology::LaneInfo_MergeTopology_TOPOLOGY_MERGE_NONE) ||
                                (it->junction_id() != 0)){
                            XLOG << "NOT NORMAL BEVLANE " << ld_seglane;
                            continue;
                            
                        }
                        XLOG << "MAP ID " << ld_seglane;
                        std::pair<Eigen::Vector3d,Eigen::Vector3d> end_point_pair;

                        int len = it->points().size();
                        if(len < 2){
                            continue;
                        }
                        Eigen::Vector3d  start_pt(it->points()[0].x(), it->points()[0].y(),0.0f);
                        start_pt = T_ego_local_ * start_pt;
                        Eigen::Vector3d  end_pt(it->points()[len - 1].x(), it->points()[len - 1].y(),0.0f);
                        end_pt = T_ego_local_ * end_pt;
                        end_point_pair.first << start_pt.x(), start_pt.y(),0.0f;
                        XLOG << "LD ID "<< it->id()<< "start_pt: " << start_pt.x() << " end_pt: " << end_pt.x();
                        end_point_pair.second << end_pt.x(), end_pt.y(),0.0f;
                        ego_map_split_points.emplace_back(it->id(), end_point_pair);
        
                    }else{
                        XLOG << "ld_seglane.lane_id: " << ld_seglane << " not found in map_routing_map";
                    }
                }
            }
            if(ego_map_split_points.empty()){
                XLOG << "ego_map_split_points is empty";
                return ;
            }
            /////匹配的ID重复 型点的去匹配
            std::vector<std::pair<uint64_t, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> unique;
            std::set<uint64_t> seen_keys;
            for (const auto& item : ego_map_split_points) {
                if (seen_keys.find(item.first) == seen_keys.end()) {
                    seen_keys.insert(item.first);
                    if(!unique.empty() && item.second.first(0) < unique.back().second.second(0)){
                        XLOG << "!!!!!!!!!!!!!!";
                        // return;
                    }
                    unique.push_back(item);
                    // XLOG << " [ " << item.second.first << "," << item.second.second <<" ] ";
                }
            }
            ego_map_split_points = unique;
            //遍历ego_map_split_points 查看区间排序
            
            //ego_map_split_points 按照起始点坐标排序
            std::sort(ego_map_split_points.begin(), ego_map_split_points.end(), [](const auto &a, const auto &b){
                return a.second.first.x() < b.second.first.x();
            });
            for (const auto &ele : ego_map_split_points) {
                XLOG << "ego_map_split_points " << ego_map_split_points.size();
                auto it_map_lane = std::find_if(map_routing_map->lanes().begin(), map_routing_map->lanes().end(), [&](const auto &lane){
                        return lane.id()   == ele.first;
                    });
                    if (it_map_lane != map_routing_map->lanes().end()) {
                        for (const auto &pt : it_map_lane->points()) {
                            Eigen::Vector3d ptVCS(pt.x(), pt.y(), 0);
                            ptVCS = T_ego_local_ * ptVCS;
                            // if(last_valid){
                            //     auto dx = ptVCS[0] - last_x;
                            //     auto dy = ptVCS[1] - last_y;
                            //     if(std::abs(dx) < std::abs(dy)){
                            //         large_angle = true;
                            //         break;
                            //     }
                            // }
                            auto *new_point = map_points.Add();  // 直接使用 Add()
                            new_point->set_x(ptVCS[0]);
                            new_point->set_y(ptVCS[1]);
                            last_valid = true;
                            last_x = ptVCS[0];
                            last_y = ptVCS[1];
                            XLOG << "MAP point " << ptVCS[0] << " " << ptVCS[1] << " " << it_map_lane->id();
    #if MAP_SMOOTH_DEBUG
                        ppm_image_->DrawPoint(ptVCS.x(), ptVCS.y(), 1);
    #endif
                        }
                        // if(large_angle){
                        //     break;

                        // }

        
                    }else{
                        XLOG << "ld_seglane not found in map_routing_map";
                    }
            }
        
        }
        
        const auto& source_pts = is_local_map_ ? map_points :bev_points;//TODO 
        const auto& target_pts = is_local_map_ ? bev_points :map_points;
        std::deque<Point> output_pts;
        // 平滑车道点集
        
        auto ret = SmoothLanePoints(source_pts, target_pts, weight,output_pts);
        if (!ret || output_pts.size() < 2) {
            XLOG << "SmoothLanePoints failed";
            return;//output_routing_map 使用复制值
        }
#if MAP_SMOOTH_DEBUG
  
        ppm_image_->DrawNumber(static_cast<int>(source_pts.size()), 0.f,  48.0f  , 2);
        ppm_image_->DrawNumber(static_cast<int>(target_pts.size()), 2.0f,  48.0f  , 0);
        ppm_image_->DrawNumber(static_cast<int>(output_pts.size()), 4.0f,  48.0f  , 1);
        ppm_image_->DrawNumber(static_cast<int>(smooth_state_.transition_frames), 6.0f,  48.0f  , 7);
        XLOG << "Weight: " << weight;
#endif
        //output_routing_map 修改点集
        const auto& target_split_points = is_local_map_ ? ego_bev_split_points  :ego_map_split_points;
        for (const auto& p : target_split_points) {
            // XLOG << "lane id: " << p.first << " offset: " << p.second.first << " " << p.second.second;
            auto it = std::find_if(output_routing_map->mutable_lanes()->begin(), output_routing_map->mutable_lanes()->end(), [&](const auto &lane){
                return lane.id()   == p.first;
                });
            if (it != output_routing_map->mutable_lanes()->end()) {
                auto lane_pts = it->mutable_points();
                lane_pts->Clear();

            }
        }
        float offset = 0.0;
        Eigen::Vector3d prev(0.0, 0.0, 0.0);
        bool first = true;
        while(!output_pts.empty()) {
            auto point = output_pts.front();
            Eigen::Vector3d ptVCS(point.x(), point.y(), 0);
            output_pts.pop_front();
            if (first) {
                first = false;
                offset = 0.0;
            }else {
                
                offset += std::hypot(point.x() - prev.x(), point.y() - prev.y());
            }
            prev =  Eigen::Vector3d(point.x(), point.y(), point.z());
            //TODO: 查询offset  在p.second.first, p.second.second 中的位置
            int index = 0;
            auto len = target_split_points.size();
            for (const auto& p : target_split_points) {
                // XLOG << " ID " <<  p.first << p.second.first.x() << " p.second.second.x() " << p.second.second.x();
                if (ptVCS.x() >= p.second.first.x() && ptVCS.x() <= p.second.second.x()) {
                    // XLOG << " MIDDLE " << ptVCS[0] << " " << ptVCS[1];

                    auto it = std::find_if(output_routing_map->mutable_lanes()->begin(), output_routing_map->mutable_lanes()->end(), [&](const auto &lane){
                        return lane.id()   == p.first;
                        });
                    if(it != output_routing_map->mutable_lanes()->end()){
                        auto *mutable_points = it->mutable_points();  // 获取可修改的点集
                        // XLOG << "+++ MIDDLE " << ptVCS[0] << " " << ptVCS[1];
#if MAP_SMOOTH_DEBUG
ppm_image_->DrawSig(0,ptVCS[0], ptVCS[1], it->id()%3);
#endif
                            // auto pt = T_local_ego_ * ptVCS;  // 逆变换回全局坐标系
                            auto new_point = mutable_points->Add();
                            new_point->set_x(ptVCS[0]);
                            new_point->set_y(ptVCS[1]);
                            break;
                    }

                }else if (ptVCS.x() < p.second.first.x() && index == 0) {//TODO:如果是首段
                    auto it = std::find_if(output_routing_map->mutable_lanes()->begin(), output_routing_map->mutable_lanes()->end(), [&](const auto &lane){
                                        return lane.id()   == p.first;
                                        });
                    // XLOG << "0000000 " << ptVCS[0] << " " << ptVCS[1];

                    if(it != output_routing_map->mutable_lanes()->end()){
                        auto *mutable_points = it->mutable_points();  // 获取可修改的点集
                        // XLOG << " +++0000000 " << ptVCS[0] << " " << ptVCS[1];
    #if MAP_SMOOTH_DEBUG
    ppm_image_->DrawSig(0,ptVCS[0], ptVCS[1], it->id()%3);
    #endif
                        // auto pt = T_local_ego_ * ptVCS;  // 逆变换回全局坐标系
                        auto new_point = mutable_points->Add();
                        new_point->set_x(ptVCS[0]);
                        new_point->set_y(ptVCS[1]);
                        break;
                    }
                    
                }else if (ptVCS.x() > p.second.second.x() && index== len -1) {//TODO:如果是尾段
                    auto it = std::find_if(output_routing_map->mutable_lanes()->begin(), output_routing_map->mutable_lanes()->end(), [&](const auto &lane){
                                        return lane.id()   == p.first;
                                        });
                    // XLOG << "END  " << ptVCS[0] << " " << ptVCS[1];

                    if(it != output_routing_map->mutable_lanes()->end()){
                        auto *mutable_points = it->mutable_points();  // 获取可修改的点集
                        // XLOG << "+++END  " << ptVCS[0] << " " << ptVCS[1];
    #if MAP_SMOOTH_DEBUG
    ppm_image_->DrawSig(0,ptVCS[0], ptVCS[1], it->id()%3);
    #endif
                        // auto pt = T_local_ego_ * ptVCS;  // 逆变换回全局坐标系
                        auto new_point = mutable_points->Add();
                        new_point->set_x(ptVCS[0]);
                        new_point->set_y(ptVCS[1]);
                        break;
                    
                    }
                }else{
                    // XLOG << " !!!!!Drop " << ptVCS.x() << " " << p.second.first.x() << " " << p.second.second.x() << index;
                }
                index++;
        }
        }
        //  对前后继之间重复 断开的点进行修复 
        FixLaneConnections(output_routing_map,target_split_points);
        //发布转换回去全局坐标
        for (const auto& p : target_split_points) {
            auto it = std::find_if(output_routing_map->mutable_lanes()->begin(), output_routing_map->mutable_lanes()->end(), [&](const auto &lane){
                return lane.id()   == p.first;
                });
            XLOG << "====== ID " << p.first;
            if (it != output_routing_map->mutable_lanes()->end()) {
                auto lane_pts = it->mutable_points();
                for(auto&pt:*lane_pts){
                    Eigen::Vector3d ptVCS(pt.x(), pt.y(), 0);
                    XLOG << "FINAL PTS" << ptVCS[0] << " " << ptVCS[1];
                    ptVCS = T_local_ego_*ptVCS;
                    pt.set_x(ptVCS[0]);
                    pt.set_y(ptVCS[1]);

                }

            }
        }

#if MAP_SMOOTH_DEBUG
        ppm_image_->Save();
        #endif
        XLOG << "MAP_SMOOTH_DEBUG SAVE";

    }

}

bool Messenger::SmoothLanePoints(const Points& source_points,
                               const Points& target_points,
                               double weight, std::deque<Point> &output_points) {
    
    output_points.clear();
    // 1. 检查点数差异是否过大
    size_t min_size = std::min(source_points.size(), target_points.size());
    size_t size_diff = std::max(source_points.size(), target_points.size()) - min_size;
    XLOG << "min_size " << min_size  << "size_diff " << size_diff;
    if (source_points.empty() || target_points.empty() || size_diff > min_size) {
        XLOG << "RETURN";
        return false;
    }
    
    // 重采样并匹配点集
    Points resampled_source, resampled_target;
    ResampleAndMatchPointsByShortCutoff(source_points, target_points, resampled_source, resampled_target);
    
    // 对匹配的点进行插值
    for (size_t i = 0; i < resampled_source.size() && i < resampled_target.size(); ++i) {
        auto &smoothed_point = output_points.emplace_back();
        smoothed_point.set_x( Lerp(resampled_source[i].x(), resampled_target[i].x(), weight));
        smoothed_point.set_y( Lerp(resampled_source[i].y(), resampled_target[i].y(), weight));
        XLOG << "smoothed_point: " << smoothed_point.x() << ", " << smoothed_point.y();
        
    }
#if MAP_SMOOTH_DEBUG
    XLOG << "resampled_source size: " << resampled_source.size() << ", resampled_target size: " << resampled_target.size();
    // for (const auto& point : resampled_target) {
    //     ppm_image_->DrawPoint(point.x(), point.y(), 2);
    // }
    // for (const auto& point : resampled_source) {
    //     ppm_image_->DrawPoint(point.x(), point.y(), 0);
    // }
    // for (const auto& point : output_points) {
    //     ppm_image_->DrawPoint(point.x(), point.y(), 1);
    // }
#endif
    //TODO 增加对smoothed_point合理性的校验
    // 2.smoothed_point出现线的回折  返回false
        // 2. 检查是否有回折
    if (output_points.size() >= 3) {
        for (size_t i = 2; i < output_points.size(); ++i) {
            if (HasSharpTurn(output_points[i-2], output_points[i-1], output_points[i])) {
                XLOG << "Sharp turn detected at point " << i;
                output_points.clear();
                return false;
            }
        }
    }
    return true;
}

void Messenger::ResampleAndMatchPoints(const Points& source_points,
                                     const Points& target_points,
                                     Points& resampled_source,
                                     Points& resampled_target) {
    resampled_source.Clear();
    resampled_target.Clear();
    
    // 使用较大的点集数量作为重采样基准
    size_t sample_count = std::max(source_points.size(), target_points.size());
    sample_count = std::min(sample_count, static_cast<size_t>(100)); // 限制最大点数
    
    for (size_t i = 0; i < sample_count; ++i) {
        double t = static_cast<double>(i) / (sample_count - 1);
        
        // 在源点集中插值
        if (source_points.size() > 1) {
            double source_pos = t * (source_points.size() - 1);
            int idx1 = static_cast<int>(source_pos);
            int idx2 = std::min(idx1 + 1, static_cast<int>(source_points.size() - 1));
            double frac = source_pos - idx1;
            
            byd::msg::orin::routing_map::Point p1 = source_points[idx1];
            byd::msg::orin::routing_map::Point p2 = source_points[idx2];
            auto *pt = resampled_source.Add();
            pt->set_x(InterpolatePoint(p1, p2, frac).x());
            pt->set_y(InterpolatePoint(p1, p2, frac).y());
        } else if (!source_points.empty()) {
            auto *pt = resampled_source.Add();
            pt->set_x(source_points[0].x());
            pt->set_y(source_points[0].y());
        }
        
        // 在目标点集中插值
        if (target_points.size() > 1) {
            double target_pos = t * (target_points.size() - 1);
            int idx1 = static_cast<int>(target_pos);
            int idx2 = std::min(idx1 + 1, static_cast<int>(target_points.size() - 1));
            double frac = target_pos - idx1;

            byd::msg::orin::routing_map::Point p1 = target_points[idx1];
            byd::msg::orin::routing_map::Point p2 = target_points[idx2];
            auto *pt = resampled_target.Add();
            pt->set_x(InterpolatePoint(p1, p2, frac).x());
            pt->set_y(InterpolatePoint(p1, p2, frac).y());
        } else if (!target_points.empty()) {
            auto *pt = resampled_target.Add();
            pt->set_x(target_points[0].x());
            pt->set_y(target_points[0].y());

        }
    }
}

byd::msg::orin::routing_map::Point Messenger::InterpolatePoint(const byd::msg::orin::routing_map::Point& p1, const byd::msg::orin::routing_map::Point& p2, double t) {
    byd::msg::orin::routing_map::Point result;
    result.set_x( p1.x() + (p2.x() - p1.x()) * t);
    result.set_y( p1.y() + (p2.y() - p1.y()) * t);
    // result.z = p1.z + (p2.z - p1.z) * t;
    // result.curvature = p1.curvature + (p2.curvature - p1.curvature) * t;
    return result;
}

void Messenger::SmoothBoundaryPoints(const std::vector<LaneBoundaryInfo>& source_boundaries,
                                   const std::vector<LaneBoundaryInfo>& target_boundaries,
                                   double weight, std::vector<LaneBoundaryInfo>& output_boundaries) {
    output_boundaries = target_boundaries;
    
    for (size_t i = 0; i < output_boundaries.size() && i < source_boundaries.size(); ++i) {
        // 边界点平滑处理（根据实际数据结构实现）
    }
}

void Messenger::PulishFusionMap(bool is_local_map, const BevMapInfoPtr bev_map_info, const RoutingMapPtr routing_map,
                                const std::vector<cem::message::env_model::StopLine> stop_lines_ptr, bool is_on_highway_flag,
                                cem::fusion::navigation::AccLaneInfo  sd_acc_lane_info, const std::string &info) {
    routing_map_->mutable_map_info()->set_engine_version(info);
    is_local_map_ = is_local_map;

    if (is_local_map) {
        //待优化: 整个输出routing map希望在外层统一，地图功能降级后切换使用感知时，也需带上降级信息，规控才能降级。
        SensorStatusInfo ssi;
        if (routing_map != nullptr) {
            ssi.sensor_status = routing_map->sensor_status_info.sensor_status;
            ssi.special_type = routing_map->sensor_status_info.special_type;
            ssi.warning_type = routing_map->sensor_status_info.warning_type;
            AdaptInternal2Output(routing_map, smooth_state_.last_map_lane, is_on_highway_flag, sd_acc_lane_info);
        }
        AdaptInternal2Output(bev_map_info, stop_lines_ptr, ssi, routing_map_, is_on_highway_flag);
    } else {
        if (routing_map == nullptr) {
            return;
        }
        //存感知
        SensorStatusInfo ssi;
        ssi.sensor_status = routing_map->sensor_status_info.sensor_status;
        ssi.special_type = routing_map->sensor_status_info.special_type;
        ssi.warning_type = routing_map->sensor_status_info.warning_type;
        AdaptInternal2Output(bev_map_info, stop_lines_ptr, ssi, smooth_state_.last_perception_lane, is_on_highway_flag);

        AdaptInternal2Output(routing_map, routing_map_, is_on_highway_flag, sd_acc_lane_info);
    }
    XLOG << "current_bev_has_top_ " << current_bev_has_top_ << " timestamp  " <<  std::to_string(routing_map_->header().measurement_timestamp()) ;
    if (!is_on_highway_flag ||  current_bev_has_top_) {//当前帧有拓扑也不做
        return;
    }
    MsgRoutingMapPtr smooth_routing_map_ptr = std::make_shared<byd::msg::orin::routing_map::RoutingMap>(*routing_map_);//复制当前帧
        // 更新状态并保存当前数据
    FunctionTimer::measureTime("MapSourceSmoother", [&]() {MapSourceSmoother(is_local_map, routing_map_, smooth_routing_map_ptr, bev_map_info);});
    if (smooth_state_.is_transitioning) {
        routing_map_ = std::move(smooth_routing_map_ptr);
    }
}


void Messenger::FixLaneConnections(MsgRoutingMapPtr routing_map, 
                                  const std::vector<std::pair<uint64_t,std::pair<Eigen::Vector3d,Eigen::Vector3d>>>& split_points) {
    // 构建车道ID到车道对象的映射
    std::unordered_map<uint64_t, decltype(routing_map->mutable_lanes()->begin())> lane_map;
    for (auto it = routing_map->mutable_lanes()->begin(); it != routing_map->mutable_lanes()->end(); ++it) {
        lane_map[it->id()] = it;
    }
    
    // 修复每个车道的连接点
    bool first_lane = true;
    for (const auto& p : split_points) {
        if(first_lane){
            first_lane = false;
            continue;
        }
        auto current_lane_it = lane_map.find(p.first);
        if (current_lane_it == lane_map.end()) {
            continue;
        }
        
        auto& current_lane = *(current_lane_it->second);
        auto* current_points = current_lane.mutable_points();
        
        if (current_points->size() < 2) {
            continue;
        }
        
        // 修复与前驱车道的连接
        if (!current_lane.previous_lane_ids().empty()) {
            // for (uint64_t prev_lane_id : current_lane.previous_lane_ids()) {
            // auto prev_lane_id_it = current_lane.previous_lane_ids()[0];
            auto prev_lane_id_it = split_points.end();
            for(auto prev_id : current_lane.previous_lane_ids()){
                prev_lane_id_it = std::find_if(split_points.begin(),split_points.end(),[&](const auto& p){return p.first == prev_id;});
                if( prev_lane_id_it != split_points.end()){
                    break;
                }
            }

            auto prev_lane_id = prev_lane_id_it->first;
                auto prev_lane_it = lane_map.find(prev_lane_id);
                if (prev_lane_it != lane_map.end()) {
                    auto& prev_lane = *(prev_lane_it->second);
                    auto* prev_points = prev_lane.mutable_points();
                    
                    if (!prev_points->empty()) {
                        // 获取前驱车道的最后一个点
                        const auto& prev_last_point = prev_points->Get(prev_points->size() - 1);
                        Eigen::Vector3d prev_end(prev_last_point.x(), prev_last_point.y(), 0);
                        
                        // 获取当前车道的第一个点
                        const auto& current_first_point = current_points->Get(0);
                        Eigen::Vector3d current_start(current_first_point.x(), current_first_point.y(), 0);
                        
                        // 计算两点距离（用于辅助判断）
                        double distance = (current_start - prev_end).norm();
                        
                        // 检测断开：前驱终点x坐标远小于当前起点x坐标（存在间隙）
                        bool is_disconnected_x = (current_start.x() - prev_end.x()) > 0.01; // x方向间隙大于2米
                        bool is_overlap_x = prev_end.x() > current_start.x();
                        
                        // 断开条件：x坐标存在明显间隙 且 y方向一致 且 距离较大
                        if (is_disconnected_x ) {
                            XLOG << "修复断开连接: 车道 " << prev_lane_id << " -> " << p.first 
                                 << ", x间隙: " << current_start.x()  <<prev_end.x()
                                 << "米, 距离: " << distance << "米";
                            auto* new_point = prev_points->Add();
                            new_point->set_x(current_start.x());
                            new_point->set_y(current_start.y());
                            
                        } 

                    }
                }
            // }
        }

    }

    //删除点
    if(split_points.empty()){
        return; // 没有匹配对
    }
    const auto &end_pair =   split_points.back();
    XLOG << "END PAIR" << end_pair.first << " " << end_pair.second.second.x() << " " << end_pair.second.second.y();
    auto end_lane_it = lane_map.find(end_pair.first);
    if (end_lane_it == lane_map.end()) {
        return;
    }
    auto& end_lane = *(end_lane_it->second);
    auto* end_points = end_lane.mutable_points();
    //超过的部分删除
    for(auto it =end_points->begin(); it != end_points->end();){
        if(it->x() > end_pair.second.second.x()){
            it = end_points->erase(it);
        }else{
            it++;
        }
    }
    if(end_points->empty()){
        return;
    }
    auto end_point = *(end_points->end() -1);
    Eigen::Vector3d pt(end_point.x(), end_point.y(), 0.0);
    pt = T_local_ego_*pt;
    //后继做  平移
    auto  next_lanes = end_lane.next_lane_ids();
    if(next_lanes.empty()){
        return;
    }
    auto next_lane_it = lane_map.find(next_lanes[0]);//TODO
    if(next_lane_it == lane_map.end() || next_lane_it->second->mutable_points()->empty()){
        return;
    }
    auto start_point = next_lane_it->second->mutable_points()->at(0);
    int max_count = 5;
    while(max_count--){
        auto end_lane_t = *(end_lane_it->second);//从匹配对的末段开始往后
        auto  next_lanes_t = end_lane_t.next_lane_ids();
        if(next_lanes_t.empty()){
            break;
        }
        auto next_lane_it = lane_map.find(next_lanes_t[0]);//如果有两个后继也只处理第一个
        if(next_lane_it == lane_map.end()){
            break;
        }
        auto *next_points = next_lane_it->second->mutable_points();
        //平移
        // XLOG << "后继车道 " << next_lanes[0];

        for(auto it =next_points->begin(); it != next_points->end();++it){
            it->set_x(it->x() - start_point.x() + pt.x());
            it->set_y(it->y() - start_point.y() + pt.y());
            XLOG << "车道平移: " << it->x() << " " << it->y() << " " << pt.y();
        }
        end_lane_it = next_lane_it;
    }
    XLOG << "车道连接修复完成";
}

byd::msg::orin::routing_map::Point Messenger::SamplePointWithCompletion(
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& short_curve,
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& long_curve,
    double target_arc_length, double short_length, double long_length) {
    
    if (short_curve.empty() || long_curve.empty()) {
        return byd::msg::orin::routing_map::Point();
    }
    
    // 如果目标弧长在短曲线长度范围内，正常采样短曲线
    if (target_arc_length <= short_length) {
        return SamplePointByArcLength(short_curve, target_arc_length);
    }
    
    // 如果超出短曲线长度，从长曲线上获取对应点，然后进行平移调整
    double exceed_ratio = (target_arc_length - short_length) / (long_length - short_length);
    
    // 从长曲线上获取对应位置的点
    auto long_curve_point = SamplePointByArcLength(long_curve, target_arc_length);
    
    // 获取短曲线终点和长曲线对应位置的点
    const auto& short_end_point = short_curve[short_curve.size() - 1];
    auto long_curve_short_end = SamplePointByArcLength(long_curve, short_length);
    
    // 计算平移向量：从长曲线在短曲线终点位置的点平移到实际长曲线对应点
    double dx = long_curve_point.x() - long_curve_short_end.x();
    double dy = long_curve_point.y() - long_curve_short_end.y();
    
    // 创建调整后的点：短曲线终点 + 平移向量
    byd::msg::orin::routing_map::Point adjusted_point;
    adjusted_point.set_x(short_end_point.x() + dx);
    adjusted_point.set_y(short_end_point.y() + dy);
    
    return adjusted_point;
}
void Messenger::ResampleAndMatchPointsByShortCutoff(
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& source_points,
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& target_points,
    google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& resampled_source,
    google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& resampled_target) {
    
    resampled_source.Clear();
    resampled_target.Clear();
    
    if (source_points.empty() || target_points.empty()) return;
    
    // 1. 计算两条曲线的总弧长
    double source_arc_length = CalculatePolylineLength(source_points);
    double target_arc_length = CalculatePolylineLength(target_points);
    
    // 2. 确定短曲线的长度（作为截断基准）
    const double short_length = std::min(source_arc_length, target_arc_length);
    const bool source_is_shorter = source_arc_length <= target_arc_length;
    const auto& short_curve = source_is_shorter ? source_points : target_points;
    const auto& long_curve = source_is_shorter ? target_points : source_points;
    
    // 3. 以短曲线长度为基准进行采样
    int sample_count = std::min(100, static_cast<int>(short_curve.size() * 2));
    
    for (int i = 0; i < sample_count; ++i) {
        double t = static_cast<double>(i) / (sample_count - 1);
        double arc_length = t * short_length; // 只采样到短曲线长度
        
        // 4. 在两条曲线上分别采样（都不超过短曲线长度）
        auto short_pt = SamplePointByArcLength(short_curve, arc_length);
        auto long_pt = SamplePointByArcLength(long_curve, arc_length);
        
        // 5. 根据原始顺序分配结果
        if (source_is_shorter) {
            // source是短曲线，target是长曲线
            auto* pt_source = resampled_source.Add();
            pt_source->set_x(short_pt.x());
            pt_source->set_y(short_pt.y());
            
            auto* pt_target = resampled_target.Add();
            pt_target->set_x(long_pt.x());
            pt_target->set_y(long_pt.y());
        } else {
            // target是短曲线，source是长曲线
            auto* pt_source = resampled_source.Add();
            pt_source->set_x(long_pt.x());
            pt_source->set_y(long_pt.y());
            
            auto* pt_target = resampled_target.Add();
            pt_target->set_x(short_pt.x());
            pt_target->set_y(short_pt.y());
        }
    }
}
void Messenger::ResampleAndMatchPointsByArcLengthCompletion(
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& source_points,
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& target_points,
    google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& resampled_source,
    google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& resampled_target) {
    
    resampled_source.Clear();
    resampled_target.Clear();
    
    if (source_points.empty() || target_points.empty()) return;
    
    // 1. 计算两条曲线的总弧长
    double source_arc_length = CalculatePolylineLength(source_points);
    double target_arc_length = CalculatePolylineLength(target_points);
    
    // 2. 确定基准曲线（弧长长的）和需要补齐的曲线（弧长短的）
    const bool source_is_longer = source_arc_length >= target_arc_length;
    const auto& long_curve = source_is_longer ? source_points : target_points;
    const auto& short_curve = source_is_longer ? target_points : source_points;
    const double long_length = source_is_longer ? source_arc_length : target_arc_length;
    const double short_length = source_is_longer ? target_arc_length : source_arc_length;
    
    // 3. 对长曲线进行均匀弧长采样
    int sample_count = std::min(100, static_cast<int>(long_curve.size() * 2));
    
    for (int i = 0; i < sample_count; ++i) {
        double t = static_cast<double>(i) / (sample_count - 1);
        double arc_length = t * long_length;
        
        // 4. 在长曲线上采样
        auto long_pt = SamplePointByArcLength(long_curve, arc_length);
        
        // 5. 在短曲线上采样，如果超出短曲线长度则用平移补齐
        auto short_pt = SamplePointWithCompletion(short_curve, long_curve, arc_length, short_length, long_length);
        
        // 6. 根据原始顺序分配结果
        if (source_is_longer) {
            // source是长曲线，target是短曲线
            auto* pt_source = resampled_source.Add();
            pt_source->set_x(long_pt.x());
            pt_source->set_y(long_pt.y());
            
            auto* pt_target = resampled_target.Add();
            pt_target->set_x(short_pt.x());
            pt_target->set_y(short_pt.y());
        } else {
            // target是长曲线，source是短曲线
            auto* pt_source = resampled_source.Add();
            pt_source->set_x(short_pt.x());
            pt_source->set_y(short_pt.y());
            
            auto* pt_target = resampled_target.Add();
            pt_target->set_x(long_pt.x());
            pt_target->set_y(long_pt.y());
        }
    }
}

double Messenger::CalculatePolylineLength(
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& points) {
    if (points.size() < 2) return 0.0;
    
    double length = 0.0;
    for (int i = 1; i < points.size(); ++i) {
        double dx = points[i].x() - points[i-1].x();
        double dy = points[i].y() - points[i-1].y();
        length += std::sqrt(dx*dx + dy*dy);
    }
    return length;
}

byd::msg::orin::routing_map::Point Messenger::SamplePointByArcLength(
    const google::protobuf::RepeatedPtrField<byd::msg::orin::routing_map::Point>& points, 
    double target_length) {
    
    if (points.empty()) return byd::msg::orin::routing_map::Point();
    if (points.size() == 1) return points[0];
    
    double accumulated_length = 0.0;
    for (int i = 1; i < points.size(); ++i) {
        double dx = points[i].x() - points[i-1].x();
        double dy = points[i].y() - points[i-1].y();
        double segment_length = std::sqrt(dx*dx + dy*dy);
        
        if (accumulated_length + segment_length >= target_length) {
            double frac = (target_length - accumulated_length) / segment_length;
            return InterpolatePoint(points[i-1], points[i], frac);
        }
        accumulated_length += segment_length;
    }
    
    return points[points.size() - 1];
}
bool Messenger::HasSharpTurn(const Point& p1, const Point& p2, const Point& p3, 
                           double max_angle_deg) {
    // 计算向量
    double v1x = p2.x() - p1.x();
    double v1y = p2.y() - p1.y();
    double v2x = p3.x() - p2.x();
    double v2y = p3.y() - p2.y();
    
    // 计算向量长度
    double len1 = std::hypot(v1x, v1y);
    double len2 = std::hypot(v2x, v2y);
    
    if (len1 < 1e-6 || len2 < 1e-6) {
        return false; // 点太近，不算回折
    }
    
    // 计算点积
    double dot = v1x * v2x + v1y * v2y;
    
    // 计算角度
    double cos_angle = dot / (len1 * len2);
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    double angle_deg = std::acos(cos_angle) * 180.0 / M_PI;
    
    return angle_deg < (180.0 - max_angle_deg); // 角度小于阈值认为是锐角回折
}
} // namespace fusion
} // namespace cem
