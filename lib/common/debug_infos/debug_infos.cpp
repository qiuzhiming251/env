#include "debug_infos.h"
#include "lib/common/utility.h"
using namespace cem::fusion;
namespace cem {
namespace fusion {

    void DebugInfos::SetBEVMapDebugInfo(BevMapInfoPtr GlobalBevMapOutPut, Eigen::Isometry3d T_local_ego_){
        if (GlobalBevMapOutPut == nullptr) {
            return;
        }
        const float CloseLaneDisThreshold = 4.2; 
        json bev_map_debug_json;
        // 判断车道集合是否为空
        bev_map_debug_json["bev_lanes_empty"] = false;
        if (GlobalBevMapOutPut->lane_infos.empty()){
            bev_map_debug_json["bev_lanes_empty"] = true;
        }

        // 判断ego_lane是否存在
        std::vector<int> ego_lane_inds;
        std::vector<uint64_t> ego_lane_ids;
        bev_map_debug_json["bev_ego_lane_exist"] = false;
        for (int i = 0; i < GlobalBevMapOutPut->lane_infos.size(); ++i) {
            auto &lane = GlobalBevMapOutPut->lane_infos[i];
            if (lane.line_points.size() < 2) {
                continue;
            }
            if (lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
                bev_map_debug_json["bev_ego_lane_exist"] = true;
                ego_lane_inds.emplace_back(i); 
                ego_lane_ids.emplace_back(lane.id); 
            }
        }

        // 通过先前遍历得到的ego_lane_inds，遍历可能的ego_lane，并获取其剩余长度
        // 如果ego_lane_inds为空，则进入分支求解
        std::vector<double> ego_lane_lengths;
        std::vector<uint64_t> closest_ego_lane_ids;
        auto T_ego_local_ = T_local_ego_.inverse();
        if (!ego_lane_inds.empty()){
            for (const auto& ind : ego_lane_inds){
            auto ego_lane = GlobalBevMapOutPut->lane_infos[ind];
            double ego_lane_length = 0.0;
            double adjust_length = 0.0;
            int start_ind = -1; 
            // 跳过之前一段PointSource不为0，1，2，3的点源
            for (int i = ego_lane.line_points.size() - 1; i > 0; i--) {
                auto point_ed = ego_lane.line_points[i];
                // 如果点源是未知、BEV、拓扑插入或路段扩展，则跳出循环
                if (point_ed.point_source == cem::message::common::PointSource::PS_UNKNOWN ||
                    point_ed.point_source == cem::message::common::PointSource::PS_BEV     || 
                    point_ed.point_source == cem::message::common::PointSource::TOPO_INSETRT ||
                    point_ed.point_source == cem::message::common::PointSource::SECTION_EXTENTED_FROM_LANEMARKER) {
                    start_ind = i; 
                    break; 
                }
            }
            // 从后向前计算ego_lane的剩余长度，并记录第一个点在ego坐标系下的x值
            for (int i = start_ind; i > 0; i--) {
                auto point_ed = ego_lane.line_points[i];
                auto point_st = ego_lane.line_points[i-1];
                double temp_length = static_cast<double>(CalculatePoint2PointDist(point_st, point_ed)); 
                ego_lane_length += temp_length; 
                Eigen::Vector3d line_point_st_ego;
                Eigen::Vector3d line_point_st;
                // Eigen::Vector3d line_point_ed;
                // Eigen::Vector3d line_point_ed_ego;
                line_point_st.x() = point_st.x;
                line_point_st.y() = point_st.y;
                line_point_st_ego = T_ego_local_ * line_point_st;
                // line_point_ed.x() = point_ed.x;
                // line_point_ed.y() = point_ed.y;
                // line_point_ed_ego = T_ego_local_ * line_point_ed;
                if (line_point_st_ego.x() < 0){
                    // AINFO << "****Timestamp: " << std::fixed << std::setprecision(3) << GlobalBevMapOutPut->header.timestamp;
                    // AINFO << "****point_st_local: (" << line_point_st.x() << ", " << line_point_st.y() << ")";
                    // AINFO << "****point_st_ego: (" << line_point_st_ego.x() << ", " << line_point_st_ego.y() << ")";
                    // AINFO << "****point_ed_local: (" << line_point_ed.x() << ", " << line_point_ed.y() << ")";
                    // AINFO << "****point_ed_ego: (" << line_point_ed_ego.x() << ", " << line_point_ed_ego.y() << ")";
                    adjust_length = abs(line_point_st_ego.x());
                    break;
                }
            }
            // 计算ego_lane的长度，并减去调整长度 
            // TODO: 这里可能存在ego_lane_length不准的情况，自车位置可能落在端点的最后端和最前端，落在最后端会导致ego_lane_length<0，落在最前端会导致计算的ego_lane_length<实际长度
            ego_lane_length -= adjust_length;
            ego_lane_lengths.emplace_back(ego_lane_length);
            }
        } else {
            for (const auto &lane : GlobalBevMapOutPut->lane_infos) {
            if (lane.line_points.size() < 2) {
                continue;
            }
            // 根据当前lane的起始点终止点否成线段
            auto lane_seg_st = lane.line_points.front();
            auto lane_seg_ed = lane.line_points.back();
            cem::message::common::Point2DF ego_pos; 
            ego_pos.x = T_local_ego_.translation().x(); 
            ego_pos.y = T_local_ego_.translation().y(); 
            float dist_to_seg = CalculatePoint2SegmentDist(ego_pos, lane_seg_st, lane_seg_ed);
            if (dist_to_seg < CloseLaneDisThreshold){
                closest_ego_lane_ids.emplace_back(lane.id);
            }
            }
        }
        
        bev_map_debug_json["ego_lane_ids"] = ego_lane_ids;
        bev_map_debug_json["ego_lane_lengths"] = ego_lane_lengths;
        bev_map_debug_json["closest_lane_inds_no_ego"] = closest_ego_lane_ids;
        std::string debug_str = bev_map_debug_json.dump();
        std::string field = "{\"bev_ego_lane_exist\"";
        size_t field_ind = GlobalBevMapOutPut->debug_infos.find(field);
        // AINFO << "field_ind: " << field_ind;
        if (field_ind != std::string::npos) {
            GlobalBevMapOutPut->debug_infos = GlobalBevMapOutPut->debug_infos.substr(0, field_ind);
        } 
        GlobalBevMapOutPut->debug_infos += debug_str;
        // AINFO << "********Debug_Test_BEV******";
        // AINFO << GlobalBevMapOutPut->debug_infos; 
        // AINFO << "Timestamp: " << std::fixed << std::setprecision(3) << GlobalBevMapOutPut->header.timestamp;
        // AINFO << "****************************";
    }

    void DebugInfos::SetLDMapDebugInfo(RoutingMapPtr GlobalLDMapOutPut, BevMapInfoPtr GlobalBevMapOutPut){
        if (GlobalLDMapOutPut == nullptr) {
            return;
        }
        json ld_map_debug_json;
        ld_map_debug_json["ld_lanes_empty"] = false;
        if (GlobalLDMapOutPut->lanes.empty()){
            ld_map_debug_json["ld_lanes_empty"] = true;
        }

        // ld_map_debug_json["ld_ego_lane_exist"] = false;
        // for (auto &lane : GlobalLDMapOutPut->lanes) {
        //   // if (lane.points.size() < 2) {
        //   //   continue;
        //   // }
        //   if (lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO)) {
        //     ld_map_debug_json["ld_ego_lane_exist"] = true;
        //     break;
        //   }
        // }

        std::string debug_str = ld_map_debug_json.dump();
        if(GlobalBevMapOutPut == nullptr){
            //  "{\"ld_lanes_empty\""字段不能重复
            std::string field = "{\"ld_lanes_empty\"";
            size_t field_ind = GlobalLDMapOutPut->debug_infos.find(field);
            // AINFO << "field_ind: " << field_ind;
            if (field_ind != std::string::npos) {
                GlobalLDMapOutPut->debug_infos = GlobalLDMapOutPut->debug_infos.substr(0, field_ind);
            } 
            GlobalLDMapOutPut->debug_infos += debug_str;
        } else{
            // 保证输出格式为前半段为BEV感知的debug_info后面才是关于ld_map相关的debug_info
            GlobalLDMapOutPut->debug_infos = "";
            //  "{\"bev_ego_lane_exist\""和"{\"ld_lanes_empty\""字段不能共存
            std::string field = "{\"bev_ego_lane_exist\"";
            size_t field_ind = GlobalBevMapOutPut->debug_infos.find(field);
            if (field_ind != std::string::npos) {
                GlobalBevMapOutPut->debug_infos = GlobalBevMapOutPut->debug_infos.substr(0, field_ind);
            } 
            GlobalLDMapOutPut->debug_infos += (GlobalBevMapOutPut->debug_infos + debug_str);
        }
        
        // AINFO << "********Debug_Test******";
        // AINFO << GlobalLDMapOutPut->debug_infos; 
        // AINFO << "************************";
    }

    void DebugInfos::SetDebugInfo(const bool& used_ld_map, const BevMapInfoPtr& bev_map_info, Eigen::Isometry3d T_local_ego_, 
                                  const RoutingMapPtr& ld_map_info){
        if (!used_ld_map){
            SetBEVMapDebugInfo(bev_map_info, T_local_ego_);
        } else {
            SetLDMapDebugInfo(ld_map_info, bev_map_info);
        }
    }

}
}