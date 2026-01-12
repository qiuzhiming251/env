#include "lib/perception_and_ld_map_fusion/data_preprocessor/build_lane_topology.h"


using cem::message::common::Point2DF;
using cem::message::sensor::BevArrowType;
using cem::message::sensor::BevLaneInfo;
using cem::message::sensor::BevLaneMarker;
using cem::message::sensor::BevLaneMarkerType;
using cem::message::sensor::BevLanePosition;
using cem::message::sensor::BevMapInfo;

namespace cem {
namespace fusion {

BuildLaneTopology::BuildLaneTopology()
{

}
BuildLaneTopology::~BuildLaneTopology()
{

}
bool BuildLaneTopology::isICCmode()
{
    CAN1Ptr canoutptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(canoutptr);
    if (canoutptr != nullptr)
    {
        if (canoutptr->DNP_Stats_S == 6)
        {
            is_ICC_mode = false;
        }
        else
        {
            is_ICC_mode = true;
        }
    }
    return true;
}
float BuildLaneTopology::GetCurrentSpeed()
{
    Ibs20msPdu15Ptr ibs20ms_pdu15_ptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(ibs20ms_pdu15_ptr);
    if (ibs20ms_pdu15_ptr != nullptr)
    {
        current_speed = ibs20ms_pdu15_ptr->ivehspdavg / 3.6f;
    }
    return current_speed;
}
void BuildLaneTopology::TransPoint(Eigen::Vector3d & P0, const Eigen::Isometry3d &rotate_translate_matrix)
{
    Eigen::Vector3d point_before_dr(P0(0), P0(1), 0);
    Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
    P0(0) = point_after_dr[0];
    P0(1) = point_after_dr[1];
}
Eigen::Isometry3d BuildLaneTopology::DR2BodyTrans(const double &timestamp)
{
    LocalizationPtr odomT0Ptr{nullptr};
    LocalizationPtr odomT1Ptr{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(timestamp, 0.05,odomT0Ptr);

    if (odomT0Ptr == nullptr)
    {
        return Eigen::Isometry3d::Identity();
    }

    Eigen::Isometry3d Twb_0 = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd Rwb_0((odomT0Ptr->attitude_dr) * M_PI / 180.0,Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d trans_0(odomT0Ptr->posne_dr.at(0), odomT0Ptr->posne_dr.at(1),0.0);
    Twb_0.rotate(Rwb_0);
    Twb_0.pretranslate(trans_0);

    return Twb_0;
}

Eigen::Vector2f BuildLaneTopology::getStartPointInDR(const double &Tn, const Lane_ID_Storage & storage)
{
    Eigen::Vector2f PointN;
    Eigen::Isometry3d Twb_n = DR2BodyTrans(Tn).inverse();
    Eigen::Vector3d pts;
    pts(0) = storage.P0(0);
    pts(1) = storage.P0(1);
    pts(2) = 0;
    TransPoint(pts, Twb_n);

    PointN(0) = pts(0);
    PointN(1) = pts(1);

    return PointN;
}

uint32_t BuildLaneTopology::generateNewLaneID(BevMapInfoPtr local_map)
{
    uint32_t new_lane_id = 99;

    for (uint32_t k = 99; k >= 1; --k)
    {
        auto tmp = std::find_if(local_map->lane_infos.begin(),
                                local_map->lane_infos.end(),
                                [k](const BevLaneInfo &lane)
        {
            return lane.id == k;
        });
        if (tmp == local_map->lane_infos.end())
        {
            new_lane_id = k;
            break;
        }
    }

    return new_lane_id;
}

uint32_t BuildLaneTopology::generateNewLaneMarkerID(BevMapInfoPtr local_map)
{
    uint32_t new_lm_id = 99;
    for (uint32_t k = 99; k >= 1; --k)
    {
        auto tmp = std::find_if(local_map->lanemarkers.begin(),
                                local_map->lanemarkers.end(),
                                [k](const BevLaneMarker &lanemarker)
        {
            return lanemarker.id == k;
        });
        if (tmp == local_map->lanemarkers.end())
        {
            new_lm_id = k;
            break;
        }
    }
    return new_lm_id;
}
float BuildLaneTopology::GetDistPointLane(
  const Eigen::Vector2f &point_a,
  const Eigen::Vector2f &point_b,
  const Eigen::Vector2f &point_c) {

  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = point_c - point_b;
  Eigen::Vector2f BA = point_a - point_b;

  if (abs(BC.norm()) < 0.0001) {
    return abs(BA.y());
  }

  float dist_proj = 0.0f;
  if(BC.norm() > 0)
  {
    dist_proj = BA.dot(BC) / BC.norm();
  }
  // A到BC的垂心为P
  Eigen::Vector2f BP = dist_proj * BC.normalized();
  Eigen::Vector2f AP = (point_b - point_a) + BP;
  return AP.norm();
}
float BuildLaneTopology::Point2LineDistanece(
    const Eigen::Vector2f &point,
    const std::vector<Eigen::Vector2f> & line)
{
    Eigen::Vector2f A(point(0), point(1));
    int index = -1;
    float minX = FLT_MAX;
    for(int i = 0; i < line.size(); i++)
    {
        if(fabs(point(0) - line[i](0)) < minX)
        {
            minX = fabs(point(0) - line[i](0));
            index = i;
        }
    }
    if(index >= 0 && line.size() > 2)
    {
        Eigen::Vector2f B,C;
        B(0) = line[index](0);
        B(1) = line[index](1);
        if(index == line.size()-1)
        {
            C(0) = line[line.size()-2](0);
            C(1) = line[line.size()-2](1);
        }
        else if(index == 0)
        {
            C(0) = line[1](0);
            C(1) = line[1](1);
        }
        else
        {
            C(0) = line[index+1](0);
            C(1) = line[index+1](1);
        }
        return fabs(GetDistPointLane(A,B,C));
    }
    return -1000.0;
}
bool BuildLaneTopology::Computelane2EdgeWidth(
    BevMapInfoPtr local_map,
    const std::vector<Eigen::Vector2f> &source_line,
    const std::vector<Eigen::Vector2f> & left_edge_line)
{
    bool ret= false;
    int dist_2dot5_both_edge_lane_idx = -1;
    //自车道中心线到左路沿线横向距离小于1.25(2.5/2.0）米
    if(source_line.size() > 0 && left_edge_line.size() > 0)
    {
        for(int i = 0; i < source_line.size(); i++)
        {
            Eigen::Vector2f A(source_line[i](0), source_line[i](1));
            int index = -1;
            float minX = FLT_MAX;
            for(int j = 0; j < left_edge_line.size(); j++)
            {
                if(fabs(source_line[i](0) - left_edge_line[j](0)) < minX)
                {
                    minX = fabs(source_line[i](0) - left_edge_line[j](0));
                    index = j;
                }
            }
            if(index >= 0 && left_edge_line.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = left_edge_line[index](0);
                B(1) = left_edge_line[index](1);
                if(index == left_edge_line.size()-1)
                {
                    C(0) = left_edge_line[left_edge_line.size()-2](0);
                    C(1) = left_edge_line[left_edge_line.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = left_edge_line[1](0);
                    C(1) = left_edge_line[1](1);
                }
                else
                {
                    C(0) = left_edge_line[index+1](0);
                    C(1) = left_edge_line[index+1](1);
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                if(dist_2dot5_both_edge_lane_idx < 0 && dist_diff < LANE_MIN_DIST)
                {
                    dist_2dot5_both_edge_lane_idx = i;
                    break;
                }
            }
        }
    }
    if(dist_2dot5_both_edge_lane_idx >= 0)
    {
        ret = true;
    }

    return ret;
}
bool BuildLaneTopology::ComputeEgoToAdjacentLaneWidth(
    BevMapInfoPtr local_map,
    Lane_ID_Storage &store_merge_topo,
    const int & valid_count,
    Ego_Lane_Crop_Info & result,
    int & target_lane_id,
    bool &has_narrow_lane,
    bool &is_selected_adjacent_left_lane,
    bool &is_selected_adjacent_right_lane,
    std::vector<Eigen::Vector2f> & widths)
{
    bool ret = false;
    std::vector<Eigen::Vector2f> ego_lane;
    std::vector<Eigen::Vector2f> left_first_lane;
    std::vector<Eigen::Vector2f> right_first_lane;
    int ego_lane_id = -1;
    int first_left_lane_lane_id = -1;
    int first_right_lane_lane_id = -1;

    for (size_t i = 0; i < local_map->lane_infos.size(); i++)
    {
        auto &bev_lane = local_map->lane_infos[i];
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_lane_id = bev_lane.id;
            for(auto &pt: bev_lane.line_points)
            {
                Eigen::Vector2f pts;
                pts(0) = pt.x;
                pts(1) = pt.y;
                if(pt.x >= 0)
                {
                    ego_lane.push_back(pts);
                }
            }
        }
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST))
        {
            first_left_lane_lane_id = bev_lane.id;
            for(auto &pt: bev_lane.line_points)
            {
                Eigen::Vector2f pts;
                pts(0) = pt.x;
                pts(1) = pt.y;
                if(pt.x >= 0)
                {
                    left_first_lane.push_back(pts);
                }
            }
        }
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST))
        {
            first_right_lane_lane_id = bev_lane.id;
            for(auto &pt: bev_lane.line_points)
            {
                Eigen::Vector2f pts;
                pts(0) = pt.x;
                pts(1) = pt.y;
                if(pt.x >= 0)
                {
                    right_first_lane.push_back(pts);
                }
            }
        }
    }
    //约束x or y哪些匝道不构建merge拓扑。
    bool is_left_lane_avail = false;
    bool is_right_lane_avail = false;
    if(ego_lane.size() > 0 && left_first_lane.size() > 0 )
    {
        int index = -1;
        float minX = FLT_MAX;
        for(int i = 0; i < left_first_lane.size(); i++)
        {
            if(fabs(ego_lane.front()(0) - left_first_lane[i](0)) < minX && left_first_lane[i](0) < 10.0f)
            {
                minX = fabs(ego_lane.front()(0) - left_first_lane[i](0));
                index = i;
            }
        }
        if(index >= 0)
        {
            if(fabs(ego_lane.front()(1) - left_first_lane[index](1)) < 2*LANE_MIN_DIST && ego_lane.front()(0) < 5.0f)
            {
                is_left_lane_avail = true;
            }
        }
        if(fabs(ego_lane.back()(0) - left_first_lane.back()(0)) < MERGE_TOPO_DIST && valid_count < 3)
        {
            is_left_lane_avail = false;
        }
    }
    if(ego_lane.size() > 0 && right_first_lane.size() > 0 )
    {
        int index = -1;
        float minX = FLT_MAX;
        for(int i = 0; i < right_first_lane.size(); i++)
        {
            if(fabs(ego_lane.front()(0) - right_first_lane[i](0)) < minX && right_first_lane[i](0) < 10.0f)
            {
                minX = fabs(ego_lane.front()(0) - right_first_lane[i](0));
                index = i;
            }
        }
        if(index >= 0)
        {
            if(fabs(ego_lane.front()(1) - right_first_lane[index](1)) < 2*LANE_MIN_DIST && ego_lane.front()(0) < 5.0f)
            {
                is_right_lane_avail = true;
            }
        }
        if(fabs(ego_lane.back()(0) - right_first_lane.back()(0)) < MERGE_TOPO_DIST && valid_count < 3)
        {
            is_right_lane_avail = false;
        }
    }

    std::vector<Eigen::Vector2f> left_widths;
    std::vector<Eigen::Vector2f> right_widths;
    if(((ego_lane.size() > 0 && left_first_lane.size() > 0 &&
        ego_lane.back()(0) < left_first_lane.back()(0) &&
        false == store_merge_topo.need_to_build_topo) ||
        (ego_lane.size() > 0 && left_first_lane.size() > 0 &&
        (store_merge_topo.target_lane_id == first_left_lane_lane_id) &&
        true == store_merge_topo.need_to_build_topo)) && true == is_left_lane_avail)
    {//前面3次判断需要自车道比相邻车道短 或后面已经构建了就算自车道长一点也需要构建。
        for(size_t idx = 0; idx < ego_lane.size(); idx++)
        {
            auto &pt = ego_lane[idx];
            Eigen::Vector2f A(pt(0), pt(1));
            int index = -1;
            float minX = FLT_MAX;
            for(int i = 0; i < left_first_lane.size(); i++)
            {
                if(fabs(pt(0) - left_first_lane[i](0)) < minX)
                {
                    minX = fabs(pt(0) - left_first_lane[i](0));
                    index = i;
                }
            }
            if(index >= 0 && left_first_lane.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = left_first_lane[index](0);
                B(1) = left_first_lane[index](1);
                if(index == left_first_lane.size()-1)
                {
                    C(0) = left_first_lane[left_first_lane.size()-2](0);
                    C(1) = left_first_lane[left_first_lane.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = left_first_lane[1](0);
                    C(1) = left_first_lane[1](1);
                }
                else
                {
                    C(0) = left_first_lane[index+1](0);
                    C(1) = left_first_lane[index+1](1);
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                float w = static_cast<int>(dist_diff*10.0)/10.0;
                Eigen::Vector2f wPts;
                wPts(0) = pt(0);
                wPts(1) = w;
                left_widths.emplace_back(wPts);
            }
        }
    }
    if(((ego_lane.size() > 0 && right_first_lane.size() > 0 &&
        ego_lane.back()(0) < right_first_lane.back()(0) &&
        false == store_merge_topo.need_to_build_topo) ||
        (ego_lane.size() > 0 && right_first_lane.size() > 0 &&
        (store_merge_topo.target_lane_id == first_right_lane_lane_id) &&
        true == store_merge_topo.need_to_build_topo)) && true == is_right_lane_avail)
    {//前面3次判断需要自车道比相邻车道短 或后面已经构建了就算自车道长一点也需要构建。
        for(size_t idx = 0; idx < ego_lane.size(); idx++)
        {
            auto &pt = ego_lane[idx];
            Eigen::Vector2f A(pt(0), pt(1));
            int index = -1;
            float minX = FLT_MAX;
            for(int i = 0; i < right_first_lane.size(); i++)
            {
                if(fabs(pt(0) - right_first_lane[i](0)) < minX)
                {
                    minX = fabs(pt(0) - right_first_lane[i](0));
                    index = i;
                }
            }
            if(index >= 0 && right_first_lane.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = right_first_lane[index](0);
                B(1) = right_first_lane[index](1);
                if(index == right_first_lane.size()-1)
                {
                    C(0) = right_first_lane[right_first_lane.size()-2](0);
                    C(1) = right_first_lane[right_first_lane.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = right_first_lane[1](0);
                    C(1) = right_first_lane[1](1);
                }
                else
                {
                    C(0) = right_first_lane[index+1](0);
                    C(1) = right_first_lane[index+1](1);
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                float w = static_cast<int>(dist_diff*10.0)/10.0;
                Eigen::Vector2f wPts;
                wPts(0) = pt(0);
                wPts(1) = w;
                right_widths.emplace_back(wPts);
            }
        }
    }

    const float NARROW_LANE_WIDTH = 2.5f;
    const float DELTA_LANE_WIDTH = 0.6f;
    const float MAX_REMOVED_POINT_X = 60.0f;
    if(ego_lane.size() > 0 && left_widths.size() > 0)
    {
        float first_point_w = left_widths.front()(1);
        float last_point_w = left_widths.back()(1);
        bool narrow_lane_exist = false;
        int count = 0;
        int first_remove_point_index = -1;
        for(size_t idx = 0; idx < left_widths.size(); idx++)
        {
            auto &elem = left_widths[idx];
            if(elem(1) < NARROW_LANE_WIDTH)
            {
                narrow_lane_exist = true;
            }
            if(elem(1) < NARROW_LANE_WIDTH && first_remove_point_index < 0)
            {
                first_remove_point_index = idx;
            }
            if((idx+2) < left_widths.size())
            {
                auto &elem2 = left_widths[idx+2];
                float delta = elem(1) - elem2(1);
                if(delta > 0)
                {
                    count++;
                }
                else if(delta > -0.2)
                {
                    count++;
                }
            }
        }
        if(first_remove_point_index > 0 && left_widths[first_remove_point_index](0) < result.remove_start_point(0))
        {
            result.remove_start_point(0) = left_widths[first_remove_point_index](0);
        }
        if(true == narrow_lane_exist &&
           first_point_w > 0 && last_point_w > 0 &&
           (fabs(last_point_w-first_point_w)) > DELTA_LANE_WIDTH &&
           count == (left_widths.size() - 2))
        {
           ret = true;
           has_narrow_lane = true;
           is_selected_adjacent_left_lane = true;
           target_lane_id = first_left_lane_lane_id;
           widths.swap(left_widths);
        }

        if(ego_lane.size() > 0 && ego_lane.back()(0) >= -20.0 &&
          (store_merge_topo.target_lane_id == first_left_lane_lane_id) &&
           store_merge_topo.need_to_build_topo == true && false == ret)
        {
            ret = true;
            has_narrow_lane = true;
            is_selected_adjacent_left_lane = true;
            target_lane_id = first_left_lane_lane_id;
            widths.swap(left_widths);
        }
    }
    if(ego_lane.size() > 0 && right_widths.size() > 0)
    {
        float first_point_w = right_widths.front()(1);
        float last_point_w = right_widths.back()(1);
        bool narrow_lane_exist = false;
        int count = 0;
        int first_remove_point_index = -1;
        for(size_t idx = 0; idx < right_widths.size(); idx++)
        {
            auto &elem = right_widths[idx];
            if(elem(1) < NARROW_LANE_WIDTH)
            {
                narrow_lane_exist = true;
            }
            if(elem(1) < NARROW_LANE_WIDTH && first_remove_point_index < 0)
            {
                first_remove_point_index = idx;
            }
            if((idx+2) < right_widths.size())
            {
                auto &elem2 = right_widths[idx+2];
                float delta = elem(1) - elem2(1);
                if(delta > 0)
                {
                    count++;
                }
                else if(delta > -0.2)
                {
                    count++;
                }
            }
        }
        if(first_remove_point_index > 0 && right_widths[first_remove_point_index](0) < result.remove_start_point(0))
        {
            result.remove_start_point(0) = right_widths[first_remove_point_index](0);
        }
        if(true == narrow_lane_exist &&
           first_point_w > 0 && last_point_w > 0 &&
           (fabs(last_point_w-first_point_w)) > DELTA_LANE_WIDTH &&
           count == (right_widths.size() - 2))
        {
           ret = true;
           has_narrow_lane = true;
           is_selected_adjacent_right_lane = true;
           target_lane_id = first_right_lane_lane_id;
           widths.swap(right_widths);
        }

        if(ego_lane.size() > 0 && ego_lane.back()(0) >= -20.0 &&
          (store_merge_topo.target_lane_id == first_right_lane_lane_id) &&
           store_merge_topo.need_to_build_topo == true && false == ret)
        {
            ret = true;
            has_narrow_lane = true;
            is_selected_adjacent_right_lane = true;
            target_lane_id = first_right_lane_lane_id;
            widths.swap(right_widths);
        }
    }
    return ret;
}
bool BuildLaneTopology::ComputeTwoLanesWidth(
    BevMapInfoPtr local_map,
    const int & src_lane_id,
    const float &current_speed,
    const int & src_left_lanemarker_id,
    const int & src_right_lanemarker_id,
    std::vector<Eigen::Vector2f> & widths)
{
    bool ret= false;
    int ego_lane_index = -1;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &lane = local_map->lane_infos[idx];
        if (ego_lane_index < 0 && lane.id == src_lane_id)
        {
            ego_lane_index = idx;
            break;
        }
    }
    if(ego_lane_index >= 0)
    {
        auto & ego_left_laneMarker_id  = local_map->lane_infos[ego_lane_index].left_lane_marker_id;
        auto & ego_right_laneMarker_id = local_map->lane_infos[ego_lane_index].right_lane_marker_id;
        std::vector<Eigen::Vector4f> left_lm_points;
        std::vector<Eigen::Vector4f> right_lm_points;
        for (size_t idx = 0; idx < local_map->lanemarkers.size(); idx++)
        {
            auto &laneMarker = local_map->lanemarkers[idx];
            if(ego_left_laneMarker_id > 0 && laneMarker.id == ego_left_laneMarker_id && false == laneMarker.is_virtual)
            {
                int count = 0;
                for(auto &pt: laneMarker.line_points)
                {
                    if(pt.x < 0 || pt.x > 1.5*DETECT_LIMIT)continue;
                    Eigen::Vector4f pts;
                    pts(0) = pt.x;
                    pts(1) = pt.y;
                    pts(2) = count++;
                    pts(3) = 0;
                    left_lm_points.emplace_back(pts);
                }
            }
            if(ego_right_laneMarker_id > 0 && laneMarker.id == ego_right_laneMarker_id && false == laneMarker.is_virtual)
            {
                int count2 = 0;
                for(auto &pt: laneMarker.line_points)
                {
                    if(pt.x < 0)continue;
                    Eigen::Vector4f pts;
                    pts(0) = pt.x;
                    pts(1) = pt.y;
                    pts(2) = count2++;
                    pts(3) = 0;
                    right_lm_points.emplace_back(pts);
                }
            }
        }
        for(size_t idx = 0; idx < left_lm_points.size(); idx++)
        {
            auto &pt = left_lm_points[idx];
            Eigen::Vector2f A(pt(0), pt(1));
            int index = -1;
            float minX = FLT_MAX;
            for(int i = 0; i < right_lm_points.size(); i++)
            {
                if(fabs(pt(0) - right_lm_points[i](0)) < minX)
                {
                    minX = fabs(pt(0) - right_lm_points[i](0));
                    index = i;
                }
            }
            if(index >= 0 && right_lm_points.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = right_lm_points[index](0);
                B(1) = right_lm_points[index](1);
                if(index == right_lm_points.size()-1)
                {
                    C(0) = right_lm_points[right_lm_points.size()-2](0);
                    C(1) = right_lm_points[right_lm_points.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = right_lm_points[1](0);
                    C(1) = right_lm_points[1](1);
                }
                else
                {
                    C(0) = right_lm_points[index+1](0);
                    C(1) = right_lm_points[index+1](1);
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                pt(3) = static_cast<int>(dist_diff*10.0)/10.0;
                Eigen::Vector2f wPts;
                wPts(0) = pt(0);
                wPts(1) = pt(3);
                widths.emplace_back(wPts);
            }
        }
    }

    if( widths.size() >= 2)
    {
        float first_2dot5_dist = 0;
        for(auto p: widths)
        {
            if(p(1) > 0 && p(1) <= LANE_MIN_DIST)
            {
                first_2dot5_dist = p(0);
                break;
            }
        }
        if(first_2dot5_dist > 0 && first_2dot5_dist < current_speed*(1.2f+0.3f)/*规控1.2秒锁定转向，再加0.3秒三帧连续是窄车道时间判断*/)
        {
            return true;
        }
        int last_idx = (int)(widths.size() - 1);
        if(widths[last_idx](1) <= LANE_MIN_DIST &&
           widths[last_idx](0) < DETECT_LIMIT &&
           fabs(widths[0](1) - widths[last_idx](1)) >= 0.5)
        {
           ret = true;
        }
    }
    return ret;
}

bool BuildLaneTopology::JudgeTargetLaneIfAvail(
    BevMapInfoPtr local_map,
    const float &current_speed,
    const std::vector<Eigen::Vector2f> & left_edge_line,
    const std::vector<Eigen::Vector2f> & right_edge_line,
    const int & target_lane_id)
{
    bool res = true;
    int bev_left_lanemarker_id = -1;
    int bev_right_lanemarker_id = -1;
    int target_lane_idx = -1;
    for (size_t i = 0; i < local_map->lane_infos.size(); i++)
    {
        auto &bev_lane = local_map->lane_infos[i];
        if(bev_lane.id != target_lane_id)
        {
            continue;
        }
        if(bev_lane.lane_type != BevLaneType::LANE_TYPE_MAIN)
        {//非正常车道类型
            res = false;
        }

        if(bev_lane.is_blocked == true)
        {//断头路
            res = false;
        }
        target_lane_idx = i;
    }
    if(target_lane_idx > 0)
    {
        bev_left_lanemarker_id  = local_map->lane_infos[target_lane_idx].left_lane_marker_id;
        bev_right_lanemarker_id = local_map->lane_infos[target_lane_idx].right_lane_marker_id;
    }
    std::vector<Eigen::Vector2f>  lane_width_values;
    bool is_fade_away = ComputeTwoLanesWidth(local_map,
                                         target_lane_id,
                                         current_speed,
                                         bev_left_lanemarker_id,
                                         bev_right_lanemarker_id,
                                         lane_width_values);
    if(true == is_fade_away)
    {//车道逐渐变窄且小于2.5米
        //res = false;
    }

    std::vector<MotionObject2Lane> objstacleInfoForBev;
    std::shared_ptr<TrafficFlowManager> pTrafficFlowManager = std::make_shared<TrafficFlowManager>();
    if(nullptr != pTrafficFlowManager)
    {
        std::unordered_map<uint32_t, FusionObjs> traffic_flow_objs;
        pTrafficFlowManager->ParseTrafficFlowRawData(traffic_flow_objs);
        pTrafficFlowManager->BindObstaclesToBevLane(local_map,objstacleInfoForBev);
    }
    int ego_bev_lane_id = 0;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_bev_lane_id = bev_lane.id;
            break;
        }
    }
    FusionObjs bev_ego_lane_front_nearest_obj;
    float bev_min_x = FLT_MAX;
    for(auto & obj: objstacleInfoForBev)
    {
        if(obj.lane_id == ego_bev_lane_id)
        {
            for(auto &elem: obj.motionObjects)
            {
                if(elem.position.x > 0)
                {
                    if(elem.position.x < bev_min_x)
                    {
                        bev_min_x = elem.position.x;
                        bev_ego_lane_front_nearest_obj = elem;
                    }
                }
            }
        }
    }
    if(bev_min_x < DETECT_LIMIT)
    {//自车道前方有车不生成merge拓扑
        res = false;
    }
    return res;
}
bool BuildLaneTopology::RemoveInvalidLaneSeg(
    BevMapInfoPtr local_map,
    const int & ego_lane_id,
    const Eigen::Vector2f & removed_start_point,
    const std::set<int> &target_lanemarker_ids,
    Eigen::Vector2f & end_point)
{
    int src_lane_id = -1;
    int ego_lane_idx = -1;
    int ego_lane_pts_idx = -1;
    int src_left_lm_id = -1;
    int src_right_lm_id = -1;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if(bev_lane.id != ego_lane_id)
        {
            continue;
        }
        if (src_lane_id < 0 && bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            src_lane_id = bev_lane.id;
            ego_lane_idx = idx;
            float min_x = FLT_MAX;
            for(int i = 0; i < bev_lane.line_points.size(); i++)
            {
                if(fabs( bev_lane.line_points[i].x - removed_start_point(0)) < min_x)
                {
                    ego_lane_pts_idx = i;
                    min_x = fabs(bev_lane.line_points[i].x - removed_start_point(0));
                }
            }
        }
    }
    if(ego_lane_idx >= 0)
    {
        src_left_lm_id  = local_map->lane_infos[ego_lane_idx].left_lane_marker_id;
        src_right_lm_id = local_map->lane_infos[ego_lane_idx].right_lane_marker_id;

        int left_lm_idx = -1;
        int left_lm_pts_idx = -1;
        int right_lm_idx = -1;
        int right_lm_pts_idx = -1;
        for (size_t idx = 0; idx < local_map->lanemarkers.size(); idx++)
        {
            auto& bev_lm = local_map->lanemarkers.at(idx);
            if(src_left_lm_id == bev_lm.id)
            {
                left_lm_idx = idx;
                float min_x = FLT_MAX;
                for(int i = 0; i < bev_lm.line_points.size(); i++)
                {
                    if(fabs( bev_lm.line_points[i].x - removed_start_point(0)) < min_x)
                    {
                        left_lm_pts_idx = i;
                        min_x = fabs(bev_lm.line_points[i].x - removed_start_point(0));
                    }
                }
            }
            if(src_right_lm_id == bev_lm.id)
            {
                right_lm_idx = idx;
                float min_x = FLT_MAX;
                for(int i = 0; i < bev_lm.line_points.size(); i++)
                {
                    if(fabs( bev_lm.line_points[i].x - removed_start_point(0)) < min_x)
                    {
                        right_lm_pts_idx = i;
                        min_x = fabs(bev_lm.line_points[i].x - removed_start_point(0));
                    }
                }
            }
        }
        if(ego_lane_pts_idx > 0 && left_lm_pts_idx > 0 && right_lm_pts_idx > 0)
        {
            //lane
            std::vector<Point2DF>   new_line_points;
            for(int i = 0; i < local_map->lane_infos[ego_lane_idx].line_points.size(); i++)
            {
                if(i < ego_lane_pts_idx)
                {
                    auto & pts = local_map->lane_infos[ego_lane_idx].line_points.at(i);
                    new_line_points.push_back(pts);
                }
            }
            end_point(0) = new_line_points.back().x;
            end_point(1) = new_line_points.back().y;
            new_line_points.swap(local_map->lane_infos[ego_lane_idx].line_points);
            local_map->lane_infos[ego_lane_idx].number_of_points = local_map->lane_infos[ego_lane_idx].line_points.size();
            //left lanemarker
            std::vector<Point2DF>   new_left_lm_points;
            for(int i = 0; i < local_map->lanemarkers[left_lm_idx].line_points.size(); i++)
            {
                if(i < left_lm_pts_idx)
                {
                    auto & pts = local_map->lanemarkers[left_lm_idx].line_points.at(i);
                    new_left_lm_points.push_back(pts);
                }
            }
            if(target_lanemarker_ids.count(src_left_lm_id))
            {////如果过是共线，另外起一个车道ID，不能把其他车道的车道线裁剪了。
                uint64_t new_left_lanemarker_id = generateNewLaneMarkerID(local_map);
                BevLaneMarker new_lm = local_map->lanemarkers[left_lm_idx];
                new_lm.id = new_left_lanemarker_id;
                local_map->lane_infos[ego_lane_idx].left_lane_marker_id = new_left_lanemarker_id;
                std::vector<cem::message::common::Point2DF>().swap(new_lm.line_points);
                for(int i = 0; i < local_map->lanemarkers[right_lm_idx].line_points.size(); i++)
                {
                    if(i < left_lm_pts_idx)
                    {
                        auto & pts = local_map->lanemarkers[right_lm_idx].line_points.at(i);
                        new_lm.line_points.push_back(pts);
                    }
                }
                new_lm.number_of_points = new_lm.line_points.size();
                local_map->lanemarkers.push_back(new_lm);
            }
            else
            {
                new_left_lm_points.swap(local_map->lanemarkers[left_lm_idx].line_points);
                local_map->lanemarkers[left_lm_idx].number_of_points = local_map->lanemarkers[left_lm_idx].line_points.size();
            }
            //right lanemarker
            std::vector<Point2DF>   new_right_lm_points;
            for(int i = 0; i < local_map->lanemarkers[right_lm_idx].line_points.size(); i++)
            {
                if(i < right_lm_pts_idx)
                {
                    auto & pts = local_map->lanemarkers[right_lm_idx].line_points.at(i);
                    new_right_lm_points.push_back(pts);
                }
            }
            if(target_lanemarker_ids.count(src_right_lm_id))
            {////如果过是共线，另外起一个车道ID，不能把其他车道的车道线裁剪了。
                uint64_t new_right_lanemarker_id = generateNewLaneMarkerID(local_map);
                BevLaneMarker new_lm = local_map->lanemarkers[right_lm_idx];
                new_lm.id = new_right_lanemarker_id;
                local_map->lane_infos[ego_lane_idx].right_lane_marker_id = new_right_lanemarker_id;
                std::vector<cem::message::common::Point2DF>().swap(new_lm.line_points);
                for(int i = 0; i < local_map->lanemarkers[right_lm_idx].line_points.size(); i++)
                {
                    if(i < right_lm_pts_idx)
                    {
                        auto & pts = local_map->lanemarkers[right_lm_idx].line_points.at(i);
                        new_lm.line_points.push_back(pts);
                    }
                }
                new_lm.number_of_points = new_lm.line_points.size();
                local_map->lanemarkers.push_back(new_lm);
            }
            else
            {
                new_right_lm_points.swap(local_map->lanemarkers[right_lm_idx].line_points);
                local_map->lanemarkers[right_lm_idx].number_of_points = local_map->lanemarkers[right_lm_idx].line_points.size();
            }
        }
    }
    return true;
}
void BuildLaneTopology::SegmentLaneTwoParts(
    BevMapInfoPtr local_map,
    const int & segment_lane_id,
    const Eigen::Vector2f & segment_point,
    const bool & is_on_topo,
    int & new_lane_id)
{
    int lane_id = -1;
    int lane_idx = -1;
    int lane_pts_idx = -1;
    int left_lm_id = -1;
    int right_lm_id = -1;
    int lane_num_of_points = 0;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (lane_id < 0 && bev_lane.id == segment_lane_id)
        {
            lane_id = bev_lane.id;
            lane_idx = idx;
            float min_x = FLT_MAX;
            lane_num_of_points = bev_lane.line_points.size();
            for(int i = 0; i < bev_lane.line_points.size(); i++)
            {
                if(fabs( bev_lane.line_points[i].x - segment_point(0)) < min_x && fabs( bev_lane.line_points[i].x - segment_point(0)) < 10.0f)
                {
                    lane_pts_idx = i;
                    min_x = fabs(bev_lane.line_points[i].x - segment_point(0));
                }
            }
        }
    }
    if(lane_idx >= 0)
    {
        left_lm_id  = local_map->lane_infos[lane_idx].left_lane_marker_id;
        right_lm_id = local_map->lane_infos[lane_idx].right_lane_marker_id;

        int left_lm_idx = -1;
        int left_lm_pts_idx = -1;
        int right_lm_idx = -1;
        int right_lm_pts_idx = -1;

        int left_lm_num_of_points = 0;
        int right_lm_num_of_points = 0;
        for (size_t idx = 0; idx < local_map->lanemarkers.size(); idx++)
        {
            auto& bev_lm = local_map->lanemarkers.at(idx);
            if(left_lm_id == bev_lm.id)
            {
                left_lm_idx = idx;
                float min_x = FLT_MAX;
                left_lm_num_of_points = bev_lm.line_points.size();
                for(int i = 0; i < bev_lm.line_points.size(); i++)
                {
                    if(fabs( bev_lm.line_points[i].x - segment_point(0)) < min_x)
                    {
                        left_lm_pts_idx = i;
                        min_x = fabs(bev_lm.line_points[i].x - segment_point(0));
                    }
                }
            }
            if(right_lm_id == bev_lm.id)
            {
                right_lm_idx = idx;
                float min_x = FLT_MAX;
                right_lm_num_of_points = bev_lm.line_points.size();
                for(int i = 0; i < bev_lm.line_points.size(); i++)
                {
                    if(fabs( bev_lm.line_points[i].x - segment_point(0)) < min_x)
                    {
                        right_lm_pts_idx = i;
                        min_x = fabs(bev_lm.line_points[i].x - segment_point(0));
                    }
                }
            }
        }
        if(lane_pts_idx < 0 && lane_num_of_points > 5)
        {
            lane_pts_idx = lane_num_of_points - 3;
        }
        if(left_lm_pts_idx < 0 && left_lm_num_of_points > 5)
        {
            left_lm_pts_idx = left_lm_num_of_points - 3;
        }
        if(right_lm_pts_idx < 0 && right_lm_num_of_points > 5)
        {
            right_lm_pts_idx = right_lm_num_of_points - 3;
        }
        if(lane_pts_idx >= 0 && (lane_num_of_points-lane_pts_idx) <= 2 && lane_num_of_points > 5)
        {
            lane_pts_idx = lane_num_of_points - 3;
        }
        if((left_lm_num_of_points-left_lm_pts_idx) <= 2  && left_lm_num_of_points > 5)
        {
             left_lm_pts_idx = left_lm_num_of_points - 3;
        }
        if((right_lm_num_of_points-right_lm_pts_idx) <= 2 && right_lm_num_of_points > 5)
        {
            right_lm_pts_idx = right_lm_num_of_points - 3;
        }
        if(lane_pts_idx >= 0 && left_lm_pts_idx >= 0 && right_lm_pts_idx >= 0 &&
           (lane_num_of_points-lane_pts_idx) > 2 &&
           (left_lm_num_of_points-left_lm_pts_idx) > 2 && left_lm_pts_idx < left_lm_num_of_points &&
           (right_lm_num_of_points-right_lm_pts_idx) > 2 && right_lm_pts_idx < right_lm_num_of_points)
        {
            uint64_t eog_next_lane_id = 0;
            Point2DF target_lane_last_point;
            target_lane_last_point.x = 0;
            target_lane_last_point.y = 0;
            for (size_t i = 0; i < local_map->lane_infos.size(); i++)
            {
                auto &bev_lane = local_map->lane_infos[i];
                if(bev_lane.next_lane_ids.size() > 0 && segment_lane_id == bev_lane.id)
                {
                    for(auto &id : bev_lane.next_lane_ids)
                    {//当前切割车道next是新车道中心线来接线
                        eog_next_lane_id = id;
                        break;
                    }
                    if(bev_lane.line_points.size() > 0)
                    {
                        target_lane_last_point = bev_lane.line_points.back();
                    }
                }
            }
            std::vector<Point2DF>   origin_line_points;
            std::vector<Point2DF>   new_line_points;
            for(int i = 0; i < local_map->lane_infos[lane_idx].line_points.size(); i++)
            {
                auto & pts = local_map->lane_infos[lane_idx].line_points.at(i);
                if(i < lane_pts_idx)
                {
                    origin_line_points.push_back(pts);
                }
                else
                {
                    new_line_points.push_back(pts);
                }
            }
            origin_line_points.swap(local_map->lane_infos[lane_idx].line_points);
            local_map->lane_infos[lane_idx].number_of_points = local_map->lane_infos[lane_idx].line_points.size();

            std::vector<Point2DF>   origin_left_lm_points;
            std::vector<Point2DF>   new_left_lm_points;
            for(int i = 0; i < local_map->lanemarkers[left_lm_idx].line_points.size(); i++)
            {
                auto & pts = local_map->lanemarkers[left_lm_idx].line_points.at(i);
                if(i < left_lm_pts_idx)
                {
                    origin_left_lm_points.push_back(pts);
                }
                else
                {
                    new_left_lm_points.push_back(pts);
                }
            }
            origin_left_lm_points.swap(local_map->lanemarkers[left_lm_idx].line_points);
            local_map->lanemarkers[left_lm_idx].number_of_points = local_map->lanemarkers[left_lm_idx].line_points.size();

            std::vector<Point2DF>   origin_right_lm_points;
            std::vector<Point2DF>   new_right_lm_points;
            for(int i = 0; i < local_map->lanemarkers[right_lm_idx].line_points.size(); i++)
            {
                auto & pts = local_map->lanemarkers[right_lm_idx].line_points.at(i);
                if(i < right_lm_pts_idx)
                {
                    origin_right_lm_points.push_back(pts);
                }
                else
                {
                    new_right_lm_points.push_back(pts);
                }
            }
            origin_right_lm_points.swap(local_map->lanemarkers[right_lm_idx].line_points);
            local_map->lanemarkers[right_lm_idx].number_of_points = local_map->lanemarkers[left_lm_idx].line_points.size();

            BevLaneMarker new_left_lm = local_map->lanemarkers[left_lm_idx];
            new_left_lm.id = generateNewLaneMarkerID(local_map);
            new_left_lm_points.swap(new_left_lm.line_points);
            local_map->lanemarkers.emplace_back(new_left_lm);

            BevLaneMarker new_right_lm = local_map->lanemarkers[right_lm_idx];
            new_right_lm.id = generateNewLaneMarkerID(local_map);
            new_right_lm_points.swap(new_right_lm.line_points);
            local_map->lanemarkers.emplace_back(new_right_lm);

            BevLaneInfo new_lane = local_map->lane_infos[lane_idx];
            new_lane.id = generateNewLaneID(local_map);
            if(eog_next_lane_id > 0 && new_lane.id > 0)
            {
                std::vector<uint64_t>().swap(new_lane.next_lane_ids);
                new_lane.next_lane_ids.push_back(eog_next_lane_id);
                int index = -1;
                for (size_t idx = 0; index < 0 && idx < local_map->lane_infos.size(); idx++)
                {
                    auto &bev_lane = local_map->lane_infos[idx];
                    if(bev_lane.previous_lane_ids.size() > 0)
                    {
                        for(auto &id : bev_lane.previous_lane_ids)
                        {
                            if(id == segment_lane_id)
                            {//找到前继previous是当前切割车道
                                index = idx;
                                break;
                            }
                        }
                    }
                }
                if(index >= 0)
                {//把剩余非当前切割车道作为previous和新的new id一起作为previous车道
                    std::vector<uint64_t> previous_lane_ids;
                    for(auto &id : local_map->lane_infos[index].previous_lane_ids)
                    {
                        if(id != segment_lane_id)
                        {
                            previous_lane_ids.push_back(id);
                        }
                    }
                    previous_lane_ids.push_back(new_lane.id);
                    local_map->lane_infos[index].previous_lane_ids.swap(previous_lane_ids);
                }
            }
            new_line_points.swap(new_lane.line_points);
            new_lane_id = new_lane.id;
            new_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
            new_lane.left_lane_marker_id = new_left_lm.id;
            new_lane.right_lane_marker_id = new_right_lm.id;
            local_map->lane_infos.emplace_back(new_lane);
        }
    }

    return;
}
bool BuildLaneTopology::CheckLDMapIfTopoExist(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids)
{
    bool ret = false;
    bool is_judge_ld_map_topo = true;
    BevAndMapChecking(local_map,ld_map,bev_map_ids,is_judge_ld_map_topo);
    bool is_ld_map_low_precision = isLDMapLowPrecision();
    if(false == is_judge_ld_map_topo || true == is_ld_map_low_precision)
    {
        return ret;
    }
    int lane_id = -1;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (lane_id < 0 && bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            lane_id = bev_lane.id;
        }
    }
    bool ld_map_split_exist = false;
    bool ld_map_merge_exist = false;
    for(auto &elem: bev_map_ids.ldmap_merge_info)
    {
        if(elem.bev_lane_to == lane_id &&
           elem.distance_to_ego < DETECT_LIMIT &&
           (elem.match_type == LdMapProcessor::MergeSplitType::kMergeLeft ||
            elem.match_type == LdMapProcessor::MergeSplitType::kMergeRight))
        {
            ld_map_merge_exist = true;
        }
    }
    for(auto &elem: bev_map_ids.ldmap_split_info)
    {
        if(elem.bev_lane_from == lane_id &&
           elem.distance_to_ego < DETECT_LIMIT &&
           (elem.match_type == LdMapProcessor::MergeSplitType::kSplitLeft ||
            elem.match_type == LdMapProcessor::MergeSplitType::kSplitRight))
        {
            ld_map_split_exist = true;
        }
    }
    if(true == ld_map_merge_exist || true == ld_map_split_exist)
    {
        ret = true;
    }
    return ret;
}
bool BuildLaneTopology::JudgeIfBuildMergeTopo(
    BevMapInfoPtr local_map,
    const std::vector<Eigen::Vector2f> &source_line,
    const std::vector<Eigen::Vector3f> & src_left_lm_line,
    const std::vector<Eigen::Vector3f> & src_right_lm_line,
    const std::vector<Eigen::Vector2f> & left_edge_line,
    const std::vector<Eigen::Vector2f> & right_edge_line,
    const float &current_speed,
    float & lane_width_limit,
    Lane_ID_Storage &store_merge_topo,
    Ego_Lane_Crop_Info & result)
{
    bool ret = false;
    if(source_line.size() > 0)
    {
        result.remove_start_point(0) = source_line.back()(0);
        result.remove_start_point(1) = source_line.back()(1);
    }
    int dist_2dot5_both_lm_idx = -1;
    Eigen::Vector2f left_seg_point(0.0,0.0);
    int dist_2dot5_edge_left_lm_idx = -1;
    Eigen::Vector2f edge_left_seg_point(0.0,0.0);
    int dist_2dot5_edge_right_lm_idx = -1;
    Eigen::Vector2f edge_right_seg_point(0.0,0.0);

    int dist_prev_cut_lm_idx = -1;
    bool has_left_lane = true;
    bool has_right_lane = true;

    //自车道宽度小于2.5米!
    for(size_t idx = 0; idx < src_left_lm_line.size(); idx++)
    {
        auto &pt = src_left_lm_line[idx];
        Eigen::Vector2f A(pt(0), pt(1));
        int index = -1;
        float minX = FLT_MAX;
        for(int i = 0; i < src_right_lm_line.size(); i++)
        {
            if(fabs(pt(0) - src_right_lm_line[i](0)) < minX)
            {
                minX = fabs(pt(0) - src_right_lm_line[i](0));
                index = i;
            }
        }
        if(index >= 0 && src_right_lm_line.size() > 2)
        {
            Eigen::Vector2f B,C;
            B(0) = src_right_lm_line[index](0);
            B(1) = src_right_lm_line[index](1);
            if(index == src_right_lm_line.size()-1)
            {
                C(0) = src_right_lm_line[src_right_lm_line.size()-2](0);
                C(1) = src_right_lm_line[src_right_lm_line.size()-2](1);
            }
            else if(index == 0)
            {
                C(0) = src_right_lm_line[1](0);
                C(1) = src_right_lm_line[1](1);
            }
            else
            {
                C(0) = src_right_lm_line[index+1](0);
                C(1) = src_right_lm_line[index+1](1);
            }
            float dist_diff =  static_cast<int>(fabs(GetDistPointLane(A,B,C))*10.0)/10.0;
            if(dist_2dot5_both_lm_idx < 0 && dist_diff < lane_width_limit)
            {
                dist_2dot5_both_lm_idx = idx;
                result.has_narrow_lane = true;
                left_seg_point(0) = src_left_lm_line[idx](0);
                left_seg_point(1) = src_left_lm_line[idx](1);
            }
        }
    }
    //判断左边
    if(src_right_lm_line.size() > 0)
    {
        #if 0
        for(int i = static_cast<int>(src_right_lm_line.size()-1); i >= 0; i--)
        {
            if(src_right_lm_line.back()(0) > (src_right_lm_line[i](0) + DIST_2_END_CUT - current_speed*0.05) && dist_prev_cut_lm_idx < 0)
            {
                dist_prev_cut_lm_idx = i;
            }
        }
        if(dist_2dot5_both_lm_idx >= 0 && dist_prev_cut_lm_idx >= 0)
        {
            dist_2dot5_both_lm_idx = std::min(dist_2dot5_both_lm_idx,dist_prev_cut_lm_idx);
        }
        if(dist_2dot5_both_lm_idx < 0 && dist_prev_cut_lm_idx >= 0)
        {
            dist_2dot5_both_lm_idx = dist_prev_cut_lm_idx;
        }
        #endif
        for(size_t idx = 0; idx < src_right_lm_line.size(); idx++)
        {
            auto &pt = src_right_lm_line[idx];
            Eigen::Vector2f A(pt(0), pt(1));
            int index = -1;
            float minX = FLT_MAX;
            for(int i = 0; i < left_edge_line.size(); i++)
            {
                if(fabs(pt(0) - left_edge_line[i](0)) < minX)
                {
                    minX = fabs(pt(0) - left_edge_line[i](0));
                    index = i;
                }
            }
            if(index >= 0 && left_edge_line.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = left_edge_line[index](0);
                B(1) = left_edge_line[index](1);
                if(index == left_edge_line.size()-1)
                {
                    C(0) = left_edge_line[left_edge_line.size()-2](0);
                    C(1) = left_edge_line[left_edge_line.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = left_edge_line[1](0);
                    C(1) = left_edge_line[1](1);
                }
                else
                {
                    C(0) = left_edge_line[index+1](0);
                    C(1) = left_edge_line[index+1](1);
                }
                float dist_diff =  static_cast<int>(fabs(GetDistPointLane(A,B,C))*10.0)/10.0;
                if(dist_2dot5_edge_left_lm_idx < 0 && dist_diff < lane_width_limit)
                {
                    dist_2dot5_edge_left_lm_idx = idx;
                    edge_left_seg_point(0) = src_right_lm_line[idx](0);
                    edge_left_seg_point(1) = src_right_lm_line[idx](1);
                }
                if(dist_diff < LANEMARKER_2_EDGE_DIST)
                {//ORIN-1141656
                    has_left_lane = false;
                    result.has_left_edge = true;
                }
            }
        }
    }
    //判断右边
    if(src_left_lm_line.size() > 0)
    {
        #if 0
        for(int i = static_cast<int>(src_left_lm_line.size()-1); i >= 0; i--)
        {
            if(src_left_lm_line.back()(0) > (src_left_lm_line[i](0) + DIST_2_END_CUT - current_speed*0.05) && dist_prev_cut_lm_idx < 0)
            {
                dist_prev_cut_lm_idx = i;
            }
        }
        if(dist_2dot5_both_lm_idx >= 0 && dist_prev_cut_lm_idx >= 0)
        {
            dist_2dot5_both_lm_idx = std::min(dist_2dot5_both_lm_idx,dist_prev_cut_lm_idx);
        }
        if(dist_2dot5_both_lm_idx < 0 && dist_prev_cut_lm_idx >= 0)
        {
            dist_2dot5_both_lm_idx = dist_prev_cut_lm_idx;
        }
        #endif
        for(size_t idx = 0; idx < src_left_lm_line.size(); idx++)
        {
            auto &pt = src_left_lm_line[idx];
            Eigen::Vector2f A(pt(0), pt(1));
            int index = -1;
            float minX = FLT_MAX;
            for(int i = 0; i < right_edge_line.size(); i++)
            {
                if(fabs(pt(0) - right_edge_line[i](0)) < minX)
                {
                    minX = fabs(pt(0) - right_edge_line[i](0));
                    index = i;
                }
            }
            if(index >= 0 && right_edge_line.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = right_edge_line[index](0);
                B(1) = right_edge_line[index](1);
                if(index == right_edge_line.size()-1)
                {
                    C(0) = right_edge_line[right_edge_line.size()-2](0);
                    C(1) = right_edge_line[right_edge_line.size()-2](1);
                }
                else if(index == 0)
                {
                    C(0) = right_edge_line[1](0);
                    C(1) = right_edge_line[1](1);
                }
                else
                {
                    C(0) = right_edge_line[index+1](0);
                    C(1) = right_edge_line[index+1](1);
                }
                float dist_diff =  static_cast<int>(fabs(GetDistPointLane(A,B,C))*10.0)/10.0;
                if(dist_2dot5_edge_right_lm_idx < 0 && dist_diff < lane_width_limit)
                {
                    dist_2dot5_edge_right_lm_idx = idx;
                    edge_right_seg_point(0) = src_left_lm_line[idx](0);
                    edge_right_seg_point(1) = src_left_lm_line[idx](1);
                }
                if(dist_diff < LANEMARKER_2_EDGE_DIST)
                {//ORIN-1141656
                    has_right_lane = false;
                    result.has_right_edge = true;
                }
            }
        }
    }
    for(int i = 0; i < src_left_lm_line.size(); i++)
    {
        if(src_left_lm_line[i](2) > 0.0f)
        {//左边是实线，不能创建merge拓扑！
            has_left_lane = false;
        }
    }
    for(int i = 0; i < src_right_lm_line.size(); i++)
    {
        if(src_right_lm_line[i](2) > 0.0f)
        {//右边是实线，不能创建merge拓扑！
            has_right_lane = false;
        }
    }
    result.less_2dot5_both_lm_idx = dist_2dot5_both_lm_idx;
    result.less_2dot5_edge_left_lm_idx = dist_2dot5_edge_left_lm_idx;
    result.less_2dot5_edge_right_lm_idx = dist_2dot5_edge_right_lm_idx;
    result.dist_prev_cut_lm_idx = dist_prev_cut_lm_idx;
    if(true == has_left_lane)
    {
        result.left_build_merge = true;
    }
    if(true == has_right_lane)
    {
        result.right_build_merge = true;
    }
    //自车道太窄开始裁剪的点
    if(source_line.size() > 0 )
    {
        result.remove_start_point(0) = source_line.back()(0);
        result.remove_start_point(1) = source_line.back()(1);
        bool found = false;
        //left
        if(dist_2dot5_both_lm_idx >= 0 && dist_2dot5_edge_right_lm_idx >= 0 && !found)
        {
            if(dist_2dot5_both_lm_idx >= dist_2dot5_edge_right_lm_idx)
            {
                result.remove_start_point(0) = edge_right_seg_point(0);
                result.remove_start_point(1) = edge_right_seg_point(1);
                found = true;
            }
            else
            {
                result.remove_start_point(0) = left_seg_point(0);
                result.remove_start_point(1) = left_seg_point(1);
                found = true;
            }
        }
        else if(dist_2dot5_both_lm_idx >= 0 && dist_2dot5_edge_right_lm_idx < 0 && !found)
        {
            result.remove_start_point(0) = left_seg_point(0);
            result.remove_start_point(1) = left_seg_point(1);
            found = true;
        }
        else if(dist_2dot5_both_lm_idx < 0 && dist_2dot5_edge_right_lm_idx >= 0 && !found)
        {
            result.remove_start_point(0) = edge_right_seg_point(0);
            result.remove_start_point(1) = edge_right_seg_point(1);
            found = true;
        }

        //right
        if(dist_2dot5_both_lm_idx >= 0 && dist_2dot5_edge_left_lm_idx >= 0 && !found)
        {
            if(dist_2dot5_both_lm_idx >= dist_2dot5_edge_left_lm_idx)
            {
                result.remove_start_point(0) = edge_left_seg_point(0);
                result.remove_start_point(1) = edge_left_seg_point(1);
                found = true;
            }
            else
            {
                result.remove_start_point(0) = left_seg_point(0);
                result.remove_start_point(1) = left_seg_point(1);
                found = true;
            }
        }
        else if(dist_2dot5_both_lm_idx >= 0 && dist_2dot5_edge_left_lm_idx < 0 && !found)
        {
            result.remove_start_point(0) = left_seg_point(0);
            result.remove_start_point(1) = left_seg_point(1);
            found = true;
        }
        else if(dist_2dot5_both_lm_idx < 0 && dist_2dot5_edge_left_lm_idx >= 0 && !found)
        {
            result.remove_start_point(0) = edge_left_seg_point(0);
            result.remove_start_point(1) = edge_left_seg_point(1);
            found = true;
        }
    }
    if(true == has_left_lane || true == has_right_lane)
    {
        ret = true;
    }

    return ret;
}
void BuildLaneTopology::LaneEdgeChecking(
    BevMapInfoPtr local_map,
    std::vector<map_id_point> &ego_lane_points,
    std::vector<map_id_point> &split_lane_points,
    std::vector<map_id_point> &main_lane_points,
    bool &edge_exist)
{
    std::vector<Eigen::Vector2f> topo_points;
    if(ego_lane_points.size() > 0)
    {
        if(ego_lane_points.back().line_points.size() > 0)
        {
            Eigen::Vector2f pts;
            pts(0) = ego_lane_points.back().line_points.back().x;
            pts(1) = ego_lane_points.back().line_points.back().y;
            topo_points.push_back(pts);
        }
    }
    if(split_lane_points.size() > 0)
    {
        if(split_lane_points.front().line_points.size() > 0)
        {
            Eigen::Vector2f pts;
            pts(0) = split_lane_points.front().line_points.front().x;
            pts(1) = split_lane_points.front().line_points.front().y;
            topo_points.push_back(pts);
        }
    }
    if(main_lane_points.size() > 0)
    {
        if(main_lane_points.front().line_points.size() > 0)
        {
            Eigen::Vector2f pts;
            pts(0) = main_lane_points.front().line_points.front().x;
            pts(1) = main_lane_points.front().line_points.front().y;
            topo_points.push_back(pts);
        }
    }
    if(topo_points.size() > 0)
    {
        for(auto &pt: topo_points)
        {
            for (size_t i = 0;  i < local_map->edges.size() ; i++)
            {
                auto & edge = local_map->edges[i];
                if(edge.position == static_cast<int>(BevRoadEdgePosition::BEV_REP__LEFT) ||
                   edge.position == static_cast<int>(BevRoadEdgePosition::BEV_REP__RIGHT))
                {
                    for(int k = 0; k < edge.line_points.size() ; k++)
                    {
                        auto & pts = edge.line_points[k];
                        Eigen::Vector2f dist_diff;
                        dist_diff(0) = pts.x - pt(0);
                        dist_diff(1) = pts.y - pt(1);
                        if(dist_diff.norm() < LANE_MIN_DIST/2.0)
                        {
                            edge_exist = true;
                            return;
                        }
                    }
                }
            }
        }
    }
    return ;
}
bool BuildLaneTopology::LineObjectsChecking(
    std::unordered_map<uint32_t, FusionObjs> & traffic_flow,
    std::vector<map_id_point> &lane_points)
{
    for(auto &elem : traffic_flow)
    {
        float x_coord = static_cast<float>(elem.second.position.x);
        float y_coord = static_cast<float>(elem.second.position.y);
        Eigen::Vector2f A(x_coord, y_coord);
        bool found = false;
        int lane_idx = -1;
        for (size_t i = 0; i < lane_points.size() && !found; i++)
        {
            auto &line = lane_points[i];
            if(line.line_points.size() <= 1)
            {
                continue;
            }

            std::vector<Point> map_raw_points;
            for(auto &pt: line.line_points)
            {
                if(pt.x > 0)
                {
                    map_raw_points.push_back(pt);
                }
            }
            int index = -1;
            float minX = FLT_MAX;
            for(int j = 0; j < map_raw_points.size(); j++)
            {
                if(fabs(map_raw_points[j].x - x_coord) < minX)
                {
                    minX = fabs(map_raw_points[j].x - x_coord);
                    index = j;
                }
            }
            if(index >= 0 && minX < FLT_MAX && map_raw_points.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = map_raw_points[index].x;
                B(1) = map_raw_points[index].y;
                if(index == map_raw_points.size()-1)
                {
                    C(0) = map_raw_points[map_raw_points.size()-2].x;
                    C(1) = map_raw_points[map_raw_points.size()-2].y;
                }
                else if(index == 0)
                {
                    C(0) = map_raw_points[1].x;
                    C(1) = map_raw_points[1].y;
                }
                else
                {
                    C(0) = map_raw_points[index+1].x;
                    C(1) = map_raw_points[index+1].y;
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                if(dist_diff < 2.5f/2.0)
                {
                    found = true;
                    lane_idx = i;
                }
            }
            if(true == found && lane_idx >= 0)
            {
                return true;
            }
        }
    }
    return false;
}
void BuildLaneTopology::trafficFlowChecking(
    std::unordered_map<uint32_t, FusionObjs> & traffic_flow,
    std::vector<map_id_point> &ego_lane_points,
    std::vector<map_id_point> &split_lane_points,
    std::vector<map_id_point> &main_lane_points,
    bool &traffic_flow_exist)
{
    if(traffic_flow.size() <= 0)
    {
        return;
    }
    traffic_flow_exist = LineObjectsChecking(traffic_flow,ego_lane_points);
    traffic_flow_exist = LineObjectsChecking(traffic_flow,split_lane_points);
    traffic_flow_exist = LineObjectsChecking(traffic_flow,main_lane_points);

    return;
}
void BuildLaneTopology::BevAndMapChecking(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    bool &is_need_build)
{
    //感知与地图没有匹配上就没有对应关系
    typedef struct bevMapLane_comp_s
    {
        uint32_t position = 0;
        std::vector<Eigen::Vector2f> coords;
    }bevMapLane_comp;

    std::vector<bevMapLane_comp> bevLanesComp;
    std::vector<bevMapLane_comp> mapLanesComp;
    std::set<uint64_t> bev_id_list;
    int max_count = 50;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) ||
           bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) ||
           bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST))
        {
            bev_id_list.insert(bev_lane.id);
            bevMapLane_comp elem;
            elem.position = bev_lane.position;
            for(auto &pt: bev_lane.line_points)
            {
                if(pt.x >= 0 && pt.x < max_count)
                {
                    Eigen::Vector2f lane_pts;
                    lane_pts(0) = pt.x;
                    lane_pts(1) = pt.y;
                    elem.coords.push_back(lane_pts);
                }
            }
            if(elem.coords.size() > 0 )
            {
                bevLanesComp.push_back(elem);
            }
        }
    }
    //感知与地图没有匹配上就没有对应关系
    for(auto & bev_id: bev_id_list)
    {
        for(auto &elem : bev_map_ids.local_map_matched_ids)
        {
            if(bev_id == elem.first)
            {
                if(elem.second.size() == 0)
                {
                    is_need_build = false;
                    return;
                }
            }
        }
    }
    return;
}
std::vector<Point> BuildLaneTopology::GetMapPointByBevLaneId(
    uint64_t map_lane_id,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map)
{
    std::vector<Point> res;
    for(int idx = 0; idx < ld_map->lanes.size(); idx++)
    {
        auto &map_lane = ld_map->lanes[idx];
        if(map_lane_id > 0 && map_lane_id == map_lane.id)
        {
            for(int pts_idx = 0 ; pts_idx < map_lane.points.size(); pts_idx++)
            {
                res.emplace_back(map_lane.points[pts_idx]);
            }
        }
    }
    return res;
}

std::vector<Point2D> BuildLaneTopology::GetMapPointByBevLanemarkerId(
    uint64_t map_lanemarker_id,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map)
{
    std::vector<Point2D> res;
    for(int idx = 0; idx < ld_map->lane_boundaries.size(); idx++)
    {
        auto &map_lm = ld_map->lane_boundaries[idx];
        if(map_lanemarker_id > 0 && map_lanemarker_id == map_lm.id)
        {
            for(int pts_idx = 0 ; pts_idx < map_lm.points.size(); pts_idx++)
            {
                res.emplace_back(map_lm.points[pts_idx]);
            }
        }
    }
    return res;
}
void BuildLaneTopology::GetMapEgoIdByMatch(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    uint64_t & map_ego_lane_id)
{
    if(nullptr == ld_map)
    {
        return;
    }
    uint64_t ego_bev_lane_id = 0;
    std::vector<uint64_t> ego_map_lane_ids;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_bev_lane_id = bev_lane.id;
        }
    }
    for(auto & elem: bev_map_ids.ldmap_split_info )
    {
        if(elem.bev_lane_from == ego_bev_lane_id)
        {
            for(auto &p: elem.map_lanes_from)
            {
                ego_map_lane_ids.emplace_back(p);
            }
        }
    }
    if(ego_bev_lane_id > 0)
    {
        uint64_t current_map_id = 0;
        for(int i = 0; 0 == current_map_id && i < ego_map_lane_ids.size(); i++)
        {
            std::vector<Point>  ego_lane_points = GetMapPointByBevLaneId(ego_map_lane_ids[i],ld_map);
            for(int j = 0; 0 == current_map_id && j < ego_lane_points.size();j++)
            {
                if(ego_lane_points[j].x > -10.0f && ego_lane_points[j].x < 10.0f &&
                   ego_lane_points[j].y > -1.5f  && ego_lane_points[j].y < 1.5f)
                {
                    current_map_id = ego_map_lane_ids[i];
                }
            }
        }
        map_ego_lane_id = current_map_id;
    }

    return;
}
void BuildLaneTopology::GetMapTopoLinePoints(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    std::vector<map_id_point> &ego_lane_points,
    std::vector<map_id_point> &split_lane_points,
    std::vector<map_id_point> &main_lane_points,
    bool & is_split_left,
    const float & max_len)
{
    if(nullptr == ld_map)
    {
        return;
    }
    const float split_max_len = 30.0f;
    uint64_t ego_bev_lane_id = 0;
    std::vector<uint64_t> ego_map_lane_ids;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_bev_lane_id = bev_lane.id;
        }
    }
    for(auto & elem: bev_map_ids.ldmap_split_info )
    {
        if(elem.bev_lane_from == ego_bev_lane_id)
        {
            for(auto &p: elem.map_lanes_from)
            {
                ego_map_lane_ids.emplace_back(p);
            }
        }
    }
    if(ego_bev_lane_id > 0)
    {
        std::vector<Point> main_points;
        std::vector<Point> split_points;
        std::vector<Point> current_points;
        uint64_t current_map_id = 0;
        uint64_t main_lane_map_id = 0;
        uint64_t split_lane_map_id = 0;
        for(int i = 0; 0 == current_map_id && i < ego_map_lane_ids.size(); i++)
        {
            std::vector<Point>  ego_points = GetMapPointByBevLaneId(ego_map_lane_ids[i],ld_map);
            for(int j = 0; 0 == current_map_id && j < ego_points.size();j++)
            {
                if(ego_points[j].x > -10.0f && ego_points[j].x < 10.0f &&
                   ego_points[j].y > -1.5f  && ego_points[j].y < 1.5f)
                {
                    current_map_id = ego_map_lane_ids[i];
                }
            }
            if(current_map_id > 0 && ego_points.size() > 0)
            {
                map_id_point id_pts;
                id_pts.map_id = current_map_id;
                id_pts.line_points.swap(ego_points);
                ego_lane_points.emplace_back(id_pts);
            }
        }
        if(current_map_id > 0)
        {
            uint64_t ego_map_id = current_map_id;
            bool terminated = false;
            while(ego_map_id > 0 && !terminated)
            {
                int index = - 1;
                uint64_t next_ego_map_id = 0;
                for(size_t idx = 0; idx < ld_map->lanes.size() && !terminated; idx++)
                {
                    auto &map_lane = ld_map->lanes[idx];
                    if(map_lane.id == ego_map_id && map_lane.next_lane_ids.size() == 1)
                    {
                        index = idx;
                        next_ego_map_id = map_lane.next_lane_ids[0];
                    }
                }
                if(index >= 0 && ego_map_id != next_ego_map_id && next_ego_map_id > 0)
                {
                    std::vector<Point>  ego_points = GetMapPointByBevLaneId(next_ego_map_id,ld_map);
                    map_id_point id_pts;
                    id_pts.map_id = next_ego_map_id;
                    for(size_t pts_idx = 1; pts_idx < ego_points.size(); pts_idx++)
                    {//第一个点与前面段重复，需要去重
                       id_pts.line_points.emplace_back(ego_points[pts_idx]);
                    }
                    if(id_pts.line_points.size() > 0)
                    {
                        ego_lane_points.emplace_back(id_pts);
                        ego_map_id = next_ego_map_id;
                    }
                    else
                    {
                        terminated = true;
                    }
                }
                else
                {
                    terminated = true;
                }
            }
            current_map_id = ego_map_id;
        }
        uint64_t work_map_id = current_map_id;
        std::vector<uint64_t> spit_map_ids;
        bool is_terminated = false;
        while(work_map_id > 0 && !is_terminated)
        {
            bool non_exist = false;
            for(int idx = 0; idx < ld_map->lanes.size() && !is_terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == work_map_id)
                {
                    non_exist = true;
                    for(int pts_idx = 0 ; pts_idx < map_lane.points.size() && !is_terminated; pts_idx++)
                    {
                        if(map_lane.points[pts_idx].x < max_len)
                        {
                            current_points.emplace_back(map_lane.points[pts_idx]);
                        }
                        else
                        {
                            is_terminated = true;
                        }
                    }
                }
            }
            if(!non_exist)
            {
                is_terminated = true;
                continue;
            }
            for(int idx = 0; idx < ld_map->lanes.size() && !is_terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == work_map_id)
                {
                    if(map_lane.next_lane_ids.size() == 1)
                    {
                        work_map_id = map_lane.next_lane_ids[0];
                    }
                    else if(map_lane.next_lane_ids.size() == 2)
                    {
                        for(auto &id: map_lane.next_lane_ids)
                        {
                            spit_map_ids.emplace_back(id);
                        }
                        is_terminated = true;
                    }
                    else
                    {
                        std::vector<map_id_point>().swap(ego_lane_points);
                        std::vector<map_id_point>().swap(split_lane_points);
                        std::vector<map_id_point>().swap(main_lane_points);
                        is_terminated = true;
                        return;
                    }
                }
            }
        }
        std::map<uint64_t,std::vector<Point>> main_split_points;
        for(int idx = 0; idx < ld_map->lanes.size(); idx++)
        {
            auto &map_lane = ld_map->lanes[idx];
            for(auto &id: spit_map_ids)
            {
                if(map_lane.id == id)
                {//可能需要多个map id，否则不够30米
                    if(map_lane.points.size() >= 2)
                    {
                        std::vector<Point> one_map_line_points;
                        for(int pts_idx = 0 ; pts_idx < map_lane.points.size(); pts_idx++)
                        {
                            //if(map_lane.points[pts_idx].x >= current_points.back().x && map_lane.points[pts_idx].x < (current_points.back().x + split_max_len))
                            {
                                one_map_line_points.emplace_back(map_lane.points[pts_idx]);
                            }
                        }
                        if(one_map_line_points.size() > 1)
                        {
                            main_split_points.insert(std::pair<uint64_t,std::vector<Point>>(id,one_map_line_points));
                        }
                    }
                }
            }
        }
        uint64_t main_map_id = 0;
        uint64_t split_map_id = 0;
        for(auto & elem: bev_map_ids.ldmap_split_info )
        {
            if(LdMapProcessor::MergeSplitType::kSplitLeft == elem.match_type)
            {
                uint64_t max_id = 0;
                float max_y = -FLT_MAX;
                uint64_t min_id = 0;
                float min_y = FLT_MAX;
                for(auto &elem: main_split_points)
                {
                    if(elem.second.size() > 1)
                    {
                        if(elem.second[1].y > max_y)
                        {
                            max_id = elem.first;
                            max_y = elem.second[1].y;
                        }
                        if(elem.second[1].y < min_y)
                        {
                            min_id = elem.first;
                            min_y = elem.second[1].y;
                        }
                    }
                }
                if(max_y > -FLT_MAX)
                {//left
                    for(auto &elem: main_split_points)
                    {
                        if(max_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                split_points.emplace_back(pt);
                            }
                        }
                    }
                    split_lane_map_id = max_id;
                    map_id_point id_pts;
                    id_pts.map_id = split_lane_map_id;
                    id_pts.line_points.swap(split_points);
                    if(id_pts.line_points.size() > 0)
                    {
                        split_lane_points.emplace_back(id_pts);
                    }
                }
                if(min_y < FLT_MAX)
                {//main
                    for(auto &elem: main_split_points)
                    {
                        if(min_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                main_points.emplace_back(pt);
                            }
                        }
                    }
                    main_lane_map_id = min_id;
                    map_id_point id_pts;
                    id_pts.map_id = main_lane_map_id;
                    id_pts.line_points.swap(main_points);
                    if(id_pts.line_points.size() > 0)
                    {
                        main_lane_points.emplace_back(id_pts);
                    }
                }
                is_split_left = true;
            }
            if(LdMapProcessor::MergeSplitType::kSplitRight == elem.match_type)
            {
                uint64_t max_id = 0;
                float max_y = -FLT_MAX;
                uint64_t min_id = 0;
                float min_y = FLT_MAX;
                for(auto &elem: main_split_points)
                {
                    if(elem.second.size() > 1)
                    {
                        if(elem.second[1].y > max_y)
                        {
                            max_id = elem.first;
                            max_y = elem.second[1].y;
                        }
                        if(elem.second[1].y < min_y)
                        {
                            min_id = elem.first;
                            min_y = elem.second[1].y;
                        }
                    }
                }
                if(max_y > -FLT_MAX)
                {//main
                    for(auto &elem: main_split_points)
                    {
                        if(max_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                main_points.emplace_back(pt);
                            }
                        }
                    }
                    main_lane_map_id = max_id;
                    map_id_point id_pts;
                    id_pts.map_id = main_lane_map_id;
                    id_pts.line_points.swap(main_points);
                    if(id_pts.line_points.size() > 0)
                    {
                        main_lane_points.emplace_back(id_pts);
                    }
                }
                if(min_y < FLT_MAX)
                {//right
                    for(auto &elem: main_split_points)
                    {
                        if(min_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                split_points.emplace_back(pt);
                            }
                        }
                    }
                    split_lane_map_id = min_id;
                    map_id_point id_pts;
                    id_pts.map_id = split_lane_map_id;
                    id_pts.line_points.swap(split_points);
                    if(id_pts.line_points.size() > 0)
                    {
                        split_lane_points.emplace_back(id_pts);
                    }
                }
                is_split_left = false;
            }
        }
    }

    if(split_lane_points.size() > 0)
    {
        uint64_t split_map_id = split_lane_points[0].map_id;
        bool terminated = false;
        while(split_map_id > 0 && !terminated)
        {
            int index = - 1;
            uint64_t next_split_map_id = 0;
            for(size_t idx = 0; idx < ld_map->lanes.size() && !terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == split_map_id && map_lane.next_lane_ids.size() == 1)
                {
                    index = idx;
                    next_split_map_id = map_lane.next_lane_ids[0];
                }
                else if(map_lane.next_lane_ids.size() >= 2)
                {
                    terminated = true;
                    continue;
                }
                else
                {
                    terminated = true;
                    continue;
                }
            }
            if(index >= 0 && split_map_id != next_split_map_id && next_split_map_id > 0)
            {
                std::vector<Point>  split_points = GetMapPointByBevLaneId(next_split_map_id,ld_map);
                if(split_points.size() <= 0 || (split_points.size() > 0 && split_points.front().x > FRONT_MAX_RANGE - 20))
                {
                    terminated = true;
                }
                map_id_point id_pts;
                id_pts.map_id = next_split_map_id;
                for(size_t pts_idx = 1; pts_idx < split_points.size(); pts_idx++)
                {//第一个点与前面段重复，需要去重
                    if(split_points[pts_idx].x < FRONT_MAX_RANGE)
                    {
                        id_pts.line_points.emplace_back(split_points[pts_idx]);
                    }
                }
                if(id_pts.line_points.size() > 2)
                {
                    split_lane_points.emplace_back(id_pts);
                    split_map_id = next_split_map_id;
                }
                else
                {
                    terminated = true;
                }
            }
            else
            {
                terminated = true;
            }
        }
    }
    if(main_lane_points.size() > 0)
    {
        uint64_t main_map_id = main_lane_points[0].map_id;
        bool terminated = false;
        while(main_map_id > 0 && !terminated)
        {
            int index = - 1;
            uint64_t next_main_map_id = 0;
            for(size_t idx = 0; idx < ld_map->lanes.size() && !terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == main_map_id && map_lane.next_lane_ids.size() == 1)
                {
                    index = idx;
                    next_main_map_id = map_lane.next_lane_ids[0];
                }
                else if(map_lane.next_lane_ids.size() >= 2)
                {
                    terminated = true;
                    continue;
                }
                else
                {
                    terminated = true;
                    continue;
                }
            }
            if(index >= 0 && main_map_id != next_main_map_id && next_main_map_id > 0)
            {
                std::vector<Point>  main_points = GetMapPointByBevLaneId(next_main_map_id,ld_map);
                if(main_points.size() <= 0 || (main_points.size() > 0 && main_points.front().x > FRONT_MAX_RANGE - 20))
                {
                    terminated = true;
                }
                map_id_point id_pts;
                id_pts.map_id = next_main_map_id;
                for(size_t pts_idx = 1; pts_idx < main_points.size(); pts_idx++)
                {//第一个点与前面段重复，需要去重
                    if(main_points[pts_idx].x < FRONT_MAX_RANGE)
                    {
                        id_pts.line_points.emplace_back(main_points[pts_idx]);
                    }
                }
                if(id_pts.line_points.size() > 2)
                {
                    main_lane_points.emplace_back(id_pts);
                    main_map_id = next_main_map_id;
                }
                else
                {
                    terminated = true;
                }
            }
            else
            {
                terminated = true;
            }
        }
    }
    return;
}
void BuildLaneTopology::OtherLineWidthSorting(std::vector<bev_important_line_point> & bilp)
{
    int j = 0;
    for(int i = 1; i < bilp.size(); i++)
    {
        auto &tmp = bilp[i];
        j = i - 1;
        while(j >= 0 && bilp[j].avg_width > tmp.avg_width)
        {
            bilp[j+1] = bilp[j];
            j--;
        }
        bilp[j+1] = tmp;
    }
    return;
}
void BuildLaneTopology::GetMapSegmentPointForNoTopo(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    std::vector<map_id_point> & ego_lane_points,
    std::vector<map_id_point> & split_lane_points,
    std::vector<map_id_point> & main_lane_points,
    std::vector<MotionObject2Lane> & objstacleInfoForLdMap,
     std::vector<segment_map_as_bev_info>  & segment_info)
{

    return;
}

void BuildLaneTopology::GetMapSegmentPointForSplit(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    const bool & map_split_has_motion_objs,
    std::vector<map_id_point> & ego_lane_points,
    std::vector<map_id_point> & split_lane_points,
    std::vector<map_id_point> & main_lane_points,
    std::vector<MotionObject2Lane> & objstacleInfoForLdMap,
     std::vector<segment_map_as_bev_info>  & segment_info)
{
    segment_map_as_bev_info bev_map_ego_lane;
    segment_map_as_bev_info bev_map_main_lane;
    segment_map_as_bev_info bev_map_split_lane;

    bool map_ego_lane_split_topo_exist = false;
    Point split_point;
    Point ego_end_split_point;
    if(split_lane_points.size() > 0 && main_lane_points.size() > 0)
    {
        map_ego_lane_split_topo_exist = true;
        if(split_lane_points.begin()->line_points.size() > 0 &&
           main_lane_points.begin()->line_points.size() > 0 &&
           fabs(split_lane_points.begin()->line_points.front().x - main_lane_points.begin()->line_points.front().x) < 0.1)
        {
            bev_map_main_lane.split_point = split_lane_points.begin()->line_points.front();
            bev_map_split_lane.split_point = split_lane_points.begin()->line_points.front();
        }
    }
    if(split_lane_points.size() > 0 && ego_lane_points.size() > 0)
    {
        if(split_lane_points.begin()->line_points.size() > 0 &&
           ego_lane_points[ego_lane_points.size()-1].line_points.size() > 0 &&
           fabs(split_lane_points.begin()->line_points.front().x - ego_lane_points[ego_lane_points.size()-1].line_points.back().x) < 0.1)
        {
            bev_map_ego_lane.split_point = ego_lane_points[ego_lane_points.size()-1].line_points.back();
        }
    }
    float lower_limit_value = 0.0f;
    float ego_last_x = 0;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) && bev_lane.line_points.size() > 0)
        {
            ego_last_x = bev_lane.line_points.back().x;
            if(bev_lane.line_points.back().x < 3.0f && bev_map_split_lane.split_point.x <= 0.0f && bev_map_split_lane.split_point.x > -5.0f)
            {//CNOAC2-122709,track自车道在车后.
                lower_limit_value = bev_map_split_lane.split_point.x - 0.5;
            }
        }
    }

    if(true == map_ego_lane_split_topo_exist &&
        bev_map_split_lane.split_point.x > lower_limit_value &&
        bev_map_split_lane.split_point.x < SPLIT_TOPO_DISTANCE)
    {
        bev_map_main_lane.ego_lane_split_topo_exist = false;
        bev_map_split_lane.ego_lane_split_topo_exist = false;
        bev_map_ego_lane.ego_lane_split_topo_exist = false;
        bev_map_main_lane.bev_split_type = MAP_MAIN_LANE_NEW_INTERNAL_TYPE;
        bev_map_split_lane.bev_split_type = MAP_SPLIT_LANE_NEW_INTERNAL_TYPE;
        bev_map_ego_lane.bev_split_type = MAP_EGO_LANE_NEW_INTERNAL_TYPE;

        //查找bev前方是否有other车道线
        int next_id_size = 0;
        int ego_lane_idx = -1;
        int ego_split_seg_pts_idx = -1;
        std::vector<Point2DF> new_ego_lane_points;
        std::vector<bev_important_line_point> other_line_points;
        float ego_lane_last_point_x = -1;
        for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
        {
            auto &bev_lane = local_map->lane_infos[idx];
            if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER) ||
                bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) ||
                bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST))
            {
                //确认other线不是其他拓扑的next和previous！！！
                if(bev_lane.next_lane_ids.size() > 0 || bev_lane.previous_lane_ids.size() > 0 || bev_lane.line_points.size() < 3 )
                {
                    continue;
                }
                //确保other不是特殊车道类型
                if(bev_lane.lane_type == BevLaneType::LANE_TYPE_UNKNOWN ||
                   bev_lane.lane_type == BevLaneType::LANE_TYPE_SIDE ||
                   bev_lane.lane_type == BevLaneType::LANE_TYPE_OTHER ||
                   bev_lane.lane_type == BevLaneType::LANE_TYPE_MAIN )
                {
                    bev_important_line_point bilp;
                    bilp.bev_id = bev_lane.id;
                    bilp.bev_index = idx;
                    for(auto &pt: bev_lane.line_points)
                    {
                        bilp.line_points.emplace_back(pt);
                    }
                    if(bev_lane.line_points.size() > 0 &&
                      (bev_lane.line_points.back().x < bev_map_ego_lane.split_point.x  ||
                       bev_lane.line_points.front().x < ego_last_x))
                    {//在车后或候选的线front与ego线的back差异很大不选
                        continue;
                    }
                    if(bev_lane.line_points.size() > 0 && fabs(bev_lane.line_points.front().x - bev_map_ego_lane.split_point.x) < SPLIT_TOPO_DIST)
                    {//另外太远的bev也不能用，在转弯时构建拓扑有异常！
                        other_line_points.emplace_back(bilp);
                    }
                }
            }
            if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
            {
                next_id_size = bev_lane.next_lane_ids.size();
                bev_map_ego_lane.origin_bev_id = bev_lane.id;
                bev_map_ego_lane.origin_bev_index = idx;
                for(size_t k = 0; k < bev_lane.line_points.size(); k++)
                {
                    if(bev_lane.line_points[k].x >= bev_map_ego_lane.split_point.x)
                    {//判断自车道是否超出地图split点
                        ego_split_seg_pts_idx = k;
                        break;
                    }
                    else
                    {
                        new_ego_lane_points.emplace_back(bev_lane.line_points[k]);
                    }
                    ego_lane_last_point_x = bev_lane.line_points.back().x;
                }
            }
        }
        if(next_id_size <= 1)
        {
            std::vector<Eigen::Vector2f> split_line;
            std::vector<Eigen::Vector2f> main_line;
            for(auto &elem: split_lane_points)
            {
                for(auto &pt: elem.line_points)
                {
                    Eigen::Vector2f pts;
                    pts(0) = pt.x;
                    pts(1) = pt.y;
                    if(pt.x < FRONT_MAX_RANGE)
                    {
                        split_line.emplace_back(pts);
                    }
                }
            }
            for(auto &elem: main_lane_points)
            {
                for(auto &pt: elem.line_points)
                {
                    Eigen::Vector2f pts;
                    pts(0) = pt.x;
                    pts(1) = pt.y;
                    if(pt.x < FRONT_MAX_RANGE)
                    {
                        main_line.emplace_back(pts);
                    }
                }
            }
            const float lane_width = 3.0;
            std::vector<bev_important_line_point> split_other_line_points;
            std::set<uint64_t> split_id_set;
            if(split_line.size() > 2)
            {//计算与地图split车道线线近的other车道线
                for(auto &elem: other_line_points)
                {
                    if(elem.line_points.size() < 3)
                    {
                        continue;
                    }
                    std::vector<float> widths;
                    float avg_width = 10.0;
                    float max_width = 0.0;
                    float sum_width = 0;
                    for(auto &pt: elem.line_points)
                    {
                        if(pt.x < split_line.front()(0) || pt.x > split_line.back()(0))
                        {
                            continue;
                        }
                        Eigen::Vector2f point;
                        point(0) = pt.x;
                        point(1) = pt.y;
                        float width = Point2LineDistanece(point,split_line);
                        sum_width = sum_width + width;
                        max_width = max(max_width,width);
                        widths.emplace_back(width);
                    }
                    if(widths.size() > 2)
                    {
                        avg_width = sum_width/(widths.size()*1.0);
                    }
                    if(avg_width < lane_width/2.0 && max_width < lane_width)
                    {
                        bev_important_line_point bilp;
                        bilp.bev_id = elem.bev_id;
                        bilp.bev_index = elem.bev_index;
                        for(auto &pt: elem.line_points)
                        {
                            bilp.line_points.emplace_back(pt);
                        }
                        bilp.avg_width = avg_width;
                        bilp.max_width = max_width;
                        split_other_line_points.emplace_back(bilp);
                        split_id_set.insert(elem.bev_id);
                    }
                }
            }
            std::vector<bev_important_line_point> main_other_line_points;
            if(main_line.size() > 2)
            {//计算与地图主路main车道线线近的other车道线
                for(auto &elem: other_line_points)
                {
                    if(elem.line_points.size() < 3 || split_id_set.count(elem.bev_id)/*一条中心线只能属于split或主路*/)
                    {
                        continue;
                    }
                    std::vector<float> widths;
                    float avg_width = 10.0;
                    float max_width = 0.0;
                    float sum_width = 0;
                    for(auto &pt: elem.line_points)
                    {
                        if(pt.x < main_line.front()(0) || pt.x > main_line.back()(0))
                        {
                            continue;
                        }
                        Eigen::Vector2f point;
                        point(0) = pt.x;
                        point(1) = pt.y;
                        float width = Point2LineDistanece(point,main_line);
                        sum_width = sum_width + width;
                        max_width = max(max_width,width);
                        widths.emplace_back(width);
                    }
                    if(widths.size() > 2)
                    {
                        avg_width = sum_width/(widths.size()*1.0);
                    }
                    if(avg_width < lane_width/2.0 && max_width < lane_width )
                    {
                        bev_important_line_point bilp;
                        bilp.bev_id = elem.bev_id;
                        for(auto &pt: elem.line_points)
                        {
                            bilp.line_points.emplace_back(pt);
                        }
                        bilp.avg_width = avg_width;
                        bilp.max_width = max_width;
                        //最小平均间隔的插入到最前面，优先使用。
                        main_other_line_points.emplace_back(bilp);
                    }
                }
            }
            if(main_other_line_points.size() <= 0 && split_other_line_points.size() > 0 &&
               true == map_split_has_motion_objs && /*没有车流补需要去补形点，应该感知bev训练数据，比如CNOAC2-119414误构建*/
               ego_lane_last_point_x > (bev_map_ego_lane.split_point.x + SPLIT_TOPO_DIST))
            {
                int new_eog_other_lane_id = -1;
                Eigen::Vector2f segment_point;
                segment_point(0) = bev_map_ego_lane.split_point.x;
                segment_point(1) = bev_map_ego_lane.split_point.y;
                SegmentLaneTwoParts(local_map,
                                    bev_map_ego_lane.origin_bev_id,
                                    segment_point,
                                    false,
                                    new_eog_other_lane_id);
                if(new_eog_other_lane_id > 0)
                {
                    int ego_other_index = -1;
                    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
                    {
                        auto &bev_lane = local_map->lane_infos[idx];
                        if(new_eog_other_lane_id == bev_lane.id)
                        {
                            ego_other_index = idx;
                            break;
                        }
                    }
                    bev_important_line_point bilp;
                    bilp.bev_id = new_eog_other_lane_id;
                    for(int idx = 0; ego_other_index >= 0 && idx < local_map->lane_infos[ego_other_index].line_points.size(); idx++)
                    {
                        bilp.line_points.emplace_back(local_map->lane_infos[ego_other_index].line_points[idx]);
                    }
                    bilp.avg_width = 0;
                    bilp.max_width = 0;
                    main_other_line_points.emplace_back(bilp);
                }
            }
            OtherLineWidthSorting(split_other_line_points);
            OtherLineWidthSorting(main_other_line_points);

            size_t SEG_LOWER_POINT_INDEX = 2;
            size_t SEG_UPPER_POINT_INDEX = 15;
            bool is_need_create_topo = false;
            if(next_id_size == 1)
            {//(1) 有一个拓扑需要明确是哪个split还是主路，返回标记；另外一个分支需要补齐另外缺失。
                //【暂时关闭】
            }
            else if(next_id_size == 0)
            {//(2) 缺少拓扑： 1） 有2根中心线；2）有1根中心线；3）没有中心线, 若bev存在没有拓扑的other车道线太远，不需要，需要根据地图重新生成。
                if(split_other_line_points.size() > 0 && main_other_line_points.size() > 0)
                {//1. split&main感知bev两条拓扑车道线存在,需要处理track提供的形点重叠问题。
                    for(size_t i = 0; i < split_other_line_points.size(); i++)
                    {
                        auto & elem = split_other_line_points[i];
                        for(size_t idx = 0; idx < elem.line_points.size(); idx++)
                        {
                            Point pts;
                            pts.x = elem.line_points[idx].x;
                            pts.y = elem.line_points[idx].y;
                            bev_map_split_lane.all_line_points.emplace_back(pts);
                            bev_map_split_lane.seg_line_points.emplace_back(pts);
                        }
                        bev_map_split_lane.origin_bev_id = elem.bev_id;
                        break;
                    }
                    for(size_t i = 0; i < main_other_line_points.size(); i++)
                    {
                        auto & elem = main_other_line_points[i];
                        for(size_t idx = 0; idx < elem.line_points.size(); idx++)
                        {
                            Point pts;
                            pts.x = elem.line_points[idx].x;
                            pts.y = elem.line_points[idx].y;
                            bev_map_main_lane.all_line_points.emplace_back(pts);
                            bev_map_main_lane.seg_line_points.emplace_back(pts);
                        }
                        bev_map_main_lane.origin_bev_id = elem.bev_id;
                        break;
                    }

                    bev_map_split_lane.use_map_bev = false;
                    bev_map_main_lane.use_map_bev = false;
                }
                if(split_other_line_points.size() > 0 && main_other_line_points.size() <= 0)
                {//2. split: 感知bev的split存在，主路main: 需要根据地图补。
                    for(size_t i = 0; i < split_other_line_points.size(); i++)
                    {
                        auto & elem = split_other_line_points[i];
                        for(size_t idx = 0; idx < elem.line_points.size(); idx++)
                        {
                            Point pts;
                            pts.x = elem.line_points[idx].x;
                            pts.y = elem.line_points[idx].y;
                            bev_map_split_lane.all_line_points.emplace_back(pts);
                            bev_map_split_lane.seg_line_points.emplace_back(pts);
                        }
                        bev_map_split_lane.origin_bev_id = elem.bev_id;
                        break;
                    }
                    int count = 0;
                    for(size_t i = 0; count <= SEG_UPPER_POINT_INDEX && i < main_lane_points.size(); i++)
                    {
                        for(size_t j = 0; count <= SEG_UPPER_POINT_INDEX && j < main_lane_points[i].line_points.size(); j++)
                        {
                            auto &pt = main_lane_points[i].line_points[j];
                            if(j >= SEG_LOWER_POINT_INDEX && j <= SEG_UPPER_POINT_INDEX)
                            {
                                bev_map_main_lane.seg_line_points.push_back(pt);
                            }
                            bev_map_main_lane.all_line_points.emplace_back(pt);
                            count++;
                        }
                        bev_map_main_lane.map_ld_list.emplace_back(main_lane_points[i].map_id);
                    }
                    bev_map_split_lane.use_map_bev = false;
                    bev_map_main_lane.use_map_bev = true;
                    is_need_create_topo = true;
                }
                if(split_other_line_points.size() <= 0 && main_other_line_points.size() > 0)
                {//3. 主路main: bev存在，split:需要根据地图补,自车道很长，没有split线的不支持！
                    int count = 0;
                    for(size_t i = 0; count <= SEG_UPPER_POINT_INDEX && i < split_lane_points.size(); i++)
                    {
                        for(size_t j = 0; count <= SEG_UPPER_POINT_INDEX && j < split_lane_points[i].line_points.size(); j++)
                        {
                            auto &pt = split_lane_points[i].line_points[j];
                            if(j >= SEG_LOWER_POINT_INDEX && j <= SEG_UPPER_POINT_INDEX)
                            {
                                bev_map_split_lane.seg_line_points.push_back(pt);
                            }
                            bev_map_split_lane.map_ld_list.emplace_back(split_lane_points[i].map_id);
                            bev_map_split_lane.all_line_points.emplace_back(pt);
                            count++;
                        }
                    }
                    for(size_t i = 0; i < main_other_line_points.size(); i++)
                    {
                        auto & elem = main_other_line_points[i];
                        for(size_t idx = 0; idx < elem.line_points.size(); idx++)
                        {
                            Point pts;
                            pts.x = elem.line_points[idx].x;
                            pts.y = elem.line_points[idx].y;
                            bev_map_main_lane.all_line_points.emplace_back(pts);
                            bev_map_main_lane.seg_line_points.emplace_back(pts);
                        }
                        bev_map_main_lane.origin_bev_id = elem.bev_id;
                        if(elem.bev_id == bev_map_ego_lane.origin_bev_id)
                        {//主路的id和ego自车道id一样不构建
                            bev_map_ego_lane.ego_lane_split_topo_exist = true;
                        }
                        break;
                    }
                    bev_map_split_lane.use_map_bev = true;
                    bev_map_main_lane.use_map_bev = false;
                }
                if(split_other_line_points.size() <= 0 && main_other_line_points.size() <= 0)
                {//【暂时关闭】4. main主路和split分支: 在bev都不存在，需要根据地图形点补.
                    int count = 0;
                    for(size_t i = 0; count <= SEG_UPPER_POINT_INDEX && i < main_lane_points.size(); i++)
                    {
                        for(size_t j = 0; count <= SEG_UPPER_POINT_INDEX && j < main_lane_points[i].line_points.size(); j++)
                        {
                            auto &pt = main_lane_points[i].line_points[j];
                            if(j >= SEG_LOWER_POINT_INDEX && j <= SEG_UPPER_POINT_INDEX)
                            {
                                bev_map_main_lane.seg_line_points.push_back(pt);
                            }
                            bev_map_main_lane.all_line_points.emplace_back(pt);
                            count++;
                        }
                        bev_map_main_lane.map_ld_list.emplace_back(main_lane_points[i].map_id);
                    }
                    int count2 = 0;
                    for(size_t i = 0; count2 <= SEG_UPPER_POINT_INDEX && i < split_lane_points.size(); i++)
                    {
                        for(size_t j = 0; count2 <= SEG_UPPER_POINT_INDEX && j < split_lane_points[i].line_points.size(); j++)
                        {
                            auto &pt = split_lane_points[i].line_points[j];
                            if(j >= SEG_LOWER_POINT_INDEX && j <= SEG_UPPER_POINT_INDEX)
                            {
                                bev_map_split_lane.seg_line_points.push_back(pt);
                            }
                            bev_map_split_lane.map_ld_list.emplace_back(split_lane_points[i].map_id);
                            bev_map_split_lane.all_line_points.emplace_back(pt);
                            count2++;
                        }
                    }
                    bev_map_split_lane.use_map_bev = true;
                    bev_map_main_lane.use_map_bev = true;
                    //NOTE: bev前面主路和split都没有，暂时不处理，所以设置存在拓扑跳过不处理。
                    bev_map_ego_lane.ego_lane_split_topo_exist = true;
                }
            }
            if(ego_lane_idx >= 0 && ego_split_seg_pts_idx >= 0 && true == is_need_create_topo)
            {//有split拓扑，自车道bev形点超出split，需要删除
                int removed_count = 0;
                Eigen::Vector2f segment_point(0.0,0.0);
                bool is_segment_point = false;
                for(size_t idx = 0; idx < local_map->lane_infos[ego_lane_idx].line_points.size(); idx++)
                {
                    if(local_map->lane_infos[ego_lane_idx].line_points[idx].x > bev_map_ego_lane.split_point.x)
                    {
                        removed_count++;
                        if(!is_segment_point)
                        {
                            segment_point(0) = local_map->lane_infos[ego_lane_idx].line_points[idx].x;
                            segment_point(1) = local_map->lane_infos[ego_lane_idx].line_points[idx].y;
                            is_segment_point = true;
                        }
                    }
                }
                //int new_lane_id = 0;
                //SegmentLaneTwoParts(local_map,bev_map_ego_lane.origin_bev_id,segment_point,false,new_lane_id);
                local_map->lane_infos[ego_lane_idx].line_points.swap(new_ego_lane_points);
            }
            std::set<uint64_t> remove_other_line_ids;
            for(size_t idx = 0; idx < split_other_line_points.size(); idx++)
            {
                if(idx >= 1)
                {
                    remove_other_line_ids.insert(split_other_line_points[idx].bev_id);
                }
            }
            for(size_t idx = 0; idx < main_other_line_points.size(); idx++)
            {
                if(idx >= 1)
                {
                    remove_other_line_ids.insert(main_other_line_points[idx].bev_id);
                }
            }

            segment_info.emplace_back(bev_map_ego_lane);
            segment_info.emplace_back(bev_map_main_lane);
            segment_info.emplace_back(bev_map_split_lane);
        }
        else
        {
            bev_map_ego_lane.ego_lane_split_topo_exist = true;
        }
    }
    return;
}

void BuildLaneTopology::GetMapNewSegment(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids,
    const Eigen::Vector2f &segment_point,
    const float & max_len,
    int & new_lane_id)
{
    if(nullptr == ld_map)
    {
        return;
    }
    const float split_max_len = 30.0f;
    uint64_t ego_bev_lane_id = 0;
    std::vector<uint64_t> ego_map_lane_ids;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if (bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_bev_lane_id = bev_lane.id;
        }
    }
    for(auto & elem: bev_map_ids.ldmap_split_info )
    {
        if(elem.bev_lane_from == ego_bev_lane_id)
        {
            for(auto &p: elem.map_lanes_from)
            {
                ego_map_lane_ids.emplace_back(p);
            }
        }
    }
    if(ego_bev_lane_id > 0)
    {
        std::vector<Point> main_lane_points;
        std::vector<Point> split_lane_points;
        std::vector<Point> current_lane_points;
        float segment_x_coord = segment_point(0);
        uint64_t current_map_id = 0;
        uint64_t main_lane_map_id = 0;
        uint64_t split_lane_map_id = 0;
        for(int i = 0; 0 == current_map_id && i < ego_map_lane_ids.size(); i++)
        {
            std::vector<Point>  ego_lane_points = GetMapPointByBevLaneId(ego_map_lane_ids[i],ld_map);
            for(int j = 0; 0 == current_map_id && j < ego_lane_points.size();j++)
            {
                if(ego_lane_points[j].x > -10.0f && ego_lane_points[j].x < 10.0f &&
                   ego_lane_points[j].y > -1.5f && ego_lane_points[j].y < 1.5f)
                {
                    current_map_id = ego_map_lane_ids[i];
                }
            }
        }
        uint64_t work_map_id = current_map_id;
        std::vector<uint64_t> spit_map_ids;
        bool is_terminated = false;
        while(work_map_id > 0 && !is_terminated)
        {
            for(int idx = 0; idx < ld_map->lanes.size() && !is_terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == work_map_id)
                {
                    for(int pts_idx = 0 ; pts_idx < map_lane.points.size() && !is_terminated; pts_idx++)
                    {
                        if(map_lane.points[pts_idx].x < max_len)
                        {
                            current_lane_points.emplace_back(map_lane.points[pts_idx]);
                        }
                        else
                        {
                            is_terminated = true;
                        }
                    }
                }
            }
            for(int idx = 0; idx < ld_map->lanes.size() && !is_terminated; idx++)
            {
                auto &map_lane = ld_map->lanes[idx];
                if(map_lane.id == work_map_id)
                {
                    if(map_lane.next_lane_ids.size() == 1)
                    {
                        work_map_id = map_lane.next_lane_ids[0];
                    }
                    else
                    {
                        for(auto &id: map_lane.next_lane_ids)
                        {
                            spit_map_ids.emplace_back(id);
                        }
                        is_terminated = true;
                    }
                }
            }
        }
        std::map<uint64_t,std::vector<Point>> split_points;
        for(int idx = 0; idx < ld_map->lanes.size(); idx++)
        {
            auto &map_lane = ld_map->lanes[idx];
            for(auto &id: spit_map_ids)
            {
                if(map_lane.id == id)
                {//可能需要多个map id，否则不够30米
                    if(map_lane.points.size() >= 2)
                    {
                        std::vector<Point> one_map_line_points;
                        for(int pts_idx = 0 ; pts_idx < map_lane.points.size(); pts_idx++)
                        {
                            if(map_lane.points[pts_idx].x >= current_lane_points.back().x && map_lane.points[pts_idx].x < (current_lane_points.back().x + split_max_len))
                            {
                                one_map_line_points.emplace_back(map_lane.points[pts_idx]);
                            }
                        }
                        if(one_map_line_points.size() > 1)
                        {
                            split_points.insert(std::pair<uint64_t,std::vector<Point>>(id,one_map_line_points));
                        }
                    }
                }
            }
        }
        uint64_t main_map_id = 0;
        uint64_t split_map_id = 0;
        float ego_back_y = 0.0;
        if(current_lane_points.size() > 0)
        {
            ego_back_y = current_lane_points.back().y;
        }
        for(auto & elem: bev_map_ids.ldmap_split_info )
        {
            if(LdMapProcessor::MergeSplitType::kSplitLeft == elem.match_type)
            {
                uint64_t max_id = 0;
                float max_y = -FLT_MAX;
                uint64_t min_id = 0;
                float min_y = FLT_MAX;
                for(auto &elem: split_points)
                {
                    if(elem.second.size() > 1)
                    {
                        if(elem.second[1].y > max_y)
                        {
                            max_id = elem.first;
                            max_y = elem.second[1].y;
                        }
                        if(elem.second[1].y < min_y)
                        {
                            min_id = elem.first;
                            min_y = elem.second[1].y;
                        }
                    }
                }
                if(max_y > -FLT_MAX)
                {//left
                    for(auto &elem: split_points)
                    {
                        if(max_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                split_lane_points.emplace_back(pt);
                            }
                        }
                    }
                    split_lane_map_id = max_id;
                }
                if(min_y < FLT_MAX)
                {//main
                    for(auto &elem: split_points)
                    {
                        if(min_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                main_lane_points.emplace_back(pt);
                            }
                        }
                    }
                    main_lane_map_id = min_id;
                }
            }
            if(LdMapProcessor::MergeSplitType::kSplitRight == elem.match_type)
            {
                uint64_t max_id = 0;
                float max_y = -FLT_MAX;
                uint64_t min_id = 0;
                float min_y = FLT_MAX;
                for(auto &elem: split_points)
                {
                    if(elem.second.size() > 1)
                    {
                        if(elem.second[1].y > max_y)
                        {
                            max_id = elem.first;
                            max_y = elem.second[1].y;
                        }
                        if(elem.second[1].y < min_y)
                        {
                            min_id = elem.first;
                            min_y = elem.second[1].y;
                        }
                    }
                }
                if(max_y > -FLT_MAX)
                {//right
                    for(auto &elem: split_points)
                    {
                        if(max_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                main_lane_points.emplace_back(pt);
                            }
                        }
                    }
                    main_lane_map_id = max_id;
                }
                if(min_y < FLT_MAX)
                {//main
                    for(auto &elem: split_points)
                    {
                        if(min_id == elem.first)
                        {
                            for(auto &pt: elem.second)
                            {
                                split_lane_points.emplace_back(pt);
                            }
                        }
                    }
                    split_lane_map_id = min_id;
                }
            }
        }
    }

    return;
}
void BuildLaneTopology::BuildMergeTopo(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids)
{
    if(nullptr == local_map)
    {
        return;
    }
    bool is_build_left_lane = false;
    static Lane_ID_Storage store_merge_topo;
    //累计构建拓扑距离，防止持续构建。
    static float all_dist_acc  = 0.0f;
    //检测自车道在最左边并逐渐消失
    std::vector<Eigen::Vector2f> source_line;
    int bev_left_lanemarker_id = -1;
    int bev_right_lanemarker_id = -1;
    bool has_no_left_lane = true;
    bool source_next_merge_topo_exist = false;
    int origin_next_id = 0;
    static int prev_origin_next_id = 0;
    static bool is_blocked = false;
    bool is_in_ego_line = false;
    uint32_t source_position = 0;
    uint32_t target_left_position = 0;
    uint32_t target_right_position = 0;
    uint32_t source_lane_id = 0;
    uint32_t target_left_lane_id = 0;
    uint32_t target_right_lane_id = 0;
    uint32_t ego_lane_id = 0;
    uint32_t current_ego_lane_id = 0;
    uint32_t current_ego_left_anemarker_id = 0;
    uint32_t current_ego_right_lanemarker_id = 0;
    for (size_t i = 0; i < local_map->lane_infos.size(); i++)
    {
        auto &bev_lane = local_map->lane_infos[i];
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) ||
           bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_SECOND) ||
           bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_THIRD))
        {
            has_no_left_lane = false;
        }
        if(target_left_lane_id == 0 && bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST))
        {
            target_left_lane_id = bev_lane.id;
            target_left_position = bev_lane.position;
        }
        if(target_right_lane_id == 0 && bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST))
        {
            target_right_lane_id = bev_lane.id;
            target_right_position = bev_lane.position;
        }
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            current_ego_lane_id = bev_lane.id;
            current_ego_left_anemarker_id = bev_lane.left_lane_marker_id;;
            current_ego_right_lanemarker_id = bev_lane.right_lane_marker_id;
        }
        if(bev_lane.position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) ||
           (store_merge_topo.source_lane_id != 0 && store_merge_topo.source_lane_id != bev_lane.id))
        {
            continue;
        }
        if(bev_lane.is_blocked == true && !is_blocked)
        {
            is_blocked = true;
        }
        ego_lane_id = bev_lane.id;
        source_lane_id = bev_lane.id;
        source_position = bev_lane.position;
        if(bev_lane.next_lane_ids.size() == 1)
        {
            origin_next_id = bev_lane.next_lane_ids[0];
        }
        if(bev_lane.next_lane_ids.size() >= 2)
        {
            return;
        }
        bev_left_lanemarker_id = bev_lane.left_lane_marker_id;
        bev_right_lanemarker_id = bev_lane.right_lane_marker_id;
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            for(auto &pt: bev_lane.line_points)
            {
                Eigen::Vector2f pts;
                pts(0) = pt.x;
                pts(1) = pt.y;
                source_line.push_back(pts);
                if(fabs(pt.y) < 1.0f &&  fabs(pt.x) < 5.0f)
                {
                    is_in_ego_line = true;
                }
            }
        }
    }

    if(false == is_in_ego_line && false == store_merge_topo.need_to_build_topo)
    {
        return;
    }
    bool exist_bev_merge_topo = false;
    for (size_t i = 0; i < local_map->lane_infos.size(); i++)
    {
        auto &bev_lane = local_map->lane_infos[i];
        if(bev_lane.previous_lane_ids.size() == 2 && origin_next_id == bev_lane.id)
        {
            for(auto &id : bev_lane.previous_lane_ids)
            {
                if(id == store_merge_topo.source_lane_id)
                {
                    source_next_merge_topo_exist = true;
                    break;
                }
            }
        }
        if(bev_lane.previous_lane_ids.size() == 2 && origin_next_id == bev_lane.id)
        {
            for(auto &id : bev_lane.previous_lane_ids)
            {
                if(id == store_merge_topo.target_lane_id)
                {
                    exist_bev_merge_topo = true;
                    break;
                }
            }
        }
    }

    if(true == source_next_merge_topo_exist && true == exist_bev_merge_topo && true == store_merge_topo.need_to_build_topo)
    {//感知bev又突然生成了对应构建拓扑相邻车道的topo，不构建拓扑。
        for (size_t i = 0; i < local_map->lane_infos.size(); i++)
        {
            auto &bev_lane = local_map->lane_infos[i];
            if(bev_lane.previous_lane_ids.size() == 2 && origin_next_id == bev_lane.id)
            {//自车道的next的车道中心线的前继的id是自车道，应清除。
                std::vector<uint64_t> previous_lane_ids;
                for(auto &id : bev_lane.previous_lane_ids)
                {
                    if(id != store_merge_topo.source_lane_id)
                    {
                        previous_lane_ids.push_back(id);
                    }
                }
                bev_lane.previous_lane_ids.swap(previous_lane_ids);
            }
            if(bev_lane.next_lane_ids.size() == 1 && bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
            {//自车道的后继next应该清除。
                std::vector<uint64_t>().swap(bev_lane.next_lane_ids);
            }
        }
    }

    std::vector<Eigen::Vector2f> left_edge_line;
    std::vector<Eigen::Vector2f> right_edge_line;
    for (size_t i = 0;  i < local_map->edges.size() ; i++)
    {
        auto & edge = local_map->edges[i];
        if(edge.position == static_cast<int>(BevRoadEdgePosition::BEV_REP__LEFT))
        {
            for(int k = 0; k < edge.line_points.size() ; k++)
            {
                auto & pts = edge.line_points[k];
                Eigen::Vector2f p;
                p(0) = pts.x;
                p(1) = pts.y;
                left_edge_line.push_back(p);
            }
        }
        if(edge.position == static_cast<int>(BevRoadEdgePosition::BEV_REP__RIGHT) )
        {
            for(int k = 0; k < edge.line_points.size(); k++)
            {
                auto & pts = edge.line_points[k];
                Eigen::Vector2f p;
                p(0) = pts.x;
                p(1) = pts.y;
                right_edge_line.push_back(p);
            }
        }
    }

    std::vector<Eigen::Vector3f> ego_left_lm;
    std::vector<Eigen::Vector3f> ego_right_lm;
    int left_solid_lm_num_pts = 0;
    int right_solid_lm_num_pts = 0;
    bool left_has_virtual_point = false;
    bool right_has_virtual_point = false;
    float ego_left_lm_last_x = 0.0f;
    float ego_right_lm_last_x = 0.0f;
    if(source_line.size() >= 2)
    {
        float last_x = source_line[source_line.size()-1](0);
        for (size_t j = 0; j < local_map->lanemarkers.size(); j++)
        {
            auto& bev_lm = local_map->lanemarkers.at(j);
            if(bev_left_lanemarker_id == bev_lm.id)
            {
                for(int k = 0; k < bev_lm.line_points.size(); k++)
                {
                    auto &pt = bev_lm.line_points[k];
                    ego_left_lm_last_x = bev_lm.line_points.back().x;
                    if(pt.x > last_x || pt.x < 0)
                    {
                        continue;
                    }
                    bool is_left_solid = false;
                    for(auto &attr: bev_lm.type_segs)
                    {
                        if(k >= attr.start_index && k < attr.end_index &&
                           (attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__SOLID) ||
                            attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_SOLID) ||
                            attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_SOLID)))
                        {
                            is_left_solid = true;
                            left_solid_lm_num_pts++;
                        }
                        if(k >= attr.start_index && k < attr.end_index &&
                           attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__INVALID))
                        {
                            left_has_virtual_point = true;
                        }
                    }
                    Eigen::Vector3f left_pts;
                    left_pts(0) = pt.x;
                    left_pts(1) = pt.y;
                    left_pts(2) = 0.0f;
                    if(true == is_left_solid)
                    {
                        left_pts(2) = 1.0f;
                    }
                    ego_left_lm.emplace_back(left_pts);
                }
            }
            if(bev_right_lanemarker_id == bev_lm.id)
            {
                for(int k = 0; k < bev_lm.line_points.size(); k++)
                {
                    auto &pt = bev_lm.line_points[k];
                    ego_right_lm_last_x = bev_lm.line_points.back().x;
                    if(pt.x > last_x || pt.x < 0)
                    {
                        continue;
                    }
                    bool is_right_solid = false;
                    for(auto &attr: bev_lm.type_segs)
                    {
                        if(pt.x >= attr.start_index && pt.x < attr.end_index &&
                           (attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__SOLID) ||
                            attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__DOUBLE_SOLID_SOLID) ||
                            attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__DOUBLE_DASHED_SOLID)))
                        {
                            is_right_solid = true;
                            right_solid_lm_num_pts++;
                        }
                        if(k >= attr.start_index && k < attr.end_index &&
                           attr.type == static_cast<uint32_t>(BevLaneMarkerType::BEV_LMT__INVALID))
                        {
                            right_has_virtual_point = true;
                        }
                    }
                    Eigen::Vector3f right_pts;
                    right_pts(0) = pt.x;
                    right_pts(1) = pt.y;
                    right_pts(2) = 0.0f;
                    if(true == is_right_solid)
                    {
                        right_pts(2) = 1.0f;
                    }
                    ego_right_lm.emplace_back(right_pts);
                }
            }
        }
    }
    else
    {
        return;
    }
    if(false == left_has_virtual_point && false == right_has_virtual_point && false == store_merge_topo.need_to_build_topo)
    {//感知bev提供的信息，cnoa在自车道变窄直到消失，左或右车道线必须右虚拟车道线点,若是已经创建，就不判断比如CNOAC2-102455
        return;
    }
    //计算车前剩余中心线长度
    float remainning_lane_length = 0;
    static float prev_remainning_lane_length = 0.0f;
    int front_index = -1;
    for (int i = 0; i  < source_line.size(); ++i)
    {
        if(source_line[i](0) >= 0 && front_index < 0)
        {
            front_index = i;
            break;
        }
    }
    if(source_line.size() > 0)
    {
        float y_coord = source_line.back()(1);
        float y_coord2 = source_line.front()(1);
        for (int i = 0; i  < source_line.size(); ++i)
        {
            if(source_line[i](0) >= 0)
            {
                y_coord = std::max(y_coord,source_line[i](1));
                y_coord2 = std::min(y_coord2,source_line[i](1));
            }
        }
        if(fabs(y_coord - y_coord2) > 10.0f && false == store_merge_topo.need_to_build_topo)
        {
            return;
        }
    }

    for (int i = front_index; front_index >= 0 && i + 1 < source_line.size(); ++i)
    {
        remainning_lane_length = remainning_lane_length + (source_line.at(i + 1) - source_line.at(i)).norm();
    }
    if(true == source_next_merge_topo_exist)
    {
        store_merge_topo.has_origin_merge_topo = true;
    }

    static int valid_count = 0;
    Ego_Lane_Crop_Info Judged_Result;
    Judged_Result.remove_start_point(0) = 0.0;
    Judged_Result.remove_start_point(1) = 0.0;
    float lane_width_limit = LANE_MIN_DIST;
    float topo_stride = MERGE_TOPO_DIST;
    if(target_right_lane_id > 0 && target_left_lane_id > 0)
    {//逐渐消失的窄车道在道路中间，由于车速过快，需要远一点开发构建merge拓扑。
        lane_width_limit = 2.2f;
        topo_stride = 1.5f*MERGE_TOPO_DIST;
    }
    bool ifCropped = JudgeIfBuildMergeTopo(local_map,
                                           source_line,
                                           ego_left_lm,
                                           ego_right_lm,
                                           left_edge_line,
                                           right_edge_line,
                                           current_speed,
                                           lane_width_limit,
                                           store_merge_topo,
                                           Judged_Result);

    //CNOA/MNOA形点判断是否构建merge的关键,因为AI感知会超相邻车道车道线虚拟
    int adjacent_lane_id = 0;
    bool is_selected_adjacent_left_lane = false;
    bool is_selected_adjacent_right_lane = false;
    std::vector<Eigen::Vector2f>  adjacent_lane_width_values;
    bool is_adjacent_avail = ComputeEgoToAdjacentLaneWidth(
        local_map,
        store_merge_topo,
        valid_count,
        Judged_Result,
        adjacent_lane_id,
        Judged_Result.has_narrow_lane,
        is_selected_adjacent_left_lane,
        is_selected_adjacent_right_lane,
        adjacent_lane_width_values);

    uint32_t target_lane_id = 0;
    uint32_t target_position = 0;
    if(((true == Judged_Result.right_build_merge ||
        true == store_merge_topo.need_to_build_topo) &&
        store_merge_topo.target_lane_id == target_right_lane_id) ||
        (false == store_merge_topo.need_to_build_topo &&
        true == is_selected_adjacent_right_lane))
    {
        is_build_left_lane = false;
        Judged_Result.right_build_merge  = true;
        target_lane_id = target_right_lane_id;
        target_position = target_right_position;
    }
    else if(((true == Judged_Result.left_build_merge ||
        true == store_merge_topo.need_to_build_topo) &&
        store_merge_topo.target_lane_id == target_left_lane_id) ||
        (false == store_merge_topo.need_to_build_topo &&
        true == is_selected_adjacent_left_lane))
    {
        is_build_left_lane = true;
        Judged_Result.left_build_merge = true;
        target_lane_id = target_left_lane_id;
        target_position = target_left_position;
    }
    for (size_t i = 0; i < local_map->lane_infos.size(); i++)
    {
        auto &bev_lane = local_map->lane_infos[i];
        if(target_lane_id == bev_lane.id && bev_lane.lane_type >= BevLaneType::LANE_TYPE_SIDE)
        {//非正常车道类型,不构建!
            return;
        }
    }
    if(true == Judged_Result.has_narrow_lane)
    {
       store_merge_topo.has_narrow_lane = true;
    }
    bool is_fade_away = false;
    static int invalid_count = 0;
    static int check_target_lane_count = 0;
    if(true == Judged_Result.has_narrow_lane || is_blocked)
    {//自车道前方有小于2.5米宽或有断头路,且自车道逐渐消失判断
        std::vector<Eigen::Vector2f>  lane_width_values;
        is_fade_away = ComputeTwoLanesWidth(local_map,
                                            source_lane_id,
                                            current_speed,
                                            bev_left_lanemarker_id,
                                            bev_right_lanemarker_id,
                                            lane_width_values);

        if(false == is_fade_away)
        {
            is_fade_away = Computelane2EdgeWidth(local_map, source_line,left_edge_line);
        }
        if(false == is_fade_away && true == is_adjacent_avail)
        {
            is_fade_away = true;
        }
        valid_count++;
        if(false == is_ICC_mode && true == is_blocked && valid_count <= 3)
        {//在3次窄路判断时，NOA模式下一旦有断头路不创建merge拓扑
            store_merge_topo.need_to_build_topo = false;
            prev_remainning_lane_length = 0.0f;
            return;
        }
    }
    if(false == is_fade_away && remainning_lane_length < DETECT_LIMIT && fabs(remainning_lane_length - prev_remainning_lane_length) > 5.0f)
    {
        invalid_count++;
    }
    PLanningResultPtr planning_result{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(planning_result);
    if(nullptr != planning_result)
    {
        if(planning_result->lc_state != PLanningResultData::Keeping && valid_count < 3)
        {//在判断3次满足构建merge拓扑时，若存在规控变道，不构建merge拓扑！
            return;
        }
    }

    if((false == is_fade_away && false == store_merge_topo.need_to_build_topo) || //自车道没有小于2.5米且逐渐变窄消失
      (false == Judged_Result.left_build_merge && false == Judged_Result.right_build_merge) || //自车道左右有路沿或都是实线
       /*true == Judged_Result.has_right_edge ||  //最右边有路沿的暂时不考虑*/
       (valid_count < 3 && prev_origin_next_id <= 0 && false == store_merge_topo.need_to_build_topo)) //统计三次以上
    {
        prev_remainning_lane_length = remainning_lane_length;
        bool target_lane_is_avail = JudgeTargetLaneIfAvail(local_map,current_speed,left_edge_line,right_edge_line,target_lane_id);
        if(target_lane_id > 0 && false == target_lane_is_avail)
        {////merge生成的目标车道不具备正常行驶条件: 连续在3帧满足窄车道，如果都检查相邻车道不具备merge，后面都不生成merge，防止感知检测不稳定，生成merge也不稳定。
            check_target_lane_count++;
        }
        prev_origin_next_id = origin_next_id;
        return;
    }
    prev_origin_next_id = origin_next_id;
    if(check_target_lane_count > 3)
    {
        return;
    }
    double Tn = local_map->header.timestamp;
    if(true == store_merge_topo.first_remove)
    {
        Eigen::Vector2f res = getStartPointInDR(Tn, store_merge_topo);
        Judged_Result.remove_start_point(0) = res(0);
    }
    if((store_merge_topo.source_lane_id > 0 && source_lane_id > 0 && store_merge_topo.source_lane_id != source_lane_id ||
       store_merge_topo.target_lane_id > 0 && target_lane_id > 0 && store_merge_topo.target_lane_id != target_lane_id) &&
       true == store_merge_topo.need_to_build_topo)
    {//ORIN-349936,构建了merge拓扑，然后感知评测到另外左边车道继续构建不需要的拓扑
        store_merge_topo.need_to_build_topo = false;
        return;
    }
    if(Judged_Result.remove_start_point(0) <= 10.0f && valid_count >= 3 && valid_count <= 5)
    {//开始构建的第一和第二帧，判断分割点在车前20米后就不构建merge拓扑,以防止无法控车。
        store_merge_topo.need_to_build_topo = false;
        is_fade_away = 0;
        all_dist_acc = MERGE_TOPO_FINISHED_DISTANCE+1.0f;
    }
    store_merge_topo.is_blocked = is_blocked;
    store_merge_topo.source_lane_id = source_lane_id;
    store_merge_topo.source_left_lanemarker_id = bev_left_lanemarker_id;
    store_merge_topo.source_right_lanemarker_id = bev_right_lanemarker_id;
    store_merge_topo.source_position = source_position;
    store_merge_topo.target_lane_id = target_lane_id;
    store_merge_topo.left_lane_id = target_left_lane_id;
    store_merge_topo.right_lane_id = target_right_lane_id;
    store_merge_topo.target_position = target_position;

    //判断是否生成车前merge拓扑,通过merge时还没有到前面段，车后也需要生成拓扑连接.
    if((Judged_Result.left_build_merge || Judged_Result.right_build_merge) && //左右相邻车道能够构建merge拓扑（没有路沿，没有实线）
        (is_fade_away || true == store_merge_topo.need_to_build_topo) && //自车道前方宽度小于2.5米或有断头路，而且自车道逐渐变窄直到消失
        source_line.size() > 2 && //自车道有线或过了拓扑，车后有原有的自车道线，否则无法构建merge拓扑
        Judged_Result.remove_start_point(0) < DETECT_LIMIT && //80米内才构建merge拓扑，太远了感知det检测不稳定。
        ((true == store_merge_topo.has_new_merge_topo) || //经过自车道还未达到相邻车道
        (source_line.back()(0) > 0 && prev_remainning_lane_length > 0.0f &&  //自车道前方随着行车逐渐变短
        fabs(remainning_lane_length - prev_remainning_lane_length) < 3.0f)))
    {
        bool is_running_on_topo = false;
        if(source_line.size() > 0 && source_line.back()(0) < 0 && target_position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) )
        {//NOTE: 另外需要考虑还没有到右边车道，右边车道已经是ego，但是不满足规控的半个车道-0.15米，然后会降级ACC。
            is_running_on_topo = true;
        }
        int ego_lane_seg_idx = front_index;
        Eigen::Vector2f removed_start_point(0.0,0.0);
        removed_start_point(0) = Judged_Result.remove_start_point(0);
        removed_start_point(1) = Judged_Result.remove_start_point(1);

        int target_left_lanemarker_id = -1;
        int target_right_lanemarker_id = -1;
        float target_y_coord = 0.0;
        Eigen::Vector2f target_lane_last_pts(0.0,0.0);
        Eigen::Vector2f target_left_last_pts(0.0,0.0);
        Eigen::Vector2f target_right_last_pts(0.0,0.0);
        for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
        {
            auto &bev_lane = local_map->lane_infos[idx];
            if(bev_lane.id == target_lane_id )
            {
                target_lane_last_pts(0) = bev_lane.line_points.back().x;
                target_lane_last_pts(1) = bev_lane.line_points.back().y;
                target_left_lanemarker_id = bev_lane.left_lane_marker_id;
                target_right_lanemarker_id = bev_lane.right_lane_marker_id;
                int index = -1;
                float minX = FLT_MAX;
                for(size_t k = 0; k < bev_lane.line_points.size(); k++)
                {
                    if(source_line.size() > 0 && fabs(bev_lane.line_points[k].x - source_line.back()(0)) < minX)
                    {
                        index = k;
                        minX = fabs(bev_lane.line_points[k].x - source_line.back()(0));
                    }
                }
                if(index >= 0)
                {
                    target_y_coord = bev_lane.line_points[index].y;
                }
                break;
            }
        }
        for (size_t idx = 0; idx < local_map->lanemarkers.size(); idx++)
        {
            auto &bev_lm = local_map->lanemarkers[idx];
            if(bev_lm.id == target_left_lanemarker_id && bev_lm.line_points.size() > 0)
            {
                target_left_last_pts(0) = bev_lm.line_points.back().x;
                target_left_last_pts(1) = bev_lm.line_points.back().y;
            }
            if(bev_lm.id == target_right_lanemarker_id && bev_lm.line_points.size() > 0 )
            {
                target_right_last_pts(0) = bev_lm.line_points.back().x;
                target_right_last_pts(1) = bev_lm.line_points.back().y;
            }
        }
        std::set<int> target_lanemarker_ids;
        if(target_left_lanemarker_id > 0 && target_right_lanemarker_id > 0)
        {//裁剪车道要谨慎，其他车道也共用车道线问题，裁剪会导致旁边车道变短无法使用。
            int other_lanemarker_id = -1;
            for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
            {
                auto &bev_lane = local_map->lane_infos[idx];
                if(bev_lane.id != source_lane_id )
                {
                    if (bev_left_lanemarker_id == bev_lane.left_lane_marker_id || bev_left_lanemarker_id == bev_lane.right_lane_marker_id)
                    {
                        other_lanemarker_id = bev_left_lanemarker_id;
                    }
                    if (target_right_lanemarker_id == bev_lane.left_lane_marker_id || target_right_lanemarker_id == bev_lane.right_lane_marker_id)
                    {
                        other_lanemarker_id = target_right_lanemarker_id;
                    }
                }
            }
            if(other_lanemarker_id > 0)
            {
                target_lanemarker_ids.insert(other_lanemarker_id);
            }
            target_lanemarker_ids.insert(target_left_lanemarker_id);
            target_lanemarker_ids.insert(target_right_lanemarker_id);
        }
        bool need_to_crop = true;
        float max_len = removed_start_point(0) + MERGE_TOPO_DIST;
        if(max_len > (target_lane_last_pts(0) - 5.0) ||
           max_len > (target_left_last_pts(0) - 5.0) ||
           max_len > (target_right_last_pts(0) - 5.0))
        {
            need_to_crop = false;
        }
        bool need_to_segment = true;
        int new_lane_id = -1;
        if(false == store_merge_topo.need_to_build_topo &&
          fabs(target_lane_last_pts(0) - Judged_Result.remove_start_point(0)) < MERGE_TOPO_DIST)
        {//弯道处，黑夜或雨雪天气，远方有逐渐消失车道，这个判断是线都差不多长无法构建merge拓扑
            need_to_segment = false;
        }
        if(source_line.size() > 0 && fabs(target_y_coord) > 0 && fabs(source_line.back()(1) - target_y_coord) > 3*LANE_MIN_DIST)
        {//自车道左拐（感知检测出窄车道），右边相邻车道比较长，需要判断横向距离
            need_to_segment = false;
        }
        if(true == store_merge_topo.need_to_build_topo && (Judged_Result.remove_start_point(0) + topo_stride + 10.0f) < 0.0f)
        {//已经超过了变道到相邻车道线分割点
            need_to_segment = false;
        }
        if(false == store_merge_topo.need_to_build_topo &&
           (ego_left_lm_last_x > 0 && fabs(target_lane_last_pts(0) - ego_left_lm_last_x) < (MERGE_TOPO_DIST+ 10.0f))||
           (ego_right_lm_last_x > 0 && fabs(target_lane_last_pts(0) - ego_right_lm_last_x) < (MERGE_TOPO_DIST+ 10.0f)))
        {//自车道中心线比较短，导致误构建merge拓扑
            //need_to_segment = false;
        }

        if(fabs(target_right_last_pts(0) - target_left_last_pts(0)) >= 10.0  && true == need_to_segment)
        {//补齐左右车道，依据路沿和实线
            #if 0
            alignLaneMarker(local_map,
                            target_lane_id,
                            target_left_lanemarker_id,
                            target_right_lanemarker_id,
                            target_lane_last_pts,
                            target_left_last_pts,
                            target_right_last_pts);
            #endif
        }
        if( true == need_to_segment)
        {
            Eigen::Vector2f src_lane_end_point(0.0,0.0);
            bool is_removed_invalid = RemoveInvalidLaneSeg(local_map,
                                                      ego_lane_id,
                                                      removed_start_point,
                                                      target_lanemarker_ids,
                                                      src_lane_end_point);
            //DR Pn X, 为了稳定连接，使用DR设置的固定连接点，不再使用剪切车道点。
            src_lane_end_point(0) = Judged_Result.remove_start_point(0) + topo_stride;
            SegmentLaneTwoParts(local_map,target_lane_id,src_lane_end_point,is_running_on_topo,new_lane_id);
        }
        if(new_lane_id > 0 )
        {
            for (size_t lane_idx = 0; lane_idx < local_map->lane_infos.size(); lane_idx++)
            {
                auto &bev_lane = local_map->lane_infos[lane_idx];
                if(bev_lane.id == ego_lane_id)
                {
                    bev_lane.next_lane_ids.clear();
                }
                if(bev_lane.id == target_lane_id)
                {
                    bev_lane.next_lane_ids.clear();
                }
            }

            for (size_t lane_idx = 0; lane_idx < local_map->lane_infos.size(); lane_idx++)
            {
                auto &bev_lane = local_map->lane_infos[lane_idx];
                if(bev_lane.id == new_lane_id)
                {
                    bev_lane.previous_lane_ids.push_back(ego_lane_id);
                    bev_lane.previous_lane_ids.push_back(target_lane_id);
                    bev_lane.connect_type = BevLaneConnectType::NORMAL;
                    bev_lane.is_build_merge = true;
                }
                if(bev_lane.id == target_lane_id)
                {
                    bev_lane.next_lane_ids.push_back(new_lane_id);
                    bev_lane.connect_type = BevLaneConnectType::NORMAL;
                    bev_lane.is_build_merge = true;
                }
                if(bev_lane.id == ego_lane_id)
                {
                    bev_lane.connect_type = BevLaneConnectType::MERGE;
                    bev_lane.next_lane_ids.push_back(new_lane_id);
                    bev_lane.is_build_merge = true;
                    bev_lane.is_build_merge_left = is_build_left_lane;
                }
            }
            store_merge_topo.need_to_build_topo = true;
            if(false == store_merge_topo.first_remove)
            {
                store_merge_topo.T0 = Tn;
                store_merge_topo.P0 = Judged_Result.remove_start_point;
                store_merge_topo.Twb0_r = DR2BodyTrans(store_merge_topo.T0);
                Eigen::Vector3d pts;
                pts(0) = store_merge_topo.P0(0);
                pts(1) = store_merge_topo.P0(1);
                pts(2) = 0;
                TransPoint(pts, store_merge_topo.Twb0_r);
                store_merge_topo.P0(0) = pts(0);
                store_merge_topo.P0(1) = pts(1);
                store_merge_topo.first_remove = true;
            }
        }

        for (size_t lane_idx = 0; lane_idx < local_map->lane_infos.size(); lane_idx++)
        {
            auto &bev_lane = local_map->lane_infos[lane_idx];
            if(bev_lane.id == store_merge_topo.source_lane_id && bev_lane.next_lane_ids.size() > 0)
            {
                store_merge_topo.has_new_merge_topo = true;
            }
        }
    }
    prev_remainning_lane_length = remainning_lane_length;
    //reset: 还需要考虑没有merge的情况也要清除，为下一次做准备。
    static float  dist_acc  = 0.0f;
    static int count = 0;
    if(source_line.size() > 0 && source_line.back()(0 ) < 0 && true == store_merge_topo.has_new_merge_topo )
    {
        count++;
        dist_acc = count*0.05*current_speed;
    }

    static int count2 = 0;
    if(true == store_merge_topo.need_to_build_topo )
    {
        count2++;
        all_dist_acc = count2*0.05*current_speed;
    }
    if(target_position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) ||
       dist_acc >= MERGE_TOPO_DIST ||
       all_dist_acc >= MERGE_TOPO_FINISHED_DISTANCE)
    {
        store_merge_topo.is_blocked = false;
        store_merge_topo.source_lane_id = 0;
        store_merge_topo.source_position = 0;
        store_merge_topo.target_lane_id = 0;
        store_merge_topo.target_position = 0;
        store_merge_topo.has_new_merge_topo = false;
        store_merge_topo.T0 = 0.0;
        store_merge_topo.P0(0) = 0.0;
        store_merge_topo.P0(1) = 0.0;
        store_merge_topo.Twb0_r = Eigen::Isometry3d::Identity();
        store_merge_topo.first_remove = false;
        prev_remainning_lane_length = 0.0f;
        dist_acc = 0.0f;
        count = 0;
        store_merge_topo.has_narrow_lane = false;
        all_dist_acc = 0.0f;
        count2 = 0;
        store_merge_topo.need_to_build_topo = false;
        valid_count = 0;
        check_target_lane_count = 0;
    }
    return;
}

void BuildLaneTopology::BuildSplitTopoWithLdMap(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids)
{

    if(nullptr == local_map || nullptr == ld_map)
    {
        return;
    }
    //获取车流数据并绑定车道ID
    std::vector<MotionObject2Lane> objstacleInfoForLdMap;
    std::unordered_map<uint32_t, FusionObjs> traffic_flow_objs;
    #if 1 //暂时关闭这里不需要的车流，但是merge构建拓扑需要车流
    std::shared_ptr<TrafficFlowManager> pTrafficFlowManager = std::make_shared<TrafficFlowManager>();
    if(nullptr != pTrafficFlowManager)
    {
        pTrafficFlowManager->ParseTrafficFlowRawData(traffic_flow_objs);
        pTrafficFlowManager->BindObstaclesToLdMapLane(ld_map,objstacleInfoForLdMap);
    }

    #endif
    static Lane_ID_Storage traffic_flow_store_merge_topo;
    int ego_bev_lane_id = -1;
    static int prev_ego_bev_lane_id = -1;
    float bev_0_y = -10;
    float map_0_y = 10;
    const float max_len = FRONT_MAX_RANGE;
    int split_next_size = 0;
    uint64_t curr_bev_ego_id = 0;
    static uint64_t prev_bev_ego_id = 0;
    bool is_in_ego_line = false;
    for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
    {
        auto &bev_lane = local_map->lane_infos[idx];
        if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
        {
            ego_bev_lane_id = bev_lane.id;
            curr_bev_ego_id = bev_lane.id;
            if(true == bev_lane.is_build_split_by_map || bev_lane.line_points.size() < 2)
            {//上游已经创建拓扑且锁定ID跳变或自车道ego点数小于2,不需要构建!
                return;
            }
            if(bev_lane.next_lane_ids.size() > 0)
            {
                split_next_size = bev_lane.next_lane_ids.size();
            }
            for(auto &pt: bev_lane.line_points)
            {
                if(fabs(pt.y) < 1.0f &&  fabs(pt.x) < 5.0f)
                {
                    is_in_ego_line = true;
                }
                if(pt.x < 0.0f)
                {
                    continue;
                }
                if(bev_0_y <= -9.0f && pt.x < 5.0)
                {
                    bev_0_y = pt.y;
                    break;
                }
            }
            for(int k = static_cast<int>(bev_lane.line_points.size()) - 1; bev_lane.line_points.size() > 0 && k >= 0; k--)
            {
                if(bev_lane.line_points[k].x > 0.0f)
                {
                    continue;
                }
                if(bev_0_y <= -9.0f && bev_lane.line_points[k].x >= -5.0)
                {
                    bev_0_y = bev_lane.line_points[k].y;
                    break;
                }
            }
            break;
        }
    }
    if(false == is_in_ego_line && false == traffic_flow_store_merge_topo.need_to_build_topo)
    {
        return;
    }
    if(prev_bev_ego_id > 0 && curr_bev_ego_id > 0 && prev_bev_ego_id != curr_bev_ego_id &&
       true == traffic_flow_store_merge_topo.need_to_build_topo)
    {
        traffic_flow_store_merge_topo.need_to_build_topo = false;
        prev_bev_ego_id = 0;
        return;
    }
    prev_bev_ego_id = curr_bev_ego_id;
    uint64_t ego_map_lane_id = 0;
    GetMapEgoIdByMatch(local_map,ld_map,bev_map_ids,ego_map_lane_id);
    for (size_t idx = 0; ego_map_lane_id > 0 && idx < ld_map->lanes.size(); idx++)
    {
        auto &map_lane = ld_map->lanes[idx];
        if(map_lane.id == ego_map_lane_id)
        {
            std::vector<Point>  ego_lane_points = GetMapPointByBevLaneId(ego_map_lane_id,ld_map);
            int index = -1;
            float minX = FLT_MAX;
            for(int j = 0; j < ego_lane_points.size();j++)
            {
                if(ego_lane_points[j].x > -10.0f && ego_lane_points[j].x < 10.0f &&
                   ego_lane_points[j].y > -2.5f && ego_lane_points[j].y < 2.5f)
                {
                    if(fabs(ego_lane_points[j].x) < minX)
                    {
                        minX = fabs(ego_lane_points[j].x);
                        index = j;
                    }
                }
            }
            if(index >= 0)
            {
                map_0_y = ego_lane_points[index].y;
            }
            break;
        }
    }
    std::vector<map_id_point> ego_lane_points;
    std::vector<map_id_point> split_lane_points;
    std::vector<map_id_point> main_lane_points;
    bool is_split_at_left = false;
    GetMapTopoLinePoints(
        local_map,
        ld_map,
        bev_map_ids,
        ego_lane_points,
        split_lane_points,
        main_lane_points,
        is_split_at_left,
        max_len);

    if(ego_lane_points.size() == 0 &&
       split_lane_points.size() == 0 &&
       main_lane_points.size() == 0)
    {
        return;
    }
    Point split_point;
    if(split_lane_points.size() > 0 && main_lane_points.size() > 0)
    {
        if(split_lane_points.begin()->line_points.size() > 0 &&
           main_lane_points.begin()->line_points.size() > 0 &&
           fabs(split_lane_points.begin()->line_points.front().x - main_lane_points.begin()->line_points.front().x) < 0.1)
        {
            split_point = split_lane_points.begin()->line_points.front();
        }
    }
    if(split_lane_points.size() > 0 && ego_lane_points.size() > 0)
    {
        if(split_lane_points.begin()->line_points.size() > 0 &&
           ego_lane_points[ego_lane_points.size()-1].line_points.size() > 0 &&
           fabs(split_lane_points.begin()->line_points.front().x - ego_lane_points[ego_lane_points.size()-1].line_points.back().x) < 0.1)
        {
            split_point = ego_lane_points[ego_lane_points.size()-1].line_points.back();
        }
    }
    static float const max_limit = 30.0f;
    bool is_motion_objescts_exist_in_split = false;
    for(auto elem: bev_map_ids.ldmap_split_info)
    {
        if(ego_bev_lane_id == elem.bev_lane_from)
        {
            for(auto map_id: elem.map_lanes_from)
            {
                for(auto objs: objstacleInfoForLdMap)
                {
                    if(map_id == objs.lane_id && objs.motionObjects.size() > 0)
                    {
                        for(auto p: objs.motionObjects)
                        {
                            if(fabs(split_point.x - p.position.x) < max_limit)
                            {
                                is_motion_objescts_exist_in_split = true;
                            }
                        }
                    }
                }
            }
            for(auto objs: objstacleInfoForLdMap)
            {
                if(ego_bev_lane_id == objs.lane_id && objs.motionObjects.size() > 0)
                {
                    for(auto p: objs.motionObjects)
                    {
                        if(fabs(split_point.x - p.position.x) < 1.5*max_limit)
                        {
                            is_motion_objescts_exist_in_split = true;
                        }
                    }
                }
            }
            break;
        }
    }
    #if 0
    bool traffic_flow_exist = false;
    trafficFlowChecking(
        traffic_flow_objs,
        ego_lane_points,
        split_lane_points,
        main_lane_points,
        traffic_flow_exist);
    #endif
    bool is_need_build = true;
    BevAndMapChecking(local_map,ld_map,bev_map_ids,is_need_build);

    if(ego_bev_lane_id < 0 || ego_map_lane_id <= 0 || //bev和map的ego ID非法
      (map_0_y < 10 && bev_0_y > -10 && fabs(map_0_y - bev_0_y) > 1.0) || //track的ego算法的可能与地图的ego相差一个车道,尤其是变道中.
      false == is_need_build || //检查自车道、左一和右一的bev与map不一致。
      split_next_size > 0 ) //有拓扑不构建
    {
        return;
    }

    //判断是否穿过路沿,为了减少路沿计算量，前面判断约束后再计算路沿，case少，计算量少。
    bool edge_exist = false;
    LaneEdgeChecking(
        local_map,
        ego_lane_points,
        split_lane_points,
        main_lane_points,
        edge_exist);
    if(true == edge_exist)
    {//自车道、main主路和split存在路沿或靠路沿太近，不适合构建拓扑。
        return;
    }
    std::vector<segment_map_as_bev_info> segment_info;
    GetMapSegmentPointForSplit(
        local_map,
        ld_map,
        bev_map_ids,
        is_motion_objescts_exist_in_split,
        ego_lane_points,
        split_lane_points,
        main_lane_points,
        objstacleInfoForLdMap,
        segment_info);

    if(segment_info.size() >= 3 )
    {
        uint64_t ego_id = 0;
        size_t eog_index = 0;
        uint64_t split_id = 0;
        int split_index = -1;
        uint64_t main_id = 0;
        int main_index = -1;
        //ego
        for(auto &elem: segment_info)
        {
            if(elem.bev_split_type == MAP_EGO_LANE_NEW_INTERNAL_TYPE)
            {
                if(true == elem.ego_lane_split_topo_exist)
                {
                    return;
                }
                ego_id = elem.origin_bev_id;
            }
            if(elem.bev_split_type == MAP_MAIN_LANE_NEW_INTERNAL_TYPE)
            {
                if(true == elem.use_map_bev)
                {
                    uint64_t new_bev_main_id = generateNewLaneID(local_map);
                    main_id = new_bev_main_id;
                    BevLaneInfo new_lane;
                    new_lane.id = new_bev_main_id;
                    new_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
                    for(auto &pt: elem.seg_line_points)
                    {
                        Point2DF pts;
                        pts.x = pt.x;
                        pts.y = pt.y;
                        new_lane.line_points.emplace_back(pts);
                    }
                    local_map->lane_infos.emplace_back(new_lane);
                }
                else
                {
                    main_id = elem.origin_bev_id;
                    //TODO: 需要判断track给出车道且没有拓扑，但是接合处有重叠，无法构建正确拓扑线.
                }
                if(elem.split_point.x < -20.0f)
                {//自车已经过了split拓扑，不需要构建了。
                    traffic_flow_store_merge_topo.need_to_build_topo = false;
                    prev_bev_ego_id = 0;
                    return;
                }
            }
            if(elem.bev_split_type == MAP_SPLIT_LANE_NEW_INTERNAL_TYPE)
            {
                if(true == elem.use_map_bev)
                {
                    uint64_t new_bev_split_id = generateNewLaneID(local_map);
                    split_id = new_bev_split_id;
                    BevLaneInfo new_lane;
                    new_lane.id = new_bev_split_id;
                    new_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);
                    for(auto &pt: elem.seg_line_points)
                    {
                        Point2DF pts;
                        pts.x = pt.x;
                        pts.y = pt.y;
                        new_lane.line_points.emplace_back(pts);
                    }
                    local_map->lane_infos.emplace_back(new_lane);
                }
                else
                {
                    split_id = elem.origin_bev_id;
                    //TODO: 需要判断track给出车道且没有拓扑，但是接合处有重叠，无法构建正确拓扑线.
                }
            }
        }
        for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
        {
            auto &bev_lane = local_map->lane_infos[idx];
            if(bev_lane.id == main_id)
            {
               main_index = static_cast<int>(idx);
            }
            if(bev_lane.id == split_id)
            {
               split_index = static_cast<int>(idx);
            }
        }
        for (size_t idx = 0; idx < local_map->lane_infos.size(); idx++)
        {
            auto &bev_lane = local_map->lane_infos[idx];
            if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
            {
                if(main_index >= 0)
                {
                    bev_lane.next_lane_ids.emplace_back(main_id);
                    local_map->lane_infos[main_index].previous_lane_ids.emplace_back(bev_lane.id);
                    local_map->lane_infos[main_index].connect_type = BevLaneConnectType::NORMAL;
                    local_map->lane_infos[main_index].is_build_split = true;
                }
                if(split_index >= 0)
                {
                    bev_lane.next_lane_ids.emplace_back(split_id);
                    local_map->lane_infos[split_index].previous_lane_ids.emplace_back(bev_lane.id);
                    local_map->lane_infos[split_index].connect_type = BevLaneConnectType::SPLIT;
                    local_map->lane_infos[split_index].is_build_split = true;
                }
                bev_lane.connect_type = BevLaneConnectType::NORMAL;
                bev_lane.is_build_split = true;
                bev_lane.is_build_split_left = is_split_at_left;
                traffic_flow_store_merge_topo.need_to_build_topo = true;
            }
        }
    }
    else
    {
        traffic_flow_store_merge_topo.need_to_build_topo = false;
        prev_bev_ego_id = 0;
    }
    static float all_dist_acc  = 0.0f;
    static int count = 0;
    if(true == traffic_flow_store_merge_topo.need_to_build_topo )
    {
        count++;
        all_dist_acc = count*0.05*current_speed;
    }
    if( all_dist_acc >= (SPLIT_TOPO_DISTANCE + SPLIT_TOPO_DIST))
    {
        all_dist_acc = 0.0f;
        count = 0;
        traffic_flow_store_merge_topo.need_to_build_topo = false;
        prev_bev_ego_id = 0;
    }
    return ;
}
bool BuildLaneTopology::SegmentBevEgoLane(
    BevMapInfoPtr local_map,
    const uint64_t target_lane_id,
    const double distance_to_ego,
    uint64_t& new_lane_id,
    std::pair<cem::message::common::Point2DF, cem::message::common::Point2DF> target_bev_line
)
{
    constexpr double MaxDistSqMarker = 10.0 * 10.0;  // 目标添加拓扑点的最小距离（车道标志线）
    constexpr double MoveDistanceThreshold = 8.0;  // 目标添加拓扑点的最小距离（车道标志线）
    // 定义子函数，用于生成新的车道标记ID
    auto generate_lanemarker_id = [&]()
    {
        uint32_t new_id = 101;
        for (uint32_t i = 1; i < 100UL; ++i)
        {
            auto tmp = std::find_if(local_map->lanemarkers.begin(), local_map->lanemarkers.end(),
                                    [i](const cem::message::sensor::BevLaneMarker& lane_marker) {
                                return lane_marker.id == i;
            });
            if (tmp == local_map->lanemarkers.end())
            {
                new_id = i;
                break;
            }
        }
        return new_id;
    };

    bool break_success_flag = false;
    // 根据target_lane_id
    auto target_lane = std::find_if(local_map->lane_infos.begin(), local_map->lane_infos.end(),
                                    [target_lane_id](const cem::message::sensor::BevLaneInfo &lane) { return target_lane_id == lane.id; });

    if (target_lane == local_map->lane_infos.end()){
        return break_success_flag;
    }

    if (target_lane->line_points.size() < 4) {
        return break_success_flag;
    }

    // std::vector<cem::message::common::Point2DF>::iterator split_point;
    // bool get_split_point = false;
    // // 打断备选规则1：通过地图查询的拓扑点距离打断
    // for (auto iter = target_lane->line_points.begin();
    //           iter != target_lane->line_points.end(); ++iter) {
    //   auto temp_point = *iter;
    //   // 确保拓扑点符号与distance_to_ego一致
    //   if (temp_point.x * distance_to_ego > 0) {
    //     double temp_dist = copysign(1.0, distance_to_ego) * std::sqrt(std::pow(temp_point.x, 2) + std::pow(temp_point.y, 2));
    //     if (temp_dist > distance_to_ego) {
    //       get_split_point = true;
    //       split_point = iter;
    //       break;
    //     }
    //   }
    // }

    // if (get_split_point) {
    //   AINFO << "*******SplitPointResult******";
    //   AINFO << "timestamp: " << std::fixed << std::setprecision(3) << local_map->header.timestamp;
    //   AINFO << "sequence number: " << local_map->header.cycle_counter;
    //   AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
    //   AINFO << "*******************************";
    // }

    std::vector<cem::message::common::Point2DF>::iterator split_point1;
    double distance_pt2seg = std::numeric_limits<double>::max();
    // 打断备选规则2：找到距离最近点，向前挪动8m（暂定），如果打断点数不够就找最远的点
    for (auto iter = target_lane->line_points.begin() + 1; iter != target_lane->line_points.end(); ++iter)
    {
        // 求与输入的车道line_segment悬空车道起点的最近点
        auto temp_start = *(iter - 1);
        auto temp_end = *iter;
        double tmp_dist = CalculatePoint2SegmentDist(target_bev_line.first, temp_start, temp_end);
        if (distance_pt2seg > tmp_dist) {
            split_point1 = iter;
            distance_pt2seg = tmp_dist;
        }
    }

    // 定义合法的终止迭代器（避免越界）
    auto end_iter = target_lane->line_points.begin() + 2;
    // 没有足够的点打断
    if (split_point1 < end_iter) {
        return break_success_flag;
    }

    // 基于当前的split_point前移8m
    std::vector<cem::message::common::Point2DF>::iterator split_point = split_point1;
    for (auto iter = split_point1;
                iter != target_lane->line_points.begin() + 2; --iter) {
        // 求与输入的车道line_segment与代表悬空车道的line是否相交，找到第一个相交片段
        auto temp_point = *iter;
        double tmp_dist = CalculatePoint2PointDist(*split_point1, temp_point);
        split_point = iter;
        if (tmp_dist > MoveDistanceThreshold) {
            break;
        }
    }

    // AINFO << "*******SplitPointResultModify******";
    // AINFO << "timestamp: " << std::fixed << std::setprecision(3) << local_map->header.timestamp;
    // AINFO << "sequence number: " << local_map->header.cycle_counter;
    // AINFO << "split point: (" << split_point1->x << ", " << split_point1->y << ")";
    // AINFO << "split point: (" << split_point->x << ", " << split_point->y << ")";
    // AINFO << "*******************************";

    // if (!get_split_point){
    //   return break_success_flag;
    // }

    // 校验结果split_point是否可以分出足够数量的行点
    int break_lane1_pts_num = split_point - target_lane->line_points.begin();
    int break_lane2_pts_num = target_lane->line_points.end() - split_point;
    if (break_lane1_pts_num < 2 || break_lane2_pts_num < 2) {
        return break_success_flag;
    }

    // 打断车道中心线，改变中心线拓扑关系
    uint64_t new_id = 101;
    for (uint64_t i = 1; i < 100UL; ++i)
    {
        auto tmp = std::find_if(local_map->lane_infos.begin(), local_map->lane_infos.end(),
                                [i](const cem::message::sensor::BevLaneInfo& lane)
                                {
                                    return lane.id == i;
                                });
        if (tmp == local_map->lane_infos.end()) {
            new_id = i;
            break;
        }
    }
    auto breaked_lane = *target_lane;
    breaked_lane.line_points.clear();
    breaked_lane.line_points.insert(breaked_lane.line_points.end(), split_point,
                                    target_lane->line_points.end());
    breaked_lane.number_of_points = breaked_lane.line_points.size();
    breaked_lane.id = new_id;
    breaked_lane.position = static_cast<uint32_t>(BevLanePosition::LANE_LOC_OTHER);

    target_lane->line_points.erase(split_point, target_lane->line_points.end());
    target_lane->number_of_points = target_lane->line_points.size();
    target_lane->next_lane_ids.clear();
    target_lane->next_lane_ids.push_back(new_id);

    breaked_lane.previous_lane_ids.clear();
    breaked_lane.previous_lane_ids.push_back(target_lane->id);

    // 假如原始target_lane存在后继，修改后继lane的前继，将其改为打断后的
    for (const auto& next_lane_id : breaked_lane.next_lane_ids)
    {
        auto target_lane_next = std::find_if(local_map->lane_infos.begin(), local_map->lane_infos.end(),
                                    [next_lane_id](const cem::message::sensor::BevLaneInfo& lane) {
                                    return next_lane_id == lane.id;
            });
        if (target_lane_next == local_map->lane_infos.end())
        {
            continue;
        }
        std::replace(target_lane_next->previous_lane_ids.begin(),
                    target_lane_next->previous_lane_ids.end(), target_lane->id,
                    breaked_lane.id);
    }

    // 打断 lane_marker
    // 打断左边lane_marker
    auto id = target_lane->left_lane_marker_id;
    auto lane_line = std::find_if(
        local_map->lanemarkers.begin(), local_map->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return id == lane_marker.id;
        });
    if (lane_line != local_map->lanemarkers.end())
    {
        if (!lane_line->line_points.empty())
        {
            auto left_lane_marker_first = *lane_line;
            auto left_lane_marker_second = *lane_line;
            left_lane_marker_second.line_points.clear();

            double distance_square = std::numeric_limits<double>::max();
            std::vector<cem::message::common::Point2DF>::iterator marker_split_point;
            for (auto iter = left_lane_marker_first.line_points.begin();iter != left_lane_marker_first.line_points.end(); ++iter)
            {
                double tmp_square =
                    (split_point->x - iter->x) * (split_point->x - iter->x) +
                    (split_point->y - iter->y) * (split_point->y - iter->y);
                if (distance_square > tmp_square) {
                    marker_split_point = iter;
                    distance_square = tmp_square;
                }
            }

            if (distance_square < MaxDistSqMarker)
            {
                left_lane_marker_second.line_points.insert(
                    left_lane_marker_second.line_points.end(), marker_split_point, left_lane_marker_first.line_points.end());
                left_lane_marker_second.number_of_points = left_lane_marker_second.line_points.size();
                left_lane_marker_second.id = generate_lanemarker_id();
                breaked_lane.left_lane_marker_id = left_lane_marker_second.id;
                local_map->lanemarkers.push_back(std::move(left_lane_marker_second));

                left_lane_marker_first.line_points.erase(
                    marker_split_point, left_lane_marker_first.line_points.end());
                left_lane_marker_first.number_of_points = left_lane_marker_first.line_points.size();
                left_lane_marker_first.id = generate_lanemarker_id();
                target_lane->left_lane_marker_id = left_lane_marker_first.id;
                local_map->lanemarkers.push_back(std::move(left_lane_marker_first));
            }
        }
    }

    // 打断左边右侧lane_marker
    id = target_lane->right_lane_marker_id;
    lane_line = std::find_if(
        local_map->lanemarkers.begin(), local_map->lanemarkers.end(),
        [id](const cem::message::sensor::BevLaneMarker& lane_marker) {
            return id == lane_marker.id;
        });
    if (lane_line != local_map->lanemarkers.end())
    {
        if (!lane_line->line_points.empty())
        {
            auto right_lane_marker_first = *lane_line;
            auto right_lane_marker_second = *lane_line;
            right_lane_marker_second.line_points.clear();

            double distance_square = std::numeric_limits<double>::max();
            std::vector<cem::message::common::Point2DF>::iterator marker_split_point;
            for (auto iter = right_lane_marker_first.line_points.begin(); iter != right_lane_marker_first.line_points.end(); ++iter)
            {
                double tmp_square =
                    (split_point->x - iter->x) * (split_point->x - iter->x) +
                    (split_point->y - iter->y) * (split_point->y - iter->y);
                if (distance_square > tmp_square) {
                    marker_split_point = iter;
                    distance_square = tmp_square;
                }
            }

            if (distance_square < MaxDistSqMarker) {
                right_lane_marker_second.line_points.insert(
                    right_lane_marker_second.line_points.end(), marker_split_point,
                    right_lane_marker_first.line_points.end());
                right_lane_marker_second.number_of_points = right_lane_marker_second.line_points.size();
                right_lane_marker_second.id = generate_lanemarker_id();
                breaked_lane.right_lane_marker_id = right_lane_marker_second.id;
                local_map->lanemarkers.push_back(std::move(right_lane_marker_second));

                right_lane_marker_first.line_points.erase(
                    marker_split_point, right_lane_marker_first.line_points.end());
                right_lane_marker_first.number_of_points = right_lane_marker_first.line_points.size();
                right_lane_marker_first.id = generate_lanemarker_id();
                target_lane->right_lane_marker_id = right_lane_marker_first.id;
                local_map->lanemarkers.push_back(std::move(right_lane_marker_first));
            }
        }
    }

    local_map->lanemarkers_num = local_map->lanemarkers.size();
    local_map->lane_infos.push_back(std::move(breaked_lane));
    break_success_flag = true;

    return break_success_flag;
}

void BuildLaneTopology::GetBevEgoAdjoinLane(
    BevMapInfoPtr local_map,
    std::vector<Eigen::Vector2f> bev_geos,
    Eigen::Vector2f map_split_point,
    std::string bev_adjoin_type,
    uint32_t& bev_ego_adjoin_lane_id,
    Eigen::Vector2f segment_point,
    std::vector<Eigen::Vector2f> adjoin_lane_geos
)
{
    /*
        待创建拓扑的bev_ego_lane相邻车道起点至少在50m以外，感知车道30m内才会进行排序，
        直接找bev_ego_lane在起点在50m外的横向最近相邻车道
    */

    if(local_map == nullptr || bev_geos.empty() || bev_adjoin_type == "")
    {
        return;
    }

    std::vector<BevLaneInfo*> bigger_50m_bev_lanes;
    for(auto& bev_lane : local_map->lane_infos)
    {
        if(bev_lane.id == 0) continue;
        auto lane_pts = *bev_lane.geos;
        if(!lane_pts.empty())
        {
            float length = std::fabs(lane_pts.front().x() - lane_pts.back().x());
            if(lane_pts.front().x() >= 50.0 && length >= 20.0)
            // todo
            // 小于50m也得创建拓扑， 可以设置个标志位，创建过拓扑，且小于50m创建拓扑
            {
                bigger_50m_bev_lanes.emplace_back(&bev_lane);
            }
        }
    }

    for(auto& bev_lane : bigger_50m_bev_lanes)
    {
        auto lane_pts = *bev_lane->geos;
        std::pair<float, float> distance = {0.0, 0.0};
        bool flag = LaneGeometry::CalculateAverageDistanceBetweenTwoLines(
            lane_pts, bev_geos, distance);

        bool is_left_flag = LaneGeometry::JudgeIsLeft(lane_pts, bev_geos);
        std::string ad_type = is_left_flag ? "LEFT" : "RIGHT";

        float map_bev_adjoin_offset = std::fabs(map_split_point.x() - lane_pts.front().x());
        if( flag && distance.first > 0 && distance.first < 4.5 &&
            ad_type != "" && ad_type == bev_adjoin_type &&
            map_split_point.x() < lane_pts.front().x() &&
            map_bev_adjoin_offset < 50.0
        )
        {
            adjoin_lane_geos = lane_pts;
            segment_point = {lane_pts.front().x() - 10.0, lane_pts.front().y()};
            bev_ego_adjoin_lane_id = bev_lane->id;
            break;
        }
    }
}

bool BuildLaneTopology::IsNotRunningOnNaviLane(
    std::vector<Eigen::Vector2f> bev_geos,
    std::vector<Eigen::Vector2f> map_main_lane_geos,
    std::vector<Eigen::Vector2f> map_sub_lane_geos,
    Eigen::Vector2f map_split_point)
{
    if(bev_geos.empty() || map_main_lane_geos.empty() || map_sub_lane_geos.empty())
    {
        return false;
    }

    std::vector<Eigen::Vector2f> bev_ego_pts = bev_geos;
    std::sort(bev_ego_pts.begin(), bev_ego_pts.end(),
              [](const Eigen::Vector2f a, const Eigen::Vector2f b){ return a.x() > b.x();});

    std::vector<Eigen::Vector2f> map_bigger_split_point_pts;
    for(auto& point : map_main_lane_geos)
    {
        if(point.x() > map_split_point.x())
        {
            map_bigger_split_point_pts.push_back(point);
        }
    }

    if(bev_ego_pts.empty()|| map_bigger_split_point_pts.empty())
    {
        return false;
    }

    double dist_thresh  = 2.0;
    int far_point_cnt = 0;
    for(auto& pt: bev_ego_pts)
    {
        int outIndex = 0;
        double dist = 0.0;
        Eigen::Vector2f foot;
        bool flag = GeoMathUtil::hasMappingPointOnLine<Eigen::Vector2f>(pt, map_bigger_split_point_pts,  0, map_bigger_split_point_pts.size()-1, false, foot, dist, outIndex, 0.3);
        if(!flag) continue;
        if(dist > dist_thresh)
        {
            far_point_cnt++;
        }
    }

    bool is_not_nvi_road = far_point_cnt > 10? true : false;
    return is_not_nvi_road;
}

void BuildLaneTopology::GetBuildTopoLaneInfo(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    To_Build_Split_Topo_Lane_Info& to_build_topo_lane_info)
    {

        bool bev_ego_lane_no_topo = false;
        uint32_t bev_ego_lane_id = 0;
        std::vector<Eigen::Vector2f> bev_ego_lane_pts;
        for(auto& bev_lane : local_map->lane_infos)
        {
            if(bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO))
            {
                to_build_topo_lane_info.bev_ego_lane_id = bev_lane.id;
                if(bev_lane.previous_lane_ids.empty() && bev_lane.next_lane_ids.empty())
                {
                    to_build_topo_lane_info.bev_ego_lane_no_topo = true;
                    to_build_topo_lane_info.bev_geos = *bev_lane.geos;
                    to_build_topo_lane_info.bev_ego_lane_left_lanemarker_id = bev_lane.left_lane_marker_id;
                    to_build_topo_lane_info.bev_ego_lane_right_lanemarker_id = bev_lane.right_lane_marker_id;
                }
            }
        }
        // 增加地图与感知车道距离匹配，确定bev_ego_lane和地图map_ego_lane
        // 获取地图分离点
        {}

        // 判断bev_ego_lane与地图导航车道方向不一致
        to_build_topo_lane_info.bev_ego_lane_is_on_wrong_way =
        IsNotRunningOnNaviLane(
                to_build_topo_lane_info.bev_geos,
                to_build_topo_lane_info.map_main_lane_geos,
                to_build_topo_lane_info.map_sub_lane_geos,
                to_build_topo_lane_info.map_split_point);

        // 判断地图分离点是否有效
        if( to_build_topo_lane_info.map_split_point.x() <= 80.0 &&
            to_build_topo_lane_info.map_split_point.x() <= to_build_topo_lane_info.bev_geos.back().x() - 20)
        {
            to_build_topo_lane_info.map_split_point_valid = true;
        }

        // 判断地图主路和子路的位置关系
        bool main_lane_is_left =
            LaneGeometry::JudgeIsLeft(
                to_build_topo_lane_info.map_main_lane_geos,
                to_build_topo_lane_info.map_sub_lane_geos);

        to_build_topo_lane_info.bev_adjoin_type = main_lane_is_left? "LEFT" : "RIGHT";

        // 获取bev_ego_lane 起点在大于50m的相邻车道（待创建SPLIT拓扑）
        GetBevEgoAdjoinLane(
            local_map,
            to_build_topo_lane_info.bev_geos,
            to_build_topo_lane_info.map_split_point,
            to_build_topo_lane_info.bev_adjoin_type,
            to_build_topo_lane_info.bev_ego_adjoin_lane_id,
            to_build_topo_lane_info.segment_point,
            to_build_topo_lane_info.bev_adjoin_lane_geos);

        // 判断是否满足拓扑创建条件
        if( to_build_topo_lane_info.bev_ego_lane_no_topo  &&
            to_build_topo_lane_info.map_ego_lane_has_topo &&
            to_build_topo_lane_info.map_split_point_valid &&
            to_build_topo_lane_info.bev_ego_lane_is_on_wrong_way &&
            to_build_topo_lane_info.bev_ego_lane_length >= 20.0 &&
            to_build_topo_lane_info.bev_ego_adjoin_lane_id != 0)
        {
            to_build_topo_lane_info.ready_to_build_split_topo = true;
        }
    }

void BuildLaneTopology::BuildSplitTopo(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map)
{

    To_Build_Split_Topo_Lane_Info to_build_split_topo_lane_info;
    if(local_map == nullptr || ld_map == nullptr)
    {
        return;
    }

    GetBuildTopoLaneInfo(local_map, ld_map, to_build_split_topo_lane_info);
    if(to_build_split_topo_lane_info.ready_to_build_split_topo)
    {
        std::pair<cem::message::common::Point2DF, cem::message::common::Point2DF> adjoin_lane_func_point;
        if (to_build_split_topo_lane_info.bev_adjoin_lane_geos.size() > 1)
        {
            adjoin_lane_func_point.first.x = to_build_split_topo_lane_info.bev_adjoin_lane_geos[0].x();
            adjoin_lane_func_point.first.y = to_build_split_topo_lane_info.bev_adjoin_lane_geos[0].y();

            adjoin_lane_func_point.second.x = to_build_split_topo_lane_info.bev_adjoin_lane_geos[1].x();
            adjoin_lane_func_point.second.y = to_build_split_topo_lane_info.bev_adjoin_lane_geos[1].y();
        }

        bool seg_valid = SegmentBevEgoLane(
                            local_map,
                            to_build_split_topo_lane_info.bev_ego_lane_id,
                            to_build_split_topo_lane_info.segment_point.x(),
                            to_build_split_topo_lane_info.new_lane_id,
                            adjoin_lane_func_point);

        if(seg_valid && to_build_split_topo_lane_info.new_lane_id != 0)
        {
            AINFO << "BUILD SPLIT TOPO： " << to_build_split_topo_lane_info.bev_ego_lane_id << " -- "
                                           << to_build_split_topo_lane_info.bev_ego_adjoin_lane_id << " -- "
                                           << to_build_split_topo_lane_info.new_lane_id;
        }
    }
}

void BuildLaneTopology::CheckMergeTopo(
    BevMapInfoPtr &bevMapPtr,
    RoutingMapPtr &routingMapPtr,
    std::unordered_map<uint64_t,
    std::vector<uint64_t>>& bev_ld_match)
{

    if(bevMapPtr == nullptr || routingMapPtr == nullptr) {
       return;
    }
    bool has_ld_map_flag = true;
    if(routingMapPtr->type == cem::message::env_model::MapType::HD_MAP) {
        has_ld_map_flag = true;
    }
    else{
        has_ld_map_flag = false;
    }
    // 根据对应的ld地图车道，校验感知自车道merge拓扑是否是误检拓扑
    static uint64_t ego_lane_id = 0;
    std::vector<BevLaneInfo>::iterator bev_ego_lane_iter = bevMapPtr->lane_infos.end();
    std::vector<BevLaneInfo>::iterator left_side_lane_iter = bevMapPtr->lane_infos.end();
    std::vector<BevLaneInfo>::iterator right_side_lane_iter = bevMapPtr->lane_infos.end();
    std::vector<pair<uint32_t,std::vector<Eigen::Vector2f>>> lmk_geos(2,{0,std::vector<Eigen::Vector2f>()});
    std::vector<LaneInfo>::iterator ld_ego_lane_iter = routingMapPtr->lanes.end();
    bool lmk_valid_flag = false;
    BevLaneInfo *other_merge_lane_ptr = nullptr,*next_lane_ptr = nullptr;
    bev_ego_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                            [&](const cem::message::sensor::BevLaneInfo &lane) { return static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) == lane.position; });
    if(bev_ego_lane_iter == bevMapPtr->lane_infos.end()) {
       if(ego_lane_id) {
            bev_ego_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                            [&](const cem::message::sensor::BevLaneInfo &lane) { return ego_lane_id == lane.id; });
       }
    }
    if(bev_ego_lane_iter == bevMapPtr->lane_infos.end()) {
        return;
    }

    if(bev_ego_lane_iter->next_lane_ids.size() == 1) {
        for(auto &lane : bevMapPtr->lane_infos) {
            if(lane.id == bev_ego_lane_iter->next_lane_ids[0]) {
              next_lane_ptr = &lane;
            }
            else if(lane.id != bev_ego_lane_iter->id) {
                for(auto next_id : lane.next_lane_ids) {
                    if(next_id == bev_ego_lane_iter->next_lane_ids[0]) {
                        other_merge_lane_ptr = &lane;
                        break;
                    }
                }
            }
        }
    }
    //感知检出ego lane 有merge拓扑需要校验
    if(next_lane_ptr && other_merge_lane_ptr && next_lane_ptr->geos->size() > 3) {
        //根据感知merge拓扑后继车道和左一/右一车道的距离来判断自车道是否误检汇入其他车道
        bool final_ego_merge_invalid = false;
        auto JudgeEgoMergeByLaneDist = [&](double dist_threshold)->bool{
            //自车道向左汇入场景
            if(LaneGeometry::JudgeIsLeft(*other_merge_lane_ptr->geos, *bev_ego_lane_iter->geos)) {
                auto right_side_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
                                                        return bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST) == bev_lane.position && !bev_lane.geos->empty();});
                if(right_side_lane_iter != bevMapPtr->lane_infos.end()) {
                    if(!right_side_lane_iter->next_lane_ids.empty()) {//右一车道有后继
                    }
                    //merge拓扑后继车道和右一车道的距离超过5.0m,则自车道错误汇入
                    double next_2_right_dist = LaneGeometry::GetDistanceBetweenLines(*next_lane_ptr->geos, *right_side_lane_iter->geos);
                    if(next_2_right_dist > dist_threshold) {
                        return false;
                    }
                }
            }
            else{//自车道向右汇入
                auto left_side_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
                                                        return bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) && !bev_lane.geos->empty();});
                if(left_side_lane_iter != bevMapPtr->lane_infos.end()) {
                        if(!left_side_lane_iter->next_lane_ids.empty()) {//左一车道有后继
                        }
                        //merge拓扑后继车道和右一车道的距离超过5.0m,则自车道错误汇入
                        double next_2_right_dist = LaneGeometry::GetDistanceBetweenLines(*next_lane_ptr->geos, *left_side_lane_iter->geos);
                        if(next_2_right_dist > dist_threshold) {
                            return false;
                        }
                }
            }
            return true;
        };

        double bev_ld_ego_dist = 0.0, bev_ld_next_dist = 0.0;
        std::shared_ptr<std::vector<Eigen::Vector2f>> ld_ego_tmp_geos = std::make_shared<std::vector<Eigen::Vector2f>>();
        //有图时，根据对应地图车道的拓扑关系判断感知merge拓扑是否误检
        if(has_ld_map_flag) {
            if(bev_ld_match.find(bev_ego_lane_iter->id) != bev_ld_match.end() && !bev_ld_match[bev_ego_lane_iter->id].empty()) {
                ld_ego_lane_iter = std::find_if(routingMapPtr->lanes.begin(), routingMapPtr->lanes.end(),
                                                [&](const cem::message::env_model::LaneInfo &lane) { return bev_ld_match[bev_ego_lane_iter->id].back() == lane.id; });
                for(auto ld_id : bev_ld_match[bev_ego_lane_iter->id]) {
                    auto  ld_lane_tmp_iter = std::find_if(routingMapPtr->lanes.begin(), routingMapPtr->lanes.end(),
                                                    [&](const cem::message::env_model::LaneInfo &lane) { return ld_id == lane.id; });
                    if(ld_lane_tmp_iter != routingMapPtr->lanes.end()) {
                        for(auto& pt : ld_lane_tmp_iter->points) {
                            if(!ld_ego_tmp_geos->empty() && pt.x < ld_ego_tmp_geos->back().x()) {
                                continue;
                            }
                            ld_ego_tmp_geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                        }
                    }
                }
            }
            if(ld_ego_lane_iter == routingMapPtr->lanes.end() || ld_ego_tmp_geos->empty()) {
                return;
            }
            else{
                bev_ld_ego_dist = LaneGeometry::CalLinesSimilar(*bev_ego_lane_iter->geos,*ld_ego_tmp_geos);
                if(bev_ld_ego_dist > 1.0) {
                    return;
                }
            }
            if(ld_ego_lane_iter->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_NONE) {//地图自车道无merge拓扑
                    bev_ld_next_dist = LaneGeometry::CalLinesSimilar(*next_lane_ptr->geos, *ld_ego_tmp_geos);
                    if(fabs(bev_ld_next_dist - bev_ld_ego_dist) > 2.5 && !JudgeEgoMergeByLaneDist(5.0)) {
                        final_ego_merge_invalid = true;
                    }
            }
            else if(ld_ego_lane_iter->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_LEFT ||
                ld_ego_lane_iter->merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_RIGHT) {//校验感知是否跨车道汇入

                auto ld_main_lane_iter = std::find_if(routingMapPtr->lanes.begin(), routingMapPtr->lanes.end(),
                                                [&](const cem::message::env_model::LaneInfo &lane) { return ld_ego_lane_iter->next_lane_ids.size() == 1 &&  ld_ego_lane_iter->next_lane_ids[0] == lane.id
                                                && lane.merge_topology == cem::message::env_model::MergeTopology::TOPOLOGY_MERGE_NONE; });
                if(ld_main_lane_iter == routingMapPtr->lanes.end()) {
                    return;
                }
                std::shared_ptr<std::vector<Eigen::Vector2f>> ld_main_tmp_geos = std::make_shared<std::vector<Eigen::Vector2f>>();
                for(auto& pt : ld_main_lane_iter->points) {
                    if(!ld_main_tmp_geos->empty() && pt.x < ld_main_tmp_geos->back().x()){
                        continue;
                    }
                    ld_main_tmp_geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                }
                double next_lane_2_ld_ego_dist =  LaneGeometry::GetDistanceBetweenLines(*next_lane_ptr->geos, *ld_main_tmp_geos);
                if(fabs(next_lane_2_ld_ego_dist - bev_ld_ego_dist) > 2.5 && !JudgeEgoMergeByLaneDist(7.5)) {//感知自车道merge跨车道汇入
                    final_ego_merge_invalid = true;
                }
            }
        }
        else{   //无图时，根据后继车道到左一/右一道中心线距离判断感知merge拓扑是否误检
                if(!JudgeEgoMergeByLaneDist(5.0)) {
                    final_ego_merge_invalid = true;
                }
        }
        //误检自车道merge到其他车道
        if(final_ego_merge_invalid) {
            // AINFO<<"has_ld_map_flag:"<<has_ld_map_flag<<" ego lane merge err";
            ego_lane_id = bev_ego_lane_iter->id;
            //删除自车道原拓扑
            next_lane_ptr->previous_lane_ids.erase(std::remove(next_lane_ptr->previous_lane_ids.begin(),next_lane_ptr->previous_lane_ids.end(),bev_ego_lane_iter->id),
                                                    next_lane_ptr->previous_lane_ids.end());
            bev_ego_lane_iter->next_lane_ids.erase(std::remove(bev_ego_lane_iter->next_lane_ids.begin(),bev_ego_lane_iter->next_lane_ids.end(),
                                                    next_lane_ptr->id),bev_ego_lane_iter->next_lane_ids.end());
            //寻找左右车道线
            GetLaneMarkerBasedPosition(bev_ego_lane_iter->id,left_side_lane_iter,right_side_lane_iter,bevMapPtr,lmk_geos);
            if(lmk_geos[0].first && lmk_geos[1].first) {
                double lmk_dist  = LaneGeometry::GetDistanceBetweenLines(lmk_geos[0].second, lmk_geos[1].second);
                if(lmk_dist > 2.5 && lmk_dist < 5.0) {
                    lmk_valid_flag = true;
                }
            }
            else if(has_ld_map_flag) {
                GetLaneMarkerBasedLd(bev_ego_lane_iter->id,ld_ego_tmp_geos,bevMapPtr,lmk_geos);
                if(lmk_geos[0].first && lmk_geos[1].first) {
                    double lmk_dist  = LaneGeometry::GetDistanceBetweenLines(lmk_geos[0].second, lmk_geos[1].second);
                    if(lmk_dist > 2.5 && lmk_dist < 5.0) {
                        lmk_valid_flag = true;
                    }
                }
            }
            if(!lmk_valid_flag) {
               return;
            }
            // AINFO<<"left lmk id:"<<lmk_geos[0].first<<" right lmk id:"<<lmk_geos[1].first;
            //寻找可能得后继
            bool exit_correct_next_lane_flag = false;
            auto correct_next_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
                                                    return bev_lane.id != next_lane_ptr->id && !bev_lane.geos->empty() && bev_lane.previous_lane_ids.empty()
                                                    && LaneGeometry::JudgeIsLeft(lmk_geos[0].second, *bev_lane.geos)
                                                    && LaneGeometry::JudgeIsLeft(*bev_lane.geos, lmk_geos[1].second) && bev_lane.geos->front().x() > bev_ego_lane_iter->geos->back().x();});
            if(correct_next_lane_iter == bevMapPtr->lane_infos.end()) {
                return;
            }
            else{
                exit_correct_next_lane_flag = true;
            }
            if(has_ld_map_flag) {
                auto temp_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
                                                        return bev_lane.id != next_lane_ptr->id && !bev_lane.geos->empty() && bev_lane.previous_lane_ids.empty()
                                                        && fabs(bev_ld_ego_dist - LaneGeometry::CalLinesSimilar(*bev_lane.geos, *ld_ego_tmp_geos)) < 0.5
                                                        && bev_lane.geos->front().x() > bev_ego_lane_iter->geos->back().x();});
                if(temp_lane_iter != bevMapPtr->lane_infos.end()) {
                    if(temp_lane_iter->id != correct_next_lane_iter->id){
                        exit_correct_next_lane_flag = false;
                    }
                }
                else{
                    return;
                }
            }
            if(exit_correct_next_lane_flag) {
                bev_ego_lane_iter->next_lane_ids.push_back(correct_next_lane_iter->id);
                correct_next_lane_iter->previous_lane_ids.push_back(bev_ego_lane_iter->id);
                return;
            }
            else{
                //找不到合理的后继中心线，需要利用找到的左右车道线，对自车道中心线做延长处理
                Eigen::Vector2f target_lane_last_pts(0.0,0.0);
                Eigen::Vector2f target_left_last_pts(0.0,0.0);
                Eigen::Vector2f target_right_last_pts(0.0,0.0);
                if(bev_ego_lane_iter->geos->size() > 0) {
                    target_lane_last_pts = bev_ego_lane_iter->geos->back();
                    for(size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
                    {
                        auto &bev_lm = bevMapPtr->lanemarkers[idx];
                        if(lmk_geos[0].first && bev_lm.id == lmk_geos[0].first)
                        {
                            target_left_last_pts = bev_lm.geos->back();
                        }
                        else if(lmk_geos[1].first && bev_lm.id == lmk_geos[1].first)
                        {
                            target_right_last_pts = bev_lm.geos->back();
                        }
                    }
                    AlignLaneMarker(bevMapPtr,bev_ego_lane_iter->id,lmk_geos[0].first,lmk_geos[1].first,target_lane_last_pts,target_left_last_pts,target_right_last_pts);
                }
            }
        }
    }
}
//根据position指定的左车道的右车道线，右车道的左车道线
void BuildLaneTopology::GetLaneMarkerBasedPosition(
    uint32_t bev_ego_id,
    std::vector<cem::message::sensor::BevLaneInfo>::iterator left_side_lane_iter,
    std::vector<cem::message::sensor::BevLaneInfo>::iterator right_side_lane_iter,
    BevMapInfoPtr &bevMapPtr,
    std::vector<pair<uint32_t,std::vector<Eigen::Vector2f>>>& lmk_geos)
{

    if(bevMapPtr == nullptr) {
        return;
    }
    auto bev_ego_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
             return bev_lane.id == bev_ego_id;});
    if(bev_ego_lane_iter == bevMapPtr->lane_infos.end()) {
        return;
    }
    uint64_t left_lmk_extend_id = 0,right_lmk_extend_id = 0;
    right_side_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                    return bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST);});

    if(right_side_lane_iter != bevMapPtr->lane_infos.end()) {
        auto lmk_tmp_iter = std::find_if(bevMapPtr->lanemarkers.begin(),bevMapPtr->lanemarkers.end(),[&](const cem::message::sensor::BevLaneMarker& lmk) {
                        return right_side_lane_iter->left_lane_marker_id == lmk.id;});
        if(lmk_tmp_iter != bevMapPtr->lanemarkers.end() && !lmk_tmp_iter->geos->empty() && lmk_tmp_iter->geos->back().x() - bev_ego_lane_iter->geos->back().x() > 50.0) {
             right_lmk_extend_id = lmk_tmp_iter->id;
        }
        else if(!right_side_lane_iter->next_lane_ids.empty()) {
                auto temp_right_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                                  return right_side_lane_iter->next_lane_ids[0] == bev_lane.id && !bev_lane.geos->empty();});
                for(size_t i = 1; i < right_side_lane_iter->next_lane_ids.size(); i++) {
                    auto lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                                  return right_side_lane_iter->next_lane_ids[i] == bev_lane.id && !bev_lane.geos->empty();});
                    if(lane_iter != bevMapPtr->lane_infos.end()) {
                        bool is_left = false;
                        double start_x = std::max(lane_iter->geos->front().x(),temp_right_lane_iter->geos->front().x());
                        double end_x = std::min(lane_iter->geos->back().x(),temp_right_lane_iter->geos->back().x());
                        if(end_x < start_x) {
                           size_t l1_num = lane_iter->geos->size();
                           size_t l2_num = temp_right_lane_iter->geos->size();
                           is_left = (*lane_iter->geos)[l1_num / 2].y() > (*temp_right_lane_iter->geos)[l2_num / 2].y()? true:false;
                        }
                        else
                        {
                           is_left = LaneGeometry::JudgeIsLeft(*lane_iter->geos, *temp_right_lane_iter->geos);
                        }
                        if(is_left) temp_right_lane_iter = lane_iter;
                    }
                }
                right_side_lane_iter = temp_right_lane_iter;
                if(right_side_lane_iter != bevMapPtr->lane_infos.end()) right_lmk_extend_id = right_side_lane_iter->left_lane_marker_id;

        }
    }
    left_side_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                    return bev_lane.position == static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST);});

    if(left_side_lane_iter != bevMapPtr->lane_infos.end()) {
        auto lmk_tmp_iter = std::find_if(bevMapPtr->lanemarkers.begin(),bevMapPtr->lanemarkers.end(),[&](const cem::message::sensor::BevLaneMarker& lmk) {
                        return left_side_lane_iter->right_lane_marker_id == lmk.id;});
        if(lmk_tmp_iter != bevMapPtr->lanemarkers.end() && !lmk_tmp_iter->geos->empty() && lmk_tmp_iter->geos->back().x() - bev_ego_lane_iter->geos->back().x() > 50.0) {
             left_lmk_extend_id = lmk_tmp_iter->id;
        }
        else if(!left_side_lane_iter->next_lane_ids.empty()) {
                auto temp_left_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                                  return left_side_lane_iter->next_lane_ids[0] == bev_lane.id && !bev_lane.geos->empty();});
                for(size_t i = 1; i < left_side_lane_iter->next_lane_ids.size(); i++)
                {
                    auto lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo& bev_lane) {
                                                  return left_side_lane_iter->next_lane_ids[i] == bev_lane.id && !bev_lane.geos->empty();});
                    if(lane_iter != bevMapPtr->lane_infos.end()) {
                       bool is_right = false;
                       double start_x = std::max(lane_iter->geos->front().x(),temp_left_lane_iter->geos->front().x());
                       double end_x = std::min(lane_iter->geos->back().x(),temp_left_lane_iter->geos->back().x());
                       if(end_x < start_x) {
                           size_t l1_num = lane_iter->geos->size();
                           size_t l2_num = temp_left_lane_iter->geos->size();
                           is_right = (*lane_iter->geos)[l1_num / 2].y() < (*temp_left_lane_iter->geos)[l2_num / 2].y()? true:false;
                        }
                        else
                        {
                           is_right = !LaneGeometry::JudgeIsLeft(*lane_iter->geos, *temp_left_lane_iter->geos);
                        }
                        if(is_right) temp_left_lane_iter = lane_iter;
                    }
                }
                left_side_lane_iter = temp_left_lane_iter;
                if(left_side_lane_iter != bevMapPtr->lane_infos.end()) left_lmk_extend_id = left_side_lane_iter->right_lane_marker_id;

        }
    }
    for(auto& lane_mark : bevMapPtr->lanemarkers) {
        if(lane_mark.id == left_lmk_extend_id) {
            lmk_geos[0].first = lane_mark.id;
            for(auto& pt : *lane_mark.geos){
                if(!lmk_geos[0].second.empty() && lmk_geos[0].second.back().x() > pt.x()) {
                    continue;
                }
                lmk_geos[0].second.push_back(Eigen::Vector2f(pt.x(),pt.y()));
            }
        }
        else if( lane_mark.id == right_lmk_extend_id) {
                lmk_geos[1].first = lane_mark.id;
                for(auto& pt : *lane_mark.geos){
                    if(!lmk_geos[1].second.empty() && lmk_geos[1].second.back().x() > pt.x()) {
                        continue;
                    }
                    lmk_geos[1].second.push_back(Eigen::Vector2f(pt.x(),pt.y()));
                }
        }
    }
}
//根据到地图自车道中心线的距离寻找左右车道线
void BuildLaneTopology::GetLaneMarkerBasedLd(
    uint32_t bev_id, std::shared_ptr<std::vector<Eigen::Vector2f>> &ld_ego_tmp_geos,
    BevMapInfoPtr &bevMapPtr,
    std::vector<pair<uint32_t,std::vector<Eigen::Vector2f>>>& lmk_geos)
{

    if(ld_ego_tmp_geos || ld_ego_tmp_geos->empty()) {
        return;
    }

    auto bev_ego_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(),bevMapPtr->lane_infos.end(),[&](const cem::message::sensor::BevLaneInfo &bev_lane) {
             return bev_lane.id == bev_id;});
    if(bev_ego_lane_iter == bevMapPtr->lane_infos.end()) {
        return;
    }
    if(bev_ego_lane_iter->left_lane_marker_id || bev_ego_lane_iter->right_lane_marker_id) {
        return;
    }

    lmk_geos[0].first = 0;
    lmk_geos[0].second.clear();
    lmk_geos[1].first = 0;
    lmk_geos[1].second.clear();
    uint32_t left_lmk_id_raw = bev_ego_lane_iter->left_lane_marker_id, right_lmk_id_raw = bev_ego_lane_iter->right_lane_marker_id;
    uint32_t left_lmk_extend_id = 0,right_lmk_extend_id = 0;
    std::map<uint32_t,std::vector<Eigen::Vector2f>> lmk_geos_tmp;
    for(auto& lane_mark : bevMapPtr->lanemarkers) {
            lmk_geos_tmp.insert({lane_mark.id,std::vector<Eigen::Vector2f>()});
            for(auto& pt : *lane_mark.geos){
                if(!lmk_geos_tmp[lane_mark.id].empty() && lmk_geos_tmp[lane_mark.id].back().x() > pt.x()) {
                    continue;
                }
                lmk_geos_tmp[lane_mark.id].push_back(Eigen::Vector2f(pt.x(),pt.y()));
            }
            if(lmk_geos_tmp[lane_mark.id].empty()){
                lmk_geos_tmp.erase(lane_mark.id);
            }
    }
    if(lmk_geos_tmp.find(left_lmk_id_raw) != lmk_geos_tmp.end() && fabs(lmk_geos_tmp[left_lmk_id_raw].back().x() - bev_ego_lane_iter->geos->back().x()) > 50.0) {
                    left_lmk_extend_id = left_lmk_id_raw;
    }
    if(lmk_geos_tmp.find(right_lmk_id_raw) != lmk_geos_tmp.end() && fabs(lmk_geos_tmp[right_lmk_id_raw].back().x() - bev_ego_lane_iter->geos->back().x()) > 50.0) {
                        right_lmk_extend_id = right_lmk_id_raw;
    }
    for(auto lmk : lmk_geos_tmp) {
        if(lmk.first != left_lmk_id_raw && LaneGeometry::JudgeIsLeft(lmk.second, *ld_ego_tmp_geos) && fabs(lmk.second.back().x() - bev_ego_lane_iter->geos->back().x()) > 50.0) {
            double dist_tmp = LaneGeometry::GetDistanceBetweenLines(lmk.second, *ld_ego_tmp_geos);
            if(dist_tmp < 2.0) {
                left_lmk_extend_id = lmk.first;
            }
        }
        if(lmk.first != right_lmk_id_raw && !LaneGeometry::JudgeIsLeft(lmk.second, *ld_ego_tmp_geos) && fabs(lmk.second.back().x() - bev_ego_lane_iter->geos->back().x()) > 50.0) {
            double dist_tmp = LaneGeometry::GetDistanceBetweenLines(lmk.second, *ld_ego_tmp_geos);;
            if(dist_tmp< 2.0) {
               right_lmk_extend_id = lmk.first;
            }
        }
    }
    if(LaneGeometry::GetDistanceBetweenLines(lmk_geos_tmp[left_lmk_extend_id], lmk_geos_tmp[right_lmk_extend_id]) > 4.0) {
        return;
    }
    for(auto lmk : lmk_geos_tmp) {
            if(lmk.first == left_lmk_extend_id) {
                lmk_geos[0] = lmk;
            }
            else if(lmk.first == right_lmk_extend_id) {
                lmk_geos[1] = lmk;
            }
    }
}
//左右车道和中心线其中一根线缺少一段进行补齐
void BuildLaneTopology::AlignLaneMarker(
    BevMapInfoPtr &bevMapPtr,
    const int & target_lane_id,
    const int & bev_left_lanemarker_id,
    const int & bev_right_lanemarker_id,
    const Eigen::Vector2f &target_lane_last_pts,
    const Eigen::Vector2f &target_left_lanemarker_last_pts,
    const Eigen::Vector2f &target_right_lanemarker_last_pts)
{
    bool left_need_to_align = false;
    bool lane_need_to_align = false;
    bool right_need_to_align = false;
    static const int LEFT = 0;
    static const int LANE = 1;
    static const int RIGHT = 2;
    std::vector<index2value> values;
    index2value left,lane,right;
    left.idx = LEFT;
    left.val = target_left_lanemarker_last_pts(0);
    values.emplace_back(left);
    lane.idx = LANE;
    lane.val = target_lane_last_pts(0);
    values.emplace_back(lane);
    right.idx = RIGHT;
    right.val = target_right_lanemarker_last_pts(0);
    values.emplace_back(right);

    sort(values.begin(), values.end(), cmp);

    if(values.size() > 2 && values[0].idx == LEFT)
    {
        left_need_to_align = true;
    }
    if(values.size() > 2  && values[0].idx == LANE)
    {
        lane_need_to_align = true;
    }
    if(values.size() > 2  && values[0].idx == RIGHT)
    {
        right_need_to_align = true;
    }
    // //计算补齐车道的宽度
    float target_lane_width = 0.0f;
    for(size_t idx = 0; idx < bevMapPtr->lane_infos.size(); idx++)
    {
       if(target_lane_id == bevMapPtr->lane_infos[idx].id)
       {
            target_lane_width = bevMapPtr->lane_infos[idx].width;
       }
    }
    // Eigen::Vector2f min_lm_pts(0.0,0.0);
    // min_lm_pts(0) = std::min(target_left_lanemarker_last_pts(0),target_right_lanemarker_last_pts(0));
    // for (size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
    // {
    //     std::vector<Curve> fit_curves;
    //     std::vector<std::vector<Point2DF>> bev_seg_groups;
    //     const bool removed_invalid = false;
    //     auto &bev_lm = bevMapPtr->lanemarkers[idx];
    //     if(bev_lm.id == bev_right_lanemarker_id && true == left_need_to_align)
    //     {
    //         FitRawCurveLineWithCoeffs(bev_lm.line_points,removed_invalid,fit_curves,bev_seg_groups);
    //         for(int idx = 0; idx < bev_seg_groups.size() ; idx++)
    //         {
    //             if(min_lm_pts(0) <= bev_seg_groups[idx].back().x &&
    //                min_lm_pts(0) >= bev_seg_groups[idx].front().x)
    //             {
    //                 auto &curve = fit_curves[idx];
    //                 min_lm_pts(1) = static_cast<float>(pow(min_lm_pts(0),3)*curve.c3 + pow(min_lm_pts(0),2)*curve.c2 + min_lm_pts(0)*curve.c1 + curve.c0);
    //                 target_lane_width = static_cast<int>((min_lm_pts - target_left_lanemarker_last_pts).norm()*10.0f)/10.0;
    //             }
    //         }
    //     }
    //     if(bev_lm.id == bev_left_lanemarker_id && true == right_need_to_align)
    //     {
    //         FitRawCurveLineWithCoeffs(bev_lm.line_points,removed_invalid,fit_curves,bev_seg_groups);
    //         for(int idx = 0; idx < bev_seg_groups.size() ; idx++)
    //         {
    //             if(min_lm_pts(0) <= bev_seg_groups[idx].back().x &&
    //                min_lm_pts(0) >= bev_seg_groups[idx].front().x)
    //             {
    //                 auto &curve = fit_curves[idx];
    //                 min_lm_pts(1) = static_cast<float>(pow(min_lm_pts(0),3)*curve.c3 + pow(min_lm_pts(0),2)*curve.c2 + min_lm_pts(0)*curve.c1 + curve.c0);
    //                 target_lane_width = static_cast<int>((min_lm_pts - target_right_lanemarker_last_pts).norm()*10.0f)/10.0;
    //             }
    //         }
    //     }
    // }
    if(true == left_need_to_align && values.size() == 3)
    {//左车道线需要补齐
       std::vector<Point2DF> left_line_extend_points;
       std::vector<Point2DF> base_points;
       for(size_t idx = 0; idx < bevMapPtr->lane_infos.size(); idx++)
       {
          auto &bev_lane = bevMapPtr->lane_infos[idx];
          if(bev_lane.id == target_lane_id && bev_lane.line_points.size() > 0)
          {
             for(auto & pt : bev_lane.line_points)
             {
                if(pt.x > values[0].val)
                {
                    base_points.push_back(pt);
                }
             }
          }
       }
       if(base_points.size() > 2)
       {
          for (size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
          {
            auto &bev_lm = bevMapPtr->lanemarkers[idx];
            if(bev_right_lanemarker_id == bev_lm.id)
            {
                std::vector<Curve> fit_curves;
                std::vector<std::vector<Point2DF>> bev_seg_groups;
                const bool removed_invalid = false;
                FitRawCurveLineWithCoeffs(bev_lm.line_points,removed_invalid,fit_curves,bev_seg_groups);
                // AINFO<<"fit_curves size:"<<fit_curves.size()<<" bev_seg_groups size:"<<bev_seg_groups.size();
                for(int pt_idx = 0; pt_idx < base_points.size();pt_idx)
                {
                    auto &pt = base_points[pt_idx];
                    for(int seg_idx = 0; seg_idx < bev_seg_groups.size(),seg_idx < fit_curves.size(); seg_idx++)
                    {
                        if(bev_seg_groups[seg_idx].size() >= 2 && pt.x <= bev_seg_groups[seg_idx].back().x && pt.x >= bev_seg_groups[seg_idx].front().x)
                        {
                            auto &curve = fit_curves[seg_idx];
                            Point2DF r_point;
                            r_point.x = pt.x;
                            r_point.y = static_cast<float>(pow(pt.x,3) * curve.c3 + pow(pt.x,2) * curve.c2 + pt.x * curve.c1 + curve.c0);
                            Eigen::Vector2f delta;
                            delta(0) = pt.x - r_point.x;
                            delta(1) = pt.y - r_point.y;
                            delta = delta/delta.norm();
                            Point2DF l_point;
                            l_point.x = r_point.x + target_lane_width*delta(0);
                            l_point.y = r_point.y + target_lane_width*delta(1);
                            left_line_extend_points.emplace_back(l_point);
                        }
                    }
                }
            }
          }
          for(size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
          {
            auto &bev_lm = bevMapPtr->lanemarkers[idx];
            if(bev_left_lanemarker_id == bev_lm.id)
            {
                for(auto &pt : left_line_extend_points)
                {
                   if(!bev_lm.line_points.empty() && bev_lm.line_points.back().x >= pt.x)
                   {
                      continue;
                   }
                   bev_lm.line_points.push_back(pt);
                   if(!bev_lm.geos->empty() && bev_lm.geos->back().x() >= pt.x)
                   {
                      continue;
                   }
                   bev_lm.geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                }
            }
          }
       }
    }
    if(true == right_need_to_align && values.size() == 3)
    {//右车道线需要补齐
       std::vector<Point2DF> right_line_extend_points;
       std::vector<Point2DF> base_points;
       for(size_t idx = 0; idx < bevMapPtr->lane_infos.size(); idx++)
       {
          auto &bev_lane = bevMapPtr->lane_infos[idx];
          if(bev_lane.id == target_lane_id && bev_lane.line_points.size() > 0)
          {
             for(auto & pt : bev_lane.line_points)
             {
                if(pt.x > values[0].val)
                {
                    base_points.push_back(pt);
                }
             }
          }
       }
       if(base_points.size() > 2)
       {
          for (size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
          {
            auto &bev_lm = bevMapPtr->lanemarkers[idx];
            if(bev_left_lanemarker_id == bev_lm.id)
            {
                std::vector<Curve> fit_curves;
                std::vector<std::vector<Point2DF>> bev_seg_groups;
                const bool removed_invalid = false;
                FitRawCurveLineWithCoeffs(bev_lm.line_points,removed_invalid,fit_curves,bev_seg_groups);
                // AINFO<<"fit_curves size:"<<fit_curves.size()<<" bev_seg_groups size:"<<bev_seg_groups.size();
                for(int pt_idx = 0; pt_idx < base_points.size();pt_idx)
                {
                    auto &pt = base_points[pt_idx];
                    for(int seg_idx = 0; seg_idx < bev_seg_groups.size(),seg_idx < fit_curves.size(); seg_idx++)
                    {
                        if(bev_seg_groups[seg_idx].size() >= 2 && pt.x <= bev_seg_groups[seg_idx].back().x && pt.x >= bev_seg_groups[seg_idx].front().x)
                        {
                            auto &curve = fit_curves[seg_idx];
                            Point2DF l_point;
                            l_point.x = pt.x;
                            l_point.y = static_cast<float>(pow(pt.x,3) * curve.c3 + pow(pt.x,2) * curve.c2 + pt.x * curve.c1 + curve.c0);
                            Eigen::Vector2f delta;
                            delta(0) = pt.x - l_point.x;
                            delta(1) = pt.y - l_point.y;
                            delta = delta/delta.norm();
                            Point2DF r_point;
                            r_point.x = l_point.x + target_lane_width*delta(0);
                            r_point.y = l_point.y + target_lane_width*delta(1);
                            right_line_extend_points.emplace_back(r_point);
                        }
                    }
                }
            }
          }
          for(size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
          {
            auto &bev_lm = bevMapPtr->lanemarkers[idx];
            if(bev_right_lanemarker_id == bev_lm.id)
            {
                for(auto &pt : right_line_extend_points)
                {
                   if(!bev_lm.line_points.empty() && bev_lm.line_points.back().x >= pt.x)
                   {
                      continue;
                   }
                   bev_lm.line_points.push_back(pt);
                   if(!bev_lm.geos->empty() && bev_lm.geos->back().x() >= pt.x)
                   {
                     continue;
                   }
                   bev_lm.geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                }
            }
          }
       }
    }
    if(true == lane_need_to_align && values.size() == 3)
    {//车道中心线补齐
       std::vector<Point2DF> lane_extend_points;
       std::vector<Point2DF> base_points;
       for(size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
       {
          auto &bev_lm = bevMapPtr->lanemarkers[idx];
          if(bev_lm.id == bev_right_lanemarker_id && bev_lm.line_points.size() > 0)
          {
             for(auto & pt : bev_lm.line_points)
             {
                if(pt.x > values[0].val)
                {
                    base_points.push_back(pt);
                }
             }
          }
       }
       if(base_points.size() > 2)
       {
          for (size_t idx = 0; idx < bevMapPtr->lanemarkers.size(); idx++)
          {
            auto &bev_lm = bevMapPtr->lanemarkers[idx];
            if(bev_left_lanemarker_id == bev_lm.id)
            {
                std::vector<Curve> fit_curves;
                std::vector<std::vector<Point2DF>> bev_seg_groups;
                const bool removed_invalid = false;
                FitRawCurveLineWithCoeffs(bev_lm.line_points,removed_invalid,fit_curves,bev_seg_groups);
                // AINFO<<"fit_curves size:"<<fit_curves.size()<<" bev_seg_groups size:"<<bev_seg_groups.size();
                for(int pt_idx = 0; pt_idx < base_points.size();pt_idx)
                {
                    auto &pt = base_points[pt_idx];
                    for(int seg_idx = 0; seg_idx < bev_seg_groups.size(),seg_idx < fit_curves.size(); seg_idx++)
                    {
                        if(bev_seg_groups[seg_idx].size() >= 2 && pt.x <= bev_seg_groups[seg_idx].back().x && pt.x >= bev_seg_groups[seg_idx].front().x)
                        {
                            auto &curve = fit_curves[idx];
                            Point2DF l_point;
                            l_point.x = pt.x;
                            l_point.y = static_cast<float>(pow(pt.x,3) * curve.c3 + pow(pt.x,2) * curve.c2 + pt.x * curve.c1 + curve.c0);
                            Eigen::Vector2f delta;
                            delta(0) = pt.x - l_point.x;
                            delta(1) = pt.y - l_point.y;
                            delta = delta/delta.norm();
                            Point2DF lane_point;
                            lane_point.x = (l_point.x + pt.x) / 2;
                            lane_point.y = (l_point.y + pt.y) / 2;
                            lane_extend_points.emplace_back(lane_point);
                        }
                    }
                }
            }
          }

          for(size_t idx = 0; idx < bevMapPtr->lane_infos.size(); idx++)
          {
            auto &bev_lane = bevMapPtr->lane_infos[idx];
            if(target_lane_id == bev_lane.id)
            {
                for(auto &pt : lane_extend_points)
                {
                   if(!bev_lane.line_points.empty() && bev_lane.line_points.back().x >= pt.x)
                   {
                      continue;
                   }
                   bev_lane.line_points.push_back(pt);
                   if(!bev_lane.geos->empty() && bev_lane.geos->back().x() >= pt.x)
                   {
                     continue;
                   }
                   bev_lane.geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                }
            }
          }
       }
    }
}

//校验自车道 误检split拓扑
void BuildLaneTopology::CheckSplitTopo(BevMapInfoPtr &bevMapPtr,RoutingMapPtr &routingMapPtr,const bev_map_ids_info &bev_map_ids)
{
    bool has_ld_map = true;
    if(!bevMapPtr || bevMapPtr->lane_infos.empty())
    {
        return;
    }
    auto bev_ego_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                            [&](const cem::message::sensor::BevLaneInfo &lane) { return (static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) == lane.position) && (lane.next_lane_ids.size() == 2); });
    if(bev_ego_lane_iter == bevMapPtr->lane_infos.end() || bev_ego_lane_iter->geos->size() < 3) {
        return;
    }
    if(!routingMapPtr || routingMapPtr->lanes.empty()) {
        has_ld_map = false;
    }
    //有图split拓扑校验
    if(has_ld_map) {
            for(auto& split_info : bev_map_ids.ldmap_split_info) {
                if(split_info.bev_lane_from == bev_ego_lane_iter->id && (split_info.match_type == LdMapProcessor::MergeSplitType::kSplitLeft || split_info.match_type == LdMapProcessor::MergeSplitType::kSplitRight)) {
                    return;
                }
            }
            // AINFO<<"ego id:"<<bev_ego_lane_iter->id<<" split err";
            //感知误检split拓扑,需要删除错误的拓扑关系
            vector<uint64_t> bev_split_ids(bev_ego_lane_iter->next_lane_ids);
            std::shared_ptr<std::vector<Eigen::Vector2f>> ld_ego_geos = std::make_shared<std::vector<Eigen::Vector2f>>();
            std::unordered_map<uint64_t, std::unordered_set<uint64_t>> bev_split_matched_lds_set;
            std::unordered_map<uint64_t,uint64_t> bev_split_main_cnt;
            for(auto split_id : bev_split_ids) {
                bev_split_main_cnt.insert({split_id, 0});
                if(bev_map_ids.local_map_matched_ids.find(split_id) != bev_map_ids.local_map_matched_ids.end()) {
                    auto match_lds = bev_map_ids.local_map_matched_ids.at(split_id);
                    bev_split_matched_lds_set.insert({split_id, std::unordered_set<uint64_t>()});
                    for(auto ld_id : match_lds) {
                            bev_split_matched_lds_set[split_id].insert(ld_id);
                    }
                }
            }
            if(bev_map_ids.local_map_matched_ids.find(bev_ego_lane_iter->id) != bev_map_ids.local_map_matched_ids.end() && !bev_map_ids.local_map_matched_ids.at(bev_ego_lane_iter->id).empty()) {
                auto ld_id = bev_map_ids.local_map_matched_ids.at(bev_ego_lane_iter->id).front();
                auto ld_lane_it = std::find_if(routingMapPtr->lanes.begin(), routingMapPtr->lanes.end(),
                                                            [&](const cem::message::env_model::LaneInfo &lane) { return ld_id == lane.id; });
                while(ld_lane_it != routingMapPtr->lanes.end()) {
                    for(auto split_id : bev_split_ids) {
                        if(bev_split_matched_lds_set[split_id].find(ld_lane_it->id) != bev_split_matched_lds_set[split_id].end()) {
                            bev_split_main_cnt[split_id] += 1;
                        }
                    }
                    for(auto &pt : ld_lane_it->points) {
                            if(!ld_ego_geos->empty() && ld_ego_geos->back().x() >= pt.x) {
                                continue;
                            }
                            ld_ego_geos->push_back(Eigen::Vector2f(pt.x,pt.y));
                    }
                    if(ld_lane_it->next_lane_ids.size() == 1) {
                        ld_lane_it = std::find_if(routingMapPtr->lanes.begin(), routingMapPtr->lanes.end(),
                                                            [&](const cem::message::env_model::LaneInfo &lane) { return ld_lane_it->next_lane_ids[0] == lane.id; });
                    }
                    else {
                            break;
                    }
                }
            }
            if(bev_ego_lane_iter->geos->size() >= 3 && ld_ego_geos->size() >= 3 && LaneGeometry::CalLinesSimilar(*bev_ego_lane_iter->geos, *ld_ego_geos) < 1.5) {
                auto bev_split_lane_iter0 = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return bev_split_ids[0] == lane.id && lane.geos->size() >= 3;});
                auto bev_split_lane_iter1 = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return bev_split_ids[1] == lane.id  && lane.geos->size() >= 3;});
                if(bev_split_lane_iter0 == bevMapPtr->lane_infos.end() || bev_split_lane_iter1 == bevMapPtr->lane_infos.end()) {
                    return;
                }
                double split_2_ld_ego_dist_0 = LaneGeometry::CalLinesSimilar(*bev_split_lane_iter0->geos, *ld_ego_geos);
                double split_2_ld_ego_dist_1 = LaneGeometry::CalLinesSimilar(*bev_split_lane_iter1->geos, *ld_ego_geos);
                if(bev_split_main_cnt[bev_split_ids[0]] < bev_split_main_cnt[bev_split_ids[1]] && split_2_ld_ego_dist_0 > split_2_ld_ego_dist_1) {
                    bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(),bev_ego_lane_iter->next_lane_ids.end(), bev_split_ids[0]), bev_ego_lane_iter->next_lane_ids.end());
                    bev_split_lane_iter0->previous_lane_ids.erase(remove(bev_split_lane_iter0->previous_lane_ids.begin(), bev_split_lane_iter0->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter0->previous_lane_ids.end());
                    // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter0->id;
                }
                else if(bev_split_main_cnt[bev_split_ids[0]] > bev_split_main_cnt[bev_split_ids[1]] && split_2_ld_ego_dist_0 < split_2_ld_ego_dist_1) {
                    bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(), bev_ego_lane_iter->next_lane_ids.end(), bev_split_ids[1]), bev_ego_lane_iter->next_lane_ids.end());
                    bev_split_lane_iter1->previous_lane_ids.erase(remove(bev_split_lane_iter1->previous_lane_ids.begin(), bev_split_lane_iter1->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter1->previous_lane_ids.end());
                    // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter1->id;
                }
            }
            else {
                return;
            }
    }
    else {//无图拓扑校验：CNOAC2-146645
          auto left_first_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                            [&](const cem::message::sensor::BevLaneInfo &lane) { return (static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) == lane.position) && lane.next_lane_ids.empty(); });
          auto right_first_lane_iter = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                            [&](const cem::message::sensor::BevLaneInfo &lane) { return (static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST) == lane.position) && lane.next_lane_ids.empty(); });
          if(left_first_lane_iter == bevMapPtr->lane_infos.end() || right_first_lane_iter == bevMapPtr->lane_infos.end()) {
              return;
          }
          double ego_2_left_dist = std::numeric_limits<double>::max();
          double ego_2_right_dist = std::numeric_limits<double>::max();
          if(left_first_lane_iter->geos->size() > 3 && right_first_lane_iter->geos->size() > 3) {
            ego_2_left_dist = LaneGeometry::GetDistanceBetweenLines(*bev_ego_lane_iter->geos, *left_first_lane_iter->geos);
            ego_2_right_dist = LaneGeometry::GetDistanceBetweenLines(*bev_ego_lane_iter->geos, *right_first_lane_iter->geos);
            if(ego_2_left_dist > 3.0 &&  ego_2_left_dist < 5.0 && ego_2_right_dist > 3.0 &&  ego_2_right_dist < 5.0) {
                auto bev_split_lane_iter0 = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return bev_ego_lane_iter->next_lane_ids[0] == lane.id && lane.geos->size() >= 3;});
                auto bev_split_lane_iter1 = std::find_if(bevMapPtr->lane_infos.begin(), bevMapPtr->lane_infos.end(),
                                                    [&](const cem::message::sensor::BevLaneInfo &lane) { return bev_ego_lane_iter->next_lane_ids[1] == lane.id && lane.geos->size() >= 3;});
                if(bev_split_lane_iter0 == bevMapPtr->lane_infos.end() || bev_split_lane_iter1 == bevMapPtr->lane_infos.end()) {
                    return;
                }
                if(LaneGeometry::JudgeIsLeft(*bev_split_lane_iter0->geos, *bev_split_lane_iter1->geos)) {
                   double split0_2_left_dist = LaneGeometry::GetDistanceBetweenLines(*bev_split_lane_iter0->geos, *left_first_lane_iter->geos);
                   double split1_2_right_dist = LaneGeometry::GetDistanceBetweenLines(*bev_split_lane_iter1->geos, *right_first_lane_iter->geos);
                   if(split0_2_left_dist < 2.0 && split1_2_right_dist > 3.5 && split1_2_right_dist < 5.0) {
                        bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(),bev_ego_lane_iter->next_lane_ids.end(), bev_split_lane_iter0->id), bev_ego_lane_iter->next_lane_ids.end());
                        bev_split_lane_iter0->previous_lane_ids.erase(remove(bev_split_lane_iter0->previous_lane_ids.begin(), bev_split_lane_iter0->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter0->previous_lane_ids.end());
                        // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter0->id;
                   }
                   else if(split1_2_right_dist < 2.0 && split0_2_left_dist > 3.0 && split0_2_left_dist < 5.0) {
                        bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(), bev_ego_lane_iter->next_lane_ids.end(), bev_split_lane_iter1->id), bev_ego_lane_iter->next_lane_ids.end());
                        bev_split_lane_iter1->previous_lane_ids.erase(remove(bev_split_lane_iter1->previous_lane_ids.begin(), bev_split_lane_iter1->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter1->previous_lane_ids.end());
                        // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter1->id;
                   }
                }
                else{
                        double split0_2_right_dist = LaneGeometry::GetDistanceBetweenLines(*bev_split_lane_iter0->geos, *right_first_lane_iter->geos);
                        double split1_2_left_dist = LaneGeometry::GetDistanceBetweenLines(*bev_split_lane_iter1->geos, *left_first_lane_iter->geos);
                        if(split0_2_right_dist < 2.0 && split1_2_left_dist > 3.5 && split1_2_left_dist < 5.0) {
                            bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(),bev_ego_lane_iter->next_lane_ids.end(), bev_split_lane_iter0->id), bev_ego_lane_iter->next_lane_ids.end());
                            bev_split_lane_iter0->previous_lane_ids.erase(remove(bev_split_lane_iter0->previous_lane_ids.begin(), bev_split_lane_iter0->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter0->previous_lane_ids.end());
                            // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter0->id;
                        }
                        else if(split1_2_left_dist < 2.0 && split0_2_right_dist > 3.5 && split0_2_right_dist < 5.0) {
                            bev_ego_lane_iter->next_lane_ids.erase(remove(bev_ego_lane_iter->next_lane_ids.begin(), bev_ego_lane_iter->next_lane_ids.end(), bev_split_lane_iter1->id), bev_ego_lane_iter->next_lane_ids.end());
                            bev_split_lane_iter1->previous_lane_ids.erase(remove(bev_split_lane_iter1->previous_lane_ids.begin(), bev_split_lane_iter1->previous_lane_ids.end(), bev_ego_lane_iter->id), bev_split_lane_iter1->previous_lane_ids.end());
                            // AINFO<<"erase split topo ego lane id:"<<bev_ego_lane_iter->id<<" split id:"<<bev_split_lane_iter1->id;
                        }
                }
            }
            else{
                 return;
            }
          }
          else {
              return;
          }
    }
}
bool BuildLaneTopology::CheckMatchRes(BevMapInfoPtr &bevMapPtr,RoutingMapPtr &routingMapPtr,std::unordered_map<uint64_t, std::vector<uint64_t>>& bev_ld_match)
{
   if(!bevMapPtr || !routingMapPtr || bev_ld_match.empty())
   {
      return false;
   }
   return true;
}
bool BuildLaneTopology::CheckValidRoadClass()
{
    bool ret = false;
    auto *sensor_manager = SensorDataManager::Instance();
    if (!sensor_manager)
    {
        return ret;
    }
    RoutingMapPtr routing_map_raw = nullptr;
    sensor_manager->GetLatestSensorFrame(routing_map_raw);
    if(nullptr == routing_map_raw)
    {
        return ret;
    }
    if(true == routing_map_raw->is_on_highway)
    {
        ret = true;
        return ret;
    }
    #if 0
    const auto& navi_start = routing_map_raw->sd_route.navi_start;
    const auto& mpp_section = routing_map_raw->sd_route.mpp_sections;
    auto it_navi_seciton = std::find_if(
        mpp_section.begin(), mpp_section.end(),
        [&navi_start](const cem::message::env_model::SDSectionInfo& p) {
          return p.id == navi_start.section_id;
        });

    double navi_seciton_length = -navi_start.s_offset;
    while (it_navi_seciton != mpp_section.end() )
    {
        navi_seciton_length += it_navi_seciton->length;
        auto road_class = it_navi_seciton->road_class;
        if (road_class <= SDRoadClass::SD_CITY_FAST_WAY)
        {
           ret = true;
        }
        it_navi_seciton += 1;
    }
    #endif
    return ret;
}
bool BuildLaneTopology::isNearToToll()
{
    bool ret = false;
    auto *sensor_manager = SensorDataManager::Instance();
    if (!sensor_manager)
    {
        return ret;
    }
    MapEventPtr map_event_ptr;
    sensor_manager->GetLatestSensorFrame(map_event_ptr);
    if (!map_event_ptr)
    {
        return ret;
    }

    for (const auto &reminder_info : map_event_ptr->reminder_way_info)
    {
        if (reminder_info.reminder_waypoint_type == ReminderWayPoint::TOLL_BOOTH &&
            map_event_ptr->control_waypoint_dis < NOT_BUILD_TOPO_DISTANCE)
        {
            ret = true;
            break;
        }
    }
    return ret;
}
bool BuildLaneTopology::isLDMapLowPrecision()
{
    bool ret = false;
    MapEventPtr map_event_ptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(map_event_ptr);
    const static float LD_MAP_LOW_PRECISION_DISTANCE_LIMIT = 100.0f;
    if(nullptr != map_event_ptr)
    {
        bool has_low_precision_zone = false;
        double min_start_offset = 100000.0;
        double max_end_offset = -100000.0;

        for(const auto& oneOddInfo : map_event_ptr->odd_info)
        {
            if (LOW_PRECISION == oneOddInfo.value)
            {
                //Get the min distance of the low precision.
                has_low_precision_zone = true;
                if (min_start_offset > oneOddInfo.start_offset)
                {
                    min_start_offset = oneOddInfo.start_offset;
                }
                if (max_end_offset < oneOddInfo.end_offset)
                {
                    max_end_offset = oneOddInfo.end_offset;
                }
            }
        }
        if(true == has_low_precision_zone)
        {
            if(min_start_offset < LD_MAP_LOW_PRECISION_DISTANCE_LIMIT && max_end_offset > 0)
            {
                ret = true;
            }
        }
    }
    return ret;
}
void BuildLaneTopology::BuildNewMergeSplitTopo(
    BevMapInfoPtr local_map,
    std::shared_ptr<cem::message::env_model::RoutingMap> ld_map,
    const bev_map_ids_info & bev_map_ids)
{

    isICCmode();
    GetCurrentSpeed();
    bool valid_road_class = CheckValidRoadClass();
    bool isLowPrecision = isLDMapLowPrecision();
    bool isInToll = isNearToToll();
    if(false == valid_road_class)
    {//20251202： 暂时只考虑高快有LD地图场景才构建和校验拓扑。
        return;
    }
    /*1. 过滤一些非高快和城市道路，不构建merge拓扑。;
     *2. 靠近收费站不构建split拓扑;
     */
    if(true == valid_road_class && false == isInToll)
    {
        BuildMergeTopo(local_map,ld_map,bev_map_ids);
    }
    /*1. 地图低精不需要构建split拓扑;
     *2. 靠近收费站不构建split拓扑;
     */
    if(nullptr != ld_map && false == isLowPrecision && false == isInToll)
    {
        BuildSplitTopoWithLdMap(local_map,ld_map,bev_map_ids);
    }
    std::unordered_map<uint64_t, std::vector<uint64_t>> bev_ld_match;
    for(auto& match : bev_map_ids.local_map_matched_ids) {
      vector<uint64_t> ld_map_ids;
      for(auto& map_id: match.second) {
        ld_map_ids.push_back(map_id);
      }
      bev_ld_match.insert({match.first,ld_map_ids});
    }
    CheckMergeTopo(local_map,ld_map,bev_ld_match);
    CheckSplitTopo(local_map,ld_map,bev_map_ids);
    return;
}
}
}
