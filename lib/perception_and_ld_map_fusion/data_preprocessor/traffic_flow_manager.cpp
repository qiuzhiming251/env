#include "traffic_flow_manager.h"

namespace cem {
namespace fusion {

TrafficFlowManager::TrafficFlowManager()
{

}
TrafficFlowManager::~TrafficFlowManager()
{

}
float TrafficFlowManager::GetDistPointLane(
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
float TrafficFlowManager::Point2LineDistanece(
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
void TrafficFlowManager::ParseTrafficFlowRawData(std::unordered_map<uint32_t, FusionObjs> & objs)
{
    FusObjInfoPtr fusion_obj_info_ptr_{nullptr};
    SensorDataManager::Instance()->GetLatestSensorFrame(fusion_obj_info_ptr_);

    std::unordered_map<uint32_t, FusionObjs>().swap(fusion_objs_) ;
    if (fusion_obj_info_ptr_ != nullptr)
    {
        for (const auto& object : fusion_obj_info_ptr_->fused_obstacles)
        {
            if(object.type == static_cast<uint32_t>(ObjectTypeClass::CAR) ||
               object.type == static_cast<uint32_t>(ObjectTypeClass::TRUCK) ||
               object.type == static_cast<uint32_t>(ObjectTypeClass::TRAM) ||
               object.type == static_cast<uint32_t>(ObjectTypeClass::TRAILER_HEAD) ||
               object.type == static_cast<uint32_t>(ObjectTypeClass::TRAILER_REAR))
            {
                auto& obj = fusion_objs_[object.id];
                obj.id = object.id;
                obj.source = object.fusion_source;
                obj.position.x = object.position(0);
                obj.position.y = object.position(1);
                obj.position.z = object.position(2);
                obj.speed = object.speed;
                obj.length = object.length;
                obj.width = object.width;
                obj.heading = object.heading_angle;
                obj.curvature = object.curvature;
                objs[object.id] = obj;
            }
        }
    }
    return;
}
void TrafficFlowManager::BindObstaclesToBevLane(const BevMapInfoPtr& localmap,std::vector<MotionObject2Lane> & object2Lanes)
{
    if(fusion_objs_.size() <= 0)
    {
        return;
    }
    for(auto &elem : fusion_objs_)
    {
        float x_coord = static_cast<float>(elem.second.position.x);
        float y_coord = static_cast<float>(elem.second.position.y);
        uint32_t lane_id = 0;
        int bev_left_lanemarker_id = -1;
        int bev_right_lanemarker_id = -1;
        Eigen::Vector2f A(x_coord, y_coord);
        bool found = false;
        int lane_idx = -1;
        for (size_t i = 0; i < localmap->lane_infos.size() && !found; i++)
        {
            auto &bev_lane = localmap->lane_infos[i];
            lane_id = bev_lane.id;
            if(lane_id <= 0 || bev_lane.line_points.size() < 2)
            {
                continue;
            }
            if(bev_lane.position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_LEFT_FIRST) ||
               bev_lane.position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_EGO) ||
               bev_lane.position != static_cast<uint32_t>(BevLanePosition::LANE_LOC_RIGHT_FIRST) )
            {
                continue;
            }
            int index = -1;
            float minX = FLT_MAX;
            for(int j = 0; j < bev_lane.line_points.size(); j++)
            {
                if(fabs(bev_lane.line_points[j].x - x_coord) < minX)
                {
                    minX = fabs(bev_lane.line_points[j].x - x_coord);
                    index = j;
                }
            }
            if(index >= 0 && minX < FLT_MAX && bev_lane.line_points.size() > 2)
            {
                Eigen::Vector2f B,C;
                B(0) = bev_lane.line_points[index].x;
                B(1) = bev_lane.line_points[index].y;
                if(index == bev_lane.line_points.size()-1)
                {
                    C(0) = bev_lane.line_points[bev_lane.line_points.size()-2].x;
                    C(1) = bev_lane.line_points[bev_lane.line_points.size()-2].y;
                }
                else if(index == 0)
                {
                    C(0) = bev_lane.line_points[1].x;
                    C(1) = bev_lane.line_points[1].y;
                }
                else
                {
                    C(0) = bev_lane.line_points[index+1].x;
                    C(1) = bev_lane.line_points[index+1].y;
                }
                float dist_diff =  fabs(GetDistPointLane(A,B,C));
                if(dist_diff < bev_lane.width/2.0)
                {
                    found = true;
                    lane_idx = i;
                }
            }
        }

        if(true == found && lane_id > 0)
        {
            int index = -1;
            for(size_t k = 0; k < object2Lanes.size(); k++)
            {
                if(object2Lanes[k].lane_id == lane_id)
                {
                    index = static_cast<int>(k);
                    break;
                }
            }
            if(index < 0)
            {
                MotionObject2Lane mol;
                mol.lane_id = lane_id;
                mol.motionObjects.emplace_back(elem.second);
                object2Lanes.emplace_back(mol);
            }
            else if(index >=0 && index < object2Lanes.size())
            {
                object2Lanes[index].motionObjects.emplace_back(elem.second);
            }
        }
    }
    return;
}

void TrafficFlowManager::BindObstaclesToLdMapLane(const RoutingMapPtr& ldmap,std::vector<MotionObject2Lane> & object2Lanes)
{
    if(fusion_objs_.size() <= 0)
    {
        return;
    }
    for(auto &elem : fusion_objs_)
    {
        float x_coord = static_cast<float>(elem.second.position.x);
        float y_coord = static_cast<float>(elem.second.position.y);
        uint32_t lane_id = 0;
        int bev_left_lanemarker_id = -1;
        int bev_right_lanemarker_id = -1;
        Eigen::Vector2f A(x_coord, y_coord);
        bool found = false;
        int lane_idx = -1;
        for (size_t i = 0; i < ldmap->lanes.size() && !found; i++)
        {
            auto &bev_lane = ldmap->lanes[i];
            lane_id = bev_lane.id;
            if(lane_id <= 0 || bev_lane.points.size() <= 0)
            {
                continue;
            }

            std::vector<Point> map_raw_points;
            for(auto &pt: bev_lane.points)
            {
                Point pts;
                pts.x = pt.x;
                pts.y = pt.y;
                map_raw_points.push_back(pts);
            }
            #if 0
            std::vector<Point> map_interpolated_points;
            int idx = 0;
            float INTERVAL = 2.0;
            Point last_point;
            size_t last_idx = (map_raw_points.size() - 1);
            while(idx < map_raw_points.size())
            {
                if(idx == 0 || idx == last_idx)
                 {//first or last
                    map_interpolated_points.emplace_back(map_raw_points[idx]);
                    last_point = map_raw_points[idx];
                    idx++;
                    continue;
                }
                Eigen::Vector2f delta;
                delta(0) = map_raw_points[idx].x - last_point.x;
                delta(1) = map_raw_points[idx].y - last_point.y;
                if(delta.norm() > INTERVAL)
                {//interploting
                    int n = 1;
                    float STEP = INTERVAL/2.0;
                    Eigen::Vector2f unit_delta_vec;
                    unit_delta_vec = delta/delta.norm();
                    while(n*STEP < (delta.norm()-INTERVAL/4.0))
                    {
                        Point new_pts;
                        new_pts.x = last_point.x + unit_delta_vec(0)*n*STEP;
                        new_pts.y = last_point.y + unit_delta_vec(1)*n*STEP;
                        map_interpolated_points.emplace_back(new_pts);
                        n++;
                    }
                    map_interpolated_points.emplace_back(map_raw_points[idx]);
                    last_point = map_raw_points[idx];
                }
                else if(delta.norm() >= INTERVAL/2.0 && delta.norm() <= INTERVAL)
                {
                    if(idx == (last_idx - 2))
                    {
                        Eigen::Vector2f last_delta;
                        last_delta(0) = map_raw_points[last_idx].x - map_raw_points[idx].x;
                        last_delta(1) = map_raw_points[last_idx].y - map_raw_points[idx].y;
                        if(last_delta.norm() <= INTERVAL)
                        {//skip the last_idx - 2
                            idx = idx + 2;
                            continue;
                        }
                    }
                    map_interpolated_points.emplace_back(map_raw_points[idx]);
                    last_point = map_raw_points[idx];
                }
                idx++;
            }
            #endif
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
                if(dist_diff < 1.5)
                {
                    found = true;
                    lane_idx = i;
                }
            }
        }

        if(true == found && lane_id > 0)
        {
            int index = -1;
            for(size_t k = 0; k < object2Lanes.size(); k++)
            {
                if(object2Lanes[k].lane_id == lane_id)
                {
                    index = static_cast<int>(k);
                    break;
                }
            }
            if(index < 0)
            {
                MotionObject2Lane mol;
                mol.lane_id = lane_id;
                mol.motionObjects.emplace_back(elem.second);
                object2Lanes.emplace_back(mol);
            }
            else if(index >=0 && index < object2Lanes.size())
            {
                object2Lanes[index].motionObjects.emplace_back(elem.second);
            }
        }
    }

    return;
}

}
}
