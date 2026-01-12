#include "road_divider.h"

namespace cem {
namespace fusion {


void RoadDivider::Process(BevMapInfo& bev_map) {
    if (is_highway_) {
        FindEgoLaneIds(bev_map);
    }

    for (auto& lane : bev_map.lane_infos) {
        if (lane.is_topo_disassembled == true) {
            lane.road_id = 3; //拆开拓扑为subpath3
        } else if (lane.direction == BevLaneDirection::DIRECTION_BACKWARD) {
            lane.road_id = 2; //逆向车道为subpath2
            if (is_highway_ && ego_lane_ids_.count(lane.id)) {
                lane.road_id = 0; //高速上保证自车道为主路（城区小路超车时可能开到逆向车道，所以不能兜底）
            }
        } else {
            lane.road_id = 0; //主路为route
        }
    }

    std::sort(bev_map.lane_infos.begin(), bev_map.lane_infos.end(),
              [](const BevLaneInfo& a, const BevLaneInfo& b) {
                  return a.road_id < b.road_id;
              });
}

void RoadDivider::FindEgoLaneIds(BevMapInfo& bev_map) {
    ego_lane_ids_.clear();
    for (const auto& lane : bev_map.lane_infos) {
        if (!lane.geos) continue;
        for (const auto& ego_point : *(lane.geos)) {
            if (std::abs(ego_point.x()) < 10.0 && std::abs(ego_point.y()) < 2.0) {
                ego_lane_ids_.insert(lane.id);
                break;
            }
        }
    }
}


bool RoadDivider::FindDiversionRouteAndSubpath(BevMapInfo &bev_map, std::vector<BevMapSectionInfo> &sections) {
    // 计算导流区虚拟路沿，并按从左到右的顺序排序
    if(bev_map.diversion_zone.empty()) return false;
    std::vector<std::vector<Eigen::Vector3d>> diver_lines;
    for(const auto& diver_zone : bev_map.diversion_zone) {
        if(diver_zone.geos && diver_zone.geos->size()==4) {
            std::vector<Eigen::Vector2f> points = *diver_zone.geos;
            std::sort(points.begin(), points.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a.x() < b.x(); });
            Eigen::Vector3d bottom = {(points[0].x() + points[1].x()) / 2.0, (points[0].y() + points[1].y()) / 2.0, 0};
            Eigen::Vector3d top    = {(points[2].x() + points[3].x()) / 2.0, (points[2].y() + points[3].y()) / 2.0, 0};
            Eigen::Vector3d front  = bottom + (top - bottom) / 10.0f;
            Eigen::Vector3d center = (bottom + top) / 2.0f;
            Eigen::Vector3d back   = top - (top - bottom) / 10.0f;
            std::vector<Eigen::Vector3d> line = {bottom, front, center, back, top};
            diver_lines.emplace_back(line);
        }
    }
    size_t at_bottom=0, at_front=1, at_center=2, at_back=3, at_top=4;
    std::sort(diver_lines.begin(), diver_lines.end(),
              [at_center](const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b) {
                  return a[at_center].y() > b[at_center].y();
              });
    
    // 计算自车车身虚拟线，用于判断section是否在自车后方2米至前方8米的范围内
    Eigen::Vector3d front  = Eigen::Vector3d(-2, 0, 0);
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d back   = Eigen::Vector3d(8, 0, 0);
    const std::vector<Eigen::Vector3d> ego_line = {front, center, back};

    std::vector<BevMapSectionInfo> subpath_sections;
    uint64_t subpath_section_id = 100;
    for(auto &section : sections) {
        // 如果存在路口连接线，那么全部作为route
        bool exist_junction = false;
        for(const auto &lane : section.lane_infos) {
            if(lane.id % 1000 > 100) {
                exist_junction = true;
                break;
            }
        }
        if(exist_junction) continue;

        // 如果section起始的切割线不存在，那么采用lane起点计算切割线
        if(section.front_left == section.front_right) {
            std::vector<Eigen::Vector3d> lane_points;
            for(const auto &lane : section.lane_infos) {
                Eigen::Vector3d lane_point(lane.line_points.front().x, lane.line_points.front().y, 0);
                lane_points.emplace_back(T_local_ego_.inverse() * lane_point);
            }
            auto min_it = std::min_element(lane_points.begin(), lane_points.end(), 
                                           [](Eigen::Vector3d &a, Eigen::Vector3d &b){ return a.y() < b.y(); });
            if(min_it == lane_points.end()) continue;
            auto max_it = std::max_element(lane_points.begin(), lane_points.end(), 
                                           [](Eigen::Vector3d &a, Eigen::Vector3d &b){ return a.y() > b.y(); });
            if(max_it == lane_points.end()) continue;
            if(std::abs(max_it->x() - min_it->x()) < 1e-5 && std::abs(max_it->y() - min_it->y()) < 1e-5) {
                section.front_left = *max_it + Eigen::Vector3d(0, 1, 0);
                section.front_right = *min_it - Eigen::Vector3d(0, 1, 0);
            } else {
                section.front_left = *max_it;
                section.front_right = *min_it;
            }
        }
        // 如果section末尾的切割线不存在，那么采用lane尾点计算切割线
        if(section.back_left == section.back_right) {
            std::vector<Eigen::Vector3d> lane_points;
            for(const auto &lane : section.lane_infos) {
                Eigen::Vector3d lane_point(lane.line_points.back().x, lane.line_points.back().y, 0);
                lane_points.emplace_back(T_local_ego_.inverse() * lane_point);
            }
            auto min_it = std::min_element(lane_points.begin(), lane_points.end(), 
                                           [](Eigen::Vector3d &a, Eigen::Vector3d &b){ return a.y() < b.y(); });
            if(min_it == lane_points.end()) continue;
            auto max_it = std::max_element(lane_points.begin(), lane_points.end(), 
                                           [](Eigen::Vector3d &a, Eigen::Vector3d &b){ return a.y() > b.y(); });
            if(max_it == lane_points.end()) continue;
            if(std::abs(max_it->x() - min_it->x()) < 1e-5 && std::abs(max_it->y() - min_it->y()) < 1e-5) {
                section.back_left = *max_it + Eigen::Vector3d(0, 1, 0);
                section.back_right = *min_it - Eigen::Vector3d(0, 1, 0);
            } else {
                section.back_left = *max_it;
                section.back_right = *min_it;
            }
        }
        BevMapSectionInfo route_section = section;
        route_section.lane_infos.clear();

        // 判断当前section是否在自车附近
        if(ego_line.size() != 3) return false;
        Eigen::Vector3d section_front_vec = section.front_right - section.front_left;
        Eigen::Vector3d ego_back_vec = ego_line.back() - section.front_left;
        Eigen::Vector3d section_back_vec = section.back_right - section.back_left;
        Eigen::Vector3d ego_front_vec = ego_line.front() - section.back_left;
        if(section_front_vec.cross(ego_back_vec).z() < 0 || section_back_vec.cross(ego_front_vec).z() > 0) {
            continue; //如果当前section不在自车附近，那么跳过
        }

        // 判断当前section是否包含导流区，并且导流区是否在自车附近
        for(const auto &diver_line : diver_lines) {
            if(diver_line.size() != 5) continue;
            Eigen::Vector3d diver_back_vec = diver_line[at_back] - section.front_left;
            Eigen::Vector3d diver_front_vec = diver_line[at_front] - section.back_left;
            if(section_front_vec.cross(diver_back_vec).z() < 0 || section_back_vec.cross(diver_front_vec).z() > 0) {
                continue; //如果当前section不包含导流区，那么跳过
            }
            if(diver_line[at_top].x() < -1 || diver_line[at_bottom].x() > 4) {
                continue; //如果导流区不在自车后方1米至前方4米的范围内，那么跳过
            }

            // 根据导流区进行section划分
            BevMapSectionInfo left_section, right_section;
            Eigen::Vector3d ego_vec = ego_line.back() - ego_line.front();
            Eigen::Vector3d diver_vec = diver_line[at_center] - ego_line.front();
            bool is_ego_left_of_diver = diver_vec.cross(ego_vec).z() > 0;
            for(const auto &lane : section.lane_infos) {
                Eigen::Vector3d lane_front, lane_back;
                if(lane.line_points.empty()) {
                    continue;
                } else if(lane.line_points.size() == 1) {
                    lane_front = T_local_ego_.inverse() * Eigen::Vector3d(lane.line_points.front().x, lane.line_points.front().y, 0);
                    lane_back = lane_front + Eigen::Vector3d(1, 0, 0);
                } else {
                    lane_front = T_local_ego_.inverse() * Eigen::Vector3d(lane.line_points.front().x, lane.line_points.front().y, 0);
                    lane_back = T_local_ego_.inverse() * Eigen::Vector3d(lane.line_points.back().x, lane.line_points.back().y, 0);
                }
                Eigen::Vector3d lane_vec = lane_back - lane_front;
                Eigen::Vector3d diver_vec = diver_line[at_center] - lane_front;
                bool is_lane_left_of_diver = diver_vec.cross(lane_vec).z() > 0;
                if(lane.position == 0) {
                    is_lane_left_of_diver = is_ego_left_of_diver;
                }
                if(is_lane_left_of_diver) { //lane位于导流区左边
                    left_section.lane_infos.emplace_back(lane);
                } else {
                    right_section.lane_infos.emplace_back(lane);
                }
            }

            // 将自车所在岔路放进route，将其余岔路放进subpath
            if(left_section.lane_infos.empty()) continue;
            if(is_ego_left_of_diver && route_section.lane_infos.empty()) { //自车位于left_section，则作为route
                route_section.lane_infos = left_section.lane_infos;
            } else {                                                       //自车不位于left_section，则作为subpath
                left_section.id = ++subpath_section_id;
                subpath_sections.emplace_back(left_section);
            }
            section.lane_infos = right_section.lane_infos; //导流区右边lane继续划分
        }

        // 如果route为空，那么将最右边的lane作为route，否则作为subpath
        if(route_section.lane_infos.empty()) {
            route_section.lane_infos = section.lane_infos;
        } else {
            BevMapSectionInfo sub_section;
            sub_section.lane_infos = section.lane_infos;
            sub_section.id = ++subpath_section_id;
            subpath_sections.emplace_back(sub_section);
        }
        section = route_section;
    }
    
    // 如果subpath非空，那么返回true，否则返回false
    if(!subpath_sections.empty()) {
        BevMapSubPathInfo subpath;
        subpath.sections = subpath_sections;
        subpath.enter_section_id = subpath_sections.front().id;
        bev_map.route.subpaths.emplace_back(subpath);
        return true;
    } else {
        return false;
    }
}

}
}