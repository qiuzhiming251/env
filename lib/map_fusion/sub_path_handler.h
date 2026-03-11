#ifndef SUB_PATH_HANDLER_H
#define SUB_PATH_HANDLER_H

#include "message/env_model/routing_map/routing_map.h"
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "message/env_model/routing_map/routing_map.h"
#include "modules/perception/env/src/lib/message/internal_message.h"

namespace cem {
namespace fusion {

struct RouteSubPathsInfo
{
    struct SubPathInfo
    {
        uint64_t enter_section_id;         // 分流或合流的入口 section ID
        double distance_to_enter;          // 到入口点的距离
        bool is_current_exit;              // 是否为导航当前选择的出口（仅对分流有效）
        std::vector<uint64_t> section_ids; // 路由中相关的 section ID 列表
    };

    struct CrossPoint
    {
        double x;
        double y;
        double z;
        bool is_merging; // true 表示合流，false 表示分流
        uint64_t section_id;
    };

    std::vector<SubPathInfo> merging_subpaths;            // 合流子路径
    std::vector<SubPathInfo> diverging_subpaths;          // 分流子路径
    std::vector<SubPathInfo> diverging_lane_dec_subpaths; // 车道减少的分流子路径
    std::vector<CrossPoint> cross_points;                 // 分/合流交叉点信息
};

class SubPathProcessor
{
public:
    SubPathProcessor();
    void Process(const RoutingMapPtr& routing_map_ptr);
    const RouteSubPathsInfo* GetRouteSubPathsInfo() const;
    void PrintRouteSubPathsInfo(const RoutingMapPtr& routing_map_ptr) const;

private:
    void Reset();
    void ExtractMergingSubPaths(const std::vector<SectionInfo>& sections, size_t index, double distance_to_start, const std::unordered_set<uint64_t>& route_section_ids);
    void ExtractDivergingSubPaths(const std::vector<SectionInfo>& sections, size_t index, double distance_to_end, const std::unordered_set<uint64_t>& route_section_ids);
    void ExtractCrossPoints(const RoutingMapPtr& routing_map_ptr);
    void ExtractMergingCrossPoints(const RoutingMapPtr& routing_map_ptr);
    void ExtractDivergingCrossPoints(const RoutingMapPtr& routing_map_ptr);
    const SectionInfo* FindSectionById(const std::vector<SectionInfo>& sections, uint64_t section_id) const;
    const LaneInfo* FindLaneById(const std::vector<LaneInfo>& lanes, uint64_t lane_id) const;
    const LaneBoundaryInfo* FindLaneBoundaryById(const std::vector<LaneBoundaryInfo>& boundaries, uint64_t boundary_id) const;
    bool IsRampSection(const RoutingMapPtr& routing_map_ptr, const SectionInfo* section) const;
    const LaneBoundaryInfo* GetRightmostBoundary(const RoutingMapPtr& routing_map_ptr, const SectionInfo* section) const;

    void ExtractSubPathsFromSubpaths(const RoutingMapPtr& routing_map_ptr, const NaviPosition& current_position);
    double ComputeDistanceToEnterSection(const NaviPosition& current_position, const std::vector<SectionInfo>& sections, uint64_t enter_section_id);
    bool IsSubPathMerging(const SubPath& subpath, uint64_t enter_section_id);

    // 数据成员
    std::unique_ptr<RouteSubPathsInfo> subpaths_info_;             // 子路径信息
    std::unordered_map<uint64_t, const SectionInfo*> section_map_; // section ID 到 SectionInfo 的映射
    std::unordered_map<uint64_t, const LaneInfo*> lane_map_;       // lane ID 到 LaneInfo 的映射
    std::vector<double> cumulative_distances_;                     // 路由中每个 section 的累计距离
    size_t current_index_;                                         // 当前 section 在路由中的索引
    double current_distance_;                                      // 当前位置的累计距离
};

} // namespace fusion
} // namespace cem

#endif // SUB_PATH_HANDLER_H