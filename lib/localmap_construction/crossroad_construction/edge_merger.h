/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-08-08
 * @copyright Copyright (c) 2025, BYD
 */
 #ifndef __EDGE_MERGER_H__
 #define __EDGE_MERGER_H__
#include "base/params_manager/params_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "cross_common.h"
#include <Eigen/Core>
#include <common/utils/lane_geometry.h>
#include "lib/common/utility.h"
#include <cstdint>
#include <unordered_map>
#include <variant>
#include "lib/common/log_custom.h"
#include "draw_point.h"
#include "occ_processor.h"


using namespace byd::common::math;
using namespace cem::message::sensor;

namespace cem {
namespace fusion{
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
class CrossDataManager;

class EdgeMerger { 
private:
    std::vector<Point> vertices;
    const double EPSILON = 1e-6;
    std::shared_ptr<BevMapInfo> map_info_ptr_{};
    BevLaneMarker plane_lane_marker_{};
    std::unordered_map<uint64_t,std::vector<Eigen::Vector2d>> occ_edges_;//输入
    std::unordered_map<uint64_t,std::vector<Eigen::Vector2d>> bev_edges_;//输入
    std::unordered_map<uint64_t,std::vector<Eigen::Vector2d>> merged_edges_;//输出
    
    std::shared_ptr<CrossDataManager> data_manager_ptr_{nullptr};
    Eigen::Isometry3d Twb_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d Tbw_{Eigen::Isometry3d::Identity()};
    std::vector<std::pair<int, int>> offsets_;
    RoutingMapPtr routing_map_{nullptr};
    BevMapInfoPtr bev_map_{nullptr};
    GridMapFrame current_map;
    std::shared_ptr<ppm::PPMImage> ppmImage_{nullptr};
    std::deque<GridMapFrame> history_occ_grids_;


public:


    // 计算叉积 (P1P2 × P1P3)
    double CrossProduct(const Point& p1, const Point& p2, const Point& p3);
    double pointDistance(const Point2D& p1, const Point2D& p2);
    void  SplitBoundaries(std::vector<RoadBoundaryInfo>& road_boundaries, 
                     double split_threshold = 2.0);
    void  SplitBoundaries(std::vector<BevLaneMarker>& road_boundaries, 
                     double split_threshold = 2.0);
    
    int PositionFunction(const Point& a, const Point& b, const Point& p) ;
    void Preprocess();
    void Process();
    void MergeRoutingMapEdge();
    void MergeRoutingMapEdgeOccFirst();

public:
    void AddVertex(const double& x, const double& y);
    bool HasSelfIntersection();
    // 点包含检测主函数（支持自交多边形）
    bool Contains(const Point& p);
    // 奇偶规则实现
    bool ContainsEvenOdd(const Point& p) ;
    // 非零环绕数实现
    bool ContainsNonZero(const Point& p) ;
    void ClearVertices();
    void Init();
    void SetCrossDataManager( std::shared_ptr<CrossDataManager> ptr) { data_manager_ptr_ = ptr; }//可修改
    bool on_highway_{false};
};
}//cem
}//fusion
#endif // __EDGE_MERGER_H__