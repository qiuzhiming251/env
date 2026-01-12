/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-03
 * @copyright Copyright (c) 2025, BYD
 */
 #ifndef __LANE_LINE_REFINER_H__
 #define __LANE_LINE_REFINER_H__
#include "base/params_manager/params_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "cross_common.h"
#include <Eigen/Core>
// #include "cross_data_manager.h"
#include <common/utils/lane_geometry.h>
#include "lib/common/utility.h"
#include <cstdint>
#include <unordered_map>
#include <variant>
#include "lib/common/log_custom.h"

using namespace byd::common::math;
using namespace cem::message::sensor;
#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "cross_data_manager.h"

namespace cem {
namespace fusion{
class CrossDataManager;

enum FillRule {
    EvenOdd,    // 奇偶规则
    NonZero     // 非零环绕数
};

class LaneLineRefiner { 
private:
    std::vector<Point> vertices;
    const double EPSILON = 1e-5;
    std::shared_ptr<BevMapInfo> map_info_ptr_{};
    BevLaneMarker plane_lane_marker_{};
    std::shared_ptr<CrossDataManager> cross_data_manager_ptr_{};
public:


    // 计算叉积 (P1P2 × P1P3)
    double CrossProduct(const Point& p1, const Point& p2, const Point& p3);

    
    int PositionFunction(const Point& a, const Point& b, const Point& p) ;
    void Process();

public:
    void AddVertex(const double& x, const double& y);
    bool HasSelfIntersection();
    // 点包含检测主函数（支持自交多边形）
    bool Contains(const Point& p, FillRule rule = EvenOdd);
    // 奇偶规则实现
    bool ContainsEvenOdd(const Point& p) ;
    // 非零环绕数实现
    bool ContainsNonZero(const Point& p) ;
    void ClearVertices();
    void SetBevMap(std::shared_ptr<BevMapInfo> map_info_ptr) { map_info_ptr_ = map_info_ptr; }
    void SetCrossDateManager(std::shared_ptr<CrossDataManager> data_manager_ptr) {  cross_data_manager_ptr_= data_manager_ptr; }
    void SetPlaneLanemarker(BevLaneMarker& bev_lane_marker) {
          plane_lane_marker_ = bev_lane_marker;
            ClearVertices();
            for (auto& p : plane_lane_marker_.line_points) {
                AddVertex(p.x, p.y);
            }
        };//保持上个最近值 可以带上时间戳
    Point GetCentroid();
};
}//cem
}//fusion
#endif // __LANE_LINE_REFINER_H__