/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-015
 * @copyright Copyright (c) 2025, BYD
 */
 #ifndef __POINT_DBSCAN_H__
 #define __POINT_DBSCAN_H__
#include <opencv2/opencv.hpp>
#include "base/params_manager/params_manager.h"
#include "base/sensor_data_manager.h"
#include "lib/message/internal_message.h"
#include "lib/common/log_custom.h"
#include "message/sensor/camera/bev_lane/bev_lane.h"
#include "cross_common.h"
#include <Eigen/Core>
#include "cross_data_manager.h"
#include <common/utils/lane_geometry.h>
#include "lib/common/utility.h"
#include <cstdint>
#include <unordered_map>
#include <variant>
#include "lib/common/log_custom.h"
#include "cross_data_manager.h"
#include "draw_point.h"
using namespace byd::common::math;
using namespace cem::message::sensor;

namespace cem {
namespace fusion{
const int UNCLASSIFIED = -1;
const int NOISE = -2;
std::vector<int> regionQueryKDTree(
     cv::flann::Index& kdTree, 
    const cv::Point2f& point, 
    float eps, 
    const std::vector<cv::Point2f>& points
);

void GridDBScan(GridMap& grid,const std::vector<GridIndex>& validPoints, int  eps, int minPts);
std::vector<int> dbscan(const std::vector<cv::Point2f>& points, float eps, int minPts, std::vector<int>& labels);
Eigen::VectorXd polyfit(const std::vector<cv::Point2f>& points,bool along_x, int order);
}//cem
}//fusion
#endif // __LANE_LINE_REFINER_H__