/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-07
 * @copyright Copyright (c) 2025, BYD
 */
 #ifndef __OCC_PROCESSOR_H__
 #define __OCC_PROCESSOR_H__
// #include<opencv2/ximgproc.hpp>
#include <opencv2/opencv.hpp>
#include "base/params_manager/params_manager.h"
#include "base/sensor_data_manager.h"
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
#include "lib/common/log_custom.h"
// #include "kde_smoother.h"
#include "kde_regression.h"
using namespace byd::common::math;
using namespace cem::message::sensor;
#define OCC_HEIGHT 404 //608 + 20*5  后面扩展20m
#define OCC_WIEDTH 384//96*2    侧面扩展一倍
namespace cem {
namespace fusion{
class CrossDataManager;
typedef struct gridindex
{
    int x{-1};
    int y{-1};
    gridindex(int x_, int y_) : x(x_), y(y_) {};
    gridindex() : x(-1), y(-1) {};
    void SetIndex(int x_, int y_)
    {
        x = x_;
        y = y_;
    }
    bool operator==(const gridindex& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const gridindex& other) const {
        return !(*this == other);
    }
} GridIndex;
typedef struct gridmap
{
    double measure_time_{0.0};
    int width_{OCC_WIEDTH};
    int height_{OCC_HEIGHT};
    double resolution_{0.4};
    double origin_x_{0.f};
    double origin_y_{0.f}; 
    uint8_t  data[OCC_HEIGHT][OCC_WIEDTH] ; // 1: free, 3: dynamic, 4: static  
    int      count[OCC_HEIGHT][OCC_WIEDTH] ; // 1: free, 3: dynamic, 4: static
    int       cluster_id_[OCC_HEIGHT][OCC_WIEDTH]; // 用于标识栅格所属的cluster 
    void Reset()
    {
        width_ = OCC_WIEDTH;
        height_ = OCC_HEIGHT;
        resolution_ = 0.4;
        origin_x_ = 0.f;
        origin_y_ = 0.f;
        for (int i = 0; i < OCC_HEIGHT; ++i) {
            for (int j = 0; j < OCC_WIEDTH; ++j) {
                data[i][j] = 1; // 初始化为free
                count[i][j] = 0; // 初始化计数为0
                cluster_id_[i][j] = -1; // 初始化cluster_id为-1
            }
        }
    }
} GridMap;
typedef struct grid_mapFrame
{
  Eigen::Isometry3d Twb;
  double timestamp;
  std::shared_ptr<GridMap> occ_mask_ptr; // 记录occupancy mask信息
  void Reset()
  {
    Twb = Eigen::Isometry3d::Identity();
    timestamp = 0.0;
    if(occ_mask_ptr != nullptr)
        occ_mask_ptr->Reset();
  }
} GridMapFrame;
// 车体坐标系 → 世界坐标系栅格坐标转换
class OccProcessor {
    
    private:
    std::vector<std::shared_ptr<BevLaneMarker>> edge_info_ptr_ ;//输出前对所有最近接地线实例（两两检验角度是否有重复 ）在做一次最近提取
    std::vector<std::vector<Eigen::Vector2d>>   occ_nearest_edge_;//近处边的实例集合 可能有重合
    std::vector<std::vector<Eigen::Vector2d>>   range_points_ = std::vector<std::vector<Eigen::Vector2d>>(720, std::vector<Eigen::Vector2d>());//360角度和角度上对应的点 车体坐标
    std::unordered_map<int,std::vector<Eigen::Vector2d>>   range_points_map_ ;
    std::list<std::pair<Eigen::Isometry3d,GridMapFrame>>   map_list_ ;
    const double EPSILON = 1e-3;
    std::shared_ptr<CrossDataManager> data_manager_ptr_{nullptr};
    Eigen::Isometry3d Twb_first;//上帧位姿
    Eigen::Isometry3d Twb_diff;//上帧位姿
    double  timestamp_ = 0.f;
    std::shared_ptr<GridMap> occ_mask_ptr_{nullptr}; // 每次的观测
    GridMap occ_grid_; // 最近点
    std::vector<GridIndex> valid_grid_index_; // 有效栅格
    GridMap last_occ_map_; // 图的范围内 观测的计数  带fix标志（满五帧）
    std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>> edges_;
    bool firstFlag_ = true;

    // std::shared_ptr<KdeRegression> kde_smoother_ptr_{nullptr};

    public:
     Eigen::Isometry3d Twb_current_;
    
    // 计算叉积 (P1P2 × P1P3)
    void Init(const std::shared_ptr<CrossDataManager> &data_manager_ptr) {
        firstFlag_ = true;
        data_manager_ptr_ = data_manager_ptr;
        if(occ_mask_ptr_ != nullptr)
            occ_mask_ptr_->Reset();
        Twb_first = Eigen::Isometry3d::Identity();
        timestamp_ = 0.f;
        last_occ_map_.Reset();

    }
    double CrossProduct(const Point& p1, const Point& p2, const Point& p3);
    void AccordingBresenhamGetGridInLine(GridIndex p0, GridIndex p1,std::vector<GridIndex>& grid_in_line,
                                            const GridMap &freespace_mask);
    GridIndex AccordingBresenhamOpt(GridIndex p0, GridIndex p1,
                                        const GridMap &freespace_mask);
    Eigen::Vector2d TransformPointVec(const Eigen::Vector2d &point,const Eigen::Isometry3d &rotate_translate_matrix);
    void GetObserveOccInfo(std::shared_ptr<CrossDataManager> &data_manager_ptr) ;
    void Process();
    void GetEdageInfoPtr( std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges) 
    {  
        edges = edges_ ;
        return;
    
    };
   const auto & GetOCCGrid(){return occ_grid_;}
   const auto &GetOCCMaskPtr(){return occ_mask_ptr_;}
   const auto & GetValidGridIndex(){return valid_grid_index_;}
   bool RaySegmentIntersection(const Eigen::Vector2d& p, double angle_degree,
                           const std::vector<Eigen::Vector2d>& segment,
                           Eigen::Vector2d& intersection);
    void ConvertFromRLE(
        const std::vector<cem::env_model::occ::RLEEntry>& rle_result,int width, int height,
        std::shared_ptr<GridMap> freespace_mask);
    std::shared_ptr<std::vector<cv::Point2f>> ResamplePoints(const std::vector<cv::Point2f>& points, bool along_x);
    void SplitLClusterAndResample(std::unordered_map<int, std::shared_ptr<std::vector<cv::Point2f>>> & map,
        std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges_resample);
    void SplitLCluster(
        std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges_resample);
    void SortClusterPts(std::unordered_map<int, std::shared_ptr<std::vector<cv::Point2f>>> & map,
        std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges_resample);
    const auto  &GetTwbDiff(){return Twb_diff;};
};
}//cem
}//fusion
#endif // __LANE_LINE_REFINER_H__