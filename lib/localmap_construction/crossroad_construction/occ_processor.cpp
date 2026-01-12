/**
 * @file
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-07
 * @copyright Copyright (c) 2025, BYD
 */
// #include<opencv2/ximgproc.hpp>
#include "occ_processor.h"
#include <opencv2/opencv.hpp>
#include "point_dbscan.h"
#include "hptimer.h"
namespace cem {
namespace fusion{


    // 计算叉积 (P1P2 × P1P3)
double OccProcessor::CrossProduct(const Point& p1, const Point& p2, const Point& p3)  {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

void OccProcessor::AccordingBresenhamGetGridInLine(GridIndex p0, GridIndex p1,
                                                                      std::vector<GridIndex>& grid_in_line,const GridMap &freespace_mask)
{
    int16_t dx = p1.x - p0.x;
    int16_t dy = p1.y - p0.y;
    int16_t ux = dx > 0 ? 1 : -1;
    int16_t uy = dy > 0 ? 1 : -1;
    int16_t dx2 = abs(dx << 1);
    int16_t dy2 = abs(dy << 1);

    // 预计算路径长度并分配内存
    int estimated_size = std::max(abs(dx), abs(dy)) + 1;
    grid_in_line.resize(estimated_size);
    size_t idx = 0;

    if (abs(dx) > abs(dy)) {
        int16_t e = -dx;
        int16_t x = p0.x, y = p0.y;
        while (x != p1.x + ux) {
            grid_in_line[idx++] = {x, y}; // 直接赋值
            if(freespace_mask.data[(int)x][(int)y]== 8){
                XLOG << "occ_mask: " << x << " " << y;
            }
            e += dy2;
            int16_t mask = e >> 15;      // 无分支条件计算
            y += uy & ~mask;
            e -= dx2 & ~mask;
            x += ux;
        }
    } else {
        int16_t e = -dy;
        int16_t x = p0.x, y = p0.y;
        while (y != p1.y + uy) {
            grid_in_line[idx++] = {x, y};
            e += dx2;
            int16_t mask = e >> 15;
            x += ux & ~mask;
            e -= dy2 & ~mask;
            y += uy;
        }
    }

    // /* Ensure the endpoint p1 is included */
    // if (p1.x >= 0 && p1.y >= 0 && p1.x <1024 && p1.y < 448) {
    //     grid_in_line.emplace_back(p1);
    // }
}

GridIndex OccProcessor::AccordingBresenhamOpt(GridIndex p0, GridIndex p1, const GridMap& occ_mask) {
    // 1. 动态交换坐标轴（处理高斜率）
    bool steep = abs(p1.y - p0.y) > abs(p1.x - p0.x);
    if (steep) { std::swap(p0.x, p0.y); std::swap(p1.x, p1.y); }

    // 2. 无分支Bresenham算法 + 实时检测
    int16_t dx = abs(p1.x - p0.x), dy = abs(p1.y - p0.y);
    int16_t ux = p0.x < p1.x ? 1 : -1;
    int16_t uy = p0.y < p1.y ? 1 : -1;
    int16_t e = -dx, x = p0.x, y = p0.y;
    while (x != p1.x + ux) {
        GridIndex pt = steep ? GridIndex{y, x} : GridIndex{x, y};
#if 0
        auto x_ = 141.6f - (pt.x)*0.4 ;
        auto y_ = 76.8f - (pt.y)*0.4  ;
        if(data_manager_ptr_ &&  data_manager_ptr_->ppmImage)
            // data_manager_ptr_->ppmImage->DrawPointOther(x_, y_, 14);
#endif
        if (occ_mask.data[pt.x][pt.y] == 8 && occ_mask.count[pt.x][pt.y] > 1 ) {
#if 0

        if(data_manager_ptr_ &&  data_manager_ptr_->ppmImage)
            data_manager_ptr_->ppmImage->DrawPoint(x_, y_, 0);
#endif
            return  pt; // 返回原始坐标顺序
        }
        // 无分支误差更新
        e += 2 * dy;
        int16_t mask = e >> 15;
        y += uy & ~mask;
        e -= 2 * dx & ~mask;
        x += ux;
    }
    // XLOG << "########## No Obstacle Found in Line: " << p0.x << " " << p0.y << " to " << p1.x << " " << p1.y;
    return {-1, -1}; // 无障碍点
}

void OccProcessor::GetObserveOccInfo(std::shared_ptr<CrossDataManager> &data_manager_ptr)  {//单帧的range_points_

    data_manager_ptr_=data_manager_ptr;
    OCCInfoPtr    occInfo = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(occInfo);//查询measure timeamp 差距过大返回最后
    if(occInfo == nullptr) {
        XLOG << "!!!!!occInfo  NULL";
        return;
    };
    //1.提取实例
    auto timestamp = occInfo->header.timestamp;
    timestamp_ = timestamp;
    int temp_time_s = static_cast<int>(timestamp);
    int temp_time_us = static_cast<int>((timestamp-temp_time_s)*1000);
    // XLOG<< "\\\\\\\\OCC INFO TIME\\\\\\\\\\ : " << std::to_string( timestamp);
    // ppm::PPMImage ppm_image;
    // XLOG<< "\\\\\\\\\\\\\\\\\\ systime : " << std::to_string( GetMwTimeNowSec()) << " measuretime: " <<std::to_string( timestamp_);
    // ppm_image->DrawNumber(static_cast<int>(temp_time_us), 0.f,  48.0f - 8*2.f  , 2);
    // ppm_image->DrawNumber(static_cast<int>(temp_time_s), 0.f,  48.0f, 2);
    occ_mask_ptr_ = std::make_shared<GridMap>();
    occ_mask_ptr_->Reset();
    occ_mask_ptr_->measure_time_ = timestamp_;
    ConvertFromRLE(occInfo->occ_mask.rle_units, occInfo->occ_mask.width, occInfo->occ_mask.height,occ_mask_ptr_);

    // ppm_image->DrawPoint(0.f, 0.f, 0);
    // ppm_image->Save();
    return ;
}
void OccProcessor::Process()  {
    edges_.clear();//清空上次的边缘信息
    LocalizationPtr odom_t0_ptr = nullptr;
    SensorDataManager::Instance()->GetLatestSensorFrame(timestamp_,0.05,odom_t0_ptr);
    if(odom_t0_ptr == nullptr || data_manager_ptr_== nullptr || occ_mask_ptr_ == nullptr) {
        XLOG << "XXXXX no odom_t0_ptr";
        return ;
    }
    Eigen::Isometry3d Twb_current_ = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd Rwb_0((odom_t0_ptr->attitude_dr)* M_PI / 180.0,
                        Eigen::Vector3d(0, 0, 1));
    Eigen::Vector3d trans_0(odom_t0_ptr->posne_dr.at(0),
                            odom_t0_ptr->posne_dr.at(1),
                                                    0.0);
    Twb_current_.rotate(Rwb_0);
    Twb_current_.pretranslate(trans_0);
    //auto Twb_current_ = data_manager_ptr_->Twb();
    if(firstFlag_){//从进入路口
        firstFlag_ = false;
        Twb_first = Twb_current_;
        // last_occ_map_ = occ_mask_;
        // last_occ_map_.Twb = Twb_current_;
        Twb_diff = Eigen::Isometry3d::Identity();
        return ;
    }
    int temp_time_s = static_cast<int>(timestamp_);
    int temp_time_us = static_cast<int>((timestamp_-temp_time_s)*1000);
#if  OCC_DEBUG
    std::shared_ptr<ppm::PPMImage> ppm_image = data_manager_ptr_->ppmImage;
    // ppm_image->DrawNumber(static_cast<int>(temp_time_us), 0.f,  48.0f - 8*2.f  , 2);
    // ppm_image->DrawNumber(static_cast<int>(temp_time_s), 0.f,  48.0f, 2);
#endif
    auto Tdiff =  Twb_first.inverse()*Twb_current_;
    Twb_diff = Tdiff;
    GridMap current_frame;
    current_frame.Reset();//空白标签
   
    //全局转当前
    for (int i = 0; i < last_occ_map_.height_; i++) {
        for (int j = 0; j < last_occ_map_.width_; j++) {
            if (last_occ_map_.data[i][j] == 8) {
                auto x = 141.6f - i * 0.4;  
                auto y = 76.8f - j * 0.4;   
                Eigen::Vector3d pt(x, y, 0.0);     
                
                // 2. 应用相对变换 T_diff
                Eigen::Vector3d pt_world = Tdiff.inverse() * pt;  // 
                auto x_new = static_cast<int>(std::lround((141.6 - pt_world.x()) / 0.4));
                auto y_new = static_cast<int>(std::lround((76.8 - pt_world.y()) / 0.4));
                
#if  0
            ppm_image->DrawPoint(pt_world.x(), pt_world.y(),8);
            
#endif  
                if(x_new < 0 || y_new < 0 || x_new >= OCC_HEIGHT || y_new >= OCC_WIEDTH) {
                    continue; 
                }
                current_frame.data[x_new][y_new] = 8; // 
                current_frame.count[x_new][y_new] = last_occ_map_.count[i][j]; //
                
            }
        }
    }
    //观测存到全局
    for (int i = 0; i < occ_mask_ptr_->height_; i++) {
        for (int j = 0; j < occ_mask_ptr_->width_; j++) {
            auto x = 101.6f - i * 0.2;  
            auto y = 9.6f - j * 0.2; 
            Eigen::Vector3d pt(x, y, 0.0);     
            
            // 2. 应用相对变换 T_diff
            Eigen::Vector3d pt_world = Tdiff * pt;  // 
            auto x_new = static_cast<int>(std::lround((141.6 - pt_world.x()) /0.4));
            auto y_new = static_cast<int>(std::lround((76.8 - pt_world.y()) /0.4));
            if(x_new < 0 || y_new < 0 || x_new >= OCC_HEIGHT || y_new >= OCC_WIEDTH) {
                continue; 
            }

            if (occ_mask_ptr_->data[i][j] == 8 || occ_mask_ptr_->data[i][j] == 7) {
                last_occ_map_.data[x_new][y_new] = 8; // 
                last_occ_map_.count[x_new][y_new] += 1; // 
                
            }else if(last_occ_map_.data[x_new][y_new] == 8){
                last_occ_map_.count[x_new][y_new] -= 1; //
            }
        }
    }


    

    valid_grid_index_.clear();

    for (int i = 0; i < OCC_HEIGHT; i++) {
        for (int j = 0; j < OCC_WIEDTH; j++) {
            if (current_frame.data[i][j] == 8 && current_frame.count[i][j] > 1) {//同一地方观测大于3次
                auto x = 141.6f - i * 0.4; 
                if(x < -0.1) {
                    continue;
                }
#if 0
                auto y = 76.8f - j * 0.4;  
                ppm_image->DrawPoint(x,y,8);
                 
#endif
                valid_grid_index_.emplace_back(i, j);
            }
        }
    }
    if (valid_grid_index_.size() < 2) {//所有点混合
        XLOG << "No valid points found in the occupancy mask.";
        return;
    }

    // 对points进行DBSCAN聚类 聚类内部拟合
    {


        int eps = 5*2; // 10
        int minPts = 1;  // 最小点数，yi个点周围超过5m没有找到点 认为是噪声
        // GridDBScan(current_frame, valid_grid_index_, eps, minPts);
    }

    //总体在做一次扫描
    auto cross_center = data_manager_ptr_->GetSdSections().cross_point;
    int x_center = static_cast<int>(std::round((141.6 - cross_center.x()) /0.4));//在2倍大图中的位置
    int y_center = static_cast<int>(std::round((76.8 - cross_center.y()) /0.4));
    int ori_x = static_cast<int>(std::round((141.6 - 0) /0.4));//在2倍大图中的位置
    int ori_y = static_cast<int>(std::round((76.8 - 0) /0.4));
    if(x_center < 0 || y_center < 0 || x_center >= OCC_HEIGHT || y_center >= OCC_WIEDTH) {
        XLOG << "cross_center out of range: " << cross_center.x() << " " << cross_center.y();
        return;
    }
#if OCC_DEBUG
            ppm_image->DrawPoint(cross_center.x(), cross_center.y(),0); // 绘制扫描点
#endif
    occ_grid_.Reset();
    {
        for(int j = 0; j < OCC_WIEDTH; j++) {//上边
            auto grid_index = AccordingBresenhamOpt(GridIndex(x_center,y_center),GridIndex(0,j),current_frame);
            if(grid_index.x < 0 || grid_index.y < 0 || grid_index.x >= OCC_HEIGHT || grid_index.y >= OCC_WIEDTH) {
                continue;
            }
            occ_grid_.data[grid_index.x][grid_index.y] = 8; // 设置为占用栅格
            occ_grid_.cluster_id_[grid_index.x][grid_index.y] = 1; // 设置为占用栅格

        }




        for(int i = 0; i < OCC_HEIGHT; i++) {//右侧边
            auto grid_index = AccordingBresenhamOpt(GridIndex(x_center,y_center),GridIndex(i,OCC_WIEDTH),current_frame);
            if(grid_index.x < 0 || grid_index.y < 0 || grid_index.x >= OCC_HEIGHT || grid_index.y >= OCC_WIEDTH) {
                continue;
            }
            occ_grid_.data[grid_index.x][grid_index.y] = 8; // 设置为占用栅格
            occ_grid_.cluster_id_[grid_index.x][grid_index.y] = 2; // 设置为占用栅格
        }
        for(int i = 0; i < OCC_HEIGHT; i++) {//左边
            auto grid_index = AccordingBresenhamOpt(GridIndex(x_center,y_center),GridIndex(i,0),current_frame);
            if(grid_index.x < 0 || grid_index.y < 0 || grid_index.x >= OCC_HEIGHT || grid_index.y >= OCC_WIEDTH) {
                continue;
            }
            occ_grid_.data[grid_index.x][grid_index.y] = 8; // 设置为占用栅格
            occ_grid_.cluster_id_[grid_index.x][grid_index.y] = 3; // 设置为占用栅格
            
             grid_index = AccordingBresenhamOpt(GridIndex(ori_x,ori_y),GridIndex(i,0),current_frame);
            if(grid_index.x < 0 || grid_index.y < 0 || grid_index.x >= OCC_HEIGHT || grid_index.y >= OCC_WIEDTH) {
                continue;
            }
            occ_grid_.data[grid_index.x][grid_index.y] = 8; // 设置为占用栅格
            occ_grid_.cluster_id_[grid_index.x][grid_index.y] = 3; // 设置为占用栅格

        }
        for(int j = 0; j < OCC_WIEDTH; j++) {//下边
            auto grid_index = AccordingBresenhamOpt(GridIndex(x_center,y_center),GridIndex(OCC_HEIGHT,j),current_frame);//最后一行
            if(grid_index.x < 0 || grid_index.y < 0 || grid_index.x >= OCC_HEIGHT || grid_index.y >= OCC_WIEDTH) {
                continue;
            }
            occ_grid_.data[grid_index.x][grid_index.y] = 8; // 设置为占用栅格 重复自动去除
            occ_grid_.cluster_id_[grid_index.x][grid_index.y] = 4; // 设置为占用栅格

        }
    }
    //一个实例一个点集
    unordered_map<int, std::shared_ptr<std::vector<cv::Point2f>>> id_pts_map;

    for (int i = 0; i < OCC_HEIGHT; i++) {
        for (int j = 0; j < OCC_WIEDTH; j++) {
            if (occ_grid_.data[i][j] == 8) {
                auto &cluster_id = occ_grid_.cluster_id_[i][j];
                if(cluster_id < 0){
                    continue;
                }

                // 确保每个cluster_id都有有效的vector
                if (id_pts_map.find(cluster_id) == id_pts_map.end()) {
                    // 首次发现该聚类ID，创建新的点集容器
                    id_pts_map[cluster_id] = std::make_shared<std::vector<cv::Point2f>>();
                }

                // 转换并添加坐标点 (从图片信息理解坐标转换)
                float x = 141.6 - i * 0.4;  // 左侧坐标值
                float y = 76.8 - j * 0.4;   // 底部坐标值
                auto dis_y =  (y > 0.f ? y : -y);
                if(std::abs(dis_y - 9.6) > 1.5f) {
                    id_pts_map[cluster_id]->emplace_back(x, y);
                }

            }
        }
    }
    // 排序
    SortClusterPts(id_pts_map,edges_);
    
    for (const auto& edge : id_pts_map) {
        if(edge.second == nullptr || edge.second->size()==0) continue;
        edges_.emplace_back(edge.first, edge.second);
    }

   // L形路沿分开
//    SplitLCluster(edges_);
    for (const auto& edge : edges_) {
        int cluster_id = edge.first;
        const auto& points_ptr = edge.second;
        if(points_ptr == nullptr || points_ptr->size()==0) continue;
#if OCC_DEBUG
        for (const auto& pt : *points_ptr) {
            ppm_image->DrawPoint(pt.x, pt.y, cluster_id%14); // 绘制聚类点

        }
        ppm_image->DrawNumber(cluster_id, points_ptr->front().x, points_ptr->front().y,0);
#endif
    }


#if OCC_DEBUG
    ppm_image->DrawPoint(0.f, 0.f, 1); // 绘制原点
    // ppm_image->Save();
#endif

    return ;
}
bool OccProcessor::RaySegmentIntersection(const Eigen::Vector2d& p, double angle_degree,
                           const std::vector<Eigen::Vector2d>& segment,
                           Eigen::Vector2d& intersection) {
    if (segment.size() != 2) return false; // 确保线段有两个端点

    // 1. 计算射线方向向量
    double theta = angle_degree * M_PI / 180.0;
    Eigen::Vector2d dir(cos(theta), sin(theta));

    const Eigen::Vector2d& s0 = segment[0];
    const Eigen::Vector2d& s1 = segment[1];
    Eigen::Vector2d segVec = s1 - s0; // 线段方向向量

    // 2. 构造线性方程组：A * [t, u]^T = b
    double A = dir.x();
    double B = -segVec.x();
    double C = dir.y();
    double D = -segVec.y();
    double det = A * D - B * C; // 系数矩阵行列式

    // 3. 处理非平行情况（唯一交点）
    if (std::abs(det) > EPSILON) {
        Eigen::Vector2d b = s0 - p;
        double t = (D * b.x() - B * b.y()) / det;
        double u = (A * b.y() - C * b.x()) / det;

        // 验证交点有效性
        if (t >= 0 && u >= 0 && u <= 1) {
            intersection = p + t * dir;
            return true;
        }
        return false;
    }

    // 4. 处理平行或共线情况
    Eigen::Vector2d s0p = s0 - p;
    double cross = s0p.x() * dir.y() - s0p.y() * dir.x();

    // 检查是否真正共线（叉积接近0）
    if (std::abs(cross) > EPSILON) return false;

    // 5. 检查点p是否在线段上
    double segLength2 = segVec.squaredNorm();
    if (segLength2 < EPSILON) { // 线段退化为点
        if ((s0 - p).squaredNorm() < EPSILON) return false;
    } else {
        double u = s0p.dot(segVec) / segLength2;
        if (u >= 0 && u <= 1) return false; // p在线段上
    }

    // 6. 寻找最近有效端点
    double dot0 = s0p.dot(dir);
    double dot1 = (s1 - p).dot(dir);
    Eigen::Vector2d candidate;
    bool found = false;

    // 选择正方向上最近的端点
    if (dot0 >= 0 && dot1 >= 0) {
        candidate = (dot0 < dot1) ? s0 : s1;
        found = true;
    } else if (dot0 >= 0) {
        candidate = s0;
        found = true;
    } else if (dot1 >= 0) {
        candidate = s1;
        found = true;
    }

    if (found) {
        intersection = candidate;
        return true;
    }
    return false;
}
  Eigen::Vector2d OccProcessor::TransformPointVec(const Eigen::Vector2d &point,
    const Eigen::Isometry3d &rotate_translate_matrix) {
    Eigen::Vector2d point_out;
    Eigen::Vector3d point_before_dr(point.x(), point.y(), 0);
    Eigen::Vector3d point_after_dr = rotate_translate_matrix * point_before_dr;
    point_out[0] = point_after_dr[0];
    point_out[1] = point_after_dr[1];
    return point_out;
}

void OccProcessor::ConvertFromRLE(
    const std::vector<cem::env_model::occ::RLEEntry>& rle_result,int width, int height,
    std::shared_ptr<GridMap> freespace_mask)
{
    if (rle_result.empty()) {
        XLOG << "OccProcessor::ConvertFromRLE rle_result is empty";
        return;
    }
    // 计算解码后总长度
    size_t total_length = 0;
    for (const auto& entry : rle_result) {
        total_length += entry.count;  // 累加每个条目的计数
    }
    if(total_length !=width*height ){
        XLOG << "OccProcessor::ConvertFromRLE total_length  width*height " <<  total_length << " " << width*height << " " << rle_result.size();;
        return;
    }
    freespace_mask->width_ = width;
    freespace_mask->height_ = height;
    int current_index = 0; // 当前索引位置
    for (const auto& [value, count] : rle_result) {
        for (size_t i = 0; i < count; ++i) {
            freespace_mask->data[current_index/width][current_index%width] = value;

            current_index++;
        }
    }
}

void OccProcessor::SplitLClusterAndResample(std::unordered_map<int, std::shared_ptr<std::vector<cv::Point2f>>> & map,
                                std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges_resample){
    for(auto& edge : map){
        auto& cluster_points = *edge.second;
        if (cluster_points.size() < 2) continue; // 小于2个点的聚类不处理

        // 计算聚类在X和Y方向的范围
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;

        for (const auto& pt : cluster_points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        // 判断是否为路口路沿轮廓（X和Y跨度相近）
        const float ASPECT_RATIO_THRESH = 0.5f; // 阈值可调整
        const float MIN_SPAN = 5.0f; // 最小跨度

        float x_span = max_x - min_x;
        float y_span = max_y - min_y;
        float aspect_ratio = std::min(x_span, y_span) / std::max(x_span, y_span);

        // 检查是否满足路口路沿轮廓条件
        bool is_curb_contour = (aspect_ratio > ASPECT_RATIO_THRESH) &&
                              (x_span > MIN_SPAN) &&
                              (y_span > MIN_SPAN);

        if (is_curb_contour) {
            // === 步骤1：拆分点集 ===
            float center_x = (min_x + max_x) * 0.5f;
            float center_y = (min_y + max_y) * 0.5f;

            std::vector<cv::Point2f> cluster1_points; // 水平方向点集（点集1）
            std::vector<cv::Point2f> cluster2_points; // 垂直方向点集（点集2）

            // 计算阈值（范围差值的百分比）
            float x_threshold = (max_x - min_x) * 0.1f;
            float y_threshold = (max_y - min_y) * 0.1f;

            // 每个点 x y 方向距离中心点的距离
            for (const auto& pt : cluster_points) {
                float dx = std::abs(pt.x - center_x);
                float dy = std::abs(pt.y - center_y);

                if (dx < x_threshold) {
                    // X方向接近中心 - 归为垂直方向点集（点集2）
                    cluster2_points.push_back(pt);
                }
                if (dy < y_threshold) {
                    // Y方向接近中心 - 归为水平方向点集（点集1）
                    cluster1_points.push_back(pt);
                }
            }

            // === 步骤2：直接重采样生成有序点集 ===
            // 处理点集1（水平方向）
            XLOG << "OccProcessor::SplitLClusterAndResample ID " << edge.first << " :" << cluster1_points.size() << " " << cluster2_points.size();
            if (cluster1_points.size() >= 2) {
                // 直接重采样会生成沿X方向排序的点集
                auto resampled_ptr = ResamplePoints(cluster1_points, true);
                edges_resample.emplace_back(edge.first, resampled_ptr);
            }

            // 处理点集2（垂直方向）
            if (cluster2_points.size() >= 2) {
                // 直接重采样会生成沿Y方向排序的点集
                auto resampled_ptr = ResamplePoints(cluster2_points, false);
                edges_resample.emplace_back(edge.first + 50, resampled_ptr);
            }
        } else {
            // === 处理非路口路沿的普通聚类 ===
            // 判断聚类主方向 (X或Y)
            bool primary_x = (x_span) > (y_span);

            // 直接重采样生成有序点集
            // auto resampled_ptr = ResamplePoints(cluster_points, primary_x);
   
            // auto result = kde_smoother_ptr_->smooth(cluster_points);

            // auto resampled_ptr = ResamplePoints(cluster_points, primary_x);
            // edges_resample.emplace_back(edge.first, std::make_shared<std::vector<cv::Point2f>>(result.begin(), result.end()));
        }


    }
}
void OccProcessor::SortClusterPts(std::unordered_map<int, std::shared_ptr<std::vector<cv::Point2f>>> & map,
                                std::vector<std::pair<int ,std::shared_ptr<std::vector<cv::Point2f>>>>& edges_resample){
    for(auto& edge : map){
        auto& cluster_points = *edge.second;
        if (cluster_points.size() < 2) continue; // 小于2个点的聚类不处理

        // 计算聚类在X和Y方向的范围
        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;

        for (const auto& pt : cluster_points) {
            min_x = std::min(min_x, pt.x);
            max_x = std::max(max_x, pt.x);
            min_y = std::min(min_y, pt.y);
            max_y = std::max(max_y, pt.y);
        }

        // 判断是否为路口路沿轮廓（X和Y跨度相近）
        const float ASPECT_RATIO_THRESH = 0.5f; // 阈值可调整
        const float MIN_SPAN = 5.0f; // 最小跨度

        float x_span = max_x - min_x;
        float y_span = max_y - min_y;
        float aspect_ratio = std::min(x_span, y_span) / std::max(x_span, y_span);

       
            bool primary_x = (x_span) > (y_span);
            std::sort(cluster_points.begin(), cluster_points.end(), [primary_x](const cv::Point2f& a, const cv::Point2f& b) {
                if(primary_x) {
                    if(std::abs(a.x - b.x) < 1e-5){
                        return false;
                    }
                    return a.x < b.x;
                }else {
                    if(std::abs(a.y - b.y) < 1e-5){
                        return false;
                    }
                    return a.y < b.y;
                }
                
            });

            // 直接重采样生成有序点集
            // auto result = kde_smoother_ptr_->smooth(cluster_points);
            // edges_resample.emplace_back(edge.first, std::make_shared<std::vector<cv::Point2f>>(result.begin(), result.end()));

    }
    return;
}

using cv::Point2f;

struct PCAResult {
    Eigen::Vector2f primary_dir;
    Eigen::Vector2f secondary_dir;
    cv::Point2f centroid;
};

// 计算点集的主成分
PCAResult ComputePCA(const std::vector<cv::Point2f>& points) {
    if (points.empty()) return {};

    // 1. 计算质心
    cv::Point2f centroid(0, 0);
    for (const auto& p : points) {
        centroid += p;
    }
    centroid /= static_cast<float>(points.size());
    
    // 2. 构建协方差矩阵
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto& p : points) {
        float dx = p.x - centroid.x;
        float dy = p.y - centroid.y;
        covariance(0, 0) += dx * dx;
        covariance(0, 1) += dx * dy;
        covariance(1, 0) += dy * dx;
        covariance(1, 1) += dy * dy;
    }
    covariance /= (points.size() - 1);
    
    // 3. 特征分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    auto eigenvalues = solver.eigenvalues();
    auto eigenvectors = solver.eigenvectors();
    
    // 4. 获取主方向和次方向
    int primary_idx = (eigenvalues[0] > eigenvalues[1]) ? 0 : 1;
    int secondary_idx = 1 - primary_idx;
    
    return {
        eigenvectors.col(primary_idx).normalized(),
        eigenvectors.col(secondary_idx).normalized(),
        centroid
    };
}
std::shared_ptr<std::vector<cv::Point2f>> OccProcessor::ResamplePoints(
    const std::vector<cv::Point2f>& points,
    bool along_x
) {
    if (points.size() < 2) {
        return std::make_shared<std::vector<cv::Point2f>>(points);
    }

    // 获取坐标范围
    float min_val = FLT_MAX, max_val = -FLT_MAX;
    for (const auto& pt : points) {
        float val = along_x ? pt.x : pt.y;
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
    }

    // 多项式拟合（二次）
    Eigen::VectorXd coeffs = polyfit(points, along_x, 2);

    // 重采样
    const float RESAMPLE_STEP = 1.f; // 重采样步长（米）
    int num_samples = (max_val - min_val) / RESAMPLE_STEP;
    if (num_samples < 1) num_samples = 1;

    auto resampled_ptr = std::make_shared<std::vector<cv::Point2f>>();
    for (int i = 0; i <= num_samples; i++) {
        float val = min_val + i * (max_val - min_val) / num_samples;

        // 使用拟合曲线计算坐标
        double poly_val = coeffs[0] + coeffs[1] * val + coeffs[2] * val * val;

        if (along_x) {
            resampled_ptr->emplace_back(val, static_cast<float>(poly_val));
        } else {
            resampled_ptr->emplace_back(static_cast<float>(poly_val), val);
        }
    }

    return resampled_ptr;
}

void OccProcessor::SplitLCluster(std::vector<std::pair<int, std::shared_ptr<std::vector<cv::Point2f>>>>& edges) {
        
    for (auto& [cluster_id, points_ptr] : edges) {
        if(!points_ptr) continue;
        auto& cluster_points = *points_ptr;
        if (cluster_points.size() < 8) continue; // 小于10个点跳过处理
        
        // 1. 计算PCA主方向
        auto pca = ComputePCA(cluster_points);
        
        // 2. 判断是否为弧形分布
        float aspect_ratio = std::min(pca.primary_dir.norm(), pca.secondary_dir.norm()) /
                             std::max(pca.primary_dir.norm(), pca.secondary_dir.norm());
        const bool is_Lcurved = aspect_ratio > 0.3f; // 纵横比接近表示弧形分布

        if (is_Lcurved) {
            // === 弧形点集分割 - 使用PCA方向分割 ===
            // 3. 沿次方向投影并排序
            vector<float> curvatures;
            for (int i = 1; i < cluster_points.size() - 1; i++) {
                // 计算相邻点向量夹角
                auto v1 = cluster_points[i] - cluster_points[i-1];
                auto v2 = cluster_points[i+1] - cluster_points[i];
                float angle = std::acos(v1.dot(v2) / (cv::norm(v1) * cv::norm(v2)));
                curvatures.push_back(angle);
            }
            auto max_it = std::max_element(curvatures.begin(), curvatures.end());
            if(max_it == curvatures.end() ){
                return;
            }
            auto split_idx = std::distance(curvatures.begin(), max_it);
            XLOG << "========== SPLIT " <<  split_idx ;
            std::vector<cv::Point2f> cluster1, cluster2;
            for (size_t i = 0; i < cluster_points.size(); ++i) {
                if (i < split_idx) {
                    cluster1.push_back(cluster_points[i]);
                } else {
                    cluster2.push_back(cluster_points[i]);
                }
            }
            //删除原来的edges中的cluster_id
            edges.erase(std::remove_if(edges.begin(), edges.end(), [&](const auto& edge) {
                return edge.first == cluster_id;
            }), edges.end());
            // 6. 分别重采样并存储
            auto processSegment = [&](int base_id, const std::vector<cv::Point2f>& segment, const Eigen::Vector2f& dir) 
                    -> std::pair<int, std::shared_ptr<std::vector<cv::Point2f>>> 
                {
                    if (segment.size() < 2) return {base_id, nullptr};
                    bool sort_by_x = std::abs(dir.x()) > std::abs(dir.y());
                    auto resampled = ResamplePoints(segment, sort_by_x);
                    // 修复点2：正确构造pair和shared_ptr
                    return {base_id, resampled};
                };
                auto edge1 =std::make_pair(cluster_id, std::make_shared<std::vector<cv::Point2f>>(cluster1));
                auto edge2 =std::make_pair(cluster_id + 50, std::make_shared<std::vector<cv::Point2f>>(cluster2));

                edges.push_back(edge1);
                edges.push_back(edge2);

            return;
        } else {
            return;
        }
    }
}
}//namespace fusion
}//namespace cem