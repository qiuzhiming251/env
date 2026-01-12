/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-07-015
 * @copyright Copyright (c) 2025, BYD
 */
#include <queue>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp> // 引入FLANN库实现KD树
#include <Eigen/Dense> // 引入Eigen库进行多项式拟合
#include "hptimer.h"
#include "lib/common/log_custom.h"
#include "occ_processor.h"

// KD树加速邻域查询（避免暴力搜索O(n²)）
namespace cem {
namespace fusion {
#include <queue>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>

std::vector<int> regionQueryKDTree(
    cv::flann::Index& kdTree, 
    const cv::Point2f& point, 
    float eps, 
    const std::vector<cv::Point2f>& points
) {
    float query_data[2] = { point.x, point.y };
    cv::Mat queryMat(1, 2, CV_32F, query_data);
    queryMat = queryMat.clone();
    
    // 验证查询矩阵有效性
    if (queryMat.empty() || queryMat.rows != 1) {
        return {};
    }

    std::vector<int> indices;
    std::vector<float> dists;
    const int max_neighbors = 256; // 限制最大邻居数量
    
    kdTree.radiusSearch(
        queryMat, 
        indices, 
        dists, 
        eps,
        max_neighbors,                      // 防止内存溢出
        cv::flann::SearchParams(32, 0, true) // 精度+结果排序
    );

    std::vector<int> valid;
    for (int idx : indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < points.size()) {
            valid.push_back(idx);
        }
    }
    return valid;
}

void dbscan_optimized(
    const std::vector<cv::Point2f>& points, 
    float eps, 
    int minPts, 
    std::vector<int>& labels
) {
    const int UNCLASSIFIED = -1;
    const int NOISE = -2;
    int cluster_id = 0;
    size_t n = points.size();
    if (n == 0) {
        labels.clear();
        return;
    }
    HPTimer timer;
    labels.assign(n, UNCLASSIFIED);
    
    // 构建KD树（修正距离类型）
    std::vector<float> pointsData;
    pointsData.reserve(n * 2);
    for (const auto& p : points) {
        pointsData.emplace_back(std::move(p.x));
        pointsData.emplace_back(std::move(p.y));
        XLOG << ":Point: " << p.x << ", " << p.y;
    }
    cv::Mat pointsMat(n, 2, CV_32F, pointsData.data()); // 零拷贝
    cv::flann::Index kdTree(
        pointsMat, 
        cv::flann::KDTreeIndexParams(1)  // 
    );
    
    std::vector<bool> visited(n, false);
    std::vector<int> seed_stack;
    seed_stack.reserve(100); // 预分配栈内存
    XLOG << "KDTreeIndexParams : " << timer.ElapsedMilliSeconds() << " ms";

    for (size_t i = 0; i < n; ++i) {
        if (labels[i] != UNCLASSIFIED) continue;

        auto neighbors = regionQueryKDTree(kdTree, points[i], eps, points);
        if (neighbors.size() < static_cast<size_t>(minPts)) {
            labels[i] = NOISE;
            continue;
        }

        labels[i] = cluster_id;
        visited[i] = true;
        seed_stack = neighbors;

        XLOG << "regionQueryKDTree LOOP TIME: " << timer.ElapsedMilliSeconds() << " ms"  << "stack size: " << seed_stack.size();

        // 修改DBSCAN主循环中的扩展逻辑
        while (!seed_stack.empty()) {
            int idx = seed_stack.back();
            seed_stack.pop_back();//出栈 每次从头弹出
            
            if (visited[idx]) continue; 
            visited[idx] = true; // 立即标记已访问

            // 所有点都尝试标记（包括边界点）
            if (labels[idx] == UNCLASSIFIED || labels[idx] == NOISE) {
                labels[idx] = cluster_id;
            }

            // 继续扩展
            auto neighbors2 = regionQueryKDTree(kdTree, points[idx], eps, points);
            // if (neighbors2.size() < static_cast<size_t>(minPts)) continue;

            for (int n_idx : neighbors2) {
                // 优先连接未分类点
                if (!visited[n_idx] && labels[n_idx] != cluster_id) {
                    seed_stack.push_back(n_idx);//压栈
                    // 提前标记避免重复处理
                    if (labels[n_idx] == UNCLASSIFIED) {
                        labels[n_idx] = cluster_id;//同样类别
                    }
                }
            }
        }
        ++cluster_id;
        XLOG << "stack LOOP TIME: " << timer.ElapsedMilliSeconds() << " ms";

    }
}
// void GridDBScan(GridMap& grid,const std::vector<GridIndex>& validPoints, int  eps, int minPts){
    
//     struct GridIndexHash {
//         size_t operator()(const GridIndex& p) const {
//             return static_cast<size_t>(p.x) * 131071 + p.y;
//         }
//     };
//     int cluster_id = 1;
//     std::unordered_set<GridIndex, GridIndexHash> visitedSet;
//     std::unordered_set<GridIndex, GridIndexHash> validSet(validPoints.begin(), validPoints.end());
//        // 3. 预处理偏移量模板（提升性能）
//     std::vector<std::pair<int, int>> offsets;
//     for (int dy = -eps; dy <= eps; ++dy) {
//         for (int dx = -eps; dx <= eps; ++dx) {
//             if (dx == 0 && dy == 0) continue;  // 跳过自身
//             offsets.emplace_back(dx, dy);
//         }
//     }
//     // 3. 遍历所有有效栅格
//     for (const auto& pt : validPoints) {
//         if (visitedSet.find(pt) != visitedSet.end()) continue;
//         visitedSet.insert(pt);
//         // 4. 内联实现邻域查询（替代getNeighbors函数）
//         std::vector<GridIndex> neighbors;
//         for (const auto& [dx, dy] : offsets) {
//             GridIndex nb(pt.x + dx, pt.y + dy);
//             // 边界检查 & 有效性验证
//             if (nb.x >= 0 && nb.x < grid.height_ && 
//                 nb.y >= 0 && nb.y < grid.width_ &&
//                 grid.data[nb.x][nb.y] == 8 &&      // 栅格值需为有效点
//                 validSet.find(nb) != validSet.end()) {
//                 neighbors.push_back(nb);
//             }
//         }
        
//         // 5. 自己点以外邻域点小于1 
//         if (neighbors.size() < static_cast<size_t>(minPts)) { 
//             grid.cluster_id_[pt.x][pt.y] = -1; // 标记为噪声点
//             grid.data[pt.x][pt.y] = 1; // 恢复为free栅格
//             continue;
//         }

//         // 6. 给当前格子分配标签 并且查询其邻域扩展分配同样标签
//         grid.cluster_id_[pt.x][pt.y] = cluster_id;
//         std::stack<GridIndex> seedStack;
//         for (const auto& nb : neighbors) {
//             if (visitedSet.find(nb) == visitedSet.end()) { // 邻域点未访问过进栈
//                 seedStack.push(nb);
//             }
//         }

//         // 7. 簇扩展（BFS）
//         while (!seedStack.empty()) {
//             GridIndex cur = seedStack.top();
//             seedStack.pop();
            
//             // 定位当前点在validPoints中的索引
//             if (visitedSet.find(cur) != visitedSet.end()){
//                 continue;
//             } 
//             grid.cluster_id_[cur.x][cur.y] = cluster_id;
//             visitedSet.insert(cur);
                

//             // 8. 内联二次邻域查询
//             std::vector<GridIndex> cur_neighbors;
//             for (const auto& [dx, dy] : offsets) {
//                 GridIndex curNb(cur.x + dx, cur.y + dy);
//                 if (curNb.x >= 0 && curNb.x < grid.height_ && 
//                     curNb.y >= 0 && curNb.y < grid.width_ &&
//                     grid.data[curNb.x][curNb.y] == 8 &&
//                     validSet.find(curNb) != validSet.end()) {
//                     cur_neighbors.push_back(curNb);
//                 }
//             }

//             // 9. 核心点判断与扩展
//             if (cur_neighbors.size()  >= static_cast<size_t>(minPts)) {
//                 for (const auto& nb : cur_neighbors) {
//                     if (visitedSet.find(nb) == visitedSet.end()) {
//                         seedStack.push(nb);
//                     }
//                 }
//             }
//         }
        
//         ++cluster_id; // 完成一个簇
//     }
//     return;
// }
void GridDBScan(GridMap& grid, const std::vector<GridIndex>& validPoints, int eps, int minPts) {
    int cluster_id = 1;
    // 用二维向量替代 unordered_set，初始为 false
    std::vector<std::vector<bool>> visited(grid.height_, std::vector<bool>(grid.width_, false));
    std::vector<std::vector<bool>> valid(grid.height_, std::vector<bool>(grid.width_, false));
    
    // 标记有效点（初始化时遍历一次）
    for (const auto& pt : validPoints) {
        valid[pt.x][pt.y] = true;
    }

    // 预计算偏移量模板（保持原有）
    std::vector<std::pair<int, int>> offsets;
    for (int dy = -eps; dy <= eps; ++dy) {
        for (int dx = -eps; dx <= eps; ++dx) {
            if (dx == 0 && dy == 0) continue;
            offsets.emplace_back(dx, dy);
        }
    }

    for (const auto& pt : validPoints) {
        if (visited[pt.x][pt.y]) continue;  // 直接数组访问，无需哈希查找
        visited[pt.x][pt.y] = true;

        // 邻域查询
        std::vector<GridIndex> neighbors;
        for (const auto& [dx, dy] : offsets) {
            GridIndex nb(pt.x + dx, pt.y + dy);
            if (nb.x >= 0 && nb.x < grid.height_ && 
                nb.y >= 0 && nb.y < grid.width_ &&
                grid.data[nb.x][nb.y] == 8 && 
                valid[nb.x][nb.y]) {          // 直接数组检查有效性
                neighbors.push_back(nb);
            }
        }

        if (neighbors.size() < static_cast<size_t>(minPts)) {
            grid.cluster_id_[pt.x][pt.y] = -1;
            grid.data[pt.x][pt.y] = 1;
            continue;
        }

        grid.cluster_id_[pt.x][pt.y] = cluster_id;
        std::stack<GridIndex> seedStack;
        for (const auto& nb : neighbors) {
            if (!visited[nb.x][nb.y]) {   // 直接数组检查
                seedStack.push(nb);
            }
        }

        while (!seedStack.empty()) {
            GridIndex cur = seedStack.top();
            seedStack.pop();
            if (visited[cur.x][cur.y]) continue;  // 数组检查
            grid.cluster_id_[cur.x][cur.y] = cluster_id;
            visited[cur.x][cur.y] = true;

            std::vector<GridIndex> cur_neighbors;
            for (const auto& [dx, dy] : offsets) {
                GridIndex curNb(cur.x + dx, cur.y + dy);
                if (curNb.x >= 0 && curNb.x < grid.height_ && 
                    curNb.y >= 0 && curNb.y < grid.width_ &&
                    grid.data[curNb.x][curNb.y] == 8 &&
                    valid[curNb.x][curNb.y]) {
                    cur_neighbors.push_back(curNb);
                }
            }

            if (cur_neighbors.size() >= static_cast<size_t>(minPts)) {
                for (const auto& nb : cur_neighbors) {
                    if (!visited[nb.x][nb.y]) {
                        seedStack.push(nb);
                    }
                }
            }
        }
        ++cluster_id;
    }
}
Eigen::VectorXd polyfit(
    const std::vector<cv::Point2f>& points, 
    bool along_x, 
    int order
) {
    int n = points.size();
    Eigen::MatrixXd A(n, order + 1);
    Eigen::VectorXd b(n);
    
    for (int i = 0; i < n; i++) {
        float val = along_x ? points[i].x : points[i].y;
        float target = along_x ? points[i].y : points[i].x;
        
        // 多项式基
        for (int j = 0; j <= order; j++) {
            A(i, j) = std::pow(val, j);
        }
        b(i) = target;
    }
    
    // 最小二乘法求解
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
}
}//namespace fusion
}//namespace cem