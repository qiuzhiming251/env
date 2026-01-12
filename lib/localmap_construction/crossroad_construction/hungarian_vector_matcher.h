/**
 * @file 
 * @author xu.xuefeng1 (xu.xuefeng1@byd.com)
 * @brief
 * @date 2025-08-07
 * @copyright Copyright (c) 2025, BYD
 */
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <Eigen/Core>

class HungarianVectorMatcher {
public:
    // 计算点到直线的垂直距离
    static float CalculateVerticalDistance(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1, 
                                          const Eigen::Vector2f& query_point) {
        Eigen::Vector2f line_vec = p1 - p0;
        if (line_vec.norm() < 1e-6) return std::numeric_limits<float>::max();  // 避免除零错误
        
        Eigen::Vector2f point_vec = query_point - p0;
        float cross_product = std::abs(line_vec.x() * point_vec.y() - line_vec.y() * point_vec.x());
        return cross_product / line_vec.norm();
    }

    // 执行匈牙利匹配
    static std::vector<int> Solve(const std::vector<std::vector<float>>& cost_matrix) {
        int rows = cost_matrix.size();
        int cols = rows > 0 ? cost_matrix[0].size() : 0;
        if (rows == 0 || cols == 0) return {};

        // 初始化变量
        std::vector<float> u(rows + 1, 0);
        std::vector<float> v(cols + 1, 0);
        std::vector<int> p(cols + 1, 0);
        std::vector<int> way(cols + 1, 0);
        std::vector<int> assignment(rows, -1);

        for (int i = 1; i <= rows; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<float> minv(cols + 1, std::numeric_limits<float>::max());
            std::vector<bool> used(cols + 1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];
                float delta = std::numeric_limits<float>::max();
                int j1 = 0;

                // 寻找最小delta
                for (int j = 1; j <= cols; ++j) {
                    if (!used[j]) {
                        float cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                // 更新标杆
                for (int j = 0; j <= cols; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);

            // 回溯更新路径
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        // 构建分配结果
        for (int j = 1; j <= cols; ++j) {
            if (p[j] != 0) {
                assignment[p[j] - 1] = j - 1;
            }
        }
        return assignment;
    }
};