#include "kde_smoother.h"
#include <cmath>
#include <algorithm>
#include <cfloat> // FLT_MAX
#include <numeric> // std::accumulate
#include "lib/common/log_custom.h"

// 高斯核实现
double KDESmoother::gaussianKernel(double u, double v) const {
    return std::exp(-0.5 * (u*u + v*v)) / (2 * M_PI);
}

// Silverman带宽计算
void KDESmoother::computeBandwidth(const std::vector<cv::Point2f>& points) {
    if (points.empty()) return;
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(points, mean, stddev);
    double n = points.size();
    bandwidth_ = 1.06 * (stddev[0] + stddev[1]) / 2 * std::pow(n, -1.0/6.0);
}

// 局部密度计算（K近邻平均距离）
std::vector<double> KDESmoother::computeLocalDensity(const std::vector<cv::Point2f>& points) {
    std::vector<double> densities(points.size(), 0.0);
    for (size_t i = 0; i < points.size(); ++i) {
        std::vector<double> dists;
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;
            double dx = points[i].x - points[j].x;
            double dy = points[i].y - points[j].y;
            dists.push_back(std::sqrt(dx*dx + dy*dy));
        }
        std::nth_element(dists.begin(), dists.begin() + kNeighbors_, dists.end());//前kNeighbors_排序
        densities[i] = std::accumulate(dists.begin(), dists.begin() + kNeighbors_, 0.0) / kNeighbors_;
    }
    return densities;
}

// 密度→带宽映射（指数衰减模型）
double KDESmoother::densityToBandwidth(double density, double maxDensity) {
    double ratio = density / maxDensity;
    return hMin_ + (hMax_ - hMin_) / (1.0 + ratio);
}

// 构造函数
KDESmoother::KDESmoother(double bandwidth, int gridRes) 
    : bandwidth_(bandwidth), gridResolution_(gridRes), epsilon_(1e-9) {}

// 参数设置
void KDESmoother::setBandwidth(double h) { bandwidth_ = h; }
void KDESmoother::setGridResolution(int res) { gridResolution_ = res; }
void KDESmoother::enableAdaptiveBandwidth(bool enable, int k, double hMin, double hMax) {
    useAdaptive_ = enable;
    kNeighbors_ = k;
    hMin_ = hMin;
    hMax_ = hMax;
}

// 核心平滑算法
std::vector<cv::Point2f> KDESmoother::smooth(const std::vector<cv::Point2f>& inputPoints) {
    if (inputPoints.empty()) return {};
    
    // 自动计算基础带宽
    if (bandwidth_ <= epsilon_) computeBandwidth(inputPoints);
    
    // 计算点集边界
    float xMin = FLT_MAX, xMax = -FLT_MAX;
    float yMin = FLT_MAX, yMax = -FLT_MAX;
    for (const auto& pt : inputPoints) {
        xMin = std::min(xMin, pt.x);
        xMax = std::max(xMax, pt.x);
        yMin = std::min(yMin, pt.y);
        yMax = std::max(yMax, pt.y);
    }
    
    // 生成网格点
    std::vector<cv::Point2f> gridPoints;
    float dx = (xMax - xMin) / gridResolution_;
    float dy = (yMax - yMin) / gridResolution_;
    for (int i = 0; i <= gridResolution_; ++i) {
        for (int j = 0; j <= gridResolution_; ++j) {
            gridPoints.emplace_back(xMin + i * dx, yMin + j * dy);
            XLOG << "Grid Point: " << gridPoints.back().x << ", " << gridPoints.back().y;
        }
    }
    
    // 自适应带宽计算
    std::vector<double> bandwidths(gridPoints.size(), bandwidth_);
    if (useAdaptive_ && !inputPoints.empty()) {
        auto densities = computeLocalDensity(inputPoints);
        double maxDensity = *std::max_element(densities.begin(), densities.end());
        
        for (size_t i = 0; i < gridPoints.size(); ++i) {
            // 查找最近输入点
            double minDist = FLT_MAX;
            int nearestIdx = -1;
            for (size_t j = 0; j < inputPoints.size(); ++j) {
                double dx = gridPoints[i].x - inputPoints[j].x;
                double dy = gridPoints[i].y - inputPoints[j].y;
                double dist = dx*dx + dy*dy;
                if (dist < minDist) {
                    minDist = dist;
                    nearestIdx = j;
                }
            }
            bandwidths[i] = densityToBandwidth(densities[nearestIdx], maxDensity);
            XLOG << "Bandwidth: "  << bandwidths[i] << " for point: " << gridPoints[i].x << ", " << gridPoints[i].y;
        }
    }
    std::function kernel = [&](double u, double v, double h) {
        return std::exp(-0.5*(u*u + v*v)) / (2 * M_PI * h * h); // 正确归一化
    };
    // 密度计算（支持动态带宽）
    std::vector<double> densities(gridPoints.size(), 0.0);
    for (size_t i = 0; i < gridPoints.size(); ++i) {
        for (const auto& pt : inputPoints) {
            double u = (gridPoints[i].x - pt.x) / bandwidths[i];
            double v = (gridPoints[i].y - pt.y) / bandwidths[i];
             densities[i] += gaussianKernel(u, v);
        }
         densities[i] /= (inputPoints.size() * bandwidths[i] * bandwidths[i]);
    }
    
    // 密度加权采样
    std::vector<cv::Point2f> smoothedPoints;
    auto it = std::max_element(densities.begin(), densities.end());
    if (it == densities.end() || *it < 1e-6) {
        XLOG << "No significant density found, returning empty result.";
        return smoothedPoints; // 返回空结果
    }
    double maxDensity = *it;
    double threshold = 0.05 * maxDensity; // 可调参数
    for (size_t i = 0; i < gridPoints.size(); ++i) {
        if (densities[i] >threshold) {
            smoothedPoints.push_back(gridPoints[i]);
        }
    }
    return smoothedPoints;
}