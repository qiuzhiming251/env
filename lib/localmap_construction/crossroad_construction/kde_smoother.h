#ifndef KD_SMOOTHER_H
#define KD_SMOOTHER_H

#include <vector>
#include <opencv2/core.hpp>  // 包含OpenCV核心模块

class KDESmoother {
private:
    double bandwidth_;         // 基础带宽参数
    double epsilon_;           // 容差值（防止除零）
    int gridResolution_;       // 网格分辨率
    bool useAdaptive_ = false; // 自适应带宽开关
    int kNeighbors_ = 5;      // 近邻数（用于密度计算）
    double hMin_ = 0.1;       // 最小带宽限制
    double hMax_ = 2.0;       // 最大带宽限制

    // 高斯核函数（二维）
    double gaussianKernel(double u, double v) const;

    // 计算Silverman带宽（二维数据）
    void computeBandwidth(const std::vector<cv::Point2f>& points);

    // 计算局部密度（用于自适应带宽）
    std::vector<double> computeLocalDensity(const std::vector<cv::Point2f>& points);

    // 密度到带宽的映射函数
    double densityToBandwidth(double density, double maxDensity);

public:
    // 构造函数
    KDESmoother(double bandwidth = 0.0, int gridRes = 100);
    
    // 参数设置接口
    void setBandwidth(double h);
    void setGridResolution(int res);
    void enableAdaptiveBandwidth(bool enable, int k = 5, double hMin = 0.1, double hMax = 2.0);

    // 核心平滑函数
    std::vector<cv::Point2f> smooth(const std::vector<cv::Point2f>& inputPoints);
};

#endif // KD_SMOOTHER_H