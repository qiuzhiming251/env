#ifndef _TWO_SEGMENT_BEZIER_INTERPOLATOR_H_
#define _TWO_SEGMENT_BEZIER_INTERPOLATOR_H_

#include <iostream>
#include <Eigen/Dense>
#include "modules/common/math/vec2d.h"
#include "common/utility.h"
#include <algorithm>
namespace cem {
namespace fusion {
namespace interpolator {
    
typedef byd::common::math::Vec2d Point;

// 三次贝塞尔曲线计算
Point bezierPoint(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t);

// 贝塞尔曲线一阶导数（切线）
Point bezierDerivative(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) ;

// 贝塞尔曲线二阶导数
Point bezierSecondDerivative(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) ;

// 计算曲率
double curvature(const Point& firstDeriv, const Point& secondDeriv);


// 检查曲线质量
struct CurveQuality {
    double maxCurvature;
    double minCurvature;
    double avgCurvature;
    int signChanges;  // 曲率符号变化次数
    bool hasSShape;   // 是否出现S型曲线
};
// 计算斜率（方向角）
// double calculateSlopeAngle(const Point& derivative) {
//     if (std::abs(derivative.x) < 1e-6 && std::abs(derivative.y) < 1e-6) 
//         return 0.0;
//     return std::atan2(derivative.y, derivative.x);
// }
CurveQuality evaluateCurveQuality(
    const Point& P0, const Point& P1, const Point& P2, const Point& P3,
    const Point& Q0, const Point& Q1, const Point& Q2, const Point& Q3,
    int samples = 20
) ;

// 基于曲率优化的平滑曲线生成
std::vector<Point> generateCurvatureOptimizedCurve(
    const std::vector<Point>& line1, 
    const std::vector<Point>& line2,
    double maxCurvatureThreshold = 0.18,  // 最大允许曲率
    double moveDistance = 5.0,           // 每次移动距离
    int maxIterations = 10                 // 最大优化迭代次数
);

std::vector<Point> generateSmoothCurve(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    int numSegments = 50             // 每段曲线的采样点数
) ;
std::vector<Point> GenerateStaticSmoothCurve(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    int numSegments = 50             // 每段曲线的采样点数
) ;
std::vector<Point> GenerateStaticSmoothCurveUturn(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    const std::vector<Point>& obstacles, // 障碍物点集
    int numSegments = 50             // 每段曲线的采样点数
) ;
std::vector<Point> GenerateUturnCircle(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    const std::vector<Point>& obstacles, // 障碍物点集
    int numSegments = 10             // 每段曲线的采样点数
) ;

class TrajectoryPlanner {
public:
    // 规划掉头轨迹
    static std::vector<Point> planUTurn(
        const Point& start, double startYaw,  // 起点和航向角(弧度)
        const Point& end, double endYaw,      // 终点和航向角
        const std::vector<Point>& obstacles,  // 障碍物点集
        double safetyMargin = 1.5) ;
private:
    // 计算安全偏移距离
    static double calculateSafeOffset(
        const Point& start, const Point& end,
        const Point& leftTurnDir,  // 逆时针方向向量
        const std::vector<Point>& obstacles,
        double safetyMargin);
    
    // 检查偏移是否安全
    static bool isOffsetSafe(
        const Point& start, const Point& end,
        const Point& leftTurnDir,
        const std::vector<Point>& obstacles,
        double safetyMargin, double offset);
    
    // 生成单段贝塞尔曲线
    static void generateBezierSegment(
        const Point& p0, const Point& p1,
        const Point& p2, const Point& p3,
        std::vector<Point>& output,
        int segments = 20);
    
    // 强制生成逆时针轨迹
    static std::vector<Point> forceCounterClockwise(
        const Point& start, double startYaw,
        const Point& end, double endYaw,
        const std::vector<Point>& obstacles,
        double safetyMargin);
    
};

}
} // namespace fusion
} // namespace cem

#endif
