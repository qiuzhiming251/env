

#include "two_segment_bezier_interpolator.h"
namespace cem {
namespace fusion {
namespace interpolator {
    
typedef byd::common::math::Vec2d Point;

// 三次贝塞尔曲线计算
Point bezierPoint(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) {
    double u = 1 - t;
    double u2 = u * u;
    double t2 = t * t;
    double bu0 = u2 * u;
    double bu1 = 3 * u2 * t;
    double bu2 = 3 * u * t2;
    double bu3 = t2 * t;
    return bu0 * p0 + bu1 * p1 + bu2 * p2 + bu3 * p3;
}

// 贝塞尔曲线一阶导数（切线）
Point bezierDerivative(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) {
    double u = 1 - t;
    return 3 * u * u * (p1 - p0) + 
           6 * u * t * (p2 - p1) + 
           3 * t * t * (p3 - p2);
}

// 贝塞尔曲线二阶导数
Point bezierSecondDerivative(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t) {
    double u = 1 - t;
    return 6 * u * (p2 - 2 * p1 + p0) + 
           6 * t * (p3 - 2 * p2 + p1);
}

// 计算曲率
double curvature(const Point& firstDeriv, const Point& secondDeriv) {
    double dx = firstDeriv.x(), dy = firstDeriv.y();
    double ddx = secondDeriv.x(), ddy = secondDeriv.y();
    double numerator = std::abs(dx * ddy - dy * ddx);
    double denominator = std::pow(dx * dx + dy * dy, 1.5);
    return denominator > 1e-10 ? numerator / denominator : 0.0;
}

// 计算斜率（方向角）
// double calculateSlopeAngle(const Point& derivative) {
//     if (std::abs(derivative.x) < 1e-6 && std::abs(derivative.y) < 1e-6) 
//         return 0.0;
//     return std::atan2(derivative.y, derivative.x);
// }
CurveQuality evaluateCurveQuality(
    const Point& P0, const Point& P1, const Point& P2, const Point& P3,
    const Point& Q0, const Point& Q1, const Point& Q2, const Point& Q3,
    int samples
) {
    CurveQuality quality = {
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(),
        0.0, 0, false
    };

    double lastCurvature = 0.0;
    bool lastSignPositive = false;
    bool firstSample = true;

    // 检查第一段曲线
    for (int i = 0; i <= samples; i++) {
        double t = static_cast<double>(i) / samples;
        Point d1 = bezierDerivative(P0, P1, P2, P3, t);
        Point d2 = bezierSecondDerivative(P0, P1, P2, P3, t);
        double k = curvature(d1, d2);
        
        if (k > quality.maxCurvature) quality.maxCurvature = k;
        if (k < quality.minCurvature) quality.minCurvature = k;
        quality.avgCurvature += k;
        
        // 检查曲率符号变化
        if (!firstSample) {
            bool currentSignPositive = k >= 0;
            if (currentSignPositive != lastSignPositive) {
                quality.signChanges++;
            }
            lastSignPositive = currentSignPositive;
        } else {
            firstSample = false;
            lastSignPositive = k >= 0;
        }
        lastCurvature = k;
    }

    // 检查第二段曲线
    firstSample = true;
    for (int i = 0; i <= samples; i++) {
        double t = static_cast<double>(i) / samples;
        Point d1 = bezierDerivative(Q0, Q1, Q2, Q3, t);
        Point d2 = bezierSecondDerivative(Q0, Q1, Q2, Q3, t);
        double k = curvature(d1, d2);
        
        if (k > quality.maxCurvature) quality.maxCurvature = k;
        if (k < quality.minCurvature) quality.minCurvature = k;
        quality.avgCurvature += k;
        
        // 检查曲率符号变化
        if (!firstSample) {
            bool currentSignPositive = k >= 0;
            if (currentSignPositive != lastSignPositive) {
                quality.signChanges++;
            }
            lastSignPositive = currentSignPositive;
        } else {
            firstSample = false;
            lastSignPositive = k >= 0;
        }
        lastCurvature = k;
    }

    quality.avgCurvature /= (2 * samples + 2);
    quality.hasSShape = quality.signChanges >= 2;  // 两次符号变化表示S型曲线

    return quality;
}

// 基于曲率优化的平滑曲线生成
std::vector<Point> generateCurvatureOptimizedCurve(
    const std::vector<Point>& line1, 
    const std::vector<Point>& line2,
    double maxCurvatureThreshold,  // 最大允许曲率
    double moveDistance,           // 每次移动距离
    int maxIterations                 // 最大优化迭代次数
) {
    // 验证输入
    if (line1.size() < 2 || line2.size() < 2) {
        std::cerr << "错误: 每条直线至少需要2个点." << std::endl;
        return {};
    }

    // 提取关键点
    Point A1 = line1[line1.size() - 2];
    Point originalA2 = line1[line1.size() - 1];
    Point B1 = line2[0];
    Point B2 = line2[1];
    Point M(0, 0);

    // 计算初始方向
    Point dir1 = (originalA2 - A1);
    dir1.Normalize();
    Point dir2 = (B2 - B1);
    dir2.Normalize();
    
    // 初始化优化变量
    Point currentA1 = A1;
    Point currentA2 = originalA2;
    double currentMove = 0.0;
    int iteration = 0;
    bool curveAccepted = false;
    
    // 存储最佳曲线
    std::vector<Point> bestCurve;
    double bestMaxCurvature = std::numeric_limits<double>::max();
    int bestSignChanges = std::numeric_limits<int>::max();
    
    // 优化循环
    while (iteration < maxIterations && !curveAccepted) {
        // 计算方向（可能需要重新计算）
        if (iteration > 0) {
            dir1 = (currentA2 - currentA1);
            dir1.Normalize();
        }
        
        // 计算起点和终点到M的距离
        double d_A2_M = (currentA2 - M).Length();
        double d_M_B1 = (M - B1).Length();
        double totalDist = d_A2_M + d_M_B1;
        
        // 控制点系数 - 随迭代次数衰减
        double alpha = 0.2f;
        double beta = 0.5f;
        // double alpha = 0.35 * (1.0 - 0.2 * iteration);
        // double beta = 0.35 * (1.0 - 0.2 * iteration);
        
        // 计算控制点距离
        double k1 = alpha * d_A2_M;
        double k2 = alpha * d_M_B1;
        double k3 = beta * d_A2_M;
        double k4 = beta * d_M_B1;
        
        // 计算整体方向向量
        Point T = (B1 - currentA2);
        T.Normalize();
        
        // 计算第一段曲线控制点
        Point P0 = currentA2;
        Point P1 = currentA2 + k1 * dir1;
        Point P2 = M - k2 * T;
        Point P3 = M;
        AINFO << "p0: " << P0.x() << ", " << P0.y() << ", p1: " << P1.x() << ", " << P1.y();

        // 计算第二段曲线控制点
        Point Q0 = M;
        Point Q1 = M + k3 * T;
        Point Q2 = B1 - k4 * dir2;
        Point Q3 = B1;
        AINFO << "q0: " << Q0.x() << ", " << Q0.y() << ", q1: " << Q1.x() << ", " << Q1.y();

        // 评估曲线质量
        CurveQuality quality = evaluateCurveQuality(P0, P1, P2, P3, Q0, Q1, Q2, Q3);
        
        AINFO << "迭代 " << iteration << ": ";
        AINFO << "最大曲率: " << quality.maxCurvature << ", ";
        AINFO << "符号变化: " << quality.signChanges << ", ";
        AINFO << "S型曲线: " << (quality.hasSShape ? "是" : "否") << std::endl;
        
        // 检查曲线是否可接受
        if (quality.maxCurvature <= maxCurvatureThreshold && !quality.hasSShape) {
            curveAccepted = true;
            AINFO << "曲线满足条件!" << std::endl;
        }
        
        // 保存最佳曲线
        // if (quality.maxCurvature < bestMaxCurvature && quality.signChanges < bestSignChanges) {
        if (curveAccepted) {
            // 生成曲线点
            std::vector<Point> curvePoints;
            for (int i = 0; i <= 20; i++) {
                double t = static_cast<double>(i) / 20;
                curvePoints.push_back(bezierPoint(P0, P1, P2, P3, t));
                AINFO << "曲线点: " << curvePoints.back().x() << ", " << curvePoints.back().y();
            }
            for (int i = 1; i <= 20; i++) {
                double t = static_cast<double>(i) / 20;
                curvePoints.push_back(bezierPoint(Q0, Q1, Q2, Q3, t));
            }
            bestCurve = curvePoints;
            bestMaxCurvature = quality.maxCurvature;
            bestSignChanges = quality.signChanges;
        }
        
        // 如果曲线不可接受，向前移动端点
        if (!curveAccepted) {
            currentMove += moveDistance;
            currentA1 = currentA1 - moveDistance * dir1;
            currentA2 = currentA2 - moveDistance * dir1;
            iteration++;
            
            AINFO << "移动端点 " << moveDistance << "米. ";
            AINFO << "新位置: A1= " << currentA1.x() << " , " << currentA1.y()  <<  ", A2= " << currentA2.x() << " , " << currentA2.y();
        }
    }
    
    // 输出最终结果
    if (curveAccepted) {
        AINFO << "成功生成优化曲线!" << std::endl;
    } else {
        AINFO << "达到最大迭代次数，使用最佳曲线." << std::endl;
        AINFO << "最佳曲线最大曲率: " << bestMaxCurvature << std::endl;
        AINFO << "最佳曲线符号变化: " << bestSignChanges << std::endl;
    }
    if(bestCurve.empty()) {
        AINFO << "没有生成有效曲线." << std::endl;
        return {};
    }
    //
    return bestCurve;
}

std::vector<Point> generateSmoothCurve(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    int numSegments             // 每段曲线的采样点数
) {
    // 验证输入
    if (line1.size() < 2 || line2.size() < 2) {
        std::cerr << "Error: Each line must have at least 2 points." << std::endl;
        return {};
    }

    // 提取关键点
    Point A1 = line1[line1.size() - 2]; // 第一条直线倒数第二点
    Point A2 = line1[line1.size() - 1]; // 第一条直线末点
    AINFO << "A1: " << A1.x() << ", " << A1.y() << ", A2: " << A2.x() << ", " << A2.y();
    Point B1 = line2[0];                // 第二条直线起点
    Point B2 = line2[1];                // 第二条直线第二点
    AINFO<< "B1: " << B1.x() << ", " << B1.y() << ", B2: " << B2.x() << ", " << B2.y();
    Point M(0, 0);                      // 必经点 (0,0)

    // 计算中间切向量（起点到终点的方向）
    Point T = B1 - A2;

    // 计算起点和终点到M的距离
    double distStart = (A2 - M).Length();
    if (distStart < 1e-6) {
        std::cerr << "Error: Start point is too close to M." << std::endl;
        return {};
    }
    double distEnd = (B1 - M).Length();
    if (distEnd < 1e-6) {
        std::cerr << "Error: End point is too close to M." << std::endl;
        return {};
    }

    // 设置弯曲系数（可调整，范围0.1~0.5）
    double alpha = 0.3;
    double k1 = alpha * distStart; // 起点控制点系数
    double k2 = alpha * distEnd;   // 终点控制点系数

    // 计算方向向量并单位化
    Point dirStart = (A2 - A1); // 
    dirStart.Normalize(); // 第一条直线起始方向
    Point dirEnd = (B2 - B1);   // 第二条直线起始方向
    dirEnd.Normalize(); // 第二条直线起始方向

    // 计算第一段曲线（A2 -> M）的控制点
    Point P0 = A2;
    Point P1 = A2 + k1 * dirStart;
    Point P2 = M - (1.0/5.0) * T; // 确保C1连续
    Point P3 = M;

    // 计算第二段曲线（M -> B1）的控制点
    Point Q0 = M;
    Point Q1 = M + (1.0/5.0) * T; // 确保C1连续
    Point Q2 = B1 - k2 * dirEnd;
    Point Q3 = B1;

    // 采样曲线点
    std::vector<Point> curvePoints;
    
    // 第一段曲线采样（包含M点）
    for (int i = 0; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        curvePoints.push_back(bezierPoint(P0, P1, P2, P3, t));
    }

    // 第二段曲线采样（跳过M点避免重复）
    for (int i = 1; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        curvePoints.push_back(bezierPoint(Q0, Q1, Q2, Q3, t));
    }

    return curvePoints;
}


std::vector<Point> GenerateStaticSmoothCurve(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    int numSegments             // 每段曲线的采样点数
) {
    // 验证输入
    if (line1.size() < 2 || line2.size() < 2) {
        std::cerr << "Error: Each line must have at least 2 points." << std::endl;
        return {};
    }
    AINFO << "输入点序 TOP"  << line1[0].x() << ", " << line1[1].x();
    AINFO << "输入点序 EGO"  << line2[0].x() << ", " << line2[1].x();

    // 设置弯曲系数（可调整，范围0.1~0.5）
    double alpha = 0.5;
    
    
    // 计算第一段曲线（A2 -> M）的控制点
    Point P0 = line2[1];  //插值起点
    Point P1 = {} ;
    Point P2 = {} ; // 可以根据实际距离调整
    Point P3 = line1[1];//插值终点
    
    // 计算方向向量并单位化
    Point P0P3 = (P3 - P0); // 
    auto len_total = P0P3.Length();
    
    auto dir_line1 = (line1[1] - line1[0]);
    dir_line1.Normalize();//toplane
    auto dir_line2 = (line2[1] - line2[0]);
    dir_line2.Normalize();
    
    P1 = P0 + 20 * dir_line2 ;
    P2 = P3 + 20 * dir_line1 ;
    

    AINFO << "P0: " << P0.x() << ", " << P0.y();
    AINFO << "P1: " << P1.x() << ", " << P1.y();
    AINFO << "P2: " << P2.x() << ", " << P2.y();
    AINFO << "P3: " << P3.x() << ", " << P3.y();


   

    // 采样曲线点
    std::vector<Point> curvePoints;
    
    // 第一段曲线采样（包含M点）
    for (int i = 0; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        curvePoints.push_back(bezierPoint(P0, P1, P2, P3, t));
        AINFO << "采样点: " << curvePoints[i].x() << ", " << curvePoints[i].y();
    }

    // 第二段曲线采样（跳过M点避免重复）
   
    return curvePoints;
}
std::vector<Point> GenerateStaticSmoothCurveUturn(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    const std::vector<Point>& obstacles, // 障碍物点集
    int numSegments             // 每段曲线的采样点数
) {
    // 验证输入
    if (line1.size() < 2 || line2.size() < 2) {
        std::cerr << "Error: Each line must have at least 2 points." << std::endl;
        return {};
    }
    AINFO << "输入点序 TOP "  << line1[0].x() << ", " << line1[1].x();
    AINFO << "            "  << line1[0].y() << ", " << line1[1].y();
    AINFO << "输入点序 EGO "  << line2[0].x() << ", " << line2[1].x();
    AINFO << "            "  << line2[0].y() << ", " << line2[1].y();
    auto dir_line1 = (line1[1] - line1[0]);
    dir_line1.Normalize();//toplane
    auto dir_line2 = (line2[1] - line2[0]);
    dir_line2.Normalize();
    dir_line1 = dir_line2; // 逆时针旋转180度
    // dir_line1.SelfRotate(M_PI); // 逆时针旋转180度
    
    // 采样曲线点
    std::vector<Point> curvePoints;
    auto last_point = line2[1];//这个动态调整 根据obstacles
    for (int i = 1; i < 6; i++) {
    last_point = last_point + dir_line2; // 取前10个点
    curvePoints.emplace_back(last_point);
    }

    // 设置弯曲系数（可调整，范围0.1~0.5）
    double alpha = 0.5;
    
    
    // 计算第一段曲线（A2 -> M）的控制点
    Point P0 = last_point;  //插值起点
    Point P1 = {} ;
    Point P2 = {} ; // 可以根据实际距离调整
    Point P3 = line1[1];//插值终点
    
    // 计算方向向量并单位化
    Point P0P3 = (P3 - P0); // 
    auto len_total = P0P3.Length();
    
    
    P1 = P0 + 6 * dir_line2 ;
    P2 = P3 + 6 * dir_line1 ;
    

    AINFO << "P0: " << P0.x() << ", " << P0.y();
    AINFO << "P1: " << P1.x() << ", " << P1.y();
    AINFO << "P2: " << P2.x() << ", " << P2.y();
    AINFO << "P3: " << P3.x() << ", " << P3.y();


   

    
    // 第一段曲线采样（包含M点）
    for (int i = 0; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        curvePoints.push_back(bezierPoint(P0, P1, P2, P3, t));
        // AINFO << "采样点: " << curvePoints[i].x() << ", " << curvePoints[i].y();
    }

    // 第二段曲线采样（跳过M点避免重复）
   
    return curvePoints;
}
std::vector<Point> GenerateUturnCircle(
    const std::vector<Point>& line1, // 第一条直线（取最后两点）
    const std::vector<Point>& line2, // 第二条直线（取前两点）
    const std::vector<Point>& obstacles, // 障碍物点集
    int numSegments             // 每段曲线的采样点数
) {
    // 验证输入
    if (line1.size() < 2 || line2.size() < 2) {
        std::cerr << "Error: Each line must have at least 2 points." << std::endl;
        return {};
    }
    AINFO << "输入点序 TOP "  << line1[0].x() << ", " << line1[1].x();
    AINFO << "            "  << line1[0].y() << ", " << line1[1].y();
    AINFO << "输入点序 EGO "  << line2[0].x() << ", " << line2[1].x();
    AINFO << "            "  << line2[0].y() << ", " << line2[1].y();
    auto dir_line2 = (line2[1] - line2[0]);
    dir_line2.Normalize();
    auto dirRotate = (line2[1] - line2[0]);
    dirRotate.Normalize();//toplane
    dirRotate.SelfRotate(M_PI/ 2); // 逆时针旋转90度
    
    // 采样曲线点
    std::vector<Point> curvePointsSeg1;
    auto last_point = line2[1];//这个动态调整 根据obstacles
    for (int i = 1; i < 10; i++) {
        last_point = last_point + dir_line2; // 取前10个点
        curvePointsSeg1.emplace_back(last_point);
    }
    auto center = curvePointsSeg1.back() + dirRotate * 6; // 圆心点，距离EGO点6米
    AINFO << "Center: " << center.x() << ", " << center.y();
    auto lenSeg1 = curvePointsSeg1.size();
    auto P0 =  curvePointsSeg1[lenSeg1 - 4]; // 从第六个开始弯曲
    auto P1 =  curvePointsSeg1[lenSeg1 - 1]; // 倒数第二个点
    
    
    // 第二段曲线采样（包含M点）
    // auto curvePointsSeg2 = std::vector<Point>();
    // auto mid = (line1[0] + last_point) * 0.5; // 中间点
    // auto AB = (line1[0] - last_point); // AB向量
    // auto d = AB.Length()*0.5; // AB向量长度
    // auto V_AB_norm = AB;
    // V_AB_norm.Normalize();
    // V_AB_norm.SelfRotate(M_PI/ 2); // 单位化AB向量
    // auto distance = std::sqrt(6*6 -  d*d); // 计算圆弧半径
    // auto center = mid + V_AB_norm * distance; // 圆心点，距离中间点6米
    // auto startAngle = std::atan2(last_point.y() - center.y(), last_point.x() - center.x());
    // auto endAngle =  std::atan2(line1[0].y()  - center.y(), line1[0].x()  - center.x());
    // AINFO << "Center: " << center.x() << ", " << center.y();
    // for (int i = 1; i <= numSegments; ++i) {
    //     double t = static_cast<double>(i) / numSegments;
    //     auto angle = startAngle +  t * (endAngle - startAngle);
    //     // rDir.SelfRotate(-M_PI/ 2); // 时针旋转90度
    //     double x = center.x() + 6 * cos(angle);
    //     double y = center.y() + 6 * sin(angle);
    //     AINFO << "圆弧采样点: " << x << ", " << y;
    //     curvePointsSeg2.push_back(Point(x, y));
    //     // AINFO << "采样点: " << curvePoints[i].x() << ", " << curvePoints[i].y();
    // }
    std::vector<Point> curvePointsSeg2;
    auto startAngle = std::atan2(last_point.y() - center.y(), last_point.x() - center.x());
    auto endAngle =  startAngle + M_PI; // 逆时针旋转180度

    for (int i = 1; i <= numSegments; ++i) {
        double t = static_cast<double>(i) / numSegments;
        auto angle = startAngle +  t * (endAngle - startAngle);
        // rDir.SelfRotate(-M_PI/ 2); // 时针旋转90度
        double x = center.x() + 6 * cos(angle);
        double y = center.y() + 6 * sin(angle);
        AINFO << "圆弧采样点: " << x << ", " << y;
        curvePointsSeg2.push_back(Point(x, y));
        // AINFO << "采样点: " << curvePoints[i].x() << ", " << curvePoints[i].y();
    }

    // auto P2 = curvePointsSeg2[0] ; // 插值控制点
    // auto P3 = curvePointsSeg2[5]; // 插值终点
    //     // 采样曲线点
    //     auto inserSeg1 = [&](){
    //         auto pts = std::vector<Point>();        
    //         for (int i = 0; i <= numSegments; ++i) {
    //             double t = static_cast<double>(i) / numSegments;
    //             pts.push_back(bezierPoint(P0, P1, P2, P3, t));
    //         }
    //         return pts;
    //     }();
        
    // auto lenSeg2 = curvePointsSeg2.size();
    // auto Q0  = curvePointsSeg2[lenSeg2 -3]; // 插值起点 
    // auto Q1  = curvePointsSeg2[lenSeg2 -1]; // 插值终点
    // //TO DO 生成对齐的直线
    
    // std::vector<Point> curvePointsSeg3;
    // for (int i = lenSeg1 - 1; i >= 0; i--) {
    //     last_point.set_x(2*center.x() - curvePointsSeg1[i].x());//关于center点对称  
    //     last_point.set_y(2*center.y() - curvePointsSeg1[i].y());//关于center点对称  
    //     curvePointsSeg3.emplace_back(last_point);
    // }

        
    // auto Q2  = line1[1]; // TO DO 
    // auto Q3  = curvePointsSeg3[6]; //  TO DO
        
    // auto inserSeg2 = [&](){
    //     auto pts = std::vector<Point>();        
    //     for (int i = 0; i <= numSegments; ++i) {
    //                 double t = static_cast<double>(i) / numSegments;
    //                 pts.push_back(bezierPoint(Q0, Q1, Q2, Q3, t));
    //             }
    //             return pts;
    //     }();
    std::vector<Point> curvePoints;
    curvePoints.insert(curvePoints.end(), curvePointsSeg1.begin(), curvePointsSeg1.end());
    curvePoints.insert(curvePoints.end(), curvePointsSeg2.begin(), curvePointsSeg2.end());
    // curvePoints.insert(curvePoints.end(), inserSeg1.begin(), inserSeg1.end());
    // curvePoints.insert(curvePoints.end(), curvePointsSeg2.begin() +5 , curvePointsSeg2.end() -3);
    // curvePoints.insert(curvePoints.end(), inserSeg2.begin() +1 , inserSeg2.end());
    // curvePoints.insert(curvePoints.end(), curvePointsSeg3.begin() + 5 , curvePointsSeg3.end());
    return curvePoints;
}


// 计算点集的最小包围盒
void boundingBox(const std::vector<Point>& points, Point& minPt, Point& maxPt) {
    if (points.empty()) return;
    minPt = maxPt = points[0];
    for (const auto& p : points) {
        minPt.set_x(std::min(minPt.x(), p.x()));
        minPt.set_y(std::min(minPt.y(), p.y()));
        maxPt.set_x(std::max(maxPt.x(), p.x()));
        maxPt.set_y(std::max(maxPt.y(), p.y()));
    }
}

// 计算点到线段的距离
double pointToSegmentDistance(const Point& p, const Point& a, const Point& b) {
    Point ab = b - a;
    Point ap = p - a;
    
    double abLengthSq = ab.x()*ab.x() + ab.y()*ab.y();
    if (abLengthSq < 1e-6) {
        // a和b重合，直接计算点到点的距离
        double dx = p.x() - a.x();
        double dy = p.y() - a.y();
        return std::sqrt(dx*dx + dy*dy);
    }
    
    double dot = ap.x()*ab.x() + ap.y()*ab.y();
    double t = std::clamp(dot / abLengthSq, 0.0, 1.0);
    
    Point projection = a + ab * t;
    double dx = p.x() - projection.x();
    double dy = p.y() - projection.y();
    return std::sqrt(dx*dx + dy*dy);
}

// 计算轨迹的旋转方向（逆时针为正）
double calculateTrajectoryRotation(const std::vector<Point>& trajectory) {
    if (trajectory.size() < 3) return 0.0;
    
    double totalRotation = 0.0;
    Point prevVec = trajectory[1] - trajectory[0];
    
    for (size_t i = 2; i < trajectory.size(); ++i) {
        Point currVec = trajectory[i] - trajectory[i-1];
        double crossProd = prevVec.CrossProd(currVec);
        double dotProd = prevVec.x()*currVec.x() + prevVec.y()*currVec.y();
        
        // 只考虑实际转向部分
        if (std::abs(crossProd) > 1e-3 && currVec.Length() > 0.1) {
            double angle = std::atan2(crossProd, dotProd);
            totalRotation += angle;
        }
        
        prevVec = currVec;
    }
    
    return totalRotation;
}

// 主轨迹规划类

    // 规划掉头轨迹
std::vector<Point> TrajectoryPlanner::planUTurn(
    const Point& start, double startYaw,  // 起点和航向角(弧度)
    const Point& end, double endYaw,      // 终点和航向角
    const std::vector<Point>& obstacles,  // 障碍物点集
    double safetyMargin)            // 安全距离
{
    // 1. 计算方向向量
    Point startDir(std::cos(startYaw), std::sin(startYaw));
    Point endDir(std::cos(endYaw), std::sin(endYaw));
    Point   T = end - start;
    if (T.Length() < 1e-6) {
        AINFO << "起点和终点重合，无法规划掉头轨迹.";
        return {};
    }
    
    // 2. 确保逆时针方向 (左侧掉头)
    Point mid = (start + end) * 0.5;
    Point leftTurnDir(-startDir.y(), startDir.x()); // 左侧垂直方向
    
    // 3. 计算安全偏移距离
    double offsetDistance = calculateSafeOffset(start, end, leftTurnDir, obstacles, safetyMargin);
    offsetDistance = 20.f; // 固定偏移距离，避免过小导致轨迹不明显
    
    // 4. 调整中间点位置 (确保逆时针)
    mid = mid + leftTurnDir * offsetDistance;
    
    // 5. 计算控制点 (确保逆时针方向)
    AINFO  << "start: " << start.x() << ", " << start.y();
    Point control1 = start + startDir * offsetDistance;
    AINFO <<  "control1: " << control1.x() << ", " << control1.y();
    Point control2 = mid - 0.2*T; // 调整曲率
    AINFO <<  "control2: " << control2.x() << ", " << control2.y();
    AINFO << "mid: " << mid.x() << ", " << mid.y();
    Point control3 = mid + 0.2*T; // 调整曲率
    AINFO <<  "control3: " << control3.x() << ", " << control3.y();
    Point control4 = end - endDir * offsetDistance;
    AINFO <<  "control4: " << control4.x() << ", " << control4.y();
    AINFO << "end: " << end.x() << ", " << end.y();    
    // 6. 生成两段贝塞尔曲线
    std::vector<Point> trajectory;
    generateBezierSegment(start, control1, control2, mid, trajectory);
    generateBezierSegment(mid, control3, control4, end, trajectory);
    
    // 7. 验证旋转方向
    // double totalRotation = calculateTrajectoryRotation(trajectory);
    // if (totalRotation < 0) {
    //     // 如果检测到顺时针方向，强制调整为逆时针
    //     return forceCounterClockwise(start, startYaw, end, endYaw, 
    //                                 obstacles, safetyMargin);
    // }
    
    return trajectory;
}

double TrajectoryPlanner::calculateSafeOffset(
    const Point& start, const Point& end,
    const Point& leftTurnDir,  // 逆时针方向向量
    const std::vector<Point>& obstacles,
    double safetyMargin)
{
    if (obstacles.empty()) return 3.0; // 默认偏移
    
    // 计算包围盒尺寸
    Point minPt, maxPt;
    std::vector<Point> allPoints = {start, end};
    allPoints.insert(allPoints.end(), obstacles.begin(), obstacles.end());
    boundingBox(allPoints, minPt, maxPt);
    
    double width = maxPt.x() - minPt.x();
    double height = maxPt.y() - minPt.y();
    double maxDim = std::max(width, height);
    
    // 尝试不同偏移量 (从最小到最大)
    for (double offset = maxDim * 0.1; offset <= maxDim * 0.5; offset += maxDim * 0.05) {
        if (isOffsetSafe(start, end, leftTurnDir, obstacles, safetyMargin, offset)) {
            return offset;
        }
    }
    
    return maxDim * 0.3; // 默认返回中等偏移
}

// 检查偏移是否安全
bool TrajectoryPlanner::isOffsetSafe(
    const Point& start, const Point& end,
    const Point& leftTurnDir,
    const std::vector<Point>& obstacles,
    double safetyMargin, double offset)
{
    Point mid = (start + end) * 0.5;
    
    // 创建测试控制点 (逆时针方向)
    Point control1 = start + Point(std::cos(0), std::sin(0)) * offset;
    Point control2 = mid - leftTurnDir * offset * 0.7;
    Point control3 = mid + leftTurnDir * offset * 0.7;
    Point control4 = end - Point(std::cos(M_PI), std::sin(M_PI)) * offset;
    
    // 生成测试轨迹
    std::vector<Point> testTraj;
    generateBezierSegment(start, control1, control2, mid, testTraj, 10);
    generateBezierSegment(mid, control3, control4, end, testTraj, 10);
    
    // 检查所有轨迹点与障碍物的距离
    for (const auto& pt : testTraj) {
        for (const auto& obs : obstacles) {
            double dx = pt.x() - obs.x();
            double dy = pt.y() - obs.y();
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < safetyMargin) {
                return false;
            }
        }
    }
    
    return true;
}

// 生成单段贝塞尔曲线
void TrajectoryPlanner::generateBezierSegment(
    const Point& p0, const Point& p1,
    const Point& p2, const Point& p3,
    std::vector<Point>& output,
    int segments)
{
    for (int i = 0; i <= segments; ++i) {
        double t = static_cast<double>(i) / segments;
        output.push_back(bezierPoint(p0, p1, p2, p3, t));
    }
}

// 强制生成逆时针轨迹
 std::vector<Point> TrajectoryPlanner::forceCounterClockwise(
    const Point& start, double startYaw,
    const Point& end, double endYaw,
    const std::vector<Point>& obstacles,
    double safetyMargin)
{
    // 增加偏移距离确保逆时针
    Point startDir(std::cos(startYaw), std::sin(startYaw));
    Point leftTurnDir(-startDir.y(), startDir.x()); // 左侧垂直方向
    Point endDir(std::cos(endYaw), std::sin(endYaw));
    
    // 计算包围盒尺寸
    Point minPt, maxPt;
    std::vector<Point> allPoints = {start, end};
    allPoints.insert(allPoints.end(), obstacles.begin(), obstacles.end());
    boundingBox(allPoints, minPt, maxPt);
    double maxDim = std::max(maxPt.x()-minPt.x(), maxPt.y()-minPt.y());
    
    // 使用更大偏移确保逆时针
    double offset = maxDim * 1;
    Point mid = (start + end) * 0.5 + (startDir) * offset;
    
    // 控制点设置
    Point control1 = start + startDir * offset;
    Point control2 = mid;

    Point control3 = mid + leftTurnDir * offset * 0.5;
    Point control4 = end + endDir * offset;
    
    // 生成轨迹
    std::vector<Point> trajectory;
    generateBezierSegment(start, control1, control2, mid, trajectory);
    generateBezierSegment(mid, control3, control4, end, trajectory);
    
    return trajectory;
}

}
} // namespace fusion
} // namespace cem


