#include <ceres/ceres.h>

#include <cmath>

// c0, c1, c2, c3: 目标曲线系数
struct Point2CurveResidual {
  Point2CurveResidual(double x, double y, double c0, double c1, double c2,
                            double c3, double start_x, double start_y,
                            double end_x, double end_y, double factor)
      : x_(x),
        y_(y),
        c0_(c0),
        c1_(c1),
        c2_(c2),
        c3_(c3),
        start_x_(start_x),
        start_y_(start_y),
        end_x_(end_x),
        end_y_(end_y),
        factor_(factor) {}

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    // params[0]: theta, params[1]: tx, params[2]: ty
    T theta = -params[0];
    // T tx = -params[1];
    T ty = -params[2];

    // 变换后的点
    // T x_new = T(x_) * ceres::cos(theta) - T(y_) * ceres::sin(theta) + tx;
    T x_new = T(x_) * ceres::cos(theta) - T(y_) * ceres::sin(theta);
    T y_new = T(x_) * ceres::sin(theta) + T(y_) * ceres::cos(theta) + ty;

    if (x_new < start_x_) {
      T delta_x = x_new - T(start_x_);
      T delta_y = y_new - T(start_y_);
      residual[0] = ceres::sqrt(delta_x * delta_x + delta_y * delta_y);
    }
    if (x_new > end_x_) {
      T delta_x = x_new - T(end_x_);
      T delta_y = y_new - T(end_y_);
      residual[0] = ceres::sqrt(delta_x * delta_x + delta_y * delta_y);
    } else {
      // 曲线2在x_new处的y值
      T y_curve = T(c0_) + T(c1_) * x_new + T(c2_) * x_new * x_new +
                  T(c3_) * x_new * x_new * x_new;

      // 残差: 变换后点与曲线的垂直距离（也可用几何距离，但这个收敛好）
      residual[0] = T(factor_) * (y_new - y_curve);
    }

    return true;
  }

  double x_, y_;
  double c0_, c1_, c2_, c3_;
  double start_x_, start_y_, end_x_, end_y_;
  double factor_;
};

struct Point2PointResidual {
  Point2PointResidual(double x1, double y1, double x2, double y2)
      : x1_(x1),
        y1_(y1),
        x2_(x2),
        y2_(y2){}

  template <typename T>
  bool operator()(const T* const params, T* residual) const {
    // params[0]: theta, params[1]: tx, params[2]: ty
    T theta = params[0];
    T tx = params[1];
    T ty = params[2];

    // 变换后的点
    // T delta_x = T(x1_) * ceres::cos(theta) - T(y1_) * ceres::sin(theta) + tx - x2_;
    // // T delta_x = T(x1_) * ceres::cos(theta) - T(y1_) * ceres::sin(theta) - x2_;
    // T delta_y = T(x1_) * ceres::sin(theta) + T(y1_) * ceres::cos(theta) + ty - y2_;

    // residual[0] = ceres::sqrt(delta_x * delta_x + delta_y * delta_y);

    residual[0] = T(x1_) * ceres::cos(theta) - T(y1_) * ceres::sin(theta) + tx - x2_;
    // T delta_x = T(x1_) * ceres::cos(theta) - T(y1_) * ceres::sin(theta) - x2_;
    residual[1] = T(x1_) * ceres::sin(theta) + T(y1_) * ceres::cos(theta) + ty - y2_;

    return true;
  }

  double x1_, y1_, x2_, y2_;
};