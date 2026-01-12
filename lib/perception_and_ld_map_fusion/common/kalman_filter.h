#pragma once
#include <Eigen/Dense>

namespace cem {
namespace fusion {

class KalmanFilter {
 private:
  int state_size_;     // 状态变量维度
  int meas_size_;      // 观测变量维度
  int u_size_;         // 控制变量维度
  Eigen::VectorXd x_;  // 状态向量
  Eigen::VectorXd z_;  // 观测向量
  Eigen::MatrixXd B_;  // 控制矩阵
  Eigen::VectorXd u_;  // 控制向量
  Eigen::MatrixXd P_;  // 误差协方差矩阵
  Eigen::MatrixXd H_;  // 观测矩阵
  Eigen::MatrixXd R_;  // 观测噪声协方差矩阵
  Eigen::MatrixXd Q_;  // 过程噪声协方差矩阵
  bool is_init_ = false;

 protected:
  Eigen::MatrixXd F_;  // 状态转移矩阵

 public:
  KalmanFilter(int state_size, int meas_size, int u_size);
  ~KalmanFilter() {}
  void Init(Eigen::VectorXd &x, Eigen::MatrixXd &P, Eigen::MatrixXd &R,
            Eigen::MatrixXd &Q);
  void Init(Eigen::VectorXd &x);
  bool IsInit();
  void Reset();
  Eigen::VectorXd Predict(Eigen::MatrixXd &F);
  Eigen::VectorXd Predict(Eigen::MatrixXd &F, Eigen::MatrixXd &Q);
  Eigen::VectorXd Predict(Eigen::MatrixXd &F, Eigen::MatrixXd &B,
                          Eigen::VectorXd &u);
  Eigen::VectorXd Predict(double delta_t);
  Eigen::VectorXd Predict(double delta_t, Eigen::MatrixXd &Q);
  void Update(Eigen::MatrixXd &H, Eigen::VectorXd &z);
  void Update(Eigen::VectorXd &z);
  void Update(Eigen::VectorXd &z, Eigen::MatrixXd &R);
  const Eigen::VectorXd &GetState() const { return x_; };

 private:
  virtual void SetF(double delta_t) = 0;
};

}  // namespace fusion
}  // namespace cem