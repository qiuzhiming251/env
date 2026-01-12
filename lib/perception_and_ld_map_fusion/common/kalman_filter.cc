#include "lib/perception_and_ld_map_fusion/common/kalman_filter.h"

#include "cyber/cyber.h"

namespace cem {
namespace fusion {

KalmanFilter::KalmanFilter(int state_size, int meas_size, int u_size)
    : state_size_(state_size), meas_size_(meas_size), u_size_(u_size) {}

void KalmanFilter::Init(Eigen::VectorXd &x, Eigen::MatrixXd &P,
                        Eigen::MatrixXd &R, Eigen::MatrixXd &Q) {
  if (state_size_ == 0 || meas_size_ == 0) {
    AERROR << "Error, State size and measurement size must be greater than 0\n";
  }
  if (x.size() != state_size_) {
    AERROR << "Invalid X input. (Required size: " << state_size_
           << ", Input size: " << x.size() << ")";
  }
  if ((P.rows() != state_size_) || (P.cols() != state_size_)) {
    AERROR << "Invalid P input. (Required size: " << state_size_ << "x"
           << state_size_ << ", Input size: " << P.rows() << "x" << P.cols()
           << ")";
  }
  if ((R.rows() != meas_size_) || (R.cols() != meas_size_)) {
    AERROR << "Invalid R input. (Required size: " << meas_size_ << "x"
           << meas_size_ << ", Input size: " << R.rows() << "x" << R.cols()
           << ")";
  }
  if ((Q.rows() != state_size_) || (Q.cols() != state_size_)) {
    AERROR << "Invalid state vector input. (Required size: " << state_size_
           << ", Input size: " << x.size() << ")";
    AERROR << "Invalid Q input. (Required size: " << state_size_ << "x"
           << state_size_ << ", Input size: " << Q.rows() << "x" << Q.cols()
           << ")";
  }
  x_ = x;
  F_.resize(state_size_, state_size_);
  F_.setIdentity();
  u_.resize(u_size_);
  u_.setZero();
  B_.resize(state_size_, u_size_);
  B_.setZero();
  P_ = P;
  H_.resize(meas_size_, state_size_);
  H_.setIdentity();
  z_.resize(meas_size_);
  z_.setZero();
  R_ = R;
  Q_ = Q;
  is_init_ = true;
}

void KalmanFilter::Init(Eigen::VectorXd &x) {
  if (state_size_ == 0 || meas_size_ == 0) {
    AERROR << "Error, State size and measurement size must be greater than 0\n";
  }
  if (x.size() != state_size_) {
    AERROR << "Invalid state vector input. (Required size: " << state_size_
           << ", Input size: " << x.size() << ")";
  }
  x_ = x;
  F_.resize(state_size_, state_size_);
  F_.setIdentity();
  u_.resize(u_size_);
  u_.setZero();
  B_.resize(state_size_, u_size_);
  B_.setZero();
  P_.resize(state_size_, state_size_);
  P_.setIdentity();
  H_.resize(meas_size_, state_size_);
  H_.setIdentity();
  z_.resize(meas_size_);
  z_.setZero();
  Q_.resize(state_size_, state_size_);
  Q_.setIdentity();
  R_.resize(meas_size_, meas_size_);
  R_.setIdentity();
  is_init_ = true;
}

bool KalmanFilter::IsInit() { return is_init_; }

void KalmanFilter::Reset() { is_init_ = false; }

Eigen::VectorXd KalmanFilter::Predict(Eigen::MatrixXd &F, Eigen::MatrixXd &B,
                                      Eigen::VectorXd &u) {
  F_ = F;
  B_ = B;
  u_ = u;
  x_ = F * x_ + B * u;
  Eigen::MatrixXd F_T = F.transpose();
  P_ = F * P_ * F_T + Q_;
  return x_;
}

Eigen::VectorXd KalmanFilter::Predict(Eigen::MatrixXd &F) {
  F_ = F;
  x_ = F_ * x_;
  Eigen::MatrixXd F_T = F_.transpose();
  P_ = F_ * P_ * F_T + Q_;
  // AINFO << "\n"<<F_;
  // AINFO << "\n"<<P_;
  return x_;
}

Eigen::VectorXd KalmanFilter::Predict(Eigen::MatrixXd &F, Eigen::MatrixXd &Q) {
  Q_ = Q;
  return Predict(F);
};

Eigen::VectorXd KalmanFilter::Predict(double delta_t) {
  SetF(delta_t);
  return Predict(F_);
}

Eigen::VectorXd KalmanFilter::Predict(double delta_t, Eigen::MatrixXd &Q) {
  SetF(delta_t);
  return Predict(F_, Q);
}

void KalmanFilter::Update(Eigen::MatrixXd &H, Eigen::VectorXd &z) {
  H_ = H;
  // AINFO << "\n"<<P_;
  // AINFO << "\n"<<R_;
  // AINFO << "\n"<<H_;
  Eigen::MatrixXd H_T = H_.transpose();
  Eigen::MatrixXd temp1 = H_ * P_ * H_T + R_;
  // AINFO << "\n"<<temp1;
  Eigen::MatrixXd temp2 = temp1.inverse();
  // AINFO << "\n"<<temp2;
  Eigen::MatrixXd K = P_ * H_T * temp2;
  // AINFO <<"\n"<< K;
  z_ = H_ * x_;
  x_ = x_ + K * (z - z_);
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_size_, state_size_);
  P_ = (I - K * H_) * P_;
  // AINFO << "\n"<<P_;
}

void KalmanFilter::Update(Eigen::VectorXd &z) { Update(H_, z); }

void KalmanFilter::Update(Eigen::VectorXd &z, Eigen::MatrixXd &R) {
  R_ = R;
  Update(H_, z);
}

}  // namespace fusion
}  // namespace cem