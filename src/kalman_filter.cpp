#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = (F_ * P_ * (F_.transpose())) + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z) {
  Eigen::VectorXd y(2);
  Eigen::MatrixXd S(2, 2);
  Eigen::MatrixXd K(4, 2);
  Eigen::MatrixXd I;

  y = z - (H_ * x_);
  S = (H_ * P_ * (H_.transpose())) + R_;
  K = P_ * (H_.transpose()) * (S.inverse());
  x_ = x_ + (K * y);
  int size = x_.size();
  I = Eigen::MatrixXd::Identity(size, size);
  P_ = (I - (K * H_)) * P_;
}
