#include "tracking.h"
#include <iostream>
// #define SAVE_TO_CSV

Tracking::Tracking() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = Eigen::MatrixXd(2, 2);
  H_laser_ = Eigen::MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.1, 0,
              0, 0.1;

  kf_.P_ = Eigen::MatrixXd(4, 4);
  kf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


}

Tracking::~Tracking() {}

void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {

    // initial measurement
    kf_.x_ = Eigen::VectorXd(4);
    kf_.x_ << 1, 1, 1, 1;

    kf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
    return;
  }

  long double time = measurement_pack.timestamp_;
  double delta_t = (time - previous_timestamp_);
  previous_timestamp_ = time;

  double delta_t2 = delta_t * delta_t;
  double delta_t3 = delta_t2 * delta_t;
  double delta_t4 = delta_t3 * delta_t;
  double noise_ax = 4;
  double noise_ay = 4;

  kf_.Q_ = Eigen::MatrixXd(4, 4);
  kf_.Q_ << (delta_t4 * noise_ax) / 4, 0, (delta_t3 * noise_ax) / 2, 0,
             0, (delta_t4 * noise_ay) / 4, 0, (delta_t3 * noise_ay) / 2,
             (delta_t3 * noise_ax) / 2, 0, delta_t2 * noise_ax, 0,
             0, (delta_t3 * noise_ay) / 2, 0, delta_t2 * noise_ay;

  kf_.F_ = Eigen::MatrixXd(4, 4);
  kf_.F_ << 1, 0, delta_t, 0,
             0, 1, 0, delta_t,
             0, 0, 1, 0,
             0, 0, 0, 1;

  kf_.Predict();

  kf_.H_ = H_laser_;
  kf_.R_ = R_laser_;
  kf_.Update(measurement_pack.raw_measurements_);

  // print the output
  #ifdef SAVE_TO_CSV
  std::cout << kf_.x_[0] << "," << kf_.x_[1] << std::endl;
  #else
  std::cout << "x_ = " << kf_.x_ << std::endl;
  std::cout << "P_ = " << kf_.P_ << std::endl;
  #endif
}
