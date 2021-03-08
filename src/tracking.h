#ifndef Tracking_H_
#define Tracking_H_

#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"

class Tracking {
 public:
  Tracking();
  virtual ~Tracking();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter kf_;

 private:
  bool is_initialized_;
  long double previous_timestamp_;

  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd H_laser_;
};

#endif
