#include <math.h>
#include <iostream>
#include <fstream>
#include "tracking.h"

int main() {
  std::ifstream inputFile;
  inputFile.open("../observations.csv");
  
  if (!inputFile.is_open()) {
    std::cout << "Laser measurement data file missing" << std::endl;
    return 0;
  }

  std::string timestamp;
  std::string x;
  std::string y;
  std::string measurement;
  std::vector<MeasurementPackage> measurements_list;

  while (inputFile.good()) {
    getline(inputFile, timestamp, ',');
    getline(inputFile, x, ',');
    getline(inputFile, y, '\n');
    MeasurementPackage measurements;
    measurements.raw_measurements_ = Eigen::VectorXd(2);
    measurements.raw_measurements_ << std::stod(x),std::stod(y);
    measurements.timestamp_ = std::stold(timestamp);
    measurements_list.push_back(measurements);
  }
  size_t N = measurements_list.size();
  Tracking tracking;

  for (size_t k = 0; k < N; k++) {
    tracking.ProcessMeasurement(measurements_list[k]);
  }

  if (inputFile.is_open()) {
    inputFile.close();
  }

  return 0;

}
