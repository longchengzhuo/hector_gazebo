/*!
 * @file
 * @brief Orientation Estimation Algorithms
 * 
 * orientation: quaternion
 * rBody: transformation matrix( vBody = Rbody * vWorld)
 * omegaBody: angular vel in body frame
 * omegaWorld: ... in world frame
 * rpy: roll pitch yaw
 */

#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "StateEstimatorContainer.h"
#include "Math/orientation_tools.h"

/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */

class CheaterOrientationEstimator : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup() {}
};

class OrientationEstimator : public GenericEstimator {
 public:
  virtual void run();
  virtual void setup();

  Vec3<double> gyro_raw;
  Vec3<double> accel_raw;
  double kalman_roll = 0;
  double kalman_pitch = 0;
  double kalman_roll_ucty = 0.5*0.5;
  double kalman_pitch_ucty = 0.5*0.5;
  double yaw = 0;
  Mat3<double> R;
  double dt = 0.001;
  Vec3<double> accel_data;
  Vec3<double> gyro_data;
  Vec3<double> gyro_corrected;
  Vec3<double> accel_corrected;

  bool debug = false;

};

#endif