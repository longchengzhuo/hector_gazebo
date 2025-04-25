#include "../../include/common/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
void CheaterOrientationEstimator::run() {
//   this->_stateEstimatorData.result->orientation[0] =
//       this->_stateEstimatorData.lowState->imu.quaternion[0];
//   this->_stateEstimatorData.result->orientation[1] =
//       this->_stateEstimatorData.lowState->imu.quaternion[1];
//   this->_stateEstimatorData.result->orientation[2] =
//       this->_stateEstimatorData.lowState->imu.quaternion[2];
//   this->_stateEstimatorData.result->orientation[3] =
//       this->_stateEstimatorData.lowState->imu.quaternion[3];
this->_stateEstimatorData.result->rpy[0] = this->_stateEstimatorData.lowState->rpy[0];
this->_stateEstimatorData.result->rpy[1] = this->_stateEstimatorData.lowState->rpy[1];
this->_stateEstimatorData.result->rpy[2] = this->_stateEstimatorData.lowState->rpy[2];

this->_stateEstimatorData.result->orientation = ori::rpyToQuat(this->_stateEstimatorData.result->rpy);

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaBody(0) =
      this->_stateEstimatorData.lowState->imu.gyroscope[0]*1.0;
  this->_stateEstimatorData.result->omegaBody(1) =
      this->_stateEstimatorData.lowState->imu.gyroscope[1]*1.0;
  this->_stateEstimatorData.result->omegaBody(2) =
      this->_stateEstimatorData.lowState->imu.gyroscope[2]*1.0;
    //   this->_stateEstimatorData.result->rpy =
    // ori::quatToRPY(this->_stateEstimatorData.result->orientation);

    this->_stateEstimatorData.result->omegaWorld =
        this->_stateEstimatorData.result->rBody.transpose()*
        this->_stateEstimatorData.result->omegaBody;

      
}