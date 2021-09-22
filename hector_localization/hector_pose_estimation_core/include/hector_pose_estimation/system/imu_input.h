//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_POSE_ESTIMATION_IMvector_INPUT_H
#define HECTOR_POSE_ESTIMATION_IMvector_INPUT_H

#include <hector_pose_estimation/input.h>
#include <hector_pose_estimation/matrix.h>

#include <sensor_msgs/Imu.h>

namespace hector_pose_estimation {

class ImuInput : public Input_<6>
{
public:
  enum InputIndex {
    ACCEL_X = 0,
    ACCEL_Y,
    ACCEL_Z,
    GYRO_X,
    GYRO_Y,
    GYRO_Z
  };
  typedef typename Vector::ConstFixedSegmentReturnType<3>::Type AccelerationType;
  typedef typename Vector::ConstFixedSegmentReturnType<3>::Type RateType;

  ImuInput() {}
  ImuInput(const sensor_msgs::Imu& imu) { *this = imu; }
  virtual ~ImuInput() {}

  virtual const std::string& getName() const { static std::string name("imu"); return name; }

  ImuInput &operator=(const sensor_msgs::Imu& imu) {
    u_(ACCEL_X) = imu.linear_acceleration.x;
    u_(ACCEL_Y) = imu.linear_acceleration.y;
    u_(ACCEL_Z) = imu.linear_acceleration.z;
    u_(GYRO_X)  = imu.angular_velocity.x;
    u_(GYRO_Y)  = imu.angular_velocity.y;
    u_(GYRO_Z)  = imu.angular_velocity.z;

    // TODO: set variance if message contains non-zero covariance matrix
    return *this;
  }

  AccelerationType getAcceleration() const { return u_.segment<3>(ACCEL_X); }
  RateType getRate() const { return u_.segment<3>(GYRO_X); }
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_IMvector_INPUT_H
