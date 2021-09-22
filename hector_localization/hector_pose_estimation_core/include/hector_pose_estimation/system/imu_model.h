//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_IMU_MODEL_H
#define HECTOR_POSE_ESTIMATION_IMU_MODEL_H

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/system.h>

#include <hector_pose_estimation/system/imu_input.h>

namespace hector_pose_estimation {

class GyroModel : public TimeContinuousSystemModel_<GyroModel,3>
{
public:
  using TimeContinuousSystemModel_<GyroModel,3>::SubState;

  GyroModel();
  virtual ~GyroModel();

  bool init(PoseEstimation& estimator, System &system, State& state);
  void getPrior(State &state);

  ColumnVector3 getError() const { return bias_->getVector(); }

  ColumnVector3 getRate(const ImuInput::RateType& imu_rate, const State& state) const;
  void getRateJacobian(SystemMatrixBlock& C, const State& state, bool init = true);
  void getRateNoise(CovarianceBlock Q, const State& state, bool init = true);

  using TimeContinuousSystemModel_<GyroModel,3>::getSystemNoise;
  void getSystemNoise(NoiseVariance& Q, const State& state, bool init = true);

private:
  typename SubState::Ptr bias_;
  double rate_stddev_;
  double rate_drift_;
};

class AccelerometerModel : public TimeContinuousSystemModel_<AccelerometerModel,3>
{
public:
  using TimeContinuousSystemModel_<AccelerometerModel,3>::SubState;

  AccelerometerModel();
  virtual ~AccelerometerModel();

  bool init(PoseEstimation& estimator, System &system, State& state);
  void getPrior(State &state);

  ColumnVector3 getError() const { return bias_->getVector(); }

  ColumnVector3 getAcceleration(const ImuInput::AccelerationType& imu_acceleration, const State& state) const;
  void getAccelerationJacobian(SystemMatrixBlock& C, const State& state, bool init = true);
  void getAccelerationNoise(CovarianceBlock Q, const State& state, bool init = true);

  using TimeContinuousSystemModel_<AccelerometerModel,3>::getSystemNoise;
  void getSystemNoise(NoiseVariance& Q, const State& state, bool init = true);

private:
  typename SubState::Ptr bias_;
  double acceleration_stddev_;
  double acceleration_drift_;
};

typedef System_<GyroModel> Gyro;
extern template class System_<GyroModel>;

typedef System_<AccelerometerModel> Accelerometer;
extern template class System_<AccelerometerModel>;

}

#endif // HECTOR_POSE_ESTIMATION_IMU_MODEL_H
