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

#include <hector_pose_estimation/system/imu_model.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class System_<GyroModel>;
template class System_<AccelerometerModel>;

static const Matrix3 MinusIdentity = -Matrix3::Identity();

GyroModel::GyroModel()
{
  rate_stddev_ = 1.0 * M_PI/180.0;
  rate_drift_ = 1.0e-1 * M_PI/180.0;
  parameters().add("stddev", rate_stddev_);
  parameters().add("drift", rate_drift_);
}

GyroModel::~GyroModel()
{}

bool GyroModel::init(PoseEstimation& estimator, System &system, State& state)
{
  bias_ = state.addSubState<3,3>(this, system.getName() + "_bias");
  return (bias_.get() != 0);
}

void GyroModel::getPrior(State &state)
{
  bias_->block(state.P()) = 3600./2. * pow(rate_drift_, 2) * SymmetricMatrix3::Identity();
}

void GyroModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (!init) return;
  bias_->block(Q)(X,X) = bias_->block(Q)(Y,Y) = pow(rate_drift_, 2);
  bias_->block(Q)(Z,Z) = pow(rate_drift_, 2);
}

ColumnVector3 GyroModel::getRate(const ImuInput::RateType& imu_rate, const State& state) const
{
  return imu_rate - bias_->getVector();
}

void GyroModel::getRateJacobian(SystemMatrixBlock& C, const State& state, bool init)
{
  if (!init) return;
  bias_->cols(C) = MinusIdentity;
}

void GyroModel::getRateNoise(CovarianceBlock Q, const State &, bool init)
{
  if (!init) return;
  Q(X,X) = Q(Y,Y) = Q(Z,Z) = pow(rate_stddev_, 2);
}

//void GyroModel::getDerivative(StateVector &x_dot, const State &state)
//{
//  x_dot.setZero();
//  if (state.orientation() && !state.rate()) {
//    state.orientation()->segment(x_dot).head(3) = state.R() * bias_->vector();
//  }
//}

//void GyroModel::getStateJacobian(SystemMatrix& A, const State& state)
//{
//  A.setZero();
//  if (state.orientation() && !state.rate()) {
//    state.orientation()->block(A, *bias_) = state.R();
//  }
//}

AccelerometerModel::AccelerometerModel()
{
  acceleration_stddev_ = 1.0e-2;
  acceleration_drift_ = 1.0e-2;
  parameters().add("stddev", acceleration_stddev_);
  parameters().add("drift", acceleration_drift_);
}

AccelerometerModel::~AccelerometerModel()
{}

bool AccelerometerModel::init(PoseEstimation& estimator, System &system, State& state)
{
  bias_ = state.addSubState<3,3>(this, system.getName() + "_bias");
  return (bias_.get() != 0);
}

void AccelerometerModel::getPrior(State &state)
{
  bias_->block(state.P()) = 3600./2. * pow(acceleration_drift_, 2) * SymmetricMatrix3::Identity();
}

void AccelerometerModel::getSystemNoise(NoiseVariance& Q, const State&, bool init)
{
  if (!init) return;
  bias_->block(Q)(X,X) = bias_->block(Q)(Y,Y) = pow(acceleration_drift_, 2);
  bias_->block(Q)(Z,Z) = pow(acceleration_drift_, 2);
}

ColumnVector3 AccelerometerModel::getAcceleration(const ImuInput::AccelerationType& imu_acceleration, const State& state) const
{
  return imu_acceleration - bias_->getVector();
}

void AccelerometerModel::getAccelerationJacobian(SystemMatrixBlock& C, const State&, bool init)
{
  if (!init) return;
  bias_->cols(C) = MinusIdentity;
}

void AccelerometerModel::getAccelerationNoise(CovarianceBlock Q, const State &, bool init)
{
  if (!init) return;
  Q(X,X) = Q(Y,Y) = Q(Z,Z) = pow(acceleration_stddev_, 2);
}

//void AccelerometerModel::getDerivative(StateVector &x_dot, const State &state)
//{
//  x_dot.setZero();
//  if (state.velocity() && !state.acceleration()) {
//    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
//      state.velocity()->segment(x_dot)(X) = bias_nav_.x();
//      state.velocity()->segment(x_dot)(Y) = bias_nav_.y();
//    }
//    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
//      state.velocity()->segment(x_dot)(Z) = bias_nav_.z();
//    }
//  }
//}

//void AccelerometerModel::getStateJacobian(SystemMatrixBlock& A, const State& state)
//{
//  A.setZero();
//  if (state.velocity() && !state.acceleration()) {
//    const State::RotationMatrix &R = state.R();

//    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
//      state.velocity()->block(A, *bias_).row(X) = R.row(X);
//      state.velocity()->block(A, *bias_).row(Y) = R.row(Y);
//    }
//    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
//      state.velocity()->block(A, *bias_).row(Z) = R.row(Z);
//    }

//    if (state.getSystemStatus() & STATE_VELOCITY_XY) {
//      state.velocity()->block(A, *state.orientation())(X,X) = 0.0;
//      state.velocity()->block(A, *state.orientation())(X,Y) =  bias_nav_.z();
//      state.velocity()->block(A, *state.orientation())(X,Z) = -bias_nav_.y();

//      state.velocity()->block(A, *state.orientation())(Y,X) = -bias_nav_.z();
//      state.velocity()->block(A, *state.orientation())(Y,Y) = 0.0;
//      state.velocity()->block(A, *state.orientation())(Y,Z) =  bias_nav_.x();
//    }

//    if (state.getSystemStatus() & STATE_VELOCITY_Z) {
//      state.velocity()->block(A, *state.orientation())(Z,X) =  bias_nav_.y();
//      state.velocity()->block(A, *state.orientation())(Z,Y) = -bias_nav_.x();
//      state.velocity()->block(A, *state.orientation())(Z,Z) = 0.0;
//    }
//  }
//}

} // namespace hector_pose_estimation
