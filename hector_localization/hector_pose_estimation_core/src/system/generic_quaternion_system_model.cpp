//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <hector_pose_estimation/pose_estimation.h>

namespace hector_pose_estimation {

template class System_<GenericQuaternionSystemModel>;

GenericQuaternionSystemModel::GenericQuaternionSystemModel()
{
  angular_acceleration_stddev_ = 360.0 * M_PI/180.0;
  rate_stddev_ = 0.0;
  acceleration_stddev_ = 0.0;
  velocity_stddev_ = 0.0;

//  parameters().addAlias("gravity", gravity_);
  parameters().add("angular_acceleration_stddev", angular_acceleration_stddev_);
  parameters().add("rate_stddev", rate_stddev_);
  parameters().add("acceleration_stddev", acceleration_stddev_);
  parameters().add("velocity_stddev", velocity_stddev_);
}

GenericQuaternionSystemModel::~GenericQuaternionSystemModel()
{
}

bool GenericQuaternionSystemModel::init(PoseEstimation& estimator, System &system, State& state)
{
  gravity_ = estimator.parameters().get("gravity_magnitude");

  imu_ = estimator.getInputType<ImuInput>("imu");
  if (imu_ && state.orientation()) {
    gyro_ = estimator.getSystem_<Gyro>("gyro");
    if (!gyro_) {
      gyro_.reset(new Gyro("gyro"));
      estimator.addSystem(gyro_);
    }
  }
  if (imu_ && state.velocity()) {
    accelerometer_ = estimator.getSystem_<Accelerometer>("accelerometer");
    if (!accelerometer_) {
      accelerometer_.reset(new Accelerometer("accelerometer"));
      estimator.addSystem(accelerometer_);
    }

  }

//  // precalculate Q in order to just copy the parts that are active
//  // Note: Q might be too small as we add other substates later.
//  Q_ = State::Covariance::Zero(state.getCovarianceDimension(), state.getCovarianceDimension());
//  if (state.orientation()) {
//    if (!state.rate() && imu_ && gyro_) {
//      gyro_->getModel()->getRateNoise(state.orientation()->block(Q_), state, true);
//    }
//    state.orientation()->block(Q_) += pow(rate_stddev_, 2) * SymmetricMatrix3::Identity();
//  }
//  if (state.rate()) {
//    state.rate()->block(Q_) = pow(angular_acceleration_stddev_, 2) * SymmetricMatrix3::Identity();
//  }
//  if (state.position()) {
//    state.position()->block(Q_) = pow(velocity_stddev_, 2) * SymmetricMatrix3::Identity();
//  }
//  if (state.velocity()) {
//    if (!state.acceleration() && imu_ && accelerometer_) {
//      accelerometer_->getModel()->getAccelerationNoise(state.velocity()->block(Q_), state, true);
//    }
//    state.velocity()->block(Q_) += pow(acceleration_stddev_, 2) * SymmetricMatrix3::Identity();
//  }

  return true;
}

void GenericQuaternionSystemModel::getPrior(State &state) {
  if (state.orientation()) {
    state.orientation()->P()(X,X) = 1.0;
    state.orientation()->P()(Y,Y) = 1.0;
    state.orientation()->P()(Z,Z) = 0.0;
  }

  if (state.rate()) {
    state.rate()->P()(X,X) = pow(0.0 * M_PI/180.0, 2);
    state.rate()->P()(Y,Y) = pow(0.0 * M_PI/180.0, 2);
    state.rate()->P()(Z,Z) = pow(0.0 * M_PI/180.0, 2);
  }

  if (state.position()) {
    state.position()->P()(X,X) = 0.0;
    state.position()->P()(Y,Y) = 0.0;
    state.position()->P()(Z,Z) = 0.0;
  }

  if (state.velocity()) {
    state.velocity()->P()(X,X) = 0.0;
    state.velocity()->P()(Y,Y) = 0.0;
    state.velocity()->P()(Z,Z) = 0.0;
  }
}

bool GenericQuaternionSystemModel::prepareUpdate(State& state, double dt)
{
  if (state.rate()) {
    rate_nav_ = state.R() * state.getRate();
  }
  else if (rate_input_) {
    rate_nav_ = state.R() * rate_input_->getVector();
  }
  else if (imu_) {
    if (gyro_) {
      rate_nav_ = state.R() * gyro_->getModel()->getRate(imu_->getRate(), state);
    } else {
      rate_nav_ = state.R() * imu_->getRate();
    }
  } else {
    rate_nav_.setZero();
  }

  if (state.acceleration()) {
    acceleration_nav_ = state.R() * state.getAcceleration();
  }
  else if (force_input_) {
    acceleration_nav_ = state.R() * force_input_->getVector();
  }
  else if (imu_) {
    if (accelerometer_) {
      acceleration_nav_ = state.R() * accelerometer_->getModel()->getAcceleration(imu_->getAcceleration(), state);
    } else {
      acceleration_nav_ = state.R() * imu_->getAcceleration();
    }
  } else {
    acceleration_nav_.setZero();
  }

  ROS_DEBUG_STREAM_NAMED("system", "rate_nav = [" << rate_nav_.transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("system", "acceleration_nav = [" << acceleration_nav_.transpose() << "]");
  return true;
}

void GenericQuaternionSystemModel::getDerivative(StateVector& x_dot, const State& state)
{
  x_dot.setZero();

  if (state.rate()) {
    if (torque_input_) {
      state.rate()->segment(x_dot) = torque_input_->getVector();
    }
  }

  if (state.orientation()) {
    state.orientation()->segment(x_dot).head(3) = rate_nav_;
    if (!(state.getSystemStatus() & STATE_YAW) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.orientation()->segment(x_dot).z() = 0.0;
    }
  }

  if (state.velocity()) {
    if ((state.getSystemStatus() & STATE_VELOCITY_XY) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.velocity()->segment(x_dot)(X) = acceleration_nav_.x();
      state.velocity()->segment(x_dot)(Y) = acceleration_nav_.y();
    }
    if ((state.getSystemStatus() & STATE_VELOCITY_Z) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.velocity()->segment(x_dot)(Z) = acceleration_nav_.z();
      if (imu_) {
        state.velocity()->segment(x_dot)(Z) += gravity_;
      }
    }
  }

  if (state.position()) {
    State::ConstVelocityType v(state.getVelocity());
    if ((state.getSystemStatus() & STATE_POSITION_XY) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.position()->segment(x_dot)(X) = v.x();
      state.position()->segment(x_dot)(Y) = v.y();
    }
    if ((state.getSystemStatus() & STATE_POSITION_Z) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.position()->segment(x_dot)(Z) = v.z();
    }
  }
}

void GenericQuaternionSystemModel::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
//  if (init) Q.setZero();
//  Q.topLeftCorner(Q_.rows(), Q_.cols()) = Q_;

//  if (state.orientation()) {
//    if (!(state.getSystemStatus() & STATE_YAW) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
//      state.orientation()->block(Q)(Z,Z) = 0.0;
//    }
//  }

//  if (state.velocity()) {
//    if (!(state.getSystemStatus() & STATE_VELOCITY_XY) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
//      state.velocity()->block(Q)(X,X) = 0.0;
//      state.velocity()->block(Q)(Y,Y) = 0.0;
//    }
//    if (!(state.getSystemStatus() & STATE_VELOCITY_Z) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
//      state.velocity()->block(Q)(Z,Z) = 0.0;
//    }
//  }

//  if (state.position()) {
//    if (!(state.getSystemStatus() & STATE_POSITION_XY) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
//      state.position()->block(Q)(X,X) = 0.0;
//      state.position()->block(Q)(Y,Y) = 0.0;
//    }
//    if (!(state.getSystemStatus() & STATE_POSITION_Z) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
//      state.position()->block(Q)(Z,Z) = 0.0;
//    }
//  }

  if (!init) return;

  Q.setZero();
  if (state.orientation()) {
    if (!state.rate() && imu_ && gyro_) {
      gyro_->getModel()->getRateNoise(state.orientation()->block(Q), state, init);
    }
    state.orientation()->block(Q) += pow(rate_stddev_, 2) * SymmetricMatrix3::Identity();
  }
  if (state.rate()) {
    state.rate()->block(Q) = pow(angular_acceleration_stddev_, 2) * SymmetricMatrix3::Identity();
  }
  if (state.position()) {
    state.position()->block(Q) = pow(velocity_stddev_, 2) * SymmetricMatrix3::Identity();
  }
  if (state.velocity()) {
    if (!state.acceleration() && imu_ && accelerometer_) {
      accelerometer_->getModel()->getAccelerationNoise(state.velocity()->block(Q), state, init);
    }
    state.velocity()->block(Q) += pow(acceleration_stddev_, 2) * SymmetricMatrix3::Identity();
  }
}

void GenericQuaternionSystemModel::getStateJacobian(SystemMatrix& A, const State& state, bool)
{
  const State::RotationMatrix &R = state.R();
  A.setZero();

  if (state.orientation()) {
    if (state.rate()) {
      state.orientation()->block(A, *state.rate()) = R;
    } else if (imu_ && gyro_) {
      GyroModel::SystemMatrixBlock A_orientation = state.orientation()->rows(A);
      gyro_->getModel()->getRateJacobian(A_orientation, state);
      state.orientation()->rows(A) = R * A_orientation;
    }

    state.orientation()->block(A) += SkewSymmetricMatrix(-rate_nav_);

    if (!(state.getSystemStatus() & STATE_YAW) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.orientation()->rows(A).row(2).setZero();
      state.orientation()->block(A).col(2).setZero();
    }
  }

  if (state.velocity()) {
    if (state.acceleration()) {
      state.velocity()->block(A, *state.acceleration()) = R;
    } else if (imu_ && accelerometer_) {
      AccelerometerModel::SystemMatrixBlock A_velocity = state.velocity()->rows(A);
      accelerometer_->getModel()->getAccelerationJacobian(A_velocity, state);
      state.velocity()->rows(A) = R * A_velocity;
    }

    if (state.orientation()) {
      state.velocity()->block(A, *state.orientation()) += SkewSymmetricMatrix(-acceleration_nav_);
    }

    if (!(state.getSystemStatus() & STATE_VELOCITY_XY) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.velocity()->rows(A).topRows(2).setZero();
    }

    if (!(state.getSystemStatus() & STATE_VELOCITY_Z) || (state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.velocity()->rows(A).row(2).setZero();
    }
  }

  if (state.position() && state.velocity()) {
    state.position()->block(A, *state.velocity()).setIdentity();

    if ((state.getSystemStatus() & STATE_POSITION_XY) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.position()->block(A, *state.velocity())(X,X) = 1.0;
      state.position()->block(A, *state.velocity())(Y,Y) = 1.0;
    }
    if ((state.getSystemStatus() & STATE_POSITION_Z) && !(state.getSystemStatus() & STATUS_ALIGNMENT)) {
      state.position()->block(A, *state.velocity())(Z,Z) = 1.0;
    }
  }

}

SystemStatus GenericQuaternionSystemModel::getStatusFlags(const State& state)
{
  SystemStatus flags = state.getMeasurementStatus();
  if (flags & STATE_POSITION_XY) flags |= STATE_VELOCITY_XY;
  if (flags & STATE_POSITION_Z)  flags |= STATE_VELOCITY_Z;
  if (imu_) {
    if (flags & STATE_VELOCITY_XY)      flags |= STATE_ROLLPITCH;
    if (flags & STATE_ROLLPITCH)        flags |= STATE_RATE_XY;
    if (flags & STATE_PSEUDO_ROLLPITCH) flags |= STATE_PSEUDO_RATE_XY;
    if (flags & STATE_YAW)              flags |= STATE_RATE_Z;
    if (flags & STATE_PSEUDO_YAW)       flags |= STATE_PSEUDO_RATE_Z;
  }
  return flags & STATE_MASK;
}

} // namespace hector_pose_estimation
