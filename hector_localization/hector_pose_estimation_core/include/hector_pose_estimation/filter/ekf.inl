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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_EKF_INL
#define HECTOR_POSE_ESTIMATION_FILTER_EKF_INL

#include <hector_pose_estimation/filter/ekf.h>
#include <boost/utility/enable_if.hpp>

#include <ros/console.h>

namespace hector_pose_estimation {
namespace filter {

template <class ConcreteModel, typename Enabled>
bool EKF::Predictor_<ConcreteModel, Enabled>::predict(double dt) {
  this->model_->getExpectedDiff(x_diff, state(), dt);
  this->model_->getStateJacobian(A, state(), dt, this->init_);
  this->model_->getSystemNoise(Q, state(), dt, this->init_);

  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "dt * f(x) = [" << x_diff.transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "dt * Q    = [" << std::endl << Q  << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "dt * A    = [" << std::endl << A << "]");

//  state().P0() = A * state().P() * A.transpose() + Q;
//  state().P0().assertSymmetric();
//  state().x0() = x_pred;

  this->init_ = false;
  return true;
}

template <class ConcreteModel, typename Enabled>
bool EKF::Corrector_<ConcreteModel, Enabled>::correct(const typename ConcreteModel::MeasurementVector &y, const typename ConcreteModel::NoiseVariance &R) {
  this->model_->getExpectedValue(y_pred, state());
  this->model_->getStateJacobian(C, state(), this->init_);

  ROS_DEBUG_STREAM_NAMED("ekf.correction", "x_prior  = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "P_prior  = [" << std::endl << state().getCovariance() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "y        = [" << y.transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "R        = [" << std::endl << R << "]");

  ROS_DEBUG_STREAM_NAMED("ekf.correction", "h(x)     = [" << y_pred.transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "C        = [" << std::endl << C << "]");

  CP.noalias() = C * state().P();
  S.noalias()  = CP * C.transpose() + R;
  K.noalias() = CP.transpose() * S.inverse();
  state().P().noalias() -= K * CP;
  state().P().assertSymmetric();

  error.noalias() = y - y_pred;
  this->model_->limitError(error);
  update.noalias() = K * error;

  state().update(update);

  ROS_DEBUG_STREAM_NAMED("ekf.correction", "S        = [" << std::endl << S << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "K        = [" << std::endl << K << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "error    = [" << error.transpose() << "]");

  ROS_DEBUG_STREAM_NAMED("ekf.correction", "x_post   = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.correction", "P_post   = [" << std::endl << state().getCovariance() << "]");

  this->init_ = false;
  return true;
}

} // namespace filter
} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_INL
