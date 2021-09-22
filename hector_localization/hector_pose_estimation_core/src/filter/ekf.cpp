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

#include <hector_pose_estimation/filter/ekf.h>
#include <hector_pose_estimation/system.h>

#include <boost/pointer_cast.hpp>

#ifdef USE_HECTOR_TIMING
  #include <hector_diagnostics/timing.h>
#endif

namespace hector_pose_estimation {
namespace filter {

EKF::EKF(State &state)
  : Filter(state)
{}

EKF::~EKF()
{}

bool EKF::init(PoseEstimation &estimator)
{
  x_diff = State::Vector(state_.getVectorDimension());
  A = State::SystemMatrix(state_.getCovarianceDimension(), state_.getCovarianceDimension());
  Q = State::Covariance(state_.getCovarianceDimension(), state_.getCovarianceDimension());
  return true;
}

bool EKF::preparePredict(double dt)
{
  x_diff.setZero();
  A.setIdentity();
  Q.setZero();
  return Filter::preparePredict(dt);
}

bool EKF::predict(const SystemPtr& system, double dt)
{
  if (!Filter::predict(system, dt)) return false;
  EKF::Predictor *predictor = boost::dynamic_pointer_cast<EKF::Predictor>(system->predictor());
  x_diff += predictor->x_diff;
  A += predictor->A;
  Q += predictor->Q;
  return true;
}

bool EKF::doPredict(double dt) {
  ROS_DEBUG_NAMED("ekf.prediction", "EKF prediction (dt = %f):", dt);

  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "A      = [" << std::endl << A << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "Q      = [" << std::endl << Q << "]");

#ifdef USE_HECTOR_TIMING
  { hector_diagnostics::TimingSection section("predict.ekf.covariance");
#endif
  state().P() = A * state().P() * A.transpose() + Q;
  state().P().assertSymmetric();

#ifdef USE_HECTOR_TIMING
  }
  { hector_diagnostics::TimingSection section("predict.ekf.state");
#endif
  state().update(x_diff);

#ifdef USE_HECTOR_TIMING
  }
#endif

  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "x_pred = [" << state().getVector().transpose() << "]");
  ROS_DEBUG_STREAM_NAMED("ekf.prediction", "P_pred = [" << std::endl << state().getCovariance() << "]");

  Filter::doPredict(dt);
  return true;
}

} // namespace filter
} // namespace hector_pose_estimation
