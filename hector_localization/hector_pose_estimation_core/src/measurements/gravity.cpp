//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include <hector_pose_estimation/measurements/gravity.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/set_filter.h>

namespace hector_pose_estimation {

template class Measurement_<GravityModel>;

GravityModel::GravityModel()
  : gravity_(MeasurementVector::Zero())
{
  parameters().add("stddev", stddev_, 1.0);
  parameters().add("use_bias", use_bias_, std::string("accelerometer_bias"));
}

GravityModel::~GravityModel() {}

bool GravityModel::init(PoseEstimation &estimator, Measurement &measurement, State &state) {
  if (!use_bias_.empty()) {
    bias_ = state.getSubState<3,3>(use_bias_);
    if (!bias_) {
      ROS_ERROR("Could not find bias substate '%s' during initialization of gravity measurement '%s'.", use_bias_.c_str(), measurement.getName().c_str());
      return false;
    }
  } else {
    bias_.reset();
  }

  setGravity(estimator.parameters().getAs<double>("gravity_magnitude"));
  return true;
}


void GravityModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = R(2,2) = pow(stddev_, 2);
  }
}

void GravityModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  const State::RotationMatrix &R = state.R();
  y_pred = -R.row(2).transpose() * gravity_.z();
  if (bias_) {
    y_pred += bias_->getVector();
  }
}

void GravityModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  const State::RotationMatrix &R = state.R();
  if (state.orientation()) {
//    C(0,state.getOrientationCovarianceIndex() + W) =  gravity_.z()*2*q.y();
//    C(0,state.getOrientationCovarianceIndex() + X) = -gravity_.z()*2*q.z();
//    C(0,state.getOrientationCovarianceIndex() + Y) =  gravity_.z()*2*q.w();
//    C(0,state.getOrientationCovarianceIndex() + Z) = -gravity_.z()*2*q.x();
//    C(1,state.getOrientationCovarianceIndex() + W) = -gravity_.z()*2*q.x();
//    C(1,state.getOrientationCovarianceIndex() + X) = -gravity_.z()*2*q.w();
//    C(1,state.getOrientationCovarianceIndex() + Y) = -gravity_.z()*2*q.z();
//    C(1,state.getOrientationCovarianceIndex() + Z) = -gravity_.z()*2*q.y();
//    C(2,state.getOrientationCovarianceIndex() + W) = -gravity_.z()*2*q.w();
//    C(2,state.getOrientationCovarianceIndex() + X) =  gravity_.z()*2*q.x();
//    C(2,state.getOrientationCovarianceIndex() + Y) =  gravity_.z()*2*q.y();
//    C(2,state.getOrientationCovarianceIndex() + Z) = -gravity_.z()*2*q.z();

     state.orientation()->cols(C)(X,X) = -gravity_.z() * R(1,0);
     state.orientation()->cols(C)(X,Y) =  gravity_.z() * R(0,0);
     state.orientation()->cols(C)(Y,X) = -gravity_.z() * R(1,1);
     state.orientation()->cols(C)(Y,Y) =  gravity_.z() * R(0,1);
     state.orientation()->cols(C)(Z,X) = -gravity_.z() * R(1,2);
     state.orientation()->cols(C)(Z,Y) =  gravity_.z() * R(0,2);
  }

//  Only the bias component in direction of the gravity is observable, under the assumption that we do not accelerate vertically.
  if (bias_) {
    bias_->cols(C) = R.row(2).transpose() * R.row(2);
  }
}

} // namespace hector_pose_estimation
