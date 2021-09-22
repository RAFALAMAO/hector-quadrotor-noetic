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

#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <Eigen/Geometry>

namespace hector_pose_estimation {

template class Measurement_<MagneticModel>;

MagneticModel::MagneticModel()
  : declination_(0.0), inclination_(60.0 * M_PI/180.0), magnitude_(0.0)
{
  parameters().add("stddev", stddev_, 1.0);
  parameters().add("declination", declination_);
  parameters().add("inclination", inclination_);
  parameters().add("magnitude", magnitude_);
}

MagneticModel::~MagneticModel() {}

bool MagneticModel::init(PoseEstimation &estimator, Measurement &measurement, State &state)
{
  updateMagneticField();
  return true;
}

void MagneticModel::setReference(const GlobalReference::Heading &reference_heading) {
  magnetic_field_reference_.x() = reference_heading.cos * magnetic_field_north_.x() - reference_heading.sin * magnetic_field_north_.y();
  magnetic_field_reference_.y() = reference_heading.sin * magnetic_field_north_.x() + reference_heading.cos * magnetic_field_north_.y();
  magnetic_field_reference_.z() = magnetic_field_north_.z();
}

void MagneticModel::getMeasurementNoise(NoiseVariance& R, const State&, bool init)
{
  if (init) {
    R(0,0) = R(1,1) = R(2,2) = pow(stddev_, 2);
  }
}

void MagneticModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  const State::RotationMatrix &R = state.R();
  y_pred = R.transpose() * magnetic_field_reference_;
}

void MagneticModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  if (state.orientation()) {
    const State::RotationMatrix &R = state.R();
    state.orientation()->cols(C)(X,Z) = R(0,0) * magnetic_field_reference_.y() - R(1,0) * magnetic_field_reference_.x();
    state.orientation()->cols(C)(Y,Z) = R(0,1) * magnetic_field_reference_.y() - R(1,1) * magnetic_field_reference_.x();
    state.orientation()->cols(C)(Z,Z) = R(0,2) * magnetic_field_reference_.y() - R(1,2) * magnetic_field_reference_.x();
  }
}

double MagneticModel::getMagneticHeading(const State& state, const MeasurementVector &y) const {
  MeasurementVector y_nav;
  y_nav = state.R() * y;
  return atan2(y_nav.y(), y_nav.x()) - state.getYaw();
}

double MagneticModel::getTrueHeading(const State& state, const MeasurementVector &y) const {
  return getMagneticHeading(state, y) + declination_;
}

void MagneticModel::updateMagneticField()
{
  double cos_inclination, sin_inclination;
  sincos(inclination_, &sin_inclination, &cos_inclination);

  double cos_declination, sin_declination;
  sincos(declination_, &sin_declination, &cos_declination);

  // return normalized magnetic field if magnitude is zero
  double magnitude = magnitude_;
  if (magnitude == 0.0) magnitude = 1.0;

  magnetic_field_north_.x() = magnitude * (cos_inclination *   cos_declination);
  magnetic_field_north_.y() = magnitude * (cos_inclination * (-sin_declination));
  magnetic_field_north_.z() = magnitude * (-sin_inclination);
}

Magnetic::Magnetic(const std::string &name)
  : Measurement_<MagneticModel>(name)
  , auto_heading_(true)
  , deviation_(3)
{
  deviation_.setZero();
  parameters().add("auto_heading", auto_heading_);
  parameters().add("deviation", deviation_);
}

void Magnetic::onReset() {
  reference_.reset();
}

const MagneticModel::MeasurementVector& Magnetic::getVector(const Magnetic::Update& update, const State& state) {
  y_ = Measurement_<MagneticModel>::getVector(update, state) + deviation_;
  if (getModel()->hasMagnitude()) return y_;

  double norm = y_.norm();
  if (norm < 1e-5) {
    y_.setZero();
  } else {
    y_ = y_ / norm;
  }
  return y_;
}

//const MagneticModel::NoiseVariance& Magnetic::getVariance(const Magnetic::Update& update, const State& state) {
//  if (getModel()->hasMagnitude()) return Measurement_<MagneticModel>::getVariance(update, state);

//  R_ = Measurement_<MagneticModel>::getVariance(update, state);
//  double norm = Measurement_<MagneticModel>::getVector(update, state).norm();
//  if (norm < 1e-5) {
//    R_ = NoiseVariance(1.0, 1.0, 1.0);
//  } else {
//    R_ =  R_ / (norm*norm);
//  }
//  return R_;
//}

bool Magnetic::prepareUpdate(State &state, const Update &update) {
  // reset reference position if Magnetic has not been updated for a while
  if (timedout()) reference_.reset();

  if (reference_ != GlobalReference::Instance()) {
    reference_ = GlobalReference::Instance();
    if (auto_heading_) reference_->setCurrentHeading(state, getModel()->getTrueHeading(state, update.getVector()));
  }

  getModel()->setReference(reference_->heading());
  return true;
}

} // namespace hector_pose_estimation
