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

#include <hector_pose_estimation/measurements/baro.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <boost/bind.hpp>

namespace hector_pose_estimation {

template class Measurement_<BaroModel>;

BaroModel::BaroModel()
{
  stddev_ = 1.0;
  qnh_ = 1013.25;
  parameters().add("qnh", qnh_);
}

BaroModel::~BaroModel() {}

void BaroModel::getExpectedValue(MeasurementVector& y_pred, const State& state)
{
  y_pred(0) = qnh_ * pow(1.0 - (0.0065 * (state.getPosition().z() + getElevation())) / 288.15, 5.255);
}

void BaroModel::getStateJacobian(MeasurementMatrix& C, const State& state, bool)
{
  if (state.position()) {
    state.position()->cols(C)(0,Z) = qnh_ * 5.255 * pow(1.0 - (0.0065 * (state.getPosition().z() + getElevation())) / 288.15, 4.255) * (-0.0065 / 288.15);
  }
}

double BaroModel::getAltitude(const BaroUpdate& update)
{
  return 288.15 / 0.0065 * (1.0 - pow(update.getVector()(0) / qnh_, 1.0/5.255));
}

BaroUpdate::BaroUpdate() : qnh_(0) {}
BaroUpdate::BaroUpdate(double pressure) : qnh_(0) { *this = pressure; }
BaroUpdate::BaroUpdate(double pressure, double qnh) : qnh_(qnh) { *this = pressure; }

Baro::Baro(const std::string &name)
  : Measurement_<BaroModel>(name)
  , HeightBaroCommon(this)
{
  parameters().add("auto_elevation", auto_elevation_);
}

void Baro::onReset()
{
  HeightBaroCommon::onReset();
}

bool Baro::prepareUpdate(State &state, const Update &update) {
  if (update.qnh() != 0) setQnh(update.qnh());
  // Note: boost::bind is not real-time safe!
  setElevation(resetElevation(state, boost::bind(&BaroModel::getAltitude, getModel(), update)));
  return true;
}

} // namespace hector_pose_estimation
