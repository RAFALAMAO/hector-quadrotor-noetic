//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
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

#include <hector_pose_estimation/system/ground_vehicle_model.h>
#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/set_filter.h>

#include <limits>

namespace hector_pose_estimation {

template class System_<GroundVehicleModel>;

GroundVehicleModel::GroundVehicleModel()
{
  gain_ = 1.0;
  base_height_ = 0.0;
  min_height_ = -std::numeric_limits<double>::quiet_NaN();
  max_height_ =  std::numeric_limits<double>::quiet_NaN();

  parameters().add("gain", gain_);
  parameters().add("base_height", base_height_);
  parameters().add("min_height", min_height_);
  parameters().add("max_height", max_height_);

  // derivative of the 3rd column of the rotation matrix
  dR3 <<  0.0, 1.0, 0.0,
         -1.0, 0.0, 0.0,
          0.0, 0.0, 0.0;
}

GroundVehicleModel::~GroundVehicleModel()
{
}

void GroundVehicleModel::getPrior(State &state)
{
  GenericQuaternionSystemModel::getPrior(state);
  if (state.position()) state.position()->vector().z() = base_height_;
}

void GroundVehicleModel::getDerivative(StateVector& x_dot, const State& state)
{
  // forward to GenericQuaternionSystemModel
  GenericQuaternionSystemModel::getDerivative(x_dot, state);

  const State::RotationMatrix &R = state.R();
  State::ConstVelocityType v(state.getVelocity());

  // Update the body z velocity towards 0
  if (state.velocity()) {
    // v_z_body = R.col(2).dot(v)
    state.velocity()->segment(x_dot) += -gain_ * R.col(2) * (R.col(2).dot(v));
  }
}

void GroundVehicleModel::getStateJacobian(SystemMatrix& A, const State& state, bool init)
{
  GenericQuaternionSystemModel::getStateJacobian(A, state, init);

  const State::RotationMatrix &R = state.R();
  State::ConstVelocityType v(state.getVelocity());

  if (state.velocity()) {
    state.velocity()->block(A) += -gain_ * R.col(2) * R.col(2).transpose();

    if (state.orientation()) {
      state.velocity()->block(A, *state.orientation()) += -gain_ * (dR3 * (R.col(2).dot(v)) + R.col(2) * (v.transpose() * dR3));
    }
  }
}

SystemStatus GroundVehicleModel::getStatusFlags(const State& state)
{
  SystemStatus flags = GenericQuaternionSystemModel::getStatusFlags(state);
  if (flags & STATE_VELOCITY_XY) {
    flags |= STATE_VELOCITY_Z;
    flags |= STATE_POSITION_Z;
  }
  return flags;
}

bool GroundVehicleModel::limitState(State& state)
{
  bool result = GenericQuaternionSystemModel::limitState(state);
  if (state.position()) {
    if (state.position()->vector().z() < min_height_) {
      state.position()->vector().z() = min_height_;
      result = false;
    }
    if (state.position()->vector().z() > max_height_) {
      state.position()->vector().z() = max_height_;
      result = false;
    }
  }
  return result;
}

} // namespace hector_pose_estimation
