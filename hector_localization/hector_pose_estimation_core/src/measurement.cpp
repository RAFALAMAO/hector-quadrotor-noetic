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

#include <hector_pose_estimation/measurement.h>

#include <ros/console.h>

namespace hector_pose_estimation {

Measurement::Measurement(const std::string& name)
  : name_(name)
  , status_flags_(0)
  , enabled_(true)
  , min_interval_(0.0)
  , timeout_(0.0)
  , timer_(0.0)
{
  parameters().add("enabled", enabled_);
  parameters().add("timeout", timeout_);
  parameters().add("min_interval", min_interval_);
  parameters().add("timeout", timeout_);
}

Measurement::~Measurement()
{
}

bool Measurement::init(PoseEstimation& estimator, State& state)
{
  if (getModel() && !getModel()->init(estimator, *this, state)) return false;
  if (!onInit(estimator)) return false;
  return true;
}

void Measurement::cleanup()
{
  if (getModel()) getModel()->cleanup();
  onCleanup();
}

void Measurement::reset(State& state)
{
  queue().clear();
  timer_ = 0;
  status_flags_ = 0;

  if (getModel()) getModel()->reset(state);
  onReset();
}

bool Measurement::active(const State& state) {
  bool active = enabled() && (getModel() ? getModel()->active(state) : !(state.getSystemStatus() & STATUS_ALIGNMENT));
  if (!active) status_flags_ = 0;
  if (min_interval_ > 0.0 && timer_ < min_interval_) return false;
  return active;
}

void Measurement::increase_timer(double dt) {
  timer_ += dt;
}

bool Measurement::timedout() const {
  return timeout_ > 0.0 && timer_ > timeout_;
}

void Measurement::add(const MeasurementUpdate& update) {
  queue().push(update);
}

bool Measurement::process() {
  bool result = true;

  while(!(queue().empty())) {
    result &= update(queue().pop());
  }

  // check for timeout
  if (timedout()) {
    if (status_flags_ > 0) ROS_WARN("Measurement %s timed out.", getName().c_str());
    status_flags_ = 0;
  }
  return result;
}

bool Measurement::update(const MeasurementUpdate &update)
{
  if (!filter() || !active(filter()->state())) return false;

  if (!updateImpl(update)) return false;
  filter()->state().updated();

  timer_ = 0.0;
  if (getModel()) status_flags_ = getModel()->getStatusFlags();
  return true;
}

} // namespace hector_pose_estimation
