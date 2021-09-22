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

#include <hector_pose_estimation/filter.h>
#include <hector_pose_estimation/pose_estimation.h>

#ifdef USE_HECTOR_TIMING
  #include <hector_diagnostics/timing.h>
#endif

namespace hector_pose_estimation {

Filter::Filter(State &state)
  : state_(state)
{
}

Filter::~Filter()
{
}

bool Filter::init(PoseEstimation& estimator)
{
  return true;
}

void Filter::cleanup()
{
}

void Filter::reset()
{
  state_.reset();
}

bool Filter::preparePredict(double)
{
  return true;
}

bool Filter::predict(const Systems& systems, double dt) {
  bool result = true;

#ifdef USE_HECTOR_TIMING
  hector_diagnostics::TimingSection section("predict");
#endif

  if (!preparePredict(dt)) return false;

  // Iterate through system models. For an EKF, this will populate the x_diff vector, A and Q matrices.
  for(Systems::iterator it = systems.begin(); it != systems.end(); it++) {
    const SystemPtr& system = *it;
    result &= predict(system, dt);
  }

  // Call the filter's global predict method. This will actually calculate the updated state vector and variance.
  result &= doPredict(dt);

  return result;
}

bool Filter::predict(const SystemPtr& system, double dt) {
#ifdef USE_HECTOR_TIMING
  hector_diagnostics::TimingSection section("predict." + system->getName());
#endif
  return system->update(dt);
}

bool Filter::doPredict(double dt) {
  // already done in System::update()
  // state_.updated();
  return true;
}

bool Filter::prepareCorrect()
{
  return true;
}

bool Filter::correct(const Measurements& measurements) {
  bool result = true;

#ifdef USE_HECTOR_TIMING
  hector_diagnostics::TimingSection section("correct");
#endif

  if (!prepareCorrect()) return false;

  // Iterate through measurement models. This will process the correction step directly.
  for(Measurements::iterator it = measurements.begin(); it != measurements.end(); it++) {
    const MeasurementPtr& measurement = *it;
    result &= correct(measurement);
  }

  // Call the filter's global correct method. No-op for EKF.
  result &= doCorrect();

  return result;
}

bool Filter::correct(const MeasurementPtr& measurement) {
#ifdef USE_HECTOR_TIMING
  hector_diagnostics::TimingSection section("correct." + measurement->getName());
#endif
  return measurement->process();
}

bool Filter::doCorrect() {
  // already done in Measurement::update()
  // state_.updated();
  return true;
}

} // namespace hector_pose_estimation
