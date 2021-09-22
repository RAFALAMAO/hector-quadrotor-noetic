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

#ifndef HECTOR_POSE_ESTIMATION_TYPES_H
#define HECTOR_POSE_ESTIMATION_TYPES_H

#include <hector_pose_estimation/matrix.h>
#include <hector_pose_estimation/collection.h>

namespace hector_pose_estimation {

  enum VectorIndex {
    X = 0,
    Y = 1,
    Z = 2,
    W = 3
  };

  enum {
    STATUS_ALIGNMENT = 0x1,
    STATUS_DEGRADED = 0x2,
    STATUS_READY = 0x4,
    STATUS_MASK = 0xf,

    STATE_ROLLPITCH = 0x10,
    STATE_YAW = 0x20,
    STATE_RATE_XY = 0x100,
    STATE_RATE_Z = 0x200,
    STATE_VELOCITY_XY = 0x1000,
    STATE_VELOCITY_Z = 0x2000,
    STATE_POSITION_XY = 0x10000,
    STATE_POSITION_Z = 0x20000,
    STATE_MASK = 0x33330,

    STATE_PSEUDO_ROLLPITCH = 0x40,
    STATE_PSEUDO_YAW = 0x80,
    STATE_PSEUDO_RATE_XY = 0x400,
    STATE_PSEUDO_RATE_Z = 0x800,
    STATE_PSEUDO_VELOCITY_XY = 0x4000,
    STATE_PSEUDO_VELOCITY_Z = 0x8000,
    STATE_PSEUDO_POSITION_XY = 0x40000,
    STATE_PSEUDO_POSITION_Z = 0x80000,
    STATE_PSEUDO_MASK = 0xcccc0
  };
  typedef unsigned int SystemStatus;

  std::string getSystemStatusString(const SystemStatus& status, const SystemStatus& asterisk_status = 0);
  static inline std::ostream& operator<<(std::ostream& os, const SystemStatus& status) {
    return os << getSystemStatusString(status);
  }

  class Model;

  class SystemModel;
  class System;
  template <class Derived> class System_;
  typedef boost::shared_ptr<System> SystemPtr;
  typedef boost::weak_ptr<System> SystemWPtr;
  typedef Collection<System> Systems;

  class MeasurementModel;
  class MeasurementUpdate;
  class Measurement;
  template <class Derived> class Measurement_;
  typedef boost::shared_ptr<Measurement> MeasurementPtr;
  typedef boost::weak_ptr<Measurement> MeasurementWPtr;
  typedef Collection<Measurement> Measurements;

  class Input;
  typedef boost::shared_ptr<Input> InputPtr;
  typedef boost::weak_ptr<Input> InputWPtr;
  typedef Collection<Input> Inputs;

  class PoseEstimation;
  class Filter;
  typedef boost::shared_ptr<Filter> FilterPtr;
  class State;
  typedef boost::shared_ptr<State> StatePtr;

  class SubState;
  template <int VectorDimension, int CovarianceDimension> class SubState_;
  class BaseState;
  typedef boost::shared_ptr<SubState> SubStatePtr;
  typedef boost::weak_ptr<SubState> SubStateWPtr;

  class GlobalReference;
  typedef boost::shared_ptr<GlobalReference> GlobalReferencePtr;

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_TYPES_H
