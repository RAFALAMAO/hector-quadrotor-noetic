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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_SET_FILTER_H
#define HECTOR_POSE_ESTIMATION_FILTER_SET_FILTER_H

#include <hector_pose_estimation/system.h>
#include <hector_pose_estimation/measurement.h>

#include <hector_pose_estimation/filter/ekf.h>

#include <ros/console.h>

namespace hector_pose_estimation {

template <class ConcreteModel>
void System_<ConcreteModel>::setFilter(Filter *filter) {
  if (filter->derived<filter::EKF>()) {
    predictor_ = Filter::factory(filter->derived<filter::EKF>()).addPredictor<ConcreteModel>(this->getModel());
  } else {
    ROS_ERROR_NAMED(getName(), "Unknown filter type: %s", filter->getType().c_str());
  }
}

template <class ConcreteModel>
void Measurement_<ConcreteModel>::setFilter(Filter *filter) {
  if (filter->derived<filter::EKF>()) {
    corrector_ = Filter::factory(filter->derived<filter::EKF>()).addCorrector<ConcreteModel>(this->getModel());
  } else {
    ROS_ERROR_NAMED(getName(), "Unknown filter type: %s", filter->getType().c_str());
  }
}

}

#endif // HECTOR_POSE_ESTIMATION_FILTER_SET_FILTER_H
