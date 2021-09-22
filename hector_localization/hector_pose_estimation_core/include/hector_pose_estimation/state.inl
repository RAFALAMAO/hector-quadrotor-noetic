//=================================================================================================
// Copyright (c) 2014, Johannes Meyer and contributors, Technische Universitat Darmstadt
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

#ifndef HECTOR_POSE_ESTIMATION_STATE_INL
#define HECTOR_POSE_ESTIMATION_STATE_INL

#include <hector_pose_estimation/state.h>

namespace hector_pose_estimation {

template <typename Derived>
void State::setOrientation(const Eigen::MatrixBase<Derived>& orientation) {
  eigen_assert(orientation.rows() == 4 && orientation.cols() == 1);
  fake_orientation_ = orientation;
}

template <typename Derived>
void State::setRate(const Eigen::MatrixBase<Derived>& rate) {
  eigen_assert(rate.rows() == 3 && rate.cols() == 1);
  fake_rate_ = rate;
}

template <typename Derived>
void State::setPosition(const Eigen::MatrixBase<Derived>& position) {
  eigen_assert(position.rows() == 3 && position.cols() == 1);
  fake_position_ = position;
}

template <typename Derived>
void State::setVelocity(const Eigen::MatrixBase<Derived>& velocity) {
  eigen_assert(velocity.rows() == 3 && velocity.cols() == 1);
  fake_velocity_ = velocity;
}

template <typename Derived>
void State::setAcceleration(const Eigen::MatrixBase<Derived>& acceleration) {
  eigen_assert(acceleration.rows() == 3 && acceleration.cols() == 1);
  fake_acceleration_ = acceleration;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_STATE_INL
