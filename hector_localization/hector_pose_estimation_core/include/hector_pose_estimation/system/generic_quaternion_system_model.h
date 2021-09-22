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

#ifndef HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H
#define HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/system.h>

#include <hector_pose_estimation/system/imu_input.h>
#include <hector_pose_estimation/system/imu_model.h>

namespace hector_pose_estimation {

class GenericQuaternionSystemModel;

//namespace traits {
//  template <> struct Input<GenericQuaternionSystemModel> {
//    enum { Dimension = ImuInput::Dimension };
//    typedef ImuInput Type;
//    typedef ImuInput::Vector Vector;
//    typedef ImuInput::Variance Variance;
//  };
//} // namespace traits

class GenericQuaternionSystemModel : public TimeContinuousSystemModel_<GenericQuaternionSystemModel>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GenericQuaternionSystemModel();
  virtual ~GenericQuaternionSystemModel();

  virtual bool init(PoseEstimation& estimator, System &system, State& state);

  virtual void getPrior(State &state);

  virtual SystemStatus getStatusFlags(const State& state);

  bool prepareUpdate(State& state, double dt);

  using TimeContinuousSystemModel_<GenericQuaternionSystemModel>::getDerivative;
  virtual void getDerivative(StateVector& x_dot, const State& state);
  using TimeContinuousSystemModel_<GenericQuaternionSystemModel>::getSystemNoise;
  virtual void getSystemNoise(NoiseVariance& Q, const State& state, bool init = true);
  using TimeContinuousSystemModel_<GenericQuaternionSystemModel>::getStateJacobian;
  virtual void getStateJacobian(SystemMatrix& A, const State& state, bool init = true);

//  void setGravity(double gravity) { gravity_ = gravity; }
//  double getGravity() const { return gravity_; }

protected:
  AliasT<double> gravity_;
  double rate_stddev_;
  double acceleration_stddev_;
  double angular_acceleration_stddev_;
  double velocity_stddev_;

  boost::shared_ptr<ImuInput> imu_;
  boost::shared_ptr<Gyro> gyro_;
  boost::shared_ptr<Accelerometer> accelerometer_;

  ColumnVector3 rate_nav_;
  ColumnVector3 acceleration_nav_;
//  State::Covariance Q_;

  typedef Input_<3> TorqueInput;
  TorqueInput::Ptr torque_input_;
  typedef Input_<3> RateInput;
  RateInput::Ptr rate_input_;
  typedef Input_<3> ForceInput;
  ForceInput::Ptr force_input_;
};

extern template class System_<GenericQuaternionSystemModel>;

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_GENERIC_QUATERNION_SYSTEM_MODEL_H
