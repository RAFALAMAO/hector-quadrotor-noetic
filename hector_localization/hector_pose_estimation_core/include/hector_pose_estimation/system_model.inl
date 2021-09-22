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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL
#define HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL

#include <hector_pose_estimation/system_model.h>

namespace hector_pose_estimation {

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void SystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getPrior(State &state)
{
  const double dt = 10.0;
  getSystemNoise(state.P(), state, dt, true);
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void SystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getExpectedDiff(StateVector& x_diff, const State& state, double dt)
{
  x_diff.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void SystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init)
{
  if (init) A.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void SystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getInputJacobian(InputMatrix& B, const State& state, double dt, bool init)
{
  if (init) B.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void SystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init)
{
  if (init) Q.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getDerivative(StateVector& x_dot, const State& state)
{
  x_dot.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getStateJacobian(SystemMatrix& A, const State& state, bool init)
{
  if (init) A.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getInputJacobian(InputMatrix& B, const State& state, bool init)
{
  if (init) B.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getSystemNoise(NoiseVariance& Q, const State& state, bool init)
{
  if (init) Q.setZero();
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
struct TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::internal {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typename ConcreteModel::StateVector x_diff;
  typename ConcreteModel::SystemMatrix A;
  typename ConcreteModel::InputMatrix B;
  typename ConcreteModel::NoiseVariance Q;

  internal(const State &state)
    : x_diff(state.getVectorDimension())
    , A(state.getCovarianceDimension(), state.getCovarianceDimension())
    , B(state.getCovarianceDimension(), IndexType(InputDimension))
    , Q(state.getCovarianceDimension(), state.getCovarianceDimension())
  {}
};

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::TimeContinuousSystemModel_()
  : internal_(0)
{}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::~TimeContinuousSystemModel_()
{
  delete internal_;
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getExpectedDiff(StateVector& x_diff, const State& state, double dt) {
  if (!internal_) internal_ = new internal(state);
  //internal_->x_diff = ColumnVector::Zero(x_diff.rows());
  getDerivative(internal_->x_diff, state);
  x_diff = dt * internal_->x_diff;
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getStateJacobian(SystemMatrix& A, const State& state, double dt, bool init) {
  if (!internal_) internal_ = new internal(state);
  //if (init) internal_->A = SystemMatrix::Zero(A.rows(), A.cols());
  getStateJacobian(internal_->A, state, init);
  A = dt * internal_->A;
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getInputJacobian(InputMatrix& B, const State& state, double dt, bool init) {
  if (!internal_) internal_ = new internal(state);
  //if (init) internal_->B = InputMatrix::Zero(B.rows(), B.cols());
  getInputJacobian(internal_->B, state, init);
  B = dt * internal_->B;
}

template <class ConcreteModel, int _VectorDimension, int _CovarianceDimension>
void TimeContinuousSystemModel_<ConcreteModel, _VectorDimension, _CovarianceDimension>::getSystemNoise(NoiseVariance& Q, const State& state, double dt, bool init) {
  if (!internal_) internal_ = new internal(state);
  //if (init) internal_->Q = NoiseVariance::Zero(Q.rows(), Q.cols());
  getSystemNoise(internal_->Q, state, init);
  Q = dt * internal_->Q;
  Q.assertSymmetric();
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_MODEL_INL
