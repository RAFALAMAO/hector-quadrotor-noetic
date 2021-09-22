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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_EKF_H
#define HECTOR_POSE_ESTIMATION_FILTER_EKF_H

#include <hector_pose_estimation/filter.h>

namespace hector_pose_estimation {
namespace filter {

class EKF : public Filter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EKF(State &state);
  virtual ~EKF();

  virtual std::string getType() const { return "EKF"; }

  virtual bool init(PoseEstimation &estimator);
  virtual bool preparePredict(double dt);
  virtual bool predict(const SystemPtr& system, double dt);
  virtual bool doPredict(double dt);

  class Predictor
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Predictor(EKF *filter)
      : filter_(filter)
      , x_diff(filter->state().getVectorDimension())
      , A(filter->state().getCovarianceDimension(), filter->state().getCovarianceDimension())
      , Q(filter->state().getCovarianceDimension(), filter->state().getCovarianceDimension())
    {
      x_diff.setZero();
      A.setZero();
      Q.setZero();
    }
    virtual ~Predictor() {}
    virtual bool predict(double dt) = 0;

  protected:
    EKF *filter_;

  public:
    State::Vector x_diff;
    State::SystemMatrix A;
    State::Covariance Q;
  };

  template <class ConcreteModel, typename Enabled = void>
  class Predictor_ : public Filter::template Predictor_<ConcreteModel>, public Predictor
  {
  public:
    typedef ConcreteModel Model;
    typedef typename Filter::template Predictor_<ConcreteModel> Base;
    using Filter::template Predictor_<ConcreteModel>::state;

    Predictor_(EKF *filter, Model *model)
      : Base(filter, model)
      , Predictor(filter)
    {}
    virtual ~Predictor_() {}

    virtual bool predict(double dt);
  };

  class Corrector
  {
  public:
    Corrector(EKF *filter) : filter_(filter) {}
    virtual ~Corrector() {}

  protected:
    EKF *filter_;
  };

  template <class ConcreteModel, typename Enabled = void>
  class Corrector_ : public Filter::template Corrector_<ConcreteModel>, public Corrector
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ConcreteModel Model;
    typedef typename Filter::template Corrector_<ConcreteModel> Base;
    using Filter::template Corrector_<ConcreteModel>::state;

    Corrector_(EKF *filter, Model *model)
      : Base(filter, model)
      , Corrector(filter)
      , y_pred(model->getDimension())
      , error(model->getDimension())
      , C(model->getDimension(), filter->state().getCovarianceDimension())
      , CP(model->getDimension(), filter->state().getCovarianceDimension())
      , S(model->getDimension(), model->getDimension())
      , K(filter->state().getCovarianceDimension(), model->getDimension())
      , update(filter->state().getCovarianceDimension())
    {
      y_pred.setZero();
      error.setZero();
      C.setZero();
      CP.setZero();
      S.setZero();
      K.setZero();
      update.setZero();
    }
    virtual ~Corrector_() {}

    virtual bool correct(const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R);
    virtual typename ConcreteModel::MeasurementVector getResidual() const { return error; }

  public:
    typename Model::MeasurementVector y_pred;
    typename Model::MeasurementVector error;
    typename Model::MeasurementMatrix C;
    typename Matrix_<ConcreteModel::MeasurementDimension, Dynamic>::type CP;
    typename Model::NoiseVariance S;
    typename Model::GainMatrix K;
    typename Model::UpdateVector update;
  };

public:
  State::Vector x_diff;
  State::SystemMatrix A;
  State::Covariance Q;
};

} // namespace filter
} // namespace hector_pose_estimation

#include "ekf.inl"

#endif // HECTOR_POSE_ESTIMATION_FILTER_EKF_H
