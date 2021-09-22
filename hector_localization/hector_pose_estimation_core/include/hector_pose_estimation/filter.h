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

#ifndef HECTOR_POSE_ESTIMATION_FILTER_H
#define HECTOR_POSE_ESTIMATION_FILTER_H

#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/types.h>

#include <map>

namespace hector_pose_estimation {

class Filter {
public:
  Filter(State &state);
  virtual ~Filter();

  virtual std::string getType() const = 0;

  virtual bool init(PoseEstimation& estimator);
  virtual void cleanup();
  virtual void reset();

  virtual const State& state() const { return state_; }
  virtual State& state() { return state_; }

  virtual bool preparePredict(double dt);
  virtual bool predict(const Systems& systems, double dt);
  virtual bool predict(const SystemPtr& system, double dt);
  virtual bool doPredict(double dt);

  virtual bool prepareCorrect();
  virtual bool correct(const Measurements& measurements);
  virtual bool correct(const MeasurementPtr& measurement);
  virtual bool doCorrect();

  class Predictor
  {
  public:
    Predictor(Filter *filter) : filter_(filter) {}
    virtual ~Predictor() {}

    virtual void reset() { init_ = true; }

    Filter *base() { return filter_; }
    const Filter *base() const { return filter_; }

    State& state() { return filter_->state(); }
    const State& state() const { return filter_->state(); }

  protected:
    Filter *filter_;
    bool init_;
  };

  template <class ConcreteModel> struct Predictor_  : public Predictor {
    Predictor_(Filter *filter, ConcreteModel *model) : Predictor(filter), model_(model) { reset(); }
    virtual ~Predictor_() {}

    virtual bool predict(double dt) = 0;

    template <typename Derived> typename Derived::template Predictor_<ConcreteModel> *derived() { return dynamic_cast<typename Derived::template Predictor_<ConcreteModel> *>(this); }
    template <typename Derived> const typename Derived::template Predictor_<ConcreteModel> *derived() const { return dynamic_cast<const typename Derived::template Predictor_<ConcreteModel> *>(this); }

  protected:
    ConcreteModel *model_;
  };

  class Corrector
  {
  public:
    Corrector(Filter *filter) : filter_(filter) {}
    virtual ~Corrector() {}

    virtual void reset() { init_ = true; }

    Filter *base() { return filter_; }
    const Filter *base() const { return filter_; }

    State& state() { return filter_->state(); }
    const State& state() const { return filter_->state(); }

  protected:
    Filter *filter_;
    bool init_;
  };

  template <class ConcreteModel> struct Corrector_ : public Corrector {
    Corrector_(Filter *filter, ConcreteModel *model) : Corrector(filter), model_(model) { reset(); }
    virtual ~Corrector_() {}

    virtual bool correct(const typename ConcreteModel::MeasurementVector& y, const typename ConcreteModel::NoiseVariance& R) = 0;

    virtual typename ConcreteModel::MeasurementVector getResidual() const { return typename ConcreteModel::MeasurementVector(); }

    template <typename Derived> typename Derived::template Corrector_<ConcreteModel> *derived() { return dynamic_cast<typename Derived::template Corrector_<ConcreteModel> *>(this); }
    template <typename Derived> const typename Derived::template Corrector_<ConcreteModel> *derived() const { return dynamic_cast<const typename Derived::template Corrector_<ConcreteModel> *>(this); }

  protected:
    ConcreteModel *model_;
  };

  template <typename Derived> Derived *derived() { return dynamic_cast<Derived *>(this); }
  template <typename Derived> const Derived *derived() const { return dynamic_cast<const Derived *>(this); }

  template <typename Derived>
  struct Factory {
    Factory(Derived *filter) : filter_(filter) {}
    template <class ConcreteModel> boost::shared_ptr<Predictor_<ConcreteModel> > addPredictor(ConcreteModel *model) { return boost::static_pointer_cast<Predictor_<ConcreteModel> >(boost::make_shared<typename Derived::template Predictor_<ConcreteModel> >(filter_, model)); }
    template <class ConcreteModel> boost::shared_ptr<Corrector_<ConcreteModel> > addCorrector(ConcreteModel *model) { return boost::static_pointer_cast<Corrector_<ConcreteModel> >(boost::make_shared<typename Derived::template Corrector_<ConcreteModel> >(filter_, model)); }

  private:
    Derived *filter_;
  };
  template <typename Derived> static Factory<Derived> factory(Derived *filter) { return Factory<Derived>(filter); }

protected:
  State& state_;
  Inputs inputs_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_FILTER_H
