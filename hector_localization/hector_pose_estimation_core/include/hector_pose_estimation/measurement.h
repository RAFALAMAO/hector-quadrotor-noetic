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

#ifndef HECTOR_POSE_ESTIMATION_MEASUREMENT_H
#define HECTOR_POSE_ESTIMATION_MEASUREMENT_H

#include <hector_pose_estimation/measurement_model.h>
#include <hector_pose_estimation/measurement_update.h>
#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/queue.h>
#include <hector_pose_estimation/filter.h>

namespace hector_pose_estimation {

template <class ConcreteModel> class Measurement_;

class Measurement
{
public:
  Measurement(const std::string& name);
  virtual ~Measurement();

  template <class ConcreteModel> static boost::shared_ptr<Measurement_<ConcreteModel> > create(ConcreteModel *model, const std::string& name);

  virtual const std::string& getName() const { return name_; }
  void setName(const std::string& name) { name_ = name; }

  virtual MeasurementModel* getModel() const { return 0; }
  virtual int getDimension() const { return 0; }

  virtual Filter *filter() const { return filter_; }
  virtual void setFilter(Filter *filter) { filter_ = filter; }

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void cleanup();
  virtual void reset(State& state);

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void add(const MeasurementUpdate &update);
  virtual bool process();
  virtual bool update(const MeasurementUpdate &update);

  bool enabled() const { return enabled_; }
  void enable() { enabled_ = true; }
  void disable() { enabled_ = false; status_flags_ = 0; }

  virtual bool active(const State& state);
  virtual SystemStatus getStatusFlags() const { return status_flags_; }

  void setTimeout(double timeout) { timeout_ = timeout; }
  double getTimeout() const { return timeout_; }

  void setMinInterval(double min_interval) { min_interval_ = min_interval; }
  double getMinInterval() const { return min_interval_; }

  void increase_timer(double dt);
  bool timedout() const;

protected:
  virtual Queue& queue() = 0;
  virtual bool updateImpl(const MeasurementUpdate &update) { return false; }

  virtual bool onInit(PoseEstimation& estimator) { return true; } // currently unsed...
  virtual void onReset() { }
  virtual void onCleanup() { } // currently unsed...

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;

  bool enabled_;
  double min_interval_;

  double timeout_;
  double timer_;

  Filter *filter_;
};

template <class ConcreteModel>
class Measurement_ : public Measurement {
public:
  typedef ConcreteModel Model;
  typedef typename traits::Update<ConcreteModel>::type Update;

  enum { MeasurementDimension = Model::MeasurementDimension };
  typedef typename Model::MeasurementVector MeasurementVector;
  typedef typename Model::NoiseVariance NoiseVariance;

  Measurement_(const std::string& name)
    : Measurement(name)
    , model_(new Model)
  {
    parameters_.add(model_->parameters());
  }

  Measurement_(Model *model, const std::string& name)
    : Measurement(name)
    , model_(model)
  {
    parameters_.add(model_->parameters());
  }

  virtual ~Measurement_() {
  }

  virtual Model* getModel() const { return model_.get(); }
  virtual int getDimension() const { return MeasurementDimension; }

  virtual Filter *filter() const { return corrector_ ? corrector_->base() : 0; }
  virtual const boost::shared_ptr< Filter::Corrector_<Model> >& corrector() const { return corrector_; }
  virtual void setFilter(Filter *filter = 0); // implemented in filter/set_filter.h

  virtual bool init(PoseEstimation& estimator, State& state) {
    if (!Measurement::init(estimator, state)) return false;
    model_->getMeasurementNoise(R_, state, true);
    return true;
  }

  virtual void reset(State& state) {
    model_->getMeasurementNoise(R_, state, true);
    Measurement::reset(state);
    if (corrector()) corrector()->reset();
  }

  virtual MeasurementVector const& getVector(const Update &update, const State &state) {
    const MeasurementVector *fixed = getModel()->getFixedMeasurementVector();
    if (fixed) return *fixed;
    return traits::UpdateInspector<ConcreteModel>(update).getVector(state);
  }

  virtual NoiseVariance const& getVariance(const Update &update, const State &state) {
    if (update.hasVariance()) return traits::UpdateInspector<ConcreteModel>(update).getVariance(state);
    model_->getMeasurementNoise(R_, state, false);
    return R_;
  }

  virtual void setNoiseVariance(NoiseVariance const& R) {
//    if (!R_) R_.reset(new NoiseVariance);
    R_ = R;
  }

protected:
  virtual bool updateImpl(const MeasurementUpdate &update);
  virtual bool prepareUpdate(State &state, const Update &update) { return getModel()->prepareUpdate(state, update); }
  virtual void afterUpdate(State &state) { getModel()->afterUpdate(state); }

protected:
  boost::shared_ptr<Model> model_;
  NoiseVariance R_;

  Queue_<Update> queue_;
  virtual Queue& queue() { return queue_; }

  boost::shared_ptr< Filter::Corrector_<Model> > corrector_;
};

template <class ConcreteModel>
boost::shared_ptr<Measurement_<ConcreteModel> > Measurement::create(ConcreteModel *model, const std::string& name)
{
  return boost::make_shared<Measurement_<ConcreteModel> >(model, name);
}

} // namespace hector_pose_estimation

#include "measurement.inl"

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_H
