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

#ifndef HECTOR_POSE_ESTIMATION_SYSTEM_H
#define HECTOR_POSE_ESTIMATION_SYSTEM_H

#include <hector_pose_estimation/system_model.h>
#include <hector_pose_estimation/types.h>
#include <hector_pose_estimation/state.h>
#include <hector_pose_estimation/input.h>
#include <hector_pose_estimation/filter.h>

#include <ros/console.h>

namespace hector_pose_estimation {

template <class ConcreteModel> class System_;

class System
{
public:
  System(const std::string& name);
  virtual ~System();

  template <class ConcreteModel> static boost::shared_ptr<System_<ConcreteModel> > create(ConcreteModel *model, const std::string& name = "system");

  virtual const std::string& getName() const { return name_; }
  virtual void setName(const std::string& name) { name_ = name; }

  virtual SystemModel *getModel() const { return 0; }

  virtual Filter *filter() const = 0;
  virtual Filter::Predictor *predictor() const = 0;
  virtual void setFilter(Filter *filter) = 0;

  virtual bool init(PoseEstimation& estimator, State& state);
  virtual void cleanup();
  virtual void reset(State& state);

  virtual bool active(const State& state);
  virtual SystemStatus getStatusFlags() const { return status_flags_; }

  virtual ParameterList& parameters() { return parameters_; }
  virtual const ParameterList& parameters() const { return parameters_; }

  virtual void getPrior(State &state) const;

  virtual bool update(double dt);

  virtual void updated();
  virtual bool limitState(State& state);

protected:
  virtual bool updateImpl(double dt) = 0;
  virtual bool prepareUpdate(State &state, double dt) { return getModel()->prepareUpdate(state, dt); }
  virtual void afterUpdate(State &state) { getModel()->afterUpdate(state); }

protected:
  std::string name_;
  ParameterList parameters_;
  SystemStatus status_flags_;
};

template <class ConcreteModel>
class System_ : public System
{
public:
  typedef ConcreteModel Model;
  typedef typename traits::Input<ConcreteModel>::Type InputType;
  typedef typename traits::Input<ConcreteModel>::Vector InputVector;

  System_(const std::string& name = "system")
    : System(name)
    , model_(new Model)
  {
    parameters_.add(model_->parameters());
  }

  System_(Model *model, const std::string& name)
    : System(name)
    , model_(model)
  {
    parameters_.add(model_->parameters());
  }

  virtual ~System_() {}

  virtual void reset(State& state) {
    System::reset(state);
    if (predictor()) predictor()->reset();
  }

  virtual Model *getModel() const { return model_.get(); }

  virtual Filter *filter() const { return predictor_ ? predictor_->base() : 0; }
  virtual Filter::Predictor_<Model> *predictor() const { return predictor_.get(); }
  virtual void setFilter(Filter *filter = 0); // implemented in filter/set_filter.h

protected:
  virtual bool updateImpl(double dt);

private:
  boost::shared_ptr<Model> model_;
  boost::shared_ptr< Filter::Predictor_<Model> > predictor_;
};

template <class ConcreteModel>
boost::shared_ptr<System_<ConcreteModel> > System::create(ConcreteModel *model, const std::string& name)
{
  return boost::make_shared<System_<ConcreteModel> >(model, name);
}

} // namespace hector_pose_estimation

#include "system.inl"

#endif // HECTOR_POSE_ESTIMATION_SYSTEM_H
