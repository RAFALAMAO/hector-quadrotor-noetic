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

#ifndef HECTOR_POSE_ESTIMATION_SUBSTATE_H
#define HECTOR_POSE_ESTIMATION_SUBSTATE_H

#include <hector_pose_estimation/state.h>

namespace hector_pose_estimation {

class SubState
{
public:
  SubState(State& state) : state_(state) {}
  virtual ~SubState() {}

  virtual int getVectorDimension() const = 0;
  virtual int getCovarianceDimension() const = 0;
  virtual int getVectorIndex() const = 0;
  virtual int getCovarianceIndex() const = 0;

  virtual void reset() {}
  virtual void updated() {}
  virtual void normalize() {}

protected:
  State& state_;
  template <int _VectorDimension, int _CovarianceDimension> class initializer;
};

template <int _VectorDimension, int _CovarianceDimension> class SubState::initializer {
public:
  enum { VectorDimension = _VectorDimension };
  enum { CovarianceDimension = _CovarianceDimension };
  initializer(State& state) : index_(state.getVector().rows()), covariance_index_(state.getCovariance().rows()) {
    assert(index_ + VectorDimension <= MaxVectorSize);
    state.x().conservativeResize(index_ + VectorDimension);
    assert(covariance_index_ + CovarianceDimension <= MaxMatrixRowsCols);
    state.P().conservativeResize(covariance_index_ + CovarianceDimension, covariance_index_ + CovarianceDimension);
  }
protected:
  const IndexType index_;
  const IndexType covariance_index_;
};

template <> class SubState::initializer<Dynamic,Dynamic> {
public:
  enum { VectorDimension = Dynamic };
  enum { CovarianceDimension = Dynamic };
  initializer(State& state) : index_(0), covariance_index_(0)
  {}
protected:
  const IndexType index_;
  const IndexType covariance_index_;
};
extern template class SubState::initializer<Dynamic,Dynamic>;

template <int _VectorDimension, int _CovarianceDimension = _VectorDimension>
class SubState_ : public SubState, public SubState::initializer<_VectorDimension, _CovarianceDimension>
{
public:
  enum { VectorDimension = SubState::initializer<_VectorDimension, _CovarianceDimension>::VectorDimension };
  enum { CovarianceDimension = SubState::initializer<_VectorDimension, _CovarianceDimension>::CovarianceDimension };
  typedef ColumnVector_<VectorDimension> Vector;

  typedef VectorBlock<State::Vector,VectorDimension> VectorSegment;
  typedef Block<State::Covariance,CovarianceDimension,CovarianceDimension> CovarianceBlock;
  typedef Block<State::Covariance,Dynamic,CovarianceDimension> CrossVarianceBlock;

  typedef VectorBlock<const State::Vector,VectorDimension> ConstVectorSegment;
  typedef Block<const State::Covariance,CovarianceDimension,CovarianceDimension> ConstCovarianceBlock;
  typedef Block<const State::Covariance,Dynamic,CovarianceDimension> ConstCrossVarianceBlock;

  typedef boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > Ptr;

  using SubState::initializer<_VectorDimension, _CovarianceDimension>::index_;
  using SubState::initializer<_VectorDimension, _CovarianceDimension>::covariance_index_;

public:
  SubState_(State& state)
    : SubState(state)
    , SubState::initializer<_VectorDimension, _CovarianceDimension>(state)
  {}
  virtual ~SubState_() {}

  int getVectorDimension() const { return VectorDimension; }
  int getCovarianceDimension() const { return CovarianceDimension; }
  int getVectorIndex() const { return index_; }
  int getCovarianceIndex() const { return covariance_index_; }

  ConstVectorSegment getVector() const { return ConstVectorSegment(state_.getVector(), index_, getVectorDimension()); }
  ConstCovarianceBlock getCovariance() const { return ConstCovarianceBlock(state_.getCovariance(), covariance_index_, covariance_index_, getCovarianceDimension(), getCovarianceDimension()); }
  template <typename OtherSubState> Block<const State::Covariance,CovarianceDimension,OtherSubState::CovarianceDimension> getCrossVariance(const OtherSubState &other) const { return Block<const State::Covariance,CovarianceDimension,OtherSubState::CovarianceDimension>(state_.getCovariance(), covariance_index_, covariance_index_, getCovarianceDimension(), other.getCovarianceDimension()); }
  template <int Size> VectorBlock<const Vector, Size> getSegment(IndexType start) const { return VectorBlock<const Vector, Size>(state_.getVector(), index_ + start); }

  VectorSegment vector() { return VectorSegment(state_.x(), index_, getVectorDimension()); }
  CovarianceBlock P() { return CovarianceBlock(state_.P(), covariance_index_, covariance_index_, getCovarianceDimension(), getCovarianceDimension()); }
  CrossVarianceBlock P01() { return CrossVarianceBlock(state_.P(), 0, covariance_index_, state_.getCovarianceDimension(), getCovarianceDimension()); }

  template <typename VectorType> VectorBlock<VectorType,VectorDimension> segment(VectorType &vector) { return VectorBlock<VectorType,VectorDimension>(vector, index_, getVectorDimension()); }
  template <typename MatrixType> Block<MatrixType,CovarianceDimension,CovarianceDimension> block(MatrixType &matrix) { return Block<MatrixType,CovarianceDimension,CovarianceDimension>(matrix, covariance_index_, covariance_index_, getCovarianceDimension(), getCovarianceDimension()); }
  template <typename MatrixType, typename OtherSubState> Block<MatrixType,CovarianceDimension,OtherSubState::CovarianceDimension> block(MatrixType &matrix, const OtherSubState &other) { return Block<MatrixType,CovarianceDimension,OtherSubState::CovarianceDimension>(matrix, covariance_index_, other.getCovarianceIndex(), getCovarianceDimension(), other.getCovarianceDimension()); }
  template <typename MatrixType> Block<MatrixType,CovarianceDimension,MatrixType::ColsAtCompileTime> rows(MatrixType &matrix) { return Block<MatrixType,CovarianceDimension,MatrixType::ColsAtCompileTime>(matrix, covariance_index_, 0, getCovarianceDimension(), matrix.cols()); }
  template <typename MatrixType> Block<MatrixType,MatrixType::RowsAtCompileTime,CovarianceDimension> cols(MatrixType &matrix) { return Block<MatrixType,MatrixType::RowsAtCompileTime,CovarianceDimension>(matrix, 0, covariance_index_, matrix.rows(), getCovarianceDimension()); }
};
extern template class SubState_<Dynamic,Dynamic>;

class BaseState : public SubState_<Dynamic,Dynamic>
{
public:
    BaseState(State& state, int vector_dimension, int covariance_dimension)
      : SubState_<Dynamic,Dynamic>(state)
      , dimension_(vector_dimension)
      , covariance_dimension_(covariance_dimension)
    {}
    virtual ~BaseState() {}

    int getVectorDimension() const { return dimension_; }
    int getCovarianceDimension() const { return covariance_dimension_; }

private:
    const IndexType dimension_;
    const IndexType covariance_dimension_;
};

template <int _VectorDimension, int _CovarianceDimension>
boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > State::getSubState(const Model *model) const {
  return boost::dynamic_pointer_cast<SubState_<_VectorDimension, _CovarianceDimension> >(substates_by_model_.count(model) ? substates_by_model_.at(model).lock() : SubStatePtr());
}

//template <>
//inline boost::shared_ptr<BaseState> State::getSubState<Dynamic>(const Model *) const {
//  return base_;
//}

template <int _VectorDimension, int _CovarianceDimension>
boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > State::getSubState(const std::string& name) const {
  return boost::dynamic_pointer_cast<SubState_<_VectorDimension, _CovarianceDimension> >(substates_by_name_.count(name) ? substates_by_name_.at(name).lock() : SubStatePtr());
}

template <int _VectorDimension, int _CovarianceDimension>
boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > State::addSubState(const std::string& name) {
  return addSubState<_VectorDimension, _CovarianceDimension>(0, name);
}

template <int _VectorDimension, int _CovarianceDimension>
boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > State::addSubState(const Model *model, const std::string& name) {
  boost::shared_ptr<SubState_<_VectorDimension, _CovarianceDimension> > substate;
  if (!name.empty()) substate = getSubState<_VectorDimension, _CovarianceDimension>(name);
  if (!substate) {
    if (substates_by_name_.count(name)) return substate; // state already exists but with a different type
    substate.reset(new SubState_<_VectorDimension, _CovarianceDimension>(*this));
    substates_.push_back(boost::static_pointer_cast<SubState>(substate));
    if (!name.empty()) substates_by_name_[name] = SubStateWPtr(boost::static_pointer_cast<SubState>(substate));
  }
  if (model) {
    substates_by_model_[model] = SubStateWPtr(boost::static_pointer_cast<SubState>(substate));
  }
  return substate;
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_SUBSTATE_H
