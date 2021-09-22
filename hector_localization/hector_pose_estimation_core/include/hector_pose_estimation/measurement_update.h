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

#ifndef HECTOR_POSE_ESTIMATION_MEASUREMENT_UPDATE_H
#define HECTOR_POSE_ESTIMATION_MEASUREMENT_UPDATE_H

#include <boost/type_traits/is_base_of.hpp>

namespace hector_pose_estimation {

class MeasurementUpdate
{
public:
  MeasurementUpdate() {}
  virtual ~MeasurementUpdate() {}

  virtual bool hasVariance() const { return false; }
};

template <class MeasurementModel>
class Update_ : public MeasurementUpdate {
public:
  enum { MeasurementDimension = MeasurementModel::MeasurementDimension };
  typedef Update_<MeasurementModel> Type;
  typedef typename MeasurementModel::MeasurementVector Vector;
  typedef typename MeasurementModel::NoiseVariance Variance;

  Update_()
    : has_variance_(false)
  {
    y_.setZero();
  }
  Update_(Vector const& y)
    : has_variance_(false)
  {
    *this = y;
  }
  Update_(double y)
    : has_variance_(false)
  {
    *this = y;
  }
  Update_(double x, double y, double z)
    : has_variance_(false)
  {
    y_ = Vector(x, y, z);
  }
  template <typename OtherDerived> Update_(const Eigen::MatrixBase<OtherDerived>& other)
    : y_(other)
    , has_variance_(false)
  {}
  virtual ~Update_() {}

  virtual Vector &operator=(Vector const& y) {  y_ = y; return y_; }
  virtual Vector &operator=(double y) { y_(0) = y; return y_; }

  virtual Vector const &getVector() const { return y_; }

  virtual bool hasVariance() const { return has_variance_; }
  virtual Variance const &getVariance() const { return R_; }
  virtual void setVariance(Variance const& R) { R_ = R; has_variance_ = true; }

protected:
  Vector y_;
  Variance R_;
  bool has_variance_;
};

namespace traits {

  template <class ConcreteModel> struct Update { typedef Update_<ConcreteModel> type; };

  template <class ConcreteModel, class Enable = void>
  class UpdateInspector {
  public:
    UpdateInspector(const typename Update<ConcreteModel>::type& update) : update_(update) {}
    typename ConcreteModel::MeasurementVector const& getVector(const State&) { return *static_cast<typename ConcreteModel::MeasurementVector *>(0); }
    typename ConcreteModel::NoiseVariance const& getVariance(const State&) { return *static_cast<typename ConcreteModel::NoiseVariance *>(0); }
  private:
    const typename Update<ConcreteModel>::type& update_;
  };

  template <class ConcreteModel>
  class UpdateInspector<ConcreteModel, typename boost::enable_if<boost::is_base_of<Update_<ConcreteModel>, typename Update<ConcreteModel>::type> >::type >
  {
  public:
    UpdateInspector(const typename Update<ConcreteModel>::type& update) : update_(update) {}
    typename ConcreteModel::MeasurementVector const& getVector(const State&) { return update_.getVector(); }
    typename ConcreteModel::NoiseVariance const& getVariance(const State&) { return update_.getVariance(); }
  private:
    const typename Update<ConcreteModel>::type& update_;
  };

} // namespace traits

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_MEASUREMENT_UPDATE_H
