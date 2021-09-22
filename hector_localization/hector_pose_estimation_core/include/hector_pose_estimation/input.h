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

#ifndef HECTOR_POSE_ESTIMATION_INPUT_H
#define HECTOR_POSE_ESTIMATION_INPUT_H

#include <hector_pose_estimation/types.h>

namespace hector_pose_estimation {

class Input
{
public:
  Input() {}
  virtual ~Input() {}

  virtual int getDimension() const = 0;

  virtual const std::string& getName() const { return name_; }
  virtual void setName(const std::string& name) { name_ = name; }

  virtual Input& operator=(const Input& other) = 0;

private:
  std::string name_;
};

template <int _Dimension>
class Input_ : public Input {
public:
  enum { Dimension = _Dimension };
  typedef Input_<Dimension> Type;
  typedef typename ColumnVector_<Dimension>::type Vector;
  typedef typename SymmetricMatrix_<Dimension>::type Variance;

  typedef boost::shared_ptr< Input_<_Dimension> > Ptr;

  Input_() { u_.setZero(); }
  template <typename Derived> Input_(const Eigen::MatrixBase<Derived>& u) : u_(u) {}
  template <typename Derived> Input_(const Eigen::MatrixBase<Derived>& u, const Variance& Q) : u_(u), variance_(new Variance(Q)) {}
  Input_(double u) { *this = u; }
  Input_(double u, const Variance& Q) : variance_(new Variance(Q)) { *this = u; }
  Input_(const Input& other) { *this = other; }
  virtual ~Input_() {}

  virtual int getDimension() const { return Dimension; }

  virtual Vector const &getVector() const { return u_; }
  virtual Vector& u() { return u_; }

  virtual Variance &setVariance(const Variance &other) {
    if (!variance_) variance_.reset(new Variance);
    *variance_ = other;
    return *variance_;
  }

  virtual bool hasVariance() const { return (variance_.get() != 0); }
  virtual Variance const &getVariance() { if (!variance_) variance_.reset(new Variance); return *variance_; }
  virtual Variance const &getVariance() const { return *variance_; }
  virtual Variance& variance() { if (!variance_) variance_.reset(new Variance); return *variance_; }

  virtual Input_<Dimension>& operator=(const Input& other) {
    *this = static_cast<const Input_<Dimension>&>(other);
    return *this;
  }

  virtual Input_<Dimension>& operator=(const Input_<Dimension>& other) {
    u_ = other.getVector();
    if (other.variance_) setVariance(*other.variance_);
    return *this;
  }

  template <typename Derived> Vector &operator=(const Eigen::MatrixBase<Derived>& other) {
    u_ = other;
    return u_;
  }

  virtual Vector &operator=(double u) { u_.setConstant(u); return u_; }

protected:
  Vector u_;
  boost::shared_ptr<Variance> variance_;
};

namespace traits {
  template <class Model> struct Input {
    enum { Dimension = 0 };
    typedef Input_<Dimension> Type;
    typedef ColumnVector_<0> Vector;
    typedef SymmetricMatrix_<0> Variance;
  };
}

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_INPUT_H
