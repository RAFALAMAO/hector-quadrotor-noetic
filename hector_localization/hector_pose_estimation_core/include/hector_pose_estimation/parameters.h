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

#ifndef HECTOR_POSE_ESTIMATION_PARAMETERS_H
#define HECTOR_POSE_ESTIMATION_PARAMETERS_H

#include <list>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/type_traits.hpp>

namespace hector_pose_estimation {

  class Parameter;
  template <typename T> class ParameterT;
  typedef boost::shared_ptr<Parameter> ParameterPtr;
  typedef boost::shared_ptr<const Parameter> ParameterConstPtr;
  typedef boost::function<void(ParameterPtr)> ParameterRegisterFunc;

  class Parameter {
  public:
    std::string key;
    Parameter(const std::string& key) : key(key), parameter_(this) {}
    Parameter(Parameter& other) : key(other.key), parameter_(&other) {}
    virtual ~Parameter() {}

    virtual ParameterPtr clone() { return parameter_ ? parameter_->clone() : ParameterPtr(); }
    virtual const char *type() const { return parameter_ ? parameter_->type() : 0; }

    virtual bool empty() const { return !parameter_; }
    virtual bool isAlias() const { return false; }

    template <typename T> bool hasType() const {
      return dynamic_cast<const ParameterT<T> *>(parameter_) != 0;
    }

    template <typename T> const T& as() const {
      const ParameterT<T>& p = dynamic_cast<const ParameterT<T> &>(*parameter_);
      return p.value();
    }

    template <typename T> T& as() {
      ParameterT<T>& p = dynamic_cast<ParameterT<T> &>(*parameter_);
      return p.value();
    }

    operator void*() const { return reinterpret_cast<void *>(!empty()); }

    template <typename T> Parameter& operator =(const T& value) {
      ParameterT<T>& p = dynamic_cast<ParameterT<T>&>(*parameter_);
      p.set(value);
      return *this;
    }

  protected:
    Parameter *parameter_;
  };

  template <typename T>
  class ParameterT : public Parameter {
  public:
    typedef typename boost::remove_reference<typename boost::remove_const<T>::type>::type param_type;

    ParameterT(const std::string& key, param_type &value) : Parameter(key), value_(value) {}
    ParameterT(Parameter& other) : Parameter(other), value_(other.as<T>()) {}
    virtual ~ParameterT() {}

    ParameterPtr clone() { return ParameterPtr(new ParameterT<T>(*this)); }
    const char *type() const { return typeid(param_type).name(); }

    operator param_type&() const { return value_; }
    param_type& value() { return value_; }
    const param_type& value() const { return value_; }
    void set(const param_type& value) { value_ = value; }

  protected:
    param_type& value_;
  };

  class Alias : public Parameter {
  public:
    Alias() : Parameter(std::string()) { parameter_ = 0; }
    Alias(const ParameterPtr& other) : Parameter(other->key) { *this = parameter_; }
    Alias(const ParameterPtr& other, const std::string& key) : Parameter(key) { *this = parameter_; }
    virtual ~Alias() {}

    virtual bool isAlias() const { return true; }

    using Parameter::operator =;
    Alias& operator =(const ParameterPtr& other) {
      parameter_ = other.get();
      if (key.empty()) key = other->key;
      return *this;
    }
  };

  template <typename T>
  class AliasT : public Alias {
  public:
    typedef typename boost::remove_reference<typename boost::remove_const<T>::type>::type param_type;

    AliasT() {}
    AliasT(const ParameterPtr& other) : Alias(other) {}
    AliasT(const ParameterPtr& other, const std::string& key) : Alias(other, key) {}
    virtual ~AliasT() {}

    operator param_type&() const { return dynamic_cast<ParameterT<T> &>(*parameter_).value(); }
    param_type& value() { return dynamic_cast<ParameterT<T> &>(*parameter_).value(); }
    const param_type& value() const { return dynamic_cast<const ParameterT<T> &>(*parameter_).value(); }
    void set(const param_type& value) { dynamic_cast<ParameterT<T> &>(*parameter_).set(value); }

    using Alias::operator =;
  };

  class ParameterList : public std::list<ParameterPtr> {
  public:
    using std::list<ParameterPtr>::iterator;
    using std::list<ParameterPtr>::const_iterator;

    ~ParameterList() {}

    template <typename T> ParameterList& add(const std::string& key, T& value, const T& default_value) {
      value = default_value;
      return add(key, value);
    }

    template <typename T> ParameterList& add(const std::string& key, T* value) {
      return add(key, *value);
    }

    template <typename T> ParameterList& add(const std::string& key, T& value);

    ParameterList& add(ParameterPtr const& parameter);
    ParameterList& add(ParameterList const& other);
    ParameterList& add(Parameter& alias, const std::string& key = std::string());
    ParameterList& addAlias(const std::string& key, Alias& alias) { return add(alias, key); }

    ParameterList& copy(const std::string& prefix, ParameterList const& parameters);
    ParameterList& copy(ParameterList const& parameters);

    ParameterPtr const& get(const std::string& key) const;
    template <typename T> T& getAs(const std::string& key) const {
      return get(key)->as<T>();
    }

    using std::list<ParameterPtr>::erase;
    iterator erase(const std::string& key);

    void initialize(ParameterRegisterFunc func) const;
  };

  template <typename T> inline ParameterList& ParameterList::add(const std::string& key, T& value) {
    return add(ParameterPtr(new ParameterT<T>(key, value)));
  }

  static inline ParameterList operator+(ParameterList const& list1, ParameterList const& list2)
  {
    ParameterList result;
    return result.add(list1).add(list2);
  }

  struct ParameterRegistry {
    virtual void operator()(ParameterPtr) {}
  };

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_PARAMETERS_H
