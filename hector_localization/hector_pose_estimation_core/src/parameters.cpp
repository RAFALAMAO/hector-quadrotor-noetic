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

#include <hector_pose_estimation/parameters.h>
#include <hector_pose_estimation/ros/parameters.h>

#include <boost/algorithm/string.hpp>

#include <hector_pose_estimation/matrix.h>

#include <iostream>

namespace hector_pose_estimation {

namespace {
  struct null_deleter {
    void operator()(void const *) const {}
  };
}

ParameterList& ParameterList::add(ParameterPtr const& parameter) {
  erase(parameter->key);
  push_back(parameter);
  return *this;
}

ParameterList& ParameterList::add(ParameterList const& other) {
  for(ParameterList::const_iterator it = other.begin(); it != other.end(); ++it) push_back(*it);
  return *this;
}

ParameterList& ParameterList::add(Parameter& alias, const std::string& key) {
  if (!key.empty()) alias.key = key;
  return add(ParameterPtr(&alias, null_deleter()));
}

ParameterList& ParameterList::copy(const std::string& prefix, ParameterList const& parameters) {
  for(ParameterList::const_iterator it = parameters.begin(); it != parameters.end(); ++it) {
    ParameterPtr copy((*it)->clone());
    if (!copy) continue;
    if (!prefix.empty()) copy->key = prefix + copy->key;
    push_back(copy);
  }
  return *this;
}

ParameterList& ParameterList::copy(ParameterList const& parameters) {
  copy(std::string(), parameters);
  return *this;
}

ParameterPtr const& ParameterList::get(const std::string& key) const {
  for(const_iterator it = begin(); it != end(); ++it) {
    if ((*it)->key == key) {
      return *it;
    }
  }
  throw std::runtime_error("parameter not found");
}

ParameterList::iterator ParameterList::erase(const std::string& key) {
  iterator it = begin();
  for(; it != end(); ++it) {
    if ((*it)->key == key) return erase(it);
  }
  return it;
}

template <typename T>
struct ParameterRegistryROS::Handler
{
  bool operator()(const ParameterPtr& parameter, ros::NodeHandle& nh, bool set_all = false) {
    try {
      ParameterT<T> p(*parameter);
      std::string param_key(boost::algorithm::to_lower_copy(parameter->key));
      if (!nh.getParam(param_key, p.value())) {
        if (set_all) {
          nh.setParam(param_key, p.value());
          ROS_DEBUG_STREAM("Registered parameter " << param_key << " with new value " << p.value());
        }
      } else {
        ROS_DEBUG_STREAM("Found parameter " << param_key << " with value " << p.value());
      }
      return true;
    } catch(std::bad_cast&) {
      return false;
    }
  }
};

template <>
struct ParameterRegistryROS::Handler<ColumnVector>
{
  bool operator()(const ParameterPtr& parameter, ros::NodeHandle& nh, bool set_all = false) {
    try {
      ParameterT<ColumnVector> p(*parameter);
      std::string param_key(boost::algorithm::to_lower_copy(parameter->key));
      XmlRpc::XmlRpcValue vector;
      if (!nh.getParam(param_key, vector)) {
        if (set_all) {
          /// nh.setParam(param_key, p.value);
          ROS_DEBUG_STREAM("Not registered vector parameter " << param_key << ". Using defaults.");
        }
      } else {
        if (vector.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("Found parameter " << param_key << ", but it's not an array!");
          return false;
        }
        p.value().resize(vector.size());
        for(int i = 0; i < vector.size(); ++i) p.value()[i] = vector[i];
        ROS_DEBUG_STREAM("Found parameter " << param_key << " with value " << p.value());
      }
      return true;
    } catch(std::bad_cast&) {
      return false;
    }
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vector) {
  os << "[";
  for(typename std::vector<T>::const_iterator it = vector.begin(); it != vector.end(); ++it) {
    if (it != vector.begin()) os << ", ";
    os << *it;
  }
  os << "]";
  return os;
}

template <typename T>
struct ParameterRegistryROS::Handler< std::vector<T> >
{
  bool operator()(const ParameterPtr& parameter, ros::NodeHandle& nh, bool set_all = false) {
    try {
      ParameterT< std::vector<T> > p(*parameter);
      std::string param_key(boost::algorithm::to_lower_copy(parameter->key));
      XmlRpc::XmlRpcValue vector;
      if (!nh.getParam(param_key, vector)) {
        if (set_all) {
          /// nh.setParam(param_key, p.value);
          ROS_DEBUG_STREAM("Not registered vector parameter " << param_key << ". Using defaults.");
        }
      } else {
        if (vector.getType() != XmlRpc::XmlRpcValue::TypeArray) {
          ROS_WARN_STREAM("Found parameter " << param_key << ", but it's not an array!");
          return false;
        }
        p.value().resize(vector.size());
        for(int i = 0; i < vector.size(); ++i) p.value()[i] = vector[i];
        ROS_DEBUG_STREAM("Found parameter " << param_key << " with value " << p.value());
      }
      return true;
    } catch(std::bad_cast&) {
      return false;
    }
  }
};

ParameterRegistryROS::ParameterRegistryROS(ros::NodeHandle nh)
  : nh_(nh)
  , set_all_(false)
{
  nh_.getParam("set_all_parameters", set_all_);
}

void ParameterRegistryROS::operator ()(ParameterPtr parameter) {
  // call initialize recursively for ParameterList parameters
  if (parameter->hasType<ParameterList>()) {
    ParameterList with_prefix;
    with_prefix.copy(parameter->key + "/", parameter->as<ParameterList>());
    with_prefix.initialize(*this);
    return;
  }

  ROS_DEBUG_STREAM("Registering ROS parameter " << parameter->key);

  if (Handler<std::string>()(parameter, nh_, set_all_) ||
      Handler<double>()(parameter, nh_, set_all_) ||
      Handler<std::vector<double> >()(parameter, nh_, set_all_) ||
      Handler<int>()(parameter, nh_, set_all_) ||
      Handler<bool>()(parameter, nh_, set_all_) ||
      Handler<ColumnVector>()(parameter, nh_, set_all_)
     ) {
    return;
  }

  ROS_ERROR("Parameter %s has unknown type %s!", parameter->key.c_str(), parameter->type());
}

void ParameterList::initialize(ParameterRegisterFunc func) const {
  for(const_iterator it = begin(); it != end(); ++it) {
    const ParameterPtr& parameter = *it;
    if (parameter->empty()) continue;
    if (parameter->isAlias()) continue;

    func(*it);
  }
}

} // namespace hector_pose_estimation
