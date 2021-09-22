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

#ifndef HECTOR_POSE_ESTIMATION_COLLECTION_H
#define HECTOR_POSE_ESTIMATION_COLLECTION_H

#include <list>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>

namespace hector_pose_estimation {

template <typename T, typename key_type = std::string>
class Collection {
public:
  typedef boost::shared_ptr<T> Ptr;
  typedef boost::weak_ptr<T> WPtr;
  typedef std::list<Ptr> ListType; // lists do no invalidate iterators on insert operations
  typedef std::map<key_type, WPtr> MapType;
  typedef typename ListType::const_iterator iterator;
  typedef typename ListType::const_iterator const_iterator;

  const Ptr& add(const Ptr& p, const key_type& key) {
    list_.push_back(p);
    map_[key] = p;
    return list_.back();
  }

  template <typename Derived> boost::shared_ptr<Derived> add(Derived *p) { return boost::static_pointer_cast<Derived>(add(Ptr(p), p->getName())); }
  template <typename Derived> boost::shared_ptr<Derived> add(Derived *p, const key_type& key) { return boost::static_pointer_cast<Derived>(add(Ptr(p), key)); }

  template <typename Derived> const Ptr& create() {
    Derived *p = new Derived();
    return add(p);
  }

  template <typename Derived> const Ptr& create(const key_type& key) {
    Derived *p = new Derived();
    return add(p, key);
  }

//  Ptr get(std::size_t index) const {
//    if (index >= size()) return Ptr();
//    return list_[index];
//  }

  Ptr get(const key_type& key) const {
    if (!map_.count(key)) return Ptr();
    return map_.at(key).lock();
  }

//  template <typename Derived>
//  boost::shared_ptr<Derived> getType(std::size_t index) const {
//    return boost::dynamic_pointer_cast<Derived>(get(index));
//  }

  template <typename Derived>
  boost::shared_ptr<Derived> getType(const key_type& key) const {
    return boost::dynamic_pointer_cast<Derived>(get(key));
  }

  bool empty()           const { return list_.empty(); }
  std::size_t size()     const { return list_.size(); }
  const_iterator begin() const { return list_.begin(); }
  const_iterator end()   const { return list_.end(); }

  void clear() {
    map_.clear();
    list_.clear();
  }

private:
  ListType list_;
  MapType map_;
};

}

#endif // HECTOR_POSE_ESTIMATION_COLLECTION_H
