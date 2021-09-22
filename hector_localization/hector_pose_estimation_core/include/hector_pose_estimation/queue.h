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

#ifndef HECTOR_POSE_ESTIMATION_QUEUE_H
#define HECTOR_POSE_ESTIMATION_QUEUE_H

#include <boost/array.hpp>
#include <hector_pose_estimation/measurement_update.h>

namespace hector_pose_estimation {

class Queue {
public:
  static const size_t capacity_ = 10;

  virtual ~Queue() {}
  virtual bool empty() const = 0;
  virtual bool full() const = 0;
  virtual size_t size() const = 0;
  virtual size_t capacity() const = 0;

  virtual void push(const MeasurementUpdate& update) = 0;
  virtual const MeasurementUpdate& pop() = 0;
  virtual void clear() = 0;
};

template <class Update>
class Queue_ : public Queue
{
public:
  Queue_() : in_(0), out_(0), size_(0) {}
  virtual ~Queue_() {}

  virtual bool empty() const { return size_ == 0; }
  virtual bool full() const { return size_ == capacity_; }
  virtual size_t size() const { return size_; }
  virtual size_t capacity() const { return capacity_; }

  virtual void push(const MeasurementUpdate& update) {
    if (full()) return;
    data_[inc(in_)] = static_cast<Update const &>(update);
    size_++;
  }

  virtual const Update& pop() {
    if (empty()) throw std::runtime_error("queue is empty");
    size_--;
    return data_[inc(out_)];
  }

  virtual void clear() { out_ = in_ = size_ = 0; }

private:
  static size_t inc(size_t& index) { size_t temp = index++; index %= Queue::capacity_; return temp; }

  boost::array<Update, Queue::capacity_> data_;
  size_t in_, out_, size_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_QUEUE_H
