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

#ifndef HECTOR_POSE_ESTIMATION_STATE_H
#define HECTOR_POSE_ESTIMATION_STATE_H

#include <hector_pose_estimation/types.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <ros/time.h>

namespace hector_pose_estimation {

class State {
public:
  typedef ColumnVector Vector;
  typedef SymmetricMatrix Covariance;
  typedef Matrix SystemMatrix;

  typedef VectorBlock<Vector> VectorSegment;
  typedef Block<Covariance::Base> CovarianceBlock;
  typedef VectorBlock<const Vector> ConstVectorSegment;
  typedef Block<const Covariance::Base> ConstCovarianceBlock;

  typedef SubState_<4,3> OrientationStateType;
  typedef VectorBlock<Vector,4> OrientationType;
  typedef VectorBlock<const Vector,4> ConstOrientationType;
  typedef SubState_<3,3> RateStateType;
  typedef VectorBlock<Vector,3> RateType;
  typedef VectorBlock<const Vector,3> ConstRateType;
  typedef SubState_<3,3> PositionStateType;
  typedef VectorBlock<Vector,3> PositionType;
  typedef VectorBlock<const Vector,3> ConstPositionType;
  typedef SubState_<3,3> VelocityStateType;
  typedef VectorBlock<Vector,3> VelocityType;
  typedef VectorBlock<const Vector,3> ConstVelocityType;
  typedef SubState_<3,3> AccelerationStateType;
  typedef VectorBlock<Vector,3> AccelerationType;
  typedef VectorBlock<const Vector,3> ConstAccelerationType;

  typedef std::vector<SubStatePtr> SubStates;

  typedef typename Matrix_<3,3>::type RotationMatrix;

public:
  virtual ~State();

  virtual IndexType getVectorDimension() const { return vector_.rows(); }
  virtual IndexType getCovarianceDimension() const { return covariance_.rows(); }

  virtual void reset();
  virtual void updated();
  virtual void normalize();

  virtual bool valid() const;

  virtual BaseState& base() { return *base_; }
  virtual const BaseState& base() const { return *base_; }

  virtual const Vector& getVector() const { return vector_; }
  virtual const Covariance& getCovariance() const { return covariance_; }
  template <int Size> VectorBlock<const Vector, Size> getSegment(IndexType start) const { return vector_.segment<Size>(start); }

  virtual Vector& x() { return vector_; }
  virtual Covariance& P() { return covariance_; }
//  virtual VectorSegment x0() { return vector_.segment<VectorDimension>(0); }
//  virtual CovarianceBlock P0() { return covariance_.block<CovarianceDimension,CovarianceDimension>(0, 0); }

  virtual void update(const Vector &vector_update);
  virtual void updateOrientation(const ColumnVector3 &rotation_vector);

  virtual SystemStatus getSystemStatus() const { return system_status_; }
  virtual SystemStatus getMeasurementStatus() const { return measurement_status_; }

  virtual bool inSystemStatus(SystemStatus test_status) const;
  virtual bool setSystemStatus(SystemStatus new_status);
  virtual bool setMeasurementStatus(SystemStatus new_status);
  virtual bool updateSystemStatus(SystemStatus set, SystemStatus clear);
  virtual bool updateMeasurementStatus(SystemStatus set, SystemStatus clear);

  typedef boost::function<bool(SystemStatus&)> SystemStatusCallback;
  virtual void addSystemStatusCallback(const SystemStatusCallback& callback);

  virtual const boost::shared_ptr<OrientationStateType> &orientation() const   { return orientation_; }
  virtual const boost::shared_ptr<RateStateType> &rate() const                 { return rate_; }
  virtual const boost::shared_ptr<PositionStateType> &position() const         { return position_; }
  virtual const boost::shared_ptr<VelocityStateType> &velocity() const         { return velocity_; }
  virtual const boost::shared_ptr<AccelerationStateType> &acceleration() const { return acceleration_; }

  virtual ConstOrientationType getOrientation() const;
  virtual ConstRateType getRate() const;
  virtual ConstPositionType getPosition() const;
  virtual ConstVelocityType getVelocity() const;
  virtual ConstAccelerationType getAcceleration() const;

  void setOrientation(const Quaternion& orientation);
  template <typename Derived> void setOrientation(const Eigen::MatrixBase<Derived>& orientation);
  void setRollPitch(const Quaternion& orientation);
  void setRollPitch(ScalarType roll, ScalarType pitch);
  void setYaw(const Quaternion& orientation);
  void setYaw(ScalarType yaw);
  template <typename Derived> void setRate(const Eigen::MatrixBase<Derived>& rate);
  template <typename Derived> void setPosition(const Eigen::MatrixBase<Derived>& position);
  template <typename Derived> void setVelocity(const Eigen::MatrixBase<Derived>& velocity);
  template <typename Derived> void setAcceleration(const Eigen::MatrixBase<Derived>& acceleration);

  void getRotationMatrix(RotationMatrix &R) const;
  const State::RotationMatrix &R() const;
  double getYaw() const;
  void getEuler(double &roll, double &pitch, double &yaw) const;
  ColumnVector3 getEuler() const;

  const SubStates& getSubStates() const { return substates_; }
  template <int SubVectorDimension, int SubCovarianceDimension> boost::shared_ptr<SubState_<SubVectorDimension,SubCovarianceDimension> > getSubState(const Model *model) const;
  template <int SubVectorDimension, int SubCovarianceDimension> boost::shared_ptr<SubState_<SubVectorDimension,SubCovarianceDimension> > getSubState(const std::string& name) const;
  template <int SubVectorDimension, int SubCovarianceDimension> boost::shared_ptr<SubState_<SubVectorDimension,SubCovarianceDimension> > addSubState(const std::string& name = std::string());
  template <int SubVectorDimension, int SubCovarianceDimension> boost::shared_ptr<SubState_<SubVectorDimension,SubCovarianceDimension> > addSubState(const Model *model, const std::string& name = std::string());

  const ros::Time& getTimestamp() const { return timestamp_; }
  void setTimestamp(const ros::Time& timestamp) { timestamp_ = timestamp; }

protected:
  State();
  void construct();

  OrientationType orientationPart();
  RateType ratePart();
  PositionType positionPart();
  VelocityType velocityPart();
  AccelerationType accelerationPart();

  void orientationSet();
  void rollpitchSet();
  void yawSet();
  void rateSet();
  void positionSet();
  void velocitySet();
  void accelerationSet();

protected:
  Vector vector_;
  Covariance covariance_;

  SystemStatus system_status_;
  SystemStatus measurement_status_;

  std::vector<SystemStatusCallback> status_callbacks_;

  SubStates substates_;
  std::map<const Model *, SubStateWPtr> substates_by_model_;
  std::map<std::string, SubStateWPtr> substates_by_name_;

  boost::shared_ptr<BaseState> base_;
  boost::shared_ptr<OrientationStateType> orientation_;
  boost::shared_ptr<RateStateType> rate_;
  boost::shared_ptr<PositionStateType> position_;
  boost::shared_ptr<VelocityStateType> velocity_;
  boost::shared_ptr<AccelerationStateType> acceleration_;

  Vector fake_orientation_;
  Vector fake_rate_;
  Vector fake_position_;
  Vector fake_velocity_;
  Vector fake_acceleration_;

  ros::Time timestamp_;

  // cached rotation matrix
  mutable RotationMatrix R_;
  mutable bool R_valid_;
};

class FullState : public State
{
public:
  FullState();
  virtual ~FullState();
};

class OrientationPositionVelocityState : public State
{
public:
  OrientationPositionVelocityState();
  virtual ~OrientationPositionVelocityState();
};

class OrientationOnlyState : public State
{
public:
  OrientationOnlyState();
  virtual ~OrientationOnlyState();
};

class PositionVelocityState : public State
{
public:
  PositionVelocityState();
  virtual ~PositionVelocityState();
};

} // namespace hector_pose_estimation

#include <hector_pose_estimation/state.inl>

#endif // HECTOR_POSE_ESTIMATION_STATE_H
