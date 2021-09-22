//=================================================================================================
// Copyright (c) 2011, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#include <hector_pose_estimation/pose_estimation.h>
#include <hector_pose_estimation/filter/ekf.h>
#include <hector_pose_estimation/global_reference.h>

#include <hector_pose_estimation/system/imu_input.h>
#include <hector_pose_estimation/system/imu_model.h>

#include <boost/weak_ptr.hpp>

namespace hector_pose_estimation {

namespace {
  static PoseEstimation *the_instance = 0;
}

PoseEstimation::PoseEstimation(const SystemPtr& system, const StatePtr& state)
  : state_(state ? state : StatePtr(new OrientationPositionVelocityState))
  , rate_update_(new Rate("rate"))
  , gravity_update_(new Gravity ("gravity"))
  , zerorate_update_(new ZeroRate("zerorate"))
{
  if (!the_instance) the_instance = this;
  if (system) addSystem(system);

  world_frame_ = "/world";
  nav_frame_ = "nav";
  base_frame_ = "base_link";
  stabilized_frame_ = "base_stabilized";
  footprint_frame_ = "base_footprint";
  // position_frame_ = "base_position";
  alignment_time_ = 0.0;
  gravity_ = -9.8065;

  parameters().add("world_frame", world_frame_);
  parameters().add("nav_frame", nav_frame_);
  parameters().add("base_frame", base_frame_);
  parameters().add("stabilized_frame", stabilized_frame_);
  parameters().add("footprint_frame", footprint_frame_);
  parameters().add("position_frame", position_frame_);
  parameters().add(globalReference()->parameters());
  parameters().add("alignment_time", alignment_time_);
  parameters().add("gravity_magnitude", gravity_);

  // add default measurements
  addMeasurement(rate_update_);
  addMeasurement(gravity_update_);
  addMeasurement(zerorate_update_);
}

PoseEstimation::~PoseEstimation()
{
  cleanup();
}

PoseEstimation *PoseEstimation::Instance() {
  if (!the_instance) the_instance = new PoseEstimation();
  return the_instance;
}

bool PoseEstimation::init()
{
#ifdef EIGEN_RUNTIME_NO_MALLOC
  Eigen::internal::set_is_malloc_allowed(true);
#endif

  // initialize global reference
  globalReference()->reset();

  // check if system is initialized
  if (systems_.empty()) return false;

  // create new filter
  filter_.reset(new filter::EKF(*state_));

  // initialize systems (new systems could be added during initialization!)
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it)
    if (!(*it)->init(*this, state())) return false;

  // initialize measurements (new systems could be added during initialization!)
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    if (!(*it)->init(*this, state())) return false;

  // initialize filter
  filter_->init(*this);

  // call setFilter for each system and each measurement
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it)
    (*it)->setFilter(filter_.get());
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it)
    (*it)->setFilter(filter_.get());

  // reset (or initialize) filter and measurements
  reset();

  return true;
}

void PoseEstimation::cleanup()
{
  // cleanup system
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) (*it)->cleanup();

  // cleanup measurements
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) (*it)->cleanup();

  // delete filter instance
  if (filter_) filter_.reset();
}

void PoseEstimation::reset()
{
  // check if system is initialized
  if (systems_.empty()) return;

  // set initial status
  if (filter_) filter_->reset();

  // restart alignment
  alignment_start_ = ros::Time();
  if (alignment_time_ > 0) {
    state().setSystemStatus(STATUS_ALIGNMENT);
  }

  // reset systems and measurements
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    (*it)->reset(state());
    (*it)->getPrior(state());
  }

  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    (*it)->reset(state());
  }

  updated();
}

void PoseEstimation::update(ros::Time new_timestamp)
{
  // check if system is initialized
  if (systems_.empty()) return;

  ros::Duration dt;
  if (!getTimestamp().isZero()) {
    if (new_timestamp.isZero()) new_timestamp = ros::Time::now();
    dt = new_timestamp - getTimestamp();
  }
  setTimestamp(new_timestamp);

  // do the update step
  update(dt.toSec());
}

void PoseEstimation::update(double dt)
{
  // check dt
  if (dt < -1.0)
    reset();
  else if (dt < 0.0)
    return;
  else if (dt > 1.0)
    dt = 1.0;

  // check if system and filter is initialized
  if (systems_.empty() || !filter_) return;

  // filter rate measurement first
  boost::shared_ptr<ImuInput> imu = getInputType<ImuInput>("imu");
  if (imu) {
    // Should the biases already be integrated here?
    // Note: The state set here only has an effect if the state vector does not have a rate/acceleration component.
    state().setRate(imu->getRate());
    state().setAcceleration(imu->getAcceleration() + state().R().row(2).transpose() * gravity_);

    if (state().rate() && rate_update_) {
      rate_update_->update(Rate::Update(imu->getRate()));
    }
  }

  // time update step
  filter_->predict(systems_, dt);

  // pseudo measurement updates (if required)
  if (imu && !(getSystemStatus() & STATE_ROLLPITCH)) {
    gravity_update_->enable();
    gravity_update_->update(Gravity::Update(imu->getAcceleration()));
  } else {
    gravity_update_->disable();
  }
  if (!(getSystemStatus() & STATE_RATE_Z)) {
    zerorate_update_->enable();
    zerorate_update_->update(ZeroRate::Update());
  } else {
    zerorate_update_->disable();
  }

  // measurement updates
  filter_->correct(measurements_);

  // updated hook
  updated();

  // set measurement status and increase timers
  SystemStatus measurement_status = 0;
  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); it++) {
    const MeasurementPtr& measurement = *it;
    measurement_status |= measurement->getStatusFlags();
    measurement->increase_timer(dt);
  }
  setMeasurementStatus(measurement_status);

  // set system status
  SystemStatus system_status = 0;
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); it++) {
    const SystemPtr& system = *it;
    system_status |= system->getStatusFlags();
  }
  updateSystemStatus(system_status, STATE_MASK | STATE_PSEUDO_MASK);

  // check for invalid state
  if (!state().valid()) {
    ROS_FATAL("Invalid state, resetting...");
    reset();
    return;
  }

  // switch overall system status
  if (inSystemStatus(STATUS_ALIGNMENT)) {
    if (alignment_start_.isZero()) alignment_start_ = getTimestamp();
    if ((getTimestamp() - alignment_start_).toSec() >= alignment_time_) {
      updateSystemStatus(STATUS_DEGRADED, STATUS_ALIGNMENT);
    }
  } else if (inSystemStatus(STATE_ROLLPITCH | STATE_YAW | STATE_POSITION_XY | STATE_POSITION_Z)) {
    updateSystemStatus(STATUS_READY, STATUS_DEGRADED);
  } else {
    updateSystemStatus(STATUS_DEGRADED, STATUS_READY);
  }


#ifdef EIGEN_RUNTIME_NO_MALLOC
  // No memory allocations allowed after the first update!
  Eigen::internal::set_is_malloc_allowed(false);
#endif
}

void PoseEstimation::updated() {
  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    (*it)->limitState(state());
  }
}

const SystemPtr& PoseEstimation::addSystem(const SystemPtr& system, const std::string& name) {
  if (!name.empty() && system->getName().empty()) system->setName(name);
  parameters().add(system->getName(), system->parameters());
  return systems_.add(system, system->getName());
}

InputPtr PoseEstimation::addInput(const InputPtr& input, const std::string& name)
{
  if (!name.empty()) input->setName(name);
  return inputs_.add(input, input->getName());
}

InputPtr PoseEstimation::setInput(const Input& value, std::string name)
{
  if (name.empty()) name = value.getName();
  InputPtr input = inputs_.get(name);
  if (!input) {
    ROS_WARN("Set input \"%s\", but this input is not registered by any system model.", name.c_str());
    return InputPtr();
  }

  *input = value;
  return input;
}

const MeasurementPtr& PoseEstimation::addMeasurement(const MeasurementPtr& measurement, const std::string& name) {
  if (!name.empty()) measurement->setName(name);
  parameters().add(measurement->getName(), measurement->parameters());
  return measurements_.add(measurement, measurement->getName());
}

const State::Vector& PoseEstimation::getStateVector() {
//  if (state_is_dirty_) {
//    state_ = filter_->PostGet()->ExpectedValueGet();
//    state_is_dirty_ = false;
//  }
  return state().getVector();
}

const State::Covariance& PoseEstimation::getCovariance() {
//  if (covariance_is_dirty_) {
//    covariance_ = filter_->PostGet()->CovarianceGet();
//    covariance_is_dirty_ = false;
//  }
  return state().getCovariance();
}

SystemStatus PoseEstimation::getSystemStatus() const {
  return state().getSystemStatus();
}

SystemStatus PoseEstimation::getMeasurementStatus() const {
  return state().getMeasurementStatus();
}

bool PoseEstimation::inSystemStatus(SystemStatus test_status) const {
  return state().inSystemStatus(test_status);
}

bool PoseEstimation::setSystemStatus(SystemStatus new_status) {
  return state().setSystemStatus(new_status);
}

bool PoseEstimation::setMeasurementStatus(SystemStatus new_measurement_status) {
  return state().setMeasurementStatus(new_measurement_status);
}

bool PoseEstimation::updateSystemStatus(SystemStatus set, SystemStatus clear) {
  return state().updateSystemStatus(set, clear);
}

bool PoseEstimation::updateMeasurementStatus(SystemStatus set, SystemStatus clear) {
  return state().updateMeasurementStatus(set, clear);
}

const ros::Time& PoseEstimation::getTimestamp() const {
  return state().getTimestamp();
}

void PoseEstimation::setTimestamp(const ros::Time& timestamp) {
  state().setTimestamp(timestamp);
}

void PoseEstimation::getHeader(std_msgs::Header& header) {
  header.stamp = getTimestamp();
  header.frame_id = nav_frame_;
}

void PoseEstimation::getState(nav_msgs::Odometry& msg, bool with_covariances) {
  getHeader(msg.header);
  getPose(msg.pose.pose);
  getVelocity(msg.twist.twist.linear);
  getRate(msg.twist.twist.angular);
  msg.child_frame_id = base_frame_;

  // rotate body vectors to nav frame
  ColumnVector3 rate_nav = state().R() * ColumnVector3(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
  msg.twist.twist.angular.x = rate_nav.x();
  msg.twist.twist.angular.y = rate_nav.y();
  msg.twist.twist.angular.z = rate_nav.z();

  // fill covariances
  if (with_covariances) {
    Eigen::Map< Eigen::Matrix<geometry_msgs::PoseWithCovariance::_covariance_type::value_type,6,6> >  pose_covariance_msg(msg.pose.covariance.data());
    Eigen::Map< Eigen::Matrix<geometry_msgs::TwistWithCovariance::_covariance_type::value_type,6,6> > twist_covariance_msg(msg.twist.covariance.data());

    // position covariance
    if (state().position()) {
      pose_covariance_msg.block<3,3>(0,0) = state().position()->getCovariance();
    }

    // rotation covariance (world-fixed)
    if (state().orientation()) {
      pose_covariance_msg.block<3,3>(3,3) = state().orientation()->getCovariance();
    }

    // position/orientation cross variance
    if (state().position() && state().orientation()) {
      pose_covariance_msg.block<3,3>(0,3) = state().orientation()->getCrossVariance(*state().position());
      pose_covariance_msg.block<3,3>(3,0) = pose_covariance_msg.block<3,3>(0,3).transpose();
    }

    // velocity covariance
    if (state().velocity()) {
      twist_covariance_msg.block<3,3>(0,0) = state().velocity()->getCovariance();
    }

    // angular rate covariance
    if (state().rate()) {
      twist_covariance_msg.block<3,3>(3,3) = state().rate()->getCovariance();
    }

    // cross velocity/angular_rate variance
    if (state().velocity() && state().rate()) {
      pose_covariance_msg.block<3,3>(0,3) = state().velocity()->getCrossVariance(*state().rate());
      pose_covariance_msg.block<3,3>(3,0) = pose_covariance_msg.block<3,3>(0,3).transpose();
    }
  }
}

void PoseEstimation::getPose(tf::Pose& pose) {
  tf::Quaternion quaternion;
  getPosition(pose.getOrigin());
  getOrientation(quaternion);
  pose.setRotation(quaternion);
}

void PoseEstimation::getPose(tf::Stamped<tf::Pose>& pose) {
  getPose(static_cast<tf::Pose &>(pose));
  pose.stamp_ = getTimestamp();
  pose.frame_id_ = nav_frame_;
}

void PoseEstimation::getPose(geometry_msgs::Pose& pose) {
  getPosition(pose.position);
  getOrientation(pose.orientation);
}

void PoseEstimation::getPose(geometry_msgs::PoseStamped& pose) {
  getHeader(pose.header);
  getPose(pose.pose);
}

void PoseEstimation::getPosition(tf::Point& point) {
  State::ConstPositionType position(state().getPosition());
  point = tf::Point(position.x(), position.y(), position.z());
}

void PoseEstimation::getPosition(tf::Stamped<tf::Point>& point) {
  getPosition(static_cast<tf::Point &>(point));
  point.stamp_ = getTimestamp();
  point.frame_id_ = nav_frame_;
}

void PoseEstimation::getPosition(geometry_msgs::Point& point) {
  State::ConstPositionType position(state().getPosition());
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
}

void PoseEstimation::getPosition(geometry_msgs::PointStamped& point) {
  getHeader(point.header);
  getPosition(point.point);
}

void PoseEstimation::getGlobal(double &latitude, double &longitude, double &altitude) {
  State::ConstPositionType position(state().getPosition());
  double north =  position.x() * globalReference()->heading().cos - position.y() * globalReference()->heading().sin;
  double east  = -position.x() * globalReference()->heading().sin - position.y() * globalReference()->heading().cos;
  latitude  = globalReference()->position().latitude  + north / globalReference()->radius().north;
  longitude = globalReference()->position().longitude + east  / globalReference()->radius().east;
  altitude  = globalReference()->position().altitude  + position.z();
}

void PoseEstimation::getGlobalPosition(double &latitude, double &longitude, double &altitude) {
  getGlobal(latitude, longitude, altitude);
}

void PoseEstimation::getGlobal(geographic_msgs::GeoPoint& global)
{
  getGlobalPosition(global.latitude, global.longitude, global.altitude);
  global.latitude  *= 180.0/M_PI;
  global.longitude *= 180.0/M_PI;
}

void PoseEstimation::getGlobal(sensor_msgs::NavSatFix& global)
{
  getHeader(global.header);
  global.header.frame_id = world_frame_;

  if ((getSystemStatus() & STATE_POSITION_XY) && globalReference()->hasPosition()) {
    global.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  } else {
    global.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  }

  getGlobalPosition(global.latitude, global.longitude, global.altitude);
  global.latitude  *= 180.0/M_PI;
  global.longitude *= 180.0/M_PI;

  if (getSystemStatus() & STATE_POSITION_XY) {
    global.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  } else {
    global.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  }
}

void PoseEstimation::getGlobalPosition(sensor_msgs::NavSatFix& global)
{
  getGlobal(global);
}

void PoseEstimation::getGlobal(geographic_msgs::GeoPoint& position, geometry_msgs::Quaternion& quaternion)
{
  getGlobal(position);
  Quaternion global_orientation = globalReference()->heading().quaternion() * Quaternion(state().getOrientation());
  quaternion.w = global_orientation.w();
  quaternion.x = global_orientation.x();
  quaternion.y = global_orientation.y();
  quaternion.z = global_orientation.z();
}

void PoseEstimation::getGlobal(geographic_msgs::GeoPose& global)
{
  getGlobal(global.position, global.orientation);
}

void PoseEstimation::getOrientation(tf::Quaternion& quaternion) {
  Quaternion orientation(state().getOrientation());
  quaternion = tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
}

void PoseEstimation::getOrientation(tf::Stamped<tf::Quaternion>& quaternion) {
  getOrientation(static_cast<tf::Quaternion &>(quaternion));
  quaternion.stamp_ = getTimestamp();
  quaternion.frame_id_ = nav_frame_;
}

void PoseEstimation::getOrientation(geometry_msgs::Quaternion& quaternion) {
  Quaternion orientation(state().getOrientation());
  quaternion.w = orientation.w();
  quaternion.x = orientation.x();
  quaternion.y = orientation.y();
  quaternion.z = orientation.z();
}

void PoseEstimation::getOrientation(geometry_msgs::QuaternionStamped& quaternion) {
  getHeader(quaternion.header);
  getOrientation(quaternion.quaternion);
}

void PoseEstimation::getOrientation(double &yaw, double &pitch, double &roll) {
  tf::Quaternion quaternion;
  getOrientation(quaternion);
#ifdef TF_MATRIX3x3_H
  tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
#else
  btMatrix3x3(quaternion).getRPY(roll, pitch, yaw);
#endif
}

void PoseEstimation::getImuWithBiases(geometry_msgs::Vector3& linear_acceleration, geometry_msgs::Vector3& angular_velocity) {
  boost::shared_ptr<const ImuInput>  input     = boost::dynamic_pointer_cast<const ImuInput>(getInput("imu"));
  boost::shared_ptr<const Accelerometer> accel = boost::dynamic_pointer_cast<const Accelerometer>(getSystem("accelerometer"));

  if (input) {
    linear_acceleration.x = input->getAcceleration().x();
    linear_acceleration.y = input->getAcceleration().y();
    linear_acceleration.z = input->getAcceleration().z();
  } else {
    linear_acceleration.x = 0.0;
    linear_acceleration.y = 0.0;
    linear_acceleration.z = 0.0;
  }

  if (accel) {
    linear_acceleration.x -= accel->getModel()->getError().x();
    linear_acceleration.y -= accel->getModel()->getError().y();
    linear_acceleration.z -= accel->getModel()->getError().z();
  }

  getRate(angular_velocity);
}

void PoseEstimation::getVelocity(tf::Vector3& vector) {
  State::ConstVelocityType velocity(state().getVelocity());
  vector = tf::Vector3(velocity.x(), velocity.y(), velocity.z());
}

void PoseEstimation::getVelocity(tf::Stamped<tf::Vector3>& vector) {
  getVelocity(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = getTimestamp();
  vector.frame_id_ = nav_frame_;
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3& vector) {
  State::ConstVelocityType velocity(state().getVelocity());
  vector.x = velocity.x();
  vector.y = velocity.y();
  vector.z = velocity.z();
}

void PoseEstimation::getVelocity(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getVelocity(vector.vector);
}

void PoseEstimation::getRate(tf::Vector3& vector) {
  geometry_msgs::Vector3 rate;
  getRate(rate);
  vector = tf::Vector3(rate.x, rate.y, rate.z);
}

void PoseEstimation::getRate(tf::Stamped<tf::Vector3>& vector) {
  getRate(static_cast<tf::Vector3 &>(vector));
  vector.stamp_ = getTimestamp();
  vector.frame_id_ = base_frame_;
}

void PoseEstimation::getRate(geometry_msgs::Vector3& vector) {
  if (state().rate()) {
    State::ConstRateType rate(state().getRate());
    vector.x    = rate.x();
    vector.y    = rate.y();
    vector.z    = rate.z();

  } else {
    boost::shared_ptr<const ImuInput> input = boost::dynamic_pointer_cast<const ImuInput>(getInput("imu"));
    boost::shared_ptr<const Gyro> gyro      = boost::dynamic_pointer_cast<const Gyro>(getSystem("gyro"));

    if (input) {
      vector.x = input->getRate().x();
      vector.y = input->getRate().y();
      vector.z = input->getRate().z();
    } else {
      vector.x = 0.0;
      vector.y = 0.0;
      vector.z = 0.0;
    }

    if (gyro) {
      vector.x -= gyro->getModel()->getError().x();
      vector.y -= gyro->getModel()->getError().y();
      vector.z -= gyro->getModel()->getError().z();
    }
  }
}

void PoseEstimation::getRate(geometry_msgs::Vector3Stamped& vector) {
  getHeader(vector.header);
  getRate(vector.vector);
  vector.header.frame_id = base_frame_;
}

void PoseEstimation::getBias(geometry_msgs::Vector3& angular_velocity, geometry_msgs::Vector3& linear_acceleration) {
  boost::shared_ptr<const Accelerometer> accel = boost::dynamic_pointer_cast<const Accelerometer>(getSystem("accelerometer"));
  boost::shared_ptr<const Gyro> gyro           = boost::dynamic_pointer_cast<const Gyro>(getSystem("gyro"));

  if (gyro) {
    angular_velocity.x = gyro->getModel()->getError().x();
    angular_velocity.y = gyro->getModel()->getError().y();
    angular_velocity.z = gyro->getModel()->getError().z();
  } else {
    angular_velocity.x = 0.0;
    angular_velocity.y = 0.0;
    angular_velocity.z = 0.0;
  }

  if (accel) {
    linear_acceleration.x = accel->getModel()->getError().x();
    linear_acceleration.y = accel->getModel()->getError().y();
    linear_acceleration.z = accel->getModel()->getError().z();
  } else {
    linear_acceleration.x = 0.0;
    linear_acceleration.y = 0.0;
    linear_acceleration.z = 0.0;
  }
}

void PoseEstimation::getBias(geometry_msgs::Vector3Stamped& angular_velocity, geometry_msgs::Vector3Stamped& linear_acceleration) {
  getBias(angular_velocity.vector, linear_acceleration.vector);
  angular_velocity.header.stamp = getTimestamp();
  angular_velocity.header.frame_id = base_frame_;
  linear_acceleration.header.stamp = getTimestamp();
  linear_acceleration.header.frame_id = base_frame_;
}

void PoseEstimation::getTransforms(std::vector<tf::StampedTransform>& transforms) {
  tf::Quaternion orientation;
  tf::Point position;
  getOrientation(orientation);
  getPosition(position);

  tf::Transform transform(orientation, position);
  double y,p,r;
  transform.getBasis().getEulerYPR(y,p,r);

  std::string parent_frame = nav_frame_;

  if(!position_frame_.empty()) {
    tf::Transform position_transform;
    position_transform.getBasis().setIdentity();
    position_transform.setOrigin(tf::Point(position.x(), position.y(), position.z()));
    transforms.push_back(tf::StampedTransform(position_transform, getTimestamp(), parent_frame, position_frame_ ));
  }

  if (!footprint_frame_.empty()) {
    tf::Transform footprint_transform;
    footprint_transform.getBasis().setEulerYPR(y, 0.0, 0.0);
    footprint_transform.setOrigin(tf::Point(position.x(), position.y(), 0.0));
    transforms.push_back(tf::StampedTransform(footprint_transform, getTimestamp(), parent_frame, footprint_frame_));

    parent_frame = footprint_frame_;
    transform = footprint_transform.inverseTimes(transform);
  }

  if (!stabilized_frame_.empty()) {
    tf::Transform stabilized_transform(transform);
#ifdef TF_MATRIX3x3_H
    tf::Matrix3x3 rollpitch_rotation; rollpitch_rotation.setEulerYPR(0.0, p, r);
#else
    btMatrix3x3 rollpitch_rotation; rollpitch_rotation.setEulerYPR(0.0, p, r);
#endif
    stabilized_transform = stabilized_transform * tf::Transform(rollpitch_rotation.inverse());
    transforms.push_back(tf::StampedTransform(stabilized_transform, getTimestamp(), parent_frame, stabilized_frame_));

    parent_frame = stabilized_frame_;
    transform = stabilized_transform.inverseTimes(transform);
  }

  transforms.push_back(tf::StampedTransform(transform, getTimestamp(), parent_frame, base_frame_));

//  transforms.resize(3);

//  transforms[0].stamp_ = getTimestamp();
//  transforms[0].frame_id_ = nav_frame_;
//  transforms[0].child_frame_id_ = footprint_frame_;
//  transforms[0].setOrigin(tf::Point(position.x(), position.y(), 0.0));
//  rotation.setEulerYPR(y,0.0,0.0);
//  transforms[0].setBasis(rotation);

//  transforms[1].stamp_ = getTimestamp();
//  transforms[1].frame_id_ = footprint_frame_;
//  transforms[1].child_frame_id_ = stabilized_frame_;
//  transforms[1].setIdentity();
//  transforms[1].setOrigin(tf::Point(0.0, 0.0, position.z()));

//  transforms[2].stamp_ = getTimestamp();
//  transforms[2].frame_id_ = stabilized_frame_;
//  transforms[2].child_frame_id_ = base_frame_;
//  transforms[2].setIdentity();
//  rotation.setEulerYPR(0.0,p,r);
//  transforms[2].setBasis(rotation);
}

void PoseEstimation::updateWorldToOtherTransform(tf::StampedTransform& world_to_other_transform) {
  world_to_other_transform.frame_id_ = world_frame_;

  double y,p,r;
  world_to_other_transform.getBasis().getEulerYPR(y,p,r);
  if (!(getSystemStatus() & (STATE_ROLLPITCH   | STATE_PSEUDO_ROLLPITCH)))   { r = p = 0.0; }
  if (!(getSystemStatus() & (STATE_YAW         | STATE_PSEUDO_YAW)))         { y = 0.0; }
  if (!(getSystemStatus() & (STATE_POSITION_XY | STATE_PSEUDO_POSITION_XY))) { world_to_other_transform.getOrigin().setX(0.0); world_to_other_transform.getOrigin().setY(0.0); }
  if (!(getSystemStatus() & (STATE_POSITION_Z  | STATE_PSEUDO_POSITION_Z)))  { world_to_other_transform.getOrigin().setZ(0.0); }
  world_to_other_transform.getBasis().setEulerYPR(y, p, r);
}

bool PoseEstimation::getWorldToNavTransform(geometry_msgs::TransformStamped& transform) {
  return globalReference()->getWorldToNavTransform(transform, world_frame_, nav_frame_, getTimestamp());
}

const GlobalReferencePtr &PoseEstimation::globalReference() {
  return GlobalReference::Instance();
}

} // namespace hector_pose_estimation
