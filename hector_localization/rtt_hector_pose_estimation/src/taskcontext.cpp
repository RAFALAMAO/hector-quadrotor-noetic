//=================================================================================================
// Copyright (c) 2012, Johannes Meyer and Martin Nowara, TU Darmstadt
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

#include <hector_pose_estimation/rtt/taskcontext.h>
#include "services.h"
#include "parameters.h"

#include <hector_pose_estimation/system/generic_quaternion_system_model.h>
#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/measurements/height.h>
#include <hector_pose_estimation/measurements/magnetic.h>
#include <hector_pose_estimation/measurements/gps.h>

#include <hector_pose_estimation/ros/parameters.h>

#include <rtt/Component.hpp>

namespace hector_pose_estimation {

PoseEstimationTaskContext::PoseEstimationTaskContext(const std::string& name, const SystemPtr& system)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  , PoseEstimation(system)
{
  if (!system) addSystem(System::create(new GenericQuaternionSystemModel));

  this->addEventPort("raw_imu", imu_input_);
  this->addPort("poseupdate", pose_update_input_);
  this->addPort("magnetic", magnetic_input_);
  this->addPort("pressure_height", height_input_);
  this->addPort("fix", gps_input_);
  this->addPort("fix_velocity", gps_velocity_input_);
  this->addPort("state", state_output_);
  this->addPort("imu", imu_output_);
  this->addPort("pose", pose_output_);
  this->addPort("velocity", velocity_output_);
  this->addPort("global", global_position_output_);
  this->addPort("angular_velocity_bias", angular_velocity_bias_output_);
  this->addPort("linear_acceleration_bias", linear_acceleration_bias_output_);
  this->addEventPort("syscommand", command_input_, boost::bind(&PoseEstimationTaskContext::commandCallback, this, _1));

  param_namespace_ = "~";
  this->addProperty("param_namespace", param_namespace_);

  this->addOperation("reset", &PoseEstimationTaskContext::reset, this, RTT::OwnThread);
  this->addOperation("getSystemStatus", &PoseEstimation::getSystemStatus, static_cast<PoseEstimation *>(this), RTT::OwnThread);
  this->addOperation("getMeasurementStatus", &PoseEstimation::getMeasurementStatus, static_cast<PoseEstimation *>(this), RTT::OwnThread);

  PoseEstimation::addMeasurement(new PoseUpdate("PoseUpdate"));
  PoseEstimation::addMeasurement(new Height("Height"));
  PoseEstimation::addMeasurement(new Magnetic("Magnetic"));
  PoseEstimation::addMeasurement(new GPS("GPS"));

  for(Systems::iterator it = systems_.begin(); it != systems_.end(); ++it) {
    this->provides()->addService(RTT::Service::shared_ptr(new SystemService(this, *it)));
  }

  for(Measurements::iterator it = measurements_.begin(); it != measurements_.end(); ++it) {
    this->provides()->addService(RTT::Service::shared_ptr(new MeasurementService(this, *it)));
  }

  PoseEstimation::parameters().initialize(ParameterRegistryROS(ros::NodeHandle(param_namespace_)));
  PoseEstimation::parameters().initialize(ParameterRegistryProperties(this->properties()));
}

PoseEstimationTaskContext::~PoseEstimationTaskContext()
{
  stop();
  cleanup();
}

bool PoseEstimationTaskContext::configureHook()
{
  if (!PoseEstimation::init()) {
    RTT::log(RTT::Error) << "Intitialization of pose estimation failed!" << RTT::endlog();
    return false;
  }

  // publish initial state
  updateOutputs();

  return true;
}

void PoseEstimationTaskContext::cleanupHook()
{
  PoseEstimation::cleanup();
}

bool PoseEstimationTaskContext::startHook()
{
  return true;
}

void PoseEstimationTaskContext::stopHook()
{
}

void PoseEstimationTaskContext::updateHook()
{
  while(magnetic_input_.read(magnetic_) == RTT::NewData && PoseEstimation::getMeasurement("Magnetic"))
  {
    Magnetic::MeasurementVector update(3);
    update.x() = magnetic_.vector.x;
    update.y() = magnetic_.vector.y;
    update.z() = magnetic_.vector.z;
    PoseEstimation::getMeasurement("Magnetic")->add(Magnetic::Update(update));
  }

  while(height_input_.read(height_) == RTT::NewData && PoseEstimation::getMeasurement("Height"))
  {
    Height::Update update;
#ifdef USE_MAV_MSGS
    update = height_.height;
#else
    update = height_.point.z;
#endif
    PoseEstimation::getMeasurement("Height")->add(update);
  }

  while(gps_input_.read(gps_) == RTT::NewData && gps_velocity_input_.read(gps_velocity_) != RTT::NoData && PoseEstimation::getMeasurement("GPS"))
  {
    if (gps_.status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    {
      GPS::Update update;
      update.latitude = gps_.latitude * M_PI/180.0;
      update.longitude = gps_.longitude * M_PI/180.0;
      update.velocity_north =  gps_velocity_.vector.x;
      update.velocity_east  = -gps_velocity_.vector.y;
      PoseEstimation::getMeasurement("GPS")->add(update);
    }
  }

  while(pose_update_input_.read(pose_update_) == RTT::NewData && PoseEstimation::getMeasurement("PoseUpdate"))
  {
    PoseEstimation::getMeasurement("PoseUpdate")->add(PoseUpdate::Update(pose_update_));
  }

  while(imu_input_.read(imu_in_) == RTT::NewData) {
    setInput(ImuInput(imu_in_));
    PoseEstimation::update(imu_in_.header.stamp);
    updateOutputs();
  }
}

void PoseEstimationTaskContext::updateOutputs()
{
  getState(state_);
  state_output_.write(state_);

  if (imu_output_.connected()) {
    imu_out_.header = state_.header;
    imu_out_.orientation = state_.pose.pose.orientation;
    getImuWithBiases(imu_out_.linear_acceleration, imu_out_.angular_velocity);
    imu_output_.write(imu_out_);
  }

  if (pose_output_.connected()) {
    pose_.header = state_.header;
    pose_.pose = state_.pose.pose;
    pose_output_.write(pose_);
  }

  if (velocity_output_.connected()) {
    velocity_.header = state_.header;
    velocity_.vector = state_.twist.twist.linear;
    velocity_output_.write(velocity_);
  }

  if (global_position_output_.connected()) {
    getGlobalPosition(global_position_);
    global_position_output_.write(global_position_);
  }

  if (angular_velocity_bias_output_.connected() || linear_acceleration_bias_output_.connected()) {
    getHeader(angular_velocity_bias_.header);
    getHeader(linear_acceleration_bias_.header);
    getBias(angular_velocity_bias_, linear_acceleration_bias_);
    angular_velocity_bias_output_.write(angular_velocity_bias_);
    linear_acceleration_bias_output_.write(linear_acceleration_bias_);
  }
}


void PoseEstimationTaskContext::reset() {
  RTT::log(RTT::Info) << "Resetting pose_estimation" << RTT::endlog();
  PoseEstimation::reset();
  updateOutputs();
}

void PoseEstimationTaskContext::commandCallback(RTT::base::PortInterface *) {
  std_msgs::String command;
  if (command_input_.read(command) == RTT::NewData) {
    if (command.data == "reset") reset();
  }
}

} // namespace

ORO_CREATE_COMPONENT( hector_pose_estimation::PoseEstimationTaskContext )
