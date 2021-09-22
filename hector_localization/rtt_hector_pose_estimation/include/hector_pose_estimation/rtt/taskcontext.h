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

#ifndef HECTOR_POSE_ESTIMATION_RTT_TASKCONTEXT_H
#define HECTOR_POSE_ESTIMATION_RTT_TASKCONTEXT_H

#include <hector_pose_estimation/pose_estimation.h>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>

#include <sensor_msgs/typekit/Imu.h>

#include <nav_msgs/typekit/Odometry.h>
#include <geometry_msgs/typekit/PoseStamped.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>
#ifdef USE_MAV_MSGS
  #include <mav_msgs/Height.h>
#else
  #include <geometry_msgs/typekit/PointStamped.h>
#endif
#include <geometry_msgs/typekit/PoseWithCovarianceStamped.h>
#include <sensor_msgs/typekit/NavSatFix.h>
#include <std_msgs/String.h>

#include <ros/ros.h>

namespace hector_pose_estimation {

class PoseEstimationTaskContext : public RTT::TaskContext, PoseEstimation {
public:
  PoseEstimationTaskContext(const std::string& name = "PoseEsimation", const SystemPtr& system = SystemPtr());
  virtual ~PoseEstimationTaskContext();

  void updateOutputs();
  void reset();

  using TaskCore::cleanup;

protected:
  bool configureHook();
  void cleanupHook();

  bool startHook();
  void stopHook();

  void updateHook();

  void commandCallback(RTT::base::PortInterface *);

private:
  RTT::InputPort<sensor_msgs::Imu> imu_input_;
  sensor_msgs::Imu imu_in_;

  RTT::InputPort<geometry_msgs::PoseWithCovarianceStamped> pose_update_input_;
  geometry_msgs::PoseWithCovarianceStamped pose_update_;

  RTT::InputPort<geometry_msgs::Vector3Stamped> magnetic_input_;
  geometry_msgs::Vector3Stamped magnetic_;

#ifdef USE_MAV_MSGS
  RTT::InputPort<mav_msgs::Height> height_input_;
  mav_msgs::Height height_;
#else
  RTT::InputPort<geometry_msgs::PointStamped> height_input_;
  geometry_msgs::PointStamped height_;
#endif

  RTT::InputPort<sensor_msgs::NavSatFix> gps_input_;
  RTT::InputPort<geometry_msgs::Vector3Stamped> gps_velocity_input_;
  sensor_msgs::NavSatFix gps_;
  geometry_msgs::Vector3Stamped gps_velocity_;

  RTT::OutputPort<nav_msgs::Odometry> state_output_;
  nav_msgs::Odometry state_;

  RTT::OutputPort<sensor_msgs::Imu> imu_output_;
  sensor_msgs::Imu imu_out_;

  RTT::OutputPort<geometry_msgs::PoseStamped> pose_output_;
  geometry_msgs::PoseStamped pose_;

  RTT::OutputPort<geometry_msgs::Vector3Stamped> velocity_output_;
  geometry_msgs::Vector3Stamped velocity_;

  RTT::OutputPort<sensor_msgs::NavSatFix> global_position_output_;
  sensor_msgs::NavSatFix global_position_;

  RTT::OutputPort<geometry_msgs::Vector3Stamped> angular_velocity_bias_output_, linear_acceleration_bias_output_;
  geometry_msgs::Vector3Stamped angular_velocity_bias_;
  geometry_msgs::Vector3Stamped linear_acceleration_bias_;

  RTT::InputPort<std_msgs::String> command_input_;

  ros::NodeHandle node_handle_;
  std::string param_namespace_;
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_RTT_TASKCONTEXT_H
