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

#ifndef HECTOR_POSE_ESTIMATION_POSEUPDATE_H
#define HECTOR_POSE_ESTIMATION_POSEUPDATE_H

#include <hector_pose_estimation/measurement.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <boost/function.hpp>

namespace hector_pose_estimation {

class PositionXYModel : public MeasurementModel_<PositionXYModel,2> {
public:
  PositionXYModel() {}
  virtual ~PositionXYModel() {}

  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);

  void updateState(State& state, const ColumnVector &diff) const;
};

class PositionZModel : public MeasurementModel_<PositionZModel,1> {
public:
  PositionZModel() {}
  virtual ~PositionZModel() {}

  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);

  void updateState(State& state, const ColumnVector &diff) const;
};

class YawModel : public MeasurementModel_<YawModel,1> {
public:
  YawModel() {}
  virtual ~YawModel() {}

  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);

  void updateState(State& state, const ColumnVector &diff) const;
};

class TwistModel : public MeasurementModel_<TwistModel,6> {
public:
  TwistModel() {}
  virtual ~TwistModel() {}

  virtual void getExpectedValue(MeasurementVector& y_pred, const State& state);
  virtual void getStateJacobian(MeasurementMatrix& C, const State& state, bool init);
};

class PoseUpdate : public Measurement
{
public:
  PoseUpdate(const std::string& name = "poseupdate");
  virtual ~PoseUpdate();

  class Update : public MeasurementUpdate {
  public:
    Update() {}
    Update(const geometry_msgs::PoseWithCovarianceStamped& pose) : pose(new geometry_msgs::PoseWithCovarianceStamped(pose)) {}
    Update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) : pose(pose) {}
    Update(const geometry_msgs::TwistWithCovarianceStamped& twist) : twist(new geometry_msgs::TwistWithCovarianceStamped(twist)) {}
    Update(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist) : twist(twist) {}
    Update(const geometry_msgs::PoseWithCovarianceStamped& pose, const geometry_msgs::TwistWithCovarianceStamped& twist) : pose(new geometry_msgs::PoseWithCovarianceStamped(pose)), twist(new geometry_msgs::TwistWithCovarianceStamped(twist)) {}
    Update(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist) : pose(pose), twist(twist) {}
    geometry_msgs::PoseWithCovarianceStampedConstPtr pose;
    geometry_msgs::TwistWithCovarianceStampedConstPtr twist;
  };

  virtual bool updateImpl(const MeasurementUpdate &update);

private:
  PositionXYModel position_xy_model_;
  PositionZModel position_z_model_;
  YawModel yaw_model_;
  TwistModel twist_model_;

  double fixed_alpha_, fixed_beta_;
  bool interpret_covariance_as_information_matrix_;
  double max_time_difference_;
  bool predict_pose_;
  bool jump_on_max_error_;

  double fixed_position_xy_stddev_;
  double fixed_position_z_stddev_;
  double fixed_yaw_stddev_;

  double fixed_velocity_xy_stddev_;
  double fixed_velocity_z_stddev_;
  double fixed_angular_rate_xy_stddev_;
  double fixed_angular_rate_z_stddev_;

  double max_position_xy_error_;
  double max_position_z_error_;
  double max_yaw_error_;

  double max_velocity_xy_error_;
  double max_velocity_z_error_;
  double max_angular_rate_xy_error_;
  double max_angular_rate_z_error_;

  typedef boost::function<void(State &state, const ColumnVector &diff)> JumpFunction;
  double calculateOmega(const SymmetricMatrix &Ix, const SymmetricMatrix &Iy);

  template <typename MeasurementVector, typename MeasurementMatrix, typename NoiseVariance>
  double updateInternal(State &state, const NoiseVariance &Iy, const MeasurementVector &error, const MeasurementMatrix &H, const std::string& text, const double max_error = 0.0, JumpFunction jump_function = JumpFunction());

protected:
  Queue_<Update> queue_;
  virtual Queue& queue() { return queue_; }
};

} // namespace hector_pose_estimation

#endif // HECTOR_POSE_ESTIMATION_POSEUPDATE_H
