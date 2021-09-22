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

#include <hector_pose_estimation/measurements/poseupdate.h>
#include <hector_pose_estimation/pose_estimation.h>

#include <Eigen/Core>

#include <boost/bind.hpp>

namespace hector_pose_estimation {

PoseUpdate::PoseUpdate(const std::string& name)
  : Measurement(name)
{
  fixed_alpha_ = 0.0;
  fixed_beta_  = 0.0;
  interpret_covariance_as_information_matrix_ = true;

  max_time_difference_ = 1.0;
  predict_pose_ = true;

  jump_on_max_error_ = true;

  fixed_position_xy_stddev_ = 0.0;
  fixed_position_z_stddev_ = 0.0;
  fixed_yaw_stddev_ = 0.0;

  fixed_velocity_xy_stddev_ = 0.0;
  fixed_velocity_z_stddev_ = 0.0;
  fixed_angular_rate_xy_stddev_ = 0.0;
  fixed_angular_rate_z_stddev_ = 0.0;

  max_position_xy_error_ = 3.0; // 3 sigma
  max_position_z_error_ = 3.0; // 3 sigma
  max_yaw_error_ = 3.0; // 3 sigma

  max_velocity_xy_error_ = 3.0; // 3 sigma
  max_velocity_z_error_ = 3.0; // 3 sigma
  max_angular_rate_xy_error_ = 3.0; // 3 sigma
  max_angular_rate_z_error_ = 3.0; // 3 sigma

  parameters().add("fixed_alpha", fixed_alpha_);
  parameters().add("fixed_beta", fixed_beta_);
  parameters().add("interpret_covariance_as_information_matrix", interpret_covariance_as_information_matrix_);
  parameters().add("max_time_difference", max_time_difference_);
  parameters().add("predict_pose", predict_pose_);
  parameters().add("jump_on_max_error", jump_on_max_error_);

  parameters().add("fixed_position_xy_stddev", fixed_position_xy_stddev_);
  parameters().add("fixed_position_z_stddev", fixed_position_z_stddev_);
  parameters().add("fixed_yaw_stddev", fixed_yaw_stddev_);
  parameters().add("fixed_velocity_xy_stddev", fixed_velocity_xy_stddev_);
  parameters().add("fixed_velocity_z_stddev", fixed_velocity_z_stddev_);
  parameters().add("fixed_angular_rate_xy_stddev", fixed_angular_rate_xy_stddev_);
  parameters().add("fixed_angular_rate_z_stddev", fixed_angular_rate_z_stddev_);
  parameters().add("max_position_xy_error", max_position_xy_error_ );
  parameters().add("max_position_z_error", max_position_z_error_);
  parameters().add("max_yaw_error", max_yaw_error_);
  parameters().add("max_velocity_xy_error", max_velocity_xy_error_ );
  parameters().add("max_velocity_z_error", max_velocity_z_error_);
  parameters().add("max_angular_rate_xy_error", max_angular_rate_xy_error_ );
  parameters().add("max_angular_rate_z_error", max_angular_rate_z_error_);
}

PoseUpdate::~PoseUpdate()
{
}

bool PoseUpdate::updateImpl(const MeasurementUpdate &update_)
{
  Update const &update = static_cast<Update const &>(update_);

  while (update.pose) {
    // convert incoming update information to Eigen
    Eigen::Vector3d update_pose(update.pose->pose.pose.position.x, update.pose->pose.pose.position.y, update.pose->pose.pose.position.z);
    Eigen::Quaterniond update_orientation(update.pose->pose.pose.orientation.w, update.pose->pose.pose.orientation.x, update.pose->pose.pose.orientation.y, update.pose->pose.pose.orientation.z);
    Eigen::Vector3d update_euler;

    // information is the information matrix if interpret_covariance_as_information_matrix_ is true and a covariance matrix otherwise
    // zero elements are counted as zero information in any case
    SymmetricMatrix6 information(Eigen::Map<const SymmetricMatrix6>(update.pose->pose.covariance.data()));

    ROS_DEBUG_STREAM_NAMED("poseupdate", "PoseUpdate: x = [ " << filter()->state().getVector().transpose() << " ], P = [ " << filter()->state().getCovariance() << " ]" << std::endl
                                      << "update: pose = [ " << update_pose.transpose() << " ], rpy = [ " << update_euler.transpose() << " ], information = [ " << information << " ]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "dt = " << (filter()->state().getTimestamp() - update.pose->header.stamp).toSec() << " s");

    // predict update pose using the estimated velocity and degrade information
    if (!update.pose->header.stamp.isZero()) {
      double dt = (filter()->state().getTimestamp() - update.pose->header.stamp).toSec();
      if (dt < 0.0) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring pose update as it has a negative time difference: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0 && dt >= max_time_difference_) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring pose update as the time difference is too large: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0){
        if (interpret_covariance_as_information_matrix_)
          information = information * (1.0 - dt/max_time_difference_);
        else
          information = information / (1.0 - dt/max_time_difference_);
      }

      if (predict_pose_) {
        State::ConstVelocityType state_velocity(filter()->state().getVelocity());
        update_pose = update_pose + state_velocity * dt;

        State::ConstRateType state_rate(filter()->state().getRate());
        update_orientation = update_orientation * Eigen::Quaterniond().fromRotationVector(state_rate * dt);
      }
    }

    // Calculate euler angles
    {
        const Eigen::Quaterniond &q = update_orientation;
        /* roll  = */ update_euler(0) = atan2(2*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
        /* pitch = */ update_euler(1) = -asin(2*(q.x()*q.z() - q.w()*q.y()));
        /* yaw   = */ update_euler(2) = atan2(2*(q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    }

    // update PositionXY
    if (information(0,0) > 0.0 || information(1,1) > 0.0) {
      // fetch observation matrix H and current state x
      PositionXYModel::MeasurementMatrix H(position_xy_model_.getDimension(), filter()->state().getCovarianceDimension());
      PositionXYModel::MeasurementVector x(position_xy_model_.getDimension());
      position_xy_model_.getStateJacobian(H, filter()->state(), true);
      position_xy_model_.getExpectedValue(x, filter()->state());

      PositionXYModel::MeasurementVector y(update_pose.segment<2>(0));
      PositionXYModel::NoiseVariance Iy(information.block<2,2>(0,0));

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse().eval();

      // fixed_position_xy_stddev_ = 1.0;
      if (fixed_position_xy_stddev_ != 0.0) {
        Iy.setZero();
        Iy(0,0) = Iy(1,1) = 1.0 / (fixed_position_xy_stddev_*fixed_position_xy_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Position Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = [" << x.transpose() << "], H = [ " << H << " ], Px = [" <<  (H * filter()->state().P() * H.transpose()) << "], Ix = [ " << (H * filter()->state().P() * H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]");
      double innovation = updateInternal(filter()->state(), Iy, y - x, H, "position_xy", max_position_xy_error_, boost::bind(&PositionXYModel::updateState, &position_xy_model_, _1, _2));
      position_xy_model_.getExpectedValue(x, filter()->state());
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = [" << x << "], Pxy = [ " << (H * filter()->state().P() * H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_POSITION_XY;
    }

    // update PositionZ
    if (information(2,2) > 0.0) {
      // fetch observation matrix H and current state x
      PositionZModel::MeasurementMatrix H(position_z_model_.getDimension(), filter()->state().getCovarianceDimension());
      PositionZModel::MeasurementVector x(position_z_model_.getDimension());
      position_z_model_.getStateJacobian(H, filter()->state(), true);
      position_z_model_.getExpectedValue(x, filter()->state());

      PositionZModel::MeasurementVector y(update_pose.segment<1>(2));
      PositionZModel::NoiseVariance Iy(information.block<1,1>(2,2));

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse().eval();

      // fixed_position_z_stddev_ = 1.0;
      if (fixed_position_z_stddev_ != 0.0) {
        Iy.setZero();
        Iy(0,0) = 1.0 / (fixed_position_z_stddev_*fixed_position_z_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Height Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = " << x(0) << ", H = [ " << H << " ], Px = [" <<  (H * filter()->state().P() * H.transpose()) << "], Ix = [ " << (H * filter()->state().P() * H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = " << y(0) << ", Iy = [ " << Iy << " ]");
      double innovation = updateInternal(filter()->state(), Iy, y - x, H, "position_z", max_position_z_error_, boost::bind(&PositionZModel::updateState, &position_z_model_, _1, _2));
      position_z_model_.getExpectedValue(x, filter()->state());
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = " << x(0) << ", Pxy = [ " << (H * filter()->state().P() * H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_POSITION_Z;
    }

    // update Yaw
    if (information(5,5) > 0.0) {
      YawModel::MeasurementMatrix H(yaw_model_.getDimension(), filter()->state().getCovarianceDimension());
      YawModel::MeasurementVector x(yaw_model_.getDimension());
      yaw_model_.getStateJacobian(H, filter()->state(), true);
      yaw_model_.getExpectedValue(x, filter()->state());

      YawModel::MeasurementVector y(update_euler.tail<1>());
      YawModel::NoiseVariance Iy(information.block<1,1>(5,5));

      // invert Iy if information is a covariance matrix
      if (!interpret_covariance_as_information_matrix_) Iy = Iy.inverse().eval();

      // fixed_yaw_stddev_ = 5.0 * M_PI/180.0;
      if (fixed_yaw_stddev_ != 0.0) {
        Iy.setZero();
        Iy(0,0) = 1.0 / (fixed_yaw_stddev_*fixed_yaw_stddev_);
      }

      ROS_DEBUG_STREAM_NAMED("poseupdate", "Yaw Update: ");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = " << x(0) * 180.0/M_PI << "°, H = [ " << H << " ], Px = [" <<  (H * filter()->state().P() * H.transpose()) << "], Ix = [ " << (H * filter()->state().P() * H.transpose()).inverse() << "]");
      ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = " << y(0) * 180.0/M_PI << "°, Iy = [ " << Iy << " ]");

      YawModel::MeasurementVector error(y - x);
      error(0) = error(0) - 2.0*M_PI * round(error(0) / (2.0*M_PI));

      double innovation = updateInternal(filter()->state(), Iy, error, H, "yaw", max_yaw_error_, boost::bind(&YawModel::updateState, &yaw_model_, _1, _2));
      yaw_model_.getExpectedValue(x, filter()->state());
      ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = " << x(0) * 180.0/M_PI << "°, Pxy = [ " << (H * filter()->state().P() * H.transpose()) << " ], innovation = " << innovation);

      status_flags_ |= STATE_YAW;
    }

    break;
  }

  while (update.twist) {
    // convert incoming update information to Eigen
    Eigen::Vector3d update_linear(update.twist->twist.twist.linear.x, update.twist->twist.twist.linear.y, update.twist->twist.twist.linear.z);
    Eigen::Vector3d update_angular(update.twist->twist.twist.angular.x, update.twist->twist.twist.angular.y, update.twist->twist.twist.angular.z);

    // information is the information matrix if interpret_covariance_as_information_matrix_ is true and a covariance matrix otherwise
    // zero elements are counted as zero information in any case
    SymmetricMatrix6 information(Eigen::Map<const SymmetricMatrix6>(update.twist->twist.covariance.data()));

    ROS_DEBUG_STREAM_NAMED("poseupdate", "TwistUpdate:  state = [ " << filter()->state().getVector().transpose() << " ], P = [ " << filter()->state().getCovariance() << " ]" << std::endl
                                      << "     update: linear = [ " << update_linear.transpose() << " ], angular = [ " << update_angular.transpose() << " ], information = [ " << information << " ]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "                 dt = " << (filter()->state().getTimestamp() - update.twist->header.stamp).toSec() << " s");

    // degrade information if the time difference is too large
    if (!update.twist->header.stamp.isZero()) {
      double dt = (filter()->state().getTimestamp() - update.twist->header.stamp).toSec();
      if (dt < 0.0) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring twist update as it has a negative time difference: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0 && dt >= max_time_difference_) {
        ROS_DEBUG_STREAM_NAMED("poseupdate", "Ignoring twist update as the time difference is too large: dt = " << dt << "s");
        break;

      } else if (max_time_difference_ > 0.0){
        if (interpret_covariance_as_information_matrix_)
          information = information * (1.0 - dt/max_time_difference_);
        else
          information = information / (1.0 - dt/max_time_difference_);
      }
    }

    // fetch observation matrix H and current state x
    TwistModel::MeasurementMatrix H(twist_model_.getDimension(), filter()->state().getCovarianceDimension());
    TwistModel::MeasurementVector x(twist_model_.getDimension());
    twist_model_.getStateJacobian(H, filter()->state(), true);
    twist_model_.getExpectedValue(x, filter()->state());

    TwistModel::MeasurementVector y(twist_model_.getDimension());
    TwistModel::NoiseVariance Iy(information);
    y.segment<3>(0) = update_linear;
    y.segment<3>(3) = update_angular;

    // invert Iy if information is a covariance matrix
    if (!interpret_covariance_as_information_matrix_) {
      ROS_DEBUG_NAMED("poseupdate", "Twist updates with covariance matrices are currently not supported");
      break;
    }

    // update VelocityXY
    if (information(0,0) > 0.0 || information(0,0) > 0.0) {
      status_flags_ |= STATE_VELOCITY_XY;

      // fixed_velocity_xy_stddev_ = 1.0;
      if (fixed_velocity_xy_stddev_ != 0.0) {
        for(int i = 0; i < 6; ++i) Iy(0,i) = Iy(1,i) = Iy(i,0) = Iy(i,1) = 0.0;
        Iy(0,0) = Iy(1,1) = 1.0 / (fixed_velocity_xy_stddev_*fixed_velocity_xy_stddev_);
      }
    }

    // update VelocityZ
    if (information(2,2) > 0.0) {
      status_flags_ |= STATE_VELOCITY_Z;

      // fixed_velocity_z_stddev_ = 1.0;
      if (fixed_velocity_z_stddev_ != 0.0) {
          for(int i = 0; i < 6; ++i) Iy(2,i) = Iy(i,2) = 0.0;
        Iy(2,2) = 1.0 / (fixed_velocity_z_stddev_*fixed_velocity_z_stddev_);
      }
    }

    // update RateXY
    if (information(3,3) > 0.0 || information(4,4) > 0.0) {
      status_flags_ |= STATE_RATE_XY;

      // fixed_angular_rate_xy_stddev_ = 1.0;
      if (fixed_angular_rate_xy_stddev_ != 0.0) {
        for(int i = 0; i < 6; ++i) Iy(3,i) = Iy(3,i) = Iy(i,4) = Iy(i,4) = 0.0;
        Iy(4,4) = Iy(5,5) = 1.0 / (fixed_angular_rate_xy_stddev_*fixed_angular_rate_xy_stddev_);
      }
    }

    // update RateZ
    if (information(5,5) > 0.0) {
      status_flags_ |= STATE_RATE_Z;

      // fixed_angular_rate_z_stddev_ = 1.0;
      if (fixed_angular_rate_z_stddev_ != 0.0) {
        for(int i = 0; i < 6; ++i) Iy(5,i) = Iy(i,5) = 0.0;
        Iy(5,5) = 1.0 / (fixed_angular_rate_z_stddev_*fixed_angular_rate_z_stddev_);
      }
    }

    ROS_DEBUG_STREAM_NAMED("poseupdate", "Twist Update: ");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "      x = [" << x.transpose() << "], H = [ " << H << " ], Px = [" <<  (H * filter()->state().P() * H.transpose()) << "], Ix = [ " << (H * filter()->state().P() * H.transpose()).inverse() << "]");
    ROS_DEBUG_STREAM_NAMED("poseupdate", "      y = [" << y.transpose() << "], Iy = [ " << Iy << " ]");
    double innovation = updateInternal(filter()->state(), Iy, y - x, H, "twist", 0.0);
    twist_model_.getExpectedValue(x, filter()->state());
    ROS_DEBUG_STREAM_NAMED("poseupdate", " ==> xy = [" << x.transpose() << "], Pxy = [ " << (H * filter()->state().P() * H.transpose()) << " ], innovation = " << innovation);

    break;
  }

  // already done in Measurement::update()
  // filter()->state().updated();
  return true;
}

double PoseUpdate::calculateOmega(const SymmetricMatrix &Ix, const SymmetricMatrix &Iy) {
  double tr_x = Ix.trace();
  double tr_y = Iy.trace();
  return tr_y / (tr_x + tr_y);
}

template <typename MeasurementVector, typename MeasurementMatrix, typename NoiseVariance>
double PoseUpdate::updateInternal(State &state, const NoiseVariance &Iy, const MeasurementVector &error, const MeasurementMatrix &H, const std::string& text, const double max_error, JumpFunction jump_function) {
  NoiseVariance H_Px_HT(H * state.P() * H.transpose());

  if (H_Px_HT.determinant() <= 0) {
    ROS_DEBUG_STREAM("Ignoring poseupdate for " << text << " as the a-priori state covariance is zero!");
    return 0.0;
  }
  NoiseVariance Ix(H_Px_HT.inverse().eval());

  ROS_DEBUG_STREAM_NAMED("poseupdate", "H = [" << H << "]");
  ROS_DEBUG_STREAM_NAMED("poseupdate", "Ix = [" << Ix << "]");

  double alpha = fixed_alpha_, beta = fixed_beta_;
  if (alpha == 0.0 && beta == 0.0) {
    beta = calculateOmega(Ix, Iy);
    alpha = 1.0 - beta;

//    if (beta > 0.8) {
//      ROS_DEBUG_STREAM("Reducing update variance for " << text << " due to high information difference between Ix = [" << Ix << "] and Iy = [" << Iy << "]");
//      beta = 0.8;
//      alpha = 1.0 - beta;
//    }
  }
  ROS_DEBUG_STREAM_NAMED("poseupdate", "alpha = " << alpha << ", beta = " << beta);

  if (max_error > 0.0) {
    double error2 = (error.transpose() * Ix * (Ix + Iy).inverse() * Iy * error)(0);
    if (error2 > max_error * max_error) {
      if (!jump_on_max_error_ || !jump_function) {
        ROS_WARN_STREAM_NAMED("poseupdate", "Ignoring poseupdate for " << text << " as the error [ " << error.transpose() << " ], |error| = " << sqrt(error2) << " sigma exceeds max_error!");
        return 0.0;
      } else {
        ROS_WARN_STREAM_NAMED("poseupdate", "Update for " << text << " with error [ " << error.transpose() << " ], |error| = " << sqrt(error2) << " sigma exceeds max_error!");
        jump_function(state, error);
        return 0.0;
      }
    }
  }

//  SymmetricMatrix Ii(Ix * (alpha - 1) + Iy * beta);
//  double innovation = Ii.determinant();
//  ROS_DEBUG_STREAM_NAMED("poseupdate", "Ii = [" << Ii << "], innovation = " << innovation);

  // S_1 is equivalent to S^(-1) = (H*P*H^T + R)^(-1) in the standard Kalman gain
  NoiseVariance S_1(Ix - Ix * (Ix * alpha + Iy * beta).inverse() * Ix);
  typename Matrix_<State::Covariance::ColsAtCompileTime, MeasurementMatrix::RowsAtCompileTime>::type P_HT((H * state.P()).transpose());
  ROS_DEBUG_STREAM_NAMED("poseupdate", "P*HT = [" << (P_HT) << "]");

  double innovation = S_1.determinant();
  state.P() = state.P() - P_HT * S_1 * P_HT.transpose(); // may invalidate Px if &Pxy == &Px
  state.P().assertSymmetric();
  state.update(P_HT * Iy * beta * error);
  // state.x() = state.x() + P_HT * Iy * beta * error;

  ROS_DEBUG_STREAM_NAMED("poseupdate", "K = [" << (P_HT * Iy * beta) << "]");
  ROS_DEBUG_STREAM_NAMED("poseupdate", "dx = [" << (P_HT * Iy * beta * error).transpose() << "]");

  return innovation;
}

void PositionXYModel::getExpectedValue(MeasurementVector &y_pred, const State &state) {
  y_pred = state.getPosition().head<2>();
}

void PositionXYModel::getStateJacobian(MeasurementMatrix &C, const State &state, bool init) {
  if (init) {
    if (state.position()) {
      state.position()->cols(C)(0,X)   = 1.0;
      state.position()->cols(C)(1,Y)   = 1.0;
    }
  }
}

void PositionXYModel::updateState(State &state, const ColumnVector &diff) const {
  if (state.position()) {
    state.position()->vector().head<2>() += diff;
  }
}

void PositionZModel::getExpectedValue(MeasurementVector &y_pred, const State &state) {
  y_pred(0) = state.getPosition().z();
}

void PositionZModel::getStateJacobian(MeasurementMatrix &C, const State &state, bool init) {
  if (init && state.position()) {
    state.position()->cols(C)(0,Z)   = 1.0;
  }
}

void PositionZModel::updateState(State &state, const ColumnVector &diff) const {
  if (state.position()) {
    state.position()->vector().segment<1>(Z) += diff;
  }
}

void YawModel::getExpectedValue(MeasurementVector &y_pred, const State &state) {
  y_pred(0) = state.getYaw();
}

void YawModel::getStateJacobian(MeasurementMatrix &C, const State &state, bool init) {
  if (init && state.orientation()) {
    state.orientation()->cols(C)(0,Z) = 1.0;
  }
}

void YawModel::updateState(State &state, const ColumnVector &diff) const {
  Eigen::Quaterniond::Matrix3 rotation(Eigen::AngleAxisd(diff(0), Eigen::Vector3d::UnitZ()));

  Eigen::MatrixXd S(Eigen::MatrixXd::Identity(state.getCovarianceDimension(), state.getCovarianceDimension()));

  if (state.orientation()) {
//    S.block(state.orientation()->getCovarianceIndex(), state.orientation()->getCovarianceIndex(), 4, 4) <<
//      /* QUATERNION_X: */  rotation.w(), -rotation.z(),  rotation.y(), rotation.x(),
//      /* QUATERNION_Y: */  rotation.z(),  rotation.w(), -rotation.x(), rotation.y(),
//      /* QUATERNION_Z: */ -rotation.y(),  rotation.x(),  rotation.w(), rotation.z(),
//      /* QUATERNION_W: */ -rotation.x(), -rotation.y(), -rotation.z(), rotation.w();
    S.block(state.orientation()->getCovarianceIndex(), state.orientation()->getCovarianceIndex(), 3, 3) = rotation.transpose();
    state.updateOrientation(ColumnVector3(0.0, 0.0, -diff(0)));
  }

  if (state.velocity()) {
    S.block(state.velocity()->getCovarianceIndex(), state.velocity()->getCovarianceIndex(), 3, 3) = rotation.transpose();
    state.velocity()->vector() = rotation.transpose() * state.velocity()->vector();
  }

// Rate vector is in body frame. No need to rotate it.
//  if (state.rate()) {
//    S.block(state.rate()->getCovarianceIndex(), state.rate()->getCovarianceIndex(), 3, 3) = rotation.transpose();
//    state.rate()->vector() = rotation.transpose() * state.rate()->vector();
//  }

  ROS_DEBUG_STREAM_NAMED("poseupdate", "Jump yaw by " << (diff(0) * 180.0/M_PI) << " degrees. rotation = [" << rotation << "], S = [" << S << "].");

  // update covariance matrix P
  state.P() = S * state.P() * S.transpose();
}

void TwistModel::getExpectedValue(MeasurementVector &y_pred, const State &state) {
  y_pred.segment<3>(0) = state.getVelocity();
  y_pred.segment<3>(3) = state.getRate();
}

void TwistModel::getStateJacobian(MeasurementMatrix &C, const State &state, bool init) {
  if (init && state.velocity()) {
    state.velocity()->cols(C)(0,X) = 1.0;
    state.velocity()->cols(C)(1,Y) = 1.0;
    state.velocity()->cols(C)(2,Z) = 1.0;
  }

  if (init && state.rate()) {
    state.rate()->cols(C)(3,X) = 1.0;
    state.rate()->cols(C)(4,Y) = 1.0;
    state.rate()->cols(C)(5,Z) = 1.0;
  }
}

} // namespace hector_pose_estimation
