// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_control/control/under_actuated_nmpc_lqi_controller.h>

using namespace aerial_robot_control;

UnderActuatedNMPCLQIController::UnderActuatedNMPCLQIController()
  : target_roll_(0), target_pitch_(0), candidate_yaw_term_(0)
{
  lqi_roll_pitch_weight_.setZero();
  lqi_yaw_weight_.setZero();
  lqi_z_weight_.setZero();
}

void UnderActuatedNMPCLQIController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                double ctrl_loop_rate)
{
  NMPCController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rosParamInit();

  // publisher
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ =
      nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  // dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  lqi_server_ = boost::make_shared<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> >(
      ros::NodeHandle(control_nh, "lqi"));
  dynamic_reconf_func_lqi_ = boost::bind(&UnderActuatedNMPCLQIController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  // gains
  pitch_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  roll_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  z_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));
  yaw_gains_.resize(motor_num_, Eigen::Vector3d(0, 0, 0));

  // message
  target_base_thrust_.resize(motor_num_);

  if (!robot_model_->isModelFixed())
    realtime_update_ = true;
  if (realtime_update_)
  {
    gain_generator_thread_ = std::thread(boost::bind(&UnderActuatedNMPCLQIController::gainGeneratorFunc, this));
  }
}

UnderActuatedNMPCLQIController::~UnderActuatedNMPCLQIController()
{
  if (realtime_update_)
    gain_generator_thread_.join();
}

void UnderActuatedNMPCLQIController::gainGeneratorFunc()
{
  double rate;
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  lqi_nh.param("gain_generate_rate", rate, 15.0);
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    if (checkRobotModel())
    {
      if (optimalGain())
      {
        clampGain();
        publishGain();
      }
      else
        ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
    }
    else
    {
      resetGain();
    }

    loop_rate.sleep();
  }
}

void UnderActuatedNMPCLQIController::activate()
{
  ControlBase::activate();

  // publish gains in start phase for general multirotor
  if (optimalGain())
  {
    clampGain();
    publishGain();
    ROS_INFO_NAMED("LQI gain generator", "LQI gain generator: send LQI gains");
  }
  else
  {
    ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
  }
}

void UnderActuatedNMPCLQIController::sendCmd()
{
  NMPCController::sendCmd();
  sendRotationalInertiaComp();
}

void UnderActuatedNMPCLQIController::controlCore()
{
  NMPCController::controlCore();
}

bool UnderActuatedNMPCLQIController::optimalGain()
{
  // referece:
  // M, Zhao, et.al, "Transformable multirotor with two-dimensional multilinks: modeling, control, and whole-body aerial
  // manipulation" Sec. 3.2

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, motor_num_);
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  P_dash.row(0) = P.row(2) / robot_model_->getMass();                                               // z
  P_dash.bottomRows(lqi_mode_ - 1) = (inertia.inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1);  // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);
  for (int i = 0; i < lqi_mode_; i++)
  {
    A(2 * i, 2 * i + 1) = 1;
    B.row(2 * i + 1) = P_dash.row(i);
    C(i, 2 * i) = 1;
  }
  A.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n" << B);

  Eigen::VectorXd q_diagonals(lqi_mode_ * 3);
  if (lqi_mode_ == 3)
  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
        lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1),
        lqi_roll_pitch_weight_(1);
  }
  else
  {
    q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2),
        lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_z_weight_(1),
        lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  }
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(motor_num_, motor_num_);
  for (int i = 0; i < motor_num_; ++i)
    R(i, i) = r_.at(i);

  /* solve continuous-time algebraic Ricatti equation */
  double t = ros::Time::now().toSec();

  if (K_.cols() != lqi_mode_ * 3)
  {
    resetGain();  // four axis -> three axis and vice versa
  }

  bool use_kleinman_method = true;
  if (K_.cols() == 0 || K_.rows() == 0)
  {
    ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: do not use kleinman method");
    use_kleinman_method = false;
  }
  if (!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
  {
    ROS_ERROR_STREAM_NAMED("LQI gain generator",
                           "LQI gain generator: error in solver of continuous-time algebraic riccati equation");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator:  K \n" << K_);

  for (int i = 0; i < motor_num_; ++i)
  {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i, 2), K_(i, lqi_mode_ * 2 + 1), -K_(i, 3));
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i, 4), K_(i, lqi_mode_ * 2 + 2), -K_(i, 5));
    z_gains_.at(i) = Eigen::Vector3d(-K_(i, 0), K_(i, lqi_mode_ * 2), -K_(i, 1));
    if (lqi_mode_ == 4)
      yaw_gains_.at(i) = Eigen::Vector3d(-K_(i, 6), K_(i, lqi_mode_ * 2 + 3), -K_(i, 7));
    else
      yaw_gains_.at(i).setZero();
  }

  return true;
}

void UnderActuatedNMPCLQIController::clampGain()
{
  /* avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for (int i = 0; i < motor_num_; ++i)
  {
    if (max_roll_p_gain < fabs(roll_gains_.at(i)[0]))
      max_roll_p_gain = fabs(roll_gains_.at(i)[0]);
    if (max_roll_d_gain < fabs(roll_gains_.at(i)[2]))
      max_roll_d_gain = fabs(roll_gains_.at(i)[2]);
    if (max_pitch_p_gain < fabs(pitch_gains_.at(i)[0]))
      max_pitch_p_gain = fabs(pitch_gains_.at(i)[0]);
    if (max_pitch_d_gain < fabs(pitch_gains_.at(i)[2]))
      max_pitch_d_gain = fabs(pitch_gains_.at(i)[2]);
    if (max_yaw_d_gain < fabs(yaw_gains_.at(i)[2]))
      max_yaw_d_gain = fabs(yaw_gains_.at(i)[2]);
  }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1,
         yaw_d_gain_scale = 1;
  if (max_roll_p_gain > max_gain_thresh)
  {
    ROS_WARN_STREAM_NAMED("LQI gain generator",
                          "LQI gain generator: the max roll p gain violate the range of int16_t: " << max_roll_p_gain);
    roll_p_gain_scale = max_gain_thresh / max_roll_p_gain;
  }
  if (max_roll_d_gain > max_gain_thresh)
  {
    ROS_WARN_STREAM_NAMED("LQI gain generator",
                          "LQI gain generator: the max roll d gain violate the range of int16_t: " << max_roll_d_gain);
    roll_d_gain_scale = max_gain_thresh / max_roll_d_gain;
  }
  if (max_pitch_p_gain > max_gain_thresh)
  {
    ROS_WARN_STREAM_NAMED(
        "LQI gain generator",
        "LQI gain generator: the max pitch p gain violate the range of int16_t: " << max_pitch_p_gain);
    pitch_p_gain_scale = max_gain_thresh / max_pitch_p_gain;
  }
  if (max_pitch_d_gain > max_gain_thresh)
  {
    ROS_WARN_STREAM_NAMED(
        "LQI gain generator",
        "LQI gain generator: the max pitch d gain violate the range of int16_t: " << max_pitch_d_gain);
    pitch_d_gain_scale = max_gain_thresh / max_pitch_d_gain;
  }
  if (max_yaw_d_gain > max_gain_thresh)
  {
    ROS_WARN_STREAM_NAMED("LQI gain generator",
                          "LQI gain generator: the max yaw d gain violate the range of int16_t: " << max_yaw_d_gain);
    yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
  }

  for (int i = 0; i < motor_num_; ++i)
  {
    roll_gains_.at(i)[0] *= roll_p_gain_scale;
    roll_gains_.at(i)[2] *= roll_d_gain_scale;

    pitch_gains_.at(i)[0] *= pitch_p_gain_scale;
    pitch_gains_.at(i)[2] *= pitch_d_gain_scale;

    yaw_gains_.at(i)[2] *= yaw_d_gain_scale;
  }
}

bool UnderActuatedNMPCLQIController::checkRobotModel()
{
  if (!robot_model_->initialized())
  {
    ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
    return false;
  }

  return true;
}

void UnderActuatedNMPCLQIController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  getParam<bool>(lqi_nh, "clamp_gain", clamp_gain_, true);
  getParam<bool>(lqi_nh, "realtime_update", realtime_update_, false);
  getParam<bool>(lqi_nh, "gyro_moment_compensation", gyro_moment_compensation_, false);

  /* propeller direction and lqi R */
  r_.resize(motor_num_);  // motor_num is not set
  for (int i = 0; i < robot_model_->getRotorNum(); ++i)
  {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    getParam<double>(lqi_nh, std::string("r") + ss.str(), r_.at(i), 1.0);
  }

  getParam<double>(lqi_nh, "roll_pitch_p", lqi_roll_pitch_weight_[0], 1.0);
  getParam<double>(lqi_nh, "roll_pitch_i", lqi_roll_pitch_weight_[1], 1.0);
  getParam<double>(lqi_nh, "roll_pitch_d", lqi_roll_pitch_weight_[2], 1.0);
  getParam<double>(lqi_nh, "yaw_p", lqi_yaw_weight_[0], 1.0);
  getParam<double>(lqi_nh, "yaw_i", lqi_yaw_weight_[1], 1.0);
  getParam<double>(lqi_nh, "yaw_d", lqi_yaw_weight_[2], 1.0);
  getParam<double>(lqi_nh, "z_p", lqi_z_weight_[0], 1.0);
  getParam<double>(lqi_nh, "z_i", lqi_z_weight_[1], 1.0);
  getParam<double>(lqi_nh, "z_d", lqi_z_weight_[2], 1.0);

  getParam<int>(lqi_nh, "lqi_mode", lqi_mode_, 4);
  if (lqi_mode_ != 3 && lqi_mode_ != 4)
  {
    ROS_ERROR_STREAM_NAMED("LQI gain generator",
                           "LQI gain generator: lqi model should be 3 or 4, " << lqi_mode_ << " is not allowed.");
  }
}

void UnderActuatedNMPCLQIController::publishGain()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg;  // to spinal
  rpy_gain_msg.motors.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    four_axis_gain_msg.roll_p_gain.push_back(roll_gains_.at(i)[0]);
    four_axis_gain_msg.roll_i_gain.push_back(roll_gains_.at(i)[1]);
    four_axis_gain_msg.roll_d_gain.push_back(roll_gains_.at(i)[2]);

    four_axis_gain_msg.pitch_p_gain.push_back(pitch_gains_.at(i)[0]);
    four_axis_gain_msg.pitch_i_gain.push_back(pitch_gains_.at(i)[1]);
    four_axis_gain_msg.pitch_d_gain.push_back(pitch_gains_.at(i)[2]);

    four_axis_gain_msg.yaw_p_gain.push_back(yaw_gains_.at(i)[0]);
    four_axis_gain_msg.yaw_i_gain.push_back(yaw_gains_.at(i)[1]);
    four_axis_gain_msg.yaw_d_gain.push_back(yaw_gains_.at(i)[2]);

    four_axis_gain_msg.z_p_gain.push_back(z_gains_.at(i)[0]);
    four_axis_gain_msg.z_i_gain.push_back(z_gains_.at(i)[1]);
    four_axis_gain_msg.z_d_gain.push_back(z_gains_.at(i)[2]);

    /* to flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i)[0] * 1000;
    rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i)[1] * 1000;
    rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i)[2] * 1000;

    rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i)[0] * 1000;
    rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i)[1] * 1000;
    rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i)[2] * 1000;

    rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;
  }
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);
}

void UnderActuatedNMPCLQIController::cfgLQICallback(aerial_robot_control::LQIConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.lqi_flag)
  {
    switch (level)
    {
      case Levels::RECONFIGURE_LQI_ROLL_PITCH_P:
        ROS_INFO_STREAM_NAMED("LQI gain generator",
                              "LQI gain generator: change the p gain weight of roll and pitch from "
                                  << lqi_roll_pitch_weight_.x() << " to " << config.roll_pitch_p);
        lqi_roll_pitch_weight_.x() = config.roll_pitch_p;
        break;
      case Levels::RECONFIGURE_LQI_ROLL_PITCH_I:
        ROS_INFO_STREAM_NAMED("LQI gain generator",
                              "LQI gain generator: change the i gain weight of roll and pitch from "
                                  << lqi_roll_pitch_weight_.y() << " to " << config.roll_pitch_i);
        lqi_roll_pitch_weight_.y() = config.roll_pitch_i;
        break;
      case Levels::RECONFIGURE_LQI_ROLL_PITCH_D:
        ROS_INFO_STREAM_NAMED("LQI gain generator",
                              "LQI gain generator: change the d gain weight of roll and pitch from "
                                  << lqi_roll_pitch_weight_.z() << " to " << config.roll_pitch_d);
        lqi_roll_pitch_weight_.z() = config.roll_pitch_d;
        break;
      case Levels::RECONFIGURE_LQI_YAW_P:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of yaw from "
                                                        << lqi_yaw_weight_.x() << " to " << config.yaw_p);
        lqi_yaw_weight_.x() = config.yaw_p;
        break;
      case Levels::RECONFIGURE_LQI_YAW_I:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of yaw from "
                                                        << lqi_yaw_weight_.y() << " to " << config.yaw_i);
        lqi_yaw_weight_.y() = config.yaw_i;
        break;
      case Levels::RECONFIGURE_LQI_YAW_D:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of yaw from "
                                                        << lqi_yaw_weight_.z() << " to " << config.yaw_d);
        lqi_yaw_weight_.z() = config.yaw_d;
        break;
      case Levels::RECONFIGURE_LQI_Z_P:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of z from "
                                                        << lqi_z_weight_.x() << " to " << config.z_p);
        lqi_z_weight_.x() = config.z_p;
        break;
      case Levels::RECONFIGURE_LQI_Z_I:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of z from "
                                                        << lqi_z_weight_.y() << " to " << config.z_i);
        lqi_z_weight_.y() = config.z_i;
        break;
      case Levels::RECONFIGURE_LQI_Z_D:
        ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of z from "
                                                        << lqi_z_weight_.z() << " to " << config.z_d);
        lqi_z_weight_.z() = config.z_d;
        break;
      default:
        break;
    }

    if (!realtime_update_)
    {
      // instantly modify gain if no joint angles

      if (optimalGain())
      {
        clampGain();
        publishGain();
      }
      else
      {
        ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
      }
    }
  }
}

void UnderActuatedNMPCLQIController::sendRotationalInertiaComp()
{
  if (!gyro_moment_compensation_)
    return;

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));

  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;  // to spinal
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for (int i = 0; i < motor_num_; ++i)
  {
    /* the p matrix pseudo inverse and inertia */
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
    p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
    if (lqi_mode_ == 4)
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
    else
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
  }

  /* the articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedNMPCLQIController, aerial_robot_control::ControlBase);
