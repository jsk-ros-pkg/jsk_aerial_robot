// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#include <aerial_robot_control/control/fully_actuated_controller.h>

using namespace std;

namespace aerial_robot_control
{
  FullyActuatedController::FullyActuatedController():
    PoseLinearController(),
    wrench_allocation_matrix_pub_stamp_(0),
    torque_allocation_matrix_inv_pub_stamp_(0)
  {
  }

  void FullyActuatedController::initialize(ros::NodeHandle nh,
                                                   ros::NodeHandle nhp,
                                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                   double ctrl_loop_rate)
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    rosParamInit();

    q_mat_.resize(6, motor_num_);
    q_mat_inv_.resize(motor_num_, 6);
    target_base_thrust_.resize(motor_num_);

    pid_msg_.x.total.resize(motor_num_),
    pid_msg_.y.total.resize(motor_num_),
    pid_msg_.z.total.resize(motor_num_),

    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
    wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
    wrench_allocation_matrix_inv_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix_inv", 1);
  }

  void FullyActuatedController::reset()
  {
    PoseLinearController::reset();

    setAttitudeGains();
  }

  void FullyActuatedController::controlCore()
  {
    PoseLinearController::controlCore();

    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());
    tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;


    //wrench allocation matrix
    Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
    double mass_inv =  1 / robot_model_->getMass();
    Eigen::MatrixXd q_mat = robot_model_->calcWrenchMatrixOnCoG();
    q_mat_.topRows(3) =  mass_inv * q_mat.topRows(3) ;
    q_mat_.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
    q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

     Eigen::VectorXd target_thrust_x_term = q_mat_inv_.col(X) * target_acc_cog.x();
     Eigen::VectorXd target_thrust_y_term = q_mat_inv_.col(Y) * target_acc_cog.y();
     Eigen::VectorXd target_thrust_z_term = q_mat_inv_.col(Z) * target_acc_cog.z();

    // constraint x and y
    int index;
    double max_term = target_thrust_x_term.cwiseAbs().maxCoeff(&index);
    double residual = max_term - pid_controllers_.at(X).getLimitSum();
    if(residual > 0)
      {
        ROS_DEBUG("the position x control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(X).getLimitSum());
        target_thrust_x_term *= (1 - residual / max_term);
      }

    max_term = target_thrust_y_term.cwiseAbs().maxCoeff(&index);
    residual = max_term - pid_controllers_.at(Y).getLimitSum();
    if(residual > 0)
      {
        ROS_DEBUG("the position y control exceed the limit in rotor %d, %f vs %f ", index, max_term, pid_controllers_.at(Y).getLimitSum());
        target_thrust_y_term *= (1 - residual / max_term);
      }

    // constraint z (also  I term)
    max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
    residual = max_term - pid_controllers_.at(Z).getLimitSum();
    if(residual > 0)
      {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
        target_thrust_z_term *= (1 - residual / max_term);
      }


    for(int i = 0; i < motor_num_; i++)
      {
        target_base_thrust_.at(i) = target_thrust_x_term(i) + target_thrust_y_term(i) + target_thrust_z_term(i);

        pid_msg_.x.total.at(i) =  target_thrust_x_term(i);
        pid_msg_.y.total.at(i) =  target_thrust_y_term(i);
        pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
      }

    // special process for yaw since the bandwidth between PC and spinal
    double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
    for (unsigned int i = 0; i < motor_num_; i++)
      {
        if(q_mat_inv_(i, YAW) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW);
      }
    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
  }

  void FullyActuatedController::sendCmd()
  {
    PoseLinearController::sendCmd();

    if (ros::Time::now().toSec() - wrench_allocation_matrix_pub_stamp_ > wrench_allocation_matrix_pub_interval_)
      {
        wrench_allocation_matrix_pub_stamp_ = ros::Time::now().toSec();
        aerial_robot_msgs::WrenchAllocationMatrix msg;
        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.push_back(q_mat_(0, i));
          msg.f_y.push_back(q_mat_(1, i));
          msg.f_z.push_back(q_mat_(2, i));
          msg.t_x.push_back(q_mat_(3, i));
          msg.t_y.push_back(q_mat_(4, i));
          msg.t_z.push_back(q_mat_(5, i));
        }

        wrench_allocation_matrix_pub_.publish(msg);

        for (unsigned int i = 0; i < motor_num_; i++) {
          msg.f_x.push_back(q_mat_inv_(i, 0));
          msg.f_y.push_back(q_mat_inv_(i, 1));
          msg.f_z.push_back(q_mat_inv_(i, 2));
          msg.t_x.push_back(q_mat_inv_(i, 3));
          msg.t_y.push_back(q_mat_inv_(i, 4));
          msg.t_z.push_back(q_mat_inv_(i, 5));
        }
        wrench_allocation_matrix_inv_pub_.publish(msg);
      }

    sendFourAxisCommand();

    sendTorqueAllocationMatrixInv();
  }

  void FullyActuatedController::sendFourAxisCommand()
  {
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = navigator_->getTargetRPY().x();
    flight_command_data.angles[1] = navigator_->getTargetRPY().y();
    flight_command_data.angles[2] = candidate_yaw_term_;
    flight_command_data.base_thrust = target_base_thrust_;
    flight_cmd_pub_.publish(flight_command_data);
  }

  void FullyActuatedController::sendTorqueAllocationMatrixInv()
  {
    if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
      {
        torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

        spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
        torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
        Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
        if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
          ROS_ERROR("Torque Allocation Matrix overflow");
        for (unsigned int i = 0; i < motor_num_; i++)
          {
            torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
          }
        torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
      }
  }

  void FullyActuatedController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
    getParam<double>(control_nh, "wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);
  }

  void FullyActuatedController::setAttitudeGains()
  {
    spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
    /* to flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }

} //namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FullyActuatedController, aerial_robot_control::ControlBase);
