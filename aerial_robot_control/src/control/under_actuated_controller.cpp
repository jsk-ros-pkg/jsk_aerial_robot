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

#include <aerial_robot_control/control/under_actuated_controller.h>

namespace aerial_robot_control
{
  UnderActuatedController::UnderActuatedController():
    PoseLinearController(),
    torque_allocation_matrix_inv_pub_stamp_(0)
  {
  }

  void UnderActuatedController::initialize(ros::NodeHandle nh,
                                                   ros::NodeHandle nhp,
                                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                                   double ctrl_loop_rate)
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    rosParamInit();

    q_mat_.resize(4, motor_num_);
    q_mat_inv_.resize(motor_num_, 4);
    target_base_thrust_.resize(motor_num_);

    pid_msg_.z.total.resize(motor_num_);
    z_limit_ = pid_controllers_.at(Z).getLimitSum();
    pid_controllers_.at(Z).setLimitSum(1e6); // do not clamp the sum of PID terms for z axis

    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  }

  void UnderActuatedController::reset()
  {
    PoseLinearController::reset();

    setAttitudeGains();
  }

  void UnderActuatedController::controlCore()
  {
    PoseLinearController::controlCore();

    // wrench allocation matrix
    const std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& rotor_direction = robot_model_->getRotorDirection();
    const double m_f_rate = robot_model_->getMFRate();
    double uav_mass_inv = 1.0 / robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
    for (unsigned int i = 0; i < motor_num_; ++i) {
      q_mat_(0, i) = rotors_normal.at(i).z() * uav_mass_inv;
      q_mat_.block(1, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
    }
    q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);


    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());

    tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

    Eigen::VectorXd target_thrust_z_term;
    if(hovering_approximate_)
      {
        target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
        target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
        target_thrust_z_term = q_mat_inv_.col(0) * target_acc_w.z();
      }
    else
      {
        target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
        target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
        target_thrust_z_term = q_mat_inv_.col(0) * target_acc_w.length();
      }

     // constraint z (also  I term)
    int index;
    double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
    double residual = max_term - z_limit_;
    if(residual > 0)
      {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
        target_thrust_z_term *= (1 - residual / max_term);
      }

    for(int i = 0; i < motor_num_; i++)
      {
        target_base_thrust_.at(i) = target_thrust_z_term(i);
        pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
      }

    // special process for yaw since the bandwidth between PC and spinal
    double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
    for (unsigned int i = 0; i < motor_num_; i++)
      {
        if(q_mat_inv_(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW - 2);
      }

    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
  }

  void UnderActuatedController::sendCmd()
  {
    PoseLinearController::sendCmd();

    sendFourAxisCommand();
    sendTorqueAllocationMatrixInv();
  }

  void UnderActuatedController::sendFourAxisCommand()
  {
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = target_roll_;
    flight_command_data.angles[1] = target_pitch_;
    flight_command_data.angles[2] = candidate_yaw_term_;
    flight_command_data.base_thrust = target_base_thrust_;
    flight_cmd_pub_.publish(flight_command_data);
  }

  void UnderActuatedController::sendTorqueAllocationMatrixInv()
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

  void UnderActuatedController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
    getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  }

  void UnderActuatedController::setAttitudeGains()
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
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedController, aerial_robot_control::ControlBase);
