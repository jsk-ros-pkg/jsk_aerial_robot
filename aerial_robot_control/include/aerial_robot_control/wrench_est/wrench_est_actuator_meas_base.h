//
// Created by jinjie on 24/11/04.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H

#include "aerial_robot_control/wrench_est/wrench_est_base.h"

#include "sensor_msgs/JointState.h"
#include "spinal/ESCTelemetryArray.h"

namespace aerial_robot_control
{

class WrenchEstActuatorMeasBase : public WrenchEstBase
{
public:
  WrenchEstActuatorMeasBase() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>& navigator, double ctrl_loop_du) override
  {
    WrenchEstBase::initialize(nh, robot_model, estimator, navigator, ctrl_loop_du);

    // TODO: combine this part with the controller. especially the subscriber part.
    ros::NodeHandle motor_nh(nh_, "motor_info");
    getParam<double>(motor_nh, "krpm_rate", krpm2_d_thrust_, 0.0);

    joint_angles_.resize(robot_model_->getJointNum(), 0.0);
    sub_joint_states_ = nh_.subscribe("joint_states", 1, &WrenchEstActuatorMeasBase::callbackJointStates, this);

    thrust_meas_.resize(robot_model_->getRotorNum(), 0.0);
    sub_esc_telem_ = nh_.subscribe("esc_telem", 1, &WrenchEstActuatorMeasBase::callbackESCTelem, this);
  }

  Eigen::VectorXd calWrenchFromActuatorMeas()
  {
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * robot_model_->getRotorNum());
    for (int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      z(2 * i) = thrust_meas_[i] * sin(joint_angles_[i]);
      z(2 * i + 1) = thrust_meas_[i] * cos(joint_angles_[i]);
    }

    // allocate * z
    Eigen::VectorXd est_wrench_cog = alloc_mat_ * z;
    return est_wrench_cog;
  }

private:
  // for servo angles
  std::vector<double> joint_angles_;

  ros::Subscriber sub_joint_states_;
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int i = 0; i < robot_model_->getJointNum(); i++)
      joint_angles_[i] = msg->position[i];
  }

  // for actual thrust
  std::vector<double> thrust_meas_;

  ros::Subscriber sub_esc_telem_;
  double krpm2_d_thrust_;
  void callbackESCTelem(const spinal::ESCTelemetryArrayConstPtr& msg)
  {
    double krpm;
    krpm = (double)msg->esc_telemetry_1.rpm * 0.001;
    thrust_meas_[0] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_2.rpm * 0.001;
    thrust_meas_[1] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_3.rpm * 0.001;
    thrust_meas_[2] = krpm * krpm / krpm2_d_thrust_;

    krpm = (double)msg->esc_telemetry_4.rpm * 0.001;
    thrust_meas_[3] = krpm * krpm / krpm2_d_thrust_;
  }
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACTUATOR_MEAS_BASE_H
