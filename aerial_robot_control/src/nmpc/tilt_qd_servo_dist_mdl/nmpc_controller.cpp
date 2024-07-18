//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltQdServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("disturbance_wrench", 1);
}

std::vector<double> nmpc::TiltQdServoDistNMPC::meas2VecX()
{
  vector<double> bx0 = TiltQdServoNMPC::meas2VecX();

  /* disturbance rejection */
  calcDisturbWrench();
  bx0[13 + joint_num_ + 0] = dist_force_w_.x;
  bx0[13 + joint_num_ + 1] = dist_force_w_.y;
  bx0[13 + joint_num_ + 2] = dist_force_w_.z;
  bx0[13 + joint_num_ + 3] = dist_torque_cog_.x;
  bx0[13 + joint_num_ + 4] = dist_torque_cog_.y;
  bx0[13 + joint_num_ + 5] = dist_torque_cog_.z;
  return bx0;
}

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc::TiltQdServoDistNMPC::callbackViz(const ros::TimerEvent& event)
{
  TiltQdServoNMPC::callbackViz(event);

  /* disturbance wrench */
  geometry_msgs::WrenchStamped dist_wrench_;
  dist_wrench_.header.frame_id = "beetle1/cog";

  dist_wrench_.wrench.torque = dist_torque_cog_;

  tf::Matrix3x3 rot_mtx_cog2w = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 dist_force_w = tf::Vector3(dist_force_w_.x, dist_force_w_.y, dist_force_w_.z);
  tf::Vector3 dist_force_cog = rot_mtx_cog2w.inverse() * dist_force_w;
  dist_wrench_.wrench.force.x = dist_force_cog.x();
  dist_wrench_.wrench.force.y = dist_force_cog.y();
  dist_wrench_.wrench.force.z = dist_force_cog.z();

  dist_wrench_.header.stamp = ros::Time::now();

  pub_disturb_wrench_.publish(dist_wrench_);
}
