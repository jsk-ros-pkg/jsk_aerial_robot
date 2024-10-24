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

void nmpc::TiltQdServoDistNMPC::initPlugins()
{
  /* plugin: wrench estimator */
  wrench_est_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::WrenchEstBase>>(
      "aerial_robot_control", "aerial_robot_control::WrenchEstBase");
  try
  {
    wrench_est_ptr_ = wrench_est_loader_ptr_->createInstance("aerial_robot_control::WrenchEstITerm");
    wrench_est_ptr_->initialize(nh_, ctrl_loop_du_);
    ROS_INFO("Wrench estimator plugin loaded");  // TODO: change to instance name variable
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
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

void nmpc::TiltQdServoDistNMPC::calcDisturbWrench()
{
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  tf::Vector3 target_pos;
  tf::Quaternion target_q;
  if (is_traj_tracking_)
  {
    target_pos.setX(x_u_ref_.x.data.at(0));
    target_pos.setY(x_u_ref_.x.data.at(1));
    target_pos.setZ(x_u_ref_.x.data.at(2));

    target_q.setW(x_u_ref_.x.data.at(6));
    target_q.setX(x_u_ref_.x.data.at(7));
    target_q.setY(x_u_ref_.x.data.at(8));
    target_q.setZ(x_u_ref_.x.data.at(9));
  }
  else
  {
    target_pos = navigator_->getTargetPos();

    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    target_q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  }

  /* update I term */
  if (wrench_est_ptr_ != nullptr)
  {
    wrench_est_ptr_->update(target_pos, target_q, pos, q);
  }
  else
  {
    ROS_ERROR("wrench_est_ptr_ is nullptr");
  }

  /* get the disturbance wrench */
  dist_force_w_ = wrench_est_ptr_->getDistForceW();
  dist_torque_cog_ = wrench_est_ptr_->getDistTorqueCOG();
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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoDistNMPC, aerial_robot_control::ControlBase);
