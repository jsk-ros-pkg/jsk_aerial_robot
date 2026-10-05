//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "if_use_est_wrench_4_control", if_use_est_wrench_4_control_, false);

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("ext_wrench_est/value", 1);
}

void nmpc::TiltMtServoDistNMPC::controlCore(bool is_warmup)
{
  if (!is_warmup)
  {
    updateITerm();
  }

  TiltMtServoNMPC::controlCore();
}

void nmpc::TiltMtServoDistNMPC::sendCmd()
{
  TiltMtServoNMPC::sendCmd();
}

void nmpc::TiltMtServoDistNMPC::resetPlugins()
{
  wrench_est_i_term_.reset();
}

void nmpc::TiltMtServoDistNMPC::initPlugins()
{
  /* I Term is always loaded  */
  wrench_est_i_term_.initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);
}

void nmpc::TiltMtServoDistNMPC::updateITerm()
{
  /* HANDLING MODEL ERROR */
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  // Note that the target state is the estimated state from NMPC solver.
  tf::Vector3 pos_x0(mpc_solver_ptr_->xo_.at(0).at(0), mpc_solver_ptr_->xo_.at(0).at(1),
                     mpc_solver_ptr_->xo_.at(0).at(2));
  tf::Vector3 pos_x1(mpc_solver_ptr_->xo_.at(1).at(0), mpc_solver_ptr_->xo_.at(1).at(1),
                     mpc_solver_ptr_->xo_.at(1).at(2));
  tf::Vector3 target_pos = pos_x0 + (pos_x1 - pos_x0) * t_nmpc_samp_ / t_nmpc_step_;

  tf::Quaternion quat_x0(mpc_solver_ptr_->xo_.at(0).at(7), mpc_solver_ptr_->xo_.at(0).at(8),
                         mpc_solver_ptr_->xo_.at(0).at(9), mpc_solver_ptr_->xo_.at(0).at(6));
  quat_x0.normalize();
  tf::Quaternion quat_x1(mpc_solver_ptr_->xo_.at(1).at(7), mpc_solver_ptr_->xo_.at(1).at(8),
                         mpc_solver_ptr_->xo_.at(1).at(9), mpc_solver_ptr_->xo_.at(1).at(6));
  quat_x1.normalize();
  tf::Quaternion target_q = quat_x0.slerp(quat_x1, t_nmpc_samp_ / t_nmpc_step_);

  /* update I term */
  wrench_est_i_term_.update(target_pos, target_q, pos, q);
}

void nmpc::TiltMtServoDistNMPC::initNMPCParams()
{
  TiltMtServoNMPC::initNMPCParams();
  idx_p_dist_end_ = idx_p_phys_end_ + 6;
}

void nmpc::TiltMtServoDistNMPC::prepareNMPCParams()
{
  TiltMtServoNMPC::prepareNMPCParams();

  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_cog = wrench_est_i_term_.getDistTorqueCOG();

  vector<double> p = { mdl_error_force_w.x,    mdl_error_force_w.y,    mdl_error_force_w.z,
                       mdl_error_torque_cog.x, mdl_error_torque_cog.y, mdl_error_torque_cog.z };
  mpc_solver_ptr_->setParameters(p, idx_p_phys_end_ + 1);
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX(bool is_modified_by_traj_frame)
{
  vector<double> bx0 = TiltMtServoNMPC::meas2VecX(is_modified_by_traj_frame);

  return bx0;
}

void nmpc::TiltMtServoDistNMPC::initAllocMat()
{
  TiltMtServoNMPC::initAllocMat();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoDistNMPC, aerial_robot_control::ControlBase);
