//
// Created by li-jinjie on 24-11-21.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_imp_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustImpNMPC::initCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double pMxy, pMz, Qv_xy, Qv_z, Qp_xy, Qp_z, oMxy, oMz, Qw_xy, Qw_z, Qq_xy, Qq_z, Qa, Qt, Rtc_d, Rac_d;
  getParam<double>(nmpc_nh, "pMxy", pMxy, 1.5);
  getParam<double>(nmpc_nh, "pMz", pMz, 1.5);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 6);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 6);

  getParam<double>(nmpc_nh, "oMxy", oMxy, 0.9);
  getParam<double>(nmpc_nh, "oMz", oMz, 0.9);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 10);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 6);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 6);

  getParam<double>(nmpc_nh, "Qa", Qa, 0);
  getParam<double>(nmpc_nh, "Qt", Qt, 0);

  getParam<double>(nmpc_nh, "Rtc_d", Rtc_d, 1);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);

  // impedance matrix
  auto imp_mpc_solver_ptr =
      boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
  if (imp_mpc_solver_ptr)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pMx", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMy", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMz", pMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDx", Qv_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDy", Qv_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDz", Qv_z, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKx", Qp_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKy", Qp_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKz", Qp_z, false);

    imp_mpc_solver_ptr->setImpedanceWeight("oMx", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMy", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMz", oMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDx", Qw_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDy", Qw_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDz", Qw_z, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKx", Qq_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKy", Qq_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKz", Qq_z, true);  // update W and WN
  }
  else
  {
    ROS_ERROR("The MPC solver is not the impedance model. Please check the MPC solver!!!!");
  }
}

void nmpc::TiltMtServoThrustImpNMPC::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustImpNMPC, aerial_robot_control::ControlBase)