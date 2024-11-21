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
  double pMxy, pMz, pDxy, pDz, pKxy, pKz, oMxy, oMz, oDxy, oDz, oKxy, oKz, Qa, Qt, Rtc_d, Rac_d;
  getParam<double>(nmpc_nh, "pMxy", pMxy, 1.5);
  getParam<double>(nmpc_nh, "pMz", pMz, 1.5);
  getParam<double>(nmpc_nh, "pDxy", pDxy, 10);
  getParam<double>(nmpc_nh, "pDz", pDz, 10);
  getParam<double>(nmpc_nh, "pKxy", pKxy, 6);
  getParam<double>(nmpc_nh, "pKz", pKz, 6);

  getParam<double>(nmpc_nh, "oMxy", oMxy, 0.9);
  getParam<double>(nmpc_nh, "oMz", oMz, 0.9);
  getParam<double>(nmpc_nh, "oDxy", oDxy, 10);
  getParam<double>(nmpc_nh, "oDz", oDz, 10);
  getParam<double>(nmpc_nh, "oKxy", oKxy, 6);
  getParam<double>(nmpc_nh, "oKz", oKz, 6);

  getParam<double>(nmpc_nh, "Qa", Qa, 0);
  getParam<double>(nmpc_nh, "Qt", Qt, 0);

  getParam<double>(nmpc_nh, "Rtc_d", Rtc_d, 1);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250);

  // impedance matrix
  auto imp_mpc_solver_ptr =
      boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
  if (imp_mpc_solver_ptr)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pMx", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMy", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMz", pMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDx", pDxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDy", pDxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDz", pDz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKx", pKxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKy", pKxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKz", pKz, false);

    imp_mpc_solver_ptr->setImpedanceWeight("oMx", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMy", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMz", oMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDx", oDxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDy", oDxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDz", oDz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKx", oKxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKy", oKxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKz", oKz, true);  // update W and WN
  }
  else
  {
    ROS_ERROR("The MPC solver is not the impedance model. Please check the MPC solver!!!!");
  }

  // diagonal matrix
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
}

void nmpc::TiltMtServoThrustImpNMPC::prepareNMPCParams()
{
}

void nmpc::TiltMtServoThrustImpNMPC::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustImpNMPC, aerial_robot_control::ControlBase)