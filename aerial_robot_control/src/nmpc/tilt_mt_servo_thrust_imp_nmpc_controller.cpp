//
// Created by li-jinjie on 24-11-21.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_imp_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustImpNMPC::prepareNMPCParams()
{
  // TODO: wrap this part to a function
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
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  // if the norm of vel is less than 0.2 m/s, than the I term is updated.
  if (vel.length() < 0.1)
  {
    wrench_est_i_term_.update(target_pos, target_q, pos, q);
  }

  auto dist_force_w = wrench_est_i_term_.getDistForceW();
  auto dist_torque_cog = wrench_est_i_term_.getDistTorqueCOG();

  vector<int> idx = { 4, 5, 6, 7, 8, 9 };
  vector<double> p = { dist_force_w.x,    dist_force_w.y,    dist_force_w.z,
                       dist_torque_cog.x, dist_torque_cog.y, dist_torque_cog.z };
  mpc_solver_ptr_->setParamSparseAllStages(idx, p);
}

std::vector<double> nmpc::TiltMtServoThrustImpNMPC::meas2VecX()
{
  auto bx0 = TiltMtServoNMPC::meas2VecX();

  for (int i = 0; i < motor_num_; i++)
    bx0[13 + joint_num_ + i] = thrust_meas_[i];

  /* disturbance rejection */
  geometry_msgs::Vector3 external_force_w = wrench_est_ptr_->getDistForceW();
  geometry_msgs::Vector3 external_torque_cog = wrench_est_ptr_->getDistTorqueCOG();

  bx0[13 + joint_num_ + motor_num_ + 0] = external_force_w.x;
  bx0[13 + joint_num_ + motor_num_ + 1] = external_force_w.y;
  bx0[13 + joint_num_ + motor_num_ + 2] = external_force_w.z;
  bx0[13 + joint_num_ + motor_num_ + 3] = external_torque_cog.x;
  bx0[13 + joint_num_ + motor_num_ + 4] = external_torque_cog.y;
  bx0[13 + joint_num_ + motor_num_ + 5] = external_torque_cog.z;
  return bx0;
}

void nmpc::TiltMtServoThrustImpNMPC::calcDisturbWrench()
{
  /* update the external wrench estimator based on the Nav State */
  auto nav_state = navigator_->getNaviState();

  if (nav_state == aerial_robot_navigation::TAKEOFF_STATE || nav_state == aerial_robot_navigation::HOVER_STATE ||
      nav_state == aerial_robot_navigation::LAND_STATE)
  {
    if (estimator_->getPos(Frame::COG, estimate_mode_).z() > 0.3)  // TODO: change to a state: IN_AIR
    {
      /* get external wrench */
      if (wrench_est_ptr_ != nullptr)
        wrench_est_ptr_->update();
      else
        ROS_ERROR("wrench_est_ptr_ is nullptr");
    }

    // TODO: 1. move this function to tilt_mt_servo_dist_nmpc_controller.cpp
    // TODO: 2. change the logic, change if_use_est_wrench_4_control_ to the place where the external wrench is added to
    // controller

    // the external wrench is only added when the robot is in the hover state
    if (if_use_est_wrench_4_control_ && nav_state == aerial_robot_navigation::HOVER_STATE)
    {
      if (!wrench_est_ptr_->getOffsetFlag())
        wrench_est_ptr_->toggleOffsetFlag();

      // TODO: change this part to ext_force_w_ and mdl_err_force_w_
      dist_force_w_ = wrench_est_ptr_->getDistForceW();
      dist_torque_cog_ = wrench_est_ptr_->getDistTorqueCOG();
    }
  }
}

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
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.nmpc_flag)
  {
    auto imp_mpc_solver_ptr =
        boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
    if (!imp_mpc_solver_ptr)
    {
      ROS_ERROR("The MPC solver is not the impedance model. Please check the MPC solver!!!!");
      return;
    }

    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_NMPC_Q_P_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("pKx", config.Qp_xy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("pKy", config.Qp_xy, true);
          ROS_INFO_STREAM("change Qp_xy (pKxy) for NMPC '" << config.Qp_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_P_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("pKz", config.Qp_z, true);
          ROS_INFO_STREAM("change Qp_z (pKz) for NMPC '" << config.Qp_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("pDx", config.Qv_xy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("pDy", config.Qv_xy, true);
          ROS_INFO_STREAM("change Qv_xy (pDxy) for NMPC '" << config.Qv_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_V_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("pDz", config.Qv_z, true);
          ROS_INFO_STREAM("change Qv_z (pDz) for NMPC '" << config.Qv_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_PM_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("pMx", config.pMxy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("pMy", config.pMxy, true);
          ROS_INFO_STREAM("change pMxy for NMPC '" << config.pMxy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_PM_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("pMz", config.pMz, true);
          ROS_INFO_STREAM("change pMz for NMPC '" << config.pMz << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("oKx", config.Qq_xy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("oKy", config.Qq_xy, true);
          ROS_INFO_STREAM("change Qq_xy (oKxy) for NMPC '" << config.Qq_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("oKz", config.Qq_z, true);
          ROS_INFO_STREAM("change Qq_z (oKz) for NMPC '" << config.Qq_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("oDx", config.Qw_xy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("oDy", config.Qw_xy, true);
          ROS_INFO_STREAM("change Qw_xy (oDxy) for NMPC '" << config.Qw_xy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_W_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("oDz", config.Qw_z, true);
          ROS_INFO_STREAM("change Qw_z (oDz) for NMPC '" << config.Qw_z << "'");
          break;
        }
        case Levels ::RECONFIGURE_NMPC_OM_XY: {
          imp_mpc_solver_ptr->setImpedanceWeight("oMx", config.oMxy, false);
          imp_mpc_solver_ptr->setImpedanceWeight("oMy", config.oMxy, true);
          ROS_INFO_STREAM("change oMxy for NMPC '" << config.oMxy << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_OM_Z: {
          imp_mpc_solver_ptr->setImpedanceWeight("oMz", config.oMz, true);
          ROS_INFO_STREAM("change oMz for NMPC '" << config.oMz << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_A: {
          for (int i = 13; i < 13 + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
          ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_Q_T: {
          for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Qt);
          ROS_INFO_STREAM("change Qt for NMPC '" << config.Qt << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_TC_D: {
          for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rtc_d, false);
          ROS_INFO_STREAM("change Rtc_d for NMPC '" << config.Rtc_d << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_R_AC_D: {
          for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
            mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
          ROS_INFO_STREAM("change Rac_d for NMPC '" << config.Rac_d << "'");
          break;
        }
        default: {
          ROS_INFO_STREAM("The setting variable is not in the list!");
          break;
        }
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("NMPC config failed: " << e.what());
    }
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustImpNMPC, aerial_robot_control::ControlBase)