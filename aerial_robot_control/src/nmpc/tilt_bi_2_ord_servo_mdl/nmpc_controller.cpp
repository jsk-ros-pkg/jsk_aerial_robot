//
// Created by lijinjie on 24/06/09.
//

#include "aerial_robot_control/nmpc/tilt_bi_2_ord_servo_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc_tilt_bi_2_ord::NMPCController::initCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double Qp_xy, Qp_z, Qv_xy, Qv_z, Qq_xy, Qq_z, Qw_xy, Qw_z, Qa, Qb, Rt, Rac_d;
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 300);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 400);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 300);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5);
  getParam<double>(nmpc_nh, "Qa", Qa, 1);
  getParam<double>(nmpc_nh, "Qb", Qb, 0);
  getParam<double>(nmpc_nh, "Rt", Rt, 1);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
  mpc_solver_ptr_->setCostWDiagElement(0, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(1, Qp_xy);
  mpc_solver_ptr_->setCostWDiagElement(2, Qp_z);
  mpc_solver_ptr_->setCostWDiagElement(3, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(4, Qv_xy);
  mpc_solver_ptr_->setCostWDiagElement(5, Qv_z);
  mpc_solver_ptr_->setCostWDiagElement(6, 0);
  mpc_solver_ptr_->setCostWDiagElement(7, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(8, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(9, Qq_z);
  mpc_solver_ptr_->setCostWDiagElement(10, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(11, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(12, Qw_z);
  for (int i = 13; i < 15; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 15; i < 17; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qb);
  for (int i = 17; i < 19; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rt, false);
  for (int i = 19; i < 21; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
  mpc_solver_ptr_->setCostWeight(true, true);
}

std::vector<double> nmpc_tilt_bi_2_ord::NMPCController::meas2VecX()
{
  vector<double> bx0(mpc_solver_ptr_->NBX0_, 0);
  bx0[0] = odom_.pose.pose.position.x;
  bx0[1] = odom_.pose.pose.position.y;
  bx0[2] = odom_.pose.pose.position.z;
  bx0[3] = odom_.twist.twist.linear.x;
  bx0[4] = odom_.twist.twist.linear.y;
  bx0[5] = odom_.twist.twist.linear.z;
  bx0[6] = odom_.pose.pose.orientation.w;
  bx0[7] = odom_.pose.pose.orientation.x;
  bx0[8] = odom_.pose.pose.orientation.y;
  bx0[9] = odom_.pose.pose.orientation.z;
  bx0[10] = odom_.twist.twist.angular.x;
  bx0[11] = odom_.twist.twist.angular.y;
  bx0[12] = odom_.twist.twist.angular.z;
  for (int i = 0; i < joint_num_; i++)
    bx0[13 + i] = joint_angles_[i];
  for (int i = 0; i < joint_num_; i++)
    bx0[13 + joint_num_ + i] = joint_vel_[i];
  return bx0;
}

void nmpc_tilt_bi_2_ord::NMPCController::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_angles_[0] = msg->position[0];
  joint_angles_[1] = msg->position[1];
  joint_vel_[0] = msg->velocity[0];
  joint_vel_[1] = msg->velocity[1];
}

void nmpc_tilt_bi_2_ord::NMPCController::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (config.nmpc_flag)
  {
    switch (level)
    {
      case Levels::RECONFIGURE_NMPC_Q_P_XY: {
        mpc_solver_ptr_->setCostWDiagElement(0, config.Qp_xy);
        mpc_solver_ptr_->setCostWDiagElement(1, config.Qp_xy);
        ROS_INFO_STREAM("change Qp_xy for NMPC '" << config.Qp_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_P_Z: {
        mpc_solver_ptr_->setCostWDiagElement(2, config.Qp_z);
        ROS_INFO_STREAM("change Qp_z for NMPC '" << config.Qp_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_V_XY: {
        mpc_solver_ptr_->setCostWDiagElement(3, config.Qv_xy);
        mpc_solver_ptr_->setCostWDiagElement(4, config.Qv_xy);
        ROS_INFO_STREAM("change Qv_xy for NMPC '" << config.Qv_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_V_Z: {
        mpc_solver_ptr_->setCostWDiagElement(5, config.Qv_z);
        ROS_INFO_STREAM("change Qv_z for NMPC '" << config.Qv_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_Q_XY: {
        mpc_solver_ptr_->setCostWDiagElement(7, config.Qq_xy);
        mpc_solver_ptr_->setCostWDiagElement(8, config.Qq_xy);
        ROS_INFO_STREAM("change Qq_xy for NMPC '" << config.Qq_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_Q_Z: {
        mpc_solver_ptr_->setCostWDiagElement(9, config.Qq_z);
        ROS_INFO_STREAM("change Qq_z for NMPC '" << config.Qq_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_W_XY: {
        mpc_solver_ptr_->setCostWDiagElement(10, config.Qw_xy);
        mpc_solver_ptr_->setCostWDiagElement(11, config.Qw_xy);
        ROS_INFO_STREAM("change Qw_xy for NMPC '" << config.Qw_xy << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_W_Z: {
        mpc_solver_ptr_->setCostWDiagElement(12, config.Qw_z);
        ROS_INFO_STREAM("change Qw_z for NMPC '" << config.Qw_z << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_Q_A: {
        for (int i = 13; i < 13 + joint_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
        ROS_INFO_STREAM("change Qa for NMPC '" << config.Qa << "'");
        break;
      }
        //      case Levels::RECONFIGURE_NMPC_Q_B: {  // TODO： add if necessary
        //        for (int i = 13 + joint_num_; i < 13 + joint_num_ + joint_num_; ++i)
        //          mpc_solver_ptr_->setCostWDiagElement(i, config.Qb);
        //        ROS_INFO_STREAM("change Qb for NMPC '" << config.Qb << "'");
        //        break;
        //      }
      case Levels::RECONFIGURE_NMPC_R_T: {
        for (int i = 13 + joint_num_ * 2; i < 13 + joint_num_ * 2 + motor_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Rt, false);
        ROS_INFO_STREAM("change Rt for NMPC '" << config.Rt << "'");
        break;
      }
      case Levels::RECONFIGURE_NMPC_R_AC_D: {
        for (int i = 13 + joint_num_ * 2 + motor_num_; i < 13 + joint_num_ * 2 + motor_num_ + joint_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
        ROS_INFO_STREAM("change Rac_d for NMPC '" << config.Rac_d << "'");
        break;
      }
      default:
        break;
    }
    mpc_solver_ptr_->setCostWeight(true, true);
  }
}

void nmpc_tilt_bi_2_ord::NMPCController::calXrUrRef(const tf::Vector3 target_pos, const tf::Vector3 target_vel,
                                                    const tf::Vector3 target_rpy, const tf::Vector3 target_omega,
                                                    const Eigen::VectorXd& target_wrench)
{
  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * target_wrench;
  double a1_ref = atan2(x_lambda(0), x_lambda(1));
  double ft1_ref = sqrt(x_lambda(0) * x_lambda(0) + x_lambda(1) * x_lambda(1));
  double a2_ref = atan2(x_lambda(2), x_lambda(3));
  double ft2_ref = sqrt(x_lambda(2) * x_lambda(2) + x_lambda(3) * x_lambda(3));

  tf::Quaternion q;
  q.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());

  int& NX = mpc_solver_ptr_->NX_;
  int& NU = mpc_solver_ptr_->NU_;
  int& NN = mpc_solver_ptr_->NN_;

  double x[] = { target_pos.x(),
                 target_pos.y(),
                 target_pos.z(),
                 target_vel.x(),
                 target_vel.y(),
                 target_vel.z(),
                 q.w(),
                 q.x(),
                 q.y(),
                 q.z(),
                 target_omega.x(),
                 target_omega.y(),
                 target_omega.z(),
                 a1_ref,
                 a2_ref,
                 0.0,
                 0.0 };
  double u[] = { ft1_ref, ft2_ref, 0.0, 0.0 };

  // Aim: gently add the target point to the end of the reference trajectory
  // - x: NN + 1, u: NN
  // - for 0 ~ NN-2 x and u, shift
  // - copy x to x: NN-1 and NN, copy u to u: NN-1
  for (int i = 0; i < NN - 1; i++)
  {
    // shift one step
    std::copy(x_u_ref_.x.data.begin() + NX * (i + 1), x_u_ref_.x.data.begin() + NX * (i + 2),
              x_u_ref_.x.data.begin() + NX * i);
    std::copy(x_u_ref_.u.data.begin() + NU * (i + 1), x_u_ref_.u.data.begin() + NU * (i + 2),
              x_u_ref_.u.data.begin() + NU * i);
  }
  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * (NN - 1));
  std::copy(u, u + NU, x_u_ref_.u.data.begin() + NU * (NN - 1));

  std::copy(x, x + NX, x_u_ref_.x.data.begin() + NX * NN);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc_tilt_bi_2_ord::NMPCController, aerial_robot_control::ControlBase);
