//
// Created by li-jinjie on 23-11-25.
//
#include "aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::FixQdNMPC::initAllocMat()
{
  alloc_mat_ = Eigen::Matrix<double, 6, 4>::Zero();

  const auto& rotor_p = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Vector3d p1_b = rotor_p[0];
  Eigen::Vector3d p2_b = rotor_p[1];
  Eigen::Vector3d p3_b = rotor_p[2];
  Eigen::Vector3d p4_b = rotor_p[3];

  const map<int, int> rotor_dr = robot_model_->getRotorDirection();
  int dr1 = rotor_dr.find(1)->second;
  int dr2 = rotor_dr.find(2)->second;
  int dr3 = rotor_dr.find(3)->second;
  int dr4 = rotor_dr.find(4)->second;

  double kq_d_kt = robot_model_->getThrustWrenchUnits()[0][5];

  // - force
  alloc_mat_(2, 0) = 1;
  alloc_mat_(2, 1) = 1;
  alloc_mat_(2, 2) = 1;
  alloc_mat_(2, 3) = 1;
  // - torque
  alloc_mat_(3, 0) = p1_b.y();
  alloc_mat_(3, 1) = p2_b.y();
  alloc_mat_(3, 2) = p3_b.y();
  alloc_mat_(3, 3) = p4_b.y();
  alloc_mat_(4, 0) = -p1_b.x();
  alloc_mat_(4, 1) = -p2_b.x();
  alloc_mat_(4, 2) = -p3_b.x();
  alloc_mat_(4, 3) = -p4_b.x();
  alloc_mat_(5, 0) = -dr1 * kq_d_kt;
  alloc_mat_(5, 1) = -dr2 * kq_d_kt;
  alloc_mat_(5, 2) = -dr3 * kq_d_kt;
  alloc_mat_(5, 3) = -dr4 * kq_d_kt;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::FixQdNMPC, aerial_robot_control::ControlBase);
