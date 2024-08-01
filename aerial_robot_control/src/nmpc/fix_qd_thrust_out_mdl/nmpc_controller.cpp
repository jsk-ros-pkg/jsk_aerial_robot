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

void nmpc::FixQdNMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                   const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                   const Eigen::VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u) const
{
  x.at(0) = ref_pos_i.x();
  x.at(1) = ref_pos_i.y();
  x.at(2) = ref_pos_i.z();
  x.at(3) = ref_vel_i.x();
  x.at(4) = ref_vel_i.y();
  x.at(5) = ref_vel_i.z();
  x.at(6) = ref_quat_ib.w();
  x.at(7) = ref_quat_ib.x();
  x.at(8) = ref_quat_ib.y();
  x.at(9) = ref_quat_ib.z();
  x.at(10) = ref_omega_b.x();
  x.at(11) = ref_omega_b.y();
  x.at(12) = ref_omega_b.z();

  Eigen::VectorXd x_lambda = alloc_mat_pinv_ * ref_wrench_b;
  for (int i = 0; i < x_lambda.size(); i++)  // only for fixed multirotor
    u.at(i) = x_lambda(i);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::FixQdNMPC, aerial_robot_control::ControlBase);
