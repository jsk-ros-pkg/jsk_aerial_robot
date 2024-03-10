#include "pinocchio/fwd.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/model.hxx"
#include "pinocchio/autodiff/casadi.hpp"
// #include <casadi/casadi.hpp>
// #include <CasadiEigen/CasadiEigen.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/sugihara/ros/test_ws/src/jsk_aerial_robot/robots/dragon/robots"
#endif

int main(int argc, char ** argv)
{
  // You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("/quad/robot.urdf") : argv[1];

  // Load the urdf model
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;

  // Create data required by the algorithms
  pinocchio:: Data data(model);

  std::cout << "model.nq: " << model.nq << std::endl;
  std::cout << "model.nv: " << model.nv << std::endl;

  // Sample a random configuration
  Eigen::VectorXd q = randomConfiguration(model);
  q = Eigen::VectorXd::Zero(q.rows());
  // q(model.getJointId("joint1_yaw")) = ;
  // q(model.getJointId("joint2_yaw")) = 1.57;
  // q(model.getJointId("joint3_yaw")) = 1.57;
  std::cout << "q: " << q.transpose() << std::endl;

  auto model_casadi = model.cast<casadi::SX>();
  pinocchio::DataTpl<casadi::SX> data_casadi(model_casadi);
  Eigen::Matrix<casadi::SX, 22, 1> q_casadi;
  q_casadi(0) = casadi::SX::sym("gimbal1_roll", 1);
  q_casadi(1) = casadi::SX::sym("gimbal1_pitch", 1);
  // q_casadi(2) = casadi::SX::sym("gimbal1_roll", 1);
  // q_casadi(3) = casadi::SX::sym("gimbal1_roll", 1);
  q_casadi(4) = casadi::SX::sym("joint1_pitch", 1);
  q_casadi(5) = casadi::SX::sym("joint1_yaw", 1);
  q_casadi(6) = casadi::SX::sym("gimbal2_roll", 1);
  q_casadi(7) = casadi::SX::sym("gimbal2_pitch", 1);
  // q_casadi(8) = casadi::SX::sym("gimbal1_roll", 1);
  // q_casadi(9) = casadi::SX::sym("gimbal1_pitch", 1);
  q_casadi(10) = casadi::SX::sym("joint2_pitch", 1);
  q_casadi(11) = casadi::SX::sym("joint2_yaw", 1);
  q_casadi(12) = casadi::SX::sym("gimbal3_roll", 1);
  q_casadi(13) = casadi::SX::sym("gimbal3_pitch", 1);
  // q_casadi(14) = casadi::SX::sym("gimbal1_roll", 1);
  // q_casadi(15) = casadi::SX::sym("gimbal1_pitch", 1);
  q_casadi(16) = casadi::SX::sym("joint3_pitch", 1);
  q_casadi(17) = casadi::SX::sym("joint3_yaw", 1);
  q_casadi(18) = casadi::SX::sym("gimbal4_roll", 1);
  q_casadi(19) = casadi::SX::sym("gimbal4_pitch", 1);
  // q_casadi(20) = casadi::SX::sym("gimbal1_roll", 1);
  // q_casadi(21) = casadi::SX::sym("gimbal1_pitch", 1);

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::forwardKinematics(model_casadi, data_casadi, q_casadi);

  // Print out the placement of each joint of the kinematic tree
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::setw(15) << std::left
              << model.joints[joint_id].shortname() << ": "
              << std::fixed << std::setprecision(5)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;

  std::cout << std::endl;
  std::cout << std::endl;

  std::cout << "q_casadi: " << q_casadi.transpose() << std::endl;
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_casadi.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model_casadi.names[joint_id] << ": "
              << std::setw(15) << std::left
              << model_casadi.joints[joint_id].shortname() << ": "
              << std::fixed << std::setprecision(5)
              << data_casadi.oMi[joint_id].translation().transpose()
              << std::endl;

  std::cout << std::endl;

  // check the rotor pos (hard coded)
  Eigen::Matrix<casadi::SX, 3, 1> pos = data_casadi.oMi[model_casadi.getJointId("gimbal1_pitch")].translation();
  std::cout << std::setw(20) << std::left
            << "gimbal1_pitch pos" << ": " << pos.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

  Eigen::Matrix<casadi::SX, 3, 1> pos_j;
  for(int i = 0; i < pos.rows(); i++)
    {
      pos_j(i) = jacobian(pos(i), q_casadi(0));
      casadi::Function S = casadi::Function("S", {q_casadi(0)}, {pos_j(i)});
    }
  std::cout << std::setw(20) << std::left
            << "gimbal1_pitch pos'" << ": " << pos_j.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;


  // check jacobian
  pinocchio::computeJointJacobians(model, data, q);
  std::cout << "data.J: \n" << data.J << std::endl;
  std::cout << std::endl;

  pinocchio::computeJointJacobians(model_casadi, data_casadi, q_casadi);
  std::cout << "data.J: \n" << data_casadi.J << std::endl;
  std::cout << std::endl;

  // kinematics derivatives
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  // Computes the kinematics derivatives for all the joints of the robot
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  // Retrieve the kinematics derivatives of a specific joint, expressed in the LOCAL frame of the joints.
  pinocchio::JointIndex joint_id = (pinocchio::JointIndex)(model.njoints-1);
  pinocchio::Data::Matrix6x v_partial_dq(6, model.nv), a_partial_dq(6, model.nv), a_partial_dv(6, model.nv), a_partial_da(6, model.nv);
  v_partial_dq.setZero();
  a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  pinocchio::getJointAccelerationDerivatives(model, data, joint_id, pinocchio::LOCAL, v_partial_dq,
                                             a_partial_dq, a_partial_dv, a_partial_da);

  std::cout << "v partial dq: \n" << v_partial_dq << std::endl;
  std::cout << std::endl;
  // Remark: we are not directly computing the quantity v_partial_dv as it is also equal to a_partial_da.

  // But we can also expressed the same quantities in the frame centered on the end-effector joint, but expressed in the axis aligned with the world frame.
  pinocchio::getJointAccelerationDerivatives(model,data, joint_id, pinocchio::WORLD, v_partial_dq,
                                             a_partial_dq, a_partial_dv, a_partial_da);
  std::cout << "v partial dq: \n" << v_partial_dq << std::endl;

}
