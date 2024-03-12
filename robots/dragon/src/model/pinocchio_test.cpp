#include "pinocchio/fwd.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hxx"
#include "pinocchio/algorithm/model.hxx"
#include "pinocchio/autodiff/casadi.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
// #include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/sugihara/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/dragon/robots"
#endif

class PinocchioRobotModel
{
  public:
  PinocchioRobotModel();
  ~PinocchioRobotModel() = default;

  void modelInit();
  void kinematicsInit();
  void inertialInit();

  pinocchio::Model model_dbl_;
  pinocchio::Data data_dlb_;
  pinocchio::ModelTpl<casadi::SX> model_;
  pinocchio::DataTpl<casadi::SX> data_;

  std::vector<casadi::SX> rotor_origin_root_;
  std::vector<casadi::SX> rotor_normal_root_;

  casadi::SX q_cs_;
  Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q_;
  casadi::SX mass_;
  casadi::SX cog_pos_;
  casadi::SX inertia_;

  std::map<std::string, int> joint_index_map_;
  int rotor_num_;

};

PinocchioRobotModel::PinocchioRobotModel()
{
  modelInit();
  kinematicsInit();
  inertialInit();
}

Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, Eigen::VectorXd x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
  {
    for(int j = 0; j < y.size2(); j++)
    {
      casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
      casadi::DM y_dbl = f(eigenVectorToCasadiDm(x_dbl));
      ret(i, j) = y_dbl;
    }
  }
  return casadiDmToEigenMatrix(ret);
}

Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, casadi::DM x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
  {
    for(int j = 0; j < y.size2(); j++)
    {
      casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
      casadi::DM y_dbl = f(x_dbl);
      ret(i, j) = y_dbl;
    }
  }
  return casadiDmToEigenMatrix(ret);
}

void PinocchioRobotModel::modelInit()
{
  // You should change here to set up your own URDF file or just pass it as an argument of this example.
  const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/quad/robot.urdf");

  // Load the urdf model
  pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_dbl_);
  std::cout << "model name: " << model_dbl_.name << std::endl;

  // Create data required by the algorithms
  pinocchio::Data data_dlb_(model_dbl_);

  // declaraion of model and data with casadi
  model_ = model_dbl_.cast<casadi::SX>();
  data_ = pinocchio::DataTpl<casadi::SX>(model_);

  std::cout << "model_.nq: " << model_.nq << std::endl;
  std::cout << "model_.nv: " << model_.nv << std::endl;
  std::cout << "model_.njoints: " << model_.njoints << std::endl;
  std::cout << std::endl;

  std::vector<int> q_dims(model_.njoints);
  for(int i = 0; i < model_.njoints; i++)
  {
    std::string joint_type = model_.joints[i].shortname();
    // std::cout << model_.names[i] << " " << joint_type << std::endl;
    if(joint_type == "JointModelFreeFlyer")
      q_dims.at(i) = 7;
    else if(joint_type == "JointModelRUBX" || joint_type == "JointModelRUBY" || joint_type == "JointModelRUBZ")
      q_dims.at(i) = 2;
    else
      q_dims.at(i) = 1;
  }

  int joint_index = 0;
  rotor_num_ = 0;
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
  {
    std::cout << model_.names[joint_id] << std::endl;
    if(model_.names[joint_id] == "universe")
    {
      std::cout << "find joint named universe" << std::endl;
    }
    else
    {
      joint_index_map_[model_.names[joint_id]] = joint_index;
      joint_index += q_dims.at(joint_id);

      // special process for rotor
      if(model_.names[joint_id].find("rotor") != std::string::npos)
      {
        std::cout << "found rotor named " << model_.names[joint_id] << std::endl; 
        rotor_num_++;
      }
    }
  }
  std::cout << std::endl;
  // std::cout << "rotor num: " << rotor_num_ << std::endl;

  // check the joint index map
  std::cout << "joint index map: \n";
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
  {
    if(joint_index_map_.count(model_.names[joint_id]))
    {
      std::cout << model_.names[joint_id] << " :" << joint_index_map_.at(model_.names[joint_id]) << std::endl;
    }
  }
  std::cout << std::endl;
}

void PinocchioRobotModel::kinematicsInit()
{
  // init q
  q_cs_ = casadi::SX::sym("q", model_.nq);
  q_.resize(model_.nq, 1);
  for(int i = 0; i < model_.nq; i++)
  {
    q_(i) = q_cs_(i);
  }
  // std::cout << "q: " << q_.transpose() << std::endl;

  // solve FK
  pinocchio::forwardKinematics(model_, data_, q_);

  // setup rotor info
  rotor_origin_root_.resize(rotor_num_);
  rotor_normal_root_.resize(rotor_num_);

  for(int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_name = "rotor" + std::to_string(i + 1);
    int joint_id =  model_.getJointId(rotor_name);
    // std::cout << rotor_name << " " << joint_id << std::endl;
    casadi::SX rotor_origin_root = casadi::SX::zeros(3);
    for(int j = 0; j < 3; j++)
      rotor_origin_root(j) = data_.oMi[joint_id].translation()(j);
    rotor_origin_root_.at(i) = rotor_origin_root;
    // std::cout << "rotor origin: " << i << " " << rotor_origin_root_.at(i) << std::endl;

    casadi::SX rotor_normal_root = casadi::SX::zeros(3);
    int rotor_axis_type;
    if(model_.joints[joint_id].shortname() == "JointModelRX" || model_.joints[joint_id].shortname() == "JointModelRUBX")
      rotor_axis_type = 0;
    else if(model_.joints[joint_id].shortname() == "JointModelRY" || model_.joints[joint_id].shortname() == "JointModelRUBY")
      rotor_axis_type = 1;
    else if(model_.joints[joint_id].shortname() == "JointModelRZ" || model_.joints[joint_id].shortname() == "JointModelRUBZ")
      rotor_axis_type = 2;

    for(int j = 0; j < 3; j++)
    {
      rotor_normal_root(j) = data_.oMi[joint_id].rotation()(j, rotor_axis_type);
    }
    // std::cout << "rotor rotation: \n" << data_.oMi[joint_id].rotation() << std::endl;
    rotor_normal_root_.at(i) = rotor_normal_root;
    // std::cout << "rotor axis for rotor" << i << ": " << rotor_axis_type << std::endl;
    // std::cout << "rotor axis: " << i << " " << rotor_normal_root_.at(i) << std::endl;
    // std::cout << std::endl;
  }

  // Print out the placement of each joint of the kinematic tree
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model_.names[joint_id] << ": "
  //             << std::setw(24) << std::left
  //             << model_.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << data_.oMi[joint_id].translation().transpose()
  //             << std::endl;
}

void PinocchioRobotModel::inertialInit()
{
  mass_ = pinocchio::computeTotalMass(model_);
  std::cout << "mass: " << mass_ << std::endl;

  auto cog_pos = pinocchio::centerOfMass(model_, data_, q_, true);
  pinocchio::casadi::copy(cog_pos, cog_pos_);
  // std::cout << "cog_pos_: "cog_pos_ << std::endl;
  std::cout << std::endl;

  casadi::DM q_test = casadi::DM(model_.nq, 1);
  q_test(joint_index_map_["joint1_yaw"]) = 1.57;
  q_test(joint_index_map_["joint2_yaw"]) = 1.57;
  q_test(joint_index_map_["joint3_yaw"]) = 1.57;
  std::cout << "q_test: " << q_test << std::endl;
  std::cout << std::endl;
  std::cout << "real cog: " << computeRealValue(cog_pos_, q_cs_, q_test).transpose() << std::endl;
  std::cout << std::endl;
}

int main(int argc, char ** argv)
{
  PinocchioRobotModel pinocchio_robot_model;

  // // zero configuration
  // Eigen::VectorXd q = randomConfiguration(model);
  // q = Eigen::VectorXd::Zero(q.rows());
  // std::cout << "q: " << q.transpose() << std::endl;

  // // joint space with casadi
  // auto q_cs = casadi::SX::sym("q", model.nq);
  // Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q_casadi;
  // q_casadi.resize(model.nq, 1);
  // for(int i = 0; i < model.nq; i++)
  //   q_casadi(i) = q_cs(i);

  // // Perform the forward kinematics over the kinematic tree
  // pinocchio::forwardKinematics(model, data, q);

  // // Print out the placement of each joint of the kinematic tree
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model.names[joint_id] << ": "
  //             << std::setw(15) << std::left
  //             << model.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << data.oMi[joint_id].translation().transpose()
  //             << std::endl;

  // std::cout << std::endl;
  // std::cout << std::endl;

  // std::cout << "q_casadi: " << q_casadi.transpose() << std::endl;
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_casadi.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model_casadi.names[joint_id] << ": "
  //             << std::setw(15) << std::left
  //             << model_casadi.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << data_casadi.oMi[joint_id].translation().transpose()
  //             << std::endl;

  // std::cout << std::endl;

  // // check the position of each joint
  // casadi::DM casadi_dbl = casadi::DM({0, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  // std::cout << "casadi_dbl: " << casadi_dbl << std::endl;
  // for(int i = 0; i < model.nv; i++)
  // {
  //   for(int j = 0; j < 3; j++)
  //   {
  //     casadi::Function f = casadi::Function("f", {q_cs}, {data_casadi.oMi[i].translation()(j)});
  //     casadi::DM pos_j = f(casadi_dbl);
  //     std::cout << pos_j << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << std::endl;

  // // check jacobian
  // pinocchio::computeJointJacobians(model, data, q);
  // std::cout << "data.J: \n" << data.J << std::endl;
  // std::cout << std::endl;

  // pinocchio::computeJointJacobians(model_casadi, data_casadi, q_casadi);
  // std::cout << "data.J: \n" << data_casadi.J << std::endl;
  // std::cout << std::endl;

  // // kinematics derivatives
  // Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  // Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  // // Computes the kinematics derivatives for all the joints of the robot
  // pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  // // Retrieve the kinematics derivatives of a specific joint, expressed in the LOCAL frame of the joints.
  // pinocchio::JointIndex joint_id = (pinocchio::JointIndex)(model.njoints-1);
  // pinocchio::Data::Matrix6x v_partial_dq(6, model.nv), a_partial_dq(6, model.nv), a_partial_dv(6, model.nv), a_partial_da(6, model.nv);
  // v_partial_dq.setZero();
  // a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  // pinocchio::getJointAccelerationDerivatives(model, data, joint_id, pinocchio::LOCAL, v_partial_dq,
  //                                            a_partial_dq, a_partial_dv, a_partial_da);

  // std::cout << "v partial dq: \n" << v_partial_dq << std::endl;
  // std::cout << std::endl;
  // // Remark: we are not directly computing the quantity v_partial_dv as it is also equal to a_partial_da.

  // // But we can also expressed the same quantities in the frame centered on the end-effector joint, but expressed in the axis aligned with the world frame.
  // pinocchio::getJointAccelerationDerivatives(model,data, joint_id, pinocchio::WORLD, v_partial_dq,
  //                                            a_partial_dq, a_partial_dv, a_partial_da);
  // std::cout << "v partial dq: \n" << v_partial_dq << std::endl;

}
