#include <aerial_robot_dynamics/robot_model.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ros/ros.h>
#include <iostream>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_test");
  ros::NodeHandle nh;

  bool verbose = false;
  if(argc > 1)
    {
      std::string arg = argv[1];
      if(arg != "0")
        verbose = true;
    }

  aerial_robot_dynamics::PinocchioRobotModel robot_model;

  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_model.getModel()->nq);
  q(6) = 1;
  q(robot_model.getModel()->joints[robot_model.getModel()->getJointId("joint1_yaw")].idx_q()) = M_PI / 2.0;
  q(robot_model.getModel()->joints[robot_model.getModel()->getJointId("joint2_yaw")].idx_q()) = M_PI / 2.0;
  q(robot_model.getModel()->joints[robot_model.getModel()->getJointId("joint3_yaw")].idx_q()) = M_PI / 2.0;

  pinocchio::forwardKinematics(*(robot_model.getModel()), *(robot_model.getData()), q);
  std::cout << std::endl;
  std::cout << "gimbal1_roll " << robot_model.getData()->oMi[robot_model.getModel()->getJointId("gimbal1_roll")].translation().transpose() << std::endl;
  std::cout << "gimbal2_roll " << robot_model.getData()->oMi[robot_model.getModel()->getJointId("gimbal2_roll")].translation().transpose() << std::endl;
  std::cout << "gimbal3_roll " << robot_model.getData()->oMi[robot_model.getModel()->getJointId("gimbal3_roll")].translation().transpose() << std::endl;
  std::cout << "gimbal4_roll " << robot_model.getData()->oMi[robot_model.getModel()->getJointId("gimbal4_roll")].translation().transpose() << std::endl;
  std::cout << std::endl;

  // Test the robot model
  // FD with thrust and its test
  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool fd_test = robot_model.forwardDynamicsTest(verbose);
      std::cout << "forwardDynamicsTest: " << fd_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }

  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool id_test = robot_model.inverseDynamicsTest(verbose);
      std::cout << "inverseDynamicsTest: " << id_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }

  return 0;
}
