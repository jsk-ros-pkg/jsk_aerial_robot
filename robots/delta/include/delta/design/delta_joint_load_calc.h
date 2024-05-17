#pragma once

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_model/model/aerial_robot_model_ros.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <delta/model/delta_robot_model.h>
#include <fstream>
#include <sensor_msgs/JointState.h>


class DeltaJointLoadCalc
{
public:
  DeltaJointLoadCalc(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~DeltaJointLoadCalc(){}
  void jointAngleSearch();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<RollingRobotModel> transformable_robot_model_;

  std::ofstream ofs_;
  bool verbose_;
  int estimated_max_count_;
  int search_cnt_;
  int wrench_mode_;
  double start_time_;

  // robot model
  int rotor_num_;
  std::vector<double> rotor_tilt_;
  double max_thrust_;
  Eigen::VectorXd joint_torque_;
  KDL::JntArray joint_positions_;


  // joint
  double joint_angle_min_;
  double joint_angle_max_;
  int joint_angle_cnt_max_;


  // pose
  double pose_roll_max_abs_;
  double pose_pitch_max_abs_;
  int pose_roll_cnt_max_;
  int pose_pitch_cnt_max_;


  // wrench matrix
  Eigen::VectorXd desired_wrench_;
  Eigen::MatrixXd full_q_mat_;
  Eigen::MatrixXd full_q_mat_inv_;


  // wrench z
  int wrench_z_cnt_max_;
  double wrench_z_min_acc_;
  double wrench_z_max_acc_;


  // wrench torque
  double wrench_roll_max_abs_;
  double wrench_pitch_max_abs_;
  int wrench_torque_cnt_max_;


  // thruster inputs
  Eigen::VectorXd target_vectoring_force_;
  Eigen::VectorXd target_thrust_;
  Eigen::VectorXd target_gimbal_angles_;


  void wrenchAllocation();
  void computeJointTorque();

  template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }

};
