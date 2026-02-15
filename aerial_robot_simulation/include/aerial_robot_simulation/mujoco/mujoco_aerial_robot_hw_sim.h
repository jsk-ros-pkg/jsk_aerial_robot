#pragma once

#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <mujoco_ros_control/default_robot_hw_sim.h>
#include <nav_msgs/Odometry.h>

namespace mujoco_ros_control
{
class AerialRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
  AerialRobotHWSim(){};
  ~AerialRobotHWSim()
  {
  }

  bool initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
               const std::string& robot_namespace, ros::NodeHandle model_nh, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;

  void writeSim(ros::Time time, ros::Duration period) override;

protected:
  hardware_interface::MujocoSpinalInterface spinal_interface_;

  std::vector<std::string> rotor_list_;
  ros::Publisher ground_truth_pub_;
  ros::Publisher mocap_pub_;
  double ground_truth_pub_rate_;
  double mocap_pub_rate_;
  double mocap_rot_noise_, mocap_pos_noise_;
  double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
  double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
  double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
  double joint_state_pub_rate_ = 0.02;

  ros::Time last_mocap_time_;
};
}  // namespace mujoco_ros_control
