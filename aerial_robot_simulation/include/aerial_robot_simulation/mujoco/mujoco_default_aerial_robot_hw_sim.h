#pragma once

#include <aerial_robot_simulation/rotor_controller.h>
#include <mujoco_ros_control/default_robot_hw_sim.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace mujoco_ros::control
{
class DefaultAerialRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
  DefaultAerialRobotHWSim(){};
  ~DefaultAerialRobotHWSim()
  {
  }

  bool InitSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
               const std::string& robot_namespace, ros::NodeHandle model_nh, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions, bool ignore_actuators = false) override;

  void ReadSim(ros::Time time, ros::Duration period) override;

  void WriteSim(ros::Time time, ros::Duration period) override;

private:
  hardware_interface::RotorInterface rotor_interface_;

  double odom_pub_rate_;
  double odom_pub_last_time_;
  ros::Publisher odom_pub_;

  double tf_broadcast_rate_;
  double tf_broadcast_last_time_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string robot_ns_;

  std::vector<std::string> rotor_names_;
  std::vector<double> rotor_cmd_;
  std::vector<double> rotor_pos_;
  std::vector<double> rotor_vel_;
  std::vector<double> rotor_eff_;

  std::string root_link_name_ = "";
  bool publish_odom_ = true;
  bool publish_tf_ = false;

  void publishOdometry(ros::Time time);
  void publishTF(ros::Time time);
};
}  // namespace mujoco_ros::control
