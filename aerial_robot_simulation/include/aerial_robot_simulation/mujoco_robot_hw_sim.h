#pragma once

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>
#include <transmission_interface/transmission_info.h>
#include <pluginlib/class_list_macros.h>

#include <aerial_robot_simulation/mujoco_robot_hw_sim_plugin.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

namespace mujoco_ros_control
{
  class MujocoRobotHWSim : public mujoco_ros_control::MujocoRobotHWSimPlugin
  {
  public:
    MujocoRobotHWSim() {};
    ~MujocoRobotHWSim() {}

    bool init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              );

    void read(const ros::Time& time, const ros::Duration& period);

    void write(const ros::Time& time, const ros::Duration& period);

    void controlInputCallback(const sensor_msgs::JointState & msg);

    const static uint8_t FORCE_CONTROL_MODE = 0;
    const static uint8_t SIM_VEL_MODE = 1;
    const static uint8_t SIM_POS_MODE = 2;

  protected:
    mjModel* mujoco_model_;
    mjData* mujoco_data_;
    hardware_interface::MujocoSpinalInterface spinal_interface_;

  private:
    uint8_t control_mode_;
    ros::Subscriber sim_vel_sub_, sim_pos_sub_;
    std::vector<std::string> joint_list_;
    std::vector<std::string> rotor_list_;
    ros::Publisher ground_truth_pub_;
    ros::Publisher mocap_pub_;
    ros::Publisher joint_state_pub_;
    ros::Subscriber control_input_sub_;
    double ground_truth_pub_rate_;
    double mocap_pub_rate_;
    double mocap_rot_noise_, mocap_pos_noise_;
    double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
    double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
    double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
    double joint_state_pub_rate_ = 0.02;
    std::vector<double> control_input_;

    ros::Time last_mocap_time_, last_joint_state_time_;

    geometry_msgs::TwistStamped cmd_vel_;
    geometry_msgs::PoseStamped cmd_pos_;

    void cmdVelCallback(const geometry_msgs::TwistStampedConstPtr& cmd_vel)
    {
      control_mode_ = SIM_VEL_MODE;
      cmd_vel_ = *cmd_vel;
    }

    void cmdPosCallback(const geometry_msgs::PoseStampedConstPtr& cmd_pos)
    {
      control_mode_ = SIM_POS_MODE;
      cmd_pos_ = *cmd_pos;
    }
  };
}
