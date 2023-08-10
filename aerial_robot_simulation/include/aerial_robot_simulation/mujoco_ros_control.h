#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/tf.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <aerial_robot_simulation/mujoco_visualization_utils.h>
#include <aerial_robot_msgs/ControlInput.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <rosgraph_msgs/Clock.h>

class MujocoRosControl
{
public:
  MujocoRosControl(ros::NodeHandle nh);
  virtual ~MujocoRosControl();

  void update();
  void clockCallback(const ros::TimerEvent & e);
  void publishCallback(const ros::TimerEvent & e);

private:
  ros::NodeHandle nh_;
  ros::Timer clock_timer_;
  ros::Timer publish_timer_;
  ros::Time zero_time_;

  ros::Subscriber control_input_sub_;
  ros::Subscriber four_axis_command_sub_;
  ros::Publisher mocap_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher joint_state_pub_;
  ros::Publisher clock_pub_;

  float mocap_pub_last_time_;
  float imu_pub_last_time_;
  float joint_state_pub_last_time_;
  double mocap_pub_rate_ = 0.0100;
  double imu_pub_rate_ = 0.00500;
  double joint_state_pub_rate_ = 0.0200;
  int publish_cnt_ = 0;
  void fourAxisCommandCallback(const spinal::FourAxisCommand & msg);
  void controlInputCallback(const aerial_robot_msgs::ControlInput & msg);
  double getCurrentTime();

  ros::AsyncSpinner clock_spinner_;
  ros::AsyncSpinner publish_spinner_;
  ros::AsyncSpinner callback_spinner_;
  ros::CallbackQueue clock_loop_queue_;
  ros::CallbackQueue publish_loop_queue_;

  mjModel* mujoco_model_;
  mjData* mujoco_data_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> rotor_names_;
  std::vector<double> control_input_;
};
