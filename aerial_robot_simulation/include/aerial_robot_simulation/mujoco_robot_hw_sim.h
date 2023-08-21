#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>

#include <aerial_robot_model/model/aerial_robot_model.h>
#include <aerial_robot_simulation/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/mujoco_attitude_controller.h>
#include <aerial_robot_simulation/mujoco_visualization_utils.h>
#include <mujoco/mujoco.h>


class MujocoRobotHWSim
{
public:
  MujocoRobotHWSim(ros::NodeHandle nh);
  virtual ~MujocoRobotHWSim();
  bool initSim();
  void readSim(const ros::TimerEvent & e);
  void writeSim();
  void clockCallback(const ros::TimerEvent & e);

private:
  ros::NodeHandle nh_;
  ros::Timer clock_timer_;
  ros::Timer read_timer_;
  ros::Time zero_time_;

  ros::Publisher clock_pub_;
  ros::Publisher mocap_pub_;

  ros::AsyncSpinner clock_spinner_;
  ros::AsyncSpinner read_spinner_;
  ros::CallbackQueue clock_loop_queue_;
  ros::CallbackQueue read_loop_queue_;

  mjModel* mujoco_model_;
  mjData* mujoco_data_;

  boost::shared_ptr <hardware_interface::MujocoSpinalInterface> spinal_interface_;
  MujocoAttitudeController simulation_attitude_controller_;

  double mocap_pub_rate_ = 0.0100;
  ros::Time last_mocap_time_ = (ros::Time) 0.0;

};

