#pragma once

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

#include <memory>

namespace hardware_interface
{
class RotorInterface : public JointCommandInterface
{
};
}  // namespace hardware_interface

namespace rotor_controllers
{
class RotorController : public controller_interface::Controller<hardware_interface::RotorInterface>
{
public:
  struct Commands
  {
    double force_ = 0.0;  // Last commanded force
  };

  RotorController();
  ~RotorController();

  bool init(hardware_interface::RotorInterface* robot, ros::NodeHandle& n);

  void setCommand(double force_target);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

  hardware_interface::JointHandle rotor_;
  urdf::JointConstSharedPtr joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;  // pre-allocated memory that is re-used to set the realtime buffer

private:
  int loop_count_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_;

  ros::Subscriber sub_command_;

  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
  void enforceJointLimits(double& command);
};
}  // namespace rotor_controllers
