#include <aerial_robot_simulation/rotor_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace rotor_controllers
{

RotorController::RotorController() : loop_count_(0)
{
}

RotorController::~RotorController()
{
  sub_command_.shutdown();
}

bool RotorController::init(hardware_interface::RotorInterface* robot, ros::NodeHandle& n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Start realtime state publisher
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &RotorController::setCommandCB, this);

  // Get joint handle from hardware interface
  rotor_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void RotorController::starting(const ros::Time& time)
{
  command_struct_.force_ = 0.0;
  command_.initRT(command_struct_);
}

void RotorController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_force = command_struct_.force_;

  enforceJointLimits(command_force);

  rotor_.setCommand(command_force);

  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.command = command_force;

      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void RotorController::setCommand(double target_force)
{
  command_struct_.force_ = target_force;

  command_.writeFromNonRT(command_struct_);
}

void RotorController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

void RotorController::enforceJointLimits(double& command)
{
  if (command > joint_urdf_->limits->upper)  // above upper limit
  {
    command = joint_urdf_->limits->upper;
  }
  else if (command < joint_urdf_->limits->lower)  // below lower limit
  {
    command = joint_urdf_->limits->lower;
  }
}
}  // namespace rotor_controllers

PLUGINLIB_EXPORT_CLASS(rotor_controllers::RotorController, controller_interface::ControllerBase)
