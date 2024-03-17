#include <aerial_robot_model/model/pinocchio_robot_model_ros.h>

namespace aerial_robot_model {
  PinocchioRobotModelRos::PinocchioRobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh),
    nhp_(nhp),
    pinocchio_robot_model_loader_("aerial_robot_model", "aerial_robot_model::PinocchioRobotModel")
  {
    // load robot model plugin
    std::string plugin_name;
    if(nh_.getParam("pinocchio_robot_model_plugin_name", plugin_name))
      {
        try
          {
            pinocchio_robot_model_ = pinocchio_robot_model_loader_.createInstance(plugin_name);
          }
        catch(pluginlib::PluginlibException& ex)
          {
            ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
          }
      }
    else
      {
        ROS_ERROR("can not find plugin rosparameter for robot model, use default class: aerial_robot_model::PinocchioRobotModel");
        pinocchio_robot_model_ = boost::make_shared<aerial_robot_model::PinocchioRobotModel>();
      }


    joint_state_sub_ = nh_.subscribe("joint_states", 1, &PinocchioRobotModelRos::jointStateCallback, this);
  }

  void PinocchioRobotModelRos::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    pinocchio_robot_model_->updateRobotModel(*msg);
  }
}
