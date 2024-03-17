#pragma once

#include <aerial_robot_model/model/pinocchio_robot_model.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace aerial_robot_model {
  class PinocchioRobotModelRos {
  public:
    PinocchioRobotModelRos(ros::NodeHandle nh, ros::NodeHandle nhp);
    virtual ~PinocchioRobotModelRos() = default;
    boost::shared_ptr<aerial_robot_model::PinocchioRobotModel> getPinocchioRobotModel() {return pinocchio_robot_model_;}

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber joint_state_sub_;

    pluginlib::ClassLoader<aerial_robot_model::PinocchioRobotModel> pinocchio_robot_model_loader_;
    boost::shared_ptr<aerial_robot_model::PinocchioRobotModel> pinocchio_robot_model_;

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  };

}
