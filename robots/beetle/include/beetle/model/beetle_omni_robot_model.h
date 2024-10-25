// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <algorithm>
using namespace aerial_robot_model;

class BeetleOmniRobotModel : public GimbalrotorRobotModel
{
public:
  BeetleOmniRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_t_min_thre = 0,
                       double epsilon = 10)
    : GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon) {};
  ~BeetleOmniRobotModel() override = default;

private:
  ros::NodeHandle nh_;

protected:
};