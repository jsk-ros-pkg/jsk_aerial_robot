#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <gazebo_msgs/ModelStates.h> 
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <cmath>

class CollisionDetection
{
public:
  CollisionDetection();
  ~CollisionDetection() {}
  void CollisionCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  const double radius_drone = 0.47/2;
  const double radius_tree1 = 0.5;

//   // unit configurable parameters
//   int unit_rotor_num_; // rotor number per unit
//   double unit_mass_; // mass per unit
//   int units_num_; // nubmer of units
//   double m_f_rate_;
//   double max_thrust_;
//   double pos_bound_;
//   double fc_f_min_weight_, fc_t_min_weight_;

//   ros::Publisher visualize_pub_;
//   ros::Publisher feasible_control_info_pub_;
//   ros::Publisher feasible_control_info_pub_unit_;
};

int getIndex(std::vector<std::string> v, std::string value);
double distance_2d(geometry_msgs::Pose a, geometry_msgs::Pose b);