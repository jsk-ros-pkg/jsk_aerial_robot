/*
 1. nodelet lization => the topic betwen nodes.
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <aerial_tracking/basic_tracking.h>
//#include <tracking/tracker/6dof.h>
#include <aerial_tracking/tracker/bouding_box.h>
#include <aerial_robot_msgs/KduinoImu.h>

class Tracking: public BasicTracking
{
 public:
 Tracking(ros::NodeHandle nh,	ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private, "tracking")
    {
      navi_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("flight_nav", 1); 

      full_state_sub_ = nh_.subscribe<aerial_robot_base::States>("/estimator/full_states", 1, &Tracking::fullStatesCallback, this, ros::TransportHints().tcpNoDelay());

      tracking_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Tracking::trackingCallback, this, ros::TransportHints().tcpNoDelay());

      imu_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoImu>("kduino/imu", 1, &ImuData::AttitudeCallback, this, ros::TransportHints().tcpNoDelay()); 

      tracking_flag_ = false;

    }

  void navigation(aerial_robot_base::FlightNav navi_command)
    {
      navi_pub_.publish(navi_command);
    }

  ~Tracking(){}

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber tracking_joy_sub_;
  ros::Subscriber full_state_sub_;
  ros::Subscriber imu_sub_
  ros::Publisher  navi_pub_;

  bool tracking_flag_;

  //plugin
  //CheckerBoard* checker_board_tracker_;
  BoundingBox* bounding_box_tracker_;

  void trackingCallback(const sensor_msgs::JoyConstPtr &joy_msg)
  {
      if(joy_msg->buttons[12] == 1 && !tracking_flag_)
        {
          ROS_INFO("start tracking");
          tracking_flag_ = true;

          //trial
          bounding_shift_tracker_ = new CamShift(nh_, nh_private_, this);
        }
      if(joy_msg->buttons[14] == 1 && tracking_flag_)
        {
          ROS_INFO("stop tracking");
          tracking_flag_ = false;

          //trial
          delete cam_shift_tracker_;
        }

  }


  void AttitudeCallback(const aerial_robot_msgs::KduinoImuConstPtr& msg)
  {

    float roll_  = M_PI * imu_msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
    float pitch_ = M_PI * imu_msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
    float yaw_   = M_PI * imu_msg->angle[2] / 180.0;

    setPsi(roll_);
    setTheta(pitch_);
    setPhy(yaw_);
  }

  void fullStatesCallback(const aerial_robot_base::StatesConstPtr& msg)
  {
    setPosX(msg->states[0].pos);
    setVelX(msg->states[0].vel);
    setPosY(msg->states[1].pos);
    setVelY(msg->states[1].vel);
    setPosZ(msg->states[2].pos);
    setVelZ(msg->states[2].vel);
  }

  void rosParamInit();

};


#endif


