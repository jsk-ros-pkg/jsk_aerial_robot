/*
 1. nodelet lization => the topic betwen nodes.
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <aerial_tracking/basic_tracking.h>
#include <sensor_msgs/Joy.h>
//#include <tracking/tracker/6dof.h>
#include <aerial_tracking/tracker/bouding_box.h>
#include <aerial_robot_msgs/KduinoImu.h>

class Tracking: public BasicTracking
{
 public:
 Tracking(ros::NodeHandle nh,	ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private, "tracking")
    {
      navi_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("flight_nav", 1); 

      stop_teleop_pub_ = nh_.advertise<std_msgs::UInt8>("stop_teleop", 1); 

      full_state_sub_ = nh_.subscribe<aerial_robot_base::States>("/estimator/full_states", 1, &Tracking::fullStatesCallback, this, ros::TransportHints().tcpNoDelay());

      tracking_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Tracking::trackingCallback, this, ros::TransportHints().tcpNoDelay());

      imu_sub_ = nh_.subscribe<aerial_robot_msgs::KduinoImu>("kduino/imu", 1, &Tracking::AttitudeCallback, this, ros::TransportHints().tcpNoDelay()); 

      start_tracking_pub_ = nh_.advertise<std_msgs::UInt8>("/start_tracking", 1);

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
  ros::Subscriber imu_sub_;
  ros::Publisher  navi_pub_;
  ros::Publisher  stop_teleop_pub_;
  ros::Publisher start_tracking_pub_;

  bool tracking_flag_;

  //plugin
  //CheckerBoard* checker_board_tracker_;
  BoundingBox* bounding_box_tracker_;

  void trackingCallback(const sensor_msgs::JoyConstPtr &joy_msg)
  {
      if(joy_msg->buttons[12] == 1 && !tracking_flag_)
        {//triangle
          ROS_INFO("start tracking");
          tracking_flag_ = true;

          //send to base
          std_msgs::UInt8 stop_msg;
          stop_msg.data = 1;
          stop_teleop_pub_.publish(stop_msg);

          //trial
          bounding_box_tracker_ = new BoundingBox(nh_, nh_private_, this);

          //trial, start tracking
          startTracking();

        }
      if(joy_msg->buttons[14] == 1 && tracking_flag_)
        {//cross
          ROS_INFO("stop tracking");
          tracking_flag_ = false;

          //send to base
          std_msgs::UInt8 stop_msg;
          stop_msg.data = 0;
          stop_teleop_pub_.publish(stop_msg);


          //trial
          delete bounding_box_tracker_;
        }

  }


  void AttitudeCallback(const aerial_robot_msgs::KduinoImuConstPtr& msg)
  {

    float roll_  = M_PI * msg->angle[0] / 10.0 / 180.0; //raw data is 10 times
    float pitch_ = M_PI * msg->angle[1] / 10.0 / 180.0; //raw data is 10 times
    float yaw_   = M_PI * msg->angle[2] / 180.0;

    setPsi(yaw_);
    setTheta(pitch_);
    setPhy(roll_);
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

  void startTracking()
  {
    std_msgs::UInt8 start_msg;
    start_msg.data = 1;
    start_tracking_pub_.publish(start_msg);

    /* image_processing::StartTracking srv; */
    /* srv.request.tracking_req = true; */
    /* if(start_tracking_client_.call(srv)) */
    /*   { */
    /*     if(srv.response.tracking_res) */
    /*       ROS_INFO("start tracking from aerial tracker"); */
    /*   } */
    /* else */
    /*   { */
    /*     ROS_ERROR("Filaed to call service"); */
    /*   } */
  }



};


#endif


