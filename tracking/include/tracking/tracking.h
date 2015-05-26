/*
 1. nodelet lization => the topic betwen nodes.
 */


#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <tracking/tracker/checkerboard.h>
#include <tracking/tracker/camshift.h>

#include <sensor_msgs/Joy.h>
#include <aerial_robot_base/FlightNav.msg>



class Tracking
{
 public:
 Tracking(ros::NodeHandle nh,	ros::NodeHandle nh_private):nh_(nh), nh_private_(nh_private, "tracking")
    {
      navi_pub_ = nh_.advertise<aerial_robot_base::FlightNav>("flight_nav", 1); 

      full_state_sub_ = nh_.subscribe<aerial_robot_base::States>("full_states", 1, &CamShift::fullStatesCallback, this, ros::TransportHints().tcpNoDelay());

      tracking_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Tracking::trackingCallback, this, ros::TransportHints().tcpNoDelay());

      tracking_flag_ = false;

    }

  inline void setPosX(float pos_x) {pos_x_ = pos_x; }
  inline float getPosX() {return pos_x_; }
  inline void setVelX(float vel_x) {vel_x_ = vel_x; }
  inline float getVelX() {return vel_x_; }
  inline void setPosY(float pos_y) {pos_y_ = pos_y; }
  inline float getPosY() {return pos_y_; }
  inline void setVelY(float vel_y) {vel_y_ = vel_y; }
  inline float getVelY() {return vel_y_; }
  inline void setPosZ(float pos_z) {pos_z_ = pos_z; }
  inline float getPosZ() {return pos_z_; }
  inline void setVelZ(float vel_z) {vel_z_ = vel_z; }
  inline float getVelZ() {return vel_z_; }
  inline void setPsi(float psi) {psi_ = psi: }
  inline float getPsi() {return psi_; }
  inline void setVelPsi(float vel_psi) {vel_psi_ = vel_psi; }
  inline float getVelPsi() {return vel_psi_; }
  inline void setTheta(float theta) {theta_ = theta: }
  inline float getTheta() {return theta_; }
  inline void setVelTheta(float vel_theta) {vel_theta_ = vel_theta; }
  inline float getVelTheta() {return vel_theta_; }
  inline void setPhy(float phy) {phy_ = phy: }
  inline float getPhy() {return phy_; }
  inline void setVelPhy(float vel_phy) {vel_phy_ = vel_phy; }
  inline float getVelPhy() {return vel_phy_; }

  ~Tracking(){}

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber tracking_joy_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher  navi_pub_;

  bool tracking_flag_;

  float pos_x_;
  float vel_x_;
  float pos_y_;
  float vel_y_;
  float pos_z_;
  float vel_z_;
  float psi_;
  float vel_psi_;
  float phy_;
  float vel_phy_;
  float theta_;
  float vel_theta_;


  //plugin
  CheckerBoard* checker_board_tracker_;
  CamShift* cam_shift_tracker_;

  void trackingCallback(const sensor_msgs::JoyConstPtr &joy_msg)
  {
      if(joy_msg->buttons[12] == 1 && !tracking_flag_)
        {
          ROS_INFO("start tracking");
          tracking_flag_ = true;

          //trial
          cam_shift_tracker_ = new CamShift(nh_, nh_private_, this);
        }
      if(joy_msg->buttons[14] == 1 && tracking_flag_)
        {
          ROS_INFO("stop tracking");
          tracking_flag_ = false;

          //trial
          delete cam_shift_tracker_;
        }

  }

  void fullStatesCallback(const aerial_robot_base::StatesConstPtr& msg)
  {
    setPosX(msg->states[0].pos);
    setVelX(msg->states[0].vel);
    setPosY(msg->states[1].pos);
    setVelY(msg->states[1].vel);
    setPosZ(msg->states[2].pos);
    setVelZ(msg->states[2].vel);

    setPsi(msg->states[3].pos);
    setVelPsi(msg->states[3].vel);
    setTheta(msg->states[4].pos);
    setVelTheta(msg->states[4].vel);
    setPhy(msg->states[5].pos);
    setVelPhy(msg->states[5].vel);
  }

  void rosParamInit();

};


#endif


