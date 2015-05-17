#ifndef AERIAL_ROBOT_BASE_H
#define AERIAL_ROBOT_BASE_H

#include <ros/ros.h>
#include <aerial_robot_base/flight_control.h>
#include <aerial_robot_base/flight_navigation.h>
//#include <aerial_robot_base/tracking.h>
#include <aerial_robot_base/state_estimation.h>
#include <boost/thread.hpp>


class AerialRobotBase
{
 public : 
  AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~ AerialRobotBase();

  void rosParamInit(ros::NodeHandle nh);
  void rxFunction(const ros::TimerEvent & e);
  void txFunction(const ros::TimerEvent & e);
  //void tfPubFunction(const ros::TimerEvent & e);
  void tfPubFunction(); //for thread


 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  tx_timer_;
  ros::Timer  rx_timer_;

  double rx_loop_rate_;
  double tx_loop_rate_; //navigation + cntrol

  boost::thread tf_thread_;


  FlightCtrlInput* flight_ctrl_input_;
  PidController* controller_;
  RigidEstimator*  estimator_; 
  TeleopNavigator* navigator_;


  //*** tracking function
  /* bool trackingFlag_; */
  /*  Tracking* tracker;  */


  //*** simulation 
  bool simulation_flag_;
  //*** tf publish timer
  double tf_pub_loop_rate_;



};


#endif
