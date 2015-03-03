#ifndef QUADCOPRER_H
#define QUADCOPRER_H

#include <ros/ros.h>
#include <jsk_quadcopter/flight_control.h>
#include <jsk_quadcopter/flight_navigation.h>
#include <jsk_quadcopter/tracking.h>
#include <jsk_quadcopter/basic_controller_superior_controller_interface_lev0.h>
#include <jsk_quadcopter/quadcopter_state_estimation.h>
#include <boost/thread.hpp>

class Quadcopter{
 public : 
  Quadcopter(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~Quadcopter();

  virtual void rxFunction(const ros::TimerEvent & e);
  virtual void txFunction(const ros::TimerEvent & e);

 protected:
  ros::NodeHandle quadcopterNodeHandle_;
  ros::NodeHandle quadcopterNodeHandlePrivate_;
  ros::Timer  txTimer_;
  ros::Timer  rxTimer_;

  double rxLoopRate_;
  double txLoopRate_; //navigation + cntrol


};

class JskQuadcopter : public Quadcopter
{
 public : 
  JskQuadcopter(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~ JskQuadcopter();

  void rosParamInit(ros::NodeHandle nh);
  void rxFunction(const ros::TimerEvent & e);
  void txFunction(const ros::TimerEvent & e);
  //void tfPubFunction(const ros::TimerEvent & e);
  void tfPubFunction(); //for thread

  const static int X = 0;
  const static int Y = 1;
  const static int Z = 2;
  const static int THETA = 3;
  const static int PHY = 4;
  const static int PSI = 5;

  //*** Sending Packet Type
  static const uint8_t FLIGHT_CONTROL = 0 ;

  //*** Receive Packet Type
  static const uint8_t PD_STATUS = 0x02;
  //static const uint8_t PD_IMUDATA = 0x03;
  static const uint8_t PD_CTRLINPUT = 0x11;
  static const uint8_t PD_ACK_RES = 0x40;
  static const uint8_t PD_IMUDATA = 0x50;

  /* const static int KDUINO_BOARD = 0; */
  /* const static int ASCTEC_BOARD = 1; */


  bool asctecGetSpecificPacket(Estimator1* estimator,
                               Navigator* navigator,
                               SerialInterface* serial_interface,
                               bool & polling_flag);


 private:
  /* ros::NodeHandle quadcopterNodeHandle_; */
  /* ros::NodeHandle   quadcopterNodeHandlePrivate_; */
  //ros::Timer  tfPubTimer_; //deprecated
  boost::thread tf_thread;
  
  PidController* controller;
  Estimator1*    estimator;
  SerialInterface* serialInterface;
  TeleopNavigator* navigator;
  FlightCtrlInput* flightCtrlInput;
  Tracking* tracker;


  //*** simulation 
  bool simulationFlag_;
  //*** tf publish timer
  double tfPubLoopRate_;

  //*** tracking function
  bool trackingFlag_;

  //*** asctec param (including serial comm without ros)
  bool asctecFlag_;
  bool asctecDataPollingFlag_; 
  std::string port_;
  int speedInt_;
  uint32_t speed_;
  int everyByteInterval_;

};


#endif
