#ifndef BASIC_CONTROLLER_SUPERIOR_CONTROLLER_INTERFACE_H
#define BASIC_CONTROLLER_SUPERIOR_CONTROLLER_INTERFACE_H

//* ros
#include <ros/ros.h>

//* serial comunication
#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <jsk_quadcopter/flight_navigation.h>
#include <jsk_quadcopter/flight_control.h>
#include <jsk_quadcopter/state_estimation.h>
#include "jsk_quadcopter/crc16.h"


class SerialInterface
{
 public:
  SerialInterface (std::string port, uint32_t speed, int every_byte_interval);
  ~SerialInterface ();

  //void output (char *output, int len);
  void output (unsigned char *output, int len);
  void sendPacket (const int& packet_type, Navigator * navigator, Estimator * estimator, FlightCtrlInput* flight_ctrl_input);

  bool getPacket (uint8_t* spacket, uint8_t* packet_type,  uint16_t* packet_size, ros::Time& packet_time_stamp);

  bool status;

  #if 0
  int *scan;
  int pt[800];
  int counter;
  #endif


  //** MACRO DEFINE
  //*** Sending Packet Type
  static const uint8_t FLIGHT_CONTROL = 0 ;
  //**** Control Packet Type
  static const uint8_t PD_START_COMMAND = 0x41;
  static const uint8_t PD_STOP_COMMAND= 0x42;
  static const uint8_t PD_CTRL_COMMAND= 0x43; 


 private:
  speed_t bitrate (int Bitrate);
  void txFlush ();
  void rxFlush ();
  void txRxFlush ();
  
  void drain ();

  //+*+*+: interval depends on the specification of the CPU
  int wait (int bytes_requested, float timeout = 0.01);

  int serialDev;
  std::string serialPortName;
  uint32_t serialPortSpeed;
  speed_t serialPortBaud;
  int everyByteInterval;

};


#endif

