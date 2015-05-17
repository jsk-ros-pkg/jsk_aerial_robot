#ifndef CONTROL_INPUT_ARRAY_H
#define CONTROL_INPUT_ARRAY_H

//* ros
#include <ros/ros.h>

class FlightCtrlInput
{
 public:
  FlightCtrlInput ()
{
  pitch = 0;
  roll = 0;
  yaw = 0;
  throttle = 0;

}

  ~FlightCtrlInput (){ }


  inline  void setPitchValue(short pitch_value){  pitch = pitch_value;}
  inline  void setRollValue(short roll_value){  roll = roll_value; }
  inline  void setYawValue(short yaw_value){  yaw = yaw_value; }
  inline  void setThrottleValue(short throttle_value){  throttle = throttle_value;}

  inline  void addPitchValue(short pitch_value){  pitch += pitch_value;}
  inline  void addRollValue(short roll_value){  roll += roll_value;}
  inline  void addYawValue(short yaw_value){  yaw += yaw_value;}
  inline  void addThrottleValue(short throttle_value){  throttle += throttle_value;}

  inline  short getPitchValue(){ return pitch;}
  inline  short getRollValue(){ return roll;}
  inline  short getYawValue(){ return yaw;}
  inline  short getThrottleValue(){ return throttle;}


    
 private:

  short pitch;
  //roll input: -2047..2047 (0=neutral)
  short roll;
  //R/C stick input: -2047..2047 (0=neutral)
  short yaw;
  //collective: 0..4095 = 0..100%
  short throttle;


};

#endif
