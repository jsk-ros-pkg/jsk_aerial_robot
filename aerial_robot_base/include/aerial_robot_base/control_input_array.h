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

  inline  void setPitchValue(float pitch_value){  pitch = pitch_value;}
  inline  void setRollValue(float roll_value){  roll = roll_value; }
  inline  void setYawValue(float yaw_value){  yaw = yaw_value; }
  inline  void setThrottleValue(uint16_t throttle_value){  throttle = throttle_value;}

  inline  void addPitchValue(float pitch_value){  pitch += pitch_value;}
  inline  void addRollValue(float roll_value){  roll += roll_value;}
  inline  void addYawValue(float yaw_value){  yaw += yaw_value;}
  inline  void addThrottleValue(uint16_t throttle_value){  throttle += throttle_value;}

  inline  float getPitchValue(){ return pitch;}
  inline  float getRollValue(){ return roll;}
  inline  float getYawValue(){ return yaw;}
  inline  uint16_t getThrottleValue(){ return throttle;}

   
 private:

  float pitch;
  float roll;
  float yaw;
  uint16_t throttle;


};

#endif
