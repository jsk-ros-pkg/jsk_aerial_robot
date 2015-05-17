#ifndef CONTROL_INPUT_ARRAY_H
#define CONTROL_INPUT_ARRAY_H

//* ros
#include <ros/ros.h>

class FlightCtrlInput
{
 public:
  FlightCtrlInput ();
  ~FlightCtrlInput ();


  void setPitchValue(short pitch_value);
  void setRollValue(short roll_value);
  void setYawValue(short yaw_value);
  void setThrottleValue(short throttle_value);
  void addPitchValue(short pitch_value);
  void addRollValue(short roll_value);
  void addYawValue(short yaw_value);
  void addThrottleValue(short throttle_value);


  short getPitchValue();
  short getRollValue();
  short getYawValue();
  short getThrottleValue();

  void setCtrlInputArray();
  uint8_t* getCtrlInputArray();
  int getCtrlInputSize();
    
 private:

  short pitch;
  //roll input: -2047..2047 (0=neutral)
  short roll;
  //R/C stick input: -2047..2047 (0=neutral)
  short yaw;
  //collective: 0..4095 = 0..100%
  short throttle;

  uint8_t* ctrlInputArray;

};

#endif
