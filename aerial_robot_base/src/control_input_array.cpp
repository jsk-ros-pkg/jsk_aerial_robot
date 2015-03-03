#include "jsk_quadcopter/control_input_array.h"

FlightCtrlInput::FlightCtrlInput()
{
  pitch = 0;
  roll = 0;
  yaw = 0;
  throttle = 0;

  ctrlInputArray = (uint8_t*)malloc(4 * sizeof(short));
}

FlightCtrlInput::~FlightCtrlInput()
{
  printf(" deleted flight control input!\n");
  // delete ctrlInputArray;
}

void FlightCtrlInput::setPitchValue(short pitch_value)
{
  pitch = pitch_value;
}

void FlightCtrlInput::setRollValue(short roll_value)
{
  roll = roll_value;
}

void FlightCtrlInput::setYawValue(short yaw_value)
{
  yaw = yaw_value;
}

void FlightCtrlInput::setThrottleValue(short throttle_value)
{
  throttle = throttle_value;
}

void FlightCtrlInput::addPitchValue(short pitch_value)
{
  pitch += pitch_value;
}

void FlightCtrlInput::addRollValue(short roll_value)
{
  roll += roll_value;
}

void FlightCtrlInput::addYawValue(short yaw_value)
{
  yaw += yaw_value;
}

void FlightCtrlInput::addThrottleValue(short throttle_value)
{
  throttle += throttle_value;
}


short FlightCtrlInput::getPitchValue()
{
  return pitch;
}

short FlightCtrlInput::getRollValue()
{
  return roll;
}

short FlightCtrlInput::getYawValue()
{
  return yaw;
}

short FlightCtrlInput::getThrottleValue()
{
  return throttle;
}

void  FlightCtrlInput::setCtrlInputArray()
{
  //Big Edian
  //TODO 8-> future work
  ctrlInputArray[0] = (uint8_t)(0xff & (pitch >> 8));
  ctrlInputArray[1] = (uint8_t)(0xff & pitch);
  ctrlInputArray[2] = (uint8_t)(0xff & (roll >> 8));
  ctrlInputArray[3] = (uint8_t)(0xff & roll);
  ctrlInputArray[4] = (uint8_t)(0xff & (yaw >> 8));
  ctrlInputArray[5] = (uint8_t)(0xff & yaw);
  ctrlInputArray[6] = (uint8_t)(0xff & (throttle >> 8));
  ctrlInputArray[7] = (uint8_t)(0xff & throttle);

}

uint8_t*  FlightCtrlInput::getCtrlInputArray()
{
  //Big Edian
  //TODO 8-> future work
  ctrlInputArray[0] = (uint8_t)(0xff & (pitch >> 8));
  ctrlInputArray[1] = (uint8_t)(0xff & pitch);
  ctrlInputArray[2] = (uint8_t)(0xff & (roll >> 8));
  ctrlInputArray[3] = (uint8_t)(0xff & roll);
  ctrlInputArray[4] = (uint8_t)(0xff & (yaw >> 8));
  ctrlInputArray[5] = (uint8_t)(0xff & yaw);
  ctrlInputArray[6] = (uint8_t)(0xff & (throttle >> 8));
  ctrlInputArray[7] = (uint8_t)(0xff & throttle);


  return ctrlInputArray;
}

int FlightCtrlInput::getCtrlInputSize()
{
  int ctrlInputSize = sizeof(pitch) + sizeof(roll) + sizeof(yaw) + sizeof(throttle);

  return ctrlInputSize;
}




