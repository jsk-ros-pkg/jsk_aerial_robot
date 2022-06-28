/*
******************************************************************************
* File Name          : baro.h
* Description        : Baro Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __BARO_H
#define __BARO_H

#include <stdint.h>
#include <utility>
#include "config.h"
#include <math.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

class BaroBackend
{

public:
  // constructor
  BaroBackend();
  ~BaroBackend(){}

  static const uint16_t CALIBRATE_COUNT =100;// 500;

  // initialise the barometer object, loading backend drivers
  //virtual void init(void);

  // update the barometer object, asking backends to push data to
  // the frontend
  void baseUpdate(void);
  // pressure in Pascal. Divide by 100 for millibars or hectopascals
  float getPressure(void) const { return pressure_; }

  // temperature in degrees C
  float getTemperature(void) const { return temperature_; }

  // get current altitude in meters relative to altitude at the time
  float getAltitude(void) const { return altitude_; }

  void setAltOffset(float alt_offset) {alt_offset_ = alt_offset;}

  // get altitude difference in meters relative given a base
  // pressure in Pascal
  float getAltitudeDifference(float base_pressure, float pressure) const;

  // get scale factor required to convert equivalent to true airspeed
  float getEAS2TAS(void);

  // get air density / sea level density - decreases as altitude climbs
  float getAirDensityRatio(void);

  // get current climb rate in meters/s. A positive number means
  float getClimbRate(void);
  // get last time sample was taken (in ms)
  uint32_t getLastUpdate(void) const { return last_timer_; }

  bool getUpdate() { return update_; }
  void setUpdate(bool update) { update_ = update; }

  inline float getPressure(){return pressure_;}
  inline float getTemperature(){return temperature_;}
  inline float getAltitude(){return altitude_;}

protected:

  float pressure_;                 // pressure in Pascal
  float base_pressure_;                 // pressure in Pascal
  float temperature_;              // temperature in degrees C
  float altitude_;                // calculated altitude
  bool calibrated_;
  uint16_t calibrate_count_;

  float alt_offset_;
  float alt_offset_active_;
  float last_altitude_EAS2TAS_;
  float EAS2TAS_;
  uint32_t last_timer_;

  bool update_;

  I2C_HandleTypeDef* i2c_handle_;


  void SimpleAtmosphere(const float alt, float &sigma, float &delta, float &theta);
};

#endif
