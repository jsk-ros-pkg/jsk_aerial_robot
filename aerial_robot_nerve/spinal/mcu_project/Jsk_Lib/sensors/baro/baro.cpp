/*
******************************************************************************
* File Name          : baro.cpp
* Description        : Baro Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif


#include "sensors/baro/baro.h"
#include <utility>

/*
  Baro constructor
*/
BaroBackend::BaroBackend() :
  pressure_(0.0f),
  base_pressure_(0.0f),
  temperature_(0.0f),
  altitude_(0.0f),
  alt_offset_(0.0f),
  alt_offset_active_(0.0f),
  last_altitude_EAS2TAS_(0.0f),
  EAS2TAS_(0.0f)
{
  calibrate_count_ = CALIBRATE_COUNT;
  calibrated_ = false;
}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float BaroBackend::getAltitudeDifference(float base_pressure, float pressure) const
{
  float ret;
  float temp    = getTemperature() + 273.15f;
  float scaling = pressure / base_pressure;

  // This is an exact calculation that is within +-2.5m of the standard
  // atmosphere tables in the troposphere (up to 11,000 m amsl).
  ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

  return ret;
}


// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
float BaroBackend::getEAS2TAS(void)
{
  float altitude = getAltitude();
  if ((fabsf(altitude - last_altitude_EAS2TAS_) < 100.0f) && EAS2TAS_  != 0) {
    // not enough change to require re-calculating
    return EAS2TAS_;
  }

  float limited_temp = (getTemperature() > 25)? 25: getTemperature();

  float tempK = limited_temp + 273.15f - 0.0065f * altitude;
  EAS2TAS_ = sqrtf(1.225f / ((float)getPressure() / (287.26f * tempK)));
  last_altitude_EAS2TAS_ = altitude;
  return EAS2TAS_;
}

// return air density / sea level density - decreases as altitude climbs
float BaroBackend::getAirDensityRatio(void)
{
  float eas2tas = getEAS2TAS();
  if (eas2tas > 0.0f) {
    return 1.0f/(sqrtf(getEAS2TAS()));
  } else {
    return 1.0f;
  }
}

/*
  call update on all drivers
*/
void BaroBackend::baseUpdate(void)
{
  if (fabsf(alt_offset_ - alt_offset_active_) > 0.1f) {
    // if there's more than 10cm difference then slowly slew to it via LPF.
    // The EKF does not like step inputs so this keeps it happy
    alt_offset_active_ = (0.9f*alt_offset_active_) + (0.1f*alt_offset_);
  } else {
    alt_offset_active_ = alt_offset_;
  }

  // update altitude calculation
  float altitude = getAltitudeDifference(base_pressure_, pressure_);
  // sanity check altitude
  altitude_ = altitude + alt_offset_active_;

  update_ = true;
}

