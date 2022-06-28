/*
******************************************************************************
* File Name          : gps_backend.h
* Description        : Basic class of gps node
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef GPS_BACKEND_H
#define GPS_BACKEND_H

#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <spinal/Gps.h>
#include <spinal/GpsFull.h>

#define GPS_BUFFER_SIZE 512

using namespace ap;

/// GPS status codes
enum GPS_Status {
  NO_GPS = 0,             ///< No GPS connected/detected
  NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
  GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
  GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
  GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
  GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
};

struct GPS_State {
  // all the following fields must all be filled by the backend driver
  uint8_t status;                  ///< driver fix status
  uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
  uint32_t utc_time;
  uint16_t utc_year;
  uint8_t utc_month;
  uint8_t utc_day;
  uint8_t utc_hour;
  uint8_t utc_min;
  uint8_t utc_sec;
  int32_t utc_nano;
  uint8_t utc_acc;
  uint8_t utc_valid;
  Location location;                  ///< last fix location
  float ground_speed;                 ///< ground speed in m/sec
  int32_t ground_course_cd;           ///< ground course in 100ths of a degree
  uint16_t hdop;                      ///< horizontal dilution of precision in cm
  uint16_t vdop;                      ///< vertical dilution of precision in cm
  uint8_t num_sats;                   ///< Number of visible satelites
  Vector3f velocity;                  ///< 3D velocitiy in m/s, in NED format
  float speed_accuracy;
  float horizontal_accuracy;
  float vertical_accuracy;
  bool have_vertical_velocity;      ///< does this GPS give vertical velocity? // Note: please do not use ":1" to specify 1 bit, which induces hard fault in RTOS 
  bool have_speed_accuracy;
  bool have_horizontal_accuracy;
  bool have_vertical_accuracy;
  uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
  bool mag_valid;
  float mag_dec;
};

struct GPS_timing {
  // the time we got our last fix in system milliseconds
  uint32_t last_fix_time_ms;
  // the time we got our last fix in system milliseconds
  uint32_t last_message_time_ms;
};

class GPS_Backend
{
public:
  GPS_Backend():
    gps_config_sub_("gps_config_cmd", &GPS_Backend::gpsConfigCallback, this),
    gps_pub_("gps", &gps_msg_)  // gps_full_pub_("gps_full", &gps_full_msg_)
  {
    state_.status = NO_FIX;
    state_.mag_valid = false;
  }

  virtual ~GPS_Backend(void) {}

  virtual void update() = 0;

  const GPS_State& getGpsState() { return state_; }
  const Location& location() const { return state_.location; }
  const Location& location(uint8_t instance) const { return state_.location; }
  const Vector3f& velocity() const { return state_.velocity; }  // 3D velocity in NED format
  float ground_speed() const { return state_.ground_speed; }  // ground speed in m/s
  uint32_t ground_speed_cm(void) { return ground_speed() * 100; }   // ground speed in cm/s
  int32_t ground_course_cd() const { return state_.ground_course_cd; }  // ground course in centidegrees
  uint8_t num_sats() const {  return state_.num_sats; }   // number of locked satellites
  uint32_t time_week_ms() const { return state_.time_week_ms; } // GPS time of week in milliseconds
  uint32_t last_fix_time_ms() const { return timing_.last_fix_time_ms; }
  uint32_t last_message_time_ms() const { return timing_.last_message_time_ms; }
  bool getMagValid() const { return state_.mag_valid; }
  float getMagDeclination() const { return state_.mag_dec; }

  void write(const uint8_t data_byte)
  {
    uint8_t data[1];
    data[0] = data_byte;
    HAL_UART_Transmit(huart_, data, 1, 100); //timeout: 100[ms]
  }

  void write(const uint8_t * data_byte, uint16_t size)
  {
    HAL_UART_Transmit(huart_, (uint8_t *)data_byte, size, 100); //timeout: 100[ms]
  }


protected:

  UART_HandleTypeDef *huart_;
  ros::NodeHandle* nh_;
  ros::Subscriber<std_msgs::UInt8, GPS_Backend> gps_config_sub_;
  ros::Publisher gps_pub_;
  spinal::Gps gps_msg_;
  // ros::Publisher gps_full_pub_;
  //spinal::GpsFull gps_full_msg_;

  GPS_State state_; ///< public state for this instance
  GPS_timing timing_;

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh);
  void gpsConfigCallback(const std_msgs::UInt8& config_msg){}

  virtual void publish();
  virtual void processMessage() = 0;

  bool available();
  int read();

};

#endif //GPS_BACKEND_H__
