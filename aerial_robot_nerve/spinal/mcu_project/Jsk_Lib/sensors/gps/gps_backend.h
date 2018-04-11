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

#include "stm32f7xx_hal.h"
#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>

#define GPS_RX_SIZE 4 //match the dma rule(circuler mode=> MBurst Value)
#define RING_BUFFER_SIZE 400

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
  uint16_t time_week;               ///< GPS week number
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
  bool have_vertical_velocity:1;      ///< does this GPS give vertical velocity?
  bool have_speed_accuracy:1;
  bool have_horizontal_accuracy:1;
  bool have_vertical_accuracy:1;
  uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
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
    gps_config_sub_("/gps_config_cmd", &GPS_Backend::gpsConfigCallback, this),
    update_(false)
  {
    state_.status = NO_FIX;
  }

  virtual ~GPS_Backend(void) {}

  virtual void update() = 0;

  GPS_State getGpsState() { return state_; }
  const Location &location() const { return state_.location; }
  const Location &location(uint8_t instance) const { return state_.location; }
  const Vector3f &velocity() const { return state_.velocity; }  // 3D velocity in NED format
  float ground_speed() const { return state_.ground_speed; }  // ground speed in m/s
  uint32_t ground_speed_cm(void) { return ground_speed() * 100; }   // ground speed in cm/s
  int32_t ground_course_cd() const { return state_.ground_course_cd; }  // ground course in centidegrees
  uint8_t num_sats() const {  return state_.num_sats; }   // number of locked satellites
  uint32_t time_week_ms() const { return state_.time_week_ms; } // GPS time of week in milliseconds
  uint16_t time_week() const { return state_.time_week; }
  uint32_t last_fix_time_ms() const { return timing_.last_fix_time_ms; }
  uint32_t last_message_time_ms() const { return timing_.last_message_time_ms; }

  void startReceiveDMA();
  uint16_t available();

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

  bool pop(uint8_t& pop_value);
  uint8_t* getRxPointer();

  UART_HandleTypeDef* getHuart() { return huart_; }
  uint16_t getRxSize() { return (uint16_t)GPS_RX_SIZE; }

  bool getUpdate() { return update_; }
  void setUpdate(bool update) { update_ = update; }

  static void UBLOX_UART_DMAReceiveCpltUBLOX(DMA_HandleTypeDef *hdma);

protected:

  UART_HandleTypeDef *huart_;
  ros::NodeHandle* nh_;
  ros::Subscriber<std_msgs::UInt8, GPS_Backend> gps_config_sub_;

  GPS_State state_; ///< public state for this instance
  GPS_timing timing_;
  bool update_;

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh);
  virtual bool parsePacket() = 0;
  virtual bool read(uint8_t data) = 0;

  void gpsConfigCallback(const std_msgs::UInt8& config_msg)
  {
    //TODO
  }

};

#endif //GPS_BACKEND_H__
