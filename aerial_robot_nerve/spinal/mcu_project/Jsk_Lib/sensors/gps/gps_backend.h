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

enum GPS_Engine_Setting {
  GPS_ENGINE_NONE        = -1,
  GPS_ENGINE_PORTABLE    = 0,
  GPS_ENGINE_STATIONARY  = 2,
  GPS_ENGINE_PEDESTRIAN  = 3,
  GPS_ENGINE_AUTOMOTIVE  = 4,
  GPS_ENGINE_SEA         = 5,
  GPS_ENGINE_AIRBORNE_1G = 6,
  GPS_ENGINE_AIRBORNE_2G = 7,
  GPS_ENGINE_AIRBORNE_4G = 8
};

enum GPS_Config {
  GPS_ALL_CONFIGURED = 255
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

static uint32_t millis(){ return HAL_GetTick();}

class GPS_Backend
{
public:
  GPS_Backend(): gps_config_sub_("/gps_config_cmd", &GPS_Backend::gpsConfigCallback, this )
  {}

  virtual ~GPS_Backend(void) {}

  virtual bool read(uint8_t data) = 0;
  virtual void update() = 0;

  virtual bool is_configured(void) { return true; }
  virtual void inject_data(uint8_t *data, uint8_t len) { return; }

  GPS_State getGpsState()
  {
    return state;
  }

  // location of last fix
  const Location &location() const {
    return state.location;
  }

  const Location &location(uint8_t instance) const {
    return state.location;
  }

  bool speed_accuracy(float &sacc) const {
    if(state.have_speed_accuracy) {
      sacc = state.speed_accuracy;
      return true;
    }
    return false;
  }

  bool horizontal_accuracy(float &hacc) const {
    if(state.have_horizontal_accuracy) {
      hacc = state.horizontal_accuracy;
      return true;
    }
    return false;
  }

  bool vertical_accuracy(float &vacc) const {
    if(state.have_vertical_accuracy) {
      vacc = state.vertical_accuracy;
      return true;
    }
    return false;
  }

  // 3D velocity in NED format
  const Vector3f &velocity() const {
    return state.velocity;
  }

  // ground speed in m/s
  float ground_speed() const {
    return state.ground_speed;
  }

  // ground speed in cm/s
  uint32_t ground_speed_cm(void) {
    return ground_speed() * 100;
  }

  // ground course in centidegrees
  int32_t ground_course_cd() const {
    return state.ground_course_cd;
  }

  // number of locked satellites
  uint8_t num_sats() const {
    return state.num_sats;
  }

  // GPS time of week in milliseconds
  uint32_t time_week_ms() const {
    return state.time_week_ms;
  }

  // GPS week
  uint16_t time_week() const {
    return state.time_week;
  }

  // horizontal dilution of precision
  uint16_t get_hdop() const {
    return state.hdop;
  }

  // vertical dilution of precision
  uint16_t get_vdop() const {
    return state.vdop;
  }

  // the time we got our last fix in system milliseconds. This is
  // used when calculating how far we might have moved since that fix
  uint32_t last_fix_time_ms() const {
    return _timing.last_fix_time_ms;
  }

  // the time we last processed a message in milliseconds. This is
  // used to indicate that we have new GPS data to process
  uint32_t last_message_time_ms() const {
    return _timing.last_message_time_ms;
  }

  // return last fix time since the 1/1/1970 in microseconds
  uint64_t time_epoch_usec()
  {
    const GPS_State &istate = state;
    if (istate.last_gps_time_ms == 0) {
      return 0;
    }
    const uint64_t ms_per_week = 7000ULL*86400ULL;
    const uint64_t unix_offset = 17000ULL*86400ULL + 52*10*7000ULL*86400ULL - 15000ULL;
    uint64_t fix_time_ms = unix_offset + istate.time_week*ms_per_week + istate.time_week_ms;
    // add in the milliseconds since the last fix
    return (fix_time_ms + (millis() - istate.last_gps_time_ms)) * 1000ULL;
  }

  // return true if the GPS supports vertical velocity values
  bool have_vertical_velocity() const { 
    return state.have_vertical_velocity; 
  }

  // the expected lag (in seconds) in the position and velocity readings from the gps
  float get_lag() const { return 0.2f; }


  //UART Part
  uint16_t available()
  {
    return gps_rx_buf_.length();
  }

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

  
  bool pop(uint8_t& pop_value)
  {
    bool is_valid = gps_rx_buf_.pop(pop_value);
    return is_valid;
  }

  /*
    void add(uint8_t new_value)
    {
    rx_buf_.push(new_value);
    }

    bool add()
    {
    for(std::size_t i = 0; i < GPS_RX_SIZE; i++)
    {
    if(!rx_buf_.push(rx_value_[i])) 
    {
    return false;
    }
    }
    return true;
    }
  */

  UART_HandleTypeDef* getHuart()
  {
    return huart_;
  }

  uint8_t* getRxPointer()
  {
    return gps_rx_value_;
  }
  uint16_t getRxSize()
  {
    return (uint16_t)GPS_RX_SIZE;
  }

  bool getUpdate() { return update_; }
  void setUpdate(bool update) { update_ = update; }

  //uart rx part
  static RingBufferFiFo<uint8_t, RING_BUFFER_SIZE> gps_rx_buf_;
  static uint8_t gps_rx_value_[GPS_RX_SIZE];
  static uint16_t gps_rx_size_;

protected:

  UART_HandleTypeDef *huart_;
  ros::NodeHandle* nh_;
  ros::Subscriber<std_msgs::UInt8, GPS_Backend> gps_config_sub_;

  GPS_State state;           ///< public state for this instance
  GPS_timing _timing;

  int8_t _type;
  int8_t _navfilter;
  int8_t _auto_switch;
  int8_t _min_dgps;
  int16_t _sbp_logmask;
  uint32_t _last_instance_swap_ms;
  int8_t _sbas_mode;
  int8_t _min_elevation;
  int8_t _raw_data;
  int8_t _gnss_mode;

  bool update_;

  void baseInit (UART_HandleTypeDef* huart, ros::NodeHandle* nh)
  {
    huart_ =huart;
    nh_ = nh;
    //nh_->subscribe<std_msgs::UInt8, GPS_Backend>(gps_config_sub_);
    nh_->subscribe< ros::Subscriber<std_msgs::UInt8, GPS_Backend> >(gps_config_sub_);


    _type = 1;
    _gnss_mode = 0;

    _last_instance_swap_ms = 0;
    _auto_switch = 1;
    _min_dgps = 100;
    _sbas_mode = 2;
    _min_elevation = -100;
    _raw_data = 0;

    _navfilter = GPS_ENGINE_AIRBORNE_4G;
    _sbp_logmask = 0xFF00;

    state.status = NO_FIX;

    state.have_speed_accuracy = false;
    state.have_horizontal_accuracy = false;
    state.have_vertical_accuracy = false;  

    //UART part
    gps_rx_buf_.init();
    gps_rx_size_ = GPS_RX_SIZE;

    update_ = false;
  }

  // common utility functions
  int32_t swap_int32(int32_t v) const
  {
    const uint8_t *b = (const uint8_t *)&v;
    union {
      int32_t v;
      uint8_t b[4];
    } u;

    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];

    return u.v;
  }
  int16_t swap_int16(int16_t v) const
  {
    const uint8_t *b = (const uint8_t *)&v;
    union {
      int16_t v;
      uint8_t b[2];
    } u;

    u.b[0] = b[1];
    u.b[1] = b[0];

    return u.v;
  }

  void fill_3d_velocity(void)
  {
    float gps_heading = (state.ground_course_cd * 0.01f) * 3.1415926f / 180.0f;

    state.velocity.x = state.ground_speed * cosf(gps_heading);
    state.velocity.y = state.ground_speed * sinf(gps_heading);
    state.velocity.z = 0;
    state.have_vertical_velocity = false;
  }

  void gpsConfigCallback(const std_msgs::UInt8& config_msg)
  {
    //TODO
  }

};

#endif //GPS_BACKEND_H__
