/*
******************************************************************************
* File Name          : gps_ublox.cpp
* Description        : Ublox interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/gps/gps_ublox.h"
#include "config.h"

namespace
{
  /* State machine state for parsing gps packet */
  uint8_t step_ = 0;
  uint8_t msg_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_counter_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t class_ = 0;

  bool start_receive_ = false;
  uint32_t start_delay = 5000; //start delay
  uint32_t init_time_;
  bool receive_packet_ = false;
  uint8_t packet_buf_[UBLOX_PACKET_MAX_SIZE];
}

GPS::GPS(): GPS_Backend()
{
}

void GPS::init(UART_HandleTypeDef *huart, ros::NodeHandle* nh)
{
  init_time_ = HAL_GetTick();
  GPS_Backend::init(huart, nh);
}

void GPS::update()
{
  if(!start_receive_)
  {
	  if(HAL_GetTick() - init_time_ > start_delay)
	  {
		  /* start usart revceive */
		  startReceive();
		  start_receive_ = true;
	  }
  }

  LED2_L;

  if(!receive_packet_) return;

  /* date copy */
  memcpy(raw_packet_.bytes, packet_buf_, UBLOX_PACKET_MAX_SIZE);

  switch (msg_id_)
    {
    case MSG_PVT:
      {
        LED2_H;

        /* https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)_Public.pdf */

        /* iTow: GPS time of week */
        state_.time_week_ms = raw_packet_.pvt.time;
        /* UTC */
        state_.utc_year = raw_packet_.pvt.year;
        state_.utc_month = raw_packet_.pvt.month;
        state_.utc_day = raw_packet_.pvt.day;
        state_.utc_hour = raw_packet_.pvt.hour;
        state_.utc_min = raw_packet_.pvt.min;
        state_.utc_sec = raw_packet_.pvt.sec;
        state_.utc_nano = raw_packet_.pvt.nano;
        uint8_t valid_time  = VALID_DATE | VALID_TIME | VALID_FULLY_RESOLVED;
        if((raw_packet_.pvt.valid & valid_time) == valid_time)
          state_.utc_valid = 1;
        else
          state_.utc_valid = 0;

        /* fix type and status */
        state_.status = raw_packet_.pvt.fix_type;

        /* location */
        state_.location.lng    = raw_packet_.pvt.longitude;
        state_.location.lat    = raw_packet_.pvt.latitude;
        state_.location.alt    = raw_packet_.pvt.altitude_msl / 10;
        state_.horizontal_accuracy = raw_packet_.pvt.horizontal_accuracy*1.0e-3f;
        state_.vertical_accuracy = raw_packet_.pvt.vertical_accuracy*1.0e-3f;
        state_.have_horizontal_accuracy = true;
        state_.have_vertical_accuracy = true;

        state_.ground_speed     = raw_packet_.pvt.speed_2d*0.001f;          // m/s
        state_.ground_course_cd = (raw_packet_.pvt.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
        state_.have_vertical_velocity = true;
        state_.velocity.x = raw_packet_.pvt.ned_north * 0.001f;
        state_.velocity.y = raw_packet_.pvt.ned_east * 0.001f;
        state_.velocity.z = raw_packet_.pvt.ned_down * 0.001f;
        state_.have_speed_accuracy = true;
        state_.speed_accuracy = raw_packet_.pvt.speed_accuracy*0.001f;
        state_.hdop        = raw_packet_.pvt.position_DOP;
        state_.num_sats    = raw_packet_.pvt.satellites;

        /* asynchronized update */
        update_ = true;

        break;
      }
    case MSG_POSLLH:
      {
        state_.location.lng    = raw_packet_.posllh.longitude;
        state_.location.lat    = raw_packet_.posllh.latitude;
        state_.location.alt    = raw_packet_.posllh.altitude_msl / 10;
        state_.horizontal_accuracy = raw_packet_.posllh.horizontal_accuracy*1.0e-3f;
        state_.vertical_accuracy = raw_packet_.posllh.vertical_accuracy*1.0e-3f;
        state_.have_horizontal_accuracy = true;
        state_.have_vertical_accuracy = true;

        break;
      }
    case MSG_VELNED:
      {
        state_.ground_speed     = raw_packet_.velned.speed_2d*0.01f;          // m/s
        state_.ground_course_cd = (raw_packet_.velned.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
        state_.have_vertical_velocity = true;
        state_.velocity.x = raw_packet_.velned.ned_east * 0.01f;
        state_.velocity.y = raw_packet_.velned.ned_east * 0.01f;
        state_.velocity.z = raw_packet_.velned.ned_down * 0.01f;
        state_.have_speed_accuracy = true;
        state_.speed_accuracy = raw_packet_.velned.speed_accuracy*0.01f;

        break;
      }
    default:
      {
        break;
      }
    }

  receive_packet_ = false;
}

void GPS::updateChecksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
  while (len--)
    {
      ck_a += *data;
      ck_b += ck_a;
      data++;
    }
}

void GPS::sendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size)
{
  struct ubx_header header;
  uint8_t ck_a=0, ck_b=0;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id    = msg_id;
  header.length    = size;

  updateChecksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
  updateChecksum((uint8_t *)msg, size, ck_a, ck_b);

  write((const uint8_t *)&header, sizeof(header));
  write((const uint8_t *)msg, size);
  write((const uint8_t *)&ck_a, 1);
  write((const uint8_t *)&ck_b, 1);
}

bool GPS::configureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  struct ubx_cfg_msg_rate msg;
  msg.msg_class = msg_class;
  msg.msg_id    = msg_id;
  msg.rate          = rate;
  sendMessage(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
  return true;
}

void GPS::configureRate(uint16_t rate)
{
  struct ubx_cfg_nav_rate msg;
  msg.measure_rate_ms = rate; // MEASURE_RATE;
  msg.nav_rate        = 1;
  msg.timeref         = 0;     // UTC time
  sendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

extern "C" void GPS_RxCpltCallback(UART_HandleTypeDef *huart)
{
  //LED0_L; //debug
  for(std::size_t i = 0; i < GPS_RX_SIZE; i++)
    {
      uint8_t data = (GPS_Backend::getRxBuf())[i];

      switch(step_)
        {
        case 1:
          if (PREAMBLE2 == data) {
            step_++;
            break;
          }
          step_ = 0;
          /* no break */
        case 0:
          if(PREAMBLE1 == data)
            {
              step_++;
            }
          break;
        case 2:
          step_++;
          class_ = data;
          ck_b_ = ck_a_ = data;  // reset the checksum accumulators
          break;
        case 3:
          step_++;
          ck_b_ += (ck_a_ += data);  // checksum byte
          msg_id_ = data;
          break;
        case 4:
          step_++;
          ck_b_ += (ck_a_ += data);  // checksum byte
          payload_length_ = data;   // payload length low byte
          break;
        case 5:
          step_++;
          ck_b_ += (ck_a_ += data);   // checksum byte

          payload_length_ += (uint16_t)(data<<8);
          if (payload_length_ > sizeof(packet_buf_))
            {
              // assume any payload bigger then what we know about is noise
              payload_length_ = 0;
              step_ = 0;
            }
          payload_counter_ = 0; // prepare to receive payload
          break;
        case 6: /* Receive message data */
          ck_b_ += (ck_a_ += data); // checksum byte
          if (payload_counter_ < sizeof(packet_buf_))
            packet_buf_[payload_counter_] = data;

          if (++payload_counter_ == payload_length_) step_++;
          break;
        case 7:     /* Checksum and message processing */
          step_++;
          if (ck_a_ != data) {
            step_ = 0;
          }
          break;
        case 8:
          step_ = 0;
          if (ck_b_ != data) {
            break; // bad checksum
          }
          /* receive some package */
          receive_packet_= true;
          //LED0_H; //debug
          break;
        }
    }

  HAL_UART_Receive_IT(huart, GPS_Backend::getRxBuf() , GPS_RX_SIZE);
}

