/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, UT.
 * All rights reserved.
 *
  */

#ifndef ROS_STMH7_HARDWARE_H_
#define ROS_STMH7_HARDWARE_H_

#include "stm32h7xx_hal.h"
#include "lwip.h"
#include "lwip/udp.h"
#include <cstring>

#define RX_BUFFER_SIZE 512 // byte

template <typename T,  int SIZE>
class RingBuffer
{
public:
  RingBuffer()
  {
    byte_to_pop_ = 0;
    byte_to_push_ = 0;
    buffer_length_ = (uint16_t)SIZE;

    lock_ = false;
  }
  ~RingBuffer(){  }

  bool pop(T* pop_buf, uint16_t len = 1)
  {
    if (byte_to_pop_ + len <= byte_to_push_)
      {
        std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * len);
        lock_ = true;
        {
          byte_to_pop_ += len;
        }
        lock_ = false;

        return true;
      }
    else
      {
        if(byte_to_pop_ <= byte_to_push_ || byte_to_pop_ + len > byte_to_push_ + buffer_length_)
          {
            return false;
          }
        else
          {
            if (byte_to_pop_ + len <= buffer_length_)
              {
                std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * len);
                lock_ = true;
                byte_to_pop_ += len;
                if (byte_to_pop_ == buffer_length_) byte_to_pop_ = 0;
                lock_ = false;
              }
            else
              {
                std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * (buffer_length_ - byte_to_pop_));
                std::memcpy(&pop_buf[buffer_length_ - byte_to_pop_], &buf_[0], sizeof(T) * (byte_to_pop_ + len - buffer_length_));
                lock_ = true;
                byte_to_pop_ = byte_to_pop_ + len - buffer_length_;
                lock_ = false;
                return true;
              }
          }
      }

    return false;
  }

  bool push(T* push_buf, uint16_t len = 1)
  {
    /* forcibly write the buffer, regardless of the byte_to_pop */

    if (byte_to_push_ + len <=  buffer_length_)
      {
        std::memcpy(&buf_[byte_to_push_], push_buf, sizeof(T) * len);

        lock_ = true;
        {
          byte_to_push_ += len;

          if (byte_to_push_ == buffer_length_)
            {
              byte_to_push_ = 0;
            }
        }
        lock_ = false;
      }
    else
      {
        std::memcpy(&buf_[byte_to_push_], push_buf, sizeof(T) * (buffer_length_ - byte_to_push_));
        uint16_t residual = len - buffer_length_ + byte_to_push_;
        std::memcpy(&buf_[0], &push_buf[buffer_length_ - byte_to_push_], sizeof(T) * residual);

        lock_ = true;
        {
          byte_to_push_ = residual;
        }
        lock_ = false;
      }

    return true;
  }

  uint16_t length()
  {
    if(lock_) return 0;

    if(byte_to_push_ - byte_to_pop_ >= 0)
      return (byte_to_push_ - byte_to_pop_);
    else
      return (byte_to_push_ + buffer_length_ - byte_to_pop_);
  }

private:
  T buf_[SIZE];
  uint16_t byte_to_pop_, byte_to_push_;
  uint16_t buffer_length_;
  bool lock_;
};

class STMH7Hardware {
public:
  typedef ip4_addr_t  Address;

  STMH7Hardware(){}
  ~STMH7Hardware(){}

  void init() {}
  void init(char *portName){}

  void init(const ip4_addr_t dst_addr, const uint16_t src_port, const uint16_t dst_port);
  int read();
  int read(uint8_t* data, uint16_t length);
  void write(uint8_t* data, uint16_t length);
  int publish();
  uint32_t time(){return HAL_GetTick();}
};



#endif
