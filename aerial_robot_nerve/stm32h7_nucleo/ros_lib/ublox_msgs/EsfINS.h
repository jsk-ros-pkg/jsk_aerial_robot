#ifndef _ROS_ublox_msgs_EsfINS_h
#define _ROS_ublox_msgs_EsfINS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class EsfINS : public ros::Msg
  {
    public:
      typedef uint32_t _bitfield0_type;
      _bitfield0_type bitfield0;
      uint8_t reserved1[4];
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _xAngRate_type;
      _xAngRate_type xAngRate;
      typedef int32_t _yAngRate_type;
      _yAngRate_type yAngRate;
      typedef int32_t _zAngRate_type;
      _zAngRate_type zAngRate;
      typedef int32_t _xAccel_type;
      _xAccel_type xAccel;
      typedef int32_t _yAccel_type;
      _yAccel_type yAccel;
      typedef int32_t _zAccel_type;
      _zAccel_type zAccel;
      enum { CLASS_ID =  16 };
      enum { MESSAGE_ID =  21 };
      enum { BITFIELD0_VERSION =  255             };
      enum { BITFIELD0_X_ANG_RATE_VALID =  256    };
      enum { BITFIELD0_Y_ANG_RATE_VALID =  512    };
      enum { BITFIELD0_Z_ANG_RATE_VALID =  1024   };
      enum { BITFIELD0_X_ACCEL_VALID =  2048      };
      enum { BITFIELD0_Y_ACCEL_VALID =  4096      };
      enum { BITFIELD0_Z_ACCEL_VALID =  8192      };

    EsfINS():
      bitfield0(0),
      reserved1(),
      iTOW(0),
      xAngRate(0),
      yAngRate(0),
      zAngRate(0),
      xAccel(0),
      yAccel(0),
      zAccel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->bitfield0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->bitfield0 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->bitfield0 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->bitfield0 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bitfield0);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_xAngRate;
      u_xAngRate.real = this->xAngRate;
      *(outbuffer + offset + 0) = (u_xAngRate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xAngRate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xAngRate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xAngRate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_yAngRate;
      u_yAngRate.real = this->yAngRate;
      *(outbuffer + offset + 0) = (u_yAngRate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yAngRate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yAngRate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yAngRate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_zAngRate;
      u_zAngRate.real = this->zAngRate;
      *(outbuffer + offset + 0) = (u_zAngRate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zAngRate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zAngRate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zAngRate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_xAccel;
      u_xAccel.real = this->xAccel;
      *(outbuffer + offset + 0) = (u_xAccel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xAccel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xAccel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xAccel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xAccel);
      union {
        int32_t real;
        uint32_t base;
      } u_yAccel;
      u_yAccel.real = this->yAccel;
      *(outbuffer + offset + 0) = (u_yAccel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yAccel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yAccel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yAccel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yAccel);
      union {
        int32_t real;
        uint32_t base;
      } u_zAccel;
      u_zAccel.real = this->zAccel;
      *(outbuffer + offset + 0) = (u_zAccel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zAccel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zAccel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zAccel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zAccel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->bitfield0 =  ((uint32_t) (*(inbuffer + offset)));
      this->bitfield0 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bitfield0 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->bitfield0 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->bitfield0);
      for( uint32_t i = 0; i < 4; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_xAngRate;
      u_xAngRate.base = 0;
      u_xAngRate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xAngRate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xAngRate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xAngRate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xAngRate = u_xAngRate.real;
      offset += sizeof(this->xAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_yAngRate;
      u_yAngRate.base = 0;
      u_yAngRate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yAngRate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yAngRate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yAngRate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yAngRate = u_yAngRate.real;
      offset += sizeof(this->yAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_zAngRate;
      u_zAngRate.base = 0;
      u_zAngRate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zAngRate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zAngRate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zAngRate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zAngRate = u_zAngRate.real;
      offset += sizeof(this->zAngRate);
      union {
        int32_t real;
        uint32_t base;
      } u_xAccel;
      u_xAccel.base = 0;
      u_xAccel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xAccel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xAccel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xAccel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xAccel = u_xAccel.real;
      offset += sizeof(this->xAccel);
      union {
        int32_t real;
        uint32_t base;
      } u_yAccel;
      u_yAccel.base = 0;
      u_yAccel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yAccel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yAccel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yAccel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yAccel = u_yAccel.real;
      offset += sizeof(this->yAccel);
      union {
        int32_t real;
        uint32_t base;
      } u_zAccel;
      u_zAccel.base = 0;
      u_zAccel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zAccel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zAccel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zAccel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zAccel = u_zAccel.real;
      offset += sizeof(this->zAccel);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfINS"; };
    virtual const char * getMD5() override { return "975d05e9503737524d7e47a42cb9fff1"; };

  };

}
#endif
