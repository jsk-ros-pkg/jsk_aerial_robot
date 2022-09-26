#ifndef _ROS_ublox_msgs_NavVELECEF_h
#define _ROS_ublox_msgs_NavVELECEF_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavVELECEF : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _ecefVX_type;
      _ecefVX_type ecefVX;
      typedef int32_t _ecefVY_type;
      _ecefVY_type ecefVY;
      typedef int32_t _ecefVZ_type;
      _ecefVZ_type ecefVZ;
      typedef uint32_t _sAcc_type;
      _sAcc_type sAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  17 };

    NavVELECEF():
      iTOW(0),
      ecefVX(0),
      ecefVY(0),
      ecefVZ(0),
      sAcc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVX;
      u_ecefVX.real = this->ecefVX;
      *(outbuffer + offset + 0) = (u_ecefVX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVY;
      u_ecefVY.real = this->ecefVY;
      *(outbuffer + offset + 0) = (u_ecefVY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVZ;
      u_ecefVZ.real = this->ecefVZ;
      *(outbuffer + offset + 0) = (u_ecefVZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVZ);
      *(outbuffer + offset + 0) = (this->sAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sAcc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVX;
      u_ecefVX.base = 0;
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVX = u_ecefVX.real;
      offset += sizeof(this->ecefVX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVY;
      u_ecefVY.base = 0;
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVY = u_ecefVY.real;
      offset += sizeof(this->ecefVY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVZ;
      u_ecefVZ.base = 0;
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVZ = u_ecefVZ.real;
      offset += sizeof(this->ecefVZ);
      this->sAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavVELECEF"; };
    virtual const char * getMD5() override { return "97299f597364a39a6c0e96ed2ee4e702"; };

  };

}
#endif
