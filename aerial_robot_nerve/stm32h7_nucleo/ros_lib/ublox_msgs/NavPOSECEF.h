#ifndef _ROS_ublox_msgs_NavPOSECEF_h
#define _ROS_ublox_msgs_NavPOSECEF_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavPOSECEF : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _ecefX_type;
      _ecefX_type ecefX;
      typedef int32_t _ecefY_type;
      _ecefY_type ecefY;
      typedef int32_t _ecefZ_type;
      _ecefZ_type ecefZ;
      typedef uint32_t _pAcc_type;
      _pAcc_type pAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  1 };

    NavPOSECEF():
      iTOW(0),
      ecefX(0),
      ecefY(0),
      ecefZ(0),
      pAcc(0)
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
      } u_ecefX;
      u_ecefX.real = this->ecefX;
      *(outbuffer + offset + 0) = (u_ecefX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.real = this->ecefY;
      *(outbuffer + offset + 0) = (u_ecefY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.real = this->ecefZ;
      *(outbuffer + offset + 0) = (u_ecefZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefZ);
      *(outbuffer + offset + 0) = (this->pAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pAcc);
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
      } u_ecefX;
      u_ecefX.base = 0;
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefX = u_ecefX.real;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.base = 0;
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefY = u_ecefY.real;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.base = 0;
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefZ = u_ecefZ.real;
      offset += sizeof(this->ecefZ);
      this->pAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavPOSECEF"; };
    virtual const char * getMD5() override { return "6f1f4f9473d5586f7fa1427a3c53cee9"; };

  };

}
#endif
