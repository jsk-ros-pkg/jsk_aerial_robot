#ifndef _ROS_ublox_msgs_NavVELNED_h
#define _ROS_ublox_msgs_NavVELNED_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavVELNED : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _velN_type;
      _velN_type velN;
      typedef int32_t _velE_type;
      _velE_type velE;
      typedef int32_t _velD_type;
      _velD_type velD;
      typedef uint32_t _speed_type;
      _speed_type speed;
      typedef uint32_t _gSpeed_type;
      _gSpeed_type gSpeed;
      typedef int32_t _heading_type;
      _heading_type heading;
      typedef uint32_t _sAcc_type;
      _sAcc_type sAcc;
      typedef uint32_t _cAcc_type;
      _cAcc_type cAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  18 };

    NavVELNED():
      iTOW(0),
      velN(0),
      velE(0),
      velD(0),
      speed(0),
      gSpeed(0),
      heading(0),
      sAcc(0),
      cAcc(0)
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
      } u_velN;
      u_velN.real = this->velN;
      *(outbuffer + offset + 0) = (u_velN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velN.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velN.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velN.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velN);
      union {
        int32_t real;
        uint32_t base;
      } u_velE;
      u_velE.real = this->velE;
      *(outbuffer + offset + 0) = (u_velE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velE.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velE.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velE.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velE);
      union {
        int32_t real;
        uint32_t base;
      } u_velD;
      u_velD.real = this->velD;
      *(outbuffer + offset + 0) = (u_velD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velD);
      *(outbuffer + offset + 0) = (this->speed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->speed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->speed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->speed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      *(outbuffer + offset + 0) = (this->gSpeed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gSpeed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gSpeed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gSpeed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      *(outbuffer + offset + 0) = (this->sAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sAcc);
      *(outbuffer + offset + 0) = (this->cAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cAcc);
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
      } u_velN;
      u_velN.base = 0;
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velN = u_velN.real;
      offset += sizeof(this->velN);
      union {
        int32_t real;
        uint32_t base;
      } u_velE;
      u_velE.base = 0;
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velE = u_velE.real;
      offset += sizeof(this->velE);
      union {
        int32_t real;
        uint32_t base;
      } u_velD;
      u_velD.base = 0;
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velD = u_velD.real;
      offset += sizeof(this->velD);
      this->speed =  ((uint32_t) (*(inbuffer + offset)));
      this->speed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->speed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->speed);
      this->gSpeed =  ((uint32_t) (*(inbuffer + offset)));
      this->gSpeed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->gSpeed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->gSpeed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->gSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      this->sAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sAcc);
      this->cAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->cAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavVELNED"; };
    virtual const char * getMD5() override { return "b03402bb86164e74f01e04bff1850150"; };

  };

}
#endif
