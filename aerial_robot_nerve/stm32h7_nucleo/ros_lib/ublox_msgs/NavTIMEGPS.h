#ifndef _ROS_ublox_msgs_NavTIMEGPS_h
#define _ROS_ublox_msgs_NavTIMEGPS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavTIMEGPS : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _fTOW_type;
      _fTOW_type fTOW;
      typedef int16_t _week_type;
      _week_type week;
      typedef int8_t _leapS_type;
      _leapS_type leapS;
      typedef uint8_t _valid_type;
      _valid_type valid;
      typedef uint32_t _tAcc_type;
      _tAcc_type tAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  32 };
      enum { VALID_TOW =  1         };
      enum { VALID_WEEK =  2        };
      enum { VALID_LEAP_S =  4      };

    NavTIMEGPS():
      iTOW(0),
      fTOW(0),
      week(0),
      leapS(0),
      valid(0),
      tAcc(0)
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
      } u_fTOW;
      u_fTOW.real = this->fTOW;
      *(outbuffer + offset + 0) = (u_fTOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fTOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fTOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fTOW.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.real = this->week;
      *(outbuffer + offset + 0) = (u_week.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_week.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->week);
      union {
        int8_t real;
        uint8_t base;
      } u_leapS;
      u_leapS.real = this->leapS;
      *(outbuffer + offset + 0) = (u_leapS.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->leapS);
      *(outbuffer + offset + 0) = (this->valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      *(outbuffer + offset + 0) = (this->tAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tAcc);
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
      } u_fTOW;
      u_fTOW.base = 0;
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fTOW = u_fTOW.real;
      offset += sizeof(this->fTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.base = 0;
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->week = u_week.real;
      offset += sizeof(this->week);
      union {
        int8_t real;
        uint8_t base;
      } u_leapS;
      u_leapS.base = 0;
      u_leapS.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->leapS = u_leapS.real;
      offset += sizeof(this->leapS);
      this->valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->valid);
      this->tAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavTIMEGPS"; };
    virtual const char * getMD5() override { return "94098180ac3e5e36144a6337c1462f46"; };

  };

}
#endif
