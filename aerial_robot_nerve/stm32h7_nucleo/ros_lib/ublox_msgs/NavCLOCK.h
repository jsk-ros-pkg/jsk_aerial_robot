#ifndef _ROS_ublox_msgs_NavCLOCK_h
#define _ROS_ublox_msgs_NavCLOCK_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavCLOCK : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _clkB_type;
      _clkB_type clkB;
      typedef int32_t _clkD_type;
      _clkD_type clkD;
      typedef uint32_t _tAcc_type;
      _tAcc_type tAcc;
      typedef uint32_t _fAcc_type;
      _fAcc_type fAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  34 };

    NavCLOCK():
      iTOW(0),
      clkB(0),
      clkD(0),
      tAcc(0),
      fAcc(0)
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
      } u_clkB;
      u_clkB.real = this->clkB;
      *(outbuffer + offset + 0) = (u_clkB.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clkB.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clkB.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clkB.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clkB);
      union {
        int32_t real;
        uint32_t base;
      } u_clkD;
      u_clkD.real = this->clkD;
      *(outbuffer + offset + 0) = (u_clkD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clkD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clkD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clkD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clkD);
      *(outbuffer + offset + 0) = (this->tAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tAcc);
      *(outbuffer + offset + 0) = (this->fAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fAcc);
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
      } u_clkB;
      u_clkB.base = 0;
      u_clkB.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clkB.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clkB.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clkB.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clkB = u_clkB.real;
      offset += sizeof(this->clkB);
      union {
        int32_t real;
        uint32_t base;
      } u_clkD;
      u_clkD.base = 0;
      u_clkD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clkD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clkD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clkD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clkD = u_clkD.real;
      offset += sizeof(this->clkD);
      this->tAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tAcc);
      this->fAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->fAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavCLOCK"; };
    virtual const char * getMD5() override { return "a9acfdf2e7ac2bf086926ae4e6a182a0"; };

  };

}
#endif
