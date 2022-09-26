#ifndef _ROS_ublox_msgs_NavDGPS_SV_h
#define _ROS_ublox_msgs_NavDGPS_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavDGPS_SV : public ros::Msg
  {
    public:
      typedef uint8_t _svid_type;
      _svid_type svid;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint16_t _ageC_type;
      _ageC_type ageC;
      typedef float _prc_type;
      _prc_type prc;
      typedef float _prrc_type;
      _prrc_type prrc;
      enum { FLAGS_CHANNEL_MASK =  15    };
      enum { FLAGS_DGPS =  16            };

    NavDGPS_SV():
      svid(0),
      flags(0),
      ageC(0),
      prc(0),
      prrc(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->ageC >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ageC >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ageC);
      union {
        float real;
        uint32_t base;
      } u_prc;
      u_prc.real = this->prc;
      *(outbuffer + offset + 0) = (u_prc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prc);
      union {
        float real;
        uint32_t base;
      } u_prrc;
      u_prrc.real = this->prrc;
      *(outbuffer + offset + 0) = (u_prrc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prrc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prrc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prrc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prrc);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->ageC =  ((uint16_t) (*(inbuffer + offset)));
      this->ageC |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->ageC);
      union {
        float real;
        uint32_t base;
      } u_prc;
      u_prc.base = 0;
      u_prc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prc = u_prc.real;
      offset += sizeof(this->prc);
      union {
        float real;
        uint32_t base;
      } u_prrc;
      u_prrc.base = 0;
      u_prrc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prrc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prrc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prrc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prrc = u_prrc.real;
      offset += sizeof(this->prrc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavDGPS_SV"; };
    virtual const char * getMD5() override { return "c16a60aa23db6f4f1a80cf6720b95063"; };

  };

}
#endif
