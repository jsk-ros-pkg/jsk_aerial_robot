#ifndef _ROS_ublox_msgs_NavSBAS_SV_h
#define _ROS_ublox_msgs_NavSBAS_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSBAS_SV : public ros::Msg
  {
    public:
      typedef uint8_t _svid_type;
      _svid_type svid;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint8_t _udre_type;
      _udre_type udre;
      typedef uint8_t _svSys_type;
      _svSys_type svSys;
      typedef uint8_t _svService_type;
      _svService_type svService;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef int16_t _prc_type;
      _prc_type prc;
      typedef uint16_t _reserved2_type;
      _reserved2_type reserved2;
      typedef int16_t _ic_type;
      _ic_type ic;

    NavSBAS_SV():
      svid(0),
      flags(0),
      udre(0),
      svSys(0),
      svService(0),
      reserved1(0),
      prc(0),
      reserved2(0),
      ic(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->udre >> (8 * 0)) & 0xFF;
      offset += sizeof(this->udre);
      *(outbuffer + offset + 0) = (this->svSys >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svSys);
      *(outbuffer + offset + 0) = (this->svService >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svService);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      union {
        int16_t real;
        uint16_t base;
      } u_prc;
      u_prc.real = this->prc;
      *(outbuffer + offset + 0) = (u_prc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->prc);
      *(outbuffer + offset + 0) = (this->reserved2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserved2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->reserved2);
      union {
        int16_t real;
        uint16_t base;
      } u_ic;
      u_ic.real = this->ic;
      *(outbuffer + offset + 0) = (u_ic.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ic.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->ic);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->udre =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->udre);
      this->svSys =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svSys);
      this->svService =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svService);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      union {
        int16_t real;
        uint16_t base;
      } u_prc;
      u_prc.base = 0;
      u_prc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->prc = u_prc.real;
      offset += sizeof(this->prc);
      this->reserved2 =  ((uint16_t) (*(inbuffer + offset)));
      this->reserved2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->reserved2);
      union {
        int16_t real;
        uint16_t base;
      } u_ic;
      u_ic.base = 0;
      u_ic.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ic.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ic = u_ic.real;
      offset += sizeof(this->ic);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSBAS_SV"; };
    virtual const char * getMD5() override { return "45259031fe19a4df2f5a4a667356a0bc"; };

  };

}
#endif
