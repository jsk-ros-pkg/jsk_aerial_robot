#ifndef _ROS_ublox_msgs_NavSBAS_h
#define _ROS_ublox_msgs_NavSBAS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/NavSBAS_SV.h"

namespace ublox_msgs
{

  class NavSBAS : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _geo_type;
      _geo_type geo;
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef int8_t _sys_type;
      _sys_type sys;
      typedef uint8_t _service_type;
      _service_type service;
      typedef uint8_t _cnt_type;
      _cnt_type cnt;
      uint8_t reserved0[3];
      uint32_t sv_length;
      typedef ublox_msgs::NavSBAS_SV _sv_type;
      _sv_type st_sv;
      _sv_type * sv;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  50 };
      enum { MODE_DISABLED =  0 };
      enum { MODE_ENABLED_INTEGRITY =  1 };
      enum { MODE_ENABLED_TESTMODE =  3 };
      enum { SYS_UNKNOWN =  -1 };
      enum { SYS_WAAS =  0 };
      enum { SYS_EGNOS =  1 };
      enum { SYS_MSAS =  2 };
      enum { SYS_GAGAN =  3 };
      enum { SYS_GPS =  16 };
      enum { SERVICE_RANGING =  1 };
      enum { SERVICE_CORRECTIONS =  2 };
      enum { SERVICE_INTEGRITY =  4 };
      enum { SERVICE_TESTMODE =  8 };

    NavSBAS():
      iTOW(0),
      geo(0),
      mode(0),
      sys(0),
      service(0),
      cnt(0),
      reserved0(),
      sv_length(0), st_sv(), sv(nullptr)
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
      *(outbuffer + offset + 0) = (this->geo >> (8 * 0)) & 0xFF;
      offset += sizeof(this->geo);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        int8_t real;
        uint8_t base;
      } u_sys;
      u_sys.real = this->sys;
      *(outbuffer + offset + 0) = (u_sys.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sys);
      *(outbuffer + offset + 0) = (this->service >> (8 * 0)) & 0xFF;
      offset += sizeof(this->service);
      *(outbuffer + offset + 0) = (this->cnt >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cnt);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      *(outbuffer + offset + 0) = (this->sv_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sv_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sv_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sv_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sv_length);
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->sv[i].serialize(outbuffer + offset);
      }
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
      this->geo =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->geo);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        int8_t real;
        uint8_t base;
      } u_sys;
      u_sys.base = 0;
      u_sys.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sys = u_sys.real;
      offset += sizeof(this->sys);
      this->service =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->service);
      this->cnt =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cnt);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      uint32_t sv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_length);
      if(sv_lengthT > sv_length)
        this->sv = (ublox_msgs::NavSBAS_SV*)realloc(this->sv, sv_lengthT * sizeof(ublox_msgs::NavSBAS_SV));
      sv_length = sv_lengthT;
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->st_sv.deserialize(inbuffer + offset);
        memcpy( &(this->sv[i]), &(this->st_sv), sizeof(ublox_msgs::NavSBAS_SV));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSBAS"; };
    virtual const char * getMD5() override { return "4ea0a5f1e74c114f14024eb1bc277c57"; };

  };

}
#endif
