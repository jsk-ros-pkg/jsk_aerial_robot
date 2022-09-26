#ifndef _ROS_ublox_msgs_NavSVINFO_h
#define _ROS_ublox_msgs_NavSVINFO_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/NavSVINFO_SV.h"

namespace ublox_msgs
{

  class NavSVINFO : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _numCh_type;
      _numCh_type numCh;
      typedef uint8_t _globalFlags_type;
      _globalFlags_type globalFlags;
      typedef uint16_t _reserved2_type;
      _reserved2_type reserved2;
      uint32_t sv_length;
      typedef ublox_msgs::NavSVINFO_SV _sv_type;
      _sv_type st_sv;
      _sv_type * sv;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  48 };
      enum { CHIPGEN_ANTARIS =  0    };
      enum { CHIPGEN_UBLOX5 =  1     };
      enum { CHIPGEN_UBLOX6 =  2     };
      enum { CHIPGEN_UBLOX7 =  3     };
      enum { CHIPGEN_UBLOX8 =  4     };

    NavSVINFO():
      iTOW(0),
      numCh(0),
      globalFlags(0),
      reserved2(0),
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
      *(outbuffer + offset + 0) = (this->numCh >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numCh);
      *(outbuffer + offset + 0) = (this->globalFlags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->globalFlags);
      *(outbuffer + offset + 0) = (this->reserved2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserved2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->reserved2);
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
      this->numCh =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numCh);
      this->globalFlags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->globalFlags);
      this->reserved2 =  ((uint16_t) (*(inbuffer + offset)));
      this->reserved2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->reserved2);
      uint32_t sv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_length);
      if(sv_lengthT > sv_length)
        this->sv = (ublox_msgs::NavSVINFO_SV*)realloc(this->sv, sv_lengthT * sizeof(ublox_msgs::NavSVINFO_SV));
      sv_length = sv_lengthT;
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->st_sv.deserialize(inbuffer + offset);
        memcpy( &(this->sv[i]), &(this->st_sv), sizeof(ublox_msgs::NavSVINFO_SV));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSVINFO"; };
    virtual const char * getMD5() override { return "869d73cea8ef4943b3f757dcb81282ea"; };

  };

}
#endif
