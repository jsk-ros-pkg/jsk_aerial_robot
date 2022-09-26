#ifndef _ROS_ublox_msgs_NavSAT_h
#define _ROS_ublox_msgs_NavSAT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/NavSAT_SV.h"

namespace ublox_msgs
{

  class NavSAT : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _numSvs_type;
      _numSvs_type numSvs;
      uint8_t reserved0[2];
      uint32_t sv_length;
      typedef ublox_msgs::NavSAT_SV _sv_type;
      _sv_type st_sv;
      _sv_type * sv;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  53 };

    NavSAT():
      iTOW(0),
      version(0),
      numSvs(0),
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
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->numSvs >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSvs);
      for( uint32_t i = 0; i < 2; i++){
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
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->numSvs =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSvs);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      uint32_t sv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_length);
      if(sv_lengthT > sv_length)
        this->sv = (ublox_msgs::NavSAT_SV*)realloc(this->sv, sv_lengthT * sizeof(ublox_msgs::NavSAT_SV));
      sv_length = sv_lengthT;
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->st_sv.deserialize(inbuffer + offset);
        memcpy( &(this->sv[i]), &(this->st_sv), sizeof(ublox_msgs::NavSAT_SV));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSAT"; };
    virtual const char * getMD5() override { return "e8ea6afd23cb79e7e7385313416d9c15"; };

  };

}
#endif
