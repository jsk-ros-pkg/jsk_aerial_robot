#ifndef _ROS_pr2_msgs_PeriodicCmd_h
#define _ROS_pr2_msgs_PeriodicCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class PeriodicCmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _profile_type;
      _profile_type profile;
      typedef double _period_type;
      _period_type period;
      typedef double _amplitude_type;
      _amplitude_type amplitude;
      typedef double _offset_type;
      _offset_type offset;

    PeriodicCmd():
      header(),
      profile(""),
      period(0),
      amplitude(0),
      offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_profile = strlen(this->profile);
      varToArr(outbuffer + offset, length_profile);
      offset += 4;
      memcpy(outbuffer + offset, this->profile, length_profile);
      offset += length_profile;
      union {
        double real;
        uint64_t base;
      } u_period;
      u_period.real = this->period;
      *(outbuffer + offset + 0) = (u_period.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_period.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_period.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_period.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_period.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_period.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_period.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_period.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->period);
      union {
        double real;
        uint64_t base;
      } u_amplitude;
      u_amplitude.real = this->amplitude;
      *(outbuffer + offset + 0) = (u_amplitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_amplitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_amplitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_amplitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_amplitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_amplitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_amplitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_amplitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->amplitude);
      union {
        double real;
        uint64_t base;
      } u_offset;
      u_offset.real = this->offset;
      *(outbuffer + offset + 0) = (u_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_profile;
      arrToVar(length_profile, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_profile; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_profile-1]=0;
      this->profile = (char *)(inbuffer + offset-1);
      offset += length_profile;
      union {
        double real;
        uint64_t base;
      } u_period;
      u_period.base = 0;
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->period = u_period.real;
      offset += sizeof(this->period);
      union {
        double real;
        uint64_t base;
      } u_amplitude;
      u_amplitude.base = 0;
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->amplitude = u_amplitude.real;
      offset += sizeof(this->amplitude);
      union {
        double real;
        uint64_t base;
      } u_offset;
      u_offset.base = 0;
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->offset = u_offset.real;
      offset += sizeof(this->offset);
     return offset;
    }

    const char * getType(){ return "pr2_msgs/PeriodicCmd"; };
    const char * getMD5(){ return "95ab7e548e3d4274f83393129dd96c2e"; };

  };

}
#endif
