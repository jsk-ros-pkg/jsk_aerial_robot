#ifndef _ROS_jsk_topic_tools_TopicInfo_h
#define _ROS_jsk_topic_tools_TopicInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_topic_tools
{

  class TopicInfo : public ros::Msg
  {
    public:
      typedef const char* _topic_name_type;
      _topic_name_type topic_name;
      typedef double _rate_type;
      _rate_type rate;

    TopicInfo():
      topic_name(""),
      rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_topic_name = strlen(this->topic_name);
      varToArr(outbuffer + offset, length_topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, length_topic_name);
      offset += length_topic_name;
      union {
        double real;
        uint64_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_topic_name;
      arrToVar(length_topic_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      union {
        double real;
        uint64_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
     return offset;
    }

    const char * getType(){ return "jsk_topic_tools/TopicInfo"; };
    const char * getMD5(){ return "78edf14defd72c2fcd29e4fad0165ea9"; };

  };

}
#endif
