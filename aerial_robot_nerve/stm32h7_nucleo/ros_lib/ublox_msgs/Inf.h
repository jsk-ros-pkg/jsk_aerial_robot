#ifndef _ROS_ublox_msgs_Inf_h
#define _ROS_ublox_msgs_Inf_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class Inf : public ros::Msg
  {
    public:
      uint32_t str_length;
      typedef uint8_t _str_type;
      _str_type st_str;
      _str_type * str;
      enum { CLASS_ID =  4 };

    Inf():
      str_length(0), st_str(), str(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->str_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->str_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->str_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->str_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->str_length);
      for( uint32_t i = 0; i < str_length; i++){
      *(outbuffer + offset + 0) = (this->str[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->str[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t str_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      str_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      str_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      str_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->str_length);
      if(str_lengthT > str_length)
        this->str = (uint8_t*)realloc(this->str, str_lengthT * sizeof(uint8_t));
      str_length = str_lengthT;
      for( uint32_t i = 0; i < str_length; i++){
      this->st_str =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_str);
        memcpy( &(this->str[i]), &(this->st_str), sizeof(uint8_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/Inf"; };
    virtual const char * getMD5() override { return "d1c433234e5eccc57045e40aca48eb6e"; };

  };

}
#endif
