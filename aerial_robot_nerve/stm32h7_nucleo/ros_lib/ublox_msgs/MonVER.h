#ifndef _ROS_ublox_msgs_MonVER_h
#define _ROS_ublox_msgs_MonVER_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/MonVER_Extension.h"

namespace ublox_msgs
{

  class MonVER : public ros::Msg
  {
    public:
      uint8_t swVersion[30];
      uint8_t hwVersion[10];
      uint32_t extension_length;
      typedef ublox_msgs::MonVER_Extension _extension_type;
      _extension_type st_extension;
      _extension_type * extension;
      enum { CLASS_ID =  10 };
      enum { MESSAGE_ID =  4 };

    MonVER():
      swVersion(),
      hwVersion(),
      extension_length(0), st_extension(), extension(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 30; i++){
      *(outbuffer + offset + 0) = (this->swVersion[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->swVersion[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      *(outbuffer + offset + 0) = (this->hwVersion[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hwVersion[i]);
      }
      *(outbuffer + offset + 0) = (this->extension_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->extension_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->extension_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->extension_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->extension_length);
      for( uint32_t i = 0; i < extension_length; i++){
      offset += this->extension[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 30; i++){
      this->swVersion[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->swVersion[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      this->hwVersion[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->hwVersion[i]);
      }
      uint32_t extension_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      extension_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      extension_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      extension_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->extension_length);
      if(extension_lengthT > extension_length)
        this->extension = (ublox_msgs::MonVER_Extension*)realloc(this->extension, extension_lengthT * sizeof(ublox_msgs::MonVER_Extension));
      extension_length = extension_lengthT;
      for( uint32_t i = 0; i < extension_length; i++){
      offset += this->st_extension.deserialize(inbuffer + offset);
        memcpy( &(this->extension[i]), &(this->st_extension), sizeof(ublox_msgs::MonVER_Extension));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/MonVER"; };
    virtual const char * getMD5() override { return "83c8a03d7744e76cc085d953e1a5a293"; };

  };

}
#endif
