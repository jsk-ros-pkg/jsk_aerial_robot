#ifndef _ROS_ublox_msgs_MonVER_Extension_h
#define _ROS_ublox_msgs_MonVER_Extension_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class MonVER_Extension : public ros::Msg
  {
    public:
      uint8_t field[30];

    MonVER_Extension():
      field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 30; i++){
      *(outbuffer + offset + 0) = (this->field[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->field[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 30; i++){
      this->field[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->field[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/MonVER_Extension"; };
    virtual const char * getMD5() override { return "ef92c9d93e6bd2c2701384b0595ac2b4"; };

  };

}
#endif
