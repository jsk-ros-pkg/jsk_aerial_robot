#ifndef _ROS_ublox_msgs_CfgHNR_h
#define _ROS_ublox_msgs_CfgHNR_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgHNR : public ros::Msg
  {
    public:
      typedef uint8_t _highNavRate_type;
      _highNavRate_type highNavRate;
      uint8_t reserved0[3];
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  92 };

    CfgHNR():
      highNavRate(0),
      reserved0()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->highNavRate >> (8 * 0)) & 0xFF;
      offset += sizeof(this->highNavRate);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->highNavRate =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->highNavRate);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgHNR"; };
    virtual const char * getMD5() override { return "9398ae76435d6cc9cd4bf6ee2c626072"; };

  };

}
#endif
