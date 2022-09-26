#ifndef _ROS_ublox_msgs_CfgMSG_h
#define _ROS_ublox_msgs_CfgMSG_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgMSG : public ros::Msg
  {
    public:
      typedef uint8_t _msgClass_type;
      _msgClass_type msgClass;
      typedef uint8_t _msgID_type;
      _msgID_type msgID;
      typedef uint8_t _rate_type;
      _rate_type rate;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  1 };

    CfgMSG():
      msgClass(0),
      msgID(0),
      rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->msgClass >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgClass);
      *(outbuffer + offset + 0) = (this->msgID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgID);
      *(outbuffer + offset + 0) = (this->rate >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->msgClass =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msgClass);
      this->msgID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msgID);
      this->rate =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rate);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgMSG"; };
    virtual const char * getMD5() override { return "9f2fcd2333c19c76cbfdf6a57fc64a9d"; };

  };

}
#endif
