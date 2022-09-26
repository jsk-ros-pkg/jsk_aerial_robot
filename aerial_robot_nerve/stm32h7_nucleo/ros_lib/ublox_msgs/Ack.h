#ifndef _ROS_ublox_msgs_Ack_h
#define _ROS_ublox_msgs_Ack_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class Ack : public ros::Msg
  {
    public:
      typedef uint8_t _clsID_type;
      _clsID_type clsID;
      typedef uint8_t _msgID_type;
      _msgID_type msgID;
      enum { CLASS_ID =  5 };
      enum { NACK_MESSAGE_ID =  0 };
      enum { ACK_MESSAGE_ID =  1 };

    Ack():
      clsID(0),
      msgID(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->clsID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clsID);
      *(outbuffer + offset + 0) = (this->msgID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgID);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->clsID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->clsID);
      this->msgID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msgID);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/Ack"; };
    virtual const char * getMD5() override { return "fc3270816d86d7c962dbc4b52a6c5386"; };

  };

}
#endif
