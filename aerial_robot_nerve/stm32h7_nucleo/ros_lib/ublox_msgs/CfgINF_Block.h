#ifndef _ROS_ublox_msgs_CfgINF_Block_h
#define _ROS_ublox_msgs_CfgINF_Block_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgINF_Block : public ros::Msg
  {
    public:
      typedef uint8_t _protocolID_type;
      _protocolID_type protocolID;
      uint8_t reserved1[3];
      uint8_t infMsgMask[6];
      enum { PROTOCOL_ID_UBX =  0 };
      enum { PROTOCOL_ID_NMEA =  1 };
      enum { INF_MSG_ERROR =  1               };
      enum { INF_MSG_WARNING =  2             };
      enum { INF_MSG_NOTICE =  4              };
      enum { INF_MSG_TEST =  8                };
      enum { INF_MSG_DEBUG =  16              };

    CfgINF_Block():
      protocolID(0),
      reserved1(),
      infMsgMask()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->protocolID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->protocolID);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->infMsgMask[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->infMsgMask[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->protocolID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->protocolID);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      this->infMsgMask[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->infMsgMask[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgINF_Block"; };
    virtual const char * getMD5() override { return "71c7fcecf939acbf06ee4e1259793fce"; };

  };

}
#endif
