#ifndef _ROS_ublox_msgs_RxmRTCM_h
#define _ROS_ublox_msgs_RxmRTCM_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmRTCM : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _flags_type;
      _flags_type flags;
      uint8_t reserved0[2];
      typedef uint16_t _refStation_type;
      _refStation_type refStation;
      typedef uint16_t _msgType_type;
      _msgType_type msgType;
      enum { CLASS_ID =  2 };
      enum { MESSAGE_ID =  50 };
      enum { FLAGS_CRC_FAILED =  1     };

    RxmRTCM():
      version(0),
      flags(0),
      reserved0(),
      refStation(0),
      msgType(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      *(outbuffer + offset + 0) = (this->refStation >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->refStation >> (8 * 1)) & 0xFF;
      offset += sizeof(this->refStation);
      *(outbuffer + offset + 0) = (this->msgType >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->msgType >> (8 * 1)) & 0xFF;
      offset += sizeof(this->msgType);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      this->refStation =  ((uint16_t) (*(inbuffer + offset)));
      this->refStation |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->refStation);
      this->msgType =  ((uint16_t) (*(inbuffer + offset)));
      this->msgType |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->msgType);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmRTCM"; };
    virtual const char * getMD5() override { return "86ca768ef7c0009454812a5f8c6badfc"; };

  };

}
#endif
