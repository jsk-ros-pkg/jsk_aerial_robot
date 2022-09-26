#ifndef _ROS_ublox_msgs_MonGNSS_h
#define _ROS_ublox_msgs_MonGNSS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class MonGNSS : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _supported_type;
      _supported_type supported;
      typedef uint8_t _defaultGnss_type;
      _defaultGnss_type defaultGnss;
      typedef uint8_t _enabled_type;
      _enabled_type enabled;
      typedef uint8_t _simultaneous_type;
      _simultaneous_type simultaneous;
      uint8_t reserved1[3];
      enum { CLASS_ID =  10 };
      enum { MESSAGE_ID =  40 };
      enum { BIT_MASK_GPS =  1 };
      enum { BIT_MASK_GLONASS =  2 };
      enum { BIT_MASK_BEIDOU =  4 };
      enum { BIT_MASK_GALILEO =  8 };

    MonGNSS():
      version(0),
      supported(0),
      defaultGnss(0),
      enabled(0),
      simultaneous(0),
      reserved1()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->supported >> (8 * 0)) & 0xFF;
      offset += sizeof(this->supported);
      *(outbuffer + offset + 0) = (this->defaultGnss >> (8 * 0)) & 0xFF;
      offset += sizeof(this->defaultGnss);
      *(outbuffer + offset + 0) = (this->enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      *(outbuffer + offset + 0) = (this->simultaneous >> (8 * 0)) & 0xFF;
      offset += sizeof(this->simultaneous);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->supported =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->supported);
      this->defaultGnss =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->defaultGnss);
      this->enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->enabled);
      this->simultaneous =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->simultaneous);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/MonGNSS"; };
    virtual const char * getMD5() override { return "0986728889e4abf5eac46b70b74047ed"; };

  };

}
#endif
