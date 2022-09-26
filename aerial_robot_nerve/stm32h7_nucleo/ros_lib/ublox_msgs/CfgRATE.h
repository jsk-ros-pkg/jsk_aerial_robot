#ifndef _ROS_ublox_msgs_CfgRATE_h
#define _ROS_ublox_msgs_CfgRATE_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgRATE : public ros::Msg
  {
    public:
      typedef uint16_t _measRate_type;
      _measRate_type measRate;
      typedef uint16_t _navRate_type;
      _navRate_type navRate;
      typedef uint16_t _timeRef_type;
      _timeRef_type timeRef;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  8 };
      enum { TIME_REF_UTC =  0 };
      enum { TIME_REF_GPS =  1 };
      enum { TIME_REF_GLONASS =  2    };
      enum { TIME_REF_BEIDOU =  3     };
      enum { TIME_REF_GALILEO =  4    };

    CfgRATE():
      measRate(0),
      navRate(0),
      timeRef(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->measRate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->measRate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->measRate);
      *(outbuffer + offset + 0) = (this->navRate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->navRate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->navRate);
      *(outbuffer + offset + 0) = (this->timeRef >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeRef >> (8 * 1)) & 0xFF;
      offset += sizeof(this->timeRef);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->measRate =  ((uint16_t) (*(inbuffer + offset)));
      this->measRate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->measRate);
      this->navRate =  ((uint16_t) (*(inbuffer + offset)));
      this->navRate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->navRate);
      this->timeRef =  ((uint16_t) (*(inbuffer + offset)));
      this->timeRef |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->timeRef);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgRATE"; };
    virtual const char * getMD5() override { return "13e27469d3f7d85353464015f687d6b2"; };

  };

}
#endif
