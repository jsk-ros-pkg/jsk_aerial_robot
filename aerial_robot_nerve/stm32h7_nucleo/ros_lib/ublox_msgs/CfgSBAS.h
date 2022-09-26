#ifndef _ROS_ublox_msgs_CfgSBAS_h
#define _ROS_ublox_msgs_CfgSBAS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgSBAS : public ros::Msg
  {
    public:
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef uint8_t _usage_type;
      _usage_type usage;
      typedef uint8_t _maxSBAS_type;
      _maxSBAS_type maxSBAS;
      typedef uint8_t _scanmode2_type;
      _scanmode2_type scanmode2;
      typedef uint32_t _scanmode1_type;
      _scanmode1_type scanmode1;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  22 };
      enum { MODE_ENABLED =  1     };
      enum { MODE_TEST =  2        };
      enum { USAGE_RANGE =  1      };
      enum { USAGE_DIFF_CORR =  2  };
      enum { USAGE_INTEGRITY =  4  };

    CfgSBAS():
      mode(0),
      usage(0),
      maxSBAS(0),
      scanmode2(0),
      scanmode1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->usage >> (8 * 0)) & 0xFF;
      offset += sizeof(this->usage);
      *(outbuffer + offset + 0) = (this->maxSBAS >> (8 * 0)) & 0xFF;
      offset += sizeof(this->maxSBAS);
      *(outbuffer + offset + 0) = (this->scanmode2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->scanmode2);
      *(outbuffer + offset + 0) = (this->scanmode1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->scanmode1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->scanmode1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->scanmode1 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scanmode1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      this->usage =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->usage);
      this->maxSBAS =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->maxSBAS);
      this->scanmode2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->scanmode2);
      this->scanmode1 =  ((uint32_t) (*(inbuffer + offset)));
      this->scanmode1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->scanmode1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->scanmode1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->scanmode1);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgSBAS"; };
    virtual const char * getMD5() override { return "b03a1b853ac45d2da104aafaa036e7e8"; };

  };

}
#endif
