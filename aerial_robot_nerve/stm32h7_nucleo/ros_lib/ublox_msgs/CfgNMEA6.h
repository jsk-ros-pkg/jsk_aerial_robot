#ifndef _ROS_ublox_msgs_CfgNMEA6_h
#define _ROS_ublox_msgs_CfgNMEA6_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgNMEA6 : public ros::Msg
  {
    public:
      typedef uint8_t _filter_type;
      _filter_type filter;
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _numSV_type;
      _numSV_type numSV;
      typedef uint8_t _flags_type;
      _flags_type flags;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  23 };
      enum { FILTER_POS =  1           };
      enum { FILTER_MSK_POS =  2       };
      enum { FILTER_TIME =  4          };
      enum { FILTER_DATE =  8          };
      enum { FILTER_SBAS_FILT =  16    };
      enum { FILTER_TRACK =  32        };
      enum { NMEA_VERSION_2_3 =  35      };
      enum { NMEA_VERSION_2_1 =  33      };
      enum { FLAGS_COMPAT =  1           };
      enum { FLAGS_CONSIDER =  2         };

    CfgNMEA6():
      filter(0),
      version(0),
      numSV(0),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->filter >> (8 * 0)) & 0xFF;
      offset += sizeof(this->filter);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->numSV >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSV);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->filter =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->filter);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->numSV =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSV);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgNMEA6"; };
    virtual const char * getMD5() override { return "9ffbd21c832ce4472519430326bb44e3"; };

  };

}
#endif
