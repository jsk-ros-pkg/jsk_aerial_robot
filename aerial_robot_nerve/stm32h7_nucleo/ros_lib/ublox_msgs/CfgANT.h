#ifndef _ROS_ublox_msgs_CfgANT_h
#define _ROS_ublox_msgs_CfgANT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgANT : public ros::Msg
  {
    public:
      typedef uint16_t _flags_type;
      _flags_type flags;
      typedef uint16_t _pins_type;
      _pins_type pins;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  19 };
      enum { FLAGS_SVCS =  1          };
      enum { FLAGS_SCD =  2           };
      enum { FLAGS_OCD =  4           };
      enum { FLAGS_PDWN_ON_SCD =  8   };
      enum { FLAGS_RECOVERY =  16     };
      enum { PIN_SWITCH_MASK =  31     };
      enum { PIN_SCD_MASK =  992       };
      enum { PIN_OCD_MASK =  31744     };
      enum { PIN_RECONFIG =  32768     };

    CfgANT():
      flags(0),
      pins(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->pins >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pins >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pins);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      this->pins =  ((uint16_t) (*(inbuffer + offset)));
      this->pins |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pins);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgANT"; };
    virtual const char * getMD5() override { return "3fb4e960ecb1d1999afa14ab38e0b768"; };

  };

}
#endif
