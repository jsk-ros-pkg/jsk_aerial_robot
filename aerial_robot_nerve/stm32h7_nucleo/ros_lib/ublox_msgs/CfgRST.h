#ifndef _ROS_ublox_msgs_CfgRST_h
#define _ROS_ublox_msgs_CfgRST_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgRST : public ros::Msg
  {
    public:
      typedef uint16_t _navBbrMask_type;
      _navBbrMask_type navBbrMask;
      typedef uint8_t _resetMode_type;
      _resetMode_type resetMode;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  4 };
      enum { NAV_BBR_HOT_START =  0        };
      enum { NAV_BBR_WARM_START =  1       };
      enum { NAV_BBR_COLD_START =  65535   };
      enum { NAV_BBR_EPH =  1        };
      enum { NAV_BBR_ALM =  2        };
      enum { NAV_BBR_HEALTH =  4     };
      enum { NAV_BBR_KLOB =  8       };
      enum { NAV_BBR_POS =  16       };
      enum { NAV_BBR_CLKD =  32      };
      enum { NAV_BBR_OSC =  64       };
      enum { NAV_BBR_UTC =  128      };
      enum { NAV_BBR_RTC =  256      };
      enum { NAV_BBR_AOP =  32768    };
      enum { RESET_MODE_HW_IMMEDIATE =  0        };
      enum { RESET_MODE_SW =  1                  };
      enum { RESET_MODE_GNSS =  2                };
      enum { RESET_MODE_HW_AFTER_SHUTDOWN =  4   };
      enum { RESET_MODE_GNSS_STOP =  8           };
      enum { RESET_MODE_GNSS_START =  9          };

    CfgRST():
      navBbrMask(0),
      resetMode(0),
      reserved1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->navBbrMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->navBbrMask >> (8 * 1)) & 0xFF;
      offset += sizeof(this->navBbrMask);
      *(outbuffer + offset + 0) = (this->resetMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->resetMode);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->navBbrMask =  ((uint16_t) (*(inbuffer + offset)));
      this->navBbrMask |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->navBbrMask);
      this->resetMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->resetMode);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgRST"; };
    virtual const char * getMD5() override { return "a1d02f583dd30373e526af3ea4137f5d"; };

  };

}
#endif
