#ifndef _ROS_ublox_msgs_CfgCFG_h
#define _ROS_ublox_msgs_CfgCFG_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgCFG : public ros::Msg
  {
    public:
      typedef uint32_t _clearMask_type;
      _clearMask_type clearMask;
      typedef uint32_t _saveMask_type;
      _saveMask_type saveMask;
      typedef uint32_t _loadMask_type;
      _loadMask_type loadMask;
      typedef uint8_t _deviceMask_type;
      _deviceMask_type deviceMask;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  9 };
      enum { MASK_IO_PORT =  1        };
      enum { MASK_MSG_CONF =  2       };
      enum { MASK_INF_MSG =  4        };
      enum { MASK_NAV_CONF =  8       };
      enum { MASK_RXM_CONF =  16      };
      enum { MASK_SEN_CONF =  256     };
      enum { MASK_RINV_CONF =  512    };
      enum { MASK_ANT_CONF =  1024    };
      enum { MASK_LOG_CONF =  2048    };
      enum { MASK_FTS_CONF =  4096    };
      enum { DEV_BBR =  1              };
      enum { DEV_FLASH =  2            };
      enum { DEV_EEPROM =  4           };
      enum { DEV_SPI_FLASH =  16       };

    CfgCFG():
      clearMask(0),
      saveMask(0),
      loadMask(0),
      deviceMask(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->clearMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clearMask >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clearMask >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clearMask >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clearMask);
      *(outbuffer + offset + 0) = (this->saveMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->saveMask >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->saveMask >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->saveMask >> (8 * 3)) & 0xFF;
      offset += sizeof(this->saveMask);
      *(outbuffer + offset + 0) = (this->loadMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->loadMask >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->loadMask >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->loadMask >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loadMask);
      *(outbuffer + offset + 0) = (this->deviceMask >> (8 * 0)) & 0xFF;
      offset += sizeof(this->deviceMask);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->clearMask =  ((uint32_t) (*(inbuffer + offset)));
      this->clearMask |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->clearMask |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->clearMask |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->clearMask);
      this->saveMask =  ((uint32_t) (*(inbuffer + offset)));
      this->saveMask |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->saveMask |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->saveMask |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->saveMask);
      this->loadMask =  ((uint32_t) (*(inbuffer + offset)));
      this->loadMask |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->loadMask |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->loadMask |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->loadMask);
      this->deviceMask =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->deviceMask);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgCFG"; };
    virtual const char * getMD5() override { return "82e4847c642bca8fb5b8af4595e063a1"; };

  };

}
#endif
