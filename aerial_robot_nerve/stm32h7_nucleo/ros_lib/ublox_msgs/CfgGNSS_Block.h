#ifndef _ROS_ublox_msgs_CfgGNSS_Block_h
#define _ROS_ublox_msgs_CfgGNSS_Block_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgGNSS_Block : public ros::Msg
  {
    public:
      typedef uint8_t _gnssId_type;
      _gnssId_type gnssId;
      typedef uint8_t _resTrkCh_type;
      _resTrkCh_type resTrkCh;
      typedef uint8_t _maxTrkCh_type;
      _maxTrkCh_type maxTrkCh;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef uint32_t _flags_type;
      _flags_type flags;
      enum { GNSS_ID_GPS =  0 };
      enum { GNSS_ID_SBAS =  1 };
      enum { GNSS_ID_GALILEO =  2 };
      enum { GNSS_ID_BEIDOU =  3 };
      enum { GNSS_ID_IMES =  4 };
      enum { GNSS_ID_QZSS =  5 };
      enum { GNSS_ID_GLONASS =  6 };
      enum { RES_TRK_CH_GPS =  8 };
      enum { RES_TRK_CH_QZSS =  0 };
      enum { RES_TRK_CH_SBAS =  0 };
      enum { RES_TRK_CH_GLONASS =  8 };
      enum { MAX_TRK_CH_MAJOR_MIN =  4          };
      enum { MAX_TRK_CH_GPS =  16 };
      enum { MAX_TRK_CH_GLONASS =  14 };
      enum { MAX_TRK_CH_QZSS =  3 };
      enum { MAX_TRK_CH_SBAS =  3 };
      enum { FLAGS_ENABLE =  1                 };
      enum { FLAGS_SIG_CFG_MASK =  16711680    };
      enum { SIG_CFG_GPS_L1CA =  65536         };
      enum { SIG_CFG_SBAS_L1CA =  65536        };
      enum { SIG_CFG_GALILEO_E1OS =  65536     };
      enum { SIG_CFG_BEIDOU_B1I =  65536       };
      enum { SIG_CFG_IMES_L1 =  65536          };
      enum { SIG_CFG_QZSS_L1CA =  65536        };
      enum { SIG_CFG_QZSS_L1SAIF =  262144     };
      enum { SIG_CFG_GLONASS_L1OF =  65536     };

    CfgGNSS_Block():
      gnssId(0),
      resTrkCh(0),
      maxTrkCh(0),
      reserved1(0),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gnssId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gnssId);
      *(outbuffer + offset + 0) = (this->resTrkCh >> (8 * 0)) & 0xFF;
      offset += sizeof(this->resTrkCh);
      *(outbuffer + offset + 0) = (this->maxTrkCh >> (8 * 0)) & 0xFF;
      offset += sizeof(this->maxTrkCh);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->flags >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->flags >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flags);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->gnssId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gnssId);
      this->resTrkCh =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->resTrkCh);
      this->maxTrkCh =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->maxTrkCh);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      this->flags =  ((uint32_t) (*(inbuffer + offset)));
      this->flags |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->flags);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgGNSS_Block"; };
    virtual const char * getMD5() override { return "f786023414ba20add907814936842e32"; };

  };

}
#endif
