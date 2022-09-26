#ifndef _ROS_ublox_msgs_NavSTATUS_h
#define _ROS_ublox_msgs_NavSTATUS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSTATUS : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _gpsFix_type;
      _gpsFix_type gpsFix;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint8_t _fixStat_type;
      _fixStat_type fixStat;
      typedef uint8_t _flags2_type;
      _flags2_type flags2;
      typedef uint32_t _ttff_type;
      _ttff_type ttff;
      typedef uint32_t _msss_type;
      _msss_type msss;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  3 };
      enum { GPS_NO_FIX =  0 };
      enum { GPS_DEAD_RECKONING_ONLY =  1 };
      enum { GPS_2D_FIX =  2 };
      enum { GPS_3D_FIX =  3 };
      enum { GPS_GPS_DEAD_RECKONING_COMBINED =  4 };
      enum { GPS_TIME_ONLY_FIX =  5 };
      enum { FLAGS_GPS_FIX_OK =  1       };
      enum { FLAGS_DIFF_SOLN =  2        };
      enum { FLAGS_WKNSET =  4           };
      enum { FLAGS_TOWSET =  8           };
      enum { FIX_STAT_DIFF_CORR_MASK =  1        };
      enum { FIX_STAT_MAP_MATCHING_MASK =  192 };
      enum { MAP_MATCHING_NONE =  0       };
      enum { MAP_MATCHING_VALID =  64     };
      enum { MAP_MATCHING_USED =  128     };
      enum { MAP_MATCHING_DR =  192       };
      enum { FLAGS2_PSM_STATE_MASK =  3 };
      enum { PSM_STATE_ACQUISITION =  0                 };
      enum { PSM_STATE_TRACKING =  1                    };
      enum { PSM_STATE_POWER_OPTIMIZED_TRACKING =  2    };
      enum { PSM_STATE_INACTIVE =  3                    };
      enum { FLAGS2_SPOOF_DET_STATE_MASK =  24 };
      enum { SPOOF_DET_STATE_UNKNOWN =  0     };
      enum { SPOOF_DET_STATE_NONE =  8        };
      enum { SPOOF_DET_STATE_SPOOFING =  16   };
      enum { SPOOF_DET_STATE_MULTIPLE =  24   };

    NavSTATUS():
      iTOW(0),
      gpsFix(0),
      flags(0),
      fixStat(0),
      flags2(0),
      ttff(0),
      msss(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      *(outbuffer + offset + 0) = (this->gpsFix >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gpsFix);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->fixStat >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fixStat);
      *(outbuffer + offset + 0) = (this->flags2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags2);
      *(outbuffer + offset + 0) = (this->ttff >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ttff >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ttff >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ttff >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ttff);
      *(outbuffer + offset + 0) = (this->msss >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->msss >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->msss >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->msss >> (8 * 3)) & 0xFF;
      offset += sizeof(this->msss);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      this->gpsFix =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gpsFix);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->fixStat =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fixStat);
      this->flags2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags2);
      this->ttff =  ((uint32_t) (*(inbuffer + offset)));
      this->ttff |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->ttff |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->ttff |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->ttff);
      this->msss =  ((uint32_t) (*(inbuffer + offset)));
      this->msss |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->msss |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->msss |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->msss);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSTATUS"; };
    virtual const char * getMD5() override { return "68047fb8ca04a038a6b031cd1a908762"; };

  };

}
#endif
