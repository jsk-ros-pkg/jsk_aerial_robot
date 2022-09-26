#ifndef _ROS_ublox_msgs_NavSAT_SV_h
#define _ROS_ublox_msgs_NavSAT_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSAT_SV : public ros::Msg
  {
    public:
      typedef uint8_t _gnssId_type;
      _gnssId_type gnssId;
      typedef uint8_t _svId_type;
      _svId_type svId;
      typedef uint8_t _cno_type;
      _cno_type cno;
      typedef int8_t _elev_type;
      _elev_type elev;
      typedef int16_t _azim_type;
      _azim_type azim;
      typedef int16_t _prRes_type;
      _prRes_type prRes;
      typedef uint32_t _flags_type;
      _flags_type flags;
      enum { FLAGS_QUALITY_IND_MASK =  7      };
      enum { QUALITY_IND_NO_SIGNAL =  0                      };
      enum { QUALITY_IND_SEARCHING_SIGNAL =  1               };
      enum { QUALITY_IND_SIGNAL_ACQUIRED =  2                };
      enum { QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE =  3   };
      enum { QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC =  4      };
      enum { QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1 =  5  };
      enum { QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2 =  6  };
      enum { QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 =  7  };
      enum { FLAGS_SV_USED =  8                       };
      enum { FLAGS_HEALTH_MASK =  48                  };
      enum { HEALTH_UNKNOWN =  0                        };
      enum { HEALTH_HEALTHY =  1                        };
      enum { HEALTH_UNHEALTHY =  2                      };
      enum { FLAGS_DIFF_CORR =  64                    };
      enum { FLAGS_SMOOTHED =  128                    };
      enum { FLAGS_ORBIT_SOURCE_MASK =  1792          };
      enum { ORBIT_SOURCE_UNAVAILABLE =  0              };
      enum { ORBIT_SOURCE_EPH =  256                    };
      enum { ORBIT_SOURCE_ALM =  512                    };
      enum { ORBIT_SOURCE_ASSIST_OFFLINE =  768         };
      enum { ORBIT_SOURCE_ASSIST_AUTONOMOUS =  1024     };
      enum { ORBIT_SOURCE_OTHER1 =  1280                };
      enum { ORBIT_SOURCE_OTHER2 =  1536                };
      enum { ORBIT_SOURCE_OTHER3 =  1792                };
      enum { FLAGS_EPH_AVAIL =  2048                  };
      enum { FLAGS_ALM_AVAIL =  4096                  };
      enum { FLAGS_ANO_AVAIL =  8192                  };
      enum { FLAGS_AOP_AVAIL =  16384                 };
      enum { FLAGS_SBAS_CORR_USED =  65536            };
      enum { FLAGS_RTCM_CORR_USED =  131072           };
      enum { FLAGS_PR_CORR_USED =  1048576            };
      enum { FLAGS_CR_CORR_USED =  2097152            };
      enum { FLAGS_DO_CORR_USED =  4194304            };

    NavSAT_SV():
      gnssId(0),
      svId(0),
      cno(0),
      elev(0),
      azim(0),
      prRes(0),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gnssId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gnssId);
      *(outbuffer + offset + 0) = (this->svId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svId);
      *(outbuffer + offset + 0) = (this->cno >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cno);
      union {
        int8_t real;
        uint8_t base;
      } u_elev;
      u_elev.real = this->elev;
      *(outbuffer + offset + 0) = (u_elev.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->elev);
      union {
        int16_t real;
        uint16_t base;
      } u_azim;
      u_azim.real = this->azim;
      *(outbuffer + offset + 0) = (u_azim.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azim.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->azim);
      union {
        int16_t real;
        uint16_t base;
      } u_prRes;
      u_prRes.real = this->prRes;
      *(outbuffer + offset + 0) = (u_prRes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prRes.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->prRes);
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
      this->svId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svId);
      this->cno =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cno);
      union {
        int8_t real;
        uint8_t base;
      } u_elev;
      u_elev.base = 0;
      u_elev.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->elev = u_elev.real;
      offset += sizeof(this->elev);
      union {
        int16_t real;
        uint16_t base;
      } u_azim;
      u_azim.base = 0;
      u_azim.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azim.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->azim = u_azim.real;
      offset += sizeof(this->azim);
      union {
        int16_t real;
        uint16_t base;
      } u_prRes;
      u_prRes.base = 0;
      u_prRes.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prRes.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->prRes = u_prRes.real;
      offset += sizeof(this->prRes);
      this->flags =  ((uint32_t) (*(inbuffer + offset)));
      this->flags |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->flags);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSAT_SV"; };
    virtual const char * getMD5() override { return "902ea92ca9ebf53188dcf1cdef64a9a1"; };

  };

}
#endif
