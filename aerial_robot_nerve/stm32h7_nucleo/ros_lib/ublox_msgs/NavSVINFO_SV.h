#ifndef _ROS_ublox_msgs_NavSVINFO_SV_h
#define _ROS_ublox_msgs_NavSVINFO_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSVINFO_SV : public ros::Msg
  {
    public:
      typedef uint8_t _chn_type;
      _chn_type chn;
      typedef uint8_t _svid_type;
      _svid_type svid;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint8_t _quality_type;
      _quality_type quality;
      typedef uint8_t _cno_type;
      _cno_type cno;
      typedef int8_t _elev_type;
      _elev_type elev;
      typedef int16_t _azim_type;
      _azim_type azim;
      typedef int32_t _prRes_type;
      _prRes_type prRes;
      enum { FLAGS_SV_USED =  1                      };
      enum { FLAGS_DIFF_CORR =  2                    };
      enum { FLAGS_ORBIT_AVAIL =  4                  };
      enum { FLAGS_ORBIT_EPH =  8                    };
      enum { FLAGS_UNHEALTHY =  16                   };
      enum { FLAGS_ORBIT_ALM =  32                   };
      enum { FLAGS_ORBIT_AOP =  64                   };
      enum { FLAGS_SMOOTHED =  128                   };
      enum { QUALITY_IDLE =  0                       };
      enum { QUALITY_SEARCHING =  1                  };
      enum { QUALITY_ACQUIRED =  2                    };
      enum { QUALITY_DETECTED =  3                   };
      enum { QUALITY_CODE_LOCK =  4                  };
      enum { QUALITY_CODE_AND_CARRIER_LOCKED1 =  5   };
      enum { QUALITY_CODE_AND_CARRIER_LOCKED2 =  6   };
      enum { QUALITY_CODE_AND_CARRIER_LOCKED3 =  7   };

    NavSVINFO_SV():
      chn(0),
      svid(0),
      flags(0),
      quality(0),
      cno(0),
      elev(0),
      azim(0),
      prRes(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->chn >> (8 * 0)) & 0xFF;
      offset += sizeof(this->chn);
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->quality >> (8 * 0)) & 0xFF;
      offset += sizeof(this->quality);
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
        int32_t real;
        uint32_t base;
      } u_prRes;
      u_prRes.real = this->prRes;
      *(outbuffer + offset + 0) = (u_prRes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prRes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prRes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prRes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->prRes);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->chn =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->chn);
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->quality =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->quality);
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
        int32_t real;
        uint32_t base;
      } u_prRes;
      u_prRes.base = 0;
      u_prRes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prRes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prRes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prRes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->prRes = u_prRes.real;
      offset += sizeof(this->prRes);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSVINFO_SV"; };
    virtual const char * getMD5() override { return "fd3d8150f431e87f504da9aafff163a1"; };

  };

}
#endif
