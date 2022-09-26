#ifndef _ROS_ublox_msgs_CfgNAV5_h
#define _ROS_ublox_msgs_CfgNAV5_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgNAV5 : public ros::Msg
  {
    public:
      typedef uint16_t _mask_type;
      _mask_type mask;
      typedef uint8_t _dynModel_type;
      _dynModel_type dynModel;
      typedef uint8_t _fixMode_type;
      _fixMode_type fixMode;
      typedef int32_t _fixedAlt_type;
      _fixedAlt_type fixedAlt;
      typedef uint32_t _fixedAltVar_type;
      _fixedAltVar_type fixedAltVar;
      typedef int8_t _minElev_type;
      _minElev_type minElev;
      typedef uint8_t _drLimit_type;
      _drLimit_type drLimit;
      typedef uint16_t _pDop_type;
      _pDop_type pDop;
      typedef uint16_t _tDop_type;
      _tDop_type tDop;
      typedef uint16_t _pAcc_type;
      _pAcc_type pAcc;
      typedef uint16_t _tAcc_type;
      _tAcc_type tAcc;
      typedef uint8_t _staticHoldThresh_type;
      _staticHoldThresh_type staticHoldThresh;
      typedef uint8_t _dgnssTimeOut_type;
      _dgnssTimeOut_type dgnssTimeOut;
      typedef uint8_t _cnoThreshNumSvs_type;
      _cnoThreshNumSvs_type cnoThreshNumSvs;
      typedef uint8_t _cnoThresh_type;
      _cnoThresh_type cnoThresh;
      uint8_t reserved1[2];
      typedef uint16_t _staticHoldMaxDist_type;
      _staticHoldMaxDist_type staticHoldMaxDist;
      typedef uint8_t _utcStandard_type;
      _utcStandard_type utcStandard;
      uint8_t reserved2[5];
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  36 };
      enum { MASK_DYN =  1                  };
      enum { MASK_MIN_EL =  2               };
      enum { MASK_FIX_MODE =  4             };
      enum { MASK_DR_LIM =  8               };
      enum { MASK_POS_MASK =  16            };
      enum { MASK_TIME_MASK =  32           };
      enum { MASK_STATIC_HOLD_MASK =  64    };
      enum { MASK_DGPS_MASK =  128          };
      enum { MASK_CNO =  256                };
      enum { MASK_UTC =  1024               };
      enum { DYN_MODEL_PORTABLE =  0         };
      enum { DYN_MODEL_STATIONARY =  2       };
      enum { DYN_MODEL_PEDESTRIAN =  3       };
      enum { DYN_MODEL_AUTOMOTIVE =  4       };
      enum { DYN_MODEL_SEA =  5              };
      enum { DYN_MODEL_AIRBORNE_1G =  6      };
      enum { DYN_MODEL_AIRBORNE_2G =  7      };
      enum { DYN_MODEL_AIRBORNE_4G =  8      };
      enum { DYN_MODEL_WRIST_WATCH =  9      };
      enum { DYN_MODEL_BIKE =  10            };
      enum { FIX_MODE_2D_ONLY =  1           };
      enum { FIX_MODE_3D_ONLY =  2           };
      enum { FIX_MODE_AUTO =  3              };
      enum { UTC_STANDARD_AUTOMATIC =  0  };
      enum { UTC_STANDARD_GPS =  3        };
      enum { UTC_STANDARD_GLONASS =  6    };
      enum { UTC_STANDARD_BEIDOU =  7     };

    CfgNAV5():
      mask(0),
      dynModel(0),
      fixMode(0),
      fixedAlt(0),
      fixedAltVar(0),
      minElev(0),
      drLimit(0),
      pDop(0),
      tDop(0),
      pAcc(0),
      tAcc(0),
      staticHoldThresh(0),
      dgnssTimeOut(0),
      cnoThreshNumSvs(0),
      cnoThresh(0),
      reserved1(),
      staticHoldMaxDist(0),
      utcStandard(0),
      reserved2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mask >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mask);
      *(outbuffer + offset + 0) = (this->dynModel >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dynModel);
      *(outbuffer + offset + 0) = (this->fixMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fixMode);
      union {
        int32_t real;
        uint32_t base;
      } u_fixedAlt;
      u_fixedAlt.real = this->fixedAlt;
      *(outbuffer + offset + 0) = (u_fixedAlt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fixedAlt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fixedAlt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fixedAlt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fixedAlt);
      *(outbuffer + offset + 0) = (this->fixedAltVar >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fixedAltVar >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fixedAltVar >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fixedAltVar >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fixedAltVar);
      union {
        int8_t real;
        uint8_t base;
      } u_minElev;
      u_minElev.real = this->minElev;
      *(outbuffer + offset + 0) = (u_minElev.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->minElev);
      *(outbuffer + offset + 0) = (this->drLimit >> (8 * 0)) & 0xFF;
      offset += sizeof(this->drLimit);
      *(outbuffer + offset + 0) = (this->pDop >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pDop >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pDop);
      *(outbuffer + offset + 0) = (this->tDop >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tDop >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tDop);
      *(outbuffer + offset + 0) = (this->pAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pAcc >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pAcc);
      *(outbuffer + offset + 0) = (this->tAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tAcc >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tAcc);
      *(outbuffer + offset + 0) = (this->staticHoldThresh >> (8 * 0)) & 0xFF;
      offset += sizeof(this->staticHoldThresh);
      *(outbuffer + offset + 0) = (this->dgnssTimeOut >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dgnssTimeOut);
      *(outbuffer + offset + 0) = (this->cnoThreshNumSvs >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cnoThreshNumSvs);
      *(outbuffer + offset + 0) = (this->cnoThresh >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cnoThresh);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->staticHoldMaxDist >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->staticHoldMaxDist >> (8 * 1)) & 0xFF;
      offset += sizeof(this->staticHoldMaxDist);
      *(outbuffer + offset + 0) = (this->utcStandard >> (8 * 0)) & 0xFF;
      offset += sizeof(this->utcStandard);
      for( uint32_t i = 0; i < 5; i++){
      *(outbuffer + offset + 0) = (this->reserved2[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->mask =  ((uint16_t) (*(inbuffer + offset)));
      this->mask |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->mask);
      this->dynModel =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dynModel);
      this->fixMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fixMode);
      union {
        int32_t real;
        uint32_t base;
      } u_fixedAlt;
      u_fixedAlt.base = 0;
      u_fixedAlt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fixedAlt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fixedAlt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fixedAlt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fixedAlt = u_fixedAlt.real;
      offset += sizeof(this->fixedAlt);
      this->fixedAltVar =  ((uint32_t) (*(inbuffer + offset)));
      this->fixedAltVar |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fixedAltVar |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fixedAltVar |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fixedAltVar);
      union {
        int8_t real;
        uint8_t base;
      } u_minElev;
      u_minElev.base = 0;
      u_minElev.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->minElev = u_minElev.real;
      offset += sizeof(this->minElev);
      this->drLimit =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->drLimit);
      this->pDop =  ((uint16_t) (*(inbuffer + offset)));
      this->pDop |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pDop);
      this->tDop =  ((uint16_t) (*(inbuffer + offset)));
      this->tDop |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tDop);
      this->pAcc =  ((uint16_t) (*(inbuffer + offset)));
      this->pAcc |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pAcc);
      this->tAcc =  ((uint16_t) (*(inbuffer + offset)));
      this->tAcc |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tAcc);
      this->staticHoldThresh =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->staticHoldThresh);
      this->dgnssTimeOut =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dgnssTimeOut);
      this->cnoThreshNumSvs =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cnoThreshNumSvs);
      this->cnoThresh =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cnoThresh);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->staticHoldMaxDist =  ((uint16_t) (*(inbuffer + offset)));
      this->staticHoldMaxDist |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->staticHoldMaxDist);
      this->utcStandard =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->utcStandard);
      for( uint32_t i = 0; i < 5; i++){
      this->reserved2[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgNAV5"; };
    virtual const char * getMD5() override { return "ac3dd34c30655c54e2d11a724de7dcd6"; };

  };

}
#endif
