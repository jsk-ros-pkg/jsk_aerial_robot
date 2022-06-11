#ifndef _ROS_ublox_msgs_NavRELPOSNED9_h
#define _ROS_ublox_msgs_NavRELPOSNED9_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavRELPOSNED9 : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef uint16_t _refStationId_type;
      _refStationId_type refStationId;
      typedef uint32_t _iTow_type;
      _iTow_type iTow;
      typedef int32_t _relPosN_type;
      _relPosN_type relPosN;
      typedef int32_t _relPosE_type;
      _relPosE_type relPosE;
      typedef int32_t _relPosD_type;
      _relPosD_type relPosD;
      typedef int32_t _relPosLength_type;
      _relPosLength_type relPosLength;
      typedef int32_t _relPosHeading_type;
      _relPosHeading_type relPosHeading;
      uint8_t reserved2[4];
      typedef int8_t _relPosHPN_type;
      _relPosHPN_type relPosHPN;
      typedef int8_t _relPosHPE_type;
      _relPosHPE_type relPosHPE;
      typedef int8_t _relPosHPD_type;
      _relPosHPD_type relPosHPD;
      typedef int8_t _relPosHPLength_type;
      _relPosHPLength_type relPosHPLength;
      typedef uint32_t _accN_type;
      _accN_type accN;
      typedef uint32_t _accE_type;
      _accE_type accE;
      typedef uint32_t _accD_type;
      _accD_type accD;
      typedef uint32_t _accLength_type;
      _accLength_type accLength;
      typedef uint32_t _accHeading_type;
      _accHeading_type accHeading;
      uint8_t reserved3[4];
      typedef uint32_t _flags_type;
      _flags_type flags;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  60 };
      enum { FLAGS_GNSS_FIX_OK =  1       };
      enum { FLAGS_DIFF_SOLN =  2         };
      enum { FLAGS_REL_POS_VALID =  4     };
      enum { FLAGS_CARR_SOLN_MASK =  24   };
      enum { FLAGS_CARR_SOLN_NONE =  0      };
      enum { FLAGS_CARR_SOLN_FLOAT =  8     };
      enum { FLAGS_CARR_SOLN_FIXED =  16    };
      enum { FLAGS_IS_MOVING =  32        };
      enum { FLAGS_REF_POS_MISS =  64     };
      enum { FLAGS_REF_OBS_MISS =  128    };
      enum { FLAGS_REL_POS_HEAD_VALID =  256    };
      enum { FLAGS_REL_POS_NORM =  512    };

    NavRELPOSNED9():
      version(0),
      reserved1(0),
      refStationId(0),
      iTow(0),
      relPosN(0),
      relPosE(0),
      relPosD(0),
      relPosLength(0),
      relPosHeading(0),
      reserved2(),
      relPosHPN(0),
      relPosHPE(0),
      relPosHPD(0),
      relPosHPLength(0),
      accN(0),
      accE(0),
      accD(0),
      accLength(0),
      accHeading(0),
      reserved3(),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->refStationId >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->refStationId >> (8 * 1)) & 0xFF;
      offset += sizeof(this->refStationId);
      *(outbuffer + offset + 0) = (this->iTow >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTow >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTow >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTow >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTow);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosN;
      u_relPosN.real = this->relPosN;
      *(outbuffer + offset + 0) = (u_relPosN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relPosN.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relPosN.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relPosN.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relPosN);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosE;
      u_relPosE.real = this->relPosE;
      *(outbuffer + offset + 0) = (u_relPosE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relPosE.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relPosE.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relPosE.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relPosE);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosD;
      u_relPosD.real = this->relPosD;
      *(outbuffer + offset + 0) = (u_relPosD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relPosD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relPosD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relPosD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relPosD);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosLength;
      u_relPosLength.real = this->relPosLength;
      *(outbuffer + offset + 0) = (u_relPosLength.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relPosLength.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relPosLength.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relPosLength.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relPosLength);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosHeading;
      u_relPosHeading.real = this->relPosHeading;
      *(outbuffer + offset + 0) = (u_relPosHeading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_relPosHeading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_relPosHeading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_relPosHeading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->relPosHeading);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->reserved2[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPN;
      u_relPosHPN.real = this->relPosHPN;
      *(outbuffer + offset + 0) = (u_relPosHPN.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relPosHPN);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPE;
      u_relPosHPE.real = this->relPosHPE;
      *(outbuffer + offset + 0) = (u_relPosHPE.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relPosHPE);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPD;
      u_relPosHPD.real = this->relPosHPD;
      *(outbuffer + offset + 0) = (u_relPosHPD.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relPosHPD);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPLength;
      u_relPosHPLength.real = this->relPosHPLength;
      *(outbuffer + offset + 0) = (u_relPosHPLength.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->relPosHPLength);
      *(outbuffer + offset + 0) = (this->accN >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accN >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accN >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accN >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accN);
      *(outbuffer + offset + 0) = (this->accE >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accE >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accE >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accE >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accE);
      *(outbuffer + offset + 0) = (this->accD >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accD >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accD >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accD >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accD);
      *(outbuffer + offset + 0) = (this->accLength >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accLength >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accLength >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accLength >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accLength);
      *(outbuffer + offset + 0) = (this->accHeading >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accHeading >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accHeading >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accHeading >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accHeading);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->reserved3[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved3[i]);
      }
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->flags >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->flags >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flags);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      this->refStationId =  ((uint16_t) (*(inbuffer + offset)));
      this->refStationId |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->refStationId);
      this->iTow =  ((uint32_t) (*(inbuffer + offset)));
      this->iTow |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTow |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTow |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTow);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosN;
      u_relPosN.base = 0;
      u_relPosN.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relPosN.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relPosN.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relPosN.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relPosN = u_relPosN.real;
      offset += sizeof(this->relPosN);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosE;
      u_relPosE.base = 0;
      u_relPosE.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relPosE.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relPosE.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relPosE.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relPosE = u_relPosE.real;
      offset += sizeof(this->relPosE);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosD;
      u_relPosD.base = 0;
      u_relPosD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relPosD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relPosD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relPosD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relPosD = u_relPosD.real;
      offset += sizeof(this->relPosD);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosLength;
      u_relPosLength.base = 0;
      u_relPosLength.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relPosLength.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relPosLength.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relPosLength.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relPosLength = u_relPosLength.real;
      offset += sizeof(this->relPosLength);
      union {
        int32_t real;
        uint32_t base;
      } u_relPosHeading;
      u_relPosHeading.base = 0;
      u_relPosHeading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_relPosHeading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_relPosHeading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_relPosHeading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->relPosHeading = u_relPosHeading.real;
      offset += sizeof(this->relPosHeading);
      for( uint32_t i = 0; i < 4; i++){
      this->reserved2[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPN;
      u_relPosHPN.base = 0;
      u_relPosHPN.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relPosHPN = u_relPosHPN.real;
      offset += sizeof(this->relPosHPN);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPE;
      u_relPosHPE.base = 0;
      u_relPosHPE.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relPosHPE = u_relPosHPE.real;
      offset += sizeof(this->relPosHPE);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPD;
      u_relPosHPD.base = 0;
      u_relPosHPD.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relPosHPD = u_relPosHPD.real;
      offset += sizeof(this->relPosHPD);
      union {
        int8_t real;
        uint8_t base;
      } u_relPosHPLength;
      u_relPosHPLength.base = 0;
      u_relPosHPLength.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->relPosHPLength = u_relPosHPLength.real;
      offset += sizeof(this->relPosHPLength);
      this->accN =  ((uint32_t) (*(inbuffer + offset)));
      this->accN |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accN |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accN |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accN);
      this->accE =  ((uint32_t) (*(inbuffer + offset)));
      this->accE |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accE |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accE |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accE);
      this->accD =  ((uint32_t) (*(inbuffer + offset)));
      this->accD |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accD |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accD |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accD);
      this->accLength =  ((uint32_t) (*(inbuffer + offset)));
      this->accLength |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accLength |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accLength |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accLength);
      this->accHeading =  ((uint32_t) (*(inbuffer + offset)));
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accHeading);
      for( uint32_t i = 0; i < 4; i++){
      this->reserved3[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved3[i]);
      }
      this->flags =  ((uint32_t) (*(inbuffer + offset)));
      this->flags |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->flags);
     return offset;
    }

    const char * getType(){ return "ublox_msgs/NavRELPOSNED9"; };
    const char * getMD5(){ return "6dedd6e7018b83e2b0ae65fb6873207e"; };

  };

}
#endif
