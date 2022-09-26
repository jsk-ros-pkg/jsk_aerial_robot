#ifndef _ROS_ublox_msgs_CfgTMODE3_h
#define _ROS_ublox_msgs_CfgTMODE3_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgTMODE3 : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef uint16_t _flags_type;
      _flags_type flags;
      typedef int32_t _ecefXOrLat_type;
      _ecefXOrLat_type ecefXOrLat;
      typedef int32_t _ecefYOrLon_type;
      _ecefYOrLon_type ecefYOrLon;
      typedef int32_t _ecefZOrAlt_type;
      _ecefZOrAlt_type ecefZOrAlt;
      typedef int8_t _ecefXOrLatHP_type;
      _ecefXOrLatHP_type ecefXOrLatHP;
      typedef int8_t _ecefYOrLonHP_type;
      _ecefYOrLonHP_type ecefYOrLonHP;
      typedef int8_t _ecefZOrAltHP_type;
      _ecefZOrAltHP_type ecefZOrAltHP;
      typedef uint8_t _reserved2_type;
      _reserved2_type reserved2;
      typedef uint32_t _fixedPosAcc_type;
      _fixedPosAcc_type fixedPosAcc;
      typedef uint32_t _svinMinDur_type;
      _svinMinDur_type svinMinDur;
      typedef uint32_t _svinAccLimit_type;
      _svinAccLimit_type svinAccLimit;
      uint8_t reserved3[8];
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  113 };
      enum { FLAGS_MODE_MASK =  255       };
      enum { FLAGS_MODE_DISABLED =  0       };
      enum { FLAGS_MODE_SURVEY_IN =  1      };
      enum { FLAGS_MODE_FIXED =  2          };
      enum { FLAGS_LLA =  256             };

    CfgTMODE3():
      version(0),
      reserved1(0),
      flags(0),
      ecefXOrLat(0),
      ecefYOrLon(0),
      ecefZOrAlt(0),
      ecefXOrLatHP(0),
      ecefYOrLonHP(0),
      ecefZOrAltHP(0),
      reserved2(0),
      fixedPosAcc(0),
      svinMinDur(0),
      svinAccLimit(0),
      reserved3()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefXOrLat;
      u_ecefXOrLat.real = this->ecefXOrLat;
      *(outbuffer + offset + 0) = (u_ecefXOrLat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefXOrLat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefXOrLat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefXOrLat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefXOrLat);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefYOrLon;
      u_ecefYOrLon.real = this->ecefYOrLon;
      *(outbuffer + offset + 0) = (u_ecefYOrLon.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefYOrLon.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefYOrLon.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefYOrLon.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefYOrLon);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZOrAlt;
      u_ecefZOrAlt.real = this->ecefZOrAlt;
      *(outbuffer + offset + 0) = (u_ecefZOrAlt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefZOrAlt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefZOrAlt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefZOrAlt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefZOrAlt);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefXOrLatHP;
      u_ecefXOrLatHP.real = this->ecefXOrLatHP;
      *(outbuffer + offset + 0) = (u_ecefXOrLatHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefXOrLatHP);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefYOrLonHP;
      u_ecefYOrLonHP.real = this->ecefYOrLonHP;
      *(outbuffer + offset + 0) = (u_ecefYOrLonHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefYOrLonHP);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefZOrAltHP;
      u_ecefZOrAltHP.real = this->ecefZOrAltHP;
      *(outbuffer + offset + 0) = (u_ecefZOrAltHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ecefZOrAltHP);
      *(outbuffer + offset + 0) = (this->reserved2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2);
      *(outbuffer + offset + 0) = (this->fixedPosAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fixedPosAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fixedPosAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fixedPosAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fixedPosAcc);
      *(outbuffer + offset + 0) = (this->svinMinDur >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->svinMinDur >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->svinMinDur >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->svinMinDur >> (8 * 3)) & 0xFF;
      offset += sizeof(this->svinMinDur);
      *(outbuffer + offset + 0) = (this->svinAccLimit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->svinAccLimit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->svinAccLimit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->svinAccLimit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->svinAccLimit);
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->reserved3[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved3[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefXOrLat;
      u_ecefXOrLat.base = 0;
      u_ecefXOrLat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefXOrLat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefXOrLat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefXOrLat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefXOrLat = u_ecefXOrLat.real;
      offset += sizeof(this->ecefXOrLat);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefYOrLon;
      u_ecefYOrLon.base = 0;
      u_ecefYOrLon.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefYOrLon.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefYOrLon.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefYOrLon.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefYOrLon = u_ecefYOrLon.real;
      offset += sizeof(this->ecefYOrLon);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZOrAlt;
      u_ecefZOrAlt.base = 0;
      u_ecefZOrAlt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefZOrAlt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefZOrAlt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefZOrAlt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefZOrAlt = u_ecefZOrAlt.real;
      offset += sizeof(this->ecefZOrAlt);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefXOrLatHP;
      u_ecefXOrLatHP.base = 0;
      u_ecefXOrLatHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefXOrLatHP = u_ecefXOrLatHP.real;
      offset += sizeof(this->ecefXOrLatHP);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefYOrLonHP;
      u_ecefYOrLonHP.base = 0;
      u_ecefYOrLonHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefYOrLonHP = u_ecefYOrLonHP.real;
      offset += sizeof(this->ecefYOrLonHP);
      union {
        int8_t real;
        uint8_t base;
      } u_ecefZOrAltHP;
      u_ecefZOrAltHP.base = 0;
      u_ecefZOrAltHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ecefZOrAltHP = u_ecefZOrAltHP.real;
      offset += sizeof(this->ecefZOrAltHP);
      this->reserved2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2);
      this->fixedPosAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->fixedPosAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->fixedPosAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->fixedPosAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->fixedPosAcc);
      this->svinMinDur =  ((uint32_t) (*(inbuffer + offset)));
      this->svinMinDur |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->svinMinDur |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->svinMinDur |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->svinMinDur);
      this->svinAccLimit =  ((uint32_t) (*(inbuffer + offset)));
      this->svinAccLimit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->svinAccLimit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->svinAccLimit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->svinAccLimit);
      for( uint32_t i = 0; i < 8; i++){
      this->reserved3[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved3[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgTMODE3"; };
    virtual const char * getMD5() override { return "818be20c97f2b940a885faaabc0d98a1"; };

  };

}
#endif
