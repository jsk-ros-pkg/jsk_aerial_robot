#ifndef _ROS_ublox_msgs_AidHUI_h
#define _ROS_ublox_msgs_AidHUI_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class AidHUI : public ros::Msg
  {
    public:
      typedef uint32_t _health_type;
      _health_type health;
      typedef double _utcA0_type;
      _utcA0_type utcA0;
      typedef double _utcA1_type;
      _utcA1_type utcA1;
      typedef int32_t _utcTOW_type;
      _utcTOW_type utcTOW;
      typedef int16_t _utcWNT_type;
      _utcWNT_type utcWNT;
      typedef int16_t _utcLS_type;
      _utcLS_type utcLS;
      typedef int16_t _utcWNF_type;
      _utcWNF_type utcWNF;
      typedef int16_t _utcDN_type;
      _utcDN_type utcDN;
      typedef int16_t _utcLSF_type;
      _utcLSF_type utcLSF;
      typedef int16_t _utcSpare_type;
      _utcSpare_type utcSpare;
      typedef float _klobA0_type;
      _klobA0_type klobA0;
      typedef float _klobA1_type;
      _klobA1_type klobA1;
      typedef float _klobA2_type;
      _klobA2_type klobA2;
      typedef float _klobA3_type;
      _klobA3_type klobA3;
      typedef float _klobB0_type;
      _klobB0_type klobB0;
      typedef float _klobB1_type;
      _klobB1_type klobB1;
      typedef float _klobB2_type;
      _klobB2_type klobB2;
      typedef float _klobB3_type;
      _klobB3_type klobB3;
      typedef uint32_t _flags_type;
      _flags_type flags;
      enum { CLASS_ID =  11 };
      enum { MESSAGE_ID =  2 };
      enum { FLAGS_HEALTH =  1      };
      enum { FLAGS_UTC =  2         };
      enum { FLAGS_KLOB =  4        };

    AidHUI():
      health(0),
      utcA0(0),
      utcA1(0),
      utcTOW(0),
      utcWNT(0),
      utcLS(0),
      utcWNF(0),
      utcDN(0),
      utcLSF(0),
      utcSpare(0),
      klobA0(0),
      klobA1(0),
      klobA2(0),
      klobA3(0),
      klobB0(0),
      klobB1(0),
      klobB2(0),
      klobB3(0),
      flags(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->health >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->health >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->health >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->health >> (8 * 3)) & 0xFF;
      offset += sizeof(this->health);
      union {
        double real;
        uint64_t base;
      } u_utcA0;
      u_utcA0.real = this->utcA0;
      *(outbuffer + offset + 0) = (u_utcA0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcA0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_utcA0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_utcA0.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_utcA0.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_utcA0.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_utcA0.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_utcA0.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->utcA0);
      union {
        double real;
        uint64_t base;
      } u_utcA1;
      u_utcA1.real = this->utcA1;
      *(outbuffer + offset + 0) = (u_utcA1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcA1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_utcA1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_utcA1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_utcA1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_utcA1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_utcA1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_utcA1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->utcA1);
      union {
        int32_t real;
        uint32_t base;
      } u_utcTOW;
      u_utcTOW.real = this->utcTOW;
      *(outbuffer + offset + 0) = (u_utcTOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcTOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_utcTOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_utcTOW.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->utcTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_utcWNT;
      u_utcWNT.real = this->utcWNT;
      *(outbuffer + offset + 0) = (u_utcWNT.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcWNT.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcWNT);
      union {
        int16_t real;
        uint16_t base;
      } u_utcLS;
      u_utcLS.real = this->utcLS;
      *(outbuffer + offset + 0) = (u_utcLS.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcLS.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcLS);
      union {
        int16_t real;
        uint16_t base;
      } u_utcWNF;
      u_utcWNF.real = this->utcWNF;
      *(outbuffer + offset + 0) = (u_utcWNF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcWNF.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcWNF);
      union {
        int16_t real;
        uint16_t base;
      } u_utcDN;
      u_utcDN.real = this->utcDN;
      *(outbuffer + offset + 0) = (u_utcDN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcDN.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcDN);
      union {
        int16_t real;
        uint16_t base;
      } u_utcLSF;
      u_utcLSF.real = this->utcLSF;
      *(outbuffer + offset + 0) = (u_utcLSF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcLSF.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcLSF);
      union {
        int16_t real;
        uint16_t base;
      } u_utcSpare;
      u_utcSpare.real = this->utcSpare;
      *(outbuffer + offset + 0) = (u_utcSpare.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_utcSpare.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->utcSpare);
      union {
        float real;
        uint32_t base;
      } u_klobA0;
      u_klobA0.real = this->klobA0;
      *(outbuffer + offset + 0) = (u_klobA0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobA0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobA0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobA0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobA0);
      union {
        float real;
        uint32_t base;
      } u_klobA1;
      u_klobA1.real = this->klobA1;
      *(outbuffer + offset + 0) = (u_klobA1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobA1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobA1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobA1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobA1);
      union {
        float real;
        uint32_t base;
      } u_klobA2;
      u_klobA2.real = this->klobA2;
      *(outbuffer + offset + 0) = (u_klobA2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobA2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobA2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobA2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobA2);
      union {
        float real;
        uint32_t base;
      } u_klobA3;
      u_klobA3.real = this->klobA3;
      *(outbuffer + offset + 0) = (u_klobA3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobA3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobA3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobA3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobA3);
      union {
        float real;
        uint32_t base;
      } u_klobB0;
      u_klobB0.real = this->klobB0;
      *(outbuffer + offset + 0) = (u_klobB0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobB0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobB0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobB0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobB0);
      union {
        float real;
        uint32_t base;
      } u_klobB1;
      u_klobB1.real = this->klobB1;
      *(outbuffer + offset + 0) = (u_klobB1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobB1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobB1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobB1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobB1);
      union {
        float real;
        uint32_t base;
      } u_klobB2;
      u_klobB2.real = this->klobB2;
      *(outbuffer + offset + 0) = (u_klobB2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobB2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobB2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobB2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobB2);
      union {
        float real;
        uint32_t base;
      } u_klobB3;
      u_klobB3.real = this->klobB3;
      *(outbuffer + offset + 0) = (u_klobB3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_klobB3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_klobB3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_klobB3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->klobB3);
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
      this->health =  ((uint32_t) (*(inbuffer + offset)));
      this->health |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->health |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->health |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->health);
      union {
        double real;
        uint64_t base;
      } u_utcA0;
      u_utcA0.base = 0;
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_utcA0.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->utcA0 = u_utcA0.real;
      offset += sizeof(this->utcA0);
      union {
        double real;
        uint64_t base;
      } u_utcA1;
      u_utcA1.base = 0;
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_utcA1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->utcA1 = u_utcA1.real;
      offset += sizeof(this->utcA1);
      union {
        int32_t real;
        uint32_t base;
      } u_utcTOW;
      u_utcTOW.base = 0;
      u_utcTOW.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcTOW.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_utcTOW.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_utcTOW.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->utcTOW = u_utcTOW.real;
      offset += sizeof(this->utcTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_utcWNT;
      u_utcWNT.base = 0;
      u_utcWNT.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcWNT.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcWNT = u_utcWNT.real;
      offset += sizeof(this->utcWNT);
      union {
        int16_t real;
        uint16_t base;
      } u_utcLS;
      u_utcLS.base = 0;
      u_utcLS.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcLS.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcLS = u_utcLS.real;
      offset += sizeof(this->utcLS);
      union {
        int16_t real;
        uint16_t base;
      } u_utcWNF;
      u_utcWNF.base = 0;
      u_utcWNF.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcWNF.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcWNF = u_utcWNF.real;
      offset += sizeof(this->utcWNF);
      union {
        int16_t real;
        uint16_t base;
      } u_utcDN;
      u_utcDN.base = 0;
      u_utcDN.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcDN.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcDN = u_utcDN.real;
      offset += sizeof(this->utcDN);
      union {
        int16_t real;
        uint16_t base;
      } u_utcLSF;
      u_utcLSF.base = 0;
      u_utcLSF.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcLSF.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcLSF = u_utcLSF.real;
      offset += sizeof(this->utcLSF);
      union {
        int16_t real;
        uint16_t base;
      } u_utcSpare;
      u_utcSpare.base = 0;
      u_utcSpare.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_utcSpare.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->utcSpare = u_utcSpare.real;
      offset += sizeof(this->utcSpare);
      union {
        float real;
        uint32_t base;
      } u_klobA0;
      u_klobA0.base = 0;
      u_klobA0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobA0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobA0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobA0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobA0 = u_klobA0.real;
      offset += sizeof(this->klobA0);
      union {
        float real;
        uint32_t base;
      } u_klobA1;
      u_klobA1.base = 0;
      u_klobA1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobA1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobA1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobA1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobA1 = u_klobA1.real;
      offset += sizeof(this->klobA1);
      union {
        float real;
        uint32_t base;
      } u_klobA2;
      u_klobA2.base = 0;
      u_klobA2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobA2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobA2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobA2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobA2 = u_klobA2.real;
      offset += sizeof(this->klobA2);
      union {
        float real;
        uint32_t base;
      } u_klobA3;
      u_klobA3.base = 0;
      u_klobA3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobA3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobA3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobA3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobA3 = u_klobA3.real;
      offset += sizeof(this->klobA3);
      union {
        float real;
        uint32_t base;
      } u_klobB0;
      u_klobB0.base = 0;
      u_klobB0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobB0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobB0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobB0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobB0 = u_klobB0.real;
      offset += sizeof(this->klobB0);
      union {
        float real;
        uint32_t base;
      } u_klobB1;
      u_klobB1.base = 0;
      u_klobB1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobB1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobB1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobB1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobB1 = u_klobB1.real;
      offset += sizeof(this->klobB1);
      union {
        float real;
        uint32_t base;
      } u_klobB2;
      u_klobB2.base = 0;
      u_klobB2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobB2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobB2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobB2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobB2 = u_klobB2.real;
      offset += sizeof(this->klobB2);
      union {
        float real;
        uint32_t base;
      } u_klobB3;
      u_klobB3.base = 0;
      u_klobB3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_klobB3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_klobB3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_klobB3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->klobB3 = u_klobB3.real;
      offset += sizeof(this->klobB3);
      this->flags =  ((uint32_t) (*(inbuffer + offset)));
      this->flags |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->flags |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->flags);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/AidHUI"; };
    virtual const char * getMD5() override { return "60cd4ce940333cb9b38edd447085ce5c"; };

  };

}
#endif
