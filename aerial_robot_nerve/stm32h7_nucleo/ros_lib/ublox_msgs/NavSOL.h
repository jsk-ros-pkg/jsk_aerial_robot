#ifndef _ROS_ublox_msgs_NavSOL_h
#define _ROS_ublox_msgs_NavSOL_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSOL : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _fTOW_type;
      _fTOW_type fTOW;
      typedef int16_t _week_type;
      _week_type week;
      typedef uint8_t _gpsFix_type;
      _gpsFix_type gpsFix;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef int32_t _ecefX_type;
      _ecefX_type ecefX;
      typedef int32_t _ecefY_type;
      _ecefY_type ecefY;
      typedef int32_t _ecefZ_type;
      _ecefZ_type ecefZ;
      typedef uint32_t _pAcc_type;
      _pAcc_type pAcc;
      typedef int32_t _ecefVX_type;
      _ecefVX_type ecefVX;
      typedef int32_t _ecefVY_type;
      _ecefVY_type ecefVY;
      typedef int32_t _ecefVZ_type;
      _ecefVZ_type ecefVZ;
      typedef uint32_t _sAcc_type;
      _sAcc_type sAcc;
      typedef uint16_t _pDOP_type;
      _pDOP_type pDOP;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef uint8_t _numSV_type;
      _numSV_type numSV;
      typedef uint32_t _reserved2_type;
      _reserved2_type reserved2;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  6 };
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

    NavSOL():
      iTOW(0),
      fTOW(0),
      week(0),
      gpsFix(0),
      flags(0),
      ecefX(0),
      ecefY(0),
      ecefZ(0),
      pAcc(0),
      ecefVX(0),
      ecefVY(0),
      ecefVZ(0),
      sAcc(0),
      pDOP(0),
      reserved1(0),
      numSV(0),
      reserved2(0)
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
      union {
        int32_t real;
        uint32_t base;
      } u_fTOW;
      u_fTOW.real = this->fTOW;
      *(outbuffer + offset + 0) = (u_fTOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fTOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fTOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fTOW.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.real = this->week;
      *(outbuffer + offset + 0) = (u_week.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_week.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->week);
      *(outbuffer + offset + 0) = (this->gpsFix >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gpsFix);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefX;
      u_ecefX.real = this->ecefX;
      *(outbuffer + offset + 0) = (u_ecefX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.real = this->ecefY;
      *(outbuffer + offset + 0) = (u_ecefY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.real = this->ecefZ;
      *(outbuffer + offset + 0) = (u_ecefZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefZ);
      *(outbuffer + offset + 0) = (this->pAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVX;
      u_ecefVX.real = this->ecefVX;
      *(outbuffer + offset + 0) = (u_ecefVX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVY;
      u_ecefVY.real = this->ecefVY;
      *(outbuffer + offset + 0) = (u_ecefVY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVZ;
      u_ecefVZ.real = this->ecefVZ;
      *(outbuffer + offset + 0) = (u_ecefVZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ecefVZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ecefVZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ecefVZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ecefVZ);
      *(outbuffer + offset + 0) = (this->sAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sAcc);
      *(outbuffer + offset + 0) = (this->pDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pDOP);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->numSV >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSV);
      *(outbuffer + offset + 0) = (this->reserved2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserved2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reserved2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reserved2 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reserved2);
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
      union {
        int32_t real;
        uint32_t base;
      } u_fTOW;
      u_fTOW.base = 0;
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fTOW.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fTOW = u_fTOW.real;
      offset += sizeof(this->fTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.base = 0;
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->week = u_week.real;
      offset += sizeof(this->week);
      this->gpsFix =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gpsFix);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefX;
      u_ecefX.base = 0;
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefX = u_ecefX.real;
      offset += sizeof(this->ecefX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefY;
      u_ecefY.base = 0;
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefY = u_ecefY.real;
      offset += sizeof(this->ecefY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefZ;
      u_ecefZ.base = 0;
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefZ = u_ecefZ.real;
      offset += sizeof(this->ecefZ);
      this->pAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVX;
      u_ecefVX.base = 0;
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVX = u_ecefVX.real;
      offset += sizeof(this->ecefVX);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVY;
      u_ecefVY.base = 0;
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVY = u_ecefVY.real;
      offset += sizeof(this->ecefVY);
      union {
        int32_t real;
        uint32_t base;
      } u_ecefVZ;
      u_ecefVZ.base = 0;
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ecefVZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ecefVZ = u_ecefVZ.real;
      offset += sizeof(this->ecefVZ);
      this->sAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sAcc);
      this->pDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->pDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pDOP);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      this->numSV =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSV);
      this->reserved2 =  ((uint32_t) (*(inbuffer + offset)));
      this->reserved2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->reserved2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->reserved2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->reserved2);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSOL"; };
    virtual const char * getMD5() override { return "fbabf6cbcea22aacacf0f8dbb86da71f"; };

  };

}
#endif
