#ifndef _ROS_ublox_msgs_NavPVT7_h
#define _ROS_ublox_msgs_NavPVT7_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavPVT7 : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint16_t _year_type;
      _year_type year;
      typedef uint8_t _month_type;
      _month_type month;
      typedef uint8_t _day_type;
      _day_type day;
      typedef uint8_t _hour_type;
      _hour_type hour;
      typedef uint8_t _min_type;
      _min_type min;
      typedef uint8_t _sec_type;
      _sec_type sec;
      typedef uint8_t _valid_type;
      _valid_type valid;
      typedef uint32_t _tAcc_type;
      _tAcc_type tAcc;
      typedef int32_t _nano_type;
      _nano_type nano;
      typedef uint8_t _fixType_type;
      _fixType_type fixType;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint8_t _flags2_type;
      _flags2_type flags2;
      typedef uint8_t _numSV_type;
      _numSV_type numSV;
      typedef int32_t _lon_type;
      _lon_type lon;
      typedef int32_t _lat_type;
      _lat_type lat;
      typedef int32_t _height_type;
      _height_type height;
      typedef int32_t _hMSL_type;
      _hMSL_type hMSL;
      typedef uint32_t _hAcc_type;
      _hAcc_type hAcc;
      typedef uint32_t _vAcc_type;
      _vAcc_type vAcc;
      typedef int32_t _velN_type;
      _velN_type velN;
      typedef int32_t _velE_type;
      _velE_type velE;
      typedef int32_t _velD_type;
      _velD_type velD;
      typedef int32_t _gSpeed_type;
      _gSpeed_type gSpeed;
      typedef int32_t _heading_type;
      _heading_type heading;
      typedef uint32_t _sAcc_type;
      _sAcc_type sAcc;
      typedef uint32_t _headAcc_type;
      _headAcc_type headAcc;
      typedef uint16_t _pDOP_type;
      _pDOP_type pDOP;
      uint8_t reserved1[6];
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  7 };
      enum { VALID_DATE =  1             };
      enum { VALID_TIME =  2             };
      enum { VALID_FULLY_RESOLVED =  4   };
      enum { VALID_MAG =  8              };
      enum { FIX_TYPE_NO_FIX =  0 };
      enum { FIX_TYPE_DEAD_RECKONING_ONLY =  1 };
      enum { FIX_TYPE_2D =  2                            };
      enum { FIX_TYPE_3D =  3 };
      enum { FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED =  4  };
      enum { FIX_TYPE_TIME_ONLY =  5                     };
      enum { FLAGS_GNSS_FIX_OK =  1           };
      enum { FLAGS_DIFF_SOLN =  2             };
      enum { FLAGS_PSM_MASK =  28             };
      enum { PSM_OFF =  0                        };
      enum { PSM_ENABLED =  4                    };
      enum { PSM_ACQUIRED =  8                   };
      enum { PSM_TRACKING =  12                  };
      enum { PSM_POWER_OPTIMIZED_TRACKING =  16  };
      enum { PSM_INACTIVE =  20                  };
      enum { FLAGS_HEAD_VEH_VALID =  32          };
      enum { FLAGS_CARRIER_PHASE_MASK =  192  };
      enum { CARRIER_PHASE_NO_SOLUTION =  0      };
      enum { CARRIER_PHASE_FLOAT =  64           };
      enum { CARRIER_PHASE_FIXED =  128          };
      enum { FLAGS2_CONFIRMED_AVAILABLE =  32    };
      enum { FLAGS2_CONFIRMED_DATE =  64         };
      enum { FLAGS2_CONFIRMED_TIME =  128        };

    NavPVT7():
      iTOW(0),
      year(0),
      month(0),
      day(0),
      hour(0),
      min(0),
      sec(0),
      valid(0),
      tAcc(0),
      nano(0),
      fixType(0),
      flags(0),
      flags2(0),
      numSV(0),
      lon(0),
      lat(0),
      height(0),
      hMSL(0),
      hAcc(0),
      vAcc(0),
      velN(0),
      velE(0),
      velD(0),
      gSpeed(0),
      heading(0),
      sAcc(0),
      headAcc(0),
      pDOP(0),
      reserved1()
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
      *(outbuffer + offset + 0) = (this->year >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->year >> (8 * 1)) & 0xFF;
      offset += sizeof(this->year);
      *(outbuffer + offset + 0) = (this->month >> (8 * 0)) & 0xFF;
      offset += sizeof(this->month);
      *(outbuffer + offset + 0) = (this->day >> (8 * 0)) & 0xFF;
      offset += sizeof(this->day);
      *(outbuffer + offset + 0) = (this->hour >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hour);
      *(outbuffer + offset + 0) = (this->min >> (8 * 0)) & 0xFF;
      offset += sizeof(this->min);
      *(outbuffer + offset + 0) = (this->sec >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sec);
      *(outbuffer + offset + 0) = (this->valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      *(outbuffer + offset + 0) = (this->tAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_nano;
      u_nano.real = this->nano;
      *(outbuffer + offset + 0) = (u_nano.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nano.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nano.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nano.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nano);
      *(outbuffer + offset + 0) = (this->fixType >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fixType);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->flags2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags2);
      *(outbuffer + offset + 0) = (this->numSV >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSV);
      union {
        int32_t real;
        uint32_t base;
      } u_lon;
      u_lon.real = this->lon;
      *(outbuffer + offset + 0) = (u_lon.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lon.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lon.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lon.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lon);
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.real = this->lat;
      *(outbuffer + offset + 0) = (u_lat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        int32_t real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.real = this->hMSL;
      *(outbuffer + offset + 0) = (u_hMSL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hMSL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hMSL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hMSL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hMSL);
      *(outbuffer + offset + 0) = (this->hAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->hAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->hAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hAcc);
      *(outbuffer + offset + 0) = (this->vAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_velN;
      u_velN.real = this->velN;
      *(outbuffer + offset + 0) = (u_velN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velN.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velN.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velN.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velN);
      union {
        int32_t real;
        uint32_t base;
      } u_velE;
      u_velE.real = this->velE;
      *(outbuffer + offset + 0) = (u_velE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velE.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velE.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velE.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velE);
      union {
        int32_t real;
        uint32_t base;
      } u_velD;
      u_velD.real = this->velD;
      *(outbuffer + offset + 0) = (u_velD.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velD.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velD.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velD.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velD);
      union {
        int32_t real;
        uint32_t base;
      } u_gSpeed;
      u_gSpeed.real = this->gSpeed;
      *(outbuffer + offset + 0) = (u_gSpeed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gSpeed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gSpeed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gSpeed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      *(outbuffer + offset + 0) = (this->sAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sAcc);
      *(outbuffer + offset + 0) = (this->headAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->headAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->headAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->headAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headAcc);
      *(outbuffer + offset + 0) = (this->pDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pDOP);
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
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
      this->year =  ((uint16_t) (*(inbuffer + offset)));
      this->year |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->year);
      this->month =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->month);
      this->day =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->day);
      this->hour =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->hour);
      this->min =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->min);
      this->sec =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sec);
      this->valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->valid);
      this->tAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_nano;
      u_nano.base = 0;
      u_nano.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nano.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nano.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nano.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nano = u_nano.real;
      offset += sizeof(this->nano);
      this->fixType =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fixType);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->flags2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags2);
      this->numSV =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSV);
      union {
        int32_t real;
        uint32_t base;
      } u_lon;
      u_lon.base = 0;
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lon = u_lon.real;
      offset += sizeof(this->lon);
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.base = 0;
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lat = u_lat.real;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        int32_t real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.base = 0;
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hMSL = u_hMSL.real;
      offset += sizeof(this->hMSL);
      this->hAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->hAcc);
      this->vAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->vAcc);
      union {
        int32_t real;
        uint32_t base;
      } u_velN;
      u_velN.base = 0;
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velN.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velN = u_velN.real;
      offset += sizeof(this->velN);
      union {
        int32_t real;
        uint32_t base;
      } u_velE;
      u_velE.base = 0;
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velE.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velE = u_velE.real;
      offset += sizeof(this->velE);
      union {
        int32_t real;
        uint32_t base;
      } u_velD;
      u_velD.base = 0;
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velD.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velD = u_velD.real;
      offset += sizeof(this->velD);
      union {
        int32_t real;
        uint32_t base;
      } u_gSpeed;
      u_gSpeed.base = 0;
      u_gSpeed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gSpeed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gSpeed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gSpeed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gSpeed = u_gSpeed.real;
      offset += sizeof(this->gSpeed);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      this->sAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sAcc);
      this->headAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->headAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->headAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->headAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->headAcc);
      this->pDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->pDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pDOP);
      for( uint32_t i = 0; i < 6; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavPVT7"; };
    virtual const char * getMD5() override { return "105e506639ea25be87af0076ceb8b057"; };

  };

}
#endif
