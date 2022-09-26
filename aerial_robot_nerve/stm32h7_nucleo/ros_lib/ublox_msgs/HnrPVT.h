#ifndef _ROS_ublox_msgs_HnrPVT_h
#define _ROS_ublox_msgs_HnrPVT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class HnrPVT : public ros::Msg
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
      typedef int32_t _nano_type;
      _nano_type nano;
      typedef uint8_t _gpsFix_type;
      _gpsFix_type gpsFix;
      typedef uint8_t _flags_type;
      _flags_type flags;
      uint8_t reserved0[2];
      typedef int32_t _lon_type;
      _lon_type lon;
      typedef int32_t _lat_type;
      _lat_type lat;
      typedef int32_t _height_type;
      _height_type height;
      typedef int32_t _hMSL_type;
      _hMSL_type hMSL;
      typedef int32_t _gSpeed_type;
      _gSpeed_type gSpeed;
      typedef int32_t _speed_type;
      _speed_type speed;
      typedef int32_t _headMot_type;
      _headMot_type headMot;
      typedef int32_t _headVeh_type;
      _headVeh_type headVeh;
      typedef uint32_t _hAcc_type;
      _hAcc_type hAcc;
      typedef uint32_t _vAcc_type;
      _vAcc_type vAcc;
      typedef uint32_t _sAcc_type;
      _sAcc_type sAcc;
      typedef uint32_t _headAcc_type;
      _headAcc_type headAcc;
      uint8_t reserved1[4];
      enum { CLASS_ID =  40 };
      enum { MESSAGE_ID =  0 };
      enum { VALID_DATE =  1             };
      enum { VALID_TIME =  2             };
      enum { VALID_FULLY_RESOLVED =  4   };
      enum { VALID_MAG =  8              };
      enum { FIX_TYPE_NO_FIX =  0 };
      enum { FIX_TYPE_DEAD_RECKONING_ONLY =  1 };
      enum { FIX_TYPE_2D =  2                            };
      enum { FIX_TYPE_3D =  3 };
      enum { FIX_TYPE_GPS_DEAD_RECKONING_COMBINED =  4   };
      enum { FIX_TYPE_TIME_ONLY =  5                     };
      enum { FLAGS_GNSS_FIX_OK =  1           };
      enum { FLAGS_DIFF_SOLN =  2             };
      enum { FLAGS_WKN_SET =  4               };
      enum { FLAGS_TOW_SET =  8               };
      enum { FLAGS_HEAD_VEH_VALID =  32       };

    HnrPVT():
      iTOW(0),
      year(0),
      month(0),
      day(0),
      hour(0),
      min(0),
      sec(0),
      valid(0),
      nano(0),
      gpsFix(0),
      flags(0),
      reserved0(),
      lon(0),
      lat(0),
      height(0),
      hMSL(0),
      gSpeed(0),
      speed(0),
      headMot(0),
      headVeh(0),
      hAcc(0),
      vAcc(0),
      sAcc(0),
      headAcc(0),
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
      *(outbuffer + offset + 0) = (this->gpsFix >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gpsFix);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
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
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        int32_t real;
        uint32_t base;
      } u_headMot;
      u_headMot.real = this->headMot;
      *(outbuffer + offset + 0) = (u_headMot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_headMot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_headMot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_headMot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headMot);
      union {
        int32_t real;
        uint32_t base;
      } u_headVeh;
      u_headVeh.real = this->headVeh;
      *(outbuffer + offset + 0) = (u_headVeh.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_headVeh.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_headVeh.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_headVeh.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headVeh);
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
      for( uint32_t i = 0; i < 4; i++){
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
      this->gpsFix =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gpsFix);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
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
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        int32_t real;
        uint32_t base;
      } u_headMot;
      u_headMot.base = 0;
      u_headMot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_headMot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_headMot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_headMot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->headMot = u_headMot.real;
      offset += sizeof(this->headMot);
      union {
        int32_t real;
        uint32_t base;
      } u_headVeh;
      u_headVeh.base = 0;
      u_headVeh.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_headVeh.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_headVeh.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_headVeh.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->headVeh = u_headVeh.real;
      offset += sizeof(this->headVeh);
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
      for( uint32_t i = 0; i < 4; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/HnrPVT"; };
    virtual const char * getMD5() override { return "1803c15f4ff593453ea993864baf0f33"; };

  };

}
#endif
