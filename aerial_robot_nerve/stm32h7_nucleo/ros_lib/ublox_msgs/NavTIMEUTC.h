#ifndef _ROS_ublox_msgs_NavTIMEUTC_h
#define _ROS_ublox_msgs_NavTIMEUTC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavTIMEUTC : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint32_t _tAcc_type;
      _tAcc_type tAcc;
      typedef int32_t _nano_type;
      _nano_type nano;
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
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  33 };
      enum { VALID_TOW =  1          };
      enum { VALID_WKN =  2          };
      enum { VALID_UTC =  4          };
      enum { VALID_UTC_STANDARD_MASK =  240   };
      enum { UTC_STANDARD_NOT_AVAILABLE =  0     };
      enum { UTC_STANDARD_CRL =  16              };
      enum { UTC_STANDARD_NIST =  32             };
      enum { UTC_STANDARD_USNO =  48             };
      enum { UTC_STANDARD_BIPM =  64             };
      enum { UTC_STANDARD_EL =  80               };
      enum { UTC_STANDARD_SU =  96               };
      enum { UTC_STANDARD_NTSC =  112            };
      enum { UTC_STANDARD_UNKNOWN =  240 };

    NavTIMEUTC():
      iTOW(0),
      tAcc(0),
      nano(0),
      year(0),
      month(0),
      day(0),
      hour(0),
      min(0),
      sec(0),
      valid(0)
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
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavTIMEUTC"; };
    virtual const char * getMD5() override { return "bd39089655ebb13ff8d01cbe3a087db1"; };

  };

}
#endif
