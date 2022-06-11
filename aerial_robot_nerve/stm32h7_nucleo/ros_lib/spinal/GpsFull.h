#ifndef _ROS_spinal_GpsFull_h
#define _ROS_spinal_GpsFull_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace spinal
{

  class GpsFull : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef uint8_t _status_type;
      _status_type status;
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
      typedef int32_t _nano_type;
      _nano_type nano;
      typedef uint8_t _time_valid_type;
      _time_valid_type time_valid;
      double location[2];
      typedef float _h_acc_type;
      _h_acc_type h_acc;
      float velocity[2];
      typedef float _v_acc_type;
      _v_acc_type v_acc;
      typedef uint8_t _sat_num_type;
      _sat_num_type sat_num;
      enum { FIX_TYPE_NO_FIX =  0 };
      enum { FIX_TYPE_DEAD_RECKONING_ONLY =  1 };
      enum { FIX_TYPE_2D =  2                            };
      enum { FIX_TYPE_3D =  3 };
      enum { FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED =  4  };
      enum { FIX_TYPE_TIME_ONLY =  5                     };

    GpsFull():
      stamp(),
      status(0),
      year(0),
      month(0),
      day(0),
      hour(0),
      min(0),
      sec(0),
      nano(0),
      time_valid(0),
      location(),
      h_acc(0),
      velocity(),
      v_acc(0),
      sat_num(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
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
      *(outbuffer + offset + 0) = (this->time_valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->time_valid);
      for( uint32_t i = 0; i < 2; i++){
      union {
        double real;
        uint64_t base;
      } u_locationi;
      u_locationi.real = this->location[i];
      *(outbuffer + offset + 0) = (u_locationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_locationi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_locationi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_locationi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_locationi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_locationi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_locationi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_locationi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->location[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_h_acc;
      u_h_acc.real = this->h_acc;
      *(outbuffer + offset + 0) = (u_h_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_h_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_h_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_h_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->h_acc);
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_velocityi;
      u_velocityi.real = this->velocity[i];
      *(outbuffer + offset + 0) = (u_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocityi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_v_acc;
      u_v_acc.real = this->v_acc;
      *(outbuffer + offset + 0) = (u_v_acc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_acc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_acc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_acc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_acc);
      *(outbuffer + offset + 0) = (this->sat_num >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sat_num);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
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
      this->time_valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->time_valid);
      for( uint32_t i = 0; i < 2; i++){
      union {
        double real;
        uint64_t base;
      } u_locationi;
      u_locationi.base = 0;
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_locationi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->location[i] = u_locationi.real;
      offset += sizeof(this->location[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_h_acc;
      u_h_acc.base = 0;
      u_h_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_h_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_h_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_h_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->h_acc = u_h_acc.real;
      offset += sizeof(this->h_acc);
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_velocityi;
      u_velocityi.base = 0;
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocityi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity[i] = u_velocityi.real;
      offset += sizeof(this->velocity[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_v_acc;
      u_v_acc.base = 0;
      u_v_acc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_acc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_acc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_acc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_acc = u_v_acc.real;
      offset += sizeof(this->v_acc);
      this->sat_num =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sat_num);
     return offset;
    }

    const char * getType(){ return "spinal/GpsFull"; };
    const char * getMD5(){ return "05ebd68b847d9ec86915e2f1d0c24f52"; };

  };

}
#endif
