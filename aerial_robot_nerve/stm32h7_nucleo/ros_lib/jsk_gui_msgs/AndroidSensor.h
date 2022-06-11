#ifndef _ROS_jsk_gui_msgs_AndroidSensor_h
#define _ROS_jsk_gui_msgs_AndroidSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_gui_msgs
{

  class AndroidSensor : public ros::Msg
  {
    public:
      typedef double _accel_x_type;
      _accel_x_type accel_x;
      typedef double _accel_y_type;
      _accel_y_type accel_y;
      typedef double _accel_z_type;
      _accel_z_type accel_z;
      typedef double _orientation_x_type;
      _orientation_x_type orientation_x;
      typedef double _orientation_y_type;
      _orientation_y_type orientation_y;
      typedef double _orientation_z_type;
      _orientation_z_type orientation_z;

    AndroidSensor():
      accel_x(0),
      accel_y(0),
      accel_z(0),
      orientation_x(0),
      orientation_y(0),
      orientation_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_accel_x;
      u_accel_x.real = this->accel_x;
      *(outbuffer + offset + 0) = (u_accel_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_accel_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_accel_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_accel_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_accel_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->accel_x);
      union {
        double real;
        uint64_t base;
      } u_accel_y;
      u_accel_y.real = this->accel_y;
      *(outbuffer + offset + 0) = (u_accel_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_accel_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_accel_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_accel_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_accel_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->accel_y);
      union {
        double real;
        uint64_t base;
      } u_accel_z;
      u_accel_z.real = this->accel_z;
      *(outbuffer + offset + 0) = (u_accel_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_accel_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_accel_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_accel_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_accel_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->accel_z);
      union {
        double real;
        uint64_t base;
      } u_orientation_x;
      u_orientation_x.real = this->orientation_x;
      *(outbuffer + offset + 0) = (u_orientation_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_orientation_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_orientation_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_orientation_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_orientation_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->orientation_x);
      union {
        double real;
        uint64_t base;
      } u_orientation_y;
      u_orientation_y.real = this->orientation_y;
      *(outbuffer + offset + 0) = (u_orientation_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_orientation_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_orientation_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_orientation_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_orientation_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->orientation_y);
      union {
        double real;
        uint64_t base;
      } u_orientation_z;
      u_orientation_z.real = this->orientation_z;
      *(outbuffer + offset + 0) = (u_orientation_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_orientation_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_orientation_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_orientation_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_orientation_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->orientation_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_accel_x;
      u_accel_x.base = 0;
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_accel_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->accel_x = u_accel_x.real;
      offset += sizeof(this->accel_x);
      union {
        double real;
        uint64_t base;
      } u_accel_y;
      u_accel_y.base = 0;
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_accel_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->accel_y = u_accel_y.real;
      offset += sizeof(this->accel_y);
      union {
        double real;
        uint64_t base;
      } u_accel_z;
      u_accel_z.base = 0;
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_accel_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->accel_z = u_accel_z.real;
      offset += sizeof(this->accel_z);
      union {
        double real;
        uint64_t base;
      } u_orientation_x;
      u_orientation_x.base = 0;
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_orientation_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->orientation_x = u_orientation_x.real;
      offset += sizeof(this->orientation_x);
      union {
        double real;
        uint64_t base;
      } u_orientation_y;
      u_orientation_y.base = 0;
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_orientation_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->orientation_y = u_orientation_y.real;
      offset += sizeof(this->orientation_y);
      union {
        double real;
        uint64_t base;
      } u_orientation_z;
      u_orientation_z.base = 0;
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_orientation_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->orientation_z = u_orientation_z.real;
      offset += sizeof(this->orientation_z);
     return offset;
    }

    const char * getType(){ return "jsk_gui_msgs/AndroidSensor"; };
    const char * getMD5(){ return "d832dbe3be7e7f061d963f2188f1a407"; };

  };

}
#endif
