#ifndef _ROS_spinal_Gyro_h
#define _ROS_spinal_Gyro_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace spinal
{

  class Gyro : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      int16_t x[6];
      int16_t y[6];
      int16_t z[6];

    Gyro():
      stamp(),
      x(),
      y(),
      z()
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
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_zi;
      u_zi.real = this->z[i];
      *(outbuffer + offset + 0) = (u_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->z[i]);
      }
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
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_xi;
      u_xi.base = 0;
      u_xi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x[i] = u_xi.real;
      offset += sizeof(this->x[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_yi;
      u_yi.base = 0;
      u_yi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y[i] = u_yi.real;
      offset += sizeof(this->y[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_zi;
      u_zi.base = 0;
      u_zi.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zi.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->z[i] = u_zi.real;
      offset += sizeof(this->z[i]);
      }
     return offset;
    }

    const char * getType(){ return "spinal/Gyro"; };
    const char * getMD5(){ return "8729df063c771805b10be4aa77006574"; };

  };

}
#endif
