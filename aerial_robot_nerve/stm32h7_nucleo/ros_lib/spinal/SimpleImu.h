#ifndef _ROS_spinal_SimpleImu_h
#define _ROS_spinal_SimpleImu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace spinal
{

  class SimpleImu : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      int16_t accData[3];
      int16_t angle[3];
      typedef uint16_t _debug_type;
      _debug_type debug;

    SimpleImu():
      stamp(),
      accData(),
      angle(),
      debug(0)
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_accDatai;
      u_accDatai.real = this->accData[i];
      *(outbuffer + offset + 0) = (u_accDatai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accDatai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->accData[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      *(outbuffer + offset + 0) = (this->debug >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->debug >> (8 * 1)) & 0xFF;
      offset += sizeof(this->debug);
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
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_accDatai;
      u_accDatai.base = 0;
      u_accDatai.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accDatai.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accData[i] = u_accDatai.real;
      offset += sizeof(this->accData[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_anglei;
      u_anglei.base = 0;
      u_anglei.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglei.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle[i] = u_anglei.real;
      offset += sizeof(this->angle[i]);
      }
      this->debug =  ((uint16_t) (*(inbuffer + offset)));
      this->debug |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->debug);
     return offset;
    }

    const char * getType(){ return "spinal/SimpleImu"; };
    const char * getMD5(){ return "8a35967b47c4f44865264910c5c0a243"; };

  };

}
#endif
