#ifndef _ROS_spinal_FourAxisCommand_h
#define _ROS_spinal_FourAxisCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class FourAxisCommand : public ros::Msg
  {
    public:
      float angles[3];
      uint32_t base_thrust_length;
      typedef float _base_thrust_type;
      _base_thrust_type st_base_thrust;
      _base_thrust_type * base_thrust;

    FourAxisCommand():
      angles(),
      base_thrust_length(0), base_thrust(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      *(outbuffer + offset + 0) = (this->base_thrust_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_thrust_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->base_thrust_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->base_thrust_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_thrust_length);
      for( uint32_t i = 0; i < base_thrust_length; i++){
      union {
        float real;
        uint32_t base;
      } u_base_thrusti;
      u_base_thrusti.real = this->base_thrust[i];
      *(outbuffer + offset + 0) = (u_base_thrusti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_thrusti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_thrusti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_thrusti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_thrust[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.base = 0;
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angles[i] = u_anglesi.real;
      offset += sizeof(this->angles[i]);
      }
      uint32_t base_thrust_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      base_thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      base_thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      base_thrust_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->base_thrust_length);
      if(base_thrust_lengthT > base_thrust_length)
        this->base_thrust = (float*)realloc(this->base_thrust, base_thrust_lengthT * sizeof(float));
      base_thrust_length = base_thrust_lengthT;
      for( uint32_t i = 0; i < base_thrust_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_base_thrust;
      u_st_base_thrust.base = 0;
      u_st_base_thrust.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_base_thrust.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_base_thrust.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_base_thrust.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_base_thrust = u_st_base_thrust.real;
      offset += sizeof(this->st_base_thrust);
        memcpy( &(this->base_thrust[i]), &(this->st_base_thrust), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "spinal/FourAxisCommand"; };
    const char * getMD5(){ return "058c4b50fedc70ca5454fb118cb70947"; };

  };

}
#endif
