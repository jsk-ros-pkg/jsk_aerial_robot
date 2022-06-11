#ifndef _ROS_spinal_MotorInfo_h
#define _ROS_spinal_MotorInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class MotorInfo : public ros::Msg
  {
    public:
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _max_thrust_type;
      _max_thrust_type max_thrust;
      float polynominal[5];
      enum { SQRT_MODE =  0 };
      enum { POLYNOMINAL_MODE =  1 };

    MotorInfo():
      voltage(0),
      max_thrust(0),
      polynominal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_max_thrust;
      u_max_thrust.real = this->max_thrust;
      *(outbuffer + offset + 0) = (u_max_thrust.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_thrust.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_thrust.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_thrust.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_thrust);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_polynominali;
      u_polynominali.real = this->polynominal[i];
      *(outbuffer + offset + 0) = (u_polynominali.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_polynominali.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_polynominali.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_polynominali.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->polynominal[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_max_thrust;
      u_max_thrust.base = 0;
      u_max_thrust.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_thrust.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_thrust.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_thrust.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_thrust = u_max_thrust.real;
      offset += sizeof(this->max_thrust);
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_polynominali;
      u_polynominali.base = 0;
      u_polynominali.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_polynominali.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_polynominali.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_polynominali.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->polynominal[i] = u_polynominali.real;
      offset += sizeof(this->polynominal[i]);
      }
     return offset;
    }

    const char * getType(){ return "spinal/MotorInfo"; };
    const char * getMD5(){ return "b35f31955ac7841cc3d42390c4224ee5"; };

  };

}
#endif
