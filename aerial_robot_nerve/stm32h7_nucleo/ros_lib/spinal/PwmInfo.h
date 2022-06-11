#ifndef _ROS_spinal_PwmInfo_h
#define _ROS_spinal_PwmInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/MotorInfo.h"

namespace spinal
{

  class PwmInfo : public ros::Msg
  {
    public:
      typedef float _min_pwm_type;
      _min_pwm_type min_pwm;
      typedef float _max_pwm_type;
      _max_pwm_type max_pwm;
      typedef float _min_thrust_type;
      _min_thrust_type min_thrust;
      typedef float _force_landing_thrust_type;
      _force_landing_thrust_type force_landing_thrust;
      typedef uint8_t _pwm_conversion_mode_type;
      _pwm_conversion_mode_type pwm_conversion_mode;
      uint32_t motor_info_length;
      typedef spinal::MotorInfo _motor_info_type;
      _motor_info_type st_motor_info;
      _motor_info_type * motor_info;

    PwmInfo():
      min_pwm(0),
      max_pwm(0),
      min_thrust(0),
      force_landing_thrust(0),
      pwm_conversion_mode(0),
      motor_info_length(0), motor_info(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_min_pwm;
      u_min_pwm.real = this->min_pwm;
      *(outbuffer + offset + 0) = (u_min_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_pwm);
      union {
        float real;
        uint32_t base;
      } u_max_pwm;
      u_max_pwm.real = this->max_pwm;
      *(outbuffer + offset + 0) = (u_max_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_pwm);
      union {
        float real;
        uint32_t base;
      } u_min_thrust;
      u_min_thrust.real = this->min_thrust;
      *(outbuffer + offset + 0) = (u_min_thrust.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_thrust.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_thrust.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_thrust.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_thrust);
      union {
        float real;
        uint32_t base;
      } u_force_landing_thrust;
      u_force_landing_thrust.real = this->force_landing_thrust;
      *(outbuffer + offset + 0) = (u_force_landing_thrust.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force_landing_thrust.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force_landing_thrust.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force_landing_thrust.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force_landing_thrust);
      *(outbuffer + offset + 0) = (this->pwm_conversion_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pwm_conversion_mode);
      *(outbuffer + offset + 0) = (this->motor_info_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_info_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_info_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_info_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_info_length);
      for( uint32_t i = 0; i < motor_info_length; i++){
      offset += this->motor_info[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_min_pwm;
      u_min_pwm.base = 0;
      u_min_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_pwm = u_min_pwm.real;
      offset += sizeof(this->min_pwm);
      union {
        float real;
        uint32_t base;
      } u_max_pwm;
      u_max_pwm.base = 0;
      u_max_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_pwm = u_max_pwm.real;
      offset += sizeof(this->max_pwm);
      union {
        float real;
        uint32_t base;
      } u_min_thrust;
      u_min_thrust.base = 0;
      u_min_thrust.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_thrust.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_thrust.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_thrust.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_thrust = u_min_thrust.real;
      offset += sizeof(this->min_thrust);
      union {
        float real;
        uint32_t base;
      } u_force_landing_thrust;
      u_force_landing_thrust.base = 0;
      u_force_landing_thrust.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force_landing_thrust.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force_landing_thrust.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force_landing_thrust.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force_landing_thrust = u_force_landing_thrust.real;
      offset += sizeof(this->force_landing_thrust);
      this->pwm_conversion_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pwm_conversion_mode);
      uint32_t motor_info_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_info_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_info_length);
      if(motor_info_lengthT > motor_info_length)
        this->motor_info = (spinal::MotorInfo*)realloc(this->motor_info, motor_info_lengthT * sizeof(spinal::MotorInfo));
      motor_info_length = motor_info_lengthT;
      for( uint32_t i = 0; i < motor_info_length; i++){
      offset += this->st_motor_info.deserialize(inbuffer + offset);
        memcpy( &(this->motor_info[i]), &(this->st_motor_info), sizeof(spinal::MotorInfo));
      }
     return offset;
    }

    const char * getType(){ return "spinal/PwmInfo"; };
    const char * getMD5(){ return "eec31372c8b2144723787801d78deb56"; };

  };

}
#endif
