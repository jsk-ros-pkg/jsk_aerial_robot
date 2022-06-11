#ifndef _ROS_spinal_Pwms_h
#define _ROS_spinal_Pwms_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class Pwms : public ros::Msg
  {
    public:
      uint32_t motor_value_length;
      typedef uint16_t _motor_value_type;
      _motor_value_type st_motor_value;
      _motor_value_type * motor_value;

    Pwms():
      motor_value_length(0), motor_value(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motor_value_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_value_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_value_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_value_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_value_length);
      for( uint32_t i = 0; i < motor_value_length; i++){
      *(outbuffer + offset + 0) = (this->motor_value[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_value[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motor_value[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t motor_value_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_value_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_value_length);
      if(motor_value_lengthT > motor_value_length)
        this->motor_value = (uint16_t*)realloc(this->motor_value, motor_value_lengthT * sizeof(uint16_t));
      motor_value_length = motor_value_lengthT;
      for( uint32_t i = 0; i < motor_value_length; i++){
      this->st_motor_value =  ((uint16_t) (*(inbuffer + offset)));
      this->st_motor_value |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->st_motor_value);
        memcpy( &(this->motor_value[i]), &(this->st_motor_value), sizeof(uint16_t));
      }
     return offset;
    }

    const char * getType(){ return "spinal/Pwms"; };
    const char * getMD5(){ return "2c8c334b8b8efa66767058af395f7c74"; };

  };

}
#endif
