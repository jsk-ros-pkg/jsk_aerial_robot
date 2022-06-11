#ifndef _ROS_spinal_ServoTorqueStates_h
#define _ROS_spinal_ServoTorqueStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class ServoTorqueStates : public ros::Msg
  {
    public:
      uint32_t torque_enable_length;
      typedef uint8_t _torque_enable_type;
      _torque_enable_type st_torque_enable;
      _torque_enable_type * torque_enable;

    ServoTorqueStates():
      torque_enable_length(0), torque_enable(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->torque_enable_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->torque_enable_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->torque_enable_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->torque_enable_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque_enable_length);
      for( uint32_t i = 0; i < torque_enable_length; i++){
      *(outbuffer + offset + 0) = (this->torque_enable[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torque_enable[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t torque_enable_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      torque_enable_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      torque_enable_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      torque_enable_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->torque_enable_length);
      if(torque_enable_lengthT > torque_enable_length)
        this->torque_enable = (uint8_t*)realloc(this->torque_enable, torque_enable_lengthT * sizeof(uint8_t));
      torque_enable_length = torque_enable_lengthT;
      for( uint32_t i = 0; i < torque_enable_length; i++){
      this->st_torque_enable =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_torque_enable);
        memcpy( &(this->torque_enable[i]), &(this->st_torque_enable), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "spinal/ServoTorqueStates"; };
    const char * getMD5(){ return "a690708f2dc38a03d93a1e5fa2b4ae3d"; };

  };

}
#endif
