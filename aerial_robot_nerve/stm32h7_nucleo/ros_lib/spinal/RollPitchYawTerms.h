#ifndef _ROS_spinal_RollPitchYawTerms_h
#define _ROS_spinal_RollPitchYawTerms_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/RollPitchYawTerm.h"

namespace spinal
{

  class RollPitchYawTerms : public ros::Msg
  {
    public:
      uint32_t motors_length;
      typedef spinal::RollPitchYawTerm _motors_type;
      _motors_type st_motors;
      _motors_type * motors;

    RollPitchYawTerms():
      motors_length(0), motors(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motors_length);
      for( uint32_t i = 0; i < motors_length; i++){
      offset += this->motors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t motors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motors_length);
      if(motors_lengthT > motors_length)
        this->motors = (spinal::RollPitchYawTerm*)realloc(this->motors, motors_lengthT * sizeof(spinal::RollPitchYawTerm));
      motors_length = motors_lengthT;
      for( uint32_t i = 0; i < motors_length; i++){
      offset += this->st_motors.deserialize(inbuffer + offset);
        memcpy( &(this->motors[i]), &(this->st_motors), sizeof(spinal::RollPitchYawTerm));
      }
     return offset;
    }

    const char * getType(){ return "spinal/RollPitchYawTerms"; };
    const char * getMD5(){ return "e02c97843dff2be50cd609663998aa9e"; };

  };

}
#endif
