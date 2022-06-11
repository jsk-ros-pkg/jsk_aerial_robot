#ifndef _ROS_spinal_ServoTorqueCmd_h
#define _ROS_spinal_ServoTorqueCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class ServoTorqueCmd : public ros::Msg
  {
    public:
      uint32_t index_length;
      typedef uint8_t _index_type;
      _index_type st_index;
      _index_type * index;
      uint32_t torque_enable_length;
      typedef uint8_t _torque_enable_type;
      _torque_enable_type st_torque_enable;
      _torque_enable_type * torque_enable;

    ServoTorqueCmd():
      index_length(0), index(NULL),
      torque_enable_length(0), torque_enable(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->index_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->index_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->index_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->index_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index_length);
      for( uint32_t i = 0; i < index_length; i++){
      *(outbuffer + offset + 0) = (this->index[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->index[i]);
      }
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
      uint32_t index_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      index_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->index_length);
      if(index_lengthT > index_length)
        this->index = (uint8_t*)realloc(this->index, index_lengthT * sizeof(uint8_t));
      index_length = index_lengthT;
      for( uint32_t i = 0; i < index_length; i++){
      this->st_index =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_index);
        memcpy( &(this->index[i]), &(this->st_index), sizeof(uint8_t));
      }
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

    const char * getType(){ return "spinal/ServoTorqueCmd"; };
    const char * getMD5(){ return "3bf69950d8290f79f2ce423f967a9338"; };

  };

}
#endif
