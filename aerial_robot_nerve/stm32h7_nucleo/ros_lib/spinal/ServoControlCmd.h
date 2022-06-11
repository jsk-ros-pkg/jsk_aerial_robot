#ifndef _ROS_spinal_ServoControlCmd_h
#define _ROS_spinal_ServoControlCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class ServoControlCmd : public ros::Msg
  {
    public:
      uint32_t index_length;
      typedef uint8_t _index_type;
      _index_type st_index;
      _index_type * index;
      uint32_t angles_length;
      typedef int16_t _angles_type;
      _angles_type st_angles;
      _angles_type * angles;

    ServoControlCmd():
      index_length(0), index(NULL),
      angles_length(0), angles(NULL)
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
      *(outbuffer + offset + 0) = (this->angles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->angles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->angles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->angles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles_length);
      for( uint32_t i = 0; i < angles_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angles[i]);
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
      uint32_t angles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->angles_length);
      if(angles_lengthT > angles_length)
        this->angles = (int16_t*)realloc(this->angles, angles_lengthT * sizeof(int16_t));
      angles_length = angles_lengthT;
      for( uint32_t i = 0; i < angles_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_angles;
      u_st_angles.base = 0;
      u_st_angles.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angles.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_angles = u_st_angles.real;
      offset += sizeof(this->st_angles);
        memcpy( &(this->angles[i]), &(this->st_angles), sizeof(int16_t));
      }
     return offset;
    }

    const char * getType(){ return "spinal/ServoControlCmd"; };
    const char * getMD5(){ return "7b019fc22cfaf6e7fcb8bd9471a1cbba"; };

  };

}
#endif
