#ifndef _ROS_spinal_ServoState_h
#define _ROS_spinal_ServoState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class ServoState : public ros::Msg
  {
    public:
      typedef uint8_t _index_type;
      _index_type index;
      typedef int16_t _angle_type;
      _angle_type angle;
      typedef uint8_t _temp_type;
      _temp_type temp;
      typedef int16_t _load_type;
      _load_type load;
      typedef uint8_t _error_type;
      _error_type error;

    ServoState():
      index(0),
      angle(0),
      temp(0),
      load(0),
      error(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->index >> (8 * 0)) & 0xFF;
      offset += sizeof(this->index);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle);
      *(outbuffer + offset + 0) = (this->temp >> (8 * 0)) & 0xFF;
      offset += sizeof(this->temp);
      union {
        int16_t real;
        uint16_t base;
      } u_load;
      u_load.real = this->load;
      *(outbuffer + offset + 0) = (u_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_load.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->load);
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->index =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->index);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      this->temp =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->temp);
      union {
        int16_t real;
        uint16_t base;
      } u_load;
      u_load.base = 0;
      u_load.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_load.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->load = u_load.real;
      offset += sizeof(this->load);
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
     return offset;
    }

    const char * getType(){ return "spinal/ServoState"; };
    const char * getMD5(){ return "8bb74606b2a7f0d9c5d2efdff5bd8fa0"; };

  };

}
#endif
