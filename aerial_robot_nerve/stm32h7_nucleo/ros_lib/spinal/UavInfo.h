#ifndef _ROS_spinal_UavInfo_h
#define _ROS_spinal_UavInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class UavInfo : public ros::Msg
  {
    public:
      typedef uint8_t _motor_num_type;
      _motor_num_type motor_num;
      typedef uint8_t _uav_model_type;
      _uav_model_type uav_model;
      typedef uint8_t _baselink_type;
      _baselink_type baselink;
      enum { DRONE =  0  };
      enum { HYDRUS =  16  };
      enum { HYDRUS_XI =  17  };
      enum { DRAGON =  32  };

    UavInfo():
      motor_num(0),
      uav_model(0),
      baselink(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motor_num >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motor_num);
      *(outbuffer + offset + 0) = (this->uav_model >> (8 * 0)) & 0xFF;
      offset += sizeof(this->uav_model);
      *(outbuffer + offset + 0) = (this->baselink >> (8 * 0)) & 0xFF;
      offset += sizeof(this->baselink);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->motor_num =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->motor_num);
      this->uav_model =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->uav_model);
      this->baselink =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->baselink);
     return offset;
    }

    const char * getType(){ return "spinal/UavInfo"; };
    const char * getMD5(){ return "9df05674c34b7ea4b3399195c6c615ca"; };

  };

}
#endif
