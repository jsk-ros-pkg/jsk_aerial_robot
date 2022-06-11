#ifndef _ROS_jsk_network_tools_CompressedAngleVectorPR2_h
#define _ROS_jsk_network_tools_CompressedAngleVectorPR2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_network_tools
{

  class CompressedAngleVectorPR2 : public ros::Msg
  {
    public:
      uint8_t angles[17];

    CompressedAngleVectorPR2():
      angles()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 17; i++){
      *(outbuffer + offset + 0) = (this->angles[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 17; i++){
      this->angles[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->angles[i]);
      }
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/CompressedAngleVectorPR2"; };
    const char * getMD5(){ return "41a167b428fc98b1c378a7ba1bae8d54"; };

  };

}
#endif
