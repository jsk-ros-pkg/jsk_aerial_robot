#ifndef _ROS_ublox_msgs_CfgINF_h
#define _ROS_ublox_msgs_CfgINF_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/CfgINF_Block.h"

namespace ublox_msgs
{

  class CfgINF : public ros::Msg
  {
    public:
      uint32_t blocks_length;
      typedef ublox_msgs::CfgINF_Block _blocks_type;
      _blocks_type st_blocks;
      _blocks_type * blocks;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  2 };

    CfgINF():
      blocks_length(0), st_blocks(), blocks(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->blocks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->blocks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->blocks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->blocks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blocks_length);
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->blocks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t blocks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->blocks_length);
      if(blocks_lengthT > blocks_length)
        this->blocks = (ublox_msgs::CfgINF_Block*)realloc(this->blocks, blocks_lengthT * sizeof(ublox_msgs::CfgINF_Block));
      blocks_length = blocks_lengthT;
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->st_blocks.deserialize(inbuffer + offset);
        memcpy( &(this->blocks[i]), &(this->st_blocks), sizeof(ublox_msgs::CfgINF_Block));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgINF"; };
    virtual const char * getMD5() override { return "42fb321cf0c63684f2f7086e850ed61e"; };

  };

}
#endif
