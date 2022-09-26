#ifndef _ROS_ublox_msgs_EsfRAW_h
#define _ROS_ublox_msgs_EsfRAW_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/EsfRAW_Block.h"

namespace ublox_msgs
{

  class EsfRAW : public ros::Msg
  {
    public:
      uint8_t reserved0[4];
      uint32_t blocks_length;
      typedef ublox_msgs::EsfRAW_Block _blocks_type;
      _blocks_type st_blocks;
      _blocks_type * blocks;
      enum { CLASS_ID =  16 };
      enum { MESSAGE_ID =  3 };

    EsfRAW():
      reserved0(),
      blocks_length(0), st_blocks(), blocks(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
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
      for( uint32_t i = 0; i < 4; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      uint32_t blocks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->blocks_length);
      if(blocks_lengthT > blocks_length)
        this->blocks = (ublox_msgs::EsfRAW_Block*)realloc(this->blocks, blocks_lengthT * sizeof(ublox_msgs::EsfRAW_Block));
      blocks_length = blocks_lengthT;
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->st_blocks.deserialize(inbuffer + offset);
        memcpy( &(this->blocks[i]), &(this->st_blocks), sizeof(ublox_msgs::EsfRAW_Block));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfRAW"; };
    virtual const char * getMD5() override { return "b942250c5ec94c6b6e69c63d82d9a946"; };

  };

}
#endif
