#ifndef _ROS_ublox_msgs_CfgGNSS_h
#define _ROS_ublox_msgs_CfgGNSS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/CfgGNSS_Block.h"

namespace ublox_msgs
{

  class CfgGNSS : public ros::Msg
  {
    public:
      typedef uint8_t _msgVer_type;
      _msgVer_type msgVer;
      typedef uint8_t _numTrkChHw_type;
      _numTrkChHw_type numTrkChHw;
      typedef uint8_t _numTrkChUse_type;
      _numTrkChUse_type numTrkChUse;
      typedef uint8_t _numConfigBlocks_type;
      _numConfigBlocks_type numConfigBlocks;
      uint32_t blocks_length;
      typedef ublox_msgs::CfgGNSS_Block _blocks_type;
      _blocks_type st_blocks;
      _blocks_type * blocks;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  62 };

    CfgGNSS():
      msgVer(0),
      numTrkChHw(0),
      numTrkChUse(0),
      numConfigBlocks(0),
      blocks_length(0), st_blocks(), blocks(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->msgVer >> (8 * 0)) & 0xFF;
      offset += sizeof(this->msgVer);
      *(outbuffer + offset + 0) = (this->numTrkChHw >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numTrkChHw);
      *(outbuffer + offset + 0) = (this->numTrkChUse >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numTrkChUse);
      *(outbuffer + offset + 0) = (this->numConfigBlocks >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numConfigBlocks);
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
      this->msgVer =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->msgVer);
      this->numTrkChHw =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numTrkChHw);
      this->numTrkChUse =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numTrkChUse);
      this->numConfigBlocks =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numConfigBlocks);
      uint32_t blocks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      blocks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->blocks_length);
      if(blocks_lengthT > blocks_length)
        this->blocks = (ublox_msgs::CfgGNSS_Block*)realloc(this->blocks, blocks_lengthT * sizeof(ublox_msgs::CfgGNSS_Block));
      blocks_length = blocks_lengthT;
      for( uint32_t i = 0; i < blocks_length; i++){
      offset += this->st_blocks.deserialize(inbuffer + offset);
        memcpy( &(this->blocks[i]), &(this->st_blocks), sizeof(ublox_msgs::CfgGNSS_Block));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgGNSS"; };
    virtual const char * getMD5() override { return "c1777482e22a0ac50132c66d3284e86f"; };

  };

}
#endif
