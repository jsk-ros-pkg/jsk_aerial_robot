#ifndef _ROS_SERVICE_GetBoardInfo_h
#define _ROS_SERVICE_GetBoardInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/BoardInfo.h"

namespace spinal
{

static const char GETBOARDINFO[] = "spinal/GetBoardInfo";

  class GetBoardInfoRequest : public ros::Msg
  {
    public:

    GetBoardInfoRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETBOARDINFO; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetBoardInfoResponse : public ros::Msg
  {
    public:
      uint32_t boards_length;
      typedef spinal::BoardInfo _boards_type;
      _boards_type st_boards;
      _boards_type * boards;

    GetBoardInfoResponse():
      boards_length(0), boards(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->boards_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->boards_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->boards_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->boards_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->boards_length);
      for( uint32_t i = 0; i < boards_length; i++){
      offset += this->boards[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t boards_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      boards_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->boards_length);
      if(boards_lengthT > boards_length)
        this->boards = (spinal::BoardInfo*)realloc(this->boards, boards_lengthT * sizeof(spinal::BoardInfo));
      boards_length = boards_lengthT;
      for( uint32_t i = 0; i < boards_length; i++){
      offset += this->st_boards.deserialize(inbuffer + offset);
        memcpy( &(this->boards[i]), &(this->st_boards), sizeof(spinal::BoardInfo));
      }
     return offset;
    }

    const char * getType(){ return GETBOARDINFO; };
    const char * getMD5(){ return "66fd709be8b736fcb084865c0e818d87"; };

  };

  class GetBoardInfo {
    public:
    typedef GetBoardInfoRequest Request;
    typedef GetBoardInfoResponse Response;
  };

}
#endif
