#ifndef _ROS_ublox_msgs_AidALM_h
#define _ROS_ublox_msgs_AidALM_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class AidALM : public ros::Msg
  {
    public:
      typedef uint32_t _svid_type;
      _svid_type svid;
      typedef uint32_t _week_type;
      _week_type week;
      uint32_t dwrd_length;
      typedef uint32_t _dwrd_type;
      _dwrd_type st_dwrd;
      _dwrd_type * dwrd;
      enum { CLASS_ID =  11 };
      enum { MESSAGE_ID =  48 };

    AidALM():
      svid(0),
      week(0),
      dwrd_length(0), st_dwrd(), dwrd(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->svid >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->svid >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->svid >> (8 * 3)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->week >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->week >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->week >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->week >> (8 * 3)) & 0xFF;
      offset += sizeof(this->week);
      *(outbuffer + offset + 0) = (this->dwrd_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dwrd_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dwrd_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dwrd_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dwrd_length);
      for( uint32_t i = 0; i < dwrd_length; i++){
      *(outbuffer + offset + 0) = (this->dwrd[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dwrd[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dwrd[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dwrd[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dwrd[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->svid =  ((uint32_t) (*(inbuffer + offset)));
      this->svid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->svid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->svid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->svid);
      this->week =  ((uint32_t) (*(inbuffer + offset)));
      this->week |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->week |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->week |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->week);
      uint32_t dwrd_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dwrd_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dwrd_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dwrd_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dwrd_length);
      if(dwrd_lengthT > dwrd_length)
        this->dwrd = (uint32_t*)realloc(this->dwrd, dwrd_lengthT * sizeof(uint32_t));
      dwrd_length = dwrd_lengthT;
      for( uint32_t i = 0; i < dwrd_length; i++){
      this->st_dwrd =  ((uint32_t) (*(inbuffer + offset)));
      this->st_dwrd |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_dwrd |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_dwrd |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_dwrd);
        memcpy( &(this->dwrd[i]), &(this->st_dwrd), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/AidALM"; };
    virtual const char * getMD5() override { return "de5ab2550e698fc8acfb7263c7c55fa2"; };

  };

}
#endif
