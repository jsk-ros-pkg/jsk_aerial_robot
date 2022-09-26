#ifndef _ROS_ublox_msgs_RxmSFRBX_h
#define _ROS_ublox_msgs_RxmSFRBX_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmSFRBX : public ros::Msg
  {
    public:
      typedef uint8_t _gnssId_type;
      _gnssId_type gnssId;
      typedef uint8_t _svId_type;
      _svId_type svId;
      typedef uint8_t _reserved0_type;
      _reserved0_type reserved0;
      typedef uint8_t _freqId_type;
      _freqId_type freqId;
      typedef uint8_t _numWords_type;
      _numWords_type numWords;
      typedef uint8_t _chn_type;
      _chn_type chn;
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      uint32_t dwrd_length;
      typedef uint32_t _dwrd_type;
      _dwrd_type st_dwrd;
      _dwrd_type * dwrd;
      enum { CLASS_ID =  2 };
      enum { MESSAGE_ID =  19 };

    RxmSFRBX():
      gnssId(0),
      svId(0),
      reserved0(0),
      freqId(0),
      numWords(0),
      chn(0),
      version(0),
      reserved1(0),
      dwrd_length(0), st_dwrd(), dwrd(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gnssId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gnssId);
      *(outbuffer + offset + 0) = (this->svId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svId);
      *(outbuffer + offset + 0) = (this->reserved0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0);
      *(outbuffer + offset + 0) = (this->freqId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->freqId);
      *(outbuffer + offset + 0) = (this->numWords >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numWords);
      *(outbuffer + offset + 0) = (this->chn >> (8 * 0)) & 0xFF;
      offset += sizeof(this->chn);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
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
      this->gnssId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gnssId);
      this->svId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svId);
      this->reserved0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0);
      this->freqId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->freqId);
      this->numWords =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numWords);
      this->chn =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->chn);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
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

    virtual const char * getType() override { return "ublox_msgs/RxmSFRBX"; };
    virtual const char * getMD5() override { return "c76473d828cc8e80de3a2d83cd6b9dff"; };

  };

}
#endif
