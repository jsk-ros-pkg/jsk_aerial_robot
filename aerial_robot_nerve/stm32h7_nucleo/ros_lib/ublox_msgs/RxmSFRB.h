#ifndef _ROS_ublox_msgs_RxmSFRB_h
#define _ROS_ublox_msgs_RxmSFRB_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmSFRB : public ros::Msg
  {
    public:
      typedef uint8_t _chn_type;
      _chn_type chn;
      typedef uint8_t _svid_type;
      _svid_type svid;
      uint32_t dwrd[10];
      enum { CLASS_ID =  2 };
      enum { MESSAGE_ID =  17 };

    RxmSFRB():
      chn(0),
      svid(0),
      dwrd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->chn >> (8 * 0)) & 0xFF;
      offset += sizeof(this->chn);
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      for( uint32_t i = 0; i < 10; i++){
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
      this->chn =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->chn);
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      for( uint32_t i = 0; i < 10; i++){
      this->dwrd[i] =  ((uint32_t) (*(inbuffer + offset)));
      this->dwrd[i] |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dwrd[i] |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->dwrd[i] |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->dwrd[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmSFRB"; };
    virtual const char * getMD5() override { return "eec72635c768d0528138f40de7442203"; };

  };

}
#endif
