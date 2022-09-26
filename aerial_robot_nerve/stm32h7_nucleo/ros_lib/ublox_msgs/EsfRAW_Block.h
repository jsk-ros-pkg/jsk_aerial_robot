#ifndef _ROS_ublox_msgs_EsfRAW_Block_h
#define _ROS_ublox_msgs_EsfRAW_Block_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class EsfRAW_Block : public ros::Msg
  {
    public:
      typedef uint32_t _data_type;
      _data_type data;
      typedef uint32_t _sTtag_type;
      _sTtag_type sTtag;
      enum { DATA_FIELD_MASK =  16777215 };
      enum { DATA_TYPE_MASK =  4278190080     };

    EsfRAW_Block():
      data(0),
      sTtag(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      *(outbuffer + offset + 0) = (this->sTtag >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sTtag >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sTtag >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sTtag >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sTtag);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->data =  ((uint32_t) (*(inbuffer + offset)));
      this->data |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->data |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->data);
      this->sTtag =  ((uint32_t) (*(inbuffer + offset)));
      this->sTtag |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sTtag |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sTtag |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sTtag);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfRAW_Block"; };
    virtual const char * getMD5() override { return "b688443e4ebc6f99b9ac9276b838d477"; };

  };

}
#endif
