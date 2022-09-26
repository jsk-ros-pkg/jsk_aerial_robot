#ifndef _ROS_ublox_msgs_TimTM2_h
#define _ROS_ublox_msgs_TimTM2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class TimTM2 : public ros::Msg
  {
    public:
      typedef uint8_t _ch_type;
      _ch_type ch;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint16_t _risingEdgeCount_type;
      _risingEdgeCount_type risingEdgeCount;
      typedef uint16_t _wnR_type;
      _wnR_type wnR;
      typedef uint16_t _wnF_type;
      _wnF_type wnF;
      typedef uint32_t _towMsR_type;
      _towMsR_type towMsR;
      typedef uint32_t _towSubMsR_type;
      _towSubMsR_type towSubMsR;
      typedef uint32_t _towMsF_type;
      _towMsF_type towMsF;
      typedef uint32_t _towSubMsF_type;
      _towSubMsF_type towSubMsF;
      typedef uint32_t _accEst_type;
      _accEst_type accEst;
      enum { CLASS_ID =  13 };
      enum { MESSAGE_ID =  3 };
      enum { FLAGS_MODE_RUNNING =  1   };
      enum { FLAGS_RUN =  2		 };
      enum { FLAGS_NEWFALLINGEDGE =  4 	 };
      enum { FLAGS_TIMEBASE_GNSS =  8	 };
      enum { FLAGS_TIMEBASE_UTC =  16	 };
      enum { FLAGS_UTC_AVAIL =  32	 };
      enum { FLAGS_TIME_VALID =  64	 };
      enum { FLAGS_NEWRISINGEDGE =  128	 };

    TimTM2():
      ch(0),
      flags(0),
      risingEdgeCount(0),
      wnR(0),
      wnF(0),
      towMsR(0),
      towSubMsR(0),
      towMsF(0),
      towSubMsF(0),
      accEst(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ch >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ch);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->risingEdgeCount >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->risingEdgeCount >> (8 * 1)) & 0xFF;
      offset += sizeof(this->risingEdgeCount);
      *(outbuffer + offset + 0) = (this->wnR >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wnR >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wnR);
      *(outbuffer + offset + 0) = (this->wnF >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wnF >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wnF);
      *(outbuffer + offset + 0) = (this->towMsR >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->towMsR >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->towMsR >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->towMsR >> (8 * 3)) & 0xFF;
      offset += sizeof(this->towMsR);
      *(outbuffer + offset + 0) = (this->towSubMsR >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->towSubMsR >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->towSubMsR >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->towSubMsR >> (8 * 3)) & 0xFF;
      offset += sizeof(this->towSubMsR);
      *(outbuffer + offset + 0) = (this->towMsF >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->towMsF >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->towMsF >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->towMsF >> (8 * 3)) & 0xFF;
      offset += sizeof(this->towMsF);
      *(outbuffer + offset + 0) = (this->towSubMsF >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->towSubMsF >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->towSubMsF >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->towSubMsF >> (8 * 3)) & 0xFF;
      offset += sizeof(this->towSubMsF);
      *(outbuffer + offset + 0) = (this->accEst >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accEst >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accEst >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accEst >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accEst);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->ch =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ch);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->risingEdgeCount =  ((uint16_t) (*(inbuffer + offset)));
      this->risingEdgeCount |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->risingEdgeCount);
      this->wnR =  ((uint16_t) (*(inbuffer + offset)));
      this->wnR |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wnR);
      this->wnF =  ((uint16_t) (*(inbuffer + offset)));
      this->wnF |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wnF);
      this->towMsR =  ((uint32_t) (*(inbuffer + offset)));
      this->towMsR |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->towMsR |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->towMsR |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->towMsR);
      this->towSubMsR =  ((uint32_t) (*(inbuffer + offset)));
      this->towSubMsR |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->towSubMsR |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->towSubMsR |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->towSubMsR);
      this->towMsF =  ((uint32_t) (*(inbuffer + offset)));
      this->towMsF |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->towMsF |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->towMsF |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->towMsF);
      this->towSubMsF =  ((uint32_t) (*(inbuffer + offset)));
      this->towSubMsF |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->towSubMsF |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->towSubMsF |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->towSubMsF);
      this->accEst =  ((uint32_t) (*(inbuffer + offset)));
      this->accEst |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accEst |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accEst |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accEst);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/TimTM2"; };
    virtual const char * getMD5() override { return "aae2d427845426ce522cb55dffc19b63"; };

  };

}
#endif
