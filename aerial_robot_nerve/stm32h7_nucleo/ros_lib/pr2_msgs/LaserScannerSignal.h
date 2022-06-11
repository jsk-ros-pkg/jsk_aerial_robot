#ifndef _ROS_pr2_msgs_LaserScannerSignal_h
#define _ROS_pr2_msgs_LaserScannerSignal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class LaserScannerSignal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _signal_type;
      _signal_type signal;

    LaserScannerSignal():
      header(),
      signal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_signal;
      u_signal.real = this->signal;
      *(outbuffer + offset + 0) = (u_signal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_signal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_signal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_signal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->signal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_signal;
      u_signal.base = 0;
      u_signal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_signal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_signal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_signal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->signal = u_signal.real;
      offset += sizeof(this->signal);
     return offset;
    }

    const char * getType(){ return "pr2_msgs/LaserScannerSignal"; };
    const char * getMD5(){ return "78f41e618127bce049dd6104d9c31dc5"; };

  };

}
#endif
