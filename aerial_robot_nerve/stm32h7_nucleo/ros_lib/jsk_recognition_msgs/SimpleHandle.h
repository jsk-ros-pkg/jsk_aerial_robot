#ifndef _ROS_jsk_recognition_msgs_SimpleHandle_h
#define _ROS_jsk_recognition_msgs_SimpleHandle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace jsk_recognition_msgs
{

  class SimpleHandle : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef double _handle_width_type;
      _handle_width_type handle_width;

    SimpleHandle():
      header(),
      pose(),
      handle_width(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_handle_width;
      u_handle_width.real = this->handle_width;
      *(outbuffer + offset + 0) = (u_handle_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_handle_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_handle_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_handle_width.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_handle_width.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_handle_width.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_handle_width.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_handle_width.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->handle_width);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_handle_width;
      u_handle_width.base = 0;
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_handle_width.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->handle_width = u_handle_width.real;
      offset += sizeof(this->handle_width);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/SimpleHandle"; };
    virtual const char * getMD5() override { return "3a87e21f72b9ed7090c46a47617b0740"; };

  };

}
#endif
