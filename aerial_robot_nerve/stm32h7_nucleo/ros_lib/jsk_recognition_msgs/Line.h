#ifndef _ROS_jsk_recognition_msgs_Line_h
#define _ROS_jsk_recognition_msgs_Line_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_recognition_msgs
{

  class Line : public ros::Msg
  {
    public:
      typedef double _x1_type;
      _x1_type x1;
      typedef double _y1_type;
      _y1_type y1;
      typedef double _x2_type;
      _x2_type x2;
      typedef double _y2_type;
      _y2_type y2;

    Line():
      x1(0),
      y1(0),
      x2(0),
      y2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x1;
      u_x1.real = this->x1;
      *(outbuffer + offset + 0) = (u_x1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x1);
      union {
        double real;
        uint64_t base;
      } u_y1;
      u_y1.real = this->y1;
      *(outbuffer + offset + 0) = (u_y1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y1);
      union {
        double real;
        uint64_t base;
      } u_x2;
      u_x2.real = this->x2;
      *(outbuffer + offset + 0) = (u_x2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x2);
      union {
        double real;
        uint64_t base;
      } u_y2;
      u_y2.real = this->y2;
      *(outbuffer + offset + 0) = (u_y2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y2.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y2.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y2.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y2.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y2.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x1;
      u_x1.base = 0;
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x1 = u_x1.real;
      offset += sizeof(this->x1);
      union {
        double real;
        uint64_t base;
      } u_y1;
      u_y1.base = 0;
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y1 = u_y1.real;
      offset += sizeof(this->y1);
      union {
        double real;
        uint64_t base;
      } u_x2;
      u_x2.base = 0;
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x2 = u_x2.real;
      offset += sizeof(this->x2);
      union {
        double real;
        uint64_t base;
      } u_y2;
      u_y2.base = 0;
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y2.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y2 = u_y2.real;
      offset += sizeof(this->y2);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/Line"; };
    virtual const char * getMD5() override { return "3010fad09b09b8d3fdecdd94d144f7a1"; };

  };

}
#endif
