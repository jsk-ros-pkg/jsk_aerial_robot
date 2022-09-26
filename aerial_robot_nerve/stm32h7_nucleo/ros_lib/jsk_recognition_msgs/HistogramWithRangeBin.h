#ifndef _ROS_jsk_recognition_msgs_HistogramWithRangeBin_h
#define _ROS_jsk_recognition_msgs_HistogramWithRangeBin_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_recognition_msgs
{

  class HistogramWithRangeBin : public ros::Msg
  {
    public:
      typedef double _min_value_type;
      _min_value_type min_value;
      typedef double _max_value_type;
      _max_value_type max_value;
      typedef uint32_t _count_type;
      _count_type count;

    HistogramWithRangeBin():
      min_value(0),
      max_value(0),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_min_value;
      u_min_value.real = this->min_value;
      *(outbuffer + offset + 0) = (u_min_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_value.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_value.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_value.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_value.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_value.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_value);
      union {
        double real;
        uint64_t base;
      } u_max_value;
      u_max_value.real = this->max_value;
      *(outbuffer + offset + 0) = (u_max_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_value.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_value.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_value.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_value.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_value.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_value);
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->count >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->count >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->count >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_min_value;
      u_min_value.base = 0;
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_value.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_value = u_min_value.real;
      offset += sizeof(this->min_value);
      union {
        double real;
        uint64_t base;
      } u_max_value;
      u_max_value.base = 0;
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_value.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_value = u_max_value.real;
      offset += sizeof(this->max_value);
      this->count =  ((uint32_t) (*(inbuffer + offset)));
      this->count |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->count |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->count |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->count);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/HistogramWithRangeBin"; };
    virtual const char * getMD5() override { return "a7fe6c3021fcba2c6357f3db21601551"; };

  };

}
#endif
