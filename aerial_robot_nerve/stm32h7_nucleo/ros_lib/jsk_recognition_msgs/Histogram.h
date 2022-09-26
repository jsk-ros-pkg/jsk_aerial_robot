#ifndef _ROS_jsk_recognition_msgs_Histogram_h
#define _ROS_jsk_recognition_msgs_Histogram_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_recognition_msgs
{

  class Histogram : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t histogram_length;
      typedef double _histogram_type;
      _histogram_type st_histogram;
      _histogram_type * histogram;

    Histogram():
      header(),
      histogram_length(0), st_histogram(), histogram(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->histogram_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->histogram_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->histogram_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->histogram_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->histogram_length);
      for( uint32_t i = 0; i < histogram_length; i++){
      union {
        double real;
        uint64_t base;
      } u_histogrami;
      u_histogrami.real = this->histogram[i];
      *(outbuffer + offset + 0) = (u_histogrami.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_histogrami.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_histogrami.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_histogrami.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_histogrami.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_histogrami.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_histogrami.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_histogrami.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->histogram[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t histogram_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      histogram_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->histogram_length);
      if(histogram_lengthT > histogram_length)
        this->histogram = (double*)realloc(this->histogram, histogram_lengthT * sizeof(double));
      histogram_length = histogram_lengthT;
      for( uint32_t i = 0; i < histogram_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_histogram;
      u_st_histogram.base = 0;
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_histogram.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_histogram = u_st_histogram.real;
      offset += sizeof(this->st_histogram);
        memcpy( &(this->histogram[i]), &(this->st_histogram), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/Histogram"; };
    virtual const char * getMD5() override { return "376372f016c22c39ab7ee6b6bdd0f10a"; };

  };

}
#endif
