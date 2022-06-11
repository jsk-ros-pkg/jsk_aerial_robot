#ifndef _ROS_pr2_msgs_AccelerometerState_h
#define _ROS_pr2_msgs_AccelerometerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace pr2_msgs
{

  class AccelerometerState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t samples_length;
      typedef geometry_msgs::Vector3 _samples_type;
      _samples_type st_samples;
      _samples_type * samples;

    AccelerometerState():
      header(),
      samples_length(0), samples(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->samples_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->samples_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->samples_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->samples_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->samples_length);
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->samples[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t samples_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      samples_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->samples_length);
      if(samples_lengthT > samples_length)
        this->samples = (geometry_msgs::Vector3*)realloc(this->samples, samples_lengthT * sizeof(geometry_msgs::Vector3));
      samples_length = samples_lengthT;
      for( uint32_t i = 0; i < samples_length; i++){
      offset += this->st_samples.deserialize(inbuffer + offset);
        memcpy( &(this->samples[i]), &(this->st_samples), sizeof(geometry_msgs::Vector3));
      }
     return offset;
    }

    const char * getType(){ return "pr2_msgs/AccelerometerState"; };
    const char * getMD5(){ return "26492e97ed8c13252c4a85592d3e93fd"; };

  };

}
#endif
