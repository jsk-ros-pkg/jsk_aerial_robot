#ifndef _ROS_jsk_recognition_msgs_Segment_h
#define _ROS_jsk_recognition_msgs_Segment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"

namespace jsk_recognition_msgs
{

  class Segment : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _start_point_type;
      _start_point_type start_point;
      typedef geometry_msgs::Point _end_point_type;
      _end_point_type end_point;

    Segment():
      start_point(),
      end_point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->start_point.serialize(outbuffer + offset);
      offset += this->end_point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->start_point.deserialize(inbuffer + offset);
      offset += this->end_point.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/Segment"; };
    virtual const char * getMD5() override { return "0125c553546d7123dccaeab992a9e29e"; };

  };

}
#endif
