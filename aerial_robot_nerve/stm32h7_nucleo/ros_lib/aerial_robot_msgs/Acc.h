#ifndef _ROS_aerial_robot_msgs_Acc_h
#define _ROS_aerial_robot_msgs_Acc_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace aerial_robot_msgs
{

  class Acc : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _acc_body_frame_type;
      _acc_body_frame_type acc_body_frame;
      typedef geometry_msgs::Vector3 _acc_world_frame_type;
      _acc_world_frame_type acc_world_frame;
      typedef geometry_msgs::Vector3 _acc_non_bias_world_frame_type;
      _acc_non_bias_world_frame_type acc_non_bias_world_frame;

    Acc():
      header(),
      acc_body_frame(),
      acc_world_frame(),
      acc_non_bias_world_frame()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->acc_body_frame.serialize(outbuffer + offset);
      offset += this->acc_world_frame.serialize(outbuffer + offset);
      offset += this->acc_non_bias_world_frame.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->acc_body_frame.deserialize(inbuffer + offset);
      offset += this->acc_world_frame.deserialize(inbuffer + offset);
      offset += this->acc_non_bias_world_frame.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/Acc"; };
    const char * getMD5(){ return "cda10dc3733e23caa70166844a85cacd"; };

  };

}
#endif
