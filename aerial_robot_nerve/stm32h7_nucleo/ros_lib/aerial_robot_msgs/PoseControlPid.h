#ifndef _ROS_aerial_robot_msgs_PoseControlPid_h
#define _ROS_aerial_robot_msgs_PoseControlPid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "aerial_robot_msgs/Pid.h"

namespace aerial_robot_msgs
{

  class PoseControlPid : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef aerial_robot_msgs::Pid _x_type;
      _x_type x;
      typedef aerial_robot_msgs::Pid _y_type;
      _y_type y;
      typedef aerial_robot_msgs::Pid _z_type;
      _z_type z;
      typedef aerial_robot_msgs::Pid _pitch_type;
      _pitch_type pitch;
      typedef aerial_robot_msgs::Pid _roll_type;
      _roll_type roll;
      typedef aerial_robot_msgs::Pid _yaw_type;
      _yaw_type yaw;

    PoseControlPid():
      header(),
      x(),
      y(),
      z(),
      pitch(),
      roll(),
      yaw()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->x.serialize(outbuffer + offset);
      offset += this->y.serialize(outbuffer + offset);
      offset += this->z.serialize(outbuffer + offset);
      offset += this->pitch.serialize(outbuffer + offset);
      offset += this->roll.serialize(outbuffer + offset);
      offset += this->yaw.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->x.deserialize(inbuffer + offset);
      offset += this->y.deserialize(inbuffer + offset);
      offset += this->z.deserialize(inbuffer + offset);
      offset += this->pitch.deserialize(inbuffer + offset);
      offset += this->roll.deserialize(inbuffer + offset);
      offset += this->yaw.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/PoseControlPid"; };
    const char * getMD5(){ return "5ccc2d67e6f0c4b5fe0edec49b41f733"; };

  };

}
#endif
