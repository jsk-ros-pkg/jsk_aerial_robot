#ifndef _ROS_jsk_network_tools_FC2OCSLargeData_h
#define _ROS_jsk_network_tools_FC2OCSLargeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/JointState.h"

namespace jsk_network_tools
{

  class FC2OCSLargeData : public ros::Msg
  {
    public:
      typedef sensor_msgs::Image _camera__rgb__image_rect_color_type;
      _camera__rgb__image_rect_color_type camera__rgb__image_rect_color;
      typedef sensor_msgs::JointState _joint_state_type;
      _joint_state_type joint_state;
      typedef sensor_msgs::Image _usb_cam__image_raw_type;
      _usb_cam__image_raw_type usb_cam__image_raw;

    FC2OCSLargeData():
      camera__rgb__image_rect_color(),
      joint_state(),
      usb_cam__image_raw()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->camera__rgb__image_rect_color.serialize(outbuffer + offset);
      offset += this->joint_state.serialize(outbuffer + offset);
      offset += this->usb_cam__image_raw.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->camera__rgb__image_rect_color.deserialize(inbuffer + offset);
      offset += this->joint_state.deserialize(inbuffer + offset);
      offset += this->usb_cam__image_raw.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/FC2OCSLargeData"; };
    const char * getMD5(){ return "51c7ca6e66514e69ddd1a156f6a0b404"; };

  };

}
#endif
