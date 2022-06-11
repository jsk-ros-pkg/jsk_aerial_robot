#ifndef _ROS_jsk_network_tools_OpenNISample_h
#define _ROS_jsk_network_tools_OpenNISample_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

namespace jsk_network_tools
{

  class OpenNISample : public ros::Msg
  {
    public:
      typedef sensor_msgs::Image _camera__rgb__image_rect_color_type;
      _camera__rgb__image_rect_color_type camera__rgb__image_rect_color;
      typedef sensor_msgs::PointCloud2 _camera__depth_registered__points_type;
      _camera__depth_registered__points_type camera__depth_registered__points;

    OpenNISample():
      camera__rgb__image_rect_color(),
      camera__depth_registered__points()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->camera__rgb__image_rect_color.serialize(outbuffer + offset);
      offset += this->camera__depth_registered__points.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->camera__rgb__image_rect_color.deserialize(inbuffer + offset);
      offset += this->camera__depth_registered__points.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/OpenNISample"; };
    const char * getMD5(){ return "c998c0df481d0ad4598a2534cadda5d1"; };

  };

}
#endif
