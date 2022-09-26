#ifndef _ROS_jsk_recognition_msgs_SnapItRequest_h
#define _ROS_jsk_recognition_msgs_SnapItRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

namespace jsk_recognition_msgs
{

  class SnapItRequest : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _model_type_type;
      _model_type_type model_type;
      typedef geometry_msgs::PolygonStamped _target_plane_type;
      _target_plane_type target_plane;
      typedef geometry_msgs::PointStamped _center_type;
      _center_type center;
      typedef geometry_msgs::Vector3Stamped _direction_type;
      _direction_type direction;
      typedef double _radius_type;
      _radius_type radius;
      typedef double _height_type;
      _height_type height;
      typedef double _max_distance_type;
      _max_distance_type max_distance;
      typedef double _eps_angle_type;
      _eps_angle_type eps_angle;
      enum { MODEL_PLANE = 0 };
      enum { MODEL_CYLINDER = 1 };

    SnapItRequest():
      header(),
      model_type(0),
      target_plane(),
      center(),
      direction(),
      radius(0),
      height(0),
      max_distance(0),
      eps_angle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->model_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->model_type);
      offset += this->target_plane.serialize(outbuffer + offset);
      offset += this->center.serialize(outbuffer + offset);
      offset += this->direction.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.real = this->radius;
      *(outbuffer + offset + 0) = (u_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_radius.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_radius.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_radius.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_radius.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->radius);
      union {
        double real;
        uint64_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_height.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_height.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_height.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_height.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_max_distance;
      u_max_distance.real = this->max_distance;
      *(outbuffer + offset + 0) = (u_max_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_distance);
      union {
        double real;
        uint64_t base;
      } u_eps_angle;
      u_eps_angle.real = this->eps_angle;
      *(outbuffer + offset + 0) = (u_eps_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_eps_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_eps_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_eps_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_eps_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_eps_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_eps_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_eps_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->eps_angle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->model_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->model_type);
      offset += this->target_plane.deserialize(inbuffer + offset);
      offset += this->center.deserialize(inbuffer + offset);
      offset += this->direction.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.base = 0;
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->radius = u_radius.real;
      offset += sizeof(this->radius);
      union {
        double real;
        uint64_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_height.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        double real;
        uint64_t base;
      } u_max_distance;
      u_max_distance.base = 0;
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_distance = u_max_distance.real;
      offset += sizeof(this->max_distance);
      union {
        double real;
        uint64_t base;
      } u_eps_angle;
      u_eps_angle.base = 0;
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_eps_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->eps_angle = u_eps_angle.real;
      offset += sizeof(this->eps_angle);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/SnapItRequest"; };
    virtual const char * getMD5() override { return "5733f480694296678d81cff0483b399b"; };

  };

}
#endif
