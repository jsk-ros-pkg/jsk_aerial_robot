#ifndef _ROS_aerial_robot_msgs_FlightNav_h
#define _ROS_aerial_robot_msgs_FlightNav_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace aerial_robot_msgs
{

  class FlightNav : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _control_frame_type;
      _control_frame_type control_frame;
      typedef uint8_t _target_type;
      _target_type target;
      typedef uint8_t _pos_xy_nav_mode_type;
      _pos_xy_nav_mode_type pos_xy_nav_mode;
      typedef double _target_pos_x_type;
      _target_pos_x_type target_pos_x;
      typedef float _target_vel_x_type;
      _target_vel_x_type target_vel_x;
      typedef float _target_acc_x_type;
      _target_acc_x_type target_acc_x;
      typedef double _target_pos_y_type;
      _target_pos_y_type target_pos_y;
      typedef float _target_vel_y_type;
      _target_vel_y_type target_vel_y;
      typedef float _target_acc_y_type;
      _target_acc_y_type target_acc_y;
      typedef uint8_t _yaw_nav_mode_type;
      _yaw_nav_mode_type yaw_nav_mode;
      typedef float _target_omega_z_type;
      _target_omega_z_type target_omega_z;
      typedef float _target_yaw_type;
      _target_yaw_type target_yaw;
      typedef uint8_t _pos_z_nav_mode_type;
      _pos_z_nav_mode_type pos_z_nav_mode;
      typedef float _target_pos_z_type;
      _target_pos_z_type target_pos_z;
      typedef float _target_vel_z_type;
      _target_vel_z_type target_vel_z;
      typedef float _target_pos_diff_z_type;
      _target_pos_diff_z_type target_pos_diff_z;
      enum { NO_NAVIGATION =  0 };
      enum { VEL_MODE =  1 };
      enum { POS_MODE =  2 };
      enum { ACC_MODE =  3 };
      enum { POS_VEL_MODE =  4 };
      enum { GPS_WAYPOINT_MODE =  5 };
      enum { WORLD_FRAME =  0 };
      enum { LOCAL_FRAME =  1 };
      enum { BASELINK =  0 };
      enum { COG =  1 };

    FlightNav():
      header(),
      control_frame(0),
      target(0),
      pos_xy_nav_mode(0),
      target_pos_x(0),
      target_vel_x(0),
      target_acc_x(0),
      target_pos_y(0),
      target_vel_y(0),
      target_acc_y(0),
      yaw_nav_mode(0),
      target_omega_z(0),
      target_yaw(0),
      pos_z_nav_mode(0),
      target_pos_z(0),
      target_vel_z(0),
      target_pos_diff_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->control_frame >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control_frame);
      *(outbuffer + offset + 0) = (this->target >> (8 * 0)) & 0xFF;
      offset += sizeof(this->target);
      *(outbuffer + offset + 0) = (this->pos_xy_nav_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pos_xy_nav_mode);
      union {
        double real;
        uint64_t base;
      } u_target_pos_x;
      u_target_pos_x.real = this->target_pos_x;
      *(outbuffer + offset + 0) = (u_target_pos_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_pos_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_pos_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_pos_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_target_pos_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_target_pos_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_target_pos_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_target_pos_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->target_pos_x);
      union {
        float real;
        uint32_t base;
      } u_target_vel_x;
      u_target_vel_x.real = this->target_vel_x;
      *(outbuffer + offset + 0) = (u_target_vel_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_vel_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_vel_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_vel_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_vel_x);
      union {
        float real;
        uint32_t base;
      } u_target_acc_x;
      u_target_acc_x.real = this->target_acc_x;
      *(outbuffer + offset + 0) = (u_target_acc_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_acc_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_acc_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_acc_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_acc_x);
      union {
        double real;
        uint64_t base;
      } u_target_pos_y;
      u_target_pos_y.real = this->target_pos_y;
      *(outbuffer + offset + 0) = (u_target_pos_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_pos_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_pos_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_pos_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_target_pos_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_target_pos_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_target_pos_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_target_pos_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->target_pos_y);
      union {
        float real;
        uint32_t base;
      } u_target_vel_y;
      u_target_vel_y.real = this->target_vel_y;
      *(outbuffer + offset + 0) = (u_target_vel_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_vel_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_vel_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_vel_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_vel_y);
      union {
        float real;
        uint32_t base;
      } u_target_acc_y;
      u_target_acc_y.real = this->target_acc_y;
      *(outbuffer + offset + 0) = (u_target_acc_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_acc_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_acc_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_acc_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_acc_y);
      *(outbuffer + offset + 0) = (this->yaw_nav_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->yaw_nav_mode);
      union {
        float real;
        uint32_t base;
      } u_target_omega_z;
      u_target_omega_z.real = this->target_omega_z;
      *(outbuffer + offset + 0) = (u_target_omega_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_omega_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_omega_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_omega_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_omega_z);
      union {
        float real;
        uint32_t base;
      } u_target_yaw;
      u_target_yaw.real = this->target_yaw;
      *(outbuffer + offset + 0) = (u_target_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_yaw);
      *(outbuffer + offset + 0) = (this->pos_z_nav_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pos_z_nav_mode);
      union {
        float real;
        uint32_t base;
      } u_target_pos_z;
      u_target_pos_z.real = this->target_pos_z;
      *(outbuffer + offset + 0) = (u_target_pos_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_pos_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_pos_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_pos_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_pos_z);
      union {
        float real;
        uint32_t base;
      } u_target_vel_z;
      u_target_vel_z.real = this->target_vel_z;
      *(outbuffer + offset + 0) = (u_target_vel_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_vel_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_vel_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_vel_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_vel_z);
      union {
        float real;
        uint32_t base;
      } u_target_pos_diff_z;
      u_target_pos_diff_z.real = this->target_pos_diff_z;
      *(outbuffer + offset + 0) = (u_target_pos_diff_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_pos_diff_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_pos_diff_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_pos_diff_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_pos_diff_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->control_frame =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->control_frame);
      this->target =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->target);
      this->pos_xy_nav_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pos_xy_nav_mode);
      union {
        double real;
        uint64_t base;
      } u_target_pos_x;
      u_target_pos_x.base = 0;
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_target_pos_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->target_pos_x = u_target_pos_x.real;
      offset += sizeof(this->target_pos_x);
      union {
        float real;
        uint32_t base;
      } u_target_vel_x;
      u_target_vel_x.base = 0;
      u_target_vel_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_vel_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_vel_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_vel_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_vel_x = u_target_vel_x.real;
      offset += sizeof(this->target_vel_x);
      union {
        float real;
        uint32_t base;
      } u_target_acc_x;
      u_target_acc_x.base = 0;
      u_target_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_acc_x = u_target_acc_x.real;
      offset += sizeof(this->target_acc_x);
      union {
        double real;
        uint64_t base;
      } u_target_pos_y;
      u_target_pos_y.base = 0;
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_target_pos_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->target_pos_y = u_target_pos_y.real;
      offset += sizeof(this->target_pos_y);
      union {
        float real;
        uint32_t base;
      } u_target_vel_y;
      u_target_vel_y.base = 0;
      u_target_vel_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_vel_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_vel_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_vel_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_vel_y = u_target_vel_y.real;
      offset += sizeof(this->target_vel_y);
      union {
        float real;
        uint32_t base;
      } u_target_acc_y;
      u_target_acc_y.base = 0;
      u_target_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_acc_y = u_target_acc_y.real;
      offset += sizeof(this->target_acc_y);
      this->yaw_nav_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->yaw_nav_mode);
      union {
        float real;
        uint32_t base;
      } u_target_omega_z;
      u_target_omega_z.base = 0;
      u_target_omega_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_omega_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_omega_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_omega_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_omega_z = u_target_omega_z.real;
      offset += sizeof(this->target_omega_z);
      union {
        float real;
        uint32_t base;
      } u_target_yaw;
      u_target_yaw.base = 0;
      u_target_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_yaw = u_target_yaw.real;
      offset += sizeof(this->target_yaw);
      this->pos_z_nav_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pos_z_nav_mode);
      union {
        float real;
        uint32_t base;
      } u_target_pos_z;
      u_target_pos_z.base = 0;
      u_target_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_pos_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_pos_z = u_target_pos_z.real;
      offset += sizeof(this->target_pos_z);
      union {
        float real;
        uint32_t base;
      } u_target_vel_z;
      u_target_vel_z.base = 0;
      u_target_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_vel_z = u_target_vel_z.real;
      offset += sizeof(this->target_vel_z);
      union {
        float real;
        uint32_t base;
      } u_target_pos_diff_z;
      u_target_pos_diff_z.base = 0;
      u_target_pos_diff_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_pos_diff_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_pos_diff_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_pos_diff_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_pos_diff_z = u_target_pos_diff_z.real;
      offset += sizeof(this->target_pos_diff_z);
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/FlightNav"; };
    const char * getMD5(){ return "df87572165cf24167ac560fd227d6b26"; };

  };

}
#endif
