#ifndef _ROS_aerial_robot_msgs_AerialRobotStatus_h
#define _ROS_aerial_robot_msgs_AerialRobotStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace aerial_robot_msgs
{

  class AerialRobotStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _percentage_voltage_type;
      _percentage_voltage_type percentage_voltage;

    AerialRobotStatus():
      header(),
      voltage(0),
      percentage_voltage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_percentage_voltage;
      u_percentage_voltage.real = this->percentage_voltage;
      *(outbuffer + offset + 0) = (u_percentage_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_percentage_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_percentage_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_percentage_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->percentage_voltage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_percentage_voltage;
      u_percentage_voltage.base = 0;
      u_percentage_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_percentage_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_percentage_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_percentage_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->percentage_voltage = u_percentage_voltage.real;
      offset += sizeof(this->percentage_voltage);
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/AerialRobotStatus"; };
    const char * getMD5(){ return "17b301493ace3020149ec8c93dfedaf2"; };

  };

}
#endif
