#ifndef _ROS_jsk_network_tools_OCS2FC_h
#define _ROS_jsk_network_tools_OCS2FC_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_network_tools
{

  class OCS2FC : public ros::Msg
  {
    public:
      uint8_t joint_angles[32];
      typedef bool _start_impedance_type;
      _start_impedance_type start_impedance;
      typedef bool _stop_type;
      _stop_type stop;

    OCS2FC():
      joint_angles(),
      start_impedance(0),
      stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 32; i++){
      *(outbuffer + offset + 0) = (this->joint_angles[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_angles[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_start_impedance;
      u_start_impedance.real = this->start_impedance;
      *(outbuffer + offset + 0) = (u_start_impedance.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start_impedance);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.real = this->stop;
      *(outbuffer + offset + 0) = (u_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 32; i++){
      this->joint_angles[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->joint_angles[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_start_impedance;
      u_start_impedance.base = 0;
      u_start_impedance.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start_impedance = u_start_impedance.real;
      offset += sizeof(this->start_impedance);
      union {
        bool real;
        uint8_t base;
      } u_stop;
      u_stop.base = 0;
      u_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stop = u_stop.real;
      offset += sizeof(this->stop);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/OCS2FC"; };
    const char * getMD5(){ return "036058ee69589b4817d296161ea7f432"; };

  };

}
#endif
