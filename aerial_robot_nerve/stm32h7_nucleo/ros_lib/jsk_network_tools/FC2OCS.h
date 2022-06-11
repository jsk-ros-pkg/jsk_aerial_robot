#ifndef _ROS_jsk_network_tools_FC2OCS_h
#define _ROS_jsk_network_tools_FC2OCS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_network_tools
{

  class FC2OCS : public ros::Msg
  {
    public:
      uint8_t joint_angles[32];
      uint8_t lhand_force[6];
      uint8_t rhand_force[6];
      uint8_t lfoot_force[6];
      uint8_t rfoot_force[6];
      typedef bool _servo_state_type;
      _servo_state_type servo_state;

    FC2OCS():
      joint_angles(),
      lhand_force(),
      rhand_force(),
      lfoot_force(),
      rfoot_force(),
      servo_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 32; i++){
      *(outbuffer + offset + 0) = (this->joint_angles[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->joint_angles[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->lhand_force[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lhand_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->rhand_force[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rhand_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->lfoot_force[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lfoot_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->rfoot_force[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rfoot_force[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_servo_state;
      u_servo_state.real = this->servo_state;
      *(outbuffer + offset + 0) = (u_servo_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servo_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 32; i++){
      this->joint_angles[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->joint_angles[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      this->lhand_force[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->lhand_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      this->rhand_force[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rhand_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      this->lfoot_force[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->lfoot_force[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      this->rfoot_force[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rfoot_force[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_servo_state;
      u_servo_state.base = 0;
      u_servo_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->servo_state = u_servo_state.real;
      offset += sizeof(this->servo_state);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/FC2OCS"; };
    const char * getMD5(){ return "7a556e2b1084dcaa36eeac7b2f905853"; };

  };

}
#endif
