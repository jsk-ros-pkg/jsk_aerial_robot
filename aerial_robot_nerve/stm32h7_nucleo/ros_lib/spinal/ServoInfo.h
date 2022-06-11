#ifndef _ROS_spinal_ServoInfo_h
#define _ROS_spinal_ServoInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class ServoInfo : public ros::Msg
  {
    public:
      typedef uint8_t _id_type;
      _id_type id;
      typedef uint16_t _p_gain_type;
      _p_gain_type p_gain;
      typedef uint16_t _i_gain_type;
      _i_gain_type i_gain;
      typedef uint16_t _d_gain_type;
      _d_gain_type d_gain;
      typedef uint16_t _profile_velocity_type;
      _profile_velocity_type profile_velocity;
      typedef uint16_t _current_limit_type;
      _current_limit_type current_limit;
      typedef uint8_t _send_data_flag_type;
      _send_data_flag_type send_data_flag;
      typedef uint8_t _external_encoder_flag_type;
      _external_encoder_flag_type external_encoder_flag;
      typedef uint16_t _joint_resolution_type;
      _joint_resolution_type joint_resolution;
      typedef uint16_t _servo_resolution_type;
      _servo_resolution_type servo_resolution;

    ServoInfo():
      id(0),
      p_gain(0),
      i_gain(0),
      d_gain(0),
      profile_velocity(0),
      current_limit(0),
      send_data_flag(0),
      external_encoder_flag(0),
      joint_resolution(0),
      servo_resolution(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->p_gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->p_gain);
      *(outbuffer + offset + 0) = (this->i_gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->i_gain);
      *(outbuffer + offset + 0) = (this->d_gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->d_gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->d_gain);
      *(outbuffer + offset + 0) = (this->profile_velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->profile_velocity >> (8 * 1)) & 0xFF;
      offset += sizeof(this->profile_velocity);
      *(outbuffer + offset + 0) = (this->current_limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_limit);
      *(outbuffer + offset + 0) = (this->send_data_flag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->send_data_flag);
      *(outbuffer + offset + 0) = (this->external_encoder_flag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->external_encoder_flag);
      *(outbuffer + offset + 0) = (this->joint_resolution >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_resolution >> (8 * 1)) & 0xFF;
      offset += sizeof(this->joint_resolution);
      *(outbuffer + offset + 0) = (this->servo_resolution >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servo_resolution >> (8 * 1)) & 0xFF;
      offset += sizeof(this->servo_resolution);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      this->p_gain =  ((uint16_t) (*(inbuffer + offset)));
      this->p_gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->p_gain);
      this->i_gain =  ((uint16_t) (*(inbuffer + offset)));
      this->i_gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->i_gain);
      this->d_gain =  ((uint16_t) (*(inbuffer + offset)));
      this->d_gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->d_gain);
      this->profile_velocity =  ((uint16_t) (*(inbuffer + offset)));
      this->profile_velocity |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->profile_velocity);
      this->current_limit =  ((uint16_t) (*(inbuffer + offset)));
      this->current_limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->current_limit);
      this->send_data_flag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->send_data_flag);
      this->external_encoder_flag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->external_encoder_flag);
      this->joint_resolution =  ((uint16_t) (*(inbuffer + offset)));
      this->joint_resolution |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->joint_resolution);
      this->servo_resolution =  ((uint16_t) (*(inbuffer + offset)));
      this->servo_resolution |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->servo_resolution);
     return offset;
    }

    const char * getType(){ return "spinal/ServoInfo"; };
    const char * getMD5(){ return "8485b0923dc96bd6fbbd277cbf5cd19d"; };

  };

}
#endif
