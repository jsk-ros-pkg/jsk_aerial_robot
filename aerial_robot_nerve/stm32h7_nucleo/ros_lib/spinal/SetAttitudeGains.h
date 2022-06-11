#ifndef _ROS_SERVICE_SetAttitudeGains_h
#define _ROS_SERVICE_SetAttitudeGains_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

static const char SETATTITUDEGAINS[] = "spinal/SetAttitudeGains";

  class SetAttitudeGainsRequest : public ros::Msg
  {
    public:
      typedef float _roll_pitch_p_type;
      _roll_pitch_p_type roll_pitch_p;
      typedef float _roll_pitch_i_type;
      _roll_pitch_i_type roll_pitch_i;
      typedef float _roll_pitch_d_type;
      _roll_pitch_d_type roll_pitch_d;
      typedef float _yaw_d_type;
      _yaw_d_type yaw_d;
      typedef float _roll_pitch_limit_type;
      _roll_pitch_limit_type roll_pitch_limit;
      typedef float _roll_pitch_p_limit_type;
      _roll_pitch_p_limit_type roll_pitch_p_limit;
      typedef float _roll_pitch_i_limit_type;
      _roll_pitch_i_limit_type roll_pitch_i_limit;
      typedef float _roll_pitch_d_limit_type;
      _roll_pitch_d_limit_type roll_pitch_d_limit;
      typedef float _yaw_d_limit_type;
      _yaw_d_limit_type yaw_d_limit;

    SetAttitudeGainsRequest():
      roll_pitch_p(0),
      roll_pitch_i(0),
      roll_pitch_d(0),
      yaw_d(0),
      roll_pitch_limit(0),
      roll_pitch_p_limit(0),
      roll_pitch_i_limit(0),
      roll_pitch_d_limit(0),
      yaw_d_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_p;
      u_roll_pitch_p.real = this->roll_pitch_p;
      *(outbuffer + offset + 0) = (u_roll_pitch_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_p);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_i;
      u_roll_pitch_i.real = this->roll_pitch_i;
      *(outbuffer + offset + 0) = (u_roll_pitch_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_i);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_d;
      u_roll_pitch_d.real = this->roll_pitch_d;
      *(outbuffer + offset + 0) = (u_roll_pitch_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_d);
      union {
        float real;
        uint32_t base;
      } u_yaw_d;
      u_yaw_d.real = this->yaw_d;
      *(outbuffer + offset + 0) = (u_yaw_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_d);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_limit;
      u_roll_pitch_limit.real = this->roll_pitch_limit;
      *(outbuffer + offset + 0) = (u_roll_pitch_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_p_limit;
      u_roll_pitch_p_limit.real = this->roll_pitch_p_limit;
      *(outbuffer + offset + 0) = (u_roll_pitch_p_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_p_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_p_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_p_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_p_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_i_limit;
      u_roll_pitch_i_limit.real = this->roll_pitch_i_limit;
      *(outbuffer + offset + 0) = (u_roll_pitch_i_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_i_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_i_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_i_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_i_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_d_limit;
      u_roll_pitch_d_limit.real = this->roll_pitch_d_limit;
      *(outbuffer + offset + 0) = (u_roll_pitch_d_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_pitch_d_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_pitch_d_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_pitch_d_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_pitch_d_limit);
      union {
        float real;
        uint32_t base;
      } u_yaw_d_limit;
      u_yaw_d_limit.real = this->yaw_d_limit;
      *(outbuffer + offset + 0) = (u_yaw_d_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_d_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_d_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_d_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_d_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_p;
      u_roll_pitch_p.base = 0;
      u_roll_pitch_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_p = u_roll_pitch_p.real;
      offset += sizeof(this->roll_pitch_p);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_i;
      u_roll_pitch_i.base = 0;
      u_roll_pitch_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_i = u_roll_pitch_i.real;
      offset += sizeof(this->roll_pitch_i);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_d;
      u_roll_pitch_d.base = 0;
      u_roll_pitch_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_d = u_roll_pitch_d.real;
      offset += sizeof(this->roll_pitch_d);
      union {
        float real;
        uint32_t base;
      } u_yaw_d;
      u_yaw_d.base = 0;
      u_yaw_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_d = u_yaw_d.real;
      offset += sizeof(this->yaw_d);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_limit;
      u_roll_pitch_limit.base = 0;
      u_roll_pitch_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_limit = u_roll_pitch_limit.real;
      offset += sizeof(this->roll_pitch_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_p_limit;
      u_roll_pitch_p_limit.base = 0;
      u_roll_pitch_p_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_p_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_p_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_p_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_p_limit = u_roll_pitch_p_limit.real;
      offset += sizeof(this->roll_pitch_p_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_i_limit;
      u_roll_pitch_i_limit.base = 0;
      u_roll_pitch_i_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_i_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_i_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_i_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_i_limit = u_roll_pitch_i_limit.real;
      offset += sizeof(this->roll_pitch_i_limit);
      union {
        float real;
        uint32_t base;
      } u_roll_pitch_d_limit;
      u_roll_pitch_d_limit.base = 0;
      u_roll_pitch_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_pitch_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_pitch_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_pitch_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_pitch_d_limit = u_roll_pitch_d_limit.real;
      offset += sizeof(this->roll_pitch_d_limit);
      union {
        float real;
        uint32_t base;
      } u_yaw_d_limit;
      u_yaw_d_limit.base = 0;
      u_yaw_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_d_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_d_limit = u_yaw_d_limit.real;
      offset += sizeof(this->yaw_d_limit);
     return offset;
    }

    const char * getType(){ return SETATTITUDEGAINS; };
    const char * getMD5(){ return "baad126f11e35087f7592bf3e181f0d9"; };

  };

  class SetAttitudeGainsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetAttitudeGainsResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETATTITUDEGAINS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetAttitudeGains {
    public:
    typedef SetAttitudeGainsRequest Request;
    typedef SetAttitudeGainsResponse Response;
  };

}
#endif
