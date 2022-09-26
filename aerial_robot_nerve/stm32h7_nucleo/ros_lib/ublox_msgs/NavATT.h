#ifndef _ROS_ublox_msgs_NavATT_h
#define _ROS_ublox_msgs_NavATT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavATT : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _version_type;
      _version_type version;
      uint8_t reserved0[3];
      typedef int32_t _roll_type;
      _roll_type roll;
      typedef int32_t _pitch_type;
      _pitch_type pitch;
      typedef int32_t _heading_type;
      _heading_type heading;
      typedef uint32_t _accRoll_type;
      _accRoll_type accRoll;
      typedef uint32_t _accPitch_type;
      _accPitch_type accPitch;
      typedef uint32_t _accHeading_type;
      _accHeading_type accHeading;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  5 };

    NavATT():
      iTOW(0),
      version(0),
      reserved0(),
      roll(0),
      pitch(0),
      heading(0),
      accRoll(0),
      accPitch(0),
      accHeading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        int32_t real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      *(outbuffer + offset + 0) = (this->accRoll >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accRoll >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accRoll >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accRoll >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accRoll);
      *(outbuffer + offset + 0) = (this->accPitch >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accPitch >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accPitch >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accPitch >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accPitch);
      *(outbuffer + offset + 0) = (this->accHeading >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accHeading >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accHeading >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accHeading >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accHeading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        int32_t real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        int32_t real;
        uint32_t base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
      this->accRoll =  ((uint32_t) (*(inbuffer + offset)));
      this->accRoll |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accRoll |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accRoll |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accRoll);
      this->accPitch =  ((uint32_t) (*(inbuffer + offset)));
      this->accPitch |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accPitch |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accPitch |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accPitch);
      this->accHeading =  ((uint32_t) (*(inbuffer + offset)));
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->accHeading |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->accHeading);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavATT"; };
    virtual const char * getMD5() override { return "5d7fd152cc974cdd6905d89f564451b6"; };

  };

}
#endif
