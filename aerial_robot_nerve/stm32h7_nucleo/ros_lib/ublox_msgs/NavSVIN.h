#ifndef _ROS_ublox_msgs_NavSVIN_h
#define _ROS_ublox_msgs_NavSVIN_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavSVIN : public ros::Msg
  {
    public:
      typedef uint8_t _version_type;
      _version_type version;
      uint8_t reserved0[3];
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint32_t _dur_type;
      _dur_type dur;
      typedef int32_t _meanX_type;
      _meanX_type meanX;
      typedef int32_t _meanY_type;
      _meanY_type meanY;
      typedef int32_t _meanZ_type;
      _meanZ_type meanZ;
      typedef int8_t _meanXHP_type;
      _meanXHP_type meanXHP;
      typedef int8_t _meanYHP_type;
      _meanYHP_type meanYHP;
      typedef int8_t _meanZHP_type;
      _meanZHP_type meanZHP;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      typedef uint32_t _meanAcc_type;
      _meanAcc_type meanAcc;
      typedef uint32_t _obs_type;
      _obs_type obs;
      typedef uint8_t _valid_type;
      _valid_type valid;
      typedef uint8_t _active_type;
      _active_type active;
      uint8_t reserved3[2];
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  59 };

    NavSVIN():
      version(0),
      reserved0(),
      iTOW(0),
      dur(0),
      meanX(0),
      meanY(0),
      meanZ(0),
      meanXHP(0),
      meanYHP(0),
      meanZHP(0),
      reserved1(0),
      meanAcc(0),
      obs(0),
      valid(0),
      active(0),
      reserved3()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      *(outbuffer + offset + 0) = (this->dur >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dur >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dur >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dur >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dur);
      union {
        int32_t real;
        uint32_t base;
      } u_meanX;
      u_meanX.real = this->meanX;
      *(outbuffer + offset + 0) = (u_meanX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_meanX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_meanX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_meanX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meanX);
      union {
        int32_t real;
        uint32_t base;
      } u_meanY;
      u_meanY.real = this->meanY;
      *(outbuffer + offset + 0) = (u_meanY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_meanY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_meanY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_meanY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meanY);
      union {
        int32_t real;
        uint32_t base;
      } u_meanZ;
      u_meanZ.real = this->meanZ;
      *(outbuffer + offset + 0) = (u_meanZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_meanZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_meanZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_meanZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meanZ);
      union {
        int8_t real;
        uint8_t base;
      } u_meanXHP;
      u_meanXHP.real = this->meanXHP;
      *(outbuffer + offset + 0) = (u_meanXHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->meanXHP);
      union {
        int8_t real;
        uint8_t base;
      } u_meanYHP;
      u_meanYHP.real = this->meanYHP;
      *(outbuffer + offset + 0) = (u_meanYHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->meanYHP);
      union {
        int8_t real;
        uint8_t base;
      } u_meanZHP;
      u_meanZHP.real = this->meanZHP;
      *(outbuffer + offset + 0) = (u_meanZHP.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->meanZHP);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->meanAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->meanAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->meanAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->meanAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meanAcc);
      *(outbuffer + offset + 0) = (this->obs >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obs >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obs >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obs >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obs);
      *(outbuffer + offset + 0) = (this->valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->valid);
      *(outbuffer + offset + 0) = (this->active >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved3[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved3[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      this->dur =  ((uint32_t) (*(inbuffer + offset)));
      this->dur |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dur |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->dur |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->dur);
      union {
        int32_t real;
        uint32_t base;
      } u_meanX;
      u_meanX.base = 0;
      u_meanX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_meanX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_meanX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_meanX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->meanX = u_meanX.real;
      offset += sizeof(this->meanX);
      union {
        int32_t real;
        uint32_t base;
      } u_meanY;
      u_meanY.base = 0;
      u_meanY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_meanY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_meanY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_meanY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->meanY = u_meanY.real;
      offset += sizeof(this->meanY);
      union {
        int32_t real;
        uint32_t base;
      } u_meanZ;
      u_meanZ.base = 0;
      u_meanZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_meanZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_meanZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_meanZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->meanZ = u_meanZ.real;
      offset += sizeof(this->meanZ);
      union {
        int8_t real;
        uint8_t base;
      } u_meanXHP;
      u_meanXHP.base = 0;
      u_meanXHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->meanXHP = u_meanXHP.real;
      offset += sizeof(this->meanXHP);
      union {
        int8_t real;
        uint8_t base;
      } u_meanYHP;
      u_meanYHP.base = 0;
      u_meanYHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->meanYHP = u_meanYHP.real;
      offset += sizeof(this->meanYHP);
      union {
        int8_t real;
        uint8_t base;
      } u_meanZHP;
      u_meanZHP.base = 0;
      u_meanZHP.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->meanZHP = u_meanZHP.real;
      offset += sizeof(this->meanZHP);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
      this->meanAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->meanAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->meanAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->meanAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->meanAcc);
      this->obs =  ((uint32_t) (*(inbuffer + offset)));
      this->obs |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->obs |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->obs |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->obs);
      this->valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->valid);
      this->active =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->active);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved3[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved3[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavSVIN"; };
    virtual const char * getMD5() override { return "c78016f30ce026b9e670e40dd151a8ac"; };

  };

}
#endif
