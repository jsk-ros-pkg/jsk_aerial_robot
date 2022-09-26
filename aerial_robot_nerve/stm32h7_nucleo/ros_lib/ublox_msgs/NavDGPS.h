#ifndef _ROS_ublox_msgs_NavDGPS_h
#define _ROS_ublox_msgs_NavDGPS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/NavDGPS_SV.h"

namespace ublox_msgs
{

  class NavDGPS : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _age_type;
      _age_type age;
      typedef int16_t _baseId_type;
      _baseId_type baseId;
      typedef int16_t _baseHealth_type;
      _baseHealth_type baseHealth;
      typedef int8_t _numCh_type;
      _numCh_type numCh;
      typedef uint8_t _status_type;
      _status_type status;
      typedef uint16_t _reserved1_type;
      _reserved1_type reserved1;
      uint32_t sv_length;
      typedef ublox_msgs::NavDGPS_SV _sv_type;
      _sv_type st_sv;
      _sv_type * sv;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  49 };
      enum { DGPS_CORRECTION_NONE =  0 };
      enum { DGPS_CORRECTION_PR_PRR =  1 };

    NavDGPS():
      iTOW(0),
      age(0),
      baseId(0),
      baseHealth(0),
      numCh(0),
      status(0),
      reserved1(0),
      sv_length(0), st_sv(), sv(nullptr)
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
      union {
        int32_t real;
        uint32_t base;
      } u_age;
      u_age.real = this->age;
      *(outbuffer + offset + 0) = (u_age.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_age.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_age.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_age.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->age);
      union {
        int16_t real;
        uint16_t base;
      } u_baseId;
      u_baseId.real = this->baseId;
      *(outbuffer + offset + 0) = (u_baseId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_baseId.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->baseId);
      union {
        int16_t real;
        uint16_t base;
      } u_baseHealth;
      u_baseHealth.real = this->baseHealth;
      *(outbuffer + offset + 0) = (u_baseHealth.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_baseHealth.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->baseHealth);
      union {
        int8_t real;
        uint8_t base;
      } u_numCh;
      u_numCh.real = this->numCh;
      *(outbuffer + offset + 0) = (u_numCh.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numCh);
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserved1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->reserved1);
      *(outbuffer + offset + 0) = (this->sv_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sv_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sv_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sv_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sv_length);
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->sv[i].serialize(outbuffer + offset);
      }
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
      union {
        int32_t real;
        uint32_t base;
      } u_age;
      u_age.base = 0;
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->age = u_age.real;
      offset += sizeof(this->age);
      union {
        int16_t real;
        uint16_t base;
      } u_baseId;
      u_baseId.base = 0;
      u_baseId.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_baseId.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->baseId = u_baseId.real;
      offset += sizeof(this->baseId);
      union {
        int16_t real;
        uint16_t base;
      } u_baseHealth;
      u_baseHealth.base = 0;
      u_baseHealth.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_baseHealth.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->baseHealth = u_baseHealth.real;
      offset += sizeof(this->baseHealth);
      union {
        int8_t real;
        uint8_t base;
      } u_numCh;
      u_numCh.base = 0;
      u_numCh.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->numCh = u_numCh.real;
      offset += sizeof(this->numCh);
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->reserved1 =  ((uint16_t) (*(inbuffer + offset)));
      this->reserved1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->reserved1);
      uint32_t sv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_length);
      if(sv_lengthT > sv_length)
        this->sv = (ublox_msgs::NavDGPS_SV*)realloc(this->sv, sv_lengthT * sizeof(ublox_msgs::NavDGPS_SV));
      sv_length = sv_lengthT;
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->st_sv.deserialize(inbuffer + offset);
        memcpy( &(this->sv[i]), &(this->st_sv), sizeof(ublox_msgs::NavDGPS_SV));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavDGPS"; };
    virtual const char * getMD5() override { return "b40fcf35e803e9b5ccedf7c1c7ca332d"; };

  };

}
#endif
