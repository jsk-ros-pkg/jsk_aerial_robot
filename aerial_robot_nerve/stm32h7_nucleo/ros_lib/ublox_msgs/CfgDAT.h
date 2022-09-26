#ifndef _ROS_ublox_msgs_CfgDAT_h
#define _ROS_ublox_msgs_CfgDAT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgDAT : public ros::Msg
  {
    public:
      typedef uint16_t _datumNum_type;
      _datumNum_type datumNum;
      uint8_t datumName[6];
      typedef double _majA_type;
      _majA_type majA;
      typedef double _flat_type;
      _flat_type flat;
      typedef float _dX_type;
      _dX_type dX;
      typedef float _dY_type;
      _dY_type dY;
      typedef float _dZ_type;
      _dZ_type dZ;
      typedef float _rotX_type;
      _rotX_type rotX;
      typedef float _rotY_type;
      _rotY_type rotY;
      typedef float _rotZ_type;
      _rotZ_type rotZ;
      typedef float _scale_type;
      _scale_type scale;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  6 };
      enum { DATUM_NUM_WGS84 =  0 };
      enum { DATUM_NUM_USER =  65535 };

    CfgDAT():
      datumNum(0),
      datumName(),
      majA(0),
      flat(0),
      dX(0),
      dY(0),
      dZ(0),
      rotX(0),
      rotY(0),
      rotZ(0),
      scale(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->datumNum >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->datumNum >> (8 * 1)) & 0xFF;
      offset += sizeof(this->datumNum);
      for( uint32_t i = 0; i < 6; i++){
      *(outbuffer + offset + 0) = (this->datumName[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->datumName[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_majA;
      u_majA.real = this->majA;
      *(outbuffer + offset + 0) = (u_majA.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_majA.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_majA.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_majA.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_majA.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_majA.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_majA.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_majA.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->majA);
      union {
        double real;
        uint64_t base;
      } u_flat;
      u_flat.real = this->flat;
      *(outbuffer + offset + 0) = (u_flat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flat.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_flat.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_flat.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_flat.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_flat.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->flat);
      union {
        float real;
        uint32_t base;
      } u_dX;
      u_dX.real = this->dX;
      *(outbuffer + offset + 0) = (u_dX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dX);
      union {
        float real;
        uint32_t base;
      } u_dY;
      u_dY.real = this->dY;
      *(outbuffer + offset + 0) = (u_dY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dY);
      union {
        float real;
        uint32_t base;
      } u_dZ;
      u_dZ.real = this->dZ;
      *(outbuffer + offset + 0) = (u_dZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dZ);
      union {
        float real;
        uint32_t base;
      } u_rotX;
      u_rotX.real = this->rotX;
      *(outbuffer + offset + 0) = (u_rotX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotX);
      union {
        float real;
        uint32_t base;
      } u_rotY;
      u_rotY.real = this->rotY;
      *(outbuffer + offset + 0) = (u_rotY.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotY.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotY.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotY.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotY);
      union {
        float real;
        uint32_t base;
      } u_rotZ;
      u_rotZ.real = this->rotZ;
      *(outbuffer + offset + 0) = (u_rotZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotZ);
      union {
        float real;
        uint32_t base;
      } u_scale;
      u_scale.real = this->scale;
      *(outbuffer + offset + 0) = (u_scale.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_scale.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_scale.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_scale.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->scale);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->datumNum =  ((uint16_t) (*(inbuffer + offset)));
      this->datumNum |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->datumNum);
      for( uint32_t i = 0; i < 6; i++){
      this->datumName[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->datumName[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_majA;
      u_majA.base = 0;
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_majA.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->majA = u_majA.real;
      offset += sizeof(this->majA);
      union {
        double real;
        uint64_t base;
      } u_flat;
      u_flat.base = 0;
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_flat.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->flat = u_flat.real;
      offset += sizeof(this->flat);
      union {
        float real;
        uint32_t base;
      } u_dX;
      u_dX.base = 0;
      u_dX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dX = u_dX.real;
      offset += sizeof(this->dX);
      union {
        float real;
        uint32_t base;
      } u_dY;
      u_dY.base = 0;
      u_dY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dY = u_dY.real;
      offset += sizeof(this->dY);
      union {
        float real;
        uint32_t base;
      } u_dZ;
      u_dZ.base = 0;
      u_dZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dZ = u_dZ.real;
      offset += sizeof(this->dZ);
      union {
        float real;
        uint32_t base;
      } u_rotX;
      u_rotX.base = 0;
      u_rotX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotX = u_rotX.real;
      offset += sizeof(this->rotX);
      union {
        float real;
        uint32_t base;
      } u_rotY;
      u_rotY.base = 0;
      u_rotY.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotY.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotY.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotY.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotY = u_rotY.real;
      offset += sizeof(this->rotY);
      union {
        float real;
        uint32_t base;
      } u_rotZ;
      u_rotZ.base = 0;
      u_rotZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotZ = u_rotZ.real;
      offset += sizeof(this->rotZ);
      union {
        float real;
        uint32_t base;
      } u_scale;
      u_scale.base = 0;
      u_scale.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_scale.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_scale.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_scale.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->scale = u_scale.real;
      offset += sizeof(this->scale);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgDAT"; };
    virtual const char * getMD5() override { return "05d7a26d8a386fd3054953454a03b113"; };

  };

}
#endif
