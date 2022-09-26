#ifndef _ROS_ublox_msgs_NavPOSLLH_h
#define _ROS_ublox_msgs_NavPOSLLH_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavPOSLLH : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int32_t _lon_type;
      _lon_type lon;
      typedef int32_t _lat_type;
      _lat_type lat;
      typedef int32_t _height_type;
      _height_type height;
      typedef int32_t _hMSL_type;
      _hMSL_type hMSL;
      typedef uint32_t _hAcc_type;
      _hAcc_type hAcc;
      typedef uint32_t _vAcc_type;
      _vAcc_type vAcc;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  2 };

    NavPOSLLH():
      iTOW(0),
      lon(0),
      lat(0),
      height(0),
      hMSL(0),
      hAcc(0),
      vAcc(0)
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
      } u_lon;
      u_lon.real = this->lon;
      *(outbuffer + offset + 0) = (u_lon.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lon.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lon.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lon.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lon);
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.real = this->lat;
      *(outbuffer + offset + 0) = (u_lat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        int32_t real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.real = this->hMSL;
      *(outbuffer + offset + 0) = (u_hMSL.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hMSL.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hMSL.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hMSL.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hMSL);
      *(outbuffer + offset + 0) = (this->hAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->hAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->hAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hAcc);
      *(outbuffer + offset + 0) = (this->vAcc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vAcc >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vAcc >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vAcc >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vAcc);
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
      } u_lon;
      u_lon.base = 0;
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lon.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lon = u_lon.real;
      offset += sizeof(this->lon);
      union {
        int32_t real;
        uint32_t base;
      } u_lat;
      u_lat.base = 0;
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lat = u_lat.real;
      offset += sizeof(this->lat);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        int32_t real;
        uint32_t base;
      } u_hMSL;
      u_hMSL.base = 0;
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hMSL.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->hMSL = u_hMSL.real;
      offset += sizeof(this->hMSL);
      this->hAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->hAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->hAcc);
      this->vAcc =  ((uint32_t) (*(inbuffer + offset)));
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->vAcc |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->vAcc);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavPOSLLH"; };
    virtual const char * getMD5() override { return "01aab2c75cd9f8c402a0689e82f04f01"; };

  };

}
#endif
