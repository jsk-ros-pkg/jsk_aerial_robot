#ifndef _ROS_SERVICE_Status_h
#define _ROS_SERVICE_Status_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sesame_ros
{

static const char STATUS[] = "sesame_ros/Status";

  class StatusRequest : public ros::Msg
  {
    public:

    StatusRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return STATUS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StatusResponse : public ros::Msg
  {
    public:
      typedef const char* _nickname_type;
      _nickname_type nickname;
      typedef const char* _serial_type;
      _serial_type serial;
      typedef const char* _device_id_type;
      _device_id_type device_id;
      typedef int32_t _battery_type;
      _battery_type battery;
      typedef bool _locked_type;
      _locked_type locked;
      typedef bool _responsive_type;
      _responsive_type responsive;

    StatusResponse():
      nickname(""),
      serial(""),
      device_id(""),
      battery(0),
      locked(0),
      responsive(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_nickname = strlen(this->nickname);
      varToArr(outbuffer + offset, length_nickname);
      offset += 4;
      memcpy(outbuffer + offset, this->nickname, length_nickname);
      offset += length_nickname;
      uint32_t length_serial = strlen(this->serial);
      varToArr(outbuffer + offset, length_serial);
      offset += 4;
      memcpy(outbuffer + offset, this->serial, length_serial);
      offset += length_serial;
      uint32_t length_device_id = strlen(this->device_id);
      varToArr(outbuffer + offset, length_device_id);
      offset += 4;
      memcpy(outbuffer + offset, this->device_id, length_device_id);
      offset += length_device_id;
      union {
        int32_t real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      union {
        bool real;
        uint8_t base;
      } u_locked;
      u_locked.real = this->locked;
      *(outbuffer + offset + 0) = (u_locked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->locked);
      union {
        bool real;
        uint8_t base;
      } u_responsive;
      u_responsive.real = this->responsive;
      *(outbuffer + offset + 0) = (u_responsive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->responsive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_nickname;
      arrToVar(length_nickname, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_nickname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_nickname-1]=0;
      this->nickname = (char *)(inbuffer + offset-1);
      offset += length_nickname;
      uint32_t length_serial;
      arrToVar(length_serial, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_serial; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_serial-1]=0;
      this->serial = (char *)(inbuffer + offset-1);
      offset += length_serial;
      uint32_t length_device_id;
      arrToVar(length_device_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_device_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_device_id-1]=0;
      this->device_id = (char *)(inbuffer + offset-1);
      offset += length_device_id;
      union {
        int32_t real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
      union {
        bool real;
        uint8_t base;
      } u_locked;
      u_locked.base = 0;
      u_locked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->locked = u_locked.real;
      offset += sizeof(this->locked);
      union {
        bool real;
        uint8_t base;
      } u_responsive;
      u_responsive.base = 0;
      u_responsive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->responsive = u_responsive.real;
      offset += sizeof(this->responsive);
     return offset;
    }

    const char * getType(){ return STATUS; };
    const char * getMD5(){ return "25385424a97316579ec1ed945842d240"; };

  };

  class Status {
    public:
    typedef StatusRequest Request;
    typedef StatusResponse Response;
  };

}
#endif
