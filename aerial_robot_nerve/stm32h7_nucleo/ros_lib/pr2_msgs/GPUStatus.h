#ifndef _ROS_pr2_msgs_GPUStatus_h
#define _ROS_pr2_msgs_GPUStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class GPUStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _product_name_type;
      _product_name_type product_name;
      typedef const char* _pci_device_id_type;
      _pci_device_id_type pci_device_id;
      typedef const char* _pci_location_type;
      _pci_location_type pci_location;
      typedef const char* _display_type;
      _display_type display;
      typedef const char* _driver_version_type;
      _driver_version_type driver_version;
      typedef float _temperature_type;
      _temperature_type temperature;
      typedef float _fan_speed_type;
      _fan_speed_type fan_speed;
      typedef float _gpu_usage_type;
      _gpu_usage_type gpu_usage;
      typedef float _memory_usage_type;
      _memory_usage_type memory_usage;

    GPUStatus():
      header(),
      product_name(""),
      pci_device_id(""),
      pci_location(""),
      display(""),
      driver_version(""),
      temperature(0),
      fan_speed(0),
      gpu_usage(0),
      memory_usage(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_product_name = strlen(this->product_name);
      varToArr(outbuffer + offset, length_product_name);
      offset += 4;
      memcpy(outbuffer + offset, this->product_name, length_product_name);
      offset += length_product_name;
      uint32_t length_pci_device_id = strlen(this->pci_device_id);
      varToArr(outbuffer + offset, length_pci_device_id);
      offset += 4;
      memcpy(outbuffer + offset, this->pci_device_id, length_pci_device_id);
      offset += length_pci_device_id;
      uint32_t length_pci_location = strlen(this->pci_location);
      varToArr(outbuffer + offset, length_pci_location);
      offset += 4;
      memcpy(outbuffer + offset, this->pci_location, length_pci_location);
      offset += length_pci_location;
      uint32_t length_display = strlen(this->display);
      varToArr(outbuffer + offset, length_display);
      offset += 4;
      memcpy(outbuffer + offset, this->display, length_display);
      offset += length_display;
      uint32_t length_driver_version = strlen(this->driver_version);
      varToArr(outbuffer + offset, length_driver_version);
      offset += 4;
      memcpy(outbuffer + offset, this->driver_version, length_driver_version);
      offset += length_driver_version;
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.real = this->temperature;
      *(outbuffer + offset + 0) = (u_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_fan_speed;
      u_fan_speed.real = this->fan_speed;
      *(outbuffer + offset + 0) = (u_fan_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fan_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fan_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fan_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fan_speed);
      union {
        float real;
        uint32_t base;
      } u_gpu_usage;
      u_gpu_usage.real = this->gpu_usage;
      *(outbuffer + offset + 0) = (u_gpu_usage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gpu_usage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gpu_usage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gpu_usage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gpu_usage);
      union {
        float real;
        uint32_t base;
      } u_memory_usage;
      u_memory_usage.real = this->memory_usage;
      *(outbuffer + offset + 0) = (u_memory_usage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_memory_usage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_memory_usage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_memory_usage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->memory_usage);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_product_name;
      arrToVar(length_product_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_product_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_product_name-1]=0;
      this->product_name = (char *)(inbuffer + offset-1);
      offset += length_product_name;
      uint32_t length_pci_device_id;
      arrToVar(length_pci_device_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pci_device_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pci_device_id-1]=0;
      this->pci_device_id = (char *)(inbuffer + offset-1);
      offset += length_pci_device_id;
      uint32_t length_pci_location;
      arrToVar(length_pci_location, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pci_location; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pci_location-1]=0;
      this->pci_location = (char *)(inbuffer + offset-1);
      offset += length_pci_location;
      uint32_t length_display;
      arrToVar(length_display, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_display; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_display-1]=0;
      this->display = (char *)(inbuffer + offset-1);
      offset += length_display;
      uint32_t length_driver_version;
      arrToVar(length_driver_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_driver_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_driver_version-1]=0;
      this->driver_version = (char *)(inbuffer + offset-1);
      offset += length_driver_version;
      union {
        float real;
        uint32_t base;
      } u_temperature;
      u_temperature.base = 0;
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->temperature = u_temperature.real;
      offset += sizeof(this->temperature);
      union {
        float real;
        uint32_t base;
      } u_fan_speed;
      u_fan_speed.base = 0;
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fan_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fan_speed = u_fan_speed.real;
      offset += sizeof(this->fan_speed);
      union {
        float real;
        uint32_t base;
      } u_gpu_usage;
      u_gpu_usage.base = 0;
      u_gpu_usage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gpu_usage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gpu_usage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gpu_usage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gpu_usage = u_gpu_usage.real;
      offset += sizeof(this->gpu_usage);
      union {
        float real;
        uint32_t base;
      } u_memory_usage;
      u_memory_usage.base = 0;
      u_memory_usage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_memory_usage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_memory_usage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_memory_usage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->memory_usage = u_memory_usage.real;
      offset += sizeof(this->memory_usage);
     return offset;
    }

    const char * getType(){ return "pr2_msgs/GPUStatus"; };
    const char * getMD5(){ return "4c74e5474b8aade04e56108262099c6e"; };

  };

}
#endif
