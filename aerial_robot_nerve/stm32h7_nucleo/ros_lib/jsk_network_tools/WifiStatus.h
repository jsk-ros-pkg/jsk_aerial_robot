#ifndef _ROS_jsk_network_tools_WifiStatus_h
#define _ROS_jsk_network_tools_WifiStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace jsk_network_tools
{

  class WifiStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _interface_type;
      _interface_type interface;
      typedef bool _enabled_type;
      _enabled_type enabled;
      typedef bool _connected_type;
      _connected_type connected;
      typedef const char* _ssid_type;
      _ssid_type ssid;
      typedef float _frequency_type;
      _frequency_type frequency;
      typedef const char* _access_point_type;
      _access_point_type access_point;
      typedef float _bitrate_type;
      _bitrate_type bitrate;
      typedef int16_t _tx_power_type;
      _tx_power_type tx_power;
      typedef float _link_quality_type;
      _link_quality_type link_quality;
      typedef int16_t _signal_level_type;
      _signal_level_type signal_level;

    WifiStatus():
      header(),
      interface(""),
      enabled(0),
      connected(0),
      ssid(""),
      frequency(0),
      access_point(""),
      bitrate(0),
      tx_power(0),
      link_quality(0),
      signal_level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_interface = strlen(this->interface);
      varToArr(outbuffer + offset, length_interface);
      offset += 4;
      memcpy(outbuffer + offset, this->interface, length_interface);
      offset += length_interface;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.real = this->connected;
      *(outbuffer + offset + 0) = (u_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->connected);
      uint32_t length_ssid = strlen(this->ssid);
      varToArr(outbuffer + offset, length_ssid);
      offset += 4;
      memcpy(outbuffer + offset, this->ssid, length_ssid);
      offset += length_ssid;
      union {
        float real;
        uint32_t base;
      } u_frequency;
      u_frequency.real = this->frequency;
      *(outbuffer + offset + 0) = (u_frequency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frequency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frequency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frequency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frequency);
      uint32_t length_access_point = strlen(this->access_point);
      varToArr(outbuffer + offset, length_access_point);
      offset += 4;
      memcpy(outbuffer + offset, this->access_point, length_access_point);
      offset += length_access_point;
      union {
        float real;
        uint32_t base;
      } u_bitrate;
      u_bitrate.real = this->bitrate;
      *(outbuffer + offset + 0) = (u_bitrate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bitrate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bitrate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bitrate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bitrate);
      union {
        int16_t real;
        uint16_t base;
      } u_tx_power;
      u_tx_power.real = this->tx_power;
      *(outbuffer + offset + 0) = (u_tx_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tx_power.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tx_power);
      union {
        float real;
        uint32_t base;
      } u_link_quality;
      u_link_quality.real = this->link_quality;
      *(outbuffer + offset + 0) = (u_link_quality.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_link_quality.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_link_quality.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_link_quality.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->link_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_signal_level;
      u_signal_level.real = this->signal_level;
      *(outbuffer + offset + 0) = (u_signal_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_signal_level.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->signal_level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_interface;
      arrToVar(length_interface, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_interface; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_interface-1]=0;
      this->interface = (char *)(inbuffer + offset-1);
      offset += length_interface;
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
      union {
        bool real;
        uint8_t base;
      } u_connected;
      u_connected.base = 0;
      u_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->connected = u_connected.real;
      offset += sizeof(this->connected);
      uint32_t length_ssid;
      arrToVar(length_ssid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ssid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ssid-1]=0;
      this->ssid = (char *)(inbuffer + offset-1);
      offset += length_ssid;
      union {
        float real;
        uint32_t base;
      } u_frequency;
      u_frequency.base = 0;
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frequency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frequency = u_frequency.real;
      offset += sizeof(this->frequency);
      uint32_t length_access_point;
      arrToVar(length_access_point, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_access_point; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_access_point-1]=0;
      this->access_point = (char *)(inbuffer + offset-1);
      offset += length_access_point;
      union {
        float real;
        uint32_t base;
      } u_bitrate;
      u_bitrate.base = 0;
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bitrate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bitrate = u_bitrate.real;
      offset += sizeof(this->bitrate);
      union {
        int16_t real;
        uint16_t base;
      } u_tx_power;
      u_tx_power.base = 0;
      u_tx_power.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tx_power.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tx_power = u_tx_power.real;
      offset += sizeof(this->tx_power);
      union {
        float real;
        uint32_t base;
      } u_link_quality;
      u_link_quality.base = 0;
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_link_quality.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->link_quality = u_link_quality.real;
      offset += sizeof(this->link_quality);
      union {
        int16_t real;
        uint16_t base;
      } u_signal_level;
      u_signal_level.base = 0;
      u_signal_level.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_signal_level.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->signal_level = u_signal_level.real;
      offset += sizeof(this->signal_level);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/WifiStatus"; };
    const char * getMD5(){ return "94da4b64008d69473c62c62019a8c0f6"; };

  };

}
#endif
