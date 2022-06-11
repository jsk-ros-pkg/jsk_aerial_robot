#ifndef _ROS_takasako_sps_PowerInfo_h
#define _ROS_takasako_sps_PowerInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace takasako_sps
{

  class PowerInfo : public ros::Msg
  {
    public:
      typedef float _currency_type;
      _currency_type currency;
      typedef float _voltage_type;
      _voltage_type voltage;
      typedef float _power_type;
      _power_type power;

    PowerInfo():
      currency(0),
      voltage(0),
      power(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_currency;
      u_currency.real = this->currency;
      *(outbuffer + offset + 0) = (u_currency.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currency.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currency.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currency.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currency);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.real = this->voltage;
      *(outbuffer + offset + 0) = (u_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltage.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_power;
      u_power.real = this->power;
      *(outbuffer + offset + 0) = (u_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->power);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_currency;
      u_currency.base = 0;
      u_currency.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currency.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currency.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currency.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currency = u_currency.real;
      offset += sizeof(this->currency);
      union {
        float real;
        uint32_t base;
      } u_voltage;
      u_voltage.base = 0;
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltage.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltage = u_voltage.real;
      offset += sizeof(this->voltage);
      union {
        float real;
        uint32_t base;
      } u_power;
      u_power.base = 0;
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->power = u_power.real;
      offset += sizeof(this->power);
     return offset;
    }

    const char * getType(){ return "takasako_sps/PowerInfo"; };
    const char * getMD5(){ return "31251a8069264d5c3f5ddea120dff9a7"; };

  };

}
#endif
