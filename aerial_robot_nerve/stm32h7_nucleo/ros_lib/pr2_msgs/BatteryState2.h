#ifndef _ROS_pr2_msgs_BatteryState2_h
#define _ROS_pr2_msgs_BatteryState2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace pr2_msgs
{

  class BatteryState2 : public ros::Msg
  {
    public:
      typedef bool _present_type;
      _present_type present;
      typedef bool _charging_type;
      _charging_type charging;
      typedef bool _discharging_type;
      _discharging_type discharging;
      typedef bool _power_present_type;
      _power_present_type power_present;
      typedef bool _power_no_good_type;
      _power_no_good_type power_no_good;
      typedef bool _inhibited_type;
      _inhibited_type inhibited;
      typedef ros::Time _last_battery_update_type;
      _last_battery_update_type last_battery_update;
      int16_t battery_register[48];
      bool battery_update_flag[48];
      ros::Time battery_register_update[48];

    BatteryState2():
      present(0),
      charging(0),
      discharging(0),
      power_present(0),
      power_no_good(0),
      inhibited(0),
      last_battery_update(),
      battery_register(),
      battery_update_flag(),
      battery_register_update()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.real = this->present;
      *(outbuffer + offset + 0) = (u_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->present);
      union {
        bool real;
        uint8_t base;
      } u_charging;
      u_charging.real = this->charging;
      *(outbuffer + offset + 0) = (u_charging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->charging);
      union {
        bool real;
        uint8_t base;
      } u_discharging;
      u_discharging.real = this->discharging;
      *(outbuffer + offset + 0) = (u_discharging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->discharging);
      union {
        bool real;
        uint8_t base;
      } u_power_present;
      u_power_present.real = this->power_present;
      *(outbuffer + offset + 0) = (u_power_present.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_present);
      union {
        bool real;
        uint8_t base;
      } u_power_no_good;
      u_power_no_good.real = this->power_no_good;
      *(outbuffer + offset + 0) = (u_power_no_good.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->power_no_good);
      union {
        bool real;
        uint8_t base;
      } u_inhibited;
      u_inhibited.real = this->inhibited;
      *(outbuffer + offset + 0) = (u_inhibited.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->inhibited);
      *(outbuffer + offset + 0) = (this->last_battery_update.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_battery_update.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_battery_update.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_battery_update.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_battery_update.sec);
      *(outbuffer + offset + 0) = (this->last_battery_update.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_battery_update.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_battery_update.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_battery_update.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_battery_update.nsec);
      for( uint32_t i = 0; i < 48; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_battery_registeri;
      u_battery_registeri.real = this->battery_register[i];
      *(outbuffer + offset + 0) = (u_battery_registeri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_registeri.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->battery_register[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      union {
        bool real;
        uint8_t base;
      } u_battery_update_flagi;
      u_battery_update_flagi.real = this->battery_update_flag[i];
      *(outbuffer + offset + 0) = (u_battery_update_flagi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery_update_flag[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      *(outbuffer + offset + 0) = (this->battery_register_update[i].sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->battery_register_update[i].sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->battery_register_update[i].sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->battery_register_update[i].sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_register_update[i].sec);
      *(outbuffer + offset + 0) = (this->battery_register_update[i].nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->battery_register_update[i].nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->battery_register_update[i].nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->battery_register_update[i].nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_register_update[i].nsec);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_present;
      u_present.base = 0;
      u_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->present = u_present.real;
      offset += sizeof(this->present);
      union {
        bool real;
        uint8_t base;
      } u_charging;
      u_charging.base = 0;
      u_charging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->charging = u_charging.real;
      offset += sizeof(this->charging);
      union {
        bool real;
        uint8_t base;
      } u_discharging;
      u_discharging.base = 0;
      u_discharging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->discharging = u_discharging.real;
      offset += sizeof(this->discharging);
      union {
        bool real;
        uint8_t base;
      } u_power_present;
      u_power_present.base = 0;
      u_power_present.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->power_present = u_power_present.real;
      offset += sizeof(this->power_present);
      union {
        bool real;
        uint8_t base;
      } u_power_no_good;
      u_power_no_good.base = 0;
      u_power_no_good.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->power_no_good = u_power_no_good.real;
      offset += sizeof(this->power_no_good);
      union {
        bool real;
        uint8_t base;
      } u_inhibited;
      u_inhibited.base = 0;
      u_inhibited.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->inhibited = u_inhibited.real;
      offset += sizeof(this->inhibited);
      this->last_battery_update.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_battery_update.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_battery_update.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_battery_update.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_battery_update.sec);
      this->last_battery_update.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_battery_update.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_battery_update.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_battery_update.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_battery_update.nsec);
      for( uint32_t i = 0; i < 48; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_battery_registeri;
      u_battery_registeri.base = 0;
      u_battery_registeri.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_registeri.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_register[i] = u_battery_registeri.real;
      offset += sizeof(this->battery_register[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      union {
        bool real;
        uint8_t base;
      } u_battery_update_flagi;
      u_battery_update_flagi.base = 0;
      u_battery_update_flagi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->battery_update_flag[i] = u_battery_update_flagi.real;
      offset += sizeof(this->battery_update_flag[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      this->battery_register_update[i].sec =  ((uint32_t) (*(inbuffer + offset)));
      this->battery_register_update[i].sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_register_update[i].sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->battery_register_update[i].sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->battery_register_update[i].sec);
      this->battery_register_update[i].nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->battery_register_update[i].nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->battery_register_update[i].nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->battery_register_update[i].nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->battery_register_update[i].nsec);
      }
     return offset;
    }

    const char * getType(){ return "pr2_msgs/BatteryState2"; };
    const char * getMD5(){ return "91b4acb000aa990ac3006834f9a99669"; };

  };

}
#endif
