#ifndef _ROS_pr2_msgs_BatteryServer2_h
#define _ROS_pr2_msgs_BatteryServer2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"
#include "ros/duration.h"
#include "pr2_msgs/BatteryState2.h"

namespace pr2_msgs
{

  class BatteryServer2 : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _id_type;
      _id_type id;
      typedef ros::Time _last_system_update_type;
      _last_system_update_type last_system_update;
      typedef ros::Duration _time_left_type;
      _time_left_type time_left;
      typedef int32_t _average_charge_type;
      _average_charge_type average_charge;
      typedef const char* _message_type;
      _message_type message;
      typedef ros::Time _last_controller_update_type;
      _last_controller_update_type last_controller_update;
      uint32_t battery_length;
      typedef pr2_msgs::BatteryState2 _battery_type;
      _battery_type st_battery;
      _battery_type * battery;
      enum { MAX_BAT_COUNT = 4 };
      enum { MAX_BAT_REG = 48 };

    BatteryServer2():
      header(),
      id(0),
      last_system_update(),
      time_left(),
      average_charge(0),
      message(""),
      last_controller_update(),
      battery_length(0), battery(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->last_system_update.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_system_update.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_system_update.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_system_update.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_system_update.sec);
      *(outbuffer + offset + 0) = (this->last_system_update.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_system_update.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_system_update.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_system_update.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_system_update.nsec);
      *(outbuffer + offset + 0) = (this->time_left.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_left.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_left.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_left.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_left.sec);
      *(outbuffer + offset + 0) = (this->time_left.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_left.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_left.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_left.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_left.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_average_charge;
      u_average_charge.real = this->average_charge;
      *(outbuffer + offset + 0) = (u_average_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_average_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_average_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_average_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->average_charge);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      *(outbuffer + offset + 0) = (this->last_controller_update.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_controller_update.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_controller_update.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_controller_update.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_controller_update.sec);
      *(outbuffer + offset + 0) = (this->last_controller_update.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->last_controller_update.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->last_controller_update.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->last_controller_update.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->last_controller_update.nsec);
      *(outbuffer + offset + 0) = (this->battery_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->battery_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->battery_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->battery_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_length);
      for( uint32_t i = 0; i < battery_length; i++){
      offset += this->battery[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      this->last_system_update.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_system_update.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_system_update.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_system_update.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_system_update.sec);
      this->last_system_update.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_system_update.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_system_update.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_system_update.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_system_update.nsec);
      this->time_left.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_left.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_left.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_left.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_left.sec);
      this->time_left.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_left.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_left.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_left.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_left.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_average_charge;
      u_average_charge.base = 0;
      u_average_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_average_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_average_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_average_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->average_charge = u_average_charge.real;
      offset += sizeof(this->average_charge);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      this->last_controller_update.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_controller_update.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_controller_update.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_controller_update.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_controller_update.sec);
      this->last_controller_update.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->last_controller_update.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->last_controller_update.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->last_controller_update.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->last_controller_update.nsec);
      uint32_t battery_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      battery_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      battery_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      battery_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->battery_length);
      if(battery_lengthT > battery_length)
        this->battery = (pr2_msgs::BatteryState2*)realloc(this->battery, battery_lengthT * sizeof(pr2_msgs::BatteryState2));
      battery_length = battery_lengthT;
      for( uint32_t i = 0; i < battery_length; i++){
      offset += this->st_battery.deserialize(inbuffer + offset);
        memcpy( &(this->battery[i]), &(this->st_battery), sizeof(pr2_msgs::BatteryState2));
      }
     return offset;
    }

    const char * getType(){ return "pr2_msgs/BatteryServer2"; };
    const char * getMD5(){ return "5f2cec7d06c312d756189db96c1f3819"; };

  };

}
#endif
