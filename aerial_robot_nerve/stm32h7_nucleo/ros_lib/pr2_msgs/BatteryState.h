#ifndef _ROS_pr2_msgs_BatteryState_h
#define _ROS_pr2_msgs_BatteryState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_msgs
{

  class BatteryState : public ros::Msg
  {
    public:
      typedef int32_t _lastTimeBattery_type;
      _lastTimeBattery_type lastTimeBattery;
      uint16_t batReg[48];
      uint16_t batRegFlag[48];
      int32_t batRegTime[48];

    BatteryState():
      lastTimeBattery(0),
      batReg(),
      batRegFlag(),
      batRegTime()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lastTimeBattery;
      u_lastTimeBattery.real = this->lastTimeBattery;
      *(outbuffer + offset + 0) = (u_lastTimeBattery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lastTimeBattery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lastTimeBattery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lastTimeBattery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lastTimeBattery);
      for( uint32_t i = 0; i < 48; i++){
      *(outbuffer + offset + 0) = (this->batReg[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->batReg[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->batReg[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      *(outbuffer + offset + 0) = (this->batRegFlag[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->batRegFlag[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->batRegFlag[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_batRegTimei;
      u_batRegTimei.real = this->batRegTime[i];
      *(outbuffer + offset + 0) = (u_batRegTimei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_batRegTimei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_batRegTimei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_batRegTimei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->batRegTime[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lastTimeBattery;
      u_lastTimeBattery.base = 0;
      u_lastTimeBattery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lastTimeBattery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lastTimeBattery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lastTimeBattery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lastTimeBattery = u_lastTimeBattery.real;
      offset += sizeof(this->lastTimeBattery);
      for( uint32_t i = 0; i < 48; i++){
      this->batReg[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->batReg[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->batReg[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      this->batRegFlag[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->batRegFlag[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->batRegFlag[i]);
      }
      for( uint32_t i = 0; i < 48; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_batRegTimei;
      u_batRegTimei.base = 0;
      u_batRegTimei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_batRegTimei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_batRegTimei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_batRegTimei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->batRegTime[i] = u_batRegTimei.real;
      offset += sizeof(this->batRegTime[i]);
      }
     return offset;
    }

    const char * getType(){ return "pr2_msgs/BatteryState"; };
    const char * getMD5(){ return "00e9f996c2fc26700fd25abcd8422db0"; };

  };

}
#endif
