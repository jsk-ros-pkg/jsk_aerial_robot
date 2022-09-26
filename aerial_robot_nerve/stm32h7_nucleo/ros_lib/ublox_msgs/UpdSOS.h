#ifndef _ROS_ublox_msgs_UpdSOS_h
#define _ROS_ublox_msgs_UpdSOS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class UpdSOS : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      uint8_t reserved1[3];
      enum { CLASS_ID =  9 };
      enum { MESSAGE_ID =  20 };
      enum { CMD_FLASH_BACKUP_CREATE =  0     };
      enum { CMD_FLASH_BACKUP_CLEAR =  1      };

    UpdSOS():
      cmd(0),
      reserved1()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/UpdSOS"; };
    virtual const char * getMD5() override { return "fdc2e32dbb00126a932ae76a6222eec4"; };

  };

}
#endif
