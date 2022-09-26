#ifndef _ROS_ublox_msgs_UpdSOS_Ack_h
#define _ROS_ublox_msgs_UpdSOS_Ack_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class UpdSOS_Ack : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      uint8_t reserved0[3];
      typedef uint8_t _response_type;
      _response_type response;
      uint8_t reserved1[3];
      enum { CLASS_ID =  9 };
      enum { MESSAGE_ID =  20 };
      enum { CMD_BACKUP_CREATE_ACK =  2    };
      enum { CMD_SYSTEM_RESTORED =  3      };
      enum { BACKUP_CREATE_NACK =  0                       };
      enum { BACKUP_CREATE_ACK =  1                        };
      enum { SYSTEM_RESTORED_RESPONSE_UNKNOWN =  0         };
      enum { SYSTEM_RESTORED_RESPONSE_FAILED =  1          };
      enum { SYSTEM_RESTORED_RESPONSE_RESTORED =  2        };
      enum { SYSTEM_RESTORED_RESPONSE_NOT_RESTORED =  3    };

    UpdSOS_Ack():
      cmd(0),
      reserved0(),
      response(0),
      reserved1()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      *(outbuffer + offset + 0) = (this->response >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response);
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
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
      this->response =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->response);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/UpdSOS_Ack"; };
    virtual const char * getMD5() override { return "028d9bc0701daf71dcd1ad8cef68594c"; };

  };

}
#endif
