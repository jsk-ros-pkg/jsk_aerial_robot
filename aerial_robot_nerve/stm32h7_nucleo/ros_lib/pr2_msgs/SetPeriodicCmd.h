#ifndef _ROS_SERVICE_SetPeriodicCmd_h
#define _ROS_SERVICE_SetPeriodicCmd_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "pr2_msgs/PeriodicCmd.h"
#include "ros/time.h"

namespace pr2_msgs
{

static const char SETPERIODICCMD[] = "pr2_msgs/SetPeriodicCmd";

  class SetPeriodicCmdRequest : public ros::Msg
  {
    public:
      typedef pr2_msgs::PeriodicCmd _command_type;
      _command_type command;

    SetPeriodicCmdRequest():
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETPERIODICCMD; };
    const char * getMD5(){ return "d4deedcc194c4a77110f7228904ee733"; };

  };

  class SetPeriodicCmdResponse : public ros::Msg
  {
    public:
      typedef ros::Time _start_time_type;
      _start_time_type start_time;

    SetPeriodicCmdResponse():
      start_time()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->start_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.sec);
      *(outbuffer + offset + 0) = (this->start_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->start_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.sec);
      this->start_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.nsec);
     return offset;
    }

    const char * getType(){ return SETPERIODICCMD; };
    const char * getMD5(){ return "3888666920054f1ef39d2df7a5d94b02"; };

  };

  class SetPeriodicCmd {
    public:
    typedef SetPeriodicCmdRequest Request;
    typedef SetPeriodicCmdResponse Response;
  };

}
#endif
