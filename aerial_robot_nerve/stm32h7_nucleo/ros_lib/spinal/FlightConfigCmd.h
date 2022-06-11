#ifndef _ROS_spinal_FlightConfigCmd_h
#define _ROS_spinal_FlightConfigCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

  class FlightConfigCmd : public ros::Msg
  {
    public:
      typedef uint8_t _cmd_type;
      _cmd_type cmd;
      enum { ARM_ON_CMD =  0  };
      enum { ARM_OFF_CMD =  1  };
      enum { FORCE_LANDING_CMD =  2  };
      enum { INTEGRATION_CONTROL_ON_CMD =  160 };
      enum { INTEGRATION_CONTROL_OFF_CMD =  161 };

    FlightConfigCmd():
      cmd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cmd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cmd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->cmd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cmd);
     return offset;
    }

    const char * getType(){ return "spinal/FlightConfigCmd"; };
    const char * getMD5(){ return "34283d92c354f6d8be30da642018650f"; };

  };

}
#endif
