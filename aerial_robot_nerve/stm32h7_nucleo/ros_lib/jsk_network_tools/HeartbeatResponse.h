#ifndef _ROS_jsk_network_tools_HeartbeatResponse_h
#define _ROS_jsk_network_tools_HeartbeatResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "jsk_network_tools/Heartbeat.h"

namespace jsk_network_tools
{

  class HeartbeatResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef jsk_network_tools::Heartbeat _heartbeat_type;
      _heartbeat_type heartbeat;

    HeartbeatResponse():
      header(),
      heartbeat()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->heartbeat.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->heartbeat.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/HeartbeatResponse"; };
    const char * getMD5(){ return "ffc0783982ce8ad53fd088a07b64ca4b"; };

  };

}
#endif
