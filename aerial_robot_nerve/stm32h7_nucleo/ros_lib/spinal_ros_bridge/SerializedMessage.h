#ifndef _ROS_spinal_ros_bridge_SerializedMessage_h
#define _ROS_spinal_ros_bridge_SerializedMessage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal_ros_bridge
{

  class SerializedMessage : public ros::Msg
  {
    public:
      typedef uint16_t _id_type;
      _id_type id;
      uint32_t body_length;
      typedef uint8_t _body_type;
      _body_type st_body;
      _body_type * body;

    SerializedMessage():
      id(0),
      body_length(0), body(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->body_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->body_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->body_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->body_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->body_length);
      for( uint32_t i = 0; i < body_length; i++){
      *(outbuffer + offset + 0) = (this->body[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->body[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint16_t) (*(inbuffer + offset)));
      this->id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->id);
      uint32_t body_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      body_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      body_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      body_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->body_length);
      if(body_lengthT > body_length)
        this->body = (uint8_t*)realloc(this->body, body_lengthT * sizeof(uint8_t));
      body_length = body_lengthT;
      for( uint32_t i = 0; i < body_length; i++){
      this->st_body =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_body);
        memcpy( &(this->body[i]), &(this->st_body), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "spinal_ros_bridge/SerializedMessage"; };
    const char * getMD5(){ return "0fd3bf3b278ace3ceedfe7c52ff2efdd"; };

  };

}
#endif
