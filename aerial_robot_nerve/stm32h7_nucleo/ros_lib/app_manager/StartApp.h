#ifndef _ROS_SERVICE_StartApp_h
#define _ROS_SERVICE_StartApp_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/KeyValue.h"

namespace app_manager
{

static const char STARTAPP[] = "app_manager/StartApp";

  class StartAppRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t args_length;
      typedef app_manager::KeyValue _args_type;
      _args_type st_args;
      _args_type * args;

    StartAppRequest():
      name(""),
      args_length(0), args(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->args_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->args_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->args_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->args_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->args_length);
      for( uint32_t i = 0; i < args_length; i++){
      offset += this->args[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t args_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->args_length);
      if(args_lengthT > args_length)
        this->args = (app_manager::KeyValue*)realloc(this->args, args_lengthT * sizeof(app_manager::KeyValue));
      args_length = args_lengthT;
      for( uint32_t i = 0; i < args_length; i++){
      offset += this->st_args.deserialize(inbuffer + offset);
        memcpy( &(this->args[i]), &(this->st_args), sizeof(app_manager::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return STARTAPP; };
    const char * getMD5(){ return "fcc3fd7d3a99df15b4752d0b8160ea6c"; };

  };

  class StartAppResponse : public ros::Msg
  {
    public:
      typedef bool _started_type;
      _started_type started;
      typedef int32_t _error_code_type;
      _error_code_type error_code;
      typedef const char* _message_type;
      _message_type message;
      typedef const char* _namespace_type;
      _namespace_type namespace;

    StartAppResponse():
      started(0),
      error_code(0),
      message(""),
      namespace("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_started;
      u_started.real = this->started;
      *(outbuffer + offset + 0) = (u_started.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->started);
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.real = this->error_code;
      *(outbuffer + offset + 0) = (u_error_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_code);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      uint32_t length_namespace = strlen(this->namespace);
      varToArr(outbuffer + offset, length_namespace);
      offset += 4;
      memcpy(outbuffer + offset, this->namespace, length_namespace);
      offset += length_namespace;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_started;
      u_started.base = 0;
      u_started.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->started = u_started.real;
      offset += sizeof(this->started);
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.base = 0;
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_code = u_error_code.real;
      offset += sizeof(this->error_code);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      uint32_t length_namespace;
      arrToVar(length_namespace, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_namespace; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_namespace-1]=0;
      this->namespace = (char *)(inbuffer + offset-1);
      offset += length_namespace;
     return offset;
    }

    const char * getType(){ return STARTAPP; };
    const char * getMD5(){ return "29589baf2876ff624d4cb5688c12265e"; };

  };

  class StartApp {
    public:
    typedef StartAppRequest Request;
    typedef StartAppResponse Response;
  };

}
#endif
