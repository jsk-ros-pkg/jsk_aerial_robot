#ifndef _ROS_SERVICE_AddExtraModule_h
#define _ROS_SERVICE_AddExtraModule_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Inertia.h"
#include "geometry_msgs/Transform.h"

namespace aerial_robot_model
{

static const char ADDEXTRAMODULE[] = "aerial_robot_model/AddExtraModule";

  class AddExtraModuleRequest : public ros::Msg
  {
    public:
      typedef int8_t _action_type;
      _action_type action;
      typedef const char* _module_name_type;
      _module_name_type module_name;
      typedef const char* _parent_link_name_type;
      _parent_link_name_type parent_link_name;
      typedef geometry_msgs::Transform _transform_type;
      _transform_type transform;
      typedef geometry_msgs::Inertia _inertia_type;
      _inertia_type inertia;
      enum { ADD =  1 };
      enum { REMOVE =  -1 };

    AddExtraModuleRequest():
      action(0),
      module_name(""),
      parent_link_name(""),
      transform(),
      inertia()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_action;
      u_action.real = this->action;
      *(outbuffer + offset + 0) = (u_action.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->action);
      uint32_t length_module_name = strlen(this->module_name);
      varToArr(outbuffer + offset, length_module_name);
      offset += 4;
      memcpy(outbuffer + offset, this->module_name, length_module_name);
      offset += length_module_name;
      uint32_t length_parent_link_name = strlen(this->parent_link_name);
      varToArr(outbuffer + offset, length_parent_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->parent_link_name, length_parent_link_name);
      offset += length_parent_link_name;
      offset += this->transform.serialize(outbuffer + offset);
      offset += this->inertia.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_action;
      u_action.base = 0;
      u_action.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->action = u_action.real;
      offset += sizeof(this->action);
      uint32_t length_module_name;
      arrToVar(length_module_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_name-1]=0;
      this->module_name = (char *)(inbuffer + offset-1);
      offset += length_module_name;
      uint32_t length_parent_link_name;
      arrToVar(length_parent_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parent_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parent_link_name-1]=0;
      this->parent_link_name = (char *)(inbuffer + offset-1);
      offset += length_parent_link_name;
      offset += this->transform.deserialize(inbuffer + offset);
      offset += this->inertia.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ADDEXTRAMODULE; };
    const char * getMD5(){ return "33b8aa7e2f783985962bf82f0cd13f9f"; };

  };

  class AddExtraModuleResponse : public ros::Msg
  {
    public:
      typedef bool _status_type;
      _status_type status;

    AddExtraModuleResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return ADDEXTRAMODULE; };
    const char * getMD5(){ return "3a1255d4d998bd4d6585c64639b5ee9a"; };

  };

  class AddExtraModule {
    public:
    typedef AddExtraModuleRequest Request;
    typedef AddExtraModuleResponse Response;
  };

}
#endif
