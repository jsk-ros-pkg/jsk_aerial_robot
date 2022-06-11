#ifndef _ROS_app_manager_ExchangeApp_h
#define _ROS_app_manager_ExchangeApp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/Icon.h"

namespace app_manager
{

  class ExchangeApp : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _display_name_type;
      _display_name_type display_name;
      typedef const char* _version_type;
      _version_type version;
      typedef const char* _latest_version_type;
      _latest_version_type latest_version;
      typedef const char* _description_type;
      _description_type description;
      typedef app_manager::Icon _icon_type;
      _icon_type icon;
      typedef bool _hidden_type;
      _hidden_type hidden;

    ExchangeApp():
      name(""),
      display_name(""),
      version(""),
      latest_version(""),
      description(""),
      icon(),
      hidden(0)
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
      uint32_t length_display_name = strlen(this->display_name);
      varToArr(outbuffer + offset, length_display_name);
      offset += 4;
      memcpy(outbuffer + offset, this->display_name, length_display_name);
      offset += length_display_name;
      uint32_t length_version = strlen(this->version);
      varToArr(outbuffer + offset, length_version);
      offset += 4;
      memcpy(outbuffer + offset, this->version, length_version);
      offset += length_version;
      uint32_t length_latest_version = strlen(this->latest_version);
      varToArr(outbuffer + offset, length_latest_version);
      offset += 4;
      memcpy(outbuffer + offset, this->latest_version, length_latest_version);
      offset += length_latest_version;
      uint32_t length_description = strlen(this->description);
      varToArr(outbuffer + offset, length_description);
      offset += 4;
      memcpy(outbuffer + offset, this->description, length_description);
      offset += length_description;
      offset += this->icon.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hidden;
      u_hidden.real = this->hidden;
      *(outbuffer + offset + 0) = (u_hidden.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hidden);
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
      uint32_t length_display_name;
      arrToVar(length_display_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_display_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_display_name-1]=0;
      this->display_name = (char *)(inbuffer + offset-1);
      offset += length_display_name;
      uint32_t length_version;
      arrToVar(length_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_version-1]=0;
      this->version = (char *)(inbuffer + offset-1);
      offset += length_version;
      uint32_t length_latest_version;
      arrToVar(length_latest_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_latest_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_latest_version-1]=0;
      this->latest_version = (char *)(inbuffer + offset-1);
      offset += length_latest_version;
      uint32_t length_description;
      arrToVar(length_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_description-1]=0;
      this->description = (char *)(inbuffer + offset-1);
      offset += length_description;
      offset += this->icon.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hidden;
      u_hidden.base = 0;
      u_hidden.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hidden = u_hidden.real;
      offset += sizeof(this->hidden);
     return offset;
    }

    const char * getType(){ return "app_manager/ExchangeApp"; };
    const char * getMD5(){ return "ccad20aa9f390121e44c61d218038d78"; };

  };

}
#endif
