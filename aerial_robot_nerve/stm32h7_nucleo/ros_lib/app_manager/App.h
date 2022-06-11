#ifndef _ROS_app_manager_App_h
#define _ROS_app_manager_App_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/Icon.h"
#include "app_manager/ClientApp.h"

namespace app_manager
{

  class App : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _display_name_type;
      _display_name_type display_name;
      typedef app_manager::Icon _icon_type;
      _icon_type icon;
      uint32_t client_apps_length;
      typedef app_manager::ClientApp _client_apps_type;
      _client_apps_type st_client_apps;
      _client_apps_type * client_apps;

    App():
      name(""),
      display_name(""),
      icon(),
      client_apps_length(0), client_apps(NULL)
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
      offset += this->icon.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->client_apps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->client_apps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->client_apps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->client_apps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->client_apps_length);
      for( uint32_t i = 0; i < client_apps_length; i++){
      offset += this->client_apps[i].serialize(outbuffer + offset);
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
      uint32_t length_display_name;
      arrToVar(length_display_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_display_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_display_name-1]=0;
      this->display_name = (char *)(inbuffer + offset-1);
      offset += length_display_name;
      offset += this->icon.deserialize(inbuffer + offset);
      uint32_t client_apps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      client_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      client_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      client_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->client_apps_length);
      if(client_apps_lengthT > client_apps_length)
        this->client_apps = (app_manager::ClientApp*)realloc(this->client_apps, client_apps_lengthT * sizeof(app_manager::ClientApp));
      client_apps_length = client_apps_lengthT;
      for( uint32_t i = 0; i < client_apps_length; i++){
      offset += this->st_client_apps.deserialize(inbuffer + offset);
        memcpy( &(this->client_apps[i]), &(this->st_client_apps), sizeof(app_manager::ClientApp));
      }
     return offset;
    }

    const char * getType(){ return "app_manager/App"; };
    const char * getMD5(){ return "643c1db5f71b615a47789ff5e190811e"; };

  };

}
#endif
