#ifndef _ROS_app_manager_AppInstallationState_h
#define _ROS_app_manager_AppInstallationState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/ExchangeApp.h"

namespace app_manager
{

  class AppInstallationState : public ros::Msg
  {
    public:
      uint32_t installed_apps_length;
      typedef app_manager::ExchangeApp _installed_apps_type;
      _installed_apps_type st_installed_apps;
      _installed_apps_type * installed_apps;
      uint32_t available_apps_length;
      typedef app_manager::ExchangeApp _available_apps_type;
      _available_apps_type st_available_apps;
      _available_apps_type * available_apps;

    AppInstallationState():
      installed_apps_length(0), installed_apps(NULL),
      available_apps_length(0), available_apps(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->installed_apps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->installed_apps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->installed_apps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->installed_apps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->installed_apps_length);
      for( uint32_t i = 0; i < installed_apps_length; i++){
      offset += this->installed_apps[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->available_apps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->available_apps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->available_apps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->available_apps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->available_apps_length);
      for( uint32_t i = 0; i < available_apps_length; i++){
      offset += this->available_apps[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t installed_apps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      installed_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      installed_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      installed_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->installed_apps_length);
      if(installed_apps_lengthT > installed_apps_length)
        this->installed_apps = (app_manager::ExchangeApp*)realloc(this->installed_apps, installed_apps_lengthT * sizeof(app_manager::ExchangeApp));
      installed_apps_length = installed_apps_lengthT;
      for( uint32_t i = 0; i < installed_apps_length; i++){
      offset += this->st_installed_apps.deserialize(inbuffer + offset);
        memcpy( &(this->installed_apps[i]), &(this->st_installed_apps), sizeof(app_manager::ExchangeApp));
      }
      uint32_t available_apps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      available_apps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->available_apps_length);
      if(available_apps_lengthT > available_apps_length)
        this->available_apps = (app_manager::ExchangeApp*)realloc(this->available_apps, available_apps_lengthT * sizeof(app_manager::ExchangeApp));
      available_apps_length = available_apps_lengthT;
      for( uint32_t i = 0; i < available_apps_length; i++){
      offset += this->st_available_apps.deserialize(inbuffer + offset);
        memcpy( &(this->available_apps[i]), &(this->st_available_apps), sizeof(app_manager::ExchangeApp));
      }
     return offset;
    }

    const char * getType(){ return "app_manager/AppInstallationState"; };
    const char * getMD5(){ return "46d45bbda08250199267aff8c0ee8c41"; };

  };

}
#endif
