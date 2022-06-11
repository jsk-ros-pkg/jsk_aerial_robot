#ifndef _ROS_app_manager_ClientApp_h
#define _ROS_app_manager_ClientApp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "app_manager/KeyValue.h"

namespace app_manager
{

  class ClientApp : public ros::Msg
  {
    public:
      typedef const char* _client_type_type;
      _client_type_type client_type;
      uint32_t manager_data_length;
      typedef app_manager::KeyValue _manager_data_type;
      _manager_data_type st_manager_data;
      _manager_data_type * manager_data;
      uint32_t app_data_length;
      typedef app_manager::KeyValue _app_data_type;
      _app_data_type st_app_data;
      _app_data_type * app_data;

    ClientApp():
      client_type(""),
      manager_data_length(0), manager_data(NULL),
      app_data_length(0), app_data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_client_type = strlen(this->client_type);
      varToArr(outbuffer + offset, length_client_type);
      offset += 4;
      memcpy(outbuffer + offset, this->client_type, length_client_type);
      offset += length_client_type;
      *(outbuffer + offset + 0) = (this->manager_data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->manager_data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->manager_data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->manager_data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->manager_data_length);
      for( uint32_t i = 0; i < manager_data_length; i++){
      offset += this->manager_data[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->app_data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->app_data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->app_data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->app_data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->app_data_length);
      for( uint32_t i = 0; i < app_data_length; i++){
      offset += this->app_data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_client_type;
      arrToVar(length_client_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_client_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_client_type-1]=0;
      this->client_type = (char *)(inbuffer + offset-1);
      offset += length_client_type;
      uint32_t manager_data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      manager_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      manager_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      manager_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->manager_data_length);
      if(manager_data_lengthT > manager_data_length)
        this->manager_data = (app_manager::KeyValue*)realloc(this->manager_data, manager_data_lengthT * sizeof(app_manager::KeyValue));
      manager_data_length = manager_data_lengthT;
      for( uint32_t i = 0; i < manager_data_length; i++){
      offset += this->st_manager_data.deserialize(inbuffer + offset);
        memcpy( &(this->manager_data[i]), &(this->st_manager_data), sizeof(app_manager::KeyValue));
      }
      uint32_t app_data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      app_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      app_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      app_data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->app_data_length);
      if(app_data_lengthT > app_data_length)
        this->app_data = (app_manager::KeyValue*)realloc(this->app_data, app_data_lengthT * sizeof(app_manager::KeyValue));
      app_data_length = app_data_lengthT;
      for( uint32_t i = 0; i < app_data_length; i++){
      offset += this->st_app_data.deserialize(inbuffer + offset);
        memcpy( &(this->app_data[i]), &(this->st_app_data), sizeof(app_manager::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "app_manager/ClientApp"; };
    const char * getMD5(){ return "0a8112672c3fbf73cb62ec78d67e41bb"; };

  };

}
#endif
