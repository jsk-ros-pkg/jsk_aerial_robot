#ifndef _ROS_SERVICE_Upload_h
#define _ROS_SERVICE_Upload_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gdrive_ros
{

static const char UPLOAD[] = "gdrive_ros/Upload";

  class UploadRequest : public ros::Msg
  {
    public:
      typedef const char* _file_path_type;
      _file_path_type file_path;
      typedef const char* _file_title_type;
      _file_title_type file_title;
      typedef const char* _parents_path_type;
      _parents_path_type parents_path;
      typedef const char* _parents_id_type;
      _parents_id_type parents_id;
      typedef bool _use_timestamp_folder_type;
      _use_timestamp_folder_type use_timestamp_folder;
      typedef bool _use_timestamp_file_title_type;
      _use_timestamp_file_title_type use_timestamp_file_title;

    UploadRequest():
      file_path(""),
      file_title(""),
      parents_path(""),
      parents_id(""),
      use_timestamp_folder(0),
      use_timestamp_file_title(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_file_path = strlen(this->file_path);
      varToArr(outbuffer + offset, length_file_path);
      offset += 4;
      memcpy(outbuffer + offset, this->file_path, length_file_path);
      offset += length_file_path;
      uint32_t length_file_title = strlen(this->file_title);
      varToArr(outbuffer + offset, length_file_title);
      offset += 4;
      memcpy(outbuffer + offset, this->file_title, length_file_title);
      offset += length_file_title;
      uint32_t length_parents_path = strlen(this->parents_path);
      varToArr(outbuffer + offset, length_parents_path);
      offset += 4;
      memcpy(outbuffer + offset, this->parents_path, length_parents_path);
      offset += length_parents_path;
      uint32_t length_parents_id = strlen(this->parents_id);
      varToArr(outbuffer + offset, length_parents_id);
      offset += 4;
      memcpy(outbuffer + offset, this->parents_id, length_parents_id);
      offset += length_parents_id;
      union {
        bool real;
        uint8_t base;
      } u_use_timestamp_folder;
      u_use_timestamp_folder.real = this->use_timestamp_folder;
      *(outbuffer + offset + 0) = (u_use_timestamp_folder.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_timestamp_folder);
      union {
        bool real;
        uint8_t base;
      } u_use_timestamp_file_title;
      u_use_timestamp_file_title.real = this->use_timestamp_file_title;
      *(outbuffer + offset + 0) = (u_use_timestamp_file_title.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_timestamp_file_title);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_file_path;
      arrToVar(length_file_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_path-1]=0;
      this->file_path = (char *)(inbuffer + offset-1);
      offset += length_file_path;
      uint32_t length_file_title;
      arrToVar(length_file_title, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_title; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_title-1]=0;
      this->file_title = (char *)(inbuffer + offset-1);
      offset += length_file_title;
      uint32_t length_parents_path;
      arrToVar(length_parents_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parents_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parents_path-1]=0;
      this->parents_path = (char *)(inbuffer + offset-1);
      offset += length_parents_path;
      uint32_t length_parents_id;
      arrToVar(length_parents_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parents_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parents_id-1]=0;
      this->parents_id = (char *)(inbuffer + offset-1);
      offset += length_parents_id;
      union {
        bool real;
        uint8_t base;
      } u_use_timestamp_folder;
      u_use_timestamp_folder.base = 0;
      u_use_timestamp_folder.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_timestamp_folder = u_use_timestamp_folder.real;
      offset += sizeof(this->use_timestamp_folder);
      union {
        bool real;
        uint8_t base;
      } u_use_timestamp_file_title;
      u_use_timestamp_file_title.base = 0;
      u_use_timestamp_file_title.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_timestamp_file_title = u_use_timestamp_file_title.real;
      offset += sizeof(this->use_timestamp_file_title);
     return offset;
    }

    const char * getType(){ return UPLOAD; };
    const char * getMD5(){ return "a09a0dc73a941d3514dca63d7fc3cf3b"; };

  };

  class UploadResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _file_id_type;
      _file_id_type file_id;
      typedef const char* _file_url_type;
      _file_url_type file_url;
      typedef const char* _parents_id_type;
      _parents_id_type parents_id;
      typedef const char* _parents_url_type;
      _parents_url_type parents_url;

    UploadResponse():
      success(0),
      file_id(""),
      file_url(""),
      parents_id(""),
      parents_url("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_file_id = strlen(this->file_id);
      varToArr(outbuffer + offset, length_file_id);
      offset += 4;
      memcpy(outbuffer + offset, this->file_id, length_file_id);
      offset += length_file_id;
      uint32_t length_file_url = strlen(this->file_url);
      varToArr(outbuffer + offset, length_file_url);
      offset += 4;
      memcpy(outbuffer + offset, this->file_url, length_file_url);
      offset += length_file_url;
      uint32_t length_parents_id = strlen(this->parents_id);
      varToArr(outbuffer + offset, length_parents_id);
      offset += 4;
      memcpy(outbuffer + offset, this->parents_id, length_parents_id);
      offset += length_parents_id;
      uint32_t length_parents_url = strlen(this->parents_url);
      varToArr(outbuffer + offset, length_parents_url);
      offset += 4;
      memcpy(outbuffer + offset, this->parents_url, length_parents_url);
      offset += length_parents_url;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_file_id;
      arrToVar(length_file_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_id-1]=0;
      this->file_id = (char *)(inbuffer + offset-1);
      offset += length_file_id;
      uint32_t length_file_url;
      arrToVar(length_file_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_file_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_file_url-1]=0;
      this->file_url = (char *)(inbuffer + offset-1);
      offset += length_file_url;
      uint32_t length_parents_id;
      arrToVar(length_parents_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parents_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parents_id-1]=0;
      this->parents_id = (char *)(inbuffer + offset-1);
      offset += length_parents_id;
      uint32_t length_parents_url;
      arrToVar(length_parents_url, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parents_url; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parents_url-1]=0;
      this->parents_url = (char *)(inbuffer + offset-1);
      offset += length_parents_url;
     return offset;
    }

    const char * getType(){ return UPLOAD; };
    const char * getMD5(){ return "8f896e15fe386deb4d613a31c64e0cf6"; };

  };

  class Upload {
    public:
    typedef UploadRequest Request;
    typedef UploadResponse Response;
  };

}
#endif
