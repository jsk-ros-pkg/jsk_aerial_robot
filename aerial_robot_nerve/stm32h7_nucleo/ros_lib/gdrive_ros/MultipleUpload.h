#ifndef _ROS_SERVICE_MultipleUpload_h
#define _ROS_SERVICE_MultipleUpload_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gdrive_ros
{

static const char MULTIPLEUPLOAD[] = "gdrive_ros/MultipleUpload";

  class MultipleUploadRequest : public ros::Msg
  {
    public:
      uint32_t file_paths_length;
      typedef char* _file_paths_type;
      _file_paths_type st_file_paths;
      _file_paths_type * file_paths;
      uint32_t file_titles_length;
      typedef char* _file_titles_type;
      _file_titles_type st_file_titles;
      _file_titles_type * file_titles;
      typedef const char* _parents_path_type;
      _parents_path_type parents_path;
      typedef const char* _parents_id_type;
      _parents_id_type parents_id;
      typedef bool _use_timestamp_folder_type;
      _use_timestamp_folder_type use_timestamp_folder;
      typedef bool _use_timestamp_file_title_type;
      _use_timestamp_file_title_type use_timestamp_file_title;

    MultipleUploadRequest():
      file_paths_length(0), file_paths(NULL),
      file_titles_length(0), file_titles(NULL),
      parents_path(""),
      parents_id(""),
      use_timestamp_folder(0),
      use_timestamp_file_title(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->file_paths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->file_paths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->file_paths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->file_paths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->file_paths_length);
      for( uint32_t i = 0; i < file_paths_length; i++){
      uint32_t length_file_pathsi = strlen(this->file_paths[i]);
      varToArr(outbuffer + offset, length_file_pathsi);
      offset += 4;
      memcpy(outbuffer + offset, this->file_paths[i], length_file_pathsi);
      offset += length_file_pathsi;
      }
      *(outbuffer + offset + 0) = (this->file_titles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->file_titles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->file_titles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->file_titles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->file_titles_length);
      for( uint32_t i = 0; i < file_titles_length; i++){
      uint32_t length_file_titlesi = strlen(this->file_titles[i]);
      varToArr(outbuffer + offset, length_file_titlesi);
      offset += 4;
      memcpy(outbuffer + offset, this->file_titles[i], length_file_titlesi);
      offset += length_file_titlesi;
      }
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
      uint32_t file_paths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      file_paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      file_paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      file_paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->file_paths_length);
      if(file_paths_lengthT > file_paths_length)
        this->file_paths = (char**)realloc(this->file_paths, file_paths_lengthT * sizeof(char*));
      file_paths_length = file_paths_lengthT;
      for( uint32_t i = 0; i < file_paths_length; i++){
      uint32_t length_st_file_paths;
      arrToVar(length_st_file_paths, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_file_paths; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_file_paths-1]=0;
      this->st_file_paths = (char *)(inbuffer + offset-1);
      offset += length_st_file_paths;
        memcpy( &(this->file_paths[i]), &(this->st_file_paths), sizeof(char*));
      }
      uint32_t file_titles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      file_titles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      file_titles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      file_titles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->file_titles_length);
      if(file_titles_lengthT > file_titles_length)
        this->file_titles = (char**)realloc(this->file_titles, file_titles_lengthT * sizeof(char*));
      file_titles_length = file_titles_lengthT;
      for( uint32_t i = 0; i < file_titles_length; i++){
      uint32_t length_st_file_titles;
      arrToVar(length_st_file_titles, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_file_titles; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_file_titles-1]=0;
      this->st_file_titles = (char *)(inbuffer + offset-1);
      offset += length_st_file_titles;
        memcpy( &(this->file_titles[i]), &(this->st_file_titles), sizeof(char*));
      }
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

    const char * getType(){ return MULTIPLEUPLOAD; };
    const char * getMD5(){ return "122e5b4128de0fb81389f9b17153330d"; };

  };

  class MultipleUploadResponse : public ros::Msg
  {
    public:
      uint32_t successes_length;
      typedef bool _successes_type;
      _successes_type st_successes;
      _successes_type * successes;
      uint32_t file_ids_length;
      typedef char* _file_ids_type;
      _file_ids_type st_file_ids;
      _file_ids_type * file_ids;
      uint32_t file_urls_length;
      typedef char* _file_urls_type;
      _file_urls_type st_file_urls;
      _file_urls_type * file_urls;
      typedef const char* _parents_id_type;
      _parents_id_type parents_id;
      typedef const char* _parents_url_type;
      _parents_url_type parents_url;

    MultipleUploadResponse():
      successes_length(0), successes(NULL),
      file_ids_length(0), file_ids(NULL),
      file_urls_length(0), file_urls(NULL),
      parents_id(""),
      parents_url("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->successes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->successes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->successes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->successes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->successes_length);
      for( uint32_t i = 0; i < successes_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_successesi;
      u_successesi.real = this->successes[i];
      *(outbuffer + offset + 0) = (u_successesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->successes[i]);
      }
      *(outbuffer + offset + 0) = (this->file_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->file_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->file_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->file_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->file_ids_length);
      for( uint32_t i = 0; i < file_ids_length; i++){
      uint32_t length_file_idsi = strlen(this->file_ids[i]);
      varToArr(outbuffer + offset, length_file_idsi);
      offset += 4;
      memcpy(outbuffer + offset, this->file_ids[i], length_file_idsi);
      offset += length_file_idsi;
      }
      *(outbuffer + offset + 0) = (this->file_urls_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->file_urls_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->file_urls_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->file_urls_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->file_urls_length);
      for( uint32_t i = 0; i < file_urls_length; i++){
      uint32_t length_file_urlsi = strlen(this->file_urls[i]);
      varToArr(outbuffer + offset, length_file_urlsi);
      offset += 4;
      memcpy(outbuffer + offset, this->file_urls[i], length_file_urlsi);
      offset += length_file_urlsi;
      }
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
      uint32_t successes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      successes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      successes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      successes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->successes_length);
      if(successes_lengthT > successes_length)
        this->successes = (bool*)realloc(this->successes, successes_lengthT * sizeof(bool));
      successes_length = successes_lengthT;
      for( uint32_t i = 0; i < successes_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_successes;
      u_st_successes.base = 0;
      u_st_successes.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_successes = u_st_successes.real;
      offset += sizeof(this->st_successes);
        memcpy( &(this->successes[i]), &(this->st_successes), sizeof(bool));
      }
      uint32_t file_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      file_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      file_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      file_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->file_ids_length);
      if(file_ids_lengthT > file_ids_length)
        this->file_ids = (char**)realloc(this->file_ids, file_ids_lengthT * sizeof(char*));
      file_ids_length = file_ids_lengthT;
      for( uint32_t i = 0; i < file_ids_length; i++){
      uint32_t length_st_file_ids;
      arrToVar(length_st_file_ids, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_file_ids; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_file_ids-1]=0;
      this->st_file_ids = (char *)(inbuffer + offset-1);
      offset += length_st_file_ids;
        memcpy( &(this->file_ids[i]), &(this->st_file_ids), sizeof(char*));
      }
      uint32_t file_urls_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      file_urls_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      file_urls_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      file_urls_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->file_urls_length);
      if(file_urls_lengthT > file_urls_length)
        this->file_urls = (char**)realloc(this->file_urls, file_urls_lengthT * sizeof(char*));
      file_urls_length = file_urls_lengthT;
      for( uint32_t i = 0; i < file_urls_length; i++){
      uint32_t length_st_file_urls;
      arrToVar(length_st_file_urls, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_file_urls; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_file_urls-1]=0;
      this->st_file_urls = (char *)(inbuffer + offset-1);
      offset += length_st_file_urls;
        memcpy( &(this->file_urls[i]), &(this->st_file_urls), sizeof(char*));
      }
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

    const char * getType(){ return MULTIPLEUPLOAD; };
    const char * getMD5(){ return "060c3b54ca60104d86730aba3a3da16c"; };

  };

  class MultipleUpload {
    public:
    typedef MultipleUploadRequest Request;
    typedef MultipleUploadResponse Response;
  };

}
#endif
