#ifndef _ROS_speech_recognition_msgs_PhraseRule_h
#define _ROS_speech_recognition_msgs_PhraseRule_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace speech_recognition_msgs
{

  class PhraseRule : public ros::Msg
  {
    public:
      typedef const char* _symbol_type;
      _symbol_type symbol;
      uint32_t definition_length;
      typedef char* _definition_type;
      _definition_type st_definition;
      _definition_type * definition;

    PhraseRule():
      symbol(""),
      definition_length(0), definition(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_symbol = strlen(this->symbol);
      varToArr(outbuffer + offset, length_symbol);
      offset += 4;
      memcpy(outbuffer + offset, this->symbol, length_symbol);
      offset += length_symbol;
      *(outbuffer + offset + 0) = (this->definition_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->definition_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->definition_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->definition_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->definition_length);
      for( uint32_t i = 0; i < definition_length; i++){
      uint32_t length_definitioni = strlen(this->definition[i]);
      varToArr(outbuffer + offset, length_definitioni);
      offset += 4;
      memcpy(outbuffer + offset, this->definition[i], length_definitioni);
      offset += length_definitioni;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_symbol;
      arrToVar(length_symbol, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_symbol; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_symbol-1]=0;
      this->symbol = (char *)(inbuffer + offset-1);
      offset += length_symbol;
      uint32_t definition_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      definition_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      definition_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      definition_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->definition_length);
      if(definition_lengthT > definition_length)
        this->definition = (char**)realloc(this->definition, definition_lengthT * sizeof(char*));
      definition_length = definition_lengthT;
      for( uint32_t i = 0; i < definition_length; i++){
      uint32_t length_st_definition;
      arrToVar(length_st_definition, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_definition; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_definition-1]=0;
      this->st_definition = (char *)(inbuffer + offset-1);
      offset += length_st_definition;
        memcpy( &(this->definition[i]), &(this->st_definition), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "speech_recognition_msgs/PhraseRule"; };
    const char * getMD5(){ return "8184f0f93fdc3a6768ac26cd56040fdd"; };

  };

}
#endif
