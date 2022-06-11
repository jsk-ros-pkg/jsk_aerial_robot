#ifndef _ROS_speech_recognition_msgs_Vocabulary_h
#define _ROS_speech_recognition_msgs_Vocabulary_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace speech_recognition_msgs
{

  class Vocabulary : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t words_length;
      typedef char* _words_type;
      _words_type st_words;
      _words_type * words;
      uint32_t phonemes_length;
      typedef char* _phonemes_type;
      _phonemes_type st_phonemes;
      _phonemes_type * phonemes;

    Vocabulary():
      name(""),
      words_length(0), words(NULL),
      phonemes_length(0), phonemes(NULL)
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
      *(outbuffer + offset + 0) = (this->words_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->words_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->words_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->words_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->words_length);
      for( uint32_t i = 0; i < words_length; i++){
      uint32_t length_wordsi = strlen(this->words[i]);
      varToArr(outbuffer + offset, length_wordsi);
      offset += 4;
      memcpy(outbuffer + offset, this->words[i], length_wordsi);
      offset += length_wordsi;
      }
      *(outbuffer + offset + 0) = (this->phonemes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->phonemes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->phonemes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->phonemes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phonemes_length);
      for( uint32_t i = 0; i < phonemes_length; i++){
      uint32_t length_phonemesi = strlen(this->phonemes[i]);
      varToArr(outbuffer + offset, length_phonemesi);
      offset += 4;
      memcpy(outbuffer + offset, this->phonemes[i], length_phonemesi);
      offset += length_phonemesi;
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
      uint32_t words_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      words_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->words_length);
      if(words_lengthT > words_length)
        this->words = (char**)realloc(this->words, words_lengthT * sizeof(char*));
      words_length = words_lengthT;
      for( uint32_t i = 0; i < words_length; i++){
      uint32_t length_st_words;
      arrToVar(length_st_words, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_words; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_words-1]=0;
      this->st_words = (char *)(inbuffer + offset-1);
      offset += length_st_words;
        memcpy( &(this->words[i]), &(this->st_words), sizeof(char*));
      }
      uint32_t phonemes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      phonemes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      phonemes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      phonemes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->phonemes_length);
      if(phonemes_lengthT > phonemes_length)
        this->phonemes = (char**)realloc(this->phonemes, phonemes_lengthT * sizeof(char*));
      phonemes_length = phonemes_lengthT;
      for( uint32_t i = 0; i < phonemes_length; i++){
      uint32_t length_st_phonemes;
      arrToVar(length_st_phonemes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_phonemes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_phonemes-1]=0;
      this->st_phonemes = (char *)(inbuffer + offset-1);
      offset += length_st_phonemes;
        memcpy( &(this->phonemes[i]), &(this->st_phonemes), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "speech_recognition_msgs/Vocabulary"; };
    const char * getMD5(){ return "20a1ff9e31d8f4dc29f230a64ed707d7"; };

  };

}
#endif
