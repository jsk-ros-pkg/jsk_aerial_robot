#ifndef _ROS_speech_recognition_msgs_Grammar_h
#define _ROS_speech_recognition_msgs_Grammar_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "speech_recognition_msgs/PhraseRule.h"
#include "speech_recognition_msgs/Vocabulary.h"

namespace speech_recognition_msgs
{

  class Grammar : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      uint32_t rules_length;
      typedef speech_recognition_msgs::PhraseRule _rules_type;
      _rules_type st_rules;
      _rules_type * rules;
      uint32_t categories_length;
      typedef char* _categories_type;
      _categories_type st_categories;
      _categories_type * categories;
      uint32_t vocabularies_length;
      typedef speech_recognition_msgs::Vocabulary _vocabularies_type;
      _vocabularies_type st_vocabularies;
      _vocabularies_type * vocabularies;

    Grammar():
      name(""),
      rules_length(0), rules(NULL),
      categories_length(0), categories(NULL),
      vocabularies_length(0), vocabularies(NULL)
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
      *(outbuffer + offset + 0) = (this->rules_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rules_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rules_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rules_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rules_length);
      for( uint32_t i = 0; i < rules_length; i++){
      offset += this->rules[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->categories_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->categories_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->categories_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->categories_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->categories_length);
      for( uint32_t i = 0; i < categories_length; i++){
      uint32_t length_categoriesi = strlen(this->categories[i]);
      varToArr(outbuffer + offset, length_categoriesi);
      offset += 4;
      memcpy(outbuffer + offset, this->categories[i], length_categoriesi);
      offset += length_categoriesi;
      }
      *(outbuffer + offset + 0) = (this->vocabularies_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vocabularies_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vocabularies_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vocabularies_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vocabularies_length);
      for( uint32_t i = 0; i < vocabularies_length; i++){
      offset += this->vocabularies[i].serialize(outbuffer + offset);
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
      uint32_t rules_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rules_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rules_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rules_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rules_length);
      if(rules_lengthT > rules_length)
        this->rules = (speech_recognition_msgs::PhraseRule*)realloc(this->rules, rules_lengthT * sizeof(speech_recognition_msgs::PhraseRule));
      rules_length = rules_lengthT;
      for( uint32_t i = 0; i < rules_length; i++){
      offset += this->st_rules.deserialize(inbuffer + offset);
        memcpy( &(this->rules[i]), &(this->st_rules), sizeof(speech_recognition_msgs::PhraseRule));
      }
      uint32_t categories_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      categories_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      categories_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      categories_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->categories_length);
      if(categories_lengthT > categories_length)
        this->categories = (char**)realloc(this->categories, categories_lengthT * sizeof(char*));
      categories_length = categories_lengthT;
      for( uint32_t i = 0; i < categories_length; i++){
      uint32_t length_st_categories;
      arrToVar(length_st_categories, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_categories; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_categories-1]=0;
      this->st_categories = (char *)(inbuffer + offset-1);
      offset += length_st_categories;
        memcpy( &(this->categories[i]), &(this->st_categories), sizeof(char*));
      }
      uint32_t vocabularies_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vocabularies_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vocabularies_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vocabularies_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vocabularies_length);
      if(vocabularies_lengthT > vocabularies_length)
        this->vocabularies = (speech_recognition_msgs::Vocabulary*)realloc(this->vocabularies, vocabularies_lengthT * sizeof(speech_recognition_msgs::Vocabulary));
      vocabularies_length = vocabularies_lengthT;
      for( uint32_t i = 0; i < vocabularies_length; i++){
      offset += this->st_vocabularies.deserialize(inbuffer + offset);
        memcpy( &(this->vocabularies[i]), &(this->st_vocabularies), sizeof(speech_recognition_msgs::Vocabulary));
      }
     return offset;
    }

    const char * getType(){ return "speech_recognition_msgs/Grammar"; };
    const char * getMD5(){ return "a8653cae3429492cb944a813429e7151"; };

  };

}
#endif
