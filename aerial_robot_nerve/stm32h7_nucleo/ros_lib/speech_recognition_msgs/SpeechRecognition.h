#ifndef _ROS_SERVICE_SpeechRecognition_h
#define _ROS_SERVICE_SpeechRecognition_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "speech_recognition_msgs/Vocabulary.h"
#include "speech_recognition_msgs/Grammar.h"
#include "speech_recognition_msgs/SpeechRecognitionCandidates.h"

namespace speech_recognition_msgs
{

static const char SPEECHRECOGNITION[] = "speech_recognition_msgs/SpeechRecognition";

  class SpeechRecognitionRequest : public ros::Msg
  {
    public:
      typedef speech_recognition_msgs::Vocabulary _vocabulary_type;
      _vocabulary_type vocabulary;
      typedef speech_recognition_msgs::Grammar _grammar_type;
      _grammar_type grammar;
      typedef const char* _grammar_name_type;
      _grammar_name_type grammar_name;
      typedef float _duration_type;
      _duration_type duration;
      typedef bool _quiet_type;
      _quiet_type quiet;
      typedef float _threshold_type;
      _threshold_type threshold;

    SpeechRecognitionRequest():
      vocabulary(),
      grammar(),
      grammar_name(""),
      duration(0),
      quiet(0),
      threshold(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->vocabulary.serialize(outbuffer + offset);
      offset += this->grammar.serialize(outbuffer + offset);
      uint32_t length_grammar_name = strlen(this->grammar_name);
      varToArr(outbuffer + offset, length_grammar_name);
      offset += 4;
      memcpy(outbuffer + offset, this->grammar_name, length_grammar_name);
      offset += length_grammar_name;
      union {
        float real;
        uint32_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->duration);
      union {
        bool real;
        uint8_t base;
      } u_quiet;
      u_quiet.real = this->quiet;
      *(outbuffer + offset + 0) = (u_quiet.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->quiet);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.real = this->threshold;
      *(outbuffer + offset + 0) = (u_threshold.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_threshold.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_threshold.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_threshold.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->threshold);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->vocabulary.deserialize(inbuffer + offset);
      offset += this->grammar.deserialize(inbuffer + offset);
      uint32_t length_grammar_name;
      arrToVar(length_grammar_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_grammar_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_grammar_name-1]=0;
      this->grammar_name = (char *)(inbuffer + offset-1);
      offset += length_grammar_name;
      union {
        float real;
        uint32_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
      union {
        bool real;
        uint8_t base;
      } u_quiet;
      u_quiet.base = 0;
      u_quiet.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->quiet = u_quiet.real;
      offset += sizeof(this->quiet);
      union {
        float real;
        uint32_t base;
      } u_threshold;
      u_threshold.base = 0;
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_threshold.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->threshold = u_threshold.real;
      offset += sizeof(this->threshold);
     return offset;
    }

    const char * getType(){ return SPEECHRECOGNITION; };
    const char * getMD5(){ return "af5602408bd36e4d9a80cde6f4453023"; };

  };

  class SpeechRecognitionResponse : public ros::Msg
  {
    public:
      typedef speech_recognition_msgs::SpeechRecognitionCandidates _result_type;
      _result_type result;

    SpeechRecognitionResponse():
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SPEECHRECOGNITION; };
    const char * getMD5(){ return "46fe009ac10a19a0e861b8792ad42e0b"; };

  };

  class SpeechRecognition {
    public:
    typedef SpeechRecognitionRequest Request;
    typedef SpeechRecognitionResponse Response;
  };

}
#endif
