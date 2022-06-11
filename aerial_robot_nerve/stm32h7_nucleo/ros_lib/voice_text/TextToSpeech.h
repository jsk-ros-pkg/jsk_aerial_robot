#ifndef _ROS_SERVICE_TextToSpeech_h
#define _ROS_SERVICE_TextToSpeech_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace voice_text
{

static const char TEXTTOSPEECH[] = "voice_text/TextToSpeech";

  class TextToSpeechRequest : public ros::Msg
  {
    public:
      typedef const char* _text_path_type;
      _text_path_type text_path;
      typedef const char* _wave_path_type;
      _wave_path_type wave_path;

    TextToSpeechRequest():
      text_path(""),
      wave_path("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_text_path = strlen(this->text_path);
      varToArr(outbuffer + offset, length_text_path);
      offset += 4;
      memcpy(outbuffer + offset, this->text_path, length_text_path);
      offset += length_text_path;
      uint32_t length_wave_path = strlen(this->wave_path);
      varToArr(outbuffer + offset, length_wave_path);
      offset += 4;
      memcpy(outbuffer + offset, this->wave_path, length_wave_path);
      offset += length_wave_path;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_text_path;
      arrToVar(length_text_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text_path-1]=0;
      this->text_path = (char *)(inbuffer + offset-1);
      offset += length_text_path;
      uint32_t length_wave_path;
      arrToVar(length_wave_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_wave_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_wave_path-1]=0;
      this->wave_path = (char *)(inbuffer + offset-1);
      offset += length_wave_path;
     return offset;
    }

    const char * getType(){ return TEXTTOSPEECH; };
    const char * getMD5(){ return "2313d9c56eb0e0571a953bf2b40316f4"; };

  };

  class TextToSpeechResponse : public ros::Msg
  {
    public:
      typedef bool _ok_type;
      _ok_type ok;

    TextToSpeechResponse():
      ok(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.real = this->ok;
      *(outbuffer + offset + 0) = (u_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ok);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ok;
      u_ok.base = 0;
      u_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ok = u_ok.real;
      offset += sizeof(this->ok);
     return offset;
    }

    const char * getType(){ return TEXTTOSPEECH; };
    const char * getMD5(){ return "6f6da3883749771fac40d6deb24a8c02"; };

  };

  class TextToSpeech {
    public:
    typedef TextToSpeechRequest Request;
    typedef TextToSpeechResponse Response;
  };

}
#endif
