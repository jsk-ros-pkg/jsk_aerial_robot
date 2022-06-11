#ifndef _ROS_rostwitter_TweetGoal_h
#define _ROS_rostwitter_TweetGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rostwitter
{

  class TweetGoal : public ros::Msg
  {
    public:
      typedef bool _image_type;
      _image_type image;
      typedef const char* _image_topic_name_type;
      _image_topic_name_type image_topic_name;
      typedef bool _speak_type;
      _speak_type speak;
      typedef const char* _text_type;
      _text_type text;
      typedef bool _warning_type;
      _warning_type warning;
      typedef uint8_t _warning_time_type;
      _warning_time_type warning_time;

    TweetGoal():
      image(0),
      image_topic_name(""),
      speak(0),
      text(""),
      warning(0),
      warning_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_image;
      u_image.real = this->image;
      *(outbuffer + offset + 0) = (u_image.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->image);
      uint32_t length_image_topic_name = strlen(this->image_topic_name);
      varToArr(outbuffer + offset, length_image_topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->image_topic_name, length_image_topic_name);
      offset += length_image_topic_name;
      union {
        bool real;
        uint8_t base;
      } u_speak;
      u_speak.real = this->speak;
      *(outbuffer + offset + 0) = (u_speak.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speak);
      uint32_t length_text = strlen(this->text);
      varToArr(outbuffer + offset, length_text);
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      union {
        bool real;
        uint8_t base;
      } u_warning;
      u_warning.real = this->warning;
      *(outbuffer + offset + 0) = (u_warning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->warning);
      *(outbuffer + offset + 0) = (this->warning_time >> (8 * 0)) & 0xFF;
      offset += sizeof(this->warning_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_image;
      u_image.base = 0;
      u_image.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->image = u_image.real;
      offset += sizeof(this->image);
      uint32_t length_image_topic_name;
      arrToVar(length_image_topic_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_image_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_image_topic_name-1]=0;
      this->image_topic_name = (char *)(inbuffer + offset-1);
      offset += length_image_topic_name;
      union {
        bool real;
        uint8_t base;
      } u_speak;
      u_speak.base = 0;
      u_speak.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speak = u_speak.real;
      offset += sizeof(this->speak);
      uint32_t length_text;
      arrToVar(length_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      union {
        bool real;
        uint8_t base;
      } u_warning;
      u_warning.base = 0;
      u_warning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->warning = u_warning.real;
      offset += sizeof(this->warning);
      this->warning_time =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->warning_time);
     return offset;
    }

    const char * getType(){ return "rostwitter/TweetGoal"; };
    const char * getMD5(){ return "577f0283150a250e0ca629c7c0d07aa9"; };

  };

}
#endif
