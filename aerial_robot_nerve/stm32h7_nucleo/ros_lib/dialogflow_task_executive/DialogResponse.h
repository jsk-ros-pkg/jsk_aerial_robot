#ifndef _ROS_dialogflow_task_executive_DialogResponse_h
#define _ROS_dialogflow_task_executive_DialogResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dialogflow_task_executive
{

  class DialogResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _action_type;
      _action_type action;
      typedef const char* _query_type;
      _query_type query;
      typedef const char* _response_type;
      _response_type response;
      typedef bool _fulfilled_type;
      _fulfilled_type fulfilled;
      typedef const char* _parameters_type;
      _parameters_type parameters;
      typedef float _speech_score_type;
      _speech_score_type speech_score;
      typedef float _intent_score_type;
      _intent_score_type intent_score;

    DialogResponse():
      header(),
      action(""),
      query(""),
      response(""),
      fulfilled(0),
      parameters(""),
      speech_score(0),
      intent_score(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_action = strlen(this->action);
      varToArr(outbuffer + offset, length_action);
      offset += 4;
      memcpy(outbuffer + offset, this->action, length_action);
      offset += length_action;
      uint32_t length_query = strlen(this->query);
      varToArr(outbuffer + offset, length_query);
      offset += 4;
      memcpy(outbuffer + offset, this->query, length_query);
      offset += length_query;
      uint32_t length_response = strlen(this->response);
      varToArr(outbuffer + offset, length_response);
      offset += 4;
      memcpy(outbuffer + offset, this->response, length_response);
      offset += length_response;
      union {
        bool real;
        uint8_t base;
      } u_fulfilled;
      u_fulfilled.real = this->fulfilled;
      *(outbuffer + offset + 0) = (u_fulfilled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fulfilled);
      uint32_t length_parameters = strlen(this->parameters);
      varToArr(outbuffer + offset, length_parameters);
      offset += 4;
      memcpy(outbuffer + offset, this->parameters, length_parameters);
      offset += length_parameters;
      union {
        float real;
        uint32_t base;
      } u_speech_score;
      u_speech_score.real = this->speech_score;
      *(outbuffer + offset + 0) = (u_speech_score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speech_score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speech_score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speech_score.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speech_score);
      union {
        float real;
        uint32_t base;
      } u_intent_score;
      u_intent_score.real = this->intent_score;
      *(outbuffer + offset + 0) = (u_intent_score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intent_score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intent_score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intent_score.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intent_score);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_action;
      arrToVar(length_action, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action-1]=0;
      this->action = (char *)(inbuffer + offset-1);
      offset += length_action;
      uint32_t length_query;
      arrToVar(length_query, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_query; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_query-1]=0;
      this->query = (char *)(inbuffer + offset-1);
      offset += length_query;
      uint32_t length_response;
      arrToVar(length_response, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_response-1]=0;
      this->response = (char *)(inbuffer + offset-1);
      offset += length_response;
      union {
        bool real;
        uint8_t base;
      } u_fulfilled;
      u_fulfilled.base = 0;
      u_fulfilled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fulfilled = u_fulfilled.real;
      offset += sizeof(this->fulfilled);
      uint32_t length_parameters;
      arrToVar(length_parameters, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_parameters; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_parameters-1]=0;
      this->parameters = (char *)(inbuffer + offset-1);
      offset += length_parameters;
      union {
        float real;
        uint32_t base;
      } u_speech_score;
      u_speech_score.base = 0;
      u_speech_score.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speech_score.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speech_score.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speech_score.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speech_score = u_speech_score.real;
      offset += sizeof(this->speech_score);
      union {
        float real;
        uint32_t base;
      } u_intent_score;
      u_intent_score.base = 0;
      u_intent_score.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intent_score.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intent_score.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intent_score.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->intent_score = u_intent_score.real;
      offset += sizeof(this->intent_score);
     return offset;
    }

    const char * getType(){ return "dialogflow_task_executive/DialogResponse"; };
    const char * getMD5(){ return "f90e598f24cd5cbe87ba946987c3c0af"; };

  };

}
#endif
