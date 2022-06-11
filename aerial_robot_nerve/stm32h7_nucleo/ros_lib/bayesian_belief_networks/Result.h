#ifndef _ROS_bayesian_belief_networks_Result_h
#define _ROS_bayesian_belief_networks_Result_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bayesian_belief_networks
{

  class Result : public ros::Msg
  {
    public:
      typedef const char* _node_type;
      _node_type node;
      typedef const char* _Value_type;
      _Value_type Value;
      typedef double _Marginal_type;
      _Marginal_type Marginal;

    Result():
      node(""),
      Value(""),
      Marginal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_node = strlen(this->node);
      varToArr(outbuffer + offset, length_node);
      offset += 4;
      memcpy(outbuffer + offset, this->node, length_node);
      offset += length_node;
      uint32_t length_Value = strlen(this->Value);
      varToArr(outbuffer + offset, length_Value);
      offset += 4;
      memcpy(outbuffer + offset, this->Value, length_Value);
      offset += length_Value;
      union {
        double real;
        uint64_t base;
      } u_Marginal;
      u_Marginal.real = this->Marginal;
      *(outbuffer + offset + 0) = (u_Marginal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Marginal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Marginal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Marginal.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_Marginal.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_Marginal.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_Marginal.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_Marginal.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->Marginal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_node;
      arrToVar(length_node, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node-1]=0;
      this->node = (char *)(inbuffer + offset-1);
      offset += length_node;
      uint32_t length_Value;
      arrToVar(length_Value, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Value-1]=0;
      this->Value = (char *)(inbuffer + offset-1);
      offset += length_Value;
      union {
        double real;
        uint64_t base;
      } u_Marginal;
      u_Marginal.base = 0;
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_Marginal.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->Marginal = u_Marginal.real;
      offset += sizeof(this->Marginal);
     return offset;
    }

    const char * getType(){ return "bayesian_belief_networks/Result"; };
    const char * getMD5(){ return "c12bc86306f498c9d6b12dcfb4492003"; };

  };

}
#endif
