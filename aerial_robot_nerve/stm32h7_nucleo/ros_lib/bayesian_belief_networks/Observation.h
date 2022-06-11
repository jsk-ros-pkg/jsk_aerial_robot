#ifndef _ROS_bayesian_belief_networks_Observation_h
#define _ROS_bayesian_belief_networks_Observation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace bayesian_belief_networks
{

  class Observation : public ros::Msg
  {
    public:
      typedef const char* _node_type;
      _node_type node;
      typedef const char* _evidence_type;
      _evidence_type evidence;

    Observation():
      node(""),
      evidence("")
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
      uint32_t length_evidence = strlen(this->evidence);
      varToArr(outbuffer + offset, length_evidence);
      offset += 4;
      memcpy(outbuffer + offset, this->evidence, length_evidence);
      offset += length_evidence;
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
      uint32_t length_evidence;
      arrToVar(length_evidence, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_evidence; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_evidence-1]=0;
      this->evidence = (char *)(inbuffer + offset-1);
      offset += length_evidence;
     return offset;
    }

    const char * getType(){ return "bayesian_belief_networks/Observation"; };
    const char * getMD5(){ return "381015522e25503885bf04a57ab55e63"; };

  };

}
#endif
