#ifndef _ROS_SERVICE_Query_h
#define _ROS_SERVICE_Query_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "bayesian_belief_networks/Observation.h"
#include "bayesian_belief_networks/Result.h"

namespace bayesian_belief_networks
{

static const char QUERY[] = "bayesian_belief_networks/Query";

  class QueryRequest : public ros::Msg
  {
    public:
      uint32_t query_length;
      typedef bayesian_belief_networks::Observation _query_type;
      _query_type st_query;
      _query_type * query;

    QueryRequest():
      query_length(0), query(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->query_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->query_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->query_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->query_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->query_length);
      for( uint32_t i = 0; i < query_length; i++){
      offset += this->query[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t query_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      query_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      query_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      query_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->query_length);
      if(query_lengthT > query_length)
        this->query = (bayesian_belief_networks::Observation*)realloc(this->query, query_lengthT * sizeof(bayesian_belief_networks::Observation));
      query_length = query_lengthT;
      for( uint32_t i = 0; i < query_length; i++){
      offset += this->st_query.deserialize(inbuffer + offset);
        memcpy( &(this->query[i]), &(this->st_query), sizeof(bayesian_belief_networks::Observation));
      }
     return offset;
    }

    const char * getType(){ return QUERY; };
    const char * getMD5(){ return "c82ad1bda0500c7fa7fed33d8deb2a3f"; };

  };

  class QueryResponse : public ros::Msg
  {
    public:
      uint32_t results_length;
      typedef bayesian_belief_networks::Result _results_type;
      _results_type st_results;
      _results_type * results;

    QueryResponse():
      results_length(0), results(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->results_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->results_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->results_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->results_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->results_length);
      for( uint32_t i = 0; i < results_length; i++){
      offset += this->results[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t results_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      results_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      results_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      results_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->results_length);
      if(results_lengthT > results_length)
        this->results = (bayesian_belief_networks::Result*)realloc(this->results, results_lengthT * sizeof(bayesian_belief_networks::Result));
      results_length = results_lengthT;
      for( uint32_t i = 0; i < results_length; i++){
      offset += this->st_results.deserialize(inbuffer + offset);
        memcpy( &(this->results[i]), &(this->st_results), sizeof(bayesian_belief_networks::Result));
      }
     return offset;
    }

    const char * getType(){ return QUERY; };
    const char * getMD5(){ return "f41c31876c3be91ef922fc26aa614079"; };

  };

  class Query {
    public:
    typedef QueryRequest Request;
    typedef QueryResponse Response;
  };

}
#endif
