#ifndef _ROS_aerial_robot_msgs_State_h
#define _ROS_aerial_robot_msgs_State_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace aerial_robot_msgs
{

  class State : public ros::Msg
  {
    public:
      typedef const char* _id_type;
      _id_type id;
      uint32_t state_length;
      typedef geometry_msgs::Vector3 _state_type;
      _state_type st_state;
      _state_type * state;
      uint32_t reserves_length;
      typedef float _reserves_type;
      _reserves_type st_reserves;
      _reserves_type * reserves;
      enum { EGOMOTION_ESTIMATE =  0 };
      enum { EXPERIMENT_ESTIMATE =  1 };
      enum { GROUND_TRUTH =  2 };

    State():
      id(""),
      state_length(0), state(NULL),
      reserves_length(0), reserves(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      *(outbuffer + offset + 0) = (this->state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_length);
      for( uint32_t i = 0; i < state_length; i++){
      offset += this->state[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->reserves_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserves_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->reserves_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->reserves_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reserves_length);
      for( uint32_t i = 0; i < reserves_length; i++){
      union {
        float real;
        uint32_t base;
      } u_reservesi;
      u_reservesi.real = this->reserves[i];
      *(outbuffer + offset + 0) = (u_reservesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reservesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reservesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reservesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->reserves[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      uint32_t state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_length);
      if(state_lengthT > state_length)
        this->state = (geometry_msgs::Vector3*)realloc(this->state, state_lengthT * sizeof(geometry_msgs::Vector3));
      state_length = state_lengthT;
      for( uint32_t i = 0; i < state_length; i++){
      offset += this->st_state.deserialize(inbuffer + offset);
        memcpy( &(this->state[i]), &(this->st_state), sizeof(geometry_msgs::Vector3));
      }
      uint32_t reserves_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      reserves_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      reserves_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      reserves_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->reserves_length);
      if(reserves_lengthT > reserves_length)
        this->reserves = (float*)realloc(this->reserves, reserves_lengthT * sizeof(float));
      reserves_length = reserves_lengthT;
      for( uint32_t i = 0; i < reserves_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_reserves;
      u_st_reserves.base = 0;
      u_st_reserves.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_reserves.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_reserves.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_reserves.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_reserves = u_st_reserves.real;
      offset += sizeof(this->st_reserves);
        memcpy( &(this->reserves[i]), &(this->st_reserves), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/State"; };
    const char * getMD5(){ return "5a670218e8d0304e57d6b78071c0bfbd"; };

  };

}
#endif
