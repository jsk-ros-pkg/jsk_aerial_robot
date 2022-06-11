#ifndef _ROS_pr2_msgs_LaserTrajCmd_h
#define _ROS_pr2_msgs_LaserTrajCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace pr2_msgs
{

  class LaserTrajCmd : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _profile_type;
      _profile_type profile;
      uint32_t position_length;
      typedef double _position_type;
      _position_type st_position;
      _position_type * position;
      uint32_t time_from_start_length;
      typedef ros::Duration _time_from_start_type;
      _time_from_start_type st_time_from_start;
      _time_from_start_type * time_from_start;
      typedef double _max_velocity_type;
      _max_velocity_type max_velocity;
      typedef double _max_acceleration_type;
      _max_acceleration_type max_acceleration;

    LaserTrajCmd():
      header(),
      profile(""),
      position_length(0), position(NULL),
      time_from_start_length(0), time_from_start(NULL),
      max_velocity(0),
      max_acceleration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_profile = strlen(this->profile);
      varToArr(outbuffer + offset, length_profile);
      offset += 4;
      memcpy(outbuffer + offset, this->profile, length_profile);
      offset += length_profile;
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_positioni.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_positioni.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_positioni.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_positioni.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      *(outbuffer + offset + 0) = (this->time_from_start_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start_length);
      for( uint32_t i = 0; i < time_from_start_length; i++){
      *(outbuffer + offset + 0) = (this->time_from_start[i].sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start[i].sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start[i].sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start[i].sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start[i].sec);
      *(outbuffer + offset + 0) = (this->time_from_start[i].nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start[i].nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start[i].nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start[i].nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start[i].nsec);
      }
      union {
        double real;
        uint64_t base;
      } u_max_velocity;
      u_max_velocity.real = this->max_velocity;
      *(outbuffer + offset + 0) = (u_max_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_velocity.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_velocity.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_velocity.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_velocity.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_velocity.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_velocity);
      union {
        double real;
        uint64_t base;
      } u_max_acceleration;
      u_max_acceleration.real = this->max_acceleration;
      *(outbuffer + offset + 0) = (u_max_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_acceleration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_acceleration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_acceleration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_acceleration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_acceleration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_acceleration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_profile;
      arrToVar(length_profile, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_profile; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_profile-1]=0;
      this->profile = (char *)(inbuffer + offset-1);
      offset += length_profile;
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (double*)realloc(this->position, position_lengthT * sizeof(double));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position;
      u_st_position.base = 0;
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position = u_st_position.real;
      offset += sizeof(this->st_position);
        memcpy( &(this->position[i]), &(this->st_position), sizeof(double));
      }
      uint32_t time_from_start_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      time_from_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      time_from_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      time_from_start_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->time_from_start_length);
      if(time_from_start_lengthT > time_from_start_length)
        this->time_from_start = (ros::Duration*)realloc(this->time_from_start, time_from_start_lengthT * sizeof(ros::Duration));
      time_from_start_length = time_from_start_lengthT;
      for( uint32_t i = 0; i < time_from_start_length; i++){
      this->st_time_from_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_time_from_start.sec);
      this->st_time_from_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_time_from_start.nsec);
        memcpy( &(this->time_from_start[i]), &(this->st_time_from_start), sizeof(ros::Duration));
      }
      union {
        double real;
        uint64_t base;
      } u_max_velocity;
      u_max_velocity.base = 0;
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_velocity = u_max_velocity.real;
      offset += sizeof(this->max_velocity);
      union {
        double real;
        uint64_t base;
      } u_max_acceleration;
      u_max_acceleration.base = 0;
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_acceleration = u_max_acceleration.real;
      offset += sizeof(this->max_acceleration);
     return offset;
    }

    const char * getType(){ return "pr2_msgs/LaserTrajCmd"; };
    const char * getMD5(){ return "68a1665e9079049dce55a0384cb2e9b5"; };

  };

}
#endif
