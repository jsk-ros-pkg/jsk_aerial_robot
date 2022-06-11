#ifndef _ROS_aerial_robot_msgs_FourAxisGain_h
#define _ROS_aerial_robot_msgs_FourAxisGain_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace aerial_robot_msgs
{

  class FourAxisGain : public ros::Msg
  {
    public:
      uint32_t pitch_p_gain_length;
      typedef float _pitch_p_gain_type;
      _pitch_p_gain_type st_pitch_p_gain;
      _pitch_p_gain_type * pitch_p_gain;
      uint32_t pitch_i_gain_length;
      typedef float _pitch_i_gain_type;
      _pitch_i_gain_type st_pitch_i_gain;
      _pitch_i_gain_type * pitch_i_gain;
      uint32_t pitch_d_gain_length;
      typedef float _pitch_d_gain_type;
      _pitch_d_gain_type st_pitch_d_gain;
      _pitch_d_gain_type * pitch_d_gain;
      uint32_t roll_p_gain_length;
      typedef float _roll_p_gain_type;
      _roll_p_gain_type st_roll_p_gain;
      _roll_p_gain_type * roll_p_gain;
      uint32_t roll_i_gain_length;
      typedef float _roll_i_gain_type;
      _roll_i_gain_type st_roll_i_gain;
      _roll_i_gain_type * roll_i_gain;
      uint32_t roll_d_gain_length;
      typedef float _roll_d_gain_type;
      _roll_d_gain_type st_roll_d_gain;
      _roll_d_gain_type * roll_d_gain;
      uint32_t z_p_gain_length;
      typedef float _z_p_gain_type;
      _z_p_gain_type st_z_p_gain;
      _z_p_gain_type * z_p_gain;
      uint32_t z_i_gain_length;
      typedef float _z_i_gain_type;
      _z_i_gain_type st_z_i_gain;
      _z_i_gain_type * z_i_gain;
      uint32_t z_d_gain_length;
      typedef float _z_d_gain_type;
      _z_d_gain_type st_z_d_gain;
      _z_d_gain_type * z_d_gain;
      uint32_t yaw_p_gain_length;
      typedef float _yaw_p_gain_type;
      _yaw_p_gain_type st_yaw_p_gain;
      _yaw_p_gain_type * yaw_p_gain;
      uint32_t yaw_i_gain_length;
      typedef float _yaw_i_gain_type;
      _yaw_i_gain_type st_yaw_i_gain;
      _yaw_i_gain_type * yaw_i_gain;
      uint32_t yaw_d_gain_length;
      typedef float _yaw_d_gain_type;
      _yaw_d_gain_type st_yaw_d_gain;
      _yaw_d_gain_type * yaw_d_gain;

    FourAxisGain():
      pitch_p_gain_length(0), pitch_p_gain(NULL),
      pitch_i_gain_length(0), pitch_i_gain(NULL),
      pitch_d_gain_length(0), pitch_d_gain(NULL),
      roll_p_gain_length(0), roll_p_gain(NULL),
      roll_i_gain_length(0), roll_i_gain(NULL),
      roll_d_gain_length(0), roll_d_gain(NULL),
      z_p_gain_length(0), z_p_gain(NULL),
      z_i_gain_length(0), z_i_gain(NULL),
      z_d_gain_length(0), z_d_gain(NULL),
      yaw_p_gain_length(0), yaw_p_gain(NULL),
      yaw_i_gain_length(0), yaw_i_gain(NULL),
      yaw_d_gain_length(0), yaw_d_gain(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pitch_p_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pitch_p_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pitch_p_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pitch_p_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_p_gain_length);
      for( uint32_t i = 0; i < pitch_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_pitch_p_gaini;
      u_pitch_p_gaini.real = this->pitch_p_gain[i];
      *(outbuffer + offset + 0) = (u_pitch_p_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_p_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_p_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_p_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_p_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->pitch_i_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pitch_i_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pitch_i_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pitch_i_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_i_gain_length);
      for( uint32_t i = 0; i < pitch_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_pitch_i_gaini;
      u_pitch_i_gaini.real = this->pitch_i_gain[i];
      *(outbuffer + offset + 0) = (u_pitch_i_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_i_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_i_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_i_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_i_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->pitch_d_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pitch_d_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pitch_d_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pitch_d_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_d_gain_length);
      for( uint32_t i = 0; i < pitch_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_pitch_d_gaini;
      u_pitch_d_gaini.real = this->pitch_d_gain[i];
      *(outbuffer + offset + 0) = (u_pitch_d_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_d_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_d_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_d_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_d_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->roll_p_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->roll_p_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->roll_p_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->roll_p_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_p_gain_length);
      for( uint32_t i = 0; i < roll_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_roll_p_gaini;
      u_roll_p_gaini.real = this->roll_p_gain[i];
      *(outbuffer + offset + 0) = (u_roll_p_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_p_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_p_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_p_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_p_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->roll_i_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->roll_i_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->roll_i_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->roll_i_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_i_gain_length);
      for( uint32_t i = 0; i < roll_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_roll_i_gaini;
      u_roll_i_gaini.real = this->roll_i_gain[i];
      *(outbuffer + offset + 0) = (u_roll_i_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_i_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_i_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_i_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_i_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->roll_d_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->roll_d_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->roll_d_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->roll_d_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_d_gain_length);
      for( uint32_t i = 0; i < roll_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_roll_d_gaini;
      u_roll_d_gaini.real = this->roll_d_gain[i];
      *(outbuffer + offset + 0) = (u_roll_d_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_d_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_d_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_d_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_d_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->z_p_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_p_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_p_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_p_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_p_gain_length);
      for( uint32_t i = 0; i < z_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_z_p_gaini;
      u_z_p_gaini.real = this->z_p_gain[i];
      *(outbuffer + offset + 0) = (u_z_p_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_p_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_p_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_p_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_p_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->z_i_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_i_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_i_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_i_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_i_gain_length);
      for( uint32_t i = 0; i < z_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_z_i_gaini;
      u_z_i_gaini.real = this->z_i_gain[i];
      *(outbuffer + offset + 0) = (u_z_i_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_i_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_i_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_i_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_i_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->z_d_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_d_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_d_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_d_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_d_gain_length);
      for( uint32_t i = 0; i < z_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_z_d_gaini;
      u_z_d_gaini.real = this->z_d_gain[i];
      *(outbuffer + offset + 0) = (u_z_d_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_d_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z_d_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z_d_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_d_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->yaw_p_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_p_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_p_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_p_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_p_gain_length);
      for( uint32_t i = 0; i < yaw_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yaw_p_gaini;
      u_yaw_p_gaini.real = this->yaw_p_gain[i];
      *(outbuffer + offset + 0) = (u_yaw_p_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_p_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_p_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_p_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_p_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->yaw_i_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_i_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_i_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_i_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_i_gain_length);
      for( uint32_t i = 0; i < yaw_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yaw_i_gaini;
      u_yaw_i_gaini.real = this->yaw_i_gain[i];
      *(outbuffer + offset + 0) = (u_yaw_i_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_i_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_i_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_i_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_i_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->yaw_d_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->yaw_d_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->yaw_d_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->yaw_d_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_d_gain_length);
      for( uint32_t i = 0; i < yaw_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yaw_d_gaini;
      u_yaw_d_gaini.real = this->yaw_d_gain[i];
      *(outbuffer + offset + 0) = (u_yaw_d_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_d_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_d_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_d_gaini.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_d_gain[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pitch_p_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pitch_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pitch_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pitch_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pitch_p_gain_length);
      if(pitch_p_gain_lengthT > pitch_p_gain_length)
        this->pitch_p_gain = (float*)realloc(this->pitch_p_gain, pitch_p_gain_lengthT * sizeof(float));
      pitch_p_gain_length = pitch_p_gain_lengthT;
      for( uint32_t i = 0; i < pitch_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_pitch_p_gain;
      u_st_pitch_p_gain.base = 0;
      u_st_pitch_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pitch_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pitch_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pitch_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_pitch_p_gain = u_st_pitch_p_gain.real;
      offset += sizeof(this->st_pitch_p_gain);
        memcpy( &(this->pitch_p_gain[i]), &(this->st_pitch_p_gain), sizeof(float));
      }
      uint32_t pitch_i_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pitch_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pitch_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pitch_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pitch_i_gain_length);
      if(pitch_i_gain_lengthT > pitch_i_gain_length)
        this->pitch_i_gain = (float*)realloc(this->pitch_i_gain, pitch_i_gain_lengthT * sizeof(float));
      pitch_i_gain_length = pitch_i_gain_lengthT;
      for( uint32_t i = 0; i < pitch_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_pitch_i_gain;
      u_st_pitch_i_gain.base = 0;
      u_st_pitch_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pitch_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pitch_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pitch_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_pitch_i_gain = u_st_pitch_i_gain.real;
      offset += sizeof(this->st_pitch_i_gain);
        memcpy( &(this->pitch_i_gain[i]), &(this->st_pitch_i_gain), sizeof(float));
      }
      uint32_t pitch_d_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pitch_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pitch_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pitch_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pitch_d_gain_length);
      if(pitch_d_gain_lengthT > pitch_d_gain_length)
        this->pitch_d_gain = (float*)realloc(this->pitch_d_gain, pitch_d_gain_lengthT * sizeof(float));
      pitch_d_gain_length = pitch_d_gain_lengthT;
      for( uint32_t i = 0; i < pitch_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_pitch_d_gain;
      u_st_pitch_d_gain.base = 0;
      u_st_pitch_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pitch_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pitch_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pitch_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_pitch_d_gain = u_st_pitch_d_gain.real;
      offset += sizeof(this->st_pitch_d_gain);
        memcpy( &(this->pitch_d_gain[i]), &(this->st_pitch_d_gain), sizeof(float));
      }
      uint32_t roll_p_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      roll_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      roll_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      roll_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->roll_p_gain_length);
      if(roll_p_gain_lengthT > roll_p_gain_length)
        this->roll_p_gain = (float*)realloc(this->roll_p_gain, roll_p_gain_lengthT * sizeof(float));
      roll_p_gain_length = roll_p_gain_lengthT;
      for( uint32_t i = 0; i < roll_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_roll_p_gain;
      u_st_roll_p_gain.base = 0;
      u_st_roll_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_roll_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_roll_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_roll_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_roll_p_gain = u_st_roll_p_gain.real;
      offset += sizeof(this->st_roll_p_gain);
        memcpy( &(this->roll_p_gain[i]), &(this->st_roll_p_gain), sizeof(float));
      }
      uint32_t roll_i_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      roll_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      roll_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      roll_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->roll_i_gain_length);
      if(roll_i_gain_lengthT > roll_i_gain_length)
        this->roll_i_gain = (float*)realloc(this->roll_i_gain, roll_i_gain_lengthT * sizeof(float));
      roll_i_gain_length = roll_i_gain_lengthT;
      for( uint32_t i = 0; i < roll_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_roll_i_gain;
      u_st_roll_i_gain.base = 0;
      u_st_roll_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_roll_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_roll_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_roll_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_roll_i_gain = u_st_roll_i_gain.real;
      offset += sizeof(this->st_roll_i_gain);
        memcpy( &(this->roll_i_gain[i]), &(this->st_roll_i_gain), sizeof(float));
      }
      uint32_t roll_d_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      roll_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      roll_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      roll_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->roll_d_gain_length);
      if(roll_d_gain_lengthT > roll_d_gain_length)
        this->roll_d_gain = (float*)realloc(this->roll_d_gain, roll_d_gain_lengthT * sizeof(float));
      roll_d_gain_length = roll_d_gain_lengthT;
      for( uint32_t i = 0; i < roll_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_roll_d_gain;
      u_st_roll_d_gain.base = 0;
      u_st_roll_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_roll_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_roll_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_roll_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_roll_d_gain = u_st_roll_d_gain.real;
      offset += sizeof(this->st_roll_d_gain);
        memcpy( &(this->roll_d_gain[i]), &(this->st_roll_d_gain), sizeof(float));
      }
      uint32_t z_p_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_p_gain_length);
      if(z_p_gain_lengthT > z_p_gain_length)
        this->z_p_gain = (float*)realloc(this->z_p_gain, z_p_gain_lengthT * sizeof(float));
      z_p_gain_length = z_p_gain_lengthT;
      for( uint32_t i = 0; i < z_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_z_p_gain;
      u_st_z_p_gain.base = 0;
      u_st_z_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_z_p_gain = u_st_z_p_gain.real;
      offset += sizeof(this->st_z_p_gain);
        memcpy( &(this->z_p_gain[i]), &(this->st_z_p_gain), sizeof(float));
      }
      uint32_t z_i_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_i_gain_length);
      if(z_i_gain_lengthT > z_i_gain_length)
        this->z_i_gain = (float*)realloc(this->z_i_gain, z_i_gain_lengthT * sizeof(float));
      z_i_gain_length = z_i_gain_lengthT;
      for( uint32_t i = 0; i < z_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_z_i_gain;
      u_st_z_i_gain.base = 0;
      u_st_z_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_z_i_gain = u_st_z_i_gain.real;
      offset += sizeof(this->st_z_i_gain);
        memcpy( &(this->z_i_gain[i]), &(this->st_z_i_gain), sizeof(float));
      }
      uint32_t z_d_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_d_gain_length);
      if(z_d_gain_lengthT > z_d_gain_length)
        this->z_d_gain = (float*)realloc(this->z_d_gain, z_d_gain_lengthT * sizeof(float));
      z_d_gain_length = z_d_gain_lengthT;
      for( uint32_t i = 0; i < z_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_z_d_gain;
      u_st_z_d_gain.base = 0;
      u_st_z_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_z_d_gain = u_st_z_d_gain.real;
      offset += sizeof(this->st_z_d_gain);
        memcpy( &(this->z_d_gain[i]), &(this->st_z_d_gain), sizeof(float));
      }
      uint32_t yaw_p_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_p_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_p_gain_length);
      if(yaw_p_gain_lengthT > yaw_p_gain_length)
        this->yaw_p_gain = (float*)realloc(this->yaw_p_gain, yaw_p_gain_lengthT * sizeof(float));
      yaw_p_gain_length = yaw_p_gain_lengthT;
      for( uint32_t i = 0; i < yaw_p_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_yaw_p_gain;
      u_st_yaw_p_gain.base = 0;
      u_st_yaw_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_yaw_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_yaw_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_yaw_p_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_yaw_p_gain = u_st_yaw_p_gain.real;
      offset += sizeof(this->st_yaw_p_gain);
        memcpy( &(this->yaw_p_gain[i]), &(this->st_yaw_p_gain), sizeof(float));
      }
      uint32_t yaw_i_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_i_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_i_gain_length);
      if(yaw_i_gain_lengthT > yaw_i_gain_length)
        this->yaw_i_gain = (float*)realloc(this->yaw_i_gain, yaw_i_gain_lengthT * sizeof(float));
      yaw_i_gain_length = yaw_i_gain_lengthT;
      for( uint32_t i = 0; i < yaw_i_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_yaw_i_gain;
      u_st_yaw_i_gain.base = 0;
      u_st_yaw_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_yaw_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_yaw_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_yaw_i_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_yaw_i_gain = u_st_yaw_i_gain.real;
      offset += sizeof(this->st_yaw_i_gain);
        memcpy( &(this->yaw_i_gain[i]), &(this->st_yaw_i_gain), sizeof(float));
      }
      uint32_t yaw_d_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      yaw_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      yaw_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      yaw_d_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->yaw_d_gain_length);
      if(yaw_d_gain_lengthT > yaw_d_gain_length)
        this->yaw_d_gain = (float*)realloc(this->yaw_d_gain, yaw_d_gain_lengthT * sizeof(float));
      yaw_d_gain_length = yaw_d_gain_lengthT;
      for( uint32_t i = 0; i < yaw_d_gain_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_yaw_d_gain;
      u_st_yaw_d_gain.base = 0;
      u_st_yaw_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_yaw_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_yaw_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_yaw_d_gain.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_yaw_d_gain = u_st_yaw_d_gain.real;
      offset += sizeof(this->st_yaw_d_gain);
        memcpy( &(this->yaw_d_gain[i]), &(this->st_yaw_d_gain), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/FourAxisGain"; };
    const char * getMD5(){ return "3c6a2479670464c17ccfa32890752308"; };

  };

}
#endif
