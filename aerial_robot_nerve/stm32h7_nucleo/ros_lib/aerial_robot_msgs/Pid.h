#ifndef _ROS_aerial_robot_msgs_Pid_h
#define _ROS_aerial_robot_msgs_Pid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace aerial_robot_msgs
{

  class Pid : public ros::Msg
  {
    public:
      uint32_t total_length;
      typedef float _total_type;
      _total_type st_total;
      _total_type * total;
      uint32_t p_term_length;
      typedef float _p_term_type;
      _p_term_type st_p_term;
      _p_term_type * p_term;
      uint32_t i_term_length;
      typedef float _i_term_type;
      _i_term_type st_i_term;
      _i_term_type * i_term;
      uint32_t d_term_length;
      typedef float _d_term_type;
      _d_term_type st_d_term;
      _d_term_type * d_term;
      typedef float _target_p_type;
      _target_p_type target_p;
      typedef float _err_p_type;
      _err_p_type err_p;
      typedef float _target_d_type;
      _target_d_type target_d;
      typedef float _err_d_type;
      _err_d_type err_d;

    Pid():
      total_length(0), total(NULL),
      p_term_length(0), p_term(NULL),
      i_term_length(0), i_term(NULL),
      d_term_length(0), d_term(NULL),
      target_p(0),
      err_p(0),
      target_d(0),
      err_d(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->total_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->total_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->total_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->total_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_length);
      for( uint32_t i = 0; i < total_length; i++){
      union {
        float real;
        uint32_t base;
      } u_totali;
      u_totali.real = this->total[i];
      *(outbuffer + offset + 0) = (u_totali.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_totali.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_totali.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_totali.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total[i]);
      }
      *(outbuffer + offset + 0) = (this->p_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_term_length);
      for( uint32_t i = 0; i < p_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_p_termi;
      u_p_termi.real = this->p_term[i];
      *(outbuffer + offset + 0) = (u_p_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_term[i]);
      }
      *(outbuffer + offset + 0) = (this->i_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_term_length);
      for( uint32_t i = 0; i < i_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_i_termi;
      u_i_termi.real = this->i_term[i];
      *(outbuffer + offset + 0) = (u_i_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_term[i]);
      }
      *(outbuffer + offset + 0) = (this->d_term_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->d_term_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->d_term_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->d_term_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_term_length);
      for( uint32_t i = 0; i < d_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_d_termi;
      u_d_termi.real = this->d_term[i];
      *(outbuffer + offset + 0) = (u_d_termi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_termi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_termi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_termi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d_term[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_target_p;
      u_target_p.real = this->target_p;
      *(outbuffer + offset + 0) = (u_target_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_p);
      union {
        float real;
        uint32_t base;
      } u_err_p;
      u_err_p.real = this->err_p;
      *(outbuffer + offset + 0) = (u_err_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->err_p);
      union {
        float real;
        uint32_t base;
      } u_target_d;
      u_target_d.real = this->target_d;
      *(outbuffer + offset + 0) = (u_target_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target_d);
      union {
        float real;
        uint32_t base;
      } u_err_d;
      u_err_d.real = this->err_d;
      *(outbuffer + offset + 0) = (u_err_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->err_d);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t total_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      total_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      total_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      total_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->total_length);
      if(total_lengthT > total_length)
        this->total = (float*)realloc(this->total, total_lengthT * sizeof(float));
      total_length = total_lengthT;
      for( uint32_t i = 0; i < total_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_total;
      u_st_total.base = 0;
      u_st_total.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_total.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_total.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_total.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_total = u_st_total.real;
      offset += sizeof(this->st_total);
        memcpy( &(this->total[i]), &(this->st_total), sizeof(float));
      }
      uint32_t p_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_term_length);
      if(p_term_lengthT > p_term_length)
        this->p_term = (float*)realloc(this->p_term, p_term_lengthT * sizeof(float));
      p_term_length = p_term_lengthT;
      for( uint32_t i = 0; i < p_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_p_term;
      u_st_p_term.base = 0;
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_p_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_p_term = u_st_p_term.real;
      offset += sizeof(this->st_p_term);
        memcpy( &(this->p_term[i]), &(this->st_p_term), sizeof(float));
      }
      uint32_t i_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_term_length);
      if(i_term_lengthT > i_term_length)
        this->i_term = (float*)realloc(this->i_term, i_term_lengthT * sizeof(float));
      i_term_length = i_term_lengthT;
      for( uint32_t i = 0; i < i_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_i_term;
      u_st_i_term.base = 0;
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_i_term = u_st_i_term.real;
      offset += sizeof(this->st_i_term);
        memcpy( &(this->i_term[i]), &(this->st_i_term), sizeof(float));
      }
      uint32_t d_term_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      d_term_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->d_term_length);
      if(d_term_lengthT > d_term_length)
        this->d_term = (float*)realloc(this->d_term, d_term_lengthT * sizeof(float));
      d_term_length = d_term_lengthT;
      for( uint32_t i = 0; i < d_term_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_d_term;
      u_st_d_term.base = 0;
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_d_term.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_d_term = u_st_d_term.real;
      offset += sizeof(this->st_d_term);
        memcpy( &(this->d_term[i]), &(this->st_d_term), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_target_p;
      u_target_p.base = 0;
      u_target_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_p = u_target_p.real;
      offset += sizeof(this->target_p);
      union {
        float real;
        uint32_t base;
      } u_err_p;
      u_err_p.base = 0;
      u_err_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->err_p = u_err_p.real;
      offset += sizeof(this->err_p);
      union {
        float real;
        uint32_t base;
      } u_target_d;
      u_target_d.base = 0;
      u_target_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target_d = u_target_d.real;
      offset += sizeof(this->target_d);
      union {
        float real;
        uint32_t base;
      } u_err_d;
      u_err_d.base = 0;
      u_err_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->err_d = u_err_d.real;
      offset += sizeof(this->err_d);
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/Pid"; };
    const char * getMD5(){ return "db161b2062dd6acaa882c985d44815a8"; };

  };

}
#endif
