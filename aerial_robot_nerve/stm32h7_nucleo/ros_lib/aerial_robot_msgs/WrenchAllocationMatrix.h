#ifndef _ROS_aerial_robot_msgs_WrenchAllocationMatrix_h
#define _ROS_aerial_robot_msgs_WrenchAllocationMatrix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace aerial_robot_msgs
{

  class WrenchAllocationMatrix : public ros::Msg
  {
    public:
      uint32_t f_x_length;
      typedef double _f_x_type;
      _f_x_type st_f_x;
      _f_x_type * f_x;
      uint32_t f_y_length;
      typedef double _f_y_type;
      _f_y_type st_f_y;
      _f_y_type * f_y;
      uint32_t f_z_length;
      typedef double _f_z_type;
      _f_z_type st_f_z;
      _f_z_type * f_z;
      uint32_t t_x_length;
      typedef double _t_x_type;
      _t_x_type st_t_x;
      _t_x_type * t_x;
      uint32_t t_y_length;
      typedef double _t_y_type;
      _t_y_type st_t_y;
      _t_y_type * t_y;
      uint32_t t_z_length;
      typedef double _t_z_type;
      _t_z_type st_t_z;
      _t_z_type * t_z;

    WrenchAllocationMatrix():
      f_x_length(0), f_x(NULL),
      f_y_length(0), f_y(NULL),
      f_z_length(0), f_z(NULL),
      t_x_length(0), t_x(NULL),
      t_y_length(0), t_y(NULL),
      t_z_length(0), t_z(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->f_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->f_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->f_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->f_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->f_x_length);
      for( uint32_t i = 0; i < f_x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_f_xi;
      u_f_xi.real = this->f_x[i];
      *(outbuffer + offset + 0) = (u_f_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_f_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_f_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_f_xi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_f_xi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_f_xi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_f_xi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_f_xi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->f_x[i]);
      }
      *(outbuffer + offset + 0) = (this->f_y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->f_y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->f_y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->f_y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->f_y_length);
      for( uint32_t i = 0; i < f_y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_f_yi;
      u_f_yi.real = this->f_y[i];
      *(outbuffer + offset + 0) = (u_f_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_f_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_f_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_f_yi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_f_yi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_f_yi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_f_yi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_f_yi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->f_y[i]);
      }
      *(outbuffer + offset + 0) = (this->f_z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->f_z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->f_z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->f_z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->f_z_length);
      for( uint32_t i = 0; i < f_z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_f_zi;
      u_f_zi.real = this->f_z[i];
      *(outbuffer + offset + 0) = (u_f_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_f_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_f_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_f_zi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_f_zi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_f_zi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_f_zi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_f_zi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->f_z[i]);
      }
      *(outbuffer + offset + 0) = (this->t_x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t_x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->t_x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->t_x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t_x_length);
      for( uint32_t i = 0; i < t_x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_t_xi;
      u_t_xi.real = this->t_x[i];
      *(outbuffer + offset + 0) = (u_t_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t_xi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_t_xi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_t_xi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_t_xi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_t_xi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->t_x[i]);
      }
      *(outbuffer + offset + 0) = (this->t_y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t_y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->t_y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->t_y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t_y_length);
      for( uint32_t i = 0; i < t_y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_t_yi;
      u_t_yi.real = this->t_y[i];
      *(outbuffer + offset + 0) = (u_t_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t_yi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_t_yi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_t_yi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_t_yi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_t_yi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->t_y[i]);
      }
      *(outbuffer + offset + 0) = (this->t_z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->t_z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->t_z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->t_z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->t_z_length);
      for( uint32_t i = 0; i < t_z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_t_zi;
      u_t_zi.real = this->t_z[i];
      *(outbuffer + offset + 0) = (u_t_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t_zi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_t_zi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_t_zi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_t_zi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_t_zi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->t_z[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t f_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      f_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      f_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      f_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->f_x_length);
      if(f_x_lengthT > f_x_length)
        this->f_x = (double*)realloc(this->f_x, f_x_lengthT * sizeof(double));
      f_x_length = f_x_lengthT;
      for( uint32_t i = 0; i < f_x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_f_x;
      u_st_f_x.base = 0;
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_f_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_f_x = u_st_f_x.real;
      offset += sizeof(this->st_f_x);
        memcpy( &(this->f_x[i]), &(this->st_f_x), sizeof(double));
      }
      uint32_t f_y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      f_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      f_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      f_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->f_y_length);
      if(f_y_lengthT > f_y_length)
        this->f_y = (double*)realloc(this->f_y, f_y_lengthT * sizeof(double));
      f_y_length = f_y_lengthT;
      for( uint32_t i = 0; i < f_y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_f_y;
      u_st_f_y.base = 0;
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_f_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_f_y = u_st_f_y.real;
      offset += sizeof(this->st_f_y);
        memcpy( &(this->f_y[i]), &(this->st_f_y), sizeof(double));
      }
      uint32_t f_z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      f_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      f_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      f_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->f_z_length);
      if(f_z_lengthT > f_z_length)
        this->f_z = (double*)realloc(this->f_z, f_z_lengthT * sizeof(double));
      f_z_length = f_z_lengthT;
      for( uint32_t i = 0; i < f_z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_f_z;
      u_st_f_z.base = 0;
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_f_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_f_z = u_st_f_z.real;
      offset += sizeof(this->st_f_z);
        memcpy( &(this->f_z[i]), &(this->st_f_z), sizeof(double));
      }
      uint32_t t_x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      t_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      t_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      t_x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->t_x_length);
      if(t_x_lengthT > t_x_length)
        this->t_x = (double*)realloc(this->t_x, t_x_lengthT * sizeof(double));
      t_x_length = t_x_lengthT;
      for( uint32_t i = 0; i < t_x_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_t_x;
      u_st_t_x.base = 0;
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_t_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_t_x = u_st_t_x.real;
      offset += sizeof(this->st_t_x);
        memcpy( &(this->t_x[i]), &(this->st_t_x), sizeof(double));
      }
      uint32_t t_y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      t_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      t_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      t_y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->t_y_length);
      if(t_y_lengthT > t_y_length)
        this->t_y = (double*)realloc(this->t_y, t_y_lengthT * sizeof(double));
      t_y_length = t_y_lengthT;
      for( uint32_t i = 0; i < t_y_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_t_y;
      u_st_t_y.base = 0;
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_t_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_t_y = u_st_t_y.real;
      offset += sizeof(this->st_t_y);
        memcpy( &(this->t_y[i]), &(this->st_t_y), sizeof(double));
      }
      uint32_t t_z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      t_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      t_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      t_z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->t_z_length);
      if(t_z_lengthT > t_z_length)
        this->t_z = (double*)realloc(this->t_z, t_z_lengthT * sizeof(double));
      t_z_length = t_z_lengthT;
      for( uint32_t i = 0; i < t_z_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_t_z;
      u_st_t_z.base = 0;
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_t_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_t_z = u_st_t_z.real;
      offset += sizeof(this->st_t_z);
        memcpy( &(this->t_z[i]), &(this->st_t_z), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/WrenchAllocationMatrix"; };
    const char * getMD5(){ return "210f7f48d0b8c031cdcdabd2fecc9eca"; };

  };

}
#endif
