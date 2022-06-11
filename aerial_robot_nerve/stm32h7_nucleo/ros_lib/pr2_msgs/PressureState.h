#ifndef _ROS_pr2_msgs_PressureState_h
#define _ROS_pr2_msgs_PressureState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class PressureState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t l_finger_tip_length;
      typedef int16_t _l_finger_tip_type;
      _l_finger_tip_type st_l_finger_tip;
      _l_finger_tip_type * l_finger_tip;
      uint32_t r_finger_tip_length;
      typedef int16_t _r_finger_tip_type;
      _r_finger_tip_type st_r_finger_tip;
      _r_finger_tip_type * r_finger_tip;

    PressureState():
      header(),
      l_finger_tip_length(0), l_finger_tip(NULL),
      r_finger_tip_length(0), r_finger_tip(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->l_finger_tip_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->l_finger_tip_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->l_finger_tip_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->l_finger_tip_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->l_finger_tip_length);
      for( uint32_t i = 0; i < l_finger_tip_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_l_finger_tipi;
      u_l_finger_tipi.real = this->l_finger_tip[i];
      *(outbuffer + offset + 0) = (u_l_finger_tipi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_l_finger_tipi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->l_finger_tip[i]);
      }
      *(outbuffer + offset + 0) = (this->r_finger_tip_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->r_finger_tip_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->r_finger_tip_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->r_finger_tip_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r_finger_tip_length);
      for( uint32_t i = 0; i < r_finger_tip_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_r_finger_tipi;
      u_r_finger_tipi.real = this->r_finger_tip[i];
      *(outbuffer + offset + 0) = (u_r_finger_tipi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r_finger_tipi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->r_finger_tip[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t l_finger_tip_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      l_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      l_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      l_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->l_finger_tip_length);
      if(l_finger_tip_lengthT > l_finger_tip_length)
        this->l_finger_tip = (int16_t*)realloc(this->l_finger_tip, l_finger_tip_lengthT * sizeof(int16_t));
      l_finger_tip_length = l_finger_tip_lengthT;
      for( uint32_t i = 0; i < l_finger_tip_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_l_finger_tip;
      u_st_l_finger_tip.base = 0;
      u_st_l_finger_tip.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_l_finger_tip.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_l_finger_tip = u_st_l_finger_tip.real;
      offset += sizeof(this->st_l_finger_tip);
        memcpy( &(this->l_finger_tip[i]), &(this->st_l_finger_tip), sizeof(int16_t));
      }
      uint32_t r_finger_tip_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      r_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      r_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      r_finger_tip_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->r_finger_tip_length);
      if(r_finger_tip_lengthT > r_finger_tip_length)
        this->r_finger_tip = (int16_t*)realloc(this->r_finger_tip, r_finger_tip_lengthT * sizeof(int16_t));
      r_finger_tip_length = r_finger_tip_lengthT;
      for( uint32_t i = 0; i < r_finger_tip_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_r_finger_tip;
      u_st_r_finger_tip.base = 0;
      u_st_r_finger_tip.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_r_finger_tip.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_r_finger_tip = u_st_r_finger_tip.real;
      offset += sizeof(this->st_r_finger_tip);
        memcpy( &(this->r_finger_tip[i]), &(this->st_r_finger_tip), sizeof(int16_t));
      }
     return offset;
    }

    const char * getType(){ return "pr2_msgs/PressureState"; };
    const char * getMD5(){ return "756fb3b75fa8884524fd0789a78eb04b"; };

  };

}
#endif
