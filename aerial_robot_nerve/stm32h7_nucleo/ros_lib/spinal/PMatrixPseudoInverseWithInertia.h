#ifndef _ROS_spinal_PMatrixPseudoInverseWithInertia_h
#define _ROS_spinal_PMatrixPseudoInverseWithInertia_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/PMatrixPseudoInverseUnit.h"

namespace spinal
{

  class PMatrixPseudoInverseWithInertia : public ros::Msg
  {
    public:
      uint32_t pseudo_inverse_length;
      typedef spinal::PMatrixPseudoInverseUnit _pseudo_inverse_type;
      _pseudo_inverse_type st_pseudo_inverse;
      _pseudo_inverse_type * pseudo_inverse;
      int16_t inertia[6];

    PMatrixPseudoInverseWithInertia():
      pseudo_inverse_length(0), pseudo_inverse(NULL),
      inertia()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pseudo_inverse_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pseudo_inverse_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pseudo_inverse_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pseudo_inverse_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pseudo_inverse_length);
      for( uint32_t i = 0; i < pseudo_inverse_length; i++){
      offset += this->pseudo_inverse[i].serialize(outbuffer + offset);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_inertiai;
      u_inertiai.real = this->inertia[i];
      *(outbuffer + offset + 0) = (u_inertiai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_inertiai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->inertia[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pseudo_inverse_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pseudo_inverse_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pseudo_inverse_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pseudo_inverse_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pseudo_inverse_length);
      if(pseudo_inverse_lengthT > pseudo_inverse_length)
        this->pseudo_inverse = (spinal::PMatrixPseudoInverseUnit*)realloc(this->pseudo_inverse, pseudo_inverse_lengthT * sizeof(spinal::PMatrixPseudoInverseUnit));
      pseudo_inverse_length = pseudo_inverse_lengthT;
      for( uint32_t i = 0; i < pseudo_inverse_length; i++){
      offset += this->st_pseudo_inverse.deserialize(inbuffer + offset);
        memcpy( &(this->pseudo_inverse[i]), &(this->st_pseudo_inverse), sizeof(spinal::PMatrixPseudoInverseUnit));
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_inertiai;
      u_inertiai.base = 0;
      u_inertiai.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_inertiai.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->inertia[i] = u_inertiai.real;
      offset += sizeof(this->inertia[i]);
      }
     return offset;
    }

    const char * getType(){ return "spinal/PMatrixPseudoInverseWithInertia"; };
    const char * getMD5(){ return "e67a441fd1d34930b8789fd5b10a7fe7"; };

  };

}
#endif
