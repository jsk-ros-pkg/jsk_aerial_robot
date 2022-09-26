#ifndef _ROS_ublox_msgs_NavDOP_h
#define _ROS_ublox_msgs_NavDOP_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class NavDOP : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint16_t _gDOP_type;
      _gDOP_type gDOP;
      typedef uint16_t _pDOP_type;
      _pDOP_type pDOP;
      typedef uint16_t _tDOP_type;
      _tDOP_type tDOP;
      typedef uint16_t _vDOP_type;
      _vDOP_type vDOP;
      typedef uint16_t _hDOP_type;
      _hDOP_type hDOP;
      typedef uint16_t _nDOP_type;
      _nDOP_type nDOP;
      typedef uint16_t _eDOP_type;
      _eDOP_type eDOP;
      enum { CLASS_ID =  1 };
      enum { MESSAGE_ID =  4 };

    NavDOP():
      iTOW(0),
      gDOP(0),
      pDOP(0),
      tDOP(0),
      vDOP(0),
      hDOP(0),
      nDOP(0),
      eDOP(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->iTOW >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iTOW >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->iTOW >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->iTOW >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      *(outbuffer + offset + 0) = (this->gDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->gDOP);
      *(outbuffer + offset + 0) = (this->pDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pDOP);
      *(outbuffer + offset + 0) = (this->tDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tDOP);
      *(outbuffer + offset + 0) = (this->vDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vDOP);
      *(outbuffer + offset + 0) = (this->hDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->hDOP);
      *(outbuffer + offset + 0) = (this->nDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->nDOP);
      *(outbuffer + offset + 0) = (this->eDOP >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->eDOP >> (8 * 1)) & 0xFF;
      offset += sizeof(this->eDOP);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->iTOW =  ((uint32_t) (*(inbuffer + offset)));
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->iTOW |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->iTOW);
      this->gDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->gDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->gDOP);
      this->pDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->pDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pDOP);
      this->tDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->tDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tDOP);
      this->vDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->vDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->vDOP);
      this->hDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->hDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->hDOP);
      this->nDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->nDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->nDOP);
      this->eDOP =  ((uint16_t) (*(inbuffer + offset)));
      this->eDOP |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->eDOP);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/NavDOP"; };
    virtual const char * getMD5() override { return "19fe2210fc48e52c1c14b7d2c567407f"; };

  };

}
#endif
