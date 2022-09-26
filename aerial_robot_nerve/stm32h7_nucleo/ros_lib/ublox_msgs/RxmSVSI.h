#ifndef _ROS_ublox_msgs_RxmSVSI_h
#define _ROS_ublox_msgs_RxmSVSI_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/RxmSVSI_SV.h"

namespace ublox_msgs
{

  class RxmSVSI : public ros::Msg
  {
    public:
      typedef int32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef int16_t _week_type;
      _week_type week;
      typedef uint8_t _numVis_type;
      _numVis_type numVis;
      typedef uint8_t _numSV_type;
      _numSV_type numSV;
      uint32_t sv_length;
      typedef ublox_msgs::RxmSVSI_SV _sv_type;
      _sv_type st_sv;
      _sv_type * sv;
      enum { CLASS_ID =  2 };
      enum { MESSAGE_ID =  32 };

    RxmSVSI():
      iTOW(0),
      week(0),
      numVis(0),
      numSV(0),
      sv_length(0), st_sv(), sv(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_iTOW;
      u_iTOW.real = this->iTOW;
      *(outbuffer + offset + 0) = (u_iTOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_iTOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_iTOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_iTOW.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->iTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.real = this->week;
      *(outbuffer + offset + 0) = (u_week.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_week.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->week);
      *(outbuffer + offset + 0) = (this->numVis >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numVis);
      *(outbuffer + offset + 0) = (this->numSV >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSV);
      *(outbuffer + offset + 0) = (this->sv_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sv_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sv_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sv_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sv_length);
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->sv[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_iTOW;
      u_iTOW.base = 0;
      u_iTOW.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_iTOW.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_iTOW.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_iTOW.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->iTOW = u_iTOW.real;
      offset += sizeof(this->iTOW);
      union {
        int16_t real;
        uint16_t base;
      } u_week;
      u_week.base = 0;
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_week.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->week = u_week.real;
      offset += sizeof(this->week);
      this->numVis =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numVis);
      this->numSV =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSV);
      uint32_t sv_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sv_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sv_length);
      if(sv_lengthT > sv_length)
        this->sv = (ublox_msgs::RxmSVSI_SV*)realloc(this->sv, sv_lengthT * sizeof(ublox_msgs::RxmSVSI_SV));
      sv_length = sv_lengthT;
      for( uint32_t i = 0; i < sv_length; i++){
      offset += this->st_sv.deserialize(inbuffer + offset);
        memcpy( &(this->sv[i]), &(this->st_sv), sizeof(ublox_msgs::RxmSVSI_SV));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmSVSI"; };
    virtual const char * getMD5() override { return "8358c0613232b962d48cf13f5bda0070"; };

  };

}
#endif
