#ifndef _ROS_ublox_msgs_RxmRAW_SV_h
#define _ROS_ublox_msgs_RxmRAW_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmRAW_SV : public ros::Msg
  {
    public:
      typedef double _cpMes_type;
      _cpMes_type cpMes;
      typedef double _prMes_type;
      _prMes_type prMes;
      typedef float _doMes_type;
      _doMes_type doMes;
      typedef uint8_t _sv_type;
      _sv_type sv;
      typedef int8_t _mesQI_type;
      _mesQI_type mesQI;
      typedef int8_t _cno_type;
      _cno_type cno;
      typedef uint8_t _lli_type;
      _lli_type lli;

    RxmRAW_SV():
      cpMes(0),
      prMes(0),
      doMes(0),
      sv(0),
      mesQI(0),
      cno(0),
      lli(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_cpMes;
      u_cpMes.real = this->cpMes;
      *(outbuffer + offset + 0) = (u_cpMes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cpMes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cpMes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cpMes.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_cpMes.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_cpMes.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_cpMes.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_cpMes.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cpMes);
      union {
        double real;
        uint64_t base;
      } u_prMes;
      u_prMes.real = this->prMes;
      *(outbuffer + offset + 0) = (u_prMes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_prMes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_prMes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_prMes.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_prMes.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_prMes.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_prMes.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_prMes.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->prMes);
      union {
        float real;
        uint32_t base;
      } u_doMes;
      u_doMes.real = this->doMes;
      *(outbuffer + offset + 0) = (u_doMes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_doMes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_doMes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_doMes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->doMes);
      *(outbuffer + offset + 0) = (this->sv >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sv);
      union {
        int8_t real;
        uint8_t base;
      } u_mesQI;
      u_mesQI.real = this->mesQI;
      *(outbuffer + offset + 0) = (u_mesQI.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mesQI);
      union {
        int8_t real;
        uint8_t base;
      } u_cno;
      u_cno.real = this->cno;
      *(outbuffer + offset + 0) = (u_cno.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cno);
      *(outbuffer + offset + 0) = (this->lli >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lli);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_cpMes;
      u_cpMes.base = 0;
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_cpMes.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->cpMes = u_cpMes.real;
      offset += sizeof(this->cpMes);
      union {
        double real;
        uint64_t base;
      } u_prMes;
      u_prMes.base = 0;
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_prMes.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->prMes = u_prMes.real;
      offset += sizeof(this->prMes);
      union {
        float real;
        uint32_t base;
      } u_doMes;
      u_doMes.base = 0;
      u_doMes.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_doMes.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_doMes.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_doMes.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->doMes = u_doMes.real;
      offset += sizeof(this->doMes);
      this->sv =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sv);
      union {
        int8_t real;
        uint8_t base;
      } u_mesQI;
      u_mesQI.base = 0;
      u_mesQI.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mesQI = u_mesQI.real;
      offset += sizeof(this->mesQI);
      union {
        int8_t real;
        uint8_t base;
      } u_cno;
      u_cno.base = 0;
      u_cno.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cno = u_cno.real;
      offset += sizeof(this->cno);
      this->lli =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->lli);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmRAW_SV"; };
    virtual const char * getMD5() override { return "4b32efd29577416a2c280e629abcb69a"; };

  };

}
#endif
