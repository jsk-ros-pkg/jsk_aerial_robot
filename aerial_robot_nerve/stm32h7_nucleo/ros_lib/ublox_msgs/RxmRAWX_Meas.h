#ifndef _ROS_ublox_msgs_RxmRAWX_Meas_h
#define _ROS_ublox_msgs_RxmRAWX_Meas_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmRAWX_Meas : public ros::Msg
  {
    public:
      typedef double _prMes_type;
      _prMes_type prMes;
      typedef double _cpMes_type;
      _cpMes_type cpMes;
      typedef float _doMes_type;
      _doMes_type doMes;
      typedef uint8_t _gnssId_type;
      _gnssId_type gnssId;
      typedef uint8_t _svId_type;
      _svId_type svId;
      typedef uint8_t _reserved0_type;
      _reserved0_type reserved0;
      typedef uint8_t _freqId_type;
      _freqId_type freqId;
      typedef uint16_t _locktime_type;
      _locktime_type locktime;
      typedef int8_t _cno_type;
      _cno_type cno;
      typedef uint8_t _prStdev_type;
      _prStdev_type prStdev;
      typedef uint8_t _cpStdev_type;
      _cpStdev_type cpStdev;
      typedef uint8_t _doStdev_type;
      _doStdev_type doStdev;
      typedef uint8_t _trkStat_type;
      _trkStat_type trkStat;
      typedef uint8_t _reserved1_type;
      _reserved1_type reserved1;
      enum { TRK_STAT_PR_VALID =  1        };
      enum { TRK_STAT_CP_VALID =  2        };
      enum { TRK_STAT_HALF_CYC =  4        };
      enum { TRK_STAT_SUB_HALF_CYC =  8    };

    RxmRAWX_Meas():
      prMes(0),
      cpMes(0),
      doMes(0),
      gnssId(0),
      svId(0),
      reserved0(0),
      freqId(0),
      locktime(0),
      cno(0),
      prStdev(0),
      cpStdev(0),
      doStdev(0),
      trkStat(0),
      reserved1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
        float real;
        uint32_t base;
      } u_doMes;
      u_doMes.real = this->doMes;
      *(outbuffer + offset + 0) = (u_doMes.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_doMes.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_doMes.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_doMes.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->doMes);
      *(outbuffer + offset + 0) = (this->gnssId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gnssId);
      *(outbuffer + offset + 0) = (this->svId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svId);
      *(outbuffer + offset + 0) = (this->reserved0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0);
      *(outbuffer + offset + 0) = (this->freqId >> (8 * 0)) & 0xFF;
      offset += sizeof(this->freqId);
      *(outbuffer + offset + 0) = (this->locktime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->locktime >> (8 * 1)) & 0xFF;
      offset += sizeof(this->locktime);
      union {
        int8_t real;
        uint8_t base;
      } u_cno;
      u_cno.real = this->cno;
      *(outbuffer + offset + 0) = (u_cno.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cno);
      *(outbuffer + offset + 0) = (this->prStdev >> (8 * 0)) & 0xFF;
      offset += sizeof(this->prStdev);
      *(outbuffer + offset + 0) = (this->cpStdev >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cpStdev);
      *(outbuffer + offset + 0) = (this->doStdev >> (8 * 0)) & 0xFF;
      offset += sizeof(this->doStdev);
      *(outbuffer + offset + 0) = (this->trkStat >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trkStat);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      this->gnssId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gnssId);
      this->svId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svId);
      this->reserved0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0);
      this->freqId =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->freqId);
      this->locktime =  ((uint16_t) (*(inbuffer + offset)));
      this->locktime |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->locktime);
      union {
        int8_t real;
        uint8_t base;
      } u_cno;
      u_cno.base = 0;
      u_cno.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cno = u_cno.real;
      offset += sizeof(this->cno);
      this->prStdev =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->prStdev);
      this->cpStdev =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->cpStdev);
      this->doStdev =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->doStdev);
      this->trkStat =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->trkStat);
      this->reserved1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmRAWX_Meas"; };
    virtual const char * getMD5() override { return "d6a580262875bf83a377ba14dcdd659f"; };

  };

}
#endif
