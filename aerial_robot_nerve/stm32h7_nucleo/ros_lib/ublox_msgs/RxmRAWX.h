#ifndef _ROS_ublox_msgs_RxmRAWX_h
#define _ROS_ublox_msgs_RxmRAWX_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/RxmRAWX_Meas.h"

namespace ublox_msgs
{

  class RxmRAWX : public ros::Msg
  {
    public:
      typedef double _rcvTOW_type;
      _rcvTOW_type rcvTOW;
      typedef uint16_t _week_type;
      _week_type week;
      typedef int8_t _leapS_type;
      _leapS_type leapS;
      typedef uint8_t _numMeas_type;
      _numMeas_type numMeas;
      typedef uint8_t _recStat_type;
      _recStat_type recStat;
      typedef uint8_t _version_type;
      _version_type version;
      uint8_t reserved1[2];
      uint32_t meas_length;
      typedef ublox_msgs::RxmRAWX_Meas _meas_type;
      _meas_type st_meas;
      _meas_type * meas;
      enum { CLASS_ID =  2 };
      enum { MESSAGE_ID =  21 };
      enum { REC_STAT_LEAP_SEC =  1    };
      enum { REC_STAT_CLK_RESET =  2   };

    RxmRAWX():
      rcvTOW(0),
      week(0),
      leapS(0),
      numMeas(0),
      recStat(0),
      version(0),
      reserved1(),
      meas_length(0), st_meas(), meas(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_rcvTOW;
      u_rcvTOW.real = this->rcvTOW;
      *(outbuffer + offset + 0) = (u_rcvTOW.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rcvTOW.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rcvTOW.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rcvTOW.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rcvTOW.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rcvTOW.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rcvTOW.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rcvTOW.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rcvTOW);
      *(outbuffer + offset + 0) = (this->week >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->week >> (8 * 1)) & 0xFF;
      offset += sizeof(this->week);
      union {
        int8_t real;
        uint8_t base;
      } u_leapS;
      u_leapS.real = this->leapS;
      *(outbuffer + offset + 0) = (u_leapS.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->leapS);
      *(outbuffer + offset + 0) = (this->numMeas >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numMeas);
      *(outbuffer + offset + 0) = (this->recStat >> (8 * 0)) & 0xFF;
      offset += sizeof(this->recStat);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->meas_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->meas_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->meas_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->meas_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meas_length);
      for( uint32_t i = 0; i < meas_length; i++){
      offset += this->meas[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_rcvTOW;
      u_rcvTOW.base = 0;
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rcvTOW.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rcvTOW = u_rcvTOW.real;
      offset += sizeof(this->rcvTOW);
      this->week =  ((uint16_t) (*(inbuffer + offset)));
      this->week |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->week);
      union {
        int8_t real;
        uint8_t base;
      } u_leapS;
      u_leapS.base = 0;
      u_leapS.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->leapS = u_leapS.real;
      offset += sizeof(this->leapS);
      this->numMeas =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numMeas);
      this->recStat =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->recStat);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      uint32_t meas_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      meas_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      meas_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      meas_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->meas_length);
      if(meas_lengthT > meas_length)
        this->meas = (ublox_msgs::RxmRAWX_Meas*)realloc(this->meas, meas_lengthT * sizeof(ublox_msgs::RxmRAWX_Meas));
      meas_length = meas_lengthT;
      for( uint32_t i = 0; i < meas_length; i++){
      offset += this->st_meas.deserialize(inbuffer + offset);
        memcpy( &(this->meas[i]), &(this->st_meas), sizeof(ublox_msgs::RxmRAWX_Meas));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmRAWX"; };
    virtual const char * getMD5() override { return "a2df4b27b6a2a1565e42f5669dbb11b5"; };

  };

}
#endif
