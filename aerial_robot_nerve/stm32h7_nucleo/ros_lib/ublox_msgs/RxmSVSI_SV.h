#ifndef _ROS_ublox_msgs_RxmSVSI_SV_h
#define _ROS_ublox_msgs_RxmSVSI_SV_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class RxmSVSI_SV : public ros::Msg
  {
    public:
      typedef uint8_t _svid_type;
      _svid_type svid;
      typedef uint8_t _svFlag_type;
      _svFlag_type svFlag;
      typedef int16_t _azim_type;
      _azim_type azim;
      typedef int8_t _elev_type;
      _elev_type elev;
      typedef uint8_t _age_type;
      _age_type age;
      enum { FLAG_URA_MASK =  15       };
      enum { FLAG_HEALTHY =  16        };
      enum { FLAG_EPH_VAL =  32        };
      enum { FLAG_ALM_VAL =  64        };
      enum { FLAG_NOT_AVAIL =  128     };
      enum { AGE_ALM_MASK =  15        };
      enum { AGE_EPH_MASK =  240       };

    RxmSVSI_SV():
      svid(0),
      svFlag(0),
      azim(0),
      elev(0),
      age(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->svFlag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svFlag);
      union {
        int16_t real;
        uint16_t base;
      } u_azim;
      u_azim.real = this->azim;
      *(outbuffer + offset + 0) = (u_azim.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_azim.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->azim);
      union {
        int8_t real;
        uint8_t base;
      } u_elev;
      u_elev.real = this->elev;
      *(outbuffer + offset + 0) = (u_elev.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->elev);
      *(outbuffer + offset + 0) = (this->age >> (8 * 0)) & 0xFF;
      offset += sizeof(this->age);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      this->svFlag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svFlag);
      union {
        int16_t real;
        uint16_t base;
      } u_azim;
      u_azim.base = 0;
      u_azim.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_azim.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->azim = u_azim.real;
      offset += sizeof(this->azim);
      union {
        int8_t real;
        uint8_t base;
      } u_elev;
      u_elev.base = 0;
      u_elev.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->elev = u_elev.real;
      offset += sizeof(this->elev);
      this->age =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->age);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/RxmSVSI_SV"; };
    virtual const char * getMD5() override { return "055e3ca33052c1635aff80c3f8ab6197"; };

  };

}
#endif
