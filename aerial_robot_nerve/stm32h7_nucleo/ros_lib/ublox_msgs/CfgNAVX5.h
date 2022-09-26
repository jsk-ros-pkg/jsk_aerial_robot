#ifndef _ROS_ublox_msgs_CfgNAVX5_h
#define _ROS_ublox_msgs_CfgNAVX5_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgNAVX5 : public ros::Msg
  {
    public:
      typedef uint16_t _version_type;
      _version_type version;
      typedef uint16_t _mask1_type;
      _mask1_type mask1;
      typedef uint32_t _mask2_type;
      _mask2_type mask2;
      uint8_t reserved1[2];
      typedef uint8_t _minSVs_type;
      _minSVs_type minSVs;
      typedef uint8_t _maxSVs_type;
      _maxSVs_type maxSVs;
      typedef uint8_t _minCNO_type;
      _minCNO_type minCNO;
      typedef uint8_t _reserved2_type;
      _reserved2_type reserved2;
      typedef uint8_t _iniFix3D_type;
      _iniFix3D_type iniFix3D;
      uint8_t reserved3[2];
      typedef uint8_t _ackAiding_type;
      _ackAiding_type ackAiding;
      typedef uint16_t _wknRollover_type;
      _wknRollover_type wknRollover;
      typedef uint8_t _sigAttenCompMode_type;
      _sigAttenCompMode_type sigAttenCompMode;
      uint8_t reserved4[5];
      typedef uint8_t _usePPP_type;
      _usePPP_type usePPP;
      typedef uint8_t _aopCfg_type;
      _aopCfg_type aopCfg;
      uint8_t reserved5[2];
      typedef uint16_t _aopOrbMaxErr_type;
      _aopOrbMaxErr_type aopOrbMaxErr;
      uint8_t reserved6[7];
      typedef uint8_t _useAdr_type;
      _useAdr_type useAdr;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  35 };
      enum { MASK1_MIN_MAX =  4         };
      enum { MASK1_MIN_CNO =  8         };
      enum { MASK1_INITIAL_FIX_3D =  64        };
      enum { MASK1_WKN_ROLL =  512       };
      enum { MASK1_ACK_AID =  1024      };
      enum { MASK1_PPP =  8192      };
      enum { MASK1_AOP =  16384     };
      enum { MASK2_ADR =  64                     };
      enum { MASK2_SIG_ATTEN_COMP_MODE =  128    };

    CfgNAVX5():
      version(0),
      mask1(0),
      mask2(0),
      reserved1(),
      minSVs(0),
      maxSVs(0),
      minCNO(0),
      reserved2(0),
      iniFix3D(0),
      reserved3(),
      ackAiding(0),
      wknRollover(0),
      sigAttenCompMode(0),
      reserved4(),
      usePPP(0),
      aopCfg(0),
      reserved5(),
      aopOrbMaxErr(0),
      reserved6(),
      useAdr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->version >> (8 * 1)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->mask1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mask1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mask1);
      *(outbuffer + offset + 0) = (this->mask2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mask2 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mask2 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mask2 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mask2);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->minSVs >> (8 * 0)) & 0xFF;
      offset += sizeof(this->minSVs);
      *(outbuffer + offset + 0) = (this->maxSVs >> (8 * 0)) & 0xFF;
      offset += sizeof(this->maxSVs);
      *(outbuffer + offset + 0) = (this->minCNO >> (8 * 0)) & 0xFF;
      offset += sizeof(this->minCNO);
      *(outbuffer + offset + 0) = (this->reserved2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2);
      *(outbuffer + offset + 0) = (this->iniFix3D >> (8 * 0)) & 0xFF;
      offset += sizeof(this->iniFix3D);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved3[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved3[i]);
      }
      *(outbuffer + offset + 0) = (this->ackAiding >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ackAiding);
      *(outbuffer + offset + 0) = (this->wknRollover >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wknRollover >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wknRollover);
      *(outbuffer + offset + 0) = (this->sigAttenCompMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sigAttenCompMode);
      for( uint32_t i = 0; i < 5; i++){
      *(outbuffer + offset + 0) = (this->reserved4[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved4[i]);
      }
      *(outbuffer + offset + 0) = (this->usePPP >> (8 * 0)) & 0xFF;
      offset += sizeof(this->usePPP);
      *(outbuffer + offset + 0) = (this->aopCfg >> (8 * 0)) & 0xFF;
      offset += sizeof(this->aopCfg);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved5[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved5[i]);
      }
      *(outbuffer + offset + 0) = (this->aopOrbMaxErr >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->aopOrbMaxErr >> (8 * 1)) & 0xFF;
      offset += sizeof(this->aopOrbMaxErr);
      for( uint32_t i = 0; i < 7; i++){
      *(outbuffer + offset + 0) = (this->reserved6[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved6[i]);
      }
      *(outbuffer + offset + 0) = (this->useAdr >> (8 * 0)) & 0xFF;
      offset += sizeof(this->useAdr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->version =  ((uint16_t) (*(inbuffer + offset)));
      this->version |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->version);
      this->mask1 =  ((uint16_t) (*(inbuffer + offset)));
      this->mask1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->mask1);
      this->mask2 =  ((uint32_t) (*(inbuffer + offset)));
      this->mask2 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mask2 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mask2 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mask2);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->minSVs =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->minSVs);
      this->maxSVs =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->maxSVs);
      this->minCNO =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->minCNO);
      this->reserved2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2);
      this->iniFix3D =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->iniFix3D);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved3[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved3[i]);
      }
      this->ackAiding =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ackAiding);
      this->wknRollover =  ((uint16_t) (*(inbuffer + offset)));
      this->wknRollover |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->wknRollover);
      this->sigAttenCompMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sigAttenCompMode);
      for( uint32_t i = 0; i < 5; i++){
      this->reserved4[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved4[i]);
      }
      this->usePPP =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->usePPP);
      this->aopCfg =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->aopCfg);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved5[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved5[i]);
      }
      this->aopOrbMaxErr =  ((uint16_t) (*(inbuffer + offset)));
      this->aopOrbMaxErr |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->aopOrbMaxErr);
      for( uint32_t i = 0; i < 7; i++){
      this->reserved6[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved6[i]);
      }
      this->useAdr =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->useAdr);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgNAVX5"; };
    virtual const char * getMD5() override { return "10b967e4bf2a0c03e74705b79c79a211"; };

  };

}
#endif
