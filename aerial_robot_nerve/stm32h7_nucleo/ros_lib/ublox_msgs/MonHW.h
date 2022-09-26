#ifndef _ROS_ublox_msgs_MonHW_h
#define _ROS_ublox_msgs_MonHW_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class MonHW : public ros::Msg
  {
    public:
      typedef uint32_t _pinSel_type;
      _pinSel_type pinSel;
      typedef uint32_t _pinBank_type;
      _pinBank_type pinBank;
      typedef uint32_t _pinDir_type;
      _pinDir_type pinDir;
      typedef uint32_t _pinVal_type;
      _pinVal_type pinVal;
      typedef uint16_t _noisePerMS_type;
      _noisePerMS_type noisePerMS;
      typedef uint16_t _agcCnt_type;
      _agcCnt_type agcCnt;
      typedef uint8_t _aStatus_type;
      _aStatus_type aStatus;
      typedef uint8_t _aPower_type;
      _aPower_type aPower;
      typedef uint8_t _flags_type;
      _flags_type flags;
      typedef uint8_t _reserved0_type;
      _reserved0_type reserved0;
      typedef uint32_t _usedMask_type;
      _usedMask_type usedMask;
      uint8_t VP[17];
      typedef uint8_t _jamInd_type;
      _jamInd_type jamInd;
      uint8_t reserved1[2];
      typedef uint32_t _pinIrq_type;
      _pinIrq_type pinIrq;
      typedef uint32_t _pullH_type;
      _pullH_type pullH;
      typedef uint32_t _pullL_type;
      _pullL_type pullL;
      enum { CLASS_ID =  10 };
      enum { MESSAGE_ID =  9 };
      enum { A_STATUS_INIT =  0 };
      enum { A_STATUS_UNKNOWN =  1 };
      enum { A_STATUS_OK =  2 };
      enum { A_STATUS_SHORT =  3 };
      enum { A_STATUS_OPEN =  4 };
      enum { A_POWER_OFF =  0 };
      enum { A_POWER_ON =  1 };
      enum { A_POWER_UNKNOWN =  2 };
      enum { FLAGS_RTC_CALIB =  1             };
      enum { FLAGS_SAFE_BOOT =   2            };
      enum { FLAGS_JAMMING_STATE_MASK =  12   };
      enum { JAMMING_STATE_UNKNOWN_OR_DISABLED =  0    };
      enum { JAMMING_STATE_OK =  4                     };
      enum { JAMMING_STATE_WARNING =  8                };
      enum { JAMMING_STATE_CRITICAL =  12              };
      enum { FLAGS_XTAL_ABSENT =   16         };
      enum { JAM_IND_NONE =  0           };
      enum { JAM_IND_STRONG =  255       };

    MonHW():
      pinSel(0),
      pinBank(0),
      pinDir(0),
      pinVal(0),
      noisePerMS(0),
      agcCnt(0),
      aStatus(0),
      aPower(0),
      flags(0),
      reserved0(0),
      usedMask(0),
      VP(),
      jamInd(0),
      reserved1(),
      pinIrq(0),
      pullH(0),
      pullL(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pinSel >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pinSel >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pinSel >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pinSel >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pinSel);
      *(outbuffer + offset + 0) = (this->pinBank >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pinBank >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pinBank >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pinBank >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pinBank);
      *(outbuffer + offset + 0) = (this->pinDir >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pinDir >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pinDir >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pinDir >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pinDir);
      *(outbuffer + offset + 0) = (this->pinVal >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pinVal >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pinVal >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pinVal >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pinVal);
      *(outbuffer + offset + 0) = (this->noisePerMS >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->noisePerMS >> (8 * 1)) & 0xFF;
      offset += sizeof(this->noisePerMS);
      *(outbuffer + offset + 0) = (this->agcCnt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->agcCnt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->agcCnt);
      *(outbuffer + offset + 0) = (this->aStatus >> (8 * 0)) & 0xFF;
      offset += sizeof(this->aStatus);
      *(outbuffer + offset + 0) = (this->aPower >> (8 * 0)) & 0xFF;
      offset += sizeof(this->aPower);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->reserved0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0);
      *(outbuffer + offset + 0) = (this->usedMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->usedMask >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->usedMask >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->usedMask >> (8 * 3)) & 0xFF;
      offset += sizeof(this->usedMask);
      for( uint32_t i = 0; i < 17; i++){
      *(outbuffer + offset + 0) = (this->VP[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->VP[i]);
      }
      *(outbuffer + offset + 0) = (this->jamInd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->jamInd);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->pinIrq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pinIrq >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pinIrq >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pinIrq >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pinIrq);
      *(outbuffer + offset + 0) = (this->pullH >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pullH >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pullH >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pullH >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pullH);
      *(outbuffer + offset + 0) = (this->pullL >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pullL >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pullL >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pullL >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pullL);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->pinSel =  ((uint32_t) (*(inbuffer + offset)));
      this->pinSel |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pinSel |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pinSel |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pinSel);
      this->pinBank =  ((uint32_t) (*(inbuffer + offset)));
      this->pinBank |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pinBank |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pinBank |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pinBank);
      this->pinDir =  ((uint32_t) (*(inbuffer + offset)));
      this->pinDir |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pinDir |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pinDir |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pinDir);
      this->pinVal =  ((uint32_t) (*(inbuffer + offset)));
      this->pinVal |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pinVal |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pinVal |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pinVal);
      this->noisePerMS =  ((uint16_t) (*(inbuffer + offset)));
      this->noisePerMS |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->noisePerMS);
      this->agcCnt =  ((uint16_t) (*(inbuffer + offset)));
      this->agcCnt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->agcCnt);
      this->aStatus =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->aStatus);
      this->aPower =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->aPower);
      this->flags =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->flags);
      this->reserved0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0);
      this->usedMask =  ((uint32_t) (*(inbuffer + offset)));
      this->usedMask |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->usedMask |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->usedMask |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->usedMask);
      for( uint32_t i = 0; i < 17; i++){
      this->VP[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->VP[i]);
      }
      this->jamInd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->jamInd);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->pinIrq =  ((uint32_t) (*(inbuffer + offset)));
      this->pinIrq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pinIrq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pinIrq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pinIrq);
      this->pullH =  ((uint32_t) (*(inbuffer + offset)));
      this->pullH |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pullH |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pullH |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pullH);
      this->pullL =  ((uint32_t) (*(inbuffer + offset)));
      this->pullL |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->pullL |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->pullL |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->pullL);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/MonHW"; };
    virtual const char * getMD5() override { return "605e9f0118e26136185358e2b10a0913"; };

  };

}
#endif
