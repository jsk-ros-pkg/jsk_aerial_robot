#ifndef _ROS_ublox_msgs_CfgPRT_h
#define _ROS_ublox_msgs_CfgPRT_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgPRT : public ros::Msg
  {
    public:
      typedef uint8_t _portID_type;
      _portID_type portID;
      typedef uint8_t _reserved0_type;
      _reserved0_type reserved0;
      typedef uint16_t _txReady_type;
      _txReady_type txReady;
      typedef uint32_t _mode_type;
      _mode_type mode;
      typedef uint32_t _baudRate_type;
      _baudRate_type baudRate;
      typedef uint16_t _inProtoMask_type;
      _inProtoMask_type inProtoMask;
      typedef uint16_t _outProtoMask_type;
      _outProtoMask_type outProtoMask;
      typedef uint16_t _flags_type;
      _flags_type flags;
      typedef uint16_t _reserved1_type;
      _reserved1_type reserved1;
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  0 };
      enum { PORT_ID_DDC =  0 };
      enum { PORT_ID_UART1 =  1 };
      enum { PORT_ID_UART2 =  2 };
      enum { PORT_ID_USB =  3 };
      enum { PORT_ID_SPI =  4 };
      enum { TX_READY_EN =  1                     };
      enum { TX_READY_POLARITY_HIGH_ACTIVE =  0   };
      enum { TX_READY_POLARITY_LOW_ACTIVE =  2    };
      enum { TX_READY_PIN_SHIFT =  2              };
      enum { TX_READY_PIN_MASK =  124             };
      enum { TX_READY_THRES_SHIFT =  7            };
      enum { TX_READY_THRES_MASK =  65408         };
      enum { MODE_DDC_SLAVE_ADDR_SHIFT =  1 };
      enum { MODE_DDC_SLAVE_ADDR_MASK =  254      };
      enum { MODE_RESERVED1 =  16                 };
      enum { MODE_CHAR_LEN_MASK =  192            };
      enum { MODE_CHAR_LEN_5BIT =  0                };
      enum { MODE_CHAR_LEN_6BIT =  64               };
      enum { MODE_CHAR_LEN_7BIT =  128              };
      enum { MODE_CHAR_LEN_8BIT =  192              };
      enum { MODE_PARITY_MASK =  3584             };
      enum { MODE_PARITY_EVEN =  0                  };
      enum { MODE_PARITY_ODD =  512                 };
      enum { MODE_PARITY_NO =  2048                 };
      enum { MODE_STOP_BITS_MASK =  12288         };
      enum { MODE_STOP_BITS_1 =  0                  };
      enum { MODE_STOP_BITS_15 =  4096              };
      enum { MODE_STOP_BITS_2 =  8192               };
      enum { MODE_STOP_BITS_05 =  12288             };
      enum { MODE_SPI_SPI_MODE_CPOL =  4          };
      enum { MODE_SPI_SPI_MODE_CPHA =  2          };
      enum { MODE_SPI_FLOW_CONTROL =  64          };
      enum { MODE_SPI_FF_COUNT_SHIFT =  8 };
      enum { MODE_SPI_FF_COUNT_MASK =  16128      };
      enum { PROTO_UBX =  1 };
      enum { PROTO_NMEA =  2 };
      enum { PROTO_RTCM =  4      };
      enum { PROTO_RTCM3 =  32    };
      enum { FLAGS_EXTENDED_TX_TIMEOUT =  2   };

    CfgPRT():
      portID(0),
      reserved0(0),
      txReady(0),
      mode(0),
      baudRate(0),
      inProtoMask(0),
      outProtoMask(0),
      flags(0),
      reserved1(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->portID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->portID);
      *(outbuffer + offset + 0) = (this->reserved0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0);
      *(outbuffer + offset + 0) = (this->txReady >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->txReady >> (8 * 1)) & 0xFF;
      offset += sizeof(this->txReady);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mode >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mode >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mode >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->baudRate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->baudRate >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->baudRate >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->baudRate >> (8 * 3)) & 0xFF;
      offset += sizeof(this->baudRate);
      *(outbuffer + offset + 0) = (this->inProtoMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->inProtoMask >> (8 * 1)) & 0xFF;
      offset += sizeof(this->inProtoMask);
      *(outbuffer + offset + 0) = (this->outProtoMask >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->outProtoMask >> (8 * 1)) & 0xFF;
      offset += sizeof(this->outProtoMask);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->reserved1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->reserved1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->reserved1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->portID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->portID);
      this->reserved0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0);
      this->txReady =  ((uint16_t) (*(inbuffer + offset)));
      this->txReady |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->txReady);
      this->mode =  ((uint32_t) (*(inbuffer + offset)));
      this->mode |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mode |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mode);
      this->baudRate =  ((uint32_t) (*(inbuffer + offset)));
      this->baudRate |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->baudRate |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->baudRate |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->baudRate);
      this->inProtoMask =  ((uint16_t) (*(inbuffer + offset)));
      this->inProtoMask |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->inProtoMask);
      this->outProtoMask =  ((uint16_t) (*(inbuffer + offset)));
      this->outProtoMask |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->outProtoMask);
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      this->reserved1 =  ((uint16_t) (*(inbuffer + offset)));
      this->reserved1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->reserved1);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgPRT"; };
    virtual const char * getMD5() override { return "a4f010a61091571f886628b406ed3edb"; };

  };

}
#endif
