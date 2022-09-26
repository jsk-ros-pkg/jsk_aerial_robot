#ifndef _ROS_ublox_msgs_CfgUSB_h
#define _ROS_ublox_msgs_CfgUSB_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgUSB : public ros::Msg
  {
    public:
      typedef uint16_t _vendorID_type;
      _vendorID_type vendorID;
      typedef uint16_t _productID_type;
      _productID_type productID;
      uint8_t reserved1[2];
      uint8_t reserved2[2];
      typedef uint16_t _powerConsumption_type;
      _powerConsumption_type powerConsumption;
      typedef uint16_t _flags_type;
      _flags_type flags;
      int8_t vendorString[32];
      int8_t productString[32];
      int8_t serialNumber[32];
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  27 };
      enum { FLAGS_RE_ENUM =  0        };
      enum { FLAGS_POWER_MODE =  2     };

    CfgUSB():
      vendorID(0),
      productID(0),
      reserved1(),
      reserved2(),
      powerConsumption(0),
      flags(0),
      vendorString(),
      productString(),
      serialNumber()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->vendorID >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vendorID >> (8 * 1)) & 0xFF;
      offset += sizeof(this->vendorID);
      *(outbuffer + offset + 0) = (this->productID >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->productID >> (8 * 1)) & 0xFF;
      offset += sizeof(this->productID);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved2[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2[i]);
      }
      *(outbuffer + offset + 0) = (this->powerConsumption >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->powerConsumption >> (8 * 1)) & 0xFF;
      offset += sizeof(this->powerConsumption);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_vendorStringi;
      u_vendorStringi.real = this->vendorString[i];
      *(outbuffer + offset + 0) = (u_vendorStringi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->vendorString[i]);
      }
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_productStringi;
      u_productStringi.real = this->productString[i];
      *(outbuffer + offset + 0) = (u_productStringi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->productString[i]);
      }
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_serialNumberi;
      u_serialNumberi.real = this->serialNumber[i];
      *(outbuffer + offset + 0) = (u_serialNumberi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->serialNumber[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->vendorID =  ((uint16_t) (*(inbuffer + offset)));
      this->vendorID |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->vendorID);
      this->productID =  ((uint16_t) (*(inbuffer + offset)));
      this->productID |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->productID);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      for( uint32_t i = 0; i < 2; i++){
      this->reserved2[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2[i]);
      }
      this->powerConsumption =  ((uint16_t) (*(inbuffer + offset)));
      this->powerConsumption |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->powerConsumption);
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_vendorStringi;
      u_vendorStringi.base = 0;
      u_vendorStringi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->vendorString[i] = u_vendorStringi.real;
      offset += sizeof(this->vendorString[i]);
      }
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_productStringi;
      u_productStringi.base = 0;
      u_productStringi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->productString[i] = u_productStringi.real;
      offset += sizeof(this->productString[i]);
      }
      for( uint32_t i = 0; i < 32; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_serialNumberi;
      u_serialNumberi.base = 0;
      u_serialNumberi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->serialNumber[i] = u_serialNumberi.real;
      offset += sizeof(this->serialNumber[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/CfgUSB"; };
    virtual const char * getMD5() override { return "d1797a4ed330d6193bc42a443c001b03"; };

  };

}
#endif
