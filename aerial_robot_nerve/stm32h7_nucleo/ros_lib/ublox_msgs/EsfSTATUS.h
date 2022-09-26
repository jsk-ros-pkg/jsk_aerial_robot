#ifndef _ROS_ublox_msgs_EsfSTATUS_h
#define _ROS_ublox_msgs_EsfSTATUS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ublox_msgs/EsfSTATUS_Sens.h"

namespace ublox_msgs
{

  class EsfSTATUS : public ros::Msg
  {
    public:
      typedef uint32_t _iTOW_type;
      _iTOW_type iTOW;
      typedef uint8_t _version_type;
      _version_type version;
      uint8_t reserved1[7];
      typedef uint8_t _fusionMode_type;
      _fusionMode_type fusionMode;
      uint8_t reserved2[2];
      typedef uint8_t _numSens_type;
      _numSens_type numSens;
      uint32_t sens_length;
      typedef ublox_msgs::EsfSTATUS_Sens _sens_type;
      _sens_type st_sens;
      _sens_type * sens;
      enum { CLASS_ID =  16 };
      enum { MESSAGE_ID =  16 };
      enum { FUSION_MODE_INIT =  0         };
      enum { FUSION_MODE_FUSION =  1       };
      enum { FUSION_MODE_SUSPENDED =  2    };
      enum { FUSION_MODE_DISABLED =  3     };

    EsfSTATUS():
      iTOW(0),
      version(0),
      reserved1(),
      fusionMode(0),
      reserved2(),
      numSens(0),
      sens_length(0), st_sens(), sens(nullptr)
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
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 7; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->fusionMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fusionMode);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved2[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2[i]);
      }
      *(outbuffer + offset + 0) = (this->numSens >> (8 * 0)) & 0xFF;
      offset += sizeof(this->numSens);
      *(outbuffer + offset + 0) = (this->sens_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sens_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sens_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sens_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sens_length);
      for( uint32_t i = 0; i < sens_length; i++){
      offset += this->sens[i].serialize(outbuffer + offset);
      }
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
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      for( uint32_t i = 0; i < 7; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->fusionMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fusionMode);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved2[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2[i]);
      }
      this->numSens =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->numSens);
      uint32_t sens_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sens_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sens_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sens_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sens_length);
      if(sens_lengthT > sens_length)
        this->sens = (ublox_msgs::EsfSTATUS_Sens*)realloc(this->sens, sens_lengthT * sizeof(ublox_msgs::EsfSTATUS_Sens));
      sens_length = sens_lengthT;
      for( uint32_t i = 0; i < sens_length; i++){
      offset += this->st_sens.deserialize(inbuffer + offset);
        memcpy( &(this->sens[i]), &(this->st_sens), sizeof(ublox_msgs::EsfSTATUS_Sens));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfSTATUS"; };
    virtual const char * getMD5() override { return "006f2c0e3e6e9239781223dca67e519b"; };

  };

}
#endif
