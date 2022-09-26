#ifndef _ROS_ublox_msgs_EsfSTATUS_Sens_h
#define _ROS_ublox_msgs_EsfSTATUS_Sens_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class EsfSTATUS_Sens : public ros::Msg
  {
    public:
      typedef uint8_t _sensStatus1_type;
      _sensStatus1_type sensStatus1;
      typedef uint8_t _sensStatus2_type;
      _sensStatus2_type sensStatus2;
      typedef uint8_t _freq_type;
      _freq_type freq;
      typedef uint8_t _faults_type;
      _faults_type faults;

    EsfSTATUS_Sens():
      sensStatus1(0),
      sensStatus2(0),
      freq(0),
      faults(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sensStatus1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sensStatus1);
      *(outbuffer + offset + 0) = (this->sensStatus2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sensStatus2);
      *(outbuffer + offset + 0) = (this->freq >> (8 * 0)) & 0xFF;
      offset += sizeof(this->freq);
      *(outbuffer + offset + 0) = (this->faults >> (8 * 0)) & 0xFF;
      offset += sizeof(this->faults);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->sensStatus1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sensStatus1);
      this->sensStatus2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sensStatus2);
      this->freq =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->freq);
      this->faults =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->faults);
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfSTATUS_Sens"; };
    virtual const char * getMD5() override { return "642a0b5f53044e3a4dd28074dc540ef3"; };

  };

}
#endif
