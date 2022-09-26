#ifndef _ROS_ublox_msgs_EsfMEAS_h
#define _ROS_ublox_msgs_EsfMEAS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class EsfMEAS : public ros::Msg
  {
    public:
      typedef uint32_t _timeTag_type;
      _timeTag_type timeTag;
      typedef uint16_t _flags_type;
      _flags_type flags;
      typedef uint16_t _id_type;
      _id_type id;
      uint32_t data_length;
      typedef uint32_t _data_type;
      _data_type st_data;
      _data_type * data;
      uint32_t calibTtag_length;
      typedef uint32_t _calibTtag_type;
      _calibTtag_type st_calibTtag;
      _calibTtag_type * calibTtag;
      enum { CLASS_ID =  16 };
      enum { MESSAGE_ID =  2 };
      enum { FLAGS_TIME_MARK_SENT_MASK =  3    };
      enum { TIME_MARK_NONE =  0 };
      enum { TIME_MARK_EXT0 =  1 };
      enum { TIME_MARK_EXT =  2 };
      enum { FLAGS_TIME_MARK_EDGE =  4         };
      enum { FLAGS_CALIB_T_TAG_VALID =  8      };
      enum { DATA_FIELD_MASK =  16777215      };
      enum { DATA_TYPE_MASK =  1056964608     };
      enum { DATA_TYPE_SHIFT =  24 };
      enum { DATA_TYPE_NONE =  0                      };
      enum { DATA_TYPE_Z_AXIS_GYRO =  5               };
      enum { DATA_TYPE_WHEEL_TICKS_FRONT_LEFT =  6    };
      enum { DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT =  7   };
      enum { DATA_TYPE_WHEEL_TICKS_REAR_LEFT =  8     };
      enum { DATA_TYPE_WHEEL_TICKS_REAR_RIGHT =  9    };
      enum { DATA_TYPE_SINGLE_TICK =  10              };
      enum { DATA_TYPE_SPEED =  11                    };
      enum { DATA_TYPE_GYRO_TEMPERATURE =  12         };
      enum { DATA_TYPE_GYRO_ANG_RATE_Y =  13          };
      enum { DATA_TYPE_GYRO_ANG_RATE_X =  14          };
      enum { DATA_TYPE_ACCELEROMETER_X =  16          };
      enum { DATA_TYPE_ACCELEROMETER_Y =  17          };
      enum { DATA_TYPE_ACCELEROMETER_Z =  18          };

    EsfMEAS():
      timeTag(0),
      flags(0),
      id(0),
      data_length(0), st_data(), data(nullptr),
      calibTtag_length(0), st_calibTtag(), calibTtag(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timeTag >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeTag >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeTag >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeTag >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeTag);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      *(outbuffer + offset + 0) = (this->calibTtag_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->calibTtag_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->calibTtag_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->calibTtag_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->calibTtag_length);
      for( uint32_t i = 0; i < calibTtag_length; i++){
      *(outbuffer + offset + 0) = (this->calibTtag[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->calibTtag[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->calibTtag[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->calibTtag[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->calibTtag[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->timeTag =  ((uint32_t) (*(inbuffer + offset)));
      this->timeTag |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeTag |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeTag |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeTag);
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      this->id =  ((uint16_t) (*(inbuffer + offset)));
      this->id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->id);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint32_t*)realloc(this->data, data_lengthT * sizeof(uint32_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint32_t) (*(inbuffer + offset)));
      this->st_data |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_data |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_data |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint32_t));
      }
      uint32_t calibTtag_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      calibTtag_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      calibTtag_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      calibTtag_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->calibTtag_length);
      if(calibTtag_lengthT > calibTtag_length)
        this->calibTtag = (uint32_t*)realloc(this->calibTtag, calibTtag_lengthT * sizeof(uint32_t));
      calibTtag_length = calibTtag_lengthT;
      for( uint32_t i = 0; i < calibTtag_length; i++){
      this->st_calibTtag =  ((uint32_t) (*(inbuffer + offset)));
      this->st_calibTtag |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_calibTtag |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_calibTtag |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_calibTtag);
        memcpy( &(this->calibTtag[i]), &(this->st_calibTtag), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/EsfMEAS"; };
    virtual const char * getMD5() override { return "2ee2c25c5689cb0a12cc22f118ece178"; };

  };

}
#endif
