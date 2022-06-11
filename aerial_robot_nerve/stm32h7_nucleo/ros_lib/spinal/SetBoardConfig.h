#ifndef _ROS_SERVICE_SetBoardConfig_h
#define _ROS_SERVICE_SetBoardConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spinal
{

static const char SETBOARDCONFIG[] = "spinal/SetBoardConfig";

  class SetBoardConfigRequest : public ros::Msg
  {
    public:
      typedef uint8_t _command_type;
      _command_type command;
      uint32_t data_length;
      typedef int32_t _data_type;
      _data_type st_data;
      _data_type * data;
      enum { SET_SLAVE_ID =  0 };
      enum { SET_IMU_SEND_FLAG =  1 };
      enum { SET_SERVO_HOMING_OFFSET =  2 };
      enum { SET_SERVO_PID_GAIN =  3 };
      enum { SET_SERVO_PROFILE_VEL =  4 };
      enum { SET_SERVO_SEND_DATA_FLAG =  5 };
      enum { SET_SERVO_CURRENT_LIMIT =  6 };
      enum { REBOOT =  7 };
      enum { SET_DYNAMIXEL_TTL_RS485_MIXED =  8 };
      enum { SET_SERVO_EXTERNAL_ENCODER_FLAG =  9 };
      enum { SET_SERVO_RESOLUTION_RATIO =  10 };

    SetBoardConfigRequest():
      command(0),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->command);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (int32_t*)realloc(this->data, data_lengthT * sizeof(int32_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_data;
      u_st_data.base = 0;
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_data.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_data = u_st_data.real;
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return SETBOARDCONFIG; };
    const char * getMD5(){ return "a479adad61b9876d2f6bebef8341cf8b"; };

  };

  class SetBoardConfigResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetBoardConfigResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETBOARDCONFIG; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetBoardConfig {
    public:
    typedef SetBoardConfigRequest Request;
    typedef SetBoardConfigResponse Response;
  };

}
#endif
