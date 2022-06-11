#ifndef _ROS_spinal_BoardInfo_h
#define _ROS_spinal_BoardInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/ServoInfo.h"

namespace spinal
{

  class BoardInfo : public ros::Msg
  {
    public:
      typedef uint8_t _slave_id_type;
      _slave_id_type slave_id;
      typedef uint8_t _imu_send_data_flag_type;
      _imu_send_data_flag_type imu_send_data_flag;
      typedef uint8_t _dynamixel_ttl_rs485_mixed_type;
      _dynamixel_ttl_rs485_mixed_type dynamixel_ttl_rs485_mixed;
      uint32_t servos_length;
      typedef spinal::ServoInfo _servos_type;
      _servos_type st_servos;
      _servos_type * servos;

    BoardInfo():
      slave_id(0),
      imu_send_data_flag(0),
      dynamixel_ttl_rs485_mixed(0),
      servos_length(0), servos(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->slave_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->slave_id);
      *(outbuffer + offset + 0) = (this->imu_send_data_flag >> (8 * 0)) & 0xFF;
      offset += sizeof(this->imu_send_data_flag);
      *(outbuffer + offset + 0) = (this->dynamixel_ttl_rs485_mixed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dynamixel_ttl_rs485_mixed);
      *(outbuffer + offset + 0) = (this->servos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->servos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->servos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->servos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servos_length);
      for( uint32_t i = 0; i < servos_length; i++){
      offset += this->servos[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->slave_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->slave_id);
      this->imu_send_data_flag =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->imu_send_data_flag);
      this->dynamixel_ttl_rs485_mixed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dynamixel_ttl_rs485_mixed);
      uint32_t servos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      servos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->servos_length);
      if(servos_lengthT > servos_length)
        this->servos = (spinal::ServoInfo*)realloc(this->servos, servos_lengthT * sizeof(spinal::ServoInfo));
      servos_length = servos_lengthT;
      for( uint32_t i = 0; i < servos_length; i++){
      offset += this->st_servos.deserialize(inbuffer + offset);
        memcpy( &(this->servos[i]), &(this->st_servos), sizeof(spinal::ServoInfo));
      }
     return offset;
    }

    const char * getType(){ return "spinal/BoardInfo"; };
    const char * getMD5(){ return "96d77913b56f07c2f03bd01269f08893"; };

  };

}
#endif
