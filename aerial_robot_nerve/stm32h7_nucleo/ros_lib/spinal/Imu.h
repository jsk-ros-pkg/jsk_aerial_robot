#ifndef _ROS_spinal_Imu_h
#define _ROS_spinal_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace spinal
{

  class Imu : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      float acc_data[3];
      float gyro_data[3];
      float mag_data[3];
      float angles[3];

    Imu():
      stamp(),
      acc_data(),
      gyro_data(),
      mag_data(),
      angles()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_acc_datai;
      u_acc_datai.real = this->acc_data[i];
      *(outbuffer + offset + 0) = (u_acc_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_gyro_datai;
      u_gyro_datai.real = this->gyro_data[i];
      *(outbuffer + offset + 0) = (u_gyro_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_mag_datai;
      u_mag_datai.real = this->mag_data[i];
      *(outbuffer + offset + 0) = (u_mag_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mag_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mag_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mag_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mag_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_acc_datai;
      u_acc_datai.base = 0;
      u_acc_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_data[i] = u_acc_datai.real;
      offset += sizeof(this->acc_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_gyro_datai;
      u_gyro_datai.base = 0;
      u_gyro_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_data[i] = u_gyro_datai.real;
      offset += sizeof(this->gyro_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_mag_datai;
      u_mag_datai.base = 0;
      u_mag_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mag_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mag_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mag_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mag_data[i] = u_mag_datai.real;
      offset += sizeof(this->mag_data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.base = 0;
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angles[i] = u_anglesi.real;
      offset += sizeof(this->angles[i]);
      }
     return offset;
    }

    const char * getType(){ return "spinal/Imu"; };
    const char * getMD5(){ return "68b769ca85e5d9c44dbce51dc60e4be7"; };

  };

}
#endif
