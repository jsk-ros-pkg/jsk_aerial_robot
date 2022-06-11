#ifndef _ROS_pr2_msgs_PowerBoardState_h
#define _ROS_pr2_msgs_PowerBoardState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pr2_msgs
{

  class PowerBoardState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef uint32_t _serial_num_type;
      _serial_num_type serial_num;
      typedef double _input_voltage_type;
      _input_voltage_type input_voltage;
      typedef int8_t _master_state_type;
      _master_state_type master_state;
      int8_t circuit_state[3];
      double circuit_voltage[3];
      typedef bool _run_stop_type;
      _run_stop_type run_stop;
      typedef bool _wireless_stop_type;
      _wireless_stop_type wireless_stop;
      enum { STATE_NOPOWER = 0 };
      enum { STATE_STANDBY = 1 };
      enum { STATE_PUMPING = 2 };
      enum { STATE_ON = 3 };
      enum { STATE_ENABLED = 3   };
      enum { STATE_DISABLED = 4 };
      enum { MASTER_NOPOWER = 0 };
      enum { MASTER_STANDBY = 1 };
      enum { MASTER_ON = 2 };
      enum { MASTER_OFF = 3 };
      enum { MASTER_SHUTDOWN = 4 };

    PowerBoardState():
      header(),
      name(""),
      serial_num(0),
      input_voltage(0),
      master_state(0),
      circuit_state(),
      circuit_voltage(),
      run_stop(0),
      wireless_stop(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->serial_num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->serial_num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->serial_num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->serial_num >> (8 * 3)) & 0xFF;
      offset += sizeof(this->serial_num);
      union {
        double real;
        uint64_t base;
      } u_input_voltage;
      u_input_voltage.real = this->input_voltage;
      *(outbuffer + offset + 0) = (u_input_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_voltage.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_input_voltage.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_input_voltage.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_input_voltage.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_input_voltage.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->input_voltage);
      union {
        int8_t real;
        uint8_t base;
      } u_master_state;
      u_master_state.real = this->master_state;
      *(outbuffer + offset + 0) = (u_master_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->master_state);
      for( uint32_t i = 0; i < 3; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_circuit_statei;
      u_circuit_statei.real = this->circuit_state[i];
      *(outbuffer + offset + 0) = (u_circuit_statei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->circuit_state[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_circuit_voltagei;
      u_circuit_voltagei.real = this->circuit_voltage[i];
      *(outbuffer + offset + 0) = (u_circuit_voltagei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_circuit_voltagei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_circuit_voltagei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_circuit_voltagei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_circuit_voltagei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_circuit_voltagei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_circuit_voltagei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_circuit_voltagei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->circuit_voltage[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_run_stop;
      u_run_stop.real = this->run_stop;
      *(outbuffer + offset + 0) = (u_run_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->run_stop);
      union {
        bool real;
        uint8_t base;
      } u_wireless_stop;
      u_wireless_stop.real = this->wireless_stop;
      *(outbuffer + offset + 0) = (u_wireless_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wireless_stop);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->serial_num =  ((uint32_t) (*(inbuffer + offset)));
      this->serial_num |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->serial_num |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->serial_num |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->serial_num);
      union {
        double real;
        uint64_t base;
      } u_input_voltage;
      u_input_voltage.base = 0;
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_input_voltage.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->input_voltage = u_input_voltage.real;
      offset += sizeof(this->input_voltage);
      union {
        int8_t real;
        uint8_t base;
      } u_master_state;
      u_master_state.base = 0;
      u_master_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->master_state = u_master_state.real;
      offset += sizeof(this->master_state);
      for( uint32_t i = 0; i < 3; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_circuit_statei;
      u_circuit_statei.base = 0;
      u_circuit_statei.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->circuit_state[i] = u_circuit_statei.real;
      offset += sizeof(this->circuit_state[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_circuit_voltagei;
      u_circuit_voltagei.base = 0;
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_circuit_voltagei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->circuit_voltage[i] = u_circuit_voltagei.real;
      offset += sizeof(this->circuit_voltage[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_run_stop;
      u_run_stop.base = 0;
      u_run_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->run_stop = u_run_stop.real;
      offset += sizeof(this->run_stop);
      union {
        bool real;
        uint8_t base;
      } u_wireless_stop;
      u_wireless_stop.base = 0;
      u_wireless_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wireless_stop = u_wireless_stop.real;
      offset += sizeof(this->wireless_stop);
     return offset;
    }

    const char * getType(){ return "pr2_msgs/PowerBoardState"; };
    const char * getMD5(){ return "08899b671e6a1a449e7ce0000da8ae7b"; };

  };

}
#endif
