#ifndef _ROS_jsk_network_tools_AllTypeTest_h
#define _ROS_jsk_network_tools_AllTypeTest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_network_tools
{

  class AllTypeTest : public ros::Msg
  {
    public:
      typedef bool _bool_atom_type;
      _bool_atom_type bool_atom;
      bool bool_array[4];
      typedef uint8_t _uint8_atom_type;
      _uint8_atom_type uint8_atom;
      uint8_t uint8_array[4];
      typedef int8_t _int8_atom_type;
      _int8_atom_type int8_atom;
      int8_t int8_array[4];
      typedef uint16_t _uint16_atom_type;
      _uint16_atom_type uint16_atom;
      uint16_t uint16_array[4];
      typedef int32_t _int32_atom_type;
      _int32_atom_type int32_atom;
      int32_t int32_array[4];
      typedef uint32_t _uint32_atom_type;
      _uint32_atom_type uint32_atom;
      uint32_t uint32_array[4];
      typedef int64_t _int64_atom_type;
      _int64_atom_type int64_atom;
      int64_t int64_array[4];
      typedef uint64_t _uint64_atom_type;
      _uint64_atom_type uint64_atom;
      uint64_t uint64_array[4];
      typedef float _float32_atom_type;
      _float32_atom_type float32_atom;
      float float32_array[4];
      typedef double _float64_atom_type;
      _float64_atom_type float64_atom;
      double float64_array[4];

    AllTypeTest():
      bool_atom(0),
      bool_array(),
      uint8_atom(0),
      uint8_array(),
      int8_atom(0),
      int8_array(),
      uint16_atom(0),
      uint16_array(),
      int32_atom(0),
      int32_array(),
      uint32_atom(0),
      uint32_array(),
      int64_atom(0),
      int64_array(),
      uint64_atom(0),
      uint64_array(),
      float32_atom(0),
      float32_array(),
      float64_atom(0),
      float64_array()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_bool_atom;
      u_bool_atom.real = this->bool_atom;
      *(outbuffer + offset + 0) = (u_bool_atom.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bool_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_bool_arrayi;
      u_bool_arrayi.real = this->bool_array[i];
      *(outbuffer + offset + 0) = (u_bool_arrayi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bool_array[i]);
      }
      *(outbuffer + offset + 0) = (this->uint8_atom >> (8 * 0)) & 0xFF;
      offset += sizeof(this->uint8_atom);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->uint8_array[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->uint8_array[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_int8_atom;
      u_int8_atom.real = this->int8_atom;
      *(outbuffer + offset + 0) = (u_int8_atom.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->int8_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_int8_arrayi;
      u_int8_arrayi.real = this->int8_array[i];
      *(outbuffer + offset + 0) = (u_int8_arrayi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->int8_array[i]);
      }
      *(outbuffer + offset + 0) = (this->uint16_atom >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint16_atom >> (8 * 1)) & 0xFF;
      offset += sizeof(this->uint16_atom);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->uint16_array[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint16_array[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->uint16_array[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_int32_atom;
      u_int32_atom.real = this->int32_atom;
      *(outbuffer + offset + 0) = (u_int32_atom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_int32_atom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_int32_atom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_int32_atom.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->int32_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_int32_arrayi;
      u_int32_arrayi.real = this->int32_array[i];
      *(outbuffer + offset + 0) = (u_int32_arrayi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_int32_arrayi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_int32_arrayi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_int32_arrayi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->int32_array[i]);
      }
      *(outbuffer + offset + 0) = (this->uint32_atom >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint32_atom >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uint32_atom >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uint32_atom >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uint32_atom);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->uint32_array[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint32_array[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uint32_array[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uint32_array[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uint32_array[i]);
      }
      union {
        int64_t real;
        uint64_t base;
      } u_int64_atom;
      u_int64_atom.real = this->int64_atom;
      *(outbuffer + offset + 0) = (u_int64_atom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_int64_atom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_int64_atom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_int64_atom.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_int64_atom.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_int64_atom.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_int64_atom.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_int64_atom.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->int64_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_int64_arrayi;
      u_int64_arrayi.real = this->int64_array[i];
      *(outbuffer + offset + 0) = (u_int64_arrayi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_int64_arrayi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_int64_arrayi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_int64_arrayi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_int64_arrayi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_int64_arrayi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_int64_arrayi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_int64_arrayi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->int64_array[i]);
      }
      *(outbuffer + offset + 0) = (this->uint64_atom >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint64_atom >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uint64_atom >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uint64_atom >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->uint64_atom >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->uint64_atom >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->uint64_atom >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->uint64_atom >> (8 * 7)) & 0xFF;
      offset += sizeof(this->uint64_atom);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->uint64_array[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uint64_array[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uint64_array[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uint64_array[i] >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->uint64_array[i] >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->uint64_array[i] >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->uint64_array[i] >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->uint64_array[i] >> (8 * 7)) & 0xFF;
      offset += sizeof(this->uint64_array[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_float32_atom;
      u_float32_atom.real = this->float32_atom;
      *(outbuffer + offset + 0) = (u_float32_atom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_float32_atom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_float32_atom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_float32_atom.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->float32_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_float32_arrayi;
      u_float32_arrayi.real = this->float32_array[i];
      *(outbuffer + offset + 0) = (u_float32_arrayi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_float32_arrayi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_float32_arrayi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_float32_arrayi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->float32_array[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_float64_atom;
      u_float64_atom.real = this->float64_atom;
      *(outbuffer + offset + 0) = (u_float64_atom.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_float64_atom.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_float64_atom.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_float64_atom.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_float64_atom.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_float64_atom.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_float64_atom.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_float64_atom.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->float64_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_float64_arrayi;
      u_float64_arrayi.real = this->float64_array[i];
      *(outbuffer + offset + 0) = (u_float64_arrayi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_float64_arrayi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_float64_arrayi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_float64_arrayi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_float64_arrayi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_float64_arrayi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_float64_arrayi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_float64_arrayi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->float64_array[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_bool_atom;
      u_bool_atom.base = 0;
      u_bool_atom.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bool_atom = u_bool_atom.real;
      offset += sizeof(this->bool_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        bool real;
        uint8_t base;
      } u_bool_arrayi;
      u_bool_arrayi.base = 0;
      u_bool_arrayi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bool_array[i] = u_bool_arrayi.real;
      offset += sizeof(this->bool_array[i]);
      }
      this->uint8_atom =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->uint8_atom);
      for( uint32_t i = 0; i < 4; i++){
      this->uint8_array[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->uint8_array[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_int8_atom;
      u_int8_atom.base = 0;
      u_int8_atom.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->int8_atom = u_int8_atom.real;
      offset += sizeof(this->int8_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_int8_arrayi;
      u_int8_arrayi.base = 0;
      u_int8_arrayi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->int8_array[i] = u_int8_arrayi.real;
      offset += sizeof(this->int8_array[i]);
      }
      this->uint16_atom =  ((uint16_t) (*(inbuffer + offset)));
      this->uint16_atom |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->uint16_atom);
      for( uint32_t i = 0; i < 4; i++){
      this->uint16_array[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->uint16_array[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->uint16_array[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_int32_atom;
      u_int32_atom.base = 0;
      u_int32_atom.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_int32_atom.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_int32_atom.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_int32_atom.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->int32_atom = u_int32_atom.real;
      offset += sizeof(this->int32_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_int32_arrayi;
      u_int32_arrayi.base = 0;
      u_int32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_int32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_int32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_int32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->int32_array[i] = u_int32_arrayi.real;
      offset += sizeof(this->int32_array[i]);
      }
      this->uint32_atom =  ((uint32_t) (*(inbuffer + offset)));
      this->uint32_atom |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uint32_atom |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uint32_atom |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uint32_atom);
      for( uint32_t i = 0; i < 4; i++){
      this->uint32_array[i] =  ((uint32_t) (*(inbuffer + offset)));
      this->uint32_array[i] |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uint32_array[i] |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uint32_array[i] |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uint32_array[i]);
      }
      union {
        int64_t real;
        uint64_t base;
      } u_int64_atom;
      u_int64_atom.base = 0;
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_int64_atom.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->int64_atom = u_int64_atom.real;
      offset += sizeof(this->int64_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        int64_t real;
        uint64_t base;
      } u_int64_arrayi;
      u_int64_arrayi.base = 0;
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_int64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->int64_array[i] = u_int64_arrayi.real;
      offset += sizeof(this->int64_array[i]);
      }
      this->uint64_atom =  ((uint64_t) (*(inbuffer + offset)));
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->uint64_atom |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->uint64_atom);
      for( uint32_t i = 0; i < 4; i++){
      this->uint64_array[i] =  ((uint64_t) (*(inbuffer + offset)));
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->uint64_array[i] |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->uint64_array[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_float32_atom;
      u_float32_atom.base = 0;
      u_float32_atom.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_float32_atom.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_float32_atom.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_float32_atom.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->float32_atom = u_float32_atom.real;
      offset += sizeof(this->float32_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_float32_arrayi;
      u_float32_arrayi.base = 0;
      u_float32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_float32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_float32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_float32_arrayi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->float32_array[i] = u_float32_arrayi.real;
      offset += sizeof(this->float32_array[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_float64_atom;
      u_float64_atom.base = 0;
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_float64_atom.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->float64_atom = u_float64_atom.real;
      offset += sizeof(this->float64_atom);
      for( uint32_t i = 0; i < 4; i++){
      union {
        double real;
        uint64_t base;
      } u_float64_arrayi;
      u_float64_arrayi.base = 0;
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_float64_arrayi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->float64_array[i] = u_float64_arrayi.real;
      offset += sizeof(this->float64_array[i]);
      }
     return offset;
    }

    const char * getType(){ return "jsk_network_tools/AllTypeTest"; };
    const char * getMD5(){ return "e38fde731d43d6674bf0d48497971fd6"; };

  };

}
#endif
