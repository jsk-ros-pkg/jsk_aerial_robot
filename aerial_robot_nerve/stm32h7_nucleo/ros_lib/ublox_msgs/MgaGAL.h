#ifndef _ROS_ublox_msgs_MgaGAL_h
#define _ROS_ublox_msgs_MgaGAL_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class MgaGAL : public ros::Msg
  {
    public:
      typedef uint8_t _type_type;
      _type_type type;
      typedef uint8_t _version_type;
      _version_type version;
      typedef uint8_t _svid_type;
      _svid_type svid;
      typedef uint8_t _reserved0_type;
      _reserved0_type reserved0;
      typedef uint16_t _iodNav_type;
      _iodNav_type iodNav;
      typedef int16_t _deltaN_type;
      _deltaN_type deltaN;
      typedef int32_t _m0_type;
      _m0_type m0;
      typedef uint32_t _e_type;
      _e_type e;
      typedef uint32_t _sqrtA_type;
      _sqrtA_type sqrtA;
      typedef int32_t _omega0_type;
      _omega0_type omega0;
      typedef int32_t _i0_type;
      _i0_type i0;
      typedef int32_t _omega_type;
      _omega_type omega;
      typedef int32_t _omegaDot_type;
      _omegaDot_type omegaDot;
      typedef int16_t _iDot_type;
      _iDot_type iDot;
      typedef int16_t _cuc_type;
      _cuc_type cuc;
      typedef int16_t _cus_type;
      _cus_type cus;
      typedef int16_t _crc_type;
      _crc_type crc;
      typedef int16_t _crs_type;
      _crs_type crs;
      typedef int16_t _cic_type;
      _cic_type cic;
      typedef int16_t _cis_type;
      _cis_type cis;
      typedef uint16_t _toe_type;
      _toe_type toe;
      typedef int32_t _af0_type;
      _af0_type af0;
      typedef int32_t _af1_type;
      _af1_type af1;
      typedef int8_t _af2_type;
      _af2_type af2;
      typedef uint8_t _sisaindexE1E5b_type;
      _sisaindexE1E5b_type sisaindexE1E5b;
      typedef uint16_t _toc_type;
      _toc_type toc;
      typedef int16_t _bgdE1E5b_type;
      _bgdE1E5b_type bgdE1E5b;
      uint8_t reserved1[2];
      typedef uint8_t _healthE1B_type;
      _healthE1B_type healthE1B;
      typedef uint8_t _dataValidityE1B_type;
      _dataValidityE1B_type dataValidityE1B;
      typedef uint8_t _healthE5b_type;
      _healthE5b_type healthE5b;
      typedef uint8_t _dataValidityE5b_type;
      _dataValidityE5b_type dataValidityE5b;
      uint8_t reserved2[4];
      enum { CLASS_ID =  19 };
      enum { MESSAGE_ID =  2 };

    MgaGAL():
      type(0),
      version(0),
      svid(0),
      reserved0(0),
      iodNav(0),
      deltaN(0),
      m0(0),
      e(0),
      sqrtA(0),
      omega0(0),
      i0(0),
      omega(0),
      omegaDot(0),
      iDot(0),
      cuc(0),
      cus(0),
      crc(0),
      crs(0),
      cic(0),
      cis(0),
      toe(0),
      af0(0),
      af1(0),
      af2(0),
      sisaindexE1E5b(0),
      toc(0),
      bgdE1E5b(0),
      reserved1(),
      healthE1B(0),
      dataValidityE1B(0),
      healthE5b(0),
      dataValidityE5b(0),
      reserved2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->version);
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->reserved0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0);
      *(outbuffer + offset + 0) = (this->iodNav >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->iodNav >> (8 * 1)) & 0xFF;
      offset += sizeof(this->iodNav);
      union {
        int16_t real;
        uint16_t base;
      } u_deltaN;
      u_deltaN.real = this->deltaN;
      *(outbuffer + offset + 0) = (u_deltaN.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_deltaN.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->deltaN);
      union {
        int32_t real;
        uint32_t base;
      } u_m0;
      u_m0.real = this->m0;
      *(outbuffer + offset + 0) = (u_m0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m0);
      *(outbuffer + offset + 0) = (this->e >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->e >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->e >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->e >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e);
      *(outbuffer + offset + 0) = (this->sqrtA >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sqrtA >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sqrtA >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sqrtA >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sqrtA);
      union {
        int32_t real;
        uint32_t base;
      } u_omega0;
      u_omega0.real = this->omega0;
      *(outbuffer + offset + 0) = (u_omega0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_omega0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_omega0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_omega0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->omega0);
      union {
        int32_t real;
        uint32_t base;
      } u_i0;
      u_i0.real = this->i0;
      *(outbuffer + offset + 0) = (u_i0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i0);
      union {
        int32_t real;
        uint32_t base;
      } u_omega;
      u_omega.real = this->omega;
      *(outbuffer + offset + 0) = (u_omega.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_omega.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_omega.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_omega.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->omega);
      union {
        int32_t real;
        uint32_t base;
      } u_omegaDot;
      u_omegaDot.real = this->omegaDot;
      *(outbuffer + offset + 0) = (u_omegaDot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_omegaDot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_omegaDot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_omegaDot.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->omegaDot);
      union {
        int16_t real;
        uint16_t base;
      } u_iDot;
      u_iDot.real = this->iDot;
      *(outbuffer + offset + 0) = (u_iDot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_iDot.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->iDot);
      union {
        int16_t real;
        uint16_t base;
      } u_cuc;
      u_cuc.real = this->cuc;
      *(outbuffer + offset + 0) = (u_cuc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cuc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cuc);
      union {
        int16_t real;
        uint16_t base;
      } u_cus;
      u_cus.real = this->cus;
      *(outbuffer + offset + 0) = (u_cus.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cus.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cus);
      union {
        int16_t real;
        uint16_t base;
      } u_crc;
      u_crc.real = this->crc;
      *(outbuffer + offset + 0) = (u_crc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_crc.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->crc);
      union {
        int16_t real;
        uint16_t base;
      } u_crs;
      u_crs.real = this->crs;
      *(outbuffer + offset + 0) = (u_crs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_crs.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->crs);
      union {
        int16_t real;
        uint16_t base;
      } u_cic;
      u_cic.real = this->cic;
      *(outbuffer + offset + 0) = (u_cic.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cic.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cic);
      union {
        int16_t real;
        uint16_t base;
      } u_cis;
      u_cis.real = this->cis;
      *(outbuffer + offset + 0) = (u_cis.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cis.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->cis);
      *(outbuffer + offset + 0) = (this->toe >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->toe >> (8 * 1)) & 0xFF;
      offset += sizeof(this->toe);
      union {
        int32_t real;
        uint32_t base;
      } u_af0;
      u_af0.real = this->af0;
      *(outbuffer + offset + 0) = (u_af0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_af0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_af0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_af0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->af0);
      union {
        int32_t real;
        uint32_t base;
      } u_af1;
      u_af1.real = this->af1;
      *(outbuffer + offset + 0) = (u_af1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_af1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_af1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_af1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->af1);
      union {
        int8_t real;
        uint8_t base;
      } u_af2;
      u_af2.real = this->af2;
      *(outbuffer + offset + 0) = (u_af2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->af2);
      *(outbuffer + offset + 0) = (this->sisaindexE1E5b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sisaindexE1E5b);
      *(outbuffer + offset + 0) = (this->toc >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->toc >> (8 * 1)) & 0xFF;
      offset += sizeof(this->toc);
      union {
        int16_t real;
        uint16_t base;
      } u_bgdE1E5b;
      u_bgdE1E5b.real = this->bgdE1E5b;
      *(outbuffer + offset + 0) = (u_bgdE1E5b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bgdE1E5b.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->bgdE1E5b);
      for( uint32_t i = 0; i < 2; i++){
      *(outbuffer + offset + 0) = (this->reserved1[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved1[i]);
      }
      *(outbuffer + offset + 0) = (this->healthE1B >> (8 * 0)) & 0xFF;
      offset += sizeof(this->healthE1B);
      *(outbuffer + offset + 0) = (this->dataValidityE1B >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dataValidityE1B);
      *(outbuffer + offset + 0) = (this->healthE5b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->healthE5b);
      *(outbuffer + offset + 0) = (this->dataValidityE5b >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dataValidityE5b);
      for( uint32_t i = 0; i < 4; i++){
      *(outbuffer + offset + 0) = (this->reserved2[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved2[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      this->version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->version);
      this->svid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->svid);
      this->reserved0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0);
      this->iodNav =  ((uint16_t) (*(inbuffer + offset)));
      this->iodNav |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->iodNav);
      union {
        int16_t real;
        uint16_t base;
      } u_deltaN;
      u_deltaN.base = 0;
      u_deltaN.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_deltaN.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->deltaN = u_deltaN.real;
      offset += sizeof(this->deltaN);
      union {
        int32_t real;
        uint32_t base;
      } u_m0;
      u_m0.base = 0;
      u_m0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->m0 = u_m0.real;
      offset += sizeof(this->m0);
      this->e =  ((uint32_t) (*(inbuffer + offset)));
      this->e |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->e |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->e |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->e);
      this->sqrtA =  ((uint32_t) (*(inbuffer + offset)));
      this->sqrtA |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sqrtA |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sqrtA |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sqrtA);
      union {
        int32_t real;
        uint32_t base;
      } u_omega0;
      u_omega0.base = 0;
      u_omega0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_omega0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_omega0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_omega0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->omega0 = u_omega0.real;
      offset += sizeof(this->omega0);
      union {
        int32_t real;
        uint32_t base;
      } u_i0;
      u_i0.base = 0;
      u_i0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i0 = u_i0.real;
      offset += sizeof(this->i0);
      union {
        int32_t real;
        uint32_t base;
      } u_omega;
      u_omega.base = 0;
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->omega = u_omega.real;
      offset += sizeof(this->omega);
      union {
        int32_t real;
        uint32_t base;
      } u_omegaDot;
      u_omegaDot.base = 0;
      u_omegaDot.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_omegaDot.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_omegaDot.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_omegaDot.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->omegaDot = u_omegaDot.real;
      offset += sizeof(this->omegaDot);
      union {
        int16_t real;
        uint16_t base;
      } u_iDot;
      u_iDot.base = 0;
      u_iDot.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_iDot.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->iDot = u_iDot.real;
      offset += sizeof(this->iDot);
      union {
        int16_t real;
        uint16_t base;
      } u_cuc;
      u_cuc.base = 0;
      u_cuc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cuc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cuc = u_cuc.real;
      offset += sizeof(this->cuc);
      union {
        int16_t real;
        uint16_t base;
      } u_cus;
      u_cus.base = 0;
      u_cus.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cus.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cus = u_cus.real;
      offset += sizeof(this->cus);
      union {
        int16_t real;
        uint16_t base;
      } u_crc;
      u_crc.base = 0;
      u_crc.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_crc.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->crc = u_crc.real;
      offset += sizeof(this->crc);
      union {
        int16_t real;
        uint16_t base;
      } u_crs;
      u_crs.base = 0;
      u_crs.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_crs.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->crs = u_crs.real;
      offset += sizeof(this->crs);
      union {
        int16_t real;
        uint16_t base;
      } u_cic;
      u_cic.base = 0;
      u_cic.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cic.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cic = u_cic.real;
      offset += sizeof(this->cic);
      union {
        int16_t real;
        uint16_t base;
      } u_cis;
      u_cis.base = 0;
      u_cis.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cis.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cis = u_cis.real;
      offset += sizeof(this->cis);
      this->toe =  ((uint16_t) (*(inbuffer + offset)));
      this->toe |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->toe);
      union {
        int32_t real;
        uint32_t base;
      } u_af0;
      u_af0.base = 0;
      u_af0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_af0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_af0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_af0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->af0 = u_af0.real;
      offset += sizeof(this->af0);
      union {
        int32_t real;
        uint32_t base;
      } u_af1;
      u_af1.base = 0;
      u_af1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_af1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_af1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_af1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->af1 = u_af1.real;
      offset += sizeof(this->af1);
      union {
        int8_t real;
        uint8_t base;
      } u_af2;
      u_af2.base = 0;
      u_af2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->af2 = u_af2.real;
      offset += sizeof(this->af2);
      this->sisaindexE1E5b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sisaindexE1E5b);
      this->toc =  ((uint16_t) (*(inbuffer + offset)));
      this->toc |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->toc);
      union {
        int16_t real;
        uint16_t base;
      } u_bgdE1E5b;
      u_bgdE1E5b.base = 0;
      u_bgdE1E5b.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bgdE1E5b.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->bgdE1E5b = u_bgdE1E5b.real;
      offset += sizeof(this->bgdE1E5b);
      for( uint32_t i = 0; i < 2; i++){
      this->reserved1[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved1[i]);
      }
      this->healthE1B =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->healthE1B);
      this->dataValidityE1B =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dataValidityE1B);
      this->healthE5b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->healthE5b);
      this->dataValidityE5b =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dataValidityE5b);
      for( uint32_t i = 0; i < 4; i++){
      this->reserved2[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved2[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/MgaGAL"; };
    virtual const char * getMD5() override { return "916efe401cfebd852654e34c3cd97512"; };

  };

}
#endif
