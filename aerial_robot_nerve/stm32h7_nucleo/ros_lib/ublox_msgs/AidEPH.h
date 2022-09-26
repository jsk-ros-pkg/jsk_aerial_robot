#ifndef _ROS_ublox_msgs_AidEPH_h
#define _ROS_ublox_msgs_AidEPH_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class AidEPH : public ros::Msg
  {
    public:
      typedef uint32_t _svid_type;
      _svid_type svid;
      typedef uint32_t _how_type;
      _how_type how;
      uint32_t sf1d_length;
      typedef uint32_t _sf1d_type;
      _sf1d_type st_sf1d;
      _sf1d_type * sf1d;
      uint32_t sf2d_length;
      typedef uint32_t _sf2d_type;
      _sf2d_type st_sf2d;
      _sf2d_type * sf2d;
      uint32_t sf3d_length;
      typedef uint32_t _sf3d_type;
      _sf3d_type st_sf3d;
      _sf3d_type * sf3d;
      enum { CLASS_ID =  11 };
      enum { MESSAGE_ID =  49 };

    AidEPH():
      svid(0),
      how(0),
      sf1d_length(0), st_sf1d(), sf1d(nullptr),
      sf2d_length(0), st_sf2d(), sf2d(nullptr),
      sf3d_length(0), st_sf3d(), sf3d(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->svid >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->svid >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->svid >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->svid >> (8 * 3)) & 0xFF;
      offset += sizeof(this->svid);
      *(outbuffer + offset + 0) = (this->how >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->how >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->how >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->how >> (8 * 3)) & 0xFF;
      offset += sizeof(this->how);
      *(outbuffer + offset + 0) = (this->sf1d_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf1d_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf1d_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf1d_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf1d_length);
      for( uint32_t i = 0; i < sf1d_length; i++){
      *(outbuffer + offset + 0) = (this->sf1d[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf1d[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf1d[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf1d[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf1d[i]);
      }
      *(outbuffer + offset + 0) = (this->sf2d_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf2d_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf2d_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf2d_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf2d_length);
      for( uint32_t i = 0; i < sf2d_length; i++){
      *(outbuffer + offset + 0) = (this->sf2d[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf2d[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf2d[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf2d[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf2d[i]);
      }
      *(outbuffer + offset + 0) = (this->sf3d_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf3d_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf3d_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf3d_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf3d_length);
      for( uint32_t i = 0; i < sf3d_length; i++){
      *(outbuffer + offset + 0) = (this->sf3d[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sf3d[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sf3d[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sf3d[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sf3d[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->svid =  ((uint32_t) (*(inbuffer + offset)));
      this->svid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->svid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->svid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->svid);
      this->how =  ((uint32_t) (*(inbuffer + offset)));
      this->how |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->how |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->how |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->how);
      uint32_t sf1d_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sf1d_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sf1d_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sf1d_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sf1d_length);
      if(sf1d_lengthT > sf1d_length)
        this->sf1d = (uint32_t*)realloc(this->sf1d, sf1d_lengthT * sizeof(uint32_t));
      sf1d_length = sf1d_lengthT;
      for( uint32_t i = 0; i < sf1d_length; i++){
      this->st_sf1d =  ((uint32_t) (*(inbuffer + offset)));
      this->st_sf1d |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_sf1d |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_sf1d |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_sf1d);
        memcpy( &(this->sf1d[i]), &(this->st_sf1d), sizeof(uint32_t));
      }
      uint32_t sf2d_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sf2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sf2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sf2d_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sf2d_length);
      if(sf2d_lengthT > sf2d_length)
        this->sf2d = (uint32_t*)realloc(this->sf2d, sf2d_lengthT * sizeof(uint32_t));
      sf2d_length = sf2d_lengthT;
      for( uint32_t i = 0; i < sf2d_length; i++){
      this->st_sf2d =  ((uint32_t) (*(inbuffer + offset)));
      this->st_sf2d |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_sf2d |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_sf2d |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_sf2d);
        memcpy( &(this->sf2d[i]), &(this->st_sf2d), sizeof(uint32_t));
      }
      uint32_t sf3d_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sf3d_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sf3d_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sf3d_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sf3d_length);
      if(sf3d_lengthT > sf3d_length)
        this->sf3d = (uint32_t*)realloc(this->sf3d, sf3d_lengthT * sizeof(uint32_t));
      sf3d_length = sf3d_lengthT;
      for( uint32_t i = 0; i < sf3d_length; i++){
      this->st_sf3d =  ((uint32_t) (*(inbuffer + offset)));
      this->st_sf3d |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_sf3d |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_sf3d |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_sf3d);
        memcpy( &(this->sf3d[i]), &(this->st_sf3d), sizeof(uint32_t));
      }
     return offset;
    }

    virtual const char * getType() override { return "ublox_msgs/AidEPH"; };
    virtual const char * getMD5() override { return "796d86b27ebfe497b3a42695f2e69e13"; };

  };

}
#endif
