#ifndef _ROS_jsk_recognition_msgs_ICPResult_h
#define _ROS_jsk_recognition_msgs_ICPResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace jsk_recognition_msgs
{

  class ICPResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef const char* _name_type;
      _name_type name;
      typedef double _score_type;
      _score_type score;

    ICPResult():
      header(),
      pose(),
      name(""),
      score(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        double real;
        uint64_t base;
      } u_score;
      u_score.real = this->score;
      *(outbuffer + offset + 0) = (u_score.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_score.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_score.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_score.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_score.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_score.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_score.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_score.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->score);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        double real;
        uint64_t base;
      } u_score;
      u_score.base = 0;
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_score.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->score = u_score.real;
      offset += sizeof(this->score);
     return offset;
    }

    virtual const char * getType() override { return "jsk_recognition_msgs/ICPResult"; };
    virtual const char * getMD5() override { return "2d0f1279ba6f378fd67c4a0324acf2d7"; };

  };

}
#endif
