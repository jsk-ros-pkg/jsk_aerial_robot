#ifndef _ROS_aerial_robot_msgs_DynamicReconfigureLevels_h
#define _ROS_aerial_robot_msgs_DynamicReconfigureLevels_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace aerial_robot_msgs
{

  class DynamicReconfigureLevels : public ros::Msg
  {
    public:
      enum { RECONFIGURE_KALMAN_FILTER_FLAG =  0 };
      enum { RECONFIGURE_INPUT_SIGMA =  1 };
      enum { RECONFIGURE_BIAS_SIGMA =  2 };
      enum { RECONFIGURE_MEASURE_SIGMA =  3 };
      enum { RECONFIGURE_CONTROL_FLAG =  0 };
      enum { RECONFIGURE_P_GAIN =  1 };
      enum { RECONFIGURE_I_GAIN =  2 };
      enum { RECONFIGURE_D_GAIN =  3 };
      enum { RECONFIGURE_LQI_FLAG =  0 };
      enum { RECONFIGURE_LQI_ROLL_PITCH_P =  1 };
      enum { RECONFIGURE_LQI_ROLL_PITCH_I =  2 };
      enum { RECONFIGURE_LQI_ROLL_PITCH_D =  3 };
      enum { RECONFIGURE_LQI_YAW_P =  4 };
      enum { RECONFIGURE_LQI_YAW_I =  5 };
      enum { RECONFIGURE_LQI_YAW_D =  6 };
      enum { RECONFIGURE_LQI_Z_P =  7 };
      enum { RECONFIGURE_LQI_Z_I =  8 };
      enum { RECONFIGURE_LQI_Z_D =  9 };

    DynamicReconfigureLevels()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "aerial_robot_msgs/DynamicReconfigureLevels"; };
    const char * getMD5(){ return "ed3d0732cfbb793bcb46da1ed7b2273e"; };

  };

}
#endif
