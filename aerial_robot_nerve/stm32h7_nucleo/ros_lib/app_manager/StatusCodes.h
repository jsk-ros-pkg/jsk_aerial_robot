#ifndef _ROS_app_manager_StatusCodes_h
#define _ROS_app_manager_StatusCodes_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace app_manager
{

  class StatusCodes : public ros::Msg
  {
    public:
      enum { SUCCESS =  0 };
      enum { BAD_REQUEST =  400 };
      enum { NOT_FOUND =  404 };
      enum { NOT_RUNNING =  430 };
      enum { INTERNAL_ERROR =  500 };
      enum { APP_INVALID =  510 };
      enum { MULTIAPP_NOT_SUPPORTED =  511 };

    StatusCodes()
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

    const char * getType(){ return "app_manager/StatusCodes"; };
    const char * getMD5(){ return "5f286aed2b2ab4b227e7b7185bae624d"; };

  };

}
#endif
