#ifndef _ROS_spinal_TorqueAllocationMatrixInv_h
#define _ROS_spinal_TorqueAllocationMatrixInv_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "spinal/Vector3Int16.h"

namespace spinal
{

  class TorqueAllocationMatrixInv : public ros::Msg
  {
    public:
      uint32_t rows_length;
      typedef spinal::Vector3Int16 _rows_type;
      _rows_type st_rows;
      _rows_type * rows;

    TorqueAllocationMatrixInv():
      rows_length(0), rows(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->rows_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rows_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rows_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rows_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rows_length);
      for( uint32_t i = 0; i < rows_length; i++){
      offset += this->rows[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t rows_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rows_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rows_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rows_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rows_length);
      if(rows_lengthT > rows_length)
        this->rows = (spinal::Vector3Int16*)realloc(this->rows, rows_lengthT * sizeof(spinal::Vector3Int16));
      rows_length = rows_lengthT;
      for( uint32_t i = 0; i < rows_length; i++){
      offset += this->st_rows.deserialize(inbuffer + offset);
        memcpy( &(this->rows[i]), &(this->st_rows), sizeof(spinal::Vector3Int16));
      }
     return offset;
    }

    const char * getType(){ return "spinal/TorqueAllocationMatrixInv"; };
    const char * getMD5(){ return "048fbe8a4e819a7453eec7556fbdc879"; };

  };

}
#endif
