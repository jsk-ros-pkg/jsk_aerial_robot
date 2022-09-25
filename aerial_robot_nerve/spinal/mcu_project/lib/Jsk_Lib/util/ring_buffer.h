/*
******************************************************************************
* File Name          : ring_buffer.h
* Description        : FIFO based on ring buffer
******************************************************************************
*/

#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define PACKED __attribute__((__packed__))

template <typename T,  size_t SIZE>
class RingBufferFiFo
{
public:
  RingBufferFiFo()
  {
    init();
  }
  ~RingBufferFiFo(){  }

  void init()
  {
    subscript_in_progress_ = 0;
    subscript_to_add_ = 0;
    buffer_length_ = (uint16_t)SIZE;
  }

  bool pop(T& pop_value)
  {
    if (subscript_in_progress_ != subscript_to_add_)
      {
        pop_value =  buf_[subscript_in_progress_];

        subscript_in_progress_++;
        if (subscript_in_progress_ == buffer_length_)
          subscript_in_progress_ = 0;

        return true;
      }
    return false;
  }

  bool push(T new_value)
  {
    // the process node should have higher priority than the rx it callback
#if 0
    if ((subscript_in_progress_ == (subscript_to_add_ + 1)) || ( (subscript_to_add_ == (buffer_length_ - 1) )&& (subscript_in_progress_ == 0)) ) return false;
#endif

    buf_[subscript_to_add_] = new_value;

    subscript_to_add_++;

    if (subscript_to_add_ == buffer_length_)
      {
        subscript_to_add_ = 0;
      }

    return true;
  }

  uint16_t length()
  {
    if(subscript_to_add_ - subscript_in_progress_ >= 0)
      return (subscript_to_add_ - subscript_in_progress_);
    else
      return (subscript_to_add_ - (buffer_length_ - subscript_in_progress_));
  }

private:
  T buf_[SIZE];
  int16_t subscript_in_progress_, subscript_to_add_;
  uint16_t buffer_length_;
};

#endif //__RING_BUFFER_H__
