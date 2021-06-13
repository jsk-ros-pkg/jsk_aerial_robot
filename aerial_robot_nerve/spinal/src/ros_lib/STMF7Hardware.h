/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, UT.
 * All rights reserved.
 *
  */

#ifndef ROS_STMF7_HARDWARE_H_
#define ROS_STMF7_HARDWARE_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32f7xx_hal_dma.h"
#include "cmsis_os.h"

#define BUFFER_EMPTY  -1
#define TX_BUFFER_SIZE 128
#define TX_BUFFER_WIDTH 64 // byte
#define RX_BUFFER_SIZE 512

/* TX */
template<int BUFFER_LENGTH>
struct TxBufferUnit{
  uint8_t tx_data_[BUFFER_LENGTH];//this means each txdata packet should be shorter than 255
  uint16_t tx_len_;
} ;

class STMF7Hardware {
public:
  typedef UART_HandleTypeDef serial_class;
  typedef osMutexId mutex_class;
  typedef osSemaphoreId semaphore_class;

  STMF7Hardware(){}

  void init()
  {
    // do nothing
  }

  void init(char *portName)
  {
    // do nothing
  }

  void init(serial_class* huart, mutex_class* mutex = NULL, osSemaphoreId  *semaphore = NULL);

  uint32_t time()
  {
    return HAL_GetTick();
  }

  /* rx */
  bool rx_available();
  int read();

  /* tx */
  void write(uint8_t* data, unsigned int length);
  int publish();

private:
  UART_HandleTypeDef *huart_;

  /* rx */
  uint8_t rx_buf_[RX_BUFFER_SIZE];
  uint32_t rd_ptr_ = 0;

  /* tx */
  osMutexId *tx_mutex_;
  struct TxBufferUnit<TX_BUFFER_WIDTH> tx_buffer_unit_[TX_BUFFER_SIZE];
  uint8_t subscript_in_progress_;
  uint8_t subscript_to_add_;
  bool tx_idle_;

};

#endif
