/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "servo/drivers/kondo_servo/kondo_servo.h"


#if STM32H7_V2 || STM32H7_KASANE
uint8_t dma_rx_buf_[KONDO_BUFFER_SIZE] __attribute__((section(".ServoRxBufferSection")));
#else
#ifdef STM32H7
uint8_t dma_rx_buf_[KONDO_BUFFER_SIZE] __attribute__((section(".GpsRxBufferSection")));
#else
uint8_t dma_rx_buf_[KONDO_BUFFER_SIZE];
#endif
#endif

void KondoServo::init(UART_HandleTypeDef* huart,  osMutexId* mutex)
{
  huart_ = huart;

  pinReconfig();
  
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_HalfDuplex_EnableReceiver(huart_);
  HAL_UART_Receive_DMA(huart, dma_rx_buf_, RX_BUFFER_SIZE);

  memset(dma_rx_buf_, 0, RX_BUFFER_SIZE);
  memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);

  servo_state_read_last_time_ = HAL_GetTick();

  pos_rx_ptr_ = 0;
  rd_ptr_ = 0;

  servo_num_ = 0;
  read_servo_index_ = 0;

  ping();
  // temporary
  // servo_num_ = 1;
  // servo_[0].id_ = 1;

  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    FlashMemory::addValue(&(servo_[i].p_gain_), 2);
    FlashMemory::addValue(&(servo_[i].i_gain_), 2);
    FlashMemory::addValue(&(servo_[i].d_gain_), 2);
    FlashMemory::addValue(&(servo_[i].profile_velocity_), 2);
    FlashMemory::addValue(&(servo_[i].send_data_flag_), 2);
    FlashMemory::addValue(&(servo_[i].external_encoder_flag_), 2);
    FlashMemory::addValue(&(servo_[i].joint_resolution_), 2);
    FlashMemory::addValue(&(servo_[i].servo_resolution_), 2);
    FlashMemory::addValue(&(servo_[i].joint_offset_), 4);
  }
  FlashMemory::addValue(&(ttl_rs485_mixed_), 2);

  FlashMemory::read();

  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    servo_[i].hardware_error_status_ = 0;
    if(servo_[i].servo_resolution_ == 65535 || servo_[i].joint_resolution_ == 65535){
      servo_[i].resolution_ratio_ = 1;
    }
    else{
      servo_[i].resolution_ratio_ = (float)servo_[i].servo_resolution_ / (float)servo_[i].joint_resolution_;
    }
  }

}

void KondoServo::pinReconfig()
{
  while(HAL_UART_DeInit(huart_) != HAL_OK);

  /*Change baud rate*/
  huart_->Init.BaudRate = 1250000;
  huart_->Init.WordLength = UART_WORDLENGTH_9B;
  huart_->Init.Parity = UART_PARITY_EVEN;

  /*Initialize as halfduplex mode*/
  while(HAL_HalfDuplex_Init(huart_) != HAL_OK);  

}

void KondoServo::update()
{
  // for(int i = 0; i < servo_num_; i++)
  //   {
  //     if(servo_[i].torque_enable_)
  //       {
  //         writePosCmd(servo_[i].id_, servo_[i].goal_position_);
  //       }
  //     else
  //       {
  //         writePosCmd(servo_[i].id_, 0);  //freed
  //       }
  //   }

  if(servo_[read_servo_index_].torque_enable_)
    {
      writePosCmd(servo_[read_servo_index_].id_, servo_[read_servo_index_].goal_position_);
    }
  else
    {
      writePosCmd(servo_[read_servo_index_].id_, 0);  //freed
    }
  read_servo_index_ ++;
  read_servo_index_ %= servo_num_;
}

void KondoServo::ping()
{
  for(int i = 0; i < MAX_SERVO_NUM; i++)
    {
      int tx_size = 3;
      uint8_t tx_buff[tx_size];
      uint8_t ret;

      /* transmit empty protocol */
      tx_buff[0] = 0x80 + i;
      tx_buff[1] = (uint8_t)((0x0000 & 0x3f80) >> 7); // higher 7 bits of 14 bits
      tx_buff[2] = (uint8_t)(0x0000 & 0x007f);        // lower  7 bits of 14 bits
      
      HAL_HalfDuplex_EnableTransmitter(huart_);
      ret = HAL_UART_Transmit(huart_, tx_buff, tx_size, 1);

      /* receive */
      if(ret == HAL_OK)
        {
          HAL_HalfDuplex_EnableReceiver(huart_);
        }

      HAL_Delay(10);

      /* getting data from Ring Buffer */
      while(true){
        uint8_t rx_data;
        if(read(&rx_data, 10) == HAL_TIMEOUT) break;
        pos_rx_buf_[pos_rx_ptr_] = (uint8_t)rx_data;
        if(pos_rx_ptr_ == 2)
          {
            int id = (int)(pos_rx_buf_[0] & 0x1f);
            servo_[servo_num_++].id_ = i;
            memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);
          }
        pos_rx_ptr_ ++;
        pos_rx_ptr_ %= KONDO_POSITION_RX_SIZE;
      }
    }
}

void KondoServo::writePosCmd(int id, uint16_t target_position)
{
  int tx_size = 3;
  uint8_t tx_buff[tx_size];
  uint8_t ret;

  /* getting data from Ring Buffer */
  while(true){
    uint8_t rx_data;
    if(read(&rx_data, 10) == HAL_TIMEOUT) break;
    pos_rx_buf_[pos_rx_ptr_] = (uint8_t)rx_data;

    if(pos_rx_ptr_ == 2) registerPos();

    pos_rx_ptr_ ++;
    pos_rx_ptr_ %= KONDO_POSITION_RX_SIZE;      
  }  

  /* transmit */
  tx_buff[0] = 0x80 + id;
  tx_buff[1] = (uint8_t)((target_position & 0x3f80) >> 7); // higher 7 bits of 14 bits
  tx_buff[2] = (uint8_t)(target_position & 0x007f);        // lower  7 bits of 14 bits
  HAL_HalfDuplex_EnableTransmitter(huart_);
  ret = HAL_UART_Transmit(huart_, tx_buff, tx_size, 1);

  if(HAL_GetTick() - servo_state_read_last_time_ > KONDO_SERVO_UPDATE_INTERVAL)
    {
      servo_state_read_last_time_ = HAL_GetTick();
      /* receive */
      if(ret == HAL_OK)
        {
          HAL_HalfDuplex_EnableReceiver(huart_);
        }
    }
}

HAL_StatusTypeDef KondoServo::read(uint8_t* data,  uint32_t timeout)
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      __HAL_UART_CLEAR_FLAG(huart_,
                            UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
      HAL_UART_Receive_DMA(huart_, dma_rx_buf_, RX_BUFFER_SIZE); // restart
    }
  dma_write_ptr_ =  (KONDO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (KONDO_BUFFER_SIZE);

  uint32_t tick_start = HAL_GetTick();
  while(true){
    if(rd_ptr_ != dma_write_ptr_)
      {
        *data = (int)dma_rx_buf_[rd_ptr_++];
        rd_ptr_ %= KONDO_BUFFER_SIZE;
        return HAL_OK;
      }

    if ((HAL_GetTick() - tick_start) > KONDO_SERVO_TIMEOUT || KONDO_SERVO_TIMEOUT == 0)
      {
        return HAL_TIMEOUT;
      }
  }
}

void KondoServo::setTorqueFromPresetnPos(uint8_t servo_index)
{
  if(servo_[servo_index].torque_enable_) servo_[servo_index].goal_position_ = servo_[servo_index].present_position_;
}

void KondoServo::registerPos()
{
  int id = (int)(pos_rx_buf_[0] & 0x1f);
  uint16_t present_position = (uint16_t)((0x7f & pos_rx_buf_[1]) << 7) + (uint16_t)(0x7f & pos_rx_buf_[2]);
  for (unsigned int i = 0; i < servo_num_; i++) {
    if(servo_[i].id_ == id) servo_[i].present_position_ = present_position;
  }
  memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);
}
  
uint16_t KondoServo::rad2KondoPosConv(float angle)
{
  uint16_t kondo_pos = (uint16_t)((KONDO_SERVO_POSITION_MAX-KONDO_SERVO_POSITION_MIN)*(-angle - KONDO_SERVO_ANGLE_MIN)/(KONDO_SERVO_ANGLE_MAX - KONDO_SERVO_ANGLE_MIN) + KONDO_SERVO_POSITION_MIN); //min-max normarization
  return kondo_pos;
}

float KondoServo::kondoPos2RadConv(int pos)
{
  float angle = -(float)((KONDO_SERVO_ANGLE_MAX-KONDO_SERVO_ANGLE_MIN)*(pos - KONDO_SERVO_POSITION_MIN)/(KONDO_SERVO_POSITION_MAX - KONDO_SERVO_POSITION_MIN) + KONDO_SERVO_ANGLE_MIN); //min-max normarization

  return angle;
}

//TODO: implement following functions
void KondoServo::reboot(uint8_t servo_index){}
void KondoServo::setHomingOffset(uint8_t servo_index){}
void KondoServo::setRoundOffset(uint8_t servo_index, int32_t ref_value){}
void KondoServo::setPositionGains(uint8_t servo_index){}
void KondoServo::setProfileVelocity(uint8_t servo_index){}
void KondoServo::setCurrentLimit(uint8_t servo_index){}
void KondoServo::setTorque(uint8_t servo_index){}
  
void KondoServo::setStatusReturnLevel(){}
void KondoServo::getHomingOffset(){}
void KondoServo::getCurrentLimit(){}
void KondoServo::getPositionGains(){}
void KondoServo::getProfileVelocity(){}
