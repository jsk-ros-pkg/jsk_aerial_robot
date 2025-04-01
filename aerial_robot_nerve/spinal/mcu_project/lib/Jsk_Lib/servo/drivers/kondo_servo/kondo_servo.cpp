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


#ifdef STM32H7_V2
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

  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_HalfDuplex_EnableReceiver(huart_);
  HAL_UART_Receive_DMA(huart, dma_rx_buf_, RX_BUFFER_SIZE);

  memset(dma_rx_buf_, 0, RX_BUFFER_SIZE);
  memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);

  servo_state_read_last_time_ = HAL_GetTick();

  pos_rx_ptr_ = 0;
  rd_ptr_ = 0;

  // temporary
  servo_num_ = 1;
  servo_[0].id_ = 1;

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

void KondoServo::update()
{
  for(int i = 0; i < servo_num_; i++)
    {
      if(servo_[i].torque_enable_)
        {
          writePosCmd(servo_[i].id_, servo_[i].goal_position_);
        }
      else
        {
          writePosCmd(servo_[i].id_, 0);  //freed
        }
    }
}

void KondoServo::writePosCmd(int id, uint16_t target_position)
{
  int tx_size = 3;
  uint8_t tx_buff[tx_size];
  uint8_t ret;

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

      /* getting data from Ring Buffer */
      while(true){
        uint8_t rx_data;
        if(read(&rx_data, 0) == HAL_TIMEOUT) break;
        pos_rx_buf_[pos_rx_ptr_] = (uint8_t)rx_data;

        if(pos_rx_ptr_ == 2) registerPos();

        pos_rx_ptr_ ++;
        pos_rx_ptr_ %= KONDO_POSITION_RX_SIZE;
      
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

void KondoServo::registerPos()
{
  int id = (int)(pos_rx_buf_[0] & 0x1f);
  uint16_t present_position = (uint16_t)((0x7f & pos_rx_buf_[1]) << 7) + (uint16_t)(0x7f & pos_rx_buf_[2]);
  for (unsigned int i = 0; i < servo_num_; i++) {
    if(servo_[i].id_ == id) servo_[i].present_position_ = present_position;
  }
  memset(pos_rx_buf_, 0, KONDO_POSITION_RX_SIZE);
}

  

bool KondoServo::available()
{
  dma_write_ptr_ =  (KONDO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (KONDO_BUFFER_SIZE);
  return (rd_ptr_ != dma_write_ptr_);
}

void KondoServo::setTargetPos(const std::map<uint16_t, float>& servo_map)
{
  for(auto servo : servo_map) {
    uint16_t index = servo.first;
    float angle = servo.second;
    uint16_t target_pos = rad2KondoPosConv(angle);

    // temporary command to free servo is angle = 100.0
    if(angle == 100.0)
      {
        servo_[index].torque_enable_ = false;
        servo_[index].goal_position_ = 7500;
        // char buf[100];
        // sprintf(buf, "servo id: %d is freed!", id);
        // nh_->l
      }
    else if(KONDO_SERVO_POSITION_MIN <= target_pos && target_pos <= KONDO_SERVO_POSITION_MAX)
      {
        servo_[index].torque_enable_ = true;
        servo_[index].goal_position_ = target_pos;
      }
  }
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

void KondoServo::inactivate(int id)
{
  activated_[id] = false;
}

void KondoServo::activate(int id)
{
  activated_[id] = true;
}

//TODO: implement following functions
void KondoServo::ping(){}
void KondoServo::reboot(uint8_t servo_index){}
void KondoServo::setTorque(uint8_t servo_index){}
void KondoServo::setHomingOffset(uint8_t servo_index){}
void KondoServo::setRoundOffset(uint8_t servo_index, int32_t ref_value){}
void KondoServo::setPositionGains(uint8_t servo_index){}
void KondoServo::setProfileVelocity(uint8_t servo_index){}
void KondoServo::setCurrentLimit(uint8_t servo_index){}

void KondoServo::setStatusReturnLevel(){}
void KondoServo::getHomingOffset(){}
void KondoServo::getCurrentLimit(){}
void KondoServo::getPositionGains(){}
void KondoServo::getProfileVelocity(){}
