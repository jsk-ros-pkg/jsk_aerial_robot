/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __KONDO_SERVO_H
#define __KONDO_SERVO_H

#include "config.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoState.h>
#include <map>

#define MAX_SERVO_NUM 32
#define KONDO_SERVO_UPDATE_INTERVAL 10
#define KONDO_SERVO_POSITION_MIN 3500
#define KONDO_SERVO_POSITION_MAX 11500
#define KONDO_SERVO_ANGLE_MIN -2.36
#define KONDO_SERVO_ANGLE_MAX 2.36


class KondoServo
{
private:
  UART_HandleTypeDef* port_;
  spinal::ServoStates servo_state_msg_;
  ros::NodeHandle* nh_;
  // ros::Subscriber<spinal::ServoControlCmd, KondoServo> kondo_servo_control_sub_;
  // ros::Publisher servo_state_pub_;
  uint16_t target_position_[MAX_SERVO_NUM];
  uint16_t current_position_[MAX_SERVO_NUM];
  bool activated_[MAX_SERVO_NUM];

public:
  ~KondoServo(){}
  KondoServo()// :
    // kondo_servo_control_sub_("kondo_servo_cmd", &KondoServo::servoControlCallback, this),
    // servo_state_pub_("kondo_servo_states", &servo_state_msg_)
  {
  }

  void init(UART_HandleTypeDef* port, ros::NodeHandle* nh)
  {
    port_ = port;
    nh_ = nh;

    // nh_->subscribe(kondo_servo_control_sub_);
    // nh_->advertise(servo_state_pub_);

    // /* TODO: add search process to access only existing motors*/
    // for(int i = 0; i < MAX_SERVO_NUM; i++)
    //   {
    //     target_position_[i] = readPosition(i);
    //     activated_[i] = false;
    //   }

    /* TODO: define the msg size dynamically depending on avilable servo numer*/
    servo_state_msg_.servos_length = MAX_SERVO_NUM;
    servo_state_msg_.servos = new spinal::ServoState[MAX_SERVO_NUM];
  }

  void update()
  {
    // sendServoState();
    for(int i = 0; i < MAX_SERVO_NUM; i++)
      {
        if(activated_[i])
          {
            setPosition(i, target_position_[i]);
          } else
          {
            setPosition(i, 0);  //freed
          }
      }
  }

  void setPosition(int id, uint16_t target_position)
  {
    int tx_size = 3, rx_size = 3;
    uint8_t tx_buff[tx_size], rx_buff[rx_size];
    uint8_t ret;

    tx_buff[0] = 0x80 + id;
    tx_buff[1] = (uint8_t)((target_position & 0x3f80) >> 7); // higher 7 bits of 14 bits
    tx_buff[2] = (uint8_t)(target_position & 0x007f);        // lower  7 bits of 14 bits

    HAL_HalfDuplex_EnableTransmitter(port_);
    ret = HAL_UART_Transmit(port_, tx_buff, tx_size, 1);
    // if(ret == HAL_OK)
    //   {
    //     HAL_HalfDuplex_EnableReceiver(port_);
    //   }
    // ret = HAL_UART_Receive(port_, rx_buff, rx_size, 1);
    //TODO:DMA
  }

  uint16_t readPosition(int id)
  {
    int tx_size = 2, rx_size = 4;
    uint8_t tx_buff[tx_size], rx_buff[rx_size];
    uint8_t ret;
    tx_buff[0] = 0xA0 + id;
    tx_buff[1] = 0x05;

    HAL_HalfDuplex_EnableTransmitter(port_);
    ret = HAL_UART_Transmit(port_, tx_buff, tx_size, 1);
    if(ret == HAL_OK)
      {
        HAL_HalfDuplex_EnableReceiver(port_);
      }
    ret = HAL_UART_Receive(port_, rx_buff, rx_size, 1);

    if(id == (rx_buff[0] & 0x1f))
      {
        uint16_t current_position = (uint16_t)((0x7f & rx_buff[2]) << 7) + (uint16_t)(0x7f & rx_buff[3]);
        current_position_[id] = current_position;
        return current_position;
      } else
      {
        return 0;
      }
  }

  // void servoControlCallback(const spinal::ServoControlCmd& cmd_msg)
  // {
  //   for(int i = 0; i < cmd_msg.index_length; i++)
  //     {
  //       if(0 <= cmd_msg.index[i] && cmd_msg.index[i] < MAX_SERVO_NUM)
  //         {
  //           if(KONDO_SERVO_POSITION_MIN <= cmd_msg.angles[i] && cmd_msg.angles[i] <= KONDO_SERVO_POSITION_MAX)
  //             {
  //               activated_[cmd_msg.index[i]] = true;
  //               target_position_[cmd_msg.index[i]] = cmd_msg.angles[i];
  //             }
  //           else if(cmd_msg.angles[i] == 0)
  //             {
  //               activated_[cmd_msg.index[i]] = false;
  //             }
  //         }
  //     }
  // }

  // void sendServoState()
  // {
  //   servo_state_msg_.stamp = nh_->now();
  //   bool send_flag = false;
  //   for (unsigned int i = 0; i < MAX_SERVO_NUM; i++)
  //     {
  //   	if(activated_[i])
  //         {
  //           send_flag = true;
  //           spinal::ServoState servo;
  //           servo.index = i;
  //           servo.angle = current_position_[i];
  //           servo_state_msg_.servos[i] = servo;
  //         }
  //       if(send_flag) servo_state_pub_.publish(&servo_state_msg_);
  //     }
  // }

  void setTargetPos(const std::map<uint16_t, float>& servo_map)
  {
    for(auto servo : servo_map) {
      uint16_t id = servo.first;
      float angle = servo.second;
      uint16_t target_pos = rad2KondoPosConv(angle);

      // temporary command to free servo is angle = 100.0
      if(angle == 100.0)
        {
          activated_[id] = false;
          target_position_[id] = 7500;
          // char buf[100];
          // sprintf(buf, "servo id: %d is freed!", id);
          // nh_->l
        }
      else if(KONDO_SERVO_POSITION_MIN <= target_pos && target_pos <= KONDO_SERVO_POSITION_MAX)
        {
          activated_[id] = true;
          target_position_[id] = target_pos;
        }
    }
  }

  
  uint16_t rad2KondoPosConv(float angle)
  {
    uint16_t kondo_pos = (uint16_t)((KONDO_SERVO_POSITION_MAX-KONDO_SERVO_POSITION_MIN)*(-angle - KONDO_SERVO_ANGLE_MIN)/(KONDO_SERVO_ANGLE_MAX - KONDO_SERVO_ANGLE_MIN) + KONDO_SERVO_POSITION_MIN); //min-max normarization

    return kondo_pos;
  }

  float kondoPos2RadConv(int pos)
  {
    float angle = -(float)((KONDO_SERVO_ANGLE_MAX-KONDO_SERVO_ANGLE_MIN)*(pos - KONDO_SERVO_POSITION_MIN)/(KONDO_SERVO_POSITION_MAX - KONDO_SERVO_POSITION_MIN) + KONDO_SERVO_ANGLE_MIN); //min-max normarization

    return angle;
  }

  void inactivate(int id)
  {
    activated_[id] = false;
  }

  void activate(int id)
  {
    activated_[id] = true;
    target_position_[id] = readPosition(id);
  }

};

#endif
