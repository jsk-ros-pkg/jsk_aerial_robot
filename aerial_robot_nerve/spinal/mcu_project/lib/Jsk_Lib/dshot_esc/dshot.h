//
// Created by li-jinjie on 24-1-21.
// ref: https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef DSHOT_H
#define DSHOT_H

#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_tim.h"
#include <stdbool.h>
#include <math.h>  // lrintf

/* User Configuration */
// Timer Clock
#define TIMER_CLOCK 100000000  // 100MHz

//// MOTOR 1 (PA3) - TIM5 Channel 4, DMA1 Stream 3
// #define MOTOR_1_TIM             (&htim5)
// #define MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_4
//
//// MOTOR 2 (PA2) - TIM2 Channel 3, DMA1 Stream 1
// #define MOTOR_2_TIM             (&htim2)
// #define MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_3
//
//// MOTOR 3 (PA0) - TIM2 Channel 1, DMA1 Stream 5
// #define MOTOR_3_TIM             (&htim2)
// #define MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_1
//
//// MOTOR 4 (PA1) - TIM5 Channel 2, DMA1 Stream 4
// #define MOTOR_4_TIM             (&htim5)
// #define MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_2

/* Definition */
#define MHZ_TO_HZ(x) ((x) * 1000000)

#define DSHOT600_HZ MHZ_TO_HZ(12)
#define DSHOT300_HZ MHZ_TO_HZ(6)
#define DSHOT150_HZ MHZ_TO_HZ(3)

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

#define DSHOT_FRAME_SIZE 16
#define DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

/* Enumeration */
typedef enum
{
  DSHOT150,
  DSHOT300,
  DSHOT600
} dshot_type_e;

class DShot
{
public:
  DShot(){};
  ~DShot(){};

  void init(dshot_type_e dshot_type, TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
            TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2, TIM_HandleTypeDef* htim_motor_3,
            uint32_t channel_motor_3, TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4);

  void write(uint16_t* motor_value);

private:
  TIM_HandleTypeDef* htim_motor_1_;
  uint32_t channel_motor_1_;
  TIM_HandleTypeDef* htim_motor_2_;
  uint32_t channel_motor_2_;
  TIM_HandleTypeDef* htim_motor_3_;
  uint32_t channel_motor_3_;
  TIM_HandleTypeDef* htim_motor_4_;
  uint32_t channel_motor_4_;

  static uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  static uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  static uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  static uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];

  /* Static functions */
  // dshot init
  static uint32_t dshot_choose_type(dshot_type_e dshot_type);
  static void dshot_set_timer(dshot_type_e dshot_type);
  static void dshot_dma_tc_callback(DMA_HandleTypeDef* hdma);
  static void dshot_put_tc_callback_function();
  static void dshot_start_pwm();

  // dshot write
  static uint16_t dshot_prepare_packet(uint16_t value);
  static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value);
  static void dshot_prepare_dmabuffer_all(uint16_t* motor_value);
  static void dshot_dma_start();
  static void dshot_enable_dma_request();
};

#endif  // DSHOT_H
