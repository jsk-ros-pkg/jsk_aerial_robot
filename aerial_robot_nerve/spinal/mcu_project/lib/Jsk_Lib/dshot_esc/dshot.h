//
// Created by li-jinjie on 24-1-21.
// ref: https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <cstdint>
#include <math.h>  // lrintf
#include "esc_telem.h"

/* User Configuration */
// Timer Clock
#define TIMER_CLOCK 100000000  // 100MHz

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

#define DSHOT_CMD_SPIN_DIRECTION_1 7
#define DSHOT_CMD_SPIN_DIRECTION_2 8

namespace
{
#ifdef STM32H7
  uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection1")));
  uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection2")));
  uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection3")));
  uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection4")));
#else
  uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
  uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
#endif
}

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
  void initTelemetry(UART_HandleTypeDef* huart);

  void write(uint16_t* motor_value_array, bool is_telemetry);

  /* dshot telemtry */
  bool is_telemetry_ = false;
  int id_telem_ = 0;
  int id_telem_prev_ = -1;

  ESCReader esc_reader_;

private:
  TIM_HandleTypeDef* htim_motor_1_;
  uint32_t channel_motor_1_;
  TIM_HandleTypeDef* htim_motor_2_;
  uint32_t channel_motor_2_;
  TIM_HandleTypeDef* htim_motor_3_;
  uint32_t channel_motor_3_;
  TIM_HandleTypeDef* htim_motor_4_;
  uint32_t channel_motor_4_;

  /* Static functions */
  // dshot init
  uint32_t dshot_choose_type(dshot_type_e dshot_type);
  void dshot_set_timer(dshot_type_e dshot_type);
  static void dshot_dma_tc_callback(DMA_HandleTypeDef* hdma);
  void dshot_put_tc_callback_function();
  void dshot_start_pwm();

  // dshot write
  uint16_t dshot_prepare_packet(uint16_t value, bool dshot_telemetry);
  void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry);
  void dshot_prepare_dmabuffer_all(uint16_t* motor_value, bool* is_telemetry);
  void dshot_dma_start();
  void dshot_enable_dma_request();

  int num_freq_divide = 0;  // make sure that the freq for telemetry is < 150hz
};

#endif  // DSHOT_H
