/*
******************************************************************************
* File Name          : config.h
* Description        : Basic configuration for this board(c.f. GPIO Macro, Enable flag for different processes
******************************************************************************
*/
#ifndef __CONFIG_H
#define __CONFIG_H
#define SPINAL

#if defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx) || defined (STM32F765xx) || \
    defined (STM32F767xx) || defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx) || \
    defined (STM32F722xx) || defined (STM32F723xx) || defined (STM32F732xx) || defined (STM32F733xx) || \
    defined (STM32F730xx) || defined (STM32F750xx)
#include "stm32f7xx_hal.h"
#define STM32F7
#endif

#if defined (STM32H743xx) || defined (STM32H753xx)  || defined (STM32H750xx) || defined (STM32H742xx) || \
    defined (STM32H745xx) || defined (STM32H755xx)  || defined (STM32H747xx) || defined (STM32H757xx) || \
    defined (STM32H7A3xx) || defined (STM32H7A3xxQ) || defined (STM32H7B3xx) || defined (STM32H7B3xxQ) || defined (STM32H7B0xx)  || defined (STM32H7B0xxQ) || \
    defined (STM32H735xx) || defined (STM32H733xx)  || defined (STM32H730xx) || defined (STM32H730xxQ)  || defined (STM32H725xx) || defined (STM32H723xx)
#include "stm32h7xx_hal.h"
#define STM32H7
#endif

// define function
#define GPIO_H(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define GPIO_L(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)

//0. Comm Type
#define NERVE_COMM 1

//1. Specials board options
#define STM32H7_V2 1

//2. Enable Flags
//* Please set/reset follwing flags according to your utility.

//2.1 Sensors
//2.1.1 IMU Sensor
#define IMU_FLAG 1
#define IMU_ICM 1
#define IMU_MPU 0
//2.1.2 Barometer Sensor
#define BARO_FLAG 1
//2.1.3 GPS Sensor
#define GPS_FLAG 0
//2.1.3 Direct Servo Control
#define SERVO_FLAG 1
#define DYNAMIXEL 1
#define KONDO 0
//2.1.3.1 Dynamixel Servo Control without external convertor board
#define DYNAMIXEL_BOARDLESS_CONTROL 0
//2.2 State Estimate
//2.2.1 Attitude Estimate
#define ATTITUDE_ESTIMATE_FLAG 1
//* Do not change following code!!!
///////////////////////////////////
#if !IMU_FLAG
#undef ATTITUDE_ESTIMATE_FLAG
#define  ATTITUDE_ESTIMATE_FLAG 0
#endif
///////////////////////////////////

//2.2.2 Height Estimate
#define HEIGHT_ESTIMATE_FLAG 1
//* Do not change following code!!!
///////////////////////////////////
#if !IMU_FLAG || !BARO_FLAG || !ATTITUDE_ESTIMATE_FLAG
#undef HEIGHT_ESTIMATE_FLAG
#define HEIGHT_ESTIMATE_FLAG 0
#endif
///////////////////////////////////

//2.2.3 Pos Estimate
#define POS_ESTIMATE_FLAG 1
//* Do not change following code!!!
///////////////////////////////////
#if !IMU_FLAG || !GPS_FLAG || !ATTITUDE_ESTIMATE_FLAG
#undef POS_ESTIMATE_FLAG
#define POS_ESTIMATE_FLAG 0
#endif
///////////////////////////////////

//2.3 Flight Control
#define FLIGHT_CONTROL_FLAG 1
#define DSHOT 0
//* Do not change following code!!!
///////////////////////////////////
#if !ATTITUDE_ESTIMATE_FLAG
#undef FLIGHT_CONTROL_FLAG
#define FLIGHT_CONTROL_FLAG 0
#endif
///////////////////////////////////



#endif //__CONFIG_H
