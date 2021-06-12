/*
******************************************************************************
* File Name          : config.h
* Description        : Basic configuration for this board(c.f. GPIO Macro, Enable flag for different processes
******************************************************************************
*/
#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f7xx_hal.h"

//0. Comm Type
#define NERVE_COMM 1

//1. GPIO Macro
//1.1 LEDS
#define LED0_H       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET)
#define LED0_L      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET)
#define LED1_H       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET)
#define LED1_L      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET)
#define LED2_H       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)
#define LED2_L      HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET)

//1.2 DEBUG
#define DEBUG_H      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)
#define DEBUG_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)

//2. Enable Flags
//* Please set/reset follwing flags according to your utility.

//2.1 Sensors
//2.1.1 IMU Sensor
#define IMU_FLAG 1
//2.1.2 Barometer Sensor
#define BARO_FLAG 1
//2.1.3 GPS Sensor
#define GPS_FLAG 1

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
//* Do not change following code!!!
///////////////////////////////////
#if !ATTITUDE_ESTIMATE_FLAG
#undef FLIGHT_CONTROL_FLAG
#define FLIGHT_CONTROL_FLAG 0
#endif
///////////////////////////////////



#endif //__CONFIG_H
