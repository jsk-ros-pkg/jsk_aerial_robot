#pragma once

#include <math.h>
#include <stdint.h>
#include "stm32f7xx_hal.h"

#define PI 3.14159265

typedef struct KondoServo
{
	uint8_t id_;
	uint32_t baudrate_;
	uint8_t reverse_;
	UART_HandleTypeDef port_;
	uint8_t current_angle_;
	uint8_t target_angle_;
	uint16_t current_position_;
	uint16_t target_position_;
} kondo_servo_t;


uint8_t getId(kondo_servo_t* const this);
uint32_t getBaudrate(kondo_servo_t* const this);
uint8_t getReverse(kondo_servo_t* const this);
UART_HandleTypeDef getPort(kondo_servo_t* const this);
uint8_t getCurrentAngle(kondo_servo_t* const this);
uint8_t getTargetAngle(kondo_servo_t* const this);

void setAngle(kondo_servo_t* const this, float rad);
