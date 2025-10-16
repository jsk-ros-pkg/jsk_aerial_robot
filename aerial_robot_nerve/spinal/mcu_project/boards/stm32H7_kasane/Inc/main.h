/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// #define USE_ETH // comment out if we do not have eth
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_nCS_Pin GPIO_PIN_13
#define IMU_nCS_GPIO_Port GPIOC
#define IMU_SCK_Pin GPIO_PIN_2
#define IMU_SCK_GPIO_Port GPIOE
#define IFCAN1_TX_Pin GPIO_PIN_9
#define IFCAN1_TX_GPIO_Port GPIOB
#define IFI2C1_SDA_Pin GPIO_PIN_7
#define IFI2C1_SDA_GPIO_Port GPIOB
#define IFSPI_MISO_Pin GPIO_PIN_4
#define IFSPI_MISO_GPIO_Port GPIOB
#define IFI2S_WS_Pin GPIO_PIN_15
#define IFI2S_WS_GPIO_Port GPIOA
#define BARO_CS_Pin GPIO_PIN_3
#define BARO_CS_GPIO_Port GPIOE
#define IFI2C1_SCL_Pin GPIO_PIN_8
#define IFI2C1_SCL_GPIO_Port GPIOB
#define IFCAN2_TX_Pin GPIO_PIN_6
#define IFCAN2_TX_GPIO_Port GPIOB
#define IFUART2_TX_Pin GPIO_PIN_5
#define IFUART2_TX_GPIO_Port GPIOD
#define IFSPI_nCS_Pin GPIO_PIN_2
#define IFSPI_nCS_GPIO_Port GPIOD
#define IFI2S_WSC11_Pin GPIO_PIN_11
#define IFI2S_WSC11_GPIO_Port GPIOC
#define IFI2S_CK_Pin GPIO_PIN_10
#define IFI2S_CK_GPIO_Port GPIOC
#define IFUART7_TX_Pin GPIO_PIN_1
#define IFUART7_TX_GPIO_Port GPIOE
#define IFCAN2_RX_Pin GPIO_PIN_5
#define IFCAN2_RX_GPIO_Port GPIOB
#define IFUART2_RX_Pin GPIO_PIN_6
#define IFUART2_RX_GPIO_Port GPIOD
#define WRITE_AUX_Pin GPIO_PIN_3
#define WRITE_AUX_GPIO_Port GPIOD
#define IFUART4_TX_Pin GPIO_PIN_12
#define IFUART4_TX_GPIO_Port GPIOC
#define IFUART1_TX_Pin GPIO_PIN_9
#define IFUART1_TX_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_5
#define IMU_MISO_GPIO_Port GPIOE
#define IFUART7_RX_Pin GPIO_PIN_0
#define IFUART7_RX_GPIO_Port GPIOE
#define IFSPI_MOSI_Pin GPIO_PIN_7
#define IFSPI_MOSI_GPIO_Port GPIOD
#define IFCAN1_RX_Pin GPIO_PIN_0
#define IFCAN1_RX_GPIO_Port GPIOD
#define IFI2C2_SCL_Pin GPIO_PIN_8
#define IFI2C2_SCL_GPIO_Port GPIOA
#define IFUART1_RX_Pin GPIO_PIN_10
#define IFUART1_RX_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_6
#define IMU_MOSI_GPIO_Port GPIOE
#define IFUART3_TX_Pin GPIO_PIN_1
#define IFUART3_TX_GPIO_Port GPIOD
#define IFI2C2_SDA_Pin GPIO_PIN_9
#define IFI2C2_SDA_GPIO_Port GPIOC
#define FTDI_UARTRX_Pin GPIO_PIN_7
#define FTDI_UARTRX_GPIO_Port GPIOC
#define IFETH_MDC_Pin GPIO_PIN_1
#define IFETH_MDC_GPIO_Port GPIOC
#define ADC_Press_Pin GPIO_PIN_3
#define ADC_Press_GPIO_Port GPIOC
#define IFIO1_Pin GPIO_PIN_8
#define IFIO1_GPIO_Port GPIOC
#define FTDI_UARTTX_Pin GPIO_PIN_6
#define FTDI_UARTTX_GPIO_Port GPIOC
#define IFETH_RCD0_Pin GPIO_PIN_4
#define IFETH_RCD0_GPIO_Port GPIOC
#define IFI2S_SDO_Pin GPIO_PIN_2
#define IFI2S_SDO_GPIO_Port GPIOB
#define IFPWM1_Pin GPIO_PIN_14
#define IFPWM1_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOD
#define IFIO2_Pin GPIO_PIN_11
#define IFIO2_GPIO_Port GPIOD
#define IFUSB_DP_Pin GPIO_PIN_15
#define IFUSB_DP_GPIO_Port GPIOB
#define IFETH_REF_CLK_Pin GPIO_PIN_1
#define IFETH_REF_CLK_GPIO_Port GPIOA
#define IFSPI_CLK_Pin GPIO_PIN_5
#define IFSPI_CLK_GPIO_Port GPIOA
#define IFETH_RCD1_Pin GPIO_PIN_5
#define IFETH_RCD1_GPIO_Port GPIOC
#define IFIO3_Pin GPIO_PIN_10
#define IFIO3_GPIO_Port GPIOD
#define IFUSB_DM_Pin GPIO_PIN_14
#define IFUSB_DM_GPIO_Port GPIOB
#define IFETH_MDIO_Pin GPIO_PIN_2
#define IFETH_MDIO_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOA
#define IFUART5_TX_Pin GPIO_PIN_8
#define IFUART5_TX_GPIO_Port GPIOE
#define IFPWM2_Pin GPIO_PIN_10
#define IFPWM2_GPIO_Port GPIOB
#define IFETH_TXD1_Pin GPIO_PIN_13
#define IFETH_TXD1_GPIO_Port GPIOB
#define IFIO4_Pin GPIO_PIN_9
#define IFIO4_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOD
#define IFETH_CRS_DV_Pin GPIO_PIN_7
#define IFETH_CRS_DV_GPIO_Port GPIOA
#define IFPWM3_Pin GPIO_PIN_1
#define IFPWM3_GPIO_Port GPIOB
#define IFETH_TX_EN_Pin GPIO_PIN_11
#define IFETH_TX_EN_GPIO_Port GPIOB
#define IFETH_TXD0_Pin GPIO_PIN_12
#define IFETH_TXD0_GPIO_Port GPIOB
#define IFUART6_TX_Pin GPIO_PIN_8
#define IFUART6_TX_GPIO_Port GPIOD
#define IFPWM4_Pin GPIO_PIN_12
#define IFPWM4_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
