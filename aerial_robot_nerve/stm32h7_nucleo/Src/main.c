/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "lwip.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/udp.h" /* can not find udp API from lwip.h, why */
#include "tcp_echoserver.h"
#include "udp_echoserver.h"
#include "app_ethernet.h"

#include <cstring>
#include <string>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/Imu.h>

/* Internal Communication System */
#include <flashmemory/flashmemory.h>
#include <Spine/spine.h>
#include <CANDevice/test/canfd_test.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
StateEstimate estimator_;

uint32_t msTick = 0;
uint32_t msTickPrevious = 0;

bool start_processing_flag_ = false; //to prevent systick_callback starting  beforeinit() processes
ros::NodeHandle nh_;

std_msgs::String test_msg_;
spinal::Imu imu_msg_;
ros::Publisher test_pub_("/imu", &imu_msg_);
ros::Publisher test_pub2_("/test_pub1", &test_msg_);
ros::Publisher test_pub3_("/test_pub2", &test_msg_);
void testCallback(const std_msgs::Empty& msg);
ros::Subscriber<std_msgs::Empty> test_sub_("/test_sub", &testCallback);
void test2Callback(const spinal::ServoControlCmd& msg);
ros::Subscriber<spinal::ServoControlCmd> test2_sub_("/target_servo_sub", &test2Callback);

uint32_t servo_msg_cnt = 0;
uint32_t max_cnt_diff = 0;
uint32_t servo_msg_recv_t = 0;
uint32_t servo_msg_echo_t = 0;
uint32_t max_du = 0;
uint32_t min_du = 1e6;

CANFDTest canfd_test_;
bool canfd_test_mode_ = false;
bool canfd_send_tx_ = false;
bool uart_mode_ = false;

extern osSemaphoreId coreTaskSemHandle;
extern osMailQId canMsgMailHandle;
extern osMutexId rosPubMutexHandle;
extern osSemaphoreId uartTxSemHandle;

extern ETH_HandleTypeDef heth;

extern "C"
{
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void testCallback(const std_msgs::Empty& msg)
{
  test_msg_.data = std::string("sub echo ok").c_str();
  test_pub2_.publish(&test_msg_);
}

void test2Callback(const spinal::ServoControlCmd& msg)
{
  uint32_t cnt = (uint32_t)msg.index[0] + ((uint32_t)msg.index[1] << 8) + ((uint32_t)msg.index[2] << 16) + ((uint32_t)msg.index[3] << 24);

  if( abs((int32_t)servo_msg_cnt - (int32_t)cnt) > 100) // 100 topics
    {
      // reset
      servo_msg_recv_t = 0;
      servo_msg_echo_t = 0;
      servo_msg_cnt = cnt;

      max_du = 0;
      min_du = 1e6;

      max_cnt_diff = 0;
    }

  if(servo_msg_recv_t == 0)
    {
      servo_msg_recv_t = HAL_GetTick();
      servo_msg_echo_t = HAL_GetTick();
    }
  else
    {
      uint32_t du = HAL_GetTick() - servo_msg_recv_t;
      if(max_du < du) max_du = du;
      if(min_du > du) min_du = du;

      uint32_t cnt_diff = cnt - servo_msg_cnt;
      if(max_cnt_diff < cnt_diff) max_cnt_diff = cnt_diff;

      if (HAL_GetTick() - servo_msg_echo_t > 1000) // 1000 ms
        {
          char buffer[50];
          sprintf (buffer, "%d, %d, %d, %d, %d", du, max_du, min_du, cnt_diff, max_cnt_diff);
          nh_.logerror(buffer);
          servo_msg_echo_t = HAL_GetTick();
        }
      servo_msg_recv_t = HAL_GetTick();
      servo_msg_cnt= cnt;
    }
}

extern "C"
{
  // timer callback to evoke coreTask at 1KHz
  void coreTaskEvokeCb(void const * argument)
  {
    osSemaphoreRelease (coreTaskSemHandle);
  }

  // main subrutine for update enach instance
  void coreTaskFunc(void const * argument)
  {
    MX_LWIP_Init();

    /* ros node init */
    if(uart_mode_)
      {
        nh_.initNode(&huart2, &rosPubMutexHandle, &uartTxSemHandle);
        //nh_.initNode(&huart2, &rosPubMutexHandle, NULL); // no DMA for TX
      }
    else
      {
        ip4_addr_t dst_addr;
        IP4_ADDR(&dst_addr,192,168,25,100);
        nh_.initNode(dst_addr, 12345,12345);
      }

    nh_.advertise(test_pub_);
    nh_.advertise(test_pub2_);
    nh_.advertise(test_pub3_);
    nh_.subscribe(test_sub_);
    nh_.subscribe(test2_sub_);

    osSemaphoreWait(coreTaskSemHandle, osWaitForever);

    int imu_pub_interval = 0;
    if (uart_mode_) imu_pub_interval = 5;
    uint32_t imu_pub_t = HAL_GetTick();
    uint32_t can_test_t = HAL_GetTick();

    for(;;)
      {
        osSemaphoreWait(coreTaskSemHandle, osWaitForever);

        if (!canfd_test_mode_) Spine::send();
        if (HAL_GetTick() - imu_pub_t >= imu_pub_interval)
          {
            imu_msg_.stamp = nh_.now();
            test_pub_.publish(&imu_msg_);
            imu_pub_t = HAL_GetTick();
          }
        if (!canfd_test_mode_) Spine::update();

        if (canfd_test_mode_)
          {
            CANDeviceManager::tick(1);
            if (HAL_GetTick() - can_test_t > 100) // 10ms
              {
                if (canfd_send_tx_)
                  canfd_test_.sendData();
                can_test_t = HAL_GetTick();
              }
          }

        // Workaround to handle the BUSY->TIMEOUT Error problem of ETH handler in STM32H7
        // We observe this is occasionally occur, but the ETH DMA is valid.
        if (heth.ErrorCode & HAL_ETH_ERROR_TIMEOUT)
          {
            // force to restart ETH transmit
            heth.gState = HAL_ETH_STATE_READY;
            ETH_TxDescListTypeDef *dmatxdesclist = &(heth.TxDescList);
            for (uint32_t i = 0; i < (uint32_t)ETH_TX_DESC_CNT; i++)
              {
                ETH_DMADescTypeDef *dmatxdesc = (ETH_DMADescTypeDef *)dmatxdesclist->TxDesc[i];
                CLEAR_BIT(dmatxdesc->DESC3, ETH_DMATXNDESCRF_OWN);
              }
          }
      }
  }

  void rosSpinTaskFunc(void const * argument)
  {
    for(;;)
      {
        /* ros spin means to get data from UART RX ring buffer and to call callback functions  */
        if(nh_.spinOnce() == ros::SPIN_UNAVAILABLE)
          {
            /* if no data in ring buffer, we kindly sleep for 1ms */
            osDelay(1);
          }
      }
  }

  void rosPublishTask(void const * argument)
  {
    for(;;)
      {
        /* publish one message from ring buffer */
        if(nh_.publish() == BUFFER_EMPTY)
          {
            /* if no messages in ring buffer, we kindly sleep for 1ms */
            osDelay(1);
          }
      }
  }

  void idleTaskFunc(void const * argument)
  {
    for(;;)
      {
        if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
          {
            canfd_send_tx_ = true;
          }

        if (canfd_test_mode_) HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(1000);
      }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_RNG_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_RTC_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

 /* Flash Memory */
  FlashMemory::init(0x081E0000, FLASH_SECTOR_7);
  // Bank2, Sector7: 0x081E 0000 (128KB); https://www.stmcu.jp/download/?dlid=51599_jp
  // BANK1 (with Sector7) cuases the flash failure by STLink from the second time. So we use BANK_2, which is tested OK

  // It semms like the first 64 byte of flashmemory for data is easily to be overwritten by zero, when re-flash the flash memory.
  // A possible reason is the power supply from stlink does not contain 3.3V output.
  // which is the only difference compared with the old version (STM32F7)
  // So, we introduce following dummy data for a workaround to avoid the vanishment of stored data in flash memory.
  uint8_t dummy_data[64];
  memset(dummy_data, 1, 64);
  FlashMemory::addValue(dummy_data, 64);


  /* switch between ETH/UART rosserial */
  if(HAL_GPIO_ReadPin(UART_MODE_GPIO_Port, UART_MODE_Pin) == GPIO_PIN_SET)
    {
      uart_mode_ = true;
    }

  /* switch CAN/CANFD test mode */
  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    {
      canfd_test_mode_ = true;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

      while (1)
        {
          if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET)
            {
              HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
              break;
            }

          HAL_Delay(100);
        }
    }

  /* NERVE */
  /* TODO: should be called in coreTask, but we have to replace all the HAL_Delay() to os Delay() */
  if(!canfd_test_mode_)
    {
      Spine::init(&hfdcan1, &nh_, &estimator_, LD1_GPIO_Port, LD1_Pin);
      Spine::useRTOS(&canMsgMailHandle); // use RTOS for CAN in spianl
    }
  else
    {
      CANDeviceManager::init(&hfdcan1, LD1_GPIO_Port, LD1_Pin);
      CANDeviceManager::useRTOS(&canMsgMailHandle);
      CANDeviceManager::addDevice(canfd_test_);
      CANDeviceManager::CAN_START();
    }


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 100;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 10;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ETH_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ETH_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);
  /* ETH_WKUP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);
  /* FDCAN1_IT0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x24040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x24040400;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
