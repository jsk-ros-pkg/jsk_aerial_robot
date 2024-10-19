/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* common setting for this board */
#include "config.h"

/* ros communication interface between board and PC */
#include <ros.h>

#include "flashmemory/flashmemory.h"

/* Sensors */
#if IMU_FLAG
#include "sensors/imu/imu_ros_cmd.h"
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#endif

#if BARO_FLAG
#include "sensors/baro/baro_ms5611.h"
#endif

#if GPS_FLAG
#include "sensors/gps/gps_ublox.h"
#endif

#include "sensors/encoder/mag_encoder.h"

/* State Estimate, including attitude, altitude and pos */
#if ATTITUDE_ESTIMATE_FLAG || HEIGHT_ESTIMATE_FLAG || POS_ESTIMATE_FLAG
#include "state_estimate/state_estimate.h"
#endif

/* State Estimate, including attitude, altitude and pos */
#if FLIGHT_CONTROL_FLAG
#include "flight_control/flight_control.h"
#endif

/* Battery Status */
#include "battery_status/battery_status.h"

/* Extra Servo */
#include "extra_servo/extra_servo.h"
#include "kondo_servo/kondo_servo.h"


/* Internal Communication System */
#include <Spine/spine.h>

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
bool start_processing_flag_ = false; //to prevent systick_callback starting  beforeinit() processes

ros::NodeHandle nh_;

#if IMU_MPU
IMUOnboard imu_;
#elif IMU_ICM
ICM20948 imu_;
#endif

#if BARO_FLAG
Baro baro_;
#endif

#if GPS_FLAG
GPS gps_;
#endif

MagEncoder encoder_;

#if ATTITUDE_ESTIMATE_FLAG || HEIGHT_ESTIMATE_FLAG || POS_ESTIMATE_FLAG
StateEstimate estimator_;
#endif

#if FLIGHT_CONTROL_FLAG
BatteryStatus battery_status_;
FlightControl controller_;
ExtraServo extra_servo_;
KondoServo kondo_servo_;

#endif

// defined in Src/freertos.c
extern osSemaphoreId CoreProcessSemHandle;
extern osSemaphoreId Uart1TxSemHandle;
extern osMailQId canMsgMailHandle;
extern osMutexId rosPubMutexHandle;
extern osMutexId flightControlMutexHandle;

extern "C"
{
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

  // timer callback to evoke coreTask at 1KHz
  void coreEvokeCallback(void const * argument)
  {
    if(FlashMemory::isLock()) return; // avoid overflow of ros publish buffer in rosserial UART mode
    /*
      We need to create another indipendent task to run the core function (following coreTask),
      since block action in timer callback function is not allowed.
    */
    osSemaphoreRelease (CoreProcessSemHandle);
  }

  // main subrutine for update enach instance
  void coreTask(void const * argument)
  {
    imu_.gyroCalib(true, IMU::GYRO_DEFAULT_CALIB_DURATION); // re-calibrate gyroscope because of the HAL_Delay in spine init

    for(;;)
      {
        // RTOS semephre
        osSemaphoreWait(CoreProcessSemHandle, osWaitForever);

        if(!start_processing_flag_) return;

#if NERVE_COMM
        Spine::send();
#endif

        /* sensors */
#if IMU_FLAG
        imu_.update();
#endif

#if BARO_FLAG
        baro_.update();
#endif

#if GPS_FLAG
        gps_.update();
#endif

        encoder_.update();

        /* state estimate */
#if (ATTITUDE_ESTIMATE_FLAG || HEIGHT_ESTIMATE_FLAG || POS_ESTIMATE_FLAG)
        estimator_.update();
#endif

#if FLIGHT_CONTROL_FLAG
        controller_.update();
#endif

#if NERVE_COMM
        Spine::update();
#endif
      }
  }

  void rosSpinTask(void const * argument)
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

  /*
   * We set a lower priority for publish task than rosSpinTask
   * Since,  UART_Transmit needs a timeout to wait for the flag of UART TX to be ready (about 2 ms for about 100bytes)
   * - HAL_UART_Transmit(tx_huart_, tx_buffer_unit_[subscript_in_progress_].tx_data_, tx_buffer_unit_[subscript_in_progress_].tx_len_, 2);
   * TODO: use DMA to send UART TX
   */
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

#if 0
  void StartDefaultTask(void const * argument)
  {
    for(;;)
      {
        osDelay(1000);
        LED2_H;
        osDelay(1000);
        LED2_L;
      }
  }
#endif 
  
#if FLIGHT_CONTROL_FLAG
  void voltageTask(void const * argument)
  {
    for(;;)
      {
        battery_status_.update();
        osDelay(VOLTAGE_CHECK_INTERVAL);
      }
  }
#endif

  void kondoServoTaskCallback(void const * argument)
  {
	  for(;;)
	  {
		  kondo_servo_.update();
		  osDelay(KONDO_SERVO_UPDATE_INTERVAL);
	  }
  }

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  /* ROS Interface between board and PC */
  nh_.initNode(&huart1, &rosPubMutexHandle, &Uart1TxSemHandle);

  /* Flash Memory */
  FlashMemory::init(0x08080000, FLASH_SECTOR_6);
  // Sector6: 0x0808 0000 (256KB); https://www.stmcu.jp/download/?dlid=51585_jp

  /* Sensors */
#if IMU_FLAG
  /* we need to put IMU second, because of the unknown reason abou the ublox m8n compass/gps disconnect problem */
  imu_.init(&hspi1, &hi2c2, &nh_, IMUCS_GPIO_Port, IMUCS_Pin, LED0_GPIO_Port, LED0_Pin);
  IMU_ROS_CMD::init(&nh_);
  IMU_ROS_CMD::addImu(&imu_);
#endif

#if BARO_FLAG
  baro_.init(&hi2c1, &nh_, BAROCS_GPIO_Port, BAROCS_Pin);
#endif


  /* State Estimation */
#if ATTITUDE_ESTIMATE_FLAG //imu condition

#if HEIGHT_ESTIMATE_FLAG // baro condition

#if POS_ESTIMATE_FLAG // gps condition
  estimator_.init(&imu_, &baro_, &gps_, &nh_);  // imu + baro + gps => att + alt + pos(xy)
#else  //gps condition 
  estimator_.init(&imu_, &baro_, NULL, &nh_);  // imu + baro  => att + alt
#endif // gps condition

#else // baro condition
  estimator_.init(&imu_, NULL, NULL, &nh_);  // imu  => att
#endif  // baro condition

#endif // imu condition

  /* Extra Servo Control */
  extra_servo_.init(&htim3, &htim5, &nh_);
  kondo_servo_.init(&huart3, &nh_);

  /* Magnetic Encoder */
  encoder_.init(&hi2c2, &nh_);

#if NERVE_COMM
  /* NERVE */
  Spine::init(&hcan1, &nh_, &estimator_, LED1_GPIO_Port, LED1_Pin);
#endif

#if FLIGHT_CONTROL_FLAG
  /* BATTERY_STATUS */
  battery_status_.init(&hadc2, &nh_);
  /* Start Attitude Control */
  controller_.init(&htim4, &htim8, &estimator_, &kondo_servo_, &battery_status_, &nh_, &flightControlMutexHandle);

#if NERVE_COMM
  controller_.setUavModel(Spine::getUavModel());
  controller_.setMotorNumber(Spine::getSlaveNum());
#endif

#if GPS_FLAG
  gps_.init(&huart3, &nh_, LED2_GPIO_Port, LED2_Pin);
#endif

#endif

  FlashMemory::read(); //IMU calib data (including IMU in neurons)

  start_processing_flag_ = true;
  uint32_t now_time = HAL_GetTick();

  Spine::useRTOS(&canMsgMailHandle); // use RTOS for CAN in spianl
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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

