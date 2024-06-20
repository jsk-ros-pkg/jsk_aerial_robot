/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct // same type defined in can_device_manager.h whcih has C++ description thus cannot loaded in this C file
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
} can_msg;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMailQId canMsgMailHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId rosSpinHandle;
osThreadId rosPublishHandle;
osThreadId coreProcessHandle;
osThreadId canRxHandle;
osThreadId voltageHandle;
osThreadId kondoServoTaskHandle;
osTimerId CoreTimerHandle;
osMutexId rosPubMutexHandle;
osMutexId flightControlMutexHandle;
osSemaphoreId CoreProcessSemHandle;
osSemaphoreId Uart1TxSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void rosSpinTask(void const * argument);
void rosPublishTask(void const * argument);
void coreTask(void const * argument);
void canRxTask(void const * argument);
void voltageTask(void const * argument);
void kondoServoTaskCallback(void const * argument);
void coreEvokeCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of rosPubMutex */
  osMutexDef(rosPubMutex);
  rosPubMutexHandle = osMutexCreate(osMutex(rosPubMutex));

  /* definition and creation of flightControlMutex */
  osMutexDef(flightControlMutex);
  flightControlMutexHandle = osMutexCreate(osMutex(flightControlMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of CoreProcessSem */
  osSemaphoreDef(CoreProcessSem);
  CoreProcessSemHandle = osSemaphoreCreate(osSemaphore(CoreProcessSem), 1);

  /* definition and creation of Uart1TxSem */
  osSemaphoreDef(Uart1TxSem);
  Uart1TxSemHandle = osSemaphoreCreate(osSemaphore(Uart1TxSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of CoreTimer */
  osTimerDef(CoreTimer, coreEvokeCallback);
  CoreTimerHandle = osTimerCreate(osTimer(CoreTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(CoreTimerHandle, 1); // 1 ms (1kHz)
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* add mail queue for CAN RX */
  osMailQDef(CanMail, 20, can_msg); // defualt: 20 (in case of initializer sendBoardConfig (4 servo: 1 + 4 x 3 = 13 packets))
  canMsgMailHandle = osMailCreate(osMailQ(CanMail), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of rosSpin */
  osThreadDef(rosSpin, rosSpinTask, osPriorityNormal, 0, 128);
  rosSpinHandle = osThreadCreate(osThread(rosSpin), NULL);

  /* definition and creation of rosPublish */
  osThreadDef(rosPublish, rosPublishTask, osPriorityBelowNormal, 0, 128);
  rosPublishHandle = osThreadCreate(osThread(rosPublish), NULL);

  /* definition and creation of coreProcess */
  osThreadDef(coreProcess, coreTask, osPriorityRealtime, 0, 256);
  coreProcessHandle = osThreadCreate(osThread(coreProcess), NULL);

  /* definition and creation of canRx */
  osThreadDef(canRx, canRxTask, osPriorityRealtime, 0, 256);
  canRxHandle = osThreadCreate(osThread(canRx), NULL);

  /* definition and creation of voltage */
  osThreadDef(voltage, voltageTask, osPriorityLow, 0, 128);
  voltageHandle = osThreadCreate(osThread(voltage), NULL);

  /* definition and creation of kondoServoTask */
  osThreadDef(kondoServoTask, kondoServoTaskCallback, osPriorityLow, 0, 256);
  kondoServoTaskHandle = osThreadCreate(osThread(kondoServoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_rosSpinTask */
/**
* @brief Function implementing the rosSpin thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rosSpinTask */
__weak void rosSpinTask(void const * argument)
{
  /* USER CODE BEGIN rosSpinTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rosSpinTask */
}

/* USER CODE BEGIN Header_rosPublishTask */
/**
* @brief Function implementing the rosPublish thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rosPublishTask */
__weak void rosPublishTask(void const * argument)
{
  /* USER CODE BEGIN rosPublishTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rosPublishTask */
}

/* USER CODE BEGIN Header_coreTask */
/**
* @brief Function implementing the coreProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_coreTask */
__weak void coreTask(void const * argument)
{
  /* USER CODE BEGIN coreTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END coreTask */
}

/* USER CODE BEGIN Header_canRxTask */
/**
* @brief Function implementing the canRx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canRxTask */
__weak void canRxTask(void const * argument)
{
  /* USER CODE BEGIN canRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canRxTask */
}

/* USER CODE BEGIN Header_voltageTask */
/**
* @brief Function implementing the voltage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_voltageTask */
__weak void voltageTask(void const * argument)
{
  /* USER CODE BEGIN voltageTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END voltageTask */
}

/* USER CODE BEGIN Header_kondoServoTaskCallback */
/**
* @brief Function implementing the kondoServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kondoServoTaskCallback */
__weak void kondoServoTaskCallback(void const * argument)
{
  /* USER CODE BEGIN kondoServoTaskCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END kondoServoTaskCallback */
}

/* coreEvokeCallback function */
__weak void coreEvokeCallback(void const * argument)
{
  /* USER CODE BEGIN coreEvokeCallback */

  /* USER CODE END coreEvokeCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

