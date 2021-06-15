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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct // same type defined in can_device_manager.h which has C++ description thus cannot loaded in this C file
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
osThreadId idleTaskHandle;
osThreadId coreTaskHandle;
osThreadId canRxTaskHandle;
osThreadId canTxTaskHandle;
osThreadId servoTaskHandle;
osTimerId coreTimerHandle;
osMutexId servoMutexHandle;
osSemaphoreId coreTaskSemHandle;
osSemaphoreId canTxSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void idleTaskCallback(void const * argument);
void coreTaskCallback(void const * argument);
void canRxCallback(void const * argument);
void canTxCallback(void const * argument);
void servoTaskCallback(void const * argument);
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
  /* definition and creation of servoMutex */
  osMutexDef(servoMutex);
  servoMutexHandle = osMutexCreate(osMutex(servoMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of coreTaskSem */
  osSemaphoreDef(coreTaskSem);
  coreTaskSemHandle = osSemaphoreCreate(osSemaphore(coreTaskSem), 1);

  /* definition and creation of canTxSem */
  osSemaphoreDef(canTxSem);
  canTxSemHandle = osSemaphoreCreate(osSemaphore(canTxSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of coreTimer */
  osTimerDef(coreTimer, coreEvokeCallback);
  coreTimerHandle = osTimerCreate(osTimer(coreTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(coreTimerHandle, 1); // 1 ms (1kHz)
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* add mail queue for CAN RX */
  osMailQDef(CanMail, 20, can_msg); // defualt: 20 (in case of initializer sendBoardConfig (4 servo: 1 + 4 x 3 = 13 packets))
  canMsgMailHandle = osMailCreate(osMailQ(CanMail), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of idleTask */
  osThreadDef(idleTask, idleTaskCallback, osPriorityIdle, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* definition and creation of coreTask */
  osThreadDef(coreTask, coreTaskCallback, osPriorityRealtime, 0, 256);
  coreTaskHandle = osThreadCreate(osThread(coreTask), NULL);

  /* definition and creation of canRxTask */
  osThreadDef(canRxTask, canRxCallback, osPriorityRealtime, 0, 256);
  canRxTaskHandle = osThreadCreate(osThread(canRxTask), NULL);

  /* definition and creation of canTxTask */
  osThreadDef(canTxTask, canTxCallback, osPriorityAboveNormal, 0, 128);
  canTxTaskHandle = osThreadCreate(osThread(canTxTask), NULL);

  /* definition and creation of servoTask */
  osThreadDef(servoTask, servoTaskCallback, osPriorityNormal, 0, 512);
  servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_idleTaskCallback */
/**
  * @brief  Function implementing the idleTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_idleTaskCallback */
__weak void idleTaskCallback(void const * argument)
{
  /* USER CODE BEGIN idleTaskCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END idleTaskCallback */
}

/* USER CODE BEGIN Header_coreTaskCallback */
/**
* @brief Function implementing the coreTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_coreTaskCallback */
__weak void coreTaskCallback(void const * argument)
{
  /* USER CODE BEGIN coreTaskCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END coreTaskCallback */
}

/* USER CODE BEGIN Header_canRxCallback */
/**
* @brief Function implementing the canRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canRxCallback */
__weak void canRxCallback(void const * argument)
{
  /* USER CODE BEGIN canRxCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canRxCallback */
}

/* USER CODE BEGIN Header_canTxCallback */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canTxCallback */
__weak void canTxCallback(void const * argument)
{
  /* USER CODE BEGIN canTxCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END canTxCallback */
}

/* USER CODE BEGIN Header_servoTaskCallback */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servoTaskCallback */
__weak void servoTaskCallback(void const * argument)
{
  /* USER CODE BEGIN servoTaskCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servoTaskCallback */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
