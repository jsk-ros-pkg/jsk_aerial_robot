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
typedef struct // same type defined in can_device_manager.h whcih has C++ description thus cannot loaded in this C file
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[64];
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
osThreadId coreTaskHandle;
osThreadId rosSpinTaskHandle;
osThreadId idleTaskHandle;
osThreadId canRxHandle;
osThreadId rosPublishHandle;
osTimerId coreTaskTimerHandle;
osMutexId rosPubMutexHandle;
osSemaphoreId coreTaskSemHandle;
osSemaphoreId uartTxSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void coreTaskFunc(void const * argument);
void rosSpinTaskFunc(void const * argument);
void idleTaskFunc(void const * argument);
void canRxTask(void const * argument);
void rosPublishTask(void const * argument);
void coreTaskEvokeCb(void const * argument);

extern void MX_LWIP_Init(void);
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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of coreTaskSem */
  osSemaphoreDef(coreTaskSem);
  coreTaskSemHandle = osSemaphoreCreate(osSemaphore(coreTaskSem), 1);

  /* definition and creation of uartTxSem */
  osSemaphoreDef(uartTxSem);
  uartTxSemHandle = osSemaphoreCreate(osSemaphore(uartTxSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of coreTaskTimer */
  osTimerDef(coreTaskTimer, coreTaskEvokeCb);
  coreTaskTimerHandle = osTimerCreate(osTimer(coreTaskTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(coreTaskTimerHandle, 1); // 1 ms (1kHz)

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* add mail queue for CAN RX */
  osMailQDef(CanMail, 10, can_msg); // defualt: 20 for 8 data bytes (in case of initializer sendBoardConfig (4 servo: 1 + 4 x 3 = 13 packets)); 10 for 64 data bytes
  canMsgMailHandle = osMailCreate(osMailQ(CanMail), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of coreTask */
  osThreadDef(coreTask, coreTaskFunc, osPriorityRealtime, 0, 512);
  coreTaskHandle = osThreadCreate(osThread(coreTask), NULL);

  /* definition and creation of rosSpinTask */
  osThreadDef(rosSpinTask, rosSpinTaskFunc, osPriorityNormal, 0, 256);
  rosSpinTaskHandle = osThreadCreate(osThread(rosSpinTask), NULL);

  /* definition and creation of idleTask */
  osThreadDef(idleTask, idleTaskFunc, osPriorityIdle, 0, 128);
  idleTaskHandle = osThreadCreate(osThread(idleTask), NULL);

  /* definition and creation of canRx */
  osThreadDef(canRx, canRxTask, osPriorityRealtime, 0, 256);
  canRxHandle = osThreadCreate(osThread(canRx), NULL);

  /* definition and creation of rosPublish */
  osThreadDef(rosPublish, rosPublishTask, osPriorityBelowNormal, 0, 128);
  rosPublishHandle = osThreadCreate(osThread(rosPublish), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_coreTaskFunc */
/**
* @brief Function implementing the coreTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_coreTaskFunc */
__weak void coreTaskFunc(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN coreTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END coreTaskFunc */
}

/* USER CODE BEGIN Header_rosSpinTaskFunc */
/**
* @brief Function implementing the rosSpinTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rosSpinTaskFunc */
__weak void rosSpinTaskFunc(void const * argument)
{
  /* USER CODE BEGIN rosSpinTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rosSpinTaskFunc */
}

/* USER CODE BEGIN Header_idleTaskFunc */
/**
* @brief Function implementing the idleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_idleTaskFunc */
__weak void idleTaskFunc(void const * argument)
{
  /* USER CODE BEGIN idleTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(1000);
  }
  /* USER CODE END idleTaskFunc */
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

/* coreTaskEvokeCb function */
__weak void coreTaskEvokeCb(void const * argument)
{
  /* USER CODE BEGIN coreTaskEvokeCb */

  /* USER CODE END coreTaskEvokeCb */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
