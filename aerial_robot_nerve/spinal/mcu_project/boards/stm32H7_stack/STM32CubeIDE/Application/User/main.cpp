#include <cstdio>
#include <string>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"
#include "ros.h"

extern UART_HandleTypeDef huart6;

extern osThreadId coreTaskHandle;
extern osThreadId rosSpinTaskHandle;
extern osThreadId idleTaskHandle;
extern osThreadId rosPublishHandle;
extern osThreadId voltageHandle;
extern osTimerId coreTaskTimerHandle;
extern osMutexId rosPubMutexHandle;
extern osMutexId flightControlMutexHandle;
extern osSemaphoreId coreTaskSemHandle;
extern osSemaphoreId uartTxSemHandle;

ros::NodeHandle nh_;

extern "C" void app_main(void)
{

	//while (1)
    //{
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
        HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
        HAL_Delay(500);

    //}
}

extern "C" void idleTaskFunc(void const * argument)
{
  /* USER CODE BEGIN idleTaskFunc */
  for(;;)
  {
    // HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    osDelay(1000);
  }
  /* USER CODE END idleTaskFunc */
}

/* USER CODE END Header_rosPublishTask */
extern "C" void rosSpinTaskFunc(void const * argument)
{
  /* USER CODE BEGIN rosPublishTask */
	//HAL_Half
	const char msg[] = "Hello world\r\n";
  for(;;)
    {
      /* publish one message from ring buffer */
      //if(nh_.publish() == BUFFER_EMPTY)
        //{
          /* if no messages in ring buffer, we kindly sleep for 1ms */
    	  //HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
          //osDelay(1);
        //}
	  HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);

	  if (HAL_UART_Transmit(&huart6, (uint8_t*)msg, (uint16_t)(sizeof(msg)-1), 100) != HAL_OK) {
	      Error_Handler();
	  }
	  osDelay(1000);
  }
  /* USER CODE END rosPublishTask */
}






