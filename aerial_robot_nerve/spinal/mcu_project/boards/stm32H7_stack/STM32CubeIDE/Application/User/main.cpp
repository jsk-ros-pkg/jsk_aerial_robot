#include <cstdio>
#include <string>
#include "main.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"

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
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    osDelay(100);
  }
  /* USER CODE END idleTaskFunc */
}
