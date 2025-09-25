#include <cstdio>
#include <string>
#include "main.h"
#include "stm32h7xx_hal.h"

extern "C" void app_main(void)
{

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        HAL_Delay(500);

    }
}
