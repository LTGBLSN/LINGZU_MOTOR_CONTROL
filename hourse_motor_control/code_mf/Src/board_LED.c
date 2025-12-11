//
// Created by 21481 on 2025/8/31.
//
#include "main.h"
#include "cmsis_os.h"
void board_LED()
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
        osDelay(100);
        osDelay(1);
    }
}
