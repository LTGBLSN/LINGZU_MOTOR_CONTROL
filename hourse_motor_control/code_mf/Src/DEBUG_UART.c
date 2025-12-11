//
// Created by 21481 on 2025/8/31.
//

#include "cmsis_os.h"
#include "uart_printf.h"
#include "Robstride.h"

extern RobStride_Motor RobStride_01;
void uart_sent()
{
    while (1)
    {

        usart6_printf("Motor Mechanical Speed: %f rad/s\r\n", RobStride_01.Pos_Info.Speed);
        usart6_printf("Motor Mechanical Position: %f rad\r\n", RobStride_01.drw.mechPos);
        usart6_printf("Motor Speed Reference: %f rad/s\r\n", RobStride_01.drw.spd_ref);
        osDelay(10);
    }
}
