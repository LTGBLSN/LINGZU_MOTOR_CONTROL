#include "cmsis_os.h"
#include "Robstride.h"

extern struct RobStride_Motor RobStride_01;
void get_data()//发电流
{
    osDelay(1000);
    Enable_Motor((RobStride_Motor *) &RobStride_01);


    while (1)
    {
        RobStride_Motor_current_control((RobStride_Motor *) &RobStride_01, 1.2f);
        osDelay(10);
    }
}