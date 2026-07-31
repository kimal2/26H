#include "DisplayTask.h"
#include "OLED.h"
#include "mpu6050.h"
#include "LineSensor.h"

#include "cmsis_os2.h"

void StartDisplayTask(void *argument)
{
    (void)argument;

    for (;;) {
        DisplayTask_Process();
        osDelay(20U);
    }
}

__weak void DisplayTask_Process(void)
{
    OLED_Printf(0,0,8,"x %.2f",imu.acc.x);
    OLED_Printf(0,1,8,"y %.2f",imu.acc.y);
    uint8_t dig = LineSensor_GetBit();
    OLED_Printf(0,3,8,"%d-%d-%d-%d-%d-%d-%d-%d",(dig>>0)&0x01,(dig>>1)&0x01,
                                                (dig>>2)&0x01,(dig>>3)&0x01,
                                                (dig>>4)&0x01,(dig>>5)&0x01,
                                                (dig>>6)&0x01,(dig>>7)&0x01);
    
    
}
