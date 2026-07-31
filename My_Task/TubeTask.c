#include "TubeTask.h"
#include "mpu6050.h"

#include "cmsis_os2.h"
#include "My_UartProc.h"
#include "StepMotor_Ctrl.h"

void StartTubeTask(void *argument)
{
    (void)argument;

    StepMotor_Init();
    (void)UartReceiveStart();

    for (;;) {
        TubeTask_Process();
        osDelay(10U);
    }
}

__weak void TubeTask_Process(void)
{
    StepMotor_Task();
    MPU6050_ReadAll(&imu);
    MPU6050_ComputeAttitude(&imu,0.01f);
}
