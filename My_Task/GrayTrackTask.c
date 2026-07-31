#include "GrayTrackTask.h"

#include "cmsis_os2.h"
#include "LineSensor.h"
#include "linetrack.h"

void StartGrayTrackTask(void *argument)
{
    (void)argument;

    LineSensor_Init();
    Tracking_Init();

    for (;;) {
        GrayTrackTask_Process();
        osDelay(5U);
    }
}

__weak void GrayTrackTask_Process(void)
{
    (void)Tracking_Update();
}
