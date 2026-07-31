#include "GrayTrackTask.h"

#include "cmsis_os2.h"
#include "DisplayTask.h"
#include "LineSensor.h"
#include "linetrack.h"

void StartGrayTrackTask(void *argument)
{
    (void)argument;

    while (LineSensor_Init() != 0U) {
        osDelay(1U);
    }
    Tracking_Init();

    for (;;) {
        GrayTrackTask_Process();
        osDelay(10U);
    }
}

__weak void GrayTrackTask_Process(void)
{
    if (Tracking_Update() == TRACK_EVENT_FINISH_LINE) {
        Tracking_Stop();
        DisplayTask_ReportTrackingFinished();
    }
}
