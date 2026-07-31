#include "TubeTask.h"

#include "cmsis_os2.h"

void StartTubeTask(void *argument)
{
    (void)argument;

    for (;;) {
        TubeTask_Process();
        osDelay(10U);
    }
}

__weak void TubeTask_Process(void)
{
}
