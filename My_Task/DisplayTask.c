#include "DisplayTask.h"

#include "cmsis_os2.h"

void StartDisplayTask(void *argument)
{
    (void)argument;

    for (;;) {
        DisplayTask_Process();
        osDelay(100U);
    }
}

__weak void DisplayTask_Process(void)
{
}
