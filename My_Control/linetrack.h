#ifndef __LINETRACK_H
#define __LINETRACK_H

#include <stdint.h>

typedef enum Car_State
{
    CAR_STATE_TRACKING=0,  // 正常循迹
    CAR_STATE_TURN_LEFT,     // 左转并重新找线
    CAR_STATE_TURN_RIGHT,
    CAR_STATE_STOP,
}Car_State;

typedef enum Tracking_Event
{
    TRACK_EVENT_NONE = 0,
    TRACK_EVENT_TURN_LEFT,
    TRACK_EVENT_TURN_RIGHT,
    TRACK_EVENT_LINE_REACQUIRED,
} TrackingEvent;

typedef struct Tracker_t
{
    volatile Car_State car_state;
    volatile uint8_t lap;
} Tracker_t;


/* 循迹模块初始化 */
void Tracking_Init(void);
/* 开始循迹，启动 */
void Tracking_Start(void);
/* 停止循迹，停车 */
void Tracking_Stop(void);

void Tracking_Toggle(void);

/* 
    扫描线路并发布状态转换事件
 */
TrackingEvent Tracking_Update(void);
uint8_t Tracking_GetLap(void);
void Tracking_ClearLap(void);


#endif
