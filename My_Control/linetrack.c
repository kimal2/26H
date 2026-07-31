#include "linetrack.h"
#include "LineSensor.h"
#include <stdbool.h>

/* 左转弯计数 1为左转计数 0为右转计数 */
#define TURN_LEFT_COUNT 1

/* =========== 角检测 =========== */
static uint8_t corner_cnt=0;

/* =============================== */
static LineType line_last = LINE_TYPE_UNKNOWN;

/* ============================== */
Tracker_t Tracker;

/* =========== 循迹模块初始化 =========== */
void Tracking_Init(void)
{
    Tracker.car_state = CAR_STATE_STOP;
    Tracker.lap = 0;

    line_last = LINE_TYPE_UNKNOWN;
}

/* 
    更新循迹状态
 */
static TrackingEvent Tracking_UpdateState(void)
{
    /*  
    转弯逻辑：记录当前方向，前进->左转->全白：开始转弯，但是前进->左转会有抖动
    会出现 straight,left,straight,全白 这样的结果，所以判断转弯时要用最后丢失
    的方向来判断转弯方向：当出现左转或右转时，记录下来，然后不管straight抖动，当
    来到全白时则产生转弯指令
    */
    LineType line = LineService_Get_LineType();

    /* 只有左右分支更新记忆。
       STRAIGHT 抖动、LOST、CROSS、UNKNOWN 都保持原方向。 */
    if(line == LINE_TYPE_RIGHT)
    {
        line_last = LINE_TYPE_RIGHT;
    }
    else if(line == LINE_TYPE_LEFT)
    {
        line_last = LINE_TYPE_LEFT;
    }

     /* 正常循迹时，只有 LOST 才产生转弯。 */
    if(Tracker.car_state == CAR_STATE_TRACKING)
    {
        if(line == LINE_TYPE_LOST)
        {
            if(line_last == LINE_TYPE_LEFT)
            {
                Tracker.car_state = CAR_STATE_TURN_LEFT;
                return TRACK_EVENT_TURN_LEFT;
            }
            else if(line_last == LINE_TYPE_RIGHT)
            {
                Tracker.car_state = CAR_STATE_TURN_RIGHT;
                return TRACK_EVENT_TURN_RIGHT;
            }
        }
        return TRACK_EVENT_NONE;
    }

    /* 转弯后重新找到正常线。 */
    if ((Tracker.car_state == CAR_STATE_TURN_LEFT ||
         Tracker.car_state == CAR_STATE_TURN_RIGHT) &&
        line == LINE_TYPE_STRAIGHT)
    {
        Tracker.car_state = CAR_STATE_TRACKING;
        return TRACK_EVENT_LINE_REACQUIRED;
    }

    return TRACK_EVENT_NONE;
}


/* 
    圈数判断，左转或右转5次后视为1圈
    圈数判断成功：Lap++，返回true，否则返回false
*/
static bool Tracking_Lapcount(void)
{
    static Car_State last_car_state;
    static Car_State turn_state;
    
    if(TURN_LEFT_COUNT)
    {
        turn_state=CAR_STATE_TURN_LEFT;
    }
    else
    {
        turn_state = CAR_STATE_TURN_RIGHT;
    }
    if(Tracker.car_state==turn_state&&last_car_state == CAR_STATE_TRACKING)
    {
        corner_cnt++;
    }
    last_car_state = Tracker.car_state;

    if(corner_cnt==5)
    {
        corner_cnt=1;
        Tracker.lap++;
        return true;
    }

    else return false;
}

void Tracking_Toggle(void)
{
    if(Tracker.car_state == CAR_STATE_STOP)
    {
        Tracking_Start();
    }
    else
    {
        Tracking_Stop();
    }
}

/* 开始循迹，启动 */
void Tracking_Start(void)
{
    Tracker.car_state = CAR_STATE_TRACKING;
    line_last = LINE_TYPE_UNKNOWN;
}

/* 停止循迹，停车 */
void Tracking_Stop(void)
{
    Tracker.car_state = CAR_STATE_STOP;
    line_last = LINE_TYPE_UNKNOWN;
}

/* 
    扫描线路并发布状态转换事件
 */
TrackingEvent Tracking_Update(void)
{
    TrackingEvent event;

    LineSensor_Scan();

    //停止时只更新传感器，不发布事件。
    if(Tracker.car_state==CAR_STATE_STOP)
    {
        return TRACK_EVENT_NONE;
    }

    event = Tracking_UpdateState();

    Tracking_Lapcount();
    return event;
}

uint8_t Tracking_GetLap(void)
{
    return Tracker.lap;
}

void Tracking_ClearLap(void)
{
    Tracker.lap = 0;
    corner_cnt=0;
}

Car_State Tracking_GetState(void)
{
    return Tracker.car_state;
}


