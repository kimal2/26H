#include "PTU_Ctrl.h"
#include "usart.h"
#include "tim.h"
#include "Emm_V5.h"
#include <stdint.h>
#include <stdbool.h>
#include "pid.h"
#include "math.h"

#define	ABS(x)	  ((x) > 0 ? (x) : -(x))

#define ACC       0


StepMotor_t M_Pan;
StepMotor_t M_Tilt;

PID_t PID_P={
    .Kp=10.0f,
    .Out_Max=5000.0f,
    .Out_Min=-5000.0f,
};


PID_t PID_T={
    .Kp=10.0f,
    .Out_Max=5000.0f,
    .Out_Min=-5000.0f,
};




void StepMotor_SetSpeed(StepMotor_t *M, float speed)
{
    uint8_t dir = (speed >= 0.0f) ? 0 : 1;
    float vel = (speed >= 0.0f) ? speed : -speed;
    if(vel <= 5000.0f)
    {
        Emm_V5_Vel_Control(M, dir, (uint16_t)vel, ACC, 0);
    } 
    else return;
    
}


void PTU_Init(void)
{
    M_Pan.huart = &huart2;
    M_Pan.ID = 1;

    M_Tilt.huart = &huart4;
    M_Tilt.ID = 2;


    PID_Clear(&PID_P);
    PID_Clear(&PID_T);

    Emm_V5_Origin_Trigger_Return(&M_Pan,0,0);
    Emm_V5_Origin_Trigger_Return(&M_Tilt,0,0);

}

/* 计算得到脉冲值 */
void PTU_SetTarget(float pan_deg, float tilt_deg)
{
    M_Pan.target_angle = pan_deg;
    M_Tilt.target_angle = tilt_deg;
}

void circCal(StepMotor_t *Mp, StepMotor_t *Mt)
{
    static float looptime=2.0f;
    static float R = 0.5f;
    static float theta=0;
    theta += 2*3.14f/looptime/100;
    float vx = 360.0f*R/looptime*cosf(theta);
    float vy =  360.0f*R/looptime*sinf(theta);
    Mp->target_speed =vx;
    Mt->target_speed =vy;
}

void rectCal(PID_t *PID_P, PID_t *PID_T)
{
    static uint8_t state=0;
    static float setangle=30.0f;
    static float looptime=4.0f;
    static float k=0;
    k = setangle/looptime*4;

    switch(state)
    {
        case 0:
        {
            PID_P->target += k/100;
            PID_T->target = 0.0f;
            if(PID_P->actual>=setangle)
            {
                PID_P->target=setangle;
                state=1;
            }
            break;
        }
        case 1:
        {
            //PID_P->target = 90.0f;
            PID_T->target += k/100;
            if(PID_T->actual>=setangle)
            {
                PID_T->target=setangle;
                state=2;
            }
            break;
        }
        case 2:
        {
            PID_P->target -= k/100;
            //PID_T->target = 90.0f;
            if(PID_P->actual<=0.0f)
            {
                PID_P->target=0.0f;
                state=3;
            }
            break;
        }
        case 3:
        {
            //PID_P->target = 0.0f;
            PID_T->target -= k/100;
            if(PID_T->actual<=0.0f)
            {
                PID_T->target=0.0f;
                state=0;
            }
            break;
        }

    }

}


void PTU_Task(void)
{
    // PID_P.actual = M_Pan.current_angle;
    // PID_P.target = M_Pan.target_angle;
    // PID_CalcINC(&PID_P);
    // StepMotor_SetSpeed(&M_Pan, PID_P.output);


    // PID_T.actual = M_Tilt.current_angle;
    // PID_T.target = M_Tilt.target_angle;
    // PID_CalcINC(&PID_T);
    // StepMotor_SetSpeed(&M_Tilt, PID_T.output);


    // PID_P.actual = M_Pan.current_angle;
    // PID_T.actual = M_Tilt.current_angle;
    // rectCal(&PID_P,&PID_T);
    // PID_CalcINC(&PID_P);
    // PID_CalcINC(&PID_T);

    // StepMotor_SetSpeed(&M_Pan,PID_P.output);
    // StepMotor_SetSpeed(&M_Tilt,PID_T.output);

    circCal(&M_Pan,&M_Tilt);
    StepMotor_SetSpeed(&M_Pan,M_Pan.target_speed);
    StepMotor_SetSpeed(&M_Tilt,M_Tilt.target_speed);

    
}



/* TIM 中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM6)
    {
        static uint32_t cnt=0;
        cnt++;
        if(cnt>=2)
        {
            cnt=0;
            Emm_V5_Read_Sys_Params(&M_Pan, S_CPOS);
            Emm_V5_Read_Sys_Params(&M_Tilt, S_CPOS);
        }
        
        PTU_Task();
    }

}

