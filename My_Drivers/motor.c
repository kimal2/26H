#include "motor.h"
#include "tim.h"

/* Uncomment to exchange the logical left and right motor commands. */
/* #define MOTOR_SWAP_LEFT_RIGHT */

#define PWM_PERIOD              100L

#define MOTOR_LEFT_IN1_CHANNEL  TIM_CHANNEL_2  /* PE11, AIN1 */
#define MOTOR_LEFT_IN2_CHANNEL  TIM_CHANNEL_1  /* PE9,  AIN2 */
#define MOTOR_RIGHT_IN1_CHANNEL TIM_CHANNEL_4  /* PE14, BIN1 */
#define MOTOR_RIGHT_IN2_CHANNEL TIM_CHANNEL_3  /* PE13, BIN2 */

void Motor_Init(void)
{
    HAL_StatusTypeDef status;

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    status |= HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (status != HAL_OK) {
        return;
    }

    Motor_Sleep();
}

void Motor_Set_PWM(int32_t PWM_L,int32_t PWM_R)
{

    /* 根据机械安装方向选择是否交换左右电机指令。 */
    #ifdef MOTOR_SWAP_LEFT_RIGHT
    int32_t temp;
    temp = PWM_L;
    PWM_L = PWM_R;
    PWM_R = temp;
    #endif

    /* 将左右电机指令限制在定时器允许的范围内。 */
    if(PWM_L>PWM_PERIOD)        PWM_L = PWM_PERIOD;
    else if(PWM_L<-PWM_PERIOD)  PWM_L = -PWM_PERIOD;

    if(PWM_R>PWM_PERIOD)        PWM_R = PWM_PERIOD;
    else if(PWM_R<-PWM_PERIOD)  PWM_R = -PWM_PERIOD;


    int32_t CCR_Left;
    int32_t CCR_Right;

    /* 设置左电机 H 桥两路 PWM 比较值。 */
    if(PWM_L>0)
    {
        CCR_Left = PWM_PERIOD - PWM_L;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, CCR_Left);
    }
    else
    {
        CCR_Left = PWM_PERIOD + PWM_L;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, CCR_Left);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, PWM_PERIOD);
    }

    /* 设置右电机 H 桥两路 PWM 比较值。 */
    if(PWM_R>0)
    {
        CCR_Right = PWM_PERIOD - PWM_R;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, CCR_Right);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, PWM_PERIOD);
    }
    else
    {
        CCR_Right = PWM_PERIOD + PWM_R;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, CCR_Right);
    }
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, PWM_PERIOD);
}

void Motor_Sleep(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, 0U);
}
