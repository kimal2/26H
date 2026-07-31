#include "motor.h"
#include "tim.h"

/* Uncomment to exchange the logical left and right motor commands. */
/* #define MOTOR_SWAP_LEFT_RIGHT */

#define MOTOR_COMMAND_MAX       1000L

#define MOTOR_LEFT_IN1_CHANNEL  TIM_CHANNEL_2  /* PE11, AIN1 */
#define MOTOR_LEFT_IN2_CHANNEL  TIM_CHANNEL_1  /* PE9,  AIN2 */
#define MOTOR_RIGHT_IN1_CHANNEL TIM_CHANNEL_4  /* PE14, BIN1 */
#define MOTOR_RIGHT_IN2_CHANNEL TIM_CHANNEL_3  /* PE13, BIN2 */

static uint32_t Motor_GetPeriod(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
}

static uint32_t Motor_CommandToPulse(int32_t command)
{
    uint32_t magnitude;

    if (command < 0) {
        magnitude = (uint32_t)(-command);
    } else {
        magnitude = (uint32_t)command;
    }

    return (magnitude * Motor_GetPeriod()) / (uint32_t)MOTOR_COMMAND_MAX;
}

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

void Motor_Set_PWM(int32_t PWM_L, int32_t PWM_R)
{
    uint32_t left_pulse;
    uint32_t right_pulse;

#ifdef MOTOR_SWAP_LEFT_RIGHT
    int32_t temp;

    temp = PWM_L;
    PWM_L = PWM_R;
    PWM_R = temp;
#endif

    if (PWM_L > MOTOR_COMMAND_MAX) {
        PWM_L = MOTOR_COMMAND_MAX;
    } else if (PWM_L < -MOTOR_COMMAND_MAX) {
        PWM_L = -MOTOR_COMMAND_MAX;
    }

    if (PWM_R > MOTOR_COMMAND_MAX) {
        PWM_R = MOTOR_COMMAND_MAX;
    } else if (PWM_R < -MOTOR_COMMAND_MAX) {
        PWM_R = -MOTOR_COMMAND_MAX;
    }

    left_pulse = Motor_CommandToPulse(PWM_L);
    right_pulse = Motor_CommandToPulse(PWM_R);

    if (PWM_L > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, left_pulse);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, 0U);
    } else if (PWM_L < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, left_pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, 0U);
    }

    if (PWM_R > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, right_pulse);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, 0U);
    } else if (PWM_R < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, right_pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, 0U);
    }
}

void Motor_Stop(void)
{
    uint32_t period;

    period = Motor_GetPeriod();
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, period);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, period);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, period);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, period);
}

void Motor_Sleep(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, 0U);
}
