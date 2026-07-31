#include "motor.h"
#include "tim.h"

/* Uncomment to exchange the logical left and right motor commands. */
/* #define MOTOR_SWAP_LEFT_RIGHT */

#define MOTOR_LEFT_IN1_CHANNEL  TIM_CHANNEL_2  /* PE11, AIN1 */
#define MOTOR_LEFT_IN2_CHANNEL  TIM_CHANNEL_1  /* PE9,  AIN2 */
#define MOTOR_RIGHT_IN1_CHANNEL TIM_CHANNEL_4  /* PE14, BIN1 */
#define MOTOR_RIGHT_IN2_CHANNEL TIM_CHANNEL_3  /* PE13, BIN2 */

static volatile uint64_t right_distance_counts;
static volatile uint64_t left_distance_counts;
static uint16_t encoder3_last;
static uint16_t encoder4_last;
static uint8_t encoders_started;

static uint64_t Motor_AddDistanceSaturated(uint64_t total, uint32_t increment)
{
    const uint64_t maximum = ~(uint64_t)0;

    if (total > (maximum - (uint64_t)increment)) {
        return maximum;
    }
    return total + (uint64_t)increment;
}

static uint32_t Motor_EncoderDeltaMagnitude(uint16_t current, uint16_t previous)
{
    int32_t delta;

    delta = (int32_t)(int16_t)(current - previous);
    return (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;
}

static void Motor_EncoderInit(void)
{
    HAL_StatusTypeDef status;

    if (encoders_started != 0U) {
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    status |= HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (status != HAL_OK) {
        return;
    }

    right_distance_counts = 0U;
    left_distance_counts = 0U;
    encoder3_last = 0U;
    encoder4_last = 0U;
    encoders_started = 1U;
}

void Motor_EncoderUpdate(void)
{
    uint16_t encoder3_now;
    uint16_t encoder4_now;
    uint32_t primask;

    if (encoders_started == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    encoder3_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    encoder4_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    right_distance_counts = Motor_AddDistanceSaturated(
        right_distance_counts,
        Motor_EncoderDeltaMagnitude(encoder3_now, encoder3_last));
    left_distance_counts = Motor_AddDistanceSaturated(
        left_distance_counts,
        Motor_EncoderDeltaMagnitude(encoder4_now, encoder4_last));
    encoder3_last = encoder3_now;
    encoder4_last = encoder4_now;
    if (primask == 0U) {
        __enable_irq();
    }
}

void Motor_OdometerReset(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    right_distance_counts = 0U;
    left_distance_counts = 0U;
    encoder3_last = 0U;
    encoder4_last = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

uint64_t Motor_GetOdometerCounts(void)
{
    uint64_t right;
    uint64_t left;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    right = right_distance_counts;
    left = left_distance_counts;
    if (primask == 0U) {
        __enable_irq();
    }

    return (right / 2U) + (left / 2U) +
           (((right & 1U) != 0U) && ((left & 1U) != 0U) ? 1U : 0U);
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

    Motor_EncoderInit();
    Motor_Sleep();
}

void Motor_Set_PWM(int32_t PWM_L,int32_t PWM_R)
{
    int32_t timer_counts;
    int32_t pwm_left_counts;
    int32_t pwm_right_counts;
    int32_t CCR_Left;
    int32_t CCR_Right;

    /* 根据机械安装方向选择是否交换左右电机指令。 */
    #ifdef MOTOR_SWAP_LEFT_RIGHT
    int32_t temp;
    temp = PWM_L;
    PWM_L = PWM_R;
    PWM_R = temp;
    #endif

    timer_counts = (int32_t)__HAL_TIM_GET_AUTORELOAD(&htim1) + 1L;

    /* 上层 PWM 参数直接使用 TIM1 计数值，范围由当前 ARR 决定。 */
    if(PWM_L>timer_counts)        PWM_L = timer_counts;
    else if(PWM_L<-timer_counts)  PWM_L = -timer_counts;

    if(PWM_R>timer_counts)        PWM_R = timer_counts;
    else if(PWM_R<-timer_counts)  PWM_R = -timer_counts;

    pwm_left_counts = PWM_L;
    pwm_right_counts = PWM_R;

    /* 设置左电机 H 桥两路 PWM 比较值。 */
    if(PWM_L>0)
    {
        CCR_Left = timer_counts - pwm_left_counts;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, timer_counts);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, CCR_Left);
    }
    else
    {
        CCR_Left = timer_counts + pwm_left_counts;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, CCR_Left);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, timer_counts);
    }

    /* 设置右电机 H 桥两路 PWM 比较值。 */
    if(PWM_R>0)
    {
        CCR_Right = timer_counts - pwm_right_counts;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, CCR_Right);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, timer_counts);
    }
    else
    {
        CCR_Right = timer_counts + pwm_right_counts;
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, timer_counts);
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, CCR_Right);
    }
}

void Motor_Stop(void)
{
    uint32_t timer_counts;

    timer_counts = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, timer_counts);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, timer_counts);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, timer_counts);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, timer_counts);
}

void Motor_Sleep(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_LEFT_IN2_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN1_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_RIGHT_IN2_CHANNEL, 0U);
}
