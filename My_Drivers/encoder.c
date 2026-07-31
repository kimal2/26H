#include "encoder.h"
#include "tim.h"

/* Keep the logical mapping used by the original vehicle project. */
#define ENCODER_SWAP_LR
#define ENCODER_SWAP_SIGN

typedef struct {
    TIM_HandleTypeDef *timer;
    uint16_t previous;
    int32_t count;
} Encoder_Counter_t;

static Encoder_Counter_t encoder_left = {&htim3, 0U, 0};
static Encoder_Counter_t encoder_right = {&htim4, 0U, 0};

static int32_t Encoder_Update(Encoder_Counter_t *encoder)
{
    uint16_t current;
    int16_t delta;

    current = (uint16_t)__HAL_TIM_GET_COUNTER(encoder->timer);
    delta = (int16_t)(current - encoder->previous);
    encoder->previous = current;
    encoder->count += (int32_t)delta;
    return encoder->count;
}

void Encoder_Init(void)
{
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
        return;
    }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
        return;
    }

    Encoder_Clear();
}

int32_t Encoder_Get(uint8_t L_R)
{
    if (L_R != 0U) {
        return Encoder_Get_R();
    }
    return Encoder_Get_L();
}

int32_t Encoder_Get_L(void)
{
    int32_t value;

#ifdef ENCODER_SWAP_LR
    value = Encoder_Update(&encoder_right);
#else
    value = Encoder_Update(&encoder_left);
#endif

#ifdef ENCODER_SWAP_SIGN
    value = -value;
#endif
    return value;
}

int32_t Encoder_Get_R(void)
{
    int32_t value;

#ifdef ENCODER_SWAP_LR
    value = Encoder_Update(&encoder_left);
#else
    value = Encoder_Update(&encoder_right);
#endif

#ifdef ENCODER_SWAP_SIGN
    value = -value;
#endif
    return value;
}

void Encoder_Clear(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    encoder_left.previous = 0U;
    encoder_left.count = 0;
    encoder_right.previous = 0U;
    encoder_right.count = 0;

    if (primask == 0U) {
        __enable_irq();
    }
}
