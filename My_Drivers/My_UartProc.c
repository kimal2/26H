#include "My_UartProc.h"

#include "DisplayTask.h"
#include "linetrack.h"
#include "StepMotor_Ctrl.h"
#include "usart.h"
#include "stm32f4xx_hal_flash_ex.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAMERA_UART            huart6
#define CAMERA_FRAME_SIZE      2U
#define BLUETOOTH_UART         huart5
#define BLUETOOTH_RX_SIZE      32U
#define BLUETOOTH_FRAME_SIZE   40U
#define BLUETOOTH_KEY_SIZE     16U
#define LINE_TELEMETRY_PERIOD_MS  50U
#define FLASH_SETTINGS_ADDRESS 0x08060000UL
#define FLASH_SETTINGS_MAGIC   0x48363253UL
#define FLASH_SETTINGS_VERSION 5UL

typedef enum {
    PARAM_RESULT_OK = 0,
    PARAM_RESULT_UNKNOWN,
    PARAM_RESULT_RANGE,
    PARAM_RESULT_BUSY,
    PARAM_RESULT_FLASH
} ParameterResult_t;

typedef struct {
    float line_kp;
    float line_ki;
    float line_kd;
    float base_pwm;
    float turn_pwm;
    float max_correction;
} FlashTrackingTune_t;

typedef struct {
    float ball_kp;
    float ball_ki;
    float ball_kd;
    float ball_kff;
    float feedforward_lpf_alpha;
    float ball_lpf_alpha;
    float angle_bias_deg;
    float angle_min_deg;
    float angle_max_deg;
    uint16_t run_speed_rpm;
    uint16_t reserved;
} FlashStepMotorTune_t;

typedef struct {
    uint32_t enabled;
    float entry_error_px;
    float exit_error_px;
    float target_accel_px_s2;
    float target_speed_max_px_s;
    float angle_kp;
    float angle_kv;
    float angle_limit_deg;
    float velocity_lpf_alpha;
} FlashStepMotorReturnTune_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    FlashTrackingTune_t tracking_q2;
    FlashTrackingTune_t tracking_q4;
    FlashStepMotorTune_t step_default;
    FlashStepMotorTune_t step_q3;
    FlashStepMotorTune_t step_q3_negative;
    FlashStepMotorTune_t step_q4;
    FlashStepMotorReturnTune_t q4_return;
    uint16_t camera_center_x;
    uint16_t q3_positive_x;
    uint16_t q3_negative_x;
    uint16_t reserved;
    uint16_t q3_brake_trigger_x;
    uint16_t q3_brake_speed_rpm;
    float q3_brake_angle_deg;
    uint32_t q3_brake_duration_ms;
    uint64_t q2_odometer_target;
    uint64_t q4_odometer_target;
    float q4_drive_acceleration;
    uint32_t crc32;
} FlashSettings_t;

static uint8_t camera_rx_data[CAMERA_FRAME_SIZE];
static uint8_t bluetooth_rx_data[BLUETOOTH_RX_SIZE];
static char bluetooth_frame[BLUETOOTH_FRAME_SIZE];
static volatile uint8_t bluetooth_frame_ready;
static uint8_t bluetooth_frame_index;
static uint8_t bluetooth_frame_receiving;
static uint32_t line_telemetry_tick;
static volatile uint16_t camera_debug_ball_x;
static volatile uint8_t camera_debug_pending;
static uint8_t flash_settings_loaded;

static uint32_t FlashSettings_CalculateCrc(const FlashSettings_t *settings)
{
    const uint8_t *data = (const uint8_t *)settings;
    uint32_t crc = 2166136261UL;
    uint32_t index;
    uint32_t length = (uint32_t)(sizeof(FlashSettings_t) -
                                 sizeof(settings->crc32));

    for (index = 0U; index < length; index++) {
        crc ^= data[index];
        crc *= 16777619UL;
    }
    return crc;
}

static void FlashSettings_CopyTrackingTuneFromRuntime(
    FlashTrackingTune_t *target,
    const TrackingTune_t *source)
{
    target->line_kp = source->line_kp;
    target->line_ki = source->line_ki;
    target->line_kd = source->line_kd;
    target->base_pwm = source->base_pwm;
    target->turn_pwm = source->turn_pwm;
    target->max_correction = source->max_correction;
}

static void FlashSettings_CopyTrackingTuneToRuntime(
    TrackingTune_t *target,
    const FlashTrackingTune_t *source)
{
    target->line_kp = source->line_kp;
    target->line_ki = source->line_ki;
    target->line_kd = source->line_kd;
    target->base_pwm = source->base_pwm;
    target->turn_pwm = source->turn_pwm;
    target->max_correction = source->max_correction;
}

static void FlashSettings_CopyStepTuneFromRuntime(FlashStepMotorTune_t *target,
                                                  const StepMotorTune_t *source)
{
    target->ball_kp = source->ball_kp;
    target->ball_ki = source->ball_ki;
    target->ball_kd = source->ball_kd;
    target->ball_kff = source->ball_kff;
    target->feedforward_lpf_alpha = source->feedforward_lpf_alpha;
    target->ball_lpf_alpha = source->ball_lpf_alpha;
    target->angle_bias_deg = source->angle_bias_deg;
    target->angle_min_deg = source->angle_min_deg;
    target->angle_max_deg = source->angle_max_deg;
    target->run_speed_rpm = source->run_speed_rpm;
    target->reserved = 0U;
}

static void FlashSettings_CopyStepTuneToRuntime(StepMotorTune_t *target,
                                                const FlashStepMotorTune_t *source)
{
    target->ball_kp = source->ball_kp;
    target->ball_ki = source->ball_ki;
    target->ball_kd = source->ball_kd;
    target->ball_kff = source->ball_kff;
    target->feedforward_lpf_alpha = source->feedforward_lpf_alpha;
    target->ball_lpf_alpha = source->ball_lpf_alpha;
    target->angle_bias_deg = source->angle_bias_deg;
    target->angle_min_deg = source->angle_min_deg;
    target->angle_max_deg = source->angle_max_deg;
    target->run_speed_rpm = source->run_speed_rpm;
}

static void FlashSettings_CopyReturnTuneFromRuntime(
    FlashStepMotorReturnTune_t *target,
    const StepMotorReturnTune_t *source)
{
    target->enabled = source->enabled;
    target->entry_error_px = source->entry_error_px;
    target->exit_error_px = source->exit_error_px;
    target->target_accel_px_s2 = source->target_accel_px_s2;
    target->target_speed_max_px_s = source->target_speed_max_px_s;
    target->angle_kp = source->angle_kp;
    target->angle_kv = source->angle_kv;
    target->angle_limit_deg = source->angle_limit_deg;
    target->velocity_lpf_alpha = source->velocity_lpf_alpha;
}

static void FlashSettings_CopyReturnTuneToRuntime(
    StepMotorReturnTune_t *target,
    const FlashStepMotorReturnTune_t *source)
{
    target->enabled = (source->enabled != 0U) ? 1U : 0U;
    target->entry_error_px = source->entry_error_px;
    target->exit_error_px = source->exit_error_px;
    target->target_accel_px_s2 = source->target_accel_px_s2;
    target->target_speed_max_px_s = source->target_speed_max_px_s;
    target->angle_kp = source->angle_kp;
    target->angle_kv = source->angle_kv;
    target->angle_limit_deg = source->angle_limit_deg;
    target->velocity_lpf_alpha = source->velocity_lpf_alpha;
}

static void FlashSettings_Capture(FlashSettings_t *settings)
{
    (void)memset(settings, 0, sizeof(*settings));
    settings->magic = FLASH_SETTINGS_MAGIC;
    settings->version = FLASH_SETTINGS_VERSION;
    settings->size = sizeof(*settings);

    FlashSettings_CopyTrackingTuneFromRuntime(&settings->tracking_q2,
                                              &TrackingQuestion2Tune);
    FlashSettings_CopyTrackingTuneFromRuntime(&settings->tracking_q4,
                                              &TrackingQuestion4Tune);

    FlashSettings_CopyStepTuneFromRuntime(&settings->step_default,
                                          &StepMotorTune);
    FlashSettings_CopyStepTuneFromRuntime(&settings->step_q3,
                                          &StepMotorQuestion3Tune);
    FlashSettings_CopyStepTuneFromRuntime(&settings->step_q3_negative,
                                          &StepMotorQuestion3NegativeTune);
    FlashSettings_CopyStepTuneFromRuntime(&settings->step_q4,
                                          &StepMotorQuestion4Tune);
    FlashSettings_CopyReturnTuneFromRuntime(&settings->q4_return,
                                            &StepMotorQuestion4ReturnTune);

    settings->camera_center_x = StepMotor_GetCameraCenterX();
    settings->q3_positive_x = DisplayTask_GetQuestion3PositiveX();
    settings->q3_negative_x = DisplayTask_GetQuestion3NegativeX();
    settings->q3_brake_trigger_x =
        DisplayTask_GetQuestion3BrakeTriggerX();
    settings->q3_brake_speed_rpm =
        DisplayTask_GetQuestion3BrakeSpeed();
    settings->q3_brake_angle_deg = DisplayTask_GetQuestion3BrakeAngle();
    settings->q3_brake_duration_ms =
        DisplayTask_GetQuestion3BrakeDuration();
    settings->q2_odometer_target =
        DisplayTask_GetQuestion2OdometerTarget();
    settings->q4_odometer_target =
        DisplayTask_GetQuestion4OdometerTarget();
    settings->q4_drive_acceleration =
        DisplayTask_GetQuestion4DriveAcceleration();
    settings->crc32 = FlashSettings_CalculateCrc(settings);
}

static uint8_t FlashSettings_IsValid(const FlashSettings_t *settings)
{
    if ((settings->magic != FLASH_SETTINGS_MAGIC) ||
        (settings->version != FLASH_SETTINGS_VERSION) ||
        (settings->size != sizeof(FlashSettings_t))) {
        return 0U;
    }
    return (settings->crc32 == FlashSettings_CalculateCrc(settings)) ? 1U : 0U;
}

static void FlashSettings_Apply(const FlashSettings_t *settings)
{
    FlashSettings_CopyTrackingTuneToRuntime(&TrackingQuestion2Tune,
                                            &settings->tracking_q2);
    FlashSettings_CopyTrackingTuneToRuntime(&TrackingQuestion4Tune,
                                            &settings->tracking_q4);
    Tracking_SetTune(&TrackingQuestion2Tune);
    DisplayTask_SetQuestion2OdometerTarget(settings->q2_odometer_target);
    Tracking_SetOdometerTarget(settings->q2_odometer_target);

    FlashSettings_CopyStepTuneToRuntime(&StepMotorTune,
                                        &settings->step_default);
    FlashSettings_CopyStepTuneToRuntime(&StepMotorQuestion3Tune,
                                        &settings->step_q3);
    FlashSettings_CopyStepTuneToRuntime(&StepMotorQuestion3NegativeTune,
                                        &settings->step_q3_negative);
    FlashSettings_CopyStepTuneToRuntime(&StepMotorQuestion4Tune,
                                        &settings->step_q4);
    FlashSettings_CopyReturnTuneToRuntime(&StepMotorQuestion4ReturnTune,
                                          &settings->q4_return);

    (void)StepMotor_SetCameraCenterX(settings->camera_center_x);
    DisplayTask_SetQuestion3PositiveX(settings->q3_positive_x);
    DisplayTask_SetQuestion3NegativeX(settings->q3_negative_x);
    DisplayTask_SetQuestion3BrakeTriggerX(settings->q3_brake_trigger_x);
    DisplayTask_SetQuestion3BrakeSpeed(settings->q3_brake_speed_rpm);
    DisplayTask_SetQuestion3BrakeAngle(settings->q3_brake_angle_deg);
    DisplayTask_SetQuestion3BrakeDuration(settings->q3_brake_duration_ms);
    DisplayTask_SetQuestion4OdometerTarget(settings->q4_odometer_target);
    DisplayTask_SetQuestion4DriveAcceleration(
        settings->q4_drive_acceleration);
}

static void FlashSettings_LoadOnce(void)
{
    const FlashSettings_t *settings =
        (const FlashSettings_t *)FLASH_SETTINGS_ADDRESS;

    if (flash_settings_loaded != 0U) {
        return;
    }
    flash_settings_loaded = 1U;
    if (FlashSettings_IsValid(settings) != 0U) {
        FlashSettings_Apply(settings);
    }
}

static uint8_t FlashSettings_CanSaveNow(void)
{
    StepMotorControlState_t state = StepMotor_GetState();

    if ((Tracking_IsReady() != 0U) &&
        (Tracking_GetState() != CAR_STATE_STOP)) {
        return 0U;
    }

    return ((state == STEPMOTOR_STATE_READY) ||
            (state == STEPMOTOR_STATE_DISABLED)) ? 1U : 0U;
}

static ParameterResult_t FlashSettings_SaveAll(void)
{
    FLASH_EraseInitTypeDef erase_init;
    FlashSettings_t settings;
    HAL_StatusTypeDef status;
    uint32_t sector_error = 0U;
    uint32_t offset;

    if (FlashSettings_CanSaveNow() == 0U) {
        return PARAM_RESULT_BUSY;
    }

    FlashSettings_Capture(&settings);

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return PARAM_RESULT_FLASH;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = FLASH_SECTOR_7;
    erase_init.NbSectors = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if ((status == HAL_OK) && (sector_error == 0xFFFFFFFFU)) {
        const uint8_t *source = (const uint8_t *)&settings;

        for (offset = 0U; offset < sizeof(settings); offset += 4U) {
            uint32_t word = 0xFFFFFFFFUL;
            uint32_t copy_length = sizeof(settings) - offset;

            if (copy_length > 4U) {
                copy_length = 4U;
            }
            (void)memcpy(&word, &source[offset], copy_length);
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       FLASH_SETTINGS_ADDRESS + offset,
                                       word);
            if (status != HAL_OK) {
                break;
            }
        }
    } else {
        status = HAL_ERROR;
    }

    (void)HAL_FLASH_Lock();
    return (status == HAL_OK) ? PARAM_RESULT_OK : PARAM_RESULT_FLASH;
}

static char *Bluetooth_AppendUInt64(char *destination, uint64_t value)
{
    char reversed[20];
    uint8_t length = 0U;

    do {
        reversed[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (length != 0U) {
        *destination++ = reversed[--length];
    }
    return destination;
}

static uint8_t Bluetooth_ParseUInt64(const char *text, uint64_t *value)
{
    const uint64_t maximum = ~(uint64_t)0;
    uint64_t parsed = 0U;
    uint8_t digit;

    if (*text == '\0') {
        return 0U;
    }
    while (*text != '\0') {
        if ((*text < '0') || (*text > '9')) {
            return 0U;
        }
        digit = (uint8_t)(*text - '0');
        if (parsed > ((maximum - digit) / 10U)) {
            return 0U;
        }
        parsed = (parsed * 10U) + digit;
        text++;
    }
    *value = parsed;
    return 1U;
}

static HAL_StatusTypeDef Camera_StartReceive(void)
{
    return HAL_UARTEx_ReceiveToIdle_IT(&CAMERA_UART,
                                       camera_rx_data,
                                       CAMERA_FRAME_SIZE);
}

static HAL_StatusTypeDef Bluetooth_StartReceive(void)
{
    return HAL_UARTEx_ReceiveToIdle_IT(&BLUETOOTH_UART,
                                       bluetooth_rx_data,
                                       BLUETOOTH_RX_SIZE);
}

static void Bluetooth_PushByte(uint8_t data)
{
    if (bluetooth_frame_ready != 0U) {
        return;
    }

    if (data == (uint8_t)'[') {
        bluetooth_frame_index = 0U;
        bluetooth_frame_receiving = 1U;
    }
    if (bluetooth_frame_receiving == 0U) {
        return;
    }
    if (bluetooth_frame_index >= (BLUETOOTH_FRAME_SIZE - 1U)) {
        bluetooth_frame_index = 0U;
        bluetooth_frame_receiving = 0U;
        return;
    }

    bluetooth_frame[bluetooth_frame_index++] = (char)data;
    if (data == (uint8_t)']') {
        bluetooth_frame[bluetooth_frame_index] = '\0';
        bluetooth_frame_receiving = 0U;
        bluetooth_frame_ready = 1U;
    }
}

static uint8_t Bluetooth_ValueInRange(float value, float minimum,
                                      float maximum)
{
    return ((value >= minimum) && (value <= maximum)) ? 1U : 0U;
}

static uint8_t Bluetooth_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    parsed = strtof(text, &end);
    if ((end == text) || (*end != '\0')) {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

static ParameterResult_t Bluetooth_SetTrackingParameter(TrackingTune_t *tune,
                                                        const char *parameter,
                                                        float value)
{
    if (strcmp(parameter, "kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->line_kp = value;
    } else if (strcmp(parameter, "ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->line_ki = value;
    } else if (strcmp(parameter, "kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->line_kd = value;
    } else if (strcmp(parameter, "base") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->base_pwm = value;
    } else if (strcmp(parameter, "turn") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->turn_pwm = value;
    } else if (strcmp(parameter, "max") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        tune->max_correction = value;
    } else {
        return PARAM_RESULT_UNKNOWN;
    }
    return PARAM_RESULT_OK;
}

static ParameterResult_t Bluetooth_SetParameter(const char *key, float value)
{
    if ((strcmp(key, "line_kp") == 0) ||
        (strcmp(key, "q2_line_kp") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "kp",
                                              value);
    } else if ((strcmp(key, "line_ki") == 0) ||
               (strcmp(key, "q2_line_ki") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "ki",
                                              value);
    } else if ((strcmp(key, "line_kd") == 0) ||
               (strcmp(key, "q2_line_kd") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "kd",
                                              value);
    } else if ((strcmp(key, "base_pwm") == 0) ||
               (strcmp(key, "base_speed") == 0) ||
               (strcmp(key, "q2_base_pwm") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "base",
                                              value);
    } else if ((strcmp(key, "turn_pwm") == 0) ||
               (strcmp(key, "q2_turn_pwm") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "turn",
                                              value);
    } else if ((strcmp(key, "line_max") == 0) ||
               (strcmp(key, "q2_line_max") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion2Tune, "max",
                                              value);
    } else if (strcmp(key, "q4_line_kp") == 0) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "kp",
                                              value);
    } else if (strcmp(key, "q4_line_ki") == 0) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "ki",
                                              value);
    } else if (strcmp(key, "q4_line_kd") == 0) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "kd",
                                              value);
    } else if (strcmp(key, "q4_turn_pwm") == 0) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "turn",
                                              value);
    } else if (strcmp(key, "q4_line_max") == 0) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "max",
                                              value);
    } else if ((strcmp(key, "center_x") == 0) ||
               (strcmp(key, "camera_center_x") == 0)) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, (float)STEPMOTOR_CAMERA_X_MAX) == 0U) ||
            ((float)(uint16_t)value != value) ||
            !StepMotor_SetCameraCenterX((uint16_t)value)) {
            return PARAM_RESULT_RANGE;
        }
    } else if ((strcmp(key, "q3_pos_x") == 0) ||
               (strcmp(key, "q3_positive_x") == 0)) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, (float)STEPMOTOR_CAMERA_X_MAX) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3PositiveX((uint16_t)value);
    } else if ((strcmp(key, "q3_neg_x") == 0) ||
               (strcmp(key, "q3_negative_x") == 0)) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, (float)STEPMOTOR_CAMERA_X_MAX) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3NegativeX((uint16_t)value);
    } else if (strcmp(key, "ball_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_kp = value;
    } else if (strcmp(key, "ball_ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_ki = value;
    } else if (strcmp(key, "ball_kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_kd = value;
    } else if (strcmp(key, "ball_kff") == 0) {
        if (Bluetooth_ValueInRange(value, -500.0f, 500.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_kff = value;
    } else if (strcmp(key, "ball_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_lpf_alpha = value;
    } else if (strcmp(key, "imu_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.feedforward_lpf_alpha = value;
    } else if (strcmp(key, "motor_min") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG, 0.0f) == 0U) ||
            (value >= StepMotorTune.angle_max_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.angle_min_deg = value;
    } else if (strcmp(key, "motor_max") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) ||
            (value <= StepMotorTune.angle_min_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.angle_max_deg = value;
    } else if (strcmp(key, "motor_rpm") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.run_speed_rpm = (uint16_t)value;
    } else if (strcmp(key, "q3_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.ball_kp = value;
    } else if (strcmp(key, "q3_ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.ball_ki = value;
    } else if (strcmp(key, "q3_kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.ball_kd = value;
    } else if (strcmp(key, "q3_kff") == 0) {
        if (Bluetooth_ValueInRange(value, -500.0f, 500.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.ball_kff = value;
    } else if (strcmp(key, "q3_ball_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.ball_lpf_alpha = value;
    } else if (strcmp(key, "q3_imu_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.feedforward_lpf_alpha = value;
    } else if (strcmp(key, "q3_motor_min") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG, 0.0f) == 0U) ||
            (value >= StepMotorQuestion3Tune.angle_max_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.angle_min_deg = value;
    } else if (strcmp(key, "q3_motor_max") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) ||
            (value <= StepMotorQuestion3Tune.angle_min_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.angle_max_deg = value;
    } else if (strcmp(key, "q3_motor_rpm") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3Tune.run_speed_rpm = (uint16_t)value;
    } else if (strcmp(key, "q3n_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.ball_kp = value;
    } else if (strcmp(key, "q3n_ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.ball_ki = value;
    } else if (strcmp(key, "q3n_kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.ball_kd = value;
    } else if (strcmp(key, "q3n_kff") == 0) {
        if (Bluetooth_ValueInRange(value, -500.0f, 500.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.ball_kff = value;
    } else if (strcmp(key, "q3n_ball_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.ball_lpf_alpha = value;
    } else if (strcmp(key, "q3n_imu_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.feedforward_lpf_alpha = value;
    } else if ((strcmp(key, "q3n_bias") == 0) ||
               (strcmp(key, "q3_neg_bias") == 0)) {
        if (Bluetooth_ValueInRange(
                value,
                STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG,
                STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.angle_bias_deg = value;
    } else if (strcmp(key, "q3n_motor_min") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG, 0.0f) == 0U) ||
            (value >= StepMotorQuestion3NegativeTune.angle_max_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.angle_min_deg = value;
    } else if (strcmp(key, "q3n_motor_max") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) ||
            (value <= StepMotorQuestion3NegativeTune.angle_min_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.angle_max_deg = value;
    } else if (strcmp(key, "q3n_motor_rpm") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion3NegativeTune.run_speed_rpm = (uint16_t)value;
    } else if (strcmp(key, "q3b_x") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, (float)STEPMOTOR_CAMERA_X_MAX) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3BrakeTriggerX((uint16_t)value);
    } else if (strcmp(key, "q3b_angle") == 0) {
        if (Bluetooth_ValueInRange(
                value,
                STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG,
                STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3BrakeAngle(value);
    } else if (strcmp(key, "q3b_ms") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint32_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3BrakeDuration((uint32_t)value);
    } else if (strcmp(key, "q3b_rpm") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion3BrakeSpeed((uint16_t)value);
    } else if (strcmp(key, "q4_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.ball_kp = value;
    } else if (strcmp(key, "q4_ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.ball_ki = value;
    } else if (strcmp(key, "q4_kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.ball_kd = value;
    } else if (strcmp(key, "q4_kff") == 0) {
        if (Bluetooth_ValueInRange(value, -500.0f, 500.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.ball_kff = value;
    } else if (strcmp(key, "q4_ball_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.ball_lpf_alpha = value;
    } else if (strcmp(key, "q4_imu_lpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.feedforward_lpf_alpha = value;
    } else if (strcmp(key, "q4_over_en") == 0) {
        if ((value != 0.0f) && (value != 1.0f)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.enabled = (value != 0.0f) ? 1U : 0U;
    } else if (strcmp(key, "q4_over_entry") == 0) {
        if ((Bluetooth_ValueInRange(value, 1.0f, 320.0f) == 0U) ||
            (value <= StepMotorQuestion4ReturnTune.exit_error_px)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.entry_error_px = value;
    } else if (strcmp(key, "q4_over_exit") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 320.0f) == 0U) ||
            (value >= StepMotorQuestion4ReturnTune.entry_error_px)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.exit_error_px = value;
    } else if (strcmp(key, "q4_ret_accel") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 10000.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.target_accel_px_s2 = value;
    } else if (strcmp(key, "q4_ret_vmax") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 2000.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.target_speed_max_px_s = value;
    } else if (strcmp(key, "q4_ret_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -2.0f, 2.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.angle_kp = value;
    } else if (strcmp(key, "q4_ret_kv") == 0) {
        if (Bluetooth_ValueInRange(value, -2.0f, 2.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.angle_kv = value;
    } else if (strcmp(key, "q4_ret_angle") == 0) {
        if (Bluetooth_ValueInRange(
                value, 1.0f, STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.angle_limit_deg = value;
    } else if (strcmp(key, "q4_ret_vlpf") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, 1.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4ReturnTune.velocity_lpf_alpha = value;
    } else if (strcmp(key, "q4_motor_min") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG, 0.0f) == 0U) ||
            (value >= StepMotorQuestion4Tune.angle_max_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.angle_min_deg = value;
    } else if (strcmp(key, "q4_motor_max") == 0) {
        if ((Bluetooth_ValueInRange(
                 value, 0.0f, STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG) == 0U) ||
            (value <= StepMotorQuestion4Tune.angle_min_deg)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.angle_max_deg = value;
    } else if (strcmp(key, "q4_motor_rpm") == 0) {
        if ((Bluetooth_ValueInRange(value, 0.0f, 5000.0f) == 0U) ||
            ((float)(uint16_t)value != value)) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorQuestion4Tune.run_speed_rpm = (uint16_t)value;
    } else if ((strcmp(key, "q4_drive_pwm") == 0) ||
               (strcmp(key, "q4_drive_speed") == 0)) {
        return Bluetooth_SetTrackingParameter(&TrackingQuestion4Tune, "base",
                                              value);
    } else if (strcmp(key, "q4_drive_accel") == 0) {
        if (Bluetooth_ValueInRange(
                value, 0.0f, QUESTION4_DRIVE_ACCEL_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        DisplayTask_SetQuestion4DriveAcceleration(value);
    } else if (strcmp(key, "motor_en") == 0) {
        if ((value != 0.0f) && (value != 1.0f)) {
            return PARAM_RESULT_RANGE;
        }
        if (!StepMotor_SetEnabled(value != 0.0f)) {
            return PARAM_RESULT_BUSY;
        }
    } else if (strcmp(key, "motor_zero") == 0) {
        if (value != 1.0f) {
            return PARAM_RESULT_RANGE;
        }
        if (!StepMotor_SaveCurrentAsHome()) {
            return PARAM_RESULT_BUSY;
        }
    } else if (strcmp(key, "save_all") == 0) {
        if (value != 1.0f) {
            return PARAM_RESULT_RANGE;
        }
        return FlashSettings_SaveAll();
    } else {
        return PARAM_RESULT_UNKNOWN;
    }

    return PARAM_RESULT_OK;
}

static void Bluetooth_Send(const char *text)
{
    (void)HAL_UART_Transmit(&BLUETOOTH_UART,
                            (uint8_t *)text,
                            (uint16_t)strlen(text),
                            50U);
}

static void Bluetooth_SendLineTelemetry(void)
{
    char telemetry[64];
    char *write_position;
    uint32_t now;

    now = HAL_GetTick();
    if ((uint32_t)(now - line_telemetry_tick) < LINE_TELEMETRY_PERIOD_MS) {
        return;
    }
    line_telemetry_tick = now;

    (void)snprintf(telemetry, sizeof(telemetry),
                   "eb:%d,%d\n",
                   (int)Tracking_GetLineError(),
                   (int)Tracking_GetBlackCount());
    write_position = telemetry + strlen(telemetry);
    *write_position++ = 'o';
    *write_position++ = ':';
    write_position = Bluetooth_AppendUInt64(
        write_position, Tracking_GetOdometerCounts());
    *write_position++ = '\n';
    *write_position = '\0';
    Bluetooth_Send(telemetry);
}

HAL_StatusTypeDef UartReceiveStart(void)
{
    HAL_StatusTypeDef camera_status;
    HAL_StatusTypeDef bluetooth_status;

    FlashSettings_LoadOnce();
    camera_status = Camera_StartReceive();
    bluetooth_status = Bluetooth_StartReceive();
    return (camera_status != HAL_OK) ? camera_status : bluetooth_status;
}

void Bluetooth_Process(void)
{
    char frame[BLUETOOTH_FRAME_SIZE];
    char key[BLUETOOTH_KEY_SIZE];
    char response[64];
    char *comma;
    char *closing_bracket;
    float value;
    uint64_t odometer_target;
    ParameterResult_t result;
    size_t key_length;

    Bluetooth_SendLineTelemetry();
    if (camera_debug_pending != 0U) {
        uint16_t received_ball_x = camera_debug_ball_x;

        camera_debug_pending = 0U;
        (void)snprintf(response, sizeof(response),
                           "%d,%d\n",
                           (int)received_ball_x,
                           (int)StepMotor_GetBallTargetX());
        Bluetooth_Send(response);
    }

    if (bluetooth_frame_ready == 0U) {
        return;
    }

    (void)strncpy(frame, bluetooth_frame, sizeof(frame));
    frame[sizeof(frame) - 1U] = '\0';

    comma = strchr(frame, ',');
    closing_bracket = strchr(frame, ']');
    if ((frame[0] != '[') || (comma == 0) || (closing_bracket == 0) ||
        (comma >= closing_bracket)) {
        bluetooth_frame_ready = 0U;
        Bluetooth_Send("[ERR,FORMAT]\r\n");
        return;
    }

    key_length = (size_t)(comma - &frame[1]);
    if ((key_length == 0U) || (key_length >= sizeof(key))) {
        bluetooth_frame_ready = 0U;
        Bluetooth_Send("[ERR,KEY]\r\n");
        return;
    }

    (void)memcpy(key, &frame[1], key_length);
    key[key_length] = '\0';
    *closing_bracket = '\0';

    if ((strcmp(key, "odo") == 0) ||
        (strcmp(key, "q2_odo") == 0) ||
        (strcmp(key, "q4_odo") == 0)) {
        bluetooth_frame_ready = 0U;
        if (Bluetooth_ParseUInt64(comma + 1, &odometer_target) == 0U) {
            Bluetooth_Send("[ERR,RANGE]\r\n");
            return;
        }
        if (strcmp(key, "q4_odo") == 0) {
            DisplayTask_SetQuestion4OdometerTarget(odometer_target);
        } else {
            DisplayTask_SetQuestion2OdometerTarget(odometer_target);
            if (Tracking_GetTune() == &TrackingQuestion2Tune) {
                Tracking_SetOdometerTarget(odometer_target);
            }
        }
        (void)strcpy(response, "[OK,");
        (void)strcat(response, key);
        (void)strcat(response, ",");
        closing_bracket = Bluetooth_AppendUInt64(
            response + strlen(response), odometer_target);
        (void)strcpy(closing_bracket, "]\r\n");
        Bluetooth_Send(response);
        return;
    }

    if (Bluetooth_ParseFloat(comma + 1, &value) == 0U) {
        bluetooth_frame_ready = 0U;
        Bluetooth_Send("[ERR,FORMAT]\r\n");
        return;
    }
    result = Bluetooth_SetParameter(key, value);
    bluetooth_frame_ready = 0U;

    if (result == PARAM_RESULT_OK) {
        (void)snprintf(response, sizeof(response),
                       "[OK,%s,%.3f]\r\n", key, value);
        Bluetooth_Send(response);
    } else if (result == PARAM_RESULT_RANGE) {
        Bluetooth_Send("[ERR,RANGE]\r\n");
    } else if (result == PARAM_RESULT_BUSY) {
        Bluetooth_Send("[ERR,BUSY]\r\n");
    } else if (result == PARAM_RESULT_FLASH) {
        Bluetooth_Send("[ERR,FLASH]\r\n");
    } else {
        Bluetooth_Send("[ERR,UNKNOWN]\r\n");
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t ball_x;
    uint16_t index;

    if (huart == &BLUETOOTH_UART) {
        for (index = 0U; index < size; index++) {
            Bluetooth_PushByte(bluetooth_rx_data[index]);
        }
        (void)Bluetooth_StartReceive();
        return;
    }

    if (huart != &CAMERA_UART) {
        return;
    }

    if (size == CAMERA_FRAME_SIZE) {
#if CAMERA_X_BIG_ENDIAN
        ball_x = ((uint16_t)camera_rx_data[0] << 8) |
                 (uint16_t)camera_rx_data[1];
#else
        ball_x = (uint16_t)camera_rx_data[0] |
                 ((uint16_t)camera_rx_data[1] << 8);
#endif
        if (ball_x <= STEPMOTOR_CAMERA_X_MAX) {
            StepMotor_SetBallX(ball_x);
            camera_debug_ball_x = StepMotor_GetBallX();
            camera_debug_pending = 1U;
        }
    }

    (void)Camera_StartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &CAMERA_UART) {
        (void)Camera_StartReceive();
    } else if (huart == &BLUETOOTH_UART) {
        bluetooth_frame_index = 0U;
        bluetooth_frame_receiving = 0U;
        bluetooth_frame_ready = 0U;
        (void)Bluetooth_StartReceive();
    }
}
