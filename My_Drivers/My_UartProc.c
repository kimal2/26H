#include "My_UartProc.h"

#include "DisplayTask.h"
#include "linetrack.h"
#include "StepMotor_Ctrl.h"
#include "usart.h"

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

typedef enum {
    PARAM_RESULT_OK = 0,
    PARAM_RESULT_UNKNOWN,
    PARAM_RESULT_RANGE,
    PARAM_RESULT_BUSY
} ParameterResult_t;

static uint8_t camera_rx_data[CAMERA_FRAME_SIZE];
static uint8_t bluetooth_rx_data[BLUETOOTH_RX_SIZE];
static char bluetooth_frame[BLUETOOTH_FRAME_SIZE];
static volatile uint8_t bluetooth_frame_ready;
static uint8_t bluetooth_frame_index;
static uint8_t bluetooth_frame_receiving;
static uint32_t line_telemetry_tick;
static volatile uint16_t camera_debug_ball_x;
static volatile uint8_t camera_debug_pending;

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

static ParameterResult_t Bluetooth_SetParameter(const char *key, float value)
{
    if (strcmp(key, "line_kp") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.line_kp = value;
    } else if (strcmp(key, "line_ki") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.line_ki = value;
    } else if (strcmp(key, "line_kd") == 0) {
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.line_kd = value;
    } else if ((strcmp(key, "base_pwm") == 0) ||
               (strcmp(key, "base_speed") == 0)) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.base_pwm = value;
    } else if (strcmp(key, "turn_pwm") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.turn_pwm = value;
    } else if (strcmp(key, "line_max") == 0) {
        if (Bluetooth_ValueInRange(value, 0.0f, TRACKING_PWM_MAX) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        TrackingTune.max_correction = value;
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
    } else if (strcmp(key, "q3n_bias") == 0) {
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

    if ((strcmp(key, "odo") == 0) || (strcmp(key, "q4_odo") == 0)) {
        bluetooth_frame_ready = 0U;
        if (Bluetooth_ParseUInt64(comma + 1, &odometer_target) == 0U) {
            Bluetooth_Send("[ERR,RANGE]\r\n");
            return;
        }
        if (strcmp(key, "q4_odo") == 0) {
            DisplayTask_SetQuestion4OdometerTarget(odometer_target);
        } else {
            Tracking_SetOdometerTarget(odometer_target);
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
