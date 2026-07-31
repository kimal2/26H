#include "My_UartProc.h"

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
    PARAM_RESULT_RANGE
} ParameterResult_t;

static uint8_t camera_rx_data[CAMERA_FRAME_SIZE];
static uint8_t bluetooth_rx_data[BLUETOOTH_RX_SIZE];
static char bluetooth_frame[BLUETOOTH_FRAME_SIZE];
static volatile uint8_t bluetooth_frame_ready;
static uint8_t bluetooth_frame_index;
static uint8_t bluetooth_frame_receiving;
static uint32_t line_telemetry_tick;

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
        if (Bluetooth_ValueInRange(value, -100.0f, 100.0f) == 0U) {
            return PARAM_RESULT_RANGE;
        }
        StepMotorTune.ball_kff = value;
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

    if (strcmp(key, "odo") == 0) {
        bluetooth_frame_ready = 0U;
        if (Bluetooth_ParseUInt64(comma + 1, &odometer_target) == 0U) {
            Bluetooth_Send("[ERR,RANGE]\r\n");
            return;
        }
        Tracking_SetOdometerTarget(odometer_target);
        (void)strcpy(response, "[OK,odo,");
        closing_bracket = Bluetooth_AppendUInt64(
            response + strlen(response), odometer_target);
        (void)strcpy(closing_bracket, "]\r\n");
        Bluetooth_Send(response);
        return;
    }

    value = (float)atof(comma + 1);
    result = Bluetooth_SetParameter(key, value);
    bluetooth_frame_ready = 0U;

    if (result == PARAM_RESULT_OK) {
        (void)snprintf(response, sizeof(response),
                       "[OK,%s,%.3f]\r\n", key, value);
        Bluetooth_Send(response);
    } else if (result == PARAM_RESULT_RANGE) {
        Bluetooth_Send("[ERR,RANGE]\r\n");
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
        StepMotor_SetBallX(ball_x);
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
