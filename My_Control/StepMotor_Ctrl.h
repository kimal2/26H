#ifndef STEP_MOTOR_CTRL_H
#define STEP_MOTOR_CTRL_H

#include "Emm_V5.h"
#include <stdbool.h>
#include <stdint.h>

/* Hardware and mechanism parameters. Tune these values during commissioning. */
#define STEPMOTOR_MOTOR_ID                    0U
#define STEPMOTOR_PULSES_PER_REV              3200.0f
#define STEPMOTOR_HOME_MODE                   0U
#define STEPMOTOR_HOME_WAIT_MS                1000U
#define STEPMOTOR_ZERO_SAVE_WAIT_MS           1000U
#define STEPMOTOR_RUN_SPEED_RPM               120U
#define STEPMOTOR_RUN_ACCELERATION            0U
#define STEPMOTOR_POSITIVE_DIRECTION          0U

#define STEPMOTOR_CAMERA_CENTER_X             160U
#define STEPMOTOR_CAMERA_X_MAX                320U

#define STEPMOTOR_TUBE_ANGLE_MIN_DEG          (-10.0f)
#define STEPMOTOR_TUBE_ANGLE_MAX_DEG          10.0f
#define STEPMOTOR_MOTOR_DEG_PER_TUBE_DEG      1.0f
#define STEPMOTOR_COMMAND_DEADBAND_DEG        0.1f

#define STEPMOTOR_PID_KP                      0.05f
#define STEPMOTOR_PID_KI                      0.0f
#define STEPMOTOR_PID_KD                      0.0f
#define STEPMOTOR_FEEDFORWARD_KFF             0.0f

typedef struct {
    volatile float ball_kp;
    volatile float ball_ki;
    volatile float ball_kd;
    volatile float ball_kff;
    volatile uint16_t run_speed_rpm;
} StepMotorTune_t;

typedef enum {
    STEPMOTOR_STATE_UNINITIALIZED = 0,
    STEPMOTOR_STATE_ENABLING,
    STEPMOTOR_STATE_HOMING,
    STEPMOTOR_STATE_READY,
    STEPMOTOR_STATE_DISABLED,
    STEPMOTOR_STATE_ZERO_SAVING,
    STEPMOTOR_STATE_ZERO_CLEARING
} StepMotorControlState_t;

extern StepMotor_t WaterTubeMotor;
extern StepMotorTune_t StepMotorTune;

void StepMotor_Init(void);
void StepMotor_SetBallX(uint16_t ball_x);
void StepMotor_SetFeedforwardAcceleration(float acceleration_g);
void StepMotor_SetTubeAngle(float tube_angle_deg);
bool StepMotor_SetEnabled(bool enabled);
bool StepMotor_SaveCurrentAsHome(void);
void StepMotor_Task(void);

StepMotorControlState_t StepMotor_GetState(void);
float StepMotor_GetTargetAngle(void);
uint16_t StepMotor_GetBallX(void);

#endif
