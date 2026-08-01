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
#define STEPMOTOR_RUN_SPEED_RPM               100
#define STEPMOTOR_RUN_ACCELERATION            0U
#define STEPMOTOR_POSITIVE_DIRECTION          0U

#define STEPMOTOR_CAMERA_CENTER_X             292U
#define STEPMOTOR_CAMERA_X_MAX                640U
#define STEPMOTOR_PIXEL_SWEEP_TEST_ENABLE     0U
#define STEPMOTOR_PIXEL_SWEEP_PERIOD_MS       20U

#define STEPMOTOR_TUBE_ANGLE_MIN_DEG          (-20.0f)
#define STEPMOTOR_TUBE_ANGLE_MAX_DEG          30.0f
#define STEPMOTOR_TUBE_ANGLE_HARD_MIN_DEG     (-30.0f)
#define STEPMOTOR_TUBE_ANGLE_HARD_MAX_DEG     30.0f
#define STEPMOTOR_MOTOR_DEG_PER_TUBE_DEG      1.0f
#define STEPMOTOR_COMMAND_DEADBAND_DEG        0.01f

#define STEPMOTOR_PID_KP                      0.001f
#define STEPMOTOR_PID_KI                      0.03f
#define STEPMOTOR_PID_KD                      0.05f
#define STEPMOTOR_FEEDFORWARD_KFF             0.0f
#define STEPMOTOR_FEEDFORWARD_LPF_ALPHA       0.01f
#define STEPMOTOR_BALL_X_LPF_ALPHA            0.96f

#define STEPMOTOR_Q3_PID_KP                   0.07f
#define STEPMOTOR_Q3_PID_KI                   0.0f
#define STEPMOTOR_Q3_PID_KD                   0.0f
#define STEPMOTOR_Q3_FEEDFORWARD_KFF          0.0f
#define STEPMOTOR_Q3_FEEDFORWARD_LPF_ALPHA    0.01f
#define STEPMOTOR_Q3_BALL_X_LPF_ALPHA         0.20f
#define STEPMOTOR_Q3_ANGLE_MIN_DEG            (-20.0f)
#define STEPMOTOR_Q3_ANGLE_MAX_DEG            30.0f
#define STEPMOTOR_Q3_RUN_SPEED_RPM            150U

#define STEPMOTOR_Q3_NEG_PID_KP               0.004f
#define STEPMOTOR_Q3_NEG_PID_KI               0.03f
#define STEPMOTOR_Q3_NEG_PID_KD               0.02f
#define STEPMOTOR_Q3_NEG_FEEDFORWARD_KFF      0.0f
#define STEPMOTOR_Q3_NEG_FEEDFORWARD_LPF_ALPHA 0.01f
#define STEPMOTOR_Q3_NEG_BALL_X_LPF_ALPHA     0.78f
#define STEPMOTOR_Q3_NEG_ANGLE_BIAS_DEG       (-7.8f)
#define STEPMOTOR_Q3_NEG_ANGLE_MIN_DEG        (-20.0f)
#define STEPMOTOR_Q3_NEG_ANGLE_MAX_DEG        30.0f
#define STEPMOTOR_Q3_NEG_RUN_SPEED_RPM        150U

#define STEPMOTOR_Q4_PID_KP                   0.001f
#define STEPMOTOR_Q4_PID_KI                   0.07f
#define STEPMOTOR_Q4_PID_KD                   0.8f
#define STEPMOTOR_Q4_FEEDFORWARD_KFF          0.0f
#define STEPMOTOR_Q4_FEEDFORWARD_LPF_ALPHA    0.01f
#define STEPMOTOR_Q4_BALL_X_LPF_ALPHA         0.20f
#define STEPMOTOR_Q4_ANGLE_MIN_DEG            (-20.0f)
#define STEPMOTOR_Q4_ANGLE_MAX_DEG            30.0f
#define STEPMOTOR_Q4_RUN_SPEED_RPM            29U

typedef struct {
    volatile float ball_kp;
    volatile float ball_ki;
    volatile float ball_kd;
    volatile float ball_kff;
    volatile float feedforward_lpf_alpha;
    volatile float ball_lpf_alpha;
    volatile float angle_bias_deg;
    volatile float angle_min_deg;
    volatile float angle_max_deg;
    volatile uint16_t run_speed_rpm;
} StepMotorTune_t;

typedef enum {
    STEPMOTOR_PROFILE_DEFAULT = 0,
    STEPMOTOR_PROFILE_QUESTION3,
    STEPMOTOR_PROFILE_QUESTION3_NEGATIVE,
    STEPMOTOR_PROFILE_QUESTION4
} StepMotorControlProfile_t;

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
extern StepMotorTune_t StepMotorQuestion3Tune;
extern StepMotorTune_t StepMotorQuestion3NegativeTune;
extern StepMotorTune_t StepMotorQuestion4Tune;

void StepMotor_Init(void);
void StepMotor_SetBallX(uint16_t ball_x);
bool StepMotor_SetBallTargetX(uint16_t target_x);
bool StepMotor_SetControlProfile(StepMotorControlProfile_t profile);
void StepMotor_SetAngleOverride(bool enabled, float angle_deg);
void StepMotor_SetFeedforwardAcceleration(float acceleration_g);
void StepMotor_SetTubeAngle(float tube_angle_deg);
bool StepMotor_SetEnabled(bool enabled);
bool StepMotor_SaveCurrentAsHome(void);
void StepMotor_Task(void);

StepMotorControlState_t StepMotor_GetState(void);
float StepMotor_GetTargetAngle(void);
uint16_t StepMotor_GetBallX(void);
uint16_t StepMotor_GetBallTargetX(void);
StepMotorControlProfile_t StepMotor_GetControlProfile(void);

#endif
