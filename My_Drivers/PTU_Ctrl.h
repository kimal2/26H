#ifndef __PTU_CONTROL_H
#define __PTU_CONTROL_H

#include "Emm_V5.h"

#define PULSES_PER_REV 3200

extern StepMotor_t M_Pan;
extern StepMotor_t M_Tilt; 
 


void PTU_Init(void);
void PTU_SetTarget(float pan_deg, float tilt_deg);
void PTU_Task(void);


#endif
