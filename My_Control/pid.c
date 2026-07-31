#include "pid.h"
#include <math.h>

#define PID_POS 1
//#define PID_INC

void PID_Clear(PID_t *p)
{
	p->target = 0;
	p->actual = 0;
	p->actual_last = 0;
	p->error = 0;
	p->last_error = 0;
	p->prev_error = 0;
	p->integral = 0;
	p->derivative = 0;
	p->output = 0;
}

#ifdef PID_INC
void PID_CalcINC(PID_t *p)
{
	p->prev_error = p->last_error;
	p->last_error = p->error;
	p->error = p->target - p->actual;

	float delta_out = p->Kp * (p->error - p->last_error)
	                + p->Ki * p->error
	                + p->Kd * (p->error - 2 * p->last_error + p->prev_error);

	p->output += delta_out;

	if(p->output > p->Out_Max) p->output = p->Out_Max;
	if(p->output < p->Out_Min) p->output = p->Out_Min;
}

#endif




#ifdef PID_POS
void PID_Calc(PID_t *p)
{
	p->last_error = p->error;
	p->error = p->target - p->actual;

	// === 计算积分项 ===
	if (p->Ki != 0)
	{
		p->integral += p->error;
		// === 积分限幅 ===
		if (p->integral > p->integ_limit) p->integral = p->integ_limit;
		if (p->integral < -p->integ_limit) p->integral = -p->integ_limit;
	}
	else p->integral = 0;

	// p->output = p->Kp*p->error + p->Ki*p->integral + p->Kd*p->derivative;//普通
	p->output = p->Kp * p->error 
			  + p->Ki * p->integral 
			  - p->Kd * (p->actual - p->actual_last); // 微分先行
	p->actual_last = p->actual;

	// === 死区处理 ===
	if (p->output > -p->Out_Offset && p->output < p->Out_Offset)
	{
		if (p->output < 1e-6f && p->output > -1e-6f)
		{
			p->output = 0;
		}
		else
		{
			p->output = p->Out_Offset * (p->output / fabs(p->output));
		}
	}

	// 限幅后输出
	if (p->output > p->Out_Max) p->output = p->Out_Max;
	if (p->output < p->Out_Min) p->output = p->Out_Min;
}
#endif

