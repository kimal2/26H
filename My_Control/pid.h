#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float Kp; //  比例、积分、微分系数
    float Ki;
    float Kd;

    float Out_Max;      // 输出限幅 (±out_limit)
    float Out_Min; //  输出最小值
    float integ_limit;    // 积分限幅

    float target; //  目标值，期望达到的值
    float actual; //  实际值，当前系统实际的值
    float actual_last; //  上一次的实际值，用于计算微分项
    float error; //  误差，目标值与实际值的差值
    float last_error; //  上一次的误差值
    float prev_error; //  上上次的误差值，增量式PID用

    float integral; //  积分项，用于消除系统的稳态误差
    float int_threshold; //  积分阈值，当误差小于此值时才进行积分
    float derivative; //  微分项，用于预测误差变化趋势
    float output; //  控制器输出值，经过PID计算后的最终结果

    float Out_Offset; //  输出偏移量，用于调整输出值
} PID_t;

void PID_Calc(PID_t *p);
void PID_Clear(PID_t *p);
void PID_CalcINC(PID_t *p);

#endif
