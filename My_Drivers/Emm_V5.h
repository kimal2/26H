#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include "stdbool.h"

/**********************************************************
***	Emm_V5.0 高性价比闭环步进电机驱动器
***	编写作者：ZHANGDATOU
***	技术支援：张大头闭环伺服
***	淘宝链接：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

#define					ABS(x)							((x) > 0 ? (x) : -(x))

typedef struct StepMotor_t
{
    UART_HandleTypeDef *huart;
    uint8_t ID;
	
	float current_angle;
	float target_angle;

	float target_speed;

	uint8_t tx_buf[20];   // 每个电机独立的发送缓冲区

} StepMotor_t;

typedef enum {
	S_VBUS  = 5,	// 读取总线电压
	S_CBUS  = 6,	// 读取总线电流
	S_CPHA  = 7,	// 读取相电流
	S_ENCO  = 8,	// 读取编码器原始值
	S_CLKC  = 9,	// 读取实时脉冲数
	S_ENCL  = 10,	// 读取经过线性化校准后的编码器值
	S_CLKI  = 11,	// 读取输入脉冲数
	S_TPOS  = 12,	// 读取电机目标位置
	S_SPOS  = 13,	// 读取电机实时设定的目标位置
	S_VEL   = 14,	// 读取电机实时转速
	S_CPOS  = 15,	// 读取电机实时位置
	S_PERR  = 16,	// 读取电机位置误差
	S_VBAT  = 17,	// 读取线圈驱动电池电压（Y42）
	S_TEMP  = 18,	// 读取电机实时温度（Y42）
	S_FLAG  = 19,	// 读取电机状态标志位
	S_OFLAG = 20, // 读取原点状态标志位
	S_OAF   = 21,	// 读取电机状态标志位 + 原点状态标志位（Y42）
	S_PIN   = 22,	// 读取端口状态（Y42）
}SysParams_t;


/**
***********************************************************
***********************************************************
***
***
*** @brief	后注：所有带（Y42）的为Y42专用命令，X42请勿用，否则通用
***
***
***********************************************************
***********************************************************
***/
/**********************************************************
*** 电机基本功能函数
**********************************************************/
// 触发电机线性校准
void Emm_V5_Trig_Encoder_Cal(StepMotor_t *M);
// 重启电机（Y42）
void Emm_V5_Reset_Motor(StepMotor_t *M);
// 将当前位置清零
void Emm_V5_Reset_CurPos_To_Zero(StepMotor_t *M);
// 清除堵转保护
void Emm_V5_Reset_Clog_Pro(StepMotor_t *M);
// 恢复出厂设置
void Emm_V5_Restore_Motor(StepMotor_t *M);
/**********************************************************
*** 运行控制功能
**********************************************************/
// 电机使能控制
void Emm_V5_En_Control(StepMotor_t *M, bool state, bool snF);
// 速度模式控制
void Emm_V5_Vel_Control(StepMotor_t *M, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// 位置模式控制
void Emm_V5_Pos_Control(StepMotor_t *M, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, bool snF);
// 立即单电机停止运动
void Emm_V5_Stop_Now(StepMotor_t *M, bool snF);
/**********************************************************
*** 原点返回功能
**********************************************************/
// 设置单圈上电零点位置
void Emm_V5_Origin_Set_O(StepMotor_t *M, bool svF);
// 触发回零
void Emm_V5_Origin_Trigger_Return(StepMotor_t *M, uint8_t o_mode, bool snF);
// 强制中断并退出回零
void Emm_V5_Origin_Interrupt(StepMotor_t *M);
// 读取回零参数
/**********************************************************
*** 读取系统运行参数
**********************************************************/
// 读取系统参数
void Emm_V5_Read_Sys_Params(StepMotor_t *M, SysParams_t s);

/**********************************************************
*** 获取参数和系统信息
**********************************************************/
// 获取系统状态参数
void Emm_V5_Read_System_State_Params(StepMotor_t *M);
// 获取电机配置参数
void Emm_V5_Read_Motor_Conf_Params(StepMotor_t *M);

#endif
