#include "Emm_V5.h"

static void Emm_V5_Send(StepMotor_t *M, uint16_t size)
{
    (void)HAL_UART_Transmit_DMA(M->huart, M->tx_buf, size);
}

void Emm_V5_Trig_Encoder_Cal(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x06;
    M->tx_buf[2] = 0x45;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Reset_Motor(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x08;
    M->tx_buf[2] = 0x97;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Reset_CurPos_To_Zero(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x0A;
    M->tx_buf[2] = 0x6D;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Reset_Clog_Pro(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x0E;
    M->tx_buf[2] = 0x52;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Restore_Motor(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x0F;
    M->tx_buf[2] = 0x5F;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_En_Control(StepMotor_t *M, bool state, bool snF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0xF3;
    M->tx_buf[2] = 0xAB;
    M->tx_buf[3] = (uint8_t)state;
    M->tx_buf[4] = (uint8_t)snF;
    M->tx_buf[5] = 0x6B;

    Emm_V5_Send(M, 6U);
}

void Emm_V5_Vel_Control(StepMotor_t *M,
                        uint8_t dir,
                        uint16_t vel,
                        uint8_t acc,
                        bool snF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0xF6;
    M->tx_buf[2] = dir;
    M->tx_buf[3] = (uint8_t)(vel >> 8);
    M->tx_buf[4] = (uint8_t)vel;
    M->tx_buf[5] = acc;
    M->tx_buf[6] = (uint8_t)snF;
    M->tx_buf[7] = 0x6B;

    Emm_V5_Send(M, 8U);
}

void Emm_V5_Pos_Control(StepMotor_t *M,
                        uint8_t dir,
                        uint16_t vel,
                        uint8_t acc,
                        uint32_t clk,
                        uint8_t raF,
                        bool snF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0xFD;
    M->tx_buf[2] = dir;
    M->tx_buf[3] = (uint8_t)(vel >> 8);
    M->tx_buf[4] = (uint8_t)vel;
    M->tx_buf[5] = acc;
    M->tx_buf[6] = (uint8_t)(clk >> 24);
    M->tx_buf[7] = (uint8_t)(clk >> 16);
    M->tx_buf[8] = (uint8_t)(clk >> 8);
    M->tx_buf[9] = (uint8_t)clk;
    M->tx_buf[10] = raF;
    M->tx_buf[11] = (uint8_t)snF;
    M->tx_buf[12] = 0x6B;

    Emm_V5_Send(M, 13U);
}

void Emm_V5_Stop_Now(StepMotor_t *M, bool snF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0xFE;
    M->tx_buf[2] = 0x98;
    M->tx_buf[3] = (uint8_t)snF;
    M->tx_buf[4] = 0x6B;

    Emm_V5_Send(M, 5U);
}

void Emm_V5_Origin_Set_O(StepMotor_t *M, bool svF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x93;
    M->tx_buf[2] = 0x88;
    M->tx_buf[3] = (uint8_t)svF;
    M->tx_buf[4] = 0x6B;

    Emm_V5_Send(M, 5U);
}

void Emm_V5_Origin_Trigger_Return(StepMotor_t *M,
                                  uint8_t o_mode,
                                  bool snF)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x9A;
    M->tx_buf[2] = o_mode;
    M->tx_buf[3] = (uint8_t)snF;
    M->tx_buf[4] = 0x6B;

    Emm_V5_Send(M, 5U);
}

void Emm_V5_Origin_Interrupt(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x9C;
    M->tx_buf[2] = 0x48;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Read_Sys_Params(StepMotor_t *M, SysParams_t s)
{
    uint8_t command;

    switch (s) {
        case S_VBUS:  command = 0x24; break;
        case S_CBUS:  command = 0x26; break;
        case S_CPHA:  command = 0x27; break;
        case S_ENCO:  command = 0x29; break;
        case S_CLKC:  command = 0x30; break;
        case S_ENCL:  command = 0x31; break;
        case S_CLKI:  command = 0x32; break;
        case S_TPOS:  command = 0x33; break;
        case S_SPOS:  command = 0x34; break;
        case S_VEL:   command = 0x35; break;
        case S_CPOS:  command = 0x36; break;
        case S_PERR:  command = 0x37; break;
        case S_VBAT:  command = 0x38; break;
        case S_TEMP:  command = 0x39; break;
        case S_FLAG:  command = 0x3A; break;
        case S_OFLAG: command = 0x3B; break;
        case S_OAF:   command = 0x3C; break;
        case S_PIN:   command = 0x3D; break;
        default: return;
    }

    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = command;
    M->tx_buf[2] = 0x6B;

    Emm_V5_Send(M, 3U);
}

void Emm_V5_Read_System_State_Params(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x43;
    M->tx_buf[2] = 0x7A;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}

void Emm_V5_Read_Motor_Conf_Params(StepMotor_t *M)
{
    M->tx_buf[0] = M->ID;
    M->tx_buf[1] = 0x42;
    M->tx_buf[2] = 0x6C;
    M->tx_buf[3] = 0x6B;

    Emm_V5_Send(M, 4U);
}
