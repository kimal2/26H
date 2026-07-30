#include <stdarg.h>
#include <stdio.h>

#include "OLED.h"
#include "oledfont.h"

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define OLED_ADDR 0x78


soft_iic_obj_t OLED;



void OLED_WR_Byte(uint8_t dat,uint8_t mode)
{
    IIC_Start(&OLED);
    IIC_Send_Byte(&OLED, OLED.addr);
    if(IIC_Wait_Ack(&OLED))
    {
        IIC_Stop(&OLED);
        return;
    }
    if(mode){IIC_Send_Byte(&OLED, 0x40);}
    else{IIC_Send_Byte(&OLED, 0x00);}
    if(IIC_Wait_Ack(&OLED))
    {
        IIC_Stop(&OLED);
        return;
    }
    IIC_Send_Byte(&OLED, dat);
    if(IIC_Wait_Ack(&OLED))
    {
        IIC_Stop(&OLED);
        return;
    }
    IIC_Stop(&OLED);
}

static uint8_t OLED_WR_Bytes(const uint8_t *buf, uint16_t len, uint8_t mode)
{
    uint16_t i;

    IIC_Start(&OLED);

    IIC_Send_Byte(&OLED, OLED.addr);
    if (IIC_Wait_Ack(&OLED)) {
        IIC_Stop(&OLED);
        return 1;
    }

    if (mode) {
        IIC_Send_Byte(&OLED, 0x40);   // data
    } else {
        IIC_Send_Byte(&OLED, 0x00);   // command
    }

    if (IIC_Wait_Ack(&OLED)) {
        IIC_Stop(&OLED);
        return 1;
    }

    for (i = 0; i < len; i++) {
        IIC_Send_Byte(&OLED, buf[i]);
        if (IIC_Wait_Ack(&OLED)) {
            IIC_Stop(&OLED);
            return 1;
        }
    }

    IIC_Stop(&OLED);
    return 0;
}

void OLED_Clear(void)
{
    uint8_t i;
    uint8_t clear_buf[128] = {0};

    for (i = 0; i < 8; i++) {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);

        OLED_WR_Bytes(clear_buf, 128, OLED_DATA);
    }
}

void OLED_Init(void)
{
    soft_iic_init(&OLED, OLED_SDA_GPIO_Port, OLED_SDA_Pin,
                  OLED_SCL_GPIO_Port, OLED_SCL_Pin, OLED_ADDR);

    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
} 

void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63				 
//sizey:选择字体 6x8  8x16
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey)
{      	
    uint8_t c=0,sizex=sizey/2;
    uint16_t i=0,size1;
    if(sizey==8)size1=6;
    else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    c=chr-' ';//得到偏移后的值
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++)
    {
        if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8字号
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16字号
        //		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//用户添加字号
        else return;
    }
}

void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey)
{
    uint8_t j=0;
    while (chr[j]!='\0')
    {		
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8)x+=6;
        else x+=sizey/2;
    }
}

void OLED_Printf(uint8_t x, uint8_t y, uint8_t size,
                 const char *format, ...)
{
    char buffer[32];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    OLED_ShowString(x, y, (uint8_t *)buffer, size);
}
