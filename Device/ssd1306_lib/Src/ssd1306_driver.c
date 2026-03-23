#include "ssd1306_driver.h"
#include "ssd1306_hal.h"
#include "ssd1306_font.h"
#include <string.h>
#include <stdint.h>
#include "i2c.h"

#define SSD1306_WINDOW_W 128
#define SSD1306_WINDOW_H 8
#define SSD1306_PAGES 8
#define SSD1306_WIDTH 128
#define SSD1306_I2C_ADDR        0x78     /* 7-bit 地址 0x3C 左移一位 */



static uint8_t SSD1306_GRAM[8][128] = {0};
static uint8_t dirity_page[8] = {0};
static uint8_t dirity_line[8][2] = {0};


void SSD1306_Dirty_Init(void)
{
    for(uint8_t i = 0; i < SSD1306_PAGES; i++) {
        dirity_line[i][0] = SSD1306_WINDOW_W; // 最小列初始化为最大
        dirity_line[i][1] = 0;             // 最大列初始化为最小
    }
}

uint8_t u32_to_str(uint32_t num, char *str)
{
    uint8_t i = 0;
    char temp[10];
    
    
    // 倒序存入临时数组
    while(num) {
        temp[i++] = num % 10 + '0';
        num /= 10;
    }
    
    // 反转到目标字符串
    uint8_t len = i;
    for(uint8_t j = 0; j < len; j++) {
        str[j] = temp[len - 1 - j];
    }
    str[len] = '\0';
    
    return len;
}

uint8_t int32_to_str(int32_t num, char *str)
{
    uint8_t i = 0;
    
    if(num < 0) {
        str[i++] = '-';
        num = -num;  // 注意：INT32_MIN 会溢出，见下方
    }
    
    uint8_t digits = u32_to_str((uint32_t)num, str + i);
    return i + digits;  // 总长度 = 符号位 + 数字位
}

void SSD1306_ClearScreen(void)
{
	for(uint8_t i=0;i<8;i++)
	{
		memset(SSD1306_GRAM[i],0x00,128);
		SSD1306_SetCursor(i,0);
        SSD1306_WriteArray(SSD1306_GRAM[i],128);
	}
}

void SSD1306_Driver_Init(void)
{

	
    HAL_Delay(100);  // 延时以确保显示稳定，实际使用中可以去掉或调整
	
    SSD1306_HAL_Init();

	SSD1306_WriteCommand(0xAE);	//关闭显示
	
	SSD1306_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	SSD1306_WriteCommand(0x80);
	
	SSD1306_WriteCommand(0xA8);	//设置多路复用率
	SSD1306_WriteCommand(0x3F);
	
	SSD1306_WriteCommand(0xD3);	//设置显示偏移
	SSD1306_WriteCommand(0x00);
	
	SSD1306_WriteCommand(0x40);	//设置显示开始行
	
	SSD1306_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	SSD1306_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	SSD1306_WriteCommand(0xDA);	//设置COM引脚硬件配置
	SSD1306_WriteCommand(0x12);
	
	SSD1306_WriteCommand(0x81);	//设置对比度控制
	SSD1306_WriteCommand(0xCF);

	SSD1306_WriteCommand(0xD9);	//设置预充电周期
	SSD1306_WriteCommand(0xF1);

	SSD1306_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	SSD1306_WriteCommand(0x30);

	SSD1306_WriteCommand(0xA4);	//设置整个显示打开/关闭

	SSD1306_WriteCommand(0xA6);	//设置正常/倒转显示

	SSD1306_WriteCommand(0x8D);	//设置充电泵
	SSD1306_WriteCommand(0x14);

	SSD1306_WriteCommand(0xAF);	//开启显示
    
    SSD1306_ClearScreen();
    SSD1306_Dirty_Init();
}

/**
 * @brief 显示一个字符
 * @param x 字符的横坐标
 * @param y 字符的纵坐标
 * @param ch 要显示的字符
 * 
 * @note 字符的高度为8像素，宽度为6像素,x,y为字符的左上角坐标，使用6x13字体
 */

void SSD1306_Driver_WriteChar(uint8_t line, uint8_t page, char ch)
{

    dirity_page[page] = 1;

    // 设置脏页和脏行
    if(line < dirity_line[page][0]) { //  只有更小时才更新
        dirity_line[page][0] = line;
    }
    if((line + 6) > dirity_line[page][1]) { //  只有更大时才更新
        dirity_line[page][1] = line + 6;
    }

    const uint8_t *aim_ch = SSD1306_F6x8[ch - ' '];

    for(uint8_t i = 0; i < 6; i++)
    {
        SSD1306_GRAM[page][line + i] = aim_ch[i];
    }
}

void SSD1306_Driver_WriteString(uint8_t line, uint8_t page, char *str,uint8_t len)
{

    for(uint8_t i = 0; i < len; i++)
    {
        SSD1306_Driver_WriteChar(line, page, *str);
        line += 6;
        str++;
    }

}

uint8_t SSD1306_Driver_WriteNums(uint8_t line, uint8_t page, uint32_t num)
{
    char str[10];
    uint8_t len = u32_to_str(num, str);
    SSD1306_Driver_WriteString(line, page, str, len);
    return len;
}

uint8_t SSD1306_Driver_WriteIntNums(uint8_t line, uint8_t page, int32_t num)
{
    char str[12];  // 最大11位 + 负号
    uint8_t len = int32_to_str(num, str);
    SSD1306_Driver_WriteString(line, page, str, len); 
	
    return len;
}

void SSD1306_Driver_SetGRAMzero(uint8_t page)
{
    memset(SSD1306_GRAM[page], 0, SSD1306_WINDOW_W);
    dirity_page[page] = 1;
    dirity_line[page][0] = 0;
    dirity_line[page][1] = SSD1306_WINDOW_W;
}

void SSD1306_Driver_Update(void)
{
    uint8_t len=0;
    for(uint8_t i=0;i<8;i++)
    {
        if(dirity_page[i])
        {
            len=dirity_line[i][1]-dirity_line[i][0];
            SSD1306_SetCursor(i,dirity_line[i][0]);
            SSD1306_WriteArray(SSD1306_GRAM[i]+dirity_line[i][0],len);//如果SSD1306_WriteArray以后用DMA发送的话，这段一定要改
            dirity_page[i]=0;                 //把SSD1306_GRAM[i]写到buff里
        }
    }
}

