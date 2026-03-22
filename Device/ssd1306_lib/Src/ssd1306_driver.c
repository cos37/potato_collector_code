#include "ssd1306_driver.h"
#include "ssd1306_hal.h"
#include "ssd1306_font.h"
#include "stm32f1xx_hal.h"

#define SSD1306_WINDOW_W 128
#define SSD1306_WINDOW_H 8
#define SSD1306_PAGES 8
#define SSD1306_WIDTH 128
#define SSD1306_I2C_ADDR        0x78     /* 7-bit 地址 0x3C 左移一位 */


extern I2C_HandleTypeDef hi2c1;
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

static uint8_t u32_to_str(uint32_t num, char *str)
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

static uint8_t int32_to_str(int32_t num, char *str)
{
    uint8_t i = 0;
    
    if(num < 0) {
        str[i++] = '-';
        num = -num;  // 注意：INT32_MIN 会溢出，见下方
    }
    
    uint8_t digits = u32_to_str((uint32_t)num, str + i);
    return i + digits;  // 总长度 = 符号位 + 数字位
}

void SSD1306_Driver_Init(void)
{
    

    HAL_Delay(500);  // 延时以确保显示稳定，实际使用中可以去掉或调整
	

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

void SSD1306_Driver_WriteChar(uint8_t x, uint8_t y, char ch)
{
    if(ch < ' ' || ch > '~') {
        return; // 不显示不可打印字符
    }
    if(x >= SSD1306_WIDTH || y >= SSD1306_WINDOW_H * 8) {
        return; // 超出显示范围
    }

    uint8_t page = y / 8; // 计算页码
    uint8_t page_offset = y % 8; // 计算页内偏移
    const uint8_t *char_bitmap = SSD1306_F6x8[ch - ' ']; // 获取字符的位图数据
    uint8_t col = x;     // 列号即为x坐标
    
    for(uint8_t i = 0; i < 6; i++) { // 字符宽度为6像素 
        uint8_t keep_up = SSD1306_GRAM[page][col + i] & (0xFF >> page_offset); 
        uint8_t keep_down = SSD1306_GRAM[page + 1][col + i] & (0xFF << (8 - page_offset));
        SSD1306_GRAM[page+1][col + i] = (char_bitmap[i] >> (page_offset)) | keep_down; 
        SSD1306_GRAM[page][col + i] = (char_bitmap[i] << (8 - page_offset)) | keep_up;
    }
    
    // 更新脏页和脏行信息
    dirity_page[page] = 1;
    if(page_offset > 0) {
        dirity_page[page + 1] = 1; // 如果字符跨页，下一页也标记为脏页

        if((col + 6) > dirity_line[page + 1][1]) {
            dirity_line[page + 1][1] = col + 6; // 更新下一页的最大列
        }
        if(col < dirity_line[page + 1][0]) {
            dirity_line[page + 1][0] = col; // 更新下一页的最小列
        } 
    }
    if(col < dirity_line[page][0]) {
        dirity_line[page][0] = col; // 更新最小列
    }
    if((col + 6) > dirity_line[page][1]) {
        dirity_line[page][1] = col + 6; // 更新最大列
    }



}

void SSD1306_Driver_WriteString(uint8_t x, uint8_t y, char *str,uint8_t len)
{

    for(uint8_t i = 0; i < len; i++)
    {
        SSD1306_Driver_WriteChar(x, y, *str);
        x += 6;
        str++;
    }

}

void SSD1306_Driver_WriteNums(uint8_t x, uint8_t y, uint32_t num)
{
    char str[10];
    uint8_t len = u32_to_str(num, str);
    SSD1306_Driver_WriteString(x, y, str, len);
}

void SSD1306_Driver_WriteIntNums(uint8_t x, uint8_t y, int32_t num)
{
    char str[12];  // 最大11位 + 负号
    uint8_t len = int32_to_str(num, str);
    SSD1306_Driver_WriteString(x, y, str, len); 
	
}

void SSD1306_Driver_WritePoint(uint8_t x, uint8_t y)
{
    uint8_t page = y / 8;
    SSD1306_GRAM[page][x] |= (1 << (y % 8)); // 设置对应位
    dirity_page[page] = 1; // 标记页为脏页
    if(x < dirity_line[page][0]) {
        dirity_line[page][0] = x; // 更新最小列
    }
    if((x + 1) > dirity_line[page][1]) {
        dirity_line[page][1] = x + 1; // 更新最大列
    }
}

void SSD1306_Driver_Setbite(uint8_t x,uint8_t page,uint8_t bite)
{
    SSD1306_GRAM[page][x] |= bite; // 设置对应位
    dirity_page[page] = 1; // 标记页为脏页
    if(x < dirity_line[page][0]) {
        dirity_line[page][0] = x; // 更新最小列
    }
    if((x + 1) > dirity_line[page][1]) {
        dirity_line[page][1] = x + 1; // 更新最大列
    }
}
void SSD1306_Driver_IMG(void)
{
    for(uint8_t i=0;i<8;i++)
    {
        memcpy(SSD1306_GRAM[i],face_img[i],128);
        dirity_page[i]=1;
        dirity_line[i][0]=0;
        dirity_line[i][1]=128;
    }
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


