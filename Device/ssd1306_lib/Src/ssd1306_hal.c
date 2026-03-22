#include "ssd1306_hal.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;


uint8_t SSD1306_DMA_Buffer[1100];
uint16_t write_len= 0 ;



#define SSD1306_SCL_PIN GPIO_PIN_6
#define SSD1306_SCL_PORT GPIOB
#define SSD1306_SDA_PIN GPIO_PIN_7
#define SSD1306_SDA_PORT GPIOB


#define OLED_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        0x78     /* 7-bit 地址 0x3C 左移一位 */

static void SSD1306_I2C_Write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};          /* reg = 0x00 命令，0x40 数据 */
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

void SSD1306_WriteCommand(uint8_t cmd)
{
    SSD1306_I2C_Write(0x00, cmd);
}

void SSD1306_WriteData(uint8_t dat)
{
    SSD1306_I2C_Write(0x40, dat);
}

void SSD1306_WriteArray(uint8_t *date,uint8_t len)
{
    uint8_t buff[len+1];
    buff[0]=0x40;
	
    memcpy(buff+1,date,len);
    HAL_I2C_Master_Transmit(&hi2c1,SSD1306_I2C_ADDR,buff,len+1,HAL_MAX_DELAY);
}

void SSD1306_SetCursor(uint8_t Y, uint8_t X)
{
	SSD1306_WriteCommand(0xB0 | Y);					//设置Y位置
	SSD1306_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	SSD1306_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}


void SSD1306_ClearScreen(void)
{
	
    memset(SSD1306_DMA_Buffer,0x00,128);
	for(uint8_t i=0;i<8;i++)
	{
		SSD1306_SetCursor(i,0);
        SSD1306_WriteArray(SSD1306_DMA_Buffer,128);
	}
}



