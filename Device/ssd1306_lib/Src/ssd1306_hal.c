#include "ssd1306_hal.h"
#include "i2c.h"
#include "main.h"
#include <string.h>
#include "stm32f1xx_hal.h"


extern I2C_HandleTypeDef hi2c1;



#define SSD1306_SCL_PIN GPIO_PIN_6
#define SSD1306_SCL_PORT GPIOB
#define SSD1306_SDA_PIN GPIO_PIN_7
#define SSD1306_SDA_PORT GPIOB

#define OLED_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        0x78     /* 7-bit 地址 0x3C 左移一位 */


void SSD1306_HAL_Init(void)
{
    // //使能I2C1时钟
	// __HAL_RCC_DMA1_CLK_ENABLE();
	// __HAL_RCC_I2C1_CLK_ENABLE();    
    // __HAL_RCC_GPIOB_CLK_ENABLE();   
	
	// //SSD1306 Pin角初始化
	// GPIO_InitTypeDef GPIO_I2C_PIN={0};
	// GPIO_I2C_PIN.Pin=SSD1306_SDA_PIN|SSD1306_SCL_PIN;   
	// GPIO_I2C_PIN.Mode=GPIO_MODE_AF_OD;
	// GPIO_I2C_PIN.Pull=GPIO_PULLUP;
	// GPIO_I2C_PIN.Speed = GPIO_SPEED_FREQ_HIGH;
	
	// HAL_GPIO_Init(SSD1306_SDA_PORT,&GPIO_I2C_PIN);
	
	// hdma_oled_tx.Instance                 = DMA1_Channel6;
	// hdma_oled_tx.Init.Direction=DMA_MEMORY_TO_PERIPH;
	// hdma_oled_tx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
	// hdma_oled_tx.Init.MemInc=DMA_MINC_ENABLE;
	// hdma_oled_tx.Init.Mode=DMA_NORMAL;
	// hdma_oled_tx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
	// hdma_oled_tx.Init.PeriphInc=DMA_PINC_DISABLE;
	// hdma_oled_tx.Init.Priority=DMA_PRIORITY_LOW;
	
	// if (HAL_DMA_Init(&hdma_oled_tx) != HAL_OK)
    // {
    //     Error_Handler();
    // }

    // hi2c1.Instance = I2C1;
    // hi2c1.Init.ClockSpeed = 400000;       
    // hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    // hi2c1.Init.OwnAddress1 = 0;           // 主机模式设为 0
    // hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // hi2c1.Init.OwnAddress2 = 0;
    // hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    // // 4. 初始化 I2C
    // if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    //     Error_Handler();
    // }
    
    // __HAL_LINKDMA(&hi2c1, hdmatx, hdma_oled_tx);
    // hi2c1.Instance->CR2 |= I2C_CR2_DMAEN;
	
	// HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}



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





