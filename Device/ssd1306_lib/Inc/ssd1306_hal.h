#ifndef __SSD1306_HAL_H__
#define __SSD1306_HAL_H__


#include <stdint.h>
#include <string.h>

void SSD1306_HAL_Init(void);
void SSD1306_WriteCommand(uint8_t cmd);
void SSD1306_WriteData(uint8_t dat);
void SSD1306_WriteArray(uint8_t *date,uint8_t len);
void SSD1306_SetCursor(uint8_t Y, uint8_t X);
void SSD1306_ClearScreen(void);

#endif
