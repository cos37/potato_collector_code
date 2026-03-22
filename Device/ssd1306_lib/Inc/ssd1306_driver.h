#ifndef __SSD1306_DRIVER_H__
#define __SSD1306_DRIVER_H__


#include <stdint.h>

// typedef struct
// {
//     uint8_t date[128]
//     uint8_t dirty_line[2];
//     uint8_t dirty_flag;
// } SSD1306_Page; 
void SSD1306_Driver_Init(void);
void SSD1306_Driver_WriteChar(uint8_t line, uint8_t page, char ch);
void SSD1306_Driver_WriteString(uint8_t line, uint8_t page, char *str,uint8_t len);
void SSD1306_Driver_WriteNums(uint8_t line, uint8_t page, uint32_t num);
void SSD1306_Driver_WriteIntNums(uint8_t line, uint8_t page, int32_t num);
void SSD1306_Driver_SetGRAMzero(uint8_t page);
void SSD1306_Driver_Update(void);

#endif  
