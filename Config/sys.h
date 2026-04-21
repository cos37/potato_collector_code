#ifndef _SYS_H__
#define _SYS_H__

#include <stdint.h>
#include "usart.h"

void Sys_Init(void);
void Sys_StartTimer(uint16_t interval_ms, void (*task)(void));
void Sys_SoftTime_Start(void (*task)(void));
void CloseTimer2(void);

void Sys_startTimer3(uint32_t interval_ms,void (*task)(void));
void CloseTimer3(void);
void Sys_Base_us_Init(void);
uint32_t Get_Tick_us(void);

void SYS_EnableUARTx_RXIT_CB(UART_HandleTypeDef *huart, void (*callback)(void));
void SYS_DisableUARTx_RXIT_CB(UART_HandleTypeDef *huart);
#endif

