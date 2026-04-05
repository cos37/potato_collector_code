#ifndef _SYS_H__
#define _SYS_H__

#include <stdint.h>

void Sys_Init(void);
void Sys_StartTimer(uint16_t interval_ms, void (*task)(void));
void Sys_SoftTime_Start(void (*task)(void));
void CloseTimer2(void);
#endif
