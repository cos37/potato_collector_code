#ifndef __BUJIN_H
#define __BUJIN_H

#include "stm32f1xx_hal.h"      // 仅改这一行
#include <stdint.h>

#define USE_CAN1
#ifdef USE_UART1
/* 新增DMA接口 */
void Motor_Buf_Init(void);
void Motor_Set_Vel(uint8_t id, uint8_t dir, uint16_t vel);
void DMA_State_Machine(void);
#endif

#ifdef USE_CAN1
/* CAN接口 */
void CAN1_Init(void);
#endif

/* 函数声明与原来完全一致 */
void Emm_V5_En_Control(uint8_t addr, FlagStatus state, FlagStatus snF);
void Emm_V5_Stop_Now(uint8_t addr, FlagStatus snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, FlagStatus snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, FlagStatus raF, FlagStatus snF);
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, FlagStatus svF, uint8_t ctrl_mode);
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
void Emm_V5_Synchronous_motion(uint8_t addr);
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc);

#endif
