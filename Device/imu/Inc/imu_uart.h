#ifndef __IMU_UART_H_
#define __IMU_UART_H_

#include "fixpoint.h"

void UART2_Receive_IT_Start(void);
float get_yaw_float(void);
fp16_int32_t get_yaw_fp16(void);
#endif
