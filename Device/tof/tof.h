#ifndef __TOF_H_
#define __TOF_H_
/**
 * @brief 这是tof激光测距模块的文件
 */
#include <stdint.h>


void TOF_Init(void);
void TOF_LOOP(void);
uint16_t TOF_GetDistance(uint8_t adr);

#endif
