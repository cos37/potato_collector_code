#ifndef __HC_SR04_H_
#define __HC_SR04_H_

#include <stdint.h>
#include "fixpoint.h"

void SR04_Init(void);
void SR04_LOOP(void);
uint8_t SR04_GetFlag(uint8_t dir);
fp16_int32_t SR04_GetDistance(uint8_t dir);
#endif
