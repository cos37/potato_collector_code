#ifndef _MECANUM_H_
#define _MECANUM_H_

#include "fixpoint.h"

void Mecanum_Init(void);
void Mecanum_kinematics(fp16_int32_t vx,fp16_int32_t vy,fp16_int32_t omega);
void Mecanum_Update(void);
#endif
