#ifndef __FLASH_H__
#define __FLASH_H__


#include "fixpoint.h"

void Flash_Read(fp16_int32_t* kp, fp16_int32_t* ki, fp16_int32_t* kd);
void Flash_Save(fp16_int32_t kp, fp16_int32_t ki, fp16_int32_t kd);
#endif /* __FLASH_H__ */