/*
 **********************************************************
 * @file        : imu_driver.h
 * @brief       : IMU驱动接口头文件
 * @author      : cos37
 * @date        : 2024-06-01
 * @version     : V1.0
 **********************************************************
 * @attention
 *
 * 1. 本文件定义了IMU传感器的驱动接口，供上层应用调用。
 * 2. 包含必要的头文件和函数声明，具体实现由imu_driver.c完成。
 * 3. 使用前请确保I2C总线已正确初始化，并连接好IMU传感器。
 *
 **********************************************************
 * @history
 *
 *
 **********************************************************
    */
#ifndef __IMU_DRIVER_H__
#define __IMU_DRIVER_H__
#include "fixpoint.h"

typedef enum {
    IMU_IDLE = 0,
    IMU_BUSY,
    IMU_ERROR,
} IMU_State_t;

typedef struct {
    IMU_State_t state;
    fp16_int32_t yaw;
    fp16_int32_t pitch;
    fp16_int32_t roll;
} IMU_Handle_t;

extern IMU_Handle_t imuHandle;

void IMU_Calibrate(void);
void IMU_Reboot(void);
void IMU_ReadAccel(float *ax,float *ay,float *az);
void IMU_GetEuler(fp16_int32_t *yaw, fp16_int32_t *pitch, fp16_int32_t *roll);
void IMU_DateProcess(void);
void IMU_Driver_GetEulerIT(void);
void IMU_ReadRegIT_Callback(void);

#endif /* __IMU_DRIVER_H__ */
