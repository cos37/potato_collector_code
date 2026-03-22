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


void IMU_ReadAccel(float *ax,float *ay,float *az);
fp16_int32_t IMU_GetYaw(void);
#endif /* __IMU_DRIVER_H__ */
