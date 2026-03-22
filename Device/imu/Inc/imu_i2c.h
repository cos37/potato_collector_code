/*
 **********************************************************
 * @file        : imu_i2c.h
 * @brief       : IMU I2C通信接口头文件
 * @author      : cos37
 * @date        : 2024-06-01
 * @version     : V1.0
 **********************************************************
 * @attention
 *
 * 1. 本文件定义了IMU传感器的I2C通信接口，供上层应用调用。
 * 2. 包含必要的头文件和函数声明，具体实现由imu_i2c.c完成。
 * 3. 使用前请确保I2C总线已正确初始化，并连接好IMU传感器。
 *
 **********************************************************
 * @history
 *
 *
 **********************************************************
    */
#ifndef _IMU_I2C_H
#define _IMU_I2C_H

#include <stdint.h>

void IMU_WriteReg(uint8_t regAddr, uint8_t *pData, uint16_t len);
void IMU_ReadReg(uint8_t regAddr, uint8_t *pData, uint16_t len);

#endif /* _IMU_I2C_H */
