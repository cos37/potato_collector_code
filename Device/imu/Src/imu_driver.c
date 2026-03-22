#include "imu_driver.h"
#include "imu_i2c.h" /* 包含 IMU I2C 通信接口 */
#include <stdint.h> /* 包含标准整数类型定义 */
#include <string.h>

/* I2C设备地址（7位） */
#define IMU_ADDR_7BIT           0x23
#define IMU_ADDR_WRITE          0x46     /* 0x46 */
#define IMU_ADDR_READ           0x47     /* 0x47 */


/****** IMU功能码定义 ******/
#define IMU_FUNC_VERSION        0x01  // 版本信息
#define IMU_FUNC_RAW_ACCEL      0x04  // 原始加速度数据
#define IMU_FUNC_RAW_GYRO       0x0A  // 原始陀螺仪数据
#define IMU_FUNC_RAW_MAG        0x10  // 原始磁力计数据
#define IMU_FUNC_QUAT           0x16  // 四元数数据
#define IMU_FUNC_EULER          0x26  // 欧拉角数据
#define IMU_FUNC_BARO           0x32  // 气压计数据
#define IMU_FUNC_CALIB_IMU      0x70  // IMU校准
#define IMU_FUNC_CALIB_MAG      0x71  // 磁力计校准
#define IMU_FUNC_CALIB_BARO     0x72  // 气压计校准
#define IMU_FUNC_CALIB_TEMP     0x73  // 温度计校准
#define IMU_FUNC_REQUEST_DATA   0x80  // 请求数据
#define IMU_FUNC_RETURN_STATE   0x81  // 返回状态
#define IMU_FUNC_RESET_FLASH    0xA0  // 重置Flash
#define IMU_FUNC_REBOOT_DEVICE  0xA1  // 重启设备

/*
 * @brief       : IMU驱动函数实现
 * @author      : cos37
 * @date        : 2024-06-01
 * @version     : V1.0
 */

/* ======================================================== */
/* 函数域：IMU驱动接口实现                                       */

void IMU_ReadAccel(float *ax,float *ay,float *az)
{
    uint8_t rawData[6];
    IMU_ReadReg(IMU_FUNC_RAW_ACCEL, rawData, 6);
    //小段数据拼接成16位有符号整数
    *ax = ((int16_t)(rawData[0] << 8 | rawData[1]));
    *ay = ((int16_t)(rawData[2] << 8 | rawData[3]));
    *az = ((int16_t)(rawData[4] << 8 | rawData[5]));

    // 1. 读寄存器内容
    float scale = 16.0f / 32767.0f;
    *ax = *ax * scale;
    *ay = *ay * scale;
    *az = *az * scale;
}

float IMU_GetYaw(void)
{
    uint8_t buf[12];
    float euler[3]; // 0:Roll, 1:Pitch, 2:Yaw

    // 读取 0x26 开始的 12 字节
    IMU_ReadReg(IMU_FUNC_EULER, buf, 12);

    // 解析浮点数 (小端模式)
    memcpy(&euler[0], &buf[0], 4);
    memcpy(&euler[1], &buf[4], 4);
    memcpy(&euler[2], &buf[8], 4);

    return euler[2]; // 返回 Yaw
}