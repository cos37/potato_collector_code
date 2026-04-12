/*
 * @brief       : IMU驱动函数实现
 * @author      : cos37
 * @date        : 2026-3-23
 * @version     : V1.0
 */

#include "imu_driver.h"
#include "imu_i2c.h" /* 包含 IMU I2C 通信接口 */
#include <stdint.h> /* 包含标准整数类型定义 */
#include <string.h>
#include "fixpoint.h" /* 包含定点数类型和操作函数 */
#include "i2c.h" /* 包含 I2C 发送接口 */
#include "stm32f1xx_hal.h"
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




/* 状态机函数 */
void IMU_IDLE_STAE_Func(void);
void IMU_BUSY_STATE_Func(void);
void IMU_ERROR_STATE_Func(void);

/* 文件私有变量 */
static uint8_t buff[12]; // 用于存储读取的欧拉角数据


/* 工具函数  */
//static inline int32_t buff_to_q16_16(const uint8_t *p) {
//    union {
//        uint32_t u;
//        float f;
//    } conv;
//    conv.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | 
//             ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
//    return (int32_t)(conv.f * 65536.0f);  // 直接转 Q16.16
//}

/* ======================================================== */
/* 函数域：IMU驱动接口实现                                       */

IMU_Handle_t imuHandle = { .state = IMU_IDLE, .yaw = 3<<16, .pitch = 0, .roll = 0 };

void IMU_Calibrate(void)
{
    uint8_t cmd = IMU_FUNC_CALIB_IMU;
    IMU_WriteReg(IMU_FUNC_CALIB_IMU, &cmd, 1);
}
void IMU_Reboot(void)
{
    uint8_t cmd = IMU_FUNC_REBOOT_DEVICE;
    IMU_WriteReg(IMU_FUNC_REBOOT_DEVICE, &cmd, 1);
}

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

void IMU_GetEuler(fp16_int32_t *yaw, fp16_int32_t *pitch, fp16_int32_t *roll)
{
    uint8_t buf[12];
    float euler[3]; // 0:Roll, 1:Pitch, 2:Yaw

    // 读取 0x26 开始的 12 字节
    IMU_ReadReg(IMU_FUNC_EULER, buf, 12);

    // 解析浮点数 (小端模式)
    memcpy(&euler[0], &buf[0], 4);
    memcpy(&euler[1], &buf[4], 4);
    memcpy(&euler[2], &buf[8], 4);

    *yaw = fp16_from_float(euler[2]); // 返回 Yaw
    *pitch = fp16_from_float(euler[1]); // 返回 Pitch
    *roll = fp16_from_float(euler[0]); // 返回 Roll

}
/* ======================================================== */
/**
 * 状态机设计：
 * 1. 定义状态枚举：IDLE, BUSY, ERROR
 * 2. 在函数中根据状态执行不同逻辑
 *   - IDLE: 正常读取数据
 *  - BUSY: 等待或返回忙状态
 *  - ERROR: 返回错误状态
 * 3. 状态转换：根据函数执行结果更新状态
 */

void IMU_DateProcess(void){
    switch (imuHandle.state) {
        case IMU_IDLE:
            // 正常读取数据
            IMU_IDLE_STAE_Func();
            break;
        case IMU_BUSY:
            IMU_BUSY_STATE_Func();
            break;
        case IMU_ERROR:
            IMU_ERROR_STATE_Func();
            break;
        default:
            break;
    }

}

void IMU_IDLE_STAE_Func(void) {
    // 读取数据
    
    HAL_StatusTypeDef status = IMU_ReadRegIT(IMU_FUNC_EULER, buff, 12); // 异步读取欧拉角数据
    if (status != HAL_OK) {
        return; // 读取失败，保持在空闲状态
    }
    imuHandle.state = IMU_BUSY; // 切换到忙状态
}

void IMU_BUSY_STATE_Func(void) {
    // 等待或返回忙状态
    // 这里可以添加超时机制，如果长时间没有完成，可以切换到错误状态
    return;
}

void IMU_ERROR_STATE_Func(void) {
    // 返回错误状态
    // 这里可以添加错误处理逻辑，例如重置设备或记录错误日志
    return;
}

/* 中断函数 */

void IMU_ReadRegIT_Callback(void) {
    // 处理 I2C 读取完成的中断回调
    // 这里可以解析数据并更新状态
    float euler[3];
    memcpy(&euler[0], &buff[0], 4);
    memcpy(&euler[1], &buff[4], 4);
    memcpy(&euler[2], &buff[8], 4);
    if(euler[2]!=0.0f){
        imuHandle.yaw = fp16_from_float(euler[2]);
        imuHandle.pitch = fp16_from_float(euler[1]);
        imuHandle.roll = fp16_from_float(euler[0]);

        imuHandle.state = IMU_IDLE; // 切换回空闲状态

    }

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        IMU_ReadRegIT_Callback();  // 如果用 HAL_I2C_Mem_Read_IT
    }
}

