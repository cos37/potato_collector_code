#include "imu_i2c.h"
#include "i2c.h" /* 包含 I2C 发送接口 */
#include <stdint.h> /* 包含标准整数类型定义 */

/* I2C设备地址（7位） */
#define IMU_ADDR_7BIT           0x23
#define IMU_ADDR_WRITE          0x46     /* 0x46 */
#define IMU_ADDR_READ           0x47     /* 0x47 */




// 2. 写寄存器内容
void IMU_WriteReg(uint8_t regAddr, uint8_t *pData, uint16_t len) {
    HAL_I2C_Mem_Write(&hi2c2, 
                      (uint16_t)(IMU_ADDR_7BIT << 1), 
                      (uint16_t)regAddr, 
                      I2C_MEMADD_SIZE_8BIT, 
                      pData, 
                             len, 
                             100);
}

void IMU_ReadReg(uint8_t regAddr, uint8_t *pData, uint16_t len) {
    HAL_I2C_Mem_Read(&hi2c2, 
                      (uint16_t)(IMU_ADDR_7BIT << 1), 
                      (uint16_t)regAddr, 
                      I2C_MEMADD_SIZE_8BIT, 
                      pData, 
                      len, 
                            100); // 100ms timeout
}

void IMU_ReadRegIT(uint8_t regAddr, uint8_t *pData, uint16_t len) {
    HAL_I2C_Mem_Read_IT(&hi2c2, 
                         (uint16_t)(IMU_ADDR_7BIT << 1), 
                         (uint16_t)regAddr, 
                         I2C_MEMADD_SIZE_8BIT, 
                         pData, 
                         len);
}

