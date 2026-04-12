#include "stm32f1xx_hal.h"  // 根据实际型号修改
#include <string.h>
#include "flash.h"

#define FLASH_START_ADDRESS     0x0800FC00 
#define FLASH_MAGIC_NUMBER      0x12345678
#define FLASH_DATA_SIZE         12   // 3个int32_t = 12字节
#define FLASH_TOTAL_SIZE        16   // 4字节魔数 + 12字节数据，对齐到16字节

// 存储布局结构
typedef struct {
    uint32_t magic;          // 魔术字，标识数据有效
    int32_t  kp;             // fp16_int32_t 实际存储为int32
    int32_t  ki;
    int32_t  kd;
} Flash_Data_t;


/**
 * @brief 初始化Flash存储，检查是否有有效数据，没有则写入默认值
 * @param default_kp 默认Kp值
 * @param default_ki 默认Ki值
 * @param default_kd 默认Kd值
 * @note 该函数应在系统初始化阶段调用，确保PID参数有合理的初始值
 */



void Flash_Init(fp16_int32_t default_kp, fp16_int32_t default_ki, fp16_int32_t default_kd)
{
    fp16_int32_t kp, ki, kd;
    
    // 尝试读取
    Flash_Read(&kp, &ki, &kd);
    
    // 检查是否为有效数据（通过魔术字判断）
    const Flash_Data_t* flash_ptr = (const Flash_Data_t*)FLASH_START_ADDRESS;
    
    if (flash_ptr->magic != FLASH_MAGIC_NUMBER) {
        // 首次烧录或数据损坏，写入默认值
        Flash_Save(default_kp, default_ki, default_kd);
    }
}

/**
 * @brief 从Flash读取PID参数
 * @param kp 输出：Kp参数指针
 * @param ki 输出：Ki参数指针  
 * @param kd 输出：Kd参数指针
 * @note 如果Flash无有效数据，输出值保持不变
 */
void Flash_Read(fp16_int32_t* kp, fp16_int32_t* ki, fp16_int32_t* kd)
{
    const Flash_Data_t* flash_ptr = (const Flash_Data_t*)FLASH_START_ADDRESS;
    
    // 验证魔术字
    if (flash_ptr->magic == FLASH_MAGIC_NUMBER) {
        *kp = flash_ptr->kp;
        *ki = flash_ptr->ki;
        *kd = flash_ptr->kd;
    }
    // 否则数据无效，保持传入的默认值（调用前需先赋值默认值）
}

/**
 * @brief 保存PID参数到Flash
 * @param kp Kp参数
 * @param ki Ki参数
 * @param kd Kd参数
 * @note 会擦除一页Flash后写入，操作期间关中断
 */
void Flash_Save(fp16_int32_t kp, fp16_int32_t ki, fp16_int32_t kd)
{
    HAL_StatusTypeDef status;
    Flash_Data_t data;
    uint32_t primask;
    uint32_t page_error = 0;
    
    // 准备数据
    data.magic = FLASH_MAGIC_NUMBER;
    data.kp = kp;
    data.ki = ki;
    data.kd = kd;
    
    // 关中断，防止Flash操作期间执行中断代码（如果中断向量在Flash中）
    primask = __get_PRIMASK();
    __disable_irq();
    
    // 解锁Flash
    HAL_FLASH_Unlock();
    
    // 擦除一页（F1系列每页1KB，必须整页擦除）
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = FLASH_START_ADDRESS,
        .NbPages = 1
    };
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        __set_PRIMASK(primask);
        return;  // 擦除失败，静默返回（实际项目建议加错误处理）
    }
    
    // 按半字（16bit）写入，F1系列要求
    uint16_t* src = (uint16_t*)&data;
    uint32_t addr = FLASH_START_ADDRESS;
    
    for (int i = 0; i < sizeof(Flash_Data_t) / 2; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, src[i]);
        if (status != HAL_OK) {
            break;  // 写入失败
        }
        addr += 2;
    }
    
    // 上锁并恢复中断
    HAL_FLASH_Lock();
    __set_PRIMASK(primask);
}
