#include "tof.h"
#include "usart.h"
#include "sys.h"

#define TOF_USART huart3

/* ==================== TOF400 Registers ==================== */
#define SPECIAL_REG         0x0001
#define DEV_ADDR_REG        0x0002
#define BAUD_RATE_REG       0x0003
#define RANGE_REG           0x0004
#define AUTO_OUTPUT_REG     0x0005
#define DISTANCE_REG        0x0010    // 修正：距离寄存器是0x0010

/* ==================== TOF400 Commands ==================== */
#define FUNC_READ           0x03
#define FUNC_WRITE          0x06

#define REBOOT_CMD          0x1000
#define RESTORE_CMD         0xAA55

// /* ==================== CRC16 Lookup Table (Modbus) ==================== */
// static const uint16_t crc16_table[256] = {
//     0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
//     0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
//     0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
//     0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
//     0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
//     0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
//     0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
//     0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
//     0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
//     0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
//     0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
//     0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
//     0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
//     0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
//     0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
//     0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
//     0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
//     0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
//     0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
//     0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
//     0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
//     0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
//     0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
//     0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
//     0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
//     0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
//     0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
//     0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
//     0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
//     0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
//     0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
//     0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
// };

// /* ==================== CRC16 Calculation (查表法) ==================== */
// uint16_t CRC16_Calculate(uint8_t *data, uint16_t length)
// {
//     uint16_t crc = 0xFFFF;
    
//     while (length--) {
//         crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF];
//     }
    
//     return crc;
// }

// /* ==================== Send Command ==================== */
// void TOF400_Send(uint8_t adr, uint8_t func, uint16_t reg, uint16_t data)
// {
//     uint8_t cmd[8];
    
//     cmd[0] = adr;                       // 从机地址
//     cmd[1] = func;                      // 功能码
//     cmd[2] = (reg >> 8) & 0xFF;         // 寄存器地址高字节
//     cmd[3] = reg & 0xFF;                // 寄存器地址低字节
//     cmd[4] = (data >> 8) & 0xFF;        // 数据高字节
//     cmd[5] = data & 0xFF;               // 数据低字节
    
//     uint16_t crc = CRC16_Calculate(cmd, 6);
    
//     // Modbus CRC 字节序：低字节在前，高字节在后！
//     cmd[6] = crc & 0xFF;                // CRC 低字节 (CL)
//     cmd[7] = (crc >> 8) & 0xFF;         // CRC 高字节 (CH)
    
//     HAL_UART_Transmit(&TOF_USART, cmd, 8, 100);
// }

// /* ==================== Convenience Functions ==================== */

// /**
//  * @brief 读取距离值 (功能码 0x03)
//  * @param adr 从机地址 (1-247, 0为广播)
//  */
// void TOF400_ReadDistance(uint8_t adr)
// {
//     // 读取1个寄存器 (距离值)
//     TOF400_Send(adr, FUNC_READ, DISTANCE_REG, 0x0001);
// }

// /**
//  * @brief 设置测距模式 (功能码 0x06)
//  * @param adr 从机地址
//  * @param mode 0=高精度(1.3m), 1=长距离(4m)
//  */
// void TOF400_SetRangeMode(uint8_t adr, uint8_t mode)
// {
//     TOF400_Send(adr, FUNC_WRITE, RANGE_REG, mode);
// }

// /**
//  * @brief 设置自动输出周期 (功能码 0x06)
//  * @param adr 从机地址
//  * @param period_ms 周期(ms), 0=关闭自动输出
//  */
// void TOF400_SetAutoOutput(uint8_t adr, uint16_t period_ms)
// {
//     TOF400_Send(adr, FUNC_WRITE, AUTO_OUTPUT_REG, period_ms);
// }

// /**
//  * @brief 修改设备地址 (功能码 0x06)
//  * @param adr 当前地址
//  * @param new_adr 新地址 (1-247)
//  */
// void TOF400_SetAddress(uint8_t adr, uint8_t new_adr)
// {
//     TOF400_Send(adr, FUNC_WRITE, DEV_ADDR_REG, new_adr);
// }

// /**
//  * @brief 设置波特率 (功能码 0x06)
//  * @param adr 从机地址
//  * @param baud 0=115200, 1=38400, 2=9600, 其他=115200
//  * @note 需要重启后生效
//  */
// void TOF400_SetBaudRate(uint8_t adr, uint8_t baud)
// {
//     TOF400_Send(adr, FUNC_WRITE, BAUD_RATE_REG, baud);
// }

// /**
//  * @brief 重启模块 (功能码 0x06)
//  * @param adr 从机地址
//  */
// void TOF400_Reboot(uint8_t adr)
// {
//     TOF400_Send(adr, FUNC_WRITE, SPECIAL_REG, REBOOT_CMD);
// }

// /**
//  * @brief 恢复出厂设置 (功能码 0x06)
//  * @param adr 从机地址
//  */
// void TOF400_RestoreDefault(uint8_t adr)
// {
//     TOF400_Send(adr, FUNC_WRITE, SPECIAL_REG, RESTORE_CMD);
// }

// /* ==================== Data Parsing ==================== */

// /**
//  * @brief 解析接收到的距离数据
//  * @param rx_buf 接收缓冲区 (从机返回的帧)
//  * @param len 接收长度
//  * @return 距离值(mm), -1表示错误
//  */
// int16_t TOF400_ParseDistance(uint8_t *rx_buf, uint8_t len)
// {
//     if (len < 7) return -1;  // 最小帧长度检查
    
//     // 验证功能码
//     if (rx_buf[1] != FUNC_READ) return -1;
    
//     // 验证数据字节数 (读取1个寄存器应为2字节)
//     if (rx_buf[2] != 0x02) return -1;
    
//     // 提取距离值 (高字节在前)
//     uint16_t distance = (rx_buf[3] << 8) | rx_buf[4];
    
//     // 可选：验证CRC
//     uint16_t rx_crc = (rx_buf[6] << 8) | rx_buf[5];  // 注意字节序
//     uint16_t calc_crc = CRC16_Calculate(rx_buf, 5);
    
//     if (rx_crc != calc_crc) return -1;
    
//     return (int16_t)distance;
// }

// /* ==================== Receive Handling ==================== */

// #define TOF_RX_BUF_SIZE 16

// typedef struct {
//     uint8_t buf[TOF_RX_BUF_SIZE];
//     uint8_t len;
//     uint8_t frame_ready;
//     uint16_t timeout_cnt;
// } TOF_RxTypeDef;

// TOF_RxTypeDef tof_rx = {0};

// /**
//  * @brief 串口接收中断回调 (放入 USART_IRQHandler)
//  * @param byte 接收到的字节
//  * @note 需要配合1ms定时器判断帧结束 (3.5字符时间)
//  */
// void TOF400_RxIRQ(uint8_t byte)
// {
//     uint8_t byte;
//     HAL_UART_Receive(&TOF_USART, &byte, 1, 0); 
//     if (tof_rx.len < TOF_RX_BUF_SIZE) {
//         tof_rx.buf[tof_rx.len++] = byte;
//     }
//     tof_rx.timeout_cnt = 0;  // 重置超时计数
// }

// /**
//  * @brief 定时器中断回调 (1ms调用)
//  * @note 检测Modbus帧结束 (115200波特率约3.5字符=0.3ms, 建议1ms更安全)
//  */
// void TOF400_TimIRQ(void)
// {
//     if (tof_rx.len > 0) {
//         if (++tof_rx.timeout_cnt >= 2) {  // 2ms超时
//             tof_rx.frame_ready = 1;
//         }
//     }
// }

// /**
//  * @brief 主循环处理 (非中断中调用)
//  * @return 距离值(mm), -1表示无新数据或错误
//  */
// int16_t TOF400_Process(void)
// {
//     if (!tof_rx.frame_ready) return -1;
    
//     int16_t distance = TOF400_ParseDistance(tof_rx.buf, tof_rx.len);
    
//     // 重置接收状态
//     tof_rx.len = 0;
//     tof_rx.frame_ready = 0;
//     tof_rx.timeout_cnt = 0;
    
//     return distance;
// }

// void TOF400_Init(void)
// {
//     Sys_startTimer3(1, TOF400_TimIRQ);  // 启动1ms定时器，回调函数为TOF400_TimIRQ
//     SYS_EnableUARTx_RXIT_CB(&TOF_USART, TOF400_RxIRQ);  // 启用UART接收中断回调
//     HAL_UART_Receive_IT(&TOF_USART, (uint8_t *)&tof_rx.buf[tof_rx.len], 1);  // 启动UART接收中断
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == TOF_USART.Instance) {
//         uint8_t byte;
//         HAL_UART_Receive(&TOF_USART, &byte, 1, 0); // 立即读取接收数据
//         TOF400_RxIRQ(byte);  // 调用接收处理函数
//     }
// }


/**
 * CRC16校验有点难崩,我们在开始前就知道我们的帧结构是固定的,
 * 地址已知,命令我们也提前知道,所以我们可以在发送命令时就计算好CRC,
 * 接收时我也懒得验证了,直接把数据拿出来用就行了,如果有问题再说吧.
 * 反正这个模块的通信环境还算干净,出问题了也好排查,
 * 不至于莫名其妙的通信错误.我们先把基本的发送和接收框架搭起来,后续再根据实际情况调整细节.
 */

/**
 * 发送格式大概是 [地址][功能码][寄存器地址高][寄存器地址低][数据高][数据低][CRC低][CRC高]
 * 接收格式大概是 [地址][功能码][寄存器地址高][寄存器地址低][数据高][数据低][CRC低][CRC高]
 */


/**
 * 我大概只会用读距离这个指令,就写这一个,AI给的太多了,我也懒得写了,需要其他指令再说吧.反正这个模块的寄存器也不多,功能也简单,后续如果需要再补充就好了.
 */

 /**
  * 我就先根据地址来返回CRC16的值
  */
uint16_t RetCRC16(uint8_t adr)
{
    //我们用了地址,提前写CRC16
    if(adr == 0x01) {
        return 0x85CF; 
    } else if(adr == 0x02) {
        return 0xFC85; 
    } else {
        return 0xFFFF; 
    }

}

void TOF_ReadDistanceReq(uint8_t adr)
{
    uint8_t cmd[8];
    cmd[0] = adr;                       // 从机地址
    cmd[1] = FUNC_READ;                 // 功能码
    cmd[2] = (DISTANCE_REG >> 8) & 0xFF; // 寄存器地址高字节
    cmd[3] = DISTANCE_REG & 0xFF;        // 寄存器地址低字节
    cmd[4] = 0x00;                      // 数据高字节 (读取命令数据部分通常为0)
    cmd[5] = 0x01;                      // 数据低字节 (读取1个寄存器)
    
    uint16_t crc = RetCRC16(cmd[0]);
    cmd[6] = crc & 0xFF;                // CRC 低字节 (CL)
    cmd[7] = (crc >> 8) & 0xFF;         // CRC 高字节 (CH)
    

    
    HAL_UART_Transmit(&TOF_USART, cmd, 8, 100);
}

/**
 * 写个结构体来管理数据
 */

typedef struct {
    uint8_t address;
    uint16_t distance;
} TOF400_DataTypeDef;

TOF400_DataTypeDef tof[1] = {0};
uint8_t size = 1;
uint8_t buff[7] = {0};
uint8_t tcomp_flag = 0;

void TOF_CB(void)
{
    //获取距离值
    tof[buff[0]].distance = ((uint16_t)buff[3])<<8|buff[4];
    HAL_UART_Receive_IT(&huart2, buff, 7);
    tcomp_flag = 0;
}


void TOF_Init(void)
{
    tof[0].address = 0x01 ;
    tof[0].distance = 0 ;

    HAL_UART_Receive_IT(&huart2, buff, 7);
    SYS_EnableUARTx_RXIT_CB(&TOF_USART,TOF_CB);

}

void TOF_LOOP(void)
{
    static uint8_t state = 0;
    static uint8_t read_id = 0 ;
    switch (state)
    {
    case 0:
        if(read_id >= size)
        {
            read_id = 0 ;   
        }
        TOF_ReadDistanceReq(tof[read_id].address);
        tcomp_flag = 1;
        state = 1 ;
        break;
    case 1:
        if(tcomp_flag == 1)
        {
            read_id++;
            state = 0 ;
        }
        break;
    
    default:
        break;
    }
}

uint16_t TOF_GetDistance(uint8_t adr)
{
    return tof[adr].distance;
}

