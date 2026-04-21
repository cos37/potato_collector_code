#include "imu_uart.h"
#include "usart.h"
#include <stdint.h>
#include "fixpoint.h"
#include "sys.h"

// 全局变量
uint8_t  rx_buffer[20];       // 数据缓冲区
uint8_t  rx_temp[1];          // 单字节临时接收缓冲（HAL中断接收用）
fp16_int32_t current_yaw = 0; // 水平角度（定点数表示，单位：度）

// 状态机变量
static uint8_t RxState = 0;
static uint8_t pRxPacket = 0;

// 定点数常量：180.0f / 32768.0f = 0.0054931640625
// 预计算为定点数：0.0054931640625 * 65536 = 360
#define FP16_YAW_SCALE 360  // 180/32768 * 65536 = 360
/**
 * 串口接收完成回调函数
 */

void IMU_UART_CALLBACK(void)
{
        uint8_t Rxdata = rx_temp[0];
        
        switch(RxState)
        {
            case 0:
                if (Rxdata == 0x55)
                {
                    rx_buffer[pRxPacket] = Rxdata;
                    pRxPacket++;
                    RxState = 1;
                }
                break;
                
            case 1:
                if (Rxdata == 0x53)
                {
                    rx_buffer[pRxPacket] = Rxdata;
                    pRxPacket++;
                    RxState = 2;
                }
                else
                {
                    pRxPacket = 0;
                    RxState = 0;
                }
                break;
                
            case 2:
                if (pRxPacket <= 10)
                {
                    rx_buffer[pRxPacket] = Rxdata;
                    pRxPacket++;
                }
                else
                {
                    // 接收完成，解析数据（定点数运算，无浮点）
                    int16_t yaw_raw = (rx_buffer[7] << 8) | rx_buffer[6];
                    
                    // 定点数转换：current_yaw = yaw_raw * 180 / 32768
                    // 转换为：yaw_raw * 360 >> 16 (因为 360/65536 = 180/32768)
                    current_yaw = ((int32_t)yaw_raw * FP16_YAW_SCALE) << 0; 
                    // 实际上 yaw_raw * 360 就是定点数结果，因为 360 = 180/32768 * 65536
                    // 如果 yaw_raw 是 int16_t，需要扩展为32位
                    current_yaw = (int32_t)yaw_raw * 360;  // 这就是定点数表示的角度值
                    
                    // 复位状态机
                    pRxPacket = 0;
                    RxState = 0;
                }
                break;
                
            default:
                pRxPacket = 0;
                RxState = 0;
                break;
        }
        
        HAL_UART_Receive_IT(&huart2, rx_temp, 1);

}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART2)
//     {
//         uint8_t Rxdata = rx_temp[0];
        
//         switch(RxState)
//         {
//             case 0:
//                 if (Rxdata == 0x55)
//                 {
//                     rx_buffer[pRxPacket] = Rxdata;
//                     pRxPacket++;
//                     RxState = 1;
//                 }
//                 break;
                
//             case 1:
//                 if (Rxdata == 0x53)
//                 {
//                     rx_buffer[pRxPacket] = Rxdata;
//                     pRxPacket++;
//                     RxState = 2;
//                 }
//                 else
//                 {
//                     pRxPacket = 0;
//                     RxState = 0;
//                 }
//                 break;
                
//             case 2:
//                 if (pRxPacket <= 10)
//                 {
//                     rx_buffer[pRxPacket] = Rxdata;
//                     pRxPacket++;
//                 }
//                 else
//                 {
//                     // 接收完成，解析数据（定点数运算，无浮点）
//                     int16_t yaw_raw = (rx_buffer[7] << 8) | rx_buffer[6];
                    
//                     // 定点数转换：current_yaw = yaw_raw * 180 / 32768
//                     // 转换为：yaw_raw * 360 >> 16 (因为 360/65536 = 180/32768)
//                     current_yaw = ((int32_t)yaw_raw * FP16_YAW_SCALE) << 0; 
//                     // 实际上 yaw_raw * 360 就是定点数结果，因为 360 = 180/32768 * 65536
//                     // 如果 yaw_raw 是 int16_t，需要扩展为32位
//                     current_yaw = (int32_t)yaw_raw * 360;  // 这就是定点数表示的角度值
                    
//                     // 复位状态机
//                     pRxPacket = 0;
//                     RxState = 0;
//                 }
//                 break;
                
//             default:
//                 pRxPacket = 0;
//                 RxState = 0;
//                 break;
//         }
        
//         HAL_UART_Receive_IT(&huart2, rx_temp, 1);
//     }
// }

/**
 * 初始化USART2（在MX_USART2_UART_Init之后调用）
 * 波特率: 115200
 */
void UART2_Receive_IT_Start(void)
{
    void (*callback)(void) = IMU_UART_CALLBACK;
    SYS_EnableUARTx_RXIT_CB(&huart2, callback);
    HAL_UART_Receive_IT(&huart2, rx_temp, 1);
}



/**
 * 获取当前角度的浮点值（调试用或需要浮点时调用）
 */
float get_yaw_float(void)
{
    return fp16_to_float(current_yaw);
}

fp16_int32_t get_yaw_fp16(void)
{
    return current_yaw;
}

/**
 * 串口错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        
        pRxPacket = 0;
        RxState = 0;
        
        HAL_UART_Receive_IT(&huart2, rx_temp, 1);
    }
}
