/**
 **********************************************************
 * 文件：Bujin.c
 * 描述：Emm-V5 闭环步进电机 HAL 库版驱动
 * 依赖：Serial2_SendArray（用户封装 HAL_UART_Transmit）
 *       Delay_ms（HAL 版，基于 SysTick）
 * 用法：任意 .c 文件包含 "Bujin.h" 后调用
 **********************************************************
 */
#include "Bujin.h"
#include "usart.h" /* 包含串口发送接口 */
#include <string.h>
extern UART_HandleTypeDef huart1; /* 串口 1 句柄，定义在 usart.c 中 */

// ========== 配置 ==========
#define MOTOR_NUM       4
#define FRAME_LEN       8

// ========== 外部声明（用户需提供） ==========
static uint8_t motor_buf_write[MOTOR_NUM][FRAME_LEN];   // 主循环写入
static uint8_t motor_buf_read[MOTOR_NUM][FRAME_LEN];    // DMA读取
static volatile uint8_t buff_new;                       // 新数据标志



// ========== 状态机内部变量 ==========
typedef enum {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_PENDING
} DMA_State_t;

static volatile DMA_State_t dma_state = STATE_IDLE;
static volatile uint8_t motor_idx = 0;
static uint8_t pending_cnt = 0;

// ========== 工具函数 ==========

// 初始化缓冲区模板
void Motor_Buf_Init(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        motor_buf_write[i][0] = i + 1;      // 地址
        motor_buf_write[i][1] = 0xF6;       // 速度模式
        motor_buf_write[i][2] = 0;          // 方向
        motor_buf_write[i][3] = 0;          // 速度高
        motor_buf_write[i][4] = 0;          // 速度低
        motor_buf_write[i][5] = 10;         // 加速度
        motor_buf_write[i][6] = 0;          // 同步
        motor_buf_write[i][7] = 0x6B;       // 校验
    }
    dma_state = STATE_IDLE;
    motor_idx = 0;
    buff_new = 0;
    pending_cnt = 0;
}

// 主循环调用：写入速度（会设置buff_new）
void Motor_Set_Vel(uint8_t id, uint8_t dir, uint16_t vel)
{
    if (id < 1 || id > MOTOR_NUM) return;
    uint8_t idx = id - 1;
    motor_buf_write[idx][2] = dir;
    motor_buf_write[idx][3] = vel >> 8;
    motor_buf_write[idx][4] = vel & 0xFF;
    buff_new = 1;
}

// ========== 核心状态机（运行在定时器中断） ==========

void DMA_State_Machine(void)
{
    switch (dma_state) {
        
    case STATE_IDLE:
        if (buff_new) {
            buff_new = 0;
            motor_idx = 0;
            dma_state = STATE_RUNNING;
            pending_cnt = 0;
            
            // 复制到读缓冲区
            memcpy(motor_buf_read, motor_buf_write, MOTOR_NUM * FRAME_LEN);
            
            // 启动第一个
            HAL_UART_Transmit_DMA(&huart1, motor_buf_read[0], FRAME_LEN);
        }
        break;
        
    case STATE_RUNNING:
        // 等待DMA中断链式发送，什么都不做
        break;
        
    case STATE_PENDING:
        // 容错：连续2次PENDING则强制重置
        if (++pending_cnt > 2) {
            HAL_UART_DMAStop(&huart1);
            dma_state = STATE_IDLE;
            pending_cnt = 0;
        }
        break;
        
    default:
        dma_state = STATE_IDLE;
        break;
    }
}

// ========== DMA完成中断回调（链式发送） ==========

static void Motor_DMA_Callback(void)
{
    motor_idx++;
    
    if (motor_idx < MOTOR_NUM) {
        // 继续下一个
        HAL_UART_Transmit_DMA(&huart1, motor_buf_read[motor_idx], FRAME_LEN);
    } else {
        // 全部完成
        if (buff_new) {
            // 无缝衔接新一轮
            buff_new = 0;
            motor_idx = 0;
            memcpy(motor_buf_read, motor_buf_write, MOTOR_NUM * FRAME_LEN);
            HAL_UART_Transmit_DMA(&huart1, motor_buf_read[0], FRAME_LEN);
            // 保持RUNNING状态
        } else {
            dma_state = STATE_IDLE;
        }
    }
}

// ========== 紧急停止 ==========

static void Motor_DMA_Stop(void)
{
    HAL_UART_DMAStop(&huart1);
    dma_state = STATE_IDLE;
    motor_idx = 0;
    buff_new = 0;
    pending_cnt = 0;
}

/* ======================================================== */
/* 函数域：基础单轴控制                                       */
/* ======================================================== */

static void Serial_SendArray(uint8_t *pData, uint16_t len)
{
    HAL_UART_Transmit(&huart1, pData, len, 100);
}

/**
 * @brief  电机使能/脱机控制
 * @param  addr：  电机地址（1~255）
 * @param  state： ENABLE  使能电机
 *                DISABLE 脱机
 * @param  snF：  RESET 立即执行
 *                SET   等待同步触发
 * @retval 无
 */
void Emm_V5_En_Control(uint8_t addr, FlagStatus state, FlagStatus snF)
{
    /* 协议帧：地址|功能码|辅助码|使能状态|同步标志|校验 */
    uint8_t cmd[6] = {addr, 0xF3, 0xAB, (uint8_t)state, snF, 0x6B};
    Serial_SendArray(cmd, 6);
    HAL_Delay(10);

}

/**
 * @brief  紧急停止（所有模式通用）
 * @param  addr：电机地址
 * @param  snF： 同步标志
 * @retval 无
 */
void Emm_V5_Stop_Now(uint8_t addr, FlagStatus snF)
{
    uint8_t cmd[5] = {addr, 0xFE, 0x98, snF, 0x6B};
    Serial_SendArray(cmd, 5);
    HAL_Delay(10);
}

/**
 * @brief  速度模式运行
 * @param  addr：电机地址
 * @param  dir： 0=CW（正转） 非0=CCW（反转）
 * @param  vel： 目标转速 0~5000 RPM
 * @param  acc： 加速度 0~255（0=直接启动）
 * @param  snF： 同步标志
 * @retval 无
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, FlagStatus snF)
{
    uint8_t cmd[8] = {addr, 0xF6, dir, vel >> 8, vel, acc, snF, 0x6B};
    Serial_SendArray(cmd, 8);
    HAL_Delay(5);
}

/**
 * @brief  位置模式运行（相对/绝对）
 * @param  addr：  电机地址
 * @param  dir：   旋转方向 0=CW 非0=CCW
 * @param  vel：   运行速度 0~5000 RPM
 * @param  acc：   加速度 0~255
 * @param  clk：   目标脉冲数（32 bit）
 * @param  raF：   RESET=相对运动  SET=绝对位置
 * @param  snF：   同步标志
 * @retval 无
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                        uint32_t clk, FlagStatus raF, FlagStatus snF)
{
    uint8_t cmd[13] = {
        addr, 0xFD, dir,
        vel >> 8, vel, acc,               /* 速度+加速度 */
        clk >> 24, clk >> 16, clk >> 8, clk, /* 32 bit 脉冲数 */
        raF, snF, 0x6B                    /* 模式+同步+校验 */
    };
    Serial_SendArray(cmd, 13);
    HAL_Delay(10);
}

/* ======================================================== */
/* 函数域：参数/状态控制                                       */
/* ======================================================== */

/**
 * @brief  修改开环/闭环模式
 * @param  addr：     电机地址
 * @param  svF：      RESET=不保存  SET=掉电保存
 * @param  ctrl_mode：0=关闭脉冲引脚
 *                   1=开环模式
 *                   2=闭环模式
 *                   3=En 复用限位，Dir 复用到位输出
 * @retval 无
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, FlagStatus svF, uint8_t ctrl_mode)
{
    uint8_t cmd[6] = {addr, 0x46, 0x69, svF, ctrl_mode, 0x6B};
    Serial_SendArray(cmd, 6);
    HAL_Delay(10);
}

/**
 * @brief  解除堵转保护（清除错误）
 * @param  addr：电机地址
 * @retval 无
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0x0E, 0x52, 0x6B};
    Serial_SendArray(cmd, 4);

}

/**
 * @brief  多机同步触发（当前轴加入同步组）
 * @param  addr：电机地址
 * @retval 无
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[4] = {addr, 0xFF, 0x66, 0x6B};
    Serial_SendArray(cmd, 4);

}

/* ======================================================== */
/* 函数域：便捷封装                                           */
/* ======================================================== */

/**
 * @brief  在当前位置基础上旋转指定角度（闭环）
 * @param  addr： 电机地址
 * @param  angle：目标角度（°） 可正可负
 * @param  vel：  速度 0~5000 RPM
 * @param  acc：  加速度 0~255
 * @retval 无
 * @note   3200 脉冲/圈 固件细分
 */
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc)
{
    int dir = 0;
    if (angle < 0) { angle = -angle; dir = 1; }   /* 负角变正并标记方向 */
    uint32_t pulse = (uint32_t)(3200.0f * angle / 360.0f + 0.5f); /* 四舍五入 */
    Emm_V5_Pos_Control(addr, dir, vel, acc, pulse, RESET, RESET); /* 相对运动 */
}
