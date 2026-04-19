#define USE_CAN1

#include "Bujin.h"
#ifdef USE_UART1
#include "usart.h" /* 包含串口发送接口 */
#elif defined(USE_CAN1)
#include "can.h"   /* 包含CAN发送接口 */
#endif
#include <string.h>
// extern UART_HandleTypeDef huart1; /* 串口 1 句柄，定义在 usart.c 中 */
// extern CAN_HandleTypeDef hcan1;   /* CAN 1 句柄，定义在 can.c 中 */


#if defined(USE_UART1)

/**
 **********************************************************
 * 文件：Bujin.c
 * 描述：Emm-V5 闭环步进电机 HAL 库版驱动
 * 依赖：Serial2_SendArray（用户封装 HAL_UART_Transmit）
 *       Delay_ms（HAL 版，基于 SysTick）
 * 用法：任意 .c 文件包含 "Bujin.h" 后调用
 **********************************************************
 */

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
    HAL_Delay(10);
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
#endif

#if defined(USE_CAN1)

/**
 * @brief  CAN 版本的电机控制函数实现
 * 
 */



 void CAN1_Init(void)
 {
    CAN_FilterTypeDef sFilterConfig;
    
    // 计算掩码：只匹配地址1-5，包序号任意
    // 地址在 bit[15:8]，所以掩码需要覆盖这8位中的低3位（因为5=101，需要3位区分）
    // 但实际上 1-5 的范围是 0b00000001 到 0b00000101
    // 用掩码 0x07FF = 0b0000011111111111 可以匹配 0x0000-0x0007（地址0-7）
    
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    
    // ID：基地址 0x00000100（地址1，第0包）
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = (0x0100 << 3) | 0x04;  // 0x0804
    
    // 掩码：地址0-7都接收（0x0000-0x0007），包序号任意（0x00-0xFF）
    // 掩码值：0x07FF << 3 = 0x3FF8，再加上 IDE=1
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = (0x07FF << 3) | 0x04;  // 0x3FFC
    
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    
    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

    HAL_CAN_Start(&hcan);

 }



/* 细分对应脉冲数（默认16细分=3200脉冲/圈，1.8°电机）*/
#define EM_V5_PULSE_PER_CIRCLE    3200U

/**
 * @brief  CAN底层发送函数（自动处理单包/多包分包逻辑）
 * @param  addr: 电机地址 1-255（0为广播）
 * @param  cmd:  命令数据缓冲区（不含地址字节，包含功能码+数据+0x6B校验）
 * @param  len:  命令数据长度
 * @retval 0:成功, 1:失败
 */
static uint8_t CAN_EmmV5_Send(uint8_t addr, uint8_t *cmd, uint8_t len)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;
    uint8_t pkg_num = 0;
    uint8_t sent = 0;

    TxHeader.IDE = CAN_ID_EXT;       // 扩展帧
    TxHeader.RTR = CAN_RTR_DATA;     // 数据帧

    while (sent < len)
    {
        /* 帧ID = 地址<<8 | 包序号 */
        TxHeader.ExtId = ((uint32_t)addr << 8) | pkg_num;

        if (pkg_num == 0)
        {
            /* 第一包：直接取原始数据前8字节 */
            uint8_t n = (len > 8) ? 8 : len;
            memcpy(TxData, cmd, n);
            TxHeader.DLC = n;
            sent += n;
        }
        else
        {
            /* 后续包：第1字节必须是功能码，后面接剩余数据 */
            TxData[0] = cmd[0];  // 功能码
            uint8_t remain = len - sent;
            if (remain > 7) remain = 7;
            memcpy(&TxData[1], &cmd[sent], remain);
            TxHeader.DLC = 1 + remain;
            sent += remain;
        }

        /* 等待发送邮箱空闲（超时10ms）*/
        uint32_t tickstart = HAL_GetTick();
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        {
            // if ((HAL_GetTick() - tickstart) > 10U)
            //     return 1;
        }

        if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
            return 1;

        pkg_num++;
    }
    return 0;
}

/* ============================================================
 * 控制动作命令
 * ============================================================ */

/**
 * @brief  电机使能控制
 * @param  addr:  电机地址 1-5
 * @param  state: SET=使能, RESET=不使能
 * @param  snF:   SET=启用多机同步标志(等同步指令), RESET=立即执行
 */
void Emm_V5_En_Control(uint8_t addr, FlagStatus state, FlagStatus snF)
{
    uint8_t cmd[6];
    cmd[0] = 0xF3;
    cmd[1] = 0xAB;
    cmd[2] = (state == SET) ? 0x01 : 0x00;
    cmd[3] = (snF == SET) ? 0x01 : 0x00;
    cmd[4] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 5);
}

/**
 * @brief  立即停止（紧急刹车）
 * @param  addr: 电机地址
 * @param  snF:  多机同步标志
 */
void Emm_V5_Stop_Now(uint8_t addr, FlagStatus snF)
{
    uint8_t cmd[4];
    cmd[0] = 0xFE;
    cmd[1] = 0x98;
    cmd[2] = (snF == SET) ? 0x01 : 0x00;
    cmd[3] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 4);
}

/**
 * @brief  速度模式控制
 * @param  addr: 电机地址
 * @param  dir:  0x00=CW, 0x01=CCW
 * @param  vel:  目标转速，单位 RPM（如 1500 = 0x05DC）
 * @param  acc:  加速度档位 0-255（0=直接启动，无曲线加减速）
 * @param  snF:  多机同步标志
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, FlagStatus snF)
{
    uint8_t cmd[7];
    cmd[0] = 0xF6;
    cmd[1] = dir;
    cmd[2] = (vel >> 8) & 0xFF;  // 速度高字节
    cmd[3] = vel & 0xFF;         // 速度低字节
    cmd[4] = acc;
    cmd[5] = (snF == SET) ? 0x01 : 0x00;
    cmd[6] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 7);
}

/**
 * @brief  位置模式控制
 * @param  addr: 电机地址
 * @param  dir:  0x00=CW, 0x01=CCW
 * @param  vel:  目标转速，单位 RPM
 * @param  acc:  加速度档位 0-255
 * @param  clk:  脉冲数（0x00000000 - 0xFFFFFFFF）
 * @param  raF:  SET=绝对位置模式, RESET=相对位置模式
 * @param  snF:  多机同步标志
 * @note   此命令大于8字节，会自动拆分为2包发送
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,
                        uint32_t clk, FlagStatus raF, FlagStatus snF)
{
    uint8_t cmd[12];
    cmd[0] = 0xFD;
    cmd[1] = dir;
    cmd[2] = (vel >> 8) & 0xFF;
    cmd[3] = vel & 0xFF;
    cmd[4] = acc;
    cmd[5] = (clk >> 24) & 0xFF;  // 脉冲数 Byte3 (MSB)
    cmd[6] = (clk >> 16) & 0xFF;  // 脉冲数 Byte2
    cmd[7] = (clk >> 8) & 0xFF;   // 脉冲数 Byte1
    cmd[8] = clk & 0xFF;          // 脉冲数 Byte0 (LSB)
    cmd[9] = (raF == SET) ? 0x01 : 0x00;   // 相对(00)/绝对(01)
    cmd[10] = (snF == SET) ? 0x01 : 0x00;  // 同步标志
    cmd[11] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 12);
}

/* ============================================================
 * 参数修改命令
 * ============================================================ */

/**
 * @brief  修改开环/闭环控制模式（对应屏幕 P_Pul 菜单）
 * @param  addr:      电机地址
 * @param  svF:       SET=保存到芯片(断电保持), RESET=不保存
 * @param  ctrl_mode: 0x01=开环(PUL_OPEN), 0x02=FOC闭环(PUL_FOC)
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, FlagStatus svF, uint8_t ctrl_mode)
{
    uint8_t cmd[5];
    cmd[0] = 0x46;
    cmd[1] = 0x69;
    cmd[2] = (svF == SET) ? 0x01 : 0x00;
    cmd[3] = ctrl_mode;
    cmd[4] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 5);
}

/**
 * @brief  解除堵转保护
 * @param  addr: 电机地址
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[3];
    cmd[0] = 0x0E;
    cmd[1] = 0x52;
    cmd[2] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 3);
}

/* ============================================================
 * 多机同步控制
 * ============================================================ */

/**
 * @brief  触发多机同步运动
 * @param  addr: 通常填 0（广播地址），或指定某电机地址
 * @note   先对各电机发送带 snF=SET 的速度/位置指令，再发此指令，所有电机会同时开始运动
 */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[3];
    cmd[0] = 0xFF;
    cmd[1] = 0x66;
    cmd[2] = 0x6B;
    CAN_EmmV5_Send(addr, cmd, 3);
}

/* ============================================================
 * 应用层封装
 * ============================================================ */

/**
 * @brief  角度控制（相对位置模式封装）
 * @param  addr:  电机地址
 * @param  angle: 目标角度（正数CW，负数CCW；单位：度）
 * @param  vel:   目标转速，单位 RPM
 * @param  acc:   加速度档位 0-255
 * @note   基于16细分（3200脉冲/圈）计算。若使用其他细分，请修改 EM_V5_PULSE_PER_CIRCLE
 */
void motor_to_angle_control(uint8_t addr, float angle, uint16_t vel, uint8_t acc)
{
    uint8_t dir;
    float abs_angle;
    uint32_t pulse;

    if (angle >= 0.0f)
    {
        dir = 0x00;       // CW
        abs_angle = angle;
    }
    else
    {
        dir = 0x01;       // CCW
        abs_angle = -angle;
    }

    /* 16细分：3200脉冲 = 360° */
    pulse = (uint32_t)(abs_angle / 360.0f * (float)EM_V5_PULSE_PER_CIRCLE + 0.5f);

    Emm_V5_Pos_Control(addr, dir, vel, acc, pulse, RESET, RESET);
}



#endif

