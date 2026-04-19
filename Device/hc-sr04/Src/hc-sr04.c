#include "hc-sr04.h"
#include "tim.h"
#include "fixpoint.h"
#include "sys.h"

// 340m/s,340*dt/2
#define DISTANCE_SCLAR 1114  
#define GPIO_PIN_TRIG_LEFT GPIO_PIN_4
#define GPIO_PIN_TRIG_RIGHT GPIO_PIN_5
#define GPIO_TRIG_Port GPIOB

typedef enum{
    SR04_STATE_IDLE = 0,
    SR04_STATE_WAITING = 1,
    SR04_STATE_DONE = 2,
    SR04_STATE_LOST = 3
} SR04State_t;

typedef struct 
{
    SR04State_t state;
    fp16_int32_t distance;
    uint8_t lost_flag;
    void(*TRIG)(void);
    uint32_t start_time;
    uint32_t time;
    uint32_t end_time;
} SR04_t;

SR04_t SR04Left;
SR04_t SR04Right;

void SR04_IDLE_FUNC(SR04_t* s);
void SR04_WAITING_FUNC(SR04_t* s);
void SR04_DONE_FUNC(SR04_t* s);
void SR04_LOST_FUNC(SR04_t* s);

// 【关键修改1】增加微秒级死延时函数，专供 TRIG 触发使用
static void Delay_us(uint32_t us)
{
    uint32_t start = Get_Tick_us();
    while ((Get_Tick_us() - start) < us);
}

void SR04State_Machine(SR04_t* s)
{
    switch (s->state)
    {
    case SR04_STATE_IDLE:
        SR04_IDLE_FUNC(s);
        break;
    case SR04_STATE_WAITING:
        SR04_WAITING_FUNC(s);
        break;
    case SR04_STATE_DONE:
        SR04_DONE_FUNC(s);
        break;
    case SR04_STATE_LOST:
        SR04_LOST_FUNC(s);
        break;
    default:
        break;
    }
}

void SR04_IDLE_FUNC(SR04_t* s)
{
    // 防止两个超声波一进入 IDLE 就同时发波导致打架。
}

void SR04_WAITING_FUNC(SR04_t* s)
{
    s->time = Get_Tick_us();
    if(s->time - s->start_time >= 38000) // 38ms 超时 (约6.5米外或丢波了)
    {
        s->lost_flag = 1;         
        s->state = SR04_STATE_IDLE; // 复位，准备下一次测量
    }
}

void SR04_DONE_FUNC(SR04_t* s)
{
    // 距离 = 时间差 * 比例系数
    s->distance = fp16_mul((s->end_time - s->start_time) << 16, DISTANCE_SCLAR);
    s->lost_flag = 0; // 成功测到，清除丢波标志
    s->state = SR04_STATE_IDLE;
}

void SR04_LOST_FUNC(SR04_t* s)
{
    s->lost_flag = 1;
    s->state = SR04_STATE_IDLE;
}

void Handle_Channel3(void)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
        SR04Left.start_time = Get_Tick_us(); // 记录回波开始时间 (上升沿)
    } else {
        SR04Left.end_time = Get_Tick_us();   // 记录回波结束时间 (下降沿)
        if (SR04Left.state == SR04_STATE_WAITING) {
            SR04Left.state = SR04_STATE_DONE;
        }
    }
}

void Handle_Channel4(void)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
        SR04Right.start_time = Get_Tick_us(); 
    } else {
        SR04Right.end_time = Get_Tick_us();   
        if (SR04Right.state == SR04_STATE_WAITING) {
            SR04Right.state = SR04_STATE_DONE;
        }
    }
}

void SR04_TRIGleft(void)
{
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_LEFT, GPIO_PIN_SET);
    Delay_us(15); 
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_LEFT, GPIO_PIN_RESET);
}

void SR04_TRIGright(void)
{
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_RIGHT, GPIO_PIN_SET);
    Delay_us(15); 
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_RIGHT, GPIO_PIN_RESET);
}

void SR04_Init(void)
{
    SR04Left.distance = 0;
    SR04Left.end_time = 0;
    SR04Left.lost_flag = 0;
    SR04Left.start_time = 0;
    SR04Left.time = 0;
    SR04Left.TRIG = SR04_TRIGleft;

    SR04Right.distance = 0;
    SR04Right.end_time = 0;
    SR04Right.lost_flag = 0;
    SR04Right.start_time = 0;
    SR04Right.time = 0;
    SR04Right.TRIG = SR04_TRIGright;
    
    // 确保 TIM3 初始化为了输入捕获
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);  // PB0
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);  // PB1
}

void SR04_LOOP(void)
{
    static uint32_t last_poll_tick = 0;
    static uint8_t active_sensor = 0; // 0: 左边, 1: 右边

    // 每 50ms 轮换触发一个超声波，绝对防止声波打架！
    if (HAL_GetTick() - last_poll_tick >= 50) 
    {
        last_poll_tick = HAL_GetTick();

        if (active_sensor == 0) {
            if (SR04Left.state == SR04_STATE_IDLE) {
                SR04Left.TRIG();
                SR04Left.start_time = Get_Tick_us(); // 记录基准时间
                SR04Left.state = SR04_STATE_WAITING;
            }
            active_sensor = 1; // 下一次测右边
        } 
        else {
            if (SR04Right.state == SR04_STATE_IDLE) {
                SR04Right.TRIG();
                SR04Right.start_time = Get_Tick_us(); 
                SR04Right.state = SR04_STATE_WAITING;
            }
            active_sensor = 0; // 下一次测左边
        }
    }

    // 运行状态机，处理超时和结算
    SR04State_Machine(&SR04Left);
    SR04State_Machine(&SR04Right);
}

uint8_t SR04_GetFlag(uint8_t dir)
{
    if(dir == 0) return SR04Left.lost_flag;
    else if(dir == 1) return SR04Right.lost_flag;
    else return 0xff;
}

fp16_int32_t SR04_GetDistance(uint8_t dir)
{
    if(dir == 0) return SR04Left.distance;
    else if(dir == 1) return SR04Right.distance;
    else return 0xff;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;
    
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        Handle_Channel3();
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        Handle_Channel4();
    }
}
