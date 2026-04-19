#include "hc-sr04.h"
#include "tim.h"
#include "fixpoint.h"
#include "sys.h"

#define DISTANCE_SCLAR 1114  
#define GPIO_PIN_TRIG_LEFT  GPIO_PIN_4
#define GPIO_PIN_TRIG_RIGHT GPIO_PIN_5
#define GPIO_TRIG_Port      GPIOB

#define FILTER_WINDOW_SIZE 5  

typedef enum {
    SR04_STATE_IDLE    = 0,
    SR04_STATE_WAITING = 1,
    SR04_STATE_DONE    = 2,
    SR04_STATE_LOST    = 3
} SR04State_t;

typedef struct {
    volatile SR04State_t state;        
    fp16_int32_t distance;
    uint8_t lost_flag;
    volatile uint8_t echo_started;     
    void (*TRIG)(void);
    volatile uint32_t start_time;      
    volatile uint32_t end_time;        
    
    fp16_int32_t history[FILTER_WINDOW_SIZE]; 
    uint8_t history_idx;                      
    uint8_t history_count;                    
} SR04_t;

SR04_t SR04Left;
SR04_t SR04Right;

static inline void Delay_us(uint32_t us) {
    uint32_t start = Get_Tick_us();
    while ((Get_Tick_us() - start) < us);
}

// ================= 滤波算法 =================
static fp16_int32_t SR04_MedianFilter(SR04_t* s, fp16_int32_t new_distance) {
    s->history[s->history_idx] = new_distance;
    s->history_idx++;
    if (s->history_idx >= FILTER_WINDOW_SIZE) {
        s->history_idx = 0; 
    }
    
    if (s->history_count < FILTER_WINDOW_SIZE) {
        s->history_count++;
    }

    fp16_int32_t temp_buf[FILTER_WINDOW_SIZE];
    for (uint8_t i = 0; i < s->history_count; i++) {
        temp_buf[i] = s->history[i];
    }

    if (s->history_count > 1) {
        for (uint8_t i = 0; i < s->history_count - 1; i++) {
            for (uint8_t j = i + 1; j < s->history_count; j++) {
                if (temp_buf[i] > temp_buf[j]) {
                    fp16_int32_t t = temp_buf[i];
                    temp_buf[i] = temp_buf[j];
                    temp_buf[j] = t;
                }
            }
        }
    }

    return temp_buf[s->history_count / 2];
}

// ================= 状态机内部处理 =================
void SR04_IDLE_FUNC(SR04_t* s) {
}

void SR04_WAITING_FUNC(SR04_t* s) {
    if ((Get_Tick_us() - s->start_time) >= 38000) { 
        s->state = SR04_STATE_LOST; 
    }
}

void SR04_DONE_FUNC(SR04_t* s) {
    uint32_t time_diff = s->end_time - s->start_time;
    
    // 物理常识过滤：如果时间差大于 40ms 或为 0，视为异常数据丢弃
    if (time_diff > 40000 || time_diff == 0) {
        s->state = SR04_STATE_LOST;
        return;
    }

    fp16_int32_t raw_distance = fp16_mul(time_diff << 16, DISTANCE_SCLAR);
    fp16_int32_t median_distance = SR04_MedianFilter(s, raw_distance);

    if (s->distance == 0 || s->lost_flag == 1) {
        s->distance = median_distance;
    } else {
        s->distance = s->distance - (s->distance >> 2) + (median_distance >> 2);
    }

    s->lost_flag = 0; 
    s->echo_started = 0;
    s->state = SR04_STATE_IDLE;
}

void SR04_LOST_FUNC(SR04_t* s) {
    s->lost_flag = 1;
    s->echo_started = 0;
    s->history_count = 0;
    s->history_idx = 0;
    s->state = SR04_STATE_IDLE;
}

void SR04State_Machine(SR04_t* s) {
    switch (s->state) {
        case SR04_STATE_IDLE:    SR04_IDLE_FUNC(s);    break;
        case SR04_STATE_WAITING: SR04_WAITING_FUNC(s); break;
        case SR04_STATE_DONE:    SR04_DONE_FUNC(s);    break;
        case SR04_STATE_LOST:    SR04_LOST_FUNC(s);    break;
        default: break;
    }
}

// ================= 中断处理 (核心修改区) =================
// 【修改】去掉了 GPIO 读取，改为通过状态判断并动态切换极性
static inline void Handle_EchoEdge(SR04_t* s, TIM_HandleTypeDef *htim, uint32_t TIM_Channel) {
    if (s->state != SR04_STATE_WAITING) return; 

    if (s->echo_started == 0) {
        // 1. 第一次进中断：必然是上升沿
        s->start_time = Get_Tick_us(); 
        s->echo_started = 1; 
        
        // 【关键】将定时器通道配置为下降沿捕获
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        // 2. 第二次进中断：必然是下降沿
        s->end_time = Get_Tick_us();   
        s->state = SR04_STATE_DONE;
        
        // 【关键】测试完成，立刻将定时器通道恢复为上升沿捕获，准备下一次测量
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM3) return;
    
    // 注意这里传入的是 TIM_CHANNEL_X 宏，专门用于改变寄存器极性
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        Handle_EchoEdge(&SR04Left, htim, TIM_CHANNEL_3);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        Handle_EchoEdge(&SR04Right, htim, TIM_CHANNEL_4);
    }
}

// ================= 初始化与外层控制 =================
void SR04_TRIGleft(void) {
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_LEFT, GPIO_PIN_SET);
    Delay_us(15); 
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_LEFT, GPIO_PIN_RESET);
}

void SR04_TRIGright(void) {
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_RIGHT, GPIO_PIN_SET);
    Delay_us(15); 
    HAL_GPIO_WritePin(GPIO_TRIG_Port, GPIO_PIN_TRIG_RIGHT, GPIO_PIN_RESET);
}

void SR04_Init(void) {
    SR04Left.distance = 0;
    SR04Left.lost_flag = 0;
    SR04Left.echo_started = 0;
    SR04Left.history_idx = 0;    
    SR04Left.history_count = 0;  
    SR04Left.TRIG = SR04_TRIGleft;
    SR04Left.state = SR04_STATE_IDLE; 

    SR04Right.distance = 0;
    SR04Right.lost_flag = 0;
    SR04Right.echo_started = 0;
    SR04Right.history_idx = 0;   
    SR04Right.history_count = 0; 
    SR04Right.TRIG = SR04_TRIGright;
    SR04Right.state = SR04_STATE_IDLE;
    
    uint8_t i; 
    for(i=0; i<FILTER_WINDOW_SIZE; i++) {
        SR04Left.history[i] = 0;
        SR04Right.history[i] = 0;
    }
    
    // 确保初始化时是上升沿捕获
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
    
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);  
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);  
}

void SR04_LOOP(void) {
    static uint32_t last_poll_tick = 0;
    static uint8_t active_sensor = 0; 

    if (HAL_GetTick() - last_poll_tick >= 50) {
        last_poll_tick = HAL_GetTick();

        SR04_t *current_sensor = (active_sensor == 0) ? &SR04Left : &SR04Right;
        uint32_t current_channel = (active_sensor == 0) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
        
        if (current_sensor->state == SR04_STATE_IDLE) {
            
            // 【终极防御】每次发射超声波前，强行重置捕获极性为上升沿！
            // 防止发生“丢波”后，定时器一直卡在“等下降沿”的死锁状态
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, current_channel, TIM_INPUTCHANNELPOLARITY_RISING);

            current_sensor->TRIG();
            current_sensor->start_time = Get_Tick_us(); 
            current_sensor->echo_started = 0;
            current_sensor->state = SR04_STATE_WAITING;
        }
        active_sensor ^= 1; 
    }

    SR04State_Machine(&SR04Left);
    SR04State_Machine(&SR04Right);
}

uint8_t SR04_GetFlag(uint8_t dir) {
    return (dir == 0) ? SR04Left.lost_flag : ((dir == 1) ? SR04Right.lost_flag : 0xff);
}

fp16_int32_t SR04_GetDistance(uint8_t dir) {
    return (dir == 0) ? SR04Left.distance : ((dir == 1) ? SR04Right.distance : 0xff);
}