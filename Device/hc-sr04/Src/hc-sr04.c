#include "hc-sr04.h"
#include "tim.h"
#include "fixpoint.h"
#include "sys.h"

//340m/s,340*dt/2
#define DISTANCE_SCLAR 1114  
#define GPIO_PIN_TRIG_LEFT GPIO_PIN_4
#define GPIO_PIN_TRIG_RIGHT GPIO_PIN_5
#define GPIO_TRIG_Port GPIOB


typedef enum{
    SR04_STATE_IDLE = 0,
    SR04_STATE_WAITING = 1,
    SR04_STATE_DONE = 2,
    SR04_STATE_LOST = 3
}SR04State_t;


typedef struct 
{
    SR04State_t state;
    fp16_int32_t distance;
    uint8_t lost_flag;
    void(*TRIG)(void);
    uint32_t start_time;
    uint32_t time;
    uint32_t end_time;
}SR04_t;

SR04_t SR04Left;
SR04_t SR04Right;

void SR04_IDLE_FUNC(SR04_t* s);
void SR04_WAITING_FUNC(SR04_t* s);
void SR04_DONE_FUNC(SR04_t* s);
void SR04_LOST_FUNC(SR04_t* s);

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
    s->TRIG();//触发一下电平
    s->state = SR04_STATE_WAITING;
    s->start_time = Get_Tick_us();
}

void SR04_WAITING_FUNC(SR04_t* s)
{
    s->time = Get_Tick_us();
    if(s->time-s->start_time>=50)//草死了
    {
        s->state = SR04_STATE_IDLE;
    }
}

void SR04_DONE_FUNC(SR04_t* s)
{
    s->distance = (s->end_time-s->start_time)*DISTANCE_SCLAR;
    s->end_time = 0;
    s->start_time = 0;
    s->state = SR04_STATE_IDLE;
}

void SR04_LOST_FUNC(SR04_t* s)
{
    s->lost_flag = 1;
}

void Handle_Channel3(void)
{
    SR04Left.end_time = Get_Tick_us();
    SR04Left.state = SR04_STATE_DONE;
}

void Handle_Channel4(void)
{
    SR04Right.end_time = Get_Tick_us();
    SR04Right.state = SR04_STATE_DONE;

}

void SR04_TRIGleft(void)
{
    HAL_GPIO_WritePin(GPIO_TRIG_Port,GPIO_PIN_TRIG_LEFT,GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIO_TRIG_Port,GPIO_PIN_TRIG_LEFT,GPIO_PIN_RESET);
}

void SR04_TRIGright(void)
{
    HAL_GPIO_WritePin(GPIO_TRIG_Port,GPIO_PIN_TRIG_RIGHT,GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIO_TRIG_Port,GPIO_PIN_TRIG_RIGHT,GPIO_PIN_RESET);
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
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);  // PB0
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);  // PB1
}

void SR04_LOOP(void)
{
    SR04State_Machine(&SR04Left);
    SR04State_Machine(&SR04Right);
}

//dir == 0 ,return left flag
//dir == 1 ,return right flag
uint8_t SR04_GetFlag(uint8_t dir)
{
    if(dir == 0)
    {
        return SR04Left.lost_flag;
    }else if(dir == 1)
    {
        return SR04Right.lost_flag;
    }else{
		return 0xff;
	}
}

//dir == 0 ,return left distance
//dir == 1 ,return right distance

//输出单位是cm
fp16_int32_t SR04_GetDistance(uint8_t dir)
{
    if(dir == 0)
    {
        return SR04Left.distance;
    }else if(dir == 1)
    {
        return SR04Right.distance;
    }else{
		return 0xff;
	}
	
	
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
