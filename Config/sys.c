#include "sys.h"
#include "tim.h"
#include <stdint.h>
#include "stm32f1xx.h"

void (*Sys_Task_TIM2)(void) = 0; // 定义一个函数指针,指向要执行的任务函数

/**
 * @brief 开启定时器,一段时间后会触发中断,在中断回调函数中执行相应的任务
 * @param 时间间隔,单位为毫秒;函数指针,指向要执行的任务函数
 * @retval None
 */

void Sys_StartTimer(uint16_t interval_ms, void (*task)(void))
{
    //目前先只使用定时器2,后续可以扩展到其他定时器
    //配置定时器2的时间间隔
    uint32_t systick =HAL_RCC_GetSysClockFreq();
    uint8_t psc= (uint8_t)(systick / 1000) - 1; // 预分频器,将时钟频率降低到1kHz
    uint16_t arr = (uint16_t)(interval_ms); // 自动重装载值,定时器计数到这个值时触发中断

    TIM2->PSC = psc; // 设置预分频器
    TIM2->ARR = arr; // 设置自动重装载值

    //使能定时器2的更新中断
    TIM2->DIER |= TIM_DIER_UIE;
    //使能定时器2
    TIM2->CR1 |= TIM_CR1_CEN;
    //把函数指针指向全局变量,在中断回调函数中调用
    Sys_Task_TIM2 = task;
}

void CloseTimer2(void)
{
    //禁止定时器2的更新中断
    TIM2->DIER &= ~TIM_DIER_UIE;
    //禁止定时器2
    TIM2->CR1 &= ~TIM_CR1_CEN;
    //清空函数指针
    Sys_Task_TIM2 = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) {
        if(Sys_Task_TIM2 != 0) {
            Sys_Task_TIM2();
        }
    }
}
