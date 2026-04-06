#include "sys.h"
#include "tim.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

void (*Sys_Task_TIM2)(void) = 0;
void (*Sys_Task_TIM3)(void) = 0;
void Sys_Init(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_DeInit(&htim2);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
}

void Sys_StartTimer(uint16_t interval_ms, void (*task)(void))
{
    // 1. 停止并反初始化，确保干净状态
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_DeInit(&htim2);
    
    // 2. 重新初始化时钟（DeInit会关时钟）
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    // 3. 计算并更新定时器参数
    // TIM2时钟 = 72MHz (APB1=36MHz, 但TIM2有倍频器)
    uint32_t tim_clk = 72000000;
    uint32_t psc = (tim_clk / 1000) - 1;  // 71999, 得到1kHz
    uint16_t arr = interval_ms - 1;        // ARR是0-based，5000ms=4999
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = psc;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    // 4. HAL初始化（配置PSC, ARR等，但不启动）
    HAL_TIM_Base_Init(&htim2);
    
    // 5. 【关键】清NVIC pending，防止旧中断请求
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    
    // 6. 启动中断模式
    HAL_TIM_Base_Start_IT(&htim2);
    
    Sys_Task_TIM2 = task;
}

void Sys_SoftTime_Start(void (*task)(void))
{
    // 1. 停止并反初始化，确保干净状态
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_Base_DeInit(&htim2);

    // 2. 重新初始化时钟（DeInit会关时钟）
    __HAL_RCC_TIM2_CLK_ENABLE();

    //设置1ms时基
    uint32_t tim_clk = 72000000;
    uint32_t psc = (tim_clk / 1000) - 1; 
    uint16_t arr = 1;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = psc;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = arr;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    // 4. HAL初始化（配置PSC, ARR等，但不启动）
    HAL_TIM_Base_Init(&htim2);
    
    // 5. 【关键】清NVIC pending，防止旧中断请求
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    
    // 6. 启动中断模式
    HAL_TIM_Base_Start_IT(&htim2);

    Sys_Task_TIM2 = task;
}

void CloseTimer2(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    NVIC_ClearPendingIRQ(TIM2_IRQn);  // 清pending，防止残留
    Sys_Task_TIM2 = 0;
}

void Sys_startTimer3(uint32_t interval_ms,void (*task)(void))
{
    // 1. 停止并反初始化，确保干净状态
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_DeInit(&htim3);
    
    // 2. 重新初始化时钟（DeInit会关时钟）
    __HAL_RCC_TIM3_CLK_ENABLE();

    //设置1ms时基
    uint32_t tim_clk = 72000000;
    uint32_t psc = (tim_clk / 1000) - 1; 
    uint16_t arr = interval_ms - 1;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = psc;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = arr;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // 4. HAL初始化（配置PSC, ARR等，但不启动）
    HAL_TIM_Base_Init(&htim3);

    // 5. 【关键】清NVIC pending，防止旧中断请求
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    
    // 6. 启动中断模式
    HAL_TIM_Base_Start_IT(&htim3);

    Sys_Task_TIM3 = task ;
}

void CloseTimer3(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    NVIC_ClearPendingIRQ(TIM3_IRQn);  // 清pending，防止残留
    Sys_Task_TIM3 = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) {
        if(Sys_Task_TIM2 != 0) {
            Sys_Task_TIM2();
        }
    }else if(htim->Instance == TIM2)
    {
        if(Sys_Task_TIM3 != 0)
        {
            Sys_Task_TIM3();
        }
    }
}

