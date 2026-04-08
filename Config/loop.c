#include "loop.h"
#include "ssd1306_driver.h"
#include "ddp.h"
#include "fixpoint.h"
#include "usart.h"
#include <stdint.h>
#include "mc_service.h"
#include "interact.h"
#include "move.h"
#include "sys.h"
#include "stm32f1xx_hal.h"
#include "imu_uart.h"
#include "imu_driver.h"
MoveHandle_t moveHandle ;

void Toggle_LED1(void)
{
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
}

void Toggle_LED2(void)
{
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
}

void Move_req(void)
{
    MC_Service_Enable(imuHandle.yaw,100000);
}

void Move_stop(void)
{
    MC_Service_Disable();
}
MenuItem_t item1 = {.name = "TLED1", .task = Toggle_LED1};
MenuItem_t item2 = {.name = "TLED2", .task = Toggle_LED2};
MenuItem_t item_move = {.name = "MOVE", .task = Move_req};
MenuItem_t item_stop = {.name = "STOP", .task = Move_stop};

DDP_Handle_t ddpYaw= {
    .page = 0,
    .line_start = 0,
    .str_len = 0,
    .name = "Yaw",
    .name_len = 3,
    .type = DDP_TYPE_FP16,
    .data.fp16_val = 3<<16
};

void IMU_Update(void);
void LED_Shark(void);
void OLED_Reflash(void);

void DWT_Init(void)
{
    // 使能 DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // 使能 CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 清零计数器
    DWT->CYCCNT = 0;
}


void setup()
{
	SSD1306_Driver_Init();
 	// DDP_Init(&ddpYaw);
    MC_Init();
    Menu_Init();
	Key_Init();
    Menu_AddItem(&item1);
    Menu_AddItem(&item2);
    Menu_AddItem(&item_move);
    Menu_AddItem(&item_stop); 
    Move_Init(&moveHandle);
    DWT_Init();
    UART2_Receive_IT_Start();
    HAL_Delay(1000);

}
/**** loop函数 ****/

void loop()
{
	LED_Shark();
    IMU_Update();
    Mc_StateMachine();
    Key_Update();
    OLED_Reflash();
}

/***** 任务函数实现 ******/


void LED_Shark(void)
{
    static uint32_t tick = 0;
    uint32_t current_tick = HAL_GetTick();

    if(current_tick-tick >= 500)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        tick=current_tick;
    }

}

void IMU_Update(void)
{
    imuHandle.yaw = get_yaw_fp16();
}

void OLED_Reflash(void)
{
    static uint32_t tick = 0;
    uint32_t current_tick = HAL_GetTick();
	ddpYaw.data.fp16_val = imuHandle.yaw;
    if(current_tick-tick >= 50)
    {
        
        tick=current_tick;
		
		// DDP_Update(&ddpYaw); 
        SSD1306_Driver_WriteFP16(20, 0, imuHandle.yaw);
        SSD1306_Driver_Update();
    }
}
