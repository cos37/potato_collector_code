#include "loop.h"
#include "ssd1306_driver.h"
#include "imu_driver.h"
#include "ddp.h"
#include "fixpoint.h"
#include "usart.h"
#include <stdint.h>
#include "mc_service.h"
#include "interact.h"
#include "move.h"

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
    moveHandle.move_y(50,POSITIVE);   
}
void Move_stop(void)
{
    moveHandle.stop();
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
    .data.fp16_val = 0
};

void IMU_Update(void);
void LED_Shark(void);
void Read_PIN(void);

void setup()
{
	SSD1306_Driver_Init();
 	DDP_Init(&ddpYaw);
    IMU_Reboot();
    MC_Init();
    Menu_Init();
	Key_Init();
    Menu_AddItem(&item1);
    Menu_AddItem(&item2);
    Menu_AddItem(&item_move);
    Menu_AddItem(&item_stop); // 重复添加测试链表完整性
    Move_Init(&moveHandle);
    HAL_Delay(1000);
}
/**** loop函数 ****/

void loop()
{
	LED_Shark();
    IMU_Update();
    Mc_StateMachine();
    
    Key_Update();
    SSD1306_Driver_Update();
}

/***** 任务函数实现 ******/

void IMU_Update(void)
{
	IMU_DateProcess();
	static uint32_t st = 0 ;
	uint32_t current = HAL_GetTick();
	if(current-st>=200)
	{
		ddpYaw.data.fp16_val = imuHandle.yaw;
		DDP_Update(&ddpYaw); 
	}
}

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