#include "task_service.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "imu_driver.h"
#include "fixpoint.h"
#include "ddp.h"
#include "ssd1306_driver.h"

TaskHandle_t xIMUTaskHandle;
QueueHandle_t xQueueYaw;



extern DDP_Handle_t ddpYaw;


void SERVICE_QueueCreate(void)
{
    xQueueYaw = xQueueCreate(4, sizeof(int32_t));
}

void Task_Led1(void*pvParameters)
{
    (void)pvParameters;
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void Task_Imu(void*pvParameters)
{
    (void)pvParameters;
    while(1)
    {

        IMU_DateProcess();
        ddpYaw.data.fp16_val = imuHandle.yaw;

    }
}

void Task_Display(void*pvParameters)
{
    (void)pvParameters;
    // int32_t yaw;
	
    while(1)
    {
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
        // if(xQueueReceive(xQueueYaw, &yaw, portMAX_DELAY) == pdPASS) {
            
            DDP_Update(&ddpYaw);
            SSD1306_Driver_Update();
        // }
        // ddpYaw.data.fp16_val = imuHandle.yaw;
        // DDP_Update(&ddpYaw);
        // SSD1306_Driver_WriteIntNums(0, 2, imuHandle.yaw);
        // SSD1306_Driver_Update();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
