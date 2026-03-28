#include "task_create.h"
#include "FreeRTOS.h"
#include "task.h"
#include "task_service.h"
#include "task_business.h"
#include "task_application.h"
#include "stm32f1xx_hal.h"
void USR_QueueCreate(void)
{
    SERVICE_QueueCreate();
}


void USR_TASK_CREATE(void)
{
    xTaskCreate(Task_Led1, "Task_Led1", 128/4, NULL, 1, NULL);
    xTaskCreate(Task_Imu, "Task_Imu", 256, NULL, 2, &xIMUTaskHandle);
    BaseType_t result = xTaskCreate(Task_Display, "Task_Display", 128, NULL, 2, NULL);
    if (result != pdPASS) {
    // 创建失败！内存不足或参数错误
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // 故障指示
    }
}

