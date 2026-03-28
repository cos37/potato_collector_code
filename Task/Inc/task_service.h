#ifndef _TASK_SERVICE_H__
#define _TASK_SERVICE_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"  
extern TaskHandle_t xIMUTaskHandle;
extern QueueHandle_t xQueueYaw;

void SERVICE_QueueCreate(void);

void Task_Led1(void*pvParameters);
void Task_Imu(void*pvParameters);
void Task_Display(void*pvParameters);

#endif
