#include "imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t xIMUQueue;

void vIMUTask(void *pvParameters)
{
    IMUData_TypeDef data;
    TickType_t last_wake;
    TickType_t period = pdMS_TO_TICKS(1);

    while (IMU_ModuleInit() != 0) {
        IMU_ModuleGetData(&data);
        xQueueOverwrite(xIMUQueue, &data);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    last_wake = xTaskGetTickCount();
    for (;;) {
        IMU_ModuleUpdate();
        IMU_ModuleGetData(&data);
        xQueueOverwrite(xIMUQueue, &data);

        if ((xTaskGetTickCount() - last_wake) >= period)
            last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
    }
}
