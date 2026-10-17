#include "imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t xIMUQueue;

void vIMUTask(void *pvParameters)
{
    IMUData_TypeDef data = {0};
    TickType_t last_wake;
    TickType_t period = pdMS_TO_TICKS(1);
    while(IMU_Init()){vTaskDelay(pdMS_TO_TICKS(100));}
    last_wake = xTaskGetTickCount();
    for (;;) {
        IMU_RawDataGet(&data);    
        if (data.error == 0) IMU_PosCalculate(&data,0.001);
        // IMU_TemperatureControl(&data,25,0.001);
        xQueueOverwrite(xIMUQueue, &data);
        
        if ((xTaskGetTickCount() - last_wake) >= period)
            last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
    }
}
