#include "imu_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "can_task.h"

extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xMotorQueue;


void vIMUTask(void *pvParameters)
{
    IMUData_TypeDef IMUdata = {0};
    MotorData_TypeDef Motordata = {0};
    TickType_t last_wake;
    TickType_t period = pdMS_TO_TICKS(1);
    while(IMU_Init()){vTaskDelay(pdMS_TO_TICKS(100));}
    last_wake = xTaskGetTickCount();
    for (;;) {
        
        IMU_RawDataGet(&IMUdata);   
        if(xQueuePeek(xMotorQueue,&Motordata,0) == pdTRUE){

        } 
        if (IMUdata.error == 0) IMU_PosCalculate(&IMUdata,Motordata.motor_speed[1],Motordata.motor_speed[3],0.001);
        // IMU_TemperatureControl(&data,25,0.001);
        xQueueOverwrite(xIMUQueue, &IMUdata);
        
        if ((xTaskGetTickCount() - last_wake) >= period)
            last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
    }
}
