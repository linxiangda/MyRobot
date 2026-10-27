#include "imu_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "imu.h"
#include "can_task.h"
#include "lowpass.h"

extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xMotorQueue;
extern EventGroupHandle_t xEventGroup;

LowPassFilter_t motorspeed_lowpass_L;
LowPassFilter_t motorspeed_lowpass_R;
void vIMUTask(void *pvParameters)
{
    IMUData_TypeDef IMUdata = {0};
    MotorData_TypeDef Motordata = {0};
    TickType_t last_wake;
    TickType_t period = pdMS_TO_TICKS(1);
    while(IMU_Init()){vTaskDelay(pdMS_TO_TICKS(100));}
    last_wake = xTaskGetTickCount();
    float wheel_radius = 0.04566f;
    for (;;) {
        IMU_RawDataGet(&IMUdata);   
        xQueuePeek(xMotorQueue,&Motordata,0);
        float left_speed = Motordata.motor_speed[1];
        float right_speed = Motordata.motor_speed[3];
        left_speed = LowPass_Update(&motorspeed_lowpass_L, left_speed, 50,0.001f);
        right_speed = LowPass_Update(&motorspeed_lowpass_R, right_speed, 50,0.001f);

        IMUdata.speed = wheel_radius*((Motordata.motor_speed[1] + Motordata.motor_speed[3]) * 0.5f + IMUdata.gyro[0]);
        IMUdata.position += IMUdata.speed * 0.001f;
        if (IMUdata.error == 0) {
            IMU_PosCalculate(&IMUdata,left_speed,right_speed,0.001);
            IMUdata.update_ms = HAL_GetTick();
            xQueueOverwrite(xIMUQueue, &IMUdata);
            xEventGroupSetBits(xEventGroup, (1U << 1));
        }
        // IMU_TemperatureControl(&IMUdata,45,0.001f);
        
        if ((xTaskGetTickCount() - last_wake) >= period)
            last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
    }
}
