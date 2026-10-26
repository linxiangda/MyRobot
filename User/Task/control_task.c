#include "control_task.h"
#include "imu_task.h"
#include "can_task.h"
#include "wheel_control.h"

#include <stdio.h>
#include "string.h"
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "log_task.h"
extern QueueHandle_t xIMUQueue;
extern QueueHandle_t xMotorQueue;

void vControlLegTask(void *pvParameters){	
	for(;;){
        vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void vControlWheelTask(void *pvParameters){	
    static IMUData_TypeDef IMUdata;
    static MotorData_TypeDef Motordata;
    TickType_t period = pdMS_TO_TICKS(5);
    TickType_t last_wake = xTaskGetTickCount();
	for(;;){
        if (xQueuePeek(xIMUQueue, &IMUdata,pdMS_TO_TICKS(1000)) != pdTRUE) return;
        if (xQueuePeek(xMotorQueue, &Motordata,pdMS_TO_TICKS(1000)) != pdTRUE) return;
        float state[6] = {
            IMUdata.position - 0,   
            IMUdata.speed - 0,          
            IMUdata.pitch - 0,       
            IMUdata.gyro[0] - 0,                
            IMUdata.yaw - 0,             
            IMUdata.gyro[2] - 0    
        };
        float left_torque,right_torque;
        Wheel_Control(state, &left_torque, &right_torque);
        
        char packet[32];
        int len;
        len = snprintf(packet, sizeof(packet),"%.4f,%.4f\n",left_torque,right_torque);
        static uint16_t i = 0;
        i++;
        if (len > 0 && (size_t)len < sizeof(packet) && i>=5){
            i=0;
            print(packet);
        }   
        if((xTaskGetTickCount() - last_wake >= period)) last_wake = xTaskGetTickCount();
        xTaskDelayUntil(&last_wake, period);
	}
}
