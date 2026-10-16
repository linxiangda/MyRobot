#include "bmi088_task.h"
#include "bmi088_driver.h"

#include "log_task.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

IMUData_TypeDef IMUdata;
extern QueueHandle_t xIMUQueue;
void vIMUTask(void *pvParameters){
	float sensor_data[BMI088_DATA_LENGTH];
	if (Drive_BMI088_Init() != BMI088_OK)
    {
        vTaskDelete(NULL);
        return;
    }
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
		if (Drive_BMI088_Read(sensor_data)== BMI088_OK){
			IMUdata.accel[0] = sensor_data[0];

            IMUdata.accel[1] = sensor_data[1];

            IMUdata.accel[2] = sensor_data[2];

            IMUdata.gyro[0] = sensor_data[3];

            IMUdata.gyro[1] = sensor_data[4];

            IMUdata.gyro[2] = sensor_data[5];

            IMUdata.temperature = sensor_data[6];
			
			xQueueOverwrite(xIMUQueue, &IMUdata);
		}
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
	}
}
