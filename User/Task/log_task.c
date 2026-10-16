#include "log_task.h"
#include "imu_task.h"

#include "string.h"
#include "stdio.h"

#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t xSerialQueue;
extern QueueHandle_t xIMUQueue;

void  vSerialGateKeeperTask(void *pvParameters){
	char *pMemory;
	for(;;){
		xQueueReceive(xSerialQueue,&pMemory,portMAX_DELAY);
		HAL_UART_Transmit(&huart1,(const uint8_t *)pMemory,(uint16_t)strlen((char *)pMemory),1000);
		vPortFree(pMemory);
	}
}
void print(const char *str){
	char *pMemory = pvPortMalloc(strlen(str)+1);
	
	if (pMemory == NULL) return;
	
	strcpy(pMemory,str);
	xQueueSend(xSerialQueue,&pMemory,portMAX_DELAY);
}

void vLOGTask(void *pvParameters)
{
    IMUData_TypeDef data;
    char packet[128];

    for (;;)
    {
        if (xQueueReceive(xIMUQueue, &data, portMAX_DELAY) == pdTRUE){
            int len = snprintf(
                packet, sizeof(packet),
				"ACC:%.3f,%.3f,%.3f \nGYRO:%.3f,%.3f,%.3f \nTEM%.2f\n",
                data.accel[0],
                data.accel[1],
                data.accel[2],
                data.gyro[0],
                data.gyro[1],
                data.gyro[2],
                data.temperature);

            if (len > 0){
                print(packet);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
