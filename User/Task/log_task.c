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

void print_imulog(void);

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

void vLOGTask(void *pvParameters){
    for (;;){
        print_imulog();
        vTaskDelay(pdMS_TO_TICKS(10));

    }
}

void print_imulog(void){
    int len;
    static IMUData_TypeDef data;
    char packet[128];
    if (xQueuePeek(xIMUQueue, &data,pdMS_TO_TICKS(1000)) != pdTRUE) return;

    len = snprintf(
            packet, sizeof(packet),
            "%.2f,%.2f,%.2f\n",
            (double)data.roll,
            (double)data.pitch,
            (double)data.yaw);

    if (len > 0 && (size_t)len < sizeof(packet)){
        print(packet);
    }   
}
