#include "vbus_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "adc.h"

extern QueueHandle_t xVbusQueue;
void vVbusTask(void *pvParameters){
	static volatile uint32_t adcValue = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcValue, 1);	
	for(;;){
		float voltage = adcValue / 4095.0f * 3.3f * 11.0f * 0.9853f;
        xQueueOverwrite(xVbusQueue, &voltage);
        vTaskDelay(pdMS_TO_TICKS(100));
	}
}
