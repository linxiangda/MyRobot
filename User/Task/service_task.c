#include "service_task.h"
#include "log_task.h"
#include "string.h"
#include "stdio.h"

#include "gpio.h"
#include "tim.h"
#include "adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"

extern SemaphoreHandle_t xKeySemphr;
extern TimerHandle_t xLEDTimer; 	

void vLEDTask(void *pvParameters){
	LEDBlink_TypeDef *led = (LEDBlink_TypeDef *)pvParameters;
	for(;;){
		HAL_GPIO_WritePin(led->LED_PORT,led->LED_PIN,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(led->LED_Period/2));
		HAL_GPIO_WritePin(led->LED_PORT,led->LED_PIN,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(led->LED_Period/2));
	}
}

void vKeyTask(void *pvParameters){

	for(;;){
		vTaskDelay(pdMS_TO_TICKS(20));
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){
			vTaskDelay(pdMS_TO_TICKS(10));
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){}
			vTaskDelay(pdMS_TO_TICKS(10));
				
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
			xTimerReset(xLEDTimer,portMAX_DELAY);
//			xSemaphoreGive(xKeySemphr);
				
			char data[32];
			for(int i=0;i<=10;i++){
				sprintf(data,"key on %d\n",i);
				print(data); 
			}
						
		}
	}
} 

void vBuzzerTask(void *pvParameters){
	for(;;){
		xSemaphoreTake(xKeySemphr, portMAX_DELAY);
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

		vTaskDelay(pdMS_TO_TICKS(200));

		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	}
}


void vVbusTask(void *pvParameters){
	static volatile uint32_t adcValue = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcValue, 1);	
	for(;;){
		float voltage = adcValue / 4095.0f * 3.3f * 11.0f * 0.9853f;
		int v32 = (int)(voltage * 1000);
		char data[20];
		int len = snprintf(data, sizeof(data), "Vbus = %d.%03d\n", (v32 / 1000), (v32 % 1000));

		if (len > 0){
			print(data);
		}
        vTaskDelay(pdMS_TO_TICKS(100));
	}
}


