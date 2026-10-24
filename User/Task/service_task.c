#include "service_task.h"
#include "log_task.h"

#include "gpio.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

extern SemaphoreHandle_t xKeySemphr;
extern TimerHandle_t xLEDTimer; 	

/* 闪灯任务 */
void vLEDTask(void *pvParameters){
	LEDBlink_TypeDef *led = (LEDBlink_TypeDef *)pvParameters;
	for(;;){
		HAL_GPIO_WritePin(led->LED_PORT,led->LED_PIN,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(led->LED_Period/2));
		HAL_GPIO_WritePin(led->LED_PORT,led->LED_PIN,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(led->LED_Period/2));
	}
}

/* 消抖按键任务，给led定时器复位，给蜂鸣器二进制信号量， */
void vKeyTask(void *pvParameters){

	for(;;){
		vTaskDelay(pdMS_TO_TICKS(20));
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){
			vTaskDelay(pdMS_TO_TICKS(10));
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){}
			vTaskDelay(pdMS_TO_TICKS(10));
				
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
			xTimerReset(xLEDTimer,portMAX_DELAY);
			xSemaphoreGive(xKeySemphr);
				
						
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



