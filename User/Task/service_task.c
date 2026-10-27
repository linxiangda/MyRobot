#include "service_task.h"
#include "log_task.h"

#include "gpio.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"

extern SemaphoreHandle_t xBuzzerSemphr;
extern TimerHandle_t xLEDTimer; 	
extern EventGroupHandle_t xEventGroup;

/* 闪灯任务 */
void vLED1Task(void *pvParameters){
	for(;;){
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(500));
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
void vLED2Task(void *pvParameters){
	for(;;){
		xEventGroupWaitBits(xEventGroup, (0x01 | (0x01 << 1) | (0x01 << 2)), pdFALSE, pdTRUE, portMAX_DELAY);
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
		vTaskDelay(pdMS_TO_TICKS(500));
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void vTimerCallback(TimerHandle_t XTimer){
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
}
/* 消抖按键任务，给led定时器复位，给蜂鸣器二进制信号量， */
void vKeyTask(void *pvParameters){
	for(;;){
		vTaskDelay(pdMS_TO_TICKS(20));
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){
			vTaskDelay(pdMS_TO_TICKS(10));
			while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == RESET){vTaskDelay(pdMS_TO_TICKS(5));}
			vTaskDelay(pdMS_TO_TICKS(10));
				
			HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
			xTimerReset(xLEDTimer,portMAX_DELAY);
			// xSemaphoreGive(xBuzzerSemphr);
			
			if ((xEventGroupGetBits(xEventGroup) & 1U) != 0){
				xEventGroupClearBits(xEventGroup, 1U);
			}
			else {
				xEventGroupSetBits(xEventGroup, 1U);
			}
					
		}
	}
} 

/* 蜂鸣器任务 */
void vBuzzerTask(void *pvParameters){
	for(;;){
		xSemaphoreTake(xBuzzerSemphr, portMAX_DELAY);
		HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

		vTaskDelay(pdMS_TO_TICKS(200));

		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
	}
}



