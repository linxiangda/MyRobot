#ifndef _SERVICE_TASK_H_
#define _SERVICE_TASK_H_

#include "main.h"

typedef struct{
	GPIO_TypeDef *LED_PORT;
	uint16_t LED_PIN;
	uint32_t LED_Period;
}LEDBlink_TypeDef;

void vLEDTask(void *pvParameters);
void vKeyTask(void *pvParameters);
void vBuzzerTask(void *pvParameters);
void vVbusTask(void *pvParameters);
#endif
