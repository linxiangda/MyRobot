#ifndef _SERVICE_TASK_H_
#define _SERVICE_TASK_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
void vLED1Task(void *pvParameters);
void vLED2Task(void *pvParameters);
void vTimerCallback(TimerHandle_t XTimer);
void vKeyTask(void *pvParameters);
void vBuzzerTask(void *pvParameters);
#endif
