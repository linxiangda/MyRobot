#ifndef _LOG_TASK_H_
#define _LOG_TASK_H_
#include "main.h"

void print(const char *str);
void  vSerialGateKeeperTask(void *pvParameters);
void vLOGTask(void *pvParameters);
void vSerialReceiveTask(void *pvParameters);
#endif
