#ifndef _CAN_BSP_H_
#define _CAN_BSP_H_

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef CAN1_Start(void);
HAL_StatusTypeDef CAN1_Send_Data(uint32_t id, const uint8_t *data,uint32_t len);

#endif
