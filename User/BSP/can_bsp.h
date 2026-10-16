#ifndef _CAN_BSP_H_
#define _CAN_BSP_H_

#include <stdint.h>
#include "main.h"

HAL_StatusTypeDef CAN_Start(FDCAN_HandleTypeDef *hfdcan,uint32_t rxID,uint32_t rxMask);
HAL_StatusTypeDef CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan,uint32_t txID, const uint8_t *data,uint32_t len);

#endif
