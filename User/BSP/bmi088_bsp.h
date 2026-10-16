#ifndef _BMI088_BSP_H_
#define _BMI088_BSP_H_

#include <stdint.h>

void BSP_BMI088_Init(void);
void BSP_BMI088_DelayMs(uint32_t ms);
int BSP_BMI088_Read(uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len);
int BSP_BMI088_Write(uint8_t dev, uint8_t reg, uint8_t value);

#endif
