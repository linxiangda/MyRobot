#ifndef NRF_BSP_H
#define NRF_BSP_H

#include <stdint.h>

void BSP_NRF_Init(void);
void BSP_NRF_SetCE(uint8_t level);
void BSP_NRF_SetCSN(uint8_t level);
uint8_t BSP_NRF_Transfer(uint8_t tx, uint8_t *rx);
void BSP_NRF_DelayUs(uint32_t us);
void BSP_NRF_DelayMs(uint32_t ms);

#endif
