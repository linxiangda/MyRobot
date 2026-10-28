#include "nrf_bsp.h"
#include "main.h"
#include "spi.h"

#define NRF_SPI_TIMEOUT_MS 10U

void BSP_NRF_Init(void)
{
    BSP_NRF_SetCE(0);
    BSP_NRF_SetCSN(1);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void BSP_NRF_SetCE(uint8_t level)
{
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_NRF_SetCSN(uint8_t level)
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BSP_NRF_Transfer(uint8_t tx, uint8_t *rx)
{
    uint8_t received = 0;

    if (rx == NULL)
        return 1;

    if (HAL_SPI_TransmitReceive(&hspi3, &tx, &received, 1, NRF_SPI_TIMEOUT_MS) != HAL_OK)
        return 1;

    *rx = received;
    return 0;
}

void BSP_NRF_DelayUs(uint32_t us)
{
    uint32_t cycles_per_us = (SystemCoreClock + 999999U) / 1000000U;

    while (us != 0U) {
        uint32_t chunk = us > 1000U ? 1000U : us;
        uint32_t cycles = chunk * cycles_per_us;
        uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
        }
        us -= chunk;
    }
}

void BSP_NRF_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
